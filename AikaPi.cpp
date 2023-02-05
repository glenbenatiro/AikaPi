#include "AikaPi.h"

#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/stat.h>

#include <iostream>
#include <bitset>

#include <thread>
#include <chrono>
#include <cstdio>

// --- Utility Class ---

uint32_t Utility:: 
get_bits (uint32_t input, unsigned shift, uint32_t mask)
{
  return ((input >> shift) & mask);
}

void Utility:: 
reg_write (MemoryMap mem_map, 
           uint32_t  offset, 
           uint32_t  value, 
           uint32_t  mask, 
           unsigned  shift)
{
  reg_write (Utility::get_reg32 (mem_map, offset),
             value,
             mask,
             shift);
}

void Utility:: 
reg_write (volatile uint32_t *reg,
                    uint32_t value,
                    uint32_t mask,
                    unsigned shift)
{
  *reg = (*reg & ~(mask << shift)) | (value << shift);
}

void Utility:: 
print_bits (int      bits,
            unsigned size)
{
  #define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
  
  #define BYTE_TO_BINARY(byte)  \
    (byte & 0x80 ? '1' : '0'), \
    (byte & 0x40 ? '1' : '0'), \
    (byte & 0x20 ? '1' : '0'), \
    (byte & 0x10 ? '1' : '0'), \
    (byte & 0x08 ? '1' : '0'), \
    (byte & 0x04 ? '1' : '0'), \
    (byte & 0x02 ? '1' : '0'), \
    (byte & 0x01 ? '1' : '0') 

  for (int a = 0; a < size && a < sizeof (bits); a++)
  {
    printf (BYTE_TO_BINARY_PATTERN " ", BYTE_TO_BINARY (bits >> (8 * (size - 1 - a))));
  }

  printf ("\n");
}

// 
// #include "Defaults.h"

AikaPi::AikaPi ()
{
  map_devices ();
 
  spi_init (SPI_FREQUENCY);
  // aux_spi0_init ();
}

AikaPi::~AikaPi ()
{
	
}

// --- General ---
void AikaPi:: 
delay (uint32_t microseconds)
{
  std::this_thread::sleep_for (std::chrono::duration <int, std::micro> (microseconds));
}

// Allocate uncached memory, get bus & phys addresses
void*
AikaPi::map_uncached_mem (MemoryMap *mp,
                          int        size)
{
  void *ret;
  
  mp->size = PAGE_ROUNDUP(size);
  mp->fd   = mailbox_open ();
  
  ret = (mp->h    = alloc_vc_mem (mp->fd, mp->size, DMA_MEM_FLAGS))   > 0 &&
        (mp->bus  = lock_vc_mem  (mp->fd, mp->h))                    != 0 &&
        (mp->virt = map_segment  (BUS_PHYS_ADDR(mp->bus), mp->size)) != 0 
        ? mp->virt : 0;
        
  // printf("VC mem handle %u, phys %p, virt %p\n", mp->h, mp->bus, mp->virt);
  
  return(ret);
}

void
AikaPi::map_devices ()
{
  map_periph (&m_aux_regs,  (void *) AUX_BASE,  PAGE_SIZE);
  map_periph (&m_gpio_regs, (void *) GPIO_BASE, PAGE_SIZE);
  map_periph (&m_regs_dma,  (void *) DMA_BASE,  PAGE_SIZE);
  map_periph (&m_regs_spi,  (void *) SPI0_BASE, PAGE_SIZE);
  map_periph (&m_clk_regs,  (void *) CLK_BASE,  PAGE_SIZE);
  map_periph (&m_regs_pwm,  (void *) PWM_BASE,  PAGE_SIZE);
  map_periph (&m_regs_usec, (void *) USEC_BASE, PAGE_SIZE);
}

// --- Memory ---
// use mmap to obtain virtual address, given physical
void
AikaPi::map_periph (MemoryMap *mp, void *phys, int size)
{
  mp->phys = phys;
  mp->size = PAGE_ROUNDUP(size);
  mp->bus  = (uint8_t *)phys - (uint8_t *)PHYS_REG_BASE + (uint8_t *)BUS_REG_BASE;
  mp->virt = map_segment (phys, mp->size);
}

// Free mapped peripheral or memory
void 
AikaPi::unmap_periph_mem (MemoryMap *mp)
{
  if (mp)
    {
      if (mp->fd)
        {
          unmap_segment (mp->virt, mp->size);
          unlock_vc_mem (mp->fd, mp->h);
          free_vc_mem   (mp->fd, mp->h);
          mailbox_close    (mp->fd);
        }
      else
        {
          unmap_segment (mp->virt, mp->size);
        }
    }
}

// --- Virtual Memory ---
// Get virtual memory segment for peripheral regs or physical mem
void*
AikaPi::map_segment (void *addr, 
                                  int   size)
{
  int fd;
  void *mem;

  size = PAGE_ROUNDUP(size);
  
  if ((fd = open ("/dev/mem", O_RDWR | O_SYNC | O_CLOEXEC)) < 0)
    fail ("Error: can't open /dev/mem, run using sudo\n");
        
  mem = mmap (0, size, PROT_WRITE | PROT_READ, MAP_SHARED, fd, (uint32_t)addr);
  
  close(fd);
  
  #if DEBUG
    printf("Map %p -> %p\n", (void *)addr, mem);
  #endif
  
  if (mem == MAP_FAILED)
    fail("Error: can't map memory\n");
    
  return(mem);
}

// Free mapped memory
void 
AikaPi::unmap_segment (void *mem, 
                                    int  size)
{
  if (mem)
    munmap (mem, PAGE_ROUNDUP(size));
}


// --- DMA ---

// Enable and reset DMA
void 
AikaPi::dma_enable (int chan)
{
  *REG32(m_regs_dma, DMA_ENABLE) |= (1 << chan);
  *REG32(m_regs_dma, DMA_REG(chan, DMA_CS)) = 1 << 31;
}

// Start DMA, given first control block
void 
AikaPi::dma_start (MemoryMap *mp, 
                                int      chan, 
                                DMA_CB  *cbp, 
                                uint32_t csval)
{
  *REG32(m_regs_dma, DMA_REG(chan, DMA_CONBLK_AD)) = MEM_BUS_ADDR(mp, cbp);
  *REG32(m_regs_dma, DMA_REG(chan, DMA_CS))        = 2;         // Clear 'end' flag
  *REG32(m_regs_dma, DMA_REG(chan, DMA_DEBUG))     = 7;         // Clear error bits
  *REG32(m_regs_dma, DMA_REG(chan, DMA_CS))        = 1 | csval; // Start DMA
}

// void
// AikaPi::dma_start (uint32_t dma_channel, 
//                    uint32_t dma_cb_address)
// {
//   // Load control block
//   *(Utility::get_reg32 (m_regs_dma, 
//     Utility::dma_chan_reg_offset (dma_channel, DMA_CONBLK_AD))) = dma_cb_address;

//   // Clear END flag in DMA CS
//   *(Utility::get_reg32 (m_regs_dma, 
//     Utility::dma_chan_reg_offset (dma_channel, DMA_CS))) = 2;

//   // Clear error flags in DMA DEBUG
//   *(Utility::get_reg32 (m_regs_dma, 
//     Utility::dma_chan_reg_offset (dma_channel, DMA_DEBUG))) = 7;

//   // Start DMA channel 
//   *(Utility::get_reg32 (m_regs_dma, 
//     Utility::dma_chan_reg_offset (dma_channel, DMA_CS))) = 1;
// }

// Return remaining transfer length
uint32_t 
AikaPi::dma_transfer_len (int chan)
{
 return (*REG32(m_regs_dma, DMA_REG(chan, DMA_TXFR_LEN)));
}

// Halt current DMA operation by resetting controller
void 
AikaPi::dma_stop (int chan)
{
  if (m_regs_dma.virt)
    *REG32(m_regs_dma, DMA_REG(chan, DMA_CS)) = 1 << 31;
}

void 
AikaPi::dma_wait(int chan)
{
  int n = 10000;

  do 
    usleep(10);
  while (dma_transfer_len (chan) && --n);
    
  if (n == 0)
    printf ("DMA transfer timeout\n");
}

void 
AikaPi::dma_pause (unsigned channel)
{
  printf ("in dma pause\n");

  volatile uint32_t *reg = Utility::get_reg32 (m_regs_dma, 
    DMA_REG (channel, DMA_CS));

  Utility::reg_write (reg, 0, 1, 0);

  while (Utility::get_bits (*reg, 4, 1))
  {
    printf ("DMA_CS: ");

    std::cout << std::bitset<32> (*reg) << "\n";
  }
  
  // while (Utility::get_bits (*reg, 4, 1));

  printf ("dma pause done\n");
}

void 
AikaPi::dma_play (unsigned channel)
{
  volatile uint32_t *reg = Utility::get_reg32 (m_regs_dma, 
    DMA_REG (channel, DMA_CS));

  Utility::reg_write (reg, 1, 1, 0);

  //while (!(Utility::get_bits (*reg, 4, 1)));
}

// Display DMA registers
void 
AikaPi::dma_disp (int chan)
{
  const char *m_regs_dmatrs[] = {"DMA CS", 
                                 "CB_AD", 
                                 "TI", 
                                 "SRCE_AD", 
                                 "DEST_AD",
                                 "TFR_LEN", 
                                 "STRIDE", 
                                 "NEXT_CB", 
                                 "DEBUG", 
                                 ""};
  int i = 0;
  volatile uint32_t *p = REG32(m_regs_dma, DMA_REG(chan, DMA_CS));
    

  while (m_regs_dmatrs[i][0])
    {
      printf("%-7s %08X ", m_regs_dmatrs[i++], *p++);
      
      if (i % 5 == 0 || m_regs_dmatrs[i][0] == 0)
        printf("\n");
    }
}


// --- Videocore Mailbox ---
// https://github.com/raspberrypi/firmware/wiki/Mailbox-property-interface

int 
AikaPi::mailbox_open (void)
{
  int fd;

  if ((fd = open ("/dev/vcio", 0)) < 0)
    fail ("Error: Can't open VC mailbox.\n");
     
  return (fd);
}

// Close mailbox interface
void 
AikaPi::mailbox_close (int fd)
{
  close (fd);
}

// Send message to mailbox, return first response int, 0 if error
uint32_t 
AikaPi::msg_mbox (int     fd, 
                               VC_MSG *msgp)
{
  uint32_t ret = 0, i;

  for (i = msgp->dlen/4; i <= msgp->blen/4; i += 4)
    msgp->uints[i++] = 0;
    
  msgp->len = (msgp->blen + 6) * 4;
  msgp->req = 0;
  
  if (ioctl (fd, _IOWR(100, 0, void *), msgp) < 0)
    printf("VC IOCTL failed\n");
  else if ((msgp->req & 0x80000000) == 0)
    printf("VC IOCTL error\n");
  else if (msgp->req == 0x80000001)
    printf("VC IOCTL partial error\n");
  else
    ret = msgp->uints[0];
      
  #if DEBUG
    disp_vc_msg (msgp);
  #endif
  
  return (ret);
}

// Allocate memory on PAGE_SIZE boundary, return handle
uint32_t 
AikaPi::alloc_vc_mem (int            fd, 
                                   uint32_t       size,
                                   MAILBOX_ALLOCATE_MEMORY_FLAGS flags)
{
  // https://github.com/raspberrypi/firmware/wiki/Mailbox-property-interface#allocate-memory

  VC_MSG msg = {.tag   = MAILBOX_TAG_ALLOCATE_MEMORY,
                .blen  = 12,
                .dlen  = 12,
                .uints = {PAGE_ROUNDUP(size), 
                          PAGE_SIZE, flags}};
                          
  return (msg_mbox(fd, &msg));
}

// Lock allocated memory, return bus address
void*
AikaPi::lock_vc_mem (int fd, 
                                  int h)
{
  VC_MSG msg = {.tag   = 0x3000d, 
                .blen  = 4, 
                .dlen  = 4, 
                .uints = {static_cast<uint32_t>(h)}};
  
  return (h ? (void *)msg_mbox (fd, &msg) : 0);
}

// Unlock allocated memory
uint32_t 
AikaPi::unlock_vc_mem (int fd, 
                                    int h)
{
  VC_MSG msg = {.tag   = 0x3000e, 
                .blen  = 4, 
                .dlen  = 4, 
                .uints = {static_cast<uint32_t>(h)}};
  
  return(h ? msg_mbox(fd, &msg) : 0);
}

// Free memory
uint32_t 
AikaPi::free_vc_mem (int fd, 
                                  int h)
{
  VC_MSG msg={.tag     = 0x3000f, 
              .blen    = 4,
              .dlen    = 4, 
              .uints   = {static_cast<uint32_t>(h)}};
  
  return(h ? msg_mbox(fd, &msg) : 0);
}

uint32_t 
AikaPi::fset_vc_clock (int      fd, 
                                    int      id, 
                                    uint32_t freq)
{
  VC_MSG msg1 = {.tag   = 0x38001, 
                 .blen  = 8, 
                 .dlen  = 8, 
                 .uints = {static_cast<uint32_t>(id), 1}};
                 
  VC_MSG msg2 = {.tag   = 0x38002, 
                 .blen  = 12,
                 .dlen  = 12,
                 .uints = {static_cast<uint32_t>(id), freq, 0}};
  
  msg_mbox    (fd, &msg1);
  disp_vc_msg (&msg1);
  
  msg_mbox    (fd, &msg2);
  disp_vc_msg (&msg2);
  
  return(0);
}

// Display mailbox message
void 
AikaPi::disp_vc_msg(VC_MSG *msgp)
{
    int i;

    printf("VC msg len=%X, req=%X, tag=%X, blen=%x, dlen=%x, data ",
        msgp->len, msgp->req, msgp->tag, msgp->blen, msgp->dlen);
    for (i=0; i<msgp->blen/4; i++)
        printf("%08X ", msgp->uints[i]);
    printf("\n");
}



// -- Aux ---
// Catastrophic failure in initial setup
void 
AikaPi::fail (const char *s)
{
  printf (s);
  terminate (0);
}

// Free memory & peripheral mapping and exit
void 
AikaPi::terminate (int sig)
{
  printf("Closing\n");
  
  spi_disable();
  //dma_stop(DMA_CHAN_PWM_PACING);
  //dma_stop(DMA_CHAN_SPI_RX);
  //dma_stop(DMA_CHAN_SPI_TX);
  
  // unmap_periph_mem (&m_vc_mem);
  unmap_periph_mem (&m_regs_usec);
  unmap_periph_mem (&m_regs_pwm);
  unmap_periph_mem (&m_clk_regs);
  unmap_periph_mem (&m_regs_spi);
  unmap_periph_mem (&m_regs_dma);
  unmap_periph_mem (&m_gpio_regs);
  
    //if (fifo_name)
        //destroy_fifo(fifo_name, fifo_fd);
    //if (samp_total)
        //printf("Total samples %u, overruns %u\n", samp_total, overrun_total);
    //exit(0);
}


// // --- SPI ---
// Initialise SPI0, given desired clock freq; return actual value
int AikaPi::
spi_init (double frequency)
{
  // initialize spi gpio pins on rpi
  gpio_set (SPI0_CE0_PIN,  GPIO_ALT0, GPIO_NOPULL);
  gpio_set (SPI0_CE1_PIN,  GPIO_ALT0, GPIO_NOPULL);
  gpio_set (SPI0_MISO_PIN, GPIO_ALT0, GPIO_PULLUP);
  gpio_set (SPI0_MOSI_PIN, GPIO_ALT0, GPIO_NOPULL);
  gpio_set (SPI0_SCLK_PIN, GPIO_ALT0, GPIO_NOPULL);

  // clear tx and rx fifo. one shot operation
  spi_clear_fifo ();
  
  return (spi_set_clock_rate (frequency));
}

void AikaPi:: 
spi_clear_fifo ()
{
  *(Utility::get_reg32 (m_regs_spi, SPI_CS)) = 2 << 4;
}

// Set / clear SPI chip select
void 
AikaPi::spi_cs (int set)
{
  uint32_t csval = *REG32(m_regs_spi, SPI_CS);

  *REG32(m_regs_spi, SPI_CS) = set ? csval | 0x80 : csval & ~0x80;
}

// Transfer SPI bytes
void AikaPi::
spi_xfer (uint8_t *txd, 
          uint8_t *rxd, 
          int      length)
{
  while (length--)
    {
      *REG8(m_regs_spi, SPI_FIFO) = *txd++;
      
      // wait for rx fifo to contain AT LEAST 1 byte
      while ((*REG32(m_regs_spi, SPI_CS) & (1<<17)) == 0);
      
      *rxd++ = *REG32(m_regs_spi, SPI_FIFO);
    }
}

// Disable SPI
void 
AikaPi::spi_disable (void)
{
    *REG32(m_regs_spi, SPI_CS) = SPI_CS_CLEAR;
    *REG32(m_regs_spi, SPI_CS) = 0;
}

// Display SPI registers
void AikaPi::
spi_disp ()
{
  // SPI register strings
  const char *m_regs_spitrs[] = {"CS", 
                               "FIFO", 
                               "CLK", 
                               "DLEN", 
                               "LTOH", 
                               "DC", 
                               ""};

  int i = 0;
  volatile uint32_t *p = REG32(m_regs_spi, SPI_CS);

  while (m_regs_spitrs[i][0])
    printf("%-4s %08X ", m_regs_spitrs[i++], *p++);
  
  printf("\n");
}

// page 156
int AikaPi:: 
spi_set_clock_rate (int value)
{
  uint32_t divider = 1;

  if (value <= SPI_CLOCK_HZ)
    divider = SPI_CLOCK_HZ / value;
    
  if (divider > 65535 || divider < 0)
    divider = 65535;

  // set spi frequency as rounded off clock
  *REG32 (m_regs_spi, SPI_CLK) = divider; 

  return (SPI_CLOCK_HZ / divider);  
}

// --- Bit Bang SPI ---
int AikaPi::
bb_spi_open (unsigned CS, 
             unsigned MISO, 
             unsigned MOSI,
             unsigned SCLK, 
             unsigned baud, 
             unsigned spi_flags)
{
  m_pin_info [CS].mode  = PIN_INFO_MODE_SPI_CS;
  m_pin_info [CS].baud  = baud;

  m_pin_info [CS].spi.CS        = CS;
  m_pin_info [CS].spi.SCLK      = SCLK;
  m_pin_info [CS].spi.CS_mode   = gpio_mode (CS);
  m_pin_info [CS].spi.delay     = (500'000 / baud) - 1;
  m_pin_info [CS].spi.spi_flags = spi_flags;
  
  // the SCLK pin info field is used to store full information

  m_pin_info[SCLK].spi.usage = 1;

  m_pin_info[SCLK].spi.SCLK_mode = gpio_mode (SCLK);
  m_pin_info[SCLK].spi.MISO_mode = gpio_mode (MISO);
  m_pin_info[SCLK].spi.MOSI_mode = gpio_mode (MOSI);

  m_pin_info[SCLK].spi.SCLK = SCLK;
  m_pin_info[SCLK].spi.MISO = MISO;
  m_pin_info[SCLK].spi.MOSI = MOSI;

  m_pin_info[SCLK].mode = PIN_INFO_MODE_SPI_SCLK;
  m_pin_info[MISO].mode = PIN_INFO_MODE_SPI_MISO;
  m_pin_info[MOSI].mode = PIN_INFO_MODE_SPI_MOSI;
  
  // set modes of the pins
  gpio_mode (CS,    GPIO_MODE_OUTPUT);
  gpio_mode (MOSI,  GPIO_MODE_OUTPUT);
  gpio_mode (MISO,  GPIO_MODE_INPUT);
  gpio_mode (SCLK,  GPIO_MODE_OUTPUT);

  // set CS mode: idle high or low
  if (Utility::get_bits (spi_flags, BB_SPI_FLAG_CSPOL, 1)) 
    gpio_write (CS, 0); // active high
  else                                    
    gpio_write (CS, 1); // active low

  gpio_write (MOSI, 0);

  return 0;
}

int AikaPi:: 
bb_spi_xfer (unsigned  CS,
             char     *txbuff,
             unsigned  count)
{
  // for dump only
  char rxbuff[count];

  return (bb_spi_xfer (CS, txbuff, rxbuff, count));
}

int AikaPi:: 
bb_spi_xfer (unsigned  CS, 
             char     *txbuf, 
             char     *rxbuf, 
             unsigned  count)
{
  int SCLK;

  SCLK = m_pin_info[CS].spi.SCLK;

  m_pin_info[SCLK].spi.CS        = CS;
  m_pin_info[SCLK].baud          = m_pin_info[CS].baud;
  m_pin_info[SCLK].spi.delay     = m_pin_info[CS].spi.delay;
  m_pin_info[SCLK].spi.spi_flags = m_pin_info[CS].spi.spi_flags;

  Pin_Info *info = &(m_pin_info[SCLK]);

  bb_spi_start (info);

  for (int pos = 0; pos < count; pos++)
  {
    rxbuf[pos] = bb_spi_xfer_byte (info, txbuf[pos]);
  }

  bb_spi_stop (info);

  return count;
}

uint8_t AikaPi::
bb_spi_xfer_byte (Pin_Info *pi, 
                  char      txbyte)
{
  uint8_t bit, rxbyte = 0;

  if (Utility::get_bits (pi->spi.spi_flags, BB_SPI_FLAG_CPHA, 1))
  {
    // CPHA = 1
    // write on set clock
    // read on clear clock

    for (int bit = 0; bit < 8; bit++)
    {
      bb_spi_set_SCLK (pi);

      // if write out least significant bit first
      if (Utility::get_bits (pi->spi.spi_flags, BB_SPI_FLAG_TX_LSB, 1))
      {
        gpio_write (pi->spi.MOSI, (txbyte & 0x01));
        txbyte >>= 1;
      }
      else 
      {
        gpio_write (pi->spi.MOSI, (txbyte & 0x80));
        txbyte <<= 1;
      }

      bb_spi_delay (pi);

      bb_spi_clear_SCLK (pi);

      // if receive least significant bit first
      if (Utility::get_bits (pi->spi.spi_flags, BB_SPI_FLAG_RX_LSB, 1))
      {
        rxbyte = (rxbyte >> 1) | ((gpio_read (pi->spi.MISO)) << 7);
      }
      else 
      {
        rxbyte = (rxbyte << 1) | (gpio_read (pi->spi.MISO));
      }

      bb_spi_delay (pi);
    }
  }
  else 
  {
    // CPHA = 0
    // read on set clock
    // write on clear clock

    for (int bit = 0; bit < 8; bit++)
    {
      // if write out least significant bit first
      if (Utility::get_bits (pi->spi.spi_flags, BB_SPI_FLAG_TX_LSB, 1))
      {
        gpio_write (pi->spi.MOSI, txbyte & 0x01);
        txbyte >>= 1;
      }
      else 
      {
        gpio_write (pi->spi.MOSI, txbyte & 0x80);
        txbyte <<= 1;
      }

      bb_spi_delay (pi);

      bb_spi_set_SCLK (pi);

      // if receive least significant bit first
      if (Utility::get_bits (pi->spi.spi_flags, BB_SPI_FLAG_RX_LSB, 1))
      {
        rxbyte = (rxbyte >> 1) | ((gpio_read (pi->spi.MISO)) << 7);
      }
      else 
      {
        rxbyte = (rxbyte << 1) | (gpio_read (pi->spi.MISO));
      }

      bb_spi_delay (pi);

      bb_spi_clear_SCLK (pi);
    }
  }

  return rxbyte;
}

void AikaPi:: 
bb_spi_start (Pin_Info *pi)
{
  bb_spi_clear_SCLK (pi);

  bb_spi_delay (pi);

  bb_spi_set_CS (pi);

  bb_spi_delay (pi);
}

void AikaPi:: 
bb_spi_stop (Pin_Info *pi)
{
  bb_spi_delay (pi);

  bb_spi_clear_CS (pi);

  bb_spi_delay (pi);

  bb_spi_clear_SCLK (pi);
}

void AikaPi:: 
bb_spi_delay (Pin_Info *pi)
{
  delay (pi->spi.delay);
}

void AikaPi::
bb_spi_set_CS (Pin_Info *pi)
{
  gpio_write (pi->spi.CS, Utility::get_bits (pi->spi.spi_flags, BB_SPI_FLAG_CSPOL, 1));
}

void AikaPi::
bb_spi_clear_CS    (Pin_Info *pi)
{
  gpio_write (pi->spi.CS, !(Utility::get_bits (pi->spi.spi_flags, BB_SPI_FLAG_CSPOL, 1)));
}

void AikaPi:: 
bb_spi_set_SCLK (Pin_Info *pi)
{
  gpio_write (pi->spi.SCLK, !(Utility::get_bits (pi->spi.spi_flags, BB_SPI_FLAG_CPOL, 1)));
}

void AikaPi::
bb_spi_clear_SCLK  (Pin_Info *pi)
{
  gpio_write (pi->spi.SCLK, Utility::get_bits (pi->spi.spi_flags, BB_SPI_FLAG_CPOL, 1));
}



// --- Utility Peripherals ---
void AikaPi:: 
aux_spi1_master_enable ()
{
  Utility::reg_write (Utility::get_reg32 (m_aux_regs, AUX_ENABLES), 1, 1, 1);
}
    
void AikaPi:: 
aux_spi1_master_disable ()
{
  Utility::reg_write (Utility::get_reg32 (m_aux_regs, AUX_ENABLES), 0, 1, 1);
}



// --- Utility SPI ---
void AikaPi:: 
aux_spi0_init ()
{
  gpio_set (SPI1_SCLK_PIN,  GPIO_ALT4, GPIO_NOPULL);
  gpio_set (SPI1_MOSI_PIN,  GPIO_ALT4, GPIO_NOPULL);
  gpio_set (SPI1_MISO_PIN,  GPIO_ALT4, GPIO_PULLUP);
  gpio_set (SPI1_CE2_PIN,   GPIO_ALT4, GPIO_NOPULL);

  aux_spi1_master_enable ();
  aux_spi0_enable ();

  aux_spi0_shift_length (8);
  aux_spi0_shift_out_MS_first (1);
  aux_spi0_mode (0);
  aux_spi0_chip_selects (1, 1, 1);
  aux_spi0_frequency (100'000);    
  aux_spi0_clear_fifos ();

  printf ("spi init OK \n");
}

void AikaPi:: 
aux_spi0_enable ()
{
  Utility::reg_write (Utility::get_reg32 (m_aux_regs, AUX_SPI0_CNTL0_REG), 1, 1, 11);
}

void AikaPi::
aux_spi0_disable ()
{
  Utility::reg_write (Utility::get_reg32 (m_aux_regs, AUX_SPI0_CNTL0_REG), 0, 1, 11);
}

void AikaPi:: 
aux_spi0_frequency (double frequency)
{
  /// frequency field is only 12 bit in register! 
  uint16_t divider = (static_cast<uint16_t>((static_cast<double>(CLOCK_HZ) /
    (2.0 * frequency)) - 1)) & 0x0FFF;

  Utility::reg_write (Utility::get_reg32 (m_aux_regs, AUX_SPI0_CNTL0_REG), divider, 0xFFF, 20);
}

void AikaPi::
aux_spi0_chip_selects (bool CE2, 
                      bool CE1, 
                      bool CE0)
{
  uint8_t chip_selects = (CE2 << 2) | (CE1 << 1) | CE0;

  Utility::reg_write (Utility::get_reg32 (m_aux_regs, AUX_SPI0_CNTL0_REG), chip_selects, 
    0x03, 17);
}

void AikaPi:: 
aux_spi0_clear_fifos ()
{
  Utility::reg_write (m_aux_regs, AUX_SPI0_CNTL0_REG, 1, 1, 9);

  // maybe a small delay is required? 
  std::this_thread::sleep_for (std::chrono::milliseconds (10));

  Utility::reg_write (m_aux_regs, AUX_SPI0_CNTL0_REG, 0, 1, 9);
}

// https://www.allaboutcircuits.com/technical-articles/spi-serial-peripheral-interface/
void AikaPi::
aux_spi0_mode (uint8_t mode)
{
  // SPI Mode: CPOL, CPHA

  switch (mode)
  {
    case 0: // CPOL = 0, CPHA = 0
      aux_spi0_clock_polarity (0);
      aux_spi0_in_rising (1);
      aux_spi0_out_rising (0);
      break;
    case 1: // CPOL = 0, CPHA = 1
      aux_spi0_clock_polarity (0);
      aux_spi0_in_rising (0);
      aux_spi0_out_rising (1);
      break; 
    case 2: // CPOL = 1, CPHA = 0
      aux_spi0_clock_polarity (1);
      aux_spi0_in_rising (0);
      aux_spi0_out_rising (1);
      break;
    case 3: // CPOL = 1, CPHA = 1
      aux_spi0_clock_polarity (1);
      aux_spi0_in_rising (1);
      aux_spi0_out_rising (0);
      break;
    default: // default is mode 0
      aux_spi0_mode (0);
      break;
  }
}

void AikaPi:: 
aux_spi0_clock_polarity (bool polarity)
{
  Utility::reg_write (m_aux_regs, AUX_SPI0_CNTL0_REG, polarity, 1, 7);
}

void AikaPi::
aux_spi0_in_rising (bool value)
{
  // if 1, data is clocked in on the rising edge of the SPI clock
  // if 0, data is clocked in on the falling edge of the SPI clock
  Utility::reg_write (m_aux_regs, AUX_SPI0_CNTL0_REG, value, 1, 10);
}

void AikaPi::
aux_spi0_out_rising (bool value)
{
  // if 1, data is clocked out on the rising edge of the SPI clock
  // if 0, data is clocked out on the falling edge of the SPI clock
  Utility::reg_write (m_aux_regs, AUX_SPI0_CNTL0_REG, value, 1, 8);
}

// specifies the number of bits to shift
void AikaPi::
aux_spi0_shift_length (uint8_t value)
{
  Utility::reg_write (m_aux_regs, AUX_SPI0_CNTL0_REG, value, 6, 0);
}

void AikaPi::
aux_spi0_shift_in_MS_first (bool value)
{
  Utility::reg_write (m_aux_regs, AUX_SPI0_CNTL1_REG, value, 1, 1);
}

void AikaPi::
aux_spi0_shift_out_MS_first (bool value)
{
  Utility::reg_write (m_aux_regs, AUX_SPI0_CNTL0_REG, value, 1, 6);
  aux_spi0_shift_in_MS_first (value);
}

void AikaPi::
aux_spi0_write (char         *buf, 
               unsigned int  length)
{
  char dump[length];

  aux_spi0_read (buf, dump, length);
}

void AikaPi:: 
aux_spi0_read (char        *txbuf, 
              char         *rxbuf, 
              unsigned int  length)
{
  while (length--)
  {
    uint32_t data = 0 | ((*txbuf++) << 24);
    printf ("data to write: %08X\n", data);


    uint32_t val = Utility::get_bits (*(Utility::get_reg32 
      (m_aux_regs, AUX_SPI0_CNTL0_REG)), 6, 1);

    printf ("val is: %d\n", val);


    // write to AUXSPI0_IO Register
    *(Utility::get_reg32 (m_aux_regs, AUX_SPI0_IO_REG)) = data;


    //g_reg32_peek (m_aux_regs, AUX_SPI0_IO_REG);
    printf ("ok\n");




    // indicates the module is busy transferring data (?)
    // or should you use bit count? rx fifo level?
    while ((*(Utility::get_reg32 (m_aux_regs, AUX_SPI0_STAT_REG))) & (1 << 6))
    printf ("ok\n");

    *rxbuf++ = *(Utility::get_reg32 (m_aux_regs, AUX_SPI0_PEEK_REG));
    printf ("ok\n");
  }

  printf ("%x %x\n", rxbuf[0], rxbuf[1]);
}

void AikaPi::
aux_spi_xfer (uint8_t  channel, 
              char    *txbuff, 
              char    *rxbuff, 
              uint8_t  count)
{
  // assert CE pin

  while (count--)
  {
    uint32_t data = 0 | ((*txbuff++) << 24);

    // printf ("data to write: %08X\n", data);
    // uint32_t val = g_reg_read (m_aux_regs, AUX_SPI0_CNTL0_REG, 1, 6);
    // printf ("val is: %d\n", val);

    *(Utility::get_reg32 (m_aux_regs, AUX_SPI0_IO_REG)) = data;


    // indicates the module is busy transferring data (?)
    // or should you use bit count? rx fifo level?
    while ((*(Utility::get_reg32 (m_aux_regs, AUX_SPI0_STAT_REG))) & (1 << 6))

    *rxbuff++ = *(Utility::get_reg32 (m_aux_regs, AUX_SPI0_PEEK_REG));
  }

  // deaasert CE pin
}

void AikaPi:: 
aux_spi_write (uint8_t  channel,
               char    *txbuff,
               uint8_t  count)
{
  // for dump
  char rxbuff[count];

  aux_spi_xfer (channel, txbuff, rxbuff, count);
}



// --- GPIO ---

/*
  Sets the mode and pull-up/down resistor of the specific GPIO pin
*/
void AikaPi::
gpio_set (int pin, 
          int mode, 
          int pull)
{
  gpio_mode (pin, mode);
  gpio_pull (pin, pull);
}

/* 
  Sets the mode of the specific GPIO pin (input, output, or alt functions 0-5)
*/
void AikaPi::
gpio_mode (int pin, 
           int mode)
{
  volatile uint32_t *reg = Utility::get_reg32 (m_gpio_regs, GPFSEL0) + (pin / 10);
  unsigned shift = (pin % 10) * 3;

  Utility::reg_write (reg, mode, 0x7, shift);
}

/*
  Sets the state of the pull-up/down resistors of the specific GPIO pin
  00 - off
  01 - pull down
  10 - pull up
  11 - reserved
*/
void
AikaPi::gpio_pull (int pin,
                   int pull)
{
  volatile uint32_t *reg = REG32(m_gpio_regs, GPIO_GPPUDCLK0) + pin / 32;
  *REG32(m_gpio_regs, GPIO_GPPUD) = pull;
  usleep(2);
  
  *reg = pin << (pin % 32);
  usleep(2);
    
  *REG32(m_gpio_regs, GPIO_GPPUD) = 0;

  *reg = 0;
}

void AikaPi::
gpio_write (unsigned pin,   
            bool     value)
{
  volatile uint32_t *reg = Utility::get_reg32 (m_gpio_regs, 
    value ? GPSET0 : GPCLR0) + (pin / 32);

  *reg = 1 << (pin % 32);
}

bool 
AikaPi::gpio_read (int pin)
{
  volatile uint32_t *reg = Utility::get_reg32 (m_gpio_regs, GPIO_LEV0) + (pin / 32);

  return (((*reg) >> (pin % 32)) & 1);
}

uint32_t AikaPi:: 
gpio_mode (unsigned pin)
{
  volatile uint32_t *reg = (Utility::get_reg32 (m_gpio_regs, GPFSEL0)) + (pin / 10);

  return (Utility::get_bits (*reg, (pin % 10) * 3, 0x7));
}



// --- PWM ---
// Initialise PWM
void 
AikaPi::pwm_init (int freq, 
                               int range, 
                               int val)
{
  pwm_stop();

  // check channel 1 state
  if (*REG32(m_regs_pwm, PWM_STA) & 0x100)
    {
      printf("PWM bus error\n");
      *REG32(m_regs_pwm, PWM_STA) = 0x100;
    }

  #if USE_VC_CLOCK_SET
    set_vc_clock(mbox_fd, PWM_CLOCK_ID, freq);
  #else
    // see the BCM2385 Audio Clocks datasheet PDF for reference.
    // this is how to change PWM clock speed
    int divi = CLOCK_HZ / freq;

    // CLK_PASSWD is "5a" as written on datasheet
    // https://www.scribd.com/doc/127599939/BCM2835-Audio-clocks#download

    // max PWM operating frequency is 25MHz as written on datasheet

    // 1 << 5 = KILL: kill the clock generator
    // this line stops the clock generator
    *REG32(m_clk_regs, CLK_PWM_CTL) = CLK_PASSWD | (1 << 5);

    // 1 << 7 = BUSY: Clock generator is running
    // this line waits for BUSY to 0, or for clock generator to stop
    while (*REG32(m_clk_regs, CLK_PWM_CTL) & (1 << 7)) ;

    // divi << 12 = DIVI: Integer part of divisor
    // assign divisor to DIVI field
    *REG32(m_clk_regs, CLK_PWM_DIV) = CLK_PASSWD | (divi << 12);

    // 1 << 4 = ENAB: Enable the clock generator
    // this line asserts ENAB to enable the clock generator
    *REG32(m_clk_regs, CLK_PWM_CTL) = CLK_PASSWD | 6 | (1 << 4);

    // // 1 << 7 = BUSY: Clock generator is running
    // this line waits until BUSY is 1, this means clock generator is running
    while ((*REG32(m_clk_regs, CLK_PWM_CTL) & (1 << 7)) == 0) ;
  #endif


  usleep(100);
  *REG32(m_regs_pwm, PWM_RNG1) = range;
  *REG32(m_regs_pwm, PWM_FIF1) = val;
  usleep(100);

  gpio_set(PWM_PIN, GPIO_ALT0, GPIO_PULL_DOWN);
}

// Start PWM operation
void 
AikaPi::pwm_start ()
{
  *REG32(m_regs_pwm, PWM_CTL) = PWM_CTL_USEF1 | PWM_ENAB;
  // usleep(1000);
}

// Stop PWM operation
void 
AikaPi::pwm_stop ()
{
  *(Utility::get_reg32 (m_regs_pwm, PWM_CTL)) = 0;
  // usleep (100);
}

void
AikaPi::pwm_set_frequency (float frequency)
{
  pwm_stop();

  uint32_t range = (m_pwm_frequency * 2) / (frequency * 4);

  // check channel 1 state
  if (*REG32(m_regs_pwm, PWM_STA) & 0x100)
    {
      printf("PWM bus error\n");
      *REG32(m_regs_pwm, PWM_STA) = 0x100;
    }

  // see the BCM2385 Audio Clocks datasheet PDF for reference.
  // this is how to change PWM clock speed
  int divi = CLOCK_HZ / m_pwm_frequency;

  // CLK_PASSWD is "5a" as written on datasheet
  // https://www.scribd.com/doc/127599939/BCM2835-Audio-clocks#download

  // max PWM operating frequency is 25MHz as written on datasheet

  // 1 << 5 = KILL: kill the clock generator
  // this line stops the clock generator
  *REG32(m_clk_regs, CLK_PWM_CTL) = CLK_PASSWD | (1 << 5);

  // 1 << 7 = BUSY: Clock generator is running
  // this line waits for BUSY to 0, or for clock generator to stop
  while (*REG32(m_clk_regs, CLK_PWM_CTL) & (1 << 7)) ;

  // divi << 12 = DIVI: Integer part of divisor
  // assign divisor to DIVI field
  *REG32(m_clk_regs, CLK_PWM_DIV) = CLK_PASSWD | (divi << 12);

  // 1 << 4 = ENAB: Enable the clock generator
  // this line asserts ENAB to enable the clock generator
  *REG32(m_clk_regs, CLK_PWM_CTL) = CLK_PASSWD | 6 | (1 << 4);

  // // 1 << 7 = BUSY: Clock generator is running
  // this line waits until BUSY is 1, this means clock generator is running
  while ((*REG32(m_clk_regs, CLK_PWM_CTL) & (1 << 7)) == 0) ;

  usleep(1000);
  *REG32(m_regs_pwm, PWM_RNG1) = range;
  *REG32(m_regs_pwm, PWM_FIF1) = m_pwm_value;
  usleep(1000);

  pwm_start ();
}


// --- FIFO ---
int      
AikaPi::fifo_create (const char *fifo_name)
{
  int ok = 0;

  umask(0);

  if (mkfifo (fifo_name, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH) < 0 && errno != EEXIST)
    printf("Can't open FIFO '%s'\n", fifo_name);
  else
    ok = 1;

  return (ok);
}

int      
AikaPi::fifo_open_write (const char *fifo_name)
{
  int f = open (fifo_name, O_WRONLY | O_NONBLOCK);

  return(f == -1 ? 0 : f);
}

int      
AikaPi::fifo_write (int fd, void *data, int dlen)
{
  struct pollfd pollfd = {fd, POLLOUT, 0};

  poll(&pollfd, 1, 0);

  if (pollfd.revents&POLLOUT && !(pollfd.revents&POLLERR))
    return(fd ? write (fd, data, dlen) : 0);

  return (0);
}

uint32_t 
AikaPi::fifo_get_free_space (int fd)
{
  return (fcntl(fd, F_GETPIPE_SZ));
}

void     
AikaPi::fifo_destroy (char *fifo_name, 
                                   int   fd)
{
  if (fd > 0)
    close(fd);

  unlink (fifo_name);
}

// Check if fifo exists
int AikaPi:: 
fifo_is_fifo(const char *fname)
{
  struct stat stat_p;
  stat(fname, &stat_p);
  
  return(S_ISFIFO(stat_p.st_mode));
}

// --- Utility ---
int AikaPi:: 
sleep_nano (int nano)
{
  std::this_thread::sleep_for (std::chrono::duration<double, std::nano> 
    (nano));

  return 0;
}

// int AikaPi:: 
// sleep_secs (int secs)
// {
//   std::this_thread::sleep_for (std::chrono::duration<double, std::ratio<1/60>> 
//     (secs));
  
//   return 0;
// }


// EOF