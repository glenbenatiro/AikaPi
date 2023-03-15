#include "AikaPi.h"

#include <iostream>
#include <bitset>
#include <thread>
#include <chrono>
#include <cstdio>
#include <cmath>
#include <cstdint>

#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/stat.h>

// --- Utility Class ---

uint32_t Utility:: 
get_bits (uint32_t input, unsigned shift, uint32_t mask)
{
  return ((input >> shift) & mask);
}

void Utility:: 
reg_write (AP_MemoryMap mem_map, 
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
AikaPi::map_uncached_mem (AP_MemoryMap *mp,
                          int        size)
{
  void *ret;
  
  mp->size = PAGE_ROUNDUP(size);
  mp->fd   = mailbox_open ();
  
  ret = (mp->h    = vc_mem_alloc (mp->fd, mp->size, DMA_MEM_FLAGS))   > 0 &&
        (mp->bus  = vc_mem_lock  (mp->fd, mp->h))                    != 0 &&
        (mp->virt = map_segment  (BUS_PHYS_ADDR(mp->bus), mp->size)) != 0 
        ? mp->virt : 0;
        
  // printf("VC mem handle %u, phys %p, virt %p\n", mp->h, mp->bus, mp->virt);
  
  return(ret);
}

void
AikaPi::map_devices ()
{
  map_periph (&m_aux_regs,  (void *) AUX_BASE,   PAGE_SIZE);
  map_periph (&m_regs_gpio, (void *) GPIO_BASE,  PAGE_SIZE);
  map_periph (&m_regs_dma,  (void *) DMA_BASE,   PAGE_SIZE);
  map_periph (&m_regs_spi,  (void *) SPI0_BASE,  PAGE_SIZE);
  map_periph (&m_regs_cm,   (void *) CM_BASE,    PAGE_SIZE);
  map_periph (&m_regs_pwm,  (void *) PWM_BASE,   PAGE_SIZE);
  map_periph (&m_regs_st,   (void *) AP_ST_BASE, PAGE_SIZE);
}

// --- Memory ---
void
AikaPi::map_periph (AP_MemoryMap *mp, void *phys, int size)
{
  mp->phys = phys;
  mp->size = PAGE_ROUNDUP(size);
  mp->bus  = (uint8_t *)phys - (uint8_t *)PHYS_REG_BASE + (uint8_t *)BUS_REG_BASE;
  mp->virt = map_segment (phys, mp->size);
}

// Free mapped peripheral or memory
void 
AikaPi::unmap_periph_mem (AP_MemoryMap *mp)
{
  if (mp)
    {
      if (mp->fd)
        {
          unmap_segment (mp->virt, mp->size);
          vc_mem_unlock (mp->fd, mp->h);
          vc_mem_release   (mp->fd, mp->h);
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
AikaPi::dma_start (AP_MemoryMap *mp, 
                   int        chan, 
                   AP_DMA_CB    *cbp, 
                   uint32_t   csval)
{
  *REG32(m_regs_dma, DMA_REG(chan, DMA_CONBLK_AD)) = MEM_BUS_ADDR(mp, cbp);
  *REG32(m_regs_dma, DMA_REG(chan, DMA_CS))        = 2;         // Clear 'end' flag
  *REG32(m_regs_dma, DMA_REG(chan, DMA_DEBUG))     = 7;         // Clear error bits
  *REG32(m_regs_dma, DMA_REG(chan, DMA_CS))        = 1 | csval; // Start DMA
}

void AikaPi:: 
dma_start (unsigned      channel,
           AP_MemoryMap *uncached_dma_data,
           AP_DMA_CB    *dma_cb)
{
  *(Utility::get_reg32 (m_regs_dma, Utility::dma_chan_reg_offset (channel,
    DMA_CONBLK_AD))) = Utility::mem_bus_addr (uncached_dma_data, dma_cb);
  
  *(Utility::get_reg32 (m_regs_dma, Utility::dma_chan_reg_offset (channel,
    DMA_CS))) = 1 << 1; // clear DMA End Flag
  
  *(Utility::get_reg32 (m_regs_dma, Utility::dma_chan_reg_offset (channel,
    DMA_DEBUG))) = 7; // clear erorr flags

  *(Utility::get_reg32 (m_regs_dma, Utility::dma_chan_reg_offset (channel,
    DMA_CS))) = 1; // start DMA
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
AikaPi::dma_reset (int chan)
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
  volatile uint32_t *reg = Utility::get_reg32 (m_regs_dma, 
    DMA_REG (channel, DMA_CS));

  Utility::reg_write (reg, 0, 1, 0);

  // wait for the PAUSED flag to be 1
  while (!(Utility::get_bits (*reg, 4, 1)));
}

bool 
AikaPi::is_dma_paused (unsigned channel)
{
  volatile uint32_t *reg = Utility::get_reg32 (m_regs_dma, 
    DMA_REG (channel, DMA_CS));

  if (Utility::get_bits (*reg, 4, 1))
  {
    return (true);
  }
  else 
  {
    return (false);
  }
}

void 
AikaPi::dma_play (unsigned channel)
{
  volatile uint32_t *reg = Utility::get_reg32 (m_regs_dma, 
    DMA_REG (channel, DMA_CS));

  Utility::reg_write (reg, 1, 1, 0);

  // wait for the PAUSED flag to be 0
  while (Utility::get_bits (*reg, 4, 1));
}

// Abort the current DMA control block. 
// The DMA will load the next control block and attempt to continue.
void 
AikaPi::dma_abort (unsigned channel)
{
  Utility::reg_write (Utility::get_reg32 (m_regs_dma, 
    DMA_REG (channel, DMA_CS)), 1, 1, 30);
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
AikaPi::mbox_msg (int     fd, 
                               AP_VC_MSG *msgp)
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

// Allocates contiguous memory on the GPU. Size and alignment are in bytes.
// https://github.com/raspberrypi/firmware/wiki/Mailbox-property-interface#allocate-memory
uint32_t 
AikaPi::vc_mem_alloc (int      fd, 
                      uint32_t size,
                      MAILBOX_ALLOCATE_MEMORY_FLAGS flags)
{
  AP_VC_MSG msg = 
  {
    .tag   = MAILBOX_TAG_ALLOCATE_MEMORY,
    .blen  = 12,
    .dlen  = 12,
    .uints =  
    {
      PAGE_ROUNDUP(size), 
      PAGE_SIZE, flags
    }
  };
                          
  return (mbox_msg (fd, &msg));
}

// Lock buffer in place, and return a bus address. Must be done before memory
// can be accessed. bus address != 0 is success
// https://github.com/raspberrypi/firmware/wiki/Mailbox-property-interface#lock-memory
void*
AikaPi::vc_mem_lock (int fd, 
                     int h)
{
  AP_VC_MSG msg = 
  {
    .tag   = 0x3000d, 
    .blen  = 4, 
    .dlen  = 4, 
    .uints = 
    {
      static_cast<uint32_t>(h)
    }
 };
  
  return (h ? (void *)mbox_msg (fd, &msg) : 0);
}

// Unlock buffer. It retains contents, but may move. 
// Needs to be locked before next use. status=0 is success.
// https://github.com/raspberrypi/firmware/wiki/Mailbox-property-interface#unlock-memory
uint32_t 
AikaPi::vc_mem_unlock (int fd, 
                       int h)
{
  AP_VC_MSG msg = 
  {
    .tag   = 0x3000e, 
    .blen  = 4, 
    .dlen  = 4, 
    .uints = 
    {
      static_cast<uint32_t>(h)
    }
  };
  
  return (h ? mbox_msg (fd, &msg) : 0);
}

// Free the memory buffer. status=0 is success.
// https://github.com/raspberrypi/firmware/wiki/Mailbox-property-interface#release-memory
uint32_t 
AikaPi::vc_mem_release (int fd, 
                        int h)
{
  AP_VC_MSG msg = 
  {
    .tag     = 0x3000f, 
    .blen    = 4,
    .dlen    = 4, 
    .uints   = 
    {
      static_cast<uint32_t>(h)
    }
  };
  
  return (h ? mbox_msg (fd, &msg) : 0);
}

uint32_t 
AikaPi::fset_vc_clock (int      fd, 
                                    int      id, 
                                    uint32_t freq)
{
  AP_VC_MSG msg1 = {.tag   = 0x38001, 
                 .blen  = 8, 
                 .dlen  = 8, 
                 .uints = {static_cast<uint32_t>(id), 1}};
                 
  AP_VC_MSG msg2 = {.tag   = 0x38002, 
                 .blen  = 12,
                 .dlen  = 12,
                 .uints = {static_cast<uint32_t>(id), freq, 0}};
  
  mbox_msg    (fd, &msg1);
  disp_vc_msg (&msg1);
  
  mbox_msg    (fd, &msg2);
  disp_vc_msg (&msg2);
  
  return(0);
}

// Display mailbox message
void 
AikaPi::disp_vc_msg(AP_VC_MSG *msgp)
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
  //dma_reset(LAB_DMA_CHAN_PWM_PACING);
  //dma_reset(LAB_DMA_CHAN_OSCILLOSCOPE_SPI_RX);
  //dma_reset(LAB_DMA_CHAN_OSCILLOSCOPE_SPI_TX);
  
  // unmap_periph_mem (&m_vc_mem);
  unmap_periph_mem (&m_regs_st);
  unmap_periph_mem (&m_regs_pwm);
  unmap_periph_mem (&m_regs_cm);
  unmap_periph_mem (&m_regs_spi);
  unmap_periph_mem (&m_regs_dma);
  unmap_periph_mem (&m_regs_gpio);
  
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
  gpio_set (SPI0_CE0_PIN,  AP_GPIO_FUNC_ALT0, AP_GPIO_PULL_OFF);
  gpio_set (SPI0_CE1_PIN,  AP_GPIO_FUNC_ALT0, AP_GPIO_PULL_OFF);
  gpio_set (SPI0_MISO_PIN, AP_GPIO_FUNC_ALT0, AP_GPIO_PULL_UP);
  gpio_set (SPI0_MOSI_PIN, AP_GPIO_FUNC_ALT0, AP_GPIO_PULL_OFF);
  gpio_set (SPI0_SCLK_PIN, AP_GPIO_FUNC_ALT0, AP_GPIO_PULL_OFF);

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
                               "LE_WAVE_TYPE_DC", 
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
  m_pin_info [CS].spi.CS_mode   = AP_gpio_func (CS);
  m_pin_info [CS].spi.delay     = (500'000 / baud) - 1;
  m_pin_info [CS].spi.spi_flags = spi_flags;
  
  // the SCLK pin info field is used to store full information

  m_pin_info[SCLK].spi.usage = 1;

  m_pin_info[SCLK].spi.SCLK_mode = AP_gpio_func (SCLK);
  m_pin_info[SCLK].spi.MISO_mode = AP_gpio_func (MISO);
  m_pin_info[SCLK].spi.MOSI_mode = AP_gpio_func (MOSI);

  m_pin_info[SCLK].spi.SCLK = SCLK;
  m_pin_info[SCLK].spi.MISO = MISO;
  m_pin_info[SCLK].spi.MOSI = MOSI;

  m_pin_info[SCLK].mode = PIN_INFO_MODE_SPI_SCLK;
  m_pin_info[MISO].mode = PIN_INFO_MODE_SPI_MISO;
  m_pin_info[MOSI].mode = PIN_INFO_MODE_SPI_MOSI;
  
  // set modes of the pins
  AP_gpio_func (CS,    AP_GPIO_FUNC_OUTPUT);
  AP_gpio_func (MOSI,  AP_GPIO_FUNC_OUTPUT);
  AP_gpio_func (MISO,  AP_GPIO_FUNC_INPUT);
  AP_gpio_func (SCLK,  AP_GPIO_FUNC_OUTPUT);

  // set CS mode: idle high or low
  if (Utility::get_bits (spi_flags, BB_SPI_FLAG_CSPOL, 1)) 
    AP_gpio_write(CS, 0); // active high
  else                                    
    AP_gpio_write(CS, 1); // active low

  AP_gpio_write(MOSI, 0);

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
        AP_gpio_write(pi->spi.MOSI, (txbyte & 0x01));
        txbyte >>= 1;
      }
      else 
      {
        AP_gpio_write(pi->spi.MOSI, (txbyte & 0x80));
        txbyte <<= 1;
      }

      bb_spi_delay (pi);

      bb_spi_clear_SCLK (pi);

      // if receive least significant bit first
      if (Utility::get_bits (pi->spi.spi_flags, BB_SPI_FLAG_RX_LSB, 1))
      {
        rxbyte = (rxbyte >> 1) | ((AP_gpio_read (pi->spi.MISO)) << 7);
      }
      else 
      {
        rxbyte = (rxbyte << 1) | (AP_gpio_read (pi->spi.MISO));
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
        AP_gpio_write(pi->spi.MOSI, txbyte & 0x01);
        txbyte >>= 1;
      }
      else 
      {
        AP_gpio_write(pi->spi.MOSI, txbyte & 0x80);
        txbyte <<= 1;
      }

      bb_spi_delay (pi);

      bb_spi_set_SCLK (pi);

      // if receive least significant bit first
      if (Utility::get_bits (pi->spi.spi_flags, BB_SPI_FLAG_RX_LSB, 1))
      {
        rxbyte = (rxbyte >> 1) | ((AP_gpio_read (pi->spi.MISO)) << 7);
      }
      else 
      {
        rxbyte = (rxbyte << 1) | (AP_gpio_read (pi->spi.MISO));
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
  AP_gpio_write(pi->spi.CS, Utility::get_bits (pi->spi.spi_flags, BB_SPI_FLAG_CSPOL, 1));
}

void AikaPi::
bb_spi_clear_CS    (Pin_Info *pi)
{
  AP_gpio_write(pi->spi.CS, !(Utility::get_bits (pi->spi.spi_flags, BB_SPI_FLAG_CSPOL, 1)));
}

void AikaPi:: 
bb_spi_set_SCLK (Pin_Info *pi)
{
  AP_gpio_write(pi->spi.SCLK, !(Utility::get_bits (pi->spi.spi_flags, BB_SPI_FLAG_CPOL, 1)));
}

void AikaPi::
bb_spi_clear_SCLK  (Pin_Info *pi)
{
  AP_gpio_write(pi->spi.SCLK, Utility::get_bits (pi->spi.spi_flags, BB_SPI_FLAG_CPOL, 1));
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
  gpio_set (SPI1_SCLK_PIN,  AP_GPIO_FUNC_ALT4, AP_GPIO_PULL_OFF);
  gpio_set (SPI1_MOSI_PIN,  AP_GPIO_FUNC_ALT4, AP_GPIO_PULL_OFF);
  gpio_set (SPI1_MISO_PIN,  AP_GPIO_FUNC_ALT4, AP_GPIO_PULL_UP);
  gpio_set (SPI1_CE2_PIN,   AP_GPIO_FUNC_ALT4, AP_GPIO_PULL_OFF);

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

  // deassert CE pin
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
void AikaPi::
gpio_set (unsigned pin, 
             AP_GPIO_FUNC _AP_GPIO_FUNC, 
             AP_GPIO_PULL _AP_GPIO_PULL)
{
  AP_gpio_func (pin, _AP_GPIO_FUNC);
  AP_gpio_pull (pin, _AP_GPIO_PULL);
}

void AikaPi::
gpio_set (unsigned pin, 
             AP_GPIO_FUNC _AP_GPIO_FUNC, 
             AP_GPIO_PULL _AP_GPIO_PULL,
             bool value)
{
  AP_gpio_func   (pin, _AP_GPIO_FUNC);
  AP_gpio_pull   (pin, _AP_GPIO_PULL);
  AP_gpio_write  (pin, value);
}

void AikaPi::
AP_gpio_func (unsigned     pin, 
              AP_GPIO_FUNC _AP_GPIO_FUNC)
{
  volatile uint32_t *reg = Utility::get_reg32 (m_regs_gpio, GPFSEL0) + (pin / 10);
  unsigned shift = (pin % 10) * 3;

  Utility::reg_write (reg, _AP_GPIO_FUNC, 0x7, shift);
}

void
AikaPi::AP_gpio_pull (unsigned     pin,
                      AP_GPIO_PULL _AP_GPIO_PULL)
{
  volatile uint32_t *reg = REG32(m_regs_gpio, GPIO_GPPUDCLK0) + pin / 32;
  *REG32(m_regs_gpio, GPIO_GPPUD) = _AP_GPIO_PULL;
  usleep(2);
  
  *reg = pin << (pin % 32);
  usleep(2);
    
  *REG32(m_regs_gpio, GPIO_GPPUD) = 0;

  *reg = 0;
}

void AikaPi::
AP_gpio_write (unsigned pin,   
               bool     value)
{
  volatile uint32_t *reg = Utility::get_reg32 (m_regs_gpio, 
    value ? GPSET0 : GPCLR0) + (pin / 32);

  *reg = 1 << (pin % 32);
}

bool 
AikaPi::AP_gpio_read (unsigned pin)
{
  volatile uint32_t *reg = Utility::get_reg32 (m_regs_gpio, GPIO_LEV0) + (pin / 32);

  return ((*reg >> (pin % 32)) & 1);
}

uint32_t AikaPi:: 
AP_gpio_func (unsigned pin)
{
  volatile uint32_t *reg = (Utility::get_reg32 (m_regs_gpio, GPFSEL0)) + (pin / 10);

  return (Utility::get_bits (*reg, (pin % 10) * 3, 0x7));
}



// --- PWM ---
// Initialise PWMs
int AikaPi:: 
pwm_init (double          pwm_freq,
          double          pwm_src_clk_freq,
          AP_CM_CLK_SRC   _AP_CM_CLK_SRC,
          AP_CM_CLK_MASH  _AP_CM_CLK_MASH)
{
  pwm_reset ();

  cm_pwm_clk_init (pwm_src_clk_freq, _AP_CM_CLK_SRC, _AP_CM_CLK_MASH);

  // set frequency
  pwm_frequency (0, pwm_freq);
  pwm_frequency (1, pwm_freq);

  // set duty cycle
  pwm_duty_cycle (0, 50.0);

  return 1;
}

// Start PWM operation
int AikaPi::
pwm_start (unsigned channel)
{
  if (channel == 0)
  {
    *(Utility::get_reg32 (m_regs_pwm, PWM_CTL)) |= 1 << 0;

    return 1;
  }
  else if (channel == 1)
  {
    *(Utility::get_reg32 (m_regs_pwm, PWM_CTL)) |= 1 << 8;

    return 1;
  }
  else 
  {
    return 0;
  }
}

// Stop PWM operation
int AikaPi::
pwm_stop (unsigned channel)
{
  if (channel == 0)
  {
    Utility::reg_write (Utility::get_reg32 (m_regs_pwm, PWM_CTL), 0, 1, 0);

    return 1;
  }
  else if (channel == 1)
  {
    Utility::reg_write (Utility::get_reg32 (m_regs_pwm, PWM_CTL), 0, 1, 8);

    return 1;
  }
  else 
  {
    return 0;
  }
}

// For more information, see this addendum: 
// https://www.scribd.com/doc/127599939/BCM2835-Audio-clocks
// Max PWM operating frequency is 25MHz as written on datasheet



int
AikaPi::pwm_enable (unsigned channel, 
                    bool     value)
{
  if (channel == 1 || channel == 2)
  {
    Utility::reg_write (Utility::get_reg32 (m_regs_pwm, PWM_CTL), value, 1,
      (channel == 2) ? 8 : 0);

    return value;
  }
  else 
  {
    return -1;
  }
}

int 
AikaPi::pwm_mode (unsigned channel,
                  int      value)
{
  if ((channel == 1 || channel == 2) & (value == 0 || value == 1))
  {
    Utility::reg_write (Utility::get_reg32 (m_regs_pwm, PWM_CTL), value, 1,
      (channel == 2) ? 9 : 1);

    return value;
  }
  else 
  {
    return -1;
  }
}

// 0: transmission interrupts when FIFO is empty
// 1: last data in FIFO is transmitted repeatedly until FIFO is not empty
int       
AikaPi::pwm_repeat_last_data  (unsigned channel, 
                               bool     value)
{
  if (channel == 1 || channel == 2)
  {
    Utility::reg_write (Utility::get_reg32 (m_regs_pwm, PWM_CTL), value, 1,
      (channel == 2) ? 10 : 2);

    return value;
  }
  else 
  {
    return -1;
  }
}

// 0: data register is transmitted
// 1: FIFO is used for transmission
int       
AikaPi::pwm_use_fifo  (unsigned channel, 
                       bool     value)
{
  if (channel == 0 || channel == 1)
  {
    Utility::reg_write (Utility::get_reg32 (m_regs_pwm, PWM_CTL), value, 1,
      (channel == 0) ? 5 : 13);

    return 1;
  }
  else 
  {
    return -1;
  }
}

// 0: channel is not currently transmitting
// 1: channel is transmitting data
bool       
AikaPi::pwm_channel_state (unsigned channel)
{
  if (channel >= 1 && channel <=4)
  {
    return (Utility::get_bits (*(Utility::get_reg32 (m_regs_pwm, PWM_STA)), 
      8 + channel, 1));
  }
  else 
  {
    return -1;
  }
}

int
AikaPi::pwm_fifo (uint32_t value)
{
  *(Utility::get_reg32 (m_regs_pwm, PWM_FIF1)) = value;

  return 0;
}

int 
AikaPi::pwm_range (unsigned channel, 
                   uint32_t value)
{
  if (channel == 1 || channel == 2)
  {
    *(Utility::get_reg32 (m_regs_pwm, (channel == 2) ? PWM_RNG2 : PWM_RNG1)) = value;

    return value;
  }
  else 
  {
    return -1;
  }
}

// Set the Control, Range, and Data registers of the PWM channel to 0
int
AikaPi::pwm_reset ()
{
  *Utility::get_reg32 (m_regs_pwm, PWM_CTL)   = 0x00000040;
  *Utility::get_reg32 (m_regs_pwm, PWM_STA)   = 0x000001FE;
  *Utility::get_reg32 (m_regs_pwm, PWM_RNG1)  = 0x00000020;

  return 1;
}

void AikaPi:: 
pwm_fifo_clear ()
{
  *(Utility::get_reg32 (m_regs_pwm, PWM_CTL)) |= (1 << 6);
}

void AikaPi:: 
pwm_algo (unsigned    channel,
          AP_PWM_ALGO _AP_PWM_ALGO)
{
  Utility::reg_write (Utility::get_reg32 (m_regs_pwm, PWM_CTL), _AP_PWM_ALGO,
    1, (channel == 0 ? 7 : 15));
}

double AikaPi::
pwm_frequency (unsigned channel,
               double   frequency)
{
  if (channel == 0 || channel == 1)
  {
    m_pwm_range = m_pwm_clk_src_freq / frequency;

    *(Utility::get_reg32 (m_regs_pwm, (channel == 0 ? PWM_RNG1 : PWM_RNG2))) = 
      static_cast<uint32_t>(m_pwm_range);

    return (m_pwm_range);
  }
  else 
  {
    return -1;
  }
}

double AikaPi::
pwm_duty_cycle (unsigned  channel, 
                double    duty_cycle)
{
  if (duty_cycle >= 0 || channel == 0 || channel == 1)
  {
    double dc_percentage = (std::fmod (duty_cycle, 100.0)) / (100.0);
    double fifo_data = m_pwm_range * dc_percentage;

    *(Utility::get_reg32 (m_regs_pwm, PWM_DAT1)) = fifo_data;
    *(Utility::get_reg32 (m_regs_pwm, PWM_FIF1)) = fifo_data;

    return dc_percentage;
  }
  else 
  {
    return -1;
  }
}





// --- Clock Manager Audio Clocks ---
void AikaPi:: 
cm_pcm_clk_stop ()
{
  volatile uint32_t *p = Utility::get_reg32 (m_regs_cm, CM_PCMCTL);

  *p = *p | (1 << 5);

  while (((*p) >> 7) & 0x01);
}

bool AikaPi:: 
cm_pwm_clk_is_running ()
{
  uint32_t reg = *(Utility::get_reg32 (m_regs_cm, CM_PWMCTL));

  return (((reg >> 7) & 0x1) ? true : false);
}

double AikaPi:: 
cm_pwm_clk_divisor ()
{
  uint32_t reg = *(Utility::get_reg32 (m_regs_cm, CM_PWMDIV));

  double integral   = (reg >> 12) & 0xfff;
  double fractional = (reg & 0xfff) * std::pow (10, -12);

  return (integral + fractional);
}

// Stop AND RESET the clock generator
void AikaPi:: 
cm_pwm_clk_stop ()
{
  volatile uint32_t *reg = Utility::get_reg32 (m_regs_cm, CM_PWMCTL);

  *reg = (CM_PASSWD) | (1 << 5);

  std::this_thread::sleep_for (std::chrono::microseconds (10));

  //while (cm_pwm_clk_is_running ());
}

void AikaPi:: 
cm_pwm_clk_run ()
{
  volatile uint32_t *reg = Utility::get_reg32 (m_regs_cm, CM_PWMCTL);

  *reg |= (CM_PASSWD) | (1 << 4);

   while (!Utility::get_bits (*reg, 7, 1))
  {
    std::cout << "waiting for clock to run!\n";
    Utility::disp_reg32 (m_regs_cm, CM_PWMCTL);
    std::cout << "\n\n";
  }
}

void AikaPi:: 
cm_pcm_clk_run ()
{
  volatile uint32_t *p = Utility::get_reg32 (m_regs_cm, CM_PCMCTL);

  *p = *p | CM_PASSWD | (1 << 4);

  while (((*p)>> 7) & 0x00);
}

// Do not call this function while CM'S BUSY = 1, and do not 
// change this control at the same time as asserting ENAB
int AikaPi:: 
cm_pwm_clk_src (AP_CM_CLK_SRC _AP_CM_CLK_SRC)
{
  if (cm_pwm_clk_is_running ())
  {
    return 0;
  }
  else 
  {    
    volatile uint32_t *reg = Utility::get_reg32 (m_regs_cm, CM_PWMCTL);

    *reg |= (CM_PASSWD) | (_AP_CM_CLK_SRC << 0);

    std::this_thread::sleep_for (std::chrono::microseconds (10));

    return 1;
  }  
}

int AikaPi:: 
cm_pwm_clk_init (double         pwm_clk_src_freq, 
                 AP_CM_CLK_SRC  _AP_CM_CLK_SRC, 
                 AP_CM_CLK_MASH _AP_CM_CLK_MASH)
{
  volatile uint32_t *reg = Utility::get_reg32 (m_regs_cm, CM_PWMCTL);
  volatile uint32_t *div = Utility::get_reg32 (m_regs_cm, CM_PWMDIV);

  m_pwm_clk_src_freq = pwm_clk_src_freq;

  // 1.) Stop the clock generator
  *reg = (CM_PASSWD) | (1 << 5);

  // 2.) Wait until clock generator is stopped by checking BUSY flag
  while (((*reg) >> 7) & 0x1);

  // 3.) Calculate the divisor, given the PWM source clock frequency 
  //     from the given PWM clock source (AP_CM_CLK_SRC)
  
  // double divisor = (AP_CM_CLK_SRC_FREQ.at (_AP_CM_CLK_SRC)) / pwm_clk_src_freq;
  double divisor = (500'000'000.0) / pwm_clk_src_freq;

  double integral;
  double fractional = std::modf (divisor, &integral);

  fractional = fractional * std::pow (10, 12);

  // 4.) Set the divisor integral and fractional parts
  *div = (CM_PASSWD) | (static_cast<uint32_t>(integral) & 0xfff) << 12 |
    (static_cast<uint32_t>(fractional) & 0xfff);

  // 5.) Set the clock source, clock MASH, and enable the clock
  *reg = (CM_PASSWD) | (_AP_CM_CLK_MASH << 9) | (1 << 4) | (_AP_CM_CLK_SRC << 0);

  // 6.) Wait for the clock generator to be actually running by checking BUSY flag
  while ((((*reg) >> 7) & 0x1) == 0);
  
  return 1;
}

int AikaPi:: 
cm_pwm_clk_mash (AP_CM_CLK_MASH _AP_CM_CLK_MASH)
{
  if (cm_pwm_clk_is_running ())
  {
    return 0;
  }
  else 
  {
    volatile uint32_t *reg = Utility::get_reg32 (m_regs_cm, CM_PWMCTL);

    *reg |= (CM_PASSWD) | (_AP_CM_CLK_MASH << 9);

    std::this_thread::sleep_for (std::chrono::microseconds (100));

    return 1;
  }
}

double AikaPi::
cm_pwm_clk_freq (double value)
{
  if (cm_pwm_clk_is_running ())
  {
    return -1;
  }
  else 
  {
    volatile uint32_t *reg = Utility::get_reg32 (m_regs_cm, CM_PWMDIV);

    m_pwm_clk_src_freq = value;

    // for now, this is 500MHz for PLLD.
    // change this soon.
    double divisor = static_cast<double>(500000000.0) / value;

    double integral;
    double fractional = std::modf (divisor, &integral);
    
    fractional = fractional * std::pow (10, 12);

    // DEBUG
    printf ("integral  : %9.12f\n", integral);
    printf ("fractional: %9.12f\n", fractional);
    //

    
    
    std::this_thread::sleep_for (std::chrono::microseconds (100));
    
    return value;
  }
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