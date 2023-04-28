#include "AikaPi.h"

#include <cmath>
#include <bitset>
#include <thread>
#include <iostream>
#include <exception>

#include <poll.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/ioctl.h>

SPI   AikaPi::spi   (reinterpret_cast<void*>(SPI0_BASE));
DMA   AikaPi::dma   (reinterpret_cast<void*>(DMA_BASE));
PWM   AikaPi::pwm   (reinterpret_cast<void*>(PWM_BASE));
GPIO  AikaPi::gpio  (reinterpret_cast<void*>(GPIO_BASE));

// --- Utility Class ---

/**
 * @brief Return a volatile uint32_t pointer of the virtual address of a 
 *        peripheral's register 
 */
volatile uint32_t* Utility::
reg (const AP_MemoryMap& mem_map,
                uint32_t      offset)
{
  uint32_t address = (uint32_t)(mem_map.virt) + offset;

  return ((volatile uint32_t*)(address));
}

/**
 * @brief Return a uint32_t of the bus address of a peripheral's 
 *        register 
 */
uint32_t Utility:: 
bus (const AP_MemoryMap& mem_map, 
               uint32_t      offset)
{
  uint32_t address = (uint32_t)(mem_map.bus) + offset;

  return (address);
}

/**
 * @brief Return a uint32_t of the bus address of an uncached
 *        peripheral's member data, given the member's virtual address. 
 */
uint32_t Utility:: 
bus (const AP_MemoryMap& mem_map,
                        void*         offset)
{
  uint32_t address = (uint32_t)(offset) - (uint32_t)(mem_map.virt) + 
    (uint32_t)(mem_map.bus);

  return (address);
}

/**
 * @brief Return a uint32_t bus address of an uncached peripheral's 
 *        member data, given the member's virtual address. 
 */
uint32_t Utility:: 
bus (const    AP_MemoryMap& mem_map,
                  volatile void*         offset)
{
  uint32_t address = (uint32_t)(offset) - (uint32_t)(mem_map.virt) + 
    (uint32_t)(mem_map.bus);

  return (address);
}

/**
 * @brief Return a uint32_t of the offset of a DMA channel's register
 */
uint32_t Utility:: 
dma_chan_reg_offset (uint32_t dma_channel, 
                     uint32_t dma_offset)
{
  uint32_t address = (dma_channel * 0x100) + dma_offset;

  return (address);
}

/**
 * @brief Print the contents of a peripheral's register 
 */
void Utility:: 
disp_reg_virt (const AP_MemoryMap& mem_map,
                     uint32_t      offset)
{
  volatile uint32_t* reg = Utility::reg (mem_map, offset);

  disp_bit32 (*reg);
}

void Utility:: 
disp_bit32 (uint32_t bits)
{
  std::cout << std::bitset <8> (bits >> 24) << " "
            << std::bitset <8> (bits >> 16) << " "
            << std::bitset <8> (bits >>  8) << " "
            << std::bitset <8> (bits      ) << std::endl;
}

void Utility:: 
write_reg_virt (volatile uint32_t* reg,
                         uint32_t  value,
                         uint32_t  mask,
                         uint32_t  shift)
{
  *reg = (*reg & ~(mask << shift)) | (value << shift);
}

void Utility:: 
write_reg_virt (AP_MemoryMap& mem_map,
                uint32_t      offset,
                uint32_t      value,
                uint32_t      mask,
                uint32_t      shift)
{
  write_reg_virt (reg (mem_map, offset), value, mask, shift);
}

uint32_t Utility:: 
get_bits (uint32_t input, 
          uint32_t shift, 
          uint32_t mask)
{
  return ((input >> shift) & mask);
}

//

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

uint32_t AikaPi::
page_roundup (uint32_t address)
{
  return ((address % PAGE_SIZE == 0) ? (address) : ((address + PAGE_SIZE) & ~(PAGE_SIZE - 1)));
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

uint32_t Mailbox::
page_roundup  (uint32_t addr)
{
  return ((addr % PAGE_SIZE == 0) ? (addr) : ((addr + PAGE_SIZE) & ~(PAGE_SIZE - 1)));
}

int Mailbox:: 
mb_open ()
{
  int fd ;

  try
  {
    if ((fd = open ("/dev/vcio", 0)) < 0)
      throw (std::runtime_error ("Can't open VC mailbox\n"));
  }
  catch (const std::runtime_error& err)
  {
    std::cerr << "Caught exception: " << err.what () << std::endl;

    std::terminate ();
  }

  return (fd);
}

void Mailbox:: 
mb_close (int fd) 
{
  close (fd);
}

uint32_t Mailbox:: 
message (int fd,
         Mailbox::MSG& msg)
{
  // https://jsandler18.github.io/extra/mailbox.html

  uint32_t ret = 0;

  for (int i = (msg.dlen) / 4; i <= (msg.blen) / 4; i += 4)
  {
    msg.uints[i++] = 0;
  }

  msg.len = (msg.blen + 6) * 4;
  msg.req = 0;

  try
  {
    // ioctl - https://man7.org/linux/man-pages/man2/ioctl.2.html
    if (ioctl (fd, _IOWR (100, 0, void*), &msg) < 0)
    {
      throw (std::runtime_error ("VC IOCTL failed"));
    }
    else if ((msg.req & 0x80000000) == 0)
    {
      throw (std::runtime_error ("VC IOCTL error\n"));
    }
    else if ((msg.req & 0x80000001) == 0)
    {
      throw (std::runtime_error ("VC IOCTL partial error\n"));
    }
    else 
    {
      ret = msg.uints[0];
    }
  }
  catch (const std::runtime_error& err)
  {
    std::cerr << "Caught exception: " << err.what () << std::endl;

    std::terminate ();
  }

  return (ret);
}

/**
 * @brief Allocates contiguous memory on the GPU.
 */
uint32_t Mailbox:: 
mem_alloc (int      fd,
           uint32_t size,
           Mailbox::ALLOC_MEM_FLAG flags)
{
  // https://github.com/raspberrypi/firmware/wiki/Mailbox-property-interface#allocate-memory
  
  Mailbox::MSG msg = 
  {
    .tag    = static_cast<uint32_t>(TAG::ALLOCATE_MEMORY),
    .blen   = 12,
    .dlen   = 12,
    .uints  =
    {
      page_roundup (size),
      PAGE_SIZE,
      static_cast<uint32_t>(flags)
    } 
  };

  return (message (fd, msg));
}

/**
 * @brief Lock buffer in place, and return a bus address. Must be done 
 *        before memory can be accessed. bus address != 0 is success.
 */
void* Mailbox:: 
mem_lock (int fd, 
          int h)
{
  // https://github.com/raspberrypi/firmware/wiki/Mailbox-property-interface#lock-memory

  Mailbox::MSG msg = 
  {
    .tag    = static_cast<uint32_t>(TAG::LOCK_MEMORY),
    .blen   = 4,
    .dlen   = 4,
    .uints  =
    {
      static_cast<uint32_t>(h)
    } 
  };

  return (h ? reinterpret_cast<void*>(message (fd, msg)) : 0);
}

/**
 * @brief Unlock buffer. It retains contents, but may move. Needs to be locked 
 *        before next use. status=0 is success.
 */
uint32_t Mailbox::
mem_unlock  (int fd,
             int h)
{
  // https://github.com/raspberrypi/firmware/wiki/Mailbox-property-interface#unlock-memory

  Mailbox::MSG msg = 
  {
    .tag    = static_cast<uint32_t>(TAG::UNLOCK_MEMORY),
    .blen   = 4,
    .dlen   = 4,
    .uints  =
    {
      static_cast<uint32_t>(h)
    } 
  };

  return (h ? message (fd, msg) : 0);
}

/**
 * @brief Free the memory buffer. status=0 is success.
 */
uint32_t Mailbox::
mem_release (int fd,
             int h)
{
  // https://github.com/raspberrypi/firmware/wiki/Mailbox-property-interface#release-memory

  Mailbox::MSG msg = 
  {
    .tag    = static_cast<uint32_t>(TAG::RELEASE_MEMORY),
    .blen   = 4,
    .dlen   = 4,
    .uints  =
    {
      static_cast<uint32_t>(h)
    } 
  };

  return (h ? message (fd, msg) : 0);
}  

Peripheral:: 
Peripheral (void* phys_addr)
  : m_phys (phys_addr)
{
  map_addresses (m_phys);
}

Peripheral:: 
Peripheral ()
{

}

Peripheral::
~Peripheral ()
{
  unmap_segment (m_virt, m_size);
}

uint32_t Peripheral:: 
page_roundup (uint32_t addr)
{
  return ((addr % PAGE_SIZE == 0) ? (addr) : ((addr + PAGE_SIZE) & ~(PAGE_SIZE - 1)));
}

void* Peripheral::
map_phys_to_virt (void*     phys_addr, 
                  unsigned  size)
{
  int   fd;
  void* mem;

  size = page_roundup (size);

  try
  {
    // open - https://man7.org/linux/man-pages/man2/open.2.html
    if ((fd = open ("/dev/mem", O_RDWR | O_SYNC | O_CLOEXEC)) < 0)
    {
      throw (std::runtime_error ("Can't open /dev/mem. Maybe you forgot to use sudo?\n"));
    }

    // mmap - https://man7.org/linux/man-pages/man2/mmap.2.html
    mem = mmap (0, size, PROT_WRITE | PROT_READ, MAP_SHARED, fd, 
      reinterpret_cast<uint32_t>(phys_addr));

    close (fd);

    if (mem == MAP_FAILED)
    {
      throw (std::runtime_error ("Can't map memory\n"));
    }
  }
  catch (const std::runtime_error& err)
  {
    std::cerr << "Caught exception: " << err.what () << std::endl;

    std::terminate ();
  }

  return (mem);
}

void Peripheral:: 
map_addresses (void* phys_addr)
{
  m_phys  = phys_addr;
  m_size  = page_roundup (PAGE_SIZE);
  m_bus   = reinterpret_cast<uint8_t*>(phys_addr) - 
            reinterpret_cast<uint8_t*>(PHYS_REG_BASE) + 
            reinterpret_cast<uint8_t*>(BUS_REG_BASE);
  m_virt  = map_phys_to_virt (phys_addr, m_size);
}

void* Peripheral::
conv_bus_to_phys (void* bus_addr)
{
  uint32_t phys_addr = (reinterpret_cast<uint32_t>(bus_addr) & ~0xC0000000);

  return (reinterpret_cast<void*>(phys_addr));
}

void Peripheral::   
unmap_segment (void*    virt_addr, 
               unsigned size)
{
  // munmap - https://man7.org/linux/man-pages/man3/munmap.3p.html
  munmap (virt_addr, page_roundup (size));
}

/**
 * @brief Returns a volatile uint32_t pointer to the virtual address of a 
 *        peripheral's register 
 */
volatile uint32_t* Peripheral::
reg (uint32_t offset) const
{
  uint32_t addr = reinterpret_cast<uint32_t>(m_virt) + offset;

  return (reinterpret_cast<volatile uint32_t*>(addr));
}

/**
 * @brief Changes the entire contents of a peripheral's register 
 * @param offset offset to the register
 * @param value new uint32_t value of the register 
 */
void Peripheral::  
reg (uint32_t offset, 
     uint32_t value)
{
  *(reg (offset)) = value;
}

/**
 * @brief Returns a user-defined number of bits from a peripheral's register 
 */
uint32_t Peripheral::
reg_bits (uint32_t offset, 
          unsigned shift, 
          uint32_t mask) const
{
  uint32_t bits = *(reg (offset));

  return ((bits >> shift) & mask);
}

/**
 * @brief Writes a user-defined number of bits to a peripheral's register 
 */
void Peripheral::
reg_bits (uint32_t offset, 
          uint32_t value, 
          unsigned shift, 
          uint32_t mask) 
{
  *(reg (offset)) = (*(reg (offset)) & ~(mask << shift)) | (value << shift); 
}

/**
 * @brief Returns the bus address of a peripheral's register 
 */
uint32_t Peripheral::
bus (uint32_t offset) const
{
  uint32_t addr = reinterpret_cast<uint32_t>(m_bus) + offset;

  return (addr);
}

/**
 * @brief Displays the entire content of a peripheral's register in 32-bit form 
 */
void Peripheral::    
disp_reg (uint32_t offset) const
{
  uint32_t bits = *(reg (offset));

  print (bits);
}

/**
 * @brief Returns the peripheral's base bus address 
 */
void* Peripheral::
bus () const
{
  return (m_bus);
}

/**
 * @brief Returns the peripheral's base virtual address 
 */
void* Peripheral:: 
virt () const
{
  return (m_virt);
}

/**
 * @brief Returns the peripheral's base physical address 
 */
void* Peripheral::
phys () const
{
  return (m_phys);
}

/**
 * @brief prints a uint32_t value in 32-bit MSB form, 
 *        with spaces between each byte
 */
void Peripheral::
print (uint32_t value)
{
  std::cout << std::bitset <8> (value >> 24) << " "
            << std::bitset <8> (value >> 16) << " "
            << std::bitset <8> (value >>  8) << " "
            << std::bitset <8> (value      ) << std::endl;
}

SPI:: 
SPI (void* phys_addr) : Peripheral (phys_addr)
{

}

SPI::
~SPI ()
{

}

double SPI::
clock_rate (double frequency)
{
  uint32_t divisor;

  if (frequency <= SPI_CLOCK_HZ)
  {
    divisor = SPI_CLOCK_HZ / std::abs(frequency);
  }

  if (divisor > 65'536)
  {
    divisor = 65'536;
  }

  reg (SPI_CLK, divisor);

  return (SPI_CLOCK_HZ / divisor);
}

void SPI:: 
clear_fifo ()
{
  reg (SPI_CS, 2 << 4);
}

DMA::
DMA (void* phys_addr) : Peripheral (phys_addr)
{

}

DMA:: 
~DMA ()
{
  
}

uint32_t DMA:: 
dma_chan_reg_offset (unsigned chan, uint32_t offset) const
{
  return ((chan * 0x100) + offset);
}

volatile uint32_t* DMA::
reg (unsigned dma_chan, 
          uint32_t offset) const
{
  return (Peripheral::reg (dma_chan_reg_offset (dma_chan, offset)));
}

void DMA::
reg (unsigned dma_chan, 
     uint32_t offset, 
     uint32_t value)
{
  Peripheral::reg (dma_chan_reg_offset (dma_chan, offset), value);
}

uint32_t DMA::
reg_bits (unsigned dma_chan, 
          uint32_t offset, 
          unsigned shift, 
          uint32_t mask) const
{
  return (Peripheral::reg_bits (dma_chan_reg_offset (dma_chan, offset), shift, mask));
}

void DMA::
reg_bits (unsigned dma_chan, 
          uint32_t offset,
          unsigned value, 
          unsigned shift, 
          uint32_t mask)
{
  Peripheral::reg_bits (dma_chan_reg_offset (dma_chan, offset), value, shift, mask);
}

void DMA::
disp_reg (unsigned dma_chan,
          uint32_t offset) const
{
  Peripheral::disp_reg (dma_chan_reg_offset (dma_chan, offset));
}

uint32_t DMA:: 
dest_ad (unsigned dma_chan)
{
  return (*(reg (dma_chan, DMA_DEST_AD)));
}

void DMA:: 
start (unsigned dma_chan,
       uint32_t start_cb_bus_addr)
{
  reg (dma_chan, DMA_CONBLK_AD, start_cb_bus_addr);
  reg (dma_chan, DMA_CS,    2); // Clear DMA End flag
  reg (dma_chan, DMA_DEBUG, 7); // Clear error flags
  reg (dma_chan, DMA_CS,    1); // Set ACTIVE bit to start DMA
}

void DMA::
reset (unsigned dma_chan)
{
  reg (dma_chan, DMA_CS, 1 << 31);

  std::this_thread::sleep_for (std::chrono::duration<double, std::micro> (10.0));
}

bool DMA::
is_running (unsigned dma_chan) const
{
  return (reg_bits (dma_chan, DMA_CS, 4));
}

void DMA::
pause (unsigned dma_chan)
{
  reg_bits (dma_chan, DMA_CS, 0, 0);
}

void DMA::
next_cb (unsigned dma_chan, 
         uint32_t next_cb_bus_addr)
{
  reg (dma_chan, DMA_NEXTCONBK, next_cb_bus_addr);
}

void DMA::
abort (unsigned dma_chan)
{
  reg_bits (dma_chan, DMA_CS, 1, 30);
}

void DMA::
run (unsigned dma_chan)
{
  reg_bits (dma_chan, DMA_CS, 1, 0);
}

void DMA::
stop (unsigned dma_chan)
{
  reset (dma_chan);
}

uint32_t DMA::
conblk_ad (unsigned dma_chan) const
{
  return (*(reg (dma_chan, DMA_CONBLK_AD)));
}

Uncached:: 
Uncached (unsigned size) : Peripheral ()
{
  map_uncached_mem (size);
}

Uncached:: 
Uncached () : Peripheral ()
{

}

Uncached::
~Uncached ()
{
  Mailbox::mem_unlock   (m_fd, m_h);
  Mailbox::mem_release  (m_fd, m_h);
  Mailbox::mb_close     (m_fd);
}

void* Uncached:: 
map_uncached_mem (unsigned size)
{
  m_size  = page_roundup      (size);
  m_fd    = Mailbox::mb_open  ();

  // hehe sorry
  Mailbox::ALLOC_MEM_FLAG flags = static_cast<Mailbox::ALLOC_MEM_FLAG> (
    static_cast<uint32_t>(Mailbox::ALLOC_MEM_FLAG::COHERENT) | 
    static_cast<uint32_t>(Mailbox::ALLOC_MEM_FLAG::ZERO)
  );

  try 
  {
    if ((m_h = Mailbox::mem_alloc (m_fd, m_size, flags)) <= 0)
    {
      throw (std::runtime_error ("mem_alloc failure\n"));
    }
    else if ((m_bus = Mailbox::mem_lock (m_fd, m_h)) == 0)
    {
      throw (std::runtime_error ("mem_lock failure\n"));
    }
    else if ((m_virt = map_phys_to_virt (conv_bus_to_phys (m_bus), m_size)) == 0)
    {
      throw (std::runtime_error ("map_phys_to_virt failure\n"));
    }
  }
  catch (const std::runtime_error& err)
  {
    std::cerr << "Caught exception: " << err.what () << std::endl;

    std::terminate ();
  }

  return (m_virt);  
}

/**
 * @brief Return a uint32_t bus address of the uncached mem's member variable 
 */
uint32_t Uncached:: 
bus (void* offset) const volatile
{
  uint32_t addr = reinterpret_cast<uint32_t>(offset) - 
                  reinterpret_cast<uint32_t>(m_virt) + 
                  reinterpret_cast<uint32_t>(m_bus);
  
  return (addr);
}

/**
 * @brief Return a uint32_t bus address of the uncached mem's member variable 
 */
uint32_t Uncached:: 
bus (volatile void* offset) const volatile
{
  return (bus (const_cast<void*>(offset)));
}

PWM:: 
PWM (void* phys_addr) : Peripheral (phys_addr)
{

}

PWM:: 
~PWM ()
{
  stop (0);
  stop (1);
}

void PWM::
start (unsigned channel)
{
  reg_bits (PWM_CTL, 1, channel ? 8 : 0);
}

void PWM:: 
stop(unsigned channel)
{
  reg_bits (PWM_CTL, 0, channel ? 8 : 0);
}

void PWM::
algo (unsigned    channel, 
      AP_PWM_ALGO algo)
{
  reg_bits (PWM_CTL, algo, channel ? 15 : 7);
}

void PWM::
use_fifo (unsigned channel, 
          bool     value)
{
  reg_bits (PWM_CTL, value, channel ? 13 : 5);
}

GPIO::
GPIO (void* phys_addr) : Peripheral (phys_addr)
{

}

GPIO::
~GPIO ()
{

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


/**
 * @brief Map physical memory segment to the virtual address space
 */
void* AikaPi:: 
map_segment (void*    phys_addr,
             unsigned size)
{
  int   fd;
  void* mem;

  size = page_roundup (size);

  try
  {
    // open - https://man7.org/linux/man-pages/man2/open.2.html
    if ((fd = open ("/dev/mem", O_RDWR | O_SYNC | O_CLOEXEC)) < 0)
    {
      throw (std::runtime_error ("Can't open /dev/mem. Maybe you forgot to use sudo?\n"));
    }

    // mmap - https://man7.org/linux/man-pages/man2/mmap.2.html
    mem = mmap (0, size, PROT_WRITE | PROT_READ, MAP_SHARED, fd, reinterpret_cast<uint32_t>(phys_addr));

    close (fd);

    if (mem == MAP_FAILED)
    {
      throw (std::runtime_error ("Can't map memory.\n"));
    }
  }
  catch (const std::runtime_error& err)
  {
    std::cerr << "Caught exception: " << err.what () << std::endl;

    std::terminate ();
  }

  return (mem);
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
                   int           chan, 
                   AP_DMA_CB    *cbp, 
                   uint32_t      csval)
{
  *REG32(m_regs_dma, DMA_REG(chan, DMA_CONBLK_AD)) = MEM_BUS_ADDR(mp, cbp);
  *REG32(m_regs_dma, DMA_REG(chan, DMA_CS))        = 2;         // Clear 'end' flag
  *REG32(m_regs_dma, DMA_REG(chan, DMA_DEBUG))     = 7;         // Clear error bits
  *REG32(m_regs_dma, DMA_REG(chan, DMA_CS))        = 1 | csval; // Start DMA
}

// void AikaPi:: 
// dma_start (unsigned      channel,
//            AP_MemoryMap *uncached_dma_data,
//            AP_DMA_CB    *dma_cb)
// {
//   *(Utility::reg (m_regs_dma, Utility::dma_chan_reg_offset (channel,
//     DMA_CONBLK_AD))) = Utility::bus (*uncached_dma_data, (uint32_t)(dma_cb));
  
//   *(Utility::reg (m_regs_dma, Utility::dma_chan_reg_offset (channel,
//     DMA_CS))) = 1 << 1; // clear DMA End Flag
  
//   *(Utility::reg (m_regs_dma, Utility::dma_chan_reg_offset (channel,
//     DMA_DEBUG))) = 7; // clear erorr flags

//   *(Utility::reg (m_regs_dma, Utility::dma_chan_reg_offset (channel,
//     DMA_CS))) = 1; // start DMA
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
  // if (m_regs_dma.virt)
  //   *REG32(m_regs_dma, DMA_REG(chan, DMA_CS)) = 1 << 31;

  *REG32(m_regs_dma, DMA_REG(chan, DMA_CS)) = 1 << 31;
  
  usleep(10);
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
  volatile uint32_t *reg = Utility::reg (m_regs_dma, 
    DMA_REG (channel, DMA_CS));

  Utility::write_reg_virt (reg, 0, 1, 0);

  // wait for the PAUSED flag to be 1
  while (!(Utility::get_bits (*reg, 4, 1)));

  usleep(10);
}

bool 
AikaPi::is_dma_paused (unsigned channel)
{
  volatile uint32_t *reg = Utility::reg (m_regs_dma, 
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
  volatile uint32_t *reg = Utility::reg (m_regs_dma, 
    DMA_REG (channel, DMA_CS));

  Utility::write_reg_virt (reg, 1, 1, 0);

  // wait for the PAUSED flag to be 0
  while (Utility::get_bits (*reg, 4, 1));
}

// Abort the current DMA control block. 
// The DMA will load the next control block and attempt to continue.
void 
AikaPi::dma_abort (unsigned channel)
{
  Utility::write_reg_virt (Utility::reg (m_regs_dma, 
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
      PAGE_SIZE, 
      flags
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
  //dma_reset(LAB_DMA_CHAN_OSC_RX);
  //dma_reset(LAB_DMA_CHAN_OSC_TX);
  
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
  gpio_set (SPI0_MISO_PIN, AP_GPIO_FUNC_ALT0, AP_GPIO_PULL_DOWN);
  gpio_set (SPI0_MOSI_PIN, AP_GPIO_FUNC_ALT0, AP_GPIO_PULL_OFF);
  gpio_set (SPI0_SCLK_PIN, AP_GPIO_FUNC_ALT0, AP_GPIO_PULL_OFF);

  // clear tx and rx fifo. one shot operation
  spi.clear_fifo ();

  spi.clock_rate (frequency);
  
  return (1);
}

void AikaPi:: 
spi_clear_fifo ()
{
  *(Utility::reg (m_regs_spi, SPI_CS)) = 2 << 4;
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

  std::cout << "spi clock rate divider: " << divider << "\n";

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
    gpio_write(CS, 0); // active high
  else                                    
    gpio_write(CS, 1); // active low

  gpio_write(MOSI, 0);

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
        gpio_write(pi->spi.MOSI, (txbyte & 0x01));
        txbyte >>= 1;
      }
      else 
      {
        gpio_write(pi->spi.MOSI, (txbyte & 0x80));
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
        gpio_write(pi->spi.MOSI, txbyte & 0x01);
        txbyte >>= 1;
      }
      else 
      {
        gpio_write(pi->spi.MOSI, txbyte & 0x80);
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
  gpio_write(pi->spi.CS, Utility::get_bits (pi->spi.spi_flags, BB_SPI_FLAG_CSPOL, 1));
}

void AikaPi::
bb_spi_clear_CS    (Pin_Info *pi)
{
  gpio_write(pi->spi.CS, !(Utility::get_bits (pi->spi.spi_flags, BB_SPI_FLAG_CSPOL, 1)));
}

void AikaPi:: 
bb_spi_set_SCLK (Pin_Info *pi)
{
  gpio_write(pi->spi.SCLK, !(Utility::get_bits (pi->spi.spi_flags, BB_SPI_FLAG_CPOL, 1)));
}

void AikaPi::
bb_spi_clear_SCLK  (Pin_Info *pi)
{
  gpio_write(pi->spi.SCLK, Utility::get_bits (pi->spi.spi_flags, BB_SPI_FLAG_CPOL, 1));
}



// --- Utility Peripherals ---
void AikaPi:: 
aux_spi1_master_enable ()
{
  Utility::write_reg_virt (Utility::reg (m_aux_regs, AUX_ENABLES), 1, 1, 1);
}
    
void AikaPi:: 
aux_spi1_master_disable ()
{
  Utility::write_reg_virt (Utility::reg (m_aux_regs, AUX_ENABLES), 0, 1, 1);
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
  Utility::write_reg_virt (Utility::reg (m_aux_regs, AUX_SPI0_CNTL0_REG), 1, 1, 11);
}

void AikaPi::
aux_spi0_disable ()
{
  Utility::write_reg_virt (Utility::reg (m_aux_regs, AUX_SPI0_CNTL0_REG), 0, 1, 11);
}

void AikaPi:: 
aux_spi0_frequency (double frequency)
{
  /// frequency field is only 12 bit in register! 
  uint16_t divider = (static_cast<uint16_t>((static_cast<double>(CLOCK_HZ) /
    (2.0 * frequency)) - 1)) & 0x0FFF;

  Utility::write_reg_virt (Utility::reg (m_aux_regs, AUX_SPI0_CNTL0_REG), divider, 0xFFF, 20);
}

void AikaPi::
aux_spi0_chip_selects (bool CE2, 
                      bool CE1, 
                      bool CE0)
{
  uint8_t chip_selects = (CE2 << 2) | (CE1 << 1) | CE0;

  Utility::write_reg_virt (Utility::reg (m_aux_regs, AUX_SPI0_CNTL0_REG), chip_selects, 
    0x03, 17);
}

void AikaPi:: 
aux_spi0_clear_fifos ()
{
  Utility::write_reg_virt (m_aux_regs, AUX_SPI0_CNTL0_REG, 1, 1, 9);

  // maybe a small delay is required? 
  std::this_thread::sleep_for (std::chrono::milliseconds (10));

  Utility::write_reg_virt (m_aux_regs, AUX_SPI0_CNTL0_REG, 0, 1, 9);
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
  Utility::write_reg_virt (m_aux_regs, AUX_SPI0_CNTL0_REG, polarity, 1, 7);
}

void AikaPi::
aux_spi0_in_rising (bool value)
{
  // if 1, data is clocked in on the rising edge of the SPI clock
  // if 0, data is clocked in on the falling edge of the SPI clock
  Utility::write_reg_virt (m_aux_regs, AUX_SPI0_CNTL0_REG, value, 1, 10);
}

void AikaPi::
aux_spi0_out_rising (bool value)
{
  // if 1, data is clocked out on the rising edge of the SPI clock
  // if 0, data is clocked out on the falling edge of the SPI clock
  Utility::write_reg_virt (m_aux_regs, AUX_SPI0_CNTL0_REG, value, 1, 8);
}

// specifies the number of bits to shift
void AikaPi::
aux_spi0_shift_length (uint8_t value)
{
  Utility::write_reg_virt (m_aux_regs, AUX_SPI0_CNTL0_REG, value, 6, 0);
}

void AikaPi::
aux_spi0_shift_in_MS_first (bool value)
{
  Utility::write_reg_virt (m_aux_regs, AUX_SPI0_CNTL1_REG, value, 1, 1);
}

void AikaPi::
aux_spi0_shift_out_MS_first (bool value)
{
  Utility::write_reg_virt (m_aux_regs, AUX_SPI0_CNTL0_REG, value, 1, 6);
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


    uint32_t val = Utility::get_bits (*(Utility::reg 
      (m_aux_regs, AUX_SPI0_CNTL0_REG)), 6, 1);

    printf ("val is: %d\n", val);


    // write to AUXSPI0_IO Register
    *(Utility::reg (m_aux_regs, AUX_SPI0_IO_REG)) = data;


    //g_reg32_peek (m_aux_regs, AUX_SPI0_IO_REG);
    printf ("ok\n");

    // indicates the module is busy transferring data (?)
    // or should you use bit count? rx fifo level?
    while ((*(Utility::reg (m_aux_regs, AUX_SPI0_STAT_REG))) & (1 << 6))
    printf ("ok\n");

    *rxbuf++ = *(Utility::reg (m_aux_regs, AUX_SPI0_PEEK_REG));
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

    *(Utility::reg (m_aux_regs, AUX_SPI0_IO_REG)) = data;


    // indicates the module is busy transferring data (?)
    // or should you use bit count? rx fifo level?
    while ((*(Utility::reg (m_aux_regs, AUX_SPI0_STAT_REG))) & (1 << 6))

    *rxbuff++ = *(Utility::reg (m_aux_regs, AUX_SPI0_PEEK_REG));
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
  gpio_write  (pin, value);
}

void AikaPi::
AP_gpio_func (unsigned     pin, 
              AP_GPIO_FUNC _AP_GPIO_FUNC)
{
  volatile uint32_t *reg = Utility::reg (m_regs_gpio, GPFSEL0) + (pin / 10);
  unsigned shift = (pin % 10) * 3;

  Utility::write_reg_virt (reg, _AP_GPIO_FUNC, 0x7, shift);
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
gpio_write (unsigned pin,   
               bool     value)
{
  volatile uint32_t *reg = Utility::reg (m_regs_gpio, 
    value ? GPSET0 : GPCLR0) + (pin / 32);

  *reg = 1 << (pin % 32);
}

bool 
AikaPi::AP_gpio_read (unsigned pin)
{
  volatile uint32_t *reg = Utility::reg (m_regs_gpio, GPIO_LEV0) + (pin / 32);

  return ((*reg >> (pin % 32)) & 1);
}

uint32_t AikaPi:: 
AP_gpio_func (unsigned pin)
{
  volatile uint32_t *reg = (Utility::reg (m_regs_gpio, GPFSEL0)) + (pin / 10);

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
    *(Utility::reg (m_regs_pwm, PWM_CTL)) |= 1 << 0;

    return 1;
  }
  else if (channel == 1)
  {
    *(Utility::reg (m_regs_pwm, PWM_CTL)) |= 1 << 8;

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
    Utility::write_reg_virt (Utility::reg (m_regs_pwm, PWM_CTL), 0, 1, 0);

    return 1;
  }
  else if (channel == 1)
  {
    Utility::write_reg_virt (Utility::reg (m_regs_pwm, PWM_CTL), 0, 1, 8);

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
    Utility::write_reg_virt (Utility::reg (m_regs_pwm, PWM_CTL), value, 1,
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
    Utility::write_reg_virt (Utility::reg (m_regs_pwm, PWM_CTL), value, 1,
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
    Utility::write_reg_virt (Utility::reg (m_regs_pwm, PWM_CTL), value, 1,
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
    Utility::write_reg_virt (Utility::reg (m_regs_pwm, PWM_CTL), value, 1,
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
    return (Utility::get_bits (*(Utility::reg (m_regs_pwm, PWM_STA)), 
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
  *(Utility::reg (m_regs_pwm, PWM_FIF1)) = value;

  return 0;
}

int 
AikaPi::pwm_range (unsigned channel, 
                   uint32_t value)
{
  if (channel == 1 || channel == 2)
  {
    *(Utility::reg (m_regs_pwm, (channel == 2) ? PWM_RNG2 : PWM_RNG1)) = value;

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
  *Utility::reg (m_regs_pwm, PWM_CTL)   = 0x00000040;
  *Utility::reg (m_regs_pwm, PWM_STA)   = 0x000001FE;
  *Utility::reg (m_regs_pwm, PWM_RNG1)  = 0x00000020;

  return 1;
}

void AikaPi:: 
pwm_fifo_clear ()
{
  *(Utility::reg (m_regs_pwm, PWM_CTL)) |= (1 << 6);
}

void AikaPi:: 
pwm_algo (unsigned    channel,
          AP_PWM_ALGO _AP_PWM_ALGO)
{
  Utility::write_reg_virt (Utility::reg (m_regs_pwm, PWM_CTL), _AP_PWM_ALGO,
    1, (channel == 0 ? 7 : 15));
}

double AikaPi::
pwm_frequency (unsigned channel,
               double   frequency)
{
  if (channel == 0 || channel == 1)
  {
    m_pwm_range = m_pwm_clk_src_freq / frequency;

    *(Utility::reg (m_regs_pwm, (channel == 0 ? PWM_RNG1 : PWM_RNG2))) = 
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

    *(Utility::reg (m_regs_pwm, PWM_DAT1)) = fifo_data;
    *(Utility::reg (m_regs_pwm, PWM_FIF1)) = fifo_data;

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
  volatile uint32_t *p = Utility::reg (m_regs_cm, CM_PCMCTL);

  *p = *p | (1 << 5);

  while (((*p) >> 7) & 0x01);
}

bool AikaPi:: 
cm_pwm_clk_is_running ()
{
  uint32_t reg = *(Utility::reg (m_regs_cm, CM_PWMCTL));

  return (((reg >> 7) & 0x1) ? true : false);
}

double AikaPi:: 
cm_pwm_clk_divisor ()
{
  uint32_t reg = *(Utility::reg (m_regs_cm, CM_PWMDIV));

  double integral   = (reg >> 12) & 0xfff;
  double fractional = (reg & 0xfff) * std::pow (10, -12);

  return (integral + fractional);
}

// Stop AND RESET the clock generator
void AikaPi:: 
cm_pwm_clk_stop ()
{
  volatile uint32_t *reg = Utility::reg (m_regs_cm, CM_PWMCTL);

  *reg = (CM_PASSWD) | (1 << 5);

  std::this_thread::sleep_for (std::chrono::microseconds (10));

  //while (cm_pwm_clk_is_running ());
}

void AikaPi:: 
cm_pwm_clk_run ()
{
  volatile uint32_t *reg = Utility::reg (m_regs_cm, CM_PWMCTL);

  *reg |= (CM_PASSWD) | (1 << 4);

  while (!Utility::get_bits (*reg, 7, 1));
}

void AikaPi:: 
cm_pcm_clk_run ()
{
  volatile uint32_t *p = Utility::reg (m_regs_cm, CM_PCMCTL);

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
    volatile uint32_t *reg = Utility::reg (m_regs_cm, CM_PWMCTL);

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
  volatile uint32_t *reg = Utility::reg (m_regs_cm, CM_PWMCTL);
  volatile uint32_t *div = Utility::reg (m_regs_cm, CM_PWMDIV);

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
    volatile uint32_t *reg = Utility::reg (m_regs_cm, CM_PWMCTL);

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
    volatile uint32_t *reg = Utility::reg (m_regs_cm, CM_PWMDIV);

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



// EOF