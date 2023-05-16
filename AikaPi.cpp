#include "AikaPi.h"

#include <cmath>
#include <bitset>
#include <string>
#include <thread>
#include <iostream>
#include <exception>

#include <poll.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/ioctl.h>

AikaPi::ClockManager  AikaPi::cm    (reinterpret_cast<void*>(AP::CLKMAN::BASE)); // cm should come before pwm!
AikaPi::SPI           AikaPi::spi   (reinterpret_cast<void*>(AP::SPI::BASE));
AikaPi::DMA           AikaPi::dma   (reinterpret_cast<void*>(AP::DMA::BASE));
AikaPi::PWM           AikaPi::pwm   (reinterpret_cast<void*>(AP::PWM::BASE));
AikaPi::GPIO          AikaPi::gpio  (reinterpret_cast<void*>(AP::GPIO::BASE));
AikaPi::AUX           AikaPi::aux   (reinterpret_cast<void*>(AP::AUX::BASE));
AikaPi::SystemTimer   AikaPi::st    (reinterpret_cast<void*>(AP::SYSTIMER::BASE));

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

// Mailbox
uint32_t AikaPi::Mailbox::
page_roundup (uint32_t addr)
{
  return ((addr % PAGE_SIZE == 0) ? (addr) : ((addr + PAGE_SIZE) & ~(PAGE_SIZE - 1)));
}

int AikaPi::Mailbox:: 
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

void AikaPi::Mailbox:: 
mb_close (int fd) 
{
  close (fd);
}

uint32_t AikaPi::Mailbox:: 
message (int fd,
         AikaPi::Mailbox::MSG& msg)
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
uint32_t AikaPi::Mailbox:: 
mem_alloc (int      fd,
           uint32_t size,
           AikaPi::Mailbox::ALLOC_MEM_FLAG flags)
{
  // https://github.com/raspberrypi/firmware/wiki/Mailbox-property-interface#allocate-memory
  
  AikaPi::Mailbox::MSG msg = 
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
void* AikaPi::Mailbox:: 
mem_lock (int fd, 
          int h)
{
  // https://github.com/raspberrypi/firmware/wiki/Mailbox-property-interface#lock-memory

  MSG msg = 
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
uint32_t AikaPi::Mailbox::
mem_unlock  (int fd,
             int h)
{
  // https://github.com/raspberrypi/firmware/wiki/Mailbox-property-interface#unlock-memory

  AikaPi::Mailbox::MSG msg = 
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
uint32_t AikaPi::Mailbox::
mem_release (int fd,
             int h)
{
  // https://github.com/raspberrypi/firmware/wiki/Mailbox-property-interface#release-memory

  MSG msg = 
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

AikaPi::Peripheral:: 
Peripheral (void* phys_addr)
  : m_phys (phys_addr)
{
  map_addresses (m_phys);
}

AikaPi::Peripheral:: 
Peripheral ()
{

}

AikaPi::Peripheral::
~Peripheral ()
{
  unmap_segment (m_virt, m_size);
}

uint32_t AikaPi::Peripheral:: 
page_roundup (uint32_t addr)
{
  return ((addr % PAGE_SIZE == 0) ? (addr) : ((addr + PAGE_SIZE) & ~(PAGE_SIZE - 1)));
}

void* AikaPi::Peripheral::
map_phys_to_virt (void*     phys_addr, 
                  unsigned  size)
{
  int   fd;
  void* mem;

  size = page_roundup (size);

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
    std::string msg ("Can't map memory: " + std::string (strerror (errno)));

    throw (std::runtime_error (msg));
  }
  
  return (mem);
}

void AikaPi::Peripheral:: 
map_addresses (void* phys_addr)
{
  m_phys  = phys_addr;
  m_size  = page_roundup (PAGE_SIZE);
  m_bus   = reinterpret_cast<uint8_t*>(phys_addr) - 
            reinterpret_cast<uint8_t*>(PHYS_REG_BASE) + 
            reinterpret_cast<uint8_t*>(BUS_REG_BASE);
  m_virt  = map_phys_to_virt (phys_addr, m_size);
}

void* AikaPi::Peripheral::
conv_bus_to_phys (void* bus_addr)
{
  uint32_t phys_addr = (reinterpret_cast<uint32_t>(bus_addr) & ~0xC0000000);

  return (reinterpret_cast<void*>(phys_addr));
}

void AikaPi::Peripheral::   
unmap_segment (void*    virt_addr, 
               unsigned size)
{
  // munmap - https://man7.org/linux/man-pages/man3/munmap.3p.html
  munmap (virt_addr, page_roundup (size));
}

constexpr uint32_t AikaPi::Peripheral::
wbits (uint32_t data, 
       uint32_t value, 
       uint32_t shift, 
       uint32_t mask)
{
  return ((data & ~(mask << shift)) | (value << shift));
}

uint32_t AikaPi::Peripheral:: 
rreg (uint32_t* reg) 
{
  return (*reg);
}

void AikaPi::Peripheral:: 
wreg (uint32_t* reg, uint32_t value) 
{
  *reg = value;
}

uint32_t AikaPi::Peripheral::
rbits (uint32_t* reg, 
       uint32_t  shift, 
       uint32_t  mask) 
{
  return ((*reg >> shift) & mask);
}

void AikaPi::Peripheral::  
wbits (uint32_t* reg, 
       uint32_t  value, 
       uint32_t  shift, 
       uint32_t  mask) 
{
  *reg = wbits (*reg, value, shift, mask);
}

uint32_t AikaPi::Peripheral:: 
rreg (volatile uint32_t* reg) 
{
  return (rreg (const_cast<uint32_t*>(reg)));
}

void AikaPi::Peripheral:: 
wreg (volatile uint32_t* reg, uint32_t value)
{
  wreg (const_cast<uint32_t*>(reg), value);
}

uint32_t AikaPi::Peripheral::
rbits (volatile uint32_t*  reg, 
                uint32_t   shift, 
                uint32_t   mask)
{
  return (rbits (const_cast<uint32_t*>(reg), shift, mask));
}

void AikaPi::Peripheral::  
wbits (volatile uint32_t*  reg, 
       uint32_t   value, 
       uint32_t   shift, 
       uint32_t   mask) 
{
  wbits (const_cast<uint32_t*>(reg), value, shift, mask);
}

/**
 * @brief Returns a volatile uint32_t pointer to the virtual address of a 
 *        peripheral's register 
 */
volatile uint32_t* AikaPi::Peripheral::
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
void AikaPi::Peripheral::  
reg (uint32_t offset, 
     uint32_t value)
{
  wreg (reg (offset), value);
}

/**
 * @brief Returns a user-defined number of bits from a peripheral's register 
 */
uint32_t AikaPi::Peripheral::
reg_rbits (uint32_t offset, 
           uint32_t shift, 
           uint32_t mask) const
{
  return (rbits (reg (offset), shift, mask));
}

/**
 * @brief Writes a user-defined number of bits to a peripheral's register 
 */
void AikaPi::Peripheral::
reg_wbits (uint32_t offset, 
          uint32_t value, 
          uint32_t shift, 
          uint32_t mask)
{
  wbits (reg (offset), value, shift, mask);
}

/**
 * @brief Returns the bus address of a peripheral's register 
 */
uint32_t AikaPi::Peripheral::
bus (uint32_t offset) const
{
  uint32_t addr = reinterpret_cast<uint32_t>(m_bus) + offset;

  return (addr);
}

/**
 * @brief Returns the peripheral's base bus address 
 */
void* AikaPi::Peripheral::
bus () const
{
  return (m_bus);
}

/**
 * @brief Returns the peripheral's base virtual address 
 */
void* AikaPi::Peripheral:: 
virt () const
{
  return (m_virt);
}

/**
 * @brief Returns the peripheral's base physical address 
 */
void* AikaPi::Peripheral::
phys () const
{
  return (m_phys);
}

/**
 * @brief Displays the entire content of a peripheral's register in 32-bit form 
 */
void AikaPi::Peripheral::    
disp_reg (uint32_t offset) const
{
  print_u32 (rreg (reg (offset)));
}

/**
 * @brief prints a uint32_t value in 32-bit MSB form, 
 *        with spaces between each byte
 */
void AikaPi::Peripheral::
print_u32 (uint32_t value)
{
  std::cout << std::bitset <8> (value >> 24) << " "
            << std::bitset <8> (value >> 16) << " "
            << std::bitset <8> (value >>  8) << " "
            << std::bitset <8> (value      ) << std::endl;
}

AikaPi::SPI:: 
SPI (void* phys_addr) : Peripheral (phys_addr)
{

}

AikaPi::SPI::
~SPI ()
{

}

double AikaPi::SPI::
frequency (double value)
{
  uint32_t divisor;

  if (value <= AP::RPI::SPI_CLOCK_HZ && value > 0)
  {
    divisor = std::round (AP::RPI::SPI_CLOCK_HZ / value);
  }
  else 
  {
    throw (std::runtime_error ("Invalid frequency value."));
  }

  if (divisor > 65'536)
  {
    divisor = 65'536;
  }

  reg (AP::SPI::CLK, divisor);

  return (AP::RPI::SPI_CLOCK_HZ / divisor);
}

void AikaPi::SPI:: 
clear_fifo ()
{
  reg (SPI_CS, 2 << 4);
}

AikaPi::DMA::
DMA (void* phys_addr) : Peripheral (phys_addr)
{
  init ();
}

AikaPi::DMA:: 
~DMA ()
{
  
}

uint32_t AikaPi::DMA:: 
dma_chan_reg_offset (unsigned chan, uint32_t offset) const
{
  return ((chan * 0x100) + offset);
}

void AikaPi::DMA:: 
init ()
{
  // reset DMA chans 0 to 14. skip chan 15 for now as it has a different offset
  for (int a = 0; a < AP::DMA::CHAN_COUNT - 1; a++)
  {
    reset (a);
  }
}

volatile uint32_t* AikaPi::DMA::
reg (unsigned dma_chan, 
          uint32_t offset) const
{
  return (AikaPi::Peripheral::reg (dma_chan_reg_offset (dma_chan, offset)));
}

void AikaPi::DMA::
reg (unsigned dma_chan, 
     uint32_t offset, 
     uint32_t value)
{
  AikaPi::Peripheral::reg (dma_chan_reg_offset (dma_chan, offset), value);
}

uint32_t AikaPi::DMA::
reg_rbits (unsigned dma_chan, 
          uint32_t offset, 
          unsigned shift, 
          uint32_t mask) const
{
  return (AikaPi::Peripheral::reg_rbits (dma_chan_reg_offset (dma_chan, offset), shift, mask));
}

void AikaPi::DMA::
reg_wbits (unsigned dma_chan, 
          uint32_t offset,
          unsigned value, 
          unsigned shift, 
          uint32_t mask)
{
  AikaPi::Peripheral::reg_wbits (dma_chan_reg_offset (dma_chan, offset), value, shift, mask);
}

void AikaPi::DMA::
disp_reg (unsigned dma_chan,
          uint32_t offset) const
{
  AikaPi::Peripheral::disp_reg (dma_chan_reg_offset (dma_chan, offset));
}

uint32_t AikaPi::DMA:: 
dest_ad (unsigned dma_chan)
{
  return (*(reg (dma_chan, DMA_DEST_AD)));
}

void AikaPi::DMA:: 
start (unsigned dma_chan,
       uint32_t start_cb_bus_addr)
{
  reg (dma_chan, AP::DMA::CONBLK_AD, start_cb_bus_addr);
  reg (dma_chan, AP::DMA::CS,    2); // Clear DMA End flag
  reg (dma_chan, AP::DMA::DEBUG, 7); // Clear error flags
  reg (dma_chan, AP::DMA::CS,    1); // Set ACTIVE bit to start DMA
}

void AikaPi::DMA::
reset (unsigned dma_chan)
{
  reg (dma_chan, DMA_CS, 1 << 31);

  std::this_thread::sleep_for (std::chrono::microseconds (10));
}

bool AikaPi::DMA::
is_running (unsigned dma_chan) const
{
  return (reg_rbits (dma_chan, DMA_CS, 4));
}

void AikaPi::DMA::
pause (unsigned dma_chan)
{
  reg_wbits (dma_chan, DMA_CS, 0, 0);
}

void AikaPi::DMA::
next_cb (unsigned dma_chan, 
         uint32_t next_cb_bus_addr)
{
  reg (dma_chan, DMA_NEXTCONBK, next_cb_bus_addr);
}

void AikaPi::DMA::
abort (unsigned dma_chan)
{
  reg_wbits (dma_chan, DMA_CS, 1, 30);
}

void AikaPi::DMA::
run (unsigned dma_chan)
{
  reg_wbits (dma_chan, DMA_CS, 1, 0);
}

void AikaPi::DMA::
stop (unsigned dma_chan)
{
  reset (dma_chan);
}

uint32_t AikaPi::DMA::
conblk_ad (unsigned dma_chan) const
{
  return (*(reg (dma_chan, DMA_CONBLK_AD)));
}

AikaPi::Uncached:: 
Uncached (unsigned size) : Peripheral ()
{
  map_uncached_mem (size);
}

AikaPi::Uncached:: 
Uncached () : Peripheral ()
{

}

AikaPi::Uncached::
~Uncached ()
{
  AikaPi::Mailbox::mem_unlock   (m_fd, m_h);
  AikaPi::Mailbox::mem_release  (m_fd, m_h);
  AikaPi::Mailbox::mb_close     (m_fd);
}

void* AikaPi::Uncached:: 
map_uncached_mem (unsigned size)
{
  m_size  = page_roundup      (size);
  m_fd    = AikaPi::Mailbox::mb_open  ();

  // hehe sorry
  AikaPi::Mailbox::ALLOC_MEM_FLAG flags = static_cast<AikaPi::Mailbox::ALLOC_MEM_FLAG> (
    static_cast<uint32_t>(AikaPi::Mailbox::ALLOC_MEM_FLAG::COHERENT) | 
    static_cast<uint32_t>(AikaPi::Mailbox::ALLOC_MEM_FLAG::ZERO)
  );

  try 
  {
    if ((m_h = AikaPi::Mailbox::mem_alloc (m_fd, m_size, flags)) <= 0)
    {
      throw (std::runtime_error ("mem_alloc failure\n"));
    }
    else if ((m_bus = AikaPi::Mailbox::mem_lock (m_fd, m_h)) == 0)
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
uint32_t AikaPi::Uncached:: 
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
uint32_t AikaPi::Uncached:: 
bus (volatile void* offset) const volatile
{
  return (bus (const_cast<void*>(offset)));
}

AikaPi::PWM:: 
PWM (void* phys_addr) : Peripheral (phys_addr)
{
  init ();
}

AikaPi::PWM:: 
~PWM ()
{
 // stop (0);
 // stop (1);
}

void AikaPi::PWM:: 
init ()
{
  reset ();

  cm.pwm.frequency (AP::CLKMAN::FREQUENCY);
  
  algo (0, AP::PWM::ALGO::MARKSPACE);
  algo (1, AP::PWM::ALGO::MARKSPACE);
}

void AikaPi::PWM::
start (bool channel)
{
  reg_wbits (AP::PWM::CTL, 1, channel ? 8 : 0);
}

void AikaPi::PWM:: 
stop(bool channel)
{
  reg_wbits (AP::PWM::CTL, 0, channel ? 8 : 0);
}

void AikaPi::PWM::
algo (bool          channel, 
      AP::PWM::ALGO algo)
{
  reg_wbits (AP::PWM::CTL, static_cast<bool>(algo), channel ? 15 : 7);
}

void AikaPi::PWM::
use_fifo (bool channel, 
          bool value)
{
  reg_wbits (AP::PWM::CTL, value, channel ? 13 : 5);
}

void AikaPi::PWM:: 
reset ()
{
  reg (AP::PWM::CTL,  0x40);
  std::this_thread::sleep_for (std::chrono::microseconds (10));

  reg (AP::PWM::STA,  0x1FE);
  std::this_thread::sleep_for (std::chrono::microseconds (10));

  reg (AP::PWM::RNG1, 0x20);
  std::this_thread::sleep_for (std::chrono::microseconds (10));

  reg (AP::PWM::DAT1, 0x0);
  std::this_thread::sleep_for (std::chrono::microseconds (10));

  reg (AP::PWM::DAT2, 0x0);
  std::this_thread::sleep_for (std::chrono::microseconds (10));
}

double AikaPi::PWM::
frequency (bool   channel,
           double value)
{
  uint32_t range = std::round (cm.pwm.frequency () / value);

  reg (channel ? AP::PWM::RNG2 : AP::PWM::RNG1, range);

  duty_cycle (channel, m_duty_cycle);

  return (cm.pwm.frequency () / range);
}

void AikaPi::PWM:: 
duty_cycle (bool   channel,
            double value)
{
  m_duty_cycle = std::fmod (value, 101.0);

  double dc_percentage  = m_duty_cycle / 100.0;
  double dc_data        = std::round (range (channel) * dc_percentage); 

  if (is_using_fifo (channel))
  {
    reg (AP::PWM::FIF1, dc_data);
  }
  else 
  {
    reg (channel ? AP::PWM::DAT2 : AP::PWM::DAT1, dc_data);
  }
}

uint32_t AikaPi::PWM:: 
range (bool channel)
{
  return (*reg (channel ? AP::PWM::RNG2 : AP::PWM::RNG1));
}

bool AikaPi::PWM:: 
is_using_fifo (bool channel)
{
  return (reg_rbits (AP::PWM::CTL, channel ? 13 : 5));
}

void AikaPi::PWM:: 
repeat_last_data (bool channel,
                  bool value)
{
  reg_wbits (AP::PWM::CTL, value, channel ? 10 : 2);
}

void AikaPi::PWM::
clear_fifo ()
{
  reg_wbits (AP::PWM::CTL, 1, 6);

  // For some reason, setting the clear fifo bit DOES NOT
  // completely clear out the FIFO! The FIFO empty flag
  // isn't even zero after the FIFO clear operation.

  //while (!is_fifo_empty ());

  std::this_thread::sleep_for (std::chrono::microseconds (10));
}

bool AikaPi::PWM:: 
is_fifo_empty () const
{
  return (reg_rbits (AP::PWM::STA, 1));
}

bool AikaPi::PWM:: 
is_fifo_full () const
{
  return (reg_rbits (AP::PWM::STA, 0));
}

AikaPi::GPIO::
GPIO (void* phys_addr) : Peripheral (phys_addr)
{

}

AikaPi::GPIO::
~GPIO ()
{

}

void AikaPi::GPIO::
set (unsigned       pin, 
     AP::GPIO::FUNC func_val, 
     AP::GPIO::PULL pull_val, 
     bool           value)
{
  func  (pin, func_val);
  pull  (pin, pull_val);
  write (pin, value);
}

void AikaPi::GPIO::
func  (unsigned       pin, 
       AP::GPIO::FUNC func_val)
{
  volatile uint32_t* gpf_reg = reg (GPFSEL0) + (pin / 10);
  unsigned shift = (pin % 10) * 3;

  wbits (gpf_reg, static_cast<uint32_t>(func_val), shift, 0x7);
}

void AikaPi::GPIO::
pull  (unsigned       pin, 
       AP::GPIO::PULL pull_val)
{
  // 0. Select either GPPUDCLK0 or GPPUDCLK1 depending on pin
  volatile uint32_t* gppud_reg = reg (AP::GPIO::GPPUDCLK0) + (pin / 32);
  
  // 1. Write to GPPUD to set the required control signal
  reg (AP::GPIO::GPPUD, static_cast<uint32_t>(pull_val));

  // 2. Wait 150 cycles - this provides the required set-up time
  //    for the control signal
  std::this_thread::sleep_for (std::chrono::microseconds (10));

  // 3. Write to GPPUDCLK0/1 to clock the control signal into the
  //    GPIO pads you wish to modify
  wreg (gppud_reg, pin << (pin % 32));

  // 4. Wait 150 cycles - this provides the required hold time
  //    for the control signal
  std::this_thread::sleep_for (std::chrono::microseconds (10));

  // 5. Write to GPPUD to remove the control signal
  reg (AP::GPIO::GPPUD, 0);

  // 6. Write to GPPUDCLK0/1 to remove the clock
  wreg (gppud_reg, 0);
}

void AikaPi::GPIO::
write (unsigned pin,  
       bool     value)
{
  volatile uint32_t* write_reg = reg
    (value ? AP::GPIO::GPSET0 : AP::GPIO::GPCLR0) + (pin / 32);

  wreg (write_reg, 1 << (pin % 32));
}

bool AikaPi::GPIO::
read (unsigned pin)
{
  volatile uint32_t* read_reg = reg (AP::GPIO::GPLEV0) + (pin / 32);

  return (rbits (read_reg, (pin % 32)));
}

AikaPi::AUX::SPI:: 
SPI (bool channel, AUX* aux)
  : channel (channel), aux (aux)
{
  init ();
}

/**
 * @brief Return a uint32_t offset of the specific 
 *        AUX SPI channel's SPI register
 */
uint32_t AikaPi::AUX::SPI:: 
off (uint32_t offset) const
{
  uint32_t chan_offset = offset + 
    (channel ? AP::AUX::SPI::SPI1_BASE : AP::AUX::SPI::SPI0_BASE);

  return (chan_offset);
}

bool AikaPi::AUX::SPI:: 
is_rx_fifo_empty () const
{
  return (aux->reg_rbits (off (AP::AUX::SPI::STAT_REG), 2));
}

void AikaPi::AUX::SPI:: 
init ()
{
  gpio.set (AP::RPI::PIN::AUX::SPI1::SCLK, AP::GPIO::FUNC::ALT4, AP::GPIO::PULL::OFF);
  gpio.set (AP::RPI::PIN::AUX::SPI1::MOSI, AP::GPIO::FUNC::ALT4, AP::GPIO::PULL::OFF);
  gpio.set (AP::RPI::PIN::AUX::SPI1::MISO, AP::GPIO::FUNC::ALT4, AP::GPIO::PULL::DOWN);
  gpio.set (AP::RPI::PIN::AUX::SPI1::CE0,  AP::GPIO::FUNC::ALT4, AP::GPIO::PULL::OFF);
  gpio.set (AP::RPI::PIN::AUX::SPI1::CE1,  AP::GPIO::FUNC::ALT4, AP::GPIO::PULL::OFF);
  gpio.set (AP::RPI::PIN::AUX::SPI1::CE2,  AP::GPIO::FUNC::ALT4, AP::GPIO::PULL::OFF);

  enable                  ();
  shift_length            (8);
  shift_out_ms_bit_first  (true);
  mode                    (0);
  chip_selects            (3);
  frequency               (100'000);
  clear_fifos             ();
}

void AikaPi::AUX::SPI:: 
enable ()
{
  aux->reg_wbits (off (AP::AUX::SPI::CNTL0_REG), 1, 11);
}

void AikaPi::AUX::SPI:: 
disable ()
{
  aux->reg_wbits (off (AP::AUX::SPI::CNTL0_REG), 0, 11);
}

void AikaPi::AUX::SPI:: 
shift_length (uint8_t value)
{
  aux->reg_wbits (off (AP::AUX::SPI::CNTL0_REG), value, 0, 6);
}

void AikaPi::AUX::SPI:: 
shift_out_ms_bit_first (bool value)
{
  aux->reg_wbits (off (AP::AUX::SPI::CNTL0_REG), value, 6);
}

void AikaPi::AUX::SPI:: 
mode (unsigned mode)
{
  switch (mode)
  {
    // https://www.analog.com/en/analog-dialogue/articles/introduction-to-spi-interface.html

    case 0:
    {
      clock_polarity  (0);
      in_rising       (1);
      out_rising      (0);

      break;
    }

    case 1:
    {
      clock_polarity  (0);
      in_rising       (0);
      out_rising      (1);

      break;
    }
    
    case 2:
    {
      clock_polarity  (1);
      in_rising       (1);
      out_rising      (0);

      break;
    }

    case 3:
    { 
      clock_polarity  (1);
      in_rising       (0);
      out_rising      (1);

      break;
    }

    default:
    {
      throw (std::runtime_error ("Invalid SPI mode."));
    }
  }
}

void AikaPi::AUX::SPI:: 
clock_polarity (bool value)
{
  aux->reg_wbits (off (AP::AUX::SPI::CNTL0_REG), value, 7);
}

void AikaPi::AUX::SPI:: 
in_rising (bool value)
{
  aux->reg_wbits (off (AP::AUX::SPI::CNTL0_REG), value, 10);
}

void AikaPi::AUX::SPI:: 
out_rising (bool value)
{
  aux->reg_wbits (off (AP::AUX::SPI::CNTL0_REG), value, 8);
}

void AikaPi::AUX::SPI:: 
chip_selects (uint8_t value)
{
  aux->reg_wbits (off (AP::AUX::SPI::CNTL0_REG), value, 17, 3);
}

void AikaPi::AUX::SPI:: 
frequency (double value)
{
  uint16_t divider = static_cast<uint16_t>((static_cast<double>(AP::RPI::CLOCK_HZ) /
    (2.0 * value)) - 1) & 0x0fff;

  aux->reg_wbits (off (AP::AUX::SPI::CNTL0_REG), divider, 20, 12);
}

void AikaPi::AUX::SPI:: 
clear_fifos ()
{
  aux->reg_wbits (off (AP::AUX::SPI::CNTL0_REG), 1, 9);

  // small delay?
  std::this_thread::sleep_for (std::chrono::duration <double, std::micro> (100));

  aux->reg_wbits (off (AP::AUX::SPI::CNTL0_REG), 0, 9);
}

void AikaPi::AUX::SPI:: 
xfer (char*     rxd, 
      char*     txd, 
      unsigned  length)
{
  while (length--)
  {
    // assert CE pin ?

    uint32_t txdata = *(txd++);

    // write
    aux->reg (off (AP::AUX::SPI::IO_REG), txdata);

    // wait until 1 byte is received
    while (is_rx_fifo_empty ());

    *(rxd++) = *(aux->reg (off (AP::AUX::SPI::IO_REG)));

    // deassert CE pin ?
  }
}

void AikaPi::AUX::SPI:: 
read (char*    rxd, 
      unsigned length)
{
  char txd[length] = {0};

  xfer (rxd, txd, length);
}

void AikaPi::AUX::SPI:: 
write (char*    txd, 
       unsigned length)
{
  char rxd[length] = {0};

  xfer (rxd, txd, length);
} 

AikaPi::AUX::
AUX (void* phys_addr) : Peripheral (phys_addr),
  m_spi {SPI (0, this), SPI (1, this)}
{

}

void AikaPi::AUX::
init ()
{
  master_enable_spi (0);
  master_enable_spi (1);

}

AikaPi::AUX::SPI& AikaPi::AUX:: 
spi (bool channel)
{
  return (m_spi[channel]);
}

void AikaPi::AUX::
master_enable_spi (bool channel)
{
  reg_wbits (AP::AUX::ENABLES, 1, channel ? 2 : 1);
}

void AikaPi::AUX::
master_disable_spi (bool channel)
{
  reg_wbits (AP::AUX::ENABLES, 0, channel ? 2 : 1);
}

uint32_t AikaPi::ClockManager::ClkManPeriph:: 
off (uint32_t offset) const
{
  switch (offset)
  {
    case (AP::CLKMAN::CTL):
    {
      if (m_type == AP::CLKMAN::TYPE::PCM)
      {
        return (AP::CLKMAN::PCM_CTL);
      }
      else if (m_type == AP::CLKMAN::TYPE::PWM)
      {
        return (AP::CLKMAN::PWM_CTL);
      }

      break;
    }

    case (AP::CLKMAN::DIV):
    {
      if (m_type == AP::CLKMAN::TYPE::PCM)
      {
        return (AP::CLKMAN::PCM_DIV);
      }
      else if (m_type == AP::CLKMAN::TYPE::PWM)
      {
        return (AP::CLKMAN::PWM_DIV);
      }
    
      break;
    }

    default:
    {
      throw (std::runtime_error ("Invalid AP::CLKMAN offset value."));

      break;
    }
  }

  return 0;
}

AikaPi::ClockManager::ClkManPeriph:: 
ClkManPeriph (void* phys_addr) :Peripheral (phys_addr)
{

}

void AikaPi::ClockManager::ClkManPeriph::
stop ()
{
  reg (off (AP::CLKMAN::CTL), (AP::CLKMAN::PASSWD) | (1 << 5));

  while (is_running ());
}

void AikaPi::ClockManager::ClkManPeriph::
start ()
{
  if (!is_running ())
  {
    uint32_t  data = *(reg (off (AP::CLKMAN::CTL))) | AP::CLKMAN::PASSWD;
              data = wbits (data, 1, 4); // write 1 for ENABLE bit
              data = wbits (data, 0, 5); // write 0 for KILL bit

    reg (off (AP::CLKMAN::CTL), data);
  }

  while (!is_running ());
}


bool AikaPi::ClockManager::ClkManPeriph:: 
is_running ()
{
  return (reg_rbits (off (AP::CLKMAN::CTL), 7));
}

void AikaPi::ClockManager::ClkManPeriph::
source (AP::CLKMAN::SOURCE source)
{
  if (!is_running ())
  {
    uint32_t  data = *(reg (off (AP::CLKMAN::CTL))) | AP::CLKMAN::PASSWD;
              data = wbits (data, static_cast<uint32_t>(source), 0, 4);

    reg (off (AP::CLKMAN::CTL), data);

    std::this_thread::sleep_for (std::chrono::microseconds (10));
  }
  else 
  {
    throw (std::runtime_error ("Attempt to change Clock Manager source while it is running."));
  }
}        

void AikaPi::ClockManager::ClkManPeriph:: 
mash (AP::CLKMAN::MASH mash)
{
  if (!is_running ())
  {
    uint32_t  data = *(reg (off (AP::CLKMAN::CTL))) | AP::CLKMAN::PASSWD;
              data = wbits (data, static_cast<uint32_t>(mash), 9, 2);       

    reg (off (AP::CLKMAN::CTL), data);

    std::this_thread::sleep_for (std::chrono::microseconds (10));
  }
  else 
  {
    throw (std::runtime_error ("Attempt to change Clock Manager MASH while it is running."));
  }
}

void AikaPi::ClockManager::ClkManPeriph:: 
divisor (uint32_t integral, uint32_t fractional)
{
  if (!is_running ())
  {
    reg (off (AP::CLKMAN::DIV), (AP::CLKMAN::PASSWD) | (integral << 12) |
      (fractional));
    
    std::this_thread::sleep_for (std::chrono::microseconds (10));
  }
  else 
  {
    throw (std::runtime_error ("Attempt to change Clock Manager divisor while it is running."));
  }
}

void AikaPi::ClockManager::ClkManPeriph:: 
frequency (double             value,
           AP::CLKMAN::SOURCE source_val,
           AP::CLKMAN::MASH   mash_val)
{
  // 1. Stop the clock generator
  stop ();

  // 2. Calculate the divisor
  double divi = AP::CLKMAN::SOURCE_FREQUENCY [static_cast<unsigned>(source_val)] / value;

  // 3. Extract divisor's integral and fractional parts
  double d_integral;
  double d_fractional;

  d_fractional = std::modf (divi, &d_integral);
  d_fractional = d_fractional * std::pow (10, 12);

  uint32_t u_integral   = static_cast<uint32_t>(d_integral) & 0xFFF;
  uint32_t u_fractional = static_cast<uint32_t>(d_fractional) & 0xFFF; 

  // 4. Set the value of the divisor register  
  divisor (u_integral, u_fractional);
  
  // 5. Set clock source
  source (source_val);

  // 6. Set clock mash
  mash (mash_val);

  // 7. Start the clock generator
  start ();
}

double AikaPi::ClockManager::ClkManPeriph:: 
frequency ()
{
  uint32_t  reg_val       = *(reg (off (AP::CLKMAN::DIV)));
  uint32_t  u_integral    = (reg_val >> 12) & 0xFFF;
  uint32_t  u_fractional  = reg_val & 0xFFF;
  double    d_integral    = static_cast<double>(u_integral);
  double    d_fractional  = static_cast<double>(u_fractional) / 12.0;
  double    frequency     = AP::CLKMAN::SOURCE_FREQUENCY[static_cast<unsigned>
                              (source ())] / (d_integral + d_fractional);

  return (frequency);
}

AP::CLKMAN::SOURCE AikaPi::ClockManager::ClkManPeriph::
source ()
{
  unsigned source = reg_rbits (off (AP::CLKMAN::CTL), 0, 0xF);

  switch (source)
  {
    case 1:
      return (AP::CLKMAN::SOURCE::OSCILLATOR);
      break;
    case 2:
      return (AP::CLKMAN::SOURCE::TESTDEBUG0);
      break;    
    case 3:
      return (AP::CLKMAN::SOURCE::TESTDEBUG1);
      break;   
    case 4:
      return (AP::CLKMAN::SOURCE::PLLA);
      break;
    case 5:
      return (AP::CLKMAN::SOURCE::PLLC);
      break;
    case 6:
      return (AP::CLKMAN::SOURCE::PLLD);
      break;
    case 7:
      return (AP::CLKMAN::SOURCE::HDMI);
      break;
    break;
      // return (AP::CLKMAN::SOURCE::GND);
      break;
  }

  return (AP::CLKMAN::SOURCE::GND);
}

AikaPi::ClockManager::PCM::
PCM (void* phys_addr, AP::CLKMAN::TYPE type) : ClkManPeriph (phys_addr)
{
  m_type = type;
}

AikaPi::ClockManager::PWM::
PWM (void* phys_addr, AP::CLKMAN::TYPE type) : ClkManPeriph (phys_addr)
{
  m_type = type;
}

AikaPi::ClockManager::
ClockManager (void* phys_addr) 
  : pcm (phys_addr, AP::CLKMAN::TYPE::PCM), 
    pwm (phys_addr, AP::CLKMAN::TYPE::PWM)
{

}

AikaPi::SystemTimer:: 
SystemTimer (void* phys_addr) : Peripheral (phys_addr)
{

}

AikaPi::SystemTimer::
~SystemTimer ()
{

}

uint32_t AikaPi::SystemTimer:: 
low () const 
{
  return (*(reg (AP::SYSTIMER::CLO)));
}

AikaPi::SPI_BB:: 
SPI_BB (unsigned CS, unsigned MISO, unsigned MOSI, unsigned SCLK, double baud, AP::SPI::MODE i_mode)
  : m_CS (CS), m_MISO (MISO), m_MOSI (MOSI), m_SCLK (SCLK), m_baud (baud), m_mode (i_mode)
{
  if (m_baud > AP::SPI_BB::MAX_BAUD)
  {
    throw (std::runtime_error ("Greater than maximum baud rate for SPI BB set. Max 250kHz."));
  }

  init ();
}

AikaPi::SPI_BB::
~SPI_BB ()
{
  // Reset GPIO pins to input
  gpio.set (m_CS,   AP::GPIO::FUNC::INPUT, AP::GPIO::PULL::OFF);
  gpio.set (m_MISO, AP::GPIO::FUNC::INPUT, AP::GPIO::PULL::OFF);
  gpio.set (m_MOSI, AP::GPIO::FUNC::INPUT, AP::GPIO::PULL::OFF);
  gpio.set (m_SCLK, AP::GPIO::FUNC::INPUT, AP::GPIO::PULL::OFF);
}

void AikaPi::SPI_BB:: 
init ()
{
  // TO-DO: do checking and validation of the provided RPI pins

  // 1. Set GPIO pins
  gpio.set (m_CS,   AP::GPIO::FUNC::OUTPUT, AP::GPIO::PULL::OFF, 0);
  gpio.set (m_SCLK, AP::GPIO::FUNC::OUTPUT, AP::GPIO::PULL::OFF, 0);
  
  if (m_MISO >= 0)
    gpio.set (m_MISO, AP::GPIO::FUNC::INPUT,  AP::GPIO::PULL::OFF, 0);
  
  if (m_MOSI >= 0)
    gpio.set (m_MOSI, AP::GPIO::FUNC::OUTPUT, AP::GPIO::PULL::OFF, 0);
        

  // 2. Set delay
  delay (m_baud);

  // 3. Preset chip select pin polarity
  gpio.write (m_CS, !m_CS_polarity);
}

void AikaPi::SPI_BB::
delay ()
{
  std::this_thread::sleep_for (std::chrono::duration <double, std::micro> (m_delay));
}

void AikaPi::SPI_BB:: 
delay (double baud)
{
  m_delay = (500'000.0 / baud) - 1;
}

void AikaPi::SPI_BB::
set_CS ()
{
  gpio.write (m_CS, m_CS_polarity);
}

void AikaPi::SPI_BB:: 
clear_CS ()
{
  gpio.write (m_CS, !m_CS_polarity);
}

void AikaPi::SPI_BB:: 
set_SCLK ()
{
  gpio.write (m_SCLK, !cpol ());
}

void AikaPi::SPI_BB:: 
clear_SCLK ()
{
  gpio.write (m_SCLK, cpol ());
}

void AikaPi::SPI_BB::
start ()
{
  clear_SCLK ();

  delay ();

  set_CS ();

  delay ();
}

void AikaPi::SPI_BB:: 
stop ()
{
  delay ();

  clear_CS ();

  delay ();

  clear_SCLK ();
}

uint8_t AikaPi::SPI_BB:: 
xfer_byte (char txd)
{
  uint8_t rxd;

  if (cpha ())
  {
    // CPHA = 1
    // Write on set clock
    // Read on clear clock

    for (int bit = 0; bit < 8; bit++)
    {
      set_SCLK ();

      if (m_shift_out_ms_bit_first)
      {
        gpio.write (m_MOSI, txd & 0x80);
        txd <<= 1;
      }
      else 
      {
        gpio.write (m_MOSI, txd & 0x01);
        txd >>= 1;
      }

      delay ();

      clear_SCLK ();

      if (m_receive_ms_bit_first)
      {
        rxd = (rxd << 1) | gpio.read (m_MISO);
      }
      else 
      {
        rxd = (rxd >> 1) | ((gpio.read (m_MISO)) << 7);
      }

      delay ();
    }
  }
  else 
  {
    // CPHA = 0;
    // Read on set clock
    // Write on clear clock

    for (int bit = 0; bit < 8; bit++)
    {
      if (m_shift_out_ms_bit_first)
      {
        gpio.write (m_MOSI, txd & 0x80);
        txd <<= 1;
      }
      else 
      {
        gpio.write (m_MOSI, txd & 0x01);
        txd >>= 1;
      }

      delay ();

      set_SCLK ();

      if (m_receive_ms_bit_first)
      {
        rxd = (rxd << 1) | gpio.read (m_MISO);
      }
      else 
      {
        rxd = (rxd >> 1) | ((gpio.read (m_MISO)) << 7);
      }

      delay ();

      clear_SCLK ();
    }
  }

  return (rxd);
}

bool AikaPi::SPI_BB:: 
cpol ()
{
  return (((static_cast<uint8_t>(m_mode)) >> 1) & 0x1);
}

bool AikaPi::SPI_BB:: 
cpha ()
{
  return ((static_cast<uint8_t>(m_mode)) & 0x1);
}

void AikaPi::SPI_BB::
mode (AP::SPI::MODE value)
{
  m_mode = value;
}

void AikaPi::SPI_BB:: 
baud (double value)
{
  m_baud = value;

  delay (m_baud);
}

void AikaPi::SPI_BB:: 
shift_out_msb_first (bool value)
{
  m_shift_out_ms_bit_first = value;
}

void AikaPi::SPI_BB:: 
receive_msb_first (bool value)
{
  m_receive_ms_bit_first = value;
}

/**
 * @brief Set chip select pin asserted logic level 
 */
void AikaPi::SPI_BB::
cs_polarity (bool value)
{
  m_CS_polarity = value;
}

void AikaPi::SPI_BB::
xfer (char*    rxd,
      char*    txd,
      unsigned length)
{
  start ();

  for (int pos = 0; pos < length; pos++)
  {
    rxd[pos] = xfer_byte (txd[pos]);
  }

  stop ();
}

void AikaPi::SPI_BB::
write (char*    txd,
       unsigned length)
{
  char rxd[length];

  xfer (rxd, txd, length);
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
  //dma_reset(LABC::DMA::CHANPWM_PACING);
  //dma_reset(LABC::DMA::CHANOSC_RX);
  //dma_reset(LABC::DMA::CHANOSC_TX);
  
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
  spi_clear_fifo ();

  spi_set_clock_rate (frequency);
  
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
          AP::PWM::ALGO _AP_PWM_ALGO)
{
  // Utility::write_reg_virt (Utility::reg (m_regs_pwm, PWM_CTL), _AP_PWM_ALGO,
  //   1, (channel == 0 ? 7 : 15));
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

// --- Utility ---
int AikaPi:: 
sleep_nano (int nano)
{
  std::this_thread::sleep_for (std::chrono::duration<double, std::nano> 
    (nano));

  return 0;
}



// EOF