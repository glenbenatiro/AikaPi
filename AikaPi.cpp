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

AikaPi:: 
AikaPi ()
{

}

AikaPi:: 
~AikaPi ()
{
  
}

// Mailbox
uint32_t AikaPi::Mailbox::
page_roundup (uint32_t addr)
{
  return ((addr % AP::RPI::PAGE_SIZE == 0) ? (addr) : ((addr + AP::RPI::PAGE_SIZE) & ~(AP::RPI::PAGE_SIZE - 1)));
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
      AP::RPI::PAGE_SIZE,
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
  return ((addr % AP::RPI::PAGE_SIZE == 0) ? (addr) : ((addr + AP::RPI::PAGE_SIZE) & ~(AP::RPI::PAGE_SIZE - 1)));
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
  m_size  = page_roundup (AP::RPI::PAGE_SIZE);
  m_bus   = reinterpret_cast<uint8_t*>(phys_addr) - 
            reinterpret_cast<uint8_t*>(AP::RPI::PHYS_REG_BASE) + 
            reinterpret_cast<uint8_t*>(AP::RPI::BUS_REG_BASE);
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
  reg (AP::SPI::CS, 2 << 4);
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
  return (*(reg (dma_chan, AP::DMA::DEST_AD)));
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
  reg (dma_chan, AP::DMA::CS, 1 << 31);

  std::this_thread::sleep_for (std::chrono::microseconds (10));
}

bool AikaPi::DMA::
is_running (unsigned dma_chan) const
{
  return (reg_rbits (dma_chan, AP::DMA::CS, 4));
}

void AikaPi::DMA::
pause (unsigned dma_chan)
{
  reg_wbits (dma_chan, AP::DMA::CS, 0, 0);
}

void AikaPi::DMA::
next_cb (unsigned dma_chan, 
         uint32_t next_cb_bus_addr)
{
  reg (dma_chan, AP::DMA::NEXTCONBK, next_cb_bus_addr);
}

void AikaPi::DMA::
abort (unsigned dma_chan)
{
  reg_wbits (dma_chan, AP::DMA::CS, 1, 30);
}

void AikaPi::DMA::
run (unsigned dma_chan)
{
  reg_wbits (dma_chan, AP::DMA::CS, 1, 0);
}

void AikaPi::DMA::
stop (unsigned dma_chan)
{
  reset (dma_chan);
}

uint32_t AikaPi::DMA::
conblk_ad (unsigned dma_chan) const
{
  return (*(reg (dma_chan, AP::DMA::CONBLK_AD)));
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
  volatile uint32_t* gpf_reg = reg (AP::GPIO::GPFSEL0) + (pin / 10);
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
  : m_channel (channel), m_aux (aux)
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
    (m_channel ? AP::AUX::SPI::SPI1_BASE : AP::AUX::SPI::SPI0_BASE);

  return (chan_offset);
}

bool AikaPi::AUX::SPI:: 
is_rx_fifo_empty () const
{
  return (m_aux->reg_rbits (off (AP::AUX::SPI::STAT_REG), 2));
}

void AikaPi::AUX::SPI:: 
init ()
{
  gpio.set (AP::RPI::PIN::AUX::SPI1::SCLK, AP::GPIO::FUNC::ALT4,  AP::GPIO::PULL::OFF);
  gpio.set (AP::RPI::PIN::AUX::SPI1::MOSI, AP::GPIO::FUNC::ALT4,  AP::GPIO::PULL::OFF);
  gpio.set (AP::RPI::PIN::AUX::SPI1::MISO, AP::GPIO::FUNC::ALT4,  AP::GPIO::PULL::DOWN);

  gpio.set (AP::RPI::PIN::AUX::SPI1::CE0,  AP::GPIO::FUNC::OUTPUT, AP::GPIO::PULL::OFF, !m_cs_polarity);
  gpio.set (AP::RPI::PIN::AUX::SPI1::CE1,  AP::GPIO::FUNC::OUTPUT, AP::GPIO::PULL::OFF, !m_cs_polarity);
  gpio.set (AP::RPI::PIN::AUX::SPI1::CE2,  AP::GPIO::FUNC::OUTPUT, AP::GPIO::PULL::OFF, !m_cs_polarity);

  // load default settings
  enable                  ();
  clear_fifos             ();
  mode                    (AP::SPI::MODE::_0);  
  shift_out_ms_bit_first  (true);
  shift_in_ms_bit_first   (true);
  chip_selects            (3);
  frequency               (1'000'000);
  clear_fifos             ();
  shift_length            (8);
}

unsigned AikaPi::AUX::SPI::
cs_pin (unsigned cs)
{
  switch (cs)
  {
    case 0:
      return (AP::RPI::PIN::AUX::SPI1::CE0);
      break; 
    
    case 1:
      return (AP::RPI::PIN::AUX::SPI1::CE1);
      break;

    default:
      return (AP::RPI::PIN::AUX::SPI1::CE2);
      break;
  }
}

bool AikaPi::AUX::SPI::
busy ()
{
  return (m_aux->reg_rbits (off (AP::AUX::SPI::STAT_REG), 6));
}

void AikaPi::AUX::SPI:: 
enable ()
{
  m_aux->reg_wbits (off (AP::AUX::SPI::CNTL0_REG), 1, 11);
}

void AikaPi::AUX::SPI:: 
disable ()
{
  m_aux->reg_wbits (off (AP::AUX::SPI::CNTL0_REG), 0, 11);
}

void AikaPi::AUX::SPI:: 
shift_length (uint8_t bits)
{
  m_aux->reg_wbits (off (AP::AUX::SPI::CNTL0_REG), bits, 0, 6);
}

void AikaPi::AUX::SPI:: 
shift_out_ms_bit_first (bool value)
{
  m_aux->reg_wbits (off (AP::AUX::SPI::CNTL0_REG), value, 6);
}

void AikaPi::AUX::SPI::
shift_in_ms_bit_first (bool value)
{
  m_aux->reg_wbits (off (AP::AUX::SPI::CNTL1_REG), value, 1);
}

void AikaPi::AUX::SPI:: 
mode (AP::SPI::MODE mode)
{
  switch (mode)
  {
    // https://www.analog.com/en/analog-dialogue/articles/introduction-to-spi-interface.html
    // 0: CS polarity low, data sampled rising, shifted out falling
    // 1: CS polarity low, data sampled falling, shifted out rising
    // 2: CS polarity high, data sampled rising, shifted out falling
    // 3: CS polarity high, data sampled faling, shifted out rising

    case AP::SPI::MODE::_0:
    {
      clock_polarity  (0);
      in_rising       (1);
      out_rising      (0);

      break;
    }

    case AP::SPI::MODE::_1:
    {
      clock_polarity  (0);
      in_rising       (0);
      out_rising      (1);

      break;
    }
    
    case AP::SPI::MODE::_2:
    {
      clock_polarity  (1);
      in_rising       (1);
      out_rising      (0);

      break;
    }

    case AP::SPI::MODE::_3:
    { 
      clock_polarity  (1);
      in_rising       (0);
      out_rising      (1);

      break;
    }
  }
}

void AikaPi::AUX::SPI:: 
clock_polarity (bool value)
{
  m_aux->reg_wbits (off (AP::AUX::SPI::CNTL0_REG), value, 7);
}

void AikaPi::AUX::SPI:: 
in_rising (bool value)
{
  m_aux->reg_wbits (off (AP::AUX::SPI::CNTL0_REG), value, 10);
}

void AikaPi::AUX::SPI:: 
out_rising (bool value)
{
  m_aux->reg_wbits (off (AP::AUX::SPI::CNTL0_REG), value, 8);
}

void AikaPi::AUX::SPI:: 
chip_selects (uint8_t value)
{
  m_aux->reg_wbits (off (AP::AUX::SPI::CNTL0_REG), value, 17, 3);
}

bool AikaPi::AUX::SPI::
rx_empty ()
{
  return (m_aux->reg_rbits (off (AP::AUX::SPI::STAT_REG), 7));
}

bool AikaPi::AUX::SPI::
tx_full ()
{
  return (m_aux->reg_rbits (off (AP::AUX::SPI::STAT_REG), 10));
}

uint32_t AikaPi::AUX::SPI::
tx_bits (char*    txd,
         unsigned position)
{
  uint32_t bits = 0;

  if (shift_length () <= 8)
  {
    bits = *((( uint8_t*)(txd)) + position);
  }
  else if (shift_length () <= 16)
  {
    bits = *(((uint16_t*)(txd)) + position);
  }
  else 
  {
    bits = *(((uint32_t*)(txd)) + position);
  }

  if (shift_out_ms_bit_first ())
  {
    bits <<= (32 - shift_length ());
  }

  return (bits);
}

void AikaPi::AUX::SPI:: 
rx_bits (char*    rxd,
         unsigned position)
{
  uint32_t bits = *(m_aux->reg (off (AP::AUX::SPI::IO_REG)));

  if (!shift_in_ms_bit_first ())
  {
    bits >>= (32 - shift_length ());
  }

  if (shift_length () <= 8)
  {
    *((( uint8_t*)(rxd)) + position) = bits;
  }
  else if (shift_length () <= 16)
  {
    *(((uint16_t*)(rxd)) + position) = bits;
  }
  else 
  {
    *(((uint32_t*)(rxd)) + position) = bits;
  }
}

uint32_t AikaPi::AUX::SPI:: 
shift_length ()
{
  return (m_aux->reg_rbits (off (AP::AUX::SPI::CNTL0_REG), 0, 0x3F));
}

void AikaPi::AUX::SPI:: 
frequency (double value)
{
  uint16_t divider = ((AP::RPI::CLOCK_HZ / (value * 2))) - 1;
  divider &= 0x0FFF;

  m_aux->reg_wbits (off (AP::AUX::SPI::CNTL0_REG), divider, 20, 12);
}

void AikaPi::AUX::SPI:: 
clear_fifos ()
{
  m_aux->reg_wbits (off (AP::AUX::SPI::CNTL0_REG), 1, 9);

  std::this_thread::sleep_for (std::chrono::microseconds (10));

  m_aux->reg_wbits (off (AP::AUX::SPI::CNTL0_REG), 0, 9);
}

/**
 *  @param length number of bytes to read/write
 */
void AikaPi::AUX::SPI:: 
xfer (char*     rxd, 
      char*     txd, 
      unsigned  length)
{
  unsigned  txCount = 0,
            rxCount = 0;

  // correct word count size
  if (shift_length () > 8)
  {
    length /= 2;

    if (shift_length () > 16)
    {
      length /= 2;
    }
  }

  gpio.write (cs_pin (m_cs), m_cs_polarity);         

  while ((txCount < length) || (rxCount < length))
  {
    if (rxCount < length)
    {
      if (!rx_empty ())
      {
        rx_bits (rxd, rxCount++);
      }
    }

    if (txCount < length)
    {
      if (!tx_full ())
      {
        if (txCount != (length - 1))
        {
          m_aux->reg (off (AP::AUX::SPI::TXHOLD_REG), tx_bits (txd, txCount++));
        }
        else 
        {
          m_aux->reg (off (AP::AUX::SPI::IO_REG), tx_bits (txd, txCount++));
        }
      }
    }
  }

  while (busy ());

  gpio.write (cs_pin (m_cs), !m_cs_polarity);
}

void AikaPi::AUX::SPI::
xfer (char* rxd, char* txd, unsigned length, unsigned cs_pin)
{
  cs    (cs_pin);
  xfer  (rxd, txd, length);
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

bool AikaPi::AUX::SPI::
cpol ()
{
 return (m_aux->reg_rbits (off (AP::AUX::SPI::CNTL0_REG), 7));
}

void AikaPi::AUX::SPI::
cs_polarity (bool value)
{
  m_cs_polarity = value;
}

void AikaPi::AUX::SPI::
cs (unsigned cs_pin)
{
  if (cs_pin <= 2 && cs_pin >= 0)
  {
    m_cs = cs_pin;
  }
  else 
  {
    throw (std::runtime_error ("Invalid aux SPI CS pin!"));
  }
}

bool AikaPi::AUX::SPI::
shift_out_ms_bit_first ()
{
  return (m_aux->reg_rbits (off (AP::AUX::SPI::CNTL0_REG), 6));
}

bool AikaPi::AUX::SPI::
shift_in_ms_bit_first ()
{
  return (m_aux->reg_rbits (off (AP::AUX::SPI::CNTL1_REG), 1));
}

AikaPi::AUX::
AUX (void* phys_addr) : Peripheral (phys_addr),
  m_spi {SPI (0, this), SPI (1, this)}
{
  init ();
}

void AikaPi::AUX::
init ()
{
  master_enable_spi (0);
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

// AikaPi::Utility:: 
// Utility ()
// {

// }

// constexpr double AikaPi::Utility:: 
// normalize (double input_val,
//            double input_min, 
//            double input_max, 
//            double output_min, 
//            double output_max)
// {
//   return (((input_val - input_min) / (input_max - input_min)) * 
//     (output_max - output_min) + output_min);
// }

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
shift_out_ms_bit_first (bool value)
{
  m_shift_out_ms_bit_first = value;
}

void AikaPi::SPI_BB:: 
receive_ms_bit_first (bool value)
{
  m_receive_ms_bit_first = value;
}

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

// EOF