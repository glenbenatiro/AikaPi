#ifndef AIKAPI_H
#define AIKAPI_H

#include <cstdint>

// Link to the BCM2385 datasheet:
// // https://www.raspberrypi.org/app/uploads/2012/02/BCM2835-ARM-Peripherals.pdf

#define RPI_VERSION 0

// btw: The Pi-0 and Pi-3 do "overclock" the base clock up to 400 MHz on load. 
// And it may cause SPI problems if the clock isn't fixed 
// (i.e. at 250 MHz with core_freq=250 in /boot/config.txt).

// Location of peripheral registers in physical memory
// This can be seen on page 5 of ARM BCM2385 document
constexpr uint32_t PI_01_REG_BASE = 0x20000000; // Pi Zero or 1 
constexpr uint32_t PI_23_REG_BASE = 0x3F000000; // Pi 2 or 3
constexpr uint32_t PI_4_REG_BASE  = 0xFE000000; // Pi 4

#if   RPI_VERSION == 0
  constexpr uint32_t PHYS_REG_BASE  = PI_01_REG_BASE;
  constexpr uint32_t CLOCK_HZ	      = 250000000;
//constexpr uint32_t SPI_CLOCK_HZ   = 400000000; // !! https://github.com/raspberrypi/linux/issues/2094 !!
  constexpr uint32_t SPI_CLOCK_HZ   = 250000000;  // !! https://github.com/raspberrypi/linux/issues/2094 !!
#elif RPI_VERSION == 1
  constexpr uint32_t PHYS_REG_BASE  = PI_01_REG_BASE;
  constexpr uint32_t CLOCK_HZ       = 250000000;
  constexpr uint32_t SPI_CLOCK_HZ   = 250000000;
#elif RPI_VERSION == 2 || RPI_VERSION == 3
  constexpr uint32_t PHYS_REG_BASE  = PI_23_REG_BASE;
  constexpr uint32_t CLOCK_HZ       = 250000000;
  constexpr uint32_t SPI_CLOCK_HZ   = 250000000;
#elif RPI_VERSION == 4
  constexpr uint32_t PHYS_REG_BASE  = PI_4_REG_BASE;
  constexpr uint32_t CLOCK_HZ	      = 375000000;
  constexpr uint32_t SPI_CLOCK_HZ   = 200000000;
#endif

// Location of peripheral registers in bus memory
// This can be seen on page 5 of ARM BCM2385 document
constexpr uint32_t BUS_REG_BASE = 0x7E000000;

// --- DMA ---
struct AP_DMA_CB
{
  uint32_t  ti,       // Transfer information
            srce_ad,  // Source address
            dest_ad,  // Destination address
            tfr_len,  // Transfer length, in bytes
            stride,   // Transfer stride
            next_cb,  // Next control block
            debug,    // Debug register, zero in control block
            unused;
} __attribute__ (( aligned(32) ));

constexpr uint32_t DMA_BASE           = (PHYS_REG_BASE + 0x007000);
constexpr uint32_t DMA_TI_DREQ_PWM    = 5;
constexpr uint32_t DMA_TI_DREQ_SPI_RX = 7;
constexpr uint32_t DMA_TI_DREQ_SPI_TX = 6;
constexpr uint32_t DMA_TI_SRC_DREQ    = 1 << 10;
constexpr uint32_t DMA_TI_SRC_INC     = 1 << 8;
constexpr uint32_t DMA_TI_DEST_DREQ   = 1 << 6;
constexpr uint32_t DMA_TI_DEST_INC    = 1 << 4;
constexpr uint32_t DMA_TI_WAIT_RESP   = 1 << 3;

// DMA control block macros
#define REG(r, a)       REG_BUS_ADDR(r, a)
#define MEM(m, a)       MEM_BUS_ADDR(m, a)
#define CBS(n)          MEM_BUS_ADDR(mp, &dp->cbs[(n)])

// DMA channels and data requests
#define DMA_PWM_DREQ    5
#define DMA_SPI_TX_DREQ 6
#define DMA_SPI_RX_DREQ 7




// DMA register addresses offset by 0x100 * chan_num
constexpr uint32_t DMA_CONBLK_AD = 0x04;
constexpr uint32_t DMA_CS        = 0x00;
constexpr uint32_t DMA_TI        = 0x08;
constexpr uint32_t DMA_SRCE_AD   = 0x0c;
constexpr uint32_t DMA_DEST_AD   = 0x10;
constexpr uint32_t DMA_TXFR_LEN  = 0x14;
constexpr uint32_t DMA_STRIDE    = 0x18;
constexpr uint32_t DMA_NEXTCONBK = 0x1c;
constexpr uint32_t DMA_DEBUG     = 0x20;
#define DMA_REG(ch, r)  ((r) == DMA_ENABLE ? DMA_ENABLE : (ch) * 0x100 + (r))
#define DMA_ENABLE      0xff0

// DMA register values
#define DMA_WAIT_RESP   (1 << 3)
#define DMA_CB_DEST_INC (1 << 4)
#define DMA_DEST_DREQ   (1 << 6)
#define DMA_CB_SRCE_INC (1 << 8)
#define DMA_SRCE_DREQ   (1 << 10)
#define DMA_PRIORITY(n) ((n) << 16)





// --- Clock Manager (PCM & PWM Clocks)---
// https://www.scribd.com/doc/127599939/BCM2835-Audio-clocks

constexpr uint32_t CM_BASE    = (PHYS_REG_BASE + 0x101000);
constexpr uint32_t CM_PCMCTL  = 0x98;
constexpr uint32_t CM_PWMCTL  = 0xa0;
constexpr uint32_t CM_PCMDIV  = 0x9c;
constexpr uint32_t CM_PWMDIV  = 0xa4;
constexpr uint32_t CM_PASSWD  = (0x5a << 24);

/*
  Clock Sources and their Frequencies
  https://raspberrypi.stackexchange.com/questions/1153/what-are-the-different-clock-sources-for-the-general-purpose-clocks

  0     0 Hz     Ground
  1     19.2 MHz oscillator
  2     0 Hz     testdebug0
  3     0 Hz     testdebug1
  4     0 Hz     PLLA
  5     1000 MHz PLLC (changes with overclock settings)
  6     500 MHz  PLLD
  7     216 MHz  HDMI auxiliary
  8-15  0 Hz     Ground
*/
enum AP_CM_CLK_SRC
{
  AP_CM_CLK_SRC_GND             = 0,
  AP_CM_CLK_SRC_OSCILLATOR      = 1,
  AP_CM_CLK_SRC_TESTDEBUG0      = 2,
  AP_CM_CLK_SRC_TESTDEBUG1      = 3,
  AP_CM_CLK_SRC_PLLA            = 4,
  AP_CM_CLK_SRC_PLLC            = 5,
  AP_CM_CLK_SRC_PLLD            = 6,
  AP_CM_CLK_SRC_HDMI_AUXILIARY  = 7
};

// const std::map <AP_CM_CLK_SRC, double> AP_CM_CLK_SRC_FREQ =
// {
//   {AP_CM_CLK_SRC_GND,             0.0},
//   {AP_CM_CLK_SRC_OSCILLATOR,      19'200'000.0},
//   {AP_CM_CLK_SRC_TESTDEBUG0,      0.0},
//   {AP_CM_CLK_SRC_TESTDEBUG1,      0.0},
//   {AP_CM_CLK_SRC_PLLA,            0.0},
//   {AP_CM_CLK_SRC_PLLC,            1'000'000'000.0},
//   {AP_CM_CLK_SRC_PLLD,            500'000'000.0},
//   {AP_CM_CLK_SRC_HDMI_AUXILIARY,  216'000'000.0},
// };

enum AP_CM_CLK_MASH
{
  AP_CM_CLK_MASH_INTEGER = 0,
  AP_CM_CLK_MASH_1STAGE  = 1,
  AP_CM_CLK_MASH_2STAGE  = 2,
  AP_CM_CLK_MASH_3STAGE  = 3
};

constexpr AP_CM_CLK_SRC   AP_CM_PWM_CLK_SRC   = AP_CM_CLK_SRC_PLLD;
constexpr AP_CM_CLK_MASH  AP_CM_PWM_CLK_MASH  = AP_CM_CLK_MASH_1STAGE;  



// --- General Raspberry Pi ---
constexpr int PI_MAX_USER_GPIO  = 31;

// --- ADC ---
// ADC and DAC chip-enables
constexpr int SPI_CS_CS = 0;
constexpr int DAC_CE_NUM = 1;

// --- SPI ---
// Page 148

// SPI 0 pin definitions
constexpr uint32_t SPI0_CE0_PIN   = 8;
constexpr uint32_t SPI0_CE1_PIN   = 7;
constexpr uint32_t SPI0_MISO_PIN  = 9;
constexpr uint32_t SPI0_MOSI_PIN  = 10;
constexpr uint32_t SPI0_SCLK_PIN  = 11;

// SPI registers and constants
constexpr uint32_t SPI0_BASE  = (PHYS_REG_BASE + 0x204000);
constexpr uint32_t SPI_CS     = 0x00;
constexpr uint32_t SPI_FIFO   = 0x04;
constexpr uint32_t SPI_CLK    = 0x08;
constexpr uint32_t SPI_DLEN   = 0x0c;
constexpr uint32_t SPI_LTOH   = 0x10;
constexpr uint32_t SPI_DC     = 0x14;

constexpr uint32_t SPI_CS_CLEAR     = (3 << 4);
constexpr uint32_t SPI_RX_FIFO_CLR  = (2 << 4);
constexpr uint32_t SPI_TX_FIFO_CLR  = (1 << 4);
constexpr uint32_t SPI_CS_TA        = (1 << 7);
constexpr uint32_t SPI_CS_DMAEN     = (1 << 8);
constexpr uint32_t SPI_CS_ADCS      = (1 << 11);
constexpr uint32_t SPI_RXD          = (1 << 17);
constexpr uint32_t SPI_CE0          = 0;
constexpr uint32_t SPI_CE1          = 1;

// --- Auxiliaries---
constexpr uint32_t SPI1_SCLK_PIN = 21;
constexpr uint32_t SPI1_MOSI_PIN = 20;
constexpr uint32_t SPI1_MISO_PIN = 19;
constexpr uint32_t SPI1_CE2_PIN  = 16;

constexpr uint32_t AUX_BASE            = (PHYS_REG_BASE + 0x215000);
constexpr uint32_t AUX_ENABLES         = 0x04;
constexpr uint32_t AUX_SPI0_CNTL0_REG  = 0x80; 
constexpr uint32_t AUX_SPI0_CNTL1_REG  = 0x84;
constexpr uint32_t AUX_SPI0_STAT_REG   = 0x88;
constexpr uint32_t AUX_SPI0_IO_REG     = 0xA0;
constexpr uint32_t AUX_SPI0_PEEK_REG   = 0x8C;
constexpr uint32_t AUX_SPI0_TXHOLD_REG = 0xB0;

constexpr uint32_t AUX_SPI1_ENABLE     = (1 << 1); 

// --- GPIO --- 
constexpr uint32_t GPIO_BASE       = (PHYS_REG_BASE + 0x200000);
constexpr uint32_t GPIO_MODE0      = 0x00;
constexpr uint32_t GPSET0          = 0x1C;
constexpr uint32_t GPCLR0          = 0x28;
constexpr uint32_t GPIO_LEV0       = 0x34;
constexpr uint32_t GPIO_GPPUD      = 0x94;
constexpr uint32_t GPIO_GPPUDCLK0  = 0x98;

constexpr uint32_t GPIO_GPFSEL0    = 0x00;
constexpr uint32_t GPIO_GPSET0     = 0x1C;
constexpr uint32_t GPIO_GPCLR0     = 0x28;
constexpr uint32_t GPIO_GPLEV0     = 0x34;
constexpr uint32_t GPIO_GPEDS0     = 0x40;
constexpr uint32_t GPIO_GPREN0     = 0x4C;
constexpr uint32_t GPIO_GPFEN0     = 0x58;

enum AP_GPIO_FUNC
{
  AP_GPIO_FUNC_INPUT  = 0,
  AP_GPIO_FUNC_OUTPUT = 1,
  AP_GPIO_FUNC_ALT0   = 4,
  AP_GPIO_FUNC_ALT1   = 5,
  AP_GPIO_FUNC_ALT2   = 6,
  AP_GPIO_FUNC_ALT3   = 7,
  AP_GPIO_FUNC_ALT4   = 3,
  AP_GPIO_FUNC_ALT5   = 2
};

enum AP_GPIO_PULL
{
  AP_GPIO_PULL_OFF      = 0,
  AP_GPIO_PULL_DOWN     = 1,
  AP_GPIO_PULL_UP       = 2,
  AP_GPIO_PULL_RESERVED = 3
};

// --- Microsecond Timer ---
// Page 172
constexpr uint32_t AP_ST_BASE = PHYS_REG_BASE + 0x3000;  // Physical address! 
constexpr uint32_t AP_ST_CLO  = 0x04;

// --- VideoCore Mailbox ---
// Mailbox command/response structure
typedef struct 
{
  uint32_t len,         // Overall length (bytes)
           req,         // Zero for request, 1<<31 for response
           tag,         // Command number
           blen,        // Buffer length (bytes)
           dlen,        // Data length (bytes)
           uints[32-5]; // Data (108 bytes maximum)
} AP_VC_MSG __attribute__ ((aligned (16)));

// VideoCore Mailbox Allocate Memory Flags
// https://github.com/raspberrypi/firmware/wiki/Mailbox-property-interface#allocate-memory
enum MAILBOX_ALLOCATE_MEMORY_FLAGS
{
  MEM_DISCARDABLE      = 1 << 0, // can be resized to 0 at any time. Use for cached data
  MEM_NORMAL           = 0 << 2, // normal allocating alias. Don't use from ARM
  MEM_DIRECT           = 1 << 2, // 0xC alias uncached
  MEM_COHERENT         = 2 << 2, // 0x8 alias. Non-allocating in L2 but coherent
  MEM_ZERO             = 1 << 4, // initialise buffer to all zeros
  MEM_NO_INIT          = 1 << 5, // don't initialise (default is initialise to all ones)
  MEM_HINT_PERMALOCK   = 1 << 6, // Likely to be locked for long periods of time

  MEM_L1_NONALLOCATING = (MEM_DIRECT | MEM_COHERENT) // Allocating in L2
};

// VC flags for unchached DMA memory
#define DMA_MEM_FLAGS static_cast<MAILBOX_ALLOCATE_MEMORY_FLAGS>((MEM_COHERENT | MEM_ZERO))

// --- GPIO Registers ---
constexpr int GPFSEL0 = 0x0;

// --- Memory ---
// Structure for mapped peripheral or memory


// Size of memory page
constexpr unsigned PAGE_SIZE = 0x1000;

// Round up to nearest page
#define PAGE_ROUNDUP(n) ((n) % PAGE_SIZE == 0 ? (n) : ((n) + PAGE_SIZE) & ~(PAGE_SIZE - 1))



constexpr double AP_PWM_CLK_SRC_FREQ = 100'000'000.0;
constexpr double PWM_VALUE = 2.0;

constexpr int PWM_BASE  = (PHYS_REG_BASE + 0x20C000);
constexpr int PWM_CTL   = 0x00;   // Control
constexpr int PWM_STA   = 0x04;   // Status
constexpr int PWM_DMAC  = 0x08;   // DMA control
constexpr int PWM_RNG1  = 0x10;   // Channel 1 range
constexpr int PWM_DAT1  = 0x14;   // Channel 1 data
constexpr int PWM_FIF1  = 0x18;   // Channel 1 fifo
constexpr int PWM_RNG2  = 0x20;   // Channel 2 range
constexpr int PWM_DAT2  = 0x24;   // Channel 2 data

// PWM register values
constexpr int PWM_CTL_RPTL1 = (1 << 2);  // Chan 1: repeat last data when FIFO empty
constexpr int PWM_CTL_USEF1 = (1 << 5);  // Chan 1: use FIFO
constexpr int PWM_DMAC_ENAB = (1 << 31); // Start PWM DMA
constexpr int PWM_ENAB      = 1;         // Enable PWM
constexpr int PWM_PIN       = 12;        // GPIO pin for PWM output // this was set to 18 before

// If non-zero, set PWM clock using VideoCore mailbox
constexpr int USE_VC_CLOCK_SET = 0;

// --- Helper Macros for Register Access ---
// Get virtual 8 and 32-bit pointers to register
#define REG8(m, x)          ((volatile uint8_t *)  ((uint32_t)(m.virt) + (uint32_t)(x)))
#define REG32(m, x)         ((volatile uint32_t *) ((uint32_t)(m.virt) + (uint32_t)(x)))

// Get bus address of register
#define REG_BUS_ADDR(m, x)  ((uint32_t)(m.bus)  + (uint32_t)(x))

// Convert uncached memory virtual address to bus address
#define MEM_BUS_ADDR(mp, a) ((uint32_t)a - (uint32_t)mp->virt + (uint32_t)mp->bus)

// Convert bus address to physical address (for mmap)
#define BUS_PHYS_ADDR(a)    ((void *)((uint32_t)(a) & ~0xC0000000))

// --- Enums ---

enum MAILBOX_TAG
{
  // https://github.com/raspberrypi/firmware/wiki/Mailbox-property-interface#get-vc-memory

  MAILBOX_TAG_ALLOCATE_MEMORY = 0x3000C,
  MAILBOX_TAG_LOCK_MEMORY     = 0x3000D,
};

enum PIN_INFO_MODE
{
  PIN_INFO_MODE_NONE,
  PIN_INFO_MODE_SERIAL,
  PIN_INFO_MODE_I2C_SDA,
  PIN_INFO_MODE_I2C_SCL,
  PIN_INFO_MODE_SPI_SCLK,
  PIN_INFO_MODE_SPI_MISO,
  PIN_INFO_MODE_SPI_MOSI,
  PIN_INFO_MODE_SPI_CS
};

enum BB_SPI_FLAG
{
  BB_SPI_FLAG_CPHA    = 0,
  BB_SPI_FLAG_CPOL    = 1,
  BB_SPI_FLAG_CSPOL   = 2,
  BB_SPI_FLAG_TX_LSB  = 14,
  BB_SPI_FLAG_RX_LSB  = 15
};

enum PWM_CHANNEL_MODE
{
  PWM_CHANNEL_MODE_PWM = 0,
  PWM_CHANNEL_MODE_SERIALISER = 1
};


// --- Structs ---
typedef struct
{

} Pin_Info_Serial;

typedef struct
{
      
} Pin_Info_I2C;

typedef struct
{
  unsigned  CS,
            MISO,
            MOSI,
            SCLK;
  
  unsigned  usage,
            delay;

  unsigned  spi_flags,
            MISO_mode,
            MOSI_mode,
            CS_mode,
            SCLK_mode;

} Pin_Info_SPI;

typedef struct 
{
  int       mode;
  int       gpio;
  uint32_t  baud;

  union 
  {
    Pin_Info_Serial serial;
    Pin_Info_I2C    i2c;
    Pin_Info_SPI    spi;
  };

} Pin_Info;

typedef struct 
{
  int fd,     // File descriptor
      h,      // Memory handle
      size;   // Memory size in bytes
    
  void *bus,  // Bus address
       *virt, // Virtual address
       *phys; // Physical address
  
  // Software directly accessing peripherals using the DMA engines must use bus addresses
  // Software accessing RAM directly must use physical addresses
  // Software accessing RAM using DMA engines must use bus addresses
} AP_MemoryMap;

class Utility 
{
  public:
    static volatile uint32_t* reg                 (const AP_MemoryMap& mem_map, uint32_t offset);
    static          uint32_t  bus                 (const AP_MemoryMap& mem_map, uint32_t offset);
    static          uint32_t  bus                 (const AP_MemoryMap& mem_map, void*    offset);  
    static          uint32_t  bus                 (const AP_MemoryMap& mem_map, volatile void* offset);  
    static          uint32_t  dma_chan_reg_offset (uint32_t dma_channel,  uint32_t dma_offset);
    static          void      disp_reg_virt       (const AP_MemoryMap& mem_map, uint32_t offset);
    static          void      disp_bit32          (uint32_t bits);
    static          void      write_reg_virt      (volatile uint32_t* reg, uint32_t value, uint32_t mask, uint32_t shift);
    static          void      write_reg_virt      (AP_MemoryMap& mem_map, uint32_t offset, uint32_t value, uint32_t mask, uint32_t shift);
    static          uint32_t  get_bits            (uint32_t input, uint32_t shift, uint32_t mask);  
};

namespace AP
{
  namespace RPI
  {
    // btw: The Pi-0 and Pi-3 do "overclock" the base clock up to 400 MHz on load. 
    // And it may cause SPI problems if the clock isn't fixed 
    // (i.e. at 250 MHz with core_freq=250 in /boot/config.txt).

    static constexpr uint32_t PI_01_REG_BASE = 0x20000000; // Pi Zero or 1 
    static constexpr uint32_t PI_23_REG_BASE = 0x3F000000; // Pi 2 or 3
    static constexpr uint32_t PI_4_REG_BASE  = 0xFE000000; // Pi 4

    #if RPI_VERSION == 0
      static constexpr uint32_t PHYS_REG_BASE = PI_01_REG_BASE;
      static constexpr double   CLOCK_HZ	    = 250'000'000;
   // static constexpr double   SPI_CLOCK_HZ  = 400'000'000;  // !! https://github.com/raspberrypi/linux/issues/2094 !!
      static constexpr double   SPI_CLOCK_HZ  = 250'000'000;  // !! https://github.com/raspberrypi/linux/issues/2094 !!
    
    #elif RPI_VERSION == 1
      static constexpr uint32_t PHYS_REG_BASE = PI_01_REG_BASE;
      static constexpr double   CLOCK_HZ      = 250'000'000;
      static constexpr double   SPI_CLOCK_HZ  = 250'000'000;
    
    #elif RPI_VERSION == 2 || RPI_VERSION == 3
      static constexpr uint32_t PHYS_REG_BASE = PI_23_REG_BASE;
      static constexpr double   CLOCK_HZ      = 250'000'000;
      static constexpr double   SPI_CLOCK_HZ  = 250'000'000;
    
    #elif RPI_VERSION == 4
      static constexpr uint32_t PHYS_REG_BASE = PI_4_REG_BASE;
      static constexpr double   CLOCK_HZ	    = 375'000'000;
      static constexpr double   SPI_CLOCK_HZ  = 200'000'000;
    #endif

    namespace PIN
    {
      namespace AUX
      {
        namespace SPI1
        {
          constexpr unsigned SCLK = 21;
          constexpr unsigned MOSI = 20;
          constexpr unsigned MISO = 19;
          constexpr unsigned CE0  = 18;
          constexpr unsigned CE1  = 17;
          constexpr unsigned CE2  = 16;
        };
      };

      namespace PWM
      {
        constexpr unsigned _0 = 12;
        constexpr unsigned _1 = 19;
      };

      namespace SPI
      {
        constexpr unsigned SCLK = 11;
        constexpr unsigned MOSI = 10;
        constexpr unsigned MISO = 9;
        constexpr unsigned CE0  = 8;
        constexpr unsigned CE1  = 7;
      };
    };

    constexpr uint32_t PAGE_SIZE = 0x1000;
  };

  namespace AUX
  {
    constexpr uint32_t BASE     = RPI::PHYS_REG_BASE + 0x215000;
    constexpr uint32_t ENABLES  = 0x4;

    namespace SPI
    {
      constexpr uint32_t SPI0_BASE = 0x080;
      constexpr uint32_t SPI1_BASE = 0x0C0;
      constexpr uint32_t CNTL0_REG = 0x00;
      constexpr uint32_t CNTL1_REG = 0x04;
      constexpr uint32_t STAT_REG  = 0x08;
      constexpr uint32_t IO_REG    = 0x10;
      constexpr uint32_t PEEK_REG  = 0x14;
    };
  };

  namespace GPIO
  {
    enum class FUNC
    {
      INPUT   = 0,
      OUTPUT  = 1,
      ALT0    = 4,
      ALT1    = 5,
      ALT2    = 6,
      ALT3    = 7,
      ALT4    = 3,
      ALT5    = 2
    };

    enum class PULL
    {
      OFF   = 0,
      DOWN  = 1,
      UP    = 2
    };
  };

  namespace DMA
  {
    struct CTL_BLK
    {
      uint32_t  ti,
                source_ad,
                dest_ad,
                txfr_len,
                stride,
                nextconbk,
                debug,
                unused;
    } __attribute__ (( aligned(32) ));

  
    enum class PERIPH_DREQ
    {
      NONE          = 0,
      DSI           = 1,
      PCM_TX        = 2,
      PCM_RX        = 3,
      SMI           = 4,
      PWM           = 5,
      SPI_TX        = 6,
      SPI_RX        = 7,
      BSC_TX        = 8,
      BSC_RX        = 9,
   // unused        = 10,
      EMMC          = 11,
      UART_TX       = 12,
      SD_HOST       = 13,
      UART_RX       = 14,
   // DSI           = 15,
      SLIMBUS_MCTX  = 16,
      HDMI          = 17,
      SLIMBUX_MCRX  = 18,
      SLIMBUS_DC0   = 19,
      SLIMBUS_DC1   = 20,
      SLIMBUS_DC2   = 21,
      SLIMBUS_DC3   = 22,
      SLIMBUS_DC4   = 23,
      SCALER_FIFO_0 = 24,
      SCALER_FIFO_1 = 25,
      SCALER_FIFO_2 = 26,
      SLIMBUS_DC5   = 27,
      SLIMBUS_DC6   = 28,
      SLIMBUS_DC7   = 29,
      SLIMBUS_DC8   = 30,
      SLIMBUS_DC9   = 31
    };

    namespace TI_DATA
    {
      inline constexpr uint32_t PERMAP (PERIPH_DREQ periph) {return ((static_cast<uint32_t>(periph)) << 16);}
             constexpr uint32_t SRC_DREQ  = 1 << 10;
             constexpr uint32_t SRC_INC   = 1 << 8;
             constexpr uint32_t DEST_DREQ = 1 << 6;
             constexpr uint32_t DEST_INC  = 1 << 4;
             constexpr uint32_t WAIT_RESP = 1 << 3;
             constexpr uint32_t INTEN     = 1 << 0;

    };

    constexpr uint32_t BASE       = RPI::PHYS_REG_BASE + 0x007000;
    constexpr uint32_t CS         = 0x00;
    constexpr uint32_t CONBLK_AD  = 0x04;
    constexpr uint32_t TI         = 0x08;
    constexpr uint32_t SOURCE_AD  = 0x0C;
    constexpr uint32_t DEST_AD    = 0x10;
    constexpr uint32_t TXFR_LEN   = 0x14;
    constexpr uint32_t STRIDE     = 0x18;
    constexpr uint32_t NEXTCONBK  = 0x1C;
    constexpr uint32_t DEBUG      = 0x20;
    constexpr uint32_t INT_STATUS = 0xFE0;
    constexpr uint32_t ENABLE     = 0xFF0;

    constexpr unsigned CHAN_COUNT = 16;
  };

  namespace GPIO
  {
    constexpr uint32_t BASE       = RPI::PHYS_REG_BASE + 0x200000;
    constexpr uint32_t GPFSEL0    = 0x00;
    constexpr uint32_t GPSET0     = 0x1C;
    constexpr uint32_t GPCLR0     = 0x28;
    constexpr uint32_t GPLEV0     = 0x34;
    constexpr uint32_t GPEDS0     = 0x40;
    constexpr uint32_t GPREN0     = 0x4C;
    constexpr uint32_t GPFEN0     = 0x58;
    constexpr uint32_t GPHEN0     = 0x64;
    constexpr uint32_t GPLEN0     = 0x70;
    constexpr uint32_t GPPUD      = 0x94;
    constexpr uint32_t GPPUDCLK0  = 0x98;
  };

  namespace PWM
  {
    enum class ALGO
    {
      BALANCED  = 0,
      MARKSPACE = 1
    };

    constexpr uint32_t BASE = RPI::PHYS_REG_BASE + 0x20C000;
    constexpr uint32_t CTL  = 0x00;
    constexpr uint32_t STA  = 0x04;
    constexpr uint32_t DMAC = 0x08;
    constexpr uint32_t RNG1 = 0x10;
    constexpr uint32_t DAT1 = 0x14;
    constexpr uint32_t FIF1 = 0x18;
    constexpr uint32_t RNG2 = 0x20;
    constexpr uint32_t DAT2 = 0x24;
  };

  namespace SPI
  {
    // https://www.analog.com/en/analog-dialogue/articles/introduction-to-spi-interface.html
    // 0: CS polarity low, data sampled rising, shifted out falling
    // 1: CS polarity low, data sampled falling, shifted out rising
    // 2: CS polarity high, data sampled rising, shifted out falling
    // 3: CS polarity high, data sampled faling, shifted out rising
    enum class MODE
    {
      _0,
      _1,
      _2,
      _3
    };

    inline constexpr MODE CALC_MODE (bool CPOL, bool CPHA) 
    {
      if (!CPOL && !CPHA)
      {
        return MODE::_0;
      }
      else if (!CPOL && CPHA)
      {
        return MODE::_1;
      }
      else if (CPOL && !CPHA)
      {
        return MODE::_2;
      }
      else if (CPOL && CPHA)
      {
        return MODE::_3;
      }
    };

    constexpr uint32_t BASE = RPI::PHYS_REG_BASE + 0x204000;
    constexpr uint32_t CS   = 0x00;
    constexpr uint32_t FIFO = 0x04;
    constexpr uint32_t CLK  = 0x08;
    constexpr uint32_t DLEN = 0x0C;
    constexpr uint32_t LTOH = 0x10;
    constexpr uint32_t DC   = 0x14;
  };

  namespace CLKMAN
  {
    enum class TYPE
    {
      PCM,
      PWM
    };

    enum class MASH
    {
      INTEGER,
      ONE_STAGE,
      TWO_STAGE,
      THREE_STAGE
    };

    /*
      Clock Sources and their Frequencies
      https://raspberrypi.stackexchange.com/questions/1153/what-are-the-different-clock-sources-for-the-general-purpose-clocks

      0     0 Hz     Ground
      1     19.2 MHz oscillator
      2     0 Hz     testdebug0
      3     0 Hz     testdebug1
      4     0 Hz     PLLA
      5     1000 MHz PLLC (changes with overclock settings)
      6     500 MHz  PLLD
      7     216 MHz  HDMI auxiliary
      8-15  0 Hz     Ground
    */
    enum class SOURCE
    {
      GND,
      OSCILLATOR,
      TESTDEBUG0,
      TESTDEBUG1,
      PLLA,
      PLLC,
      PLLD,
      HDMI
    };

    static unsigned SOURCE_FREQUENCY [] = 
    {
      0,
      19'200'000,
      0,
      0,
      0,
      1'000'000'000,
      500'000'000,
      216'000'000,
  };

    constexpr uint32_t BASE      = RPI::PHYS_REG_BASE + 0x101000;
    constexpr uint32_t PWM_CTL   = 0xA0;
    constexpr uint32_t PCM_CTL   = 0x98;
    constexpr uint32_t PWM_DIV   = 0xA4;
    constexpr uint32_t PCM_DIV   = 0x9C;
    constexpr uint32_t PASSWD    = 0x5a << 24;
    constexpr uint32_t CTL       = 0xA0;
    constexpr uint32_t DIV       = 0xA4;
    constexpr double   FREQUENCY = 100'000'000.0;
  };

  namespace SYSTIMER
  {
    constexpr uint32_t BASE = RPI::PHYS_REG_BASE + 0x003000;
    constexpr uint32_t CS   = 0x00;
    constexpr uint32_t CLO  = 0x04;
    constexpr uint32_t CHI  = 0x08;
    constexpr uint32_t C0   = 0x0C;
    constexpr uint32_t C1   = 0x10;
    constexpr uint32_t C2   = 0x14;
    constexpr uint32_t C3   = 0x18;
  };

  namespace SPI_BB
  {
    constexpr double MAX_BAUD = 200'000.0;
    constexpr double MIN_BAUD = 50.0;
  };
};

class AikaPi
{
  private:
        class Mailbox
    {
      // https://github.com/raspberrypi/firmware/wiki/Mailbox-property-interface

      private:
        // Mailbox command/response structure
        // https://jsandler18.github.io/extra/prop-channel.html
        typedef struct 
        {
          uint32_t  len,         // Overall length (bytes)
                    req,         // Zero for request, 1<<31 for response
                    tag,         // Command number
                    blen,        // Buffer length (bytes)
                    dlen,        // Data length (bytes)
                    uints[32-5]; // Data (108 bytes maximum)
        } MSG __attribute__ ((aligned (16)));

        enum class TAG : uint32_t
        {
          ALLOCATE_MEMORY = 0x3000C,
          LOCK_MEMORY     = 0x3000D,
          UNLOCK_MEMORY   = 0x3000E,
          RELEASE_MEMORY  = 0x3000F,
        };

      public:
        // VideoCore Mailbox Allocate Memory Flags
        // https://github.com/raspberrypi/firmware/wiki/Mailbox-property-interface#allocate-memory
        enum class ALLOC_MEM_FLAG
        {
          DISCARDABLE      = 1 << 0, // can be resized to 0 at any time. Use for cached data
          NORMAL           = 0 << 2, // normal allocating alias. Don't use from ARM
          DIRECT           = 1 << 2, // 0xC alias uncached
          COHERENT         = 2 << 2, // 0x8 alias. Non-allocating in L2 but coherent
          ZERO             = 1 << 4, // initialise buffer to all zeros
          NO_INIT          = 1 << 5, // don't initialise (default is initialise to all ones)
          HINT_PERMALOCK   = 1 << 6, // Likely to be locked for long periods of time

          L1_NONALLOCATING = (DIRECT | COHERENT) // Allocating in L2
        };

        static  uint32_t  page_roundup  (uint32_t addr);
        static  int       mb_open       ();
        static  void      mb_close      (int fd); 
        static  uint32_t  message       (int fd, MSG& msg); 
        static  uint32_t  mem_alloc     (int fd, uint32_t size, ALLOC_MEM_FLAG flags);
        static  void*     mem_lock      (int fd, int h);
        static  uint32_t  mem_unlock    (int fd, int h);
        static  uint32_t  mem_release   (int fd, int h);
    };

    class Peripheral
    {
      protected:
        int   m_fd    = 0,
              m_h     = 0,
              m_size  = 0;
        void *m_bus   = nullptr,
             *m_virt  = nullptr,
             *m_phys  = nullptr;

      protected:
        static uint32_t page_roundup      (uint32_t addr);
        static void*    map_phys_to_virt  (void* phys_addr, unsigned size);
               void     map_addresses     (void* phys_addr);
        static void*    conv_bus_to_phys  (void* bus_addr);
        static void     unmap_segment     (void* virt_addr, unsigned size);

      public:
        Peripheral (void* phys_addr);
        Peripheral ();
       ~Peripheral (); 

        static constexpr uint32_t  wbits     (uint32_t data, uint32_t value, uint32_t shift, uint32_t mask = 0x1);

        static           uint32_t  rreg      (uint32_t* reg);
        static           void      wreg      (uint32_t* reg, uint32_t value);
        static           uint32_t  rbits     (uint32_t* reg, uint32_t shift, uint32_t mask = 0x1);
        static           void      wbits     (uint32_t* reg, uint32_t value, uint32_t shift, uint32_t mask = 0x1);

        static           uint32_t  rreg      (volatile uint32_t* reg);
        static           void      wreg      (volatile uint32_t* reg, uint32_t value);
        static           uint32_t  rbits     (volatile uint32_t* reg, uint32_t shift, uint32_t mask = 0x1);
        static           void      wbits     (volatile uint32_t* reg, uint32_t value, uint32_t shift, uint32_t mask = 0x1);

        volatile         uint32_t* reg       (uint32_t offset) const;
                         void      reg       (uint32_t offset, uint32_t value);
                         uint32_t  reg_rbits (uint32_t offset, uint32_t shift, uint32_t mask = 0x1) const;
                         void      reg_wbits (uint32_t offset, uint32_t value, uint32_t shift, uint32_t mask = 0x1);

                         void*     bus       () const;
                         uint32_t  bus       (uint32_t offset) const;
                         void*     virt      () const;
                         void*     phys      () const;

                         void      disp_reg  (uint32_t offet) const;
        static           void      print_u32 (uint32_t value);
    }; 

    class GPIO : public Peripheral 
    {
      public: 
        GPIO (void* phys_addr);
       ~GPIO ();

        void set    (unsigned pin, AP::GPIO::FUNC func_val, AP::GPIO::PULL pull_val, bool value = 0);
        void func   (unsigned pin, AP::GPIO::FUNC func_val);
        void pull   (unsigned pin, AP::GPIO::PULL pull_val);
        void write  (unsigned pin, bool value);
        bool read   (unsigned pin);
    };

    class SPI : public Peripheral
    {
      private:
        double m_frequency = 10'000'000.0;

      public:
        SPI (void* phys_addr);
       ~SPI ();

        double  frequency   (double value);
        void    clear_fifo  ();
    };

    class DMA : public Peripheral 
    {
      private:
        uint32_t dma_chan_reg_offset (unsigned chan, uint32_t offset) const;

      private:
        void init ();

      public:
        DMA (void* phys_addr);
       ~DMA ();

      volatile uint32_t* reg        (unsigned dma_chan, uint32_t offset) const;
               void      reg        (unsigned dma_chan, uint32_t offset, uint32_t value);
               uint32_t  reg_rbits  (unsigned dma_chan, uint32_t offset, unsigned shift, uint32_t mask = 0x1) const;
               void      reg_wbits  (unsigned dma_chan, uint32_t offset, unsigned value, unsigned shift, uint32_t mask = 0x1);
               void      disp_reg   (unsigned dma_chan, uint32_t offset) const;    
               uint32_t  dest_ad    (unsigned dma_chan);
               void      start      (unsigned dma_chan, uint32_t start_cb_bus_addr);
               void      reset      (unsigned dma_chan);
               bool      is_running (unsigned dma_chan) const;
               void      pause      (unsigned dma_chan);
               void      next_cb    (unsigned dma_chan, uint32_t next_cb_bus_addr);
               void      abort      (unsigned dma_chan);
               void      run        (unsigned dma_chan);
               void      stop       (unsigned dma_chan);
               uint32_t  conblk_ad  (unsigned dma_chan) const;
    }; 
    
    class PWM : public Peripheral 
    {
      private:
        double m_duty_cycle = 50.0;

      private:
        void init ();       

      public:
        PWM (void* phys_addr);
       ~PWM ();
      
      void      start            (bool channel);
      void      stop             (bool channel);
      void      algo             (bool channel, AP::PWM::ALGO algo);
      void      use_fifo         (bool channel, bool value);
      void      reset            ();
      double    frequency        (bool channel, double value);
      void      duty_cycle       (bool channel, double value);
      uint32_t  range            (bool channel);
      bool      is_using_fifo    (bool channel);
      void      repeat_last_data (bool channel, bool value);
      void      clear_fifo       ();
      bool      is_fifo_empty    () const;
      bool      is_fifo_full     () const;
    };

    class AUX : public Peripheral 
    { 
      private:
        class SPI
        {
           private:
            bool channel  = 0;
            AUX* aux      = nullptr;

          private:
            uint32_t off (uint32_t offset) const;
          
          public:
            SPI (bool channel, AUX* aux);

            bool is_rx_fifo_empty       () const;
            void init                   ();
            void enable                 ();
            void disable                ();
            void shift_length           (uint8_t value);
            void shift_out_ms_bit_first (bool value);
            void mode                   (unsigned mode);
            void clock_polarity         (bool value);
            void in_rising              (bool value);
            void out_rising             (bool balue);
            void chip_selects           (uint8_t value);
            void frequency              (double value);
            void clear_fifos            ();
            void xfer                   (char* rxd, char* txd, unsigned length);
            void read                   (char* rxd, unsigned length);
            void write                  (char* txd, unsigned length);
        };

      private:
        SPI m_spi[2];
      
      public:
        AUX (void* phys_addr);

        void init               ();
        SPI& spi                (bool channel);
        void master_enable_spi  (bool channel);
        void master_disable_spi (bool channel);
    };

    class ClockManager
    {
      private:
        class ClkManPeriph : public Peripheral
        {
          protected:
            AP::CLKMAN::TYPE m_type;

          protected:
            uint32_t off (uint32_t offset) const;
          
          public:
            ClkManPeriph (void* phys_addr);

            void stop                 ();
            void start                ();
            bool is_running           ();
            void source               (AP::CLKMAN::SOURCE source);
            void mash                 (AP::CLKMAN::MASH mash);
            void divisor              (uint32_t integral, uint32_t fractional);
            void frequency            (double value, 
                                        AP::CLKMAN::SOURCE source_val = AP::CLKMAN::SOURCE::PLLD, 
                                        AP::CLKMAN::MASH   mash_val   = AP::CLKMAN::MASH::ONE_STAGE);
            double frequency          ();

            AP::CLKMAN::SOURCE source ();
        };

        class PCM : public ClkManPeriph
        {
          public:
            PCM (void* phys_addr, AP::CLKMAN::TYPE type);
        };

        class PWM : public ClkManPeriph
        {
          public:
            PWM (void* phys_addr, AP::CLKMAN::TYPE type);
        };

      public:
        PCM pcm;
        PWM pwm;

        ClockManager (void* phys_addr);
    };

    class SystemTimer : public Peripheral
    {
      // https://jsandler18.github.io/extra/sys-time.html
      // The system timer is a Free Running Timer that 
      // increments a 64 bit counter every microsecond, 
      // starting as soon as the Pi boots up, 
      // and runs in the background for as long as the Pi is on.
      
      public: 
        SystemTimer (void* phys_addr);
       ~SystemTimer ();

       uint32_t low () const;
    };


  public: 
    class Uncached : public Peripheral
    {
      public: 
        Uncached (unsigned size);
        Uncached ();
       ~Uncached ();

      void*    map_uncached_mem  (unsigned size);
      uint32_t bus               (void* offset) const volatile;
      uint32_t bus               (volatile void* offset) const volatile;
    };

    class SPI_BB
    {
      private:
        int     m_CS,
                m_MISO,
                m_MOSI,
                m_SCLK;

        double  m_baud,
                m_delay;

        bool    m_CS_polarity             = 0,  // CS is active high or low
                m_shift_out_ms_bit_first  = true,
                m_receive_ms_bit_first    = true;
        
        AP::SPI::MODE m_mode;
    
      private: 
        void    init       ();
        void    delay      ();
        void    delay      (double baud);
        void    set_CS     ();
        void    clear_CS   ();
        void    set_SCLK   ();
        void    clear_SCLK ();
        void    start      ();
        void    stop       ();
        uint8_t xfer_byte  (char txd);
        bool    cpol       ();
        bool    cpha       ();

      public:
        SPI_BB (unsigned CS, unsigned MISO, unsigned MOSI, unsigned SCLK, double baud, AP::SPI::MODE i_mode = AP::SPI::MODE::_0);
       ~SPI_BB ();

        void mode                 (AP::SPI::MODE value);
        void baud                 (double value);
        void shift_out_msb_first  (bool value);
        void receive_msb_first    (bool value);
        void cs_polarity          (bool value);
        void xfer                 (char* rxd, char* txd, unsigned length);
        void write                (char* txd, unsigned length);
    };

  public:
    static SPI          spi;
    static DMA          dma;
    static PWM          pwm;
    static GPIO         gpio;
    static AUX          aux;
    static ClockManager cm;
    static SystemTimer  st;
  
  public:
    AikaPi ();
   ~AikaPi ();
};

#endif