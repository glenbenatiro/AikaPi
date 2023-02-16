#ifndef AIKAPI_H
#define AIKAPI_H

#include <cstdint>
#include <mutex>

// Link to the BCM2385 datasheet:
// // https://www.raspberrypi.org/app/uploads/2012/02/BCM2835-ARM-Peripherals.pdf

#define RPI_VERSION  0

#if RPI_VERSION == 0
  #define PHYS_REG_BASE PI_01_REG_BASE
  #define CLOCK_HZ	    250000000
  #define SPI_CLOCK_HZ  400000000
#elif RPI_VERSION == 1
  #define PHYS_REG_BASE PI_01_REG_BASE
  #define CLOCK_HZ	    250000000
  #define SPI_CLOCK_HZ  250000000
#elif RPI_VERSION == 2 || RPI_VERSION == 3
  #define PHYS_REG_BASE PI_23_REG_BASE
  #define CLOCK_HZ	    250000000
  #define SPI_CLOCK_HZ  250000000
#elif RPI_VERSION == 4
  #define PHYS_REG_BASE PI_4_REG_BASE
  #define CLOCK_HZ	    375000000
  #define SPI_CLOCK_HZ  200000000
#endif

// Location of peripheral registers in physical memory
// This can be seen on page 5 of ARM BCM2385 document
#define PI_01_REG_BASE 0x20000000 // Pi Zero or 1 
#define PI_23_REG_BASE 0x3F000000 // Pi 2 or 3
#define PI_4_REG_BASE  0xFE000000 // Pi 4

// Location of peripheral registers in bus memory
// This can be seen on page 5 of ARM BCM2385 document
constexpr uint32_t BUS_REG_BASE = 0x7E000000;

// --- DMA ---
constexpr uint32_t DMA_BASE           = (PHYS_REG_BASE + 0x007000);
constexpr uint32_t DMA_TI_DREQ_PWM    = 5;
constexpr uint32_t DMA_TI_DREQ_SPI_RX = 7;
constexpr uint32_t DMA_TI_DREQ_SPI_TX = 6;
constexpr uint32_t DMA_TI_SRC_DREQ    = 1 << 10;
constexpr uint32_t DMA_TI_SRC_INC     = 1 << 8;
constexpr uint32_t DMA_TI_DEST_DREQ   = 1 << 6;
constexpr uint32_t DMA_TI_DEST_INC    = 1 << 4;
constexpr uint32_t DMA_TI_WAIT_RESP   = 1 << 3;

constexpr uint32_t DMA_CB_TI_PWM    = (DMA_TI_DREQ_PWM << 16) | DMA_TI_DEST_DREQ | DMA_TI_WAIT_RESP;
constexpr uint32_t DMA_CB_TI_SPI_TX = (DMA_TI_DREQ_SPI_TX << 16) | DMA_TI_DEST_DREQ | DMA_TI_SRC_INC | DMA_TI_WAIT_RESP;
constexpr uint32_t DMA_CB_TI_SPI_RX = (DMA_TI_DREQ_SPI_RX << 16) | DMA_TI_SRC_DREQ | DMA_TI_DEST_INC | DMA_TI_WAIT_RESP;

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

// DMA control block (must be 32-byte aligned)
// https://www.raspberrypi.org/app/uploads/2012/02/BCM2835-ARM-Peripherals.pdf
// 4.2.1.1 Control Block Data Structure, page 40
typedef struct {
  uint32_t ti,      // Transfer info
           srce_ad, // Source address
           dest_ad, // Destination address
           tfr_len, // Transfer length in bytes
           stride,  // Transfer stride
           next_cb, // Next control block
           debug,   // Debug register, zero in control block
           unused;
} DMA_CB __attribute__ ((aligned(32)));


// Clock Manager Audio Clock
// https://www.scribd.com/doc/127599939/BCM2835-Audio-clocks
constexpr uint32_t CM_BASE    = (PHYS_REG_BASE + 0x101000);
constexpr uint32_t CM_PWMCTL  = 0xa0;
constexpr uint32_t CM_PWMDIV  = 0xa4;
constexpr uint32_t CM_PASSWD  = 0x5a000000;
constexpr int PWM_CLOCK_ID    = 0xa;

// --- General Raspberry Pi ---
constexpr int PI_MAX_USER_GPIO  = 31;

// --- ADC ---
// ADC and DAC chip-enables
constexpr int SPI_CS_CS = 0;
constexpr int DAC_CE_NUM = 1;

// --- SPI ---
// Page 148

// SPI 0 pin definitions
#define SPI0_CE0_PIN    8
#define SPI0_CE1_PIN    7
#define SPI0_MISO_PIN   9
#define SPI0_MOSI_PIN   10
#define SPI0_SCLK_PIN   11

// SPI registers and constants
constexpr int SPI0_BASE = (PHYS_REG_BASE + 0x204000);
constexpr int SPI_CS    = 0x00;
constexpr uint32_t SPI_FIFO = 0x04;
constexpr int SPI_CLK   = 0x08;
constexpr int SPI_DLEN  = 0x0c;
constexpr int SPI_LTOH  = 0x10;
constexpr int SPI_DC    = 0x14;

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
constexpr int SPI1_SCLK_PIN = 21;
constexpr int SPI1_MOSI_PIN = 20;
constexpr int SPI1_MISO_PIN = 19;
constexpr int SPI1_CE2_PIN  = 16;

constexpr int AUX_BASE            = (PHYS_REG_BASE + 0x215000);
constexpr int AUX_ENABLES         = 0x04;
constexpr int AUX_SPI0_CNTL0_REG  = 0x80; 
constexpr int AUX_SPI0_CNTL1_REG  = 0x84;
constexpr int AUX_SPI0_STAT_REG   = 0x88;
constexpr int AUX_SPI0_IO_REG     = 0xA0;
constexpr int AUX_SPI0_PEEK_REG   = 0x8C;
constexpr int AUX_SPI0_TXHOLD_REG = 0xB0;

constexpr int AUX_SPI1_ENABLE     = (1 << 1); 

// --- GPIO --- 
constexpr int GPIO_BASE       = (PHYS_REG_BASE + 0x200000);
constexpr int GPIO_MODE0      = 0x00;
constexpr int GPSET0          = 0x1C;
constexpr int GPCLR0          = 0x28;
constexpr int GPIO_LEV0       = 0x34;
constexpr int GPIO_GPPUD      = 0x94;
constexpr int GPIO_GPPUDCLK0  = 0x98;


constexpr int GPIO_GPFSEL0    = 0x00;
constexpr int GPIO_GPSET0     = 0x1C;
constexpr int GPIO_GPCLR0     = 0x28;
constexpr int GPIO_GPLEV0     = 0x34;
constexpr int GPIO_GPEDS0     = 0x40;
constexpr int GPIO_GPREN0     = 0x4C;
constexpr int GPIO_GPFEN0     = 0x58;

constexpr int GPIO_IN     = 0;
constexpr int gpio_write   = 1;
constexpr int GPIO_ALT0   = 4;
constexpr int GPIO_ALT1   = 5;
constexpr int GPIO_ALT2   = 6;
constexpr int GPIO_ALT3   = 7;
constexpr int GPIO_ALT4   = 3;
constexpr int GPIO_ALT5   = 2;
constexpr int GPIO_NOPULL = 0;
constexpr int GPIO_PULLDN = 1;
constexpr int GPIO_PULLUP = 2;



// --- Microsecond Timer ---
// Page 172
constexpr uint32_t USEC_BASE  = PHYS_REG_BASE + 0x3000;  // Physical address! 
constexpr uint32_t USEC_TIME  = 0x04;
constexpr uint32_t USEC_CLO   = 0x04; // System Timer Counter Lower 32 bits 

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



// --- PWM ---
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

/*
  This is used by the gpio_mode function to set GPIO pin function (input, output, or alt functions 0-5)
*/
enum GPIO_MODES
{
  GPIO_MODE_INPUT   = 0,
  GPIO_MODE_OUTPUT  = 1,
  GPIO_MODE_ALT0    = 4,
  GPIO_MODE_ALT1    = 5, 
  GPIO_MODE_ALT2    = 6,
  GPIO_MODE_ALT3    = 7,
  GPIO_MODE_ALT4    = 3,
  GPIO_MODE_ALT5    = 2
};

/*
  This is used by the gpio_pull function to set GPIO pin pull-up/down resistor state
*/
enum GPIO_PULL
{
  GPIO_PULL_DISABLE = 0,
  GPIO_PULL_DOWN    = 1,
  GPIO_PULL_UP      = 2,
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

enum CM_PWM_MASH
{
  CM_PWM_MASH_INTEGER = 0,
  CM_PWM_MASH_1STAGE  = 1,
  CM_PWM_MASH_2STAGE  = 2,
  CM_PWM_MASH_3STAGE  = 3
};

// --- Structs ---
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
} MemoryMap;

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
  std::mutex mutex;

  union 
  {
    Pin_Info_Serial serial;
    Pin_Info_I2C    i2c;
    Pin_Info_SPI    spi;
  };

} Pin_Info;

class Utility 
{
  public:
    static uint32_t   get_bits   (uint32_t input, unsigned shift, uint32_t mask);
    static void       reg_write  (MemoryMap mem_map, uint32_t offset, uint32_t value, uint32_t mask, unsigned shift);


    static void reg_write (volatile uint32_t *reg, 
                           uint32_t           value, 
                           uint32_t           mask, 
                           unsigned           shift)
    {
      *reg = (*reg & ~(mask << shift)) | (value << shift);
    }

    static void      print_bits (int bits, unsigned size = 1);


  // inline functions

  // Return a uint32_t to the virtual address of specific register of a peripheral
  static volatile uint32_t* get_reg32 (MemoryMap mem_map, uint32_t  offset)
  {
    return (volatile uint32_t *)((uint32_t)(mem_map.virt) + (uint32_t)(offset));
  }

  // Return a uint32_t bus address of a specific register of a peripheral 
  static uint32_t reg_bus_addr (MemoryMap *_MemoryMap, uint32_t offset)
  {
    return ((uint32_t)(_MemoryMap->bus) + offset); 
  }

  // Return a bus address, given virtual address
  static uint32_t mem_bus_addr (MemoryMap *_MemoryMap, volatile void* offset)
  {
    return ((uint32_t)(offset) - (uint32_t)(_MemoryMap->virt) + 
    (uint32_t)(_MemoryMap->bus));
  }

  // Get the offset of a specific DMA channel's register from DMA base
  static uint32_t dma_chan_reg_offset (uint32_t dma_channel, uint32_t dma_register)
  {
    return ((dma_channel * 0x100) + dma_register);
  }
};

// --- AikaPi ---

class AikaPi
{
  private: 
    int m_pwm_value       = 10;

    double m_pwm_freq = 0.0;

    Pin_Info m_pin_info [PI_MAX_USER_GPIO + 1];

    bool m_is_pwm_init = false;

  public:
    //int m_spi_frequency = LAB_SPI_FREQUENCY;

    MemoryMap m_regs_gpio,
              m_regs_dma, 
              m_regs_clk, 
              m_regs_pwm, 
              m_regs_spi, 
              m_regs_usec,
              m_aux_regs;

    // MemoryMap m_vc_mem;
       
  public:
    int   m_fifo_fd = 0;

    uint32_t m_pwm_range,  
             m_fifo_size;


    AikaPi ();
   ~AikaPi ();

    // --- General ---
    void    delay (uint32_t microseconds);

    // --- Memory ---
    void     map_periph       (MemoryMap *mp, void *phys, int size);    
    void     map_devices      ();
    void*    map_segment      (void *addr, int size);
    void     unmap_segment    ();
    void*    map_uncached_mem (MemoryMap *mp, int size);
    void     unmap_periph_mem (MemoryMap *mp);
    
    // --- Videocore Mailbox ---
    int  	   mailbox_open        (void);
    void     disp_vc_msg      (AP_VC_MSG *msgp);
    void 	   mailbox_close       (int fd);
    void     unmap_segment    (void *mem, int  size);
    void*    vc_mem_lock      (int fd, int h);
    uint32_t mbox_msg         (int fd, AP_VC_MSG *msgp);
    uint32_t fset_vc_clock    (int fd, int id, uint32_t freq);
    uint32_t vc_mem_release      (int fd, int h);
    uint32_t vc_mem_unlock    (int fd, int h);
    uint32_t vc_mem_alloc     (int fd, uint32_t size, MAILBOX_ALLOCATE_MEMORY_FLAGS flags);
  
    // --- Aux ---
    void     fail             (const char *s);
    void     terminate        (int sig);
        
    // --- DMA ---
    void     dma_enable       (int chan);
    //void     dma_start        (uint32_t dma_channel, uint32_t dma_cb_address);

    void dma_start (MemoryMap *mp, 
                                int      chan, 
                                DMA_CB  *cbp, 
                                uint32_t csval);

    void     dma_disp         (int chan);
    void     dma_reset         (int chan);
    void     dma_wait         (int chan);
    void      dma_pause       (unsigned channel);
    bool      is_dma_paused   (unsigned channel);
    void      dma_play        (unsigned channel);
    void      dma_abort       (unsigned channel);
    uint32_t  dma_transfer_len (int chan);
    
    // --- GPIO ---
    void      gpio_set    (int pin, int mode, int pull);
    void      gpio_mode   (int pin, int mode);
    void      gpio_pull   (int pin, int pull);
    bool      gpio_read   (int pin);
    void      gpio_write  (unsigned pin, bool value); 
    uint32_t  gpio_mode   (unsigned pin);
    
    // --- SPI ---
    int       spi_init            (double frequency);
    void      spi_clear_fifo ();
    void      spi_disp            ();
    void      spi_disable         ();
    void      spi_xfer            (uint8_t *txd, uint8_t *rxd, int length);
    void      spi_cs              (int set);
    int       spi_set_clock_rate  (int value);

    // --- Bit Bang SPI ---
    int       bb_spi_open (unsigned CS, unsigned MISO, unsigned MOSI, unsigned SCLK, unsigned baud, unsigned spi_flags);
    int       bb_spi_xfer (unsigned CS, char *txbuf, unsigned count);
    int       bb_spi_xfer (unsigned CS, char *txbuf, char *rxbuf, unsigned count);
    uint8_t   bb_spi_xfer_byte (Pin_Info *pi, char txbyte);

    void      bb_spi_start       (Pin_Info *pi);
    void      bb_spi_stop        (Pin_Info *pi);
    void      bb_spi_delay       (Pin_Info *pi);
    void      bb_spi_set_CS      (Pin_Info *pi);
    void      bb_spi_clear_CS    (Pin_Info *pi);
    void      bb_spi_set_SCLK    (Pin_Info *pi);
    void      bb_spi_clear_SCLK  (Pin_Info *pi);

    // --- Utility Peripherals ---
    void aux_spi1_master_enable ();
    void aux_spi1_master_disable ();

    // --- Utility SPI ---
    // NOTE THAT THE FOLLOWING ARE ACTUALLY SPI1, NOT SPI0. 
    // THIS IS A MISNOMER. PLEASE SEE ERRATA DOCUMENT.
    void aux_spi0_init ();
    void aux_spi0_enable ();
    void aux_spi0_disable ();
    void aux_spi0_frequency (double frequency);
    void aux_spi0_chip_selects (bool CE2, bool CE1, bool CE0);
    void aux_spi0_clear_fifos ();
    void aux_spi0_mode (uint8_t mode);
    void aux_spi0_clock_polarity (bool polarity);
    void aux_spi0_in_rising (bool value);
    void aux_spi0_out_rising (bool value);
    void aux_spi0_shift_length (uint8_t value);
    void aux_spi0_shift_out_MS_first (bool value);
    void aux_spi0_shift_in_MS_first (bool value);
    void aux_spi0_write (char *buf, unsigned int length);
    void aux_spi0_read (char *txbuf, char *rxbuf, unsigned int length); 
    void aux_spi_xfer   (uint8_t channel, char *txbuff, char *rxbuff, uint8_t count);
    void aux_spi_write  (uint8_t channel, char *txbuff,               uint8_t count);

    // --- PWM ---
    int     pwm_init              (unsigned channel, double source_freq, uint32_t range, uint32_t val);
    void    pwm_start             ();
    void    pwm_stop              ();
    double  pwm_frequency         (double value, double duty_cycle);
    int     pwm_enable            (unsigned channel, bool value);
    int     pwm_mode              (unsigned channel, int value);
    int     pwm_repeat_last_data  (unsigned channel, bool value);
    int     pwm_use_fifo          (unsigned channel, bool value);
    bool    pwm_channel_state     (unsigned channel);
    int     pwm_fifo              (uint32_t value);
    int     pwm_range             (unsigned channel, uint32_t value);
    int     pwm_reset             (unsigned channel);

    // --- Clock Manager Audio Clocks ---
    double  cm_pwm_frequency     (double value);
    double  cm_pwm_frequency     (double value, uint32_t mash);

    // --- FIFO ---
    int      fifo_create      (const char *fifo_name);
    int      fifo_open_write  (const char *fifo_name);
    int      fifo_write       (int fd, void *data, int dlen);
    uint32_t fifo_get_free_space   (int fd);
    void     fifo_destroy     (char *fifo_name, int fd);
    int      fifo_is_fifo     (const char *fname);

    // --- Utility ---
    int sleep_nano (int nano);
    // int sleep_secs (int secs);
};

#endif