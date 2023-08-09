#ifndef AIKAPI_H
#define AIKAPI_H

#include <cstdint>

// BCM2385 datasheet:
// https://www.raspberrypi.org/app/uploads/2012/02/BCM2835-ARM-Peripherals.pdf

#define RPI_VERSION 3

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
      static constexpr double   SPI_CLOCK_HZ  = 500'000'000;
    
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

    constexpr uint32_t BUS_REG_BASE = 0x7E000000;
    constexpr uint32_t PAGE_SIZE    = 0x1000;
  };

  namespace AUX
  {
    constexpr uint32_t BASE     = RPI::PHYS_REG_BASE + 0x215000;
    constexpr uint32_t ENABLES  = 0x4;

    namespace SPI
    {
      constexpr uint32_t SPI0_BASE  = 0x080;
      constexpr uint32_t SPI1_BASE  = 0x0C0;
      constexpr uint32_t CNTL0_REG  = 0x00;
      constexpr uint32_t CNTL1_REG  = 0x04;
      constexpr uint32_t STAT_REG   = 0x08;
      constexpr uint32_t PEEK_REG   = 0x0C;
      constexpr uint32_t IO_REG     = 0x20;
      constexpr uint32_t TXHOLD_REG = 0x30;
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
             constexpr uint32_t WAIT_RESP = 1 << 3; // wait for a response
             constexpr uint32_t INTEN     = 1 << 0; // interrupt enable

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

    constexpr unsigned NUMBER_OF_CHANNELS = 16;
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

    enum class EVENT
    {
      RISING_EDGE,
      FALLING_EDGE,
      HIGH,
      LOW,
      ASYNC_RISING_EDGE,
      ASYNC_FALLING_EDGE,
    };

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
    constexpr uint32_t GPAREN0    = 0x7C;
    constexpr uint32_t GPAFEN0    = 0x88;
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
    // https://www.scribd.com/doc/127599939/BCM2835-Audio-clocks

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
      0,              // GND
      19'200'000,     // Oscillator
      0,              // testdebug0
      0,              // testdebug1
      0,              // PLLA
      1'000'000'000,  // PLLC
      500'000'000,    // PLLD
      216'000'000,    // HDMI auxiliary
    };

    constexpr uint32_t BASE      = RPI::PHYS_REG_BASE + 0x101000;
    constexpr uint32_t PWM_CTL   = 0xA0;
    constexpr uint32_t PCM_CTL   = 0x98;
    constexpr uint32_t PWM_DIV   = 0xA4;
    constexpr uint32_t PCM_DIV   = 0x9C;
    constexpr uint32_t PASSWD    = 0x5a << 24;
    constexpr uint32_t CTL       = 0xA0;
    constexpr uint32_t DIV       = 0xA4;
    constexpr double   FREQUENCY = 100'000'000.0; // default PWM clock source frequency
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

  namespace INTERRUPT
  {
    constexpr uint32_t BASE               = RPI::PHYS_REG_BASE + 0x00B000;
    constexpr uint32_t IRQ_BASIC_PENDING  = 0x200;
    constexpr uint32_t IRQ_PENDING_1      = 0x204;
    constexpr uint32_t IRQ_PENDING_2      = 0x208;
    constexpr uint32_t FIQ_CONTROL        = 0x20C;
    constexpr uint32_t ENABLE_IRQ_1       = 0x210;
    constexpr uint32_t ENABLE_IRQ_2       = 0x214;
    constexpr uint32_t ENABLE_BASIC_IRQ   = 0x218;
    constexpr uint32_t DISABLE_IRQ_1      = 0x21C;
    constexpr uint32_t DISABLE_IRQ_2      = 0x220;
    constexpr uint32_t DISABLE_BASIC_IRQ  = 0x224;
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

        void      set                       (unsigned pin, AP::GPIO::FUNC func_val, AP::GPIO::PULL pull_val, bool value = 0);
        void      func                      (unsigned pin, AP::GPIO::FUNC func_val);
        void      pull                      (unsigned pin, AP::GPIO::PULL pull_val);
        void      write                     (unsigned pin, bool value);
        bool      read                      (unsigned pin);
        uint32_t  event_detect_status       () const;
        bool      event_detect_status       (unsigned pin) const;
        void      clear_event_detect_status ();
        void      clear_event_detect_status (unsigned pin); 
        void      set_event_detect          (unsigned pin, AP::GPIO::EVENT event, bool state);
        void      clear_all_event_detect    (unsigned pin);
        uint32_t  level                     () const;
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
        uint32_t  dma_chan_reg_offset (unsigned chan, uint32_t offset) const;
        bool      chan_check          (unsigned dma_chan);
        bool      chan_check_throw    (unsigned dma_chan);

      private:
        void init ();

      public:
        DMA (void* phys_addr);
       ~DMA ();

      volatile uint32_t*  reg             (unsigned dma_chan, uint32_t offset) const;
               void       reg             (unsigned dma_chan, uint32_t offset, uint32_t value);
               uint32_t   reg_rbits       (unsigned dma_chan, uint32_t offset, unsigned shift, uint32_t mask = 0x1) const;
               void       reg_wbits       (unsigned dma_chan, uint32_t offset, unsigned value, unsigned shift, uint32_t mask = 0x1);
               void       disp_reg        (unsigned dma_chan, uint32_t offset) const;    
               uint32_t   dest_ad         (unsigned dma_chan);
               void       start           (unsigned dma_chan, uint32_t start_cb_bus_addr);
               void       reset           (unsigned dma_chan);
               bool       is_running      (unsigned dma_chan) const;
               void       pause           (unsigned dma_chan);
               void       next_cb         (unsigned dma_chan, uint32_t next_cb_bus_addr);
               void       abort           (unsigned dma_chan);
               void       run             (unsigned dma_chan);
               void       stop            (unsigned dma_chan);
               uint32_t   conblk_ad       (unsigned dma_chan) const;
               void       clear_interrupt (unsigned dma_chan); 
               bool       interrupt       (unsigned dma_chan) const;
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
      
      void      start             (bool channel);
      void      stop              (bool channel);
      void      algo              (bool channel, AP::PWM::ALGO algo);
      void      use_fifo          (bool channel, bool value);
      void      reset             ();
      double    frequency         (bool channel, double value);
      void      duty_cycle        (bool channel, double value);
      uint32_t  range             (bool channel);
      bool      is_using_fifo     (bool channel);
      void      repeat_last_data  (bool channel, bool value);
      void      clear_fifo        ();
      bool      is_fifo_empty     () const;
      bool      is_fifo_full      () const;
      bool      is_running        (bool channel) const;
    };

    class AUX : public Peripheral 
    { 
      private:
        class SPI
        {
           private:
            bool      m_channel     = 0;
            bool      m_cs_polarity = 0;
            unsigned  m_cs          = 0;
            AUX*      m_aux         = nullptr;

          private:
            void      init              ();
            unsigned  cs_pin            (unsigned cs);
            bool      busy              ();
            void      in_rising         (bool value);
            void      out_rising        (bool balue);
            void      clock_polarity    (bool value);
            bool      cpol              ();
            bool      is_rx_fifo_empty  () const;
            void      chip_selects      (uint8_t value);
            bool      rx_empty          ();
            bool      tx_full           ();
            uint32_t  tx_bits           (char* txd, unsigned position);
            void      rx_bits           (char* rxd, unsigned position);
            uint32_t  shift_length      ();
            
          public:
            SPI (bool channel, AUX* aux);
            
            uint32_t  off                     (uint32_t offset) const;
            void      enable                  ();
            void      disable                 ();
            void      shift_length            (uint8_t bits);
            void      shift_out_ms_bit_first  (bool value);
            void      shift_in_ms_bit_first   (bool value);
            void      mode                    (AP::SPI::MODE mode);
            void      frequency               (double value);
            void      clear_fifos             ();
            void      xfer                    (char* rxd, char* txd, unsigned length);
            void      xfer                    (char* rxd, char* txd, unsigned length, unsigned cs_pin);
            void      read                    (char* rxd, unsigned length);
            void      write                   (char* txd, unsigned length);
            void      cs_polarity             (bool value);
            void      cs                      (unsigned cs_pin);
            bool      shift_out_ms_bit_first  ();
            bool      shift_in_ms_bit_first   ();
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

    class Interrupt : public Peripheral 
    {
      private:

      public:
        Interrupt (void* phys_addr);
       ~Interrupt ();
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

        void mode                   (AP::SPI::MODE value);
        void baud                   (double value);
        void shift_out_ms_bit_first (bool value);
        void receive_ms_bit_first   (bool value);
        void cs_polarity            (bool value);
        void xfer                   (char* rxd, char* txd, unsigned length);
        void write                  (char* txd, unsigned length);
    };

  public:
    static SPI          spi;
    static DMA          dma;
    static PWM          pwm;
    static GPIO         gpio;
    static AUX          aux;
    static ClockManager cm;
    static SystemTimer  st;
    static Interrupt    interrupt;
  
  public:
    AikaPi ();
   ~AikaPi ();
};

#endif