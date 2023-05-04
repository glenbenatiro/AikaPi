#include <iostream>
#include <thread>
#include <cstring>

#include "AikaPi.h"

// compile with
// g++ -o test test.cpp AikaPi.cpp

struct DMA_Info
{
  AP::DMA::CTL_BLK cb [5];

  uint32_t pwm_fif1_data;
};

int main ()
{
  AikaPi AP;
  int count = 0;
  
  AP.gpio.set (AP::RPI::PIN::PWM::_0, AP::GPIO::FUNC::ALT0, AP::GPIO::PULL::DOWN, 0);
  AP.gpio.set (AP::RPI::PIN::PWM::_1, AP::GPIO::FUNC::ALT5, AP::GPIO::PULL::DOWN, 0);

  // chan 1
  AP.pwm.frequency        (1, 100000.0);
  AP.pwm.use_fifo         (1, false);
  AP.pwm.repeat_last_data (1, false);
  
  // chan 0
  AP.pwm.frequency        (0, 100000.0);
  AP.pwm.use_fifo         (0, true);
  AP.pwm.repeat_last_data (0, false);

  AP.pwm.clear_fifo ();

  // create uncached memory
  AikaPi::Uncached uncached;
  uncached.map_uncached_mem (10000);

  // store dma_info
  DMA_Info& info = *(static_cast<DMA_Info*>(uncached.virt ()));

  uint32_t flag = AP::DMA::TI_DATA::PERMAP (AP::DMA::PERIPH_DREQ::PWM) |
                  AP::DMA::TI_DATA::DEST_DREQ |
                  AP::DMA::TI_DATA::WAIT_RESP;

  DMA_Info dma_info = 
  {
    .cb = 
    {
      {
        flag,
        uncached.bus  (&info.pwm_fif1_data),
        AP.pwm.bus    (AP::PWM::FIF1),
        4,
        0,
        uncached.bus  (&info.cb[1]),
        0
      },
      {
        flag,
        uncached.bus  (&info.pwm_fif1_data),
        AP.pwm.bus    (AP::PWM::FIF1),
        4,
        0,
        uncached.bus  (&info.cb[0]),
        0
      }
    },

    .pwm_fif1_data = 0xFA
  };

  std::memcpy (&info, &dma_info, sizeof (dma_info));
  
  AP.pwm.reg (AP::PWM::DMAC, (1 << 31) | (8 << 8) | (1 << 0));
  AP.dma.start (7, uncached.bus (&info.cb[0]));

  AP.pwm.start (0);
  AP.pwm.start (1);

  std::cout << std::hex << "cb[0] bus addr: " << uncached.bus (&info.cb[0]) << "\n";

  while (true)
  {
    std::this_thread::sleep_for (std::chrono::seconds (1));

    info.pwm_fif1_data = AP.pwm.range (0) / 2;

    std::this_thread::sleep_for (std::chrono::seconds (1));

    info.pwm_fif1_data = AP.pwm.range (0) / 4;

    // std::cout << std::hex << *(AP.dma.reg (7, AP::DMA::CONBLK_AD)) << "\n";
  }

  return 0;
}