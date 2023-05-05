#include <iostream>
#include <thread>
#include <chrono>

#include "AikaPi.h"

// compile with
// g++ AikaPi.cpp clkman_diagnose.cpp -o clkman_diagnose

int main ()
{
  AikaPi AP;

  //AP.cm.pwm.reg (AP::CLKMAN::PWM_CTL, 0x0);
  //AP.cm.pwm.reg (AP::CLKMAN::PCM_CTL, 0x0);
  //AP.cm.pcm.reg (AP::CLKMAN::PWM_DIV, 0x0);
  //AP.cm.pcm.reg (AP::CLKMAN::PCM_DIV, 0x0);

  std::this_thread::sleep_for (std::chrono::seconds (1));

  AP.cm.pwm.disp_reg (AP::CLKMAN::PWM_CTL);
  AP.cm.pwm.disp_reg (AP::CLKMAN::PWM_DIV);
  AP.cm.pcm.disp_reg (AP::CLKMAN::PCM_CTL);
  AP.cm.pcm.disp_reg (AP::CLKMAN::PCM_CTL);

  return 0;
}