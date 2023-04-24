#include <iostream>

#include "AikaPi.h"

// compile with
// g++ -o test test.cpp AikaPi.cpp

constexpr int PWM0_PIN = 12;
constexpr int PWM_CHAN = 1;
constexpr double LAB_PWM_FREQUENCY = 20'000'000.0;
constexpr double LAB_OSCILLOSCOPE::MAX_SAMPLING_RATE = 200'000;

int main ()
{
  AikaPi _AikaPi;

  _AikaPi.gpio_set (PWM0_PIN, AP_GPIO_FUNC_OUTPUT, GPIO_PULL_DOWN);

  uint32_t m_pwm_range       = (LAB_PWM_FREQUENCY * 2) / LAB_OSCILLOSCOPE::MAX_SAMPLING_RATE;

  _AikaPi.pwm_mode              (PWM_CHAN, PWM_CHANNEL_MODE_PWM);
  _AikaPi.pwm_use_fifo          (PWM_CHAN, true);
  _AikaPi.pwm_repeat_last_data  (PWM_CHAN, true);
  _AikaPi.pwm_range             (PWM_CHAN, m_pwm_range);
  _AikaPi.pwm_fifo              (2);
  _AikaPi.pwm_enable            (PWM_CHAN, true);

  while (true);

  return 0;
}