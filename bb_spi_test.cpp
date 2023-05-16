#include <iostream>
#include <thread>
#include <cstring>

#include "AikaPi.h"
#include "../../src/LAB_AD9833.h"

// compile with
// g++ AikaPi.cpp ../../src/LAB_AD9833.cpp ../AD9833/AD9833.cpp bb_spi_test.cpp -o bb_spi_test

constexpr int CS = 13;
constexpr int MISO = 5;
constexpr int MOSI = 4;
constexpr int SCLK = 6;

int main ()
{
  LAB_AD9833 func_gen (CS, MISO, MOSI, SCLK, 100'000);

  func_gen.frequency (2);
  func_gen.wave_type (LAB_AD9833::WAVE_TYPE::SINE);

  func_gen.run ();  

  for (int a = 0; a <= 360; a++)
  {
    func_gen.phase (0);
    
    std::this_thread::sleep_for (std::chrono::milliseconds (2000));

    func_gen.phase (180);

    std::cout << a << "\n";
  }
  
  return 0;
}