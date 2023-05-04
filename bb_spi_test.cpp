#include <iostream>
#include <thread>
#include <cstring>

#include "AikaPi.h"

// compile with
// g++ AikaPi.cpp bb_spi_test.cpp -o bb_spi_test

constexpr int CS = 2;
constexpr int MISO = 3;
constexpr int MOSI = 4;
constexpr int SCLK = 14;

int main ()
{
  std::cout << "I hope you can read me." << std::endl;

  return 0;
}