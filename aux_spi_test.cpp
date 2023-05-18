#include <iostream>
#include <thread>

#include "../MCP4XXX/MCP4XXX.h"
#include "../../src/LAB_MCP4XXX.h"
#include "AikaPi.h"

// build using
// g++ AikaPi.cpp ../../src/LAB_MCP4XXX.cpp ../MCP4XXX/MCP4XXX.cpp aux_spi_test.cpp -o aux_spi_test

int main ()
{
  AikaPi AP;


  LAB_MCP4XXX pot (LAB_MCP4XXX::PART_NUMBER::MCP4161,
                    LAB_MCP4XXX::RESISTANCE_VERSION::_103,
                    2);

  for (int b = 0; b < 10; b++)
  {
    for (int a = 0; a <= 100; a++)
    {
      std::cout << pot.resistance_per (0, a) << "\n";

      std::this_thread::sleep_for (std::chrono::milliseconds (50));
    }

    for (int a = 100; a >= 0; a--)
    {
      std::cout << pot.resistance_per (0, a)  << "\n";

      std::this_thread::sleep_for (std::chrono::milliseconds (50));
    }
  }


  return 0;
}