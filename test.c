#include <stdio.h>
#include <pigpio.h>

// compile using
// gcc -Wall -pthread -o test test.c -lpigpio -lrt

int main ()
{
  gpioInitialise ();

  int fd = spiOpen (2, 100000, (1 << 8) | 2);

  char txbuf[2] = {0xFA, 0xCE};
  char rxbuf[2] = {0, 0};

  spiXfer (fd, txbuf, rxbuf, 2);

  printf ("%u %u\n", rxbuf[1], rxbuf[0]);

  gpioTerminate ();

  return 0;
}