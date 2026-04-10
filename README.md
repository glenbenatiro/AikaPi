# AikaPi

AikaPi is a C++ library for bare-metal access to Raspberry Pi peripherals. It provides direct register-level control of GPIO, SPI, DMA, PWM, system timers, interrupts, and clock management, all through a clean object-oriented interface.

## Features

### GPIO
- Pin function selection (input, output, alt functions 0-5)
- Pull-up/pull-down configuration
- Read/write individual pins
- Event detection (rising/falling edge, high/low level, async)

### SPI
- Hardware SPI (SPI0) with configurable clock frequency
- Auxiliary SPI (SPI1 and SPI2) via the AUX peripheral
- Bit-banged SPI (`SPI_BB`) for arbitrary GPIO pins, supporting all four SPI modes
- Configurable bit order (MSB/LSB first), chip select polarity, and clock polarity

### DMA
- 16-channel DMA controller access
- Control block management
- Start, stop, pause, and abort transfers
- Interrupt handling and status queries
- Peripheral DREQ mapping for paced transfers

### PWM
- Two-channel PWM output
- Balanced and mark-space algorithms
- Configurable frequency and duty cycle
- FIFO mode support

### Clock Manager
- PWM and PCM clock control
- Configurable clock source (oscillator, PLLA, PLLC, PLLD, HDMI)
- MASH noise-shaping divisor modes
- Frequency setting with automatic divisor calculation

### System Timer
- Access to the free-running 64-bit microsecond counter

### Interrupts
- IRQ and FIQ register access

### Uncached Memory
- VideoCore mailbox-based memory allocation
- Bus/physical/virtual address translation
- Used for DMA control blocks and buffers

## Supported Boards

AikaPi auto-detects the board revision at runtime and maps the correct peripheral base address:

| Processor | Boards | Peripheral Base |
|-----------|--------|-----------------|
| BCM2835 | Pi 1, Pi Zero, Pi Zero W | `0x20000000` |
| BCM2836 | Pi 2 | `0x3F000000` |
| BCM2837 | Pi 3, Pi Zero 2W | `0x3F000000` |
| BCM2711 | Pi 4, Pi 400, CM4 | `0xFE000000` |

## Architecture

AikaPi is implemented as a lazy singleton. All peripherals are accessed through the single `AikaPi` instance:

```cpp
AikaPi& rpi = AikaPi::get_instance();

// GPIO
rpi.gpio.set(pin, AP::GPIO::FUNC::OUTPUT, AP::GPIO::PULL::OFF);
rpi.gpio.write(pin, true);

// Hardware SPI
rpi.spi.frequency(1'000'000.0);

// Auxiliary SPI
rpi.aux.spi(0).frequency(500'000.0);
rpi.aux.spi(0).xfer(rx_buf, tx_buf, length);

// DMA
rpi.dma.start(channel, cb_bus_addr);

// PWM
rpi.pwm.frequency(0, 1000.0);
rpi.pwm.duty_cycle(0, 50.0);
rpi.pwm.start(0);

// System timer
uint32_t time_us = rpi.st.low();
```

### Internal Structure

```
AikaPi (singleton)
├── gpio           - GPIO register access
├── spi            - Hardware SPI (SPI0)
├── aux            - Auxiliary peripherals
│   └── spi(0/1)   - Auxiliary SPI (SPI1/SPI2)
├── dma            - DMA controller (16 channels)
├── pwm            - PWM controller (2 channels)
├── cm             - Clock manager
│   ├── pwm        - PWM clock
│   └── pcm        - PCM clock
├── st             - System timer
└── interrupt      - Interrupt controller
```

## Files

```
AikaPi.h    - Header with all class definitions, register offsets, and constants
AikaPi.cpp  - Implementation
```

## Usage

AikaPi is a header-only dependency. Add both `AikaPi.h` and `AikaPi.cpp` to your project and compile them alongside your source files.

```cmake
add_executable(my_project
  main.cpp
  path/to/AikaPi.cpp
)
```

The library requires access to `/dev/mem` and `/dev/vcio`, so the program must be run with root privileges:

```bash
sudo ./my_project
```

## Reference

- [BCM2835 ARM Peripherals Datasheet](https://datasheets.raspberrypi.com/bcm2835/bcm2835-peripherals.pdf)
- [BCM2835 Datasheet Errata](https://elinux.org/BCM2835_datasheet_errata)
- [Raspberry Pi Peripheral Addresses](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html#peripheral-addresses)
- [VideoCore Mailbox Property Interface](https://github.com/raspberrypi/firmware/wiki/Mailbox-property-interface)
- [BCM2835 Audio Clocks](https://www.scribd.com/doc/127599939/BCM2835-Audio-clocks)

## Disclaimer

This project is for research and educational purposes only. It is not intended for production use.

## License

MIT License. See [LICENSE](LICENSE) for details.
