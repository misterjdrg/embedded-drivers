# edrv-st7735

Rust driver for ST7735.

## Overview

The ST7735 is a single-chip controller/driver for 262K-color, graphic type TFT-LCD. It consists of 396 source line and 162
gate line driving circuits.

## Maintenance

This project is maintained by the embedded-drivers team. Our organization's goal is to provide consistent driver access interfaces for embedded Rust and to maintain these drivers collectively to avoid orphaned crates.

## Features

- Display driver using SPI
- Display driver using async SPI

## Usage

```rust
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;

let spi = hpm_hal::spi::Spi::new_txonly(r.spi1, r.sclk, r.mosi, r.dma_ch0, config);

let spi_bus = Mutex::<NoopRawMutex, _>::new(spi);
let spi_dev = embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice::new(&spi_bus, cs);
let mut display: ST7735<Display160x80Type2, _, _> = ST7735::new(spi_dev, dc);

let mut fb = Framebuffer::<
    Rgb565,
    _,
    BigEndian,
    160,
    80,
    { embedded_graphics::framebuffer::buffer_size::<Rgb565>(160, 80) },
>::new();

display.init(&mut Delay).await;
```

Or using blocking SPI:

```rust
use embedded_hal_bus::spi::AtomicDevice;
use embedded_hal_bus::util::AtomicCell;

let spi = hal::spi::Spi::new_blocking_txonly(r.spi1, r.sclk, r.mosi, config);
let spi_bus = AtomicCell::new(spi);

let spi_dev = AtomicDevice::new(&spi_bus, cs, Delay).unwrap();
let mut display: ST7735<Display160x80Type2, _, _> = ST7735::new(spi_dev, dc);
```

### Common Modules

- ST7735 0.96 Inch 160x80 Color TFT LCD Display Module

## Documentation

[Docs.rs link](https://docs.rs/edrv-st7735/)

## Contributing & License

Please refer to [embedded-drivers](https://github.com/embedded-drivers/embedded-drivers)
