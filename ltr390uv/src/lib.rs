//! Driver for LTR390UV.

#![no_std]

use embedded_hal_async::delay::DelayNs;

pub mod blocking;

pub const ADDRESS: u8 = 0x53;

pub mod regs {
    /// ALS/UVS operation mode control, SW reset
    pub const MAIN_CTRL: u8 = 0x00;

    /// ALS/UVS measurement rate and resolution in Active Mode
    pub const ALS_UVS_MEAS_RATE: u8 = 0x04;

    /// ALS/UVS analog Gain range
    pub const ALS_UVS_GAIN: u8 = 0x05;

    /// Part number ID and revision ID
    pub const PART_ID: u8 = 0x06;

    /// Power-On status, Interrupt status, Data status
    pub const MAIN_STATUS: u8 = 0x07;

    /// ALS ADC measurement data, LSB
    pub const ALS_DATA_0: u8 = 0x0D;

    /// ALS ADC measurement data
    pub const ALS_DATA_1: u8 = 0x0E;

    /// ALS ADC measurement data, MSB
    pub const ALS_DATA_2: u8 = 0x0F;

    /// UVS ADC measurement data, LSB
    pub const UVS_DATA_0: u8 = 0x10;

    /// UVS ADC measurement data
    pub const UVS_DATA_1: u8 = 0x11;

    /// UVS ADC measurement data, MSB
    pub const UVS_DATA_2: u8 = 0x12;

    /// Interrupt configuration
    pub const INT_CFG: u8 = 0x19;

    /// Interrupt persist setting
    pub const INT_PST: u8 = 0x1A;

    /// ALS/UVS interrupt upper threshold, LSB
    pub const ALS_UVS_THRES_UP_0: u8 = 0x21;

    /// ALS/UVS interrupt upper threshold, intervening bits
    pub const ALS_UVS_THRES_UP_1: u8 = 0x22;

    /// ALS/UVS interrupt upper threshold, MSB
    pub const ALS_UVS_THRES_UP_2: u8 = 0x23;

    /// ALS/UVS interrupt lower threshold, LSB
    pub const ALS_UVS_THRES_LOW_0: u8 = 0x24;

    /// ALS/UVS interrupt lower threshold, intervening bits
    pub const ALS_UVS_THRES_LOW_1: u8 = 0x25;

    /// ALS/UVS interrupt lower threshold, MSB
    pub const ALS_UVS_THRES_LOW_2: u8 = 0x26;
}

#[derive(Debug)]
pub enum Error<E> {
    Bus(E),
    InvalidDevice,
    OldData,
}

impl<E> From<E> for Error<E> {
    fn from(e: E) -> Self {
        Error::Bus(e)
    }
}

/// ALS/UVS resolution
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Resolution {
    /// 20 Bit, Conversion time = 400ms
    Bit20 = 0b000,
    /// 19 Bit, Conversion time = 200ms
    Bit19 = 0b001,
    /// 18 Bit, Conversion time = 100ms (default)
    #[default]
    Bit18 = 0b010,
    /// 17 Bit, Conversion time = 50ms
    Bit17 = 0b011,
    /// 16 Bit, Conversion time = 25ms
    Bit16 = 0b100,
    /// 13 Bit, Conversion time = 12.5ms
    Bit13 = 0b101,
    /// Reserved
    Reserved6 = 0b110,
    /// Reserved
    Reserved7 = 0b111,
}

impl Resolution {
    /// Get the conversion time in milliseconds
    fn conversion_time_ms(&self) -> f32 {
        match self {
            Resolution::Bit20 => 400.0,
            Resolution::Bit19 => 200.0,
            Resolution::Bit18 => 100.0,
            Resolution::Bit17 => 50.0,
            Resolution::Bit16 => 25.0,
            Resolution::Bit13 => 12.5,
            _ => 0.0, // Reserved values
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum MeasurementRate {
    /// 25ms measurement rate
    Ms25 = 0b000,
    /// 50ms measurement rate
    Ms50 = 0b001,
    /// 100ms measurement rate (default)
    #[default]
    Ms100 = 0b010,
    /// 200ms measurement rate
    Ms200 = 0b011,
    /// 500ms measurement rate
    Ms500 = 0b100,
    /// 1000ms measurement rate
    Ms1000 = 0b101,
    /// 2000ms measurement rate
    Ms2000 = 0b110,
    /// 2000ms measurement rate (alternate encoding)
    Ms2000Alt = 0b111,
}

impl MeasurementRate {
    /// Get the measurement rate in milliseconds
    pub fn rate_ms(&self) -> u16 {
        match self {
            MeasurementRate::Ms25 => 25,
            MeasurementRate::Ms50 => 50,
            MeasurementRate::Ms100 => 100,
            MeasurementRate::Ms200 => 200,
            MeasurementRate::Ms500 => 500,
            MeasurementRate::Ms1000 => 1000,
            MeasurementRate::Ms2000 | MeasurementRate::Ms2000Alt => 2000,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum GainRange {
    /// Gain Range: 1
    Gain1 = 0b000,
    /// Gain Range: 3 (default)
    #[default]
    Gain3 = 0b001,
    /// Gain Range: 6
    Gain6 = 0b010,
    /// Gain Range: 9
    Gain9 = 0b011,
    /// Gain Range: 18
    Gain18 = 0b100,
}

impl GainRange {
    /// Get the numeric gain value
    pub fn value(&self) -> u8 {
        match self {
            GainRange::Gain1 => 1,
            GainRange::Gain3 => 3,
            GainRange::Gain6 => 6,
            GainRange::Gain9 => 9,
            GainRange::Gain18 => 18,
        }
    }
}

#[derive(Debug, Default)]
pub struct Config {
    pub resolution: Resolution,
    pub rate: MeasurementRate,
    pub gain: GainRange,
}

pub struct LTR390UV<I2C: embedded_hal_async::i2c::I2c> {
    i2c: I2C,
    addr: u8,
    config: Config,
}

impl<I2C: embedded_hal_async::i2c::I2c> LTR390UV<I2C> {
    pub fn new(i2c: I2C, addr: u8) -> Self {
        Self {
            i2c,
            addr,
            config: Config::default(),
        }
    }

    pub fn new_primary(i2c: I2C) -> Self {
        Self::new(i2c, ADDRESS)
    }

    pub async fn init(&mut self, config: Config) -> Result<(), Error<I2C::Error>> {
        if self.read_reg(regs::PART_ID).await? & 0xF0 != 0b1011_0000 {
            return Err(Error::InvalidDevice);
        }

        self.write_reg(regs::MAIN_CTRL, 0x00).await?; // standby mode

        self.write_reg(
            regs::ALS_UVS_MEAS_RATE,
            ((config.resolution as u8) << 4) | config.rate as u8,
        )
        .await?;
        self.write_reg(regs::ALS_UVS_GAIN, config.gain as u8).await?;

        Ok(())
    }

    pub async fn soft_reset(&mut self) -> Result<(), Error<I2C::Error>> {
        self.write_reg(regs::MAIN_CTRL, 0b10000).await?; // software reset

        Ok(())
    }

    pub async fn read_als_data(&mut self, mut delay: impl DelayNs) -> Result<u32, Error<I2C::Error>> {
        self.write_reg(regs::MAIN_CTRL, 0b0010).await?; // active sensor

        delay
            .delay_ms(10 + self.config.resolution.conversion_time_ms() as u32 + 1)
            .await;

        //if self.read_reg(regs::MAIN_CTRL).await? & 0b10 == ! {
        //   return Err(Error::InvalidDevice);
        //}
        if self.read_reg(regs::MAIN_STATUS).await? & 0b1000 == 0 {
            return Err(Error::OldData);
        }

        let mut buf = [0u8; 3];
        self.read_regs(regs::ALS_DATA_0, &mut buf).await?;

        Ok(u32::from(buf[0]) | (u32::from(buf[1]) << 8) | (u32::from(buf[2]) << 16))
    }

    pub async fn read_uvs_data(&mut self, mut delay: impl DelayNs) -> Result<u32, Error<I2C::Error>> {
        self.write_reg(regs::MAIN_CTRL, 0b1010).await?; // active sensor

        delay
            .delay_ms(10 + self.config.resolution.conversion_time_ms() as u32 + 1)
            .await;

        //if self.read_reg(regs::MAIN_CTRL).await? & 0b10 == ! {
        //   return Err(Error::InvalidDevice);
        //}
        if self.read_reg(regs::MAIN_STATUS).await? & 0b1000 == 0 {
            return Err(Error::OldData);
        }

        let mut buf = [0u8; 3];
        self.read_regs(regs::UVS_DATA_0, &mut buf).await?;

        Ok(u32::from(buf[0]) | (u32::from(buf[1]) << 8) | (u32::from(buf[2]) << 16))
    }

    pub async fn read_lux(&mut self, mut delay: impl DelayNs) -> Result<u32, Error<I2C::Error>> {
        let als_data = self.read_als_data(&mut delay).await? as f32;

        let gain = self.config.gain.value() as f32;
        let int = (self.config.rate.rate_ms() as f32) / 100.0; // integration time

        const W_FAC: f32 = 1.0;

        let lux = 0.6 * als_data / (gain * int) * W_FAC;

        Ok(lux as u32)
    }

    /// Read UV index
    pub async fn read_uvi(&mut self, mut delay: impl DelayNs) -> Result<u32, Error<I2C::Error>> {
        let uvs_data = self.read_uvs_data(&mut delay).await? as f32;

        const UV_SENSITIVITY: f32 = 2300.0;
        const W_FAC: f32 = 1.0;

        let uvi = uvs_data / UV_SENSITIVITY * W_FAC;

        Ok(uvi as u32)
    }

    pub async fn read_reg(&mut self, reg: u8) -> Result<u8, I2C::Error> {
        let mut buf = [0];
        self.i2c.write_read(self.addr, &[reg], &mut buf).await?;
        Ok(buf[0])
    }

    async fn read_regs(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), I2C::Error> {
        self.i2c.write_read(self.addr, &[reg], buf).await
    }

    // Add this new method to write to registers
    pub async fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), I2C::Error> {
        self.i2c.write(self.addr, &[reg, value]).await
    }
}
