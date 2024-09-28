//! Driver for BMP180 and BMP085.

#![no_std]

use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::I2c;
use micromath::F32Ext;

// BMP180, BMP085 address.
pub const ADDRESS: u8 = 0x77;

pub mod regs {
    pub const CAL_AC1: u8 = 0xAA; // R   Calibration data (16 bits)
    pub const CAL_AC2: u8 = 0xAC; // R   Calibration data (16 bits)
    pub const CAL_AC3: u8 = 0xAE; // R   Calibration data (16 bits)
    pub const CAL_AC4: u8 = 0xB0; // R   Calibration data (16 bits)
    pub const CAL_AC5: u8 = 0xB2; // R   Calibration data (16 bits)
    pub const CAL_AC6: u8 = 0xB4; // R   Calibration data (16 bits)
    pub const CAL_B1: u8 = 0xB6; // R   Calibration data (16 bits)
    pub const CAL_B2: u8 = 0xB8; // R   Calibration data (16 bits)
    pub const CAL_MB: u8 = 0xBA; // R   Calibration data (16 bits)
    pub const CAL_MC: u8 = 0xBC; // R   Calibration data (16 bits)
    pub const CAL_MD: u8 = 0xBE; // R   Calibration data (16 bits)

    pub const CONTROL: u8 = 0xF4;
    pub const TEMPDATA: u8 = 0xF6;
    pub const PRESSUREDATA: u8 = 0xF6;
}

pub mod cmds {
    pub const READTEMPCMD: u8 = 0x2E;
    pub const READPRESSURECMD: u8 = 0x34;
}

#[derive(Debug)]
pub enum Error<E> {
    Bus(E),
    CalibrationDataError,
    ConversionError,
}

impl<E> From<E> for Error<E> {
    fn from(error: E) -> Self {
        Error::Bus(error)
    }
}

/// Hardware pressure sampling accuracy modes.
#[repr(u8)]
#[derive(Copy, Clone)]
pub enum Mode {
    UltraLowPower = 0,
    Standard,
    HighResolution,
    UltraHighResolution,
}

impl Mode {
    fn oversampling_settings(&self) -> u8 {
        *self as u8
    }

    fn wait_conversion<D: DelayNs>(&self, delay: &mut D) {
        let ms = match self {
            Mode::UltraLowPower => 5,
            Mode::Standard => 8,
            Mode::HighResolution => 14,
            Mode::UltraHighResolution => 26,
        };
        delay.delay_ms(ms);
    }
}

#[allow(unused)]
struct CalibrationData {
    ac1: i16,
    ac2: i16,
    ac3: i16,
    ac4: u16,
    ac5: u16,
    ac6: u16,
    b1: i16,
    b2: i16,
    mb: i16,
    mc: i16,
    md: i16,
}

impl CalibrationData {
    fn from_raw(raw: &[u8]) -> Self {
        Self {
            ac1: i16::from_be_bytes([raw[0], raw[1]]),
            ac2: i16::from_be_bytes([raw[2], raw[3]]),
            ac3: i16::from_be_bytes([raw[4], raw[5]]),
            ac4: u16::from_be_bytes([raw[6], raw[7]]),
            ac5: u16::from_be_bytes([raw[8], raw[9]]),
            ac6: u16::from_be_bytes([raw[10], raw[11]]),
            b1: i16::from_be_bytes([raw[12], raw[13]]),
            b2: i16::from_be_bytes([raw[14], raw[15]]),
            mb: i16::from_be_bytes([raw[16], raw[17]]),
            mc: i16::from_be_bytes([raw[18], raw[19]]),
            md: i16::from_be_bytes([raw[20], raw[21]]),
        }
    }
}

pub struct Config {
    mode: Mode,
}

impl Default for Config {
    fn default() -> Self {
        Config {
            mode: Mode::UltraHighResolution,
        }
    }
}

/// Represents a measurement taken by the BMP180 sensor, including temperature and pressure.
#[derive(Clone, Copy, Debug)]
pub struct Measurement {
    /// The temperature measured in degrees Celsius.
    pub temperature: f32,
    /// The pressure measured in Pascals.
    pub pressure: i32,
}

/// BMP180, or BMP085.
pub struct BMP180<I2C> {
    i2c: I2C,
    addr: u8,
    // oversampling settings
    mode: Mode,
    calib: CalibrationData,
}

impl<I2C: I2c> BMP180<I2C> {
    /// Create device driver instance.
    pub fn new(i2c: I2C, addr: u8) -> Self {
        BMP180 {
            i2c,
            addr,
            mode: Mode::UltraHighResolution,
            calib: unsafe { core::mem::zeroed() },
        }
    }

    /// Create a new instance of BMP180 with the default address.
    pub fn new_primary(i2c: I2C) -> Self {
        BMP180::new(i2c, ADDRESS)
    }

    /// Read calibration data from the EEPROM of BMP180.
    pub fn init(&mut self, config: Config) -> Result<(), Error<I2C::Error>> {
        let mut buf = [0u8; 22];
        self.i2c.write_read(self.addr, &[regs::CAL_AC1], &mut buf[..])?;

        self.calib = CalibrationData::from_raw(&buf);

        self.mode = config.mode;

        Ok(())
    }

    /// read uncompensated temperature value
    #[inline]
    fn read_ut<D: DelayNs>(&mut self, delay: &mut D) -> Result<i32, Error<I2C::Error>> {
        self.write(&[regs::CONTROL, cmds::READTEMPCMD])?;
        delay.delay_ms(5);

        let mut buf = [0u8; 2];
        self.i2c.write_read(self.addr, &[regs::TEMPDATA], &mut buf)?;

        Ok(((buf[0] as i32) << 8) + (buf[1] as i32))
    }

    /// read uncompensated pressure value
    #[inline]
    fn read_up<D: DelayNs>(&mut self, delay: &mut D) -> Result<i32, Error<I2C::Error>> {
        let oss = self.mode.oversampling_settings();
        self.write(&[regs::CONTROL, cmds::READPRESSURECMD + (oss << 6)])?;
        self.mode.wait_conversion(delay);

        let mut buf = [0u8; 3];
        self.i2c.write_read(self.addr, &[regs::PRESSUREDATA], &mut buf)?;

        let up = ((buf[0] as i32) << 16) + ((buf[1] as i32) << 8) + (buf[2] as i32) >> (8 - oss);

        Ok(up)
    }

    /// Calculate true temperature, resolution is 0.1C
    pub fn read_temperature<D: DelayNs>(&mut self, delay: &mut D) -> Result<f32, Error<I2C::Error>> {
        let ut = self.read_ut(delay)?;

        let x1 = ((ut - self.calib.ac6 as i32) * self.calib.ac5 as i32) >> 15;
        let x2 = ((self.calib.mc as i32) << 11) / (x1 + self.calib.md as i32);
        let b5 = x1 + x2;
        Ok(((b5 + 8) >> 4) as f32 / 10.0)
    }

    /// Read temperature and pressure at once
    pub fn read_measurement<D: DelayNs>(&mut self, delay: &mut D) -> Result<Measurement, Error<I2C::Error>> {
        let oss = self.mode.oversampling_settings();

        let ut = self.read_ut(delay)?;
        let up = self.read_up(delay)?;

        let x1 = ((ut - self.calib.ac6 as i32) * self.calib.ac5 as i32) >> 15;
        let x2 = ((self.calib.mc as i32) << 11) / (x1 + self.calib.md as i32);
        let b5 = x1 + x2;
        let t = ((b5 + 8) >> 4) as f32 / 10.0;

        let b6 = b5 - 4000;
        let x1 = ((self.calib.b2 as i32) * ((b6 * b6) >> 12)) >> 11;
        let x2 = ((self.calib.ac2 as i32) * b6) >> 11;
        let x3 = x1 + x2;
        // NOTE: must use i64 type
        let b3: i64 = ((((self.calib.ac1 as i64) * 4 + x3 as i64) << oss) + 2) / 4;

        let x1 = ((self.calib.ac3 as i32) * b6) >> 13;
        let x2 = ((self.calib.b1 as i32) * ((b6 * b6) >> 12)) >> 16;
        let x3 = ((x1 + x2) + 2) >> 2;
        let b4: u32 = (self.calib.ac4 as u32) * ((x3 + 32768) as u32) >> 15;

        let b7 = (up as i64 - b3 as i64) * (50000 >> oss);

        let p = if b7 < 0x80000000 {
            (b7 * 2) / (b4 as i64)
        } else {
            (b7 / (b4 as i64)) * 2
        };

        let x1 = (p >> 8) * (p >> 8);
        let x1 = (x1 * 3038) >> 16;
        let x2 = (-7357 * p) >> 16;
        let p = p + ((x1 + x2 + 3791) >> 4);

        Ok(Measurement {
            temperature: t,
            pressure: p as i32,
        })
    }

    /// Calculate true pressure, in Pa
    pub fn read_pressure<D: DelayNs>(&mut self, delay: &mut D) -> Result<i32, Error<I2C::Error>> {
        self.read_measurement(delay).map(|m| m.pressure)
    }

    /// Calculate absolute altitude
    pub fn calculate_altitude<D: DelayNs>(
        &mut self,
        delay: &mut D,
        sealevel_pressure: f32,
    ) -> Result<f32, Error<I2C::Error>> {
        let pa = self.read_pressure(delay)? as f32;
        Ok(44330.0 * (1.0 - (pa / sealevel_pressure).powf(1.0 / 5.255)))
    }

    /// Calculate pressure at sea level
    pub fn calculate_sealevel_pressure<D: DelayNs>(
        &mut self,
        delay: &mut D,
        altitude_m: f32,
    ) -> Result<u32, Error<I2C::Error>> {
        let pressure = self.read_pressure(delay)? as f32;
        let p0 = pressure / (1.0 - altitude_m / 44330.0).powf(5.255);
        Ok(p0 as u32)
    }

    fn write(&mut self, data: &[u8]) -> Result<(), Error<I2C::Error>> {
        self.i2c.write(self.addr, data)?;
        Ok(())
    }
}
