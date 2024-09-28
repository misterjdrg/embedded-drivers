//! Driver for L3G4200D.

#![no_std]

pub const ADDRESS: u8 = 0x69;

pub mod regs {
    pub const WHO_AM_I: u8 = 0x0F;

    pub const CTRL_REG1: u8 = 0x20;
    pub const CTRL_REG2: u8 = 0x21;
    pub const CTRL_REG3: u8 = 0x22;
    pub const CTRL_REG4: u8 = 0x23;
    pub const CTRL_REG5: u8 = 0x24;
    pub const REFERENCE: u8 = 0x25;
    pub const OUT_TEMP: u8 = 0x26;
    pub const STATUS_REG: u8 = 0x27;

    pub const OUT_X_L: u8 = 0x28;
    pub const OUT_X_H: u8 = 0x29;
    pub const OUT_Y_L: u8 = 0x2A;
    pub const OUT_Y_H: u8 = 0x2B;
    pub const OUT_Z_L: u8 = 0x2C;
    pub const OUT_Z_H: u8 = 0x2D;

    pub const FIFO_CTRL_REG: u8 = 0x2E;
    pub const FIFO_SRC_REG: u8 = 0x2F;

    pub const INT1_CFG: u8 = 0x30;
    pub const INT1_SRC: u8 = 0x31;
    pub const INT1_THS_XH: u8 = 0x32;
    pub const INT1_THS_XL: u8 = 0x33;
    pub const INT1_THS_YH: u8 = 0x34;
    pub const INT1_THS_YL: u8 = 0x35;
    pub const INT1_THS_ZH: u8 = 0x36;
    pub const INT1_THS_ZL: u8 = 0x37;
    pub const INT1_DURATION: u8 = 0x38;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Scale {
    Dps2000 = 0b10,
    Dps500 = 0b01,
    Dps250 = 0b00,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum DataRate {
    Hz800Bw110 = 0b1111,
    Hz800Bw50 = 0b1110,
    Hz800Bw35 = 0b1101,
    Hz800Bw30 = 0b1100,
    Hz400Bw110 = 0b1011,
    Hz400Bw50 = 0b1010,
    Hz400Bw25 = 0b1001,
    Hz400Bw20 = 0b1000,
    Hz200Bw70 = 0b0111,
    Hz200Bw50 = 0b0110,
    Hz200Bw25 = 0b0101,
    Hz200Bw12_5 = 0b0100,
    Hz100Bw25 = 0b0001,
    Hz100Bw12_5 = 0b0000,
}

#[derive(Debug)]
pub enum Error<IE> {
    Bus(IE),
    InvalidDevice,
}

impl<E> From<E> for Error<E> {
    fn from(e: E) -> Self {
        Error::Bus(e)
    }
}

pub struct Config {
    pub scale: Scale,
    pub data_rate: DataRate,
}

impl Default for Config {
    fn default() -> Self {
        Config {
            scale: Scale::Dps2000,
            data_rate: DataRate::Hz400Bw50,
        }
    }
}

pub struct L3G4200D<I2C> {
    i2c: I2C,
    addr: u8,
    dps_per_digit: f32,
}

impl<I2C> L3G4200D<I2C>
where
    I2C: embedded_hal::i2c::I2c,
{
    pub fn new(i2c: I2C, addr: u8) -> Self {
        Self {
            i2c,
            addr,
            dps_per_digit: 0.00875,
        }
    }

    pub fn new_primary(i2c: I2C) -> Self {
        Self::new(i2c, ADDRESS)
    }

    pub fn init(&mut self, config: Config) -> Result<(), Error<I2C::Error>> {
        if self.read_reg(regs::WHO_AM_I)? != 0xD3 {
            return Err(Error::InvalidDevice);
        }

        // Enable all axis and setup normal mode + Output Data Range & Bandwidth
        let mut reg1 = 0x0F; // Enable all axis and setup normal mode
        reg1 |= (config.data_rate as u8) << 4; // Set output data rate & bandwidth
        self.write_reg(regs::CTRL_REG1, reg1)?;

        // Disable high pass filter
        self.write_reg(regs::CTRL_REG2, 0x00)?;

        // Generate data ready interrupt on INT2
        self.write_reg(regs::CTRL_REG3, 0x08)?;

        // Set full scale selection in continuous mode
        self.write_reg(regs::CTRL_REG4, (config.scale as u8) << 4)?;

        // Set dpsPerDigit based on scale
        self.dps_per_digit = match config.scale {
            Scale::Dps250 => 0.00875,
            Scale::Dps500 => 0.0175,
            Scale::Dps2000 => 0.07,
        };

        // Boot in normal mode, disable FIFO, HPF disabled
        self.write_reg(regs::CTRL_REG5, 0x00)?;

        Ok(())
    }

    pub fn read_raw(&mut self) -> Result<(i16, i16, i16), Error<I2C::Error>> {
        let mut buf = [0u8; 6];

        // Read 6 bytes starting from OUT_X_L register (0x28 | 0x80 for auto-increment)
        self.i2c.write_read(self.addr, &[regs::OUT_X_L | 0x80], &mut buf)?;

        // Combine high and low bytes into 16-bit integers
        let x = i16::from_le_bytes([buf[0], buf[1]]);
        let y = i16::from_le_bytes([buf[2], buf[3]]);
        let z = i16::from_le_bytes([buf[4], buf[5]]);

        Ok((x, y, z))
    }

    pub fn read_normalized(&mut self) -> Result<(i16, i16, i16), Error<I2C::Error>> {
        let (x, y, z) = self.read_raw()?;

        // Apply normalization using dps_per_digit
        let x_norm = (x as f32 * self.dps_per_digit) as i16;
        let y_norm = (y as f32 * self.dps_per_digit) as i16;
        let z_norm = (z as f32 * self.dps_per_digit) as i16;

        Ok((x_norm, y_norm, z_norm))
    }

    pub fn read_reg(&mut self, reg: u8) -> Result<u8, I2C::Error> {
        let mut buf = [0];
        self.i2c.write_read(self.addr, &[reg], &mut buf)?;
        Ok(buf[0])
    }

    // Add this new method to write to registers
    pub fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), I2C::Error> {
        self.i2c.write(self.addr, &[reg, value])
    }
}
