// Local mods
pub mod constants;

// Public imports
use embedded_hal::i2c::I2c;
use byteorder::{LittleEndian, ByteOrder};

// Local imports
use constants::{registers, values, addresses};


/// Errors linked to I2c module.
#[derive(Debug)]
pub enum AtmosphericSensorI2cError{
    IOError
}


/// Modes for the sensor.
pub enum Mode {
    Sleep,
    Forced,
    Normal
}

impl From<u8> for Mode {
    /// Convert modes from u8 to Mode.
    fn from(item: u8) -> Self {
        match item {
            0 => Self::Sleep,
            1 => Self::Forced,
            2 => Self::Forced,
            3 => Self::Normal,
            _ => panic!("Not expected")
        }
    }
}

impl From<Mode> for u8 {
    /// Convert modes from Mode to u8.
    fn from(item: Mode) -> u8 {
        match item {
            Mode::Sleep => 0,
            Mode::Forced => 1,
            Mode::Normal => 3
        }
    }
}


/// Oversampling on the sensor.
pub enum Oversampling {
    Skipped,
    Ox1,  // new freq = freq x 1
    Ox2,  // ...
    Ox4,
    Ox8,
    Ox16
}

impl From<u8> for Oversampling {
    /// Convert from u8 to Oversampling.
    fn from(value: u8) -> Self {
        match value {
            0 => Oversampling::Skipped,
            1 => Oversampling::Ox1,
            2 => Oversampling::Ox2,
            3 => Oversampling::Ox4,
            4 => Oversampling::Ox8,
            _ => Oversampling::Ox16
        }
    }
}

impl From<Oversampling> for u8 {
    /// Convert from Oversampling to u8.
    fn from(value: Oversampling) -> u8 {
        match value {
            Oversampling::Skipped => 0,
            Oversampling::Ox1 => 1,
            Oversampling::Ox2 => 2,
            Oversampling::Ox4 => 3,
            Oversampling::Ox8 => 4,
            Oversampling::Ox16 => 5,
        }
    }
}


/// Stanby time for the sensor.
pub enum StandyTime {
    Ms0_5,
    Ms62_5,
    Ms125,
    Ms250,
    Ms500,
    Ms1000,
    Ms10,
    Ms20
}

impl From<u8> for StandyTime {
    /// Convert from u8 to StandbyTime.
    fn from(value: u8) -> Self {
        match value {
            0 => StandyTime::Ms0_5,
            1 => StandyTime::Ms62_5,
            2 => StandyTime::Ms125,
            3 => StandyTime::Ms250,
            4 => StandyTime::Ms500,
            5 => StandyTime::Ms1000,
            6 => StandyTime::Ms10,
            7 => StandyTime::Ms20,
            _ => panic!("Invalid standby value")
        }
    }
}

impl From<StandyTime> for u8 {
    fn from(value: StandyTime) -> u8 {
        match value {
             StandyTime::Ms0_5 => 0,
             StandyTime::Ms62_5 => 1,
             StandyTime::Ms125 => 2,
             StandyTime::Ms250 => 3,
             StandyTime::Ms500 => 4,
             StandyTime::Ms1000 => 5,
             StandyTime::Ms10 => 6,
             StandyTime::Ms20 => 7
        }
    }
}


/// Filter for sensor.
pub enum Filter {
    Off,
    C2,
    C4,
    C8,
    C16
}

impl From<u8> for Filter {
    /// Convert u8 to Filter. Expects 3 bits only.
    fn from(value: u8) -> Self {
    match value & 0x7 {
            0 => Filter::Off,
            1 => Filter::C2,
            2 => Filter::C4,
            3 => Filter::C8,
            _ => Filter::C16,
        }
    }
}

impl From<Filter> for u8 {
    /// Convert Filter to u8.
    fn from(value: Filter) -> u8 {
        match value {
            Filter::Off => 0,
            Filter::C2 => 1,
            Filter::C4 => 2,
            Filter::C8 => 3,
            Filter::C16 => 4,
        }
    }
}


/// Address options for the sensor.
pub enum Address {
    Default,
    Alternative
}

impl From<Address> for u8 {
    /// Convert from Address to u8.
    fn from(value: Address) -> u8 {
        match value {
            Address::Default => addresses::DEFAULT,
            Address::Alternative => addresses::ALTERNATIVE
        }
    }
}


/// A wrapper for the I2C device and adress to represent the sensor
pub struct AtmosphericSensorI2c<I2C> {
    i2c: I2C,
    address: u8
}

impl<I2C: I2c> AtmosphericSensorI2c<I2C> {
    /// Create new AtmosphericSensorI2c.
    pub fn new(i2c: I2C, address: u8) -> AtmosphericSensorI2c<I2C> {
        AtmosphericSensorI2c { i2c, address }
    }

    /// Read the ID of the chip.
    pub fn get_id(&mut self) -> u8 {
        let mut buffer = [0u8];
        read_from_register(self, registers::CHIP_ID_REG, &mut buffer).unwrap();
        *buffer.first().unwrap()
    }

    /// Reset sensor.
    pub fn reset(&mut self) {
        write_to_register(self, registers::RST_REG, &[values::SOFT_RESET]).unwrap();
    }
    
    /// Get the current mode of the sensor.
    pub fn get_mode(&mut self) -> Mode {
        let mut buffer = [0u8];
        read_from_register(self, registers::CTRL_MEAS_REG, &mut buffer).unwrap();

        // Convert value to Mode
        Mode::from(*buffer.first().unwrap() & 0x03)
    }
    
    /// Set mode to the sensor.
    pub fn set_mode(&mut self, mode: Mode) {
        let mut buffer = [0u8];
        read_from_register(self, registers::CTRL_MEAS_REG, &mut buffer).unwrap();
        let old_state = *buffer.first().unwrap() & 0xFC;
        let new_state = old_state | u8::from(mode);
        write_to_register(self, registers::CTRL_MEAS_REG, &[new_state]).unwrap();
    }

    /// Get measuring bit.
    pub fn is_measuring(&mut self) -> bool {
        // Check bit 3 is set to 1
        ((self.get_status() & 0x04) >> 2) == 1
    }

    /// Get updating bit.
    pub fn is_updating(&mut self) -> bool {
        // Check bit 0 is set to 1
        (self.get_status() & 0x01) == 1
    }

    /// Get status.
    fn get_status(&mut self) -> u8 {
        let mut buffer = [0u8];
        read_from_register(self, registers::STAT_REG, &mut buffer).unwrap();
        *buffer.first().unwrap()
    }

    /// Write oversampling for humidity sampling.
    pub fn set_humidity_oversample(&mut self, rate: Oversampling) {
        let mut buffer = [0u8];
        read_from_register(self, registers::CTRL_HUMIDITY_REG, &mut buffer).unwrap();
    
        let old_state = *buffer.first().unwrap() & 0xF8;
        let new_state = old_state | u8::from(rate);
        write_to_register(self, registers::CTRL_HUMIDITY_REG, &[new_state]).unwrap();
    }
    
    /// Write oversampling for humidity sampling.
    pub fn set_temperature_oversample(&mut self, rate: Oversampling) {
        let mut buffer = [0u8];
        read_from_register(self, registers::CTRL_MEAS_REG, &mut buffer).unwrap();
    
        let old_state = *buffer.first().unwrap() & 0x1F;
        let new_state = old_state | (u8::from(rate) << 5);
        write_to_register(self, registers::CTRL_MEAS_REG, &[new_state]).unwrap();
    }
    
    /// Write oversampling for pressure sampling.
    pub fn set_pressure_oversample(&mut self, rate: Oversampling) {
        let mut buffer = [0u8];
        read_from_register(self, registers::CTRL_MEAS_REG, &mut buffer).unwrap();
        let old_state = *buffer.first().unwrap() & 0xE3;
        let new_state = old_state | (u8::from(rate) << 2);
        write_to_register(self, registers::CTRL_MEAS_REG, &[new_state]).unwrap();
    }
    
    /// Set stamby time to sensor.
    pub fn set_standby_time(&mut self, standby: StandyTime) {
        let mut buffer = [0u8];
        read_from_register(self, registers::CONFIG_REG, &mut buffer).unwrap();
        let old_state = *buffer.first().unwrap() & 0x1F;
        let new_state = old_state | (u8::from(standby) << 5);
        write_to_register(self, registers::CONFIG_REG, &[new_state]).unwrap();
    }
    
    /// Set filter to sensor.
    pub fn set_filter(&mut self, filter: Filter) {
        let mut buffer = [0u8];
        read_from_register(self, registers::CONFIG_REG, &mut buffer).unwrap();
        let old_state = *buffer.first().unwrap() & 0xE3;
        let new_state = old_state | (u8::from(filter) << 2);
        write_to_register(self, registers::CONFIG_REG, &[new_state]).unwrap();
    }
    
    /// Get temperature value from sensor.
    pub fn get_temperature_raw(&mut self) -> u32 {
        let mut buffer = [0u8; 3];
        read_from_register(self, registers::TEMPERATURE_MSB_REG, &mut buffer[0..1]).unwrap();
        read_from_register(self, registers::TEMPERATURE_LSB_REG, &mut buffer[1..2]).unwrap();
        read_from_register(self, registers::TEMPERATURE_XLSB_REG, &mut buffer[2..3]).unwrap();
    
        (u32::from(buffer[0]) << 12) | (u32::from(buffer[1]) << 4) | ((u32::from(buffer[2]) >> 4) & 0x0F)
    }
    
    /// Get pressure value from sensor.
    pub fn get_pressure_raw(&mut self) -> u32 {
        let mut buffer = [0u8; 3];
        read_from_register(self, registers::PRESSURE_MSB_REG, &mut buffer[0..1]).unwrap();
        read_from_register(self, registers::PRESSURE_LSB_REG, &mut buffer[1..2]).unwrap();
        read_from_register(self, registers::PRESSURE_XLSB_REG, &mut buffer[2..3]).unwrap();
    
        (u32::from(buffer[0]) << 12) | (u32::from(buffer[1]) << 4) | ((u32::from(buffer[2]) >> 4) & 0x0F)
    }
    
    /// Get humidity value from sensor.
    pub fn get_humidity_raw(&mut self) -> u32 {
        let mut buffer = [0u8; 2];
        read_from_register(self, registers::HUMIDITY_MSB_REG, &mut buffer[0..1]).unwrap();
        read_from_register(self, registers::HUMIDITY_LSB_REG, &mut buffer[1..2]).unwrap();
    
        (u32::from(buffer[0]) << 8) | (u32::from(buffer[1]))
    }

    /// Get T1 value for temperature calibration.
    pub fn get_t1(&mut self) -> u16 {
        let buffer = read_multiple_registers(self, &[
            registers::DIG_T1_LSB_REG,
            registers::DIG_T1_MSB_REG
        ]).unwrap();
        LittleEndian::read_u16(&buffer)
    }

    /// Get T2 value for temperature calibration.
    pub fn get_t2(&mut self) -> i16 {
        let buffer = read_multiple_registers(self, &[
            registers::DIG_T2_LSB_REG,
            registers::DIG_T2_MSB_REG
        ]).unwrap();
        LittleEndian::read_i16(&buffer)
    }

    /// Get T3 value for temperature calibration.
    pub fn get_t3(&mut self) -> i16 {
        let buffer = read_multiple_registers(self, &[
            registers::DIG_T3_LSB_REG,
            registers::DIG_T3_MSB_REG
        ]).unwrap();
        LittleEndian::read_i16(&buffer)
    }

    /// Get P1 value for pressure calibration.
    pub fn get_p1(&mut self) -> u16 {
        let buffer = read_multiple_registers(self, &[
            registers::DIG_P1_LSB_REG,
            registers::DIG_P1_MSB_REG
        ]).unwrap();
        LittleEndian::read_u16(&buffer)
    }

    /// Get P2 value for pressure calibration.
    pub fn get_p2(&mut self) -> i16 {
        let buffer = read_multiple_registers(self, &[
            registers::DIG_P2_LSB_REG,
            registers::DIG_P2_MSB_REG
        ]).unwrap();
        LittleEndian::read_i16(&buffer)
    }

    /// Get P3 value for pressure calibration.
    pub fn get_p3(&mut self) -> i16 {
        let buffer = read_multiple_registers(self, &[
            registers::DIG_P3_LSB_REG,
            registers::DIG_P3_MSB_REG
        ]).unwrap();
        LittleEndian::read_i16(&buffer)
    }

    /// Get P4 value for pressure calibration.
    pub fn get_p4(&mut self) -> i16 {
        let buffer = read_multiple_registers(self, &[
            registers::DIG_P4_LSB_REG,
            registers::DIG_P4_MSB_REG
        ]).unwrap();
        LittleEndian::read_i16(&buffer)
    }

    /// Get P5 value for pressure calibration.
    pub fn get_p5(&mut self) -> i16 {
        let buffer = read_multiple_registers(self, &[
            registers::DIG_P5_LSB_REG,
            registers::DIG_P5_MSB_REG
        ]).unwrap();
        LittleEndian::read_i16(&buffer)
    }

    /// Get P6 value for pressure calibration.
    pub fn get_p6(&mut self) -> i16 {
        let buffer = read_multiple_registers(self, &[
            registers::DIG_P6_LSB_REG,
            registers::DIG_P6_MSB_REG
        ]).unwrap();
        LittleEndian::read_i16(&buffer)
    }

    /// Get P7 value for pressure calibration.
    pub fn get_p7(&mut self) -> i16 {
        let buffer = read_multiple_registers(self, &[
            registers::DIG_P7_LSB_REG,
            registers::DIG_P7_MSB_REG
        ]).unwrap();
        LittleEndian::read_i16(&buffer)
    }

    /// Get P8 value for pressure calibration.
    pub fn get_p8(&mut self) -> i16 {
        let buffer = read_multiple_registers(self, &[
            registers::DIG_P8_LSB_REG,
            registers::DIG_P8_MSB_REG
        ]).unwrap();
        LittleEndian::read_i16(&buffer)
    }

    /// Get P9 value for pressure calibration.
    pub fn get_p9(&mut self) -> i16 {
        let buffer = read_multiple_registers(self, &[
            registers::DIG_P9_LSB_REG,
            registers::DIG_P9_MSB_REG
        ]).unwrap();
        LittleEndian::read_i16(&buffer)
    }

    /// Get H1 value for humidity calibration.
    pub fn get_h1(&mut self) -> u8 {
        let mut buffer = read_multiple_registers(self, &[registers::DIG_H1_REG]).unwrap();
        buffer.pop().unwrap()
    }

    /// Get H2 value for humidity calibration.
    pub fn get_h2(&mut self) -> i16 {
        let buffer: Vec<u8> = read_multiple_registers(self, &[
            registers::DIG_H2_LSB_REG,
            registers::DIG_H2_MSB_REG,
        ]).unwrap();
        LittleEndian::read_i16(&buffer)
    }

    /// Get H3 value for humidity calibration.
    pub fn get_h3(&mut self) -> u8 {
        let mut buffer = read_multiple_registers(self, &[registers::DIG_H3_REG]).unwrap();
        buffer.pop().unwrap()
    }

    /// Get H4 value for humidity calibration.
    pub fn get_h4(&mut self) -> i16 {
        let mut buffer  = [0u8; 2];
        read_from_register(self, registers::DIG_H4_MSB_REG, &mut buffer[0..1]).unwrap();
        read_from_register(self, registers::DIG_H4_LSB_REG, &mut buffer[1..2]).unwrap();

        ((u16::from(buffer[0]) << 4) | (u16::from(buffer[1]) & 0x0F)) as i16
    }

    /// Get H5 value for humidity calibration.
    pub fn get_h5(&mut self) -> i16 {
        let mut buffer  = [0u8; 2];
        read_from_register(self, registers::DIG_H5_MSB_REG, &mut buffer[0..1]).unwrap();
        read_from_register(self, registers::DIG_H4_LSB_REG, &mut buffer[1..2]).unwrap();

        (((u16::from(buffer[0]) << 4)) | ((u16::from(buffer[1]) >> 4) & 0x0F)) as i16
    }

    /// Get H6 value for humidity calibration.
    pub fn get_h6(&mut self) -> i8 {
        let mut buffer  = [0u8; 1];
        read_from_register(self, registers::DIG_H6_REG, &mut buffer).unwrap();

        buffer[0] as i8
    }

}


/// Get value from a specific register in sensor.
pub fn read_from_register<I2C: I2c>(dev: &mut AtmosphericSensorI2c<I2C> , register: u8, buffer: &mut [u8]) -> Result<(), AtmosphericSensorI2cError> {
    match dev.i2c.write_read(dev.address, &[register], buffer) {
        Ok(_) => Ok(()),
        Err(_) => Err(AtmosphericSensorI2cError::IOError)
    }
}

/// Set value from a specific register in sensor.
pub fn write_to_register<I2C: I2c>(dev: &mut AtmosphericSensorI2c<I2C>, register: u8, bytes: &[u8]) -> Result<(), AtmosphericSensorI2cError> {
    let mut buffer = Vec::<u8>::with_capacity(1+bytes.len());
    buffer.push(register);
    for value in bytes {
        buffer.push(*value);
    }
    // TODO check if it matches write_bytes
    match dev.i2c.write(dev.address, &buffer) {
        Ok(_) => Ok(()),
        Err(_) => Err(AtmosphericSensorI2cError::IOError)
    }
}

/// Helper function to read multiple registers at once and store value on Vec.
fn read_multiple_registers<I2C: I2c>(dev: &mut AtmosphericSensorI2c<I2C>, registers: &[u8]) -> Result<Vec<u8>, AtmosphericSensorI2cError> {
    let mut buffer: Vec<u8> = vec![];
    for register in registers.iter() {
        let mut temp_buffer  = [0u8];
        match read_from_register(dev, *register, &mut temp_buffer) {
            Ok(_) => buffer.extend(temp_buffer),
            Err(_) => {return Err(AtmosphericSensorI2cError::IOError)}
        }
    }
    Ok(buffer)
}
