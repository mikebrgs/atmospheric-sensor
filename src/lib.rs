// Local modules
mod calibration;
mod i2c;

// Public imports
use embedded_hal::i2c::I2c;

// Local imports
use calibration::Calibration;
use i2c::AtmosphericSensorI2c;
pub use i2c::Address;


/// Atmospheric sensor
pub struct AtmosphericSensor<I2C> {
    dev: AtmosphericSensorI2c<I2C>,
    calibration: Calibration,
    t_fine: i32,
}

impl<I2C: I2c> AtmosphericSensor<I2C> {
    /// Create new AtmosphericSensor device wrapper for I2C communication.
    pub fn new(dev: I2C, address: Address) -> AtmosphericSensor<I2C> {
        let mut wrapper = AtmosphericSensorI2c::new(dev, address.into());
        let calibration = calibration::Calibration::build(&mut wrapper);
        AtmosphericSensor { dev: wrapper, calibration: calibration, t_fine: 0 }
    }

    /// Create new AtmosphericSensor and start it.
    pub fn build(dev: I2C, address: Address) -> AtmosphericSensor<I2C> {
        let mut sensor = AtmosphericSensor::new(dev, address.into());
        sensor.start().unwrap();
        sensor
    }

    /// Start all parameters from for the sensor
    pub fn start(&mut self) -> Result<(), String> {
        self.dev.set_standby_time(i2c::StandyTime::Ms0_5);
        self.dev.set_filter(i2c::Filter::Off);
        self.dev.set_temperature_oversample(i2c::Oversampling::Ox1);
        self.dev.set_pressure_oversample(i2c::Oversampling::Ox1);
        self.dev.set_humidity_oversample(i2c::Oversampling::Ox1);
        self.dev.set_mode(i2c::Mode::Normal);
        Ok(())
    }

    /// Stop the sensor.
    pub fn stop(&mut self) -> Result<(), String> {
        self.dev.set_mode(i2c::Mode::Sleep);
        Ok(())
    }

    /// Reset device.
    pub fn reset(&mut self) -> Result<(), String> {
        self.dev.reset();
        Ok(())
    }

    /// Is the device measuring.
   pub fn is_measuring(&mut self) -> Result<bool, String> {
        Ok(self.dev.is_measuring())
    }

    /// Is the device copying NVM data to image registers.
    pub fn is_updating(&mut self) -> Result<bool, String> {
        Ok(self.dev.is_updating())
    }

    /// Get temperature in celsius from sensor.
    pub fn get_temperature_celsius(&mut self) -> Result<f64, String> {
        let adc_t = self.dev.get_temperature_raw();
        self.t_fine = self.calibration.temperature.compensate_temperature(adc_t as i32);
        let output = (self.t_fine * 5 + 128) >> 8;
        Ok(f64::from(output) / 100.0)
    }

    /// Get pressure in pascal from sensor.
    pub fn get_pressure_pascal(&mut self) -> Result<f64, String> {
        let adc_p = self.dev.get_pressure_raw();
        let pressure = self.calibration.pressure.compensate_pressure(adc_p as i32, self.t_fine);
        Ok(f64::from(pressure) / 256.0)
    }

    pub fn get_humidity_relative(&mut self) -> Result<f64, String> {
        let adc_h = self.dev.get_humidity_raw();
        let humidity = self.calibration.humidity.compensate_humidity(adc_h as i32, self.t_fine);

        Ok(f64::from(humidity) / 1024.0)
    }

}


#[cfg(test)]
mod tests {
    use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTransaction};

    use super::{i2c::Address, AtmosphericSensor, i2c::constants::registers};

    #[test]
    fn read_humidity() {
        let address: u8 = Address::Default.into();
        let mut expectations = get_mock_calibration(address);
        expectations.push(
            I2cTransaction::write_read(address, vec![0xFD], vec![110]),    
        );
        expectations.push(
            I2cTransaction::write_read(address, vec![0xFE], vec![213]),
        );

        let i2c = I2cMock::new(&expectations);
        let mut i2c_clone = i2c.clone();

        let mut sensor = AtmosphericSensor::new(i2c, Address::Default); // = AtmosphericSensor::build(i2c, addresses::DEFAULT);
        // sensor.t_fine = 0;
        let humidity = sensor.get_humidity_relative().unwrap();
        
        assert!(humidity - 46.159 < 0.1);

        // Stop i2c
        i2c_clone.done();
        
    }

    #[test]
    fn read_temperature() {
        let address: u8 = Address::Default.into();
        let mut expectations = get_mock_calibration(address);
        expectations.push(
            I2cTransaction::write_read(address, vec![registers::TEMPERATURE_MSB_REG], vec![0])
        );
        expectations.push(
            I2cTransaction::write_read(address, vec![registers::TEMPERATURE_LSB_REG], vec![0])
        );
        expectations.push(
            I2cTransaction::write_read(address, vec![registers::TEMPERATURE_XLSB_REG], vec![0])
        );
        
        let i2c = I2cMock::new(&expectations);
        let mut i2c_clone = i2c.clone();

        let mut sensor = AtmosphericSensor::new(i2c, Address::Default);
        sensor.t_fine = 0;
        let temperature = sensor.get_temperature_celsius().unwrap();

        assert!(temperature > -100.);
        assert!(temperature < 100.);

        i2c_clone.done();
    }

    #[test]
    fn read_pressure() {
        let address: u8 = Address::Default.into();
        let mut expectations = get_mock_calibration(address);
        expectations.push(
            I2cTransaction::write_read(address, vec![registers::PRESSURE_MSB_REG], vec![0])
        );
        expectations.push(
            I2cTransaction::write_read(address, vec![registers::PRESSURE_LSB_REG], vec![0])
        );
        expectations.push(
            I2cTransaction::write_read(address, vec![registers::PRESSURE_XLSB_REG], vec![0])
        );
        
        let i2c = I2cMock::new(&expectations);
        let mut i2c_clone = i2c.clone();

        let mut sensor = AtmosphericSensor::new(i2c, Address::Default);
        sensor.t_fine = 0;
        let pressure = sensor.get_pressure_pascal().unwrap();

        assert!(pressure > 0.0);

        i2c_clone.done();
    }

    fn get_mock_calibration(address: u8) -> Vec<I2cTransaction> {
        let expectations = vec![
            I2cTransaction::write_read(address, vec![registers::DIG_T1_LSB_REG], ((28485_i64 & 0xFF) as u8).to_be_bytes().to_vec()),
            I2cTransaction::write_read(address, vec![registers::DIG_T1_MSB_REG], ((28485_i64 & 0xFF00 >> 8) as u8).to_be_bytes().to_vec()),
            // T2 calibration
            I2cTransaction::write_read(address, vec![registers::DIG_T2_LSB_REG], ((26735_i64 & 0xFF) as u8).to_be_bytes().to_vec()),
            I2cTransaction::write_read(address, vec![registers::DIG_T2_MSB_REG], ((26735_i64 & 0xFF00 >> 8) as u8).to_be_bytes().to_vec()),
            // T3 calibration
            I2cTransaction::write_read(address, vec![registers::DIG_T3_LSB_REG], ((50_i64 & 0xFF) as u8).to_be_bytes().to_vec()),
            I2cTransaction::write_read(address, vec![registers::DIG_T3_MSB_REG], ((50_i64 & 0xFF00 >> 8) as u8).to_be_bytes().to_vec()),

            // Pressure calibration
            // P1 calibration
            I2cTransaction::write_read(address, vec![0x8E], ((36738_i64 & 0xFF) as u8).to_be_bytes().to_vec()),
            I2cTransaction::write_read(address, vec![0x8F], ((36738_i64 & 0xFF00 >> 8) as u8).to_be_bytes().to_vec()),
            // P2 calibration
            I2cTransaction::write_read(address, vec![0x90], ((-10635_i64 & 0xFF) as u8).to_be_bytes().to_vec()),
            I2cTransaction::write_read(address, vec![0x91], ((-10635_i64 & 0xFF00 >> 8) as u8).to_be_bytes().to_vec()),
            // P3 calibration
            I2cTransaction::write_read(address, vec![0x92], ((3024_i64 & 0xFF) as u8).to_be_bytes().to_vec()),
            I2cTransaction::write_read(address, vec![0x93], ((3024_i64 & 0xFF00 >> 8) as u8).to_be_bytes().to_vec()),
            // P4 calibration
            I2cTransaction::write_read(address, vec![0x94], ((6980_i64 & 0xFF) as u8).to_be_bytes().to_vec()),
            I2cTransaction::write_read(address, vec![0x95], ((6980_i64 & 0xFF00 >> 8) as u8).to_be_bytes().to_vec()),
            // P5 calibration
            I2cTransaction::write_read(address, vec![0x96], ((-4_i64 & 0xFF) as u8).to_be_bytes().to_vec()),
            I2cTransaction::write_read(address, vec![0x97], ((-4_i64 & 0xFF00 >> 8) as u8).to_be_bytes().to_vec()),
            // P6 calibration
            I2cTransaction::write_read(address, vec![0x98], ((-7_i64 & 0xFF) as u8).to_be_bytes().to_vec()),
            I2cTransaction::write_read(address, vec![0x99], ((-7_i64 & 0xFF00 >> 8) as u8).to_be_bytes().to_vec()),
            // P7 calibration
            I2cTransaction::write_read(address, vec![0x9A], ((9900_i64 & 0xFF) as u8).to_be_bytes().to_vec()),
            I2cTransaction::write_read(address, vec![0x9B], ((9900_i64 & 0xFF00 >> 8) as u8).to_be_bytes().to_vec()),
            // P8 calibration
            I2cTransaction::write_read(address, vec![0x9C], ((-10230_i64 & 0xFF) as u8).to_be_bytes().to_vec()),
            I2cTransaction::write_read(address, vec![0x9D], ((-10230_i64 & 0xFF00 >> 8) as u8).to_be_bytes().to_vec()),
            // P9 calibration
            I2cTransaction::write_read(address, vec![0x9E], ((4285_i64 & 0xFF) as u8).to_be_bytes().to_vec()),
            I2cTransaction::write_read(address, vec![0x9F], ((4285_i64 & 0xFF00 >> 8) as u8).to_be_bytes().to_vec()),

            // TODO check all calibration values from python for sample case
            // Humidity calibration
            // H1 calibration
            I2cTransaction::write_read(address, vec![registers::DIG_H1_REG], ((75_i64 & 0xFF) as u8).to_be_bytes().to_vec()),
            // H2 calibration
            I2cTransaction::write_read(address, vec![registers::DIG_H2_LSB_REG], ((109 & 0xFF) as u8).to_be_bytes().to_vec()),
            I2cTransaction::write_read(address, vec![registers::DIG_H2_MSB_REG], ((1 & 0xFF) as u8).to_be_bytes().to_vec()),
            // H3 calibration
            I2cTransaction::write_read(address, vec![registers::DIG_H3_REG], ((0 & 0xFF) as u8).to_be_bytes().to_vec()),
            // H4 calibration
            I2cTransaction::write_read(address, vec![registers::DIG_H4_MSB_REG], ((19 & 0xFF) as u8).to_be_bytes().to_vec()),
            I2cTransaction::write_read(address, vec![registers::DIG_H4_LSB_REG], ((40 & 0xFF) as u8).to_be_bytes().to_vec()),
            // H5 calibration
            I2cTransaction::write_read(address, vec![registers::DIG_H5_MSB_REG], ((3 & 0xFF) as u8).to_be_bytes().to_vec()),
            I2cTransaction::write_read(address, vec![registers::DIG_H4_LSB_REG], ((40 & 0xFF) as u8).to_be_bytes().to_vec()),
            // H6 calibration
            I2cTransaction::write_read(address, vec![registers::DIG_H6_REG], ((30 & 0xFF) as u8).to_be_bytes().to_vec()),
        ];
        return expectations
    }
}