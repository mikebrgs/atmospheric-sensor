/// File with calibration related procedures.

// Mods

// Public imports
use embedded_hal::i2c::I2c;

// Local imports
use crate::i2c::AtmosphericSensorI2c;

pub struct Calibration {
    pub temperature: TemperatureCalibration,
    pub pressure: PressureCalibration,
    pub humidity: HumidityCalibration,
}

impl Calibration {
    pub fn new(temperature: TemperatureCalibration, pressure: PressureCalibration, humidity: HumidityCalibration) -> Calibration {
        Calibration{
            temperature,
            pressure,
            humidity
        }
    }

    pub fn build<I2C: I2c>(dev: &mut AtmosphericSensorI2c<I2C>) -> Calibration {
        Self::new(
            TemperatureCalibration::build(dev),
            PressureCalibration::build(dev),
            HumidityCalibration::build(dev)
        )
    }
}


pub struct TemperatureCalibration {
    t1: u16,
    t2: i16,
    t3: i16
}

impl TemperatureCalibration {
    fn new(t1: u16, t2: i16, t3: i16) -> TemperatureCalibration {
        TemperatureCalibration{t1,t2,t3}
    }

    fn build<I2C: I2c>(dev: &mut AtmosphericSensorI2c<I2C>) -> TemperatureCalibration {
        Self::new(
            dev.get_t1(),
            dev.get_t2(),
            dev.get_t3()
        )
    }

    pub fn compensate_temperature(self: &Self, adc_t: i32) -> i32 {
        let var1 = ((((adc_t>>3) - (i32::from(self.t1)<<1))) * (i32::from(self.t2))) >> 11;
        let var2 = (((((adc_t>>4) - i32::from(self.t1)) * ((adc_t>>4) - i32::from(self.t1))) >> 12) * i32::from(self.t3)) >> 14;
        var1 + var2
    }
}

pub struct PressureCalibration {
    p1: u16,
    p2: i16,
    p3: i16,
    p4: i16,
    p5: i16,
    p6: i16,
    p7: i16,
    p8: i16,
    p9: i16
}

impl PressureCalibration {
    fn new(p1: u16, p2: i16, p3: i16, p4: i16, p5: i16, p6: i16, p7: i16, p8: i16, p9: i16) -> PressureCalibration {
        PressureCalibration{p1,p2,p3,p4,p5,p6,p7,p8,p9}
    }

    fn build<I2C: I2c>(dev: &mut AtmosphericSensorI2c<I2C>) -> PressureCalibration {
        Self::new(
            dev.get_p1(),
            dev.get_p2(),
            dev.get_p3(),
            dev.get_p4(),
            dev.get_p5(),
            dev.get_p6(),
            dev.get_p7(),
            dev.get_p8(),
            dev.get_p9()
        )
    }

    pub fn compensate_pressure(self: &Self, adc_p: i32, t_fine: i32) -> u32 {
        let var1 = i64::from(t_fine) - 128000;
        let var2 = var1 * var1 * i64::from(self.p6);
        let var2 = var2 + ((var1 * i64::from(self.p5)) << 17);
        let var2 = var2 + (i64::from(self.p4) << 35);
        let var1 = ((var1 * var1 * i64::from(self.p3)) >> 8) + ((var1 * i64::from(self.p2)) << 12);
        let var1 = ((1_i64 << 47) + var1) * i64::from(self.p1) >> 33;

        if var1 == 0 {
            0
        } else {
            let p = 1_048_576 - i64::from(adc_p);
            let p = (((p << 31) - var2) * 3125) / var1; // Fix here
            let var1 = ((i64::from(self.p9)) * (p >> 13) * (p >> 13)) >> 25;
            let var2 = ((i64::from(self.p8)) * p) >> 19;
            let p = ((p + var1 + var2) >> 8) + ((i64::from(self.p7)) << 4);
    
            let output = p as u32;
            output    
        }
    }
}

pub struct HumidityCalibration {
    h1: u8,
    h2: i16,
    h3: u8,
    h4: i16,
    h5: i16,
    h6: i8
}

impl HumidityCalibration {
    pub fn new(h1: u8, h2: i16, h3: u8, h4: i16, h5: i16, h6: i8) -> HumidityCalibration {
        HumidityCalibration{h1,h2,h3,h4,h5,h6}
    }

    pub fn build<I2C: I2c>(dev: &mut AtmosphericSensorI2c<I2C>) -> HumidityCalibration {
        Self::new(
            dev.get_h1(),
            dev.get_h2(),
            dev.get_h3(),
            dev.get_h4(),
            dev.get_h5(),
            dev.get_h6()
        )
    }

    pub fn compensate_humidity(self: &Self, adc_h: i32, t_fine: i32) -> u32 {
        let mut var1 = t_fine - 76800i32;
        var1 = ((((adc_h << 14) - (i32::from(self.h4) << 20) - (i32::from(self.h5) * var1)) +
            (16384)) >> 15) * (((((((var1 * i32::from(self.h6)) >> 10) * (((var1 * i32::from(self.h3)) >> 11) + (32768))) >> 10) + (2097152)) *
            i32::from(self.h2) + 8192) >> 14);
        var1 = var1 - (((((var1 >> 15) * (var1 >> 15)) >> 7) * i32::from(self.h1)) >> 4);
        if var1 < 0 {
            var1 = 0;
        } else if var1 > 419430400 {
            var1 = 419430400;
        }

        (var1 >> 12) as u32
    }
}


#[cfg(test)]
mod tests {
    use byteorder::{BigEndian, ByteOrder};

    use super::*;

    fn create_temperature_calibration() -> TemperatureCalibration {
        TemperatureCalibration::new(
            28485_i64 as u16,
            26735_i64 as i16,
            50_i64 as i16
        )
    }

    #[test]
    fn temperature_calibration() {
        let t_cal = create_temperature_calibration();
        let t_buffer = BigEndian::read_u32(&[0,128,189,0]) >> 4;
        let t_fine = t_cal.compensate_temperature(t_buffer as i32);
        assert_eq!(t_fine, 116770);
    }

    fn create_pressure_calibration() -> PressureCalibration {
        PressureCalibration::new(
            36738_i64 as u16,
            -10635_i64 as i16,
            3024_i64 as i16,
            6980_i64 as i16,
            -4_i64 as i16,
            -7_i64 as i16,
            9900_i64 as i16,
            -10230_i64 as i16,
            4285_i64 as i16
        )
    }

    #[test]
    fn pressure_calibration() {
        let p_cal = create_pressure_calibration();
        let p_buffer = BigEndian::read_u32(&[0,82,79,0]) >> 4;
        let p_comp = p_cal.compensate_pressure(p_buffer as i32, 120035);
        assert!(p_comp == 26036801);
    }

    fn create_humidity_calibration() -> HumidityCalibration {
        HumidityCalibration::new(
            75_i64 as u8,
            365_i64 as i16,
            0_i64 as u8,
            312_i64 as i16,
            50_i64 as i16,
            30_i64 as i8
        )
    }

    #[test]
    fn humidity_calibration_test() {
        let h_cal = create_humidity_calibration();
        let h_buffer = BigEndian::read_u16(&[117, 97]);
        let h_comp = h_cal.compensate_humidity(h_buffer as i32, 116770);
        assert_eq!(h_comp, 57350)
    }

}