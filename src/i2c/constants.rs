pub mod values {
    pub const SOFT_RESET: u8 = 0xB6;
    pub const CHIP_ID: u8 = 0x60;
}

pub mod registers {
    pub const DIG_T1_LSB_REG: u8 = 0x88;
    pub const DIG_T1_MSB_REG: u8 = 0x89;
    pub const DIG_T2_LSB_REG: u8 = 0x8A;
    pub const DIG_T2_MSB_REG: u8 = 0x8B;
    pub const DIG_T3_LSB_REG: u8 = 0x8C;
    pub const DIG_T3_MSB_REG: u8 = 0x8D;

    pub const DIG_P1_LSB_REG: u8 = 0x8E;
    pub const DIG_P1_MSB_REG: u8 = 0x8F;
    pub const DIG_P2_LSB_REG: u8 = 0x90;
    pub const DIG_P2_MSB_REG: u8 = 0x91;
    pub const DIG_P3_LSB_REG: u8 = 0x92;
    pub const DIG_P3_MSB_REG: u8 = 0x93;
    pub const DIG_P4_LSB_REG: u8 = 0x94;
    pub const DIG_P4_MSB_REG: u8 = 0x95;
    pub const DIG_P5_LSB_REG: u8 = 0x96;
    pub const DIG_P5_MSB_REG: u8 = 0x97;
    pub const DIG_P6_LSB_REG: u8 = 0x98;
    pub const DIG_P6_MSB_REG: u8 = 0x99;
    pub const DIG_P7_LSB_REG: u8 = 0x9A;
    pub const DIG_P7_MSB_REG: u8 = 0x9B;
    pub const DIG_P8_LSB_REG: u8 = 0x9C;
    pub const DIG_P8_MSB_REG: u8 = 0x9D;
    pub const DIG_P9_LSB_REG: u8 = 0x9E;
    pub const DIG_P9_MSB_REG: u8 = 0x9F;

    pub const DIG_H1_REG: u8 = 0xA1;
    pub const DIG_H2_LSB_REG: u8 = 0xE1;
    pub const DIG_H2_MSB_REG: u8 = 0xE2;
    pub const DIG_H3_REG: u8 = 0xE3;
    pub const DIG_H4_MSB_REG: u8 = 0xE4;
    pub const DIG_H4_LSB_REG: u8 = 0xE5;
    pub const DIG_H5_MSB_REG: u8 = 0xE6;
    pub const DIG_H6_REG: u8 = 0xE7;

    pub const TEMPERATURE_MSB_REG: u8 = 0xFA;  // Temperature MSB
    pub const TEMPERATURE_LSB_REG: u8 = 0xFB;  // Temperature LSB
    pub const TEMPERATURE_XLSB_REG: u8 = 0xFC;  // Temperature XLSB

    pub const PRESSURE_MSB_REG: u8 = 0xF7;  // Pressure MSB
    pub const PRESSURE_LSB_REG: u8 = 0xF8;  // Pressure LSB
    pub const PRESSURE_XLSB_REG: u8 = 0xF9;  // Pressure XLSB

    pub const HUMIDITY_MSB_REG: u8 = 0xFD;  // Humidity MSB
    pub const HUMIDITY_LSB_REG: u8 = 0xFE;  // Humidity LSB

    pub const CTRL_HUMIDITY_REG: u8 = 0xF2;  // Ctrl Humidity Reg
    pub const STAT_REG: u8 = 0xF3;  // Status Reg
    pub const CTRL_MEAS_REG: u8 = 0xF4;  // Ctrl Measure Reg
    pub const CONFIG_REG: u8 = 0xF5;  // Configuration Reg
    pub const CHIP_ID_REG: u8 = 0xD0;  // Chip ID
    pub const RST_REG: u8 = 0xE0;  // Softreset Reg

}

pub mod addresses {
    pub const DEFAULT: u8 = 0x76;
    pub const ALTERNATIVE: u8 = 0x77;
}