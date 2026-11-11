//! Semtech SX1262 Command and Register Constants

pub const CMD_SET_SLEEP: u8 = 0x84;
pub const CMD_SET_STANDBY: u8 = 0x80;
pub const CMD_SET_FS: u8 = 0xC1;
pub const CMD_SET_TX: u8 = 0x83;
pub const CMD_SET_RX: u8 = 0x82;
pub const CMD_STOP_TIMER_ON_PREAMBLE: u8 = 0x9F;
pub const CMD_SET_REGULATOR_MODE: u8 = 0x96;
pub const CMD_CALIBRATE: u8 = 0x89;
pub const CMD_CALIBRATE_IMAGE: u8 = 0x98;
pub const CMD_SET_PA_CONFIG: u8 = 0x95;
pub const CMD_SET_RX_TX_FALLBACK_MODE: u8 = 0x93;

pub const CMD_WRITE_REGISTER: u8 = 0x0D;
pub const CMD_READ_REGISTER: u8 = 0x1D;
pub const CMD_WRITE_BUFFER: u8 = 0x0E;
pub const CMD_READ_BUFFER: u8 = 0x1E;

pub const CMD_SET_DIO_IRQ_PARAMS: u8 = 0x08;
pub const CMD_GET_IRQ_STATUS: u8 = 0x12;
pub const CMD_CLEAR_IRQ_STATUS: u8 = 0x02;
pub const CMD_SET_DIO2_AS_RF_SWITCH_CTRL: u8 = 0x9D;
pub const CMD_SET_DIO3_AS_TCXO_CTRL: u8 = 0x97;

pub const CMD_SET_RF_FREQUENCY: u8 = 0x86;
pub const CMD_SET_PACKET_TYPE: u8 = 0x8A;
pub const CMD_GET_PACKET_TYPE: u8 = 0x11;
pub const CMD_SET_TX_PARAMS: u8 = 0x8E;
pub const CMD_SET_MODULATION_PARAMS: u8 = 0x8B;
pub const CMD_SET_PACKET_PARAMS: u8 = 0x8C;
pub const CMD_SET_CAD_PARAMS: u8 = 0x88;
pub const CMD_SET_BUFFER_BASE_ADDRESS: u8 = 0x8F;
pub const CMD_SET_LORA_SYMB_NUM_TIMEOUT: u8 = 0xA0;

// LoRa Sync Word Registers
pub const REG_LORA_SYNC_WORD_MSB: u16 = 0x0740;
pub const REG_LORA_SYNC_WORD_LSB: u16 = 0x0741;
pub const LORA_SYNC_WORD_PRIVATE: [u8; 2] = [0x14, 0x24];

pub const CMD_GET_STATUS: u8 = 0xC0;
pub const CMD_GET_RX_BUFFER_STATUS: u8 = 0x13;
pub const CMD_GET_PACKET_STATUS: u8 = 0x14;
pub const CMD_GET_RSSI_INST: u8 = 0x15;

// Packet Types
pub const PACKET_TYPE_GFSK: u8 = 0x00;
pub const PACKET_TYPE_LORA: u8 = 0x01;

// Standby Modes
pub const STDBY_RC: u8 = 0x00;
pub const STDBY_XOSC: u8 = 0x01;

// IRQ Flags
pub const IRQ_TX_DONE: u16 = 0x0001;
pub const IRQ_RX_DONE: u16 = 0x0002;
pub const IRQ_PREAMBLE_DETECTED: u16 = 0x0004;
pub const IRQ_SYNC_WORD_VALID: u16 = 0x0008;
pub const IRQ_HEADER_VALID: u16 = 0x0010;
pub const IRQ_HEADER_ERR: u16 = 0x0020;
pub const IRQ_CRC_ERR: u16 = 0x0040;
pub const IRQ_CAD_DONE: u16 = 0x0080;
pub const IRQ_CAD_ACTIVITY_DETECTED: u16 = 0x0100;
pub const IRQ_TIMEOUT: u16 = 0x0200;
pub const IRQ_ALL: u16 = 0x03FF;

// TCXO Voltages
pub const TCXO_CTRL_1_6V: u8 = 0x00;
pub const TCXO_CTRL_1_7V: u8 = 0x01;
pub const TCXO_CTRL_1_8V: u8 = 0x02;
pub const TCXO_CTRL_2_2V: u8 = 0x03;
pub const TCXO_CTRL_2_4V: u8 = 0x04;
pub const TCXO_CTRL_2_7V: u8 = 0x05;
pub const TCXO_CTRL_3_0V: u8 = 0x06;
pub const TCXO_CTRL_3_3V: u8 = 0x07;

