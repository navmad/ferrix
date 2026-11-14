//! Asynchronous Semtech SX1262 Driver for ThinkNode M1
//!
//! Provides non-blocking packet TX/RX with hardware TCXO power gating
//! and IRQ event handling via DIO1.

use embassy_nrf::gpio::{Input, Output};
use embassy_nrf::spim::Spim;
use embassy_time::Timer;
use defmt::{debug, error, info};

use crate::radio::commands::*;
use crate::radio::{LoraConfig, LoraRadio};

/// Backward-compatible alias
pub type Sx1262Config = LoraConfig;

/// Asynchronous SX1262 Transceiver Driver
pub struct Sx1262<'a, T: embassy_nrf::spim::Instance> {
    spim: Spim<'a, T>,
    cs: Output<'static>,
    reset: Output<'static>,
    busy: Input<'static>,
    #[allow(dead_code)]
    dio1: Input<'static>,
    rf_sw: Output<'static>,
}

impl<'a, T: embassy_nrf::spim::Instance> Sx1262<'a, T> {
    pub fn new(
        spim: Spim<'a, T>,
        cs: Output<'static>,
        reset: Output<'static>,
        busy: Input<'static>,
        #[allow(dead_code)]
    dio1: Input<'static>,
        rf_sw: Output<'static>,
    ) -> Self {
        Self {
            spim,
            cs,
            reset,
            busy,
            dio1,
            rf_sw,
        }
    }

    /// Wait until the SX1262 BUSY pin goes low (with 50ms timeout)
    async fn wait_busy(&mut self) -> Result<(), ()> {
        let mut count = 0;
        while self.busy.is_high() {
            Timer::after_micros(20).await;
            count += 1;
            if count > 2500 {
                defmt::warn!("SX1262 BUSY timeout");
                return Err(());
            }
        }
        Ok(())
    }

    /// Execute a write command over SPI
    async fn write_cmd(&mut self, cmd: u8, data: &[u8]) -> Result<(), ()> {
        self.wait_busy().await?;
        self.cs.set_low();

        let header = [cmd];
        let _ = self.spim.write(&header).await;
        if !data.is_empty() {
            let _ = self.spim.write(data).await;
        }

        self.cs.set_high();
        Ok(())
    }

    /// Execute a read command over SPI
    async fn read_cmd(&mut self, cmd: u8, out: &mut [u8]) -> Result<(), ()> {
        self.wait_busy().await?;
        self.cs.set_low();

        let header = [cmd, 0x00]; // Command + NOP status byte
        let _ = self.spim.write(&header).await;
        let _ = self.spim.read(out).await;

        self.cs.set_high();
        Ok(())
    }

    /// Write registers starting at 16-bit address
    pub async fn write_register(&mut self, reg: u16, data: &[u8]) -> Result<(), ()> {
        self.wait_busy().await?;
        self.cs.set_low();
        let header = [CMD_WRITE_REGISTER, (reg >> 8) as u8, (reg & 0xFF) as u8];
        let _ = self.spim.write(&header).await;
        let _ = self.spim.write(data).await;
        self.cs.set_high();
        Ok(())
    }

    /// Hardware reset pulse
    pub async fn reset(&mut self) {
        self.reset.set_low();
        Timer::after_millis(5).await;
        self.reset.set_high();
        Timer::after_millis(20).await;
        let _ = self.wait_busy().await;
    }

    /// Initialize the radio with ThinkNode M1 hardware parameters
    pub async fn init(&mut self, config: &Sx1262Config) -> Result<(), ()> {
        info!("Initializing SX1262 LoRa radio...");
        // Ensure SX126X_POWER_EN (P0.21) power rail is energized
        self.rf_sw.set_high();
        Timer::after_millis(10).await;

        self.reset().await;

        // 1. Enter Standby RC mode
        self.write_cmd(CMD_SET_STANDBY, &[STDBY_RC]).await?;

        // 2. Configure TCXO on DIO3 @ 1.8V with 10ms startup delay
        //    (matches MeshCore ThinkNode M1; 0x0280 = 640 * 15.625us)
        info!("Enabling TCXO power via DIO3 (1.8V)...");
        self.write_cmd(CMD_SET_DIO3_AS_TCXO_CTRL, &[TCXO_CTRL_1_8V, 0x00, 0x02, 0x80]).await?;

        // Switch to STDBY_XOSC to start TCXO oscillator
        self.write_cmd(CMD_SET_STANDBY, &[STDBY_XOSC]).await?;
        Timer::after_millis(10).await;

        // 3. Calibrate devices & Calibrate Image for 902-928 MHz ISM band
        self.write_cmd(CMD_CALIBRATE, &[0x7F]).await?;
        let _ = self.wait_busy().await;
        self.write_cmd(CMD_CALIBRATE_IMAGE, &[0xE1, 0xE9]).await?;
        let _ = self.wait_busy().await;

        // 4. Configure DIO2 as RF Switch control
        self.write_cmd(CMD_SET_DIO2_AS_RF_SWITCH_CTRL, &[0x01]).await?;

        // 5. Set packet type to LoRa
        self.write_cmd(CMD_SET_PACKET_TYPE, &[PACKET_TYPE_LORA]).await?;

        // 6. Configure LoRa Sync Word (0x12 Private Network standard for MeshCore)
        self.write_register(REG_LORA_SYNC_WORD_MSB, &LORA_SYNC_WORD_PRIVATE).await?;
        info!("Configured LoRa sync word: 0x12 (MeshCore private)");

        // 7. Configure RF Frequency
        self.set_frequency(config.frequency_hz).await?;

        // 8. Configure PA for SX1262 (+22dBm capable)
        self.write_cmd(CMD_SET_PA_CONFIG, &[0x04, 0x07, 0x00, 0x01]).await?;

        // 9. Set TX output power
        self.set_tx_power(config.tx_power_dbm).await?;

        // 10. Set buffer base addresses (TX at 0x00, RX at 0x00)
        self.write_cmd(CMD_SET_BUFFER_BASE_ADDRESS, &[0x00, 0x00]).await?;

        // 11. Set Modulation Parameters
        let bw = match config.bandwidth_khz {
            62 | 63 => 0x03, // 62.5 kHz (US PNW standard)
            250 => 0x05,
            500 => 0x06,
            _ => 0x04, // 125 kHz default
        };
        let ldro = if config.spreading_factor >= 11 { 0x01 } else { 0x00 };
        self.write_cmd(CMD_SET_MODULATION_PARAMS, &[
            config.spreading_factor,
            bw,
            config.coding_rate,
            ldro,
        ]).await?;

        // 11. Set Packet Parameters
        let preamble_high = (config.preamble_length >> 8) as u8;
        let preamble_low = (config.preamble_length & 0xFF) as u8;
        self.write_cmd(CMD_SET_PACKET_PARAMS, &[
            preamble_high,
            preamble_low,
            0x00, // Variable length packet
            0xFF, // Max payload length (255)
            0x01, // CRC on
            0x00, // Standard IQ
        ]).await?;

        // 12. Route IRQs to DIO1 (TX_DONE | RX_DONE | TIMEOUT)
        let irq_mask = IRQ_TX_DONE | IRQ_RX_DONE | IRQ_TIMEOUT;
        self.write_cmd(CMD_SET_DIO_IRQ_PARAMS, &[
            (irq_mask >> 8) as u8,
            (irq_mask & 0xFF) as u8,
            (irq_mask >> 8) as u8,
            (irq_mask & 0xFF) as u8,
            0x00, 0x00, // DIO2 mask
            0x00, 0x00, // DIO3 mask
        ]).await?;

        // 13. Clear all IRQs
        self.clear_irq(IRQ_ALL).await;

        info!("SX1262 LoRa successfully initialized at {} MHz!", config.frequency_hz / 1_000_000);
        Ok(())
    }

    /// Set RF Frequency in Hz
    pub async fn set_frequency(&mut self, freq_hz: u32) -> Result<(), ()> {
        let steps = (((freq_hz as u64) << 25) / 32_000_000) as u32;
        let bytes = steps.to_be_bytes();
        self.write_cmd(CMD_SET_RF_FREQUENCY, &bytes).await
    }

    /// Set Output TX Power in dBm (-9 to +22)
    pub async fn set_tx_power(&mut self, power_dbm: i8) -> Result<(), ()> {
        let pwr = power_dbm.clamp(-9, 22) as u8;
        self.write_cmd(CMD_SET_TX_PARAMS, &[pwr, 0x02]).await
    }

    /// Read IRQ status flags
    pub async fn get_irq_status(&mut self) -> u16 {
        let mut status = [0u8; 2];
        let _ = self.read_cmd(CMD_GET_IRQ_STATUS, &mut status).await;
        u16::from_be_bytes(status)
    }

    /// Clear IRQ flags
    pub async fn clear_irq(&mut self, mask: u16) {
        let bytes = mask.to_be_bytes();
        let _ = self.write_cmd(CMD_CLEAR_IRQ_STATUS, &bytes).await;
    }

    /// Transmit a packet asynchronously
    pub async fn transmit(&mut self, data: &[u8]) -> Result<(), ()> {
        if data.is_empty() || data.len() > 255 {
            return Err(());
        }

        // Put radio in standby
        let _ = self.write_cmd(CMD_SET_STANDBY, &[STDBY_RC]).await;

        // Enable RF switch for TX
        self.rf_sw.set_high();

        // Write payload to buffer at offset 0x00
        let _ = self.wait_busy().await;
        self.cs.set_low();
        let header = [CMD_WRITE_BUFFER, 0x00];
        let _ = self.spim.write(&header).await;
        let _ = self.spim.write(data).await;
        self.cs.set_high();

        // Update packet length parameter
        let _ = self.write_cmd(CMD_SET_PACKET_PARAMS, &[
            0x00, 0x10, // 16 preamble symbols
            0x00,       // Variable header
            data.len() as u8,
            0x01,       // CRC on
            0x00,       // Standard IQ
        ]).await;

        self.clear_irq(IRQ_ALL).await;

        // Trigger TX with timeout 0 (no timeout)
        let _ = self.write_cmd(CMD_SET_TX, &[0x00, 0x00, 0x00]).await;

        // Wait for TX_DONE IRQ with 1s timeout
        for _ in 0..200 {
            let irq = self.get_irq_status().await;
            if (irq & IRQ_TX_DONE) != 0 {
                self.clear_irq(IRQ_TX_DONE).await;
                debug!("Packet transmitted successfully ({} bytes)", data.len());
                return Ok(());
            }
            if (irq & IRQ_TIMEOUT) != 0 {
                self.clear_irq(IRQ_TIMEOUT).await;
                error!("Packet transmission timed out");
                return Err(());
            }
            Timer::after_millis(5).await;
        }
        error!("Packet transmission loop timed out");
        Err(())
    }

    /// Start listening for incoming packets
    pub async fn start_receive(&mut self) {
        self.rf_sw.set_high(); // Ensure SX126X_POWER_EN remains active
        // Reset packet parameters for RX: explicit header, max payload 255, CRC on
        let _ = self.write_cmd(CMD_SET_PACKET_PARAMS, &[
            0x00, 0x10, // 16 preamble symbols
            0x00,       // Variable length packet (explicit header)
            0xFF,       // Max payload length (255)
            0x01,       // CRC on
            0x00,       // Standard IQ
        ]).await;
        self.clear_irq(IRQ_ALL).await;
        // 0xFFFFFF = Continuous RX mode
        let _ = self.write_cmd(CMD_SET_RX, &[0xFF, 0xFF, 0xFF]).await;
    }

    /// Check if a packet has been received, and if so, read it into `out`.
    /// Returns `Some((payload_length, rssi_dbm, snr_db))`
    pub async fn poll_receive(&mut self, out: &mut [u8]) -> Option<(usize, i16, i8)> {
        let irq = self.get_irq_status().await;
        if (irq & IRQ_RX_DONE) == 0 {
            return None;
        }

        self.clear_irq(IRQ_RX_DONE | IRQ_CRC_ERR).await;

        if (irq & IRQ_CRC_ERR) != 0 {
            debug!("Received packet with CRC error, dropping");
            self.start_receive().await;
            return None;
        }

        // Read RX buffer status: [payload_len, buffer_start_offset]
        let mut rx_status = [0u8; 2];
        if self.read_cmd(CMD_GET_RX_BUFFER_STATUS, &mut rx_status).await.is_err() {
            return None;
        }
        let payload_len = rx_status[0] as usize;
        let offset = rx_status[1];

        if payload_len == 0 || payload_len > out.len() {
            return None;
        }

        // Read packet status (RSSI and SNR) from SX1262
        let mut pkt_status = [0u8; 3];
        let _ = self.read_cmd(CMD_GET_PACKET_STATUS, &mut pkt_status).await;
        let rssi = -((pkt_status[0] as i16) / 2);
        let snr = (pkt_status[1] as i8) / 4;

        // Read payload from buffer
        if self.wait_busy().await.is_err() {
            return None;
        }
        self.cs.set_low();
        let header = [CMD_READ_BUFFER, offset, 0x00];
        let _ = self.spim.write(&header).await;
        let _ = self.spim.read(&mut out[..payload_len]).await;
        self.cs.set_high();

        // Re-enter continuous RX
        self.start_receive().await;

        Some((payload_len, rssi, snr))
    }
}

impl<'a, T: embassy_nrf::spim::Instance> LoraRadio for Sx1262<'a, T> {
    type Error = ();

    async fn init(&mut self, config: &LoraConfig) -> Result<(), Self::Error> {
        self.init(config).await
    }

    async fn transmit(&mut self, payload: &[u8]) -> Result<(), Self::Error> {
        self.transmit(payload).await
    }

    async fn transmit_at(
        &mut self,
        payload: &[u8],
        _eligible_at_ms: u64,
        _enqueued_at_ms: u64,
        _is_forward: bool,
    ) -> Result<(), Self::Error> {
        // The direct driver transmits immediately; scheduling lives in the
        // channel-based adapter used by the firmware.
        self.transmit(payload).await
    }

    async fn start_receive(&mut self) {
        self.start_receive().await;
    }

    async fn poll_receive(&mut self, buf: &mut [u8]) -> Option<(usize, i16, i8)> {
        self.poll_receive(buf).await
    }

    async fn receive(&mut self, buf: &mut [u8]) -> (usize, i16, i8) {
        loop {
            if let Some(packet) = self.poll_receive(buf).await {
                return packet;
            }
            Timer::after_millis(10).await;
        }
    }
}
