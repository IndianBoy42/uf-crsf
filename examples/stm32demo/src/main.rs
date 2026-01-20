//! STM32 Embedded CRSF Parser Example
//!
//! This example demonstrates how to use `uf-crsf` in a `no_std` embedded environment
//! using the Embassy async framework and an STM32 microcontroller.
//!
//! ### Real-world Integration vs. CLI Script
//! - **Async Executor:** On a real microcontroller, the async executor manages multiple
//!   tasks (e.g., flight control, telemetry, logging). The CRSF parser runs as part of
//!   a task that services the UART.
//! - **Power Management:** In a long-running app, you'd want to handle UART errors
//!   and signal loss gracefully to maintain control or enter failsafe.
//! - **DMA & Interrupts:** This demo uses DMA and ring buffers, which is the 
//!   recommended way to handle high-speed serial on embedded targets without 
//!   dropping bytes.
//!
//! ### Hardware/IO Considerations
//! - **Voltage Levels:** CRSF usually uses 3.3V logic. Ensure your MCU pins are
//!   3.3V tolerant or use level shifters if connecting to a 5V source.
//! - **Baud Rate Accuracy:** At 420,000 baud, ensure your MCU's clock configuration
//!   allows for an accurate baud rate generation.
//! - **Inverters:** Some receivers (like older FrSky gear) use inverted SBUS.
//!   CRSF is standard non-inverted UART logic, but always double-check your
//!   wiring.

#![no_std]
#![no_main]

mod fmt;

#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_stm32::usart::{Config as UsartConfig, Uart};
use embassy_stm32::{bind_interrupts, peripherals, usart, Config};
use embassy_time::{with_timeout, Duration, Timer};
use fmt::info;
use uf_crsf::CrsfParser;

// Standard interrupt binding for USART1
bind_interrupts!(struct Irqs {
    USART1 => usart::InterruptHandler<peripherals::USART1>;
});

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
enum ReadError {
    Timeout,
    Uart(usart::Error),
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // 1. Initialize the MCU with standard settings
    let config = Config::default();
    let p = embassy_stm32::init(config);

    // 2. Configure UART1 for CRSF (420,000 baud, 8N1)
    let mut usart_config = UsartConfig::default();
    usart_config.baudrate = 420000;

    let crsf_usart = Uart::new(
        p.USART1,
        p.PA10, // RX pin
        p.PA9,  // TX pin
        Irqs,
        p.DMA2_CH7,
        p.DMA2_CH5,
        usart_config,
    )
    .unwrap();

    let (_tx, rx) = crsf_usart.split();

    // 3. Setup ring buffered DMA reception.
    // This allows the hardware to receive data in the background while
    // our application logic processes the buffer.
    const BUFFER_SIZE: usize = 64;
    let mut dma_buf = [0u8; 128];
    let mut buffer = [0u8; BUFFER_SIZE];
    let mut buf_rx = rx.into_ring_buffered(&mut dma_buf);

    let mut parser = CrsfParser::new();
    loop {
        // 4. Periodically read from the ring buffer.
        match read_serial_data(&mut buf_rx, &mut buffer).await {
            Ok(bytes) => {
                // 5. Feed received bytes to the parser.
                for &byte in &buffer[..bytes] {
                    // push_byte returns Ok(Some(packet)) when a full valid 
                    // CRSF packet has been accumulated and verified via CRC.
                    match parser.push_byte(byte) {
                        Ok(Some(packet)) => {
                            // In a real application, you would handle the packet here.
                            // e.g., if let Packet::RCChannels(ch) = packet { ... }
                            info!("{:?}", packet);
                        }
                        Err(e) => info!("Parsing error {:?}", e),
                        Ok(None) => (), // Waiting for more bytes
                    }
                }
            }
            Err(e) => {
                info!("Read error: {:?}", e);
                // On error, wait a bit before retrying to avoid tight loops.
                Timer::after(Duration::from_millis(100)).await;
            }
        }
    }
}

/// Helper to read data with a timeout. 
/// In a real-world app, signal loss detection often relies on UART timeouts.
async fn read_serial_data(
    uart_rx: &mut (impl embedded_io_async::Read<Error = usart::Error> + Unpin),
    buffer: &mut [u8],
) -> Result<usize, ReadError> {
    const TIMEOUT: Duration = Duration::from_secs(1);

    with_timeout(TIMEOUT, uart_rx.read(buffer))
        .await
        .map_err(|_| ReadError::Timeout)?
        .map_err(ReadError::Uart)
}
