#![no_std]
#![no_main]
#![feature(trivial_bounds)]
//spell: ignore SYSTIMER TIMG UART defmt dhcpv uninit

use core::cmp::min;

use defmt::{error, info};
use defmt_rtt as _;
use embedded_io_async::Write;
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    uart::{Config, RxConfig, RxError, Uart, UartRx, UartTx},
    Async,
};

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};

extern crate alloc;

const WRITE_LEN_AFTER_OVERFLOW: usize = 64;

fn log_fifo_cnt(rx: &mut UartRx<'static, Async>) {
    let (fifo_cnt, apb_rx_raddr, rx_waddr) = rx.rx_fifo_count_debug();
    let calculated = if rx_waddr > apb_rx_raddr {
        rx_waddr - apb_rx_raddr
    } else if rx_waddr < apb_rx_raddr {
        (rx_waddr + 128) - apb_rx_raddr
    } else if fifo_cnt > 0 {
        128
    } else {
        0
    };
    if calculated != fifo_cnt {
        error!("Invalid fifo_cnt: {}, apb_rx_raddr: {}, rx_waddr: {}", fifo_cnt, apb_rx_raddr, rx_waddr);
    } else {
        info!("Valid fifo_cnt: {}, apb_rx_raddr: {}, rx_waddr: {}", fifo_cnt, apb_rx_raddr, rx_waddr);
    }
}

#[embassy_executor::task]
async fn uart_tester(mut rx_1: UartRx<'static, Async>, mut tx: UartTx<'static, Async>) {
    info!("Starting UART tester");

    let mut successes = 0;
    let mut total = 0;
    loop {
        'next_iteration: for sleep_after_flush_factor in 1..100 {
            info!("Success rate {}/{}", successes, total);
            let sleep_duration_after_flush = Duration::from_micros(10000 - 100 * sleep_after_flush_factor);
            info!(
                "Start test with {=u64:08us} seconds of sleep after tx.flush().",
                sleep_duration_after_flush.as_micros()
            );
            total += 1;
            // Make sure there is nothing left in the rx-buffer...
            let mut rbuf = [0u8; 128];
            for _ in 0..5 {
                Timer::after(Duration::from_millis(100)).await;
                rx_1.read_buffered(&mut rbuf).ok();
            }
            // write s.th. to the buffer to force an rx-overflow
            let buf: [u8; 1024] = core::array::from_fn(|i| i as u8);
            tx.write_all(&buf).await.unwrap();
            // Flush to ensure that the write side is silent when we start to read...
            tx.flush_async().await.unwrap();
            Timer::after(sleep_duration_after_flush).await;
            match rx_1.read_async(&mut rbuf).await {
                Err(RxError::FifoOverflowed) => (),
                other => {
                    error!("Expected FifoOverflowed on UART_1, got: {}", other);
                    log_fifo_cnt(&mut rx_1);
                    continue;
                }
            }
            match rx_1.read_async(&mut rbuf).await {
                Ok(128) => (),
                other => {
                    error!("Expected lenght 128 on UART_1, got: {}", other);
                    log_fifo_cnt(&mut rx_1);
                    continue;
                }
            }
            // Note, we have read 128 bytes after we got overflow, there can not be anything left in the buffer...

            let small_buf: [u8; WRITE_LEN_AFTER_OVERFLOW] = core::array::from_fn(|i| (i + 10) as u8);
            match tx.write_async(&small_buf).await {
                Ok(WRITE_LEN_AFTER_OVERFLOW) => (),
                Ok(len) => {
                    error!("write_async could only write {} of {} bytes", len, WRITE_LEN_AFTER_OVERFLOW);
                    continue;
                }
                Err(e) => {
                    error!("write_async failed unexpectedly with {}", e);
                    continue;
                }
            }
            tx.flush_async().await.unwrap();
            Timer::after(sleep_duration_after_flush).await;

            let len = match rx_1.read_async(&mut rbuf).await {
                Ok(WRITE_LEN_AFTER_OVERFLOW) => WRITE_LEN_AFTER_OVERFLOW,
                Ok(len) => {
                    error!("Expected lenght {} on UART_1, got: len={}", WRITE_LEN_AFTER_OVERFLOW, len);
                    len
                }
                Err(e) => {
                    error!("Expected lenght {} on UART_1, got: {}", WRITE_LEN_AFTER_OVERFLOW, e);
                    log_fifo_cnt(&mut rx_1);
                    continue;
                }
            };

            if len < WRITE_LEN_AFTER_OVERFLOW {
                Timer::after(Duration::from_secs(1)).await;
                match rx_1.read_buffered(&mut rbuf[len..WRITE_LEN_AFTER_OVERFLOW]) {
                    Ok(rest_len) => {
                        if rest_len != WRITE_LEN_AFTER_OVERFLOW - len {
                            error!(
                                "Expected lenght {} on UART_1, got: len={}",
                                WRITE_LEN_AFTER_OVERFLOW - len,
                                rest_len
                            );
                        }
                    }
                    Err(e) => {
                        error!("Expected lenght {} on UART_1, got: {}", WRITE_LEN_AFTER_OVERFLOW - len, e);
                        log_fifo_cnt(&mut rx_1);
                        continue;
                    }
                }
            }
            for i in 0..min(len, WRITE_LEN_AFTER_OVERFLOW) {
                if small_buf[i] != rbuf[i] {
                    error!(
                        "Corrupted data in buffer: \n{=[u8]:02X} != \n{=[u8]:02X}",
                        rbuf[0..WRITE_LEN_AFTER_OVERFLOW],
                        small_buf[0..WRITE_LEN_AFTER_OVERFLOW]
                    );
                    continue 'next_iteration;
                }
            }
            successes += 1;
        }
    }
}

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);
    esp_alloc::heap_allocator!(#[link_section = ".dram2_uninit"] size: 64000);
    use esp_hal::timer::systimer::SystemTimer;
    let systimer = SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(systimer.alarm0);
    info!("Embassy initialized!");

    let (rx_1, tx_1) = Uart::new(peripherals.UART1, Config::default().with_rx(RxConfig::default().with_timeout(10)))
        .unwrap()
        .with_tx(peripherals.GPIO43)
        .with_rx(peripherals.GPIO44)
        .into_async()
        .split();

    spawner.spawn(uart_tester(rx_1, tx_1)).unwrap();

    loop {
        Timer::after(Duration::from_secs(10)).await;
    }
}
