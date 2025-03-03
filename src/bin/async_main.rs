#![no_std]
#![no_main]
#![feature(trivial_bounds)]
//spell: ignore SYSTIMER TIMG UART defmt dhcpv uninit
use core::net::Ipv4Addr;

use alloc::vec::Vec;
use defmt_rtt as _;
use embassy_futures::select::select_array;
use embassy_net::{tcp::TcpSocket, Runner, Stack, StackResources};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, signal::Signal};
use embedded_io_async::Write;
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    rng::Rng,
    timer::timg::TimerGroup,
    uart::{Config, RxConfig, RxError, Uart, UartRx, UartTx},
    Async,
};

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_wifi::{
    init,
    wifi::{ClientConfiguration, Configuration, WifiController, WifiDevice, WifiEvent, WifiState},
    EspWifiController,
};
use static_cell::StaticCell;

// When you are okay with using a nightly compiler it'

const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");

extern crate alloc;
//
const MAX_FIFO_FULL_THRESHOLD: usize = 64;
const READ_BUFFER_SIZE: usize = MAX_FIFO_FULL_THRESHOLD + MAX_FIFO_FULL_THRESHOLD / 2;
const MAX_RETAINED_READS: usize = 15;
const MAX_READS_AFTER_ERROR: usize = 3;
const END_WITH_BYTES_PER_WRITE: usize = 256;
const START_WITH_BYTES_PER_WRITE: usize = 128;
const BYTES_PER_WRITE_STEP_RATE: usize = 64;
const DURATION_BEFORE_READING_OUT_GARBAGE: Duration = Duration::from_millis(1000);
const DURATION_BETWEEN_WRITE_STEPS: Duration = Duration::from_millis(10);
const SLEEP_DURATION_BEFORE_WRITE_STARTS: Duration = Duration::from_millis(1000);
const REQUIRED_NUMBER_OF_EMPTY_READS: usize = 10;
const NUM_RECIEVERS: usize = 3;

#[inline(never)]
pub fn black_box<T>(dummy: T) -> T {
    unsafe {
        let ret = core::ptr::read_volatile(&dummy);
        core::mem::forget(dummy);
        ret
    }
}

#[embassy_executor::task]
async fn writer(
    mut tx: UartTx<'static, Async>,
    rx_0_ready_signal: &'static Signal<NoopRawMutex, bool>,
    tx_0_ready_signal: &'static Signal<NoopRawMutex, bool>,
    rx_1_ready_signal: &'static Signal<NoopRawMutex, bool>,
    tx_1_ready_signal: &'static Signal<NoopRawMutex, bool>,
    rx_2_ready_signal: &'static Signal<NoopRawMutex, bool>,
    tx_2_ready_signal: &'static Signal<NoopRawMutex, bool>,
) {
    let write_buf: [u8; END_WITH_BYTES_PER_WRITE + 1] = core::array::from_fn(|i| i as u8);

    let rx_ready_signals: [&Signal<NoopRawMutex, bool>; NUM_RECIEVERS] =
        [rx_0_ready_signal, rx_1_ready_signal, rx_2_ready_signal];
    let tx_ready_signals: [&Signal<NoopRawMutex, bool>; NUM_RECIEVERS] =
        [tx_0_ready_signal, tx_1_ready_signal, tx_2_ready_signal];
    let mut ready: [bool; 3] = [false, false, false];

    loop {
        loop {
            let all_ready = ready.iter().all(|&is_ready| is_ready);
            info!("UART_TX: SIG Signal TX {}", if all_ready { "ready" } else { "not ready" });

            tx_ready_signals.iter().for_each(|signal| signal.signal(all_ready));
            if all_ready {
                break;
            }
            info!("UART_TX: SIG Wait for RX ready");
            let (is_ready, ix) = select_array([
                rx_0_ready_signal.wait(),
                rx_1_ready_signal.wait(),
                rx_2_ready_signal.wait(),
            ])
            .await;
            ready[ix] = is_ready;
            info!("UART_TX: UART_{} is {}", ix, if ready[ix] { "ready" } else { "not ready" });
            rx_ready_signals[ix].reset();
        }
        ready.fill(false);

        info!("UART_TX: Start sending bytes in {=u64:ms}s...", SLEEP_DURATION_BEFORE_WRITE_STARTS.as_millis());
        Timer::after(SLEEP_DURATION_BEFORE_WRITE_STARTS).await;

        for bytes_to_write in START_WITH_BYTES_PER_WRITE..=END_WITH_BYTES_PER_WRITE {
            if bytes_to_write % BYTES_PER_WRITE_STEP_RATE == 0 {
                let buf = &write_buf[0..bytes_to_write];
                if bytes_to_write == 1 {
                    info!("UART_TX: Write: [0..{}], [{=u8:02X}]", buf.len(), buf[0]);
                } else if bytes_to_write == 2 {
                    info!("UART_TX: Write: [0..{}], {=[u8]:02X}", buf.len(), buf);
                } else {
                    info!("UART_TX: Write: [0..{}], [{=u8:02X}..{=u8:02X}]", buf.len(), buf[0], buf[buf.len() - 1]);
                }
                Write::write_all(&mut tx, &write_buf[0..bytes_to_write]).await.unwrap();
                Write::flush(&mut tx).await.unwrap();
                info!("UART_TX: Sleep {=u64:03ms}s", DURATION_BETWEEN_WRITE_STEPS.as_millis());
                Timer::after(DURATION_BETWEEN_WRITE_STEPS).await;
            }
        }

        info!("UART_TX: Finished sending bytes, buffered flushed.");
    }
}
use defmt::{error, info, trace, Format};

#[derive(Format)]
struct Corruption {
    pos_in_buffer: usize,
    expected: u8,
    actual: u8,
    max_value_in_iteration: u8,
}

#[derive(Format)]
enum Reason {
    RxError(RxError),
    Corruption(Corruption),
}

struct Failure {
    at_reads: usize,
    reason: Reason,
}

#[derive(defmt::Format)]
struct FifoCnt {
    fifo_cnt: u16,
    calc_cnt: u16,
    apb_rx_raddr: u16,
    rx_waddr: u16,
}

struct ReadInfo {
    called_at: embassy_time::Instant,
    returned_at: embassy_time::Instant,
    length: usize,
    rx_error: Option<RxError>,
    content: [u8; READ_BUFFER_SIZE],
    fifo_cnt_before: FifoCnt,
    fifo_cnt_after: FifoCnt,
}

use defmt;

#[derive(defmt::Format)]

struct UartStatistics {
    fifo_over_flowed: usize,
    glitch_occurred: usize,
    frame_format_violated: usize,
    parity_mismatch: usize,
    corrupted_data: usize,
    corrupted_data_in_first_byte: usize,
    iterations: usize,
    other: usize,
}

fn read_fifo_cnt(rx: &mut UartRx<'static, Async>) -> FifoCnt {
    // let (fifo_cnt, apb_rx_raddr, rx_waddr) = rx.rx_fifo_count_debug();
    // let calc_cnt =if rx_waddr > apb_rx_raddr {
    //     rx_waddr - apb_rx_raddr
    // } else if rx_waddr < apb_rx_raddr {
    //     (rx_waddr + 128) - apb_rx_raddr
    // } else if fifo_cnt > 0 {
    //     128
    // } else {
    //     0
    // };
    // return FifoCnt { fifo_cnt, calc_cnt, apb_rx_raddr, rx_waddr}
    return FifoCnt { fifo_cnt:0, calc_cnt: 0, apb_rx_raddr:0, rx_waddr:0}

}

async fn read_garbage(id: &str, rx: &mut UartRx<'static, Async>) {
    let mut empty_reads = 0;
    info!("{}: Before reading garbage, fifo_cnt={}", id, read_fifo_cnt(rx));
    loop {
        let mut buffer: [u8; 256] = [0; 256];
        let garbage = rx.read_buffered(&mut buffer);        
        match garbage {
            Ok(len) => {
                if len == 0 {
                    empty_reads += 1;
                    if empty_reads <= REQUIRED_NUMBER_OF_EMPTY_READS {
                        Timer::after(Duration::from_millis(10)).await;
                    } else {
                        info!(
                            "{}: Read zero bytes of garbage, {} times, hope this is enough. len={} {=[u8]:02X}",
                            id,
                            empty_reads,
                            len,
                            buffer[0..len]
                        );                        
                        info!("{}: After reading garbage, fifo_cnt={}", id, read_fifo_cnt(rx));
                        break;
                    }
                } else {
                    if empty_reads > 0 {
                        error!(
                            "{}: Read got {} bytes of garbage {=[u8]:02X}, after read_buffered returned zero {} times",
                            id,
                            len,
                            buffer[0..len],
                            empty_reads
                        );
                    } else {
                        info!("{}: Read got {} bytes of garbage {=[u8]:02X}", id, len, buffer[0..len]);
                    }
                }
            }
            Err(e) => {
                if empty_reads > 0 {
                    error!("{}: Read got {}, after read_buffered returned zero {} times", id, e, empty_reads);
                } else {
                    info!("{}: Got error while reading out previous garbage: {}", id, e)
                }
            }
        }
    }
}

#[embassy_executor::task(pool_size = 3)]
async fn reader(
    mut rx: UartRx<'static, Async>,
    read_info_log: &'static mut [ReadInfo; MAX_RETAINED_READS],
    id: &'static str,
    rx_ready_signal: &'static Signal<NoopRawMutex, bool>,
    tx_ready_signal: &'static Signal<NoopRawMutex, bool>,
    rx_interruped: &'static Signal<NoopRawMutex, bool>,
) {
    info!("Starting UART reader {}...", id);

    let mut stats = UartStatistics {
        fifo_over_flowed: 0,
        glitch_occurred: 0,
        frame_format_violated: 0,
        parity_mismatch: 0,
        other: 0,
        corrupted_data: 0,
        corrupted_data_in_first_byte: 0,
        iterations: 0,
    };

    let mut uart_possible_polluted = true;

    loop {
        if uart_possible_polluted {
            info!(
                "{}: Sleeping {=u64:03ms}s, before reading out garbage...",
                id,
                DURATION_BEFORE_READING_OUT_GARBAGE.as_millis()
            );
            Timer::after(DURATION_BEFORE_READING_OUT_GARBAGE).await;
            read_garbage(id, &mut rx).await;
            uart_possible_polluted = false;
        }
        if stats.iterations > 0 {
            info!("{} STATISTICS: {}", id, stats);
        }

        info!("{}: SIG Signal RX ready=true", id);
        rx_ready_signal.signal(true);

        info!("{}: SIG Wait for all ready", id);
        loop {
            let all_ready = tx_ready_signal.wait().await;
            tx_ready_signal.reset();
            if all_ready {
                break;
            }
        }

        //let mut has_checked_for_corrupted_data_in_first_byte = false;
        let mut max_value_in_iteration: usize = START_WITH_BYTES_PER_WRITE - 1;
        let mut expected: usize = 0;
        let mut total_reads = 0;
        let mut possible_failure: Option<Failure> = None;
        let mut total_bytes_read = 0;
        'next_row: loop {
            let buf = &mut read_info_log[total_reads % MAX_RETAINED_READS];
            buf.called_at = embassy_time::Instant::now();
            buf.fifo_cnt_before = read_fifo_cnt(&mut rx);
            let read_result = embedded_io_async::Read::read(&mut rx, &mut buf.content).await;
            buf.fifo_cnt_after = read_fifo_cnt(&mut rx);
            buf.returned_at = embassy_time::Instant::now();

            match read_result {
                Ok(len) => {
                    total_bytes_read += len;
                    let content = buf.content;
                    trace!(
                        "{}: READ: 0..{=usize:03}, len={=usize:03}, {=[u8]:02X}",
                        id,
                        len,
                        content[0..len].len(),
                        content[0..len]
                    );

                    buf.rx_error = None;
                    buf.length = len;
                    for pos in 0..len {
                        if content[pos] != expected as u8 {
                            possible_failure = Some(Failure {
                                at_reads: total_reads,
                                reason: Reason::Corruption(Corruption {
                                    pos_in_buffer: pos,
                                    expected: expected as u8,
                                    actual: content[pos],
                                    max_value_in_iteration: max_value_in_iteration as u8,
                                }),
                            });
                            trace!(
                                "{}: Not what one would expect {}, actual: {}, max_value_in_iteration: {}",
                                id,
                                expected,
                                content[pos],
                                max_value_in_iteration
                            );
                            trace!("{}: CONCLUSION Found corrupt data", id);
                            break 'next_row;
                        }
                        if expected == max_value_in_iteration {
                            let bytes_written = max_value_in_iteration + 1;
                            if (bytes_written + BYTES_PER_WRITE_STEP_RATE) > END_WITH_BYTES_PER_WRITE {
                                trace!("{}: CONCLUSION Read all data without errors", id);
                                break 'next_row;
                            }
                            trace!("{}: Start next iteration, i.e set expected to 0 when expected: {}, actual: {}, max_value_in_iteration: {}", id, expected, content[pos], max_value_in_iteration);
                            expected = 0;
                            max_value_in_iteration += BYTES_PER_WRITE_STEP_RATE;
                        } else {
                            expected += 1
                        }
                    }
                }
                Err(e) => {
                    buf.length = 0;
                    buf.rx_error = Some(e);
                    possible_failure = Some(Failure {
                        at_reads: total_reads,
                        reason: Reason::RxError(e),
                    });
                    trace!("{}: CONCLUSION Got RxError", id);
                    break 'next_row;
                }
            }

            total_reads += 1;
        }


        if let Some(failure) = possible_failure {
    
            uart_possible_polluted = true;
            for _n in 0..MAX_READS_AFTER_ERROR {
                total_reads += 1;
                let buf = &mut read_info_log[total_reads % MAX_RETAINED_READS];
                buf.rx_error = None;
                buf.length = 0;
                buf.called_at = embassy_time::Instant::now();
                buf.fifo_cnt_before = read_fifo_cnt(&mut rx);
                match rx.read_buffered(&mut buf.content) {
                    Ok(len) => buf.length = len,
                    Err(e) => buf.rx_error = Some(e),
                }
                buf.fifo_cnt_after = read_fifo_cnt(&mut rx);             
                buf.returned_at = embassy_time::Instant::now();
            }
            rx_ready_signal.signal(false);
            tx_ready_signal.wait().await;
            tx_ready_signal.reset();

            match failure.reason {
                Reason::RxError(rx_error) => {
                    match rx_error {
                        RxError::FifoOverflowed => stats.fifo_over_flowed += 1,
                        RxError::GlitchOccurred => stats.glitch_occurred += 1,
                        RxError::FrameFormatViolated => stats.frame_format_violated += 1,
                        RxError::ParityMismatch => stats.parity_mismatch += 1,
                        _ => stats.other += 1,
                    }

                    log_history_reads(id, &read_info_log, total_reads, failure.at_reads).await;
                    let failed_ix = failure.at_reads % MAX_RETAINED_READS;
                    let info = &read_info_log[failed_ix];
                    error!(
                        "{}: PROBLEM [{}] {=u64:08us}..{=u64:08us} Δ{=u64:06us}s before:{}, after:{}: {}",
                        id,
                        failed_ix,
                        info.called_at.as_micros(),
                        info.returned_at.as_micros(),
                        info.returned_at.duration_since(info.called_at).as_micros(),
                        info.fifo_cnt_before, info.fifo_cnt_after,
                        rx_error
                    );
                    log_future_reads(id, &read_info_log, total_reads, failure.at_reads).await;
                }
                Reason::Corruption(corruption) => {
                    stats.corrupted_data += 1;
                    error!(
                        "{}: Corrupted at {}, actual {:02X}, expected {:02X}, max_value_in_iteration {:02X}",
                        id,
                        corruption.pos_in_buffer,
                        corruption.actual,
                        corruption.expected,
                        corruption.max_value_in_iteration
                    );

                    let failed_ix = failure.at_reads % MAX_RETAINED_READS;
                    let info = &read_info_log[failed_ix];
                    let left_of = &info.content[..corruption.pos_in_buffer];
                    let right_of = &info.content[corruption.pos_in_buffer + 1..info.length];

                    log_history_reads(id, &read_info_log, total_reads, failure.at_reads).await;

                    error!(
                        "{}: PROBLEM [{}] {=u64:08us}..{=u64:08us} Δ{=u64:06us}s before:{}, after:{}, len={=usize:03} {=[u8]:02X} {=u8:02X}<{=u8:02X}> {=[u8]:02X}",
                        id,
                        failed_ix,
                        info.called_at.as_micros(),
                        info.returned_at.as_micros(),
                        info.returned_at.duration_since(info.called_at).as_micros(),
                        info.fifo_cnt_before, info.fifo_cnt_after,

                        info.length,
                        left_of,
                        corruption.actual,
                        corruption.expected,
                        right_of
                    );
                    log_future_reads(id, &read_info_log, total_reads, failure.at_reads).await;
                }
            }
        } else  {
            info!("{}: Successfully read {} bytes", id, total_bytes_read)
        }

        stats.iterations += 1;
    }
}

struct ContinousRange {
    start: usize,
    end: usize,
}

fn log_read(id: &str, context: &str, row: usize, info: &ReadInfo) {
    let mut parts:Vec<ContinousRange> = Vec::new();
    
    
    let logged = if info.length > 0 {
        let mut prev:usize = info.content[0] as usize;
        let mut part_start = 0;        
        for i in 1..info.length {            
            if info.content[i] as usize != prev as usize + 1 {
                // info!("{}: Push range: {}, {}, {}",id, i, info.content[i], prev +1);
                parts.push(ContinousRange {start: part_start, end: i-1});
                part_start = i;
                prev = info.content[i] as usize -1;
            }
            prev+=1;
        }
        parts.push(ContinousRange {start: part_start, end: info.length-1});
    
        
        match parts.len() {
            1 =>    {    info!(
                "{}: {} [{}] {=u64:08us}..{=u64:08us} Δ{=u64:06us}s before:{}, after:{}, len={=usize:03} [{=u8:02X}..{=u8:02X}]",
                id,
                context,
                row,
                info.called_at.as_micros(),
                info.returned_at.as_micros(),
                info.returned_at.duration_since(info.called_at).as_micros(),
                info.fifo_cnt_before, info.fifo_cnt_after,
                info.length,
                &info.content[parts[0].start],
                &info.content[parts[0].end],
                
            ); true},
            2 =>      {  info!(
                "{}: {} [{}] {=u64:08us}..{=u64:08us} Δ{=u64:06us}s before:{}, after:{}, len={=usize:03} [{=u8:02X}..{=u8:02X}], [{=u8:02X}..{=u8:02X}]",
                id,
                context,
                row,
                info.called_at.as_micros(),
                info.returned_at.as_micros(),
                info.returned_at.duration_since(info.called_at).as_micros(),
                info.fifo_cnt_before, info.fifo_cnt_after,
                info.length,
                &info.content[parts[0].start],
                &info.content[parts[0].end],
                &info.content[parts[1].start],
                &info.content[parts[1].end],
            ); true},
            n => {
                info!("{}: To many parts: {}", id, n);
                false
            }            
        }
    } else {
        false
    };
    if !logged {
        info!(
            "{}: {} [{}] {=u64:08us}..{=u64:08us} Δ{=u64:06us}s before:{}, after:{}, len={=usize:03} {=[u8]:02X}",
            id,
            context,
            row,
            info.called_at.as_micros(),
            info.returned_at.as_micros(),
            info.returned_at.duration_since(info.called_at).as_micros(),
            info.fifo_cnt_before, info.fifo_cnt_after,
            info.length,
            &info.content[..info.length]
            );
        }
    }



async fn log_history_reads(
    id: &str,
    read_info_log: &[ReadInfo; MAX_RETAINED_READS],
    _total_reads: usize,
    failed_at_total_reads: usize,
) {
    let last_history_ix = failed_at_total_reads;
    let max_history_size = MAX_RETAINED_READS - MAX_READS_AFTER_ERROR - 1;
    info!("{}: max_history_size = MAX_RETAINED_READS - MAX_READS_AFTER_ERROR - 1", id);
    info!(
        "{}: {=usize:016} = {=usize:018} - {=usize:021} - 1",
        id, max_history_size, MAX_RETAINED_READS, MAX_READS_AFTER_ERROR
    );
    if last_history_ix < max_history_size {
        info!("{}: last_history_ix < max_history_size", id);
        info!("{}: for i in 0..{}", id, last_history_ix);
        for i in 0..last_history_ix {
            Timer::after(Duration::from_millis(100)).await;
            let buf_index = i % MAX_RETAINED_READS;
            log_read(id, "HISTORY", buf_index, &read_info_log[buf_index]);
        }
    } else {
        info!("{}: last_history_ix >= max_history_size", id);
        info!("{}:              {} >= {}", last_history_ix, max_history_size, id);
        info!(
            "{}: for i in {}..={}",
            id,
            last_history_ix,
            last_history_ix + MAX_RETAINED_READS - MAX_READS_AFTER_ERROR - 1
        );

        for i in last_history_ix..=last_history_ix + MAX_RETAINED_READS - MAX_READS_AFTER_ERROR - 1 {
            Timer::after(Duration::from_millis(100)).await;
            let buf_index = i % MAX_RETAINED_READS;
            log_read(id, "HISTORY", buf_index, &read_info_log[buf_index]);
        }
    }
}

async fn log_future_reads(
    id: &str,
    read_info_log: &[ReadInfo; MAX_RETAINED_READS],
    total_reads: usize,
    failed_at_total_reads: usize,
) {
    for i in failed_at_total_reads + 1..=total_reads {
        Timer::after(Duration::from_millis(100)).await;
        log_read(id, "FUTURE ", i % MAX_RETAINED_READS, &read_info_log[i % MAX_RETAINED_READS]);
    }
}

// When you are okay with using a nightly compiler it's better to use s://docs.rs/static_cell/2.1.0/static_cell/macro.make_static.html
macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[link_section = ".dram2_uninit"] size: 64000);
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let mut rng = Rng::new(peripherals.RNG);

    let esp_wifi_ctrl =
        &*mk_static!(EspWifiController<'static>, init(timg0.timer0, rng.clone(), peripherals.RADIO_CLK).unwrap());

    let (controller, interfaces) = esp_wifi::wifi::new(&esp_wifi_ctrl, peripherals.WIFI).unwrap();

    let wifi_interface = interfaces.sta;
    use esp_hal::timer::systimer::SystemTimer;
    let systimer = SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(systimer.alarm0);

    info!("Embassy initialized!");

    let config = embassy_net::Config::dhcpv4(Default::default());

    let seed = (rng.random() as u64) << 32 | rng.random() as u64;

    // Init network stack
    let (stack, runner) =
        embassy_net::new(wifi_interface, config, mk_static!(StackResources<3>, StackResources::<3>::new()), seed);

    let uart_config_0 = Config::default().with_rx(
        RxConfig::default()
            .with_fifo_full_threshold(MAX_FIFO_FULL_THRESHOLD as u16)
            .with_timeout(10),
    );

    let uart_config_1 = Config::default().with_rx(
        RxConfig::default()
            .with_fifo_full_threshold((MAX_FIFO_FULL_THRESHOLD / 2) as u16)
            .with_timeout(10),
    );
    let uart_config_2 = Config::default().with_rx(
        RxConfig::default()
            .with_fifo_full_threshold((MAX_FIFO_FULL_THRESHOLD / 4) as u16)
            .with_timeout(10),
    );
    let uart0 = Uart::new(peripherals.UART0, uart_config_0)
        .unwrap()
        .with_tx(peripherals.GPIO42)
        .with_rx(peripherals.GPIO41)
        .into_async();

    let uart1 = Uart::new(peripherals.UART1, uart_config_1)
        .unwrap()
        .with_tx(peripherals.GPIO43)
        .with_rx(peripherals.GPIO44)
        .into_async();

    let uart2 = Uart::new(peripherals.UART2, uart_config_2)
        .unwrap()
        .with_tx(peripherals.GPIO40)
        .with_rx(peripherals.GPIO39)
        .into_async();

    let (rx0, _tx0) = uart0.split();
    let (rx1, tx1) = uart1.split();
    let (rx2, _tx2) = uart2.split();

    spawner.spawn(connection(controller)).unwrap();
    spawner.spawn(net_task(runner)).unwrap();
    spawner.spawn(tcp_connector(stack)).unwrap();

    static RX_0_READY_SIGNAL: StaticCell<Signal<NoopRawMutex, bool>> = StaticCell::new();
    let rx_0_ready_signal = &*RX_0_READY_SIGNAL.init(Signal::new());
    static RX_1_READY_SIGNAL: StaticCell<Signal<NoopRawMutex, bool>> = StaticCell::new();
    let rx_1_ready_signal = &*RX_1_READY_SIGNAL.init(Signal::new());
    static RX_2_READY_SIGNAL: StaticCell<Signal<NoopRawMutex, bool>> = StaticCell::new();
    let rx_2_ready_signal = &*RX_2_READY_SIGNAL.init(Signal::new());

    static TX_0_READY_SIGNAL: StaticCell<Signal<NoopRawMutex, bool>> = StaticCell::new();
    let tx_0_ready_signal = &*TX_0_READY_SIGNAL.init(Signal::new());
    static TX_1_READY_SIGNAL: StaticCell<Signal<NoopRawMutex, bool>> = StaticCell::new();
    let tx_1_ready_signal = &*TX_1_READY_SIGNAL.init(Signal::new());
    static TX_2_READY_SIGNAL: StaticCell<Signal<NoopRawMutex, bool>> = StaticCell::new();
    let tx_2_ready_signal = &*TX_2_READY_SIGNAL.init(Signal::new());

    static RX_0_INTERRUPTED: StaticCell<Signal<NoopRawMutex, bool>> = StaticCell::new();
    let rx_0_interrupted = &*RX_0_INTERRUPTED.init(Signal::new());
    static RX_1_INTERRUPTED: StaticCell<Signal<NoopRawMutex, bool>> = StaticCell::new();
    let rx_1_interrupted = &*RX_1_INTERRUPTED.init(Signal::new());
    static RX_2_INTERRUPTED: StaticCell<Signal<NoopRawMutex, bool>> = StaticCell::new();
    let rx_2_interrupted = &*RX_2_INTERRUPTED.init(Signal::new());

    spawner
        .spawn(writer(
            tx1,
            &rx_0_ready_signal,
            &tx_0_ready_signal,
            &rx_1_ready_signal,
            &tx_1_ready_signal,
            &rx_2_ready_signal,
            &tx_2_ready_signal,
        ))
        .unwrap();

    static READ_INFO_LOG_0: StaticCell<[ReadInfo; MAX_RETAINED_READS]> = StaticCell::new();
    let read_info_log_0 = READ_INFO_LOG_0.init_with(|| {
        core::array::from_fn(|_i| ReadInfo {
            called_at: embassy_time::Instant::MIN,
            returned_at: embassy_time::Instant::MIN,
            length: 0,
            content: [0u8; READ_BUFFER_SIZE],
            rx_error: None,
            fifo_cnt_before: FifoCnt{fifo_cnt: 0, calc_cnt:0, apb_rx_raddr: 0 , rx_waddr: 0},
            fifo_cnt_after: FifoCnt{fifo_cnt: 0, calc_cnt:0, apb_rx_raddr: 0 , rx_waddr: 0}
        })
    });

    static READ_INFO_LOG_1: StaticCell<[ReadInfo; MAX_RETAINED_READS]> = StaticCell::new();
    let read_info_log_1 = READ_INFO_LOG_1.init_with(|| {
        core::array::from_fn(|_i| ReadInfo {
            called_at: embassy_time::Instant::MIN,
            returned_at: embassy_time::Instant::MIN,
            length: 0,
            content: [0u8; READ_BUFFER_SIZE],
            rx_error: None,
            fifo_cnt_before: FifoCnt{fifo_cnt: 0, calc_cnt:0, apb_rx_raddr: 0 , rx_waddr: 0},
            fifo_cnt_after: FifoCnt{fifo_cnt: 0, calc_cnt:0, apb_rx_raddr: 0 , rx_waddr: 0}
        })
    });

    static READ_INFO_LOG_2: StaticCell<[ReadInfo; MAX_RETAINED_READS]> = StaticCell::new();
    let read_info_log_2 = READ_INFO_LOG_2.init_with(|| {
        core::array::from_fn(|_i| ReadInfo {
            called_at: embassy_time::Instant::MIN,
            returned_at: embassy_time::Instant::MIN,
            length: 0,
            content: [0u8; READ_BUFFER_SIZE],
            rx_error: None,
            fifo_cnt_before: FifoCnt{fifo_cnt: 0, calc_cnt:0, apb_rx_raddr: 0 , rx_waddr: 0},
            fifo_cnt_after: FifoCnt{fifo_cnt: 0, calc_cnt:0, apb_rx_raddr: 0 , rx_waddr: 0}
        })
    });

    Timer::after(Duration::from_millis(1000)).await;
    spawner
        .spawn(reader(rx0, read_info_log_0, "UART_0", &rx_0_ready_signal, &tx_0_ready_signal, &rx_0_interrupted))
        .unwrap();
    spawner
        .spawn(reader(rx1, read_info_log_1, "UART_1", &rx_1_ready_signal, &tx_1_ready_signal, &rx_1_interrupted))
        .unwrap();
    spawner
        .spawn(reader(rx2, read_info_log_2, "UART_2", &rx_2_ready_signal, &tx_2_ready_signal, &rx_2_interrupted))
        .unwrap();
    //Timer::after(Duration::from_millis(1000)).await;

    loop {
        let random = rng.random();

        for _ in 0..random & 0x000000FF0 {
            black_box(())
        }

        Timer::after(Duration::from_millis((random & 0x000F) as u64 * 100)).await;
        rx_0_interrupted.signal(true);
        //rx_0_interrupted.signal(2);
        //rx_0_interrupted.signal(3);
        info!("Idle");
    }
}

#[embassy_executor::task]
async fn tcp_connector(stack: Stack<'static>) {
    let mut rx_buffer = [0; 4096];
    let mut tx_buffer = [0; 4096];

    loop {
        if stack.is_link_up() {
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }

    // info!("Waiting to get IP address...");
    loop {
        if let Some(_config) = stack.config_v4() {
            // info!("Got IP: {}", config.address);
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }

    loop {
        Timer::after(Duration::from_millis(1_000)).await;

        let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);

        socket.set_timeout(Some(embassy_time::Duration::from_secs(10)));

        let remote_endpoint = (Ipv4Addr::new(142, 250, 185, 115), 80);
        // info!("connecting TCP...");
        let r = socket.connect(remote_endpoint).await;
        if let Err(_e) = r {
            //info!("connect error: {:?}", e);
            continue;
        }
        // info!("connected!");
        let mut buf = [0; 1024];
        loop {
            use embedded_io_async::Write;
            let r = socket
                .write_all(b"GET / HTTP/1.0\r\nHost: www.mobile-j.de\r\n\r\n")
                .await;
            if let Err(_e) = r {
                //info!("write error: {:?}", e);
                break;
            }
            let _n = match socket.read(&mut buf).await {
                Ok(0) => {
                    // info!("read EOF");
                    break;
                }
                Ok(n) => n,
                Err(_e) => {
                    //info!("read error: {:?}", e);
                    break;
                }
            };
            //info!("{}", core::str::from_utf8(&buf[..n]).unwrap());
        }
        Timer::after(Duration::from_millis(3000)).await;
    }
}
#[embassy_executor::task]
async fn connection(mut controller: WifiController<'static>) {
    // info!("start connection task");
    loop {
        match esp_wifi::wifi::wifi_state() {
            WifiState::StaConnected => {
                // wait until we're no longer connected
                controller.wait_for_event(WifiEvent::StaDisconnected).await;
                Timer::after(Duration::from_millis(5000)).await
            }
            _ => {}
        }
        if !matches!(controller.is_started(), Ok(true)) {
            let client_config = Configuration::Client(ClientConfiguration {
                ssid: SSID.try_into().unwrap(),
                password: PASSWORD.try_into().unwrap(),
                ..Default::default()
            });
            controller.set_configuration(&client_config).unwrap();
            // info!("Starting wifi");
            controller.start_async().await.unwrap();
            // info!("Wifi started!");
        }
        // info!("About to connect...");

        match controller.connect_async().await {
            Ok(_) => trace!("Wifi connected!"),
            Err(_e) => {
                //info!("Failed to connect to wifi: {:?}", e);
                Timer::after(Duration::from_millis(5000)).await
            }
        }
        Timer::after(Duration::from_millis(5000)).await;
        controller.stop_async().await.unwrap()
    }
}

#[embassy_executor::task]
async fn net_task(mut runner: Runner<'static, WifiDevice<'static>>) {
    runner.run().await
}
