#![no_std]
#![no_main]
#![feature(trivial_bounds)]
use core::net::Ipv4Addr;

use defmt::{error, info, warn};
use defmt_rtt as _;
use embassy_net::{tcp::TcpSocket, Runner, Stack, StackResources};
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    rng::Rng,
    timer::timg::TimerGroup,
    uart::{Config, RxConfig, Uart, UartRx, UartTx},
    Async,
};

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_wifi::{
    init,
    wifi::{ClientConfiguration, Configuration, WifiController, WifiDevice, WifiEvent, WifiState},
    EspWifiController,
};

// When you are okay with using a nightly compiler it'

const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");

extern crate alloc;
// fifo_full_threshold (RX)
const READ_BUF_SIZE: usize = 64;

const DEBUG_LEVEL: usize = 5;

#[embassy_executor::task]
async fn writer(mut tx: UartTx<'static, Async>) {
    info!("Starting UART writer...");
    let wbuf: [u8; 256] = core::array::from_fn(|i| i as u8);
    use embedded_io_async::Write;
    loop {
        for n in 0..=255 {
            Write::write(&mut tx, &wbuf[0..=n]).await.unwrap();
            Write::flush(&mut tx).await.unwrap();
            if DEBUG_LEVEL > 3 {
                warn!("Wrote: {}", n + 1);
            }
            Timer::after(Duration::from_millis(50)).await;
        }
        Timer::after(Duration::from_millis(200)).await;
    }
}

#[embassy_executor::task(pool_size = 3)]
async fn reader(mut rx: UartRx<'static, Async>, id: &'static str) {
    info!("Starting UART reader {}...", id);
    const MAX_BUFFER_SIZE: usize = 10 * READ_BUF_SIZE + 16;

    let mut rbuf: [u8; MAX_BUFFER_SIZE] = [0u8; MAX_BUFFER_SIZE];
    let mut current_iteration = 0;
    let mut expected = 0;
    let mut failed = false;
    'outer: loop {
        let r = embedded_io_async::Read::read(&mut rx, &mut rbuf).await;

        match r {
            Ok(len) => {
                //info!("Read[{}]: len: {} [0]={}", id, len, rbuf[0]);
                for pos in 0..len {
                    if pos == 0 {
                        if rbuf[0] == 0 {
                            current_iteration += 1;
                            expected = 0;
                            failed = false;
                            if DEBUG_LEVEL > 3 {
                                info!(
                                    "Read[{}]: {} of {} for {}",
                                    id,
                                    len,
                                    (current_iteration) % 256,
                                    (current_iteration)
                                );
                            }
                        } else {
                            if DEBUG_LEVEL > 2 {
                                info!(
                                    "Read[{}]: {} => {} of {} for {}",
                                    id,
                                    len,
                                    len + expected,
                                    (current_iteration) % 256,
                                    (current_iteration)
                                );
                            }
                        }
                    }

                    if failed {
                        expected = rbuf[pos] as usize;
                        failed = false;
                    }
                    if rbuf[pos] != (expected as u8) {
                        if rbuf[pos] == 0 {
                            current_iteration += 1;
                            expected = 0;
                        } else {
                            error!(
                                "RX[{}]: expected {} got {} at {}/{} iteration {}",
                                id,
                                expected,
                                rbuf[pos],
                                pos,
                                len,
                                current_iteration
                            );
                            failed = true;
                            continue 'outer;
                        }
                    }
                    expected += 1;
                }
            }
            Err(e) => {
                error!("RX Error[{}]: {:?}", id, e);
                failed = true;
            }
        }
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

    let esp_wifi_ctrl = &*mk_static!(
        EspWifiController<'static>,
        init(timg0.timer0, rng.clone(), peripherals.RADIO_CLK).unwrap()
    );

    let (controller, interfaces) = esp_wifi::wifi::new(&esp_wifi_ctrl, peripherals.WIFI).unwrap();

    let wifi_interface = interfaces.sta;
    use esp_hal::timer::systimer::SystemTimer;
    let systimer = SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(systimer.alarm0);

    info!("Embassy initialized!");

    let config = embassy_net::Config::dhcpv4(Default::default());

    let seed = (rng.random() as u64) << 32 | rng.random() as u64;

    // Init network stack
    let (stack, runner) = embassy_net::new(
        wifi_interface,
        config,
        mk_static!(StackResources<3>, StackResources::<3>::new()),
        seed,
    );

    let uart_config_0 = Config::default().with_rx(
        RxConfig::default()
            .with_fifo_full_threshold(READ_BUF_SIZE as u16)
            .with_timeout(10),
    );

    let uart_config_1 = Config::default().with_rx(
        RxConfig::default()
            .with_fifo_full_threshold((READ_BUF_SIZE / 2) as u16)
            .with_timeout(10),
    );
    let uart_config_2 = Config::default().with_rx(
        RxConfig::default()
            .with_fifo_full_threshold((READ_BUF_SIZE / 4) as u16)
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

    //let mut i = 0;

    Timer::after(Duration::from_millis(10000)).await;
    spawner.spawn(reader(rx0, "UART_0")).unwrap();
    spawner.spawn(reader(rx1, "UART_1")).unwrap();
    spawner.spawn(reader(rx2, "UART_2")).unwrap();
    Timer::after(Duration::from_millis(1000)).await;
    spawner.spawn(writer(tx1)).unwrap();


    loop {
        // if i > 10000000 {
        //     info!("Idle {}", i);
        //     i = 0;
        // }
        Timer::after(Duration::from_millis(100)).await;
        info!("Idle");
        // for x in 1..100000 {
        //     i += x % 2;
        // }
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

    info!("Waiting to get IP address...");
    loop {
        if let Some(config) = stack.config_v4() {
            info!("Got IP: {}", config.address);
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }

    loop {
        Timer::after(Duration::from_millis(1_000)).await;

        let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);

        socket.set_timeout(Some(embassy_time::Duration::from_secs(10)));

        let remote_endpoint = (Ipv4Addr::new(142, 250, 185, 115), 80);
        info!("connecting TCP...");
        let r = socket.connect(remote_endpoint).await;
        if let Err(e) = r {
            info!("connect error: {:?}", e);
            continue;
        }
        info!("connected!");
        let mut buf = [0; 1024];
        loop {
            use embedded_io_async::Write;
            let r = socket
                .write_all(b"GET / HTTP/1.0\r\nHost: www.mobile-j.de\r\n\r\n")
                .await;
            if let Err(e) = r {
                info!("write error: {:?}", e);
                break;
            }
            let _n = match socket.read(&mut buf).await {
                Ok(0) => {
                    info!("read EOF");
                    break;
                }
                Ok(n) => n,
                Err(e) => {
                    info!("read error: {:?}", e);
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
    info!("start connection task");
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
            info!("Starting wifi");
            controller.start_async().await.unwrap();
            info!("Wifi started!");
        }
        info!("About to connect...");

        match controller.connect_async().await {
            Ok(_) => info!("Wifi connected!"),
            Err(e) => {
                info!("Failed to connect to wifi: {:?}", e);
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
