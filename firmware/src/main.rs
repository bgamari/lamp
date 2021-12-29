#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![feature(inherent_associated_types)]
#![feature(never_type)]
#![feature(async_closure)]

use defmt::{info, unwrap};
use defmt_rtt as _;
use panic_probe as _;

use embassy::blocking_mutex::kind;
use embassy::channel::mpsc;
use embassy::executor::Spawner;
use embassy::time::{Delay, Duration, Timer};
use embassy::util::Forever;
use embassy_stm32::{exti, gpio, Peripherals};

use futures::FutureExt;

use core::sync::atomic::{AtomicUsize, Ordering};

use embedded_hal::digital::v2::OutputPin;

mod button;
mod active_state;
use crate::button::Button;

const MIN_BAT_mV: u32 = 3_200;
const MAX_BAT_mV: u32 = 8_500;

fn oversample_adc<T: embassy_stm32::adc::Instance>(
    adc: &mut embassy_stm32::adc::Adc<T>,
    channel: &mut impl embassy_stm32::adc::AdcPin<T>,
    samples: u8,
) -> u16 {
    let mut accum: u32 = 0;
    for _i in 0..samples {
        let x: u16 = adc.read(channel);
        accum += x as u32;
    }
    (accum / samples as u32) as u16
}

async fn blink_ms<'a, T: gpio::Pin>(pin: &mut gpio::Output<'a, T>, time: Duration) {
    unwrap!(pin.set_high());
    Timer::after(time).await;
    unwrap!(pin.set_low());
}

pub fn config() -> embassy_stm32::Config {
    use embassy_stm32::rcc;
    let mut config = embassy_stm32::Config::default();
    let def: rcc::Config = core::default::Default::default();
    // N.B. The ADC is unusable in low-power run mode
    let rcc_config = def
        .clock_src(rcc::ClockSrc::HSI16(rcc::HSI16Prescaler::Div8));
    config.rcc = rcc_config;
    config.enable_debug_during_sleep = cfg!(feature = "debug");
    config
}

#[derive(defmt::Format, Clone, Copy, Debug)]
enum Mode {
    Off,
    ConstVoltage { setpoint_mV: u32 },
    ConstCurrent { setpoint_mA: u32 },
}

impl Mode {
    pub const fn from_millivolts(v: u32) -> Self {
        Mode::ConstVoltage { setpoint_mV: v }
    }

    pub const fn from_milliamps(i: u32) -> Self {
        Mode::ConstCurrent { setpoint_mA: i }
    }
}

struct Regulator<'a> {
    adc: embassy_stm32::adc::Adc<'a, embassy_stm32::peripherals::ADC1>,
    dac: embassy_stm32::dac::Dac<'a, embassy_stm32::peripherals::DAC1>,
    isense_pin: embassy_stm32::peripherals::PA5,
    vbat_pin: embassy_stm32::peripherals::PB1,
    out_en: gpio::Output<'a, embassy_stm32::peripherals::PA1>,
}

impl<'a> Regulator<'a> {
    fn read_isense_mA(&mut self) -> u32 {
        let x = self.adc.read(&mut self.isense_pin);
        let isense_mA = self.adc.to_millivolts(x) as u32 * 1000 / 1500;
        isense_mA
    }

    fn read_vbat_mV(&mut self) -> u32 {
        let x = self.adc.read(&mut self.vbat_pin);
        let vbat_mV = self.adc.to_millivolts(x) as u32 * 4090 / 1000;
        vbat_mV
    }

    fn set_output_dac(&mut self, cp: u8) {
        let y = embassy_stm32::dac::Value::Bit8(cp);
        self.dac.set(embassy_stm32::dac::Channel::Ch1, y).unwrap();
    }

    async fn enable_output(&mut self) {
        self.dac
            .enable_channel(embassy_stm32::dac::Channel::Ch1)
            .unwrap();
        // Allow DAC to settle
        embassy::time::Timer::after(embassy::time::Duration::from_micros(10)).await;
        unwrap!(self.out_en.set_high());
    }

    async fn disable_output(&mut self) {
        unwrap!(self.out_en.set_low());
        self.dac
            .disable_channel(embassy_stm32::dac::Channel::Ch1)
            .unwrap();
    }
}

#[embassy::task]
async fn feedback(
    mut msgs: mpsc::Receiver<'static, kind::CriticalSection, Mode, 1>,
    mut reg: Regulator<'static>,
) -> () {
    let mut task = active_state::new_task();
    const INITIAL_CODEPOINT: u8 = 128;
    let mut out_cp: u8 = INITIAL_CODEPOINT;
    let mut state: Mode = Mode::Off;
    task.active();
    loop {
        let vbat_mV = reg.read_vbat_mV();
        if vbat_mV < MIN_BAT_mV || vbat_mV > MAX_BAT_mV {
            state = Mode::Off;
            info!("Power off due to V_bat out-of-range: {}", vbat_mV);
        }

        info!("mode = {:?}", state);
        match state {
            Mode::Off => {
                reg.disable_output().await;
                out_cp = INITIAL_CODEPOINT;
                task.inactive();
                match msgs.recv().await {
                    Some(s) => {
                        task.active();
                        state = s;
                        reg.set_output_dac(out_cp);
                        reg.enable_output().await;
                    }
                    None => {
                        return;
                    }
                }
            }
            Mode::ConstVoltage { setpoint_mV } => {
                let cp = setpoint_mV as u8; // TODO
                out_cp = cp;
                reg.set_output_dac(cp);
                match msgs.recv().await {
                    Some(s) => {
                        state = s;
                    }
                    None => {
                        return;
                    }
                }
            }
            Mode::ConstCurrent { setpoint_mA } => {
                reg.set_output_dac(out_cp);
                const FB_DELAY: Duration = Duration::from_millis(50);
                const STEP: u8 = 1;
                const I_TOL: u32 = 10; //^ milliamps
                loop {
                    let res = futures::select_biased! {
                        s = msgs.recv().fuse() => Some(s),
                        () = Timer::after(FB_DELAY).fuse() => None,
                    };
                    match res {
                        Some(None) => {
                            return;
                        }
                        Some(Some(new_state)) => {
                            state = new_state;
                            break;
                        }
                        None => (),
                    }

                    let isense_mA = reg.read_isense_mA();
                    info!("Isense {} mA / {} mA", isense_mA, setpoint_mA);
                    if isense_mA < setpoint_mA - I_TOL {
                        info!("up {}", out_cp);
                        out_cp -= STEP;
                    } else if isense_mA > setpoint_mA + I_TOL {
                        info!("down {}", out_cp);
                        out_cp += STEP;
                    }
                    reg.set_output_dac(out_cp);

                    let vbat_mV = reg.read_vbat_mV();
                    if vbat_mV < MIN_BAT_mV || vbat_mV > MAX_BAT_mV {
                        break;
                    }
                }
            }
        }
    }
}

static MSGS_CHAN: Forever<mpsc::Channel<kind::CriticalSection, Mode, 1>> = Forever::new();
static BUTTON: Forever<Btn1> = Forever::new();
static LED1: Forever<Led1<'static>> = Forever::new();

type Btn1<'a> = Button<'static, kind::CriticalSection, embassy_stm32::peripherals::PA8>;
type Led1<'a> = gpio::Output<'a, embassy_stm32::peripherals::PA6>;
type Led2<'a> = gpio::Output<'a, embassy_stm32::peripherals::PA7>;

const VOLTAGE_MODES: [Mode; 13] = [
    Mode::Off,
    Mode::from_millivolts(0),
    Mode::from_millivolts(50),
    Mode::from_millivolts(75),
    Mode::from_millivolts(90),
    Mode::from_millivolts(100),
    Mode::from_millivolts(110),
    Mode::from_millivolts(125),
    Mode::from_millivolts(140),
    Mode::from_millivolts(150),
    Mode::from_millivolts(175),
    Mode::from_millivolts(200),
    Mode::from_millivolts(255),
];
const CURRENT_MODES: [Mode; 4] = [
    Mode::Off,
    Mode::from_milliamps(100),
    Mode::from_milliamps(200),
    Mode::from_milliamps(400),
];
const MODES: &[Mode] = &CURRENT_MODES;

fn disable_unused_peripherals() {
    unsafe {
        embassy_stm32::pac::RCC.iopenr().modify(|w| {
            w.set_gpiocen(false);
            w.set_gpioden(false);
        });
        embassy_stm32::pac::RCC.ahbenr().modify(|w| {
            w.set_dmaen(false);
        });
    }
}

#[embassy::main(config = "config()")]
async fn main(spawner: Spawner, p: Peripherals) -> ! {
    info!("Hello World!");
    embassy::time::Timer::after(embassy::time::Duration::from_secs(1)).await;

    disable_unused_peripherals();

    let btn_pin = gpio::Input::new(p.PA8, gpio::Pull::Up);
    let btn_in = exti::ExtiInput::new(btn_pin, p.EXTI8);
    let btn = BUTTON.put(Button::new(btn_in));
    let btn_events = btn.run(&spawner);

    let mut led1: &'static mut Led1 =
        LED1.put(gpio::Output::new(p.PA6, gpio::Level::Low, gpio::Speed::Low));
    let mut led2: Led2 = gpio::Output::new(p.PA7, gpio::Level::Low, gpio::Speed::Low);

    let out_en = gpio::Output::new(p.PA1, gpio::Level::Low, gpio::Speed::Low);

    let adc = embassy_stm32::adc::Adc::new(p.ADC1, &mut Delay);
    let mut dac = embassy_stm32::dac::Dac::new(p.DAC1, p.PA4, gpio::NoPin);
    dac.disable_channel(embassy_stm32::dac::Channel::Ch1);

    let isense_pin = p.PA5;
    let vbat_pin = p.PB1;

    blink_ms(&mut led1, Duration::from_millis(100)).await;
    blink_ms(&mut led2, Duration::from_millis(100)).await;

    let reg = Regulator {
        adc,
        dac,
        isense_pin,
        vbat_pin,
        out_en,
    };
    let msgs_chan: &'static mut mpsc::Channel<kind::CriticalSection, Mode, 1> =
        MSGS_CHAN.put(mpsc::Channel::new());

    let (send, recv) = mpsc::split(msgs_chan);
    unwrap!(spawner.spawn(feedback(recv, reg)));
    active_state::init(&spawner);
    ui(btn_events, send, led1, MODES).await;
}

async fn ui(
    mut btn_events: mpsc::Receiver<'static, kind::CriticalSection, button::ButtonEvent, 5>,
    send: mpsc::Sender<'static, kind::CriticalSection, Mode, 1>,
    led1: &'static mut Led1<'static>,
    modes: &[Mode],
) -> ! {
    let mut i: usize = 0;
    loop {
        let mode = modes[i % modes.len()];
        info!("ui mode: {:?}", mode);
        blink_ms(led1, Duration::from_millis(50)).await;
        unwrap!(send.send(mode).await);
        btn_events.recv().await;
        i += 1;
    }
}

static COUNT: AtomicUsize = AtomicUsize::new(0);
defmt::timestamp!("{=usize}", {
    // NOTE(no-CAS) `timestamps` runs with interrupts disabled
    let n = COUNT.load(Ordering::Relaxed);
    COUNT.store(n + 1, Ordering::Relaxed);
    n
});
