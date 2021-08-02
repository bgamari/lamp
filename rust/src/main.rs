#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![feature(min_type_alias_impl_trait)]
#![feature(impl_trait_in_bindings)]

extern crate cortex_m;
extern crate embedded_hal;

use panic_probe as _;
use defmt_rtt as _;
use defmt::{unwrap, info};

use embassy::util::mpsc;
use embassy::executor::Spawner;
use embassy::time::{Instant, Delay, Duration, Timer};
use embassy_stm32::{rcc, gpio, exti, Peripherals};

use core::sync::atomic::{AtomicUsize, Ordering};

use embedded_hal::digital::v2::OutputPin;


fn oversample_adc<T: embassy_stm32::adc::Instance>(
    adc: &mut embassy_stm32::adc::Adc<T>,
    channel: &mut impl embassy_stm32::adc::AdcPin<T>,
    samples: u8
    ) -> u16
{
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

/*
impl<'a, DebugOutput: core::fmt::Write> Regulator<'a, DebugOutput> {
    fn iterate(&mut self) {
        let isense: u16 = oversample_adc::<hal::gpio::gpioa::PA5<hal::gpio::Analog>>(&mut self.adc, &mut self.isense_pin, 2);
        //let isense: u16 = self.adc.read(&mut self.isense_pin).unwrap();
        match self.active_mode() {
            Mode::ConstCurrent(_setpoint) => {
                let response: i32 = self.pi_loop.add_sample(isense);
                let res: i32 = i32::from(self.output).saturating_sub(response);
                if res > 0xffff {
                    self.output = 0xffff;
                } else if res < 0 {
                    self.output = 0;
                } else {
                    self.output = res as u16;
                }
                self.set_duty(self.output);
                write!(self.debug_uart, "resp {}\r\n", response).unwrap();
            },
            _ => (),
        };

        if true {
            //let bat_v = self.read_batv();
            let bat_v: u16 = self.adc.read(&mut self.vbat_pin).unwrap();
            write!(self.debug_uart, "status vbat {} isense {} output {}\r\n", bat_v / 1000, isense, self.output).unwrap();
            //hprintln!("status vbat={} resp={} isense={} output={}", bat_v, response, isense, self.output).unwrap();
        }
    }
}
*/

pub fn config() -> embassy_stm32::Config {
    let rcc_config = rcc::Config::default();
    //rcc_config.enable_debug_wfe = true;
    embassy_stm32::Config::default().rcc(rcc_config)
}

#[derive(Clone, Copy, Debug)]
enum Mode {
    Off,
    ConstVoltage(u32),
    ConstCurrent(u16),
}

impl Mode {
    pub fn is_off(self) -> bool {
        match self {
            Mode::Off => true,
            _   => false,
        }
    }
}


#[embassy::task]
async fn feedback() -> () {
}

enum ButtonEvent {
    ShortPress, LongPress
}

async fn debounce_button<'d, T, M, const N: usize>(
    btn: &'d mut embassy_stm32::exti::ExtiInput<'d, T>,
    event_chan: mpsc::Sender<'d, M, ButtonEvent, N>)
  where
    M: embassy::util::Mutex<Data = ()>,
    T: gpio::Pin + embedded_hal::digital::v2::InputPin
{
    use crate::embedded_hal::digital::v2::InputPin;
    use embassy_traits::gpio::{WaitForRisingEdge, WaitForFallingEdge};

    loop {
        btn.wait_for_falling_edge();
        let t0 = Instant::now();

        btn.wait_for_rising_edge();
        let t1 = Instant::now();

        let dt = t1 - t0;
        const MIN_PRESS_TIME: Duration = Duration::from_millis(10);
        const MAX_SHORT_PRESS_TIME: Duration = Duration::from_millis(1000);
        const MAX_LONG_PRESS_TIME: Duration = Duration::from_millis(4000);
        if dt < MIN_PRESS_TIME {
            continue;
        } else if dt < MAX_SHORT_PRESS_TIME {
            // N.B. Drop events if full
            let _ = event_chan.try_send(ButtonEvent::ShortPress);
        } else if dt < MAX_LONG_PRESS_TIME {
            let _ = event_chan.try_send(ButtonEvent::LongPress);
        } else {
            continue;
        }
    }
}

#[embassy::main(config="config()")]
async fn main(_spawner: Spawner, p: Peripherals) -> ! {
    info!("Hello World!");

    let mut led1 = gpio::Output::new(p.PA6, gpio::Level::Low, gpio::Speed::Low);
    let mut led2 = gpio::Output::new(p.PA7, gpio::Level::Low, gpio::Speed::Low);
    let mut btn_pin = gpio::Input::new(p.PA8, gpio::Pull::Up);
    let btn = exti::ExtiInput::new(btn_pin, p.EXTI8);

    let mut out_en = gpio::Output::new(p.PA1, gpio::Level::Low, gpio::Speed::Low);
    out_en.set_high().unwrap();

    let mut adc = embassy_stm32::adc::Adc::new(p.ADC1, &mut Delay);
    let mut dac = embassy_stm32::dac::Dac::new(p.DAC1, p.PA4, gpio::NoPin);
    dac.enable_channel(embassy_stm32::dac::Channel::Ch1).unwrap();
    let mut isense_pin = p.PA5;

    led1.set_high().unwrap();
    led2.set_low().unwrap();
    let mut i: u32 = 0;
    loop {
        led1.set_high().unwrap();
        led1.set_low().unwrap();
        i += 1;
        let x = adc.read(&mut isense_pin);
        dac.set(embassy_stm32::dac::Channel::Ch1, embassy_stm32::dac::Value::Bit8((i >> 8) as u8)).unwrap();
        Timer::after(Duration::from_millis(300)).await;
    }
}


static COUNT: AtomicUsize = AtomicUsize::new(0);
defmt::timestamp!("{=usize}", {
    // NOTE(no-CAS) `timestamps` runs with interrupts disabled
    let n = COUNT.load(Ordering::Relaxed);
    COUNT.store(n + 1, Ordering::Relaxed);
    n
});
