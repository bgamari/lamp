#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![feature(inherent_associated_types)]
#![feature(never_type)]
#![feature(async_closure)]

extern crate cortex_m;
extern crate embedded_hal;

use panic_probe as _;
use defmt_rtt as _;
use defmt::{unwrap, info};

use embassy::util::mpsc;
use embassy::util::Forever;
use embassy::executor::Spawner;
use embassy::time::{Instant, Delay, Duration, Timer};
use embassy_stm32::{rcc, gpio, exti, Peripherals};

use core::sync::atomic::{AtomicUsize, Ordering};

use embedded_hal::digital::v2::OutputPin;

mod button;
use crate::button::{Button, ButtonEvent};

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
    //rcc_config.enable_debug_wfe = true;
    embassy_stm32::Config::default()
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

static BUTTON: Forever<Button<'static, embassy::util::CriticalSectionMutex<()>, embassy_stm32::peripherals::PA8>> = Forever::new();

#[embassy::main(config="config()")]
async fn main(spawner: Spawner, p: Peripherals) -> ! {
    info!("Hello World!");

    let mut led1 = gpio::Output::new(p.PA6, gpio::Level::Low, gpio::Speed::Low);
    let mut led2 = gpio::Output::new(p.PA7, gpio::Level::Low, gpio::Speed::Low);
    let btn_pin = gpio::Input::new(p.PA8, gpio::Pull::Up);
    let btn_in = exti::ExtiInput::new(btn_pin, p.EXTI8);
    let btn = BUTTON.put(Button::new(btn_in));
    let mut btn_events = btn.run(&spawner);

    let mut out_en = gpio::Output::new(p.PA1, gpio::Level::Low, gpio::Speed::Low);
    out_en.set_high().unwrap();

    let mut adc = embassy_stm32::adc::Adc::new(p.ADC1, &mut Delay);
    let mut dac = embassy_stm32::dac::Dac::new(p.DAC1, p.PA4, gpio::NoPin);
    dac.enable_channel(embassy_stm32::dac::Channel::Ch1).unwrap();
    let mut isense_pin = p.PA5;

    led1.set_high().unwrap();
    led2.set_high().unwrap();
    //led2.set_low().unwrap();
    let mut i: usize = 0;

    const MODES: [u8; 12] = [0, 50, 75, 90, 100, 110, 125, 140, 150, 175, 200, 255];
    loop {
        led1.set_high().unwrap();
        led2.set_high().unwrap();
        let delay = Duration::from_millis(100);
        Timer::after(delay).await;

        led1.set_low().unwrap();
        led2.set_low().unwrap();
        btn_events.recv().await;
        i += 1;
        let x = adc.read(&mut isense_pin);
        let y = MODES[i % MODES.len()];
        info!("hi adc={}, dac={}", x, y);
        let y = embassy_stm32::dac::Value::Bit8(y);
        dac.set(embassy_stm32::dac::Channel::Ch1, y).unwrap();
        Timer::after(delay).await;
    }
}

static COUNT: AtomicUsize = AtomicUsize::new(0);
defmt::timestamp!("{=usize}", {
    // NOTE(no-CAS) `timestamps` runs with interrupts disabled
    let n = COUNT.load(Ordering::Relaxed);
    COUNT.store(n + 1, Ordering::Relaxed);
    n
});
