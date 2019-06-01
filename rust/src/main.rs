#![no_main]
#![no_std]

extern crate stm32f0xx_hal;
extern crate cortex_m;
extern crate embedded_hal;

#[allow(unused)]
use panic_halt;

use stm32f0xx_hal as hal;
use stm32f0xx_hal::prelude::*;
use embedded_hal::digital::v1::OutputPin;
use cortex_m_rt::entry;

mod pi_loop;

struct Regulator<'a> {
    // Pins
    out_en: &'a mut dyn OutputPin,
    led1: &'a mut dyn OutputPin,
    led2: &'a mut dyn OutputPin,

    // State
    pi_loop: pi_loop::PILoop,
}

impl<'a> Regulator<'a> {
    fn run(mut self) {
        self.pi_loop.set_gains(100, 100);
        loop {
            self.out_en.set_high();
            self.out_en.set_low();
            self.pi_loop.add_sample(40);
        }
    }
}

#[entry]
fn main() -> ! {
    if let Some(mut p) = hal::stm32::Peripherals::take() {
        cortex_m::interrupt::free(move |cs| {
            let mut rcc = p.RCC.configure().sysclk(8.mhz()).freeze(&mut p.FLASH);
            let gpioa = p.GPIOA.split(&mut rcc);
            let gpiob = p.GPIOB.split(&mut rcc);

            let mut out_en = gpioa.pa1.into_push_pull_output(cs);
            let mut led1 = gpioa.pa6.into_push_pull_output(cs);
            let mut led2 = gpioa.pa7.into_push_pull_output(cs);
            let mut current_sense = gpioa.pa5.into_analog(cs);
            let mut vbat_sense = gpiob.pb1.into_analog(cs);
            let mut button = gpioa.pa9.into_pull_up_input(cs);
            led1.set_high();
            led2.set_low();
            let reg = Regulator {
                out_en: &mut out_en,
                led1: &mut led1,
                led2: &mut led2,
                pi_loop: pi_loop::PILoop::new(),
            };
            reg.run();
        })
    }
    loop {
        continue;
    }
}
