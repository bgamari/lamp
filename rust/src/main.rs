#![no_main]
#![no_std]

extern crate stm32f0xx_hal;
extern crate cortex_m;

#[allow(unused)]
use panic_halt;

use stm32f0xx_hal as hal;
use stm32f0xx_hal::prelude::*;
use cortex_m_rt::entry;


#[entry]
fn main() -> ! {
    if let Some(mut p) = hal::stm32::Peripherals::take() {
        cortex_m::interrupt::free(move |cs| {
            let mut rcc = p.RCC.configure().sysclk(8.mhz()).freeze(&mut p.FLASH);
            let gpioa = p.GPIOA.split(&mut rcc);

            let mut out_en = gpioa.pa1.into_push_pull_output(cs);
            let mut led1 = gpioa.pa6.into_push_pull_output(cs);
            let mut led2 = gpioa.pa7.into_push_pull_output(cs);
        })
    }
    loop {
        continue;
    }
}
