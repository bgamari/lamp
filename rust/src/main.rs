#![no_main]
#![no_std]

extern crate stm32f0xx_hal;
extern crate cortex_m;
extern crate embedded_hal;

#[allow(unused)]
use panic_halt;

use stm32f0xx_hal as hal;
use stm32f0xx_hal::time::Hertz;
use stm32f0xx_hal::prelude::*;
use embedded_hal::digital::v1::OutputPin;
use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;

mod pi_loop;
mod pwm;

struct Regulator<'a> {
    // Pins
    out_en: &'a mut dyn OutputPin,
    led1: &'a mut dyn OutputPin,
    led2: &'a mut dyn OutputPin,

    // State
    pi_loop: pi_loop::PILoop,
    setpoint_pwm: pwm::TimPwm<hal::stm32::TIM14>,
    delay: hal::delay::Delay,
    vbat_pin: hal::gpio::gpiob::PB1<hal::gpio::Analog>,
    isense_pin: hal::gpio::gpioa::PA5<hal::gpio::Analog>,
    adc: hal::adc::Adc,
}

impl<'a> Regulator<'a> {
    fn run(mut self) {
        self.setpoint_pwm.set_duty(pwm::TimerChannel::Ch1, 0x1000);
        self.setpoint_pwm.enable(pwm::TimerChannel::Ch1);
        self.pi_loop.set_gains(100, 100);
        let mut i = 0;
        let n = 1;
        loop {
            //self.out_en.set_high();
            //self.out_en.set_low();
            let isense: u16 = self.adc.read(&mut self.isense_pin).unwrap();
            self.pi_loop.add_sample(isense);

            if i == n {
                let bat_v: u16 = self.adc.read(&mut self.vbat_pin).unwrap();
                hprintln!("hello {}!", bat_v).unwrap();
                self.led1.set_high();
            } else if i == 2*n {
                i = 0;
                self.led1.set_low();
            }
            i += 1;
            self.delay.delay_ms(1000u16);
        }
    }
}

#[entry]
fn main() -> ! {
    if let Some(mut cp) = cortex_m::Peripherals::take() {
        if let Some(mut p) = hal::stm32::Peripherals::take() {
            cortex_m::interrupt::free(move |cs| {
                let mut rcc = p.RCC.configure().sysclk(8.mhz()).freeze(&mut p.FLASH);
                let gpioa = p.GPIOA.split(&mut rcc);
                let gpiob = p.GPIOB.split(&mut rcc);

                let mut setpoint_pin = gpioa.pa4.into_alternate_af4(cs);
                let mut out_en = gpioa.pa1.into_push_pull_output(cs);
                let mut led1 = gpioa.pa6.into_push_pull_output(cs);
                let mut led2 = gpioa.pa7.into_push_pull_output(cs);
                let mut isense_pin = gpioa.pa5.into_analog(cs);
                let mut vbat_pin = gpiob.pb1.into_analog(cs);
                let mut button = gpioa.pa9.into_pull_up_input(cs);
                led1.set_high();
                led2.set_low();
                let reg = Regulator {
                    out_en: &mut out_en,
                    led1: &mut led1,
                    led2: &mut led2,
                    pi_loop: pi_loop::PILoop::new(),
                    setpoint_pwm: pwm::TimPwm::new(p.TIM14, 1.khz(), &mut rcc),
                    delay: hal::delay::Delay::new(cp.SYST, &rcc),
                    vbat_pin: vbat_pin,
                    isense_pin: isense_pin,
                    adc: hal::adc::Adc::new(p.ADC, &mut rcc),
                };
                reg.run();
            })
        }
    }
    loop {
        continue;
    }
}
