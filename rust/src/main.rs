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
    debug_uart: hal::serial::Serial<
        hal::stm32::USART1,
        hal::gpio::gpioa::PA2<hal::gpio::Alternate<hal::gpio::AF1>>,
        hal::gpio::gpioa::PA3<hal::gpio::Alternate<hal::gpio::AF1>>>,

}

fn oversample_adc<Pin>(adc: &mut hal::adc::Adc, channel: &mut Pin, samples: u8) -> u16
    where Pin: embedded_hal::adc::Channel<hal::adc::Adc, ID=u8>
{
    let mut accum: u32 = 0;
    for i in 0..samples {
        let x: u16 = adc.read(channel).unwrap();
        accum += x as u32;
    }
    (accum / samples as u32) as u16
}

impl<'a> Regulator<'a> {
    fn run(mut self) {
        self.setpoint_pwm.set_duty(pwm::TimerChannel::Ch1, 0x1000);
        self.setpoint_pwm.enable(pwm::TimerChannel::Ch1);
        self.pi_loop.set_gains(1, 1000);
        self.pi_loop.set_setpoint(2000);
        self.out_en.set_high();

        self.adc.set_sample_time(hal::adc::AdcSampleTime::T_71);
        self.setpoint_pwm.set_duty(pwm::TimerChannel::Ch1, 0xffff - 65272);
        let mut i: u32 = 0;
        let n: u32 = 1;
        let mut xs: [u16; 16] = [0;16];
        let mut output: u16 = 0x7fff;
        loop {
            let isense: u16 = oversample_adc::<hal::gpio::gpioa::PA5<hal::gpio::Analog>>(&mut self.adc, &mut self.isense_pin, 4);
            //let isense: u16 = self.adc.read(&mut self.isense_pin).unwrap();
            xs[i as usize % 16] = isense;
            let response: i32 = self.pi_loop.add_sample(isense);
            let res: i32 = i32::from(output).saturating_sub(response);
            if res > 0xffff {
                output = 0xffff;
            } else if res < 0 {
                output = 0;
            } else {
                output = res as u16;
            }

            let bat_v: u16 = self.adc.read(&mut self.vbat_pin).unwrap();
            self.setpoint_pwm.set_duty(pwm::TimerChannel::Ch1, 0xffff - output);
            if i % (2*n) == 0 {
                self.led1.set_high();
            } else if i % n == 0 {
                self.led1.set_low();
            }
            if true {
                use core::fmt::Write;
                write!(self.debug_uart, "hello vbat {} i {} xs {:?} resp {} isense {} output {}\r\n", bat_v, i, xs, response, isense, output);
                //hprintln!("hello vbat={} i={} xs={:?} resp={} isense={} output={}", bat_v, i, xs, response, isense, output);
            }

            i += 1;

            self.delay.delay_ms(1u16);
        }
    }
}

#[entry]
fn main() -> ! {
    if let Some(cp) = cortex_m::Peripherals::take() {
        if let Some(mut p) = hal::stm32::Peripherals::take() {
            cortex_m::interrupt::free(move |cs| {
                let mut rcc = p.RCC.configure().sysclk(8.mhz()).freeze(&mut p.FLASH);
                let gpioa = p.GPIOA.split(&mut rcc);
                let gpiob = p.GPIOB.split(&mut rcc);

                let setpoint_pin = gpioa.pa4.into_alternate_af4(cs);
                let mut out_en = gpioa.pa1.into_push_pull_output(cs);
                let mut led1 = gpioa.pa6.into_push_pull_output(cs);
                let mut led2 = gpioa.pa7.into_push_pull_output(cs);
                let isense_pin = gpioa.pa5.into_analog(cs);
                let vbat_pin = gpiob.pb1.into_analog(cs);
                let button = gpioa.pa9.into_pull_up_input(cs);
                let mut delay = hal::delay::Delay::new(cp.SYST, &rcc);
                let debug_uart_tx = gpioa.pa2.into_alternate_af1(cs);
                let debug_uart_rx = gpioa.pa3.into_alternate_af1(cs);
                let mut debug_uart = hal::serial::Serial::usart1(p.USART1, (debug_uart_tx, debug_uart_rx), 115_200.bps(), &mut rcc);
                led1.set_high();
                led2.set_low();
                let reg = Regulator {
                    out_en: &mut out_en,
                    led1: &mut led1,
                    led2: &mut led2,
                    pi_loop: pi_loop::PILoop::new(),
                    setpoint_pwm: pwm::TimPwm::new(p.TIM14, 10.khz(), &mut rcc),
                    delay: delay,
                    vbat_pin: vbat_pin,
                    isense_pin: isense_pin,
                    adc: hal::adc::Adc::new(p.ADC, &mut rcc),
                    debug_uart: debug_uart,
                };
                reg.run();
            })
        }
    }
    loop {
        continue;
    }
}
