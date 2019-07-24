#![no_main]
#![no_std]

#[macro_use]
extern crate lazy_static;

extern crate stm32f0xx_hal;
extern crate cortex_m;
extern crate embedded_hal;


#[allow(unused)]
use panic_halt;

use stm32f0xx_hal as hal;
use stm32f0xx_hal::prelude::*;
use stm32f0xx_hal::timers::Timer;
use stm32f0xx_hal::stm32::TIM1;

use embedded_hal::digital::v1::OutputPin;

use cortex_m_rt::entry;
use cortex_m::interrupt::Mutex;
use cortex_m_semihosting::hprintln;

use core::fmt::Write;
use core::cell::RefCell;

mod pi_loop;
mod pwm;
mod debounce;

use debounce::Debounce;

const VBAT_MICROVOLT_PER_CODEPOINT: u32 = 4587;

#[derive(Clone, Copy, Debug)]
enum Mode {
    Off,
    ConstDuty(u16),
    ConstCurrent(u16)
}


impl Mode {
    pub fn from_duty(duty: u16) -> Self {
        Mode::ConstDuty(duty)
    }

    pub fn from_current(current: u16) -> Self {
        Mode::ConstCurrent(current)
    }

    pub fn is_off(self) -> bool {
        match self {
            Mode::Off => true,
            _   => false,
        }
    }
}

struct Regulator<'a> {
    // Pins
    out_en: &'a mut dyn OutputPin,
    led1: &'a mut dyn OutputPin,
    led2: &'a mut dyn OutputPin,
    button: Debounce<debounce::InvertedInputPin<hal::gpio::gpioa::PA9<hal::gpio::Input<hal::gpio::PullUp>>>, Timer<TIM1>>,

    // Environment
    modes: &'a [Mode],
    cs: &'a cortex_m::interrupt::CriticalSection,

    // State
    mode_idx: u8,
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
    output: u16,
}

lazy_static! {
    static ref MUTEX_EXTI:  Mutex<RefCell<Option<hal::stm32::EXTI>>>  = Mutex::new(RefCell::new(None));
    static ref MUTEX_SCB:  Mutex<RefCell<Option<cortex_m::peripheral::SCB>>>  = Mutex::new(RefCell::new(None));
}

fn oversample_adc<Pin>(adc: &mut hal::adc::Adc, channel: &mut Pin, samples: u8) -> u16
    where Pin: embedded_hal::adc::Channel<hal::adc::Adc, ID=u8>
{
    let mut accum: u32 = 0;
    for _i in 0..samples {
        let x: u16 = adc.read(channel).unwrap();
        accum += x as u32;
    }
    (accum / samples as u32) as u16
}

fn dump_interrupts() {
    unsafe {
        hprintln!("interrupt mask: {} {}",
                  (*hal::stm32::NVIC::ptr()).ispr[0].read(),
                  (*hal::stm32::NVIC::ptr()).ispr[0].read()).unwrap();
    }
}

#[cfg(feature="nosleep")]
fn deepsleep(cs: &cortex_m::interrupt::CriticalSection) {
    cortex_m::asm::wfi();
}

#[cfg(not(feature="nosleep"))]
fn deepsleep(cs: &cortex_m::interrupt::CriticalSection) {
    let mut scb = MUTEX_SCB.borrow(cs).borrow_mut();
    scb.as_mut().unwrap().set_sleepdeep();
    cortex_m::asm::wfi();
    scb.as_mut().unwrap().clear_sleepdeep();
}

impl<'a> Regulator<'a> {
    fn blink_ms(&mut self, time: u16) {
        self.led1.set_high();
        self.delay.delay_ms(time);
        self.led1.set_low();
    }

    fn active_mode(&self) -> Mode {
        self.modes[self.mode_idx as usize]
    }

    fn advance_mode(&mut self) {
        self.mode_idx = (self.mode_idx + 1) % self.modes.len() as u8;
        let mode = self.active_mode();
        self.set_mode(mode);
        //hprintln!("mode = {:?}", mode).unwrap();
        self.blink_ms(300);
    }

    fn check_button(&mut self) {
        match self.button.update() {
            Some(debounce::Event::PressStarted) => self.advance_mode(),
            _ => (),
        }

        // Clear interrupts
        cortex_m::interrupt::free(|cs| {
            let exti = MUTEX_EXTI.borrow(cs).borrow();
            exti.as_ref().unwrap().pr.modify(|_, w| w.pr9().set_bit());
            hal::stm32::NVIC::unpend(hal::stm32::Interrupt::EXTI4_15);
        });
        //dump_interrupts();
    }

    fn set_duty(&mut self, duty: u16) {
        self.setpoint_pwm.set_duty(pwm::TimerChannel::Ch1, 0xffff - duty);
        write!(self.debug_uart, "duty {}\r\n", duty).unwrap();
    }

    fn set_mode(&mut self, mode: Mode) {
        //self.setpoint_pwm.set_duty(pwm::TimerChannel::Ch1, 0xffff - self.active_mode().setpoint);
        if mode.is_off() {
            self.out_en.set_low();
            self.setpoint_pwm.disable(pwm::TimerChannel::Ch1);
        } else {
            self.out_en.set_high();
            self.setpoint_pwm.enable(pwm::TimerChannel::Ch1);
        }
        match mode {
            Mode::ConstDuty(duty) => self.set_duty(duty),
            Mode::ConstCurrent(setpoint) => self.pi_loop.set_setpoint(setpoint),
            _ => ()
        }
    }

    fn initialize(&mut self) {
        self.pi_loop.set_gains(1, 1000);
        self.pi_loop.set_setpoint(4000);
        self.adc.set_sample_time(hal::adc::AdcSampleTime::T_71);
        self.blink_ms(1000);
        self.set_mode(self.modes[0]);
    }

    fn run(&mut self) {
        self.initialize();
        loop {
            while self.active_mode().is_off() {
                deepsleep(self.cs);
                self.check_button();
            }

            self.check_button();
            self.iterate();
            self.delay.delay_ms(10u16);
        }
    }

    /// In microvolts.
    pub fn read_batv(&mut self) -> u32 {
        let bat_v: u16 = self.adc.read(&mut self.vbat_pin).unwrap();
        bat_v as u32 * VBAT_MICROVOLT_PER_CODEPOINT
    }

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

#[entry]
fn main() -> ! {
    if let Some(mut cp) = cortex_m::Peripherals::take() {
        if let Some(mut p) = hal::stm32::Peripherals::take() {
            cortex_m::interrupt::free(move |cs| {
                let mut rcc = p.RCC.configure().sysclk(8.mhz()).freeze(&mut p.FLASH);
                let gpioa = p.GPIOA.split(&mut rcc);
                let gpiob = p.GPIOB.split(&mut rcc);

                let _setpoint_pin = gpioa.pa4.into_alternate_af4(cs);
                let mut out_en = gpioa.pa1.into_push_pull_output(cs);
                let mut led1 = gpioa.pa6.into_push_pull_output(cs);
                let mut led2 = gpioa.pa7.into_push_pull_output(cs);
                let isense_pin = gpioa.pa5.into_analog(cs);
                let vbat_pin = gpiob.pb1.into_analog(cs);
                let button = gpioa.pa9.into_pull_up_input(cs);
                p.SYSCFG.exticr3.write(|w| w.exti9().pa9());
                p.EXTI.imr.write(|w| w.mr9().set_bit());
                p.EXTI.rtsr.write(|w| w.tr9().set_bit());
                p.EXTI.ftsr.write(|w| w.tr9().set_bit());
                cp.NVIC.enable(hal::stm32::Interrupt::EXTI4_15);
                MUTEX_EXTI.borrow(cs).replace(Some(p.EXTI));
                MUTEX_SCB.borrow(cs).replace(Some(cp.SCB));

                let delay = hal::delay::Delay::new(cp.SYST, &rcc);
                let debug_uart_tx = gpioa.pa2.into_alternate_af1(cs);
                let debug_uart_rx = gpioa.pa3.into_alternate_af1(cs);
                let debug_uart = hal::serial::Serial::usart1(p.USART1, (debug_uart_tx, debug_uart_rx), 115_200.bps(), &mut rcc);
                led1.set_high();
                led2.set_low();

                let modes = [Mode::Off, Mode::from_duty(0xfe00), Mode::from_duty(0xffff)];
                //let modes = [Mode::Off, Mode::from_current(1000), Mode::from_current(2000)];

                let mut reg = Regulator {
                    cs: &cs,
                    out_en: &mut out_en,
                    led1: &mut led1,
                    led2: &mut led2,
                    button: Debounce::new(debounce::InvertedInputPin::new(button), hal::timers::Timer::tim1(p.TIM1, 10.khz(), &mut rcc), 1.khz()),

                    modes: &modes,
                    mode_idx: 0,

                    pi_loop: pi_loop::PILoop::new(),
                    setpoint_pwm: pwm::TimPwm::new(p.TIM14, 10.khz(), &mut rcc),
                    delay: delay,
                    vbat_pin: vbat_pin,
                    isense_pin: isense_pin,
                    adc: hal::adc::Adc::new(p.ADC, &mut rcc),
                    debug_uart: debug_uart,
                    output: 0,
                };
                reg.run();
            })
        }
    }
    loop {
        continue;
    }
}
