use stm32f0xx_hal as hal;
use stm32f0xx_hal::stm32::{TIM14};
use stm32f0xx_hal::timers::{Timer};
use stm32f0xx_hal::time::Hertz;
use embedded_hal::Pwm;

pub struct TimPwm<T> {
    timer: Timer<T>
}

pub enum TimerChannel {
    Ch1
}

impl TimPwm<TIM14> {
    pub fn new<T>(timer: TIM14, period: T, rcc: &mut hal::rcc::Rcc) -> Self
    where
        T: Into<Hertz>,
    {
        let t = Timer::tim14(timer, period, rcc);
        TimPwm { timer: t }
    }
}

impl Pwm for TimPwm<TIM14> {
    type Channel = TimerChannel;
    type Time = Hertz;
    type Duty = u16;

    fn disable(&mut self, channel: Self::Channel) {
        unsafe {
            let ptr = hal::stm32::TIM14::ptr();
            match channel {
                TimerChannel::Ch1 => {
                    (*ptr).ccmr1_output.modify(|_,w| w.oc1pe().clear_bit());
                    (*ptr).ccer.modify(|_,w| w.cc1e().clear_bit());
                },
            }
        }
    }

    fn enable(&mut self, channel: Self::Channel) {
        unsafe {
            let ptr = hal::stm32::TIM14::ptr();
            match channel {
                TimerChannel::Ch1 => {
                    (*ptr).ccmr1_output.modify(|_,w| w.oc1pe().set_bit().oc1m().bits(6));
                    (*ptr).ccer.modify(|_,w| w.cc1e().set_bit());
                },
            }
        }
    }

    fn get_period(&self) -> Self::Time {
        unsafe {
            let ptr = hal::stm32::TIM14::ptr();
            let period = (*ptr).psc.read().psc().bits();
            Hertz(period.into()) // TODO: FIXME
        }
    }

    fn get_duty(&self, channel: Self::Channel) -> u16 {
        unsafe {
            let ptr = hal::stm32::TIM14::ptr();
            match channel {
                TimerChannel::Ch1 => (*ptr).ccr1.read().ccr().bits()
            }
        }
    }

    fn get_max_duty(&self) -> u16 {
        unsafe {
            let ptr = hal::stm32::TIM14::ptr();
            (*ptr).arr.read().arr().bits()
        }
    }

    fn set_duty(&mut self, channel: Self::Channel, duty: u16) {
        unsafe {
            let ptr = hal::stm32::TIM14::ptr();
            match channel {
                TimerChannel::Ch1 => (*ptr).ccr1.write(|w| w.ccr().bits(duty))
            }
        }
    }

    fn set_period<P: Into<Hertz>>(&mut self, period: P) {
        use crate::stm32f0xx_hal::prelude::_embedded_hal_timer_CountDown;
        self.timer.start(period);
    }
}

