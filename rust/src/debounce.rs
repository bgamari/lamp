use embedded_hal::digital::v2::InputPin;
use embedded_hal::timer::CountDown;

pub struct InvertedInputPin<T> {
    pin: T
}

impl<T> InvertedInputPin<T> {
    pub fn new(pin: T) -> Self {
        InvertedInputPin { pin: pin }
    }
}

impl<T: InputPin> InputPin for InvertedInputPin<T> {
    type Error = <T as InputPin>::Error;
    fn is_high(&self) -> Result<bool, Self::Error> {
        self.pin.is_low()
    }
    fn is_low(&self) -> Result<bool, Self::Error> {
        self.pin.is_high()
    }
}

pub enum Event {
    PressStarted,
    PressEnded,
}

pub struct Debounce<Pin, Timer: CountDown> {
    pin: Pin,
    timer: Timer,
    debounce_period: Timer::Time,
}

impl<Pin: InputPin, Timer: CountDown> Debounce<Pin, Timer> {
    pub fn new<Time>(pin: Pin, timer: Timer, debounce_period: Time) -> Debounce<Pin, Timer>
    where
        Time: Into<Timer::Time>
    {
        Debounce {
            pin: pin,
            timer: timer,
            debounce_period: debounce_period.into(),
        }
    }

    pub fn update(&mut self) -> Option<Event>
    where
        Timer::Time: Copy
    {
        let s = self.pin.is_high().unwrap_or(false);
        if s {
            self.timer.start(self.debounce_period);
            loop {
                match self.timer.wait() {
                    Ok(()) => {
                        break;
                    }
                    _ => continue,
                }
            }
            if s {
                Some(Event::PressStarted)
            } else {
                None
            }
        } else {
            None
        }
    }
}
