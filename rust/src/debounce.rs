use embedded_hal::digital::v1::InputPin;
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
    fn is_high(&self) -> bool {
        self.pin.is_low()
    }
    fn is_low(&self) -> bool {
        self.pin.is_high()
    }
}

pub enum Event {
    PressStarted,
    PressEnded,
}

pub struct Debounce<Pin, Timer: CountDown> {
    pin: Pin,
    state: State,
    timer: Timer,
    debounce_period: Timer::Time,
}

enum State {
    Idle,
    Activating,
    Activated,
}

impl<Pin: InputPin, Timer: CountDown> Debounce<Pin, Timer> {
    pub fn new<Time>(pin: Pin, timer: Timer, debounce_period: Time) -> Debounce<Pin, Timer>
    where
        Time: Into<Timer::Time>
    {
        Debounce {
            pin: pin,
            state: State::Idle,
            timer: timer,
            debounce_period: debounce_period.into(),
        }
    }

    pub fn update(&mut self) -> Option<Event>
    where
        Timer::Time: Copy
    {
        let s = self.pin.is_high();
        match self.state {
            State::Idle => {
                if s {
                    self.state = State::Activating;
                    self.timer.start(self.debounce_period);
                }
                None
            },
            State::Activating => {
                if s {
                    match self.timer.wait() {
                        Ok(()) => {
                            self.state = State::Activated;
                            Some(Event::PressStarted)
                        }
                        _ => None
                    }
                } else {
                    self.state = State::Idle;
                    None
                }
            },
            State::Activated => {
                if !s {
                    self.state = State::Idle;
                    Some(Event::PressEnded)
                } else {
                    None
                }
            },
        }
    }
}
