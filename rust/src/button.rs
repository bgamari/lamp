use defmt::unwrap;
use embassy::executor::Spawner;
use embassy::time::{Instant, Delay, Duration, Timer};
use embassy_stm32::{gpio, exti};
use embassy::util::mpsc;

pub enum ButtonEvent {
    ShortPress, LongPress
}

type Fut<T: 'static, M: 'static> = impl core::future::Future<Output = !> + Sized + 'static;

pub struct Button<'a, M: 'static + embassy::util::Mutex<Data = ()>, T: 'static + gpio::Pin> {
    pin: exti::ExtiInput<'a, T>,
    chan: mpsc::Channel<M, ButtonEvent, 5>,
    task: embassy::executor::raw::Task<Fut<T,M>>,
}

impl<M: embassy::util::Mutex<Data = ()>, T: gpio::Pin> Button<'static, M, T> {
    pub fn run<'a>(&'static mut self, spawner: &Spawner) -> mpsc::Receiver<'static, M, ButtonEvent, 5> {
        let task = &mut self.task;
        let pin = &mut self.pin;
        let (mut sender, receiver) = mpsc::split(&mut self.chan);
        unwrap!(spawner.spawn(
            task.spawn(async move || {
                debounce_button(pin, &mut sender).await
            })
        ));
        receiver
    }
}

impl<'a, M: embassy::util::Mutex<Data = ()>, T: gpio::Pin> Button<'a, M, T> {
    pub fn new(pin: exti::ExtiInput<'a, T>) -> Self {
        Button {
            chan: mpsc::Channel::new(),
            task: embassy::executor::raw::Task::new(),
            pin
        }
    }
}

async fn debounce_button<'d, T, M, const N: usize>(
    btn: &mut embassy_stm32::exti::ExtiInput<'d, T>,
    event_chan: &mut mpsc::Sender<'d, M, ButtonEvent, N>
    ) -> !
  where
    T: gpio::Pin,
    M: embassy::util::Mutex<Data = ()>,
{
    use embassy_traits::gpio::{WaitForRisingEdge, WaitForFallingEdge};

    loop {
        btn.wait_for_falling_edge().await;
        let t0 = Instant::now();

        btn.wait_for_rising_edge().await;
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

