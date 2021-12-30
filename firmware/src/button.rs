use defmt::{debug, unwrap};
use embassy::blocking_mutex::kind::MutexKind;
use embassy::channel::mpsc;
use embassy::executor::Spawner;
use embassy::time::{Duration, Instant};
use embassy_stm32::{exti, gpio};
use crate::active_state;

const MIN_PRESS_TIME: Duration = Duration::from_millis(10);
const MAX_SHORT_PRESS_TIME: Duration = Duration::from_millis(1000);
const MAX_LONG_PRESS_TIME: Duration = Duration::from_millis(4000);

pub enum ButtonEvent {
    ShortPress,
    LongPress,
}

type Fut<T: 'static, M: 'static> = impl core::future::Future<Output = !> + Sized + 'static;

pub struct Button<'a, M: 'static + MutexKind, T: 'static + gpio::Pin> {
    pin: exti::ExtiInput<'a, T>,
    chan: mpsc::Channel<M, ButtonEvent, 5>,
    task: embassy::executor::raw::TaskStorage<Fut<T, M>>,
}

impl<M: MutexKind, T: gpio::Pin> Button<'static, M, T> {
    pub fn run<'a>(
        &'static mut self,
        spawner: &Spawner,
    ) -> mpsc::Receiver<'static, M, ButtonEvent, 5> {
        let task = &mut self.task;
        let pin = &mut self.pin;
        let (mut sender, receiver) = mpsc::split(&mut self.chan);
        unwrap!(
            spawner.spawn(task.spawn(async move || { debounce_button(pin, &mut sender).await }))
        );
        receiver
    }
}

impl<'a, M: MutexKind, T: gpio::Pin> Button<'a, M, T> {
    pub fn new(pin: exti::ExtiInput<'a, T>) -> Self {
        Button {
            chan: mpsc::Channel::new(),
            task: embassy::executor::raw::TaskStorage::new(),
            pin,
        }
    }
}

async fn debounce_button<'d, T, M, const N: usize>(
    btn: &mut embassy_stm32::exti::ExtiInput<'d, T>,
    event_chan: &mut mpsc::Sender<'d, M, ButtonEvent, N>,
) -> !
where
    T: gpio::Pin,
    M: MutexKind,
{
    use embassy_traits::gpio::{WaitForFallingEdge, WaitForRisingEdge};
    let mut active_task = active_state::new_task();

    loop {
        active_task.inactive();
        btn.wait_for_falling_edge().await;
        active_task.active();
        let t0 = Instant::now();

        btn.wait_for_rising_edge().await;
        let t1 = Instant::now();

        let dt = t1 - t0;
        if dt < MIN_PRESS_TIME {
            debug!("too short press {} {}", t0, t1);
            continue;
        } else if dt < MAX_SHORT_PRESS_TIME {
            // N.B. Drop events if full
            let _ = event_chan.try_send(ButtonEvent::ShortPress);
            debug!("short press {} {}", t0, t1);
        } else if dt < MAX_LONG_PRESS_TIME {
            let _ = event_chan.try_send(ButtonEvent::LongPress);
            debug!("long press {} {}", t0, t1);
        } else {
            debug!("too long press {} {}", t0, t1);
            continue;
        }
    }
}
