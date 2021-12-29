use core::cell::{RefCell};
use embassy::channel::signal::Signal;
use embassy::blocking_mutex::{CriticalSectionMutex, Mutex};
use embassy::time::Duration;
use defmt::debug;

pub struct Task {
    active: bool,
    active_state: &'static ActiveState,
}

impl Drop for Task {
    fn drop(&mut self) {
        self.inactive();
    }
}

impl Task {
    pub fn active(&mut self) {
        if ! self.active {
            self.active_state.add_active();
            self.active = true;
        }
    }

    pub fn inactive(&mut self) {
        if self.active {
            self.active_state.sub_active();
            self.active = false;
        }
    }
}

struct ActiveStateInner {
    active_count: u32,
    seq_n: u32,
}

impl ActiveStateInner {
    const fn new() -> Self {
        ActiveStateInner {
            active_count: 0,
            seq_n: 0,
        }
    }
}

static ACTIVE_STATE: ActiveState = ActiveState::new();

pub fn init(spawner: &embassy::executor::Spawner) {
    ACTIVE_STATE.start(spawner);
}

pub fn new_task() -> Task {
    ACTIVE_STATE.new_task()
}

struct ActiveState {
    inner: CriticalSectionMutex<RefCell<ActiveStateInner>>,
    signal: Signal<()>,
}

impl ActiveState {
    const fn new() -> Self {
        let signal = Signal::new();
        ActiveState {
            inner: CriticalSectionMutex::new(RefCell::new(ActiveStateInner::new())),
            signal,
        }
    }
    
    pub fn new_task(&'static self) -> Task {
        let task = Task {
            active: true,
            active_state: self
        };
        self.add_active();
        task
    }

    fn start(&'static self, spawner: &embassy::executor::Spawner) {
        defmt::unwrap!(spawner.spawn(ActiveState::try_suspend(&self)));
    }

    fn is_active(&self) -> bool {
        let count = self.inner.lock(|inner| inner.borrow().active_count);
        count > 0
    }

    fn add_active(&self) {
        self.inner.lock(|inner| {
            inner.borrow_mut().active_count += 1;
            inner.borrow_mut().seq_n += 1;
        });
    }

    fn sub_active(&self) {
        let n = self.inner.lock(|inner| {
            inner.borrow_mut().active_count -= 1;
            inner.borrow_mut().seq_n += 1;
            inner.borrow().active_count
        });
        if n == 0 {
            self.signal.signal(());
        }
    }

    #[embassy::task]
    async fn try_suspend(s: &'static ActiveState) {
        const TRY_SUSPEND_TIMEOUT: Duration = Duration::from_secs(5);
        loop {
            embassy::time::Timer::after(TRY_SUSPEND_TIMEOUT).await;
            if ! s.is_active() {
                debug!("suspending");
                embassy::time::Timer::after(embassy::time::Duration::from_millis(100)).await;
                suspend().await;
            } else {
                debug!("active");
            }
        }
    }
}

const DO_SUSPEND: bool = ! cfg!(feature="no_sleep");

pub async fn suspend() {
    unsafe {
        debug!("suspend");
        if DO_SUSPEND {
            embassy_stm32::pac::PWR.cr1().modify(|w| w.set_lpms(0x1));
            let mut cp = cortex_m::peripheral::Peripherals::steal();
            cp.SCB.set_sleepdeep();
            cortex_m::asm::wfi();
            cp.SCB.clear_sleepdeep();
            embassy_stm32::pac::PWR.cr1().modify(|w| w.set_lpms(0x0));
        } else {
            cortex_m::asm::wfi();
        }
        debug!("resume");
    }
}

