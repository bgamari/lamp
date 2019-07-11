pub struct PILoop {
    setpoint: u16,
    tau: u16,
    integ_error: u32,
    p_gain: u16,
    i_gain: u16
}

impl PILoop {
    pub fn new() -> Self {
        PILoop {
            setpoint: 0,
            tau: 0,
            integ_error: 0,
            p_gain: 0,
            i_gain: 0,
        }
    }

    pub fn set_gains(&mut self, p_gain: u16, i_gain: u16) {
        self.p_gain = p_gain;
        self.i_gain = i_gain;
    }

    pub fn set_setpoint(&mut self, setpoint: u16) {
        self.setpoint = setpoint;
    }

    pub fn set_tau(&mut self, tau: u16) {
        self.tau = tau;
    }

    pub fn add_sample(&mut self, sample: u16) -> u16 {
        let error: u32 = sample as u32 - self.setpoint as u32;
        let max_tau: u32 = 0xffff;
        let tau: u32 = self.tau.into();
        self.integ_error = self.integ_error * tau / max_tau + u32::from(sample) * (max_tau - tau) / max_tau;

        let p_gain: u32 = self.p_gain.into();
        let i_gain: u32 = self.i_gain.into();
        let res: u32 = p_gain * error / 0xffff + i_gain * self.integ_error / 0xffff;
        res as u16
    }
}

