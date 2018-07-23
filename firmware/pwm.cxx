#include "pwm.hpp"

PWM::PWM(uint32_t timer_peripheral,
    enum tim_oc_id oc_id,
    enum rcc_periph_clken clk)
  : timer_peripheral(timer_peripheral), oc_id(oc_id), duty(0)
{
  rcc_periph_clock_enable(clk);
  timer_reset(timer_peripheral);
}

void PWM::start() {
  timer_reset(timer_peripheral);
  timer_set_oc_mode(timer_peripheral, oc_id, TIM_OCM_PWM1);
  timer_set_prescaler(timer_peripheral, 0x0);
  timer_enable_break_main_output(timer_peripheral);
  timer_disable_preload(timer_peripheral);
  timer_direction_up(timer_peripheral);
  timer_continuous_mode(timer_peripheral);
  timer_set_period(timer_peripheral, 0xffff);
  timer_enable_oc_preload(timer_peripheral, oc_id);
  timer_enable_oc_output(timer_peripheral, oc_id);
  timer_enable_counter(timer_peripheral);
}

void PWM::set_duty(uint16_t duty) {
  if (duty > 0) {
    if (this->duty == 0)
      start();
    timer_set_oc_value(timer_peripheral, oc_id, duty);
  } else {
    timer_reset(timer_peripheral);
  }
  this->duty = duty;
}
