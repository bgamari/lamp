#include <stdint.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <stdio.h>

#define SETPOINT_TIMER TIM14
#define LED1_TIMER TIM16
#define LED2_TIMER TIM17

struct PWM {
private:
  uint32_t timer_peripheral;
  enum tim_oc_id oc_id;
  uint16_t duty;

public:
  PWM(uint32_t timer_peripheral,
      enum tim_oc_id oc_id,
      enum rcc_periph_clken clk)
    : timer_peripheral(timer_peripheral), oc_id(oc_id), duty(0)
  {
    rcc_periph_clock_enable(clk);
    timer_set_oc_mode(timer_peripheral, oc_id, TIM_OCM_PWM1);
    timer_set_prescaler(timer_peripheral, 0x0);
    timer_enable_break_main_output(timer_peripheral);
  }

  void set_duty(uint16_t duty) {
    timer_reset(timer_peripheral);
    if (duty > 0) {
      timer_reset(timer_peripheral);
      timer_disable_preload(timer_peripheral);
      timer_direction_up(timer_peripheral);
      timer_continuous_mode(timer_peripheral);
      timer_set_period(timer_peripheral, 0xffff);
      timer_set_oc_value(timer_peripheral, oc_id, duty);
      timer_enable_oc_preload(timer_peripheral, oc_id);
      timer_enable_oc_output(timer_peripheral, oc_id);
      timer_enable_counter(timer_peripheral);
    }
  }
};

class CurrentReg {
public:
  PWM pwm;

  CurrentReg() : pwm(SETPOINT_TIMER, TIM_OC1, RCC_TIM14) {
    pwm.set_duty(0);
  }

  void step() {
    //pwm.set_duty();
  }

private:
  void set_output_enable(bool en) {
    if (en) {
      gpio_set(GPIOA, GPIO1);
    } else {
      gpio_clear(GPIOA, GPIO1);
    }
  }

  void set_duty(uint16_t duty) {
    if (duty == 0) {
      set_output_enable(false);
      pwm.set_duty(0);
    } else {
      pwm.set_duty(0xffff - duty);
      set_output_enable(true);
    }
  }
};

CurrentReg current_reg;
PWM led1(LED1_TIMER, TIM_OC1, RCC_TIM16);
PWM led2(LED2_TIMER, TIM_OC1, RCC_TIM17);

/* For semihosting on newlib */
extern "C" void initialise_monitor_handles(void);

extern "C"
int main(void) {
#if defined(ENABLE_SEMIHOSTING)
	initialise_monitor_handles();
#endif
  rcc_clock_setup_in_hsi_out_48mhz();
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_ADC);

  // PA1: Output enable
  gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO1);

  // PA4: Setpoint output (TIM14_CH1)
  gpio_set_af(GPIOA, GPIO_AF4, GPIO4);
  gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO4);

  // PA5: Current sense (ADC_IN5)
  gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO5);

  // PA6: LED1 (TIM16_CH1)
  gpio_set_af(GPIOA, GPIO_AF5, GPIO6);
  gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO6);

  // PA7: LED2 (TIM17_CH1)
  gpio_set_af(GPIOA, GPIO_AF5, GPIO7);
  gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO7);

  //gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO6);
  while (true) {
    for (int j=0; j < 0xffff; j += 10) {
      for (int i=0; i<5000; i++) __asm__("NOP");
      led1.set_duty(j);
      led2.set_duty(j);
      putchar('.');
      current_reg.pwm.set_duty(j);
    }
    gpio_toggle(GPIOA, GPIO6);
  }

  // PA9: BTN
  gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO9);

  // PB1: VBAT
  gpio_mode_setup(GPIOB, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO1);

  return 0;
}

