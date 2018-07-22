#include <stdint.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/cm3/systick.h>
#include <stdio.h>

#include "pwm.hpp"

#define SETPOINT_TIMER TIM14
#define LED1_TIMER TIM16
#define LED2_TIMER TIM17

// in milliseconds
volatile uint64_t ticks = 0;

class CurrentReg {
  PWM pwm;

public:
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

class Button {
  const uint32_t debounce_ms;
  uint64_t last_press;
  bool state;

public:
  Button(uint32_t debounce_ms)
    : debounce_ms(debounce_ms), last_press(0), state(false)
  {}

  void on_event() {
    bool cur_state = get_state();
    if (cur_state && !state) {
      last_press = ticks;
      state = true;
    } else if (!cur_state && state) {
      state = false;
      if (last_press + debounce_ms < ticks)
        on_press(ticks - last_press);
    }
  }

protected:
  virtual bool get_state();
  virtual void on_press(uint64_t press_dur_ms);
};

class TheButton : public Button {
public:
  TheButton() : Button(20) {}

private:
  bool get_state() {
    return !gpio_get(GPIOA, GPIO9);
  }
  void on_press(uint64_t press_dur_ms) {
    puts("hello\n");
  }
};

CurrentReg current_reg;
PWM led1(LED1_TIMER, TIM_OC1, RCC_TIM16);
PWM led2(LED2_TIMER, TIM_OC1, RCC_TIM17);
TheButton btn;

extern "C" void exti4_15_isr(void) {
  btn.on_event();
  exti_reset_request(EXTI9);
}

extern "C" void sys_tick_handler(void) {
  ticks += 1;
}

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

  if (!systick_set_frequency(1000, rcc_ahb_frequency)) {
    return 1;
  }
  systick_counter_enable();
  systick_interrupt_enable();

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
    }
    gpio_toggle(GPIOA, GPIO6);
  }

  // PA9: BTN
  gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO9);
  exti_select_source(EXTI9, GPIOA);
  exti_set_trigger(EXTI9, EXTI_TRIGGER_BOTH);
  exti_enable_request(EXTI9);

  // PB1: VBAT
  gpio_mode_setup(GPIOB, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO1);

  return 0;
}

