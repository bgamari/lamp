#include <stdint.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/nvic.h>
#include <stdio.h>

#include "pwm.hpp"

#define SETPOINT_TIMER TIM14
#define LED1_TIMER TIM16
#define LED2_TIMER TIM17

const uint16_t max_setpoint = 1200; //3500;

extern PWM led1, led2;

// in milliseconds
volatile uint64_t ticks = 0;

class CurrentReg {
  PWM pwm;
  uint16_t setpoint, duty;

public:
  CurrentReg() : pwm(SETPOINT_TIMER, TIM_OC1, RCC_TIM14), setpoint(0) {
    set_duty(0);
  }

  void set_setpoint(uint16_t s) {
    setpoint = s;
  }

  uint16_t get_setpoint() {
    return setpoint;
  }

  void step(uint16_t isense) {
    if (setpoint == 0) {
      set_duty(0);
      return;
    }

    int32_t scale = 3;
    int32_t delta = (int32_t) isense / scale - (int32_t) setpoint / scale;
    // Wiggle things about a bit
    if (delta == 0)
      delta = 1;
    int32_t next = (int32_t) duty - delta;
    if (next < 0)
      next = 0;
    if (next > 0xffff)
      next = 0xffff;
    set_duty(next);


    //if (isense > setpoint) {
    //  uint16_t next = duty - 0x10;
    //  if (next > duty)
    //    next = 0;
    //  set_duty(next);
    //} else {
    //  uint16_t next = duty + 0x10;
    //  if (next < duty)
    //    next = duty;
    //  set_duty(next);
    //}
  }

private:
  void set_output_enable(bool en) {
    if (en) {
      gpio_set(GPIOA, GPIO1);
    } else {
      gpio_clear(GPIOA, GPIO1);
    }
  }

  public:
  void set_duty(uint16_t duty) {
    this->duty = duty;
    led1.set_duty(duty);
    if (duty == 0) {
      set_output_enable(false);
      pwm.set_duty(0xffff);
    } else {
      pwm.set_duty(0xffff - duty);
      set_output_enable(true);
    }
  }
};

typedef void (*on_press_cb)(uint64_t);
typedef bool (*read_pin_fn)();

bool read_button_pin() {
  return !gpio_get(GPIOA, GPIO9);
}

class Button {
  const uint32_t debounce_ms;
  uint64_t last_press;
  bool state;
  const on_press_cb on_press;
  const read_pin_fn read_pin;

public:
  Button(uint32_t debounce_ms, on_press_cb on_press, read_pin_fn read_pin)
    : debounce_ms(debounce_ms), last_press(0), state(false), on_press(on_press), read_pin(read_pin)
  {}

  void on_event() {
    bool cur_state = read_pin();
    if (cur_state && !state) {
      last_press = ticks;
      state = true;
    } else if (!cur_state && state) {
      state = false;
      if (last_press + debounce_ms < ticks)
        on_press(ticks - last_press);
    }
  }
};

volatile uint16_t last_current = 0;

void on_press(uint64_t press_dur_ms);

CurrentReg current_reg;
PWM led1(LED1_TIMER, TIM_OC1, RCC_TIM16);
PWM led2(LED2_TIMER, TIM_OC1, RCC_TIM17);
Button btn(10, on_press, read_button_pin);

void on_press(uint64_t press_dur_ms) {
  current_reg.set_setpoint(current_reg.get_setpoint() + 0x1000);
}

extern "C" void adc_comp_isr() {
  last_current = adc_read_regular(ADC);
  current_reg.step(last_current);
}

extern "C" void exti4_15_isr(void) {
  //btn.on_event();
  exti_reset_request(EXTI9);
}

extern "C" void sys_tick_handler(void) {
  ticks += 1;
  if (ticks % 10 == 0 && read_button_pin()) {
    uint16_t next = current_reg.get_setpoint() + 0x1;
    if (next > max_setpoint)
      next = 0;
    current_reg.set_setpoint(next);
  }
  adc_start_conversion_regular(ADC);
}

static void init_pins()
{
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

  // PA9: BTN
  gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO9);
  exti_select_source(EXTI9, GPIOA);
  exti_set_trigger(EXTI9, EXTI_TRIGGER_BOTH);
  exti_enable_request(EXTI9);
  nvic_enable_irq(NVIC_EXTI4_15_IRQ);

  // PB1: VBAT
  gpio_mode_setup(GPIOB, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO1);
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

  // Initialize ticker
  if (!systick_set_frequency(1000, rcc_ahb_frequency)) {
    return 1;
  }
  systick_counter_enable();
  systick_interrupt_enable();

  init_pins();

  // Initialize ADC
  {
    adc_power_off(ADC);
    adc_set_clk_source(ADC1, ADC_CLKSOURCE_ADC);
    adc_calibrate(ADC);
    uint8_t channels = { 5 };
    adc_set_regular_sequence(ADC, 1, &channels);
    adc_enable_eoc_interrupt(ADC);
    nvic_enable_irq(NVIC_ADC_COMP_IRQ);
    adc_set_sample_time_on_all_channels(ADC, ADC_SMPTIME_239DOT5);
    adc_power_on(ADC);
    adc_start_conversion_regular(ADC);
  }

  while (true) {
    for (int j=0x7fff; j < 0xffff; j += 10) {
      for (int i=0; i<5000; i++) __asm__("NOP");
      //led1.set_duty(j);
      //current_reg.set_duty(j);
      led2.set_duty(j);
    }
  }

  return 0;
}

