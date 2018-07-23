#pragma once

#include <stdint.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/rcc.h>

class PWM {
private:
  uint32_t timer_peripheral;
  enum tim_oc_id oc_id;
  uint16_t duty;
  void start();

public:
  PWM(uint32_t timer_peripheral,
      enum tim_oc_id oc_id,
      enum rcc_periph_clken clk);

  void set_duty(uint16_t duty);
};
