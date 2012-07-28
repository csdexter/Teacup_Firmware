#include "pinio.h"

char ps_is_on = 0;

/// step/psu timeout
volatile uint8_t  psu_timeout = 0;

void power_on() {
  if (ps_is_on == 0) ps_is_on = 1;

  psu_timeout = 0;
}

void power_off() {
  ps_is_on = 0;
}
