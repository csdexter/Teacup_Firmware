/** \file
  \brief Main file - this is where it all starts, and ends
*/

/** \mainpage Teacup Reprap Firmware
  \section intro_sec Introduction
    Teacup Reprap Firmware (originally named FiveD on Arduino) is a firmware package for numerous reprap electronics sets.

    Please see README for a full introduction and long-winded waffle about this project
  \section install_sec  Installation
    \subsection step1 Step 1: Download
      \code git clone git://github.com/triffid/Teacup_Firmware \endcode
    \subsection step2 Step 2: configure
      \code cp config.[yourboardhere].h config.h \endcode
      Edit config.h to suit your machone
      Edit Makefile to select the correct chip and programming settings
    \subsection step3 Step 3: Compile
      \code make \endcode
      \code make program \endcode
    \subsection step4 Step 4: Test!
      \code ./func.sh mendel_reset
      ./func.sh mendel_talk
      M115
      ctrl+d \endcode
*/

#include <avr/io.h>
#include <avr/interrupt.h>

#include "config.h"
#include "fuses.h"

#include "serial.h"
#include "dda_queue.h"
#include "dda.h"
#include "gcode_parse.h"
#include "timer.h"
#include "sermsg.h"
#include "watchdog.h"
#include "debug.h"
#include "sersendf.h"
#include "pinio.h"
#include "arduino.h"
#include "clock.h"
#include "intercom.h"

/// initialise all I/O - set pins as input or output, turn off unused subsystems, etc
void io_init(void) {
  // disable modules we don't use
#ifdef PRR
  PRR = _BV(PRTWI) | _BV(PRADC) | _BV(PRSPI);
#elif defined PRR0
  PRR0 = _BV(PRTWI) | _BV(PRADC) | _BV(PRSPI);
#  if defined(PRUSART3)
  // don't use USART2 or USART3- leave USART1 for GEN3 and derivatives
  PRR1 |= _BV(PRUSART3) | _BV(PRUSART2);
#  endif
#  if defined(PRUSART2)
  // don't use USART2 or USART3- leave USART1 for GEN3 and derivatives
  PRR1 |= _BV(PRUSART2);
#  endif
#endif
  ACSR = _BV(ACD);

  // setup I/O pins
  // X Stepper
  WRITE(X_STEP_PIN, 0);  SET_OUTPUT(X_STEP_PIN);
  WRITE(X_DIR_PIN,  0);  SET_OUTPUT(X_DIR_PIN);
  #ifdef X_MIN_PIN
    SET_INPUT(X_MIN_PIN);
      WRITE(X_MIN_PIN, 0);
  #endif

  // Y Stepper
  WRITE(Y_STEP_PIN, 0);  SET_OUTPUT(Y_STEP_PIN);
  WRITE(Y_DIR_PIN,  0);  SET_OUTPUT(Y_DIR_PIN);
  #ifdef Y_MIN_PIN
    SET_INPUT(Y_MIN_PIN);
      WRITE(Y_MIN_PIN, 0);
  #endif

  // Z Stepper
  #if defined Z_STEP_PIN && defined Z_DIR_PIN
    WRITE(Z_STEP_PIN, 0);  SET_OUTPUT(Z_STEP_PIN);
    WRITE(Z_DIR_PIN,  0);  SET_OUTPUT(Z_DIR_PIN);
  #endif
  #ifdef Z_MIN_PIN
    SET_INPUT(Z_MIN_PIN);
      WRITE(Z_MIN_PIN, 0);
  #endif
  
  // Charge Pump
  WRITE(CHARGEPUMP_PIN, 0); SET_OUTPUT(CHARGEPUMP_PIN);
  // E-Stop Input
  SET_INPUT(ESTOP_IN_PIN);

  // setup PWM timers: fast PWM, no prescaler
  TCCR0A = _BV(WGM01) | _BV(WGM00);
  // PWM frequencies in TCCR0B, see page 108 of the ATmega644 reference.
  TCCR0B = _BV(CS00); // F_CPU / 256 (about 78(62.5) kHz on a 20(16) MHz chip)
  //TCCR0B = _BV(CS01);              // F_CPU / 256 / 8  (about 9.8(7.8) kHz)
  //TCCR0B = _BV(CS00) | _BV(CS01); // F_CPU / 256 / 64  (about 1220(977) Hz)
  //TCCR0B = _BV(CS02);              // F_CPU / 256 / 256  (about 305(244) Hz)
#ifndef FAST_PWM
  TCCR0B = _BV(CS00) | _BV(CS02); // F_CPU / 256 / 1024  (about 76(61) Hz)
#endif
  TIMSK0 = 0;
  OCR0A = 0;
  OCR0B = 0;

#ifdef  TCCR3A
  TCCR3A = _BV(WGM30);
  TCCR3B = _BV(WGM32) | _BV(CS30);
  TIMSK3 = 0;
  OCR3A = 0;
  OCR3B = 0;
#endif

#ifdef  TCCR4A
  TCCR4A = _BV(WGM40);
  TCCR4B = _BV(WGM42) | _BV(CS40);
  TIMSK4 = 0;
  OCR4A = 0;
  OCR4B = 0;
#endif

#ifdef  TCCR5A
  TCCR5A = _BV(WGM50);
  TCCR5B = _BV(WGM52) | _BV(CS50);
  TIMSK5 = 0;
  OCR5A = 0;
  OCR5B = 0;
#endif

#ifdef TEMP_INTERCOM
  // Enable the RS485 transceiver
  SET_OUTPUT(RX_ENABLE_PIN);
  SET_OUTPUT(TX_ENABLE_PIN);
  WRITE(RX_ENABLE_PIN,0);
  disable_transmit();
#endif
}

/// Startup code, run when we come out of reset
void init(void) {
  // set up watchdog
  wd_init();

  // set up serial
  serial_init();

  // set up G-code parsing
  gcode_init();

  // set up inputs and outputs
  io_init();

  // set up timers
  timer_init();

  // set up dda
  dda_init();

  // enable interrupts
  sei();

  // reset watchdog
  wd_reset();

  // say hi to host
  serial_writestr_P(PSTR("start\nok\n"));

}

/// this is where it all starts, and ends
///
/// just run init(), then run an endless loop where we pass characters from the serial RX buffer to gcode_parse_char() and check the clocks
int main (void)
{
  init();

  // main loop
  for (;;)
  {
    // if queue is full, no point in reading chars- host will just have to wait
    if ((serial_rxchars() != 0) && (queue_full() == 0)) {
      uint8_t c = serial_popchar();
      gcode_parse_char(c);
    }

    ifclock(clock_flag_10ms) {
      clock_10ms();
    }                
  }
}
