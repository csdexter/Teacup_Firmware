#include "clock.h"

/** \file
    \brief Do stuff periodically
*/

#include "sersendf.h"
#include "dda_queue.h"
#include "watchdog.h"
#include "timer.h"
#include "debug.h"
#include "serial.h"
#include "pinio.h"
#include "arduino.h"
#ifdef TEMP_INTERCOM
#  include "intercom.h"
#endif
#include "memory_barrier.h"

/*!  do stuff every 1/4 second

  called from clock_10ms(), do not call directly
*/
void clock_250ms() {
  ifclock(clock_flag_1s) {
    if (DEBUG_POSITION && (debug_flags & DEBUG_POSITION)) {
      // current position
      update_current_position();
      sersendf_P(PSTR("Pos: %lq,%lq,%lq,%lu\n"), current_position.X,
                 current_position.Y, current_position.Z, current_position.F);

      // target position
      sersendf_P(PSTR("Dst: %lq,%lq,%lq,%lu\n"), movebuffer[mb_tail].endpoint.X,
                 movebuffer[mb_tail].endpoint.Y, movebuffer[mb_tail].endpoint.Z,
                 movebuffer[mb_tail].endpoint.F);

      // Queue
      print_queue();

      // newline
      serial_writechar('\n');
    }
  }
#ifdef TEMP_INTERCOM
  start_send();
#endif
}

/*! do stuff every 10 milliseconds

  call from ifclock(clock_flag_10ms) in busy loops

*/
void clock_10ms() {
  //Check if E-Stop has been hit
  //TODO: come up with a better scenario for when we'll be running on two MCUs
  if(estop_hit()) {
    timer_stop();
    queue_flush();
    cli();
    for (;;) wd_reset();
  }

  // reset watchdog
  wd_reset();

  // do quarter-second tasks
  ifclock(clock_flag_250ms) {
    clock_250ms();
  }
}
