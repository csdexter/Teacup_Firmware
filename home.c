#include "home.h"

/** \file
  \brief Homing routines
*/

#include "dda.h"
#include "dda_queue.h"
#include "delay.h"
#include "pinio.h"
#include "gcode_parse.h"

/// home all 3 axes
//TODO: make homing sequence configurable, some designers are braindead enough to need it (Heiz, I'm looking at you!)
void home() {
  #if defined Z_MIN_PIN
    home_z_negative();
  #endif

  #if defined  Y_MIN_PIN
    home_y_negative();
  #endif

  #if defined  X_MIN_PIN
    home_x_negative();
  #endif
}

/// find X MIN endstop
void home_x_negative() {
#if defined X_MIN_PIN
  TARGET t = startpoint;

  t.X = -1000000;
  // hit home hard
  t.F = MAXIMUM_FEEDRATE_X;
  enqueue_home(&t, 0x1, 1);

  // back off slowly
  t.X = +1000000;
  t.F = SEARCH_FEEDRATE_X;
  enqueue_home(&t, 0x1, 0);

  // set X home
  queue_wait(); // we have to wait here, see G92
#ifdef X_MIN
  startpoint.X = next_target.target.X = (int32_t)(X_MIN * 1000.0);
#else
  startpoint.X = next_target.target.X = 0;
#endif
  dda_new_startpoint();
#endif
}

/// find Y MIN endstop
void home_y_negative() {
#if defined Y_MIN_PIN
  TARGET t = startpoint;

  t.Y = -1000000;
  // hit home hard
  t.F = MAXIMUM_FEEDRATE_Y;
  enqueue_home(&t, 0x2, 1);

  // back off slowly
  t.Y = +1000000;
  t.F = SEARCH_FEEDRATE_Y;
  enqueue_home(&t, 0x2, 0);

  // set Y home
  queue_wait();
#ifdef  Y_MIN
  startpoint.Y = next_target.target.Y = (int32_t)(Y_MIN * 1000.);
#else
  startpoint.Y = next_target.target.Y = 0;
#endif
  dda_new_startpoint();
#endif
}

/// find Z MIN endstop
void home_z_negative() {
#if defined Z_MIN_PIN
  TARGET t = startpoint;

  t.Z = -1000000;
  // hit home hard
  t.F = MAXIMUM_FEEDRATE_Z;
  enqueue_home(&t, 0x4, 1);

  // back off slowly
  t.Z = +1000000;
  t.F = SEARCH_FEEDRATE_Z;
  enqueue_home(&t, 0x4, 0);

  // set Z home
  queue_wait();
#ifdef Z_MIN
  startpoint.Z = next_target.target.Z = (int32_t)(Z_MIN * 1000.);
#else
  startpoint.Z = next_target.target.Z = 0;
#endif
  dda_new_startpoint();
#endif
}
