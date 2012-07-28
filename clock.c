#include	"clock.h"

/** \file
	\brief Do stuff periodically
*/

#include	"pinio.h"
#include	"sersendf.h"
#include	"dda_queue.h"
#include	"watchdog.h"
#include	"timer.h"
#include	"debug.h"
#include	"serial.h"
#include  "arduino.h"
#include  "pinio.h"
#ifdef	TEMP_INTERCOM
	#include	"intercom.h"
#endif
#include	"memory_barrier.h"

/*!	do stuff every 1/4 second

	called from clock_10ms(), do not call directly
*/
void clock_250ms() {
	ifclock(clock_flag_1s) {
		if (DEBUG_POSITION && (debug_flags & DEBUG_POSITION)) {
			// current position
			update_current_position();
			sersendf_P(PSTR("Pos: %lq,%lq,%lq,%lu\n"), current_position.X, current_position.Y, current_position.Z, current_position.F);

			// target position
			sersendf_P(PSTR("Dst: %lq,%lq,%lq,%lu\n"), movebuffer[mb_tail].endpoint.X, movebuffer[mb_tail].endpoint.Y, movebuffer[mb_tail].endpoint.Z, movebuffer[mb_tail].endpoint.F);

			// Queue
			print_queue();

			// newline
			serial_writechar('\n');
		}
	}
	#ifdef	TEMP_INTERCOM
	start_send();
	#endif
}

/*! do stuff every 10 milliseconds

	call from ifclock(CLOCK_FLAG_10MS) in busy loops

	We could have extended timer.c with a 80ms beacon but we'd rather not put
	extra burden on our timing source AND we also did not want to use the
	hardware to generate the Charge Pump output because we need it for the
	Spindle Speed PWM.
*/

static uint8_t clock_counter_80ms = 0;

void clock_10ms() {
	// reset watchdog
	wd_reset();

	// do quarter-second tasks
	ifclock(clock_flag_250ms) {
		clock_250ms();
	}
}

