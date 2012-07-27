#include	"gcode_process.h"

/** \file
	\brief Work out what to do with received G-Code commands
*/

#include	<string.h>
#include	<avr/interrupt.h>

#include	"gcode_parse.h"

#include	"dda.h"
#include	"dda_queue.h"
#include	"watchdog.h"
#include	"delay.h"
#include	"serial.h"
#include	"sermsg.h"
#include	"timer.h"
#include	"sersendf.h"
#include	"pinio.h"
#include	"debug.h"
#include	"clock.h"
#include	"config.h"
#include	"home.h"

/// the current tool
uint8_t tool;

/// the tool to be changed when we get an M6
uint8_t next_tool;

/************************************************************************//**

  \brief Processes command stored in global \ref next_target.
  This is where we work out what to actually do with each command we
    receive. All data has already been scaled to integers in gcode_process.
    If you want to add support for a new G or M code, this is the place.


*//*************************************************************************/

void process_gcode_command() {
	uint32_t	backup_f;

	// convert relative to absolute
	if (next_target.option_all_relative) {
		next_target.target.X += startpoint.X;
		next_target.target.Y += startpoint.Y;
		next_target.target.Z += startpoint.Z;
	}

	// implement axis limits
	#ifdef	X_MIN
		if (next_target.target.X < X_MIN * 1000.)
			next_target.target.X = X_MIN * 1000.;
	#endif
	#ifdef	X_MAX
		if (next_target.target.X > X_MAX * 1000.)
			next_target.target.X = X_MAX * 1000.;
	#endif
	#ifdef	Y_MIN
		if (next_target.target.Y < Y_MIN * 1000.)
			next_target.target.Y = Y_MIN * 1000.;
	#endif
	#ifdef	Y_MAX
		if (next_target.target.Y > Y_MAX * 1000.)
			next_target.target.Y = Y_MAX * 1000.;
	#endif
	#ifdef	Z_MIN
		if (next_target.target.Z < Z_MIN * 1000.)
			next_target.target.Z = Z_MIN * 1000.;
	#endif
	#ifdef	Z_MAX
		if (next_target.target.Z > Z_MAX * 1000.)
			next_target.target.Z = Z_MAX * 1000.;
	#endif


	// The GCode documentation was taken from http://reprap.org/wiki/Gcode .

	if (next_target.seen_T) {
	    //? --- T: Select Tool ---
	    //?
	    //? Example: T1
	    //?
	    //? Select extruder number 1 to build with.  Extruder numbering starts at 0.

	    next_tool = next_target.T;
	}

	if (next_target.seen_G) {
		uint8_t axisSelected = 0;
		switch (next_target.G) {
			case 0:
				//? G0: Rapid Linear Motion
				//?
				//? Example: G0 X12
				//?
				//? In this case move rapidly to X = 12 mm.  In fact, the RepRap firmware uses exactly the same code for rapid as it uses for controlled moves (see G1 below), as - for the RepRap machine - this is just as efficient as not doing so.  (The distinction comes from some old machine tools that used to move faster if the axes were not driven in a straight line.  For them G0 allowed any movement in space to get to the destination as fast as possible.)
				//?
				backup_f = next_target.target.F;
				next_target.target.F = MAXIMUM_FEEDRATE_X * 2L;
				enqueue(&next_target.target);
				next_target.target.F = backup_f;
				break;

			case 1:
				//? --- G1: Linear Motion at Feed Rate ---
				//?
				//? Example: G1 X90.6 Y13.8 E22.4
				//?
				//? Go in a straight line from the current (X, Y) point to the point (90.6, 13.8), extruding material as the move happens from the current extruded length to a length of 22.4 mm.
				//?
				enqueue(&next_target.target);
				break;

				//	G2 - Arc Clockwise
				// unimplemented

				//	G3 - Arc Counter-clockwise
				// unimplemented

			case 4:
				//? --- G4: Dwell ---
				//?
				//? Example: G4 P200
				//?
				//? In this case sit still doing nothing for 200 milliseconds.  During delays the state of the machine (for example the temperatures of its extruders) will still be preserved and controlled.
				//?
				queue_wait();
				// delay
				if (next_target.seen_P) {
					for (;next_target.P > 0;next_target.P--) {
						ifclock(clock_flag_10ms) {
							clock_10ms();
						}
						delay_ms(1);
					}
				}
				break;

			case 20:
				//? --- G20: Set Units to Inches ---
				//?
				//? Example: G20
				//?
				//? Units from now on are in inches.
				//?
				next_target.option_inches = 1;
				break;

			case 21:
				//? --- G21: Set Units to Millimeters ---
				//?
				//? Example: G21
				//?
				//? Units from now on are in millimeters.  (This is the RepRap default.)
				//?
				next_target.option_inches = 0;
				break;

			case 30:
				//? --- G30: Go home via point ---
				//?
				//? Undocumented.
				enqueue(&next_target.target);
				// no break here, G30 is move and then go home

			case 28:
				//? --- G28: Home ---
				//?
				//? Example: G28
				//?
				//? This causes the RepRap machine to move back to its X, Y and Z zero endstops.  It does so accelerating, so as to get there fast.  But when it arrives it backs off by 1 mm in each direction slowly, then moves back slowly to the stop.  This ensures more accurate positioning.
				//?
				//? If you add coordinates, then just the axes with coordinates specified will be zeroed.  Thus
				//?
				//? G28 X0 Y72.3
				//?
				//? will zero the X and Y axes, but not Z.  The actual coordinate values are ignored.
				//?

				queue_wait();

				if (next_target.seen_X) {
					#if defined	X_MIN_PIN
						home_x_negative();
					#elif defined X_MAX_PIN
						home_x_positive();
					#endif
					axisSelected = 1;
				}
				if (next_target.seen_Y) {
					#if defined	Y_MIN_PIN
						home_y_negative();
					#elif defined Y_MAX_PIN
						home_y_positive();
					#endif
					axisSelected = 1;
				}
				if (next_target.seen_Z) {
					#if defined Z_MAX_PIN
						home_z_positive();
					#elif defined	Z_MIN_PIN
						home_z_negative();
					#endif
					axisSelected = 1;
				}

				if (!axisSelected) {
					home();
				}
				break;

			case 90:
				//? --- G90: Set to Absolute Positioning ---
				//?
				//? Example: G90
				//?
				//? All coordinates from now on are absolute relative to the origin
				//? of the machine. This is the RepRap default.
				//?
				//? If you ever want to switch back and forth between relative and
				//? absolute movement keep in mind, X, Y and Z follow the machine's
				//? coordinate system while E doesn't change it's position in the
				//? coordinate system on relative movements.
				//?

				// No wait_queue() needed.
				next_target.option_all_relative = 0;
				break;

			case 91:
				//? --- G91: Set to Relative Positioning ---
				//?
				//? Example: G91
				//?
				//? All coordinates from now on are relative to the last position.
				//?

				// No wait_queue() needed.
				next_target.option_all_relative = 1;
				break;

			case 92:
				//? --- G92: Set Position ---
				//?
				//? Example: G92 X10 E90
				//?
				//? Allows programming of absolute zero point, by reseting the current position to the values specified.  This would set the machine's X coordinate to 10, and the extrude coordinate to 90. No physical motion will occur.
				//?

				queue_wait();

				if (next_target.seen_X) {
					startpoint.X = next_target.target.X;
					axisSelected = 1;
				}
				if (next_target.seen_Y) {
					startpoint.Y = next_target.target.Y;
					axisSelected = 1;
				}
				if (next_target.seen_Z) {
					startpoint.Z = next_target.target.Z;
					axisSelected = 1;
				}

				if (axisSelected == 0) {
					startpoint.X = next_target.target.X =
					startpoint.Y = next_target.target.Y =
					startpoint.Z = next_target.target.Z = 0;
				}

				dda_new_startpoint();
				break;

			case 161:
				//? --- G161: Home negative ---
				//?
				//? Find the minimum limit of the specified axes by searching for the limit switch.
				//?
				if (next_target.seen_X)
					home_x_negative();
				if (next_target.seen_Y)
					home_y_negative();
				if (next_target.seen_Z)
					home_z_negative();
				break;

			case 162:
				//? --- G162: Home positive ---
				//?
				//? Find the maximum limit of the specified axes by searching for the limit switch.
				//?
				if (next_target.seen_X)
					home_x_positive();
				if (next_target.seen_Y)
					home_y_positive();
				if (next_target.seen_Z)
					home_z_positive();
				break;

				// unknown gcode: spit an error
			default:
				sersendf_P(PSTR("E: Bad G-code %d"), next_target.G);
				// newline is sent from gcode_parse after we return
				return;
		}
		#ifdef	DEBUG
			if (DEBUG_POSITION && (debug_flags & DEBUG_POSITION))
				print_queue();
		#endif
	}
	else if (next_target.seen_M) {
		switch (next_target.M) {
			case 0:
				//? --- M0: machine stop ---
				//?
				//? Example: M0
				//?
				//? http://linuxcnc.org/handbook/RS274NGC_3/RS274NGC_33a.html#1002379
				//? Unimplemented, especially the restart after the stop. Fall trough to M2.
				//?

			case 2:
				//? --- M2: program end ---
				//?
				//? Example: M2
				//?
				//? http://linuxcnc.org/handbook/RS274NGC_3/RS274NGC_33a.html#1002379
				//?
				queue_wait();
				power_off();
				break;

			case 112:
				//? --- M112: Emergency Stop ---
				//?
				//? Example: M112
				//?
				//? Any moves in progress are immediately terminated, then RepRap shuts down.  All motors and heaters are turned off.
				//? It can be started again by pressing the reset button on the master microcontroller.  See also M0.
				//?

				timer_stop();
				queue_flush();
				power_off();
				cli();
				for (;;)
					wd_reset();
				break;

			case 6:
				//? --- M6: tool change ---
				//?
				//? Undocumented.
				tool = next_tool;
				break;

			// M3/M101- extruder on
			case 3:
			//TODO: insert spindle on here
				break;

			// M102- extruder reverse

			// M5/M103- extruder off
			case 5:
			//TODO: insert spindle off here
				break;

			case 7:
			//TOOD: insert cooling, flood on here
				//? --- M106: Fan On ---
				//?
				//? Example: M106
				//?
				//? Turn on the cooling fan (if any).
				//?

				#ifdef ENFORCE_ORDER
					// wait for all moves to complete
					queue_wait();
				#endif
				break;

			case 9:
			//TODO: insert cooling off here
				//? --- M107: Fan Off ---
				//?
				//? Example: M107
				//?
				//? Turn off the cooling fan (if any).
				//?

				#ifdef ENFORCE_ORDER
					// wait for all moves to complete
					queue_wait();
				#endif
				break;

			case 110:
				//? --- M110: Set Current Line Number ---
				//?
				//? Example: N123 M110
				//?
				//? Set the current line number to 123.  Thus the expected next line after this command will be 124.
				//? This is a no-op in Teacup.
				//?
				break;

			#ifdef	DEBUG
			case 111:
				//? --- M111: Set Debug Level ---
				//?
				//? Example: M111 S6
				//?
				//? Set the level of debugging information transmitted back to the host to level 6.  The level is the OR of three bits:
				//?
				//? <Pre>
				//? #define         DEBUG_PID       1
				//? #define         DEBUG_DDA       2
				//? #define         DEBUG_POSITION  4
				//? </pre>
				//?
				//? This command is only available in DEBUG builds of Teacup.

				if ( ! next_target.seen_S)
					break;
				debug_flags = next_target.S;
				break;
			#endif

			case 114:
				//? --- M114: Get Current Position ---
				//?
				//? Example: M114
				//?
				//? This causes the RepRap machine to report its current X, Y, Z and E coordinates to the host.
				//?
				//? For example, the machine returns a string such as:
				//?
				//? <tt>ok C: X:0.00 Y:0.00 Z:0.00 E:0.00</tt>
				//?
				#ifdef ENFORCE_ORDER
					// wait for all moves to complete
					queue_wait();
				#endif
				update_current_position();
				sersendf_P(PSTR("X:%lq,Y:%lq,Z:%lq,F:%ld"), current_position.X, current_position.Y, current_position.Z, current_position.F);
				// newline is sent from gcode_parse after we return
				break;

			case 115:
				//? --- M115: Get Firmware Version and Capabilities ---
				//?
				//? Example: M115
				//?
				//? Request the Firmware Version and Capabilities of the current microcontroller
				//? The details are returned to the host computer as key:value pairs separated by spaces and terminated with a linefeed.
				//?
				//? sample data from firmware:
				//?  FIRMWARE_NAME:Teacup FIRMWARE_URL:http%%3A//github.com/triffid/Teacup_Firmware/ PROTOCOL_VERSION:1.0 MACHINE_TYPE:Mendel EXTRUDER_COUNT:1 TEMP_SENSOR_COUNT:1 HEATER_COUNT:1
				//?

				sersendf_P(PSTR("FIRMWARE_NAME:Teacup FIRMWARE_URL:http%%3A//github.com/triffid/Teacup_Firmware/ PROTOCOL_VERSION:1.0 MACHINE_TYPE:Mendel"));
				// newline is sent from gcode_parse after we return
				break;

			case 116:
				//? --- M116: Wait ---
				//?
				//? Example: M116
				//?
				//? Wait for ''all'' temperatures and other slowly-changing variables to arrive at their set values.  See also M109.

				enqueue(NULL);
				break;

			case 200:
				//? --- M200: report endstop status ---
				//? Report the current status of the endstops configured in the firmware to the host.
				power_on();
				#if defined(X_MIN_PIN)
					sersendf_P(PSTR("x_min:%d "), x_min());
				#endif
				#if defined(X_MAX_PIN)
					sersendf_P(PSTR("x_max:%d "), x_max());
				#endif
				#if defined(Y_MIN_PIN)
					sersendf_P(PSTR("y_min:%d "), y_min());
				#endif
				#if defined(Y_MAX_PIN)
					sersendf_P(PSTR("y_max:%d "), y_max());
				#endif
				#if defined(Z_MIN_PIN)
					sersendf_P(PSTR("z_min:%d "), z_min());
				#endif
				#if defined(Z_MAX_PIN)
					sersendf_P(PSTR("z_max:%d "), z_max());
				#endif
				#if !(defined(X_MIN_PIN) || defined(X_MAX_PIN) || defined(Y_MIN_PIN) || defined(Y_MAX_PIN) || defined(Z_MIN_PIN) || defined(Z_MAX_PIN))
					sersendf_P(PSTR("no endstops defined"));
				#endif
				break;

			#ifdef	DEBUG
			case 240:
				//? --- M240: echo off ---
				//? Disable echo.
				//? This command is only available in DEBUG builds.
				debug_flags &= ~DEBUG_ECHO;
				serial_writestr_P(PSTR("Echo off"));
				// newline is sent from gcode_parse after we return
				break;

			case 241:
				//? --- M241: echo on ---
				//? Enable echo.
				//? This command is only available in DEBUG builds.
				debug_flags |= DEBUG_ECHO;
				serial_writestr_P(PSTR("Echo on"));
				// newline is sent from gcode_parse after we return
				break;

			case 250:
				//? --- M250: return current position, end position, queue ---
				//? Undocumented
				//? This command is only available in DEBUG builds.
				update_current_position();
				sersendf_P(PSTR("{X:%ld,Y:%ld,Z:%ld,E:%ld,F:%lu,c:%lu}\t{X:%ld,Y:%ld,Z:%ld,F:%lu,c:%lu}\t"), current_position.X, current_position.Y, current_position.Z, current_position.E, current_position.F, movebuffer[mb_tail].c, movebuffer[mb_tail].endpoint.X, movebuffer[mb_tail].endpoint.Y, movebuffer[mb_tail].endpoint.Z, movebuffer[mb_tail].endpoint.E, movebuffer[mb_tail].endpoint.F,
					#ifdef ACCELERATION_REPRAP
						movebuffer[mb_tail].end_c
					#else
						movebuffer[mb_tail].c
					#endif
					);

				print_queue();
				break;

			case 253:
				//? --- M253: read arbitrary memory location ---
				//? Undocumented
				//? This command is only available in DEBUG builds.
				if ( ! next_target.seen_S)
					break;
				if ( ! next_target.seen_P)
					next_target.P = 1;
				for (; next_target.P; next_target.P--) {
					serwrite_hex8(*(volatile uint8_t *)(next_target.S));
					next_target.S++;
				}
				// newline is sent from gcode_parse after we return
				break;

			case 254:
				//? --- M254: write arbitrary memory location ---
				//? Undocumented
				//? This command is only available in DEBUG builds.
				if ( ! next_target.seen_S || ! next_target.seen_P)
					break;
				sersendf_P(PSTR("%x:%x->%x"), next_target.S, *(volatile uint8_t *)(next_target.S), next_target.P);
				(*(volatile uint8_t *)(next_target.S)) = next_target.P;
				// newline is sent from gcode_parse after we return
				break;
			#endif /* DEBUG */

				// unknown mcode: spit an error
			default:
				sersendf_P(PSTR("E: Bad M-code %d"), next_target.M);
				// newline is sent from gcode_parse after we return
		} // switch (next_target.M)
	} // else if (next_target.seen_M)
} // process_gcode_command()

