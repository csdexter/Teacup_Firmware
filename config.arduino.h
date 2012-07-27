/* Notice to developers: this file is intentionally included twice. */

/** \file
	\brief Teacup configuration for the CNC Control Panel project

	\note this is configured for an Arduino Uno talking to a Heiz Zero3 controller connected to a Heiz High-Z S-720 CNC mill.
*/

/*
	CONTENTS

	1. Mechanical/Hardware
	2. Acceleration settings
	3. Pinouts
	4. Communication options
	5. Miscellaneous
	6. Appendix A - PWMable pins and mappings
*/


/***************************************************************************\
*                                                                           *
* 1. MECHANICAL/HARDWARE                                                    *
*                                                                           *
\***************************************************************************/

/*
	Set your microcontroller type in Makefile! atmega168/atmega328p/atmega644p/atmega1280

	If you want to port this to a new chip, start off with arduino.h and see how you go.
*/

/** \def F_CPU
	CPU clock rate
*/
#ifndef	F_CPU
	#define	F_CPU	16000000UL
#endif

/** \def HOST
	This is the motherboard, as opposed to the extruder. See extruder/ directory for GEN3 extruder firmware
*/
#define	HOST

/*
	Values reflecting the gearing of your machine.
		All numbers are fixed point integers, so no more than 3 digits to the right of the decimal point, please :-)
*/

/** \def STEPS_PER_X
	steps per meter ( = steps per mm * 1000 )

	calculate these values appropriate for your machine

	for threaded rods, this is
		(steps motor per turn) / (pitch of the thread) * 1000

	for belts, this is
		(steps per motor turn) / (number of gear teeth) / (belt module) * 1000

	half-stepping doubles the number, quarter stepping requires * 4, etc.

	valid range = 20 to 4'0960'000 (0.02 to 40960 steps/mm)
*/
#define	STEPS_PER_M_X					200000
#define	STEPS_PER_M_Y					200000
#define	STEPS_PER_M_Z					333333

/*
	Values depending on the capabilities of your stepper motors and other mechanics.
		All numbers are integers, no decimals allowed.

		Units are mm/min
*/

/// used for G0 rapid moves and as a cap for all other feedrates
#define	MAXIMUM_FEEDRATE_X		2500
#define	MAXIMUM_FEEDRATE_Y		2500
#define	MAXIMUM_FEEDRATE_Z		2500

/// used when searching endstops and as default feedrate
#define	SEARCH_FEEDRATE_X			60
#define	SEARCH_FEEDRATE_Y			60
#define	SEARCH_FEEDRATE_Z			60

/** \def SLOW_HOMING
	wether to search the home point slowly
		With some endstop configurations, like when probing for the surface of a PCB, you can't deal with overrunning the endstop. In such a case, uncomment this definition.
*/
// #define	SLOW_HOMING

/**
	Soft axis limits, in mm.
	Define them to your machine's size relative to what your host considers to be the origin.
*/

#define	X_MIN			0.0
#define	X_MAX			720.0

#define	Y_MIN			0.0
#define	Y_MAX			420.0

#define	Z_MIN			0.0
#define	Z_MAX			110.0


/***************************************************************************\
*                                                                           *
* 2. ACCELERATION                                                           *
*                                                                           *
* IMPORTANT: choose only one! These algorithms choose when to step, trying  *
*            to use more than one will have undefined and probably          *
*            disastrous results!                                            *
*                                                                           *
\***************************************************************************/

/** \def ACCELERATION_REPRAP
	acceleration, reprap style.
		Each movement starts at the speed of the previous command and accelerates or decelerates linearly to reach target speed at the end of the movement.
*/
// #define ACCELERATION_REPRAP

/** \def ACCELERATION_RAMPING
	acceleration and deceleration ramping.
		Each movement starts at (almost) no speed, linearly accelerates to target speed and decelerates just in time to smoothly stop at the target. alternative to ACCELERATION_REPRAP
*/
#define ACCELERATION_RAMPING

/** \def ACCELERATION
	how fast to accelerate when using ACCELERATION_RAMPING.
		given in mm/s^2, decimal allowed, useful range 1. to 10'000. Start with 10. for milling (high precision) or 1000. for printing
*/
#define ACCELERATION 50.

/** \def ACCELERATION_TEMPORAL
	temporal step algorithm
		This algorithm causes the timer to fire when any axis needs to step, instead of synchronising to the axis with the most steps ala bresenham.

		This algorithm is not a type of acceleration, and I haven't worked out how to integrate acceleration with it.
		However it does control step timing, so acceleration algorithms seemed appropriate

		The Bresenham algorithm is great for drawing lines, but not so good for steppers - In the case where X steps 3 times to Y's two, Y experiences massive jitter as it steps in sync with X every 2 out of 3 X steps. This is a worst-case, but the problem exists for most non-45/90 degree moves. At higher speeds, the jitter /will/ cause position loss and unnecessary vibration.
		This algorithm instead calculates when a step occurs on any axis, and sets the timer to that value.

		// TODO: figure out how to add acceleration to this algorithm
*/
// #define ACCELERATION_TEMPORAL


/***************************************************************************\
*                                                                           *
* 3. PINOUTS                                                                *
*                                                                           *
\***************************************************************************/

/*
	Machine Pin Definitions
	- make sure to avoid duplicate usage of a pin
	- comment out pins not in use, as this drops the corresponding code and makes operations faster
*/

#include	"arduino.h"

/** \def USE_INTERNAL_PULLUPS
	internal pullup resistors
		the ATmega has internal pullup resistors on it's input pins which are counterproductive with the commonly used eletronic endstops, so they should be switched off. For other endstops, like mechanical ones, you may want to uncomment this.
*/
//#define USE_INTERNAL_PULLUPS

/*
	user defined pins
	adjust to suit your electronics,
	or adjust your electronics to suit this
*/

#define	X_STEP_PIN						AIO0
#define	X_DIR_PIN							AIO1
#define	X_MIN_PIN							AIO2
//#define	X_MAX_PIN							xxxx
//#define	X_ENABLE_PIN					xxxx
#define	X_INVERT_DIR
//#define	X_INVERT_MIN
//#define	X_INVERT_MAX
//#define	X_INVERT_ENABLE

#define	Y_STEP_PIN						AIO3
#define	Y_DIR_PIN							AIO4
#define	Y_MIN_PIN							AIO5
//#define	Y_MAX_PIN							xxxx
//#define	Y_ENABLE_PIN					xxxx
//#define	Y_INVERT_DIR
//#define	Y_INVERT_MIN
//#define	Y_INVERT_MAX
//#define	Y_INVERT_ENABLE

#define	Z_STEP_PIN						DIO2
#define	Z_DIR_PIN							DIO3
#define	Z_MIN_PIN							DIO4
//#define	Z_MAX_PIN							xxxx
//#define	Z_ENABLE_PIN					xxxx
//#define	Z_INVERT_DIR
//#define	Z_INVERT_MIN
//#define	Z_INVERT_MAX
//#define	Z_INVERT_ENABLE

//TODO: we need a 12.5kHz charge-pump generator here or in the UI MCU
//#define	PS_ON_PIN							DIO9
//#define	STEPPER_ENABLE_PIN		xxxx
//#define	STEPPER_INVERT_ENABLE


/***************************************************************************\
*                                                                           *
* 4. COMMUNICATION OPTIONS                                                  *
*                                                                           *
\***************************************************************************/

/**
	Baud rate for the connection to the host. Usually 115200, other common values are 19200, 38400 or 57600.
*/
#define	BAUD	115200

/** \def XONXOFF
	Xon/Xoff flow control.
		Redundant when using RepRap Host for sending GCode, but mandatory when sending GCode files with a plain terminal emulator, like GtkTerm (Linux), CoolTerm (Mac) or HyperTerminal (Windows).
		Can also be set in Makefile
*/
#define	XONXOFF


/***************************************************************************\
*                                                                           *
* 5. MISCELLANEOUS OPTIONS                                                  *
*                                                                           *
\***************************************************************************/

/** \def DEBUG
	DEBUG
		enables /heaps/ of extra output, and some extra M-codes.
		WARNING: this WILL break most host-side talkers that expect particular responses from firmware such as reprap host and replicatorG
		use with serial terminal or other suitable talker only.
*/
// #define	DEBUG

/**
	move buffer size, in number of moves
		note that each move takes a fair chunk of ram (69 bytes as of this writing) so don't make the buffer too big - a bigger serial readbuffer may help more than increasing this unless your gcodes are more than 70 characters long on average.
		however, a larger movebuffer will probably help with lots of short consecutive moves, as each move takes a bunch of math (hence time) to set up so a longer buffer allows more of the math to be done during preceding longer moves
*/
#define	MOVEBUFFER_SIZE	8

/** \def USE_WATCHDOG
	Teacup implements a watchdog, which has to be reset every 250ms or it will reboot the controller. As rebooting (and letting the GCode sending application trying to continue the build with a then different Home point) is probably even worse than just hanging, and there is no better restore code in place, this is disabled for now.
*/
#define USE_WATCHDOG

/** \def STEP_INTERRUPT_INTERRUPTIBLE
	this option makes the step interrupt interruptible (nested).
	this should help immensely with dropped serial characters, but may also make debugging infuriating due to the complexities arising from nested interrupts
	\note disable this option if you're using a '168 or for some reason your ram usage is above 90%. This option hugely increases likelihood of stack smashing.
*/
//#define		STEP_INTERRUPT_INTERRUPTIBLE	1

/** \def ENDSTOP_STEPS
	number of steps to run into the endstops intentionally
		As Endstops trigger false alarm sometimes, Teacup debounces them by counting a number of consecutive positives. Valid range is 1...255. Use 4 or less for reliable endstops, 8 or even more for flaky ones.
*/
#define	ENDSTOP_STEPS	4


/***************************************************************************\
*                                                                           *
* 6. APPENDIX A - PWMABLE PINS AND MAPPINGS                                 *
*                                                                           *
*                                                                           *
* list of PWM-able pins and corresponding timers                            *
* timer1 is used for step timing so don't use OC1A/OC1B                     *
* they are omitted from this listing for that reason                        *
*                                                                           *
* For the atmega168/328, timer/pin mappings are as follows                  *
*                                                                           *
* OCR0A - PD6  - DIO6                                                       *
* OCR0B - PD5  - DIO5                                                       *
* OCR2A - PB3  - DIO11                                                      *
* OCR2B - PD3  - DIO3                                                       *
*                                                                           *
* For the atmega644, timer/pin mappings are as follows                      *
*                                                                           *
* OCR0A - PB3  - DIO3                                                       *
* OCR0B - PB4  - DIO4                                                       *
* OCR2A - PD7  - DIO15                                                      *
* OCR2B - PD6  - DIO14                                                      *
*                                                                           *
* For the atmega1280, timer/pin mappings are as follows                     *
*                                                                           *
* OCR0A  - PB7 - DIO13                                                      *
* OCR0B  - PG5 - DIO4                                                       *
* OCR2A  - PB4 - DIO10                                                      *
* OCR2B  - PH6 - DIO9                                                       *
* OCR3AL - PE3 - DIO5                                                       *
* OCR3BL - PE4 - DIO2                                                       *
* OCR3CL - PE5 - DIO3                                                       *
* OCR4AL - PH3 - DIO6                                                       *
* OCR4BL - PH4 - DIO7                                                       *
* OCR4CL - PH5 - DIO8                                                       *
* OCR5AL - PL3 - DIO46                                                      *
* OCR5BL - PL4 - DIO45                                                      *
* OCR5CL - PL5 - DIO44                                                      *
*                                                                           *
\***************************************************************************/
