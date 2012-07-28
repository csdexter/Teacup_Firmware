#include "gcode_parse.h"

/** \file
  \brief Parse received G-Codes
*/

#include <string.h>

#include "serial.h"
#include "sermsg.h"
#include "dda_queue.h"
#include "debug.h"
#include "sersendf.h"

#include "gcode_process.h"

/// current or previous gcode word
/// for working out what to do with data just received
uint8_t last_field = 0;

/// crude floating point data storage
decfloat read_digit  __attribute__ ((__section__ (".bss")));

/// this is where we store all the data for the current command before we work out what to do with it
GCODE_COMMAND next_target __attribute__ ((__section__ (".bss")));

/*
  decfloat_to_int() is the weakest subject to variable overflow. For evaluation, we assume a build room of +-1000 mm and STEPS_PER_MM_x between 1.000 and 4096. Accordingly for metric units:

    df->mantissa:  +-0..1048075    (20 bit - 500 for rounding)
    df->exponent:  0, 2, 3, 4 or 5 (10 bit)
    multiplicand:  1000            (10 bit)

  imperial units:

    df->mantissa:  +-0..32267      (15 bit - 500 for rounding)
    df->exponent:  0, 2, 3, 4 or 5 (10 bit)
    multiplicand:  25400           (15 bit)
*/
// decfloat_to_int() can handle a bit more:
#define DECFLOAT_EXP_MAX 3 // more is pointless, as 1 um is our presision
// (2^^32 - 1) / multiplicand - powers[DECFLOAT_EXP_MAX] / 2 =
// 4294967295 / 1000 - 5000 =
#define DECFLOAT_MANT_MM_MAX 4289967 // = 4290 mm
// 4294967295 / 25400 - 5000 =
#define DECFLOAT_MANT_IN_MAX 164093 // = 164 inches = 4160 mm

/*
  utility functions
*/
extern const uint32_t powers[];  // defined in sermsg.c

/// convert a floating point input value into an integer with appropriate scaling.
/// \param *df pointer to floating point structure that holds fp value to convert
/// \param multiplicand multiply by this amount during conversion to integer
///
/// Tested for up to 42'000 mm (accurate), 420'000 mm (precision 10 um) and
/// 4'200'000 mm (precision 100 um).
static int32_t decfloat_to_int(decfloat *df, uint16_t multiplicand) {
  uint32_t r = df->mantissa;
  uint8_t e = df->exponent;

  // e=1 means we've seen a decimal point but no digits after it, and e=2 means we've seen a decimal point with one digit so it's too high by one if not zero
  if (e) e--;

  // This raises range for mm by factor 1000 and for inches by factor 100.
  // It's a bit expensive, but we should have the time while parsing.
  while (e && multiplicand % 10 == 0) {
    multiplicand /= 10;
    e--;
  }

  r *= multiplicand;
  if (e) r = (r + powers[e] / 2) / powers[e];

  return df->sign ? -(int32_t)r : (int32_t)r;
}

void gcode_init(void) {
  next_target.flags = 0;
  // assume a G1 by default
  next_target.seen_G = 1;
  next_target.G = 1;
}

/// Character Received - add it to our command
/// \param c the next character to process
void gcode_parse_char(uint8_t c) {
  // uppercase
  if (c >= 'a' && c <= 'z') c &= ~32;

  // process previous field
  if (last_field) {
    // check if we're seeing a new field or end of line
    // any character will start a new field, even invalid/unknown ones
    if ((c >= 'A' && c <= 'Z') || c == 10 || c == 13) {
      switch (last_field) {
        case 'G':
          next_target.G = read_digit.mantissa;
          if (DEBUG_ECHO && (debug_flags & DEBUG_ECHO))
            serwrite_uint8(next_target.G);
          break;
        case 'M':
          next_target.M = read_digit.mantissa;
          if (DEBUG_ECHO && (debug_flags & DEBUG_ECHO))
            serwrite_uint8(next_target.M);
          break;
        case 'X':
          if (next_target.option_inches)
            next_target.target.X = decfloat_to_int(&read_digit, 25400);
          else
            next_target.target.X = decfloat_to_int(&read_digit, 1000);
          if (DEBUG_ECHO && (debug_flags & DEBUG_ECHO))
            serwrite_int32(next_target.target.X);
          break;
        case 'Y':
          if (next_target.option_inches)
            next_target.target.Y = decfloat_to_int(&read_digit, 25400);
          else
            next_target.target.Y = decfloat_to_int(&read_digit, 1000);
          if (DEBUG_ECHO && (debug_flags & DEBUG_ECHO))
            serwrite_int32(next_target.target.Y);
          break;
        case 'Z':
          if (next_target.option_inches)
            next_target.target.Z = decfloat_to_int(&read_digit, 25400);
          else
            next_target.target.Z = decfloat_to_int(&read_digit, 1000);
          if (DEBUG_ECHO && (debug_flags & DEBUG_ECHO))
            serwrite_int32(next_target.target.Z);
          break;
        case 'F':
          // just use raw integer, we need move distance and n_steps to convert it to a useful value, so wait until we have those to convert it
          if (next_target.option_inches)
            next_target.target.F = decfloat_to_int(&read_digit, 25400);
          else
            next_target.target.F = decfloat_to_int(&read_digit, 1);
          if (DEBUG_ECHO && (debug_flags & DEBUG_ECHO))
            serwrite_uint32(next_target.target.F);
          break;
        case 'P':
          //According to the standard, P is float and in seconds!
          next_target.P = decfloat_to_int(&read_digit, 1000);
          if (DEBUG_ECHO && (debug_flags & DEBUG_ECHO))
            serwrite_uint16(next_target.P);
          break;
        case 'T':
          next_target.T = read_digit.mantissa;
          if (DEBUG_ECHO && (debug_flags & DEBUG_ECHO))
            serwrite_uint8(next_target.T);
          break;
        case 'N':
          //TODO: evaluate whether we always want to ignore line numbers
          break;
      }
      // reset for next field
      last_field = 0;
      read_digit.sign = read_digit.mantissa = read_digit.exponent = 0;
    }
  }

  // skip comments
  if (next_target.seen_parens_comment == 0) {
    // new field?
    if ((c >= 'A' && c <= 'Z') || c == '*') {
      last_field = c;
      if (DEBUG_ECHO && (debug_flags & DEBUG_ECHO))
        serial_writechar(c);
    }

    // process character
    switch (c) {
      // each currently known command is either G or M, so preserve previous G/M unless a new one has appeared
      // FIXME: same for T command
      case 'G':
        next_target.seen_G = 1;
        next_target.seen_M = 0;
        next_target.M = 0;
        break;
      case 'M':
        next_target.seen_M = 1;
        next_target.seen_G = 0;
        next_target.G = 0;
        break;
      case 'X':
        next_target.seen_X = 1;
        break;
      case 'Y':
        next_target.seen_Y = 1;
        break;
      case 'Z':
        next_target.seen_Z = 1;
        break;
      case 'F':
        next_target.seen_F = 1;
        break;
      case 'S':
        next_target.seen_S = 1;
        break;
      case 'P':
        next_target.seen_P = 1;
        break;
      case 'T':
        next_target.seen_T = 1;
        break;
      case 'N':
        break;
      // comments
      case '(':
        next_target.seen_parens_comment = 1;
        break;
      // now for some numeracy
      case '-':
        read_digit.sign = 1;
        // force sign to be at start of number, so 1-2 = -2 instead of -12
        read_digit.exponent = 0;
        read_digit.mantissa = 0;
        break;
      case '.':
        if (read_digit.exponent == 0)
          read_digit.exponent = 1;
        break;
#ifdef DEBUG
      case '%':
      case ' ':
      case '\t':
      case 10:
      case 13:
        // ignore
        break;
#endif
      default:
        // can't do ranges in switch..case, so process actual digits here.
        if (c >= '0' && c <= '9') {
          if (read_digit.exponent < DECFLOAT_EXP_MAX + 1 &&
              ((next_target.option_inches == 0 &&
              read_digit.mantissa < DECFLOAT_MANT_MM_MAX) ||
              (next_target.option_inches &&
              read_digit.mantissa < DECFLOAT_MANT_IN_MAX))) {
            // this is simply mantissa = (mantissa * 10) + atoi(c) in different clothes
            read_digit.mantissa = (read_digit.mantissa << 3) + (read_digit.mantissa << 1) + (c - '0');
            if (read_digit.exponent)
              read_digit.exponent++;
          }
        }
#ifdef DEBUG
        else {
          // invalid
          serial_writechar('?');
          serial_writechar(c);
          serial_writechar('?');
        }
#endif
    }
  } else if (next_target.seen_parens_comment == 1 && c == ')')
    next_target.seen_parens_comment = 0; // recognize stuff after a (comment)

  // end of line
  if (c == 10 || c == 13) {
    if (DEBUG_ECHO && (debug_flags & DEBUG_ECHO))
      serial_writechar(c);

    // process
    serial_writestr_P(PSTR("ok "));
    process_gcode_command();
    serial_writechar('\n');

    // reset variables
    gcode_init(); // last_field and read_digit are reset above already
    
    if (next_target.option_all_relative) {
      next_target.target.X = next_target.target.Y = next_target.target.Z = 0;
    }
  }
}
