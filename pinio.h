/** \file
 \brief I/O primitives - step, enable, direction, endstops etc
*/

#ifndef _PINIO_H
#define _PINIO_H

#include "config.h"

/*
X Stepper
*/
#define  _x_step(st)            WRITE(X_STEP_PIN, st)
#define  x_step()              _x_step(1);
#ifndef  X_INVERT_DIR
  #define  x_direction(dir)    WRITE(X_DIR_PIN, dir)
#else
  #define  x_direction(dir)    WRITE(X_DIR_PIN, (dir)^1)
#endif
#ifdef  X_MIN_PIN
  #ifndef X_INVERT_MIN
    #define x_min()            (READ(X_MIN_PIN)?1:0)
  #else
    #define x_min()            (READ(X_MIN_PIN)?0:1)
  #endif
#else
  #define  x_min()              (0)
#endif

/*
Y Stepper
*/
#define  _y_step(st)            WRITE(Y_STEP_PIN, st)
#define  y_step()              _y_step(1);
#ifndef  Y_INVERT_DIR
  #define  y_direction(dir)    WRITE(Y_DIR_PIN, dir)
#else
  #define  y_direction(dir)    WRITE(Y_DIR_PIN, (dir)^1)
#endif
#ifdef  Y_MIN_PIN
  #ifndef Y_INVERT_MIN
    #define y_min()            (READ(Y_MIN_PIN)?1:0)
  #else
    #define y_min()            (READ(Y_MIN_PIN)?0:1)
  #endif
#else
  #define  y_min()              (0)
#endif

/*
Z Stepper
*/
#if defined Z_STEP_PIN && defined Z_DIR_PIN
  #define  _z_step(st)          WRITE(Z_STEP_PIN, st)
  #define  z_step()            _z_step(1);
  #ifndef  Z_INVERT_DIR
    #define  z_direction(dir)  WRITE(Z_DIR_PIN, dir)
  #else
    #define  z_direction(dir)  WRITE(Z_DIR_PIN, (dir)^1)
  #endif
#else
  #define  _z_step(x)          do { } while (0)
  #define  z_step()            do { } while (0)
  #define  z_direction(x)      do { } while (0)
#endif
#ifdef  Z_MIN_PIN
  #ifndef Z_INVERT_MIN
    #define z_min()            (READ(Z_MIN_PIN)?1:0)
  #else
    #define z_min()            (READ(Z_MIN_PIN)?0:1)
  #endif
#else
  #define  z_min()              (0)
#endif

#ifndef ESTOP_INVERT_IN
  #define estop_hit() (READ(ESTOP_IN_PIN) ? 1 : 0)
#else
  #define estop_hit() (READ(ESTOP_IN_PIN) ? 0 : 1)
#endif


/*
End Step - All Steppers
(so we don't have to delay in interrupt context)
*/
#define unstep()               do { _x_step(0); _y_step(0); _z_step(0); } while (0)

#endif /* _PINIO_H */
