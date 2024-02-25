/*
 * ref: 
 * 1. https://www.st.com/resource/en/design_tip/dt0140-tilt-computation-using-accelerometer-data-for-inclinometer-applications-stmicroelectronics.pdf
 * 2. https://www.nxp.com/docs/en/application-note/AN3461.pdf
 * 
 * Note:
 * 1. 3-axis accelerometer is supported only.
 * 2. Assume the accelerometer is installed in the X-Y horizontal plane and ENU convention is choosed.
 * 3. Static condition is necessary.
 */

#ifndef FILTERS_TILT_H_
#define FILTERS_TILT_H_

#include <stdint.h>
#include "conversions.h"

#ifdef __cplusplus
extern "C" {
#endif


typedef struct {
  uint32_t order;
} TiltConfigT;

typedef struct {
  double angle;  // angle of tilt
  double roll;  // angle of x-axis
  double pitch;  // angle of y-axis
} TiltStateT;

typedef struct {
  double dt;
  double acc[3];
} TiltInputT;

typedef struct {
  TiltStateT state;
  TiltConfigT config;
} TiltT;


void tilt_update(TiltT *tilt, const TiltInputT *input);

size_t tilt_memsize(void);

void tilt_init(TiltT *tilt);

void tilt_set_config(TiltT *tilt, const TiltConfigT *tilt_config);

void tilt_get_config(TiltT *tilt, TiltConfigT *tilt_config);

// The output range is 0 to 180 degrees
// From the point of view of noise and measurement errors, 
// the worst case is when tilt is halfway (α = 45 degrees).
double tilt_get_angle(TiltT *tilt);

// angle of x-axis
double tilt_get_roll(TiltT *tilt);

// angle of y-axis
double tilt_get_pitch(TiltT *tilt);



#ifdef __cplusplus
}
#endif

#endif  // FILTERS_TILT_H_