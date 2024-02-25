#include "tilt.h"
#include "vector.h"
#include "trans_utils.h"

#define _USE_MATH_DEFINES
#include <math.h>
#include <string.h>

void tilt_update(TiltT *tilt, const TiltInputT *input) {
  if(is_bit_zero(input->acc[0]) && is_bit_zero(input->acc[1]) && is_bit_zero(input->acc[2])) {
    memset(&tilt->state, 0, sizeof(TiltStateT));
    return;
  }

  // Normalized accelerometer measurement
  double a_norm[3];
  normalize_vector(a_norm, input->acc, 3);

  // Update tilt angle
  double ax = a_norm[0], ay = a_norm[1], az = a_norm[2];
  tilt->state.angle = atan2(sqrt(ax * ax + ay * ay), az);

  // Update Euler angle representation
  if(tilt->config.order == EULER_ORDER_XYZ) {
    tilt->state.roll = atan2(ay, az);
    tilt->state.pitch = atan2(-ax, sqrt(ay * ay + az * az));
  } else if(tilt->config.order == EULER_ORDER_YXZ) {
    tilt->state.roll = atan2(ay, sqrt(ax * ax + az * az));
    tilt->state.pitch = atan2(-ax, az);
  }
}

size_t tilt_memsize(void) {
  return sizeof(TiltT);
}

void tilt_init(TiltT *tilt) {
  memset(tilt, 0, sizeof(TiltT));
  tilt->config.order = EULER_ORDER_XYZ;
}

void tilt_set_config(TiltT *tilt, const TiltConfigT *tilt_config) {
  memcpy(&tilt->config, tilt_config, sizeof(TiltConfigT));
  if(!(tilt->config.order == EULER_ORDER_XYZ || tilt->config.order == EULER_ORDER_YXZ)) {
    tilt->config.order = EULER_ORDER_XYZ;
  }
}

void tilt_get_config(TiltT *tilt, TiltConfigT *tilt_config) {
  memcpy(tilt_config, &tilt->config, sizeof(TiltConfigT));
}

double tilt_get_angle(TiltT *tilt) {
  return tilt->state.angle;
}

double tilt_get_roll(TiltT *tilt) {
  return tilt->state.roll;
}

double tilt_get_pitch(TiltT *tilt) {
  return tilt->state.pitch;
}
