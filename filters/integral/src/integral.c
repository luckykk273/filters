
#include "integral.h"
#include "conversions.h"
#include "trans_utils.h"
#include "matrix.h"
#include <string.h>

#define _USE_MATH_DEFINES
#include <math.h>

static void (*update) (IntegralT *integral, const IntegralInputT *input);

// Always return the quaternion in XYZ(roll-pitch-yaw) sequence with intrinsic rotation for convenience
// TODO: to be tested more
static void integrate_simple(IntegralT *integral, const IntegralInputT *input) {
  IntegralStateT *state = &integral->state;
  size_t i;
  for(i = 0; i < 3; ++i) {
    state->theta[i] += (input->gyro[i] * input->dt);
  }
  euler_to_quat(state->quat, state->theta, 0, 1, 2, false);
}

static void integrate_series(IntegralT *integral, const IntegralInputT *input) {
  IntegralStateT *state = &integral->state;

  double O[4][4], O_n[4][4], O_n_scale[4][4], F[4][4];
  double q[4];

  omega(&O[0][0], input->gyro);
  mat_mul_scalar(&O[0][0], &O[0][0], 4, 4, 0.5 * input->dt);
  mat_all_n(&O_n[0][0], 4, 4, 1.0);
  mat_scalar(&F[0][0], 4, 4, 1.0);
  
  size_t i;
  for(i = 1; i < integral->config.order + 1; ++i) {
    mat_mul_element_wise(&O_n[0][0], &O_n[0][0], &O[0][0], 4, 4, 4, 4, 4);
    mat_mul_scalar(&O_n_scale[0][0], &O_n[0][0], 4, 4, 1.0 / ((double) factorial(i)));
    mat_add(&F[0][0], &F[0][0], &O_n_scale[0][0], 4, 4, 4, 4, 4);
  }
  
  mat_multiply(q, &F[0][0], state->quat, 4, 4, 1);
  normalize_quat(state->quat, q);
}

static void integrate_closed(IntegralT *integral, const IntegralInputT *input) {
  IntegralStateT *state = &integral->state;
  double O[4][4], I[4][4], F[4][4];
  double norm, half_theta, c, s;
  double q[4];

  omega(&O[0][0], input->gyro);
  norm = sqrt(input->gyro[0] * input->gyro[0] + input->gyro[1] * input->gyro[1] + input->gyro[2] * input->gyro[2]);
  half_theta = norm * input->dt * 0.5;
  c = cos(half_theta);
  s = sin(half_theta);
  mat_scalar(&I[0][0], 4, 4, 1.0);
  mat_mul_scalar(&I[0][0], &I[0][0], 4, 4, c);
  mat_mul_scalar(&O[0][0], &O[0][0], 4, 4, s / norm);
  mat_add(&F[0][0], &I[0][0], &O[0][0], 4, 4, 4, 4, 4);
  mat_multiply(q, &F[0][0], state->quat, 4, 4, 1);
  normalize_quat(state->quat, q);
}

void integral_update(IntegralT *integral, const IntegralInputT *input) {
  update(integral, input);
}

size_t integral_memsize(void) {
  return sizeof(IntegralT);
}

void integral_init(IntegralT *integral) {
  memset(integral, 0, sizeof(IntegralT));
  update = integrate_closed;
  integral->state.quat[0] = 1.0;
}

void integral_set_config(IntegralT *integral, const IntegralConfigT *integral_config) {
  memcpy(&integral->config, integral_config, sizeof(IntegralConfigT));

  switch (integral_config->method) {
    case INTEGRAL_METHOD_CLOSED: {
      update = integrate_closed;
      break;
    }
    case INTEGRAL_METHOD_SERIES: {
      update = integrate_series;
      // Constrain order to be in the interval [0, 6] to prevent from expensive computation
      if(integral_config->order > 6) {
        integral->config.order = 1;
      } else {
        integral->config.order = integral_config->order;
      }
      break;
    }
    case INTEGRAL_METHOD_INTEGRATION: {
      update = integrate_simple;
      break;
    }
    default: {
      update = integrate_closed;
      break;
    }
  }
}

void integral_get_config(IntegralT *integral, IntegralConfigT *integral_config) {
  memcpy(integral_config, &integral->config, sizeof(IntegralConfigT));
}

void integral_set_quat(IntegralT *integral, const double *q) {
  // ensure the quaternion in the state is unit quaternion.
  normalize_quat(integral->state.quat, q);
}

void integral_get_quat(IntegralT *integral, double *q) {
  memcpy(q, integral->state.quat, 4 * sizeof(double));
}