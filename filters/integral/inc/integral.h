/*
 * Ref: 
 * 1. Quaternion kinematics for the error-state Kalman filter: http://www.iri.upc.edu/people/jsola/JoanSola/objectes/notes/kinematics.pdf
 * 2. Indirect Kalman Filter for 3D Attitude Estimation: http://mars.cs.umn.edu/tr/reports/Trawny05b.pdf
 * 3. Quaternions: https://faculty.sites.iastate.edu/jia/files/inline-files/quaternion.pdf
 */

#ifndef FILTERS_INTEGRAL_H_
#define FILTERS_INTEGRAL_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


enum IntegralMethod {
  INTEGRAL_METHOD_CLOSED = 0,
  INTEGRAL_METHOD_SERIES,
  INTEGRAL_METHOD_INTEGRATION,
};

typedef struct {
  uint32_t method;  // 0: closed-form(default); 1: power-series expansion of Euler’s formula; 2: simple integration
  uint32_t order;   // order of power-series expansion; only valid when INTEGRAL_METHOD_SERIES is used
} IntegralConfigT;

typedef struct {
  double theta[3];
  double quat[4];
} IntegralStateT;

typedef struct {
  double dt;
  double gyro[3];
} IntegralInputT;

typedef struct {
  // void (*update) (IntegralStateT *state, const double *gyro, const double dt);
  IntegralStateT state;
  IntegralConfigT config;
} IntegralT;


void integral_update(IntegralT *integral, const IntegralInputT *input);

size_t integral_memsize(void);

void integral_init(IntegralT *integral);

void integral_set_config(IntegralT *integral, const IntegralConfigT *integral_config);

void integral_get_config(IntegralT *integral, IntegralConfigT *integral_config);

void integral_set_quat(IntegralT *integral, const double *q);

void integral_get_quat(IntegralT *integral, double *q);

#ifdef __cplusplus
}
#endif

#endif  // FILTERS_INTEGRAL_H_