/*
 * Ref: 
 */

#ifndef FILTERS_COMPLEMENTARY_H_
#define FILTERS_COMPLEMENTARY_H_


#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


enum ComplementarySensor {
  COMPLEMENTARY_SENSOR_ACC = (1 << 0),
  COMPLEMENTARY_SENSOR_MAG = (1 << 1),
};

typedef struct {
  bool do_bias_estimation;
  bool do_adaptive_gain;
  uint32_t sensor;
  double gain_acc;
  double gain_mag;
  double bias_alpha;
} ComplementaryConfigT;


typedef struct {
  double dt;
  double gyr[3];
  double acc[3];
  double mag[3];
} ComplementaryInputT;


typedef struct {
  bool is_initialized;
  bool steady_state;
  double gyr_prev[3];
  double gyr_bias[3];
  double quat[4];
} ComplementaryStateT;


typedef struct {
  ComplementaryStateT state;
  ComplementaryConfigT config;
} ComplementaryT;


void complementary_update(ComplementaryT *complementary, const ComplementaryInputT *input);

size_t complementary_memsize(void);

void complementary_init(ComplementaryT *complementary);

void complementary_set_config(ComplementaryT *complementary, const ComplementaryConfigT *complementary_config);

void complementary_get_config(ComplementaryT *complementary, ComplementaryConfigT *complementary_config);

void complementary_set_quat(ComplementaryT *complementary, const double *q);

void complementary_get_quat(ComplementaryT *complementary, double *q);


// TODO: maybe write API to get the state?

#ifdef __cplusplus
}
#endif

#endif  // FILTERS_COMPLEMENTARY_H_