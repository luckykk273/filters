#ifndef FILTERS_MAHONY_H_
#define FILTERS_MAHONY_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum MahonySensor {
  MAHONY_SENSOR_ACC = (1 << 0),
  MAHONY_SENSOR_MAG = (1 << 1),
};

typedef struct {
  uint32_t sensor;
} MahonyConfigT;

typedef struct {
  double integral_feedback[3];  // integral error terms scaled by Ki
  double quat[4];
} MahonyStateT;

typedef struct {
  double dt;
  double gyr[3];
  double acc[3];
  double mag[3];
} MahonyInputT;

typedef struct {
  MahonyStateT state;
  MahonyConfigT config;
} MahonyT;


void mahony_update(MahonyT *mahony, const MahonyInputT *input);

size_t mahony_memsize(void);

void mahony_init(MahonyT *mahony);

void mahony_set_config(MahonyT *mahony, const MahonyConfigT *mahony_config);

void mahony_get_config(MahonyT *mahony, MahonyConfigT *mahony_config);

void mahony_set_quat(MahonyT *mahony, const double *q);

void mahony_get_quat(MahonyT *mahony, double *q);

#ifdef __cplusplus
}
#endif

#endif  // FILTERS_MAHONY_H_