/*
ref: 
1. https://github.com/xioTechnologies/Fusion
2. https://x-io.co.uk/downloads/madgwick_internal_report.pdf

Note:
The Madgwick's PhD thesis mentioned in ref [1] is
<<AHRS algorithms and calibration solutions to facilitate new applications using low-cost MEMS>>
https://scholar.google.com/citations?hl=zh-TW&user=pVoC0_cAAAAJ&view_op=list_works&sortby=pubdate

Note:
In Fusion repository xioTechnologies provided, gyroscope offset correction algorithm is provided.
But in this repository, we assume all sensor datas have been calibrated already. 
*/
#ifndef FILTERS_MADGWICK_H_
#define FILTERS_MADGWICK_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

enum MadgwickSensor {
  MADGWICK_SENSOR_ACC = (1 << 0),
  MADGWICK_SENSOR_MAG = (1 << 1),
};

typedef struct {
  uint32_t sensor;
  double gain;
  double gyr_range;
  double acc_rejection;
  double mag_rejection;
  uint32_t recovery_trigger_period;
} MadgwickConfigT;

typedef struct {
  double quat[4];
  double acc[3];
  bool is_initialized;
  double ramped_gain;
  double ramped_gain_step;
  bool gyr_recovery;
  double half_acc_feedback[3];
  double half_mag_feedback[3];
  bool acc_ignored;
  int acc_recovery_trigger;
  int acc_recovery_timeout;
  bool mag_ignored;
  int mag_recovery_trigger;
  int mag_recovery_timeout;
} MadgwickStateT;

typedef struct {
  double dt;
  double gyr[3];
  double acc[3];
  double mag[3];
} MadgwickInputT;

typedef struct {
  MadgwickStateT state;
  MadgwickConfigT config;
} MadgwickT;


void madgwick_update(MadgwickT *madgwick, const MadgwickInputT *input);

size_t madgwick_memsize(void);

void madgwick_init(MadgwickT *madgwick);

void madgwick_set_config(MadgwickT *madgwick, const MadgwickConfigT *madgwick_config);

void madgwick_get_config(MadgwickT *madgwick, MadgwickConfigT *madgwick_config);

void madgwick_set_quat(MadgwickT *madgwick, const double *q);

void madgwick_get_quat(MadgwickT *madgwick, double *q);

#ifdef __cplusplus
}
#endif

#endif  // FILTERS_MADGWICK_H_