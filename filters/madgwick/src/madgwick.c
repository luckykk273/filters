#include "madgwick.h"
#include "trans_utils.h"
#include "vector.h"
#include "matrix.h"
#include <string.h>

#define _USE_MATH_DEFINES
#include <math.h>

#define INITIAL_GAIN  (10.0)  // Initial gain used during the initialization.
#define INITIALIZATION_PERIOD  (3.0)  // Initial gain used during the initialization.
#define ODR  (100)
#define DEGREE_TO_RAD  (M_PI / 180.0)


static const MadgwickConfigT k_default_madgwick_config = {
  .sensor = MADGWICK_SENSOR_ACC,
  .gain = 0.5,
  .gyr_range = 2000.0 * DEGREE_TO_RAD,  // rad/s
  .acc_rejection = 10.0,  // degrees
  .mag_rejection = 10.0,  // degrees
  .recovery_trigger_period = 5 * ODR,  // 5 seconds
};


static void MadgwickReset(MadgwickT *madgwick) {
  MadgwickStateT *state = &madgwick->state;
  memset(state->quat, 0, 4 * sizeof(double));
  state->quat[0] = 1.0;
  memset(state->acc, 0, 3 * sizeof(double));
  state->is_initialized = true;
  state->ramped_gain = INITIAL_GAIN;
  state->gyr_recovery = false;
  memset(state->half_acc_feedback, 0, 3 * sizeof(double));
  memset(state->half_mag_feedback, 0, 3 * sizeof(double));
  state->acc_ignored = false;
  state->acc_recovery_trigger = 0;
  state->acc_recovery_timeout = madgwick->config.recovery_trigger_period;
  state->mag_ignored = false;
  state->mag_recovery_trigger = 0;
  state->mag_recovery_timeout = madgwick->config.recovery_trigger_period;
}

// Assume ENU convention is used.
// Returns the direction of gravity scaled by 0.5.
static void HalfGravity(double *half_gravity, const double *q) {
  // obj. func. of eq. (25)
  // third column of transposed rotation matrix scaled by 0.5
  half_gravity[0] = q[1] * q[3] - q[0] * q[2];
  half_gravity[1] = q[2] * q[3] + q[0] * q[1];
  half_gravity[2] = q[0] * q[0] - 0.5 + q[3] * q[3];
}

// Assume ENU convention is used.
// Returns the direction of the magnetic field scaled by 0.5.
static void HalfMagnetic(double *half_magnetic, const double *q) {
  // first column of transposed rotation matrix scaled by -0.5
  half_magnetic[0] = 0.5 - q[0] * q[0] - q[1] * q[1];
  half_magnetic[1] = q[0] * q[3] - q[1] * q[2];
  half_magnetic[2] = -1.0 * (q[1] * q[3] + q[0] * q[2]);
}

static void Feedback(double *feedback, const double *sensor, const double *reference) {
  cross_product(feedback, sensor, reference);
  if(inner_product(sensor, reference, 3) < 0.0) {  // if error is > 90 degrees
    normalize_vector(feedback, feedback, 3);
  }
}

void madgwick_update(MadgwickT *madgwick, const MadgwickInputT *input) {
  MadgwickStateT *state = &madgwick->state;
  MadgwickConfigT *config = &madgwick->config;

  // Store accelerometer
  memcpy(state->acc, input->acc, 3 * sizeof(double));

  // Reinitialize if gyroscope range exceeded
  if(fabs(input->gyr[0]) > config->gyr_range ||
     fabs(input->gyr[1]) > config->gyr_range ||
     fabs(input->gyr[2]) > config->gyr_range) {
    double q_cpy[4];
    memcpy(q_cpy, state->quat, 4 * sizeof(double));
    MadgwickReset(madgwick);
    memcpy(state->quat, q_cpy, 4 * sizeof(double));
    state->gyr_recovery = true;
  }

  // Ramp down gain during initialization
  if(state->is_initialized) {
    state->ramped_gain -= (state->ramped_gain_step * input->dt);
    if(state->ramped_gain < config->gain || is_bit_zero(config->gain) == 0) {
      state->ramped_gain = config->gain;
      state->is_initialized = false;
      state->gyr_recovery = false;
    }
  }

  // Calculate direction of gravity indicated by algorithm
  double half_gravity[3];
  HalfGravity(half_gravity, state->quat);

  // Calculate accelerometer feedback
  double half_acc_feedback[3] = {0.0};
  state->acc_ignored = true;
  if(!(is_bit_zero(input->acc[0]) && is_bit_zero(input->acc[1]) && is_bit_zero(input->acc[2]))) {
    // Calculate accelerometer feedback scaled by 0.5
    double acc_norm[3];
    normalize_vector(acc_norm, input->acc, 3);
    Feedback(state->half_acc_feedback, acc_norm, half_gravity);

    // Don't ignore accelerometer if acceleration error below threshold
    if(state->is_initialized ||
       inner_product(state->half_acc_feedback, state->half_acc_feedback, 3) <= config->acc_rejection) {
      state->acc_ignored = false;
      state->acc_recovery_trigger -= 9;
    } else {
      state->acc_recovery_trigger += 1;
    }

    // Don't ignore accelerometer during acceleration recovery
    if(state->acc_recovery_trigger > state->acc_recovery_timeout) {
      state->acc_recovery_timeout = 0;
      state->acc_ignored = false;
    } else {
      state->acc_recovery_timeout = config->recovery_trigger_period;
    }

    state->acc_recovery_trigger = clip(state->acc_recovery_trigger, 0, config->recovery_trigger_period);    

    // Apply accelerometer feedback
    if(!state->acc_ignored) {
      memcpy(half_acc_feedback, state->half_acc_feedback, 3 * sizeof(double));
    }
  }

  // Calculate magnetometer feedback
  double half_mag_feedback[3] = {0.0};
  state->mag_ignored = true;
  if((config->sensor & MADGWICK_SENSOR_MAG) &&
      !(is_bit_zero(input->mag[0]) && is_bit_zero(input->mag[1]) && is_bit_zero(input->mag[2]))) {
    double half_magnetic[3];
    HalfMagnetic(half_magnetic, state->quat);

    // Calculate magnetometer feedback scaled by 0.5
    double half_g_cross_m[3];
    cross_product(half_g_cross_m, half_gravity, input->mag);
    normalize_vector(half_g_cross_m, half_g_cross_m, 3);
    Feedback(state->half_mag_feedback, half_g_cross_m, half_magnetic);

    // Don't ignore magnetometer if magnetic error below threshold
    if(state->is_initialized ||
       inner_product(state->half_mag_feedback, state->half_mag_feedback, 3) <= config->mag_rejection) {
      state->mag_ignored = false;
      state->mag_recovery_trigger -= 9;
    } else {
      state->mag_recovery_trigger += 1;
    }

    // Don't ignore magnetometer during magnetic recovery
    if(state->mag_recovery_trigger > state->mag_recovery_timeout) {
      state->mag_recovery_timeout = 0;
      state->mag_ignored = false;
    } else {
      state->mag_recovery_timeout = config->recovery_trigger_period;
    }

    state->mag_recovery_trigger = clip(state->mag_recovery_trigger, 0, config->recovery_trigger_period);    

    // Apply magnetometer feedback
    if(!state->mag_ignored) {
      memcpy(half_mag_feedback, state->half_mag_feedback, 3 * sizeof(double));
    }
  }

  // Convert gyroscope to radians per second scaled by 0.5
  // Note: our unit of gyroscope measurements has been assumed to rad/s already.
  double half_gyr[3];
  mat_mul_scalar(half_gyr, half_gyr, 3, 1, 0.5);

  // Apply feedback to gyroscope
  // adjusted_half_gyr = half_gyr + (half_acc_feedback + half_mag_feedback) * ramped_gain
  double adjusted_half_gyr[3];
  mat_add(adjusted_half_gyr, half_acc_feedback, half_mag_feedback, 1, 1, 1, 3, 1);
  mat_mul_scalar(adjusted_half_gyr, adjusted_half_gyr, 3, 1, state->ramped_gain);
  mat_add(adjusted_half_gyr, adjusted_half_gyr, half_gyr, 1, 1, 1, 3, 1);

  // Integrate rate of change of quaternion
  double integral_in_q[4], dq[4];
  mat_mul_scalar(&integral_in_q[1], adjusted_half_gyr, 3, 1, input->dt);
  quat_product(dq, state->quat, integral_in_q);
  mat_add(state->quat, state->quat, dq, 1, 1, 1, 4, 1);

  // Normalise quaternion
  normalize_quat(state->quat, state->quat);
}

size_t madgwick_memsize(void) {
  return sizeof(MadgwickT);
}

void madgwick_init(MadgwickT *madgwick) {
  memset(madgwick, 0, sizeof(MadgwickT));
  memcpy(&madgwick->config, &k_default_madgwick_config, sizeof(MadgwickConfigT));
  madgwick->state.quat[0] = 1.0;

  madgwick_set_config(madgwick, &k_default_madgwick_config);
  MadgwickReset(madgwick);
}

void madgwick_set_config(MadgwickT *madgwick, const MadgwickConfigT *madgwick_config) {
  memcpy(&madgwick->config, madgwick_config, sizeof(MadgwickConfigT));

  madgwick->config.gyr_range = is_bit_zero(madgwick_config->gyr_range) ? __DBL_MAX__ : 0.98 * madgwick_config->gyr_range;
  madgwick->config.acc_rejection = is_bit_zero(madgwick_config->acc_rejection) ? __DBL_MAX__ : pow(0.5 * sin(madgwick_config->acc_rejection), 2);
  madgwick->config.mag_rejection = is_bit_zero(madgwick_config->mag_rejection) ? __DBL_MAX__ : pow(0.5 * sin(madgwick_config->mag_rejection), 2);

  madgwick->state.acc_recovery_timeout = madgwick_config->recovery_trigger_period;
  madgwick->state.mag_recovery_timeout = madgwick_config->recovery_trigger_period;
  if(is_bit_zero(madgwick_config->gain) || madgwick_config->recovery_trigger_period == 0) {
    madgwick->config.acc_rejection = __DBL_MAX__;
    madgwick->config.mag_rejection = __DBL_MAX__;
  }

  if(!madgwick->state.is_initialized) {
    madgwick->state.ramped_gain = madgwick_config->gain;
  }

  madgwick->state.ramped_gain_step = (INITIAL_GAIN - madgwick->config.gain) / INITIALIZATION_PERIOD;
}

void madgwick_get_config(MadgwickT *madgwick, MadgwickConfigT *madgwick_config) {
  memcpy(madgwick_config, &madgwick->config, sizeof(MadgwickConfigT));
}

void madgwick_set_quat(MadgwickT *madgwick, const double *q) {
  // ensure the quaternion in the state is unit quaternion.
  normalize_quat(madgwick->state.quat, q);
}

void madgwick_get_quat(MadgwickT *madgwick, double *q) {
  memcpy(q, madgwick->state.quat, 4 * sizeof(double));    
}
