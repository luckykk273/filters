
#include "trans_utils.h"
#include "complementary.h"
#include "vector.h"
#include <string.h>

#define _USE_MATH_DEFINES
#include <math.h>

#define STEADY_ACC_THRES  (0.1)
#define STEADY_DGYR_THRES  (0.01)
#define STEADY_GYR_THRES  (0.2)


static const ComplementaryConfigT k_default_complementary_config = {
  .do_bias_estimation = true,
  .do_adaptive_gain = false,
  .sensor = COMPLEMENTARY_SENSOR_ACC,
  .gain_acc = 0.01,
  .gain_mag = 0.01,
  .bias_alpha = 0.01
};

static void get_measurement(ComplementaryStateT *state, const ComplementaryInputT *input, const uint32_t sensor) {
  // q_acc is the quaternion obtained from the acceleration vector
  // representing the orientation of the Global frame wrt the Local frame with
  // arbitrary yaw (intermediary frame). q_acc[3] is defined as 0.

  // Normalize acceleration vector.
  double ax, ay, az;
  double a_norm[3];
  double q_acc[4] = {1.0, 0.0, 0.0, 0.0};
  normalize_vector(a_norm, input->acc, 3);
  ax = a_norm[0], ay = a_norm[1], az = a_norm[2];
  if(az >= 0.0) {
    q_acc[0] = sqrt((az + 1.0) * 0.5);
    q_acc[1] = -ay / (2.0 * q_acc[0]);
    q_acc[2] = ax / (2.0 * q_acc[0]);
    q_acc[3] = 0.0;
  } else {
    double x = sqrt((1.0 - az) * 0.5);
    q_acc[0] = -ay / (2.0 * x);
    q_acc[1] = x;
    q_acc[2] = 0.0;
    q_acc[3] = ax / (2.0 * x);
  }

  double q_mag[4] = {1.0, 0.0, 0.0, 0.0};
  if(sensor & COMPLEMENTARY_SENSOR_MAG) {
    // [lx, ly, lz] is the magnetic field reading, rotated into the intermediary
    // frame by the inverse of q_acc.
    // l = R(q_acc)^-1 m
    double mx, my, mz;
    mx = input->mag[0], my = input->mag[1], mz = input->mag[2];
    // TODO: if mag should be normalized?
    double lx = (q_acc[0] * q_acc[0] + q_acc[1] * q_acc[1] - q_acc[2] * q_acc[2]) * mx +
                2.0 * (q_acc[1] * q_acc[2]) * my - 
                2.0 * (q_acc[0] * q_acc[2]) * mz;
    double ly = 2.0 * (q_acc[1] * q_acc[2]) * mx +
                (q_acc[0] * q_acc[0] - q_acc[1] * q_acc[1] + q_acc[2] * q_acc[2]) * my +
                2.0 * (q_acc[0] * q_acc[1]) * mz;
    
    // q_mag is the quaternion that rotates the Global frame (North West Up)
    // into the intermediary frame. state->quat[1]mag and state->quat[2]mag are defined as 0.
    double gamma = lx * lx + ly * ly;
    double beta = sqrt(gamma + lx * sqrt(gamma));
    q_mag[0] = beta / (sqrt(2.0 * gamma));
    q_mag[3] = ly / (sqrt(2.0) * beta);
  }

  // The quaternion multiplication between q_acc and q_mag represents the
  // quaternion, orientation of the Global frame wrt the local frame.
  // q = q_acc times q_mag
  quat_product(state->quat, q_acc, q_mag);
}

static bool check_steady_state(const ComplementaryStateT *state, const double *acc, const double *gyr) {
  double acc_magnitude = sqrt(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
  if(fabs(acc_magnitude - GRAVITY) > STEADY_ACC_THRES) {
    return false;
  }

  if(fabs(gyr[0] - state->gyr_prev[0]) > STEADY_DGYR_THRES ||
     fabs(gyr[1] - state->gyr_prev[1]) > STEADY_DGYR_THRES ||
     fabs(gyr[2] - state->gyr_prev[2]) > STEADY_DGYR_THRES) {
    return false;
  }

  if (fabs(gyr[0] - state->gyr_bias[0]) > STEADY_GYR_THRES ||
      fabs(gyr[1] - state->gyr_bias[1]) > STEADY_GYR_THRES ||
      fabs(gyr[2] - state->gyr_bias[2]) > STEADY_GYR_THRES) {
    return false;
  }

  return true;
}

static void update_biases(ComplementaryT *complementary, const double *acc, const double *gyr) {
  ComplementaryStateT *state = &complementary->state;
  ComplementaryConfigT *config = &complementary->config;
  state->steady_state = check_steady_state(state, acc, gyr);
  if(state->steady_state) {
    state->gyr_bias[0] += (config->bias_alpha * (gyr[0] - state->gyr_bias[0])); 
    state->gyr_bias[1] += (config->bias_alpha * (gyr[1] - state->gyr_bias[1])); 
    state->gyr_bias[2] += (config->bias_alpha * (gyr[2] - state->gyr_bias[2])); 
  }
  memcpy(state->gyr_prev, gyr, 3 * sizeof(double));
}

static void get_prediction(double *q_pred, const ComplementaryStateT *state, const double *gyr, const double dt) {
  double gyr_unbias[3];
  gyr_unbias[0] = gyr[0] - state->gyr_bias[0];
  gyr_unbias[1] = gyr[1] - state->gyr_bias[1];
  gyr_unbias[2] = gyr[2] - state->gyr_bias[2];

  q_pred[0] = state->quat[0] + 0.5 * dt * (gyr_unbias[0] * state->quat[1] + gyr_unbias[1] * state->quat[2] + gyr_unbias[2] * state->quat[3]);
  q_pred[1] = state->quat[1] + 0.5 * dt * (-gyr_unbias[0] * state->quat[0] - gyr_unbias[1] * state->quat[3] + gyr_unbias[2] * state->quat[2]);
  q_pred[2] = state->quat[2] + 0.5 * dt * (gyr_unbias[0] * state->quat[3] - gyr_unbias[1] * state->quat[0] - gyr_unbias[2] * state->quat[1]);
  q_pred[3] = state->quat[3] + 0.5 * dt * (-gyr_unbias[0] * state->quat[2] + gyr_unbias[1] * state->quat[1] - gyr_unbias[2] * state->quat[0]);
  normalize_quat(q_pred, q_pred);
}

static void get_acc_correction(double *dq_acc, const double *acc, const double *q) {
  // Normalize acceleration vector.
  double a_norm[3];
  normalize_vector(a_norm, acc, 3);

  // Acceleration reading rotated into the world frame by the inverse
  // predicted quaternion (predicted gravity):
  double gravity[3], q_conj[4];
  quat_conjugate(q_conj, q);
  quat_prod_vec(gravity, q_conj, acc);

  // Delta quaternion that rotates the predicted gravity into the real gravity:
  dq_acc[0] = sqrt((gravity[2] + 1.0) * 0.5);
  dq_acc[1] = -gravity[1] / (2.0 * dq_acc[0]);
  dq_acc[2] = gravity[0] / (2.0 * dq_acc[0]);
  dq_acc[3] = 0.0;
}

static void get_mag_correction(double *dq_mag, const double *mag, const double *q) {
  // Magnetic reading rotated into the world frame by the inverse predicted
  // quaternion:
  double l[3], q_conj[4];
  quat_conjugate(q_conj, q);
  quat_prod_vec(l, q_conj, mag);

  // Delta quaternion that rotates the l so that it lies in the xz-plane
  // (points north):
  double gamma = l[0] * l[0] + l[1] * l[1];
  double beta = sqrt(gamma + l[0] * sqrt(gamma));
  dq_mag[0] = beta / (sqrt(2.0 * gamma));
  dq_mag[1] = 0.0;
  dq_mag[2] = 0.0;
  dq_mag[3] = l[1] / (sqrt(2.0) * beta);
}

static double get_adaptive_gain(const double alpha, const double *acc) {
  double acc_magnitude = sqrt(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
  double error = fabs(acc_magnitude - GRAVITY) / GRAVITY;
  double factor;
  double error1 = 0.1, error2 = 0.2;
  double m = 1.0 / (error1 - error2);
  double b = 1.0 - m * error1;
  if(error < error1) {
    factor = 1.0;
  } else if(error < error2) {
    factor = m * error + b;
  } else {
    factor = 0.0;
  }

  return factor * alpha;
}

static void scale_quaternion(double *dq, const double gain) {
  double filtered_dq[4];
  double q_identity[4] = {1.0, 0.0, 0.0, 0.0};
  if(dq[0] < 0.0) {  // 0.9
    quat_slerp(filtered_dq, q_identity, dq, gain, false);
  } else {
    quat_lerp(filtered_dq, q_identity, dq, gain, false);
  }
  memcpy(dq, filtered_dq, 4 * sizeof(double));
}

void complementary_update(ComplementaryT *complementary, const ComplementaryInputT *input) {
  if(!complementary->state.is_initialized) {
    // First time - ignore prediction:
    get_measurement(&complementary->state, input, complementary->config.sensor);
    complementary->state.is_initialized = true;
    return;
  }

  // Bias estimation.
  if (complementary->config.do_bias_estimation) {
    update_biases(complementary, input->acc, input->gyr);
  }

  // Prediction.
  double q_pred[4];
  get_prediction(q_pred, &complementary->state, input->gyr, input->dt);

  // Correction (from acc):
  // q_ = q_pred * [(1-gain) * qI + gain * dq_acc]
  // where qI = identity quaternion
  double dq_acc[4];
  get_acc_correction(dq_acc, input->acc, q_pred);

  double alpha = complementary->config.gain_acc;
  if(complementary->config.do_adaptive_gain) {
    alpha = get_adaptive_gain(alpha, input->acc);
  }

  scale_quaternion(dq_acc, alpha);

  double q_temp[4];
  quat_product(q_temp, q_pred, dq_acc);

  double dq_mag[4] = {1.0, 0.0, 0.0, 0.0};
  if(complementary->config.sensor & COMPLEMENTARY_SENSOR_MAG) {
    // Correction (from mag):
    // q_ = q_temp * [(1-gain) * qI + gain * dq_mag]
    // where qI = identity quaternion
    get_mag_correction(dq_mag, input->mag, q_temp);
    scale_quaternion(dq_mag, complementary->config.gain_mag);
  }

  quat_product(complementary->state.quat, q_temp, dq_mag);
  normalize_quat(complementary->state.quat, complementary->state.quat);
}

size_t complementary_memsize(void) {
  return sizeof(ComplementaryT);
}

void complementary_init(ComplementaryT *complementary) {
  memset(complementary, 0, sizeof(ComplementaryT));
  memcpy(&complementary->config, &k_default_complementary_config, sizeof(ComplementaryConfigT));
  complementary->state.quat[0] = 1.0;
}

void complementary_set_config(ComplementaryT *complementary, const ComplementaryConfigT *complementary_config) {
  memcpy(&complementary->config, complementary_config, sizeof(ComplementaryConfigT));
  
  // TODO: check if sensor setting is valid

  if(complementary->config.gain_acc < 0.0 && 1.0 < complementary->config.gain_acc) {
    complementary->config.gain_acc = 0.01;
  }
  
  if(complementary->config.gain_mag < 0.0 && 1.0 < complementary->config.gain_mag) {
    complementary->config.gain_mag = 0.01;
  }
  
  if(complementary->config.bias_alpha < 0.0 && 1.0 < complementary->config.bias_alpha) {
    complementary->config.bias_alpha = 0.01;
  }
}

void complementary_get_config(ComplementaryT *complementary, ComplementaryConfigT *complementary_config) {
  memcpy(complementary_config, &complementary->config, sizeof(ComplementaryConfigT));
}

void complementary_set_quat(ComplementaryT *complementary, const double *q) {
  // ensure the quaternion in the state is the unit quaternion.
  normalize_quat(complementary->state.quat, q);
  // Set the quaternion to inverse (quaternion is fixed wrt body).
  quat_conjugate(complementary->state.quat, complementary->state.quat);
}

void complementary_get_quat(ComplementaryT *complementary, double *q) {
  quat_conjugate(q, complementary->state.quat);
}