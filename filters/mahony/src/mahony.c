#include "mahony.h"
#include "trans_utils.h"
#include "vector.h"
#include <string.h>

#define _USE_MATH_DEFINES
#include <math.h>

#define twoKp  (2.0 * 0.5)  // 2 * proportional gain (Kp)
#define twoKi  (2.0 * 0.0)  // 2 * integral gain (Ki)

static const MahonyConfigT k_default_mahony_config = {
  .sensor = MAHONY_SENSOR_ACC,
};

void mahony_update(MahonyT *mahony, const MahonyInputT *input) {
  double ax, ay, az;
	double gx = input->gyr[0], gy = input->gyr[1], gz = input->gyr[2];
  double *q = &mahony->state.quat[0];
  double half_vx, half_vy, half_vz;

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!(is_bit_zero(input->acc[0]) &&
       is_bit_zero(input->acc[1]) &&
       is_bit_zero(input->acc[2]))) {
    // Normalize accelerometer measurement
    double a_norm[3];
    normalize_vector(a_norm, input->acc, 3);
    ax = a_norm[0];
    ay = a_norm[1];
    az = a_norm[2];

    // Estimated direction of gravity and vector perpendicular to magnetic flux
		half_vx = q[1] * q[3] - q[0] * q[2];
		half_vy = q[0] * q[1] + q[2] * q[3];
		half_vz = q[0] * q[0] - 0.5f + q[3] * q[3];

    // 
    double mx = 0.0, my = 0.0, mz = 0.0;
    double half_wx = 0.0, half_wy = 0.0, half_wz = 0.0;
    if(mahony->config.sensor & MAHONY_SENSOR_MAG) {
      if(!(is_bit_zero(input->mag[0]) &&
           is_bit_zero(input->mag[1]) &&
           is_bit_zero(input->mag[2]))) {
        // Normalize magnetometer measurement
        double m_norm[3];
        normalize_vector(m_norm, input->mag, 3);
        mx = m_norm[0];
        my = m_norm[1];
        mz = m_norm[2];

        // Note: it can enhance efficience by adding auxiliary 
        //       variables to avoid repeated arithmetic

        // Reference direction of Earth's magnetic field
        double hx = 2.0f * (mx * (0.5f - q[2] * q[2] - q[3] * q[3]) + my * (q[1] * q[2] - q[0] * q[3]) + mz * (q[1] * q[3] + q[0] * q[2]));
        double hy = 2.0f * (mx * (q[1] * q[2] + q[0] * q[3]) + my * (0.5f - q[1] * q[1] - q[3] * q[3]) + mz * (q[2] * q[3] - q[0] * q[1]));
        double bx = sqrt(hx * hx + hy * hy);
        double bz = 2.0f * (mx * (q[1] * q[3] - q[0] * q[2]) + my * (q[2] * q[3] + q[0] * q[1]) + mz * (0.5f - q[1] * q[1] - q[2] * q[2]));

        // Estimated direction of magnetic field
        half_wx = bx * (0.5f - q[2] * q[2] - q[3] * q[3]) + bz * (q[1] * q[3] - q[0] * q[2]);
        half_wy = bx * (q[1] * q[2] - q[0] * q[3]) + bz * (q[0] * q[1] + q[2] * q[3]);
        half_wz = bx * (q[0] * q[2] + q[1] * q[3]) + bz * (0.5f - q[1] * q[1] - q[2] * q[2]);  
      }
    }

    // Error is sum of cross product between estimated direction and measured direction of field vectors
    // If no mag data or mag data are all zeros, error will degrade to 
    // sum of cross product between estimated and measured direction of gravity
    double half_ex = (ay * half_vz - az * half_vy) + (my * half_wz - mz * half_wy);
		double half_ey = (az * half_vx - ax * half_vz) + (mz * half_wx - mx * half_wz);
		double half_ez = (ax * half_vy - ay * half_vx) + (mx * half_wy - my * half_wx);
		
    // Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			mahony->state.integral_feedback[0] += (twoKi * half_ex * input->dt);	// integral error scaled by Ki
			mahony->state.integral_feedback[1] += (twoKi * half_ey * input->dt);
			mahony->state.integral_feedback[2] += (twoKi * half_ez * input->dt);
			gx += mahony->state.integral_feedback[0];	// apply integral feedback
			gy += mahony->state.integral_feedback[1];
			gz += mahony->state.integral_feedback[2];
		} else {
			mahony->state.integral_feedback[0] = 0.0f;	// prevent integral windup
			mahony->state.integral_feedback[1] = 0.0f;
			mahony->state.integral_feedback[2] = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * half_ex;
		gy += twoKp * half_ey;
		gz += twoKp * half_ez;
  }

  // Integrate rate of change of quaternion
  gx *= (0.5f * input->dt);		// pre-multiply common factors
	gy *= (0.5f * input->dt);
	gz *= (0.5f * input->dt);
  double qa = q[0], qb = q[1], qc = q[2];
	q[0] += (-qb * gx - qc * gy - q[3] * gz);
	q[1] += (qa * gx + qc * gz - q[3] * gy);
	q[2] += (qa * gy - qb * gz + q[3] * gx);
	q[3] += (qa * gz + qb * gy - qc * gx);
	
	// Normalise quaternion
  normalize_quat(mahony->state.quat, mahony->state.quat);
}

size_t mahony_memsize(void) {
  return sizeof(MahonyT);   
}

void mahony_init(MahonyT *mahony) {
  memset(mahony, 0, sizeof(MahonyT));
  memcpy(&mahony->config, &k_default_mahony_config, sizeof(MahonyConfigT));
  mahony->state.quat[0] = 1.0;
}

void mahony_set_config(MahonyT *mahony, const MahonyConfigT *mahony_config) {
    memcpy(&mahony->config, mahony_config, sizeof(MahonyConfigT));
}

void mahony_get_config(MahonyT *mahony, MahonyConfigT *mahony_config) {
  memcpy(mahony_config, &mahony->config, sizeof(MahonyConfigT));
}

void mahony_set_quat(MahonyT *mahony, const double *q) {
  // ensure the quaternion in the state is unit quaternion.
  normalize_quat(mahony->state.quat, q);
}

void mahony_get_quat(MahonyT *mahony, double *q) {
  memcpy(q, mahony->state.quat, 4 * sizeof(double));    
}
