#include "conversions.h"
#include "linalg.h"
#include "matrix.h"
#include "trans_utils.h"
#include "vector.h"
#include "integral.h"
#include "tilt.h"
#include "complementary.h"
#include "mahony.h"
#include "madgwick.h"

#include <stdio.h>

int main(void) {
  printf("#include test successfully!");

  IntegralT integral;
  integral_init(&integral);
  // Set config
  const IntegralConfigT config = {
    .method = INTEGRAL_METHOD_SERIES,
    .order = 2
  };
  integral_set_config(&integral, &config);
  
  // Read in the IMU(or MARG) sensor measurements and continuously update in the loop
  IntegralInputT input = {
    .dt = 0.01,
    .gyro = {0.123, 1.234, -0.587}
  };
  double quat[4];
  integral_update(&integral, &input);
  integral_get_quat(&integral, quat);
  printf("(qw, qx, qy, qz) = (%lf, %lf, %lf, %lf)\n", quat[0], quat[1], quat[2], quat[3]);
  return 0;
}