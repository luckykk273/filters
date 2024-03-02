# filters
Several classical filters for IMU(or MARG) sensors.

## Required sensors
| Filters                | Accelerometer(m/s^2) | Gyroscope(rad/s) | Magnetometer(uT) | 
| :--------------------: | :------------------: | :--------------: | :--------------: |
| Integral               | &#9747;              | &#9711;          | &#9747;          |
| Tilt                   | &#9711;              | &#9747;          | &#9747;          |
| Complementary Filter   | &#9711;              | &#9711;          | &#9651;          |
| Mahony                 | &#9711;              | &#9711;          | &#9651;          |
| Madgwick               | &#9711;              | &#9711;          | &#9651;          |


# Preface
All the units of the measurement inputs are defined in [Required sensors](#required-sensors), and the unit of the delta time `dt` is **second**.

# Prerequisites
I have tested the library in Windows 11, but it should be easy to compile in other platforms.

## C Compiler
I use MSYS2 to install gcc. Feel free to use your own compiler.  
**NOTE: remember to update the path of your own compiler in CMakeLists.txt:**
```cmake
set(CMAKE_C_COMPILER "path/to/your/gcc")
```

## Cmake
I write a simple CMakeLists.txt to compile and build all the filters and all the static/dynamic/executable files. I also use MSYS2 to install make and one can [make a link](https://stackoverflow.com/questions/51755089/where-is-make-on-msys2-mingw-w64) between `make` and `mingw32-make.exe` for convenience.

## Scripts
I write a simple `build.bat` to run compiling and building. Feel free to write your own scripts.


# Usage
In `main.c`, there is an example of how to use the filters(e.g. `integral`) in this repository.  
The process is as follow:  
1. Declare the filter type and initialize it.  
2. Set customized configuration.  
3. Update the internal state.  
4. Get the quaternion from the internal state.  
```C
#include "integral.h"
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
```


# Todo
1. Test the correctness and performance between filters
2. Upload an example file and write testing scripts in every filter algorithms.
3. Write Python-C-API instead of adding API to library wrapper manually. 


# Contributions
1. Integrate several classical filters into an easy-to-use library.


# License
## Complementary Filter
[BSD, following the original implementation](https://github.com/CCNYRoboticsLab/imu_tools/blob/noetic/imu_complementary_filter/include/imu_complementary_filter/complementary_filter.h)

## Mahony
[MIT, following the original implementation](https://github.com/xioTechnologies/Fusion?tab=MIT-1-ov-file#readme)

**Note: The implementation of Mahony filter is followed the Madgwick's version which is also provided by xioTechnologies. So here I just declare the license the same as the Madgwick filter.**

## Madgwick
[MIT, following the original implementation](https://github.com/xioTechnologies/Fusion?tab=MIT-1-ov-file#readme)


# References
## Integral
- [Quaternion kinematics for the error-state Kalman filter](http://www.iri.upc.edu/people/jsola/JoanSola/objectes/notes/kinematics.pdf)
- [Indirect Kalman Filter for 3D Attitude Estimation](http://mars.cs.umn.edu/tr/reports/Trawny05b.pdf)
- [Quaternions](https://faculty.sites.iastate.edu/jia/files/inline-files/quaternion.pdf)

## Tilt
- [Tilt computation using accelerometer data for inclinometer applications](https://www.st.com/resource/en/design_tip/dt0140-tilt-computation-using-accelerometer-data-for-inclinometer-applications-stmicroelectronics.pdf)
- [Tilt Sensing Using a Three-Axis Accelerometer](https://www.nxp.com/docs/en/application-note/AN3461.pdf)

## Complementary Filter
- [Keeping a Good Attitude: A Quaternion-Based Orientation Filter for IMUs and MARGs](https://www.mdpi.com/1424-8220/15/8/19302)
- [imu_tools](https://github.com/CCNYRoboticsLab/imu_tools/tree/noetic/imu_complementary_filter)

## Mahony
- [Complementary filter design on the special orthogonal group SO(3)](https://folk.ntnu.no/skoge/prost/proceedings/cdc-ecc05/pdffiles/papers/1889.pdf)
- [Attitude estimation on SO(3) based on direct inertial measurements](http://users.cecs.anu.edu.au/~Robert.Mahony/Mahony_Robert/2006_MahHamPfl-C68.pdf)
- [Nonlinear Complementary Filters on the Special Orthogonal Group](https://hal.science/hal-00488376/document)
- [A complementary filter for attitude estimation of a fixed-wing UAV](http://users.cecs.anu.edu.au/~Jonghyuk.Kim/pdf/2008_Euston_iros_v1.04.pdf)
- [xioTechnologies - Open source IMU and AHRS algorithms](https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/)

## Madgwick
- [xioTechnologies - Open source IMU and AHRS algorithms](https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/)


