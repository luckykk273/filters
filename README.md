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

**dt: second**

## Todo
1. Test the correctness and performance between filters
2. Upload an example file and write testing scripts in every filter algorithms.
3. Write Python-C-API instead of adding API to library wrapper manually. 

## Usage

## References

## Contact