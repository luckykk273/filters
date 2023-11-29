'''
complementary_filter.py

Ref: 
1. https://res.mdpi.com/sensors/sensors-15-19302/article_deploy/sensors-15-19302.pdf
2. https://github.com/CCNYRoboticsLab/imu_tools/tree/noetic
'''


from filter import BasicFilter
from utils import GRAVITY, check_vector, omega
import numpy as np
import pytransform3d.rotations as pr


GYR_THRES = 0.2  # in rad/s
ACC_THRES = 0.1  # in m/s^2
DGYR_THRES = 0.01  # in rad/s


class ComplementaryFilter(BasicFilter):
    def __init__(self, 
                 quat: np.ndarray = None, 
                 gain_acc: float = 0.01, 
                 gain_mag: float = 0.01, 
                 bias_alpha: float = 0.01,
                 do_bias_estimation: bool = True,
                 do_adaptive_gain: bool = False) -> None:
        self.quat: np.ndarray = quat
        self.gain_acc: float = gain_acc
        self.gain_mag: float = gain_mag
        self.bias_alpha: float = bias_alpha
        self.do_bias_estimation: bool = do_bias_estimation
        self.do_adaptive_gain: bool = do_adaptive_gain
        # read-only(from outside scope) variables
        self.__is_initialized: bool = False
        self.__steady_state: bool = False
        self.__gyr_prev: np.ndarray = np.zeros((3, ))
        self.__gyr_bias: np.ndarray = np.zeros((3, ))

    @property
    def quat(self):
        # Return the inverse of the state(state is fixed w.r.t. body)
        return pr.q_conj(self.__quat)

    @quat.setter
    def quat(self, q):
        # Set the state to inverse(state is fixed w.r.t. body)
        q = pr.q_id if q is None else q
        self.__quat = pr.q_conj(q)
    
    @property
    def gain_acc(self):
        return self.__gain_acc
    
    @gain_acc.setter
    def gain_acc(self, g):
        if 0.0 <= g <= 1.0:
            self.__gain_acc = g
        else:
            raise ValueError(f'gain_acc = {g} is invalid. gain_acc can only be set to [0, 1].')

    @property
    def gain_mag(self):
        return self.__gain_mag
    
    @gain_mag.setter
    def gain_mag(self, g):
        if 0.0 <= g <= 1.0:
            self.__gain_mag = g
        else:
            raise ValueError(f'gain_mag = {g} is invalid. gain_mag can only be set to [0, 1].')

    @property
    def bias_alpha(self):
        return self.__bias_alpha
    
    @bias_alpha.setter
    def bias_alpha(self, b):
        if 0.0 <= b <= 1.0:
            self.__bias_alpha = b
        else:
            raise ValueError(f'bias_alpha = {b} is invalid. bias_alpha can only be set to [0, 1].')

    @property
    def steady_state(self):
        return self.__steady_state

    @property
    def is_initialized(self):
        return self.__is_initialized
    
    @property
    def gyr_prev(self):
        return self.__gyr_prev

    @property
    def gyr_bias(self):
        return self.__gyr_bias
    
    def __get_measurement(self, acc, mag = None):
        # q_acc is the quaternion obtained from the acceleration vector
        # representing the orientation of the Global frame wrt the Local frame with
        # arbitrary yaw (intermediary frame). q3_acc is defined as 0.
        ax, ay, az = pr.norm_vector(acc)
        quat_acc = np.zeros((4, ))
        if az >= 0.0:
            quat_acc[0] = np.sqrt((az + 1.0) * 0.5)
            quat_acc[1] = -ay / (2.0 * quat_acc[0])
            quat_acc[2] = ax / (2.0 * quat_acc[0])
            quat_acc[3] = 0.0
        else:
            x = np.sqrt((1.0 - az) * 0.5)
            quat_acc[0] = -ay / (2.0 * x)
            quat_acc[1] = x
            quat_acc[2] = 0.0
            quat_acc[3] = ax / (2.0 * x)

        quat_mag = pr.q_id
        if mag is not None:
            # [lx, ly, lz] is the magnetic field reading, rotated into the intermediary
            # frame by the inverse of q_acc.
            # l = R(q_acc)^-1 m
            mag = pr.norm_vector(mag)
            lx, ly, _ = pr.q_prod_vector(pr.q_conj(quat_acc), mag)
            gamma = lx**2 + ly**2
            if lx >= 0.0:
                beta = np.sqrt(gamma + lx * np.sqrt(gamma))
                quat_mag[0] = beta / np.sqrt(2.0 * gamma)
                quat_mag[3] = ly / np.sqrt(2.0) * beta
            else:
                beta = np.sqrt(gamma - lx * np.sqrt(gamma))
                quat_mag[0] = ly / np.sqrt(2.0) * beta
                quat_mag[3] = beta / np.sqrt(2.0 * gamma)

        self.__quat = pr.concatenate_quaternions(quat_acc, quat_mag)

    def __check_state(self, acc, gyr):
        acc_norm = np.linalg.norm(acc)
        if abs(acc_norm - GRAVITY) > ACC_THRES:
            return False
        
        if any(np.abs(gyr - self.__gyr_prev) > DGYR_THRES):
            return False
        
        if any(np.abs(gyr - self.__gyr_bias) > GYR_THRES):
            return False
        
        return True

    def __update_biases(self, acc, gyr):
        self.__steady_state = self.__check_state(acc, gyr)
        if self.__steady_state:
            self.__gyr_bias += self.__bias_alpha * (gyr - self.__gyr_bias)
        self.__gyr_prev = gyr
    
    def __get_prediction(self, gyr, dt):
        gyr_unbias = gyr - self.__gyr_bias
        # Here use an 1st-order polynomial linearization method built from Taylor series to predict
        O = omega(gyr_unbias)
        # Because the quaternion representation in the class is q_GtoL, the Omega matrix should be modified
        O[0, 1:] = -O[0, 1:]
        O[1:, 0] = -O[1:, 0]
        quat_pred = self.__quat + 0.5 * dt * O @ self.__quat
        return pr.norm_vector(quat_pred)
    
    def __get_acc_correction(self, acc, q):
        acc = pr.norm_vector(acc)

        # Acceleration reading rotated into the world frame by the inverse
        # predicted quaternion (predicted gravity):
        gx, gy, gz = pr.q_prod_vector(pr.q_conj(q), acc)

        # Delta quaternion that rotates the predicted gravity into the real gravity:
        dqw = np.sqrt((gz + 1.0) * 0.5)
        return np.array([dqw, -gy / (2.0 * dqw), gx / (2.0 * dqw), 0.0])
    
    def __get_adaptive_gain(self, acc):
        acc_norm = np.linalg.norm(acc)
        error = np.abs(acc_norm - GRAVITY) / GRAVITY
        error1, error2 = 0.1, 0.2
        m = 1.0 / (error1 - error2)
        b = 1.0 - m * error1
        if error < error1:
            factor = 1.0
        elif error < error2:
            factor = m * error + b
        else:
            factor = 0.0
        return factor * self.__gain_acc
    
    def __scale_quaternion(self, gain, dq):
        if dq[0] < 0.0: # 0.9
            # Slerp (Spherical linear interpolation):
            # Not sure if shortest_path is necessary
            scaled_dq = pr.quaternion_slerp(pr.q_id, dq, gain)
        else:
            # Lerp (Linear interpolation):
            scaled_dq = gain * dq
            scaled_dq[0] += (1.0 - gain)
        return scaled_dq
    
    def __get_mag_correction(self, mag, q):
        # Magnetic reading rotated into the world frame by the inverse predicted quaternion:
        lx, ly, lz = pr.q_prod_vector(pr.q_conj(q), mag)

        # Delta quaternion that rotates the l so that it lies in the xz-plane(points north):
        gamma = lx**2 + ly**2
        beta = np.sqrt(gamma + lx * np.sqrt(gamma))
        return np.array([beta / np.sqrt(2.0 * gamma), 0.0, 0.0, ly / (np.sqrt(2.0) * beta)])

    def update(self, acc, gyr, dt: float, mag = None):
        '''
        acc: m/s^2
        gyr: rad/s
        dt: in second
        mag: units irrelevant
        '''
        acc = check_vector(acc, 3)
        gyr = check_vector(gyr, 3)
        if mag is not None:
            mag = check_vector(mag, 3)

        if not self.__is_initialized:
            # First time, ignore prediction
            self.__get_measurement(acc, mag)
            self.__is_initialized = True
            return self.__quat
        
        # Bias estimation
        if self.do_bias_estimation:
            self.__update_biases(acc, gyr)

        # Prediction
        quat_pred = self.__get_prediction(gyr, dt)

        # Correction from acc
        # q_ = q_pred * [(1-gain) * qI + gain * dq_acc]
        # where qI = identity quaternion
        dq_acc = self.__get_acc_correction(acc, quat_pred)

        if self.do_adaptive_gain:
            gain = self.__get_adaptive_gain(acc)
        else:
            gain = self.__gain_acc

        dq_acc = self.__scale_quaternion(gain, dq_acc)

        quat_temp = pr.concatenate_quaternions(quat_pred, dq_acc)

        dq_mag = pr.q_id
        if mag is not None:
            # Correction (from mag):
            # q_ = q_temp * [(1-gain) * qI + gain * dq_mag]
            # where qI = identity quaternion
            dq_mag = self.__get_mag_correction(mag, quat_temp)
            dq_mag = self.__scale_quaternion(self.__gain_mag, dq_mag)
        
        self.__quat = pr.concatenate_quaternions(quat_temp, dq_mag)
        self.__quat = pr.norm_vector(self.__quat)


if __name__ == '__main__':
    cf = ComplementaryFilter()