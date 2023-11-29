'''
madgwick.py

Ref:
1. https://courses.cs.washington.edu/courses/cse466/14au/labs/l4/madgwick_internal_report.pdf
2. https://github.com/xioTechnologies/Fusion
'''


from filter import BasicFilter
import pytransform3d.rotations as pr
import numpy as np


class Offset:
    def __init__(self, sample_rate: int, cutoff_frequency: float = 0.02,
                 timeout: int = 5, threshold: float = 3.0) -> None:
        '''
        sample_rate: Sample rate in Hz
        cutoff_frequency: Cutoff frequency in Hz
        timeout: Timeout in second
        threshold: threshold in dps
        '''
        self.filter_coefficient = 2.0 * np.pi * cutoff_frequency * (1.0 / sample_rate)
        self.timeout = timeout * sample_rate
        self.timer = 0
        self.threshold = threshold
        self.gyr_offset = np.zeros((3, ))

    def update(self, gyr: np.ndarray):
        '''
        gyr: gyroscope readings in dps
        '''
        # Subtract offset from gyroscope measurement
        gyr -= self.gyr_offset

        # Reset timer if gyroscope not stationary
        if any(np.abs(gyr) > self.threshold):
            self.timer = 0
            return gyr
        
        # Increment timer while gyroscope stationary
        if self.timer < self.timeout:
            self.timer += 1
            return gyr

        # Adjust offset if timer has elapsed
        self.gyr_offset += (gyr * self.filter_coefficient)
        return gyr


class Madgwick(BasicFilter):
    def __init__(self, quat: np.ndarray = None, gain: float = 0.5,
                 gyr_range: float = 0.0, acc_rejection: float = 90.0,
                 mag_rejection: float = 90.0, recovery_trigger_period: int = 0,
                 initial_gain: float = 10.0, initial_period: float = 3.0) -> None:
        '''
        Default settings:
            - is_initialized: 
            - initial_gain: initial gain used during the initialization
            - initial_period: initialization period in second

        Settings:
            - gain: 
            - gyr_range: in dps
            - acc_rejection: in degree
            - mag_rejection: in degree
            - recovery_trigger_period: in second
        '''
        super().__init__(quat)

        # Default settings
        self.is_initialized = False
        self.initial_gain = initial_gain
        self.initial_period = initial_period

        # Set settings
        self.gain = gain
        self.gyr_range = gyr_range
        self.acc_rejection = acc_rejection
        self.mag_rejection = mag_rejection
        self.recovery_trigger_period = recovery_trigger_period
        self.acc_recovery_timeout = self.recovery_trigger_period
        self.mag_recovery_timeout = self.recovery_trigger_period

        # disable acceleration and magnetic rejection features if gain is zero
        if gain == 0.0 or recovery_trigger_period == 0:
            self.acc_rejection = np.finfo(float).max
            self.mag_rejection = np.finfo(float).max
        
        if not self.is_initialized:
            self.ramped_gain = self.gain
        self.ramped_gain_step = (self.initial_gain - self.gain) / self.initial_period

        self.reset()

    @property
    def gyr_range(self):
        return self.__gyr_range
    
    @gyr_range.setter
    def gyr_range(self, g):
        self.__gyr_range = np.finfo(float).max if g == 0.0 else 0.98 * g
        
    @property
    def acc_rejection(self):
        return self.__acc_rejection
    
    @acc_rejection.setter
    def acc_rejection(self, a):
        self.__acc_rejection = np.finfo(float).max if a == 0.0 else (0.5 * np.sin(np.deg2rad(a)))**2

    @property
    def mag_rejection(self):
        return self.__mag_rejection
    
    @mag_rejection.setter
    def mag_rejection(self, m):
        self.__mag_rejection = np.finfo(float).max if m == 0.0 else (0.5 * np.sin(np.deg2rad(m)))**2

    def reset(self):
        '''
        Reset the states;
        This is equivalent to reinitialising the 
        algorithm while maintaining the current settings.   
        '''
        self.quat = pr.q_id
        self.acc = np.zeros((3, ))
        self.is_initialized = True
        self.ramped_gain = self.initial_gain
        self.angular_rate_recovery = False
        self.half_acc_feedback = np.zeros((3, ))
        self.half_mag_feedback = np.zeros((3, ))
        self.acc_ignored = False
        self.acc_recovery_trigger = 0
        self.acc_recovery_timeout = self.recovery_trigger_period
        self.mag_ignored = False
        self.mag_recovery_trigger = 0
        self.mag_recovery_timeout = self.recovery_trigger_period

    def __half_gravity(self):
        '''
        Returns the direction of gravity scaled by 0.5.
        Assume ENU convention is used.
        '''
        qw, qx, qy, qz = self.quat
        half_gravity = np.array([
            qx * qz - qw * qy,
            qy * qz + qw * qx,
            qw**2 - 0.5 + qz**2  # third column of transposed rotation matrix scaled by 0.5
        ])
        return half_gravity
    
    def __half_magnetic(self):
        '''
        Returns the direction of the magnetic field scaled by 0.5.
        Assume ENU convention is used.
        '''
        qw, qx, qy, qz = self.quat
        half_magnetic = np.array([
            0.5 - qw**2 - qx**2,  # // first column of transposed rotation matrix scaled by -0.5
            qw * qz - qx * qy,
            -1.0 * (qx * qz + qw * qy)
        ])
        return half_magnetic

    def __feedback(self, sensor: np.ndarray, reference: np.ndarray):
        # Use np.dot() instead of `@` operator because `sensor` and `reference` are both 1d vector.
        # `The @ operator is preferable to other methods when computing the matrix product between 2d arrays. 
        # The numpy.matmul function implements the @ operator.`
        # See: https://numpy.org/doc/stable/reference/routines.linalg.html#module-numpy.linalg
        feedback = np.cross(sensor, reference)
        if np.dot(sensor, reference) < 0.0:  # if error is > 90 degrees
            return pr.norm_vector(feedback)
        return feedback

    def update(self, acc: np.ndarray, gyr: np.ndarray, dt: float, mag = None):
        '''
        acc: in g
        gyr: in dps
        dt: in second
        mag: in arbitrary units
        '''
        # Store acc readings
        self.acc = acc.copy()
        
        # Reinitialise if gyroscope range exceeded
        if any(np.abs(gyr) > self.gyr_range):
            quat = self.quat.copy()
            self.reset()
            self.quat = quat
            self.angular_rate_recovery = True
        
        #  Ramp down gain during initialisation
        if self.is_initialized:
            self.ramped_gain -= self.ramped_gain_step * dt
            if self.ramped_gain < self.gain or self.gain == 0.0:
                self.ramped_gain = self.gain
                self.is_initialized = False
                self.angular_rate_recovery = False

        # Calculate direction of gravity indicated by algorithm
        half_gravity = self.__half_gravity()

        # Calculate accelerometer feedback
        half_acc_feedback = np.zeros((3, ))
        self.acc_ignored = True
        if not np.array_equal(acc, np.zeros((3, ))):
            # Calculate accelerometer feedback scaled by 0.5
            self.half_acc_feedback = self.__feedback(pr.norm_vector(acc), half_gravity)
        
            # Don't ignore accelerometer if acceleration error below threshold
            if self.is_initialized or np.linalg.norm(self.half_acc_feedback) <= self.acc_rejection:
                self.acc_ignored = False
                self.acc_recovery_trigger -= 9
            else:
                self.acc_recovery_trigger += 1
            
            # Don't ignore accelerometer during acceleration recovery
            if self.acc_recovery_trigger > self.acc_recovery_timeout:
                self.acc_recovery_timeout = 0
                self.acc_ignored = False
            else:
                self.acc_recovery_timeout = self.recovery_trigger_period
            
            self.acc_recovery_trigger = np.clip(self.acc_recovery_trigger, 0, self.recovery_trigger_period)

            # Apply accelerometer feedback
            if not self.acc_ignored:
                half_acc_feedback = self.half_acc_feedback
        
        # Calculate magnetometer feedback
        half_mag_feedback = np.zeros((3, ))
        self.mag_ignored = True
        if mag is not None and not np.array_equal(mag, np.zeros((3, ))):
            # Calculate direction of magnetic field indicated by algorithm
            half_magnetic = self.__half_magnetic()

            # Calculate magnetometer feedback scaled by 0.5
            self.half_mag_feedback = self.__feedback(pr.norm_vector(np.cross(half_gravity, mag)), half_magnetic)

            # Don't ignore magnetometer if magnetic error below threshold
            if self.is_initialized or np.linalg.norm(self.half_mag_feedback) <= self.mag_rejection:
                self.mag_ignored = False
                self.mag_recovery_trigger -= 9
            else:
                self.mag_recovery_trigger += 1

            # Don't ignore magnetometer during magnetic recovery
            if self.mag_recovery_trigger > self.mag_recovery_timeout:
                self.mag_recovery_timeout = 0
                self.mag_ignored = False
            else:
                self.mag_recovery_timeout = self.recovery_trigger_period
            
            self.mag_recovery_trigger = np.clip(self.mag_recovery_trigger, 0, self.recovery_trigger_period)

            # Apply magnetometer feedback
            if not self.mag_ignored:
                half_mag_feedback = self.half_mag_feedback

        # Convert gyroscope to radians per second scaled by 0.5
        half_gyr = np.deg2rad(gyr * 0.5)

        # Apply feedback to gyroscope
        adjusted_half_gyr = half_gyr + (half_acc_feedback + half_mag_feedback) * self.ramped_gain

        # Integrate rate of change of quaternion
        self.quat += pr.q_prod_vector(self.quat, adjusted_half_gyr * dt)
        return self.quat
        

if __name__ == '__main__':
    sample_rate = 100
    offset = Offset(sample_rate=sample_rate)
    madgwick = Madgwick(
        gain=0.5,
        gyr_range=2000.0,  # in dps
        acc_rejection=10.0,  # in degree
        mag_rejection=10.0,  # in degree
        recovery_trigger_period=5 * sample_rate,  # 5 seconds
    )
    while True:
        # Update gyroscope offset correction algorithm
        gyr = offset.update(gyr)
        madgwick.update()