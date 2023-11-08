'''
Integral.py

Ref: 
1. Quaternion kinematics for the error-state Kalman filter: http://www.iri.upc.edu/people/jsola/JoanSola/objectes/notes/kinematics.pdf
2. Indirect Kalman Filter for 3D Attitude Estimation: http://mars.cs.umn.edu/tr/reports/Trawny05b.pdf
3. Quaternions: https://faculty.sites.iastate.edu/jia/files/inline-files/quaternion.pdf
'''

from filter import BasicFilter
from utils import check_vector, omega
import numpy as np
import pytransform3d.rotations as pr
from ahrs.filters import AngularRate


class Integral(BasicFilter):
    def __init__(self, quat: np.ndarray = None, method: str = 'closed', order: int = 1) -> None:
        super().__init__(quat)
        self.__update = None
        self.method: str = method
        self.order: int = order

    @property
    def method(self):
        return self.__method
    
    @method.setter
    def method(self, m):
        if m not in ['closed', 'series']:
            raise ValueError(f'Method `{m}` is invalid. Only `closed` and `series` methods are supported.')
        self.__method = m
        
        if self.method == 'closed':
            self.__update = self.__integrate_closed
        elif self.method == 'series':
            self.__update = self.__integrate_series
    
    @property
    def order(self):
        return self.__order
    
    @order.setter
    def order(self, o):
        if o < 1:
            raise ValueError(f'Order {o} is invalid. The order should be greater than 0 if `series` method is used.')
        self.__order = o
    
    def __integrate_closed(self, gyr: np.ndarray, dt: float):
        O = omega(gyr)
        norm = np.linalg.norm(gyr)
        half_theta = norm * dt * 0.5
        F = np.cos(half_theta) * np.identity(4) + np.sin(half_theta) / norm * O
        self.quat = F @ self.quat
        
    def __integrate_series(self, gyr: np.ndarray, dt: float):
        O = 0.5 * dt * omega(gyr)
        O_pow = np.ones_like(O)
        factorial = 1
        F = np.identity(4)
        for i in range(1, self.order + 1):
            O_pow *= O
            factorial *= i
            F += (O_pow / factorial)
        self.quat = F @ self.quat

    def update(self, gyr, dt: float) -> np.ndarray:
        gyr = check_vector(gyr, 3)
        self.__update(gyr, dt)
        return self.quat


if __name__ == '__main__':
    gyr = np.array([0.123, 0.644, 1.1346])
    dt = 0.1
    
    integral = Integral(method='closed')
    angular_rate = AngularRate(Dt=dt)
    q1 = integral.update(gyr, dt)
    q2 = angular_rate.update(q=np.array([1.0, 0.0, 0.0, 0.0]), gyr=gyr, method='closed')
    print('closed:', q1, q2, sep='\n')
    assert np.allclose(q1, q2)
    
    for i in range(1, 6):
        integral = Integral(method='series', order=i)
        angular_rate = AngularRate(Dt=dt)
        q1 = integral.update(gyr, dt)
        q2 = angular_rate.update(q=np.array([1.0, 0.0, 0.0, 0.0]), gyr=gyr, method='series', order=i)
        print(f'series [{i}]:', q1, q2, sep='\n')
        assert np.allclose(q1, q2)
