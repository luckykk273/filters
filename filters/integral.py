'''
Integral.py

Ref: 
1. Quaternion kinematics for the error-state Kalman filter: http://www.iri.upc.edu/people/jsola/JoanSola/objectes/notes/kinematics.pdf
2. Indirect Kalman Filter for 3D Attitude Estimation: http://mars.cs.umn.edu/tr/reports/Trawny05b.pdf
3. Quaternions: https://faculty.sites.iastate.edu/jia/files/inline-files/quaternion.pdf
'''

from utils import check_vector, omega
import numpy as np
import pytransform3d.rotations as pr
from ahrs.filters import AngularRate


class Integral:
    def __init__(self, quat: np.ndarray = None, method: str = 'closed', order: int = 1) -> None:
        self.quat: np.ndarray = quat
        self.method: str = method
        self.order: int = order
    
    @property
    def quat(self):
        return self.__quat
    
    @quat.setter
    def quat(self, q):
        q = pr.q_id if q is None else q
        self.__quat = pr.check_quaternion(q, unit=True)

    @property
    def method(self):
        return self.__method
    
    @method.setter
    def method(self, m):
        if m not in ['closed', 'series']:
            raise ValueError(f'Method `{m}` is invalid. Only `closed` and `series` methods are supported.')
        self.__method = m
    
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
        self.quat = pr.norm_vector(self.quat)
        
    def __integrate_series(self, gyr: np.ndarray, dt: float):
        # TODO: can be optimized
        O = 0.5 * dt * omega(gyr)
        F = np.identity(4)
        for i in range(1, self.order + 1):
            F += (O**i / np.prod(np.arange(i) + 1))
        self.quat = F @ self.quat
        self.quat = pr.norm_vector(self.quat)

    def update(self, gyr, dt: float) -> np.ndarray:
        gyr = check_vector(gyr, 3)
        
        # TODO: use function variables
        if self.method == 'closed':
            self.__integrate_closed(gyr, dt)
        elif self.method == 'series':
            self.__integrate_series(gyr, dt)

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
