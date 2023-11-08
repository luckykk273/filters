'''
filter.py
'''


import abc
import numpy as np
import pytransform3d.rotations as pr


class BasicFilter(abc.ABC):
    def __init__(self, quat: np.ndarray = None) -> None:
        self.quat: np.ndarray = quat

    @property
    def quat(self):
        return self.__quat

    @quat.setter
    def quat(self, q):
        q = pr.q_id if q is None else q
        self.__quat = pr.check_quaternion(q, unit=True)

    @abc.abstractmethod
    def update(self):
        return NotImplemented

    