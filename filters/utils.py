import numpy as np


def check_vector(v, length: int):
    v = np.asarray(v, dtype=np.float64)
    if v.ndim != 1 or v.shape[0] != length:
        raise ValueError(f"Expected v with shape ({length}, ), got "
                          "array-like object with shape {v.shape}")
    return v


def omega(v):
    v = check_vector(v, 3)
    return np.array([
        [0.0, -v[0], -v[1], -v[2]],
        [v[0], 0.0, v[2], -v[1]],
        [v[1], -v[2], 0.0, v[0]],
        [v[2], v[1], -v[0], 0.0],
    ])
