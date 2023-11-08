import pytransform3d.rotations as pr
import numpy as np

q = np.array([1, 2, 3, 4])
q = pr.norm_vector(q)
v = np.array([1.2, 0.4, 0.75])

def test(q, v):
    vx = (q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]) * v[0] + \
         2 * (q[1] * q[2] - q[0] * q[3]) * v[1] + 2 * (q[1] * q[3] + q[0] * q[2]) * v[2]
    vy = 2 * (q[1] * q[2] + q[0] * q[3]) * v[0] + \
         (q[0] * q[0] - q[1] * q[1] + q[2] * q[2] - q[3] * q[3]) * v[1] + \
         2 * (q[2] * q[3] - q[0] * q[1]) * v[2]
    vz = 2 * (q[1] * q[3] - q[0] * q[2]) * v[0] + 2 * (q[2] * q[3] + q[0] * q[1]) * v[1] + \
         (q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]) * v[2]
    return np.array([vx, vy, vz])

print(pr.q_prod_vector(q, v))
print(test(q, v))