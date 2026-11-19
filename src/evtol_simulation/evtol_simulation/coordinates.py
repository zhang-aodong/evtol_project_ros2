"""REP-103: world NED -> ENU; sensor/body FRD -> FLU; optical right/down/forward."""
import math
import numpy as np

NED_ENU = np.array([[0., 1., 0.], [1., 0., 0.], [0., 0., -1.]])
FRD_FLU = np.diag([1., -1., -1.])


def vector(value):
    return np.array([value['x_val'], value['y_val'], value['z_val']], dtype=float)


def quaternion_matrix(q):
    q = np.asarray(q, dtype=float)
    norm = np.linalg.norm(q)
    if not np.isfinite(norm) or norm < 1e-9:
        raise ValueError('Invalid quaternion')
    x, y, z, w = q / norm
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def matrix_quaternion(m):
    # Eigenvector method remains stable at 180-degree rotations.
    k = np.array([[m[0,0]-m[1,1]-m[2,2], m[0,1]+m[1,0], m[0,2]+m[2,0], m[2,1]-m[1,2]],
                  [m[0,1]+m[1,0], m[1,1]-m[0,0]-m[2,2], m[1,2]+m[2,1], m[0,2]-m[2,0]],
                  [m[0,2]+m[2,0], m[1,2]+m[2,1], m[2,2]-m[0,0]-m[1,1], m[1,0]-m[0,1]],
                  [m[2,1]-m[1,2], m[0,2]-m[2,0], m[1,0]-m[0,1], np.trace(m)]]) / 3.
    _, eig = np.linalg.eigh(k)
    q = eig[:, -1]
    return q if q[3] >= 0 else -q


def orientation(value):
    q = [value[k] for k in ('x_val', 'y_val', 'z_val', 'w_val')]
    return matrix_quaternion(NED_ENU @ quaternion_matrix(q) @ FRD_FLU)


def extrinsic(xyz, rpy_deg):
    roll, pitch, yaw = map(math.radians, rpy_deg)
    rx = quaternion_matrix([math.sin(roll/2), 0, 0, math.cos(roll/2)])
    ry = quaternion_matrix([0, math.sin(pitch/2), 0, math.cos(pitch/2)])
    rz = quaternion_matrix([0, 0, math.sin(yaw/2), math.cos(yaw/2)])
    return FRD_FLU @ np.asarray(xyz), matrix_quaternion(FRD_FLU @ rz @ ry @ rx @ FRD_FLU)
