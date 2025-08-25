import numpy as np


def angular_velocities(q1, q2, dt):
    """
        Gets linear approximation to angular velocities for w_x, w_y, and w_y. This function is defined in:
        https://mariogc.com/post/angular-velocity-quaternions/
    """
    return (2 / dt) * np.array([
        q1[0]*q2[1] - q1[1]*q2[0] - q1[2]*q2[3] + q1[3]*q2[2],
        q1[0]*q2[2] + q1[1]*q2[3] - q1[2]*q2[0] - q1[3]*q2[1],
        q1[0]*q2[3] - q1[1]*q2[2] + q1[2]*q2[1] - q1[3]*q2[0]])
