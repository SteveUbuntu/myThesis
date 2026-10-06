#!/usr/bin/env python3
import numpy as np

d4 = 64.62e-3
d6 = 48.6e-3
r2 = 110.4e-3
r3 = 96.0e-3

elbow_branch = 1.0

def compute_theta1(Px, Py, elbow_branch=1.0):

    P05x = Px
    P05y = Py

    d4 = 64.62e-3
    alpha1 = np.arctan2(P05y, P05x)
    P05xy = np.hypot(P05x, P05y)

    if P05xy < 1e-12:
        raise ValueError("P05xy is too small; wrist-center projection is singular.")

    alpha2 = np.arccos(np.clip(d4 / P05xy, -1.0, 1.0))
    theta1 = alpha1 + elbow_branch * alpha2 + np.pi / 2.0

    return theta1

def compute_theta2(Px, Py, Pz, d4, r3, theta3):

    P14x = Px
    P14y = Py
    P14z = Pz - d4
    P14xz = np.hypot(P14x, P14z)

    if P14xz < 1e-12:
        raise ValueError("P14xy is too small; theta2 is singular.")

    phi1 = np.arctan2(P14y, -P14x)
    phi2 = np.arcsin((r3*np.sin(theta3))/P14xz)

    theta2 = np.pi / 2.0 + (phi1 - phi2)
    return theta2


def compute_theta3(Px, Py, r2, r3, elbow_branch=1.0):

    P14x = Px
    P14y = Py
    P14xy_sq = P14x**2 + P14y**2
    c3 = (r2**2 - r3**2 - P14xy_sq) / (2 * r2 * r3)
    c3 = np.clip(c3, -1.0, 1.0)

    theta3 = elbow_branch * np.arccos(c3)
    return theta3


def compute_theta4(theta3, r2, r3):

    s3 = np.sin(theta3)
    c3 = np.cos(theta3)
    P23y = r3*s3
    P23x = r2*c3

    theta4 = np.arctan2(P23y, P23x)

    return theta4


def compute_theta5(px, py, theta1, d4, d6, wrist_branch=1.0):

    c1 = np.cos(theta1)
    s1 = np.sin(theta1)
    c5 = (py * c1 + d4 - px * s1) / d6
    c5 = np.clip(c5, -1.0, 1.0)

    theta5 = wrist_branch * np.arccos(c5)
    return theta5


def compute_theta6(x_x, x_y, y_x, y_y, theta5, theta1):

    s1 = np.sin(theta1)
    c1 = np.cos(theta1)
    s5 = np.sin(theta5)

    if abs(s5) < 1e-12:
        raise ValueError("s5 is too small; theta6 is singular.")

    c6 = (x_x * s1 - x_y * c1) / s5
    s6 = (y_y * c1 - y_x * s1) / s5
    theta6 = np.arctan2(s6, c6)

    return theta6


def main():

    d4 = 64.62e-3
    d6 = 48.6e-3
    r2 = 110.4e-3
    r3 = 96.0e-3

    elbow_branch = 1.0

    Px = -0.02 + d6
    Py = -0.04
    Pz = 0.29

    x_x = 1.0
    x_y = 0.0
    y_x = 0.0
    y_y = 1.0

    # 1) theta1
    theta1 = compute_theta1(Px, Py, elbow_branch=elbow_branch)

    # 2) theta5
    theta5 = compute_theta5(Px, Py, theta1, d4, d6, wrist_branch=elbow_branch)

    # 3) theta6
    theta6 = compute_theta6(x_x, x_y, y_x, y_y, theta5, theta1)

    # 4) theta3
    theta3 = compute_theta3(Px, Py, r2, r3, elbow_branch=elbow_branch)

    # 5) theta2
    theta2 = compute_theta2(Px, Py, Pz, d4, r3, theta3)

    # 6) theta4
    theta4 = compute_theta4(theta3, r2, r3)

    print("theta1 =", np.degrees(theta1))
    print("theta5 =", np.degrees(theta5))
    print("theta6 =", np.degrees(theta6))
    print("theta3 =", np.degrees(theta3))
    print("theta2 =", np.degrees(theta2))
    print("theta4 =", np.degrees(theta4))

if __name__ == "__main__":
    main()