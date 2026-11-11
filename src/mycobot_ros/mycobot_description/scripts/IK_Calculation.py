#!/usr/bin/env python3
import numpy as np

#Links in meters
d1 = 131.56e-3
d4 = 64.62e-3
d5 = 73.18e-3
d6 = 48.6e-3
r2 = 110.4e-3
r3 = 96.0e-3

#Solution usage. Positive or Negative
elbow_branch = 1.0
wrist_branch = 1.0

#Desired end-effector position and orientation
A06 = np.array([[0.0, 0.0, 1.0, 0.0456],
                [-1.0, 0.0, 0.0, -0.0646],
                [0.0, -1.0, 0.0, 0.4111],
                [0.0, 0.0, 0.0, 1.0]])

#Desired Orientation
X_x = A06[0, 0]
X_y = A06[1, 0]
X_z = A06[2, 0]

Y_x = A06[0, 1]
Y_y = A06[1, 1]
Y_z = A06[2, 1]

Z_x = A06[0, 2]
Z_y = A06[1, 2]
Z_z = A06[2, 2]

#Desired Position
Px = A06[0, 3]
Py = A06[1, 3]
Pz = A06[2, 3]


def compute_theta1(Px, Py, Pz, Z_x, Z_y, Z_z, d4, d6, elbow_branch):

    P05x = Px - Z_x * d6
    P05y = Py - Z_y * d6
    P05z = Pz - Z_z * d6

    alpha1 = np.arctan2(P05y, P05x)
    P05xy = np.hypot(P05x, P05y)

    if P05xy < 1e-12:
        raise ValueError("P05xy is too small. wrist-center projection is singular.")

    alpha2 = np.arccos(np.clip(d4 / P05xy, -1.0, 1.0))
    theta1 = alpha1 + elbow_branch * alpha2 + np.pi / 2.0

    return theta1

def compute_theta2(P14x, P14y, P14z, r3, theta3):

    P14xz = np.hypot(P14x, P14z)

    if P14xz < 1e-12:
        raise ValueError("P14xz is too small. theta2 is singular.")

    phi1 = np.arctan2(P14y, -P14x)
    phi2 = np.arcsin((r3*np.sin(theta3))/P14xz)

    theta2 = np.pi / 2.0 + (phi1 - phi2)
    return theta2


def compute_theta3(P14xy,r2, r3, elbow_branch):


    c3 = (P14xy - r2**2 - r3**2) / (2 * r2 * r3)
    c3 = np.clip(c3, -1.0, 1.0)

    theta3 = elbow_branch * np.arccos(c3)
    return theta3


def compute_theta4(theta3):

    s3 = np.sin(theta3)
    c3 = np.cos(theta3)

    theta4 = np.arctan2(s3, c3)

    return theta4


def compute_theta5(Px, Py, theta1, d4, d6, wrist_branch):

    c1 = np.cos(theta1)
    s1 = np.sin(theta1)
    c5 = (Px * s1 - Py * c1 - d4) / d6
    c5 = np.clip(c5, -1.0, 1.0)

    theta5 = wrist_branch * np.arccos(c5)
    return theta5


def compute_theta6(X_x, X_y, Y_x, Y_y, theta5, theta1):

    s1 = np.sin(theta1)
    c1 = np.cos(theta1)
    s5 = np.sin(theta5)

    if abs(s5) < 1e-12:
        raise ValueError("s5 is too small. theta6 is singular.")

    c6 = (X_x * s1 - X_y * c1) / s5
    s6 = (Y_y * c1 - Y_x * s1) / s5
    theta6 = np.arctan2(s6, c6)

    return theta6


def main():

    # 1) theta1
    theta1 = compute_theta1(Px, Py, Pz, Z_x, Z_y, Z_z, d4, d6, elbow_branch)

    # 2) theta5
    theta5 = compute_theta5(Px, Py, theta1, d4, d6, wrist_branch)

    # 3) theta6
    theta6 = compute_theta6(X_x, X_y, Y_x, Y_y, theta5, theta1)

    A01 = np.array([[np.cos(theta1), np.sin(theta1), 0, 0],
                     [np.sin(theta1), -np.cos(theta1), 0, 0],
                     [0, 0, 1, d1],
                     [0, 0, 0, 1]])

    A45 = np.array([[np.cos(theta5), 0, -np.sin(theta5), 0],
                    [np.sin(theta5), 0, np.cos(theta5), 0],
                    [0, -1, 0, d5],
                    [0, 0, 0, 1]])

    A56 = np.array([[np.cos(theta6), -np.sin(theta6), 0, 0],
                    [np.sin(theta6), np.cos(theta6), 0, 0],
                    [0, 0, 1, d6],
                    [0, 0, 0, 1]])

    A14 = np.linalg.inv(A01) @ A06 @ np.linalg.inv(A56) @ np.linalg.inv(A45)

    P14x, P14y, P14z = A14[:3, 3]
    P14xy = np.hypot(P14y, P14x)

    # 4) theta3
    theta3 = compute_theta3(P14xy, r2, r3, elbow_branch)

    # 5) theta2
    theta2 = compute_theta2(P14x, P14y, P14z, r3, theta3)

    # 6) theta4
    theta4 = compute_theta4(theta3)

    q1 = np.degrees(theta1)
    q2_rad = theta2 + np.pi / 2.0
    q3 = np.degrees(theta3)
    q4_rad = theta4 + np.pi / 2.0
    q5_rad = theta5 + np.pi / 2.0
    q6 = np.degrees(theta6)

    q2 = np.degrees(q2_rad)
    q4 = np.degrees(q4_rad)
    q5 = np.degrees(q5_rad)

    print("q1 =", q1)
    print("q2 =", q2)
    print("q3 =", q3)
    print("q4 =", q4)
    print("q5 =", q5)
    print("q6 =", q6)

if __name__ == "__main__":
    main()