#!/usr/bin/env python3
import itertools
import numpy as np

# -----------------------------------------------------------------------------
# Robot geometry constants from the actual URDF chain.
# -----------------------------------------------------------------------------
# Links in meters
d1 = 131.56e-3
d4 = 64.62e-3
d5 = 73.18e-3
d6 = 45.6e-3
r2 = 110.4e-3
r3 = 96.0e-3

# Desired end-effector pose, expressed as g_base -> joint6_flange.
A06_target = np.array([[0.0, -1.0, 0.0, 0.169999],
                       [-1.0, 0.0, 0.0, -0.050000],
                       [0.0, 0.0, -1.0, 0.220000],
                       [0.0, 0.0, 0.0, 1.0]])


def urdf_joint_transform(xyz, rpy, theta):
    """Return a URDF joint transform: origin pose followed by rotation about local z."""
    roll, pitch, yaw = rpy
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    rotation = np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ], dtype=float)

    origin = np.eye(4)
    origin[:3, :3] = rotation
    origin[:3, 3] = xyz

    c, s = np.cos(theta), np.sin(theta)
    joint_motion = np.array([
        [c, -s, 0.0, 0.0],
        [s, c, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ], dtype=float)
    return origin @ joint_motion


def compute_fk_from_joint_angles(q):
    """Compute g_base -> joint6_flange FK using the URDF joint origins."""
    q = np.asarray(q, dtype=float)
    if q.shape != (6,):
        raise ValueError("Expected 6 joint angles in radians.")

    q1, q2, q3, q4, q5, q6 = q

    # Frame 0 is g_base/joint1; frame 6 is joint6_flange.
    # The origin values and RPY rotations below are copied from the URDF joint chain.
    A01 = urdf_joint_transform([0.0, 0.0, d1], [0.0, 0.0, 0.0], q1)
    A12 = urdf_joint_transform([0.0, 0.0, 0.0], [0.0, 1.5708, -1.5708], q2)
    A23 = urdf_joint_transform([-r2, 0.0, 0.0], [0.0, 0.0, 0.0], q3)
    A34 = urdf_joint_transform([-r3, 0.0, d4], [0.0, 0.0, -1.5708], q4)
    A45 = urdf_joint_transform([0.0, -d5, 0.0], [1.5708, -1.5708, 0.0], q5)
    A56 = urdf_joint_transform([0.0, d6, 0.0], [-1.5708, 0.0, 0.0], q6)

    A06 = A01 @ A12 @ A23 @ A34 @ A45 @ A56

    return {
        "A01": A01,
        "A12": A12,
        "A23": A23,
        "A34": A34,
        "A45": A45,
        "A56": A56,
        "A06": A06,
    }


DH_HALF_PI = 1.5708
DH_ALPHA = (0.0, DH_HALF_PI, 0.0, 0.0, DH_HALF_PI, -DH_HALF_PI)
DH_A = (0.0, 0.0, -r2, -r3, 0.0, 0.0)
DH_D = (d1, 0.0, 0.0, d4, d5, d6)
# q_URDF = theta + offset; the nonzero offsets come from the URDF joint origins.
ANGLE_OFFSET = np.array([
    0.0,
    DH_HALF_PI,
    0.0,
    DH_HALF_PI,
    -DH_HALF_PI,
    0.0,
])
# Allows small geometric overshoots from rounded URDF right-angle rotations.
TRIG_DOMAIN_TOLERANCE = 1e-3
FK_POSITION_TOLERANCE = 1e-4
FK_ORIENTATION_TOLERANCE = 1e-3
JOINT_LIMITS = np.array([
    [-2.9322, 2.9322],
    [-2.3562, 2.3562],
    [-2.6180, 2.6180],
    [-2.5307, 2.5307],
    [-2.8798, 2.8798],
    [-3.1400, 3.14159],
])


def dh_transform(alpha_i, a_i, d_i, theta_i):
    """Return one transform using the modified-DH convention."""
    c, s = np.cos(theta_i), np.sin(theta_i)
    ca, sa = np.cos(alpha_i), np.sin(alpha_i)
    return np.array([
        [c, -s, 0.0, a_i],
        [s * ca, c * ca, -sa, -sa * d_i],
        [s * sa, c * sa, ca, ca * d_i],
        [0.0, 0.0, 0.0, 1.0],
    ], dtype=float)


def fk_from_theta(theta):
    """Compute transforms for the six modified-DH joints."""
    transforms = [
        dh_transform(alpha_i, a_i, d_i, theta_i)
        for alpha_i, a_i, d_i, theta_i in zip(DH_ALPHA, DH_A, DH_D, theta)
    ]
    A01, A12, A23, A34, A45, A56 = transforms
    return {
        "A01": A01,
        "A12": A12,
        "A23": A23,
        "A34": A34,
        "A45": A45,
        "A56": A56,
        "A06": A01 @ A12 @ A23 @ A34 @ A45 @ A56,
    }


def checked_acos(value, description):
    if value < -1.0 - TRIG_DOMAIN_TOLERANCE or value > 1.0 + TRIG_DOMAIN_TOLERANCE:
        raise ValueError(
            "Target pose is unreachable: arccos input for {} is outside [-1, 1].".format(
                description
            )
        )
    return np.arccos(np.clip(value, -1.0, 1.0))


def checked_asin(value, description):
    if value < -1.0 - TRIG_DOMAIN_TOLERANCE or value > 1.0 + TRIG_DOMAIN_TOLERANCE:
        raise ValueError(
            "Target pose is unreachable: arcsin input for {} is outside [-1, 1].".format(
                description
            )
        )
    return np.arcsin(np.clip(value, -1.0, 1.0))


def compute_pose_errors(target, actual):
    position_error = np.linalg.norm(actual[:3, 3] - target[:3, 3])
    relative_rotation = target[:3, :3].T @ actual[:3, :3]
    orientation_error = np.arccos(np.clip(
        (np.trace(relative_rotation) - 1.0) / 2.0,
        -1.0,
        1.0,
    ))
    return position_error, orientation_error


def normalize_joint_angles_to_limits(q):
    """Choose equivalent revolute angles within the URDF joint limits."""
    normalized = []
    for angle, (lower, upper) in zip(q, JOINT_LIMITS):
        valid = [
            angle + 2.0 * np.pi * turns
            for turns in range(-2, 3)
            if lower <= angle + 2.0 * np.pi * turns <= upper
        ]
        if not valid:
            return None
        normalized.append(min(valid, key=lambda value: abs(value - angle)))
    return np.asarray(normalized)


def compute_ik_solution(A06, shoulder_branch, elbow_branch, wrist_branch):
    """Apply the analytic IK in the DH frames, then map angles to URDF joints."""
    target = np.asarray(A06, dtype=float)
    if target.shape != (4, 4):
        raise ValueError("Expected a 4x4 target transform.")
    if any(branch not in (-1.0, 1.0) for branch in
           (shoulder_branch, elbow_branch, wrist_branch)):
        raise ValueError("IK branch values must each be either -1 or +1.")

    Px, Py = target[:2, 3]
    P05 = target[:3, 3] - d6 * target[:3, 2]
    P05xy = np.hypot(P05[0], P05[1])
    if P05xy < 1e-12:
        raise ValueError("Wrist-center projection is singular.")

    theta1 = (
        np.arctan2(P05[1], P05[0])
        + shoulder_branch * checked_acos(d4 / P05xy, "shoulder angle")
        + np.pi / 2.0
    )

    c5 = (Px * np.sin(theta1) - Py * np.cos(theta1) - d4) / d6
    theta5 = wrist_branch * checked_acos(c5, "wrist angle")
    s5 = np.sin(theta5)
    if abs(s5) < 1e-12:
        raise ValueError("Wrist is singular: theta6 is not determined by the target pose.")

    inverse_rotation = target[:3, :3].T
    x6 = (
        inverse_rotation[0, 0] * np.sin(theta1)
        - inverse_rotation[0, 1] * np.cos(theta1)
    ) / s5
    y6 = (
        -inverse_rotation[1, 0] * np.sin(theta1)
        + inverse_rotation[1, 1] * np.cos(theta1)
    ) / s5
    theta6 = np.arctan2(y6, x6)

    A01 = dh_transform(DH_ALPHA[0], DH_A[0], DH_D[0], theta1)
    A12 = dh_transform(DH_ALPHA[1], DH_A[1], DH_D[1], 0.0)
    A23 = dh_transform(DH_ALPHA[2], DH_A[2], DH_D[2], 0.0)
    A45 = dh_transform(DH_ALPHA[4], DH_A[4], DH_D[4], theta5)
    A56 = dh_transform(DH_ALPHA[5], DH_A[5], DH_D[5], theta6)
    A46 = A45 @ A56

    A14 = np.linalg.inv(A01) @ target @ np.linalg.inv(A46)
    P14 = A14[:3, 3]
    P14xz = np.hypot(P14[0], P14[2])
    if P14xz < 1e-12:
        raise ValueError("Elbow geometry is singular.")

    c3 = (P14xz**2 - r2**2 - r3**2) / (2.0 * r2 * r3)
    theta3 = elbow_branch * checked_acos(c3, "elbow angle")

    theta2 = (
        np.arctan2(-P14[2], -P14[0])
        - checked_asin(r3 * np.sin(theta3) / P14xz, "shoulder angle")
    )

    A03 = (
        A01
        @ dh_transform(DH_ALPHA[1], DH_A[1], DH_D[1], theta2)
        @ dh_transform(DH_ALPHA[2], DH_A[2], DH_D[2], theta3)
    )
    A34 = np.linalg.inv(A03) @ target @ np.linalg.inv(A46)
    theta4 = np.arctan2(A34[1, 0], A34[0, 0])

    theta = np.array([theta1, theta2, theta3, theta4, theta5, theta6])
    q = theta + ANGLE_OFFSET
    actual = compute_fk_from_joint_angles(q)["A06"]
    position_error, orientation_error = compute_pose_errors(target, actual)
    if (position_error > FK_POSITION_TOLERANCE
            or orientation_error > FK_ORIENTATION_TOLERANCE):
        raise ValueError(
            "IK candidate failed FK validation: position error {:.6g} m, "
            "orientation error {:.6g} rad.".format(
                position_error,
                orientation_error,
            )
        )
    return q


def main():
    # Forward kinematics at the MoveIt init_pose (all six arm joints are zero).
    q_home = np.zeros(6, dtype=float)
    fk_home = compute_fk_from_joint_angles(q_home)

    print("FK g_base -> joint6_flange at init_pose:")
    print("\nA06_fk =\n", np.round(fk_home['A06'], 6))

    print("\nTarget pose:", A06_target)
    print("\nValid IK solutions (within URDF joint limits):")
    solution_count = 0
    for shoulder, elbow, wrist in itertools.product((-1.0, 1.0), repeat=3):
        branches = (shoulder, elbow, wrist)
        try:
            q = compute_ik_solution(A06_target, *branches)
        except ValueError as error:
            print("Branches {}: unavailable ({})".format(branches, error))
            continue

        q = normalize_joint_angles_to_limits(q)
        if q is None:
            print("Branches {}: outside URDF joint limits".format(branches))
            continue

        fk_solution = compute_fk_from_joint_angles(q)["A06"]
        position_error, rotation_error = compute_pose_errors(A06_target, fk_solution)
        solution_count += 1
        print("\nSolution {} — shoulder {}, elbow {}, wrist {}".format(
            solution_count,
            "+" if shoulder > 0 else "-",
            "+" if elbow > 0 else "-",
            "+" if wrist > 0 else "-",
        ))
        for index, value in enumerate(np.degrees(q), start=1):
            print("q{} = {:.6f} deg".format(index, value))
        print("FK position error (m) = {:.9g}".format(position_error))
        print("FK orientation error (rad) = {:.9g}".format(rotation_error))

    if solution_count == 0:
        print("No IK solutions satisfy the target pose and URDF joint limits.")


if __name__ == "__main__":
    main()
