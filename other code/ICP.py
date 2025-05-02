import numpy as np
import math
from typing import Union
from functools import partial
from sympy import Matrix, cos, sin


def get_correspondence_indices(
    P, Q, P_colors, Q_colors, max_dist=float("inf"), color_matching=True
):
    """
    Find nearest neighbors from P to Q while optionally ensuring color consistency.

    Args:
        P (np.ndarray): Source point cloud.
        Q (np.ndarray): Target point cloud.
        P_colors (np.ndarray): Colors corresponding to points in P.
        Q_colors (np.ndarray): Colors corresponding to points in Q.
        max_dist (float): Maximum distance threshold for matching points.
        color_matching (bool): Whether to enforce color consistency during matching.

    Returns:
        list of (index_P, index_Q) where corresponding points match.
    """
    correspondences = []
    for i, p in enumerate(P.T):  # Iterate over points in P
        p_color = P_colors[i]  # Get color of P point
        closest_idx = None
        min_dist = float("inf")

        for j, q in enumerate(Q.T):  # Iterate over points in Q
            if color_matching and Q_colors[j] != p_color:  # Skip if colors don't match
                continue

            dist = np.linalg.norm(p - q)

            # Only consider correspondences within a maximum distance
            if dist < min_dist and dist < max_dist:
                min_dist = dist
                closest_idx = j

        if closest_idx is not None:
            correspondences.append((i, closest_idx))

    if not correspondences and not color_matching:
        # If no correspondences were found, fall back to geometric matching only
        for i, p in enumerate(P.T):
            closest_idx = None
            min_dist = float("inf")
            for j, q in enumerate(Q.T):
                dist = np.linalg.norm(p - q)
                if dist < min_dist and dist < max_dist:
                    min_dist = dist
                    closest_idx = j
            if closest_idx is not None:
                correspondences.append((i, closest_idx))

    return correspondences


def dR(theta) -> np.ndarray:
    return np.array(
        [[-math.sin(theta), -math.cos(theta)], [math.cos(theta), -math.sin(theta)]]
    )


def R(theta) -> np.ndarray:
    return np.array(
        [[math.cos(theta), -math.sin(theta)], [math.sin(theta), math.cos(theta)]]
    )


def jacobian(x, p_point):
    """
    Compute the Jacobian of the error with respect to the state x = [tx, ty, theta],
    where tx, ty are the translation components, and theta is the rotation angle.

    Args:
        x: State vector [tx, ty, theta]
        p_point: A point in P (a 2x1 vector)

    Returns:
        Jacobian matrix (2x3)
    """
    theta = x[2]  # rotation angle

    # The rotation matrix corresponding to the current rotation state
    rotation = R(theta)

    # Compute the Jacobian with respect to translation and rotation
    J = np.zeros((2, 3))  # 2D point has 2 coordinates, state vector has 3 components

    # Partial derivatives with respect to translation (x[0], x[1])
    J[:, 0:2] = np.identity(2)  # Identity matrix for translation

    # Partial derivative with respect to rotation (x[2])
    # We use the derivative of the rotation matrix w.r.t. theta
    J[:, 2] = np.array([-p_point[1], p_point[0]])

    return J


# def jacobian(x, p_point):
#     # print(f"jacobian x: {x}\n p: {p_point}")
#     theta = x[2]
#     # print(f"jacobian theta: {theta}")
#     J = np.zeros((2, 3))
#     J[0:2, 0:2] = np.identity(2)
#     # print(f"jacobian J: {J}")
#     J[0:2, [2]] = dR(0).dot(p_point)
#     # print(f"jacobian J2: {J}")
#     return J


def error(x: np.ndarray, p_point: np.ndarray, q_point: np.ndarray) -> np.ndarray:
    rotation = R(x[2])
    translation = x[0:2]
    prediction = rotation.dot(p_point) + translation
    # print(
    #     f"error rotation: {rotation}\ntranslation: {translation}\nprediction: {prediction}"
    # )
    # print(f"error result: {prediction - q_point}")
    return prediction - q_point


def prepare_system(
    x: np.ndarray,
    P: np.ndarray,
    Q: np.ndarray,
    correspondences,
    P_weights,  # Added weight array for P
    kernel=lambda distance: 1.0,
) -> tuple[np.ndarray, np.ndarray, Union[np.ndarray, int]]:
    H = np.zeros((3, 3))
    g = np.zeros((3, 1))
    chi = 0

    for i, j in correspondences:
        p_point = P[:, [i]]
        q_point = Q[:, [j]]
        e = error(x, p_point, q_point)

        # Compute weight using kernel and predefined per-point weight
        kernel_weight = kernel(e)
        point_weight = P_weights[i]  # Get weight for this point
        total_weight = kernel_weight * point_weight

        J = jacobian(x, p_point)

        # Apply weighted contribution
        H += total_weight * J.T.dot(J)
        g += total_weight * J.T.dot(e)
        chi += total_weight * (e.T @ e)

    return H, g, chi


def kernel(threshold, error):
    if np.linalg.norm(error) < threshold:
        return 1.0
    return 0.0


def icp_least_squares(
    P: np.ndarray,
    Q: np.ndarray,
    P_colors: np.ndarray,
    Q_colors: np.ndarray,
    P_weights: np.ndarray,  # Added weight array
    iterations=30,
    kernel=partial(kernel, 10),
):
    x: np.ndarray = np.zeros((3, 1))
    chi_values = []
    x_values = [x.copy()]
    P_values: list[np.ndarray] = [P.copy()]
    P_copy: np.ndarray = P.copy()
    corresp_values = []

    for i in range(iterations):
        rot = R(x[2])
        t = x[0:2]

        correspondences = get_correspondence_indices(
            P_copy, Q, P_colors, Q_colors, len(Q_colors) > 5
        )
        corresp_values.append(correspondences)

        H, g, chi = prepare_system(x, P, Q, correspondences, P_weights, kernel)
        dx = np.linalg.lstsq(H, -g, rcond=None)[0]
        x += dx
        x[2] = math.atan2(math.sin(x[2]), math.cos(x[2]))

        if not isinstance(chi, int):
            chi_values.append(chi.item(0))
        x_values.append(x.copy())

        rot = R(x[2])
        t = x[0:2]
        P_copy = rot.dot(P.copy()) + t
        P_values.append(P_copy)

    corresp_values.append(corresp_values[-1])
    return P_values, chi_values, corresp_values, x_values


def compute_normals(points, step=1):
    normals = [np.array([[0, 0]])]
    normals_at_points = []
    for i in range(step, points.shape[1] - step):
        prev_point = points[:, i - step]
        next_point = points[:, i + step]
        curr_point = points[:, i]
        dx = next_point[0] - prev_point[0]
        dy = next_point[1] - prev_point[1]
        normal = np.array([[0, 0], [-dy, dx]])
        normal = normal / np.linalg.norm(normal)
        normals.append(normal[[1], :])
        normals_at_points.append(normal + curr_point)
    normals.append(np.array([[0, 0]]))
    return normals, normals_at_points


def plot_normals(normals, ax):
    label_added = False
    for normal in normals:
        if not label_added:
            ax.plot(normal[:, 0], normal[:, 1], color="grey", label="normals")
            label_added = True
        else:
            ax.plot(normal[:, 0], normal[:, 1], color="grey")
    ax.legend()
    return ax


def RotationMatrix(angle):
    return Matrix([[cos(angle), -sin(angle)], [sin(angle), cos(angle)]])


def prepare_system_normals(
    x: np.ndarray, P: np.ndarray, Q: np.ndarray, correspondences, Q_normals
):
    H = np.zeros((3, 3))
    g = np.zeros((3, 1))
    chi = 0
    for i, j in correspondences:
        p_point = P[:, [i]]
        q_point = Q[:, [j]]
        normal = Q_normals[j]
        e = normal.dot(error(x, p_point, q_point))
        J = normal.dot(jacobian(x, p_point))
        H += J.T.dot(J)
        g += J.T.dot(e)
        chi += e.T * e
    return H, g, chi
