#!/usr/bin/env python3
import numpy as np
from robotarium_node.robotarium import Robotarium


def si_to_uni(poses, dxi):
    """Simple single-integrator → unicycle mapping."""
    N = dxi.shape[1]
    v = np.zeros(N)
    w = np.zeros(N)
    proj_dist = 0.1
    for i in range(N):
        theta = poses[2, i]
        R = np.array([
            [np.cos(theta), np.sin(theta)],
            [-np.sin(theta)/proj_dist, np.cos(theta)/proj_dist]
        ])
        u = R @ dxi[:, i]
        v[i], w[i] = u[0], u[1]
    v = np.clip(v, -0.15, 0.15)
    w = np.clip(w, -np.pi, np.pi)
    return np.vstack((v, w))


def main():
    N = 8
    iterations = 2000
    workspace = np.array([-1.0, 1.0, -0.8, 0.8])

    # Create Robotarium instance
    r = Robotarium(number_of_robots=N)

    for t in range(iterations):
        poses = r.get_poses()       # 3 x N
        x = poses[0:2, :]           # 2 x N
        u = np.zeros_like(x)

        # --- Inter-robot repulsion ---
        for i in range(N):
            for j in range(N):
                if i == j:
                    continue
                d = x[:, i] - x[:, j]
                dist = np.linalg.norm(d)
                if dist < 0.4:
                    u[:, i] += 0.25 * d / (dist + 1e-6)

        # --- Boundary repulsion ---
        xmin, xmax, ymin, ymax = workspace
        margin = 0.2
        for i in range(N):
            xi, yi = x[0, i], x[1, i]
            if xi < xmin + margin:
                u[0, i] += 0.2
            elif xi > xmax - margin:
                u[0, i] -= 0.2
            if yi < ymin + margin:
                u[1, i] += 0.2
            elif yi > ymax - margin:
                u[1, i] -= 0.2

        # --- Circulation field ---
        for i in range(N):
            v = x[:, i]
            u[:, i] += 0.05 * np.array([-v[1], v[0]])

        # --- Noise ---
        u += 0.01 * np.random.randn(2, N)

        # --- Integrator control law ---
        dxi = 1.0 * u
        dxu = si_to_uni(poses, dxi)

        r.set_velocities(range(N), dxu)
        r.step()

    r.debug()


if __name__ == '__main__':
    main()

