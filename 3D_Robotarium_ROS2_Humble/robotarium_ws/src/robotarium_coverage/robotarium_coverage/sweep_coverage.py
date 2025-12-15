#!/usr/bin/env python3
import numpy as np
from robotarium_node.robotarium import Robotarium


def si_to_uni(poses, dxi):
    N = dxi.shape[1]
    v = np.zeros(N)
    w = np.zeros(N)
    proj_dist = 0.1
    for i in range(N):
        theta = poses[2, i]
        R = np.array([
            [np.cos(theta), np.sin(theta)],
            [-np.sin(theta)/proj_dist, np.cos(theta)/proj_dist],
        ])
        u = R @ dxi[:, i]
        v[i], w[i] = u[0], u[1]
    v = np.clip(v, -0.15, 0.15)
    w = np.clip(w, -np.pi, np.pi)
    return np.vstack((v, w))


def main():
    N = 8
    iterations = 2000
    workspace = np.array([-1.0, 1.0, -0.8, 0.8])  # [xmin, xmax, ymin, ymax]

    r = Robotarium(number_of_robots=N)

    xmin, xmax, ymin, ymax = workspace
    lane_width = (xmax - xmin) / N
    lane_centers = xmin + lane_width * (0.5 + np.arange(N))

    # +1 means moving toward ymax, -1 toward ymin
    direction = np.ones(N)

    for t in range(iterations):
        poses = r.get_poses()
        x = poses[0:2, :]  # 2 x N

        targets = np.zeros_like(x)

        for i in range(N):
            lane_center = lane_centers[i]
            if direction[i] > 0:
                targets[:, i] = np.array([lane_center, ymax])
            else:
                targets[:, i] = np.array([lane_center, ymin])

            # Flip direction when we hit the sweep edge
            if abs(x[1, i] - targets[1, i]) < 0.05:
                direction[i] *= -1

        # Simple proportional SI controller
        k = 1.0
        dxi = k * (targets - x)
        dxu = si_to_uni(poses, dxi)

        r.set_velocities(range(N), dxu)
        r.step()

    r.debug()


if __name__ == "__main__":
    main()
