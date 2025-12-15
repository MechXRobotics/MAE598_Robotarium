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
    workspace = np.array([-1.0, 1.0, -0.8, 0.8])

    # Discretization of the workspace
    grid_nx = 40
    grid_ny = 32  # keep aspect ratio similar to workspace

    r = Robotarium(number_of_robots=N)

    xmin, xmax, ymin, ymax = workspace
    xs = np.linspace(xmin, xmax, grid_nx)
    ys = np.linspace(ymin, ymax, grid_ny)
    X, Y = np.meshgrid(xs, ys)
    points = np.vstack((X.ravel(), Y.ravel()))  # 2 x G grid points

    G = points.shape[1]

    for t in range(iterations):
        poses = r.get_poses()
        x = poses[0:2, :]  # 2 x N

        # ----- Discrete Voronoi: assign each grid point to nearest robot -----
        # points: 2 x G, x: 2 x N
        # Make shapes 2 x N x G for diffs
        #  points[:, None, :] -> 2 x 1 x G
        #  x[:, :, None]      -> 2 x N x 1
        #  broadcast subtract -> 2 x N x G
        diffs = points[:, None, :] - x[:, :, None]     # 2 x N x G
        dist2 = np.sum(diffs ** 2, axis=0)             # N x G
        owners = np.argmin(dist2, axis=0)              # length G

        # ----- Compute centroids for each robot's Voronoi cell -----
        centroids = np.zeros_like(x)
        for i in range(N):
            mask = owners == i
            if np.any(mask):
                pts_i = points[:, mask]                # 2 x (#cells)
                centroids[:, i] = pts_i.mean(axis=1)
            else:
                centroids[:, i] = x[:, i]

        # ----- Lloyd step: move toward centroids -----
        k = 1.0
        dxi = k * (centroids - x)
        dxu = si_to_uni(poses, dxi)

        r.set_velocities(range(N), dxu)
        r.step()

    r.debug()


if __name__ == "__main__":
    main()

