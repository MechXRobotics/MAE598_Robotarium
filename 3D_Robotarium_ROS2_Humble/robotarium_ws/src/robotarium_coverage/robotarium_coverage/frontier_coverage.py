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


def kmeans(points, K, max_iters=15):
    """
    points: 2 x M array
    returns: cluster_idx (M,), centers (K x 2)
    """
    M = points.shape[1]
    # choose K random unique points as initial centers
    inds = np.random.choice(M, K, replace=False)
    centers = points[:, inds].T  # K x 2

    for _ in range(max_iters):
        # assign
        diff = points.T[:, None, :] - centers[None, :, :]  # M x K x 2
        dist2 = np.sum(diff ** 2, axis=2)                  # M x K
        labels = np.argmin(dist2, axis=1)                  # M

        # recompute
        new_centers = np.zeros_like(centers)
        for k in range(K):
            mask = labels == k
            if np.any(mask):
                new_centers[k, :] = points[:, mask].mean(axis=1)
            else:
                new_centers[k, :] = centers[k, :]
        centers = new_centers

    return labels, centers


def main():
    N = 8
    iterations = 2000
    workspace = np.array([-1.0, 1.0, -0.8, 0.8])
    grid_res = 50

    r = Robotarium(number_of_robots=N)

    xmin, xmax, ymin, ymax = workspace
    occ = np.zeros((grid_res, grid_res), dtype=np.int32)

    xs = np.linspace(xmin, xmax, grid_res)
    ys = np.linspace(ymin, ymax, grid_res)

    for t in range(iterations):
        poses = r.get_poses()
        x = poses[0:2, :]

        # --- Mark explored cells ---
        for i in range(N):
            gx = int(round((x[0, i] - xmin) / (xmax - xmin) * (grid_res - 1)))
            gy = int(round((x[1, i] - ymin) / (ymax - ymin) * (grid_res - 1)))
            gx = np.clip(gx, 0, grid_res - 1)
            gy = np.clip(gy, 0, grid_res - 1)
            occ[gy, gx] = 1

        # --- Find frontier cells ---
        frontier_idx = []
        for iy in range(1, grid_res - 1):
            for ix in range(1, grid_res - 1):
                if occ[iy, ix] == 1:
                    if np.any(occ[iy-1:iy+2, ix-1:ix+2] == 0):
                        frontier_idx.append((iy, ix))

        if not frontier_idx:
            # no more frontier
            r.set_velocities(range(N), np.zeros((2, N)))
            r.step()
            continue

        frontier_idx = np.array(frontier_idx)  # M x 2
        fy_ind = frontier_idx[:, 0]
        fx_ind = frontier_idx[:, 1]
        frontier_xy = np.vstack((xs[fx_ind], ys[fy_ind]))  # 2 x M

        M = frontier_xy.shape[1]
        K = min(N, M)

        # --- K-means clustering of frontier points ---
        labels, centers = kmeans(frontier_xy, K)  # centers: K x 2

        # --- Choose medoids: closest actual frontier point to each center ---
        medoids = np.zeros((2, K))
        for k in range(K):
            pts = frontier_xy[:, labels == k]
            if pts.shape[1] == 0:
                medoids[:, k] = centers[k, :]
                continue
            diffs = pts - centers[k, :].reshape(2, 1)
            dist2 = np.sum(diffs ** 2, axis=0)
            best = np.argmin(dist2)
            medoids[:, k] = pts[:, best]

        # --- Assign each robot to nearest medoid (simpler than Hungarian) ---
        targets = np.zeros_like(x)
        for i in range(N):
            diffs = medoids - x[:, i].reshape(2, 1)
            dist2 = np.sum(diffs ** 2, axis=0)
            idx = np.argmin(dist2)
            targets[:, i] = medoids[:, idx]

        # --- Escape forces to avoid blocking ---
        escape = np.zeros_like(x)
        for i in range(N):
            for j in range(N):
                if i == j:
                    continue
                d = x[:, i] - x[:, j]
                dist = np.linalg.norm(d)
                if dist < 0.2:
                    escape[:, i] += 0.05 * d / (dist + 1e-6)

        # --- Control law ---
        k = 1.0
        dxi = k * (targets + escape - x)
        dxu = si_to_uni(poses, dxi)

        r.set_velocities(range(N), dxu)
        r.step()

    r.debug()


if __name__ == "__main__":
    main()
