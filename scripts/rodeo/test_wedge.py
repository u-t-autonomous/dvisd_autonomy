import numpy as np

def select_best_subwedge_batch(r, theta, wedge_min, wedge_max, n_subwedges=5, epsilon=1e-6):
    """
    Vectorized selection of sub-wedge with minimal cost.

    Args:
        r: np.array of distances of LiDAR points, shape [n_points]
        theta: np.array of angles of LiDAR points, shape [n_points]
        wedge_min: min angle of wedge (rad)
        wedge_max: max angle of wedge (rad)
        n_subwedges: number of sub-wedges
        epsilon: small value to avoid division by zero

    Returns:
        best_idx: index of sub-wedge with minimal cost
        costs: np.array of cost per sub-wedge
        sub_wedges: list of (theta_min, theta_max)
    """
    n_points = r.shape[0]

    # Define sub-wedges
    sub_edges = np.linspace(wedge_min, wedge_max, n_subwedges + 1)
    sub_wedges = [(sub_edges[i], sub_edges[i+1]) for i in range(n_subwedges)]

    # Broadcast theta to [n_subwedges, n_points]
    theta_exp = theta[None, :]  # [1, n_points]
    sub_min = np.array([sw[0] for sw in sub_wedges])[:, None]  # [n_subwedges,1]
    sub_max = np.array([sw[1] for sw in sub_wedges])[:, None]

    # Create mask: which points belong to which sub-wedge
    mask = (theta_exp >= sub_min) & (theta_exp < sub_max)  # [n_subwedges, n_points]

    # Compute cost for all wedges in batch
    # Closer points -> higher cost, use 1/r
    r_exp = np.broadcast_to(r, mask.shape)  # [n_subwedges, n_points]
    masked_r = np.where(mask, r_exp, np.inf)  # points not in wedge -> ignored
    costs = np.sum(1.0 / (masked_r + epsilon), axis=1)  # sum over points for each wedge

    # Empty wedge has cost 0 (no points)
    costs = np.where(np.isfinite(costs), costs, 0.0)

    # Best wedge index
    best_idx = np.argmin(costs)
    return best_idx, costs, sub_wedges

# ---------------- Example usage ----------------
if __name__ == "__main__":
    np.random.seed(42)
    n_points = 10000  # large number of LiDAR points
    r = np.random.uniform(0.5, 10.0, size=n_points)
    theta = np.random.uniform(-np.pi/4, np.pi/4, size=n_points)

    wedge_min = -np.pi/6
    wedge_max = np.pi/6
    n_subwedges = 12

    best_idx, costs, sub_wedges = select_best_subwedge_batch(r, theta, wedge_min, wedge_max, n_subwedges)
    print("Best sub-wedge index:", best_idx)
    print("Best sub-wedge range:", sub_wedges[best_idx])