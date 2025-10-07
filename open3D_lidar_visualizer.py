# open3D_lidar_visualizer.py
import open3d as o3d
import time
import numpy as np
from lidar_simulator import simulate_scan, NUM_BEES, POINTS_PER_BEE

# -----------------------------
# Create Open3D PointCloud object
# -----------------------------
def create_point_cloud(points):
    """Create Open3D PointCloud with colors: bees red, background green."""
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    colors = np.zeros_like(points)
    colors[:, 1] = 0.6  # background green

    # Bees are last points
    bee_start = points.shape[0] - NUM_BEES * POINTS_PER_BEE
    colors[bee_start:, 0] = 1.0  # red
    colors[bee_start:, 1] = 0.0
    colors[bee_start:, 2] = 0.0

    pcd.colors = o3d.utility.Vector3dVector(colors)
    return pcd

def visualize_scans(num_scans=200, interval=0.05):
    """Run interactive Open3D visualization for simulated scans."""
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="2D LiDAR Simulation", width=600, height=600)
    pcd = o3d.geometry.PointCloud()
    vis.add_geometry(pcd)

    for t in range(num_scans):
        points, bee_clusters = simulate_scan(t)
        pcd = create_point_cloud(points)
        vis.clear_geometries()
        vis.add_geometry(pcd)
        vis.poll_events()
        vis.update_renderer()
        time.sleep(interval)

    vis.destroy_window()
