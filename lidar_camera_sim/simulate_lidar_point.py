"""
simulate_lidar_point.py
-----------------------
Step 1: Simulate a known 3D point (bee) in a 2D LiDAR plane.

This script creates:
  - A fixed 'bee' position in LiDAR space.
  - A surrounding 2D point cloud (z=0) to represent the environment.
  - Returns the 3D coordinates (x, y, z) in meters.
  - Uses PyVista for interactive visualization (compatible with macOS).
"""

import numpy as np
import pyvista as pv


def simulate_lidar_environment_2d(show=False):
    """
    Creates a simple 2D LiDAR environment with one bee at a known 3D position.

    Args:
        show (bool): If True, visualize the point cloud using PyVista.

    Returns:
        bee_lidar (np.ndarray): True 3D coordinate of the bee in LiDAR frame [x, y, z].
        lidar_points (np.ndarray): Nx3 point cloud (environment + bee).
    """

    # --- Define LiDAR coordinate frame ---
    # Assume LiDAR at origin (0,0,0), facing forward along +X.
    # Units = meters.

    # Ground truth bee location (meters) in 2D plane (z=0)
    bee_lidar = np.array([10.0, 1.5, 0.0])  # x=2m forward, y=0.5m right, z=0m

    # --- Simulate a simple 2D background point cloud (z=0) ---
    num_points = 200
    ground = np.column_stack((
        np.random.uniform(0, 20, num_points),   # x from 0 to 4m
        np.random.uniform(-2, 2, num_points),  # y from -2 to 2m
        np.zeros(num_points)                    # z = 0 for 2D LiDAR
    ))

    # Combine ground points + bee
    lidar_points = np.vstack((ground, bee_lidar))

    # --- Optional visualization ---
    if show:
        # Create PyVista point cloud for the environment
        cloud = pv.PolyData(lidar_points)

        # Create PyVista sphere for the bee
        bee_sphere = pv.Sphere(radius=0.05, center=bee_lidar)

        # Create the plotter window
        plotter = pv.Plotter()
        plotter.add_mesh(cloud, color="green", point_size=5, render_points_as_spheres=True)
        plotter.add_mesh(bee_sphere, color="red", specular=0.5, smooth_shading=True)

        # Add helpful visuals
        plotter.add_axes(line_width=3)
        plotter.show_grid()
        plotter.show(title="2D LiDAR Environment")

    return bee_lidar, lidar_points


if __name__ == "__main__":
    bee_lidar, lidar_points = simulate_lidar_environment_2d(show=True)
    print("✅ Simulated 2D LiDAR Environment")
    print("Bee position (LiDAR frame):", bee_lidar)
    print("Total points:", lidar_points.shape[0])
    print("Close the visualization window to exit.")