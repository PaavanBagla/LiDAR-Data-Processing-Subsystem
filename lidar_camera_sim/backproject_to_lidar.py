"""
backproject_to_lidar.py
-----------------------
Step 3 + Step 4: Map 2D camera pixel + depth back to LiDAR coordinates
and compare with ground truth.
"""

import numpy as np
import cv2
from project_to_camera import get_camera_calibration
from simulate_lidar_point import simulate_lidar_environment_2d
from project_to_camera import project_lidar_to_image

def backproject_pixel_to_lidar(u, v, depth, K, R, t):
    """
    Back-project pixel to 3D LiDAR coordinates.

    Args:
        u, v (float): pixel coordinates
        depth (float): depth along camera Z-axis
        K, R, t: camera calibration

    Returns:
        X_lidar (np.ndarray): 3D LiDAR coordinates [x, y, z]
    """
    fx, fy = K[0,0], K[1,1]
    cx, cy = K[0,2], K[1,2]

    # Normalized camera coordinates
    x_norm = (u - cx) / fx
    y_norm = (v - cy) / fy

    # 3D point in camera frame
    X_cam = np.array([x_norm * depth,
                      y_norm * depth,
                      depth]).reshape(3,1)

    # Camera → LiDAR
    X_lidar = np.linalg.inv(R) @ (X_cam - t)
    return X_lidar.flatten()


if __name__ == "__main__":
    # USER SETTINGS
    CONTROL_LIDAR_SHOW = False    # Show LiDAR environment
    CONTROL_IMAGE_SHOW = False    # Show synthetic camera image

    # Step 1: Simulate LiDAR environment
    bee_lidar, _ = simulate_lidar_environment_2d(show=CONTROL_LIDAR_SHOW)
    
    # Step 2: Project to camera
    K, R, t = get_camera_calibration()
    pixel, depth, image = project_lidar_to_image(bee_lidar, K, R, t)
    # Show camera image if user wants
    if CONTROL_IMAGE_SHOW:
        cv2.imshow("Camera View", image)
        print("Press 'q' to close the window.")
        while True:
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        cv2.destroyAllWindows()

    u, v = pixel

    # Step 3: Back-project
    recovered_lidar = backproject_pixel_to_lidar(u, v, depth, K, R, t)

    # Step 4: Compare
    error = np.linalg.norm(recovered_lidar - bee_lidar)

    print("Ground truth LiDAR:", bee_lidar)
    print("Pixel coordinates:", pixel)
    print("Depth (camera Z):", depth)
    print("Recovered LiDAR:", recovered_lidar)
    print("Euclidean error:", error)
