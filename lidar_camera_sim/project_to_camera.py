"""
project_to_camera.py
-------------------
Step 2: Project a 3D LiDAR point (bee) into a camera image using calibration.

Inputs:
  - bee_lidar: 3D position in LiDAR frame
  - Camera intrinsic + extrinsic parameters

Outputs:
  - pixel: (u, v) coordinates in image frame
  - depth: distance along camera z-axis
  - image: synthetic image showing projected bee
"""

import numpy as np
import cv2

def get_camera_calibration():
    """
    Returns a simple example camera calibration with camera looking straight ahead.

    Returns:
        K (np.ndarray): 3x3 intrinsic matrix
        R (np.ndarray): 3x3 rotation LiDAR->Camera
        t (np.ndarray): 3x1 translation LiDAR->Camera
    """
    # --- Intrinsics ---
    fx = 800  # focal length x (pixels)
    fy = 800  # focal length y (pixels)
    cx = 320  # principal point x
    cy = 240  # principal point y
    K = np.array([[fx, 0, cx],
                  [0, fy, cy],
                  [0,  0,  1]])

    # --- Extrinsics ---
    # Camera looking along LiDAR +X, with standard OpenCV axes:
    # X_cam: right, Y_cam: down, Z_cam: forward
    # R: rotate LiDAR to camera frame
    R = np.array([[0, -1, 0],   # LiDAR y → -X_cam
                  [0, 0, -1],   # LiDAR z → -Y_cam
                  [1, 0, 0]])   # LiDAR x → Z_cam forward

    # Camera slightly above LiDAR plane
    t = np.array([[0.0], [0.1], [0.0]])
    return K, R, t


def project_lidar_to_image(bee_lidar, K, R, t, img_w=640, img_h=480):
    """
    Projects a 3D LiDAR point into 2D camera image coordinates.

    Args:
        bee_lidar (np.ndarray): [x, y, z] LiDAR coordinate
        K, R, t: camera calibration
        img_w, img_h: image dimensions

    Returns:
        pixel (np.ndarray): [u, v] pixel coordinates
        depth (float): depth along camera z-axis
        image (np.ndarray): synthetic image with projected point
    """
    # Convert bee to camera frame
    X_cam = R @ bee_lidar.reshape(3,1) + t  # 3x1
    depth = X_cam[2,0]  # Z in camera frame

    # Project to pixel
    pixel_homog = K @ X_cam
    u = pixel_homog[0,0] / pixel_homog[2,0]
    v = pixel_homog[1,0] / pixel_homog[2,0]
    pixel = np.array([u, v])

    # Create synthetic image
    image = np.zeros((img_h, img_w, 3), dtype=np.uint8)
    u_int, v_int = int(round(u)), int(round(v))
    if 0 <= u_int < img_w and 0 <= v_int < img_h:
        cv2.circle(image, (u_int, v_int), 5, (0, 255, 0), -1)
    else:
        print("Warning: projected point outside image!")

    return pixel, depth, image

# --- Example usage ---
if __name__ == "__main__":
    from simulate_lidar_point import simulate_lidar_environment_2d

    # Simulate 2D LiDAR environment
    bee_lidar, _ = simulate_lidar_environment_2d(show=False)

    # Move bee slightly up for better projection
    bee_lidar[2] = 0.5

    # Get camera calibration
    K, R, t = get_camera_calibration()

    # Project bee to camera
    pixel, depth, image = project_lidar_to_image(bee_lidar, K, R, t)

    print("Bee LiDAR coordinates:", bee_lidar)
    print("Projected pixel:", pixel)
    print("Depth in camera frame:", depth)

    # Show synthetic image with safe exit
    cv2.imshow("Camera View", image)
    print("Press 'q' to close the window.")
    while True:
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cv2.destroyAllWindows()
