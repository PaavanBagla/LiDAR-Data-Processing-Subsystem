# starter.py
import open3d as o3d
import numpy as np
import tensorflow as tf

def load_dummy_2d_lidar():
    """
    Simulate a 2D LiDAR scan.
    Returns:
        pc: Nx3 array of points (x, y, z=0)
    """
    num_points = 360  # 1-degree resolution
    angles = np.linspace(-np.pi, np.pi, num_points)  # -180 to +180 degrees
    distances = np.random.uniform(0.5, 7.0, size=num_points)  # distances up to 7 meters

    x = distances * np.cos(angles)
    y = distances * np.sin(angles)
    z = np.zeros_like(x)  # z = 0 for 2D plane

    pc = np.stack((x, y, z), axis=1)
    return pc

def visualize_point_cloud_2d(pc):
    """
    Visualize 2D LiDAR points in Open3D
    """
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pc)
    
    # Optionally color points red
    colors = np.tile(np.array([[1.0, 0, 0]]), (pc.shape[0], 1))  # red
    pcd.colors = o3d.utility.Vector3dVector(colors)
    
    o3d.visualization.draw_geometries([pcd], window_name="2D LiDAR Point Cloud")

def main():
    print("Starting 2D LiDAR Data Processing Subsystem (Open3D)...")
    
    # Load dummy 2D LiDAR data
    pc = load_dummy_2d_lidar()
    print(f"Point cloud shape: {pc.shape}")
    
    # Visualize 2D LiDAR point cloud
    visualize_point_cloud_2d(pc)
    
    # Placeholder for ML model
    model = tf.keras.Sequential([
        tf.keras.layers.Dense(32, activation='relu', input_shape=(3,)),  # input is (x, y, z)
        tf.keras.layers.Dense(1, activation='sigmoid')
    ])
    model.summary()
    
if __name__ == "__main__":
    main()
