# starter_pyvista.py
import pyvista as pv
import numpy as np
import tensorflow as tf

def load_dummy_2d_lidar():
    """
    Simulate a 2D LiDAR scan.
    Returns:
        pc: Nx3 array of points (x, y, z=0)
    """
    num_points = 360  # 1-degree resolution
    angles = np.linspace(-np.pi, np.pi, num_points, endpoint=False)  # -180 to +180 degrees (exclusive of the end)
    # Using np.random.uniform to get varying distances
    distances = np.random.uniform(0.5, 7.0, size=num_points)  # distances up to 7 meters

    x = distances * np.cos(angles)
    y = distances * np.sin(angles)
    z = np.zeros_like(x)  # z = 0 for 2D plane

    pc = np.stack((x, y, z), axis=1)
    return pc

def visualize_point_cloud_2d_pyvista(pc):
    """
    Visualize 2D LiDAR points using PyVista
    """
    # Create a PyVista PolyData object from the numpy array
    # PyVista is excellent at handling numpy arrays for plotting
    point_cloud = pv.PolyData(pc)

    # Create a plotter object
    plotter = pv.Plotter()

    # Add the point cloud to the plotter
    # We specify the color and point size for clear visualization
    plotter.add_mesh(
        point_cloud,
        color='red',           # Set points color to red
        point_size=10,         # Increase point size for better visibility
        render_points_as_spheres=True  # Render points as spheres for a better look
    )
    
    # Optional: Set the camera view for a better 2D perspective (looking down the Z-axis)
    plotter.view_xy() 
    
    # Optional: Add a title
    plotter.add_title("2D LiDAR Point Cloud (PyVista)")

    # Display the plot
    # The 'show()' method opens an interactive window
    plotter.show()

def main():
    print("Starting 2D LiDAR Data Processing Subsystem (PyVista)...")
    
    # Load dummy 2D LiDAR data
    pc = load_dummy_2d_lidar()
    print(f"Point cloud shape: {pc.shape}")
    
    # Visualize 2D LiDAR point cloud using PyVista
    visualize_point_cloud_2d_pyvista(pc)
    
    # Placeholder for ML model
    model = tf.keras.Sequential([
        tf.keras.layers.Dense(32, activation='relu', input_shape=(3,)),  # input is (x, y, z)
        tf.keras.layers.Dense(1, activation='sigmoid')
    ])
    model.summary()
    
if __name__ == "__main__":
    main()