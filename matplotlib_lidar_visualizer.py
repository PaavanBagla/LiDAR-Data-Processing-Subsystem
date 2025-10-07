# matplotlib_lidar_visualizer.py
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from lidar_simulator import simulate_scan, NUM_BEES, POINTS_PER_BEE

# -----------------------------
# Visualization function
# -----------------------------
def visualize_scans(num_scans=200, interval=0.05):
    """
    Visualize simulated 2D LiDAR scans using Matplotlib animation.
    
    Args:
        num_scans (int): Number of frames to animate.
        interval (int): Interval between frames in milliseconds.
    """
    fig, ax = plt.subplots(figsize=(6, 6))
    bg_scat = ax.scatter([], [], s=8, c='green', label='Background')
    bee_scat = ax.scatter([], [], s=20, c='red', label='Bees')
    
    # Set plot limits and labels
    SCAN_RADIUS = 10.0
    ax.set_xlim(-SCAN_RADIUS, SCAN_RADIUS)
    ax.set_ylim(-SCAN_RADIUS, SCAN_RADIUS)
    ax.set_aspect('equal')
    ax.set_title("Simulated 2D LiDAR Scan with Pollinators")
    ax.legend(loc='upper right')

    def update(frame):
        points, bee_clusters = simulate_scan(frame)
        
        # Separate background vs bee points for coloring
        bg_points = points[:-NUM_BEES * POINTS_PER_BEE]  # last points are bees
        all_bees = np.vstack(bee_clusters)
        
        bg_scat.set_offsets(bg_points)
        bee_scat.set_offsets(all_bees)
        return bg_scat, bee_scat

    ani = FuncAnimation(fig, update, frames=num_scans, interval=interval, blit=True)
    plt.show()
