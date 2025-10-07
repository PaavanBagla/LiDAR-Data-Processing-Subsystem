# main.py
from open3D_lidar_visualizer import visualize_scans # for open3D visualizer
# from matplotlib_lidar_visualizer import visualize_scans # for matplotlib visualizer

def main():
    print("Starting LiDAR Simulation with moving pollinators...")
    visualize_scans(num_scans=200, interval=0.05) # for open3D visualizer
    # visualize_scans(num_scans=200, interval=50) # for matplotlib visualizer

if __name__ == "__main__":
    main()
