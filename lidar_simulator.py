# lidar_simulator.py
import numpy as np

# -----------------------------
# Simulation parameters
# -----------------------------
NUM_BEES = 3
SCAN_RADIUS = 10.0
NOISE_STD = 0.05
POINTS_PER_BEE = 10
STATIC_POINTS = 200

# -----------------------------
# Generate static background
# -----------------------------
def generate_static_points():
    """Generate static background points (plants/rocks)."""
    angles = np.random.rand(STATIC_POINTS) * 2 * np.pi
    radii = np.random.rand(STATIC_POINTS) * SCAN_RADIUS
    x = radii * np.cos(angles)
    y = radii * np.sin(angles)
    z = np.zeros_like(x) # 2D plane
    return np.stack((x, y, z), axis=1)

# -----------------------------
# Generate moving bees
# -----------------------------
def bee_positions(t):
    """Generate positions of moving bees at time t."""
    positions = []
    for i in range(NUM_BEES):
        radius = 2 + i * 1.5
        speed = 0.05 + 0.02 * i
        angle = speed * t + i * np.pi / 4
        x = radius * np.cos(angle) + np.random.randn() * NOISE_STD
        y = radius * np.sin(angle) + np.random.randn() * NOISE_STD
        z = 0 # flat 2D plane
        positions.append([x, y, z])
    return np.array(positions)

# -----------------------------
# Simulate one LiDAR scan
# -----------------------------
def simulate_scan(t):
    """Simulate a single LiDAR scan with static points and moving bees."""
    points = generate_static_points()
    bees = bee_positions(t)
    bee_points = []
    for bx, by, bz in bees:
        x_noise = bx + np.random.randn(POINTS_PER_BEE) * 0.05
        y_noise = by + np.random.randn(POINTS_PER_BEE) * 0.05
        z_noise = bz + np.zeros(POINTS_PER_BEE)
        cluster = np.stack((x_noise, y_noise, z_noise), axis=1)
        bee_points.append(cluster)
        points = np.vstack((points, cluster))
    return points, bee_points
