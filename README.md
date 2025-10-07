# 🌾 LiDAR Data Processing & Pollination Heatmaps System

**Author:** Paavan Bagla  
**Subsystem:** LiDAR Data Processing & Pollination Heatmaps  
**Project:** AgriPollinate Capstone

---

## 🚀 Overview

This subsystem processes **LiDAR data** to detect pollinator flight activity and generate **real-time pollination density heatmaps**.  
It forms a core component of the AgriPollinate system — helping visualize pollination effectiveness across a crop field.

### What it does:
- Processes **cleaned LiDAR point cloud data** from cloud storage (AWS / GCP).  
- Applies **ML/AI models** to detect and map pollinator flight patterns.  
- Generates **real-time heatmaps** and statistics on pollinator density.  
- Outputs data to the **Camera Processing & UI Subsystem** for visualization and validation.

---

## 🧠 System Design

### 🔹 Pipeline
SICK TiM561 LiDAR → Data Cleaning (SBC Subsystem) →
LiDAR Data Processing (This Subsystem) →
ML/AI Density Mapping →
Heatmap Visualization (UI Subsystem)

### 🔹 Tools & Frameworks
| Tool | Purpose |
|------|----------|
| **Python** | Main language for data processing and ML |
| **TensorFlow** | Machine Learning / AI inference engine |
| **Open3D** | Point cloud data visualization & processing |
| **NumPy / Pandas** | Data manipulation |
| **Matplotlib / Plotly** | Heatmap generation |
| **AWS / GCP** | Cloud-based compute and data pipeline |

---

## 🎯 Objectives, Constraints & Metrics

| Category | Description |
|-----------|--------------|
| **Objectives** | Generate accurate, real-time pollination insights |
| **Constraints** | Limited bandwidth, LiDAR noise, power efficiency |
| **Performance Metrics** | Pollinator detection accuracy ≥ 85%; Heatmap refresh rate ≤ 5 sec |

---

## 🔗 Integration with Other Subsystems

| Input | Output | Dependency |
|--------|---------|-------------|
| Cleaned LiDAR data (from **SBC Subsystem**) | Heatmaps & pollination density stats | Must sync with **Camera Processing & UI Subsystem** for cross-validation |

---

## ⚙️ Setup Instructions

### 1. Clone Repository
```bash
git clone https://github.com/paavanbagla/LiDAR-Data-Processing-Subsystem.git
cd LiDAR-Data-Processing-Subsystem
```
### 2. Create and Activate Conda Environment
```bash
conda create -n lidar_env python=3.10 -y
conda activate lidar_env
```
### 3. Install Dependencies
```bash
pip install -r requirements.txt
```
### 4. Run Main Script
```bash
python main.py
```

## 🧩 File Structure
```bash
LiDAR-Data-Processing-Subsystem/
│
├── main.py                # Entry point for LiDAR data processing pipeline
├── requirements.txt       # Python dependencies
├── capstone_env/          # Optional environment setup/config files
├── README.md              # Documentation (this file)
```

---

## 🧪 Next Steps / Development Roadmap
- Integrate with cloud-based LiDAR data source (AWS S3 / GCP Storage)
- Implement preprocessing & denoising of LiDAR point clouds
- Train ML model for pollinator flight detection
- Develop real-time heatmap visualization dashboard
- Interface with Camera & UI subsystems for validation

---

## 🛰️ Hardware Reference
- **LiDAR Model**: SICK TiM561 2D LiDAR (Mounted top-down)
- **Processing Hardware**: SBC / Edge GPU
- **Data Interface**: Ethernet or ROS2 message stream (to be defined)

---

## 📈 Expected Deliverables
- Real-time pollination heatmaps (2D spatial visualizations)
- Pollinator density metrics (zones of over/under-pollination)
- Cloud integration pipeline for scalable compute
- Documentation for deployment and validation
