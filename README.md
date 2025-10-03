# MCP Server Setup Guide

This document details the steps taken to set up the ROS-MCP server, configure Claude Desktop as an MCP client, and test integration with ROS 2 (Turtlesim). The setup follows the official installation guides for both the MCP server and Claude Desktop, with notes on issues and resolutions.

---

## References

- ROS-MCP Server Installation Guide: [https://github.com/robotmcp/ros-mcp-server/blob/main/docs/installation.md](https://github.com/robotmcp/ros-mcp-server/blob/main/docs/installation.md)
- Claude Desktop Linux Setup: [https://github.com/aaddrick/claude-desktop-debian](https://github.com/aaddrick/claude-desktop-debian)

---

## Prerequisites

Before starting, ensure the following:

- **Operating System**: Ubuntu 22.04 or higher
- **ROS 2**: ROS 2 Jazzy (or newer)
- **Python 3**: Required for `uv` and MCP server
- **Git**: To clone repositories
- **Basic build tools**: `build-essential`, `curl`, `python3-pip`  

> ⚠️ You need either ROS installed locally on your machine **OR** access over the network to a robot/computer with ROS installed. The MCP server connects to ROS systems on a robot, so a running ROS environment is required.

---

## 1. Install the MCP Server (Host machine where the LLM runs)

### 1.1. Clone the Repository

```bash
git clone https://github.com/robotmcp/ros-mcp-server.git
```
> ⚠️ Note: Save the absolute path to this directory for later configuration in Claude Desktop.

### 1.2. Install UV (Python Virtual Environment Manager)
Option A: Shell installer
```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```
Then restart your shell or source your .bashrc so uv is in PATH.
- Verify Installation:
```bash
uv --version
```
- Install required uv packages for the ROS-MCP server:
```bash
uv pip install -e .
```
---

## 2. Install and Configure a Language Model Client (Claude Desktop)

### 2.1. Download Claude Desktop (Linux/Ubuntu)
Follow instructions from: Claude Desktop Linux Setup: [https://github.com/aaddrick/claude-desktop-debian](https://github.com/aaddrick/claude-desktop-debian)

Steps:
```bash
git clone https://github.com/aaddrick/claude-desktop-debian.git
cd claude-desktop-debian

# Build default .deb package
./build.sh

# Install the package
sudo apt install ./claude-desktop_0.13.37_amd64.deb
```
> ⚠️ Warning seen:
```
N: Download is performed unsandboxed as root as file '...' couldn't be accessed by user '_apt'. - pkgAcquire::Run (13: Permission denied)
```
> ✅ This can be safely ignored.

Verify installation:
```bash
which claude-desktop
# Expected: /usr/bin/claude-desktop

# Or run the app
claude-desktop
```

### 2.2. Configure MCP Server in Claude Desktop
Edit or create the configuration file:
```bash
nano ~/.config/Claude/claude_desktop_config.json
```
Add the MCP server section:
```bash
{
  "mcpServers": {
    "ros-mcp-server": {
      "command": "uv",
      "args": [
        "--directory",
        "/home/paavan/TURTLE/ros-mcp-server",
        "run",
        "server.py"
      ]
    }
  }
}
```
> ⚠️ Note: Ensure the directory path matches the location of your cloned MCP server.

### 2.3. Test the Connection
- Launch Claude Desktop.
- The ros-mcp-server should appear in the tool list.
- You can now send commands to ROS via natural language.
<img width="614" height="640" alt="Image" src="https://github.com/user-attachments/assets/64522a5f-3637-40bc-a45c-7b5ee9657e43" />

---

## 3. Install and Run rosbridge (On ROS machine / same machine)

### 3.1. Install rosbridge_server

```bash
sudo apt install ros-${ROS_DISTRO}-rosbridge-server
```
> This package allows the MCP server to communicate with ROS over WebSockets.

### 3.2. Launch rosbridge_server
Source ROS:
```bash
source /opt/ros/jazzy/setup.bash
```
Launch the ROS bridge server:
```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```
> ✅ Expected Output:
```
[rosbridge_websocket-1] Rosbridge WebSocket server started on port 9090
```

---

## 6. Test Workflow with Turtlesim
Run the following in separate terminals:
1. Terminal 1: Start rosbridge server
```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

2. Terminal 2: Launch Claude Desktop
```bash
claude-desktop
```

3. Terminal 3: Start Turtlesim
```bash
ros2 run turtlesim turtlesim_node
```

Now, interact with your robot using Claude:
- Query topics:
```bash
What topics are currently available on the robot?
```
- Command Actions:
```bash
Make the robot move.
```
> Claude communicates with ROS via the MCP server, which internally uses rosbridge WebSocket to interface with ROS nodes.

---

## 7. Observed Issues and Fixes

| Issue | Symptoms | Solution |
|-------|----------|---------|
| Installing rosbridge_server | COnflicting with Conda Libs | Don't use conda env |

---

## 8. Summary

1. MCP server is installed and running.
2. Claude Desktop is configured to launch MCP server.
3. rosbridge is running on ROS.
4. Turtlesim is running as the test robot.
5. Claude can now send ROS commands and query topics successfully.

---

This guide documents installation, configuration, testing, and troubleshooting of the ROS-MCP server with Claude Desktop and ROS 2 integration.
