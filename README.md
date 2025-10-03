# MCP Server Setup Guide

This document details the steps taken to set up the ROS-MCP server, configure Claude Desktop as an MCP client, and test integration with ROS 2 (Turtlesim). The setup follows the official installation guides for both the MCP server and Claude Desktop, with notes on issues and resolutions.

---

## References

- ROS-MCP Server Installation Guide: [https://github.com/robotmcp/ros-mcp-server/blob/main/docs/installation.md](https://github.com/robotmcp/ros-mcp-server/blob/main/docs/installation.md)
- Claude Desktop Linux Setup: [https://github.com/aaddrick/claude-desktop-debian](https://github.com/aaddrick/claude-desktop-debian)

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
| Python shebang mismatch | rosbridge and rosapi nodes fail with `FileNotFoundError` | Updated shebang to `#!/usr/bin/python3` |
| ros-mcp-server not appearing in Claude | MCP tool missing | Ensure JSON config uses absolute path; restart Claude Desktop |
| curl to `localhost:9090/topics` returns 404 | rosbridge WebSocket only responds to MCP/ROS requests | Use MCP or rosbridge WebSocket clients instead |
| Turtlesim QSocketNotifier warning | `Can only be used with threads started with QThread` | Ignored; Turtlesim still works |
| Debian install warning | Unsandboxed download warning | Ignored; package installed successfully |

---

This guide documents installation, configuration, testing, and troubleshooting of
