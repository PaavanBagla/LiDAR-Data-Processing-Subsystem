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

---

## 2. Install ROS Bridge

### Steps

1. Install the ROS bridge server:

```bash
sudo apt install ros-jazzy-rosbridge-server ros-jazzy-rosapi
```

2. Verify installation:

```bash
ls -l /opt/ros/jazzy/lib/rosbridge_server/
# Should contain rosbridge_websocket and rosbridge_websocket.py

ls -l /opt/ros/jazzy/lib/rosapi/
# Should contain rosapi_node
```

3. Fix Python shebang for Python 3.12 (Ubuntu default):

```bash
sudo sed -i '1s|.*|#!/usr/bin/python3|' /opt/ros/jazzy/lib/rosbridge_server/rosbridge_websocket
sudo sed -i '1s|.*|#!/usr/bin/python3|' /opt/ros/jazzy/lib/rosapi/rosapi_node
```

4. Source ROS:

```bash
source /opt/ros/jazzy/setup.bash
```

5. Launch the ROS bridge server:

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

> ✅ Expected Output:
```
[rosbridge_websocket-1] Rosbridge WebSocket server started on port 9090
```

---

## 3. Clone and Install MCP Server

1. Clone the MCP server repository:

```bash
git clone https://github.com/robotmcp/ros-mcp-server.git
cd ros-mcp-server
```

2. Install `uv` (Python virtual environment manager):

```bash
pip install uv
```

3. Verify:

```bash
uv --version
# uv 0.8.22
```

---

## 4. Install Claude Desktop

1. Clone or download the Linux build of Claude Desktop:

```bash
git clone https://github.com/aaddrick/claude-desktop-debian.git
cd claude-desktop-debian
```

2. Install the `.deb` package:

```bash
sudo apt install ./claude-desktop_0.13.37_amd64.deb
```

> ⚠️ Warning seen:
```
N: Download is performed unsandboxed as root as file '...' couldn't be accessed by user '_apt'. - pkgAcquire::Run (13: Permission denied)
```
> ✅ This can be safely ignored.

---

## 5. Configure Claude Desktop to launch MCP server

1. Locate or create the config file:

```bash
~/.config/Claude/claude_desktop_config.json
```

2. Add the following under `"mcpServers"` (replace `<ABSOLUTE_PATH>` with your MCP server path):

```json
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

3. Launch Claude Desktop and verify that `ros-mcp-server` appears in the tools list.

---

## 6. Test with Turtlesim

1. Open a terminal and launch Turtlesim:

```bash
ros2 run turtlesim turtlesim_node
```

2. From Claude Desktop, issue the query:

```
What topics are available on the robot?
```

- MCP server should return ROS topics:
```
/turtle1/cmd_vel
/turtle1/pose
/rosout
```

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

## 8. Notes

- MCP server communicates with ROS via rosbridge WebSocket (STDIO transport by default).  
- Claude Desktop communicates with MCP server using STDIO (configured in JSON).  
- You can issue natural language commands to control robots:

```plaintext
Make the robot move forward.
```

- Or query ROS state:

```plaintext
What topics and services are available?
```

---

## 9. Next Steps / Tips

- For testing, use Turtlesim as a simple ROS “hello world.”  
- For remote robots, configure HTTP transport for MCP server.  
- Check logs with:

```bash
ps aux | grep rosbridge
ps aux | grep ros-mcp
```

---

This guide documents installation, configuration, testing, and troubleshooting of
