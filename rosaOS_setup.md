# ROS MCP Server — rosaOS Integration Guide

This directory is the [ros-mcp-server](https://github.com/Kushion32/ros-mcp-server) submodule.
It bridges ROS robots (ROS 1 and ROS 2) into the rosaOS agent system via the
[Model Context Protocol (MCP)](https://modelcontextprotocol.io/).

Any time a new robot is wanting to be added to the system, a new instance of this server should be launched, configure to connect to the rosbridge server running on the respective robot. The server will automatically allow MCP clients (such as the Reachy Mini MCP in our case) to discover the robot's ROS topics, services, actions, nodes, and camera feeds as MCP tools. This allows the LLM agents to interact with the robot naturally without having to know ROS specifics or modiying existing robot architecture/code.

---

## How it works

```
Robot (ROS 2)
  └── rosbridge_server (WebSocket, port 9090 on the robot)
        └── ros-mcp-server (streamable-http MCP server, one per robot)
              └── rosaOS kernel / worker agents (via config/drivers.json)
```

The ros-mcp-server connects to the robot's rosbridge WebSocket and exposes everything inward as MCP tools.

---

## Prerequisites

### On the robot (or Docker container running ROS)

Install and launch rosbridge. For ROS 2 Humble:

```bash
sudo apt update
sudo apt install ros-humble-rosbridge-server
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

Rosbridge listens on port 9090 by default

### On your laptop (running rosaOS)

```bash
cd server/ros-mcp-server
pip install -e .
```

Or with uv:

```bash
uv sync
```

---

## Starting a server instance

The entry point is `python -m ros_mcp.main`. All configuration is via command-line arguments.

```
python -m ros_mcp.main \
  --transport streamable-http \
  --host 0.0.0.0 \
  --port <MCP_PORT> \
  --rosbridge-ip <ROBOT_IP> \
  --rosbridge-port 9090 \
  --name <robot-name> \
  [--camera-topic <topic>]
```

| Argument           | Description                                     | Default                 |
| ------------------ | ----------------------------------------------- | ----------------------- |
| `--transport`      | MCP transport; use `streamable-http` for rosaOS | `stdio`                 |
| `--host`           | Interface to bind the MCP HTTP server on        | `127.0.0.1`             |
| `--port`           | Port for this MCP server instance               | `9000`                  |
| `--rosbridge-ip`   | IP address of the robot's rosbridge WebSocket   | required                |
| `--rosbridge-port` | Port of rosbridge on the robot                  | `9090`                  |
| `--name`           | Human-readable name, shown in logs              | `ros-mcp-server`        |
| `--camera-topic`   | Default topic for `capture_camera_image`        | `/image_raw/compressed` |

---

## Robot-specific examples

### HiWonder PuppyPi (robot dog)

The dog runs a ROS 2 workspace. Its rosbridge is accessible at `129.97.71.85:9090`.
Start the MCP server on port 9090 of the laptop:

```bash
cd server/ros-mcp-server
python -m ros_mcp.main \
  --transport streamable-http \
  --host 0.0.0.0 \
  --port 9090 \
  --rosbridge-ip 129.97.71.85 \
  --rosbridge-port 9090 \
  --name dog
```

Then register it in `config/drivers.json`:

```json
"dog": {
  "url": "http://localhost:9090/mcp",
  "description": "ROS MCP server for robot dog, can explore the environment"
}
```

And add `config/prompts/dog.txt` with movement instructions (already present in this repo).

**Key controls for the PuppyPi:**

Move a precise distance using the trigger service — this is the only reliable movement method:

```python
call_service(
    service="/puppy/move_distance_trigger",
    service_type="puppy_control_msgs/srv/MoveDistanceTrigger",
    args={"distance": 20, "speed": 0.5},  # distance in cm, positive = forward
    timeout=15
)
```

---

### iRobot Create 3 — TurtleBot4

The TurtleBot connects over VPN or local network. Its rosbridge runs on the robot at `10.37.84.255:9090`.
Start the MCP server on port 9091 of the laptop:

```bash
cd server/ros-mcp-server
python -m ros_mcp.main \
  --transport streamable-http \
  --host 0.0.0.0 \
  --port 9091 \
  --rosbridge-ip 10.37.84.255 \
  --rosbridge-port 9090 \
  --name turtlebot \
  --camera-topic /oakd/rgb/preview/image_raw/compressed
```

Then register it in `config/drivers.json`:

```json
"turtlebot": {
  "url": "http://localhost:9091/mcp",
  "description": "ROS MCP server for TurtleBot4, can explore and capture camera images. Camera tasks require Reachy Mini to describe images via describe_image."
}
```

And add `config/prompts/turtlebot.txt` with camera and docking instructions (already present).

**Key controls for the TurtleBot4:**

Undock and dock:

```python
send_action_goal(
    action="/undock",
    action_type="irobot_create_msgs/action/Undock",
    goal={}
)

send_action_goal(
    action="/dock",
    action_type="irobot_create_msgs/action/Dock",
    goal={}
)
```

---

## Running multiple robots at once

Each instance is an independent process. Start them in separate terminals (or add them
to `scripts/start_all.sh`):

```bash
# Terminal 1 — dog
python -m ros_mcp.main --transport streamable-http --host 0.0.0.0 --port 9090 \
  --rosbridge-ip 129.97.71.85 --name dog

# Terminal 2 — turtlebot
python -m ros_mcp.main --transport streamable-http --host 0.0.0.0 --port 9091 \
  --rosbridge-ip 10.37.84.255 --name turtlebot \
  --camera-topic /oakd/rgb/preview/image_raw/compressed

# Terminal 3 — a second turtlebot on a different IP
python -m ros_mcp.main --transport streamable-http --host 0.0.0.0 --port 9092 \
  --rosbridge-ip 10.37.84.100 --name turtlebot2 \
  --camera-topic /oakd/rgb/preview/image_raw/compressed
```

Add each to `config/drivers.json` under its own key with its unique port URL, and
optionally create a `config/prompts/<key>.txt`. The kernel will see them as separate
robots and can assign them independently to worker agents.

---
