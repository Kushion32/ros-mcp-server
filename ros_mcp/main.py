"""ROS MCP Server - MCP instance and main entry point.

This module provides the FastMCP server instance and main() function.
"""

import argparse
import sys

from fastmcp import FastMCP

from ros_mcp.prompts import register_all_prompts
from ros_mcp.resources import register_all_resources
from ros_mcp.tools import register_all_tools
from ros_mcp.utils.websocket import WebSocketManager

# Default ROS bridge connection settings
DEFAULT_ROSBRIDGE_IP = "129.97.71.85"
DEFAULT_ROSBRIDGE_PORT = 9090


def parse_arguments():
    """Parse command line arguments for MCP server configuration."""
    parser = argparse.ArgumentParser(
        description="ROS MCP Server - Connect to ROS robots via MCP protocol",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python -m ros_mcp.main                                    # Use stdio transport (default)
  python -m ros_mcp.main --transport http --host 0.0.0.0 --port 9000
  python -m ros_mcp.main --transport streamable-http --host 127.0.0.1 --port 8080
  python -m ros_mcp.main --transport streamable-http --host 0.0.0.0 --port 9091 --rosbridge-ip 10.0.0.2 --rosbridge-port 9090
        """,
    )

    parser.add_argument(
        "--transport",
        choices=["stdio", "http", "streamable-http", "sse"],
        default="stdio",
        help="MCP transport protocol to use (default: stdio)",
    )

    parser.add_argument(
        "--host",
        default="127.0.0.1",
        help="Host address for HTTP-based transports (default: 127.0.0.1)",
    )

    parser.add_argument(
        "--port",
        type=int,
        default=9000,
        help="Port number for HTTP-based transports (default: 9000)",
    )

    parser.add_argument(
        "--rosbridge-ip",
        default=DEFAULT_ROSBRIDGE_IP,
        help=f"Rosbridge WebSocket IP (default: {DEFAULT_ROSBRIDGE_IP})",
    )

    parser.add_argument(
        "--rosbridge-port",
        type=int,
        default=DEFAULT_ROSBRIDGE_PORT,
        help=f"Rosbridge WebSocket port (default: {DEFAULT_ROSBRIDGE_PORT})",
    )

    parser.add_argument(
        "--name",
        default="ros-mcp-server",
        help="MCP server name, useful to distinguish multiple instances (default: ros-mcp-server)",
    )

    parser.add_argument(
        "--camera-topic",
        default="/image_raw/compressed",
        help="Default camera topic for capture_camera_image (default: /image_raw/compressed)",
    )

    return parser.parse_args()


def main():
    """Main entry point for the MCP server console script."""
    args = parse_arguments()

    rosbridge_ip = args.rosbridge_ip
    rosbridge_port = args.rosbridge_port
    mcp_transport = args.transport.lower()
    mcp_host = args.host
    mcp_port = args.port
    camera_topic = args.camera_topic

    print(f"Connecting to rosbridge at {rosbridge_ip}:{rosbridge_port}", file=sys.stderr)

    # Initialize per-instance MCP server, WebSocket manager, and tools
    mcp = FastMCP(args.name)
    ws_manager = WebSocketManager(rosbridge_ip, rosbridge_port, default_timeout=5.0)
    register_all_tools(mcp, ws_manager, rosbridge_ip=rosbridge_ip, rosbridge_port=rosbridge_port, camera_topic=camera_topic)
    register_all_resources(mcp, ws_manager)
    register_all_prompts(mcp)

    if mcp_transport == "stdio":
        mcp.run(transport="stdio")

    elif mcp_transport in {"http", "streamable-http"}:
        print(f"Transport: {mcp_transport} -> http://{mcp_host}:{mcp_port}", file=sys.stderr)
        mcp.run(transport=mcp_transport, host=mcp_host, port=mcp_port)

    elif mcp_transport == "sse":
        print(f"Transport: {mcp_transport} -> http://{mcp_host}:{mcp_port}", file=sys.stderr)
        print("Currently unsupported. Use 'stdio', 'http', or 'streamable-http'.", file=sys.stderr)
        mcp.run(transport=mcp_transport, host=mcp_host, port=mcp_port)

    else:
        raise ValueError(
            f"Unsupported MCP_TRANSPORT={mcp_transport!r}. "
            "Use 'stdio', 'http', or 'streamable-http'."
        )


if __name__ == "__main__":
    main()
