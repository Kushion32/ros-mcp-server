#!/bin/bash
# Linux startup script for turtlesim GUI

set -e

echo "🐢 Starting Turtlesim on Linux with GUI"
echo "========================================"

# Step 1: Check if X11 is available
if [[ -z "$DISPLAY" ]]; then
    echo "❌ DISPLAY variable not set"
    echo "💡 X11 forwarding is required. Are you running in a GUI environment?"
    exit 1
fi

echo "🖥️  Using DISPLAY: $DISPLAY"

# Step 2: Allow Docker to access X11
if command -v xhost >/dev/null 2>&1; then
    echo "🔓 Allowing Docker X11 access..."
    xhost +local:docker >/dev/null 2>&1 || {
        echo "⚠️  xhost command failed, trying without sudo..."
        xhost + >/dev/null 2>&1 || echo "⚠️  Could not run xhost, X11 forwarding may not work"
    }
else
    echo "⚠️  xhost not found. Install with: sudo apt-get install x11-xserver-utils"
    echo "   Continuing anyway, but GUI may not work..."
fi

# Step 3: Export DISPLAY for Docker
echo "🐳 Docker will use DISPLAY=$DISPLAY"

# Step 4: Start the container
echo ""
echo "🚀 Starting Turtlesim container..."
echo "   If successful, you should see a window with a turtle!"
echo ""

docker compose up turtlesim

echo ""
echo "🎉 Done! The turtle window should be visible on your screen."
