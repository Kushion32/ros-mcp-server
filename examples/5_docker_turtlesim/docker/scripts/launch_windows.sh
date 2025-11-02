#!/bin/bash
# Windows (WSL/Git Bash) startup script for turtlesim GUI
# Requires an X server like VcXsrv, X410, or Xming running on Windows

set -e

echo "🐢 Starting Turtlesim on Windows with GUI"
echo "=========================================="

# Step 1: Check if running in WSL or Git Bash
if grep -qi microsoft /proc/version 2>/dev/null; then
    PLATFORM="WSL"
elif uname -s | grep -qi "MINGW\|MSYS\|CYGWIN"; then
    PLATFORM="Git Bash/MSYS"
else
    PLATFORM="Unknown"
fi

echo "🖥️  Detected platform: $PLATFORM"

# Step 2: Check if X server is running on Windows
echo "🔍 Checking for X server on Windows..."
echo "   Make sure VcXsrv, X410, or another X server is running!"
echo "   If not installed, get VcXsrv from: https://sourceforge.net/projects/vcxsrv/"

# Step 3: Set DISPLAY for Docker
# Docker Desktop on Windows uses host.docker.internal to reach Windows host
export DISPLAY="host.docker.internal:0.0"
echo "🐳 Docker will use DISPLAY=$DISPLAY"

# Step 4: Warn about common issues
cat << 'TIPS'

📝 Important Notes for Windows:
   - Start your X server (VcXsrv/X410) BEFORE running this script
   - In VcXsrv: disable "Native opengl" and enable "Disable access control"
   - If GUI doesn't appear, check Windows Firewall settings
   - Docker Desktop must be running

TIPS

# Step 5: Start the container
echo ""
echo "🚀 Starting Turtlesim container..."
echo "   If successful, you should see a window with a turtle!"
echo ""

docker compose up turtlesim

echo ""
echo "🎉 Done! The turtle window should appear in your X server."
