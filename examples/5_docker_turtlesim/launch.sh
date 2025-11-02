#!/bin/bash
# Unified launcher for turtlesim - auto-detects OS and runs appropriate script

set -e

# Detect operating system
OS_TYPE=$(uname -s)

case "$OS_TYPE" in
    Darwin*)
        echo "🍎 Detected macOS - launching with XQuartz support..."
        ./docker/scripts/launch_macos.sh
        ;;
    Linux*)
        echo "🐧 Detected Linux - launching with X11 support..."
        ./docker/scripts/launch_linux.sh
        ;;
    MINGW*|MSYS*|CYGWIN*)
        echo "🪟 Detected Windows - launching with X server support..."
        ./docker/scripts/launch_windows.sh
        ;;
    *)
        echo "❌ Unsupported OS: $OS_TYPE"
        echo "   Please run the appropriate script manually:"
        echo "   - macOS: ./docker/scripts/launch_macos.sh"
        echo "   - Linux: ./docker/scripts/launch_linux.sh"
        echo "   - Windows: ./docker/scripts/launch_windows.sh"
        exit 1
        ;;
esac
