#!/bin/bash
# Quick Start Script for Telemetry Viewer

echo "=========================================="
echo "Pico W Car Tracker - Telemetry Viewer"
echo "Quick Start Guide"
echo "=========================================="
echo ""

# Check Python
if ! command -v python3 &> /dev/null; then
    echo "❌ Python 3 not found. Please install Python 3.7 or higher."
    exit 1
fi

echo "✅ Python found: $(python3 --version)"

# Check matplotlib
if python3 -c "import matplotlib" 2>/dev/null; then
    echo "✅ matplotlib installed"
else
    echo "⚠️  matplotlib not installed"
    echo ""
    echo "Installing matplotlib..."
    pip3 install matplotlib
    
    if [ $? -eq 0 ]; then
        echo "✅ matplotlib installed successfully"
    else
        echo "❌ Failed to install matplotlib"
        echo "   Try manually: pip3 install matplotlib"
        exit 1
    fi
fi

echo ""
echo "=========================================="
echo "Setup Complete!"
echo "=========================================="
echo ""
echo "Next steps:"
echo ""
echo "Option 1: Test with Mock Server (no hardware needed)"
echo "  1. Terminal 1: python3 mock_telemetry_server.py"
echo "  2. Terminal 2: python3 telemetry_viewer.py"
echo "  3. In GUI: Change host to 'localhost', click Connect"
echo ""
echo "Option 2: Connect to Real Pico W"
echo "  1. Connect to WiFi: PicoW-CarTracker (password: tracking123)"
echo "  2. Run: python3 telemetry_viewer.py"
echo "  3. In GUI: Host=192.168.4.1, Port=8080, click Connect"
echo ""
echo "=========================================="
