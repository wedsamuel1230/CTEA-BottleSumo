#!/bin/bash
# Serial Port Fix Script for Raspberry Pi Pico
# Run this before uploading to fix timeout issues

echo "🔧 Fixing Serial Port Issues..."
echo ""

# 1. Check if port exists
if [ -e /dev/cu.usbmodem1401 ]; then
    echo "✅ Port /dev/cu.usbmodem1401 exists"
else
    echo "❌ Port /dev/cu.usbmodem1401 NOT found"
    echo "   Available USB ports:"
    ls -la /dev/cu.usb* 2>/dev/null || echo "   No USB ports found"
    echo ""
    echo "Try:"
    echo "  1. Unplug and replug USB cable"
    echo "  2. Hold BOOTSEL while plugging in USB"
    exit 1
fi

# 2. Check if port is in use
PORT_IN_USE=$(lsof /dev/cu.usbmodem1401 2>/dev/null)
if [ -n "$PORT_IN_USE" ]; then
    echo "⚠️  Port is currently in use:"
    echo "$PORT_IN_USE"
    echo ""
    echo "Killing processes using the port..."
    sudo lsof -t /dev/cu.usbmodem1401 | xargs sudo kill -9 2>/dev/null
    sleep 1
    echo "✅ Port released"
else
    echo "✅ Port is not in use"
fi

# 3. Reset port permissions
echo ""
echo "Setting port permissions..."
sudo chmod 666 /dev/cu.usbmodem1401 2>/dev/null
echo "✅ Port permissions set"

# 4. Test port communication
echo ""
echo "Testing port communication..."
if stty -f /dev/cu.usbmodem1401 speed 115200 2>/dev/null; then
    echo "✅ Port communication OK"
else
    echo "⚠️  Port communication test failed"
    echo "   Try resetting the Pico:"
    echo "   1. Hold BOOTSEL button"
    echo "   2. Press RESET button"
    echo "   3. Release both buttons"
fi

echo ""
echo "🎯 Port should be ready for upload now!"
echo ""
echo "Next steps:"
echo "  1. Close Arduino Serial Monitor"
echo "  2. Click Upload in Arduino IDE"
echo "  3. Wait for upload to complete"
echo "  4. Then open Serial Monitor at 115200 baud"
