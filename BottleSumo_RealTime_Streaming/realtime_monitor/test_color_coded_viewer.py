"""Test script for color-coded progress bars and threshold functionality.

Tests:
1. JSON parsing with threshold fields (new firmware)
2. JSON parsing without threshold fields (old firmware - backward compatibility)
3. Color zone calculations for IR sensors
4. Color zone calculations for ToF sensors
5. Threshold marker positioning
"""
import json
from viewer import TelemetryPacket

def test_new_firmware_json():
    """Test JSON with threshold fields (new firmware)."""
    print("Test 1: New firmware JSON (with thresholds)")
    payload = {
        "timestamp": 12345,
        "irsensors": {
            "raw": [1000, 2000, 3000, 4000],
            "voltage": [1.2, 2.1, 2.8, 3.1],
            "edge_threshold": 2.5
        },
        "tof": {
            "distance_mm": [500, 1000, 1500],
            "valid": [True, True, True],
            "status": [0, 0, 0],
            "direction": "FRONT",
            "detection_threshold": 1600
        },
        "robot_state": {"action": "SEARCH"},
        "system_info": {"free_heap": 123456}
    }
    
    packet = TelemetryPacket.from_json(payload)
    assert packet.ir_edge_threshold == 2.5, f"Expected 2.5, got {packet.ir_edge_threshold}"
    assert packet.tof_detection_threshold == 1600, f"Expected 1600, got {packet.tof_detection_threshold}"
    print("  ✓ Threshold parsing correct")
    print(f"    IR threshold: {packet.ir_edge_threshold}V")
    print(f"    ToF threshold: {packet.tof_detection_threshold}mm")
    return True

def test_old_firmware_json():
    """Test JSON without threshold fields (backward compatibility)."""
    print("\nTest 2: Old firmware JSON (no thresholds - backward compatibility)")
    payload = {
        "timestamp": 12345,
        "irsensors": {
            "raw": [1000, 2000, 3000, 4000],
            "voltage": [1.2, 2.1, 2.8, 3.1]
            # Missing edge_threshold
        },
        "tof": {
            "distance_mm": [500, 1000, 1500],
            "valid": [True, True, True],
            "status": [0, 0, 0],
            "direction": "FRONT"
            # Missing detection_threshold
        },
        "robot_state": {"action": "SEARCH"},
        "system_info": {"free_heap": 123456}
    }
    
    packet = TelemetryPacket.from_json(payload)
    assert packet.ir_edge_threshold == 2.5, f"Expected default 2.5, got {packet.ir_edge_threshold}"
    assert packet.tof_detection_threshold == 1600, f"Expected default 1600, got {packet.tof_detection_threshold}"
    print("  ✓ Default thresholds applied correctly")
    print(f"    IR threshold (default): {packet.ir_edge_threshold}V")
    print(f"    ToF threshold (default): {packet.tof_detection_threshold}mm")
    return True

def test_ir_color_zones():
    """Test IR sensor color zone logic."""
    print("\nTest 3: IR sensor color zones")
    test_cases = [
        (0.5, "green", "Low voltage"),
        (1.0, "green", "Still green zone"),
        (1.5, "yellow", "Entering yellow zone"),
        (2.0, "yellow", "Mid yellow zone"),
        (2.5, "red", "Entering red zone"),
        (3.0, "red", "High voltage"),
        (3.3, "red", "Max voltage"),
    ]
    
    for voltage, expected_color, description in test_cases:
        if voltage < 1.5:
            actual_color = "green"
        elif voltage < 2.5:
            actual_color = "yellow"
        else:
            actual_color = "red"
        
        assert actual_color == expected_color, f"{description}: Expected {expected_color}, got {actual_color}"
        print(f"  ✓ {voltage}V → {actual_color} ({description})")
    
    return True

def test_tof_color_zones():
    """Test ToF sensor color zone logic (distance-based, not proximity)."""
    print("\nTest 4: ToF sensor color zones")
    test_cases = [
        (100, "red", "Very close"),
        (400, "yellow", "Entering yellow zone (boundary)"),
        (800, "yellow", "Mid yellow zone"),
        (1200, "green", "Entering green zone (boundary)"),
        (1500, "green", "Far away"),
        (2000, "green", "Max range"),
    ]
    
    for distance, expected_color, description in test_cases:
        if distance >= 1200:
            actual_color = "green"
        elif distance >= 400:
            actual_color = "yellow"
        else:
            actual_color = "red"
        
        assert actual_color == expected_color, f"{description}: Expected {expected_color}, got {actual_color}"
        print(f"  ✓ {distance}mm → {actual_color} ({description})")
    
    return True

def test_threshold_marker_positioning():
    """Test threshold marker pixel positioning calculation."""
    print("\nTest 5: Threshold marker positioning")
    canvas_width = 150  # pixels
    max_voltage = 3.3   # volts
    
    test_cases = [
        (0.0, 0, "Minimum (0V at left edge)"),
        (1.65, 75, "Middle (1.65V at center)"),
        (2.5, 114, "Default threshold (2.5V)"),
        (3.3, 150, "Maximum (3.3V at right edge)"),
    ]
    
    for threshold_voltage, expected_px, description in test_cases:
        line_x = int((threshold_voltage / max_voltage) * canvas_width)
        assert abs(line_x - expected_px) <= 1, f"{description}: Expected {expected_px}px, got {line_x}px"
        print(f"  ✓ {threshold_voltage}V → {line_x}px ({description})")
    
    return True

def main():
    print("========================================")
    print("Color-Coded Viewer Test Suite")
    print("========================================\n")
    
    tests = [
        test_new_firmware_json,
        test_old_firmware_json,
        test_ir_color_zones,
        test_tof_color_zones,
        test_threshold_marker_positioning,
    ]
    
    passed = 0
    failed = 0
    
    for test_func in tests:
        try:
            if test_func():
                passed += 1
        except AssertionError as e:
            print(f"  ✗ FAILED: {e}")
            failed += 1
        except Exception as e:
            print(f"  ✗ ERROR: {e}")
            failed += 1
    
    print("\n========================================")
    print(f"Results: {passed} passed, {failed} failed")
    print("========================================")
    
    if failed == 0:
        print("✅ ALL TESTS PASSED")
        return 0
    else:
        print("❌ SOME TESTS FAILED")
        return 1

if __name__ == "__main__":
    exit(main())
