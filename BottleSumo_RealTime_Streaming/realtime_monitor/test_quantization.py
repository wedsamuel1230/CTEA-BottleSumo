"""Test script for 0.1V quantization in threshold drag.

Tests that threshold values snap to 0.1V increments.
"""
import tkinter as tk


def test_quantization():
    """Test that voltage quantization works correctly."""
    print("========================================")
    print("0.1V Quantization Test")
    print("========================================\n")
    
    test_cases = [
        # (raw_voltage, expected_quantized, description)
        # Note: Python uses banker's rounding (round half to even)
        (0.0, 0.0, "Zero (0.0V)"),
        (0.04, 0.0, "Below 0.05 rounds down (0.04V → 0.0V)"),
        (0.05, 0.0, "At 0.05 banker's rounding (0.05V → 0.0V)"),  # round(0.5) = 0
        (0.14, 0.1, "0.14V → 0.1V"),
        (0.15, 0.2, "0.15V → 0.2V"),  # round(1.5) = 2
        (0.99, 1.0, "0.99V → 1.0V"),
        (1.23, 1.2, "1.23V → 1.2V"),
        (1.27, 1.3, "1.27V → 1.3V"),
        (2.486, 2.5, "Default threshold (2.486V → 2.5V)"),
        (2.94, 2.9, "2.94V → 2.9V"),
        (2.95, 3.0, "2.95V → 3.0V"),  # round(29.5) = 30
        (3.24, 3.2, "3.24V → 3.2V"),
        (3.27, 3.3, "3.27V → 3.3V"),
        (3.3, 3.3, "Maximum (3.3V)"),
    ]
    
    passed = 0
    failed = 0
    
    for raw, expected, description in test_cases:
        # Apply quantization algorithm from viewer.py
        quantized = round(raw * 10) / 10
        
        if quantized == expected:
            print(f"  ✓ {description}: {raw}V → {quantized}V")
            passed += 1
        else:
            print(f"  ✗ {description}: Expected {expected}V, got {quantized}V")
            failed += 1
    
    print(f"\n========================================")
    print(f"Results: {passed} passed, {failed} failed")
    print(f"========================================\n")
    
    if failed == 0:
        print("✅ ALL QUANTIZATION TESTS PASSED")
        return 0
    else:
        print("❌ SOME QUANTIZATION TESTS FAILED")
        return 1


def test_quantization_with_clamping():
    """Test quantization combined with clamping (full drag pipeline)."""
    print("\n========================================")
    print("Quantization + Clamping Integration Test")
    print("========================================\n")
    
    test_cases = [
        # (mouse_x, canvas_width, expected_voltage, description)
        # Note: Python uses banker's rounding (round half to even)
        (0, 150, 0.0, "Left edge"),
        (7, 150, 0.2, "7px → 0.154V → 0.2V"),
        (15, 150, 0.3, "15px → 0.33V → 0.3V"),
        (38, 150, 0.8, "38px → 0.836V → 0.8V"),
        (75, 150, 1.6, "Center (75px → 1.65V → 1.6V)"),  # round(16.5) = 16
        (113, 150, 2.5, "Default threshold (113px → 2.486V → 2.5V)"),
        (136, 150, 3.0, "136px → 2.992V → 3.0V"),
        (150, 150, 3.3, "Right edge"),
        (200, 150, 3.3, "Beyond right (clamped to 3.3V)"),
        (-10, 150, 0.0, "Beyond left (clamped to 0.0V)"),
    ]
    
    passed = 0
    failed = 0
    
    for mouse_x, canvas_width, expected, description in test_cases:
        # Replicate full algorithm from _on_threshold_drag_motion
        voltage = (mouse_x / canvas_width) * 3.3
        voltage = max(0.0, min(3.3, voltage))  # Clamp
        voltage = round(voltage * 10) / 10      # Quantize
        
        if voltage == expected:
            print(f"  ✓ {description}: {mouse_x}px → {voltage}V")
            passed += 1
        else:
            print(f"  ✗ {description}: Expected {expected}V, got {voltage}V")
            failed += 1
    
    print(f"\n========================================")
    print(f"Results: {passed} passed, {failed} failed")
    print(f"========================================\n")
    
    if failed == 0:
        print("✅ ALL INTEGRATION TESTS PASSED")
        return 0
    else:
        print("❌ SOME INTEGRATION TESTS FAILED")
        return 1


def print_manual_test_instructions():
    """Print manual testing instructions for 0.1V stepping."""
    print("\n" + "="*60)
    print("MANUAL TESTING INSTRUCTIONS - 0.1V QUANTIZATION")
    print("="*60)
    print()
    print("1. Run: python viewer.py")
    print()
    print("2. TEST: Window Does NOT Move")
    print("   - Click and drag threshold line left/right")
    print("   - ✅ EXPECTED: Only threshold line moves, window stays still")
    print("   - ❌ BUG: Window follows mouse movement")
    print()
    print("3. TEST: 0.1V Stepping")
    print("   - Drag threshold line slowly left/right")
    print("   - Watch Spinbox value update")
    print("   - ✅ EXPECTED: Values jump in 0.1V increments")
    print("     (e.g., 2.5 → 2.4 → 2.3 → 2.2)")
    print("   - ❌ BUG: Values change continuously (e.g., 2.486, 2.473)")
    print()
    print("4. TEST: Window Drag Still Works")
    print("   - Click and drag title bar (top of window)")
    print("   - ✅ EXPECTED: Window moves around screen")
    print("   - ❌ BUG: Window doesn't move")
    print()
    print("5. TEST: Precise Values at Each Step")
    print("   - Drag threshold to various positions")
    print("   - Verify Spinbox shows only these values:")
    print("     0.0, 0.1, 0.2, ... 1.0, 1.1, ... 2.5, ... 3.2, 3.3")
    print("   - ❌ BUG: Intermediate values like 1.47 or 2.83")
    print()
    print("="*60)


def main():
    exit_code = test_quantization()
    exit_code += test_quantization_with_clamping()
    print_manual_test_instructions()
    
    if exit_code == 0:
        print("\n✅ ALL AUTOMATED QUANTIZATION TESTS PASSED")
        print("⚠️  MANUAL TESTING REQUIRED (see instructions above)")
        return 0
    else:
        print("\n❌ SOME AUTOMATED QUANTIZATION TESTS FAILED")
        return 1


if __name__ == "__main__":
    exit(main())
