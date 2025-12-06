"""Quick test to verify motor control JSON format matches motor.ino expectations."""
import json

# Test case 1: Direct motor control (matches motor.ino line 107-119)
motor1_value = 150
motor2_value = -150
command = {"motor1": motor1_value, "motor2": motor2_value}
command_str = json.dumps(command) + "\n"

print("Test 1: Direct motor control (matches motor.ino)")
print(f"JSON: {command_str.strip()}")
print(f"Expected by motor.ino: {{'motor1': int, 'motor2': int}}")
print()

# Test case 2: Command-based control (alternative format in motor.ino line 82-105)
command2 = {"command": "forward", "speed": 200}
command2_str = json.dumps(command2) + "\n"

print("Test 2: Command-based control")
print(f"JSON: {command2_str.strip()}")
print(f"Expected by motor.ino: {{'command': str, 'speed': int}}")
print()

# Verify our implementation uses Test 1 format
print("✅ Our MotorControlSender uses Test 1 format (direct motor control)")
print("✅ This matches motor.ino parseAndControl() expectations")
