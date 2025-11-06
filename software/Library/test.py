import serial

class Car:
    def __init__(self, serial_port):
        self.serial = serial_port
    
    def move(self, speed=0, direction='forward', angle=None):
        """
        Unified movement control.
        
        Args:
            speed: 0-100 (percentage)
            direction: 'forward', 'backward', 'left', 'right', 'stop'
            angle: Optional steering angle for differential drive
        """
        if direction == 'stop':
            self._send_command(f"STOP\n")
            return
        
        if angle is not None:
            # Differential drive with angle
            left_speed, right_speed = self._calculate_differential(speed, angle)
            self._send_command(f"MOTORS {left_speed} {right_speed}\n")
        else:
            # Simple directional movement
            self._send_command(f"MOVE {direction} {speed}\n")
    
    def _calculate_differential(self, speed, angle):
        """Convert speed + angle to left/right motor speeds"""
        import math
        # Simple differential steering
        turn_ratio = angle / 90.0  # -1 to 1
        left = speed * (1 - turn_ratio)
        right = speed * (1 + turn_ratio)
        return (left, right)
    
    def _send_command(self, cmd):
        """Send command to Arduino via serial"""
        print(f"Sending: {cmd.strip()}")
        # self.serial.write(cmd.encode())

# Usage:
car = Car('/dev/ttyUSB0')
car.move(speed=50)                      # Forward at 50%
car.move(speed=75, direction='backward') # Backward at 75%
car.move(speed=100, angle=45)           # Forward with 45° turn
car.move(direction='stop')              # Stop
