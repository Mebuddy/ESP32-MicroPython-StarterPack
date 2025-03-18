import time
import math
from machine import Pin, I2C
from mpu6050 import MPU6050

# Initialize I2C
i2c = I2C(scl=Pin(22), sda=Pin(21))

# Initialize MPU6050
mpu = MPU6050(i2c)

# Filter parameters
alpha = 0.8  # Low-pass filter factor
beta = 0.95  # Complementary filter factor
prev_angles = {"x": 0, "y": 0, "z": 0}
prev_time = time.ticks_ms()

# Movement thresholds - adjust based on your calibration
LEFT_THRESHOLD = -15      # Was FORWARD_THRESHOLD (Negative X angle)
RIGHT_THRESHOLD = 15      # Was BACKWARD_THRESHOLD (Positive X angle)
TURN_THRESHOLD = 20       # Z angle threshold
FORWARD_THRESHOLD = 15    # Was CRAB_THRESHOLD (Positive Y angle)
BACKWARD_THRESHOLD = -15  # Was negative CRAB_THRESHOLD (Negative Y angle)
DEAD_ZONE = 10            # Neutral zone size

def low_pass_filter(new_value, prev_value, alpha):
    """Simple low-pass filter"""
    return alpha * prev_value + (1 - alpha) * new_value

def get_direction(angle_x, angle_y, angle_z):
    """Determine the direction based on angles with modified mapping"""
    direction = "STOP"
    
    # Forward/Backward detection (now using Y axis / roll)
    if angle_y > DEAD_ZONE and angle_y > FORWARD_THRESHOLD:
        direction = "FORWARD"  # Was CRAB_RIGHT
    elif angle_y < -DEAD_ZONE and angle_y < BACKWARD_THRESHOLD:
        direction = "BACKWARD"  # Was CRAB_LEFT
    
    # Left/Right detection (now using X axis / pitch)
    if angle_x < -DEAD_ZONE and angle_x < LEFT_THRESHOLD:
        direction = "LEFT"  # Was FORWARD
    elif angle_x > DEAD_ZONE and angle_x > RIGHT_THRESHOLD:
        direction = "RIGHT"  # Was BACKWARD
    
    # Turn detection (yaw - keeping this as is)
    if abs(angle_z) > DEAD_ZONE and abs(angle_z) > TURN_THRESHOLD:
        if angle_z > 0:
            direction = "ROTATE_RIGHT"
        else:
            direction = "ROTATE_LEFT"
            
    return direction

def start_direction_display():
    global prev_angles, prev_time
    
    # Variables for smoothing display
    prev_data = {}
    prev_direction = "STOP"
    direction_count = 0
    
    print("Calibrating sensor, please keep hand in neutral position...")
    mpu.calibrate(samples=100)
    print("Calibration complete! Start moving your hand to see directions.")
    print("MODIFIED MAPPING: forward=Y+, backward=Y-, left=X-, right=X+")
    
    while True:
        # Get current time for delta calculation
        current_time = time.ticks_ms()
        dt = (current_time - prev_time) / 1000.0
        prev_time = current_time
        
        # Get raw sensor values
        data = mpu.get_values(samples=5, delay=0.01)
        
        # Apply low-pass filter if we have previous data
        if prev_data:
            for key in data:
                if key in prev_data:
                    data[key] = low_pass_filter(data[key], prev_data[key], alpha)
        
        # Calculate angles from accelerometer data
        if "AcX_g" in data:
            # Calculate angles
            accel_angle_x = math.atan2(data["AcY_g"], data["AcZ_g"]) * 180 / math.pi
            accel_angle_y = math.atan2(-data["AcX_g"], 
                                      math.sqrt(data["AcY_g"]**2 + data["AcZ_g"]**2)) * 180 / math.pi
            
            # Apply complementary filter
            angle_x = beta * (prev_angles["x"] + data["GyX_deg"] * dt) + (1 - beta) * accel_angle_x
            angle_y = beta * (prev_angles["y"] + data["GyY_deg"] * dt) + (1 - beta) * accel_angle_y
            angle_z = prev_angles["z"] + data["GyZ_deg"] * dt
            
            # Update for next iteration
            prev_angles = {"x": angle_x, "y": angle_y, "z": angle_z}
            
            # Get direction based on angles
            direction = get_direction(angle_x, angle_y, angle_z)
            
            # Smooth direction changes (prevent flickering)
            if direction == prev_direction:
                direction_count += 1
            else:
                direction_count = 0
                prev_direction = direction
            
            # Only display if direction is stable or significant change
            if direction_count >= 2:
                # Create direction output with visual indicators
                direction_display = f"{direction}"
                angle_display = f"X={angle_x:.1f}°, Y={angle_y:.1f}°, Z={angle_z:.1f}°"
                
                # Add visual indicator for direction
                indicator = ""
                if direction == "FORWARD":
                    indicator = "↑"
                elif direction == "BACKWARD":
                    indicator = "↓"
                elif direction == "LEFT":
                    indicator = "←"
                elif direction == "RIGHT":
                    indicator = "→"
                elif direction == "ROTATE_LEFT":
                    indicator = "↺"
                elif direction == "ROTATE_RIGHT":
                    indicator = "↻"
                
                # Print the direction and angles
                print(f"Direction: {direction_display} {indicator} | Angles: {angle_display}")
            
            # Update previous values
            prev_data = data.copy()
        
        time.sleep(0.1)  # 100ms refresh rate

if __name__ == "__main__":
    try:
        start_direction_display()
    except KeyboardInterrupt:
        print("Direction display stopped.")
