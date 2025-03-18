import time
from machine import I2C

class MPU6050:
    # MPU6050 Registers
    PWR_MGMT_1 = 0x6B
    GYRO_CONFIG = 0x1B
    ACCEL_CONFIG = 0x1C
    ACCEL_XOUT_H = 0x3B
    GYRO_XOUT_H = 0x43
    TEMP_OUT_H = 0x41
    
    # Scale Modifiers
    ACCEL_SCALE_MODIFIER_2G = 16384.0
    GYRO_SCALE_MODIFIER_250DEG = 131.0
    
    def __init__(self, i2c, addr=0x68):
        self.i2c = i2c
        self.addr = addr
        
        # Wake up MPU6050
        self.i2c.writeto_mem(self.addr, self.PWR_MGMT_1, b'\x00')
        time.sleep(0.1)  # Wait for device to stabilize
        
        # Configure accelerometer (+/-2g)
        self.i2c.writeto_mem(self.addr, self.ACCEL_CONFIG, b'\x00')
        
        # Configure gyroscope (250deg/s)
        self.i2c.writeto_mem(self.addr, self.GYRO_CONFIG, b'\x00')
        
        # Initialize calibration offsets
        self.accel_offsets = {"x": 0, "y": 0, "z": 0}
        self.gyro_offsets = {"x": 0, "y": 0, "z": 0}
        
    def read_raw_data(self, reg):
        data = self.i2c.readfrom_mem(self.addr, reg, 2)
        value = (data[0] << 8) | data[1]
        if value > 32767:
            value -= 65536  # Convert to signed
        return value
        
    def calibrate(self, samples=100, delay=0.01):
        """Calibrate by calculating offsets from multiple samples"""
        print("Calibrating MPU6050, keep the sensor still...")
        
        accel_sum = {"x": 0, "y": 0, "z": 0}
        gyro_sum = {"x": 0, "y": 0, "z": 0}
        
        for _ in range(samples):
            # Read raw values
            accel_x = self.read_raw_data(self.ACCEL_XOUT_H)
            accel_y = self.read_raw_data(self.ACCEL_XOUT_H + 2)
            accel_z = self.read_raw_data(self.ACCEL_XOUT_H + 4)
            
            gyro_x = self.read_raw_data(self.GYRO_XOUT_H)
            gyro_y = self.read_raw_data(self.GYRO_XOUT_H + 2)
            gyro_z = self.read_raw_data(self.GYRO_XOUT_H + 4)
            
            # Sum all readings
            accel_sum["x"] += accel_x
            accel_sum["y"] += accel_y
            accel_sum["z"] += accel_z - self.ACCEL_SCALE_MODIFIER_2G  # Remove gravity from Z
            
            gyro_sum["x"] += gyro_x
            gyro_sum["y"] += gyro_y
            gyro_sum["z"] += gyro_z
            
            time.sleep(delay)
            
        # Calculate average offsets
        self.accel_offsets["x"] = accel_sum["x"] / samples
        self.accel_offsets["y"] = accel_sum["y"] / samples
        self.accel_offsets["z"] = accel_sum["z"] / samples
        
        self.gyro_offsets["x"] = gyro_sum["x"] / samples
        self.gyro_offsets["y"] = gyro_sum["y"] / samples
        self.gyro_offsets["z"] = gyro_sum["z"] / samples
        
        print("Calibration complete")
        
    def get_values(self, samples=5, delay=0.01):
        """Get filtered and calibrated sensor values"""
        values = {
            "GyX": 0, "GyY": 0, "GyZ": 0, 
            "Tmp": 0, 
            "AcX": 0, "AcY": 0, "AcZ": 0
        }
        
        for _ in range(samples):
            # Read raw values
            accel_x = self.read_raw_data(self.ACCEL_XOUT_H) - self.accel_offsets["x"]
            accel_y = self.read_raw_data(self.ACCEL_XOUT_H + 2) - self.accel_offsets["y"]
            accel_z = self.read_raw_data(self.ACCEL_XOUT_H + 4) - self.accel_offsets["z"]
            
            gyro_x = self.read_raw_data(self.GYRO_XOUT_H) - self.gyro_offsets["x"]
            gyro_y = self.read_raw_data(self.GYRO_XOUT_H + 2) - self.gyro_offsets["y"]
            gyro_z = self.read_raw_data(self.GYRO_XOUT_H + 4) - self.gyro_offsets["z"]
            
            temp = self.read_raw_data(self.TEMP_OUT_H) / 340.0 + 36.53
            
            # Apply scaling to convert to meaningful units
            accel_x_g = accel_x / self.ACCEL_SCALE_MODIFIER_2G
            accel_y_g = accel_y / self.ACCEL_SCALE_MODIFIER_2G
            accel_z_g = accel_z / self.ACCEL_SCALE_MODIFIER_2G
            
            gyro_x_deg = gyro_x / self.GYRO_SCALE_MODIFIER_250DEG
            gyro_y_deg = gyro_y / self.GYRO_SCALE_MODIFIER_250DEG
            gyro_z_deg = gyro_z / self.GYRO_SCALE_MODIFIER_250DEG
            
            # Add to accumulators (raw values for backward compatibility)
            values["GyX"] += gyro_x
            values["GyY"] += gyro_y
            values["GyZ"] += gyro_z
            values["Tmp"] += temp
            values["AcX"] += accel_x
            values["AcY"] += accel_y
            values["AcZ"] += accel_z
            
            # Add scaled values
            if "GyX_deg" not in values:
                values["GyX_deg"] = 0
                values["GyY_deg"] = 0
                values["GyZ_deg"] = 0
                values["AcX_g"] = 0
                values["AcY_g"] = 0
                values["AcZ_g"] = 0
            
            values["GyX_deg"] += gyro_x_deg
            values["GyY_deg"] += gyro_y_deg
            values["GyZ_deg"] += gyro_z_deg
            values["AcX_g"] += accel_x_g
            values["AcY_g"] += accel_y_g
            values["AcZ_g"] += accel_z_g
            
            time.sleep(delay)
            
        # Average values
        for key in values:
            values[key] /= samples
            
        return values
        
    def apply_complementary_filter(self, accel_data, gyro_data, dt, alpha=0.98):
        """
        Apply complementary filter to combine accelerometer and gyroscope data
        for more accurate orientation estimation
        """
        # Calculate angles from accelerometer data
        accel_angle_x = math.atan2(accel_data["AcY_g"], accel_data["AcZ_g"]) * 180 / math.pi
        accel_angle_y = math.atan2(-accel_data["AcX_g"], 
                                  math.sqrt(accel_data["AcY_g"]**2 + accel_data["AcZ_g"]**2)) * 180 / math.pi
        
        # Combine with gyroscope data
        angle_x = alpha * (angle_x + gyro_data["GyX_deg"] * dt) + (1 - alpha) * accel_angle_x
        angle_y = alpha * (angle_y + gyro_data["GyY_deg"] * dt) + (1 - alpha) * accel_angle_y
        
        return {"angle_x": angle_x, "angle_y": angle_y}
