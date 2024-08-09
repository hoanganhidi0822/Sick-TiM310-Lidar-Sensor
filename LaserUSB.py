import usb.core
import usb.util
import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class LidarNotFound(Exception):
    pass

class Lidar:
    def __init__(self):
        self.device = None
        self.connect()

    def connect(self):
        self.device = usb.core.find(idVendor=0x19a2, idProduct=0x5001)  # Replace with actual IDs
        if self.device is None:
            raise LidarNotFound("LiDAR Device is not connected!")
        self.device.set_configuration()

    def connected(self):
        return self.device is not None

    def set_measurement_range(self, start_angle, stop_angle):
        # Convert angles to hex format required by your LiDAR
        start_angle_hex = f"{int(start_angle * 10000):08X}"
        stop_angle_hex = f"{int(stop_angle * 10000):08X}"
        self.send(f"sMN mLMPsetscancfg +2500 +5000 {start_angle_hex} {stop_angle_hex}")
        return self.read()

    def set_scan_frequency(self, frequency):
        # Assuming frequency is in Hz and needs to be converted to an appropriate format
        frequency_hex = f"{int(frequency * 100):04X}"
        self.send(f"sMN mLMPsetscancfg {frequency_hex}")
        return self.read()
    
    def send(self, cmd):
        if self.connected():
            try:
                print(f"Sending command: {cmd}")
                self.device.write(2 | usb.ENDPOINT_OUT, f"\x02{cmd}\x03\0", 0)  # Endpoint OUT
            except usb.core.USBError as e:
                print(f"Error sending command to LiDAR: {e}")
        else:
            print("LiDAR Device not found!")

    def read(self):
        if self.connected():
            try:
                arr = self.device.read(1 | usb.ENDPOINT_IN, 65535, timeout=100)  # Endpoint IN
                arr = "".join([chr(x) for x in arr[1:-1]])
                arr = self.check_error(arr)
                return arr
            except usb.core.USBError as e:
                print(f"Error reading from LiDAR: {e}")
                return None
        else:
            raise LidarNotFound("LiDAR Device is not connected!")

    def check_error(self, response):
        if "FA" in response:
            print("Error response received:", response)
        return response

    def firmware_version(self):
        self.send("sRN FirmwareVersion")
        return self.read()

    def device_identification(self):
        self.send("sRI 0")
        return self.read()

    def set_access_mode(self, user="03", password="F4724744"):
        self.send(f'sMN SetAccessMode {user} {password}')
        return self.read()

    
    
    def start_measurement(self):
        self.send("sMN LMCstartmeas")
        return self.read()

    def run(self):
        self.send('sMN Run')
        return self.read()

    def scan_data(self, data):
        self.send(data)
        return self.read()

def parse_telegram(telegram):
    tokens = telegram.split(' ')
    
    # Ensure that there are enough tokens
    if len(tokens) <= (18 + 8):  # Minimum valid length
        raise ValueError("Insufficient data tokens")
    
    # Extract header and validate
    header = tokens[:18]
    if header[0] != 'sRA':
        raise ValueError("Invalid command type")
    if header[1] != 'E9':
        raise ValueError("Invalid command")
    
    # Extract and validate data sections
    sections = tokens[18:]
    try:
        if int(sections[0], 16) != 0:  # No encoder data
            raise ValueError("Unexpected encoder data")
        if int(sections[1], 16) != 1:  # Exactly 1 16-bit channel block
            raise ValueError("Unexpected channel block count")
        if sections[2] != 'DIST1':  # Distance data expected
            raise ValueError("Unexpected data type")
        if sections[3] not in ['3F800000', '40000000']:  # Check scale factor
            raise ValueError("Invalid scale factor")
        
        scale_factor = 1 if sections[3] == '3F800000' else 2
        if sections[4] != '00000000':
            raise ValueError("Unexpected value in section 4")
        
        start_angle = int(sections[5], 16) / 10000.0
        angle_step = int(sections[6], 16) / 10000.0
        value_count = int(sections[7], 16)
        
        # Extract distance values and compute angles
        values = list(map(lambda x: int(x, 16) * scale_factor, sections[8:8 + value_count]))
        angles = [start_angle + angle_step * n for n in range(value_count)]
        
        return (values, angles)
    
    except ValueError as e:
        raise ValueError(f"Parsing error: {e}")

def check_obstacles(values, angles):
    """Check for obstacles within 70 cm in the right, front, and left directions."""
    threshold = 700  # 70 cm in mm

    right_range = (0, 45)
    front_range = (45, 135)
    left_range = (135, 180)

    right_close = any(value < threshold for angle, value in zip(angles, values) if right_range[0] <= angle < right_range[1])
    front_close = any(value < threshold for angle, value in zip(angles, values) if front_range[0] <= angle < front_range[1])
    left_close = any(value < threshold for angle, value in zip(angles, values) if left_range[0] <= angle < left_range[1])

    if right_close:
        print("Obstacle detected on the RIGHT")
    if front_close:
        print("Obstacle detected in FRONT")
    if left_close:
        print("Obstacle detected on the LEFT")

def main():
    lidar = Lidar()
    
    try:
        # Set Access Mode, Run the device, and start measurement
        print("Run:", lidar.run())
        time.sleep(0.1)
        
        # Setup plot
        fig, ax = plt.subplots(subplot_kw={'projection': 'polar'}, figsize=(8, 8))
        ax.set_title('LiDAR Scan Data')
        
        
        
        # Initialize scatter plot
        scatter = ax.scatter([], [], c='b', marker='o', s=15, alpha=0.7, edgecolors='none')
        ax.set_rmax(7000)  # Set maximum radial distance, adjust as needed
        ax.set_rticks([])  # Remove radial ticks
        ax.set_yticklabels([])  # Remove radial labels
        ax.set_xticks(np.linspace(0, 2 * np.pi, 8, endpoint=False))  # Add angular ticks
        ax.set_thetamax(225)  # Optionally limit the maximum theta (angle) displayed
        ax.set_thetamin(-45)  # Optionally limit the minimum theta (angle) displayed
        ax.set_theta_zero_location('E')  # Default, 'N' for North (0 degrees at the top)
        # Other options: 'E' (East), 'S' (South), 'W' (West)

        while True:
            # Get scan data
            data = lidar.scan_data("sRI E9")
            values, angles = parse_telegram(data)
            adjusted_angles = [(angle - 15) % 360 for angle in angles]
            
            # Update scatter plot data
            
            # Check for obstacles
            check_obstacles(adjusted_angles, angles)
            
            # Update plot
            scatter.set_offsets(np.column_stack((np.deg2rad(adjusted_angles), values)))
            plt.pause(0.1)  # Smooth update interval
            
    except LidarNotFound as e:
        print(e)
    except Exception as e:
        print("An error occurred:", e)

if __name__ == "__main__":
    main()