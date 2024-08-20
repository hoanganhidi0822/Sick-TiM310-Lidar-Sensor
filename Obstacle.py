import numpy as np
import matplotlib.pyplot as plt
import usb.core
import usb.util
import cv2
from sklearn.cluster import DBSCAN
from sklearn.linear_model import LinearRegression

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

    def send(self, cmd):
        if self.connected():
            try:
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
                pass
                return None
        else:
            raise LidarNotFound("LiDAR Device is not connected!")

    def check_error(self, response):
        if "FA" in response:
            pass
        return response

    def scan_data(self, data):
        self.send(data)
        return self.read()

def parse_telegram(telegram):
    tokens = telegram.split(' ')
    if len(tokens) <= (18 + 8):  
        raise ValueError("Insufficient data tokens")
    
    header = tokens[:18]
    if header[0] != 'sRA':
        raise ValueError("Invalid command type")
    if header[1] != 'E9':
        raise ValueError("Invalid command")
    
    sections = tokens[18:]
    try:
        if int(sections[0], 16) != 0:  
            raise ValueError("Unexpected encoder data")
        if int(sections[1], 16) != 1:  
            raise ValueError("Unexpected channel block count")
        if sections[2] != 'DIST1':  
            raise ValueError("Unexpected data type")
        if sections[3] not in ['3F800000', '40000000']:  
            raise ValueError("Invalid scale factor")
        
        scale_factor = 1 if sections[3] == '3F800000' else 2
        if sections[4] != '00000000':
            raise ValueError("Unexpected value in section 4")
        
        start_angle = int(sections[5], 16) / 10000.0
        angle_step = int(sections[6], 16) / 10000.0
        value_count = int(sections[7], 16)
        
        values = list(map(lambda x: int(x, 16) * scale_factor, sections[8:8 + value_count]))
        angles = [start_angle + angle_step * n for n in range(value_count)]
        
        return (values, angles)
    
    except ValueError as e:
        raise ValueError(f"Parsing error: {e}")

def get_colors(angles_rotated):
    colors = []
    for angle in angles_rotated:
        if 0 <= angle < 45:
            colors.append('green')
        elif 45 <= angle < 90:
            colors.append('red')
        elif 90 <= angle < 135:
            colors.append('brown')
        elif 135 <= angle < 180:
            colors.append('blue')
        else:
            colors.append('white')  # Default color for out-of-range angles
    return colors

def rotate_points(values, angles, rotation_angle_deg):
    rotation_angle_rad = np.deg2rad(rotation_angle_deg)

    x_coords = np.array(values) * np.cos(np.deg2rad(angles))
    y_coords = np.array(values) * np.sin(np.deg2rad(angles))

    x_rotated = x_coords * np.cos(rotation_angle_rad) - y_coords * np.sin(rotation_angle_rad)
    y_rotated = x_coords * np.sin(rotation_angle_rad) + y_coords * np.cos(rotation_angle_rad)

    values_rotated = np.sqrt(x_rotated**2 + y_rotated**2)
    angles_rotated = np.rad2deg(np.arctan2(y_rotated, x_rotated))

    return values_rotated, angles_rotated

def cluster_and_fit_line(values, angles):
    # Convert polar to cartesian coordinates
    x_coords = np.array(values) * np.cos(np.deg2rad(angles))
    y_coords = np.array(values) * np.sin(np.deg2rad(angles))
    
    # Stack coordinates
    coords = np.column_stack((x_coords, y_coords))
    
    # Cluster the coordinates
    clustering = DBSCAN(eps=10, min_samples=5).fit(coords)
    labels = clustering.labels_
    
    # Fit a line to the largest cluster
    unique_labels = set(labels)
    largest_cluster = max(unique_labels, key=lambda label: np.sum(labels == label))
    
    cluster_coords = coords[labels == largest_cluster]
    if cluster_coords.shape[0] < 2:
        raise ValueError("Not enough points to fit a line")

    # Linear regression to fit a line
    model = LinearRegression().fit(cluster_coords[:, 0].reshape(-1, 1), cluster_coords[:, 1])
    slope = model.coef_[0]
    intercept = model.intercept_
    
    # Calculate the angle of the line relative to the x-axis
    angle = np.arctan(slope) * (180 / np.pi)
    
    return cluster_coords, angle


def draw_fitted_line(img, coords, angle):
    if coords.shape[0] > 0:
        # Find the bounding box for the line
        min_x, max_x = np.min(coords[:, 0]), np.max(coords[:, 0])
        y1 = int(np.min(coords[:, 1]))
        y2 = int(np.max(coords[:, 1]))
        cv2.line(img, (int(min_x), y1), (int(max_x), y2), (255, 255, 0), 2)
    
    return img

def main():
    lidar = Lidar()
    result = [0, 0, 0, 0]  # Initialize the result array

    try:
        # Setup plot
        img = np.zeros((640, 640, 3), dtype=np.uint8)
        
        while True:
            data = lidar.scan_data("sRI E9")
            values, angles = parse_telegram(data)
            
            rotation_angle = -15  # Define the rotation angle in degrees
            values_rotated, angles_rotated = rotate_points(values, angles, rotation_angle)
            
            colors = get_colors(angles_rotated)
            
            img.fill(0)  # Clear the image
            result = [0, 0, 0, 0]  # Reset the result array
            
            # Draw the horizontal line at y = 530
            y_line = 530
            cv2.line(img, (0, y_line), (img.shape[1], y_line), (255, 255, 255), 2)
            
            # Define the proximity threshold (e.g., 100 units)
            proximity_threshold = 100

            for value, angle, color in zip(values_rotated, angles_rotated, colors):
                angle_rad = np.deg2rad(angle)
                x = int(320 + value * np.cos(angle_rad) * 0.1)  # Scaling down the distances
                y = int(320 - value * np.sin(angle_rad) * 0.1)
                
                # Check if the point is within the right side ROI
                if 0 <= x < img.shape[1] and y > y_line:
                    # Check if the point is within the proximity threshold
                    if value < proximity_threshold:
                        # Update the result array based on the proximity
                        section_index = (x * 4) // img.shape[1]
                        if 0 <= section_index < 4:
                            result[section_index] = 1  # Mark the section as occupied

                    # Draw the point with the respective color
                    if color == 'green':
                        cv2.circle(img, (x, y), 2, (0, 255, 0), -1)
                    elif color == 'red':
                        cv2.circle(img, (x, y), 2, (0, 0, 255), -1)
                    elif color == 'brown':
                        cv2.circle(img, (x, y), 2, (42, 42, 165), -1)  # Brown color in BGR
                    elif color == 'blue':
                        cv2.circle(img, (x, y), 2, (255, 0, 0), -1)
                    else:
                        cv2.circle(img, (x, y), 2, (0, 0, 0), -1)
            
            # Cluster points and fit a line
            try:
                cluster_coords, line_angle = cluster_and_fit_line(values_rotated, angles_rotated)
                img = draw_fitted_line(img, cluster_coords, line_angle)
                print(f"Line angle relative to the car: {line_angle:.2f} degrees")
            except ValueError as e:
                print(f"Error in clustering or line fitting: {e}")

            # Rotate the image for display
            img_r = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
            cv2.imshow('LiDAR Scan', img_r)
            
            # Display the detection results
            print("Detection Results:", result)

            if cv2.waitKey(10) & 0xFF == ord('q'):
                break

        cv2.destroyAllWindows()

    except LidarNotFound as e:
        print(e)
    except Exception as e:
        print("An error occurred:", e)

if __name__ == "__main__":
    main()
