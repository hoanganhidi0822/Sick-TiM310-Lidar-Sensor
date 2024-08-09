import usb.core
import usb.util
import time

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
                print(f"Raw data received: {arr}")
                arr = "".join([chr(x) for x in arr[1:-1]])
                print(f"Processed data: {arr}")
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

if __name__ == "__main__":
    lidar = Lidar()
    
    try:
        """ print("Device: ",  lidar.device_identification())
        # Set Access Mode
        print("Access:", lidar.set_access_mode())
        time.sleep(0.1)
 """
        # Run the device
        print("Run:", lidar.run())
        time.sleep(0.1)
       
        """ # Start measurement
        print("Measurement:", lidar.start_measurement())
        time.sleep(0.1) """
        
        # Get scan data
        
        
        
        data = lidar.scan_data("sEI 5B 1")
        print("Scan Data:", data)
        
        time.sleep(0.1)
        print(lidar.scan_data("sRI E9"))
        
    except LidarNotFound as e:
        print(e)
    except Exception as e:
        print("An error occurred:", e)
