import socket
import re
import time
import threading


class SensorPart:
    def __init__(self, host='192.168.2.100', port=2005):
        self.host = host
        self.port = port
        self.client_socket = None
        self.is_connected = False
        self.position_data = None  # To store 3D positioning data
        self.ocr_result = None     # To store OCR result
        self.thread = None
        self.stop_event = threading.Event()

    def connect(self):
        while not self.is_connected and not self.stop_event.is_set():
            try:
                self.client_socket = socket.socket(
                    socket.AF_INET, socket.SOCK_STREAM)
                self.client_socket.connect((self.host, self.port))
                self.is_connected = True
                print("Connected to server.")
            except socket.error as e:
                print(f"Connection failed: {e}. Retrying in 5 seconds...")
                time.sleep(5)

    def disconnect(self):
        if self.client_socket:
            self.client_socket.close()
            self.is_connected = False
            print("Disconnected from server.")

    def handle_message(self, message):
        # Regex patterns for 3D positioning and OCR results
        position_pattern = r"\((005),(P|F),(\d+),(\d+),(\d+),([-\d.]+),([-\d.]+),([-\d.]+)\)"
        ocr_pattern = r"\((OCR),(.+)\)"

        if match := re.match(position_pattern, message):
            _, status, x, y, z, rx, ry, rz = match.groups()
            if status == 'P':  # Success
                self.position_data = {
                    'x': int(x),
                    'y': int(y),
                    'z': int(z),
                    'rx': float(rx),
                    'ry': float(ry),
                    'rz': float(rz)
                }
                print(f"3D Positioning Data Updated: {self.position_data}")
            else:
                print("3D Positioning Data Invalid.")
        elif match := re.match(ocr_pattern, message):
            _, ocr_string = match.groups()  # Updated to match new pattern
            self.ocr_result = ocr_string
            print(f"OCR Result Updated: {self.ocr_result}")
        else:
            print("Unrecognized message format. Ignoring.")

    def listen(self):
        try:
            while not self.stop_event.is_set():
                data = self.client_socket.recv(1024)
                if not data:
                    print("Connection lost. Reconnecting...")
                    self.disconnect()
                    self.connect()
                else:
                    message = data.decode('utf-8').strip()
                    print(f"Received: {message}")
                    self.handle_message(message)
        except socket.error as e:
            if not self.stop_event.is_set():
                print(f"Socket error: {e}. Reconnecting...")
                self.disconnect()
                self.connect()

    def run(self):
        self.connect()
        self.listen()

    def start(self):
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()
        print("TCP Client started in a separate thread.")

    def stop(self):
        self.stop_event.set()
        self.disconnect()
        if self.thread and self.thread.is_alive():
            print("Waiting for TCP Client thread to finish...")
            if threading.current_thread() != self.thread:
                self.thread.join(timeout=1.0)
            else:
                print("⚠️ 無法在 sensopart 執行緒內 join 自己，略過 join()")
        print("TCP Client stopped.")


def main():
    client = SensorPart()
    try:
        client.start()
        while True:
            time.sleep(1)  # Simulate ROS2 node running
    except KeyboardInterrupt:
        print("Shutting down...")
        client.stop()


if __name__ == "__main__":
    main()
