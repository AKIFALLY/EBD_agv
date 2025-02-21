import time
import socket


class MockKeyencePlcCom:
    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.sock = None  # 可能是 socket 物件或 None

    def connect(self):
        """TCP/IP 連接功能"""
        print("MockKeyencePlcCom - connect")
        return True

    def reconnect(self):
        """TCP/IP 斷線重連功能"""
        self.disconnect()
        print("MockKeyencePlcCom - reconnect")
        time.sleep(1)
        return self.connect()

    def disconnect(self):
        """關閉連接"""
        print("MockKeyencePlcCom - disconnect")

    def send_command(self, command):
        """發送命令並接收回應"""
        print("MockKeyencePlcCom - 發送命令並接收回應")
        return "OK\r\n"

    def force_on(self, device_type, device_number):
        """ForceOn 功能"""
        print("MockKeyencePlcCom - ForceOn 功能")
        return "MockKeyencePlcCom - ForceOn 功能"

    def force_off(self, device_type, device_number):
        """ForceOff 功能"""
        print("MockKeyencePlcCom - ForceOff 功能")
        return "MockKeyencePlcCom - ForceOff 功能"
    
    def read_data(self, device_type, device_number):
        """讀取PLC資料"""
        print("MockKeyencePlcCom - read_data 功能")
        return "MockKeyencePlcCom - read_data 功能"

    def write_data(self, device_type, device_number, write_data):
        """寫入PLC資料"""
        print("MockKeyencePlcCom - write_data 功能")
        return "MockKeyencePlcCom - write_data 功能"

    def read_continuous_data(self, device_type, device_number, device_length):
        """連續讀取PLC資料"""
        print("MockKeyencePlcCom - read_continuous_data 功能")
        return ["MockKeyencePlcCom - read_continuous_data 功能"]

    def write_continuous_data(self, device_type, device_number, write_data):
        """連續寫入PLC資料"""
        print("MockKeyencePlcCom - write_continuous_data 功能")
        return "MockKeyencePlcCom - write_continuous_data 功能"


def receive_until(sock, end_marker=b"\r\n", buffer_size=1024, timeout=5):
    """根據協議判斷資料是否接收完畢"""
    return ""
