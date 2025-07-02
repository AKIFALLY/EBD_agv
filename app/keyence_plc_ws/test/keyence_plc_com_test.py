import socket
import time

# 常數定義
PLC_END_MARKER = "\r\n"


class KeyencePlcCom:
    """Keyence PLC 通信類別"""

    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.sock = None
        self.timeout = 5
        self.connect()

    def connect(self):
        """TCP/IP 連接"""
        if self.sock is None:
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.settimeout(self.timeout)
                self.sock.connect((self.ip, self.port))
            except Exception as e:
                print(f"[PLC] Connection Error: {e}")

    def disconnect(self):
        """關閉連接"""
        if self.sock:
            self.sock.close()
            self.sock = None

    def send_command(self, command):
        """發送命令並接收回應"""
        if self.sock:
            try:
                self.sock.sendall(command.encode("utf-8"))
                return self.receive_until()
            except Exception as e:
                print(f"[PLC] Command Error: {e}")
                return None
        else:
            print("[PLC] Not connected.")
            return None

    def receive_until(self, end_marker=PLC_END_MARKER.encode(), buffer_size=1024):
        """接收完整資料"""
        data = b""
        try:
            while True:
                chunk = self.sock.recv(buffer_size)
                if not chunk:
                    break
                data += chunk
                if end_marker in data:
                    break
            return data.decode("utf-8", errors="ignore").strip()
        except Exception as e:
            print(f"[PLC] Receive Error: {e}")
            return None


# 測試執行
if __name__ == "__main__":
    plc = KeyencePlcCom("192.168.0.100", 8501)
