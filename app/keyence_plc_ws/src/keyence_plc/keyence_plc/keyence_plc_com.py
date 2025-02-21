import socket

# 常數定義
# pc->plc 時可\r或\r\n , plc回應時為 \r\n ,統一使用 \r\n
PLC_END_MARKER = "\r\n"
CONNECT_TIMEOUT = 10


class KeyencePlcCom:
    """Keyence PLC 通信類別"""

    # 靜態錯誤訊息表
    ERROR_MESSAGES = {
        "E0": "E0:元件編號異常",
        "E1": "E1:指令異常",
        "E4": "E4:禁止寫入",
    }

    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.sock = None  # 儲存 socket 物件
        self.timeout = CONNECT_TIMEOUT  # 設定默認超時時間

    def connect(self):
        """TCP/IP 連接功能"""
        if self.sock is not None:  # 檢查 socket 是否已經存在
            try:
                self.sock.sendall(b"")
                return True
            except Exception as e:
                print(f"Error Lost connection")
                self.sock = None  # 若發生錯誤，將sock設為None

        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(self.timeout)
            self.sock.connect((self.ip, self.port))
            print(f"Successfully connected to PLC at {self.ip}:{self.port}")
            return True
        except Exception as e:
            print(f"Error connecting to PLC: {e}, ip:{self.ip}, port:{self.port}")
            self.sock = None  # 若發生錯誤，將sock設為None
            raise e  # 直接拋出異常

    def disconnect(self):
        """關閉連接"""
        if self.sock:
            try:
                self.sock.close()
            except Exception as e:
                print(f"Error closing socket: {e}")
            finally:
                self.sock = None
                print(f"Disconnected from PLC at {self.ip}:{self.port}")
        else:
            print("Not connected to PLC.")

    def send_command(self, command):
        """發送命令並接收回應"""
        if self.sock is None:
            raise ConnectionError(f"self.sock is None ({self.ip}:{self.port})")

        try:
            self.sock.sendall(command.encode("utf-8"))
            response = self.receive_until()

            if response is None or response.strip() == "":
                raise ConnectionError(
                    f"從PLC接收到空字串 判斷為斷線 ({self.ip}:{self.port})"
                )

            response = response.strip()
            if response[:2] in self.ERROR_MESSAGES:
                raise Exception(self.ERROR_MESSAGES[response[:2]])

            return response

        except Exception as e:
            # 傳送失敗時，主動斷線
            self.disconnect()
            print(f"Error sending command: {e}")
            raise

    def receive_until(self, end_marker=PLC_END_MARKER.encode(), buffer_size=1024):
        """根據協議判斷資料是否接收完畢"""
        data = b""
        try:
            while True:
                chunk = self.sock.recv(buffer_size)
                if not chunk:
                    break
                data += chunk
                if end_marker in data:
                    break
            return data.decode("utf-8", errors="ignore")
        except socket.timeout:
            print("Timeout reached while receiving data.")
        except Exception as e:
            print(f"Error receiving data: {e}")
        return None
