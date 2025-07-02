import socket
import rclpy.logging

# 常數定義
# pc->plc 時可\r或\r\n , plc回應時為 \r\n ,統一使用 \r\n
PLC_END_MARKER = "\r\n"
CONNECT_TIMEOUT = 5


class KeyencePlcCom:
    """Keyence PLC 通信類別"""

    # 靜態錯誤訊息表
    ERROR_MESSAGES = {
        "E0": "E0:元件編號異常",
        "E1": "E1:指令異常",
        "E4": "E4:禁止寫入",
    }

    def __init__(self, ip, port):
        self.logger = rclpy.logging.get_logger('keyence_plc_com')
        self.ip = ip
        self.port = port
        self.sock = None  # 儲存 socket 物件
        self.timeout = CONNECT_TIMEOUT  # 設定默認超時時間

    def connect_test(self):
        try:
            self.sock.sendall(b"")
            return True
        except Exception as e:
            self.sock = None  # 若發生錯誤，將sock設為None
            self.logger.warn(f"connect_test : False")
        return False

    def connect(self, test=False):
        """TCP/IP 連接功能"""
        #if self.sock is not None and self.connect_test():
        if self.sock :
            return True

        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(self.timeout)
            self.sock.connect((self.ip, self.port))
            self.logger.debug(f"Successfully connected to PLC at {self.ip}:{self.port}")
            return True
        except Exception as e:
            self.logger.warn(f"Error connecting to PLC: {e}, ip:{self.ip}, port:{self.port}")
            self.sock = None  # 若發生錯誤，將sock設為None
            raise e  # 直接拋出異常

    def disconnect(self):
        """關閉連接"""
        if self.sock is None:
            return

        try:
            self.sock.close()
        except Exception as e:
            self.logger.warn(f"Error closing socket: {e}")
        finally:
            self.logger.info(f"Disconnected from PLC at {self.ip}:{self.port}")

        self.sock = None

    def send_command(self, command):
        """發送命令並接收回應"""
        if self.sock is None:
            raise ConnectionError(f"self.sock is None ({self.ip}:{self.port})")

        try:
            self.sock.sendall(command.encode("utf-8"))
            response = self.receive_until()

            if response is None:
                self.logger.debug("response is None")
                raise ConnectionError("判斷斷線 response is None")

            response = response.strip()
            if response == "":
                self.logger.debug("response empty string")
                raise ConnectionError("判斷斷線 response is empty string")

            if response[:2] in self.ERROR_MESSAGES:
                raise Exception(self.ERROR_MESSAGES[response[:2]])

            return response

        except Exception as e:
            # 傳送失敗時，應主動斷線 或重連
            self.logger.warn(f"Error sending command: {e}")
            raise

    def receive_until(
        self, end_marker=PLC_END_MARKER.encode(), buffer_size=1024, max_empty_count=10
    ):
        """根據協議判斷資料是否接收完畢"""
        data = b""
        empty_count = 0
        try:
            while True:
                chunk = self.sock.recv(buffer_size)
                if not chunk:  # 沒有數據，增加空數據計數
                    raise ConnectionError("收到空字串 斷線")
                else:
                    empty_count = 0  # 如果有數據，重置空數據計數
                    data += chunk
                    if end_marker in data:
                        break

            response = data.decode("utf-8", errors="ignore")
            return response
        except socket.timeout:
            self.logger.warn("Timeout reached while receiving data.")
        except Exception as e:
            self.logger.warn(f"Error receiving data: {e}")
        return None
