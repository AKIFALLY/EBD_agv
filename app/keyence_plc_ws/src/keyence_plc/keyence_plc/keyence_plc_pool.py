import threading
from keyence_plc.keyence_plc_com import KeyencePlcCom

# 常數定義
MIN_POOL_SIZE = 1
MAX_POOL_SIZE = 5


class KeyencePlcPool:
    """基於 Semaphore 管理 PLC 連線池"""

    def __init__(self, ip, port, max_connections=MAX_POOL_SIZE):
        self.ip = ip
        self.port = port
        self.max_connections = max_connections
        self.connections = [KeyencePlcCom(ip, port) for _ in range(max_connections)]
        self.lock = threading.Lock()  # 使用鎖來保護連線池操作
        self.semaphore = threading.Semaphore(max_connections)  # 限制最大連線數量

    def _get_connection(self):
        """取得一條可用的連線"""
        # 等待可用的連線，若達到最大連線數則阻塞
        self.semaphore.acquire()
        try:
            with self.lock:
                if self.connections:
                    con = self.connections.pop()
                    if con.connect():
                        return con

            # 如果沒有可用連線，創建新連線
            con = KeyencePlcCom(self.ip, self.port)
            if con.connect():
                return con

            # 若無法建立連線，丟出例外
            raise ConnectionError(f"無法連接到 PLC ({self.ip}:{self.port})")
        except Exception as e:
            raise e  # 重新丟出例外

    def _release_connection(self, plc):
        """釋放一條連線"""
        if plc is not None and plc.sock is not None:
            with self.lock:
                self.connections.append(plc)
        self.semaphore.release()  # 釋放信號量，表明有一個連線可用

    def execute(self, command):
        """執行 PLC 命令並返回結果"""
        plc = None
        try:
            plc = self._get_connection()
            return plc.send_command(command)
        except Exception as e:
            if plc:
                plc.disconnect()
            print(f"!!!Error: {e}")
            raise  # 重新丟出例外
        finally:
            self._release_connection(plc)

    def close_connection(self):
        """同步關閉所有 PLC 連線並釋放資源"""
        with self.lock:
            for plc in self.connections:
                plc.disconnect()  # 假設 plc.disconnect() 是同步方法
