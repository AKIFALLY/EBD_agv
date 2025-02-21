import socket
import time
from concurrent.futures import ThreadPoolExecutor, Future
import threading

# 常數定義
PLC_END_MARKER = "\r\n"
MIN_POOL_SIZE = 5
MAX_POOL_SIZE = 5
IDLE_TIMEOUT = 10
NUM_TESTS = 100  # 測試執行次數

class KeyencePlcCom:
    """Keyence PLC 通信類別"""
    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.sock = None
        self.timeout = 5
        self.last_used = time.time()
        self.connect()

    def connect(self):
        """TCP/IP 連接功能"""
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
                self.sock.sendall(command.encode('utf-8'))
                response = self.receive_until()
                self.last_used = time.time()
                return response
            except Exception as e:
                print(f"[PLC] Command Error: {e}")
                return None
        else:
            print("[PLC] Not connected.")
            return None

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
            return data.decode('utf-8', errors='ignore').strip()
        except Exception as e:
            print(f"[PLC] Receive Error: {e}")
            return None

class PlcConnectionPool:
    """基於 ThreadPoolExecutor 管理 PLC 連線池"""
    def __init__(self, ip, port, max_workers=MAX_POOL_SIZE):
        self.ip = ip
        self.port = port
        self.executor = ThreadPoolExecutor(max_workers=max_workers)
        self.connections = [KeyencePlcCom(ip, port) for _ in range(MIN_POOL_SIZE)]
        self.lock = threading.Lock()

    def _get_connection(self):
        if self.connections:
            return self.connections.pop()
        return KeyencePlcCom(self.ip, self.port)

    def _release_connection(self, plc):
        if plc and plc.sock:
            self.connections.append(plc)

    def execute(self, func, *args, **kwargs):
        plc = self._get_connection()
        try:
            return func(plc, *args, **kwargs)
        finally:
            self._release_connection(plc)

    def execute_async(self, func, *args, **kwargs) -> Future:
        """非同步執行"""
        return self.executor.submit(self.execute, func, *args, **kwargs)


# 測試函數
def read_continuous_example(plc, device_type, device_number, device_length):
    return plc.send_command(f"RDS {device_type}{device_number} {device_length}{PLC_END_MARKER}")

if __name__ == "__main__":
    plc_pool = PlcConnectionPool("192.168.0.100", 8501)

    # 測試 1: 單次讀取 1000 個 word（同步）
    single_read_times = []
    for _ in range(NUM_TESTS):
        start_time = time.time()
        plc_pool.execute(read_continuous_example, "DM", "1000", "1000")
        single_read_times.append((time.time() - start_time) * 1000)

    # 測試 2: 分 5 次，每次讀取 200 個 word（同步）
    multi_read_times = []
    for _ in range(NUM_TESTS):
        start_time = time.time()
        for i in range(5):
            plc_pool.execute(read_continuous_example, "DM", str(1000 + i * 200), "200")
        multi_read_times.append((time.time() - start_time) * 1000)

    # 測試 3: 單次讀取 1000 個 word（非同步）
    async_single_read_times = []
    for _ in range(NUM_TESTS):
        start_time = time.time()
        future = plc_pool.execute_async(read_continuous_example, "DM", "1000", "1000")
        future.result()
        async_single_read_times.append((time.time() - start_time) * 1000)

    # 測試 4: 分 5 次，每次讀取 200 個 word（非同步）
    async_multi_read_times = []
    for _ in range(NUM_TESTS):
        start_time = time.time()
        futures = []
        for i in range(5):
            futures.append(plc_pool.execute_async(read_continuous_example, "DM", str(1000 + i * 200), "200"))
        for future in futures:
            future.result()
        async_multi_read_times.append((time.time() - start_time) * 1000)

    # 計算平均時間
    avg_single_read_time = sum(single_read_times) / NUM_TESTS
    avg_multi_read_time = sum(multi_read_times) / NUM_TESTS
    avg_async_single_read_time = sum(async_single_read_times) / NUM_TESTS
    avg_async_multi_read_time = sum(async_multi_read_times) / NUM_TESTS

    # 顯示結果
    print("\n=== 測試結果（平均毫秒） ===")
    print(f"同步 單次讀取 1000 word: {avg_single_read_time:.2f} ms")
    print(f"同步 分 5 次讀取 200 word: {avg_multi_read_time:.2f} ms")
    print(f"非同步 單次讀取 1000 word: {avg_async_single_read_time:.2f} ms")
    print(f"非同步 分 5 次讀取 200 word: {avg_async_multi_read_time:.2f} ms")
