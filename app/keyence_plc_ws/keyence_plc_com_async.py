import socket
import time
from concurrent.futures import ThreadPoolExecutor, Future
import threading

# 常數定義
PLC_END_MARKER = "\r\n"
MIN_POOL_SIZE = 1
MAX_POOL_SIZE = 5
IDLE_TIMEOUT = 10

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
        self.max_threads_used = 0

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
        """非同步執行，並記錄最大執行緒數量"""
        future = self.executor.submit(self.execute, func, *args, **kwargs)
        with self.lock:
            self.max_threads_used = max(self.max_threads_used, len(self.executor._threads))
        return future

    def execute_and_wait(self, func, *args, **kwargs):
        """非同步執行後等待結果"""
        return self.execute_async(func, *args, **kwargs).result()

# 測試函數
def read_example(plc, device_type, device_number):
    return plc.send_command(f"RD {device_type}{device_number}{PLC_END_MARKER}")

def write_example(plc, device_type, device_number, write_data):
    return plc.send_command(f"WR {device_type}{device_number} {write_data}{PLC_END_MARKER}")

def read_continuous_example(plc, device_type, device_number, device_length):
    return plc.send_command(f"RDS {device_type}{device_number} {device_length}{PLC_END_MARKER}")

def write_continuous_example(plc, device_type, device_number, write_data):
    data_str = " ".join(str(x) for x in write_data)
    return plc.send_command(f"WRS {device_type}{device_number} {len(write_data)} {data_str}{PLC_END_MARKER}")

def benchmark(plc_pool, func, name, async_mode=False, *args):
    start_time = time.time()
    futures = []

    for i in range(100):
        device_number = str(1000 + i)  # 動態分配不同的設備編號
        params = args + (device_number,)

        if async_mode:
            futures.append(plc_pool.execute_async(func, *params))
        else:
            plc_pool.execute(func, *params)

    if async_mode:
        for future in futures:
            future.result()

    elapsed_time = (time.time() - start_time) * 1000
    print(f"{name} 總時間: {elapsed_time:.2f} 毫秒")
    return elapsed_time

if __name__ == "__main__":
    plc_pool = PlcConnectionPool("192.168.0.100", 8501)

    print("\n=== 同步測試 ===")
    sync_read_time = benchmark(plc_pool, read_example, "同步讀取", False, "DM")
    sync_write_time = benchmark(plc_pool, write_example, "同步寫入", False, "DM", "123")
    sync_read_cont_time = benchmark(plc_pool, read_continuous_example, "同步連續讀取", False, "DM", "5")
    sync_write_cont_time = benchmark(plc_pool, write_continuous_example, "同步連續寫入", False, "DM", [1, 2, 3, 4, 5])

    print("\n=== 非同步測試 ===")
    async_read_time = benchmark(plc_pool, read_example, "非同步讀取", True, "DM")
    async_write_time = benchmark(plc_pool, write_example, "非同步寫入", True, "DM", "456")
    async_read_cont_time = benchmark(plc_pool, read_continuous_example, "非同步連續讀取", True, "DM", "5")
    async_write_cont_time = benchmark(plc_pool, write_continuous_example, "非同步連續寫入", True, "DM", [6, 7, 8, 9, 10])

    print("\n=== 測試結果統計 ===")
    print(f"最大使用執行緒數量: {plc_pool.max_threads_used}")
    print(f"同步讀取 100 次總時間: {sync_read_time:.2f} 毫秒")
    print(f"同步寫入 100 次總時間: {sync_write_time:.2f} 毫秒")
    print(f"同步連續讀取 100 次總時間: {sync_read_cont_time:.2f} 毫秒")
    print(f"同步連續寫入 100 次總時間: {sync_write_cont_time:.2f} 毫秒")
    print(f"非同步讀取 100 次總時間: {async_read_time:.2f} 毫秒")
    print(f"非同步寫入 100 次總時間: {async_write_time:.2f} 毫秒")
    print(f"非同步連續讀取 100 次總時間: {async_read_cont_time:.2f} 毫秒")
    print(f"非同步連續寫入 100 次總時間: {async_write_cont_time:.2f} 毫秒")
