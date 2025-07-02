import asyncio
import socket
import time
import uvloop

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

    async def connect(self):
        """TCP/IP 連接功能"""
        if self.sock is None:
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.settimeout(self.timeout)
                await asyncio.get_event_loop().sock_connect(self.sock, (self.ip, self.port))
            except Exception as e:
                print(f"[PLC] Connection Error: {e}")

    def disconnect(self):
        """關閉連接"""
        if self.sock:
            self.sock.close()
            self.sock = None

    async def send_command(self, command):
        """發送命令並接收回應"""
        if self.sock:
            try:
                await asyncio.get_event_loop().sock_sendall(self.sock, command.encode('utf-8'))
                response = await self.receive_until()
                self.last_used = time.time()
                return response
            except Exception as e:
                print(f"[PLC] Command Error: {e}")
                return None
        else:
            print("[PLC] Not connected.")
            return None

    async def receive_until(self, end_marker=PLC_END_MARKER.encode(), buffer_size=1024):
        """根據協議判斷資料是否接收完畢"""
        data = b""
        try:
            while True:
                chunk = await asyncio.get_event_loop().sock_recv(self.sock, buffer_size)
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
    """基於 asyncio 管理 PLC 連線池"""
    def __init__(self, ip, port, max_workers=MAX_POOL_SIZE):
        self.ip = ip
        self.port = port
        self.connections = [KeyencePlcCom(ip, port) for _ in range(MIN_POOL_SIZE)]
        self.lock = asyncio.Lock()

    async def _get_connection(self):
        if self.connections:
            return self.connections.pop()
        plc = KeyencePlcCom(self.ip, self.port)
        await plc.connect()
        return plc

    async def _release_connection(self, plc):
        if plc and plc.sock:
            self.connections.append(plc)

    async def execute(self, func, *args, **kwargs):
        plc = await self._get_connection()
        try:
            return await func(plc, *args, **kwargs)
        finally:
            await self._release_connection(plc)

# 測試函數
async def read_example(plc, device_type, device_number):
    return await plc.send_command(f"RD {device_type}{device_number}{PLC_END_MARKER}")

async def write_example(plc, device_type, device_number, write_data):
    return await plc.send_command(f"WR {device_type}{device_number} {write_data}{PLC_END_MARKER}")

async def read_continuous_example(plc, device_type, device_number, device_length):
    return await plc.send_command(f"RDS {device_type}{device_number} {device_length}{PLC_END_MARKER}")

async def write_continuous_example(plc, device_type, device_number, write_data):
    data_str = " ".join(str(x) for x in write_data)
    return await plc.send_command(f"WRS {device_type}{device_number} {len(write_data)} {data_str}{PLC_END_MARKER}")

async def benchmark(plc_pool, func, name, async_mode=False, *args):
    start_time = time.time()
    tasks = []

    for i in range(100):
        device_number = str(1000 + i)  # 動態分配不同的設備編號
        params = args + (device_number,)

        if async_mode:
            tasks.append(plc_pool.execute(func, *params))
        else:
            await plc_pool.execute(func, *params)

    if async_mode:
        await asyncio.gather(*tasks)

    elapsed_time = (time.time() - start_time) * 1000
    print(f"{name} 總時間: {elapsed_time:.2f} 毫秒")
    return elapsed_time

async def main():
    plc_pool = PlcConnectionPool("192.168.0.100", 8501)

    print("\n=== 同步測試 ===")
    sync_read_time = await benchmark(plc_pool, read_example, "同步讀取", False, "DM")
    sync_write_time = await benchmark(plc_pool, write_example, "同步寫入", False, "DM", "123")
    sync_read_cont_time = await benchmark(plc_pool, read_continuous_example, "同步連續讀取", False, "DM", "5")
    sync_write_cont_time = await benchmark(plc_pool, write_continuous_example, "同步連續寫入", False, "DM", [1, 2, 3, 4, 5])

    print("\n=== 非同步測試 ===")
    async_read_time = await benchmark(plc_pool, read_example, "非同步讀取", True, "DM")
    async_write_time = await benchmark(plc_pool, write_example, "非同步寫入", True, "DM", "456")
    async_read_cont_time = await benchmark(plc_pool, read_continuous_example, "非同步連續讀取", True, "DM", "5")
    async_write_cont_time = await benchmark(plc_pool, write_continuous_example, "非同步連續寫入", True, "DM", [6, 7, 8, 9, 10])

    print("\n=== 測試結果統計 ===")
    print(f"同步讀取 100 次總時間: {sync_read_time:.2f} 毫秒")
    print(f"同步寫入 100 次總時間: {sync_write_time:.2f} 毫秒")
    print(f"同步連續讀取 100 次總時間: {sync_read_cont_time:.2f} 毫秒")
    print(f"同步連續寫入 100 次總時間: {sync_write_cont_time:.2f} 毫秒")
    print(f"非同步讀取 100 次總時間: {async_read_time:.2f} 毫秒")
    print(f"非同步寫入 100 次總時間: {async_write_time:.2f} 毫秒")
    print(f"非同步連續讀取 100 次總時間: {async_read_cont_time:.2f} 毫秒")
    print(f"非同步連續寫入 100 次總時間: {async_write_cont_time:.2f} 毫秒")

# 使用 uvloop 加速事件迴圈
asyncio.set_event_loop_policy(uvloop.EventLoopPolicy())

if __name__ == "__main__":
    asyncio.run(main())
