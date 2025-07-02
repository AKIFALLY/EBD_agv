import socket
import time
from concurrent.futures import ThreadPoolExecutor, Future
from keyence_plc.keyence_plc_com import KeyencePlcCom, PLC_END_MARKER

# 假設 KeyencePlcCom 在 keyence_plc_com.py 檔案中

# 常數定義
MIN_POOL_SIZE = 1  # 最少保持 1 條連線
MAX_POOL_SIZE = 5  # 最大 5 條連線
IDLE_TIMEOUT = 10  # 連線閒置超過 10 秒則關閉


class PlcConnectionPool:
    """基於 ThreadPoolExecutor 管理 PLC 連線池"""

    def __init__(self, ip, port, max_workers=MAX_POOL_SIZE):
        self.ip = ip
        self.port = port
        self.executor = ThreadPoolExecutor(max_workers=max_workers)
        self.connections = [KeyencePlcCom(ip, port) for _ in range(MIN_POOL_SIZE)]

    def _get_connection(self):
        """取得一條可用的連線，若無法取得則丟出例外"""
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

    def _release_connection(self, plc):
        """將使用完的連線放回池內"""
        if plc and plc.sock:
            self.connections.append(plc)

    def execute(self, func, *args, **kwargs):
        """同步執行 PLC 指令"""
        plc = self._get_connection()
        try:
            result = func(plc, *args, **kwargs)
            return result
        except ConnectionError as e:
            # 可以在這裡處理例外，或者重新丟出
            print(f"ConnectionError: {e}")
            raise
        finally:
            self._release_connection(plc)

    def execute_async(self, func, *args, **kwargs) -> Future:
        """非同步執行 PLC 指令"""
        return self.executor.submit(self.execute, func, *args, **kwargs)


# 測試函數
def read_data(plc, device_type, device_number):
    """讀取PLC資料"""
    return plc.send_command(f"RD {device_type}{device_number}{PLC_END_MARKER}")


def write_data(plc, device_type, device_number, write_data):
    """寫入PLC資料"""
    return plc.send_command(
        f"WR {device_type}{device_number} {write_data}{PLC_END_MARKER}"
    )


def read_continuous_data(plc, device_type, device_number, device_length):
    """連續讀取PLC資料"""
    return plc.send_command(
        f"RDS {device_type}{device_number} {device_length}{PLC_END_MARKER}"
    )


def write_continuous_data(plc, device_type, device_number, write_data):
    """連續寫入PLC資料"""
    data_str = " ".join(str(x) for x in write_data)
    return plc.send_command(
        f"WRS {device_type}{device_number} {len(write_data)} {data_str}{PLC_END_MARKER}"
    )


def force_on_data(plc, device_type, device_number):
    """強制On"""
    return plc.send_command(f"ST {device_type}{device_number}{PLC_END_MARKER}")


def force_off_data(plc, device_type, device_number):
    """強制Off"""
    return plc.send_command(f"RS {device_type}{device_number}{PLC_END_MARKER}")


# 測試執行
if __name__ == "__main__":
    plc_pool = PlcConnectionPool("192.168.0.100", 8501)

    # 測試讀取
    result1 = plc_pool.execute(read_data, "DM", "100")
    print("[讀取] 結果:", result1)

    # 測試寫入
    result2 = plc_pool.execute(write_data, "DM", "100", "123")
    print("[寫入] 結果:", result2)

    # 測試連續讀取
    result3 = plc_pool.execute(read_continuous_data, "DM", "100", "5")
    print("[連續讀取] 結果:", result3)

    # 測試連續寫入
    result4 = plc_pool.execute(
        write_continuous_data, "DM", "100", ["1", "2", "3", "4", "5"]
    )
    print("[連續寫入] 結果:", result4)

    # 測試強制 ON
    result5 = plc_pool.execute(force_on_data, "MR", "10")
    print("[強制ON] 結果:", result5)

    # 測試強制 OFF
    result6 = plc_pool.execute(force_off_data, "MR", "10")
    print("[強制OFF] 結果:", result6)

    # 非同步測試
    future1 = plc_pool.execute_async(read_data, "DM", "200")
    future2 = plc_pool.execute_async(write_data, "DM", "200", "456")

    print("[非同步讀取] 結果:", future1.result())
    print("[非同步寫入] 結果:", future2.result())
