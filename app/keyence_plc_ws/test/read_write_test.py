import time
from keyence_plc.keyence_plc_com import KeyencePlcCom, PLC_END_MARKER

# 假設 KeyencePlcCom 在 keyence_plc_com.py 檔案中


class SpeedTest:
    def __init__(self, plc_ip, plc_port):
        self.plc = KeyencePlcCom(plc_ip, plc_port)

    def test_read_speed(self, device_type, device_number, num_tests=100):
        """測試讀取速度，並計算最小/最大執行時間"""
        self.plc.connect()
        total_time = 0
        min_time = float("inf")
        max_time = float("-inf")

        for _ in range(num_tests):
            start_time = time.time()
            self.plc.read_data(device_type, device_number)
            elapsed_time = time.time() - start_time

            total_time += elapsed_time
            min_time = min(min_time, elapsed_time)
            max_time = max(max_time, elapsed_time)

        avg_time = (total_time / num_tests) * 1000  # 轉換為毫秒
        print(
            f"Read ({num_tests} times) -> Avg: {avg_time:.6f} ms, Min: {min_time * 1000:.6f} ms, Max: {max_time * 1000:.6f} ms"
        )
        self.plc.disconnect()

    def test_write_speed(self, device_type, device_number, write_data, num_tests=100):
        """測試寫入速度"""
        self.plc.connect()
        total_time = 0
        min_time = float("inf")
        max_time = float("-inf")

        for _ in range(num_tests):
            start_time = time.time()
            self.plc.write_data(device_type, device_number, write_data)
            elapsed_time = time.time() - start_time

            total_time += elapsed_time
            min_time = min(min_time, elapsed_time)
            max_time = max(max_time, elapsed_time)

        avg_time = (total_time / num_tests) * 1000
        print(
            f"Write ({num_tests} times) -> Avg: {avg_time:.6f} ms, Min: {min_time * 1000:.6f} ms, Max: {max_time * 1000:.6f} ms"
        )
        self.plc.disconnect()

    def test_continuous_read_speed(
        self, device_type, device_number, device_length, num_tests=100
    ):
        """測試連續讀取速度"""
        self.plc.connect()
        total_time = 0
        min_time = float("inf")
        max_time = float("-inf")

        for _ in range(num_tests):
            start_time = time.time()
            self.plc.read_continuous_data(device_type, device_number, device_length)
            elapsed_time = time.time() - start_time

            total_time += elapsed_time
            min_time = min(min_time, elapsed_time)
            max_time = max(max_time, elapsed_time)

        avg_time = (total_time / num_tests) * 1000
        print(
            f"Continuous Read ({num_tests} times) -> Avg: {avg_time:.6f} ms, Min: {min_time * 1000:.6f} ms, Max: {max_time * 1000:.6f} ms"
        )
        self.plc.disconnect()

    def test_continuous_write_speed(
        self, device_type, device_number, write_data, num_tests=100
    ):
        """測試連續寫入速度"""
        self.plc.connect()
        total_time = 0
        min_time = float("inf")
        max_time = float("-inf")

        for _ in range(num_tests):
            start_time = time.time()
            # print(f"Rsp:{self.plc.write_continuous_data(device_type, device_number, write_data)}")
            self.plc.write_continuous_data(device_type, device_number, write_data)
            elapsed_time = time.time() - start_time

            total_time += elapsed_time
            min_time = min(min_time, elapsed_time)
            max_time = max(max_time, elapsed_time)

        avg_time = (total_time / num_tests) * 1000
        print(
            f"Continuous Write ({num_tests} times) -> Avg: {avg_time:.6f} ms, Min: {min_time * 1000:.6f} ms, Max: {max_time * 1000:.6f} ms"
        )
        self.plc.disconnect()


if __name__ == "__main__":
    plc_ip = "192.168.0.100"  # PLC IP
    plc_port = 8501  # PLC 端口

    test = SpeedTest(plc_ip, plc_port)

    print("---")
    test.test_read_speed("DM", 5000, 100)
    print("---")
    test.test_write_speed("DM", 5000, 123, 100)
    print("---")
    test.test_continuous_read_speed("DM", 5000, 1, 100)
    print("---")
    test.test_continuous_write_speed("DM", 5000, [1, 2, 3, 4, 5, 6, 7, 8, 9, 10], 100)
    print("---")
