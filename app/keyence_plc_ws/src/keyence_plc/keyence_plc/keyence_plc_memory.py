import struct
import re
from keyence_plc.keyence_plc_com import KeyencePlcCom, PLC_END_MARKER
from keyence_plc.keyence_plc_bytes import PlcBytes  # 引入 PlcBytes 類別


class PlcMemory:
    def __init__(self, size: int = 131072):  # 先用65535*2 記憶體DM區塊 當作最大值
        self.memory = PlcBytes(size)  # 使用 PlcBytes 儲存整個 PLC 記憶體

    def address_to_index(self, address: int) -> int:
        """將 PLC 地址轉換為 PlcBytes 的索引"""
        return address * 2

    def word_to_bytes(self, word: int) -> int:
        """將 PLC word長度轉換為 byte 的長度"""
        return word * 2

    def set_memory(self, address: int, data: PlcBytes) -> None:
        """將原始資料寫入記憶體"""
        index = self.address_to_index(address)
        self.memory[index : index + len(data)] = data

    def get_bytes(self, address: int, length: int) -> PlcBytes:
        """從記憶體讀取 PlcBytes 資料，回傳 PlcBytes 以支援額外的轉換方法"""
        index = self.address_to_index(address)
        return PlcBytes(self.memory[index : index + length])

    def set_int(self, address: int, value: int, length: int = 2) -> None:
        """設置 int (預設 2 bytes) 資料"""
        self.set_memory(address, PlcBytes.from_int(value, length))

    def get_int(self, address: int, length: int = 2) -> int:
        """讀取 int (預設 2 bytes 1 word) 資料"""
        return self.get_bytes(address, length).to_int()

    def set_float(self, address: int, value: float) -> None:
        """設置 float (4 bytes) 資料"""
        self.set_memory(address, PlcBytes.from_float(value))

    def get_float(self, address: int, length: int = 4) -> float:
        """讀取 float (預設 4 bytes 2 word) 資料"""
        return self.get_bytes(address, length).to_float()

    def set_bit(self, address: int, bit_index: int, value: bool) -> None:
        """設置特定 bit 為 0 或 1"""
        base_address = self.address_to_index(address)
        self.memory.set_bool(
            base_address, bit_index, value
        )  # 使用 PlcBytes 的 set_bool 方法

    def get_bit(self, address: int, bit_index: int) -> bool:
        """讀取特定 bit"""
        base_address = self.address_to_index(address)
        return self.memory.get_bit(
            base_address, bit_index
        )  # 使用 PlcBytes 的 get_bool 方法

    def set_string(self, address: int, value: str) -> None:
        """設置 ASCII 字串"""
        self.set_memory(address, PlcBytes.from_string(value))

    def get_string(self, address: int, length: int) -> str:
        """讀取 ASCII 字串"""
        return self.get_bytes(address, length).to_string()

    def get_bool(self, address_dot_index: str) -> bool:
        """讀取特定 bit,支持 DM7600.0 和 7600.0"""
        match = re.fullmatch(r"(?:DM)?(\d+)\.(\d+)", address_dot_index, re.IGNORECASE)
        if not match:
            raise ValueError(f"Invalid address format: {address_dot_index}")

        address, bit_index = map(int, match.groups())
        return self.get_bit(address, bit_index) == 1  # True / False
    


    def get_value(self, address: str, format: str = "int", length: int = 2, bit_index: int = 0) -> any:
        """通用讀取 PLC 記憶體資料的方法"""
        format = format.lower()
        if format == "int":
            return self.get_int(int(address), length)
        elif format == "float":
            return self.get_float(int(address), length)
        elif format == "string":
            return self.get_string(int(address), length)
        elif format == "bit":
            return self.get_bit(int(address), bit_index)
        elif format == "bool":
            return self.get_bool(address)
        elif format == "bytes":
            return self.get_bytes(int(address), length)
        else:
            raise ValueError(f"Unsupported format: {format}")
        
    def set_value(self, address: int, value, format: str = "int", length: int = 2, bit_index: int = 0) -> None:
        """通用寫入 PLC 記憶體資料的方法"""
        format = format.lower()
        if format == "int":
            self.set_int(address, int(value), length)
        elif format == "float":
            self.set_float(address, float(value))
        elif format == "string":
            self.set_string(address, str(value))
        elif format == "bit":
            self.set_bit(address, bit_index, bool(value))
        elif format == "bytes":
            if not isinstance(value, (bytes, bytearray, PlcBytes)):
                raise TypeError("For 'bytes' format, value must be bytes-like.")
            self.set_memory(address, PlcBytes(value))
        else:
            raise ValueError(f"Unsupported format: {format}")


# 測試程式
def main():
    # plc 記憶體 DM
    plc_memory = PlcMemory()
    plc = KeyencePlcCom("192.168.0.100", 8501)

    # 讀取 DM7600 開始的 100 個 word
    response = plc.send_command(f"RDS DM7600 100{PLC_END_MARKER}")
    print("PLC 回應:", response)

    if response:
        data = response.split()  # 拆分字串
        try:
            # **轉換成 PlcBytes，每個 16-bit 轉成 2 bytes（小端序）**
            data_bytes = PlcBytes()  # 使用 PlcBytes 來儲存資料
            for x in data:
                word = int(x) & 0xFFFF  # 確保是 16-bit 整數
                data_bytes.extend(
                    PlcBytes.from_int(word, 2)
                )  # 使用 PlcBytes.from_int 轉換

            # 儲存到記憶體
            plc_memory.set_memory(7600, data_bytes)

            # 讀取時資料長度都是幾個byte計算
            # 測試讀取字串
            print(
                "AGV_ID", plc_memory.get_string(7600, 20).replace("\x00", "")
            )  # get_string,長度是幾個byte(幾個字元)
            print("電池電量", plc_memory.get_float(7610, 4))  # 4byte = 2 word
            print("AGV_X_VEL", plc_memory.get_int(7612, 4))  # 4byte = 2 word
            print("AGV_Y_VEL", plc_memory.get_int(7614, 4))  # 4byte = 2 word
            print("AGV_A_VEL", plc_memory.get_int(7616, 4))  # 4byte = 2 word

            print("AGV_前PGV", plc_memory.get_int(7618, 4))  # 4byte = 2 word
            print("AGV_後PGV", plc_memory.get_int(7620, 4))  # 4byte = 2 word
            print("AGV_起點", plc_memory.get_int(7622, 4))  # 4byte = 2 word
            print("AGV_終點", plc_memory.get_int(7624, 4))  # 4byte = 2 word
            print("AGV_動作", plc_memory.get_int(7626, 4))  # 4byte = 2 word
            print("AGV_區防", plc_memory.get_int(7628, 4))  # 4byte = 2 word
            print("AGV_SLAM_X", plc_memory.get_int(7630, 4))  # 4byte = 2 word
            print("AGV_SLAM_Y", plc_memory.get_int(7632, 4))  # 4byte = 2 word
            print("AGV_SLAM_A", plc_memory.get_int(7634, 4))  # 4byte = 2 word
            # AGV狀態 1 DM7636
            print("AGV自動狀態:", plc_memory.get_bit(7636, 0))  # DM7636.0
            print("AGV行走中:", plc_memory.get_bit(7636, 1))  # DM7636.1
            print("AGV異常:", plc_memory.get_bit(7636, 2))  # DM7636.2
            print("AGV有路徑資料:", plc_memory.get_bit(7636, 3))  # DM7636.3
            print("AGV詢問路徑請求:", plc_memory.get_bit(7636, 4))  # DM7636.4
            print("AGV執行動作中:", plc_memory.get_bit(7636, 5))  # DM7636.5
            print("LOCAL:", plc_memory.get_bit(7636, 6))  # DM7636.6
            print("AGV取料完成:", plc_memory.get_bit(7636, 7))  # DM7636.7
            print("AGV放料完成:", plc_memory.get_bit(7636, 8))  # DM7636.8
            print("AGV電量不足:", plc_memory.get_bit(7636, 9))  # DM7636.9
            print("AGV任務取消:", plc_memory.get_bit(7636, 10))  # DM7636.10
            print("AGV交管詢問:", plc_memory.get_bit(7636, 11))  # DM7636.11
            print("AGV允許通過:", plc_memory.get_bit(7636, 12))  # DM7636.12
            print("AGV請求回充電站:", plc_memory.get_bit(7636, 13))  # DM7636.13
            print("未定義:", plc_memory.get_bit(7636, 14))  # DM7636.14 無
            print("走定義:", plc_memory.get_bit(7636, 15))  # DM7636.15 無

            # AGV狀態 2 DM7637
            print("在席1:", plc_memory.get_bit(7637, 0))  # DM7637.0
            print("在席2:", plc_memory.get_bit(7637, 1))  # DM7637.1
            print("在席3:", plc_memory.get_bit(7637, 2))  # DM7637.2
            print("在席4:", plc_memory.get_bit(7637, 3))  # DM7637.3
            print("在席5:", plc_memory.get_bit(7637, 4))  # DM7637.4
            print("BarCodeReader讀取完成:", plc_memory.get_bit(7637, 5))  # DM7636.5

            # AGV狀態 3 DM7638
            print("開關門請求:", plc_memory.get_bit(7638, 0))  # DM7638.0

            # AGV_Alarm1
            print("AGV_ALARM1", plc_memory.get_int(7639, 2))  # 4byte = 2 word
            print("AGV_ALARM2", plc_memory.get_int(7640, 2))  # 4byte = 2 word
            print("AGV_ALARM3", plc_memory.get_int(7641, 2))  # 4byte = 2 word
            print("Layer", plc_memory.get_int(7642, 2))  # 4byte = 2 word
            print("AGV_ALARM4", plc_memory.get_int(7643, 2))  # 4byte = 2 word
            print("AGV_ALARM5", plc_memory.get_int(7644, 2))  # 4byte = 2 word
            print("AGV_ALARM6", plc_memory.get_int(7645, 2))  # 4byte = 2 word

            print(
                "ID1", plc_memory.get_string(7650, 20).replace("\x00", "")
            )  # get_string,長度是幾個byte(幾個字元)
            print(
                "Magic", plc_memory.get_string(7660, 20).replace("\x00", "")
            )  # get_string,長度是幾個byte(幾個字元)

            # I/O 轉換為大寫 HEX 字串 顯示
            print("INPUT:", plc_memory.get_bytes(7670, 2).hex().upper())  # DM7670
            print("INPUT:", plc_memory.get_bytes(7671, 2).hex().upper())  # DM7671
            print("INPUT:", plc_memory.get_bytes(7672, 2).hex().upper())  # DM7672
            print("INPUT:", plc_memory.get_bytes(7673, 2).hex().upper())  # DM7673
            print("INPUT:", plc_memory.get_bytes(7674, 2).hex().upper())  # DM7674

            print("OUTPUT:", plc_memory.get_bytes(7675, 2).hex().upper())  # DM7675
            print("OUTPUT:", plc_memory.get_bytes(7676, 2).hex().upper())  # DM7676
            print("OUTPUT:", plc_memory.get_bytes(7677, 2).hex().upper())  # DM7677
            print("OUTPUT:", plc_memory.get_bytes(7678, 2).hex().upper())  # DM7678
            print("OUTPUT:", plc_memory.get_bytes(7679, 2).hex().upper())  # DM7679

            print("==============================================================")
            print("AGV電量不足:", plc_memory.get_bool("DM7636.9"))  # DM7636.9
            print("data:", plc_memory.get_bytes(7636, 2).hex().upper())  # DM7636
            print("==============================================================")

            # i = 4byte = 2 word
            agvid, battery, agv_x_vel, agv_y_vel, agv_a_vel = struct.unpack(
                "<20sf3i", plc_memory.get_bytes(7600, 36)
            )
            print("AGVID:", agvid.decode("ascii", errors="ignore"))
            print("電池電量:", battery)
            print("AGV_X_VEL", agv_x_vel)  # 4byte = 2 word
            print("AGV_Y_VEL", agv_y_vel)  # 4byte = 2 word
            print("AGV_A_VEL", agv_a_vel)  # 4byte = 2 word

            (
                pgv_f,
                pgv_b,
                agv_start,
                agv_end,
                agv_act,
                agv_safe_level,
                slam_x,
                slam_y,
                slam_a,
            ) = struct.unpack("<9i", plc_memory.get_bytes(7618, 36))
            print("AGV_前PGV", pgv_f)  # 4byte = 2 word
            print("AGV_後PGV", pgv_b)  # 4byte = 2 word
            print("AGV_起點", agv_start)  # 4byte = 2 word
            print("AGV_終點", agv_end)  # 4byte = 2 word
            print("AGV_動作", agv_act)  # 4byte = 2 word
            print("AGV_區防", agv_safe_level)  # 4byte = 2 word
            print("AGV_SLAM_X", slam_x)  # 4byte = 2 word
            print("AGV_SLAM_Y", slam_y)  # 4byte = 2 word
            print("AGV_SLAM_A", slam_a)  # 4byte = 2 word

            # 使用 struct.unpack 解包 2 個字節
            unpacked_value = struct.unpack("<H", plc_memory.get_bytes(7636, 2))[0]
            # 將每個位元轉換為布林值 (0 或 1)
            bool_values = [(unpacked_value >> i) & 1 == 1 for i in range(16)]
            is_auto = bool_values[0]
            is_run = bool_values[1]
            is_error = bool_values[2]
            is_path = bool_values[3]
            is_path_req = bool_values[4]
            is_act = bool_values[5]
            is_local = bool_values[6]
            is_get_done = bool_values[7]
            is_put_done = bool_values[8]
            is_low_battery = bool_values[9]
            is_cancel = bool_values[10]
            is_traffic_req = bool_values[11]
            is_allow_pass = bool_values[12]
            is_go_home = bool_values[13]
            is_undefined_14 = bool_values[14]
            is_undefined_15 = bool_values[15]
            print("AGV自動狀態:", is_auto)  # DM7636.0
            print("AGV行走中:", is_run)  # DM7636.1
            print("AGV異常:", is_error)  # DM7636.2
            print("AGV有路徑資料:", is_path)  # DM7636.3
            print("AGV詢問路徑請求:", is_path_req)  # DM7636.4
            print("AGV執行動作中:", is_act)  # DM7636.5
            print("LOCAL:", is_local)  # DM7636.6
            print("AGV取料完成:", is_get_done)  # DM7636.7
            print("AGV放料完成:", is_put_done)  # DM7636.8
            print("AGV電量不足:", is_low_battery)  # DM7636.9
            print("AGV任務取消:", is_cancel)  # DM7636.10
            print("AGV交管詢問:", is_traffic_req)  # DM7636.11
            print("AGV允許通過:", is_allow_pass)  # DM7636.12
            print("AGV請求回充電站:", is_go_home)  # DM7636.13
            print("未定義:", is_undefined_14)  # DM7636.14 無
            print("走定義:", is_undefined_15)  # DM7636.15 無

            # 使用 struct.unpack 解包 1 個字節 (2個byte 這裡只拿1個)
            unpacked_value = struct.unpack("<B", plc_memory.get_bytes(7637, 1))[0]
            # 將每個位元轉換為布林值 (0 或 1)
            bool_values = [(unpacked_value >> i) & 1 == 1 for i in range(8)]
            present_1 = bool_values[0]
            present_2 = bool_values[1]
            present_3 = bool_values[2]
            present_4 = bool_values[3]
            present_5 = bool_values[4]
            barcode_done = bool_values[5]
            print("在席1:", present_1)  # DM7637.0
            print("在席2:", present_2)  # DM7637.1
            print("在席3:", present_3)  # DM7637.2
            print("在席4:", present_4)  # DM7637.3
            print("在席5:", present_5)  # DM7637.4
            print("BarCodeReader讀取完成:", barcode_done)  # DM7636.5

            # AGV狀態 3 DM7638 (2個byte 這裡只拿1個) B (unsinged char)
            unpacked_value = struct.unpack("<B", plc_memory.get_bytes(7638, 1))[0]
            # 將每個位元轉換為布林值 (0 或 1)
            bool_values = [(unpacked_value >> i) & 1 == 1 for i in range(8)]
            door_req = bool_values[0]
            print("開關門請求:", door_req)  # DM7638.0

            # AGV_Alarm 這裡是用1word(2byte),7*2=14  H (unsinged short)
            (alarm_1, alarm_2, alarm_3, layer, alarm_4, alarm_5, alarm_6) = (
                struct.unpack("<7H", plc_memory.get_bytes(7639, 14))
            )
            print("AGV_ALARM1", alarm_1)  # 4byte = 2 word
            print("AGV_ALARM2", alarm_2)  # 4byte = 2 word
            print("AGV_ALARM3", alarm_3)  # 4byte = 2 word
            print("Layer", layer)  # 4byte = 2 word
            print("AGV_ALARM4", alarm_4)  # 4byte = 2 word
            print("AGV_ALARM5", alarm_5)  # 4byte = 2 word
            print("AGV_ALARM6", alarm_6)  # 4byte = 2 word

            id1, magic = struct.unpack("<20s20s", plc_memory.get_bytes(7650, 40))
            print(
                "ID1", id1.decode("ascii", errors="ignore").replace("\x00", "")
            )  # get_string,長度是幾個byte(幾個字元)
            print(
                "Magic", magic.decode("ascii", errors="ignore").replace("\x00", "")
            )  # get_string,長度是幾個byte(幾個字元)

        except ValueError as e:
            print("數據轉換錯誤:", e)


if __name__ == "__main__":
    main()
