# 常數定義
# pc->plc 時可\r或\r\n , plc回應時為 \r\n ,統一使用 \r\n
PLC_END_MARKER = "\r\n"


class KeyencePlcCommand:
    @staticmethod
    def model():
        """查詢機型 指令"""
        return f"?K{PLC_END_MARKER}"

    @staticmethod
    def get_run_mode():
        """查詢運行模式 指令"""
        return f"?M{PLC_END_MARKER}"

    @staticmethod
    def force_on(device_type, device_number):
        """ForceOn 指令"""
        return f"ST {device_type}{device_number}{PLC_END_MARKER}"

    @staticmethod
    def force_off(device_type, device_number):
        """ForceOff 指令"""
        return f"RS {device_type}{device_number}{PLC_END_MARKER}"

    @staticmethod
    def read_data(device_type, device_number):
        """讀取PLC資料指令"""
        return f"RD {device_type}{device_number}{PLC_END_MARKER}"

    @staticmethod
    def write_data(device_type, device_number, write_data):
        """寫入PLC資料指令"""
        return f"WR {device_type}{device_number} {write_data}{PLC_END_MARKER}"

    @staticmethod
    def read_continuous_data(device_type, device_number, device_length):
        """連續讀取PLC資料指令"""
        return f"RDS {device_type}{device_number} {device_length}{PLC_END_MARKER}"

    @staticmethod
    def write_continuous_data(device_type, device_number, write_data):
        """連續寫入PLC資料指令"""
        data_str = " ".join(str(x) for x in write_data)
        return f"WRS {device_type}{device_number} {len(write_data)} {data_str}{PLC_END_MARKER}"
