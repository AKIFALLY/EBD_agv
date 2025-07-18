from agv_base.robot_parameter_abc import RobotParameterABC
import struct


class CargoRobotParameter(RobotParameterABC):

    def __init__(self):
        # Port 屬性
        self.rack_port = 0
        self.boxin_port = 0
        self.boxout_port = 0

        # Layer 屬性
        self.layer_z_rack = 0
        self.layer_y_rack = 0
        self.layer_z_boxin = 0
        self.layer_y_boxin = 0
        self.layer_z_boxout = 0
        self.layer_y_boxout = 0

    def combine_words_to_double_word(self, low_word, high_word):
        """
        將兩個 16-bit 整數組合成一個 32-bit 字串（低位 + 高位的位元組合）
        """
        packed = struct.pack('<HH', low_word, high_word)
        double_word = struct.unpack('<I', packed)[0]
        return str(double_word)

    def calculate_parameter(self):
        # 移除不必要的分隔線輸出以減少日誌洪水

        # 確保 port 值為整數類型
        self.rack_port = int(self.rack_port) if self.rack_port is not None else 0
        self.boxin_port = int(self.boxin_port) if self.boxin_port is not None else 0
        self.boxout_port = int(self.boxout_port) if self.boxout_port is not None else 0

        # rack_port 計算邏輯
        if 1 <= self.rack_port <= 16:
            rack_row = ((self.rack_port-1) // 4)+1  # 整數部分
            rack_column = (self.rack_port-1) % 4+1  # 餘數部分
        elif 17 <= self.rack_port <= 32:
            rack_row = ((self.rack_port-17) // 4)+1  # 整數部分
            rack_column = (self.rack_port-17) % 4+1  # 餘數部分
        else:
            rack_row = 0
            rack_column = 0

        # 直接賦值 layer_z_rack 和 layer_y_rack
        self.layer_z_rack = rack_row    # 整數部分
        self.layer_y_rack = rack_column  # 餘數部分

        # boxin_port 計算邏輯
        boxin_row = ((self.boxin_port-1) // 2)+1    # 整數部分
        boxin_column = (self.boxin_port-1) % 2+1   # 餘數部分

        # 直接賦值 layer_z_boxin 和 layer_y_boxin
        self.layer_z_boxin = boxin_row    # 整數部分
        self.layer_y_boxin = boxin_column  # 餘數部分

        # boxout_port 計算邏輯
        boxout_row = ((self.boxout_port-1) // 2)+1    # 整數部分
        boxout_column = (self.boxout_port-1) % 2+1   # 餘數部分

        # 直接賦值 layer_z_boxout 和 layer_y_boxout
        self.layer_z_boxout = boxout_row    # 整數部分
        self.layer_y_boxout = boxout_column  # 餘數部分

    def values(self):
        # 計算雙字組合
        layer_rack = self.combine_words_to_double_word(self.layer_z_rack, self.layer_y_rack)
        layer_boxin = self.combine_words_to_double_word(self.layer_z_boxin, self.layer_y_boxin)
        layer_boxout = self.combine_words_to_double_word(self.layer_z_boxout, self.layer_y_boxout)

        return [
            layer_rack,
            layer_boxin,
            layer_boxout,
        ]
