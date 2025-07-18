from agv_base.robot_parameter_abc import RobotParameterABC
import struct


class LoaderRobotParameter(RobotParameterABC):

    def __init__(self):   # 計算哪一個RACK PORT
        # Port 屬性
        self.loader_agv_port_front = 0
        self.loader_agv_port_side = 0
        self.boxin_port = 0
        self.soaker_port = 0
        self.cleaner_port = 0
        self.pre_dryer_port = 0

        # Layer 屬性
        self.layer_z_side = 0
        self.layer_y_side = 0
        self.layer_z_front = 0
        self.layer_y_front = 0
        self.layer_z_boxin = 0
        self.layer_y_boxin = 0
        self.layer_z_cleaner = 0
        self.layer_y_cleaner = 0
        self.layer_z_soaker = 0
        self.layer_y_soaker = 0
        self.layer_z_pre_dryer = 0
        self.layer_y_pre_dryer = 0

    def combine_words_to_double_word(self, low_word, high_word):
        """
        將兩個 16-bit 整數組合成一個 32-bit 字串（低位 + 高位的位元組合）
        """
        packed = struct.pack('<HH', low_word, high_word)
        double_word = struct.unpack('<I', packed)[0]
        return str(double_word)

    def calculate_parameter(self):
        print("❌❌❌❌❌❌❌❌❌✅✅✅✅✅✅✅✅✅✅✅✅✅✅✅✅✅✅✅✅✅✅✅❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌")

        # 確保 port 值為整數類型
        self.loader_agv_port_front = int(
            self.loader_agv_port_front) if self.loader_agv_port_front is not None else 0
        self.loader_agv_port_side = int(
            self.loader_agv_port_side) if self.loader_agv_port_side is not None else 0
        self.boxin_port = int(self.boxin_port) if self.boxin_port is not None else 0
        self.soaker_port = int(self.soaker_port) if self.soaker_port is not None else 0
        self.cleaner_port = int(self.cleaner_port) if self.cleaner_port is not None else 0
        self.pre_dryer_port = int(self.pre_dryer_port) if self.pre_dryer_port is not None else 0

        # loader_agv_port_side 計算邏輯 (使用與 boxin_port 相同的計算模式)
        side_row = self.loader_agv_port_side    # 整數部分
        side_column = 0  # 餘數部分

        # 直接賦值 layer_z_side 和 layer_y_side
        self.layer_z_side = side_row    # 整數部分
        self.layer_y_side = side_column  # 餘數部分

        # loader_agv_port_front 計算邏輯 (使用與 boxin_port 相同的計算模式)
        front_row = self.loader_agv_port_front    # 整數部分
        front_column = 0  # 餘數部分

        # 直接賦值 layer_z_front 和 layer_y_front
        self.layer_z_front = front_row    # 整數部分
        self.layer_y_front = front_column  # 餘數部分

        # boxin_port 計算邏輯
        boxin_row = ((self.boxin_port-1) // 2)+1    # 整數部分
        boxin_column = (self.boxin_port-1) % 2+1   # 餘數部分

        # 直接賦值 layer_z_boxin 和 layer_y_boxin
        self.layer_z_boxin = boxin_row    # 整數部分
        self.layer_y_boxin = boxin_column  # 餘數部分

        # cleaner_port 計算邏輯 (使用與 boxin_port 相同的計算模式)
        cleaner_row = ((self.cleaner_port-1) // 2)+1    # 整數部分
        cleaner_column = (self.cleaner_port-1) % 2+1   # 餘數部分

        # 直接賦值 layer_z_cleaner 和 layer_y_cleaner
        self.layer_z_cleaner = cleaner_row    # 整數部分
        self.layer_y_cleaner = cleaner_column  # 餘數部分

        # soaker_port 計算邏輯 (使用與 boxin_port 相同的計算模式)
        soaker_row = self.soaker_port   # 整數部分
        soaker_column = 0   # 餘數部分

        # 直接賦值 layer_z_soaker 和 layer_y_soaker
        self.layer_z_soaker = soaker_row    # 整數部分
        self.layer_y_soaker = soaker_column  # 餘數部分

        # pre_dryer_port 計算邏輯 (使用與 boxin_port 相同的計算模式)
        pre_dryer_row = ((self.pre_dryer_port-1) // 4)+1    # 整數部分
        pre_dryer_column = (self.pre_dryer_port-1) % 4+1   # 餘數部分

        # 直接賦值 layer_z_pre_dryer 和 layer_y_pre_dryer
        self.layer_z_pre_dryer = pre_dryer_row    # 整數部分
        self.layer_y_pre_dryer = pre_dryer_column  # 餘數部分

    def values(self):
        # 計算雙字組合
        layer_side = self.combine_words_to_double_word(self.layer_z_side, self.layer_y_side)
        layer_front = self.combine_words_to_double_word(self.layer_z_front, self.layer_y_front)
        layer_boxin = self.combine_words_to_double_word(self.layer_z_boxin, self.layer_y_boxin)
        layer_cleaner = self.combine_words_to_double_word(
            self.layer_z_cleaner, self.layer_y_cleaner)
        layer_soaker = self.combine_words_to_double_word(self.layer_z_soaker, self.layer_y_soaker)
        layer_pre_dryer = self.combine_words_to_double_word(
            self.layer_z_pre_dryer, self.layer_y_pre_dryer)

        return [
            layer_side,
            layer_front,
            layer_boxin,
            layer_cleaner,
            layer_soaker,
            layer_pre_dryer,
        ]
