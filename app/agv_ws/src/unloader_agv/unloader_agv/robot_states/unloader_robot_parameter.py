from agv_base.robot_parameter_abc import RobotParameterABC
import struct


class UnloaderRobotParameter(RobotParameterABC):

    def __init__(self):   # 計算哪一個RACK PORT
        # Port 屬性 (Unloader AGV 特有的 ports)
        self.unloader_agv_port_back = 0
        self.boxout_port = 0
        self.pre_dryer_port = 0
        self.oven_port = 0

        # Quantity 屬性
        self.take_quantity = 0

        # Layer 屬性
        self.layer_z_back = 0
        self.layer_y_back = 0
        self.layer_z_boxout = 0
        self.layer_y_boxout = 0
        self.layer_z_pre_dryer = 0
        self.layer_y_pre_dryer = 0
        self.layer_z_oven = 0
        self.layer_y_oven = 0

    def combine_words_to_double_word(self, low_word, high_word):
        """
        將兩個 16-bit 整數組合成一個 32-bit 字串（低位 + 高位的位元組合）
        """
        packed = struct.pack('<HH', low_word, high_word)
        double_word = struct.unpack('<I', packed)[0]
        return str(double_word)

    def calculate_parameter(self):
        # 移除調試用分隔線輸出以減少日誌洪水
        # print("❌❌❌❌❌❌❌❌❌✅✅✅✅✅✅✅✅✅✅✅✅✅✅✅✅✅✅✅✅✅✅✅❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌")

        # 確保 port 值為整數類型
        self.unloader_agv_port_back = int(
            self.unloader_agv_port_back) if self.unloader_agv_port_back is not None else 0
        self.boxout_port = int(self.boxout_port) if self.boxout_port is not None else 0
        self.pre_dryer_port = int(self.pre_dryer_port) if self.pre_dryer_port is not None else 0
        self.oven_port = int(self.oven_port) if self.oven_port is not None else 0
        self.take_quantity = int(self.take_quantity) if self.take_quantity is not None else 0

        # unloader_agv_port_back 計算邏輯 (使用與 boxout_port 相同的計算模式)
        back_row = ((self.unloader_agv_port_back-1) // 2)+1    # 整數部分
        back_column = 0  # 餘數部分

        # 直接賦值 layer_z_back 和 layer_y_back
        self.layer_z_back = back_row    # 整數部分
        self.layer_y_back = back_column  # 餘數部分

        # boxout_port 計算邏輯
        boxout_row = ((self.boxout_port-1) // 2)+1    # 整數部分
        boxout_column = 0   # 餘數部分

        # 直接賦值 layer_z_boxout 和 layer_y_boxout
        self.layer_z_boxout = boxout_row    # 整數部分
        self.layer_y_boxout = boxout_column  # 餘數部分

        # pre_dryer_port 計算邏輯 (修改為新的計算模式)
        # port 1-4 → row=1, port 5-8 → row=2, column 固定為 0
        pre_dryer_row = 1 if self.pre_dryer_port <= 4 else 2    # 根據 port 範圍決定 row
        pre_dryer_column = 0   # 所有 port 的 column 都是 0

        # 直接賦值 layer_z_pre_dryer 和 layer_y_pre_dryer
        self.layer_z_pre_dryer = pre_dryer_row    # 整數部分
        self.layer_y_pre_dryer = pre_dryer_column  # 餘數部分

        # oven_port 計算邏輯 (端口到行列的對應關係)
        # 端口 1,2 → row=1, column=1
        # 端口 3,4 → row=1, column=2
        # 端口 5,6 → row=2, column=1
        # 端口 7,8 → row=2, column=2
        oven_row = ((self.oven_port-1) // 4) + 1    # 行計算
        oven_column = ((self.oven_port-1) // 2) % 2 + 1   # 列計算

        # 直接賦值 layer_z_oven 和 layer_y_oven
        self.layer_z_oven = oven_row    # 整數部分
        self.layer_y_oven = oven_column  # 餘數部分

        # take_quantity 計算邏輯 (直接使用 get_take_quantity)
        self.layer_take_quantity = self.take_quantity

    def values(self):
        # 計算雙字組合
        layer_back = self.combine_words_to_double_word(self.layer_z_back, self.layer_y_back)
        layer_boxout = self.combine_words_to_double_word(self.layer_z_boxout, self.layer_y_boxout)
        layer_pre_dryer = self.combine_words_to_double_word(
            self.layer_z_pre_dryer, self.layer_y_pre_dryer)
        layer_oven = self.combine_words_to_double_word(self.layer_z_oven, self.layer_y_oven)
        layer_quantity = self.combine_words_to_double_word(self.layer_take_quantity, 0)

        return [
            layer_back,
            layer_boxout,
            layer_pre_dryer,
            layer_oven,      # 新增烤箱參數
            layer_quantity,  # 新增 take_quantity 參數
        ]
