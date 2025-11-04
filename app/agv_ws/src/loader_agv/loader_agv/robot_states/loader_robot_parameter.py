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

    @staticmethod
    def calculate_layer_from_port(port: int) -> tuple[int, int]:
        """
        計算端口對應的 layer_z 和 layer_y（通用公式）

        Args:
            port: 端口號 (1-4)

        Returns:
            (layer_z, layer_y)

        映射關係（2列排列）:
            port1 → (1, 2)
            port2 → (1, 2)  # 注意：根據原有邏輯
            port3 → (2, 1)
            port4 → (2, 2)

        原有計算邏輯:
            boxin_row = ((port - 1) // 2) + 1     # Z 軸層級
            boxin_column = (port - 1) % 2 + 1     # Y 軸位置
        """
        layer_z = ((port - 1) // 2) + 1     # 整數部分 + 1 = 列號
        layer_y = (port - 1) % 2 + 1        # 餘數部分 + 1 = 欄號
        return (layer_z, layer_y)

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

        # soaker_port 計算邏輯（固定值設計：不管 port 是多少，row 都是 1）
        soaker_row = 1          # 整數部分：固定為 1（W118 固定值）
        soaker_column = 0       # 餘數部分：固定為 0

        # 直接賦值 layer_z_soaker 和 layer_y_soaker
        self.layer_z_soaker = soaker_row    # W118 = 1（固定值）
        self.layer_y_soaker = soaker_column  # W119 = 0（固定值）

        # pre_dryer_port 計算邏輯（8 port 特殊處理：兩組設備共用 2x2 位置）
        # Port 1,2,5,6 → [1,1], [1,2], [2,1], [2,2]
        # Port 3,4,7,8 → [1,1], [1,2], [2,1], [2,2]

        # 定義兩組映射（每組 port 都映射到相同的 2x2 位置）
        if self.pre_dryer_port in [1, 2, 5, 6]:
            # 第一組：port 1,2,5,6 的映射
            port_mapping = {1: 0, 2: 1, 5: 2, 6: 3}
            port_index = port_mapping[self.pre_dryer_port]
        elif self.pre_dryer_port in [3, 4, 7, 8]:
            # 第二組：port 3,4,7,8 的映射（相同的位置）
            port_mapping = {3: 0, 4: 1, 7: 2, 8: 3}
            port_index = port_mapping[self.pre_dryer_port]
        else:
            # 容錯處理
            port_index = 0

        # 從 index 計算 row 和 column (0→[1,1], 1→[1,2], 2→[2,1], 3→[2,2])
        pre_dryer_row = (port_index // 2) + 1      # 0,1→1 ; 2,3→2
        pre_dryer_column = (port_index % 2) + 1    # 0,2→1 ; 1,3→2

        # 直接賦值 layer_z_pre_dryer 和 layer_y_pre_dryer
        self.layer_z_pre_dryer = pre_dryer_row     # W11A = 1 或 2
        self.layer_y_pre_dryer = pre_dryer_column  # W11B = 1 或 2

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
