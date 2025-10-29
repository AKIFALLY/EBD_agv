"""
Equipment Station Configuration and Mapping

提供設備 Station 與 Port 的映射關係，支援 Work ID 解析。

設備分類：
- 標準設備：1 station = 2 ports（預烘機、清洗機、烤箱、傳送箱）
- 特殊設備：1 station = 1 port（泡藥機）

Work ID 編碼格式：
[room_id] + [equipment_type] + [station] + [action_type]

範例：
- 2060502: room_id=2, equipment_type=06(烤箱), station=05, action_type=02(PUT)
"""

from typing import List, Tuple, Dict


class EquipmentStations:
    """設備 Station 配置和映射系統"""

    # ========== 設備分類定義 ==========

    # 標準設備：1 station = 2 ports (奇數 station: 1, 3, 5, 7)
    STANDARD_EQUIPMENT = {
        203: {
            "name": "清洗機",
            "name_en": "Cleaner",
            "stations": [1, 3],  # Station 01, 03
            "ports_per_station": 2,
        },
        205: {
            "name": "預烘機",
            "name_en": "Pre-dryer",
            "stations": [1, 3, 5, 7],  # Station 01, 03, 05, 07
            "ports_per_station": 2,
        },
        206: {
            "name": "烤箱",
            "name_en": "Oven",
            "stations": [1, 3, 5, 7],  # Station 01, 03, 05, 07
            "ports_per_station": 2,
        },
        201: {
            "name": "入口傳送箱",
            "name_en": "Box-in Transfer",
            "stations": [1, 3],  # Station 01, 03
            "ports_per_station": 2,
        },
        202: {
            "name": "出口傳送箱",
            "name_en": "Box-out Transfer",
            "stations": [1, 3],  # Station 01, 03
            "ports_per_station": 2,
        },
    }

    # 特殊設備：1 station = 1 port
    SPECIAL_EQUIPMENT = {
        204: {
            "name": "泡藥機",
            "name_en": "Soaker",
            "stations": [1, 2, 3, 4, 5, 6],  # Station 01-06
            "ports_per_station": 1,
        },
    }

    # UnloaderAGV 自定義映射：1 station = 4 ports（批量處理模式）
    # 說明：UnloaderAGV 採用批量操作，一次處理 4 個 port，與 LoaderAGV 的單格操作不同
    # 預烘機：UnloaderAGV 只使用 Station 01/03（批量 4 格）
    # 烤箱：UnloaderAGV 使用 Station 01（上排/只拿）和 Station 05（下排/只放）
    # 出口傳送箱：UnloaderAGV 只使用 Station 01（批量 4 格）
    UNLOADER_CUSTOM_MAPPING = {
        202: {  # 出口傳送箱
            "name": "出口傳送箱",
            "mappings": {
                1: [1, 2, 3, 4],  # Station 01 → Port 1-2-3-4 (批量4格) - Work ID 2020102
            },
        },
        205: {  # 預烘機
            "name": "預烘機",
            "mappings": {
                1: [1, 2, 5, 6],  # Station 01 → Port 1-2-5-6 (批量4格) - Work ID 2050101
                3: [3, 4, 7, 8],  # Station 03 → Port 3-4-7-8 (批量4格) - Work ID 2050301
            },
        },
        206: {  # 烤箱
            "name": "烤箱",
            "mappings": {
                1: [1, 2, 3, 4],  # Station 01 → Port 1-2-3-4 (批量4格/上排/只拿) - Work ID 2060101
                5: [5, 6, 7, 8],  # Station 05 → Port 5-6-7-8 (批量4格/下排/只放) - Work ID 2060502
            },
        },
    }

    # ========== Station to Port 映射 ==========

    @staticmethod
    def station_to_ports(eqp_id: int, station: int, agv_type: str = "loader") -> List[int]:
        """
        將 station 轉換為 port 列表

        Args:
            eqp_id: 設備 ID (例如: 206=烤箱, 205=預烘機, 204=泡藥機)
            station: Station 編號 (例如: 1, 3, 5, 7)
            agv_type: AGV 類型 ("loader" 或 "unloader")，預設為 "loader"

        Returns:
            Port ID 列表
            - 標準設備 (LoaderAGV): [station, station+1] (例如: Station 5 → [5, 6])
            - 特殊設備: [station] (例如: Station 3 → [3])
            - UnloaderAGV 自定義: [1,2,5,6] 或 [1,2,3,4] 等 (批量4格)

        Raises:
            ValueError: 當 eqp_id 未知或 station 無效時

        範例：
            >>> EquipmentStations.station_to_ports(206, 5)  # 烤箱 Station 05 (LoaderAGV)
            [5, 6]
            >>> EquipmentStations.station_to_ports(206, 1, agv_type="unloader")  # 烤箱 Station 01 (UnloaderAGV)
            [1, 2, 3, 4]
            >>> EquipmentStations.station_to_ports(204, 3)  # 泡藥機 Station 03
            [3]
        """
        # 優先檢查 UnloaderAGV 自定義映射
        if agv_type == "unloader" and eqp_id in EquipmentStations.UNLOADER_CUSTOM_MAPPING:
            custom_config = EquipmentStations.UNLOADER_CUSTOM_MAPPING[eqp_id]
            if station in custom_config["mappings"]:
                return custom_config["mappings"][station]
            # 如果 station 不在自定義映射中，回退到標準邏輯

        if eqp_id in EquipmentStations.SPECIAL_EQUIPMENT:
            # 特殊設備（泡藥機）：1 station = 1 port
            equipment_info = EquipmentStations.SPECIAL_EQUIPMENT[eqp_id]
            if station not in equipment_info["stations"]:
                raise ValueError(
                    f"Invalid station {station} for {equipment_info['name']} (eqp_id={eqp_id}). "
                    f"Valid stations: {equipment_info['stations']}"
                )
            return [station]

        elif eqp_id in EquipmentStations.STANDARD_EQUIPMENT:
            # 標準設備：1 station = 2 ports
            equipment_info = EquipmentStations.STANDARD_EQUIPMENT[eqp_id]

            # Station 必須是奇數 (1, 3, 5, 7)
            if station % 2 == 0:
                raise ValueError(
                    f"Standard equipment requires odd station number (1, 3, 5, 7). "
                    f"Got: {station} for {equipment_info['name']} (eqp_id={eqp_id})"
                )

            if station not in equipment_info["stations"]:
                raise ValueError(
                    f"Invalid station {station} for {equipment_info['name']} (eqp_id={eqp_id}). "
                    f"Valid stations: {equipment_info['stations']}"
                )

            return [station, station + 1]

        else:
            raise ValueError(
                f"Unknown equipment ID: {eqp_id}. "
                f"Valid IDs: {list(EquipmentStations.STANDARD_EQUIPMENT.keys()) + list(EquipmentStations.SPECIAL_EQUIPMENT.keys())}"
            )

    # ========== Work ID 解析 ==========

    @staticmethod
    def extract_station_from_work_id(work_id: int) -> Tuple[int, int, int, int]:
        """
        從 work_id 中提取 station 資訊

        Work ID 格式: [room_id] + [equipment_type] + [station] + [action_type]
        7 位數字: REESSAA
        - R: room_id (1位)
        - EE: equipment_type (2位)
        - SS: station (2位)
        - AA: action_type (2位，01=TAKE, 02=PUT)

        Args:
            work_id: 7位數的 Work ID (例如: 2060502)

        Returns:
            Tuple[room_id, eqp_id, station, action_type]
            - room_id: 房間 ID
            - eqp_id: 設備 ID (room_id * 100 + equipment_type)
            - station: Station 編號
            - action_type: 動作類型 (01=TAKE, 02=PUT)

        Raises:
            ValueError: 當 work_id 格式不正確時

        範例：
            >>> EquipmentStations.extract_station_from_work_id(2060502)
            (2, 206, 5, 2)
            # room_id=2, eqp_id=206(烤箱), station=5, action_type=2(PUT)

            >>> EquipmentStations.extract_station_from_work_id(2040301)
            (2, 204, 3, 1)
            # room_id=2, eqp_id=204(泡藥機), station=3, action_type=1(TAKE)
        """
        work_id_str = str(work_id)

        # 驗證格式：必須是 7 位數
        if len(work_id_str) != 7:
            raise ValueError(
                f"Invalid work_id format: {work_id}. "
                f"Expected 7-digit format: [room_id][equipment_type][station][action_type]"
            )

        # 解析各個部分
        room_id = int(work_id_str[0])  # 第 1 位
        equipment_type = int(work_id_str[1:3])  # 第 2-3 位
        station = int(work_id_str[3:5])  # 第 4-5 位
        action_type = int(work_id_str[5:7])  # 第 6-7 位

        # 計算 eqp_id
        eqp_id = room_id * 100 + equipment_type

        return room_id, eqp_id, station, action_type

    @staticmethod
    def work_id_to_ports(work_id: int, agv_type: str = "loader") -> Tuple[int, int, List[int], int]:
        """
        從 work_id 直接解析出對應的 ports 列表

        這是一個便利函數，結合了 extract_station_from_work_id 和 station_to_ports

        Args:
            work_id: 7位數的 Work ID
            agv_type: AGV 類型 ("loader" 或 "unloader")，預設為 "loader"

        Returns:
            Tuple[room_id, eqp_id, ports, action_type]
            - room_id: 房間 ID
            - eqp_id: 設備 ID
            - ports: Port 列表
            - action_type: 動作類型

        範例：
            >>> EquipmentStations.work_id_to_ports(2060502)
            (2, 206, [5, 6], 2)
            # LoaderAGV 烤箱 Station 05 → Port 5-6, PUT 動作

            >>> EquipmentStations.work_id_to_ports(2060501, agv_type="unloader")
            (2, 206, [5, 6, 7, 8], 1)
            # UnloaderAGV 烤箱 Station 05 → Port 5-6-7-8, TAKE 動作

            >>> EquipmentStations.work_id_to_ports(2040301)
            (2, 204, [3], 1)
            # 泡藥機 Station 03 → Port 3, TAKE 動作
        """
        room_id, eqp_id, station, action_type = EquipmentStations.extract_station_from_work_id(
            work_id
        )
        ports = EquipmentStations.station_to_ports(eqp_id, station, agv_type)
        return room_id, eqp_id, ports, action_type

    # ========== 工具函數 ==========

    @staticmethod
    def get_equipment_name(eqp_id: int, language: str = "zh") -> str:
        """
        取得設備名稱

        Args:
            eqp_id: 設備 ID
            language: 語言 ('zh'=中文, 'en'=英文)

        Returns:
            設備名稱

        範例：
            >>> EquipmentStations.get_equipment_name(206, 'zh')
            '烤箱'
            >>> EquipmentStations.get_equipment_name(206, 'en')
            'Oven'
        """
        all_equipment = {
            **EquipmentStations.STANDARD_EQUIPMENT,
            **EquipmentStations.SPECIAL_EQUIPMENT,
        }

        if eqp_id not in all_equipment:
            return f"Unknown Equipment (ID: {eqp_id})"

        name_key = "name" if language == "zh" else "name_en"
        return all_equipment[eqp_id].get(name_key, "Unknown")

    @staticmethod
    def is_standard_equipment(eqp_id: int) -> bool:
        """
        檢查是否為標準設備（1 station = 2 ports）

        Args:
            eqp_id: 設備 ID

        Returns:
            True 表示標準設備，False 表示特殊設備
        """
        return eqp_id in EquipmentStations.STANDARD_EQUIPMENT

    @staticmethod
    def is_special_equipment(eqp_id: int) -> bool:
        """
        檢查是否為特殊設備（1 station = 1 port）

        Args:
            eqp_id: 設備 ID

        Returns:
            True 表示特殊設備，False 表示標準設備
        """
        return eqp_id in EquipmentStations.SPECIAL_EQUIPMENT

    @staticmethod
    def get_valid_stations(eqp_id: int) -> List[int]:
        """
        取得設備的有效 station 列表

        Args:
            eqp_id: 設備 ID

        Returns:
            有效的 station 列表

        範例：
            >>> EquipmentStations.get_valid_stations(206)  # 烤箱
            [1, 3, 5, 7]
            >>> EquipmentStations.get_valid_stations(204)  # 泡藥機
            [1, 2, 3, 4, 5, 6]
        """
        all_equipment = {
            **EquipmentStations.STANDARD_EQUIPMENT,
            **EquipmentStations.SPECIAL_EQUIPMENT,
        }

        if eqp_id not in all_equipment:
            raise ValueError(f"Unknown equipment ID: {eqp_id}")

        return all_equipment[eqp_id]["stations"]

    @staticmethod
    def calculate_eqp_port_ids(
        room_id: int, eqp_id: int, ports: List[int]
    ) -> List[int]:
        """
        計算實際的 EqpPort ID 列表

        EqpPort ID 格式: [room_id][equipment_type][port]
        例如: room_id=2, equipment_type=06, port=5 → EqpPort ID = 2065

        Args:
            room_id: 房間 ID
            eqp_id: 設備 ID (例如: 206)
            ports: Port 列表 (例如: [5, 6])

        Returns:
            EqpPort ID 列表 (例如: [2065, 2066])

        範例：
            >>> EquipmentStations.calculate_eqp_port_ids(2, 206, [5, 6])
            [2065, 2066]
        """
        # 取得設備類型（eqp_id 的後兩位）
        equipment_type = eqp_id % 100

        # 計算 base port address
        base_port_id = room_id * 1000 + equipment_type * 10

        # 計算每個 port 的實際 ID
        eqp_port_ids = [base_port_id + port for port in ports]

        return eqp_port_ids


# ========== 便利函數 ==========


def station_to_ports(eqp_id: int, station: int, agv_type: str = "loader") -> List[int]:
    """
    便利函數：將 station 轉換為 port 列表

    See Also:
        EquipmentStations.station_to_ports
    """
    return EquipmentStations.station_to_ports(eqp_id, station, agv_type)


def extract_station_from_work_id(work_id: int) -> Tuple[int, int, int, int]:
    """
    便利函數：從 work_id 中提取 station 資訊

    See Also:
        EquipmentStations.extract_station_from_work_id
    """
    return EquipmentStations.extract_station_from_work_id(work_id)


def work_id_to_ports(work_id: int, agv_type: str = "loader") -> Tuple[int, int, List[int], int]:
    """
    便利函數：從 work_id 直接解析出對應的 ports 列表

    See Also:
        EquipmentStations.work_id_to_ports
    """
    return EquipmentStations.work_id_to_ports(work_id, agv_type)
