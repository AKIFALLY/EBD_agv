"""
12. 設備初始化資料
依賴：位置
"""

from db_proxy.models import Eqp, EqpPort, EqpSignal
from ..db_install import insert_data_if_not_exists_name, insert_data_if_not_exists_name_and_not_exists_id


def initialize_equipment(session):
    """初始化設備資料"""
    print("🏭 初始化設備資料...")

    # 設備資料
    default_eqps = [
        # 預建置測試資料
        {"id": 210, "location_id": 21000, "name": "LoaderAGV",
         "description": "LoaderAGV"},
        {"id": 211, "location_id": 21100, "name": "UnloaderAGV",
         "description": "UnloaderAGV"},



        {"id": 201, "location_id": 20100, "name": "Room2_BoxIn",
         "description": "房間2 入口傳送箱"},
        {"id": 202, "location_id": 20200, "name": "Room2_BoxOut",
         "description": "房間2 出口傳送箱"},
        {"id": 203, "location_id": 20300, "name": "Room2_Cleaner",
         "description": "房間2 清洗機"},
        {"id": 204, "location_id": 20400, "name": "Room2_Soaking",
         "description": "房間2 泡藥機 群組"},
        {"id": 205, "location_id": 20500, "name": "Room2_Dryer",
         "description": "房間2 預烘機"},
        {"id": 206, "location_id": 20600, "name": "Room2_Oven",
         "description": "房間2 烤箱"},
    ]

    insert_data_if_not_exists_name(session, default_eqps, Eqp)
    print("✅ 設備資料初始化完成")


def initialize_equipment_ports(session):
    """初始化設備端口資料"""
    print("🔌 初始化設備端口資料...")

    default_eqp_port = [

        # LoaderAGV Ports
        {"id": 2101, "eqp_id": 210, "name": "LoaderAGV_Port01",
         "description": "LoaderAGV Port01"},
        {"id": 2102, "eqp_id": 210, "name": "LoaderAGV_Port02",
         "description": "LoaderAGV Port02"},
        {"id": 2103, "eqp_id": 210, "name": "LoaderAGV_Port03",
         "description": "LoaderAGV Port03"},
        {"id": 2104, "eqp_id": 210, "name": "LoaderAGV_Port04",
         "description": "LoaderAGV Port04"},

        # UnloaderAGV Ports
        {"id": 2111, "eqp_id": 211, "name": "UnloaderAGV_Port01",
         "description": "UnloaderAGV Port01"},
        {"id": 2112, "eqp_id": 211, "name": "UnloaderAGV_Port02",
         "description": "UnloaderAGV Port02"},
        {"id": 2113, "eqp_id": 211, "name": "UnloaderAGV_Port03",
         "description": "UnloaderAGV Port03"},
        {"id": 2114, "eqp_id": 211, "name": "UnloaderAGV_Port04",
         "description": "UnloaderAGV Port04"},

        # 房間2 入口 傳送箱 Ports
        {"id": 2011, "eqp_id": 201, "name": "Room2_BoxIn_Port01",
         "description": "房間2 入口傳送箱 Port01"},
        {"id": 2012, "eqp_id": 201, "name": "Room2_BoxIn_Port02",
         "description": "房間2 入口傳送箱 Port02"},
        {"id": 2013, "eqp_id": 201, "name": "Room2_BoxIn_Port03",
         "description": "房間2 入口傳送箱 Port03"},
        {"id": 2014, "eqp_id": 201, "name": "Room2_BoxIn_Port04",
         "description": "房間2 入口傳送箱 Port04"},

        # 房間2 出口 傳送箱 Ports
        {"id": 2021, "eqp_id": 202, "name": "Room2_BoxOut_Port01",
         "description": "房間2 出口傳送箱 Port01"},
        {"id": 2022, "eqp_id": 202, "name": "Room2_BoxOut_Port02",
         "description": "房間2 出口傳送箱 Port02"},
        {"id": 2023, "eqp_id": 202, "name": "Room2_BoxOut_Port03",
         "description": "房間2 出口傳送箱 Port03"},
        {"id": 2024, "eqp_id": 202, "name": "Room2_BoxOut_Port04",
         "description": "房間2 出口傳送箱 Port04"},

        # 房間2 清洗機 Ports
        {"id": 2031, "eqp_id": 203, "name": "Room2_Cleaner_Port01",
         "description": "房間2 清洗機 Port01"},
        {"id": 2032, "eqp_id": 203, "name": "Room2_Cleaner_Port02",
         "description": "房間2 清洗機 Port02"},
        {"id": 2033, "eqp_id": 203, "name": "Room2_Cleaner_Port03",
         "description": "房間2 清洗機 Port03"},
        {"id": 2034, "eqp_id": 203, "name": "Room2_Cleaner_Port04",
         "description": "房間2 清洗機 Port04"},
        {"id": 2035, "eqp_id": 203, "name": "Room2_Cleaner_Inner",
         "description": "房間2 清洗機 內部"},  # 被清洗機收進去的話carrier資料要被移到這個位置

        # 房間2 泡樂機(群組) Ports
        {"id": 2041, "eqp_id": 204, "name": "Room2_Soaking_1",
         "description": "房間2 泡樂機 1 Port"},
        {"id": 2042, "eqp_id": 204, "name": "Room2_Soaking_2",
         "description": "房間2 泡樂機 2 Port"},
        {"id": 2043, "eqp_id": 204, "name": "Room2_Soaking_3",
         "description": "房間2 泡樂機 3 Port"},
        {"id": 2044, "eqp_id": 204, "name": "Room2_Soaking_4",
         "description": "房間2 泡樂機 4 Port"},
        {"id": 2045, "eqp_id": 204, "name": "Room2_Soaking_5",
         "description": "房間2 泡樂機 5 Port"},
        {"id": 2046, "eqp_id": 204, "name": "Room2_Soaking_6",
         "description": "房間2 泡樂機 6 Port"},

        # 房間2 預烘機 Ports
        {"id": 2051, "eqp_id": 205, "name": "Room2_Dryer_Port01",
         "description": "房間2 預烘機 Port01"},
        {"id": 2052, "eqp_id": 205, "name": "Room2_Dryer_Port02",
         "description": "房間2 預烘機 Port02"},
        {"id": 2053, "eqp_id": 205, "name": "Room2_Dryer_Port03",
         "description": "房間2 預烘機 Port03"},
        {"id": 2054, "eqp_id": 205, "name": "Room2_Dryer_Port04",
         "description": "房間2 預烘機 Port04"},
        {"id": 2055, "eqp_id": 205, "name": "Room2_Dryer_Port05",
         "description": "房間2 預烘機 Port05"},
        {"id": 2056, "eqp_id": 205, "name": "Room2_Dryer_Port06",
         "description": "房間2 預烘機 Port06"},
        {"id": 2057, "eqp_id": 205, "name": "Room2_Dryer_Port07",
         "description": "房間2 預烘機 Port07"},
        {"id": 2058, "eqp_id": 205, "name": "Room2_Dryer_Port08",
         "description": "房間2 預烘機 Port08"},

        # 房間2 烤箱 Ports
        {"id": 2061, "eqp_id": 206, "name": "Room2_Oven_Port01",
         "description": "房間2 烤箱 Port01"},
        {"id": 2062, "eqp_id": 206, "name": "Room2_Oven_Port02",
         "description": "房間2 烤箱 Port02"},
        {"id": 2063, "eqp_id": 206, "name": "Room2_Oven_Port03",
         "description": "房間2 烤箱 Port03"},
        {"id": 2064, "eqp_id": 206, "name": "Room2_Oven_Port04",
         "description": "房間2 烤箱 Port04"},
        {"id": 2065, "eqp_id": 206, "name": "Room2_Oven_Port05",
         "description": "房間2 烤箱 Port05"},
        {"id": 2066, "eqp_id": 206, "name": "Room2_Oven_Port06",
         "description": "房間2 烤箱 Port06"},
        {"id": 2067, "eqp_id": 206, "name": "Room2_Oven_Port07",
         "description": "房間2 烤箱 Port07"},
        {"id": 2068, "eqp_id": 206, "name": "Room2_Oven_Port08",
         "description": "房間2 烤箱 Port08"},
        {"id": 2069, "eqp_id": 206, "name": "Room2_Oven_Inner",
         "description": "房間2 烤箱 內部"},

    ]

    insert_data_if_not_exists_name(session, default_eqp_port, EqpPort)
    print("✅ 設備端口資料初始化完成")


def initialize_equipment_signals(session):
    """初始化設備信號資料"""
    print("📡 初始化設備信號資料...")

    default_eqp_signals = [
        # LoaderAGV Ports 訊號
        {"id": 21001, "eqp_id": 210, "eqp_port_id": 2101, "name": "LoaderAGV_Port01_Presence",
         "description": "LoaderAGV Port01 在席訊號", "value": "0", "type_of_value": "bool", "dm_address": "12100.0"},
        {"id": 21002, "eqp_id": 210, "eqp_port_id": 2102, "name": "LoaderAGV_Port02_Presence",
         "description": "LoaderAGV Port02 在席訊號", "value": "0", "type_of_value": "bool", "dm_address": "12100.1"},
        {"id": 21003, "eqp_id": 210, "eqp_port_id": 2103, "name": "LoaderAGV_Port03_Presence",
         "description": "LoaderAGV Port03 在席訊號", "value": "0", "type_of_value": "bool", "dm_address": "12100.2"},
        {"id": 21004, "eqp_id": 210, "eqp_port_id": 2104, "name": "LoaderAGV_Port04_Presence",
         "description": "LoaderAGV Port04 在席訊號", "value": "0", "type_of_value": "bool", "dm_address": "12100.3"},

        # UnloaderAGV Ports 訊號
        {"id": 21101, "eqp_id": 211, "eqp_port_id": 2111, "name": "UnloaderAGV_Port01_Presence",
         "description": "UnloaderAGV Port01 在席訊號", "value": "0", "type_of_value": "bool", "dm_address": "12110.0"},
        {"id": 21102, "eqp_id": 211, "eqp_port_id": 2112, "name": "UnloaderAGV_Port02_Presence",
         "description": "UnloaderAGV Port02 在席訊號", "value": "0", "type_of_value": "bool", "dm_address": "12110.1"},
        {"id": 21103, "eqp_id": 211, "eqp_port_id": 2113, "name": "UnloaderAGV_Port03_Presence",
         "description": "UnloaderAGV Port03 在席訊號", "value": "0", "type_of_value": "bool", "dm_address": "12110.2"},
        {"id": 21104, "eqp_id": 211, "eqp_port_id": 2114, "name": "UnloaderAGV_Port04_Presence",
         "description": "UnloaderAGV Port04 在席訊號", "value": "0", "type_of_value": "bool", "dm_address": "12110.3"},

        # 房間 2 入口傳送箱 訊號
        {"id": 20101, "eqp_id": 201, "eqp_port_id": 2011, "name": "Room2_BoxIn_Port01_Presence",
         "description": "房間2 入口傳送箱 Port01 在席訊號", "value": "0", "type_of_value": "bool", "dm_address": "12000.0"},
        {"id": 20102, "eqp_id": 201, "eqp_port_id": 2012, "name": "Room2_BoxIn_Port02_Presence",
         "description": "房間2 入口傳送箱 Port02 在席訊號", "value": "0", "type_of_value": "bool", "dm_address": "12000.1"},
        {"id": 20103, "eqp_id": 201, "eqp_port_id": 2013, "name": "Room2_BoxIn_Port03_Presence",
         "description": "房間2 入口傳送箱 Port03 在席訊號", "value": "0", "type_of_value": "bool", "dm_address": "12000.2"},
        {"id": 20104, "eqp_id": 201, "eqp_port_id": 2014, "name": "Room2_BoxIn_Port04_Presence",
         "description": "房間2 入口傳送箱 Port04 在席訊號", "value": "0", "type_of_value": "bool", "dm_address": "12000.3"},
        {"id": 20105, "eqp_id": 201, "name": "Room2_BoxIn_OuterTop_Door",
         "description": "房間2 入口傳送箱 外側上方門", "value": "1", "type_of_value": "int", "dm_address": "12001"},
        {"id": 20106, "eqp_id": 201, "name": "Room2_BoxIn_OuterBottom_Door",
         "description": "房間2 入口傳送箱 外側下方門", "value": "1", "type_of_value": "int", "dm_address": "12002"},
        {"id": 20107, "eqp_id": 201, "name": "Room2_BoxIn_InnerTop_Door",
         "description": "房間2 入口傳送箱 內側上方門", "value": "1", "type_of_value": "int", "dm_address": "12003"},
        {"id": 20108, "eqp_id": 201, "name": "Room2_BoxIn_InnerBottom_Door",
         "description": "房間2 入口傳送箱 內側下方門", "value": "1", "type_of_value": "int", "dm_address": "12004"},

        # 房間 2 出口傳送箱 訊號
        {"id": 20201, "eqp_id": 202, "eqp_port_id": 2021, "name": "Room2_BoxOut_Port01_Presence",
         "description": "房間2 出口傳送箱 Port01 在席訊號", "value": "0", "type_of_value": "bool", "dm_address": "12010.0"},
        {"id": 20202, "eqp_id": 202, "eqp_port_id": 2022, "name": "Room2_BoxOut_Port02_Presence",
         "description": "房間2 出口傳送箱 Port02 在席訊號", "value": "0", "type_of_value": "bool", "dm_address": "12010.1"},
        {"id": 20203, "eqp_id": 202, "eqp_port_id": 2023, "name": "Room2_BoxOut_Port03_Presence",
         "description": "房間2 出口傳送箱 Port03 在席訊號", "value": "0", "type_of_value": "bool", "dm_address": "12010.2"},
        {"id": 20204, "eqp_id": 202, "eqp_port_id": 2024, "name": "Room2_BoxOut_Port04_Presence",
         "description": "房間2 出口傳送箱 Port04 在席訊號", "value": "0", "type_of_value": "bool", "dm_address": "12010.3"},
        {"id": 20205, "eqp_id": 202, "name": "Room2_BoxOut_OuterTop_Door",
         "description": "房間2 出口傳送箱 外側上方門", "value": "1", "type_of_value": "int", "dm_address": "12011"},
        {"id": 20206, "eqp_id": 202, "name": "Room2_BoxOut_OuterBottom_Door",
         "description": "房間2 出口傳送箱 外側下方門", "value": "1", "type_of_value": "int", "dm_address": "12012"},
        {"id": 20207, "eqp_id": 202, "name": "Room2_BoxOut_InnerTop_Door",
         "description": "房間2 出口傳送箱 內側上方門", "value": "1", "type_of_value": "int", "dm_address": "12013"},
        {"id": 20208, "eqp_id": 202, "name": "Room2_BoxOut_InnerBottom_Door",
         "description": "房間2 出口傳送箱 內側下方門", "value": "1", "type_of_value": "int", "dm_address": "12014"},


        # 房間 2 清洗機 訊號
        {"id": 20301, "eqp_id": 203, "eqp_port_id": 2031, "name": "Room2_Cleaner_Port01_Presence",
         "description": "房間2 清洗機 Port01 在席訊號", "value": "0", "type_of_value": "bool", "dm_address": "12010.0"},
        {"id": 20302, "eqp_id": 203, "eqp_port_id": 2032, "name": "Room2_Cleaner_Port02_Presence",
         "description": "房間2 清洗機 Port02 在席訊號", "value": "0", "type_of_value": "bool", "dm_address": "12010.1"},
        {"id": 20303, "eqp_id": 203, "eqp_port_id": 2033, "name": "Room2_Cleaner_Port03_Presence",
         "description": "房間2 清洗機 Port03 在席訊號", "value": "0", "type_of_value": "bool", "dm_address": "12010.2"},
        {"id": 20304, "eqp_id": 203, "eqp_port_id": 2034, "name": "Room2_Cleaner_Port04_Presence",
         "description": "房間2 清洗機 Port04 在席訊號", "value": "0", "type_of_value": "bool", "dm_address": "12010.3"},
        {"id": 20305, "eqp_id": 203, "eqp_port_id": 2031, "name": "Room2_Cleaner_Port01_Allow_Load",
         "description": "房間2 清洗機 Port01 允許/要料訊號", "value": "0", "type_of_value": "bool", "dm_address": "12011.0"},
        {"id": 20306, "eqp_id": 203, "eqp_port_id": 2032, "name": "Room2_Cleaner_Port02_Allow_Load",
         "description": "房間2 清洗機 Port02 允許/要料訊號", "value": "0", "type_of_value": "bool", "dm_address": "12011.1"},
        {"id": 20307, "eqp_id": 203, "eqp_port_id": 2033, "name": "Room2_Cleaner_Port03_Allow_Load",
         "description": "房間2 清洗機 Port03 允許/要料訊號", "value": "0", "type_of_value": "bool", "dm_address": "12011.2"},
        {"id": 20308, "eqp_id": 203, "eqp_port_id": 2034, "name": "Room2_Cleaner_Port04_Allow_Load",
         "description": "房間2 清洗機 Port04 允許/要料訊號", "value": "0", "type_of_value": "bool", "dm_address": "12011.3"},
        {"id": 20309, "eqp_id": 203, "eqp_port_id": 2031, "name": "Room2_Cleaner_Port01_Allow_Unload",
         "description": "房間2 清洗機 Port01 允許/出料訊號", "value": "0", "type_of_value": "bool", "dm_address": "12012.0"},
        {"id": 20310, "eqp_id": 203, "eqp_port_id": 2032, "name": "Room2_Cleaner_Port02_Allow_Unload",
         "description": "房間2 清洗機 Port02 允許/出料訊號", "value": "0", "type_of_value": "bool", "dm_address": "12012.1"},
        {"id": 20311, "eqp_id": 203, "eqp_port_id": 2033, "name": "Room2_Cleaner_Port03_Allow_Unload",
         "description": "房間2 清洗機 Port03 允許/出料訊號", "value": "0", "type_of_value": "bool", "dm_address": "12012.2"},
        {"id": 20312, "eqp_id": 203, "eqp_port_id": 2034, "name": "Room2_Cleaner_Port04_Allow_Unload",
         "description": "房間2 清洗機 Port04 允許/出料訊號", "value": "0", "type_of_value": "bool", "dm_address": "12012.3"},

        # 房間 2 泡藥機 訊號
        {"id": 20401, "eqp_id": 204, "eqp_port_id": 2041, "name": "Room2_Soaking_1_Presence",
         "description": "房間2 泡藥機 在席訊號", "value": "0", "type_of_value": "bool", "dm_address": "12020.0"},
        {"id": 20402, "eqp_id": 204, "eqp_port_id": 2042, "name": "Room2_Soaking_2_Presence",
         "description": "房間2 泡藥機 在席訊號", "value": "0", "type_of_value": "bool", "dm_address": "12020.1"},
        {"id": 20403, "eqp_id": 204, "eqp_port_id": 2043, "name": "Room2_Soaking_3_Presence",
         "description": "房間2 泡藥機 在席訊號", "value": "0", "type_of_value": "bool", "dm_address": "12020.2"},
        {"id": 20404, "eqp_id": 204, "eqp_port_id": 2044, "name": "Room2_Soaking_4_Presence",
         "description": "房間2 泡藥機 在席訊號", "value": "0", "type_of_value": "bool", "dm_address": "12020.3"},
        {"id": 20405, "eqp_id": 204, "eqp_port_id": 2045, "name": "Room2_Soaking_5_Presence",
         "description": "房間2 泡藥機 在席訊號", "value": "0", "type_of_value": "bool", "dm_address": "12020.4"},
        {"id": 20406, "eqp_id": 204, "eqp_port_id": 2046, "name": "Room2_Soaking_6_Presence",
         "description": "房間2 泡藥機 在席訊號", "value": "0", "type_of_value": "bool", "dm_address": "12020.5"},
        {"id": 20407, "eqp_id": 204, "eqp_port_id": 2041, "name": "Room2_Soaking_1_Allow_Load",
         "description": "房間2 泡藥機 允許/要料訊號", "value": "0", "type_of_value": "bool", "dm_address": "12021.0"},
        {"id": 20408, "eqp_id": 204, "eqp_port_id": 2042, "name": "Room2_Soaking_2_Allow_Load",
         "description": "房間2 泡藥機 允許/要料訊號", "value": "0", "type_of_value": "bool", "dm_address": "12021.1"},
        {"id": 20409, "eqp_id": 204, "eqp_port_id": 2043, "name": "Room2_Soaking_3_Allow_Load",
         "description": "房間2 泡藥機 允許/要料訊號", "value": "0", "type_of_value": "bool", "dm_address": "12021.2"},
        {"id": 20410, "eqp_id": 204, "eqp_port_id": 2044, "name": "Room2_Soaking_4_Allow_Load",
         "description": "房間2 泡藥機 允許/要料訊號", "value": "0", "type_of_value": "bool", "dm_address": "12021.3"},
        {"id": 20411, "eqp_id": 204, "eqp_port_id": 2045, "name": "Room2_Soaking_5_Allow_Load",
         "description": "房間2 泡藥機 允許/要料訊號", "value": "0", "type_of_value": "bool", "dm_address": "12021.4"},
        {"id": 20412, "eqp_id": 204, "eqp_port_id": 2046, "name": "Room2_Soaking_6_Allow_Load",
         "description": "房間2 泡藥機 允許/出料訊號", "value": "0", "type_of_value": "bool", "dm_address": "12021.5"},
        {"id": 20413, "eqp_id": 204, "eqp_port_id": 2041, "name": "Room2_Soaking_1_Allow_Unload",
         "description": "房間2 泡藥機 允許/出料訊號", "value": "0", "type_of_value": "bool", "dm_address": "12022.0"},
        {"id": 20414, "eqp_id": 204, "eqp_port_id": 2042, "name": "Room2_Soaking_2_Allow_Unload",
         "description": "房間2 泡藥機 允許/出料訊號", "value": "0", "type_of_value": "bool", "dm_address": "12022.1"},
        {"id": 20415, "eqp_id": 204, "eqp_port_id": 2043, "name": "Room2_Soaking_3_Allow_Unload",
         "description": "房間2 泡藥機 允許/出料訊號", "value": "0", "type_of_value": "bool", "dm_address": "12022.2"},
        {"id": 20416, "eqp_id": 204, "eqp_port_id": 2044, "name": "Room2_Soaking_4_Allow_Unload",
         "description": "房間2 泡藥機 允許/出料訊號", "value": "0", "type_of_value": "bool", "dm_address": "12022.3"},
        {"id": 20417, "eqp_id": 204, "eqp_port_id": 2045, "name": "Room2_Soaking_5_Allow_Unload",
         "description": "房間2 泡藥機 允許/出料訊號", "value": "0", "type_of_value": "bool", "dm_address": "12022.4"},
        {"id": 20418, "eqp_id": 204, "eqp_port_id": 2046, "name": "Room2_Soaking_6_Allow_Unload",
         "description": "房間2 泡藥機 允許/出料訊號", "value": "0", "type_of_value": "bool", "dm_address": "12022.5"},

        # 房間 2 預烘機 訊號
        {"id": 20501, "eqp_id": 205, "eqp_port_id": 2051, "name": "Room2_Dryer_Port01_Presence",
         "description": "房間2 預烘機 在席訊號", "value": "0", "type_of_value": "bool", "dm_address": "12030.0"},
        {"id": 20502, "eqp_id": 205, "eqp_port_id": 2052, "name": "Room2_Dryer_Port02_Presence",
         "description": "房間2 預烘機 在席訊號", "value": "0", "type_of_value": "bool", "dm_address": "12030.1"},
        {"id": 20503, "eqp_id": 205, "eqp_port_id": 2053, "name": "Room2_Dryer_Port03_Presence",
         "description": "房間2 預烘機 在席訊號", "value": "0", "type_of_value": "bool", "dm_address": "12030.2"},
        {"id": 20504, "eqp_id": 205, "eqp_port_id": 2054, "name": "Room2_Dryer_Port04_Presence",
         "description": "房間2 預烘機 在席訊號", "value": "0", "type_of_value": "bool", "dm_address": "12030.3"},
        {"id": 20505, "eqp_id": 205, "eqp_port_id": 2055, "name": "Room2_Dryer_Port05_Presence",
         "description": "房間2 預烘機 在席訊號", "value": "0", "type_of_value": "bool", "dm_address": "12030.4"},
        {"id": 20506, "eqp_id": 205, "eqp_port_id": 2056, "name": "Room2_Dryer_Port06_Presence",
         "description": "房間2 預烘機 在席訊號", "value": "0", "type_of_value": "bool", "dm_address": "12030.5"},
        {"id": 20507, "eqp_id": 205, "eqp_port_id": 2057, "name": "Room2_Dryer_Port07_Presence",
         "description": "房間2 預烘機 在席訊號", "value": "0", "type_of_value": "bool", "dm_address": "12030.6"},
        {"id": 20508, "eqp_id": 205, "eqp_port_id": 2058, "name": "Room2_Dryer_Port08_Presence",
         "description": "房間2 預烘機 在席訊號", "value": "0", "type_of_value": "bool", "dm_address": "12030.7"},
        {"id": 20509, "eqp_id": 205, "eqp_port_id": 2051, "name": "Room2_Dryer_Port01_Allow_Load",
         "description": "房間2 預烘機 允許/要料訊號", "value": "0", "type_of_value": "bool", "dm_address": "12031.0"},
        {"id": 20510, "eqp_id": 205, "eqp_port_id": 2052, "name": "Room2_Dryer_Port02_Allow_Load",
         "description": "房間2 預烘機 允許/要料訊號", "value": "0", "type_of_value": "bool", "dm_address": "12031.1"},
        {"id": 20511, "eqp_id": 205, "eqp_port_id": 2053, "name": "Room2_Dryer_Port03_Allow_Load",
         "description": "房間2 預烘機 允許/要料訊號", "value": "0", "type_of_value": "bool", "dm_address": "12031.2"},
        {"id": 20512, "eqp_id": 205, "eqp_port_id": 2054, "name": "Room2_Dryer_Port04_Allow_Load",
         "description": "房間2 預烘機 允許/要料訊號", "value": "0", "type_of_value": "bool", "dm_address": "12031.3"},
        {"id": 20513, "eqp_id": 205, "eqp_port_id": 2055, "name": "Room2_Dryer_Port05_Allow_Load",
         "description": "房間2 預烘機 允許/要料訊號", "value": "0", "type_of_value": "bool", "dm_address": "12031.4"},
        {"id": 20514, "eqp_id": 205, "eqp_port_id": 2056, "name": "Room2_Dryer_Port06_Allow_Load",
         "description": "房間2 預烘機 允許/要料訊號", "value": "0", "type_of_value": "bool", "dm_address": "12031.5"},
        {"id": 20515, "eqp_id": 205, "eqp_port_id": 2057, "name": "Room2_Dryer_Port07_Allow_Load",
         "description": "房間2 預烘機 允許/要料訊號", "value": "0", "type_of_value": "bool", "dm_address": "12031.6"},
        {"id": 20516, "eqp_id": 205, "eqp_port_id": 2058, "name": "Room2_Dryer_Port08_Allow_Load",
         "description": "房間2 預烘機 允許/要料訊號", "value": "0", "type_of_value": "bool", "dm_address": "12031.7"},
        {"id": 20517, "eqp_id": 205, "eqp_port_id": 2051, "name": "Room2_Dryer_Port01_Allow_Unload",
         "description": "房間2 預烘機 允許/出料訊號", "value": "0", "type_of_value": "bool", "dm_address": "12032.0"},
        {"id": 20518, "eqp_id": 205, "eqp_port_id": 2052, "name": "Room2_Dryer_Port02_Allow_Unload",
         "description": "房間2 預烘機 允許/出料訊號", "value": "0", "type_of_value": "bool", "dm_address": "12032.1"},
        {"id": 20519, "eqp_id": 205, "eqp_port_id": 2053, "name": "Room2_Dryer_Port03_Allow_Unload",
         "description": "房間2 預烘機 允許/出料訊號", "value": "0", "type_of_value": "bool", "dm_address": "12032.2"},
        {"id": 20520, "eqp_id": 205, "eqp_port_id": 2054, "name": "Room2_Dryer_Port04_Allow_Unload",
         "description": "房間2 預烘機 允許/出料訊號", "value": "0", "type_of_value": "bool", "dm_address": "12032.3"},
        {"id": 20521, "eqp_id": 205, "eqp_port_id": 2055, "name": "Room2_Dryer_Port05_Allow_Unload",
         "description": "房間2 預烘機 允許/出料訊號", "value": "0", "type_of_value": "bool", "dm_address": "12032.4"},
        {"id": 20522, "eqp_id": 205, "eqp_port_id": 2056, "name": "Room2_Dryer_Port06_Allow_Unload",
         "description": "房間2 預烘機 允許/出料訊號", "value": "0", "type_of_value": "bool", "dm_address": "12032.5"},
        {"id": 20523, "eqp_id": 205, "eqp_port_id": 2057, "name": "Room2_Dryer_Port07_Allow_Unload",
         "description": "房間2 預烘機 允許/出料訊號", "value": "0", "type_of_value": "bool", "dm_address": "12032.6"},
        {"id": 20524, "eqp_id": 205, "eqp_port_id": 2058, "name": "Room2_Dryer_Port08_Allow_Unload",
         "description": "房間2 預烘機 允許/出料訊號", "value": "0", "type_of_value": "bool", "dm_address": "12032.7"},

        # 房間 2 烤箱 訊號
        {"id": 20601, "eqp_id": 206, "eqp_port_id": 2061, "name": "Room2_Oven_Port01_Presence",
         "description": "房間2 烤箱 在席訊號", "value": "0", "type_of_value": "bool", "dm_address": "12040.0"},
        {"id": 20602, "eqp_id": 206, "eqp_port_id": 2062, "name": "Room2_Oven_Port02_Presence",
         "description": "房間2 烤箱 在席訊號", "value": "0", "type_of_value": "bool", "dm_address": "12040.1"},
        {"id": 20603, "eqp_id": 206, "eqp_port_id": 2063, "name": "Room2_Oven_Port03_Presence",
         "description": "房間2 烤箱 在席訊號", "value": "0", "type_of_value": "bool", "dm_address": "12040.2"},
        {"id": 20604, "eqp_id": 206, "eqp_port_id": 2064, "name": "Room2_Oven_Port04_Presence",
         "description": "房間2 烤箱 在席訊號", "value": "0", "type_of_value": "bool", "dm_address": "12040.3"},
        {"id": 20605, "eqp_id": 206, "eqp_port_id": 2065, "name": "Room2_Oven_Port05_Presence",
         "description": "房間2 烤箱 在席訊號", "value": "0", "type_of_value": "bool", "dm_address": "12040.4"},
        {"id": 20606, "eqp_id": 206, "eqp_port_id": 2066, "name": "Room2_Oven_Port06_Presence",
         "description": "房間2 烤箱 在席訊號", "value": "0", "type_of_value": "bool", "dm_address": "12040.5"},
        {"id": 20607, "eqp_id": 206, "eqp_port_id": 2067, "name": "Room2_Oven_Port07_Presence",
         "description": "房間2 烤箱 在席訊號", "value": "0", "type_of_value": "bool", "dm_address": "12040.6"},
        {"id": 20608, "eqp_id": 206, "eqp_port_id": 2068, "name": "Room2_Oven_Port08_Presence",
         "description": "房間2 烤箱 在席訊號", "value": "0", "type_of_value": "bool", "dm_address": "12040.7"},
        {"id": 20609, "eqp_id": 206, "eqp_port_id": 2061, "name": "Room2_Oven_Port01_Allow_Load",
         "description": "房間2 烤箱 允許/要料訊號", "value": "0", "type_of_value": "bool", "dm_address": "12041.0"},
        {"id": 20610, "eqp_id": 206, "eqp_port_id": 2062, "name": "Room2_Oven_Port02_Allow_Load",
         "description": "房間2 烤箱 允許/要料訊號", "value": "0", "type_of_value": "bool", "dm_address": "12041.1"},
        {"id": 20611, "eqp_id": 206, "eqp_port_id": 2063, "name": "Room2_Oven_Port03_Allow_Load",
         "description": "房間2 烤箱 允許/要料訊號", "value": "0", "type_of_value": "bool", "dm_address": "12041.2"},
        {"id": 20612, "eqp_id": 206, "eqp_port_id": 2064, "name": "Room2_Oven_Port04_Allow_Load",
         "description": "房間2 烤箱 允許/要料訊號", "value": "0", "type_of_value": "bool", "dm_address": "12041.3"},
        {"id": 20613, "eqp_id": 206, "eqp_port_id": 2065, "name": "Room2_Oven_Port05_Allow_Load",
         "description": "房間2 烤箱 允許/要料訊號", "value": "0", "type_of_value": "bool", "dm_address": "12041.4"},
        {"id": 20614, "eqp_id": 206, "eqp_port_id": 2066, "name": "Room2_Oven_Port06_Allow_Load",
         "description": "房間2 烤箱 允許/要料訊號", "value": "0", "type_of_value": "bool", "dm_address": "12041.5"},
        {"id": 20615, "eqp_id": 206, "eqp_port_id": 2067, "name": "Room2_Oven_Port07_Allow_Load",
         "description": "房間2 烤箱 允許/要料訊號", "value": "0", "type_of_value": "bool", "dm_address": "12041.6"},
        {"id": 20616, "eqp_id": 206, "eqp_port_id": 2068, "name": "Room2_Oven_Port08_Allow_Load",
         "description": "房間2 烤箱 允許/要料訊號", "value": "0", "type_of_value": "bool", "dm_address": "12041.7"},
        {"id": 20617, "eqp_id": 206, "eqp_port_id": 2061, "name": "Room2_Oven_Port01_Allow_Unload",
         "description": "房間2 烤箱 允許/出料訊號", "value": "0", "type_of_value": "bool", "dm_address": "12042.0"},
        {"id": 20618, "eqp_id": 206, "eqp_port_id": 2062, "name": "Room2_Oven_Port02_Allow_Unload",
         "description": "房間2 烤箱 允許/出料訊號", "value": "0", "type_of_value": "bool", "dm_address": "12042.1"},
        {"id": 20619, "eqp_id": 206, "eqp_port_id": 2063, "name": "Room2_Oven_Port03_Allow_Unload",
         "description": "房間2 烤箱 允許/出料訊號", "value": "0", "type_of_value": "bool", "dm_address": "12042.2"},
        {"id": 20620, "eqp_id": 206, "eqp_port_id": 2064, "name": "Room2_Oven_Port04_Allow_Unload",
         "description": "房間2 烤箱 允許/出料訊號", "value": "0", "type_of_value": "bool", "dm_address": "12042.3"},
        {"id": 20621, "eqp_id": 206, "eqp_port_id": 2065, "name": "Room2_Oven_Port05_Allow_Unload",
         "description": "房間2 烤箱 允許/出料訊號", "value": "0", "type_of_value": "bool", "dm_address": "12042.4"},
        {"id": 20622, "eqp_id": 206, "eqp_port_id": 2066, "name": "Room2_Oven_Port06_Allow_Unload",
         "description": "房間2 烤箱 允許/出料訊號", "value": "0", "type_of_value": "bool", "dm_address": "12042.5"},
        {"id": 20623, "eqp_id": 206, "eqp_port_id": 2067, "name": "Room2_Oven_Port07_Allow_Unload",
         "description": "房間2 烤箱 允許/出料訊號", "value": "0", "type_of_value": "bool", "dm_address": "12042.6"},
        {"id": 20624, "eqp_id": 206, "eqp_port_id": 2068, "name": "Room2_Oven_Port08_Allow_Unload",
         "description": "房間2 烤箱 允許/出料訊號", "value": "0", "type_of_value": "bool", "dm_address": "12042.7"},
    ]

    insert_data_if_not_exists_name_and_not_exists_id(
        session, default_eqp_signals, EqpSignal)
    print("✅ 設備信號資料初始化完成")
