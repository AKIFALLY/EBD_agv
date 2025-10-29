from db_proxy.connection_pool_manager import ConnectionPoolManager
# 只需匯入 SQLModel 與 model
from db_proxy.models import ProcessSettings, Product, Work, Task, TaskStatus, AGV, AGVContext, TrafficZone, Node, NodeType, Room, Machine, RackStatus, Rack, LocationStatus, Location, Carrier, Eqp, EqpPort, EqpSignal, RosoutLog, RuntimeLog
from sqlmodel import select
# from db_proxy_interfaces.srv import AcquireTrafficArea, ReleaseTrafficArea, AddTrafficArea  # Define custom services


def initialize_default_data(pool_agvc):
    """初始化預設資料，若已存在則不插入"""

    with pool_agvc.get_session() as session:

        # 預設節點Type
        default_node_types = [
            {"id": 4, "name": "充電入口點",
                "description": "KUKA充電入口點資訊由地圖檔中該點含functionList且functionType=4的Node"},
            {"id": 10, "name": "充電站點",
                "description": "KUKA充電站點資訊由地圖檔中該點含functionList且functionType=10的Node"},
            {"id": 2, "name": "貨架點",
                "description": "KUKA貨架點資訊由地圖檔中該點含functionList且functionType=2的Node"},
            {"id": 6, "name": "避讓點",
                "description": "KUKA避讓點資訊由地圖檔中該點含functionList且functionType=6的Node"},
        ]
        insert_data_if_not_exists_name_and_not_exists_id(
            session, default_node_types, NodeType)

        # 預設節點(Tag點)
        default_nodes = [
            {"id": 1001, "name": "射出機1-停車位1",
                "description": "射出機1-停車位置1", "x": "0.0", "y": "0.0"},
            {"id": 1002, "name": "射出機1-停車位2",
                "description": "射出機1-停車位置2", "x": "0.0", "y": "0.0"},
            {"id": 1003, "name": "射出機2-停車位1",
                "description": "射出機2-停車位置1", "x": "0.0", "y": "0.0"},
            {"id": 1004, "name": "射出機2-停車位2",
                "description": "射出機2-停車位置2", "x": "0.0", "y": "0.0"},
            {"id": 1005, "name": "射出機3-停車位1",
                "description": "射出機3-停車位置1", "x": "0.0", "y": "0.0"},
            {"id": 1006, "name": "射出機3-停車位2",
                "description": "射出機3-停車位置2", "x": "0.0", "y": "0.0"},
            {"id": 1007, "name": "射出機4-停車位1",
                "description": "射出機4-停車位置1", "x": "0.0", "y": "0.0"},
            {"id": 1008, "name": "射出機4-停車位2",
                "description": "射出機4-停車位置2", "x": "0.0", "y": "0.0"},

            {"id": 10001, "name": "room01 Loader Box",
                "description": "房間1入口傳送箱", "x": "0.0", "y": "0.0"},
            {"id": 10002, "name": "room01 Unloader Box",
                "description": "房間1出口傳送箱", "x": "0.0", "y": "0.0"},

            {"id": 20001, "name": "room02 Loader Box",
                "description": "房間2入口傳送箱", "x": "0.0", "y": "0.0"},
            {"id": 20002, "name": "room02 Unloader Box",
                "description": "房間2出口傳送箱", "x": "0.0", "y": "0.0"},

            {"id": 30001, "name": "room03 Loader Box",
                "description": "房間3入口傳送箱", "x": "0.0", "y": "0.0"},
            {"id": 30002, "name": "room03 Unloader Box",
                "description": "房間3出口傳送箱", "x": "0.0", "y": "0.0"},

            {"id": 40001, "name": "room04 Loader Box",
                "description": "房間4入口傳送箱", "x": "0.0", "y": "0.0"},
            {"id": 40002, "name": "room04 Unloader Box",
                "description": "房間4出口傳送箱", "x": "0.0", "y": "0.0"},

            {"id": 50001, "name": "room05 Loader Box",
                "description": "房間5入口傳送箱", "x": "0.0", "y": "0.0"},
            {"id": 50002, "name": "room05 Unloader Box",
                "description": "房間5出口傳送箱", "x": "0.0", "y": "0.0"},

            {"id": 20101, "name": "LoaderAGV Loader Box",
                "description": "房間內入口傳送箱", "x": "0.0", "y": "0.0"},

            {"id": 20301, "name": "LoaderAGV Washer Inport",
                "description": "清洗機入口", "x": "0.0", "y": "0.0"},
            {"id": 20302, "name": "LoaderAGV Washer Outport",
                "description": "清洗機出口", "x": "0.0", "y": "0.0"},

            {"id": 20401, "name": "LoaderAGV Soaker A",
                "description": "泡藥機A", "x": "0.0", "y": "0.0"},
            {"id": 20402, "name": "LoaderAGV Soaker B",
                "description": "泡藥機B", "x": "0.0", "y": "0.0"},
            {"id": 20403, "name": "LoaderAGV Soaker C",
                "description": "泡藥機C", "x": "0.0", "y": "0.0"},
            {"id": 20404, "name": "LoaderAGV Soaker D",
                "description": "泡藥機D", "x": "0.0", "y": "0.0"},
            {"id": 20405, "name": "LoaderAGV Soaker E",
                "description": "泡藥機E", "x": "0.0", "y": "0.0"},
            {"id": 20406, "name": "LoaderAGV Soaker F",
                "description": "泡藥機F", "x": "0.0", "y": "0.0"},

            {"id": 20501, "name": "LoaderAGV Prebaker 1",
                "description": "預烘機A1", "x": "0.0", "y": "0.0"},
            {"id": 20502, "name": "LoaderAGV Prebaker 2",
                "description": "預烘機A2", "x": "0.0", "y": "0.0"},
            {"id": 20503, "name": "LoaderAGV Prebaker 3",
                "description": "預烘機B1", "x": "0.0", "y": "0.0"},
            {"id": 20504, "name": "LoaderAGV Prebaker 4",
                "description": "預烘機B2", "x": "0.0", "y": "0.0"},
            {"id": 20505, "name": "LoaderAGV Prebaker 5",
                "description": "預烘機C1", "x": "0.0", "y": "0.0"},
            {"id": 20506, "name": "LoaderAGV Prebaker 6",
                "description": "預烘機C2", "x": "0.0", "y": "0.0"},
            {"id": 20507, "name": "LoaderAGV Prebaker 7",
                "description": "預烘機D1", "x": "0.0", "y": "0.0"},
            {"id": 20508, "name": "LoaderAGV Prebaker 8",
                "description": "預烘機D2", "x": "0.0", "y": "0.0"},

            {"id": 20509, "name": "UnloaderAGV Prebaker A",
                "description": "預烘機A", "x": "0.0", "y": "0.0"},
            {"id": 20510, "name": "UnloaderAGV Prebaker B",
                "description": "預烘機B", "x": "0.0", "y": "0.0"},
            {"id": 20511, "name": "UnloaderAGV Prebaker C",
                "description": "預烘機C", "x": "0.0", "y": "0.0"},
            {"id": 20512, "name": "UnloaderAGV Prebaker D",
                "description": "預烘機D", "x": "0.0", "y": "0.0"},

            {"id": 20601, "name": "UnloaderAGV Baker",
                "description": "烤箱A", "x": "0.0", "y": "0.0"},
            {"id": 20602, "name": "UnloaderAGV Baker",
                "description": "烤箱B", "x": "0.0", "y": "0.0"},

            {"id": 20201, "name": "UnloaderAGV Unload Box",
                "description": "房間內出口傳送箱", "x": "0.0", "y": "0.0"},



        ]
        insert_data_if_not_exists_name(session, default_nodes, Node)

        # 預設Location_status
        default_location_status = [
            {"id": 1, "name": "未知狀態", "description": "未知狀態"},
            {"id": 2, "name": "未佔用", "description": "空位沒有被使用"},
            {"id": 3, "name": "佔用", "description": "已經有停放的料架"},
        ]
        insert_data_if_not_exists_name(
            session, default_location_status, LocationStatus)

        # 預設射出機資料
        default_machines = [
            {"id": 1, "parking_space_1": 1001, "parking_space_2": 1002,
                "name": "射出機1", "description": "射出機1", "enable": 1},
            {"id": 2, "parking_space_1": 1003, "parking_space_2": 1004,
                "name": "射出機2", "description": "射出機2", "enable": 1},
            {"id": 3, "parking_space_1": 1005, "parking_space_2": 1006,
                "name": "射出機3", "description": "射出機3", "enable": 0},
            {"id": 4, "parking_space_1": 1007, "parking_space_2": 1008,
                "name": "射出機4", "description": "射出機4", "enable": 0},
        ]
        insert_data_if_not_exists_name(session, default_machines, Machine)

        # 預設 ProcessSettings
        default_processes = [
            {"soaking_times": 1, "description": "泡藥泡一次"},
            {"soaking_times": 2, "description": "泡藥泡兩次"}
        ]
        for proc in default_processes:
            exists = session.exec(select(ProcessSettings).where(
                ProcessSettings.soaking_times == proc["soaking_times"])).first()
            if not exists:
                session.add(ProcessSettings(**proc))
        session.commit()

        # 預設房間資料
        default_rooms = [
            {"id": 1, "process_settings_id": 1, "name": "Room1",
                "description": "第一間房間", "enable": 1},
            {"id": 2, "process_settings_id": 1, "name": "Room2",
                "description": "第二間房間", "enable": 1},
            {"id": 3, "process_settings_id": 1, "name": "Room3",
                "description": "第三間房間", "enable": 0},
            {"id": 4, "process_settings_id": 1, "name": "Room4",
                "description": "第四間房間", "enable": 0},
            {"id": 5, "process_settings_id": 1, "name": "Room5",
                "description": "第五間房間", "enable": 0},
        ]
        insert_data_if_not_exists_name(session, default_rooms, Room)

        # 預設Location
        default_location = [
            # 預建置測試資料
            {"id": 1, "location_status_id": 1, "room_id": 1,
                "node_id": 1, "name": "未知空位1", "description": "測試用未知位置"},
            {"id": 2, "location_status_id": 2, "room_id": 2,
                "node_id": 2, "name": "未知空位2", "description": "測試用資料-房間2的固定設備(入口傳送箱及出口傳送箱等等)"},

            {"id": 2001, "name": "射出機1-OP1作業位置", "description": "射出機1-OP1作業位置"},
            {"id": 2002, "name": "射出機1-OP2作業位置", "description": "射出機1-OP2作業位置"},
            {"id": 2003, "name": "射出機2-OP1作業位置", "description": "射出機2-OP1作業位置"},
            {"id": 2004, "name": "射出機2-OP2作業位置", "description": "射出機2-OP2作業位置"},
            {"id": 2005, "name": "射出機3-OP1作業位置", "description": "射出機3-OP1作業位置"},
            {"id": 2006, "name": "射出機3-OP2作業位置", "description": "射出機3-OP2作業位置"},
            {"id": 2007, "name": "射出機4-OP1作業位置", "description": "射出機4-OP1作業位置"},
            {"id": 2008, "name": "射出機4-OP2作業位置", "description": "射出機4-OP2作業位置"},

            {"id": 1001, "location_status_id": 1,
                "node_id": 1001, "name": "射出機1-停車位置1", "description": "射出機1-OP1叫車/停車位置"},
            {"id": 1002, "location_status_id": 1,
                "node_id": 1002, "name": "射出機1-停車位置2", "description": "射出機1-OP2叫車/停車位置"},
            {"id": 1003, "location_status_id": 1,
                "node_id": 1003, "name": "射出機2-停車位置1", "description": "射出機2-OP1叫車/停車位置"},
            {"id": 1004, "location_status_id": 1,
                "node_id": 1004, "name": "射出機2-停車位置2", "description": "射出機2-OP2叫車/停車位置"},
            {"id": 1005, "location_status_id": 1,
                "node_id": 1005, "name": "射出機3-停車位置1", "description": "射出機3-OP1叫車/停車位置"},
            {"id": 1006, "location_status_id": 1,
                "node_id": 1006, "name": "射出機3-停車位置2", "description": "射出機3-OP2叫車/停車位置"},
            {"id": 1007, "location_status_id": 1,
                "node_id": 1007, "name": "射出機4-停車位置1", "description": "射出機4-OP1叫車/停車位置"},
            {"id": 1008, "location_status_id": 1,
                "node_id": 1008, "name": "射出機4-停車位置2", "description": "射出機4-OP2叫車/停車位置"},

            {"id": 10101, "location_status_id": 1, "room_id": 2, "node_id": 10101,
                "name": "room01 Loader Box", "description": "房間1入口傳送箱"},
            {"id": 10201, "location_status_id": 1, "room_id": 2, "node_id": 10201,
                "name": "room01 Unloader Box", "description": "房間1出口傳送箱"},
            {"id": 20001, "location_status_id": 1, "room_id": 2, "node_id": 20001,
                "name": "room02 Loader Box", "description": "房間2入口傳送箱"},
            {"id": 20002, "location_status_id": 1, "room_id": 2, "node_id": 20002,
                "name": "room02 Unloader Box", "description": "房間2出口傳送箱"},
            {"id": 20101, "location_status_id": 1, "room_id": 2, "node_id": 20101,
                "name": "LoaderAGV Loader Box", "description": "房間內入口傳送箱"},
            {"id": 20301, "location_status_id": 1, "room_id": 2, "node_id": 20301,
                "name": "LoaderAGV Washer Inport", "description": "清洗機入口"},
            {"id": 20302, "location_status_id": 1, "room_id": 2, "node_id": 20302,
                "name": "LoaderAGV Washer Outport", "description": "清洗機出口"},
            {"id": 20401, "location_status_id": 1, "room_id": 2, "node_id": 20401,
                "name": "LoaderAGV Soaker A", "description": "泡藥機A"},
            {"id": 20402, "location_status_id": 1, "room_id": 2, "node_id": 20402,
                "name": "LoaderAGV Soaker B", "description": "泡藥機B"},
            {"id": 20403, "location_status_id": 1, "room_id": 2, "node_id": 20403,
                "name": "LoaderAGV Soaker C", "description": "泡藥機C"},
            {"id": 20404, "location_status_id": 1, "room_id": 2, "node_id": 20404,
                "name": "LoaderAGV Soaker D", "description": "泡藥機D"},
            {"id": 20405, "location_status_id": 1, "room_id": 2, "node_id": 20405,
                "name": "LoaderAGV Soaker E", "description": "泡藥機E"},
            {"id": 20406, "location_status_id": 1, "room_id": 2, "node_id": 20406,
                "name": "LoaderAGV Soaker F", "description": "泡藥機F"},
            {"id": 20501, "location_status_id": 1, "room_id": 2, "node_id": 20501,
                "name": "LoaderAGV Prebaker 1", "description": "預烘機A1"},
            {"id": 20502, "location_status_id": 1, "room_id": 2, "node_id": 20502,
                "name": "LoaderAGV Prebaker 2", "description": "預烘機A2"},
            {"id": 20503, "location_status_id": 1, "room_id": 2, "node_id": 20503,
                "name": "LoaderAGV Prebaker 3", "description": "預烘機B1"},
            {"id": 20504, "location_status_id": 1, "room_id": 2, "node_id": 20504,
                "name": "LoaderAGV Prebaker 4", "description": "預烘機B2"},
            {"id": 20505, "location_status_id": 1, "room_id": 2, "node_id": 20505,
                "name": "LoaderAGV Prebaker 5", "description": "預烘機C1"},
            {"id": 20506, "location_status_id": 1, "room_id": 2, "node_id": 20506,
                "name": "LoaderAGV Prebaker 6", "description": "預烘機C2"},
            {"id": 20507, "location_status_id": 1, "room_id": 2, "node_id": 20507,
                "name": "LoaderAGV Prebaker 7", "description": "預烘機D1"},
            {"id": 20508, "location_status_id": 1, "room_id": 2, "node_id": 20508,
                "name": "LoaderAGV Prebaker 8", "description": "預烘機D2"},
            {"id": 20509, "location_status_id": 1, "room_id": 2, "node_id": 20509,
                "name": "UnloaderAGV Prebaker A", "description": "預烘機A"},
            {"id": 20510, "location_status_id": 1, "room_id": 2, "node_id": 20510,
                "name": "UnloaderAGV Prebaker B", "description": "預烘機B"},
            {"id": 20511, "location_status_id": 1, "room_id": 2, "node_id": 20511,
                "name": "UnloaderAGV Prebaker C", "description": "預烘機C"},
            {"id": 20512, "location_status_id": 1, "room_id": 2, "node_id": 20512,
                "name": "UnloaderAGV Prebaker D", "description": "預烘機D"},
            {"id": 20601, "location_status_id": 1, "room_id": 2, "node_id": 20601,
                "name": "UnloaderAGV Baker", "description": "烤箱A"},
            {"id": 20602, "location_status_id": 1, "room_id": 2, "node_id": 20602,
                "name": "UnloaderAGV Baker", "description": "烤箱B"},
            {"id": 20201, "location_status_id": 1, "room_id": 2, "node_id": 20201,
                "name": "UnloaderAGV Unload Box", "description": "房間內出口傳送箱"},
        ]
        insert_data_if_not_exists_name(session, default_location, Location)

        # 預設EQP測試資料,入口傳送箱
        default_eqps = [
            # 預建置測試資料
            {"id": 210, "location_id": 210, "name": "LoaderAGV",
                "description": "LoaderAGV"},
            {"id": 211, "location_id": 211, "name": "UnloaderAGV",
                "description": "UnloaderAGV"},



            {"id": 201, "location_id": 201, "name": "Room2_BoxIn",
                "description": "房間2 入口傳送箱"},
            {"id": 202, "location_id": 202, "name": "Room2_BoxOut",
                "description": "房間2 出口傳送箱"},
            {"id": 203, "location_id": 203, "name": "Room2_Cleaner",
                "description": "房間2 清洗機"},
            {"id": 204, "location_id": 204, "name": "Room2_Soaking",
                "description": "房間2 泡藥機 群組"},
            {"id": 205, "location_id": 205, "name": "Room2_Dryer",
                "description": "房間2 預烘機"},
            {"id": 206, "location_id": 206, "name": "Room2_Oven",
                "description": "房間2 烤箱"},
        ]
        insert_data_if_not_exists_name(session, default_eqps, Eqp)

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

        ]
        insert_data_if_not_exists_name(session, default_eqp_port, EqpPort)

        default_eqp_signal = [
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
        insert_data_if_not_exists_name(session, default_eqp_signal, EqpSignal)

        # 預設Rack狀態
        default_rack_status = [
            {"id": 1, "name": "空架", "description": "全空料架未使用"},
            {"id": 2, "name": "滿料架-32", "description": "全滿料架(32格)"},
            {"id": 3, "name": "滿料架-16", "description": "全滿料架(16格)"},
            {"id": 4, "name": "未滿架-32", "description": "半滿料架(32格)"},
            {"id": 5, "name": "未滿架-16", "description": "半滿料架(16格)"},
        ]
        insert_data_if_not_exists_name(
            session, default_rack_status, RackStatus)

        # 預設 Product
        default_products = [
            {"name": "ABC12345", "size": "S", "process_settings_id": 1},
            {"name": "DEF67890", "size": "L", "process_settings_id": 2},
            {"name": "ABC54321", "size": "S", "process_settings_id": 1},
            {"name": "DEF09876", "size": "L", "process_settings_id": 2}
        ]
        insert_data_if_not_exists_name(session, default_products, Product)

        # 預設Rack
        default_rack = [
            {"id": 1, "name": "001", "location_id": 2001, "agv_id": None,
                "is_carry": 0, "product_id": 1, "status_id": 1, "direction": 0, "is_docked": 1},
            {"id": 2, "name": "002", "location_id": 1001, "agv_id": None,
                "is_carry": 0, "product_id": 1, "status_id": 1, "direction": 0},
            {"id": 3, "name": "003", "location_id": 1002, "agv_id": None,
                "is_carry": 0, "product_id": 1, "status_id": 1, "direction": 180},
            {"id": 4, "name": "004", "location_id": 1002, "agv_id": None,
                "is_carry": 0, "product_id": 1, "status_id": 1, "direction": 180},
            {"id": 5, "name": "005", "location_id": None, "agv_id": 123,
                "is_carry": 1, "product_id": 1, "status_id": 1, "direction": 180}
        ]
        insert_data_if_not_exists_name(session, default_rack, Rack)

        # 預設 AGV
        default_agv = [
            {"name": "cargo02", "model": "Cargo", "x": 0.0, "y": 0.0,
                "heading": 0.0, "description": "走廊AGV(暫時規劃僅負責房間2)"},
            {"name": "loader02", "model": "Loader", "x": 0.0, "y": 0.0,
                "heading": 0.0, "description": "房間2Loader(取入口傳送箱、清洗、泡藥、放預烘)"},
            {"name": "unloader02", "model": "Unloader", "x": 0.0, "y": 0.0,
                "heading": 0.0, "description": "房間2Unloader(取預烘、烤箱、放出口傳送箱)"},
            {"id": 101, "name": "KUKA001", "model": "KUKA400i", "x": 3116,
                "y": 1852, "heading": 0.0, "description": "在房間外負責料架搬運"},
            {"id": 102, "name": "KUKA002", "model": "KUKA400i", "x": 2860,
                "y": 1680, "heading": 0.0, "description": "在房間外負責料架搬運"},
            {"id": 123, "name": "KUKA003", "model": "KUKA400i", "x": 0.0,
                "y": 0.0, "heading": 0.0, "description": "(SimCar)在房間外負責料架搬運"}
        ]
        insert_data_if_not_exists_name_and_not_exists_id(
            session, default_agv, AGV)

        # 測試預設 Works
        test_work = [
            {"name": "走廊料架短產品32框 送入房間入口", "description": "走廊料架搬至房間入口所需參數",
                "parameters": {"room": 2, "rack_id": 1, "count": 32}},
            {"name": "走廊料架長產品16框 送入房間入口", "description": "走廊料架搬至房間入口所需參數",
                "parameters": {"room": 2, "rack_id": 2, "count": 16}},

            {"id": 2000102, "name": "CargoAGV放入口傳送箱",
                "description": "從料架拿carrier到入口傳送箱放", "parameters": {}},
            {"id": 2000201, "name": "CargoAGV取出口傳送箱",
                "description": "從出口傳送箱拿carrier到料架放", "parameters": {}},

            {"id": 2010101, "name": "LoaderAGV取入口傳送箱",
                "description": "從LoaderAGV取入口傳送箱拿到車上放", "parameters": {}},

            {"id": 2030102, "name": "LoaderAGV放清洗機",
                "description": "從LoaderAGV車上放到清洗機", "parameters": {}},
            {"id": 2030201, "name": "LoaderAGV取清洗機",
                "description": "從LoaderAGV取清洗機到車上放", "parameters": {}},

            {"id": 2040102, "name": "LoaderAGV放泡藥機A",
                "description": "從LoaderAGV車上放到泡藥機A", "parameters": {}},
            {"id": 2040202, "name": "LoaderAGV放泡藥機B",
                "description": "從LoaderAGV車上放到泡藥機B", "parameters": {}},
            {"id": 2040302, "name": "LoaderAGV放泡藥機C",
                "description": "從LoaderAGV車上放到泡藥機C", "parameters": {}},
            {"id": 2040402, "name": "LoaderAGV放泡藥機D",
                "description": "從LoaderAGV車上放到泡藥機D", "parameters": {}},
            {"id": 2040502, "name": "LoaderAGV放泡藥機E",
                "description": "從LoaderAGV車上放到泡藥機E", "parameters": {}},
            {"id": 2040602, "name": "LoaderAGV放泡藥機F",
                "description": "從LoaderAGV車上放到泡藥機F", "parameters": {}},

            {"id": 2040101, "name": "LoaderAGV拿泡藥機A",
                "description": "從泡藥機A拿到LoaderAGV車上放", "parameters": {}},
            {"id": 2040201, "name": "LoaderAGV拿泡藥機B",
                "description": "從泡藥機B拿到LoaderAGV車上放", "parameters": {}},
            {"id": 2040301, "name": "LoaderAGV拿泡藥機C",
                "description": "從泡藥機C拿到LoaderAGV車上放", "parameters": {}},
            {"id": 2040401, "name": "LoaderAGV拿泡藥機D",
                "description": "從泡藥機D拿到LoaderAGV車上放", "parameters": {}},
            {"id": 2040501, "name": "LoaderAGV拿泡藥機E",
                "description": "從泡藥機E拿到LoaderAGV車上放", "parameters": {}},
            {"id": 2040601, "name": "LoaderAGV拿泡藥機F",
                "description": "從泡藥機F拿到LoaderAGV車上放", "parameters": {}},

            {"id": 2050102, "name": "LoaderAGV放預烘機1",
                "description": "從LoaderAGV車上放到預烘機1", "parameters": {}},
            {"id": 2050202, "name": "LoaderAGV放預烘機2",
                "description": "從LoaderAGV車上放到預烘機2", "parameters": {}},
            {"id": 2050302, "name": "LoaderAGV放預烘機3",
                "description": "從LoaderAGV車上放到預烘機3", "parameters": {}},
            {"id": 2050402, "name": "LoaderAGV放預烘機4",
                "description": "從LoaderAGV車上放到預烘機4", "parameters": {}},
            {"id": 2050502, "name": "LoaderAGV放預烘機5",
                "description": "從LoaderAGV車上放到預烘機5", "parameters": {}},
            {"id": 2050602, "name": "LoaderAGV放預烘機6",
                "description": "從LoaderAGV車上放到預烘機6", "parameters": {}},
            {"id": 2050702, "name": "LoaderAGV放預烘機7",
                "description": "從LoaderAGV車上放到預烘機7", "parameters": {}},
            {"id": 2050802, "name": "LoaderAGV放預烘機8",
                "description": "從LoaderAGV車上放到預烘機8", "parameters": {}},

            {"id": 2050901, "name": "UnloaderAGV取預烘A",
                "description": "從預烘機A拿到UnloaderAGV車上放", "parameters": {}},
            {"id": 2051001, "name": "UnloaderAGV取預烘B",
                "description": "從預烘機B拿到UnloaderAGV車上放", "parameters": {}},
            {"id": 2051101, "name": "UnloaderAGV取預烘C",
                "description": "從預烘機C拿到UnloaderAGV車上放", "parameters": {}},
            {"id": 2051201, "name": "UnloaderAGV取預烘D",
                "description": "從預烘機D拿到UnloaderAGV車上放", "parameters": {}},

            {"id": 2060102, "name": "UnloaderAGV放烤箱A",
                "description": "從UnloaderAGV車上放到烤箱A", "parameters": {}},
            {"id": 2060202, "name": "UnloaderAGV放烤箱B",
                "description": "從UnloaderAGV車上放到烤箱B", "parameters": {}},
            {"id": 2060101, "name": "UnloaderAGV取烤箱A",
                "description": "從烤箱A拿到UnloaderAGV車上放", "parameters": {}},
            {"id": 2060201, "name": "UnloaderAGV取烤箱B",
                "description": "從烤箱B拿到UnloaderAGV車上放", "parameters": {}},

            {"id": 2020102, "name": "UnloaderAGV放出口傳送箱",
                "description": "從UnloaderAGV車上到出口傳送箱放", "parameters": {}},
        ]

        insert_data_if_not_exists_name(session, test_work, Work)

        # 預設 TaskStatus
        default_statuses = [
            {"name": "新建", "description": "任務剛建立"},
            {"name": "執行中", "description": "任務正在執行"},
            {"name": "完成", "description": "任務已完成"}
        ]

        insert_data_if_not_exists_name(session, default_statuses, TaskStatus)

        # 預設 TrafficZone
        default_traffic_zones = [
            {
                "name": "room2",
                "description": "房間2外走廊-基本交管區(points尚未定義)",
                "points": '[{"x":0,"y":0},{"x":1,"y":0},{"x":1,"y":1},{"x":0,"y":1}]',
                "status": "free",
                "owner_agv_id": None
            }
        ]

        insert_data_if_not_exists_name(
            session, default_traffic_zones, TrafficZone)

        # 測試用的 Carrier 資料，PK 為 id（自動產生），FK 包含 room_id、rack_id、port_id，其他欄位如 rack_index 與 status 可選擇性設定
        default_test_carrier = [
            {"rack_id": 123, "rack_index": 17},
            {"rack_id": 123, "rack_index": 18},
            {"rack_id": 123, "rack_index": 19},
            {"rack_id": 123, "rack_index": 20},
            {"rack_id": 123, "rack_index": 21},
            {"rack_id": 123, "rack_index": 22},
            {"rack_id": 123, "rack_index": 23},
            {"rack_id": 123, "rack_index": 24},
            {"rack_id": 123, "rack_index": 25},
            {"rack_id": 123, "rack_index": 28},
            {"rack_id": 123, "rack_index": 29},
            {"rack_id": 123, "rack_index": 32},
        ]

        # 檢查是否已存在預設資料，如果不存在則插入
        for data in default_test_carrier:
            exists = session.query(Carrier).filter_by(
                rack_id=data["rack_id"],
                rack_index=data["rack_index"]
            ).first()

            if not exists:
                session.add(Carrier(**data))

        session.commit()

        # 查找條件設定簡易範例
        # 查找 work/status/agv id
        work = session.exec(select(Work)).first()
        status_new = session.exec(select(TaskStatus).where(
            TaskStatus.name == "新建")).first()
        status_running = session.exec(select(TaskStatus).where(
            TaskStatus.name == "執行中")).first()

        select_idle_cargos = (
            select(AGV)
            # .join(AGVContext, AGV.id == AGVContext.agv_id) # 這是未定義 relationship 的寫法 , 需要寫 join AGV.id == AGVContext.agv_id
            # 這是定義 relationship 的寫法 , 不需要寫 join AGV.id == AGVContext.agv_id
            .join(AGVContext)
            .where(
                AGV.model == "Cargo",
                # AGVContext.context == "base",
                # AGVContext.current_state == "idle"
            )
        )
        agv_cargo = session.exec(select_idle_cargos).first()
        select_room2_loader = (
            select(AGV)
            # .join(AGVContext, AGV.id == AGVContext.agv_id) # 這是未定義 relationship 的寫法 , 需要寫 join AGV.id == AGVContext.agv_id
            # 這是定義 relationship 的寫法 , 不需要寫 join AGV.id == AGVContext.agv_id
            .join(AGVContext)
            .where(
                AGV.name == "Loader02",
            )
        )
        agv_loader = session.exec(select_room2_loader).first()

        # 預設 Task 實際使用情況要條件達成後才派任務
        default_tasks = [
            {
                "work_id": 2000102,
                "status_id": status_running.id if status_running else None,
                "room_id": 2,
                "name": "在房間(room_id=2)Cargo從料架(rack.id=1)取到入口傳送箱(eqp.id=1)放",
                "description": "走廊車Cargo將料架上的鏡框架依序送至入口傳送箱",
                "agv_id": 1,
                "priority": 10,
                "parameters": {"room_id": 2, "rack_id": 1, "eqp_id": 1}
            },
            {
                "work_id": 2000201,
                "status_id": status_running.id if status_running else None,
                "room_id": 2,
                "name": "在房間(room_id=2)Cargo從口傳送箱(eqp.id=2)取到料架(rack.id=1)放",
                "description": "走廊車Cargo將出口傳送箱依序放入料架",
                "agv_id": 1,
                "priority": 9,
                "parameters": {"room_id": 2, "rack_id": 2, "eqp_id": 2}
            },
            # {"work_id": 1002, "status_id": 1, "room_id": 2, "name": "1", "description": "1",
            #    "agv_id": 1, "priority": 9, "parameters": {"room_id": 2, "rack_id": 2, "eqp_id": 2}},
            # {"work_id": 1002, "status_id": 1, "room_id": 2, "name": "2", "description": "1",
            #    "agv_id": 1, "priority": 9, "parameters": {"room_id": 2, "rack_id": 2, "eqp_id": 2}},
            # {"work_id": 1002, "status_id": 1, "room_id": 2, "name": "3", "description": "1",
            #    "agv_id": 1, "priority": 9, "parameters": {"room_id": 2, "rack_id": 2, "eqp_id": 2}},
            # {"work_id": 1002, "status_id": 1, "room_id": 2, "name": "4", "description": "1",
            #    "agv_id": 1, "priority": 9, "parameters": {"room_id": 2, "rack_id": 2, "eqp_id": 2}},
            # {"work_id": 1002, "status_id": 1, "room_id": 2, "name": "5", "description": "1",
            #    "agv_id": 1, "priority": 9, "parameters": {"room_id": 2, "rack_id": 2, "eqp_id": 2}},
            # {"work_id": 1002, "status_id": 1, "room_id": 2, "name": "6", "description": "1",
            #    "agv_id": 1, "priority": 9, "parameters": {"room_id": 2, "rack_id": 2, "eqp_id": 2}},
            # {"work_id": 1002, "status_id": 1, "room_id": 2, "name": "7", "description": "1",
            #    "agv_id": 1, "priority": 9, "parameters": {"room_id": 2, "rack_id": 2, "eqp_id": 2}},
            # {"work_id": 1002, "status_id": 1, "room_id": 2, "name": "8", "description": "1",
            #    "agv_id": 1, "priority": 9, "parameters": {"room_id": 2, "rack_id": 2, "eqp_id": 2}},
            # {"work_id": 1002, "status_id": 1, "room_id": 2, "name": "9", "description": "1",
            #    "agv_id": 1, "priority": 9, "parameters": {"room_id": 2, "rack_id": 2, "eqp_id": 2}},
            # {"work_id": 1002, "status_id": 1, "room_id": 2, "name": "10", "description": "1",
            #    "agv_id": 1, "priority": 9, "parameters": {"room_id": 2, "rack_id": 2, "eqp_id": 2}},
            # {"work_id": 1002, "status_id": 1, "room_id": 2, "name": "11", "description": "1",
            #    "agv_id": 1, "priority": 9, "parameters": {"room_id": 2, "rack_id": 2, "eqp_id": 2}},
            # {"work_id": 1002, "status_id": 1, "room_id": 2, "name": "12", "description": "1",
            #    "agv_id": 1, "priority": 9, "parameters": {"room_id": 2, "rack_id": 2, "eqp_id": 2}},
            # {"work_id": 1002, "status_id": 1, "room_id": 2, "name": "13", "description": "1",
            #    "agv_id": 1, "priority": 9, "parameters": {"room_id": 2, "rack_id": 2, "eqp_id": 2}},
            # {"work_id": 1002, "status_id": 1, "room_id": 2, "name": "14", "description": "1",
            #    "agv_id": 1, "priority": 9, "parameters": {"room_id": 2, "rack_id": 2, "eqp_id": 2}},
            # {"work_id": 1002, "status_id": 1, "room_id": 2, "name": "15", "description": "1",
            #    "agv_id": 1, "priority": 9, "parameters": {"room_id": 2, "rack_id": 2, "eqp_id": 2}},
            # {"work_id": 1002, "status_id": 1, "room_id": 2, "name": "16", "description": "1",
            #    "agv_id": 1, "priority": 9, "parameters": {"room_id": 2, "rack_id": 2, "eqp_id": 2}},
            # {"work_id": 1002, "status_id": 1, "room_id": 2, "name": "17", "description": "1",
            #    "agv_id": 1, "priority": 9, "parameters": {"room_id": 2, "rack_id": 2, "eqp_id": 2}},
            # {"work_id": 1002, "status_id": 1, "room_id": 2, "name": "18", "description": "1",
            #    "agv_id": 1, "priority": 9, "parameters": {"room_id": 2, "rack_id": 2, "eqp_id": 2}},
            # {"work_id": 1002, "status_id": 1, "room_id": 2, "name": "19", "description": "1",
            #    "agv_id": 1, "priority": 9, "parameters": {"room_id": 2, "rack_id": 2, "eqp_id": 2}},
            # {"work_id": 1002, "status_id": 1, "room_id": 2, "name": "20", "description": "1",
            #    "agv_id": 1, "priority": 9, "parameters": {"room_id": 2, "rack_id": 2, "eqp_id": 2}},
            # {"work_id": 1002, "status_id": 1, "room_id": 2, "name": "21", "description": "1",
            #    "agv_id": 1, "priority": 9, "parameters": {"room_id": 2, "rack_id": 2, "eqp_id": 2}},
            # {"work_id": 1002, "status_id": 1, "room_id": 2, "name": "22", "description": "1",
            #    "agv_id": 1, "priority": 9, "parameters": {"room_id": 2, "rack_id": 2, "eqp_id": 2}},
            # {"work_id": 1002, "status_id": 1, "room_id": 2, "name": "23", "description": "1",
            #    "agv_id": 1, "priority": 9, "parameters": {"room_id": 2, "rack_id": 2, "eqp_id": 2}},
            # {"work_id": 1002, "status_id": 1, "room_id": 2, "name": "24", "description": "1",
            #    "agv_id": 1, "priority": 9, "parameters": {"room_id": 2, "rack_id": 2, "eqp_id": 2}},
            # {"work_id": 1002, "status_id": 1, "room_id": 2, "name": "25", "description": "1",
            #    "agv_id": 1, "priority": 9, "parameters": {"room_id": 2, "rack_id": 2, "eqp_id": 2}}
        ]

        insert_data_if_not_exists_name(session, default_tasks, Task)

    print("✅ 預設資料初始化完成")


def insert_data_if_not_exists_name(session, datas, model):
    """檢查是否已存在預設資料，如果不存在則插入"""
    for data in datas:
        data_name = data.get("name")
        exists = session.exec(select(model).where(
            model.name == data_name)).first()
        if not exists:
            session.add(model(**data))

    session.commit()


def insert_data_if_not_exists_name_and_not_exists_id(session, datas, model):
    """檢查是否已存在預設資料，如果不存在則插入"""
    for data in datas:
        data_id = data.get("id")  # 安全地取得 id，如果不存在會是 None
        data_name = data.get("name")
        if not data_id:
            exists = session.exec(select(model).where(
                model.name == data_name)).first()
        else:
            exists = session.exec(select(model).where(
                model.name == data_name or model.id == data_id)).first()
        if not exists:
            session.add(model(**data))

    session.commit()


def main(args=None):

    db_url_agvc = 'postgresql+psycopg2://agvc:password@192.168.100.254/agvc'

    # 使用 SQLModel metadata 建立資料表
    pool_agvc = ConnectionPoolManager(db_url_agvc, 1)

    # 初始化預設資料
    initialize_default_data(pool_agvc)

    pool_agvc.shutdown()


if __name__ == "__main__":
    main()
