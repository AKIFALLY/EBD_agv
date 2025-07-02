"""
08. 位置初始化資料
依賴：位置狀態、房間、節點
"""

from db_proxy.models import Location
from ..db_install import insert_data_if_not_exists_name


def initialize_locations(session):
    """初始化位置資料"""
    print("📍 初始化位置資料...")

    default_location = [
        # 預建置測試資料
        {"id": 1, "location_status_id": 1, "room_id": 1,
            "node_id": 1, "name": "未知空位1", "description": "測試用未知位置"},
        {"id": 2, "location_status_id": 2, "room_id": 2,
            "node_id": 2, "name": "未知空位2", "description": "測試用資料-房間2的固定設備(入口傳送箱及出口傳送箱等等)"},

        # 設備對應的位置資料 (對應 equipment 初始資料中的 location_id)
        {"id": 20100, "room_id": 2,
            "name": "Room2_BoxIn_Location", "description": "房間2 入口傳送箱位置"},
        {"id": 20200, "room_id": 2,
            "name": "Room2_BoxOut_Location", "description": "房間2 出口傳送箱位置"},
        {"id": 20300, "room_id": 2,
            "name": "Room2_Cleaner_Location", "description": "房間2 清洗機位置"},
        {"id": 20400, "room_id": 2,
            "name": "Room2_Soaking_Location", "description": "房間2 泡藥機群組位置"},
        {"id": 20500, "room_id": 2,
            "name": "Room2_Dryer_Location", "description": "房間2 預烘機位置"},
        {"id": 20600, "room_id": 2,
            "name": "Room2_Oven_Location", "description": "房間2 烤箱位置"},
        {"id": 21000, "room_id": 2,
            "name": "LoaderAGV_Location", "description": "LoaderAGV 設備位置"},
        {"id": 21100, "room_id": 2,
            "name": "UnloaderAGV_Location", "description": "UnloaderAGV 設備位置"},

        {"id": 2001, "name": "射出機1-OP1作業位置", "description": "射出機1-OP1作業位置"},
        {"id": 2002, "name": "射出機1-OP2作業位置", "description": "射出機1-OP2作業位置"},
        {"id": 2003, "name": "射出機2-OP1作業位置", "description": "射出機2-OP1作業位置"},
        {"id": 2004, "name": "射出機2-OP2作業位置", "description": "射出機2-OP2作業位置"},
        {"id": 2005, "name": "射出機3-OP1作業位置", "description": "射出機3-OP1作業位置"},
        {"id": 2006, "name": "射出機3-OP2作業位置", "description": "射出機3-OP2作業位置"},
        {"id": 2007, "name": "射出機4-OP1作業位置", "description": "射出機4-OP1作業位置"},
        {"id": 2008, "name": "射出機4-OP2作業位置", "description": "射出機4-OP2作業位置"},

        {"id": 95, "location_status_id": 1,
            "node_id": 95, "name": "射出機1-停車位置1", "description": "射出機1-OP1叫車/停車位置"},
        {"id": 96, "location_status_id": 1,
            "node_id": 96, "name": "射出機1-停車位置2", "description": "射出機1-OP2叫車/停車位置"},
        {"id": 97, "location_status_id": 1,
            "node_id": 97, "name": "射出機2-停車位置1", "description": "射出機2-OP1叫車/停車位置"},
        {"id": 98, "location_status_id": 1,
            "node_id": 98, "name": "射出機2-停車位置2", "description": "射出機2-OP2叫車/停車位置"},
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
    print("✅ 位置資料初始化完成")
