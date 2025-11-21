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
        {"id": 1, "location_status_id": 1, "room_id": 1, "node_id": 1, "name": "未知空位1", "description": "測試用未知位置"},

        # 設備對應的位置資料 (對應 equipment 初始資料中的 location_id)
        {"id": 20100, "name": "Room2_BoxIn_Location", "description": "房間2 入口傳送箱位置"},
        {"id": 20200, "name": "Room2_BoxOut_Location", "description": "房間2 出口傳送箱位置"},
        {"id": 20300, "name": "Room2_Cleaner_Location", "description": "房間2 清洗機位置"},
        {"id": 20400, "name": "Room2_Soaking_Location", "description": "房間2 泡藥機群組位置"},
        {"id": 20500, "name": "Room2_Dryer_Location", "description": "房間2 預烘機位置"},
        {"id": 20600, "name": "Room2_Oven_Location", "description": "房間2 烤箱位置"},
        {"id": 21000, "name": "LoaderAGV_Location", "description": "LoaderAGV 設備位置"},
        {"id": 21100, "name": "UnloaderAGV_Location", "description": "UnloaderAGV 設備位置"},

        # 射出機1工作區 (101-106)
        {"id": 101, "location_status_id": 1, "name": "工作區101", "description": "射出機1工作區1", "type": "op_work_space"},
        {"id": 102, "location_status_id": 1, "name": "工作區102", "description": "射出機1工作區2", "type": "op_work_space"},
        {"id": 103, "location_status_id": 1, "name": "工作區103", "description": "射出機1工作區3", "type": "op_work_space"},
        {"id": 104, "location_status_id": 1, "name": "工作區104", "description": "射出機1工作區4", "type": "op_work_space"},
        {"id": 105, "location_status_id": 1, "name": "工作區105", "description": "射出機1工作區5", "type": "op_work_space"},
        {"id": 106, "location_status_id": 1, "name": "工作區106", "description": "射出機1工作區6", "type": "op_work_space"},

        # 射出機2工作區 (201-206)
        {"id": 201, "location_status_id": 1, "name": "工作區201", "description": "射出機2工作區1", "type": "op_work_space"},
        {"id": 202, "location_status_id": 1, "name": "工作區202", "description": "射出機2工作區2", "type": "op_work_space"},
        {"id": 203, "location_status_id": 1, "name": "工作區203", "description": "射出機2工作區3", "type": "op_work_space"},
        {"id": 204, "location_status_id": 1, "name": "工作區204", "description": "射出機2工作區4", "type": "op_work_space"},
        {"id": 205, "location_status_id": 1, "name": "工作區205", "description": "射出機2工作區5", "type": "op_work_space"},
        {"id": 206, "location_status_id": 1, "name": "工作區206", "description": "射出機2工作區6", "type": "op_work_space"},

        # 射出機3工作區 (301-306)
        {"id": 301, "location_status_id": 1, "name": "工作區301", "description": "射出機3工作區1", "type": "op_work_space"},
        {"id": 302, "location_status_id": 1, "name": "工作區302", "description": "射出機3工作區2", "type": "op_work_space"},
        {"id": 303, "location_status_id": 1, "name": "工作區303", "description": "射出機3工作區3", "type": "op_work_space"},
        {"id": 304, "location_status_id": 1, "name": "工作區304", "description": "射出機3工作區4", "type": "op_work_space"},
        {"id": 305, "location_status_id": 1, "name": "工作區305", "description": "射出機3工作區5", "type": "op_work_space"},
        {"id": 306, "location_status_id": 1, "name": "工作區306", "description": "射出機3工作區6", "type": "op_work_space"},

        # 射出機4工作區 (401-406)
        {"id": 401, "location_status_id": 1, "name": "工作區401", "description": "射出機4工作區1", "type": "op_work_space"},
        {"id": 402, "location_status_id": 1, "name": "工作區402", "description": "射出機4工作區2", "type": "op_work_space"},
        {"id": 403, "location_status_id": 1, "name": "工作區403", "description": "射出機4工作區3", "type": "op_work_space"},
        {"id": 404, "location_status_id": 1, "name": "工作區404", "description": "射出機4工作區4", "type": "op_work_space"},
        {"id": 405, "location_status_id": 1, "name": "工作區405", "description": "射出機4工作區5", "type": "op_work_space"},
        {"id": 406, "location_status_id": 1, "name": "工作區406", "description": "射出機4工作區6", "type": "op_work_space"},

        {"id": 15, "location_status_id": 1, "node_id": 15, "name": "射出機1-停車位置1", "description": "射出機1-OP1叫車/停車位置"},
        {"id": 14, "location_status_id": 1, "node_id": 14, "name": "射出機1-停車位置2", "description": "射出機1-OP2叫車/停車位置"},
        {"id": 25, "location_status_id": 1, "node_id": 25, "name": "射出機2-停車位置1", "description": "射出機2-OP1叫車/停車位置"},
        {"id": 23, "location_status_id": 1, "node_id": 23, "name": "射出機2-停車位置2", "description": "射出機2-OP2叫車/停車位置"},
        {"id": 46, "location_status_id": 1, "node_id": 46, "name": "射出機3-停車位置1", "description": "射出機3-OP1叫車/停車位置"},
        {"id": 44, "location_status_id": 1, "node_id": 44, "name": "射出機3-停車位置2", "description": "射出機3-OP2叫車/停車位置"},
        {"id": 47, "location_status_id": 1, "node_id": 47, "name": "射出機4-停車位置1", "description": "射出機4-OP1叫車/停車位置"},
        {"id": 45, "location_status_id": 1, "node_id": 45, "name": "射出機4-停車位置2", "description": "射出機4-OP2叫車/停車位置"},

        # Room2 KUKA 停靠位置 (2025-11-10 新增)
        {"id": 26, "location_status_id": 1, "room_id": 2, "node_id": 26, "rotation_node_id": 86, "waypoint_node_id": 89, "name": "房間2出口(KUKA)", "description": "房間2出口KUKA停靠位置", "type": "room_outlet"},
        {"id": 27, "location_status_id": 1, "room_id": 2, "node_id": 27, "rotation_node_id": 87, "waypoint_node_id": 90, "name": "房間2入口(KUKA)", "description": "房間2入口KUKA停靠位置", "type": "room_inlet"},

        # 房間2 內部位置 
        {"id": 20101, "location_status_id": 1, "room_id": 2, "node_id": 20101, "name": "LoaderAGV Loader Box", "description": "房間內入口傳送箱"},
        {"id": 20301, "location_status_id": 1, "room_id": 2, "node_id": 20301, "name": "LoaderAGV Washer Inport", "description": "清洗機入口"},
        {"id": 20302, "location_status_id": 1, "room_id": 2, "node_id": 20302, "name": "LoaderAGV Washer Outport", "description": "清洗機出口"},
        {"id": 20401, "location_status_id": 1, "room_id": 2, "node_id": 20401, "name": "LoaderAGV Soaker A", "description": "泡藥機A"},
        {"id": 20402, "location_status_id": 1, "room_id": 2, "node_id": 20402, "name": "LoaderAGV Soaker B", "description": "泡藥機B"},
        {"id": 20403, "location_status_id": 1, "room_id": 2, "node_id": 20403, "name": "LoaderAGV Soaker C", "description": "泡藥機C"},
        {"id": 20404, "location_status_id": 1, "room_id": 2, "node_id": 20404, "name": "LoaderAGV Soaker D", "description": "泡藥機D"},
        {"id": 20405, "location_status_id": 1, "room_id": 2, "node_id": 20405, "name": "LoaderAGV Soaker E", "description": "泡藥機E"},
        {"id": 20406, "location_status_id": 1, "room_id": 2, "node_id": 20406, "name": "LoaderAGV Soaker F", "description": "泡藥機F"},
        {"id": 20501, "location_status_id": 1, "room_id": 2, "node_id": 20501, "name": "LoaderAGV Prebaker 1", "description": "預烘機A面A12門Port1"},
        {"id": 20502, "location_status_id": 1, "room_id": 2, "node_id": 20502, "name": "LoaderAGV Prebaker 2", "description": "預烘機A面A12門Port2"},
        {"id": 20503, "location_status_id": 1, "room_id": 2, "node_id": 20503, "name": "LoaderAGV Prebaker 3", "description": "預烘機A面A34門Port3"},
        {"id": 20504, "location_status_id": 1, "room_id": 2, "node_id": 20504, "name": "LoaderAGV Prebaker 4", "description": "預烘機A面A34門Port4"},
        {"id": 20505, "location_status_id": 1, "room_id": 2, "node_id": 20505, "name": "LoaderAGV Prebaker 5", "description": "預烘機A面A56門Port5"},
        {"id": 20506, "location_status_id": 1, "room_id": 2, "node_id": 20506, "name": "LoaderAGV Prebaker 6", "description": "預烘機A面A56門Port6"},
        {"id": 20507, "location_status_id": 1, "room_id": 2, "node_id": 20507, "name": "LoaderAGV Prebaker 7", "description": "預烘機A面A78門Port7"},
        {"id": 20508, "location_status_id": 1, "room_id": 2, "node_id": 20508, "name": "LoaderAGV Prebaker 8", "description": "預烘機A面A78門Port8"},
        {"id": 20509, "location_status_id": 1, "room_id": 2, "node_id": 20509, "name": "UnloaderAGV Prebaker A", "description": "預烘機B面B12門(Port1-2)"},
        {"id": 20510, "location_status_id": 1, "room_id": 2, "node_id": 20510, "name": "UnloaderAGV Prebaker B", "description": "預烘機B面B34門(Port3-4)"},
        {"id": 20511, "location_status_id": 1, "room_id": 2, "node_id": 20511, "name": "UnloaderAGV Prebaker C", "description": "預烘機B面B56門(Port5-6)"},
        {"id": 20512, "location_status_id": 1, "room_id": 2, "node_id": 20512, "name": "UnloaderAGV Prebaker D", "description": "預烘機B面B78門(Port7-8)"},
        {"id": 20601, "location_status_id": 1, "room_id": 2, "node_id": 20601, "name": "UnloaderAGV Baker", "description": "烤箱A"},
        {"id": 20602, "location_status_id": 1, "room_id": 2, "node_id": 20602, "name": "UnloaderAGV Baker", "description": "烤箱B"},
        {"id": 20201, "location_status_id": 1, "room_id": 2, "node_id": 20201, "name": "UnloaderAGV Unload Box", "description": "房間內出口傳送箱"},

        # 區域定義 (2025-10-29 更新: 重新分配位置ID, location_status_id=2 未佔用)
        # 系統準備區 (改為 ID 2-9)
        {"id": 2, "location_status_id": 2, "node_id": 2, "name": "SystemReadyArea_1", "description": "系統準備區1"},
        {"id": 3, "location_status_id": 2, "node_id": 3, "name": "SystemReadyArea_2", "description": "系統準備區2"},
        {"id": 4, "location_status_id": 2, "node_id": 4, "name": "SystemReadyArea_3", "description": "系統準備區3"},
        {"id": 5, "location_status_id": 2, "node_id": 5, "name": "SystemReadyArea_4", "description": "系統準備區4"},
        {"id": 6, "location_status_id": 2, "node_id": 6, "name": "SystemReadyArea_5", "description": "系統準備區5"},
        {"id": 7, "location_status_id": 2, "node_id": 7, "name": "SystemReadyArea_6", "description": "系統準備區6"},
        {"id": 8, "location_status_id": 2, "node_id": 8, "name": "SystemReadyArea_7", "description": "系統準備區7"},
        {"id": 9, "location_status_id": 2, "node_id": 9, "name": "SystemReadyArea_8", "description": "系統準備區8"},

        # 系統空車停放區 (改為 ID 11-13, 減少1個位置)
        {"id": 11, "location_status_id": 2, "node_id": 11, "name": "SystemEmptyRackArea_1", "description": "系統空車區9"},
        {"id": 12, "location_status_id": 2, "node_id": 12, "name": "SystemEmptyRackArea_2", "description": "系統空車區10"},
        {"id": 13, "location_status_id": 2, "node_id": 13, "name": "SystemEmptyRackArea_3", "description": "系統空車區11"},

        # 人工收料區 (KUKA WCS 使用)
        {"id": 21, "location_status_id": 2, "node_id": 21, "name": "ManualReceiveArea_1", "description": "人工收料區1"},
        {"id": 22, "location_status_id": 2, "node_id": 22, "name": "ManualReceiveArea_2", "description": "人工收料區2"},
        {"id": 71, "location_status_id": 1, "node_id": 71, "name": "NGRecycleArea_1", "description": "NG回收區"},
        {"id": 72, "location_status_id": 1, "node_id": 72, "name": "NGRecycleArea_2", "description": "NG回收區"},
            
        

    ]

    insert_data_if_not_exists_name(session, default_location, Location)
    print("✅ 位置資料初始化完成")



"""
# 區域定義 (2025-10-29 更新: 重新分配位置ID)
SYSTEM_READY_AREA = [9, 8, 7, 6, 5, 4, 3, 2]  # 系統準備區 (改為 ID 2-9)
SYSTEM_EMPTY_RACK_AREA = [13, 12, 11]  # 系統空車區 (改為 ID 11-13, 減少1個位置)
MANUAL_COLLECTION_AREA = [31001, 31002, 31003, 31004, 31005, 31006, 31007, 31008, 31009, 31010]  # 人工收料區 (Flow WCS)
MANUAL_RECEIVE_AREA = [21,22]  # 手動回收區
NG_RECYCLE_AREA = [71, 72]  # NG回收區(目前棄用)


"""