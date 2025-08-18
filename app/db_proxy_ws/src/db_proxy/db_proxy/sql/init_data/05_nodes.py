"""
05. 節點初始化資料
依賴：節點類型
"""

from db_proxy.models import Node
from ..db_install import insert_data_if_not_exists_name_and_not_exists_id


def initialize_nodes(session):
    """初始化節點資料"""
    print("🗺️ 初始化節點資料...")

    default_nodes = [
        # 基礎測試節點 (被 location 資料參考)
        {"id": 1, "name": "測試節點1",
            "description": "測試用節點1", "x": 0.0, "y": 0.0},
        {"id": 2, "name": "測試節點2",
            "description": "測試用節點2", "x": 0.0, "y": 0.0},
        
        {"id": 95, "name": "射出機1-停車位1",
            "description": "射出機1-停車位置1", "x": 0.0, "y": 0.0},
        {"id": 96, "name": "射出機1-停車位2",
            "description": "射出機1-停車位置2", "x": 0.0, "y": 0.0},
        {"id": 97, "name": "射出機2-停車位1",
            "description": "射出機2-停車位置1", "x": 0.0, "y": 0.0},
        {"id": 98, "name": "射出機2-停車位2",
            "description": "射出機2-停車位置2", "x": 0.0, "y": 0.0},
        {"id": 1005, "name": "射出機3-停車位1",
            "description": "射出機3-停車位置1", "x": 0.0, "y": 0.0},
        {"id": 1006, "name": "射出機3-停車位2",
            "description": "射出機3-停車位置2", "x": 0.0, "y": 0.0},
        {"id": 1007, "name": "射出機4-停車位1",
            "description": "射出機4-停車位置1", "x": 0.0, "y": 0.0},
        {"id": 1008, "name": "射出機4-停車位2",
            "description": "射出機4-停車位置2", "x": 0.0, "y": 0.0},

        {"id": 10001, "name": "room01 Loader Box",
            "description": "房間1入口傳送箱", "x": 0.0, "y": 0.0},
        {"id": 10002, "name": "room01 Unloader Box",
            "description": "房間1出口傳送箱", "x": 0.0, "y": 0.0},

        {"id": 20001, "name": "room02 Loader Box",
            "description": "房間2入口傳送箱", "x": 0.0, "y": 0.0},
        {"id": 20002, "name": "room02 Unloader Box",
            "description": "房間2出口傳送箱", "x": 0.0, "y": 0.0},

        # 設備對應的節點資料 (對應 equipment 初始資料中的 location_id)
        {"id": 201, "name": "Room2_BoxIn_Node",
            "description": "房間2 入口傳送箱節點", "x": 0.0, "y": 0.0},
        {"id": 202, "name": "Room2_BoxOut_Node",
            "description": "房間2 出口傳送箱節點", "x": 0.0, "y": 0.0},
        {"id": 203, "name": "Room2_Cleaner_Node",
            "description": "房間2 清洗機節點", "x": 0.0, "y": 0.0},
        {"id": 204, "name": "Room2_Soaking_Node",
            "description": "房間2 泡藥機群組節點", "x": 0.0, "y": 0.0},
        {"id": 205, "name": "Room2_Dryer_Node",
            "description": "房間2 預烘機節點", "x": 0.0, "y": 0.0},
        {"id": 206, "name": "Room2_Oven_Node",
            "description": "房間2 烤箱節點", "x": 0.0, "y": 0.0},
        {"id": 210, "name": "LoaderAGV_Node",
            "description": "LoaderAGV 設備節點", "x": 0.0, "y": 0.0},
        {"id": 211, "name": "UnloaderAGV_Node",
            "description": "UnloaderAGV 設備節點", "x": 0.0, "y": 0.0},

        {"id": 30001, "name": "room03 Loader Box",
            "description": "房間3入口傳送箱", "x": 0.0, "y": 0.0},
        {"id": 30002, "name": "room03 Unloader Box",
            "description": "房間3出口傳送箱", "x": 0.0, "y": 0.0},

        {"id": 40001, "name": "room04 Loader Box",
            "description": "房間4入口傳送箱", "x": 0.0, "y": 0.0},
        {"id": 40002, "name": "room04 Unloader Box",
            "description": "房間4出口傳送箱", "x": 0.0, "y": 0.0},

        {"id": 50001, "name": "room05 Loader Box",
            "description": "房間5入口傳送箱", "x": 0.0, "y": 0.0},
        {"id": 50002, "name": "room05 Unloader Box",
            "description": "房間5出口傳送箱", "x": 0.0, "y": 0.0},

        {"id": 20101, "name": "LoaderAGV Loader Box",
            "description": "房間內入口傳送箱", "x": 0.0, "y": 0.0},

        {"id": 20301, "name": "LoaderAGV Washer Inport",
            "description": "清洗機入口", "x": 0.0, "y": 0.0},
        {"id": 20302, "name": "LoaderAGV Washer Outport",
            "description": "清洗機出口", "x": 0.0, "y": 0.0},

        {"id": 20401, "name": "LoaderAGV Soaker A",
            "description": "泡藥機A", "x": 0.0, "y": 0.0},
        {"id": 20402, "name": "LoaderAGV Soaker B",
            "description": "泡藥機B", "x": 0.0, "y": 0.0},
        {"id": 20403, "name": "LoaderAGV Soaker C",
            "description": "泡藥機C", "x": 0.0, "y": 0.0},
        {"id": 20404, "name": "LoaderAGV Soaker D",
            "description": "泡藥機D", "x": 0.0, "y": 0.0},
        {"id": 20405, "name": "LoaderAGV Soaker E",
            "description": "泡藥機E", "x": 0.0, "y": 0.0},
        {"id": 20406, "name": "LoaderAGV Soaker F",
            "description": "泡藥機F", "x": 0.0, "y": 0.0},

        {"id": 20501, "name": "LoaderAGV Prebaker 1",
            "description": "預烘機A1", "x": 0.0, "y": 0.0},
        {"id": 20502, "name": "LoaderAGV Prebaker 2",
            "description": "預烘機A2", "x": 0.0, "y": 0.0},
        {"id": 20503, "name": "LoaderAGV Prebaker 3",
            "description": "預烘機B1", "x": 0.0, "y": 0.0},
        {"id": 20504, "name": "LoaderAGV Prebaker 4",
            "description": "預烘機B2", "x": 0.0, "y": 0.0},
        {"id": 20505, "name": "LoaderAGV Prebaker 5",
            "description": "預烘機C1", "x": 0.0, "y": 0.0},
        {"id": 20506, "name": "LoaderAGV Prebaker 6",
            "description": "預烘機C2", "x": 0.0, "y": 0.0},
        {"id": 20507, "name": "LoaderAGV Prebaker 7",
            "description": "預烘機D1", "x": 0.0, "y": 0.0},
        {"id": 20508, "name": "LoaderAGV Prebaker 8",
            "description": "預烘機D2", "x": 0.0, "y": 0.0},

        {"id": 20509, "name": "UnloaderAGV Prebaker A",
            "description": "預烘機A", "x": 0.0, "y": 0.0},
        {"id": 20510, "name": "UnloaderAGV Prebaker B",
            "description": "預烘機B", "x": 0.0, "y": 0.0},
        {"id": 20511, "name": "UnloaderAGV Prebaker C",
            "description": "預烘機C", "x": 0.0, "y": 0.0},
        {"id": 20512, "name": "UnloaderAGV Prebaker D",
            "description": "預烘機D", "x": 0.0, "y": 0.0},

        {"id": 20601, "name": "UnloaderAGV Baker",
            "description": "烤箱A", "x": 0.0, "y": 0.0},
        {"id": 20602, "name": "UnloaderAGV Baker",
            "description": "烤箱B", "x": 0.0, "y": 0.0},

        {"id": 20201, "name": "UnloaderAGV Unload Box",
            "description": "房間內出口傳送箱", "x": 0.0, "y": 0.0},

        # Flow WCS 系統使用 - 傳送箱出口節點
        {"id": 20003, "name": "傳送箱出口_03",
            "description": "傳送箱出口節點3", "x": 0.0, "y": 0.0},
        {"id": 20004, "name": "傳送箱出口_04",
            "description": "傳送箱出口節點4", "x": 0.0, "y": 0.0},
        {"id": 20005, "name": "傳送箱出口_05",
            "description": "傳送箱出口節點5", "x": 0.0, "y": 0.0},

        # 區域節點
        {"id": 11, "name": "SystemReadyArea_11",
            "description": "系統準備區節點", "x": 0.0, "y": 0.0},
        {"id": 12, "name": "SystemReadyArea_12",
            "description": "系統準備區節點", "x": 0.0, "y": 0.0},
        {"id": 13, "name": "SystemReadyArea_13",
            "description": "系統準備區節點", "x": 0.0, "y": 0.0},
        {"id": 14, "name": "SystemReadyArea_14",
            "description": "系統準備區節點", "x": 0.0, "y": 0.0},
        {"id": 15, "name": "SystemReadyArea_15",
            "description": "系統準備區節點", "x": 0.0, "y": 0.0},
        {"id": 16, "name": "SystemReadyArea_16",
            "description": "系統準備區節點", "x": 0.0, "y": 0.0},
        {"id": 17, "name": "SystemReadyArea_17",
            "description": "系統準備區節點", "x": 0.0, "y": 0.0},
        {"id": 18, "name": "SystemReadyArea_18",
            "description": "系統準備區節點", "x": 0.0, "y": 0.0},

        {"id": 31, "name": "SystemEmptyRackArea_1",
            "description": "系統空車區節點", "x": 0.0, "y": 0.0},
        {"id": 32, "name": "SystemEmptyRackArea_2",
            "description": "系統空車區節點", "x": 0.0, "y": 0.0},
        {"id": 33, "name": "SystemEmptyRackArea_3",
            "description": "系統空車區節點", "x": 0.0, "y": 0.0},
        {"id": 34, "name": "SystemEmptyRackArea_4",
            "description": "系統空車區節點", "x": 0.0, "y": 0.0},

        # 人工收料區節點 - Flow WCS 系統使用
        {"id": 31001, "name": "人工收料區_01",
            "description": "人工收料區節點1", "x": 0.0, "y": 0.0},
        {"id": 31002, "name": "人工收料區_02",
            "description": "人工收料區節點2", "x": 0.0, "y": 0.0},
        {"id": 31003, "name": "人工收料區_03",
            "description": "人工收料區節點3", "x": 0.0, "y": 0.0},
        {"id": 31004, "name": "人工收料區_04",
            "description": "人工收料區節點4", "x": 0.0, "y": 0.0},
        {"id": 31005, "name": "人工收料區_05",
            "description": "人工收料區節點5", "x": 0.0, "y": 0.0},
        {"id": 31006, "name": "人工收料區_06",
            "description": "人工收料區節點6", "x": 0.0, "y": 0.0},
        {"id": 31007, "name": "人工收料區_07",
            "description": "人工收料區節點7", "x": 0.0, "y": 0.0},
        {"id": 31008, "name": "人工收料區_08",
            "description": "人工收料區節點8", "x": 0.0, "y": 0.0},
        {"id": 31009, "name": "人工收料區_09",
            "description": "人工收料區節點9", "x": 0.0, "y": 0.0},
        {"id": 31010, "name": "人工收料區_10",
            "description": "人工收料區節點10", "x": 0.0, "y": 0.0},

        # 手動回收區節點
        {"id": 51, "name": "ManualReceiveArea_1",
            "description": "手動回收區節點", "x": 0.0, "y": 0.0},
        {"id": 52, "name": "ManualReceiveArea_2",
            "description": "手動回收區節點", "x": 0.0, "y": 0.0},
        {"id": 53, "name": "ManualReceiveArea_3",
            "description": "手動回收區節點", "x": 0.0, "y": 0.0},
        {"id": 54, "name": "ManualReceiveArea_4",
            "description": "手動回收區節點", "x": 0.0, "y": 0.0},
        {"id": 55, "name": "ManualReceiveArea_5",
            "description": "手動回收區節點", "x": 0.0, "y": 0.0},

        {"id": 71, "name": "NGRecycleArea_1",
            "description": "NG回收區節點", "x": 0.0, "y": 0.0},
        {"id": 72, "name": "NGRecycleArea_2",
            "description": "NG回收區節點", "x": 0.0, "y": 0.0},
    ]

    insert_data_if_not_exists_name_and_not_exists_id(session, default_nodes, Node)
    print("✅ 節點資料初始化完成")
