"""
13. 工作和任務初始化資料
依賴：房間、貨架、設備等
"""

from db_proxy.models import Work, Task, TaskStatus
from ..db_install import insert_data_if_not_exists_name, insert_data_if_not_exists_name_and_not_exists_id


def initialize_task_status(session):
    """初始化任務狀態資料"""
    print("📋 初始化任務狀態資料...")

    default_task_status = [
        # AGV 用 執行中 / 任務完成
        {"id": 0, "name": "請求中", "description": "UI-請求執行任務"},
        {"id": 1, "name": "待處理", "description": "WCS-任務已接受，待處理"},
        {"id": 2, "name": "待執行", "description": "RCS-任務已派發，待執行"},
        {"id": 3, "name": "執行中", "description": "AGV-任務正在執行"},
        {"id": 4, "name": "已完成", "description": "AGV-任務已完成"},

        {"id": 5, "name": "取消中", "description": "任務取消"},
        {"id": 51, "name": "WCS-取消中", "description": "WCS-任務取消中，待處理"},
        {"id": 52, "name": "RCS-取消中", "description": "RCS-任務取消中，取消中"},
        {"id": 53, "name": "AGV-取消中", "description": "AGV-取消完成"},
        {"id": 54, "name": "已取消", "description": "任務已取消"},

        {"id": 6, "name": "錯誤", "description": "錯誤"},
    ]
    insert_data_if_not_exists_name_and_not_exists_id(
        session, default_task_status, TaskStatus)
    print("✅ 任務狀態資料初始化完成")


def initialize_works(session):
    """初始化工作資料"""
    print("⚙️ 初始化工作資料...")

    # 測試預設 Works
    # 測試預設 Works
    test_work = [
        # opui-操作員呼叫空車和派滿車的工作 平板按下後觸發 ,WCS產生kuka需要的任務後監控任務完成後刪除任務
        {"id": 100001, "name": "opui-call-empty",
         "description": "作業員從opui請求將空Rack派至[人工作業準備區]", "parameters": {}},
        {"id": 100002, "name": "opui-dispatch-full",
         "description": "作業員從opui請求將Rack派至[系統準備派車區]", "parameters": {}},

        # kuka-相關的工作
        # 移動貨架和移動指令無需預定義流程 給指定的nodes(uuid)即可
        {"id": 210001, "name": "kuka-移動",
         "description": "執行指定的from,to(nodes)移動至指定置", "parameters": {"function": "move", "api": "submit_mission", "missionType": "MOVE"}},
        {"id": 220001, "name": "kuka-移動貨架",
         "description": "執行指定的from,to(nodes)將貨架搬至指定位置", "parameters": {"function": "rack_move", "api": "submit_mission", "missionType": "RACK_MOVE"}},
        # kuka的workflow流程任務觸發
        {"id": 230001, "name": "kuka-流程觸發",
         "description": "執行指定的workflow流程觸發", "parameters": {"function": "workflow", "api": "submit_mission", "missionType": "MOVE", "templateCode": "W000000001"}},


        # 房間2 相關 旋轉貨架指令合併至 CargoAGV放入口傳送箱 , CargoAGV拿出口傳送箱
        # 房間2的入口傳送箱Rack轉180度(用流程的api)
        # {"id": 2000102, "name": "CargoAGV放入口傳送箱",
        # "description": "從料架拿carrier到入口傳送箱放", "parameters": {"function": "workflow", "api": "submit_mission", "missionType": "MOVE", "templateCode": "W000000003"}},
        # 房間2的出口傳送箱Rack轉180度(用流程的api)
        # {"id": 2000201, "name": "CargoAGV拿出口傳送箱",
        # "description": "從出口傳送箱拿carrier到料架放", "parameters": {"function": "workflow", "api": "submit_mission", "missionType": "MOVE", "templateCode": "W000000004"}},
        {"id": 2000102, "name": "CargoAGV放入口傳送箱",
         "description": "從料架拿carrier到入口傳送箱放", "parameters": {"function": "rack_move", "api": "submit_mission", "missionType": "RACK_MOVE", "nodes": []}},
        # 房間2的入口傳送箱Rack轉180度 (用RackMove)
        {"id": 2000201, "name": "CargoAGV拿出口傳送箱",
         "description": "從出口傳送箱拿carrier到料架放", "parameters": {"function": "rack_move", "api": "submit_mission", "missionType": "RACK_MOVE", "nodes": []}},
        # 房間2的出口傳送箱Rack轉180度 (用RackMove)

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

    insert_data_if_not_exists_name_and_not_exists_id(session, test_work, Work)
    print("✅ 工作資料初始化完成")


def initialize_tasks(session):
    """初始化任務資料"""
    print("📝 初始化任務資料...")

    # 測試預設 Tasks - 使用實際存在的 work_id
    default_tasks = [
        {"work_id": 2000102, "status_id": 1, "room_id": 2, "name": "測試任務1", "description": "測試任務1",
            "agv_id": 1, "priority": 9, "parameters": {"room_id": 2, "rack_id": 1, "eqp_id": 1, "function": "workflow", "model": "KUKA400i", "templateCode": "W000000001"}},
        {"work_id": 2000201, "status_id": 1, "room_id": 2, "name": "測試任務2", "description": "測試任務2",
            "agv_id": 1, "priority": 9, "parameters": {"room_id": 2, "rack_id": 2, "eqp_id": 2, "function": "workflow", "model": "KUKA400i", "templateCode": "W000000001"}},
        {"work_id": 2010101, "status_id": 2, "room_id": 2, "name": "測試任務-執行中任務", "description": "測試UI上正在執行的任務顯示",
            "agv_id": 2, "priority": 5, "parameters": {"room_id": 2, "rack_id": 3, "eqp_id": 3}},
        {"work_id": 2030102, "status_id": 3, "room_id": 2, "name": "測試任務-已完成任務", "description": "測試UI上已完成的任務顯示",
            "agv_id": 3, "priority": 1, "parameters": {"room_id": 2, "rack_id": 4, "eqp_id": 4}},


        # opui 叫空車測試任務
        {"work_id": 100001, "status_id": 1, "room_id": 2,
         "name": "叫空車 - 停車位 [node_id]", "description": "測試opui-操作員從機台 machine_id 叫空車到停車位 [node_id]",
         "node_id": 95, "agv_id": 3, "priority": 1,
         "parameters": {
             "node_id": 95,
             "machine_id": 1,
             "client_id": "clientId",
             "task_type": "call_empty",
             "model": "KUKA400i",
             "kuka_agv_id": 123,
             "function": "rack_move",
             "nodes": [91, 76, 91]
         }},
        # opui 派滿車測試任務
        {"work_id": 100002, "status_id": 1, "room_id": 2,
         "name": "派滿車 - product_name x count 到停車位 [node_id]", "description": "測試opui-操作員從機台 machine_id 派滿車，產品: product_name,數量: count,目標停車位: [node_id]",
         "node_id": 95, "agv_id": 3, "priority": 1,
         "parameters": {
             "node_id": 95,
             "product_name": "ABC121345",
             "count": 32,
             "rack_id": 1,
             "room": 2,
             "side": "left",  # op1 left, op2 right
             "machine_id": 1,
             "client_id": "clientId",
             "task_type": "dispatch_full",
             "model": "KUKA400i",
             "kuka_agv_id": 123,
             "function": "rack_move",
             "nodes": [75, 74, 72, 75]
         }},


        # 未指派mission_code及agv_id 時才可由rcs指定kuka agv來執行
        # KUKA AGV 旋轉貨架
        # {"work_id": 230101, "status_id": 1, "room_id": 2, "name": "kuka-房間1入口轉貨架", "description": "執行房間1入口轉貨架流程workflow",
        #    "agv_id": 123, "priority": 1, "parameters": {"model": "KUKA400i", "templateCode": "W000000001"}},
        # KUKA AGV 移動貨架(nodes為str list時直接將position指定uuid,為int list時使用預設map layer-distrct-nodenumber 組成position)
        {"work_id": 220001, "status_id": 1, "room_id": 2, "name": "kuka-移動貨架", "description": "執行移動貨架(參數提供2個以上node)第1點為拿起位置及最後1點為目標位置",
            "agv_id": 123, "priority": 1, "parameters": {"model": "KUKA400i", "nodes": [91, 76, 91]}},
        # KUKA AGV 移動
        {"work_id": 210001, "status_id": 1, "room_id": 2, "name": "kuka-移動", "description": "執行移動(參數提供2個以上node)依序經過所有nodes",
            "agv_id": 123, "priority": 1, "parameters": {"model": "KUKA400i", "nodes": [75, 74, 72, 75]}},
    ]

    insert_data_if_not_exists_name(session, default_tasks, Task)
    print("✅ 任務資料初始化完成")
