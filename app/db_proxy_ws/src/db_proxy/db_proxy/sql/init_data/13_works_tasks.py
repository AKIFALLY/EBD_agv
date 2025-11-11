"""
13. 工作和任務初始化資料
依賴：房間、貨架、設備等
"""

from db_proxy.models import Work, Task, TaskStatus
from sqlmodel import select
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
    # 改進：逐個檢查並插入，避免批量插入失敗
    for status_data in default_task_status:
        # 檢查 id 是否存在
        exists = session.exec(select(TaskStatus).where(
            TaskStatus.id == status_data["id"])).first()
        if not exists:
            # 再檢查 name 是否存在
            exists = session.exec(select(TaskStatus).where(
                TaskStatus.name == status_data["name"])).first()
            if not exists:
                session.add(TaskStatus(**status_data))
    session.commit()
    print("✅ 任務狀態資料初始化完成")


def initialize_works(session):
    """初始化工作資料"""
    print("⚙️ 初始化工作資料...")

    # 測試預設 Works
    test_work = [
        {"id": 21, "name": "手動純路徑移動",
         "description": "手動純路徑移動", "parameters": {"nodes": []}},
        # opui-操作員呼叫空車和派滿車的工作 平板按下後觸發 ,WCS產生kuka需要的任務後監控任務完成後刪除任務
        {"id": 100001, "name": "opui-call-empty",
         "description": "作業員從opui請求將空Rack派至[人工作業準備區]", "parameters": {"nodes": []}},
        {"id": 100002, "name": "opui-dispatch-full",
         "description": "作業員從opui請求將Rack派至[系統準備派車區]", "parameters": {"nodes": []}},

        # kuka-相關的工作
        # 移動貨架和移動指令無需預定義流程 給指定的nodes(uuid)即可
        {"id": 210001, "name": "kuka-移動",
         "description": "執行指定的from,to(nodes)移動至指定置",
         "parameters": {"model": "KUKA400i", "function": "move", "api": "submit_mission", "missionType": "MOVE", "nodes": []}},
        {"id": 220001, "name": "kuka-移動貨架",
         "description": "執行指定的from,to(nodes)將貨架搬至指定位置",
         "parameters": {"model": "KUKA400i", "function": "rack_move", "api": "submit_mission", "missionType": "RACK_MOVE", "nodes": []}},
        # kuka的workflow流程任務觸發
        {"id": 230001, "name": "kuka-流程觸發",
         "description": "執行指定的workflow流程觸發",
         "parameters": {"model": "KUKA400i", "function": "workflow", "api": "submit_mission", "missionType": "MOVE", "templateCode": ""}},


        # 房間2 相關 旋轉貨架指令合併至 CargoAGV放入口傳送箱 , CargoAGV拿出口傳送箱
        # 房間2的入口傳送箱Rack轉180度(用流程的api)
        # {"id": 2000102, "name": "CargoAGV放入口傳送箱",
        # "description": "從料架拿carrier到入口傳送箱放", "parameters": {"function": "workflow", "api": "submit_mission", "missionType": "MOVE", "templateCode": "W000000003"}},
        # 房間2的出口傳送箱Rack轉180度(用流程的api)
        # {"id": 2000201, "name": "CargoAGV拿出口傳送箱",
        # "description": "從出口傳送箱拿carrier到料架放", "parameters": {"function": "workflow", "api": "submit_mission", "missionType": "MOVE", "templateCode": "W000000004"}},
        {"id": 2000102, "name": "CargoAGV放入口傳送箱",
         "description": "從料架拿carrier到入口傳送箱放", "parameters": {"function": "rack_move", "api": "submit_mission", "missionType": "RACK_MOVE", "nodes": [20001]}},
        # 房間2的入口傳送箱Rack轉180度 (用RackMove)
        {"id": 2000201, "name": "CargoAGV拿出口傳送箱",
         "description": "從出口傳送箱拿carrier到料架放", "parameters": {"function": "rack_move", "api": "submit_mission", "missionType": "RACK_MOVE", "nodes": [20002]}},
        # 房間2的出口傳送箱Rack轉180度 (用RackMove)

        # 入口傳送箱 - Station-based 編碼 (LoaderAGV 2格為單位)
        {"id": 2010101, "name": "LoaderAGV取入口傳送箱Station01",
         "description": "從入口傳送箱Station01(Port1-2/2格)取到LoaderAGV車上", "parameters": {"nodes": [20101]}},
        {"id": 2010301, "name": "LoaderAGV取入口傳送箱Station03",
         "description": "從入口傳送箱Station03(Port3-4/2格)取到LoaderAGV車上", "parameters": {"nodes": [20101]}},

        # 清洗機 - Station-based 編碼 (LoaderAGV 2格為單位)
        # 上層 Station 01 (Port 1,2) - 只拿, 下層 Station 03 (Port 3,4) - 只放
        {"id": 2030101, "name": "LoaderAGV取清洗機Station01",
         "description": "從清洗機上層Station01(Port1-2/2格)取到LoaderAGV車上", "parameters": {"nodes": [20301]}},
        {"id": 2030302, "name": "LoaderAGV放清洗機Station03",
         "description": "從LoaderAGV車上放到清洗機下層Station03(Port3-4/2格)", "parameters": {"nodes": [20301]}},

        {"id": 2040102, "name": "LoaderAGV放泡藥機A",
         "description": "從LoaderAGV車上放到泡藥機A", "parameters": {"nodes": [20401]}},
        {"id": 2040202, "name": "LoaderAGV放泡藥機B",
         "description": "從LoaderAGV車上放到泡藥機B", "parameters": {"nodes": [20402]}},
        {"id": 2040302, "name": "LoaderAGV放泡藥機C",
         "description": "從LoaderAGV車上放到泡藥機C", "parameters": {"nodes": [20403]}},
        {"id": 2040402, "name": "LoaderAGV放泡藥機D",
         "description": "從LoaderAGV車上放到泡藥機D", "parameters": {"nodes": [20404]}},
        {"id": 2040502, "name": "LoaderAGV放泡藥機E",
         "description": "從LoaderAGV車上放到泡藥機E", "parameters": {"nodes": [20405]}},
        {"id": 2040602, "name": "LoaderAGV放泡藥機F",
         "description": "從LoaderAGV車上放到泡藥機F", "parameters": {"nodes": [20406]}},

        {"id": 2040101, "name": "LoaderAGV拿泡藥機A",
         "description": "從泡藥機A拿到LoaderAGV車上放", "parameters": {"nodes": [20401]}},
        {"id": 2040201, "name": "LoaderAGV拿泡藥機B",
         "description": "從泡藥機B拿到LoaderAGV車上放", "parameters": {"nodes": [20402]}},
        {"id": 2040301, "name": "LoaderAGV拿泡藥機C",
         "description": "從泡藥機C拿到LoaderAGV車上放", "parameters": {"nodes": [20403]}},
        {"id": 2040401, "name": "LoaderAGV拿泡藥機D",
         "description": "從泡藥機D拿到LoaderAGV車上放", "parameters": {"nodes": [20404]}},
        {"id": 2040501, "name": "LoaderAGV拿泡藥機E",
         "description": "從泡藥機E拿到LoaderAGV車上放", "parameters": {"nodes": [20405]}},
        {"id": 2040601, "name": "LoaderAGV拿泡藥機F",
         "description": "從泡藥機F拿到LoaderAGV車上放", "parameters": {"nodes": [20406]}},

        # 預烘機 - Station-based 編碼 (4 個 Station)
        # LoaderAGV: 標準映射 1 station = 2 ports (1次1格共2格操作) - Station 01/03/05/07
        # UnloaderAGV: 自定義映射 1 station = 4 ports (1次2格共4格處理) - Station 01(1,2,5,6)/03(3,4,7,8)
        {"id": 2051101, "name": "UnloaderAGV取預烘Station01",
         "description": "UnloaderAGV從預烘機Station01(Port1-2-5-6/批量4格)取到車上", "parameters": {"nodes": [20503]}},
        {"id": 2050102, "name": "LoaderAGV放預烘Station01",
         "description": "LoaderAGV從車上放到預烘機Station01(Port1-2/單格)", "parameters": {"nodes": [20501]}},
        {"id": 2051301, "name": "UnloaderAGV取預烘Station03",
         "description": "UnloaderAGV從預烘機Station03(Port3-4-7-8/批量4格)取到車上", "parameters": {"nodes": [20504]}},
        {"id": 2050302, "name": "LoaderAGV放預烘Station03",
         "description": "LoaderAGV從車上放到預烘機Station03(Port3-4/單格)", "parameters": {"nodes": [20502]}},
        {"id": 2050502, "name": "LoaderAGV放預烘Station05",
         "description": "LoaderAGV從車上放到預烘機Station05(Port5-6/單格)", "parameters": {"nodes": [20501]}},
        {"id": 2050702, "name": "LoaderAGV放預烘Station07",
         "description": "LoaderAGV從車上放到預烘機Station07(Port7-8/單格)", "parameters": {"nodes": [20502]}},

        # 烤箱 - Station-based 編碼
        # UnloaderAGV: 自定義映射 1 station = 4 ports (批量處理)
        # Station 01: Port 1-2-3-4 (批量4格/上排/只拿), Station 05: Port 5-6-7-8 (批量4格/下排/只放)
        {"id": 2060101, "name": "UnloaderAGV取烤箱Station01",
         "description": "UnloaderAGV從烤箱Station01(Port1-2-3-4/批量4格/上排)取到車上", "parameters": {"nodes": [20601]}},
        {"id": 2060502, "name": "UnloaderAGV放烤箱Station05",
         "description": "UnloaderAGV從車上放到烤箱Station05(Port5-6-7-8/批量4格/下排)", "parameters": {"nodes": [20601]}},

        # 出口傳送箱 - Station-based 編碼
        # UnloaderAGV: 自定義映射 1 station = 4 ports (批量處理)
        {"id": 2020102, "name": "UnloaderAGV放出口傳送箱Station01",
         "description": "從UnloaderAGV車上放到出口傳送箱Station01(Port1-2-3-4/批量4格)", "parameters": {"nodes": [20201]}},
    ]

    # 改進：逐個檢查並插入，避免批量插入失敗
    for work_data in test_work:
        # 檢查 id 是否存在
        exists = session.exec(select(Work).where(
            Work.id == work_data["id"])).first()
        if not exists:
            # 再檢查 name 是否存在
            exists = session.exec(select(Work).where(
                Work.name == work_data["name"])).first()
            if not exists:
                work = Work(**work_data)
                session.add(work)
                # 立即 flush 以確保 ID 分配
                session.flush()
    # 最終 commit 所有工作
    session.commit()
    print("✅ 工作資料初始化完成")


def initialize_tasks(session):
    """初始化任務資料"""
    print("📝 初始化任務資料...")
    
    # 先確認必要的 work 存在
    work_100001 = session.exec(select(Work).where(Work.id == 100001)).first()
    work_100002 = session.exec(select(Work).where(Work.id == 100002)).first()
    
    if not work_100001 or not work_100002:
        print("⚠️ 警告: work_id 100001 或 100002 不存在，跳過相關任務初始化")

    default_tasks = [ ]
    # 測試預設 Tasks - 使用實際存在的 work_id
    #default_tasks = [
    #    {"work_id": 2000102, "status_id": 1, "room_id": 2, "name": "測試任務1", "description": "測試任務1",
    #        "agv_id": 1, "priority": 9, "parameters": {"room_id": 2, "rack_id": 1, "eqp_id": 1, "function": "workflow", "model": "KUKA400i", "templateCode": "W000000001"}},
    #    {"work_id": 2000201, "status_id": 1, "room_id": 2, "name": "測試任務2", "description": "測試任務2",
    #        "agv_id": 1, "priority": 9, "parameters": {"room_id": 2, "rack_id": 2, "eqp_id": 2, "function": "workflow", "model": "KUKA400i", "templateCode": "W000000001"}},
    #    {"work_id": 2010101, "status_id": 2, "room_id": 2, "name": "測試任務-執行中任務", "description": "測試UI上正在執行的任務顯示",
    #        "agv_id": 2, "priority": 5, "parameters": {"room_id": 2, "rack_id": 3, "eqp_id": 3}},
    #    {"work_id": 2030102, "status_id": 3, "room_id": 2, "name": "測試任務-已完成任務", "description": "測試UI上已完成的任務顯示",
    #        "agv_id": 3, "priority": 1, "parameters": {"room_id": 2, "rack_id": 4, "eqp_id": 4}},
#
#
    #    # opui 叫空車測試任務
    #    {"work_id": 100001, "status_id": 1, "room_id": 2,
    #     "name": "叫空車 - 停車位 [node_id]", "description": "測試opui-操作員從機台 machine_id 叫空車到停車位 [node_id]",
    #     "node_id": 95, "agv_id": 3, "priority": 1,
    #     "parameters": {
    #         "node_id": 95,
    #         "machine_id": 1,
    #         "client_id": "clientId",
    #         "task_type": "call_empty",
    #         "model": "KUKA400i",
    #         "kuka_agv_id": 123,
    #         "function": "rack_move",
    #         "nodes": [91, 76, 91]
    #     }},
    #    # opui 派滿車測試任務
    #    {"work_id": 100002, "status_id": 1, "room_id": 2,
    #     "name": "派滿車 - product_name x count 到停車位 [node_id]", "description": "測試opui-操作員從機台 machine_id 派滿車，產品: product_name,數量: count,目標停車位: [node_id]",
    #     "node_id": 95, "agv_id": 3, "priority": 1,
    #     "parameters": {
    #         "node_id": 95,
    #         "product_name": "ABC121345",
    #         "count": 32,
    #         "rack_id": 1,
    #         "room": 2,
    #         "side": "left",  # op1 left, op2 right
    #         "machine_id": 1,
    #         "client_id": "clientId",
    #         "task_type": "dispatch_full",
    #         "model": "KUKA400i",
    #         "kuka_agv_id": 123,
    #         "function": "rack_move",
    #         "nodes": [75, 74, 72, 75]
    #     }},
#
    #    
    #    # 未指派mission_code及agv_id 時才可由rcs指定kuka agv來執行
    #    # KUKA AGV 旋轉貨架
    #    # {"work_id": 230101, "status_id": 1, "room_id": 2, "name": "kuka-房間1入口轉貨架", "description": "執行房間1入口轉貨架流程workflow",
    #    #    "agv_id": 123, "priority": 1, "parameters": {"model": "KUKA400i", "templateCode": "W000000001"}},
    #    # KUKA AGV 移動貨架(nodes為str list時直接將position指定uuid,為int list時使用預設map layer-distrct-nodenumber 組成position)
    #    {"work_id": 220001, "status_id": 1, "room_id": 2, "name": "kuka-移動貨架", "description": "執行移動貨架(參數提供2個以上node)第1點為拿起位置及最後1點為目標位置",
    #        "agv_id": 123, "priority": 1, "parameters": {"model": "KUKA400i", "nodes": [91, 76, 91]}},
    #    # KUKA AGV 移動
    #    {"work_id": 210001, "status_id": 1, "room_id": 2, "name": "kuka-移動", "description": "執行移動(參數提供2個以上node)依序經過所有nodes",
    #        "agv_id": 123, "priority": 1, "parameters": {"model": "KUKA400i", "nodes": [75, 74, 72, 75]}},
    #]

    # 改進：逐個檢查並插入，避免批量插入失敗
    # 使用 no_autoflush 避免過早的 flush
    with session.no_autoflush:
        for task_data in default_tasks:
            # 檢查對應的 work_id 是否存在
            work_exists = session.exec(select(Work).where(
                Work.id == task_data.get("work_id"))).first()
            
            if not work_exists:
                print(f"⚠️ 跳過任務 '{task_data.get('name')}': work_id {task_data.get('work_id')} 不存在")
                continue
                
            # Task 沒有固定的 id，用 name 檢查
            exists = session.exec(select(Task).where(
                Task.name == task_data["name"])).first()
            if not exists:
                task = Task(**task_data)
                session.add(task)
                session.flush()  # 確保任務被插入
    
    session.commit()
    print("✅ 任務資料初始化完成")
