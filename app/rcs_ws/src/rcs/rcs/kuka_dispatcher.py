from rclpy.logging import RcutilsLogger
from db_proxy.connection_pool_manager import ConnectionPoolManager
import uuid
from itertools import zip_longest
from kuka_fleet_adapter.kuka_fleet_adapter import KukaFleetAdapter


class KukaDispatcher:
    def __init__(self, rcs_core):
        self.kuka_fleet: KukaFleetAdapter = rcs_core.kuka_fleet
        self.db_pool: ConnectionPoolManager = rcs_core.db_pool
        self.get_logger: RcutilsLogger = rcs_core.get_logger

    def dispatch(self):
        """KUKA400i AGV 選車 任務派發"""
        idle_kuka400i_agvs = self.kuka_fleet.select_agv(
            KukaFleetAdapter.STATUS_IDLE)
        idle_kuka400i_agv_ids = [int(agv["id"]) for agv in idle_kuka400i_agvs]

        self.get_logger().info(
            f"API 查詢到閒置 KUKA400i AGV: {idle_kuka400i_agv_ids}")
        with self.db_pool.get_session() as session:
            from db_proxy.models import AGV, Task
            from sqlmodel import select
            enable_kuka400i_agvs = session.exec(
                select(AGV).where(
                    AGV.enable == 1,
                    AGV.model == "KUKA400i",
                    AGV.id.in_(idle_kuka400i_agv_ids)
                )
            ).all()
            available_kuka400i_agv_ids = [
                agv.id for agv in enable_kuka400i_agvs]

            self.get_logger().info(
                f"閒置且可用 KUKA400i AGV: {available_kuka400i_agv_ids}")

            kuka400i_tasks = session.exec(
                select(Task).where(
                    Task.status_id == 1,  # 1 待執行 需改為 待旋轉的status_id
                    Task.mission_code == None,  # 尚未指定任務代碼
                    Task.parameters["Model"].as_string() == "KUKA400i"
                ).order_by(Task.priority.asc())
            ).all()
            self.get_logger().info(
                f"KUKA400i 任務: {[task.id for task in kuka400i_tasks]}")

            for agv, task in zip_longest(available_kuka400i_agv_ids, kuka400i_tasks):
                if agv and task:
                    kuka_mission_code = str(uuid.uuid4())
                    self.get_logger().info(
                        f"預派發任務 {task.id} 給 AGV {agv} kuka_mission_code:{kuka_mission_code}")
                    self.get_logger().info(f"work_id: {task.work_id}")
                    res = {"success": False}
                    # 需改由 parameters[function] 內的 function 決定 是flow, move, or rackmove
                    if task.work_id == 210001:
                        self.get_logger().info(
                            f"parameters['nodes']: {task.parameters['nodes']}")
                        if task.parameters['nodes']:
                            res = self.kuka_fleet.move(
                                task.parameters['nodes'], agv, kuka_mission_code)
                        else:
                            self.get_logger().warn(
                                f"缺少參數 parameters['nodes'] 無法執行任務 {task.id}")
                    elif task.work_id == 220001:
                        self.get_logger().info(
                            f"parameters['nodes']: {task.parameters['nodes']}")
                        if task.parameters['nodes']:
                            res = self.kuka_fleet.rack_move(
                                task.parameters['nodes'], agv, kuka_mission_code)
                        else:
                            self.get_logger().warn(
                                f"缺少參數 parameters['nodes'] 無法執行任務 {task.id}")
                    else:
                        templateCode = task.parameters['templateCode']
                        self.get_logger().info(
                            f"templateCode: {templateCode}")
                        if templateCode:
                            res = self.kuka_fleet.workflow(
                                templateCode, agv, kuka_mission_code)
                        else:
                            self.get_logger().warn(
                                f"缺少參數 parameters['templateCode'] 無法執行任務 {task.id}")
                    if res["success"]:
                        task.agv_id = agv
                        task.mission_code = kuka_mission_code
                        task.parameters["agvId"] = agv
                        self.get_logger().info(
                            f"任務 {task.id} 已派發，mission_code: {kuka_mission_code}")
                    else:
                        self.get_logger().info(f"res {res} 建立失敗,{res}")
                elif agv and not task:
                    self.get_logger().info(f"AGV {agv} 目前無任務可派發")
                elif task and not agv:
                    self.get_logger().info(f"任務 {task.id} 目前無可用 AGV")
            session.commit()
