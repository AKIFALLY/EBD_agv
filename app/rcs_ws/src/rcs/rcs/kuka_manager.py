"""
KUKA 統一管理器
整合所有 KUKA 相關功能：機器人狀態更新、容器管理、任務派發
"""
import uuid
from itertools import zip_longest
from db_proxy.connection_pool_manager import ConnectionPoolManager
from kuka_fleet_adapter.kuka_fleet_adapter import KukaFleetAdapter
from rcs.kuka_container import KukaContainer
from rcs.kuka_robot import KukaRobot


# KUKA 工作類型常數 (參考 init_work.py)
class KukaWorkType:
    """KUKA 工作類型常數定義"""
    KUKA_MOVE = 210001          # kuka-移動
    KUKA_RACK_MOVE = 220001     # kuka-移動貨架
    KUKA_WORKFLOW = 230001      # kuka-workflow

    @classmethod
    def get_work_name(cls, work_id: int) -> str:
        """取得工作類型名稱"""
        work_names = {
            cls.KUKA_MOVE: "KUKA移動",
            cls.KUKA_RACK_MOVE: "KUKA移動貨架",
            cls.KUKA_WORKFLOW: "KUKA工作流程"
        }
        return work_names.get(work_id, f"未知工作類型({work_id})")

    @classmethod
    def is_kuka_work(cls, work_id: int) -> bool:
        """檢查是否為 KUKA 工作類型"""
        return work_id in [cls.KUKA_MOVE, cls.KUKA_RACK_MOVE, cls.KUKA_WORKFLOW]

    @classmethod
    def get_work_type_by_function(cls, function: str) -> int:
        """根據 function 參數取得對應的工作類型"""
        function_map = {
            "move": cls.KUKA_MOVE,
            "rack_move": cls.KUKA_RACK_MOVE,
            "workflow": cls.KUKA_WORKFLOW
        }
        return function_map.get(function, cls.KUKA_WORKFLOW)  # 預設為 workflow


class KukaManager:
    """KUKA 統一管理器類別"""

    def __init__(self, rcs_core_node):
        """
        初始化 KUKA 管理器

        Args:
            rcs_core_node: RCS Core 節點實例
        """
        self.rcs_core = rcs_core_node
        self.logger = rcs_core_node.get_logger()
        self.db_pool: ConnectionPoolManager = rcs_core_node.db_pool

        # 創建並管理 KukaFleetAdapter
        self.kuka_fleet = KukaFleetAdapter(rcs_core_node)

        # 初始化 KUKA 相關管理器
        self.kuka_robot = KukaRobot(rcs_core_node)
        self.kuka_container = KukaContainer(rcs_core_node)

        # 設定回調函數
        self.kuka_fleet.on_robot_query_complete = self.kuka_robot.on_robot_update
        self.kuka_fleet.on_container_query_complete = self.kuka_container.on_container_update

        # 載入 KUKA AGV 資料
        self.kuka_agvs = {}
        self._load_kuka_agvs()

        # 啟動監控
        self.kuka_fleet.start_monitoring()

        # 測試：查詢閒置的 KUKA400i AGV
        idle_kuka400i_agvs = self.kuka_fleet.select_agv(
            KukaFleetAdapter.STATUS_IDLE)
        idle_kuka400i_agv_ids = [agv["id"] for agv in idle_kuka400i_agvs]
        self.logger.info(f"閒置的 KUKA400i AGV: {idle_kuka400i_agv_ids}")

    def _load_kuka_agvs(self):
        """從 AGV 資料表取得所有 enable 且 model 為 KUKA400i 的 AGV，儲存於 self.kuka_agvs。"""
        if not self.db_pool:
            self.logger.error("資料庫連線池不可用，無法載入 KUKA AGV。")
            return
        try:
            with self.db_pool.get_session() as session:
                from db_proxy.models import AGV
                from sqlmodel import select
                agvs = session.exec(
                    select(AGV).where(AGV.enable == 1, AGV.model == "KUKA400i")
                ).all()
                self.kuka_agvs = {agv.id: agv for agv in agvs}
                self.logger.info(
                    f"已載入 {len(self.kuka_agvs)} 台 KUKA400i AGV。{self.kuka_agvs}")
        except Exception as e:
            self.logger.error(f"載入 KUKA AGV 時發生錯誤: {e}")

    def dispatch(self):
        """KUKA400i AGV 選車 任務派發"""
        idle_kuka400i_agvs = self.kuka_fleet.select_agv(
            KukaFleetAdapter.STATUS_IDLE)
        idle_kuka400i_agv_ids = [int(agv["id"]) for agv in idle_kuka400i_agvs]

        # self.logger.info(
        #    f"API 查詢到閒置 KUKA400i AGV: {idle_kuka400i_agv_ids}")
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

            # self.logger.info(
            #    f"閒置且可用 KUKA400i AGV: {available_kuka400i_agv_ids}")

            # 查詢 KUKA400i 任務，包含所有 KUKA 工作類型
            kuka400i_tasks = session.exec(
                select(Task).where(
                    Task.status_id == 1,  # 1 待執行
                    Task.mission_code == None,  # 尚未指定任務代碼
                    Task.parameters["Model"].as_string() == "KUKA400i"
                ).order_by(Task.priority.asc())
            ).all()

            # 過濾出真正的 KUKA 任務
            kuka400i_tasks = [
                task for task in kuka400i_tasks
                if KukaWorkType.is_kuka_work(task.work_id)
            ]
            # self.logger.info(
            #    f"KUKA400i 任務: {[task.id for task in kuka400i_tasks]}")

            for agv, task in zip_longest(available_kuka400i_agv_ids, kuka400i_tasks):
                if agv and task:
                    kuka_mission_code = str(uuid.uuid4())
                    work_name = KukaWorkType.get_work_name(task.work_id)

                    self.logger.info(
                        f"預派發任務 {task.id} ({work_name}) 給 AGV {agv}, mission_code: {kuka_mission_code}")

                    # 執行任務
                    res = self._execute_kuka_task(task, agv, kuka_mission_code)

                    # 處理執行結果
                    if res["success"]:
                        task.agv_id = agv
                        task.mission_code = kuka_mission_code
                        task.parameters["agvId"] = agv
                        self.logger.info(
                            f"任務 {task.id} ({work_name}) 已成功派發給 AGV {agv}")
                    else:
                        self.logger.error(f"任務 {task.id} ({work_name}) 派發失敗: {res}")
                elif agv and not task:
                    # self.logger.info(f"AGV {agv} 目前無任務可派發")
                    pass
                elif task and not agv:
                    # self.logger.info(f"任務 {task.id} 目前無可用 AGV")
                    pass
            session.commit()

    def _execute_kuka_task(self, task, agv_id: int, mission_code: str):
        """
        執行 KUKA 任務

        Args:
            task: 任務物件
            agv_id: AGV ID
            mission_code: 任務代碼

        Returns:
            dict: 執行結果
        """
        work_id = task.work_id

        if work_id == KukaWorkType.KUKA_MOVE:
            return self._execute_move_task(task, agv_id, mission_code)
        elif work_id == KukaWorkType.KUKA_RACK_MOVE:
            return self._execute_rack_move_task(task, agv_id, mission_code)
        elif work_id == KukaWorkType.KUKA_WORKFLOW:
            return self._execute_workflow_task(task, agv_id, mission_code)
        else:
            # 未知的工作類型，嘗試作為 workflow 處理
            self.logger.warning(f"未知的 KUKA 工作類型 {work_id}，嘗試作為 workflow 處理")
            return self._execute_workflow_task(task, agv_id, mission_code)

    def _execute_move_task(self, task, agv_id: int, mission_code: str):
        """執行 KUKA 移動任務"""
        nodes = task.parameters.get('nodes')
        if not nodes:
            self.logger.warn(f"KUKA 移動任務 {task.id} 缺少參數 'nodes'")
            return {"success": False, "error": "缺少參數 'nodes'"}

        self.logger.info(f"執行 KUKA 移動任務 - nodes: {nodes}")
        return self.kuka_fleet.move(nodes, agv_id, mission_code)

    def _execute_rack_move_task(self, task, agv_id: int, mission_code: str):
        """執行 KUKA 移動貨架任務"""
        nodes = task.parameters.get('nodes')
        if not nodes:
            self.logger.warn(f"KUKA 移動貨架任務 {task.id} 缺少參數 'nodes'")
            return {"success": False, "error": "缺少參數 'nodes'"}

        self.logger.info(f"執行 KUKA 移動貨架任務 - nodes: {nodes}")
        return self.kuka_fleet.rack_move(nodes, agv_id, mission_code)

    def _execute_workflow_task(self, task, agv_id: int, mission_code: str):
        """執行 KUKA workflow 任務"""
        template_code = task.parameters.get('templateCode')
        if not template_code:
            self.logger.warn(f"KUKA workflow 任務 {task.id} 缺少參數 'templateCode'")
            return {"success": False, "error": "缺少參數 'templateCode'"}

        self.logger.info(f"執行 KUKA workflow 任務 - templateCode: {template_code}")
        return self.kuka_fleet.workflow(template_code, agv_id, mission_code)

    def stop_monitoring(self):
        """停止 KUKA Fleet 監控"""
        if hasattr(self, 'kuka_fleet') and self.kuka_fleet:
            self.kuka_fleet.stop_monitoring()
            self.logger.info("KUKA Fleet 監控已停止")
