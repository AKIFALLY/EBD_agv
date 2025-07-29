"""
簡化的 KUKA 車隊管理器
基於原始 kuka_dispatcher 的簡潔設計，移除複雜的 WCS 適配和優先度調度
"""
from rclpy.logging import RcutilsLogger
from db_proxy.connection_pool_manager import ConnectionPoolManager
import uuid
from itertools import zip_longest
from kuka_fleet_adapter.kuka_fleet_adapter import KukaFleetAdapter


class KukaManager:
    """簡化的 KUKA 車隊管理器"""
    
    def __init__(self, rcs_core):
        """
        初始化 KUKA 管理器
        
        Args:
            rcs_core: RCS Core 節點實例
        """
        self.rcs_core = rcs_core
        self.kuka_fleet: KukaFleetAdapter = KukaFleetAdapter(rcs_core)
        self.db_pool: ConnectionPoolManager = rcs_core.db_pool
        self.get_logger: RcutilsLogger = rcs_core.get_logger

    def dispatch(self):
        """KUKA400i AGV 簡單任務派發"""
        try:
            # 1. 查詢閒置的 KUKA400i AGV
            idle_kuka400i_agvs = self.kuka_fleet.select_agv(KukaFleetAdapter.STATUS_IDLE)
            idle_kuka400i_agv_ids = [int(agv["id"]) for agv in idle_kuka400i_agvs]

            if not idle_kuka400i_agv_ids:
                # self.get_logger().debug("目前沒有閒置的 KUKA400i AGV")
                return

            self.get_logger().info(f"API 查詢到閒置 KUKA400i AGV: {idle_kuka400i_agv_ids}")
            
            with self.db_pool.get_session() as session:
                from db_proxy.models import AGV, Task
                from sqlmodel import select
                
                # 2. 確認資料庫中的 AGV 狀態
                enable_kuka400i_agvs = session.exec(
                    select(AGV).where(
                        AGV.enable == 1,
                        AGV.model == "KUKA400i",
                        AGV.id.in_(idle_kuka400i_agv_ids)
                    )
                ).all()
                available_kuka400i_agv_ids = [agv.id for agv in enable_kuka400i_agvs]

                if not available_kuka400i_agv_ids:
                    # self.get_logger().debug("資料庫中沒有可用的 KUKA400i AGV")
                    return

                self.get_logger().info(f"閒置且可用 KUKA400i AGV: {available_kuka400i_agv_ids}")

                # 3. 查詢待執行的 KUKA 任務
                kuka400i_tasks = session.exec(
                    select(Task).where(
                        Task.status_id == 1,  # 待執行
                        Task.mission_code == None,  # 尚未指定任務代碼
                        Task.parameters["model"].as_string() == "KUKA400i"  # 使用小寫 model
                    ).order_by(Task.priority.asc())  # 優先級低的數字先執行
                ).all()
                
                if not kuka400i_tasks:
                    # self.get_logger().debug("目前沒有 KUKA400i 任務需要處理")
                    return

                self.get_logger().info(f"KUKA400i 任務: {[task.id for task in kuka400i_tasks]}")

                # 4. 簡單的任務派發邏輯
                for agv, task in zip_longest(available_kuka400i_agv_ids, kuka400i_tasks):
                    if agv and task:
                        success = self._dispatch_task_to_agv(session, task, agv)
                        if success:
                            self.get_logger().info(f"✅ 任務 {task.id} 成功派發給 AGV {agv}")
                        else:
                            self.get_logger().warning(f"❌ 任務 {task.id} 派發給 AGV {agv} 失敗")
                    elif agv and not task:
                        self.get_logger().debug(f"AGV {agv} 目前無任務可派發")
                    elif task and not agv:
                        self.get_logger().debug(f"任務 {task.id} 目前無可用 AGV")

                session.commit()

        except Exception as e:
            self.get_logger().error(f"KUKA 任務派發時發生錯誤: {e}", exc_info=True)

    def _dispatch_task_to_agv(self, session, task, agv_id: int) -> bool:
        """
        派發任務給指定 AGV
        
        Args:
            session: 資料庫會話
            task: 任務物件
            agv_id: AGV ID
            
        Returns:
            bool: 是否成功派發
        """
        try:
            # 生成任務代碼
            kuka_mission_code = str(uuid.uuid4())
            
            self.get_logger().info(
                f"預派發任務 {task.id} 給 AGV {agv_id} kuka_mission_code:{kuka_mission_code}")
            self.get_logger().info(f"work_id: {task.work_id}")
            
            # 根據 work_id 執行對應的 KUKA API
            result = self._execute_kuka_api(task, agv_id, kuka_mission_code)
            
            if result["success"]:
                # 更新任務狀態
                task.agv_id = agv_id
                task.mission_code = kuka_mission_code
                task.parameters["agvId"] = agv_id
                self.get_logger().info(
                    f"任務 {task.id} 已派發，mission_code: {kuka_mission_code}")
                return True
            else:
                self.get_logger().error(f"KUKA API 調用失敗: {result}")
                return False
                
        except Exception as e:
            self.get_logger().error(f"派發任務給 AGV {agv_id} 時發生錯誤: {e}")
            return False

    def _execute_kuka_api(self, task, agv_id: int, mission_code: str) -> dict:
        """
        執行 KUKA API 調用（簡化版本）
        
        Args:
            task: 任務物件
            agv_id: AGV ID
            mission_code: 任務代碼
            
        Returns:
            dict: API 調用結果
        """
        try:
            result = {"success": False}
            
            # 基於 work_id 的簡單派發邏輯
            if task.work_id == 210001:  # KUKA 移動
                self.get_logger().info(f"parameters['nodes']: {task.parameters['nodes']}")
                if task.parameters['nodes']:
                    result = self.kuka_fleet.move(
                        task.parameters['nodes'], agv_id, mission_code)
                else:
                    self.get_logger().warn(
                        f"缺少參數 parameters['nodes'] 無法執行任務 {task.id}")
                        
            elif task.work_id == 220001:  # KUKA 移動貨架
                self.get_logger().info(f"parameters['nodes']: {task.parameters['nodes']}")
                if task.parameters['nodes']:
                    result = self.kuka_fleet.rack_move(
                        task.parameters['nodes'], agv_id, mission_code)
                else:
                    self.get_logger().warn(
                        f"缺少參數 parameters['nodes'] 無法執行任務 {task.id}")
                        
            else:  # 其他任務作為 workflow 處理
                template_code = task.parameters.get('templateCode')
                self.get_logger().info(f"templateCode: {template_code}")
                if template_code:
                    result = self.kuka_fleet.workflow(
                        template_code, agv_id, mission_code)
                else:
                    self.get_logger().warn(
                        f"缺少參數 parameters['templateCode'] 無法執行任務 {task.id}")
            
            return result
            
        except Exception as e:
            self.get_logger().error(f"執行 KUKA API 時發生錯誤: {e}")
            return {"success": False, "error": str(e)}

    def stop_monitoring(self):
        """停止 KUKA Fleet 監控"""
        if hasattr(self, 'kuka_fleet') and self.kuka_fleet:
            self.kuka_fleet.stop_monitoring()
            self.get_logger().info("KUKA Fleet 監控已停止")