from db_proxy.models import Task, Rack, Location, Room, Product, LocationStatus, TaskStatus
from sqlmodel import select
from typing import Optional, Dict, Any
from wcs.services import WcsTaskService
import asyncio
from datetime import datetime, timezone

class TaskManager:
    def __init__(self, db_pool, logger):
        self.db_pool = db_pool
        self.logger = logger

        # 使用 WCS 特定的 Service 層
        self.wcs_task_service = WcsTaskService(db_pool, logger)

        # 狀態轉換模擬相關
        self.pending_transitions: Dict[int, asyncio.Task] = {}  # task_id -> asyncio.Task
        self.transition_delays = {
            'requesting_to_pending': 2.0,  # REQUESTING -> PENDING 延遲 2 秒
        }

    def process_tasks(self):
        '''從資料庫中取得所有待執行任務，並根據任務類型進行處理'''
        # 清理已完成的狀態轉換任務
        self.cleanup_completed_transitions()

        # 處理 REQUESTING 狀態的任務 (狀態轉換模擬)
        self._process_requesting_tasks()

        # 處理 PENDING 狀態的任務 (原有邏輯)
        self._process_pending_tasks()

    def _process_requesting_tasks(self):
        '''處理 REQUESTING 狀態的任務，模擬 WCS 接受任務的過程'''
        with self.db_pool.get_session() as session:
            # 查詢所有 REQUESTING 狀態的任務
            stmt = select(Task).where(Task.status_id == TaskStatus.REQUESTING)
            requesting_tasks = session.exec(stmt).all()

            for task in requesting_tasks:
                # 檢查是否已經在處理中
                if task.id in self.pending_transitions:
                    continue

                # 解析任務參數
                params = task.parameters or {}
                task_type = params.get("task_type")

                # 只處理 opui 相關的任務類型
                if task_type in ["call_empty", "dispatch_full"]:
                    self.logger.info(f"WCS 接受任務 {task.id} (類型: {task_type})，準備轉換為 PENDING 狀態")

                    # 啟動異步狀態轉換
                    try:
                        loop = asyncio.get_event_loop()
                        transition_task = loop.create_task(
                            self._transition_to_pending(task.id, task_type)
                        )
                        self.pending_transitions[task.id] = transition_task
                    except RuntimeError:
                        # 如果沒有事件循環，直接同步處理
                        self.logger.warning(f"沒有事件循環，直接處理任務 {task.id}")
                        self._update_task_status_sync(task.id, TaskStatus.PENDING)

    def _process_pending_tasks(self):
        '''處理 PENDING 狀態的任務（原有邏輯）'''
        with self.db_pool.get_session() as session:
            stmt = select(Task).where(Task.status_id == TaskStatus.PENDING)
            tasks = session.exec(stmt).all()

            for task in tasks:
                # task.parameters 已經是字典型態，不需要 json.loads()
                params = task.parameters or {}
                task_type = params.get("task_type")

                if task_type == "call_empty":
                    self.handle_call_empty_task(task, params, session)
                elif task_type == "dispatch_full":
                    self.handle_dispatch_full_task(task, params, session)
                elif task_type is None:
                    # 跳過沒有 task_type 的任務，這些可能是其他類型的任務（如 KUKA 任務）
                    self.logger.debug(f"任務 {task.id} 沒有 task_type，跳過 WCS 處理")
                else:
                    self.logger.warning(f"未知任務類型: {task_type}")

    async def _transition_to_pending(self, task_id: int, task_type: str):
        '''異步狀態轉換：REQUESTING -> PENDING'''
        try:
            # 等待指定的延遲時間
            delay = self.transition_delays['requesting_to_pending']
            self.logger.info(f"任務 {task_id} 將在 {delay} 秒後轉換為 PENDING 狀態")
            await asyncio.sleep(delay)

            # 更新任務狀態
            success = self._update_task_status_sync(task_id, TaskStatus.PENDING)

            if success:
                self.logger.info(f"✅ 任務 {task_id} 已成功轉換為 PENDING 狀態")
            else:
                self.logger.error(f"❌ 任務 {task_id} 狀態轉換失敗")

        except Exception as e:
            self.logger.error(f"❌ 任務 {task_id} 狀態轉換異常: {e}")
        finally:
            # 清理已完成的轉換任務
            if task_id in self.pending_transitions:
                del self.pending_transitions[task_id]

    def _update_task_status_sync(self, task_id: int, new_status: int) -> bool:
        '''同步更新任務狀態'''
        try:
            with self.db_pool.get_session() as session:
                task = session.get(Task, task_id)
                if not task:
                    self.logger.error(f"任務 {task_id} 不存在")
                    return False

                old_status = task.status_id
                task.status_id = new_status
                task.updated_at = datetime.now(timezone.utc)

                session.add(task)
                session.commit()

                self.logger.info(f"任務 {task_id} 狀態更新: {TaskStatus.get_name(old_status)} -> {TaskStatus.get_name(new_status)}")
                return True

        except Exception as e:
            self.logger.error(f"更新任務 {task_id} 狀態失敗: {e}")
            return False

    def cleanup_completed_transitions(self):
        '''清理已完成的狀態轉換任務'''
        completed_tasks = []
        for task_id, transition_task in self.pending_transitions.items():
            if transition_task.done():
                completed_tasks.append(task_id)

        for task_id in completed_tasks:
            del self.pending_transitions[task_id]

        if completed_tasks:
            self.logger.debug(f"清理了 {len(completed_tasks)} 個已完成的狀態轉換任務")

    def shutdown(self):
        '''關閉 TaskManager，取消所有待處理的狀態轉換'''
        self.logger.info("正在關閉 TaskManager...")

        for task_id, transition_task in self.pending_transitions.items():
            if not transition_task.done():
                transition_task.cancel()
                self.logger.info(f"取消任務 {task_id} 的狀態轉換")

        self.pending_transitions.clear()
        self.logger.info("TaskManager 已關閉")

    def handle_call_empty_task(self, task, params, session):
        if params.get("rack_id"):
            self.logger.info(f"任務 {task.id} 已指派 rack {params['rack_id']}")
            return

        rack_stmt = select(Rack).where(
            (Rack.status_id == 1) &
            (Rack.is_carry == 0) &  # 使用整數 0 而不是 False
            (Rack.is_docked == 1) &  # 使用整數 1 而不是 True
            (Rack.is_in_map == 1) &  # 使用整數 1 而不是 True
            (~Rack.id.in_(
                select(Task.parameters["rack_id"].as_integer()).where(Task.parameters["rack_id"].is_not(None))
            ))
        ).limit(1)
        rack = session.exec(rack_stmt).first()

        if not rack:
            self.logger.info("沒有可用空rack")
            return

        params.update({
            "rack_id": rack.id,
            "from_location_id": rack.location_id,
            "to_location_id": task.node_id,
            "reason": "dispatch_empty_rack"
        })
        # task.parameters 是字典型態，直接賦值即可
        task.parameters = params
        task.priority = 5
        session.add(task)
        session.commit()
        self.logger.info(f"已為任務 {task.id} 指派 rack {rack.id}")

    def handle_dispatch_full_task(self, task, params, session):
        rack_id = params.get("rack_id")
        room_id = params.get("room")
        target_node_id = params.get("node_id")

        if not rack_id or not room_id or not target_node_id:
            self.logger.warning(f"任務 {task.id} 參數不完整")
            return

        rack = session.get(Rack, rack_id)
        room = session.get(Room, room_id)

        if not rack or not room:
            self.logger.warning("Rack 或 Room 不存在")
            return

        # 使用 WCS Service 層進行製程相容性檢查
        is_compatible, message = self.wcs_task_service.validate_product_room_compatibility(
            rack.product_id, room_id, session
        )
        if not is_compatible:
            self.logger.warning(f"任務 {task.id}: {message}")
            return

        # 使用 WCS Service 層進行派車任務驗證
        is_valid, validation_message = self.wcs_task_service.validate_dispatch_task(
            rack_id, room_id, target_node_id, session
        )
        if not is_valid:
            self.logger.info(f"任務 {task.id}: {validation_message}")
            return

        location = self.wcs_task_service.get_location_by_node(target_node_id, session)

        if rack.location_id == target_node_id and rack.direction == location.direction:
            self.logger.info("Rack 已在目的地且方向正確，無須再派送")
            return

        self.logger.info(f"派送 rack {rack_id} 到房間 {room_id} 的 node {target_node_id}")

        # 這裡可以更新任務狀態、通知 RCS 等

    # 這些方法現在委託給 WCS Service 層處理
    def get_product_process_settings(self, product_id: int, session=None) -> Optional[int]:
        """委託給 WCS Service 層處理"""
        return self.wcs_task_service.get_product_process_settings(product_id, session)

    def get_location_by_node(self, node_id: int, session) -> Optional[Location]:
        """委託給 WCS Service 層處理"""
        return self.wcs_task_service.get_location_by_node(node_id, session)

    def location_is_available(self, location):
        """委託給 WCS Service 層處理"""
        return self.wcs_task_service.location_is_available(location)
