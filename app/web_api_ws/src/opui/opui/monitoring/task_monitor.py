"""
OPUI 任務監控模組
負責監控任務狀態變更並處理相關邏輯
"""

import asyncio
from typing import Dict, Callable, Optional
from opui.database.operations import task_crud, connection_pool, get_call_empty_work_id, get_dispatch_full_work_id
from db_proxy.models import TaskStatus


class TaskMonitor:
    """任務監控器"""

    def __init__(self):
        self.monitored_tasks: Dict[int, Dict] = {}  # task_id -> task_info
        self.task_monitor_timer = None
        self.task_monitoring_started = False
        self.completion_callback: Optional[Callable] = None

    def set_completion_callback(self, callback: Callable):
        """設定任務完成時的回調函數"""
        self.completion_callback = callback

    def start_monitoring(self):
        """啟動任務監聽"""
        if self.task_monitor_timer is None and not self.task_monitoring_started:
            try:
                # 檢查是否有運行中的事件循環
                loop = asyncio.get_running_loop()
                self.task_monitor_timer = loop.create_task(self._monitor_loop())
                self.task_monitoring_started = True
                print("✅ 任務監聽已啟動")
            except RuntimeError:
                # 沒有運行中的事件循環，稍後再啟動
                print("⏳ 等待事件循環啟動後再開始任務監聽")

    def stop_monitoring(self):
        """停止任務監聽"""
        if self.task_monitor_timer:
            self.task_monitor_timer.cancel()
            self.task_monitor_timer = None
            self.task_monitoring_started = False
            print("🛑 任務監聽已停止")

    async def _monitor_loop(self):
        """任務監聽循環，每秒檢查一次"""
        while True:
            try:
                await asyncio.sleep(1)  # 每秒檢查一次
                await self._check_monitored_tasks()
            except Exception as e:
                print(f"❌ 任務監聽錯誤: {e}")

    async def _check_monitored_tasks(self):
        """檢查監聽中的任務狀態"""
        if not self.monitored_tasks:
            print("🔍 沒有監聽中的任務")
            return

        print(f"🔍 檢查 {len(self.monitored_tasks)} 個監聽中的任務")

        try:
            with connection_pool.get_session() as session:
                for task_id, task_info in list(self.monitored_tasks.items()):
                    print(f"🔍 檢查任務 {task_id}: {task_info}")

                    # 查詢任務當前狀態
                    current_task = task_crud.get_by_id(session, task_id)
                    if not current_task:
                        # 任務不存在，移除監聽
                        print(f"❌ 任務 {task_id} 不存在，移除監聽")
                        del self.monitored_tasks[task_id]
                        continue

                    task_type = task_info.get('task_type', 'call_empty')
                    print(f"🔍 任務 {task_id} 類型: {task_type}, 狀態: {current_task.status_id}")

                    if task_type == 'call_empty':
                        # 叫車任務：檢查任務狀態變更
                        await self._check_call_empty_task(task_id, current_task, task_info)
                    elif task_type == 'dispatch_full':
                        # 派車任務：檢查停車格是否已空
                        await self._check_dispatch_full_task(task_id, current_task, task_info, session)

        except Exception as e:
            print(f"❌ 檢查任務狀態失敗: {e}")

    async def _check_call_empty_task(self, task_id: int, current_task, task_info: dict):
        """檢查叫車任務狀態"""
        current_status = current_task.status_id
        previous_status = task_info.get('previous_status', TaskStatus.REQUESTING)
        node_id = task_info['node_id']

        print(f"🚗 檢查叫車任務 {task_id}: node_id={node_id}, status={current_status}")

        # 檢查任務是否被取消
        if current_status == TaskStatus.CANCELLING:
            print(f"🚫 叫車任務 {task_id} 已取消，移除監聽")
            # 移除已取消的任務監聽
            del self.monitored_tasks[task_id]
            return

        # 檢查停車格上是否有 rack
        from opui.database.operations import connection_pool
        with connection_pool.get_session() as session:
            has_rack = self._check_rack_at_location(session, node_id)
            print(f"🔍 停車格 {node_id} 是否有 rack: {has_rack}, 任務狀態: {current_status}")

            # 叫車完成條件：有 rack 且任務狀態為執行中
            if has_rack and current_status == TaskStatus.COMPLETED:
                print(f"✅ 叫車任務 {task_id} 完成：停車格有 rack 且任務狀態為完成，觸發完成回調")
                if self.completion_callback:
                    await self.completion_callback(current_task, task_info)
                # 移除已完成的任務監聽
                del self.monitored_tasks[task_id]
            else:
                # 更新狀態
                print(
                    f"🔄 更新任務 {task_id} 狀態: status {previous_status} → {current_status}, has_rack: {has_rack}")
                self.monitored_tasks[task_id]['previous_status'] = current_status

    async def _check_dispatch_full_task(self, task_id: int, current_task, task_info: dict, session):
        """檢查派車任務狀態"""
        current_status = current_task.status_id
        node_id = task_info['node_id']

        print(f"🚛 檢查派車任務 {task_id}: node_id={node_id}, status={current_status}")

        # 檢查任務是否被取消
        if current_status == TaskStatus.CANCELLED:
            print(f"🚫 派車任務 {task_id} 已取消，移除監聽")
            # 移除已取消的任務監聽
            del self.monitored_tasks[task_id]
            return

        # 獲取任務中指定的 rack_id
        target_rack_id = self._extract_rack_id_from_task(current_task)
        if not target_rack_id:
            print(f"⚠️ 派車任務 {task_id} 無法獲取 rack_id，使用舊邏輯檢查")
            # 如果無法獲取 rack_id，回退到檢查停車格是否有任何 rack
            has_any_rack = self._check_rack_at_location(session, node_id)
            if not has_any_rack:
                print(f"✅ 派車任務 {task_id} 檢測到停車格 {node_id} 已空，觸發完成回調")
                if self.completion_callback:
                    await self.completion_callback(current_task, task_info)
                del self.monitored_tasks[task_id]
            return

        # 檢查指定的 rack 是否還在該停車格位置
        target_rack_still_at_location = self._check_specific_rack_at_location(
            session, node_id, target_rack_id)
        print(f"🔍 停車格 {node_id} 是否還有指定的 rack {target_rack_id}: {target_rack_still_at_location}")

        # 派車完成條件：指定的 rack 不再位於該停車格
        if not target_rack_still_at_location:
            print(f"✅ 派車任務 {task_id} 檢測到指定 rack {target_rack_id} 已離開停車格 {node_id}，觸發完成回調")
            if self.completion_callback:
                await self.completion_callback(current_task, task_info)
            # 移除已完成的任務監聽
            del self.monitored_tasks[task_id]
        else:
            # 更新狀態
            print(f"🔄 派車任務 {task_id}: 指定 rack {target_rack_id} 仍在停車格 {node_id}")
            self.monitored_tasks[task_id]['target_rack_id'] = target_rack_id

    def add_task(self, task_id: int, machine_id: int, node_id: int, initial_status: int, task_type: str = "call_empty"):
        """新增任務監聽

        Args:
            task_id: 任務ID
            machine_id: 機台ID
            node_id: 停車格節點ID
            initial_status: 初始任務狀態
            task_type: 任務類型 ("call_empty" 或 "dispatch_full")
        """
        task_info = {
            'machine_id': machine_id,
            'node_id': node_id,
            'previous_status': initial_status,
            'task_type': task_type,
            'created_at': asyncio.get_event_loop().time()
        }

        # 如果是派車任務，初始化rack狀態和目標rack_id
        if task_type == 'dispatch_full':
            try:
                with connection_pool.get_session() as session:
                    # 獲取任務詳細資訊以提取 rack_id
                    from opui.database.operations import task_crud
                    task_detail = task_crud.get_by_id(session, task_id)
                    if task_detail:
                        target_rack_id = self._extract_rack_id_from_task(task_detail)
                        if target_rack_id:
                            task_info['target_rack_id'] = target_rack_id
                            print(f"🔍 派車任務 {task_id} 目標 rack_id: {target_rack_id}")
                        else:
                            print(f"⚠️ 派車任務 {task_id} 無法獲取 rack_id")

                    # 檢查當前停車格是否有rack（保留舊邏輯作為備用）
                    has_rack = self._check_rack_at_location(session, node_id)
                    task_info['previous_has_rack'] = has_rack
            except Exception as e:
                print(f"❌ 初始化派車任務狀態失敗: {e}")
                task_info['previous_has_rack'] = True  # 預設為有rack

        self.monitored_tasks[task_id] = task_info
        print(f"🔍 開始監聽任務 {task_id} (類型: {task_type}, machine_id: {machine_id}, node_id: {node_id})")

    def remove_task(self, task_id: int):
        """移除任務監聽"""
        if task_id in self.monitored_tasks:
            del self.monitored_tasks[task_id]
            print(f"🗑️ 停止監聽任務 {task_id}")

    def remove_task_by_node(self, node_id: int):
        """根據停車格節點ID移除任務監聽"""
        tasks_to_remove = []

        # 找到所有匹配 node_id 的任務
        for task_id, task_info in self.monitored_tasks.items():
            if task_info.get('node_id') == node_id:
                tasks_to_remove.append(task_id)

        # 移除找到的任務
        for task_id in tasks_to_remove:
            del self.monitored_tasks[task_id]
            print(f"🗑️ 停止監聽停車格 {node_id} 的任務 {task_id}")

        if tasks_to_remove:
            print(f"✅ 已移除停車格 {node_id} 的 {len(tasks_to_remove)} 個任務監聽")
        else:
            print(f"⚠️ 未找到停車格 {node_id} 的監聽任務")

    def get_monitored_tasks(self) -> Dict[int, Dict]:
        """獲取當前監聽的任務"""
        return self.monitored_tasks.copy()

    def is_monitoring(self, task_id: int) -> bool:
        """檢查是否正在監聽指定任務"""
        return task_id in self.monitored_tasks

    async def restore_from_database(self):
        """從資料庫恢復進行中的任務監聽"""
        try:
            call_empty_work_id = get_call_empty_work_id()
            dispatch_full_work_id = get_dispatch_full_work_id()

            with connection_pool.get_session() as session:
                # 查詢所有進行中的OPUI任務
                tasks = task_crud.get_all(session)
                opui_work_ids = [call_empty_work_id, dispatch_full_work_id]

                for task in tasks:
                    if task.work_id in opui_work_ids:
                        # 解析任務參數獲取機台和停車格資訊
                        machine_id, node_id = self._extract_task_info(task)
                        if not machine_id or not node_id:
                            continue

                        # 判斷任務類型
                        task_type = "call_empty" if task.work_id == call_empty_work_id else "dispatch_full"

                        # 只處理進行中的任務，不修改停車格狀態
                        if task.status_id in [TaskStatus.REQUESTING, TaskStatus.PENDING, TaskStatus.READY_TO_EXECUTE]:  # 請求中、待處理、待執行
                            # 如果任務還在進行中，加入監聽
                            if task.id not in self.monitored_tasks:
                                self.add_task(task.id, machine_id, node_id,
                                              task.status_id, task_type)
                                print(
                                    f"🔍 恢復監聽任務: task_id={task.id}, 類型={task_type}, node_id={node_id}")

        except Exception as e:
            print(f"❌ 從資料庫恢復任務狀態失敗: {e}")

    def _check_rack_at_location(self, session, node_id: int) -> bool:
        """檢查指定停車格是否有rack

        Args:
            session: 資料庫會話
            node_id: 停車格節點ID

        Returns:
            bool: True表示有rack，False表示沒有rack
        """
        try:
            from sqlmodel import select
            from db_proxy.models import Rack

            # 查詢location_id等於node_id的rack
            statement = select(Rack).where(Rack.location_id == node_id)
            racks = session.exec(statement).all()

            print(f"🔍 查詢停車格 {node_id} 的 rack: 找到 {len(racks)} 個")
            for rack in racks:
                print(f"   - Rack {rack.id}: name={rack.name}, location_id={rack.location_id}")

            return len(racks) > 0

        except Exception as e:
            print(f"❌ 檢查停車格rack狀態失敗: {e}")
            return True  # 發生錯誤時保守處理，假設有rack

    def _check_specific_rack_at_location(self, session, node_id: int, rack_id: int) -> bool:
        """檢查指定的 rack 是否還在指定停車格位置

        Args:
            session: 資料庫會話
            node_id: 停車格節點ID
            rack_id: 指定的 rack ID

        Returns:
            bool: True表示指定rack還在該位置，False表示已離開
        """
        try:
            from sqlmodel import select
            from db_proxy.models import Rack

            # 查詢指定的 rack
            statement = select(Rack).where(Rack.id == rack_id)
            rack = session.exec(statement).first()

            if not rack:
                print(f"🔍 Rack {rack_id} 不存在")
                return False

            is_at_location = rack.location_id == node_id
            print(
                f"🔍 Rack {rack_id} (name={rack.name}) 當前位置: {rack.location_id}, 目標位置: {node_id}, 是否在位置: {is_at_location}")

            return is_at_location

        except Exception as e:
            print(f"❌ 檢查指定rack位置失敗: {e}")
            return True  # 發生錯誤時保守處理，假設還在位置

    def _extract_rack_id_from_task(self, task) -> int:
        """從任務中提取 rack_id

        Args:
            task: 任務對象

        Returns:
            int: rack_id，如果無法提取則返回 None
        """
        try:
            import json

            print(f"🔍 嘗試從任務 {task.id} 提取 rack_id")
            print(f"🔍 任務參數類型: {type(task.parameters)}")
            print(f"🔍 任務參數內容: {task.parameters}")

            # 嘗試從 parameters 欄位解析 rack_id
            if hasattr(task, 'parameters') and task.parameters:
                params = None

                if isinstance(task.parameters, str):
                    # 如果 parameters 是字串，嘗試解析 JSON
                    try:
                        params = json.loads(task.parameters)
                        print(f"🔍 從 JSON 字串解析參數: {params}")
                    except json.JSONDecodeError as e:
                        print(f"⚠️ JSON 解析失敗: {e}")
                        return None
                elif isinstance(task.parameters, dict):
                    # 如果 parameters 已經是字典
                    params = task.parameters
                    print(f"🔍 參數已是字典格式: {params}")
                else:
                    print(f"⚠️ 無法解析任務參數類型: {type(task.parameters)}")
                    return None

                if params:
                    rack_id = params.get('rack_id')
                    if rack_id is not None:
                        print(f"✅ 從任務 {task.id} 提取到 rack_id: {rack_id}")
                        return int(rack_id)
                    else:
                        print(f"⚠️ 任務 {task.id} 參數中沒有 rack_id 欄位")
                        print(f"🔍 可用欄位: {list(params.keys())}")

            print(f"⚠️ 任務 {task.id} 中未找到有效的 rack_id")
            return None

        except Exception as e:
            print(f"❌ 提取任務 rack_id 失敗: {e}")
            import traceback
            traceback.print_exc()
            return None

    def _extract_task_info(self, task):
        """從任務中提取機台和停車格資訊"""
        try:
            import json
            if hasattr(task, 'parameters') and task.parameters:
                # 如果 parameters 已經是 dict，直接使用；如果是字串，則解析
                if isinstance(task.parameters, str):
                    parameters = json.loads(task.parameters)
                else:
                    parameters = task.parameters
                machine_id = parameters.get('machine_id')
                node_id = parameters.get('node_id') or task.node_id
                return machine_id, node_id
            else:
                # 沒有參數，只使用 node_id
                return None, task.node_id
        except Exception as e:
            print(f"❌ 解析任務參數失敗: {e}")
            return None, None
