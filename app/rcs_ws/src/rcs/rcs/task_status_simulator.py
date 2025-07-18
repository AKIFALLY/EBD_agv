"""
RCS 任務狀態模擬器
負責模擬 AGV 任務的狀態轉換過程：
PENDING -> READY_TO_EXECUTE -> EXECUTING -> COMPLETED
"""

import asyncio
from typing import Dict, Any, Optional
from datetime import datetime, timezone
from db_proxy.models import Task, TaskStatus
from sqlmodel import select


class TaskStatusSimulator:
    """任務狀態模擬器"""
    
    def __init__(self, db_pool, logger):
        self.db_pool = db_pool
        self.logger = logger
        
        # 狀態轉換模擬相關
        self.pending_transitions: Dict[int, asyncio.Task] = {}  # task_id -> asyncio.Task
        self.transition_delays = {
            'pending_to_ready': 3.0,      # PENDING -> READY_TO_EXECUTE 延遲 3 秒
            'ready_to_executing': 1.0,    # READY_TO_EXECUTE -> EXECUTING 延遲 1 秒
            'executing_to_completed': 10.0  # EXECUTING -> COMPLETED 延遲 10 秒
        }
    
    def process_task_status_transitions(self):
        """處理任務狀態轉換"""
        # 清理已完成的狀態轉換任務
        self.cleanup_completed_transitions()
        
        # 處理各個狀態的任務
        self._process_pending_tasks()
        self._process_ready_to_execute_tasks()
        self._process_executing_tasks()
    
    def _process_pending_tasks(self):
        """處理 PENDING 狀態的任務，模擬 RCS 派發任務的過程"""
        with self.db_pool.get_session() as session:
            # 查詢所有 PENDING 狀態的 opui 任務
            stmt = select(Task).where(
                Task.status_id == TaskStatus.PENDING,
                Task.parameters["task_type"].as_string().in_(["call_empty", "dispatch_full"])
            )
            pending_tasks = session.exec(stmt).all()
            
            for task in pending_tasks:
                # 檢查是否已經在處理中
                if task.id in self.pending_transitions:
                    continue
                
                # 解析任務參數
                params = task.parameters or {}
                task_type = params.get("task_type")
                
                self.logger.info(f"RCS 接受任務 {task.id} (類型: {task_type})，準備派發 AGV")
                
                # 啟動異步狀態轉換
                try:
                    loop = asyncio.get_event_loop()
                    transition_task = loop.create_task(
                        self._transition_to_ready_to_execute(task.id, task_type)
                    )
                    self.pending_transitions[task.id] = transition_task
                except RuntimeError:
                    # 如果沒有事件循環，直接同步處理
                    self.logger.warning(f"沒有事件循環，直接處理任務 {task.id}")
                    self._update_task_status_sync(task.id, TaskStatus.READY_TO_EXECUTE)
    
    def _process_ready_to_execute_tasks(self):
        """處理 READY_TO_EXECUTE 狀態的任務，模擬 AGV 開始執行的過程"""
        with self.db_pool.get_session() as session:
            stmt = select(Task).where(
                Task.status_id == TaskStatus.READY_TO_EXECUTE,
                Task.parameters["task_type"].as_string().in_(["call_empty", "dispatch_full"])
            )
            ready_tasks = session.exec(stmt).all()
            
            for task in ready_tasks:
                # 檢查是否已經在處理中
                transition_key = f"{task.id}_ready_to_executing"
                if transition_key in self.pending_transitions:
                    continue
                
                params = task.parameters or {}
                task_type = params.get("task_type")
                
                self.logger.info(f"AGV 準備開始執行任務 {task.id} (類型: {task_type})")
                
                # 啟動異步狀態轉換
                try:
                    loop = asyncio.get_event_loop()
                    transition_task = loop.create_task(
                        self._transition_to_executing(task.id, task_type)
                    )
                    self.pending_transitions[transition_key] = transition_task
                except RuntimeError:
                    self.logger.warning(f"沒有事件循環，直接處理任務 {task.id}")
                    self._update_task_status_sync(task.id, TaskStatus.EXECUTING)
    
    def _process_executing_tasks(self):
        """處理 EXECUTING 狀態的任務，模擬 AGV 完成任務的過程"""
        with self.db_pool.get_session() as session:
            stmt = select(Task).where(
                Task.status_id == TaskStatus.EXECUTING,
                Task.parameters["task_type"].as_string().in_(["call_empty", "dispatch_full"])
            )
            executing_tasks = session.exec(stmt).all()
            
            for task in executing_tasks:
                # 檢查是否已經在處理中
                transition_key = f"{task.id}_executing_to_completed"
                if transition_key in self.pending_transitions:
                    continue
                
                params = task.parameters or {}
                task_type = params.get("task_type")
                
                self.logger.info(f"AGV 正在執行任務 {task.id} (類型: {task_type})，預計 {self.transition_delays['executing_to_completed']} 秒後完成")
                
                # 啟動異步狀態轉換
                try:
                    loop = asyncio.get_event_loop()
                    transition_task = loop.create_task(
                        self._transition_to_completed(task.id, task_type)
                    )
                    self.pending_transitions[transition_key] = transition_task
                except RuntimeError:
                    self.logger.warning(f"沒有事件循環，直接處理任務 {task.id}")
                    self._update_task_status_sync(task.id, TaskStatus.COMPLETED)
    
    async def _transition_to_ready_to_execute(self, task_id: int, task_type: str):
        """異步狀態轉換：PENDING -> READY_TO_EXECUTE"""
        try:
            delay = self.transition_delays['pending_to_ready']
            self.logger.info(f"任務 {task_id} 將在 {delay} 秒後轉換為 READY_TO_EXECUTE 狀態")
            await asyncio.sleep(delay)
            
            success = self._update_task_status_sync(task_id, TaskStatus.READY_TO_EXECUTE)
            
            if success:
                self.logger.info(f"✅ 任務 {task_id} 已成功轉換為 READY_TO_EXECUTE 狀態")
            else:
                self.logger.error(f"❌ 任務 {task_id} 狀態轉換失敗")
                
        except Exception as e:
            self.logger.error(f"❌ 任務 {task_id} 狀態轉換異常: {e}")
        finally:
            if task_id in self.pending_transitions:
                del self.pending_transitions[task_id]
    
    async def _transition_to_executing(self, task_id: int, task_type: str):
        """異步狀態轉換：READY_TO_EXECUTE -> EXECUTING"""
        try:
            delay = self.transition_delays['ready_to_executing']
            self.logger.info(f"任務 {task_id} 將在 {delay} 秒後轉換為 EXECUTING 狀態")
            await asyncio.sleep(delay)
            
            success = self._update_task_status_sync(task_id, TaskStatus.EXECUTING)
            
            if success:
                self.logger.info(f"✅ 任務 {task_id} 已成功轉換為 EXECUTING 狀態")
            else:
                self.logger.error(f"❌ 任務 {task_id} 狀態轉換失敗")
                
        except Exception as e:
            self.logger.error(f"❌ 任務 {task_id} 狀態轉換異常: {e}")
        finally:
            transition_key = f"{task_id}_ready_to_executing"
            if transition_key in self.pending_transitions:
                del self.pending_transitions[transition_key]
    
    async def _transition_to_completed(self, task_id: int, task_type: str):
        """異步狀態轉換：EXECUTING -> COMPLETED"""
        try:
            delay = self.transition_delays['executing_to_completed']
            self.logger.info(f"任務 {task_id} 將在 {delay} 秒後轉換為 COMPLETED 狀態")
            await asyncio.sleep(delay)
            
            success = self._update_task_status_sync(task_id, TaskStatus.COMPLETED)
            
            if success:
                self.logger.info(f"✅ 任務 {task_id} 已成功完成！")
            else:
                self.logger.error(f"❌ 任務 {task_id} 狀態轉換失敗")
                
        except Exception as e:
            self.logger.error(f"❌ 任務 {task_id} 狀態轉換異常: {e}")
        finally:
            transition_key = f"{task_id}_executing_to_completed"
            if transition_key in self.pending_transitions:
                del self.pending_transitions[transition_key]
    
    def _update_task_status_sync(self, task_id: int, new_status: int) -> bool:
        """同步更新任務狀態"""
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
        """清理已完成的狀態轉換任務"""
        completed_tasks = []
        for key, transition_task in self.pending_transitions.items():
            if transition_task.done():
                completed_tasks.append(key)
        
        for key in completed_tasks:
            del self.pending_transitions[key]
            
        if completed_tasks:
            self.logger.debug(f"清理了 {len(completed_tasks)} 個已完成的狀態轉換任務")
    
    def shutdown(self):
        """關閉模擬器，取消所有待處理的狀態轉換"""
        self.logger.info("正在關閉 TaskStatusSimulator...")
        
        for key, transition_task in self.pending_transitions.items():
            if not transition_task.done():
                transition_task.cancel()
                self.logger.info(f"取消狀態轉換: {key}")
        
        self.pending_transitions.clear()
        self.logger.info("TaskStatusSimulator 已關閉")
