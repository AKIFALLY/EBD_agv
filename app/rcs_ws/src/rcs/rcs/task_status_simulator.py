"""
RCS 任務狀態模擬器
負責模擬 AGV 任務的狀態轉換過程和RACK位置移動：

任務狀態轉換：
PENDING -> READY_TO_EXECUTE -> EXECUTING -> COMPLETED

RACK位置模擬：
- call_empty: 模擬空AGV到達叫車位置，將RACK設置為搬運狀態
- dispatch_full: 模擬AGV將RACK搬到目標房間，更新RACK位置並清除搬運狀態
"""

import asyncio
from typing import Dict, Any, Optional
from datetime import datetime, timezone
from db_proxy.models import Task, TaskStatus, Rack, Location
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
                
                # 任務完成時進行RACK位置模擬
                if new_status == TaskStatus.COMPLETED:
                    self._simulate_rack_movement(session, task)
                
                session.commit()
                
                self.logger.info(f"任務 {task_id} 狀態更新: {TaskStatus.get_name(old_status)} -> {TaskStatus.get_name(new_status)}")
                return True
                
        except Exception as e:
            self.logger.error(f"更新任務 {task_id} 狀態失敗: {e}")
            return False
    
    def _simulate_rack_movement(self, session, task: Task):
        """
        模擬RACK位置移動
        - call_empty: 將RACK從叫車位置搬到AGV上
        - dispatch_full: 將RACK從AGV搬到目標房間位置
        """
        try:
            # 解析任務參數
            params = task.parameters or {}
            task_type = params.get("task_type")
            
            if task_type == "call_empty":
                self._simulate_call_empty_completion(session, task, params)
            elif task_type == "dispatch_full":
                self._simulate_dispatch_full_completion(session, task, params)
            else:
                self.logger.debug(f"任務 {task.id} 類型 '{task_type}' 不需要RACK位置模擬")
                
        except Exception as e:
            self.logger.error(f"模擬任務 {task.id} 的RACK移動失敗: {e}")
    
    def _simulate_call_empty_completion(self, session, task: Task, params: dict):
        """
        模擬叫空車任務完成：空AGV到達叫車位置，準備搬走RACK
        1. 根據node_id找到RACK
        2. 設置RACK為搬運狀態 (is_carry=1)
        3. 記錄RACK被搬運的資訊
        """
        try:
            node_id = params.get("node_id")
            if not node_id:
                self.logger.warning(f"叫空車任務 {task.id} 缺少 node_id 參數")
                return
            
            # 查找該位置上的RACK
            stmt = select(Rack).where(Rack.location_id == node_id)
            rack = session.exec(stmt).first()
            
            if not rack:
                self.logger.warning(f"位置 {node_id} 沒有找到RACK")
                return
            
            # 模擬AGV到達並開始搬運RACK
            rack.is_carry = 1  # 設置為搬運狀態
            rack.agv_id = 1   # 模擬分配給AGV (這裡簡化使用固定ID)
            
            session.add(rack)
            
            self.logger.info(f"✅ 叫空車完成: RACK {rack.name} 在位置 {node_id} 被AGV開始搬運")
            
        except Exception as e:
            self.logger.error(f"模擬叫空車完成失敗: {e}")
    
    def _simulate_dispatch_full_completion(self, session, task: Task, params: dict):
        """
        模擬派車任務完成：AGV將RACK搬到目標房間
        1. 根據rack_id找到RACK
        2. 更新RACK位置到目標房間
        3. 清除搬運狀態
        """
        try:
            rack_id = params.get("rack_id")
            target_room = params.get("room")
            
            if not rack_id:
                self.logger.warning(f"派車任務 {task.id} 缺少 rack_id 參數")
                return
            
            if not target_room:
                self.logger.warning(f"派車任務 {task.id} 缺少 room 參數")
                return
            
            # 查找要搬運的RACK
            rack = session.get(Rack, rack_id)
            if not rack:
                self.logger.warning(f"RACK {rack_id} 不存在")
                return
            
            # 查找目標房間的一個可用位置 (簡化邏輯：使用房間內第一個位置)
            stmt = select(Location).where(Location.room_id == target_room).limit(1)
            target_location = session.exec(stmt).first()
            
            if not target_location:
                self.logger.warning(f"房間 {target_room} 沒有可用位置")
                return
            
            # 更新RACK位置
            old_location = rack.location_id
            rack.location_id = target_location.id
            rack.room_id = target_room
            rack.is_carry = 0  # 清除搬運狀態
            rack.agv_id = None  # 清除AGV關聯
            
            session.add(rack)
            
            self.logger.info(f"✅ 派車完成: RACK {rack.name} 從位置 {old_location} 搬到房間 {target_room} 位置 {target_location.id}")
            
        except Exception as e:
            self.logger.error(f"模擬派車完成失敗: {e}")
    
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
