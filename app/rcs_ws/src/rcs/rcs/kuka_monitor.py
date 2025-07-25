#!/usr/bin/env python3
"""
KUKA 車隊狀態監控服務
提供 KUKA 車隊的實時狀態監控、數據收集和分析功能
"""

import json
import time
import asyncio
from datetime import datetime, timezone, timedelta
from typing import Dict, List, Any, Optional
from dataclasses import dataclass, asdict
from concurrent.futures import ThreadPoolExecutor
import threading

from db_proxy.connection_pool_manager import ConnectionPoolManager
from kuka_fleet_adapter.kuka_fleet_adapter import KukaFleetAdapter


@dataclass
class KukaFleetMetrics:
    """KUKA 車隊指標數據類"""
    timestamp: str
    total_agvs: int
    idle_agvs: int
    busy_agvs: int
    charging_agvs: int
    error_agvs: int
    active_tasks: int
    completed_tasks_today: int
    failed_tasks_today: int
    average_battery_level: float
    fleet_utilization_rate: float
    average_task_duration: float
    total_distance_today: float


@dataclass
class AgvStatusInfo:
    """AGV 狀態資訊"""
    id: int
    name: str
    model: str
    status: str
    battery_level: float
    position_x: float
    position_y: float
    current_task_id: Optional[str]
    last_updated: str
    total_tasks_completed: int
    total_distance: float
    uptime_hours: float
    error_message: Optional[str]


@dataclass
class TaskInfo:
    """任務資訊"""
    id: int
    mission_code: str
    agv_id: int
    agv_name: str
    task_type: str
    status: str
    priority: int
    created_at: str
    started_at: Optional[str]
    completed_at: Optional[str]
    duration: Optional[float]
    progress_percentage: float


class KukaFleetMonitor:
    """KUKA 車隊監控核心"""
    
    def __init__(self, rcs_core=None):
        """
        初始化監控服務
        
        Args:
            rcs_core: RCS Core 節點實例（可選）
        """
        self.rcs_core = rcs_core
        
        # 初始化資料庫連線池
        if rcs_core:
            self.db_pool = rcs_core.db_pool
            self.kuka_fleet = rcs_core.kuka_fleet
            self.logger = rcs_core.get_logger()
        else:
            # 獨立運行模式
            self.db_pool = ConnectionPoolManager(
                'postgresql+psycopg2://agvc:password@192.168.100.254/agvc'
            )
            self.kuka_fleet = KukaFleetAdapter()
            self.logger = None
        
        # 監控數據緩存
        self.current_metrics: Optional[KukaFleetMetrics] = None
        self.agv_statuses: Dict[int, AgvStatusInfo] = {}
        self.active_tasks: Dict[int, TaskInfo] = {}
        self.historical_metrics: List[KukaFleetMetrics] = []
        
        # 配置設定
        self.config = {
            'update_interval': 5.0,  # 數據更新間隔（秒）
            'history_retention_hours': 24,  # 歷史數據保留時間
            'max_history_records': 2880,  # 最大歷史記錄數（24小時 x 12次/小時）
            'alert_thresholds': {
                'low_battery': 20.0,
                'high_utilization': 90.0,
                'task_timeout': 1800.0,  # 30分鐘
                'error_rate_threshold': 10.0
            }
        }
        
        # 執行狀態
        self.is_running = False
        self.update_thread: Optional[threading.Thread] = None
        self.subscribers: List[Any] = []  # WebSocket 連接列表
        
        # 統計數據
        self.daily_stats = {
            'start_time': datetime.now(timezone.utc).replace(hour=0, minute=0, second=0, microsecond=0),
            'completed_tasks': 0,
            'failed_tasks': 0,
            'total_distance': 0.0,
            'uptime_seconds': 0
        }
        
        self.log("KUKA 車隊監控服務已初始化")
    
    def log(self, message: str, level: str = 'info'):
        """記錄日誌"""
        if self.logger:
            getattr(self.logger, level)(f"[KukaFleetMonitor] {message}")
        else:
            print(f"[{datetime.now()}] [{level.upper()}] {message}")
    
    def start_monitoring(self):
        """開始監控服務"""
        if self.is_running:
            self.log("監控服務已在運行中", 'warning')
            return
        
        self.is_running = True
        self.update_thread = threading.Thread(target=self._monitoring_loop, daemon=True)
        self.update_thread.start()
        self.log("KUKA 車隊監控服務已啟動")
    
    def stop_monitoring(self):
        """停止監控服務"""
        self.is_running = False
        if self.update_thread:
            self.update_thread.join(timeout=10)
        self.log("KUKA 車隊監控服務已停止")
    
    def _monitoring_loop(self):
        """監控主迴圈"""
        while self.is_running:
            try:
                start_time = time.time()
                
                # 更新車隊數據
                self._update_fleet_data()
                
                # 更新任務數據
                self._update_task_data()
                
                # 計算車隊指標
                self._calculate_fleet_metrics()
                
                # 檢查警報條件
                self._check_alerts()
                
                # 清理歷史數據
                self._cleanup_historical_data()
                
                # 通知所有訂閱者
                if self.current_metrics:
                    asyncio.run(self._notify_subscribers())
                
                # 計算執行時間並調整睡眠時間
                execution_time = time.time() - start_time
                sleep_time = max(0, self.config['update_interval'] - execution_time)
                
                if execution_time > self.config['update_interval']:
                    self.log(f"監控更新耗時 {execution_time:.2f}秒，超過設定間隔", 'warning')
                
                time.sleep(sleep_time)
                
            except Exception as e:
                self.log(f"監控迴圈發生錯誤: {e}", 'error')
                time.sleep(5)  # 錯誤後稍作等待
    
    def _update_fleet_data(self):
        """更新車隊數據"""
        try:
            # 從 KUKA Fleet API 獲取所有 AGV 狀態
            fleet_status = self.kuka_fleet.get_fleet_status()
            
            if not fleet_status or not fleet_status.get('success'):
                self.log("無法獲取 KUKA Fleet 狀態", 'error')
                return
            
            # 更新 AGV 狀態資訊
            for agv_data in fleet_status.get('data', {}).get('agvs', []):
                agv_id = int(agv_data.get('id', 0))
                
                # 從資料庫獲取額外資訊
                db_agv_info = self._get_agv_database_info(agv_id)
                
                self.agv_statuses[agv_id] = AgvStatusInfo(
                    id=agv_id,
                    name=agv_data.get('name', f'KUKA{agv_id:03d}'),
                    model=agv_data.get('model', 'KUKA400i'),
                    status=agv_data.get('status', 'unknown'),
                    battery_level=float(agv_data.get('battery_level', 0)),
                    position_x=float(agv_data.get('position', {}).get('x', 0)),
                    position_y=float(agv_data.get('position', {}).get('y', 0)),
                    current_task_id=agv_data.get('current_task'),
                    last_updated=datetime.now(timezone.utc).isoformat(),
                    total_tasks_completed=db_agv_info.get('total_tasks', 0),
                    total_distance=float(db_agv_info.get('total_distance', 0)),
                    uptime_hours=float(db_agv_info.get('uptime_hours', 0)),
                    error_message=agv_data.get('error_message')
                )
                
        except Exception as e:
            self.log(f"更新車隊數據時發生錯誤: {e}", 'error')
    
    def _get_agv_database_info(self, agv_id: int) -> Dict[str, Any]:
        """從資料庫獲取 AGV 額外資訊"""
        try:
            with self.db_pool.get_session() as session:
                from db_proxy.models import AGV, Task
                from sqlmodel import select, func
                
                # 獲取 AGV 基本資訊
                agv = session.exec(select(AGV).where(AGV.id == agv_id)).first()
                if not agv:
                    return {}
                
                # 統計任務完成數
                completed_tasks = session.exec(
                    select(func.count(Task.id)).where(
                        Task.agv_id == agv_id,
                        Task.status_id == 3  # 已完成
                    )
                ).first() or 0
                
                # 計算總距離（模擬數據，實際需要從任務記錄計算）
                total_distance = completed_tasks * 50.0  # 假設每任務平均50米
                
                # 計算上線時間（從創建時間算起）
                now = datetime.now(timezone.utc)
                created_time = agv.created_at or now
                uptime_hours = (now - created_time).total_seconds() / 3600
                
                return {
                    'total_tasks': completed_tasks,
                    'total_distance': total_distance,
                    'uptime_hours': uptime_hours
                }
                
        except Exception as e:
            self.log(f"獲取 AGV {agv_id} 資料庫資訊時發生錯誤: {e}", 'error')
            return {}
    
    def _update_task_data(self):
        """更新任務數據"""
        try:
            with self.db_pool.get_session() as session:
                from db_proxy.models import Task, AGV
                from sqlmodel import select
                
                # 查詢活動中的 KUKA 任務
                active_tasks = session.exec(
                    select(Task, AGV).join(AGV).where(
                        Task.status_id.in_([1, 2]),  # 待執行或執行中
                        AGV.model == "KUKA400i",
                        Task.mission_code.isnot(None)
                    )
                ).all()
                
                self.active_tasks.clear()
                
                for task, agv in active_tasks:
                    # 計算任務進度（模擬）
                    progress = self._calculate_task_progress(task)
                    
                    # 計算任務持續時間
                    duration = None
                    if task.status_id == 2 and task.updated_at:  # 執行中
                        duration = (datetime.now(timezone.utc) - task.updated_at).total_seconds()
                    
                    self.active_tasks[task.id] = TaskInfo(
                        id=task.id,
                        mission_code=task.mission_code or '',
                        agv_id=task.agv_id or 0,
                        agv_name=agv.name,
                        task_type=self._get_task_type_name(task.work_id),
                        status=self._get_task_status_name(task.status_id),
                        priority=task.priority or 40,
                        created_at=task.created_at.isoformat() if task.created_at else '',
                        started_at=task.updated_at.isoformat() if task.updated_at and task.status_id == 2 else None,
                        completed_at=None,
                        duration=duration,
                        progress_percentage=progress
                    )
                
        except Exception as e:
            self.log(f"更新任務數據時發生錯誤: {e}", 'error')
    
    def _calculate_task_progress(self, task) -> float:
        """計算任務進度（模擬）"""
        try:
            if task.status_id == 1:  # 待執行
                return 0.0
            elif task.status_id == 2:  # 執行中
                # 基於任務開始時間估算進度
                if task.updated_at:
                    elapsed = (datetime.now(timezone.utc) - task.updated_at).total_seconds()
                    estimated_duration = 300  # 假設平均任務時間5分鐘
                    progress = min(90.0, (elapsed / estimated_duration) * 100)
                    return progress
                return 10.0
            elif task.status_id == 3:  # 已完成
                return 100.0
            else:
                return 0.0
        except:
            return 0.0
    
    def _get_task_type_name(self, work_id: int) -> str:
        """獲取任務類型名稱"""
        task_types = {
            210001: "移動",
            220001: "移動貨架",
            230001: "工作流程"
        }
        return task_types.get(work_id, f"未知({work_id})")
    
    def _get_task_status_name(self, status_id: int) -> str:
        """獲取任務狀態名稱"""
        status_names = {
            1: "待執行",
            2: "執行中",
            3: "已完成",
            4: "失敗",
            5: "已取消"
        }
        return status_names.get(status_id, f"未知({status_id})")
    
    def _calculate_fleet_metrics(self):
        """計算車隊指標"""
        try:
            now = datetime.now(timezone.utc)
            
            # 統計 AGV 狀態
            total_agvs = len(self.agv_statuses)
            idle_agvs = sum(1 for agv in self.agv_statuses.values() if agv.status == 'idle')
            busy_agvs = sum(1 for agv in self.agv_statuses.values() if agv.status == 'busy')
            charging_agvs = sum(1 for agv in self.agv_statuses.values() if agv.status == 'charging')
            error_agvs = sum(1 for agv in self.agv_statuses.values() if agv.status == 'error')
            
            # 計算平均電池電量
            battery_levels = [agv.battery_level for agv in self.agv_statuses.values()]
            avg_battery = sum(battery_levels) / len(battery_levels) if battery_levels else 0.0
            
            # 車隊利用率
            utilization_rate = (busy_agvs / total_agvs * 100) if total_agvs > 0 else 0.0
            
            # 統計今日任務
            today_completed, today_failed = self._get_daily_task_stats()
            
            # 計算平均任務時間（模擬）
            avg_task_duration = self._calculate_average_task_duration()
            
            # 計算今日總距離
            total_distance_today = sum(agv.total_distance for agv in self.agv_statuses.values())
            
            # 創建指標對象
            self.current_metrics = KukaFleetMetrics(
                timestamp=now.isoformat(),
                total_agvs=total_agvs,
                idle_agvs=idle_agvs,
                busy_agvs=busy_agvs,
                charging_agvs=charging_agvs,
                error_agvs=error_agvs,
                active_tasks=len(self.active_tasks),
                completed_tasks_today=today_completed,
                failed_tasks_today=today_failed,
                average_battery_level=avg_battery,
                fleet_utilization_rate=utilization_rate,
                average_task_duration=avg_task_duration,
                total_distance_today=total_distance_today
            )
            
            # 添加到歷史記錄
            self.historical_metrics.append(self.current_metrics)
            
        except Exception as e:
            self.log(f"計算車隊指標時發生錯誤: {e}", 'error')
    
    def _get_daily_task_stats(self) -> tuple:
        """獲取今日任務統計"""
        try:
            with self.db_pool.get_session() as session:
                from db_proxy.models import Task, AGV
                from sqlmodel import select, func
                
                today_start = datetime.now(timezone.utc).replace(hour=0, minute=0, second=0, microsecond=0)
                
                # 統計今日完成的任務
                completed = session.exec(
                    select(func.count(Task.id)).join(AGV).where(
                        Task.status_id == 3,  # 已完成
                        AGV.model == "KUKA400i",
                        Task.updated_at >= today_start
                    )
                ).first() or 0
                
                # 統計今日失敗的任務
                failed = session.exec(
                    select(func.count(Task.id)).join(AGV).where(
                        Task.status_id == 4,  # 失敗
                        AGV.model == "KUKA400i",
                        Task.updated_at >= today_start
                    )
                ).first() or 0
                
                return completed, failed
                
        except Exception as e:
            self.log(f"獲取今日任務統計時發生錯誤: {e}", 'error')
            return 0, 0
    
    def _calculate_average_task_duration(self) -> float:
        """計算平均任務持續時間"""
        try:
            # 從活動任務計算平均時間（模擬）
            durations = [task.duration for task in self.active_tasks.values() if task.duration]
            if durations:
                return sum(durations) / len(durations)
            return 300.0  # 預設5分鐘
        except:
            return 300.0
    
    def _check_alerts(self):
        """檢查警報條件"""
        try:
            if not self.current_metrics:
                return
            
            alerts = []
            thresholds = self.config['alert_thresholds']
            
            # 檢查低電量 AGV
            low_battery_agvs = [
                agv for agv in self.agv_statuses.values() 
                if agv.battery_level < thresholds['low_battery']
            ]
            if low_battery_agvs:
                alerts.append({
                    'type': 'low_battery',
                    'severity': 'warning',
                    'message': f'{len(low_battery_agvs)} 台 AGV 電量低於 {thresholds["low_battery"]}%',
                    'agvs': [agv.name for agv in low_battery_agvs]
                })
            
            # 檢查高利用率
            if self.current_metrics.fleet_utilization_rate > thresholds['high_utilization']:
                alerts.append({
                    'type': 'high_utilization',
                    'severity': 'info',
                    'message': f'車隊利用率達到 {self.current_metrics.fleet_utilization_rate:.1f}%',
                    'value': self.current_metrics.fleet_utilization_rate
                })
            
            # 檢查錯誤 AGV
            if self.current_metrics.error_agvs > 0:
                error_agvs = [
                    agv for agv in self.agv_statuses.values() 
                    if agv.status == 'error'
                ]
                alerts.append({
                    'type': 'agv_errors',
                    'severity': 'error',
                    'message': f'{self.current_metrics.error_agvs} 台 AGV 發生錯誤',
                    'agvs': [{'name': agv.name, 'error': agv.error_message} for agv in error_agvs]
                })
            
            # 檢查超時任務
            timeout_tasks = [
                task for task in self.active_tasks.values()
                if task.duration and task.duration > thresholds['task_timeout']
            ]
            if timeout_tasks:
                alerts.append({
                    'type': 'task_timeout',
                    'severity': 'warning',
                    'message': f'{len(timeout_tasks)} 個任務執行時間超過 {thresholds["task_timeout"]/60:.0f} 分鐘',
                    'tasks': [task.mission_code for task in timeout_tasks]
                })
            
            if alerts:
                self.log(f"檢測到 {len(alerts)} 個警報", 'warning')
                # 這裡可以添加警報通知邏輯
            
        except Exception as e:
            self.log(f"檢查警報時發生錯誤: {e}", 'error')
    
    def _cleanup_historical_data(self):
        """清理歷史數據"""
        try:
            # 限制歷史記錄數量
            if len(self.historical_metrics) > self.config['max_history_records']:
                # 保留最新的記錄
                self.historical_metrics = self.historical_metrics[-self.config['max_history_records']:]
            
            # 清理過期數據
            cutoff_time = datetime.now(timezone.utc) - timedelta(hours=self.config['history_retention_hours'])
            self.historical_metrics = [
                metric for metric in self.historical_metrics
                if datetime.fromisoformat(metric.timestamp.replace('Z', '+00:00')) > cutoff_time
            ]
            
        except Exception as e:
            self.log(f"清理歷史數據時發生錯誤: {e}", 'error')
    
    async def _notify_subscribers(self):
        """通知所有訂閱者"""
        if not self.subscribers:
            return
        
        try:
            # 準備數據
            data = {
                'type': 'fleet_update',
                'timestamp': datetime.now(timezone.utc).isoformat(),
                'metrics': asdict(self.current_metrics) if self.current_metrics else None,
                'agv_statuses': [asdict(agv) for agv in self.agv_statuses.values()],
                'active_tasks': [asdict(task) for task in self.active_tasks.values()]
            }
            
            # 通知所有連接的客戶端
            disconnected = []
            for subscriber in self.subscribers:
                try:
                    await subscriber.send_text(json.dumps(data, default=str))
                except:
                    disconnected.append(subscriber)
            
            # 移除斷開的連接
            for subscriber in disconnected:
                self.subscribers.remove(subscriber)
                
        except Exception as e:
            self.log(f"通知訂閱者時發生錯誤: {e}", 'error')
    
    def add_subscriber(self, websocket):
        """添加 WebSocket 訂閱者"""
        self.subscribers.append(websocket)
        self.log(f"新增訂閱者，當前連接數: {len(self.subscribers)}")
    
    def remove_subscriber(self, websocket):
        """移除 WebSocket 訂閱者"""
        if websocket in self.subscribers:
            self.subscribers.remove(websocket)
            self.log(f"移除訂閱者，當前連接數: {len(self.subscribers)}")
    
    def get_current_data(self) -> Dict[str, Any]:
        """獲取當前監控數據"""
        return {
            'timestamp': datetime.now(timezone.utc).isoformat(),
            'metrics': asdict(self.current_metrics) if self.current_metrics else None,
            'agv_statuses': [asdict(agv) for agv in self.agv_statuses.values()],
            'active_tasks': [asdict(task) for task in self.active_tasks.values()],
            'historical_metrics': [asdict(metric) for metric in self.historical_metrics[-60:]]  # 最近1小時
        }
    
    def get_system_status(self) -> Dict[str, Any]:
        """獲取監控系統狀態"""
        return {
            'is_running': self.is_running,
            'subscribers_count': len(self.subscribers),
            'update_interval': self.config['update_interval'],
            'history_records': len(self.historical_metrics),
            'last_update': self.current_metrics.timestamp if self.current_metrics else None,
            'configuration': self.config
        }


# 全域監控實例
_monitor_instance = None

def get_monitor_instance(rcs_core=None) -> KukaFleetMonitor:
    """獲取監控實例（單例模式）"""
    global _monitor_instance
    if _monitor_instance is None:
        _monitor_instance = KukaFleetMonitor(rcs_core)
    return _monitor_instance