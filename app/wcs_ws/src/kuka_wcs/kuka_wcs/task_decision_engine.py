"""
KUKA WCS 任務判斷引擎
負責分析 KUKA AGV 狀態並做出任務分配決策
"""

import json
from enum import Enum
from typing import List, Dict, Optional, Any
from dataclasses import dataclass
from datetime import datetime, timezone


class TaskPriority(Enum):
    """任務優先級"""
    LOW = 1
    NORMAL = 5
    HIGH = 10
    URGENT = 20
    EMERGENCY = 50


class RobotStatus(Enum):
    """機器人狀態"""
    IDLE = "IDLE"
    BUSY = "BUSY"
    CHARGING = "CHARGING"
    ERROR = "ERROR"
    MAINTENANCE = "MAINTENANCE"


class ContainerStatus(Enum):
    """容器狀態"""
    EMPTY = 0
    FULL = 1
    IN_MAP = 1
    OUT_MAP = 0


@dataclass
class RobotInfo:
    """機器人資訊"""
    robot_id: str
    status: str
    battery_level: int
    current_position: Optional[str] = None
    assigned_task: Optional[str] = None
    last_update: Optional[datetime] = None


@dataclass
class ContainerInfo:
    """容器資訊"""
    container_code: str
    container_model_code: str
    node_code: str
    empty_full_status: int
    in_map_status: int
    is_carry: int
    last_update: Optional[datetime] = None


@dataclass
class TaskRequest:
    """任務請求"""
    task_id: str
    task_type: str
    priority: TaskPriority
    source_location: str
    target_location: str
    container_code: Optional[str] = None
    robot_requirements: Optional[Dict[str, Any]] = None
    created_at: Optional[datetime] = None


class TaskDecisionEngine:
    """任務判斷引擎"""
    
    def __init__(self, logger):
        self.logger = logger
        self.robots: Dict[str, RobotInfo] = {}
        self.containers: Dict[str, ContainerInfo] = {}
        self.pending_tasks: List[TaskRequest] = []
        self.active_tasks: Dict[str, TaskRequest] = {}
        
    def update_robot_status(self, robots_data: List[Dict]):
        """更新機器人狀態"""
        for robot_data in robots_data:
            robot_id = robot_data.get('robotId', '')
            if robot_id:
                self.robots[robot_id] = RobotInfo(
                    robot_id=robot_id,
                    status=robot_data.get('status', 'UNKNOWN'),
                    battery_level=robot_data.get('batteryLevel', 0),
                    current_position=robot_data.get('currentPosition'),
                    assigned_task=robot_data.get('assignedTask'),
                    last_update=datetime.now(timezone.utc)
                )
                
        self.logger.debug(f"已更新 {len(robots_data)} 台機器人狀態")
        
    def update_container_status(self, containers_data: List[Dict]):
        """更新容器狀態"""
        for container_data in containers_data:
            container_code = container_data.get('containerCode', '')
            if container_code:
                self.containers[container_code] = ContainerInfo(
                    container_code=container_code,
                    container_model_code=container_data.get('containerModelCode', ''),
                    node_code=container_data.get('nodeCode', ''),
                    empty_full_status=container_data.get('emptyFullStatus', 0),
                    in_map_status=container_data.get('inMapStatus', 0),
                    is_carry=container_data.get('isCarry', 0),
                    last_update=datetime.now(timezone.utc)
                )
                
        self.logger.debug(f"已更新 {len(containers_data)} 個容器狀態")

    def update_pending_tasks(self, tasks_from_db):
        """從資料庫更新待處理任務列表"""
        self.pending_tasks.clear()

        for task_data in tasks_from_db:
            # 將資料庫任務轉換為 TaskRequest
            task_request = TaskRequest(
                task_id=str(task_data.id),
                task_type="TRANSPORT",  # 預設類型
                priority=TaskPriority(min(task_data.priority, 50)),  # 確保優先級在範圍內
                source_location=task_data.parameters,  # 可能需要解析 JSON
                target_location=str(task_data.node_id),
                container_code=None,
                robot_requirements=None,
                created_at=datetime.now(timezone.utc)
            )
            self.pending_tasks.append(task_request)

        self.logger.debug(f"已更新 {len(self.pending_tasks)} 個待處理任務")

    def add_task_request(self, task_request: TaskRequest):
        """添加任務請求"""
        self.pending_tasks.append(task_request)
        self.logger.info(f"新增任務請求: {task_request.task_id}, 優先級: {task_request.priority.name}")
        
    def get_available_robots(self) -> List[RobotInfo]:
        """獲取可用的機器人"""
        available_robots = []
        for robot in self.robots.values():
            if (robot.status == RobotStatus.IDLE.value and 
                robot.battery_level > 20):  # 電量大於20%
                available_robots.append(robot)
        return available_robots
        
    def get_idle_containers(self) -> List[ContainerInfo]:
        """獲取閒置的容器"""
        idle_containers = []
        for container in self.containers.values():
            if (container.in_map_status == ContainerStatus.IN_MAP.value and 
                container.is_carry == 0):  # 在地圖中且未被搬運
                idle_containers.append(container)
        return idle_containers
        
    def calculate_task_priority_score(self, task: TaskRequest) -> float:
        """計算任務優先級分數"""
        base_score = task.priority.value
        
        # 根據任務創建時間調整優先級（等待越久優先級越高）
        if task.created_at:
            wait_time = (datetime.now(timezone.utc) - task.created_at).total_seconds()
            time_bonus = min(wait_time / 3600, 5)  # 最多加5分，每小時加1分
            base_score += time_bonus
            
        return base_score
        
    def select_best_robot_for_task(self, task: TaskRequest) -> Optional[RobotInfo]:
        """為任務選擇最佳機器人"""
        available_robots = self.get_available_robots()
        
        if not available_robots:
            return None
            
        # 簡單的選擇邏輯：選擇電量最高的機器人
        best_robot = max(available_robots, key=lambda r: r.battery_level)
        
        # 檢查機器人是否符合任務要求
        if task.robot_requirements:
            # 這裡可以添加更複雜的匹配邏輯
            pass
            
        return best_robot
        
    def make_task_decisions(self) -> List[Dict[str, Any]]:
        """進行任務決策，返回任務分配建議"""
        decisions = []
        
        if not self.pending_tasks:
            return decisions
            
        # 按優先級排序任務
        sorted_tasks = sorted(
            self.pending_tasks, 
            key=self.calculate_task_priority_score, 
            reverse=True
        )
        
        for task in sorted_tasks[:]:  # 使用切片複製避免修改迭代中的列表
            best_robot = self.select_best_robot_for_task(task)
            
            if best_robot:
                decision = {
                    'task_id': task.task_id,
                    'robot_id': best_robot.robot_id,
                    'task_type': task.task_type,
                    'source_location': task.source_location,
                    'target_location': task.target_location,
                    'container_code': task.container_code,
                    'priority': task.priority.value,
                    'decision_time': datetime.now(timezone.utc).isoformat()
                }
                
                decisions.append(decision)
                
                # 將任務從待處理移到活動任務
                self.active_tasks[task.task_id] = task
                self.pending_tasks.remove(task)
                
                # 標記機器人為忙碌
                best_robot.assigned_task = task.task_id
                
                self.logger.info(f"任務分配決策: 任務 {task.task_id} 分配給機器人 {best_robot.robot_id}")
                
        return decisions
        
    def get_system_status(self) -> Dict[str, Any]:
        """獲取系統狀態摘要"""
        available_robots = self.get_available_robots()
        idle_containers = self.get_idle_containers()
        
        return {
            'total_robots': len(self.robots),
            'available_robots': len(available_robots),
            'total_containers': len(self.containers),
            'idle_containers': len(idle_containers),
            'pending_tasks': len(self.pending_tasks),
            'active_tasks': len(self.active_tasks),
            'last_update': datetime.now(timezone.utc).isoformat()
        }
