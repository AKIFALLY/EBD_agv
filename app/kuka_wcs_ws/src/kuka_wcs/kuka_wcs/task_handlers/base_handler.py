"""
基础任务处理器 - 所有任务处理器的抽象基类

定义任务处理器的统一接口
"""
from abc import ABC, abstractmethod
from typing import List
from datetime import datetime
from sqlmodel import Session
from db_proxy.models import Task


class BaseHandler(ABC):
    """任务处理器基类"""

    def __init__(self, node):
        """
        初始化处理器

        Args:
            node: KukaWcsNode 实例
        """
        self.node = node
        self.logger = node.get_logger()
        self.db_pool = node.db_pool

        # 初始化数据库桥接（延迟导入避免循环依赖）
        from kuka_wcs.db_bridge import KukaWcsDbBridge
        self.db = KukaWcsDbBridge(self.logger)

    @abstractmethod
    def check_and_create_tasks(self, session: Session) -> List[Task]:
        """
        检查条件并创建任务（抽象方法）

        子类必须实现此方法，执行以下操作：
        1. 查询相关数据（locations, racks, carriers 等）
        2. 检查业务逻辑条件
        3. 如果条件满足，创建相应的任务
        4. 返回创建的任务列表

        Args:
            session: 数据库 session

        Returns:
            创建的 Task 对象列表（可能为空列表）
        """
        pass

    @abstractmethod
    def check_and_mark_completed_tasks(self, session: Session) -> int:
        """
        检查并标记已完成的任务（抽象方法）

        子类必须实现此方法，执行以下操作：
        1. 查询相关数据（locations, racks, 执行中的任务等）
        2. 检查任务完成条件
        3. 如果条件满足，标记任务为完成状态（status_id=4）
        4. 返回标记为完成的任务数量

        Args:
            session: 数据库 session

        Returns:
            标记为完成的任务数量
        """
        pass

    def _mark_task_completed(
        self,
        session: Session,
        task: Task,
        reason: str
    ) -> bool:
        """
        将任务标记为完成状态（辅助方法）

        Args:
            session: 数据库 session
            task: 要标记的任务
            reason: 标记原因（用于日志）

        Returns:
            是否成功标记
        """
        try:
            task.status_id = 4  # COMPLETED
            task.updated_at = datetime.utcnow()
            session.add(task)
            session.commit()

            self.logger.info(
                f"✅ 标记任务完成: Task ID={task.id}, "
                f"Rack ID={task.rack_id}, Location ID={task.location_id}, "
                f"原因: {reason}"
            )
            return True
        except Exception as e:
            self.logger.error(f"标记任务失败: Task ID={task.id}, 错误: {e}")
            session.rollback()
            return False

    def get_handler_name(self) -> str:
        """获取处理器名称（用于日志）"""
        return self.__class__.__name__
