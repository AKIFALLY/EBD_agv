"""
基础任务处理器 - 所有任务处理器的抽象基类

定义任务处理器的统一接口
"""
from abc import ABC, abstractmethod
from typing import List
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

    def get_handler_name(self) -> str:
        """获取处理器名称（用于日志）"""
        return self.__class__.__name__
