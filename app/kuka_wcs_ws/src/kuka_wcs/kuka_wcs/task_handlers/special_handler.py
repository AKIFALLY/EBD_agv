"""
特殊任务处理器 - 清理错误状态的任务

功能：
- 监控 task 表中 work_id=220001 且 status_id=6（错误状态）的任务
- 物理删除这些错误任务
- 扫描间隔：5秒一次
- 日志策略：仅在删除时记录
"""
from typing import List, Dict, Any
from sqlmodel import Session, select
from db_proxy.models import Task
from db_proxy.utils.runtime_log_helper import TaskLogHelper
from .base_handler import BaseHandler
import time


class SpecialHandler(BaseHandler):
    """特殊任务处理器 - 清理错误状态的任务"""

    def __init__(self, node, config: Dict[str, Any]):
        """
        初始化特殊任务处理器

        Args:
            node: KukaWcsNode 实例
            config: 配置字典，包含以下键：
                - enabled: 是否启用
                - scan_interval: 扫描间隔（秒）
                - work_id: 监控的 work_id
                - error_status_id: 错误状态 ID
        """
        super().__init__(node)
        self.config = config
        self.scan_interval = config.get('scan_interval', 5.0)
        self.work_id = config.get('work_id', 220001)
        self.error_status_id = config.get('error_status_id', 6)

        # 最后扫描时间（用于控制扫描频率）
        self.last_scan_time = 0.0

        self.logger.info(
            f"✅ 初始化 SpecialHandler: "
            f"扫描间隔={self.scan_interval}秒, "
            f"work_id={self.work_id}, "
            f"error_status_id={self.error_status_id}"
        )

    def check_and_create_tasks(self, session: Session) -> List[Task]:
        """
        检查并删除错误状态的任务

        扫描逻辑：
        1. 检查是否到达扫描时间（每 scan_interval 秒执行一次）
        2. 查询 work_id=220001 且 status_id=6 的任务
        3. 物理删除这些任务（DELETE FROM task）
        4. 记录删除日志

        Args:
            session: 资料庫 session

        Returns:
            空列表（此 handler 不创建任务）
        """
        # 检查是否到达扫描时间
        current_time = time.time()
        if current_time - self.last_scan_time < self.scan_interval:
            return []

        self.last_scan_time = current_time

        # 查询错误状态的任务
        error_tasks = self._query_error_tasks(session)

        # 删除错误任务
        if error_tasks:
            self._delete_error_tasks(session, error_tasks)

        return []  # 不创建任务，返回空列表

    def check_and_mark_completed_tasks(self, session: Session) -> int:
        """
        检查并标记已完成的任务（此 handler 不需要此功能）

        Returns:
            0（不标记任何任务为完成）
        """
        return 0

    def _query_error_tasks(self, session: Session) -> List[Task]:
        """
        查询错误状态的任务

        查询条件：
        - work_id = self.work_id (默认 220001)
        - status_id = self.error_status_id (默认 6: 错误)

        Args:
            session: 资料庫 session

        Returns:
            错误任务列表
        """
        try:
            statement = select(Task).where(
                Task.work_id == self.work_id,
                Task.status_id == self.error_status_id
            )
            error_tasks = session.exec(statement).all()
            return error_tasks
        except Exception as e:
            self.logger.error(f"查询错误任务时出错: {e}")
            return []

    def _delete_error_tasks(self, session: Session, tasks: List[Task]):
        """
        删除错误任务

        Args:
            session: 资料庫 session
            tasks: 要删除的任务列表
        """
        deleted_count = 0

        for task in tasks:
            try:
                # 记录删除前的任务信息（用于日志）
                task_info = {
                    'id': task.id,
                    'work_id': task.work_id,
                    'status_id': task.status_id,
                    'rack_id': task.rack_id,
                    'location_id': task.location_id,
                    'name': task.name,
                    'created_at': task.created_at
                }

                # 物理删除任务
                session.delete(task)

                # 記錄任務刪除到 RuntimeLog
                TaskLogHelper.log_task_delete_success(
                    session=session,
                    task_id=task_info['id'],
                    work_id=task_info['work_id'],
                    status_id=task_info['status_id'],
                    task_name=task_info['name'],
                    node_name="kuka_wcs_special"
                )

                session.commit()

                # 记录删除日志
                self.logger.info(
                    f"🗑️  已删除错误任务: "
                    f"Task ID={task_info['id']}, "
                    f"Name={task_info['name']}, "
                    f"Work ID={task_info['work_id']}, "
                    f"Status ID={task_info['status_id']}, "
                    f"Rack ID={task_info['rack_id']}, "
                    f"Location ID={task_info['location_id']}, "
                    f"Created={task_info['created_at']}"
                )

                deleted_count += 1

            except Exception as e:
                # 記錄刪除錯誤到 RuntimeLog
                TaskLogHelper.log_task_delete_error(
                    session=session,
                    task_id=task.id,
                    error=str(e),
                    node_name="kuka_wcs_special"
                )
                try:
                    session.commit()  # 提交 RuntimeLog
                except:
                    pass  # 忽略 RuntimeLog 提交失敗

                self.logger.error(
                    f"删除任务 {task.id} 时出错: {e}"
                )
                session.rollback()

        if deleted_count > 0:
            self.logger.info(
                f"✅ 完成错误任务清理: 共删除 {deleted_count}/{len(tasks)} 个任务"
            )
