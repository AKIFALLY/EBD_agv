"""
Runtime Log 辅助函数
提供统一的任务操作日志记录接口
"""
from typing import Optional
from datetime import datetime, timezone
import inspect
from db_proxy.models import RuntimeLog, LogLevel
from sqlmodel import Session


class TaskLogHelper:
    """任务操作日志助手"""

    @staticmethod
    def log_task_create_success(
        session: Session,
        task_id: int,
        work_id: int,
        status_id: int,
        task_name: Optional[str] = None,
        room_id: Optional[int] = None,
        agv_type: Optional[str] = None,
        rack_id: Optional[int] = None,
        location_id: Optional[int] = None,
        node_name: str = "task_builder"
    ):
        """
        记录任务创建成功

        Args:
            session: 数据库 session
            task_id: 创建的任务 ID
            work_id: Work ID
            status_id: 任务状态 ID
            task_name: 任务名称 (可选)
            room_id: Room ID (可选)
            agv_type: AGV 类型 (可选)
            rack_id: Rack ID (可选)
            location_id: Location ID (可选)
            node_name: 节点名称
        """
        # 构建消息
        parts = [
            f"Task ID={task_id}",
            f"Work ID={work_id}",
            f"Status ID={status_id}"
        ]
        if task_name:
            parts.append(f"Name={task_name}")
        if room_id is not None:
            parts.append(f"Room ID={room_id}")
        if agv_type:
            parts.append(f"AGV Type={agv_type}")
        if rack_id is not None:
            parts.append(f"Rack ID={rack_id}")
        if location_id is not None:
            parts.append(f"Location ID={location_id}")

        message = "[TASK_CREATE_SUCCESS] " + ", ".join(parts)

        # 获取调用者信息
        frame = inspect.stack()[1]

        # 创建 RuntimeLog
        log = RuntimeLog(
            timestamp=datetime.now(timezone.utc),
            level=LogLevel.INFO,
            name=node_name,
            message=message,
            file=frame.filename,
            function=frame.function,
            line=frame.lineno,
            created_at=datetime.now(timezone.utc)
        )

        session.add(log)
        # 注意：不在此处 commit，由调用者统一 commit

    @staticmethod
    def log_task_create_skip(
        session: Session,
        work_id: int,
        room_id: Optional[int] = None,
        location_id: Optional[int] = None,
        reason: str = "已有未完成任务存在",
        node_name: str = "task_builder"
    ):
        """
        记录任务创建跳过（防重）

        Args:
            session: 数据库 session
            work_id: Work ID
            room_id: Room ID (可选)
            location_id: Location ID (可选)
            reason: 跳过原因
            node_name: 节点名称
        """
        parts = [f"Work ID={work_id}"]
        if room_id is not None:
            parts.append(f"Room ID={room_id}")
        if location_id is not None:
            parts.append(f"Location ID={location_id}")

        message = f"[TASK_CREATE_SKIP] {', '.join(parts)}: {reason}"

        frame = inspect.stack()[1]

        log = RuntimeLog(
            timestamp=datetime.now(timezone.utc),
            level=LogLevel.WARN,
            name=node_name,
            message=message,
            file=frame.filename,
            function=frame.function,
            line=frame.lineno,
            created_at=datetime.now(timezone.utc)
        )

        session.add(log)

    @staticmethod
    def log_task_create_error(
        session: Session,
        work_id: int,
        error: str,
        room_id: Optional[int] = None,
        node_name: str = "task_builder"
    ):
        """
        记录任务创建异常

        Args:
            session: 数据库 session
            work_id: Work ID
            error: 错误消息
            room_id: Room ID (可选)
            node_name: 节点名称
        """
        parts = [f"Work ID={work_id}"]
        if room_id is not None:
            parts.append(f"Room ID={room_id}")

        message = f"[TASK_CREATE_ERROR] {', '.join(parts)}: {error}"

        frame = inspect.stack()[1]

        log = RuntimeLog(
            timestamp=datetime.now(timezone.utc),
            level=LogLevel.ERROR,
            name=node_name,
            message=message,
            file=frame.filename,
            function=frame.function,
            line=frame.lineno,
            created_at=datetime.now(timezone.utc)
        )

        session.add(log)

    @staticmethod
    def log_task_delete_success(
        session: Session,
        task_id: int,
        work_id: int,
        status_id: int,
        task_name: Optional[str] = None,
        node_name: str = "task_cleaner"
    ):
        """
        记录任务删除成功

        Args:
            session: 数据库 session
            task_id: 删除的任务 ID
            work_id: Work ID
            status_id: 任务状态 ID
            task_name: 任务名称 (可选)
            node_name: 节点名称
        """
        parts = [
            f"Task ID={task_id}",
            f"Work ID={work_id}",
            f"Status ID={status_id}"
        ]
        if task_name:
            parts.append(f"Name={task_name}")

        message = "[TASK_DELETE_SUCCESS] " + ", ".join(parts)

        frame = inspect.stack()[1]

        log = RuntimeLog(
            timestamp=datetime.now(timezone.utc),
            level=LogLevel.INFO,
            name=node_name,
            message=message,
            file=frame.filename,
            function=frame.function,
            line=frame.lineno,
            created_at=datetime.now(timezone.utc)
        )

        session.add(log)

    @staticmethod
    def log_task_delete_error(
        session: Session,
        task_id: int,
        error: str,
        node_name: str = "task_cleaner"
    ):
        """
        记录任务删除异常

        Args:
            session: 数据库 session
            task_id: 任务 ID
            error: 错误消息
            node_name: 节点名称
        """
        message = f"[TASK_DELETE_ERROR] Task ID={task_id}: {error}"

        frame = inspect.stack()[1]

        log = RuntimeLog(
            timestamp=datetime.now(timezone.utc),
            level=LogLevel.ERROR,
            name=node_name,
            message=message,
            file=frame.filename,
            function=frame.function,
            line=frame.lineno,
            created_at=datetime.now(timezone.utc)
        )

        session.add(log)
