# database/log_ops.py
"""
日誌相關資料庫操作
"""

from datetime import datetime
from sqlmodel import select, func
from db_proxy.models import RosoutLog, RuntimeLog
from .connection import connection_pool


def get_rosout_logs(offset: int = 0, limit: int = 20, level: int = None,
                    node_name: str = None, start_time: datetime = None,
                    end_time: datetime = None, message_filter: str = None):
    """獲取 ROS 輸出日誌（分頁、篩選）"""
    with connection_pool.get_session() as session:
        statement = select(RosoutLog)

        # 添加篩選條件
        if level is not None:
            statement = statement.where(RosoutLog.level == level)
        if node_name:
            statement = statement.where(RosoutLog.name.ilike(f"%{node_name}%"))
        if start_time:
            statement = statement.where(RosoutLog.timestamp >= start_time)
        if end_time:
            statement = statement.where(RosoutLog.timestamp <= end_time)
        if message_filter:
            statement = statement.where(
                RosoutLog.message.ilike(f"%{message_filter}%"))

        statement = statement.order_by(
            RosoutLog.timestamp.desc()).offset(offset).limit(limit)
        return session.exec(statement).all()


def count_rosout_logs(level: int = None, node_name: str = None,
                      start_time: datetime = None, end_time: datetime = None,
                      message_filter: str = None):
    """計算 ROS 輸出日誌總數（含篩選）"""
    with connection_pool.get_session() as session:
        statement = select(func.count()).select_from(RosoutLog)

        # 添加篩選條件
        if level is not None:
            statement = statement.where(RosoutLog.level == level)
        if node_name:
            statement = statement.where(RosoutLog.name.ilike(f"%{node_name}%"))
        if start_time:
            statement = statement.where(RosoutLog.timestamp >= start_time)
        if end_time:
            statement = statement.where(RosoutLog.timestamp <= end_time)
        if message_filter:
            statement = statement.where(
                RosoutLog.message.ilike(f"%{message_filter}%"))

        return session.exec(statement).one()


def get_rosout_node_names():
    """獲取所有不同的節點名稱"""
    with connection_pool.get_session() as session:
        statement = select(RosoutLog.name).distinct().where(
            RosoutLog.name.is_not(None))
        return session.exec(statement).all()


def get_runtime_logs(offset: int = 0, limit: int = 20, level: int = None,
                     node_name: str = None, start_time: datetime = None,
                     end_time: datetime = None, message_filter: str = None):
    """獲取運行時日誌（分頁、篩選）"""
    with connection_pool.get_session() as session:
        statement = select(RuntimeLog)

        # 添加篩選條件
        if level is not None:
            statement = statement.where(RuntimeLog.level == level)
        if node_name:
            statement = statement.where(
                RuntimeLog.name.ilike(f"%{node_name}%"))
        if start_time:
            statement = statement.where(RuntimeLog.timestamp >= start_time)
        if end_time:
            statement = statement.where(RuntimeLog.timestamp <= end_time)
        if message_filter:
            statement = statement.where(
                RuntimeLog.message.ilike(f"%{message_filter}%"))

        statement = statement.order_by(
            RuntimeLog.timestamp.desc()).offset(offset).limit(limit)
        return session.exec(statement).all()


def count_runtime_logs(level: int = None, node_name: str = None,
                       start_time: datetime = None, end_time: datetime = None,
                       message_filter: str = None):
    """計算運行時日誌總數（含篩選）"""
    with connection_pool.get_session() as session:
        statement = select(func.count()).select_from(RuntimeLog)

        # 添加篩選條件
        if level is not None:
            statement = statement.where(RuntimeLog.level == level)
        if node_name:
            statement = statement.where(
                RuntimeLog.name.ilike(f"%{node_name}%"))
        if start_time:
            statement = statement.where(RuntimeLog.timestamp >= start_time)
        if end_time:
            statement = statement.where(RuntimeLog.timestamp <= end_time)
        if message_filter:
            statement = statement.where(
                RuntimeLog.message.ilike(f"%{message_filter}%"))

        return session.exec(statement).one()


def get_runtime_node_names():
    """獲取所有不重複的 runtime log 節點名稱"""
    with connection_pool.get_session() as session:
        statement = select(RuntimeLog.name).distinct().where(
            RuntimeLog.name.is_not(None))
        return session.exec(statement).all()
