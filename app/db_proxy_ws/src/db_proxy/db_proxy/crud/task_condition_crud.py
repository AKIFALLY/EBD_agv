"""
TaskCondition CRUD 操作模組
提供任務條件資料表的基本 CRUD 功能
"""

from sqlmodel import Session, select
from db_proxy.models import TaskCondition
from db_proxy.crud.base_crud import BaseCRUD
from typing import List, Optional, Dict, Any
from datetime import datetime


class TaskConditionCRUD(BaseCRUD):
    """TaskCondition 專用 CRUD 類別"""
    
    def __init__(self):
        super().__init__(TaskCondition, id_column="id")
    
    def create_condition(self, session: Session,id: int, conditions: str, results: Optional[Dict[str, Any]] = None , description: Optional[str] = None) -> TaskCondition:
        """
        建立新的任務條件記錄
        
        Args:
            session: 資料庫會話
            conditions: 條件內容（TEXT 格式）
            results: 結果資料（JSONB 格式）
            description: 條件描述
            
        Returns:
            TaskCondition: 建立的任務條件記錄
        """
        task_condition = TaskCondition(
            id=id,
            conditions=conditions,
            results=results or {} ,
            description=description
        )
        return self.create(session, task_condition)
    
    def get_by_conditions(self, session: Session, conditions: str) -> Optional[TaskCondition]:
        """
        根據條件內容查詢任務條件記錄
        
        Args:
            session: 資料庫會話
            conditions: 條件內容
            
        Returns:
            Optional[TaskCondition]: 找到的任務條件記錄，如果沒有則返回 None
        """
        statement = select(TaskCondition).where(TaskCondition.conditions == conditions)
        return session.exec(statement).first()
    
    def search_conditions(self, session: Session, keyword: str) -> List[TaskCondition]:
        """
        搜尋包含關鍵字的條件記錄
        
        Args:
            session: 資料庫會話
            keyword: 搜尋關鍵字
            
        Returns:
            List[TaskCondition]: 符合條件的記錄列表
        """
        statement = select(TaskCondition).where(TaskCondition.conditions.contains(keyword))
        return list(session.exec(statement).all())
    
    def update_results(self, session: Session, condition_id: int, results: Dict[str, Any]) -> Optional[TaskCondition]:
        """
        更新任務條件的結果資料
        
        Args:
            session: 資料庫會話
            condition_id: 條件記錄 ID
            results: 新的結果資料
            
        Returns:
            Optional[TaskCondition]: 更新後的記錄，如果記錄不存在則返回 None
        """
        task_condition = self.get_by_id(session, condition_id)
        if task_condition:
            task_condition.results = results
            session.add(task_condition)
            session.commit()
            session.refresh(task_condition)
            return task_condition
        return None
    
    def get_all_with_results(self, session: Session) -> List[TaskCondition]:
        """
        取得所有有結果資料的條件記錄
        
        Args:
            session: 資料庫會話
            
        Returns:
            List[TaskCondition]: 有結果資料的記錄列表
        """
        statement = select(TaskCondition).where(TaskCondition.results.is_not(None))
        return list(session.exec(statement).all())
    
    def delete_by_conditions(self, session: Session, conditions: str) -> bool:
        """
        根據條件內容刪除記錄

        Args:
            session: 資料庫會話
            conditions: 條件內容

        Returns:
            bool: 是否成功刪除
        """
        task_condition = self.get_by_conditions(session, conditions)
        if task_condition:
            return self.delete(session, task_condition.id)
        return False

    def search_by_description(self, session: Session, keyword: str) -> List[TaskCondition]:
        """
        根據描述搜尋條件記錄

        Args:
            session: 資料庫會話
            keyword: 搜尋關鍵字

        Returns:
            List[TaskCondition]: 符合條件的記錄列表
        """
        statement = select(TaskCondition).where(
            TaskCondition.description.contains(keyword)
        )
        return list(session.exec(statement).all())

    def update_description(self, session: Session, condition_id: int, description: str) -> Optional[TaskCondition]:
        """
        更新任務條件的描述

        Args:
            session: 資料庫會話
            condition_id: 條件記錄 ID
            description: 新的描述

        Returns:
            Optional[TaskCondition]: 更新後的記錄，如果記錄不存在則返回 None
        """
        task_condition = self.get_by_id(session, condition_id)
        if task_condition:
            task_condition.description = description
            session.add(task_condition)
            session.commit()
            session.refresh(task_condition)
            return task_condition
        return None

    def get_conditions_with_description(self, session: Session) -> List[TaskCondition]:
        """
        取得所有有描述的條件記錄

        Args:
            session: 資料庫會話

        Returns:
            List[TaskCondition]: 有描述的記錄列表
        """
        statement = select(TaskCondition).where(
            TaskCondition.description.is_not(None),
            TaskCondition.description != ""
        )
        return list(session.exec(statement).all())


# 建立全域 CRUD 實例
task_condition_crud = TaskConditionCRUD()
