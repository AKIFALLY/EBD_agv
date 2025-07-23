"""
任務條件查詢服務
從 task_condition 資料表中讀取條件並執行動態 SQL 查詢
"""

import re
import json
import asyncio
from typing import List, Dict, Any, Optional
from datetime import datetime, timezone
from sqlmodel import Session, text
from sqlalchemy.exc import SQLAlchemyError
import rclpy
from rclpy.node import Node

from db_proxy.models import TaskCondition
from db_proxy.crud.task_condition_crud import task_condition_crud
from db_proxy.connection_pool_manager import ConnectionPoolManager


class TaskConditionQueryService:
    """任務條件查詢服務類別"""
    
    def __init__(self, db_manager, logger):
        """
        初始化查詢服務
        
        Args:
            db_manager: 資料庫管理器
            logger: 日誌記錄器
        """
        self.db_manager = db_manager
        self.logger = logger
        self.query_timeout = 30  # 查詢超時時間（秒）
        
        # SQL 安全性設定
        self.allowed_keywords = {
            'SELECT', 'FROM', 'WHERE', 'AND', 'OR', 'NOT', 'IN', 'LIKE', 
            'BETWEEN', 'IS', 'NULL', 'ORDER', 'BY', 'GROUP', 'HAVING',
            'LIMIT', 'OFFSET', 'JOIN', 'INNER', 'LEFT', 'RIGHT', 'ON',
            'AS', 'DISTINCT', 'COUNT', 'SUM', 'AVG', 'MAX', 'MIN'
        }
        
        self.forbidden_keywords = {
            'INSERT', 'UPDATE', 'DELETE', 'DROP', 'CREATE', 'ALTER',
            'TRUNCATE', 'EXEC', 'EXECUTE', 'DECLARE', 'CURSOR'
        }
    
    def validate_sql_query(self, sql_query: str) -> tuple[bool, str]:
        """
        驗證 SQL 查詢的安全性
        
        Args:
            sql_query: 要驗證的 SQL 查詢
            
        Returns:
            tuple: (是否有效, 錯誤訊息)
        """
        if not sql_query or not sql_query.strip():
            return False, "SQL 查詢不能為空"
        
        # 移除註解和多餘空白
        cleaned_sql = re.sub(r'--.*$', '', sql_query, flags=re.MULTILINE)
        cleaned_sql = re.sub(r'/\*.*?\*/', '', cleaned_sql, flags=re.DOTALL)
        cleaned_sql = ' '.join(cleaned_sql.split())
        
        # 檢查是否包含禁止的關鍵字
        sql_upper = cleaned_sql.upper()
        for forbidden in self.forbidden_keywords:
            if forbidden in sql_upper:
                return False, f"包含禁止的關鍵字: {forbidden}"
        
        # 檢查是否以 SELECT 開頭
        if not sql_upper.strip().startswith('SELECT'):
            return False, "只允許 SELECT 查詢"
        
        # 檢查是否包含分號（防止多語句執行）
        if ';' in cleaned_sql[:-1]:  # 允許結尾的分號
            return False, "不允許多語句執行"
        
        return True, ""
    
    def execute_sql_query(self, sql_query: str) -> Dict[str, Any]:
        """
        執行 SQL 查詢並返回結果
        
        Args:
            sql_query: 要執行的 SQL 查詢
            
        Returns:
            Dict: 查詢結果或錯誤資訊
        """
        try:
            # 驗證 SQL 安全性
            is_valid, error_msg = self.validate_sql_query(sql_query)
            if not is_valid:
                return {
                    "success": False,
                    "error": f"SQL 驗證失敗: {error_msg}",
                    "timestamp": datetime.now(timezone.utc).isoformat()
                }
            
            # 執行查詢
            with self.db_manager.get_session() as session:
                # 設定查詢超時
                session.execute(text(f"SET statement_timeout = {self.query_timeout * 1000}"))
                
                # 執行查詢
                result = session.execute(text(sql_query))
                
                # 處理結果
                if result.returns_rows:
                    # 取得欄位名稱
                    columns = list(result.keys())
                    # 取得所有資料行
                    rows = result.fetchall()
                    
                    # 轉換為字典列表
                    data = []
                    for row in rows:
                        row_dict = {}
                        for i, value in enumerate(row):
                            # 處理特殊資料類型
                            if isinstance(value, datetime):
                                row_dict[columns[i]] = value.isoformat()
                            else:
                                row_dict[columns[i]] = value
                        data.append(row_dict)
                    
                    return {
                        "success": True,
                        "data": data,
                        "row_count": len(data),
                        "columns": columns,
                        "timestamp": datetime.now(timezone.utc).isoformat()
                    }
                else:
                    return {
                        "success": True,
                        "message": "查詢執行成功，但無返回資料",
                        "timestamp": datetime.now(timezone.utc).isoformat()
                    }
                    
        except SQLAlchemyError as e:
            error_msg = f"資料庫錯誤: {str(e)}"
            self.logger.error(error_msg)
            return {
                "success": False,
                "error": error_msg,
                "timestamp": datetime.now(timezone.utc).isoformat()
            }
        except Exception as e:
            error_msg = f"執行錯誤: {str(e)}"
            self.logger.error(error_msg)
            return {
                "success": False,
                "error": error_msg,
                "timestamp": datetime.now(timezone.utc).isoformat()
            }
    
    def process_single_condition(self, condition: TaskCondition) -> bool:
        """
        處理單一條件記錄
        
        Args:
            condition: 條件記錄
            
        Returns:
            bool: 是否處理成功
        """
        try:
            #self.logger.info(f"🔍 處理條件 ID: {condition.id}")
            
            # 執行 SQL 查詢
            query_result = self.execute_sql_query(condition.conditions)
            
            # 更新結果到資料庫
            with self.db_manager.get_session() as session:
                updated_condition = task_condition_crud.update_results(
                    session, condition.id, query_result
                )
                
                if updated_condition:
                    if query_result.get("success", False):
                        #self.logger.info(f"✅ 條件 {condition.id} 查詢成功，返回 {query_result.get('row_count', 0)} 行資料")
                        pass
                    else:
                        self.logger.warning(f"⚠️ 條件 {condition.id} 查詢失敗: {query_result.get('error', '未知錯誤')}")
                    return True
                else:
                    self.logger.error(f"❌ 條件 {condition.id} 結果更新失敗")
                    return False
                    
        except Exception as e:
            error_msg = f"處理條件 {condition.id} 時發生錯誤: {str(e)}"
            self.logger.error(error_msg)
            
            # 記錄錯誤到資料庫
            try:
                with self.db_manager.get_session() as session:
                    error_result = {
                        "success": False,
                        "error": error_msg,
                        "timestamp": datetime.now(timezone.utc).isoformat()
                    }
                    task_condition_crud.update_results(session, condition.id, error_result)
            except Exception as update_error:
                self.logger.error(f"❌ 更新錯誤結果失敗: {str(update_error)}")
            
            return False
    
    def process_all_conditions(self) -> Dict[str, Any]:
        """
        處理所有條件記錄
        
        Returns:
            Dict: 處理結果統計
        """
        start_time = datetime.now(timezone.utc)
        self.logger.info("🚀 開始處理所有任務條件...")
        
        try:
            # 取得所有條件記錄
            with self.db_manager.get_session() as session:
                all_conditions = task_condition_crud.get_all(session)
            
            if not all_conditions:
                self.logger.info("📋 沒有找到任何條件記錄")
                return {
                    "total_conditions": 0,
                    "processed": 0,
                    "successful": 0,
                    "failed": 0,
                    "start_time": start_time.isoformat(),
                    "end_time": datetime.now(timezone.utc).isoformat()
                }
            
            # 處理統計
            total_conditions = len(all_conditions)
            successful_count = 0
            failed_count = 0
            
            self.logger.info(f"📊 找到 {total_conditions} 個條件記錄，開始批次處理...")
            
            # 批次處理條件
            for i, condition in enumerate(all_conditions, 1):
                #self.logger.info(f"📝 處理進度: {i}/{total_conditions}")
                
                if self.process_single_condition(condition):
                    successful_count += 1
                else:
                    failed_count += 1
            
            end_time = datetime.now(timezone.utc)
            duration = (end_time - start_time).total_seconds()
            
            result = {
                "total_conditions": total_conditions,
                "processed": successful_count + failed_count,
                "successful": successful_count,
                "failed": failed_count,
                "duration_seconds": duration,
                "start_time": start_time.isoformat(),
                "end_time": end_time.isoformat()
            }
            
            self.logger.info(f"✅ 批次處理完成: {successful_count} 成功, {failed_count} 失敗, 耗時 {duration:.2f} 秒")
            return result
            
        except Exception as e:
            error_msg = f"批次處理失敗: {str(e)}"
            self.logger.error(error_msg)
            return {
                "total_conditions": 0,
                "processed": 0,
                "successful": 0,
                "failed": 0,
                "error": error_msg,
                "start_time": start_time.isoformat(),
                "end_time": datetime.now(timezone.utc).isoformat()
            }
