#!/usr/bin/env python3
"""
任務條件查詢命令列工具
提供手動執行、測試和管理任務條件查詢的命令列介面
"""

import argparse
import json
import sys
from datetime import datetime, timezone
from typing import Dict, Any

from db_proxy.models import TaskCondition
from db_proxy.crud.task_condition_crud import task_condition_crud
from db_proxy.connection_pool_manager import ConnectionPoolManager
from wcs_base.task_condition_query_service import TaskConditionQueryService


class TaskConditionQueryCLI:
    """任務條件查詢命令列介面"""
    
    def __init__(self, db_url: str):
        """
        初始化 CLI
        
        Args:
            db_url: 資料庫連接字串
        """
        self.db_url = db_url
        self.pool = ConnectionPoolManager(db_url)
        
        # 建立簡單的日誌記錄器
        class SimpleLogger:
            def info(self, msg): print(f"[INFO] {msg}")
            def warning(self, msg): print(f"[WARNING] {msg}")
            def error(self, msg): print(f"[ERROR] {msg}")
            def debug(self, msg): print(f"[DEBUG] {msg}")
        
        self.logger = SimpleLogger()
        self.query_service = TaskConditionQueryService(self, self.logger)
    
    def list_conditions(self) -> None:
        """列出所有條件記錄"""
        print("📋 任務條件列表")
        print("=" * 80)
        
        try:
            with self.pool.get_session() as session:
                conditions = task_condition_crud.get_all(session)
                
                if not conditions:
                    print("沒有找到任何條件記錄")
                    return
                
                for condition in conditions:
                    print(f"ID: {condition.id}")
                    print(f"條件: {condition.conditions}")
                    if condition.description:
                        print(f"描述: {condition.description}")

                    if condition.results:
                        if condition.results.get("success", False):
                            row_count = condition.results.get("row_count", 0)
                            print(f"狀態: ✅ 成功 ({row_count} 行)")
                        else:
                            error = condition.results.get("error", "未知錯誤")
                            print(f"狀態: ❌ 失敗 - {error}")
                        
                        timestamp = condition.results.get("timestamp", "")
                        if timestamp:
                            print(f"更新時間: {timestamp}")
                    else:
                        print("狀態: ⏳ 尚未執行")
                    
                    print("-" * 80)
                    
        except Exception as e:
            print(f"❌ 列出條件失敗: {e}")
    
    def add_condition(self, conditions: str, description: str = "") -> None:
        """
        新增條件記錄
        
        Args:
            conditions: 條件 SQL 語句
            description: 條件描述
        """
        print(f"➕ 新增條件: {conditions}")
        
        try:
            # 驗證 SQL
            is_valid, error_msg = self.query_service.validate_sql_query(conditions)
            if not is_valid:
                print(f"❌ SQL 驗證失敗: {error_msg}")
                return
            
            with self.pool.get_session() as session:
                initial_results = {
                    "created_at": datetime.now(timezone.utc).isoformat(),
                    "status": "pending"
                }

                condition = task_condition_crud.create_condition(
                    session, conditions, initial_results, description
                )
                
                print(f"✅ 條件新增成功，ID: {condition.id}")
                
        except Exception as e:
            print(f"❌ 新增條件失敗: {e}")
    
    def execute_condition(self, condition_id: int) -> None:
        """
        執行指定的條件
        
        Args:
            condition_id: 條件 ID
        """
        print(f"🔍 執行條件 ID: {condition_id}")
        
        try:
            with self.pool.get_session() as session:
                condition = task_condition_crud.get_by_id(session, condition_id)
                
                if not condition:
                    print(f"❌ 找不到條件 ID: {condition_id}")
                    return
                
                print(f"條件內容: {condition.conditions}")
                
                # 執行條件
                success = self.query_service.process_single_condition(condition)
                
                if success:
                    # 重新取得更新後的條件
                    updated_condition = task_condition_crud.get_by_id(session, condition_id)
                    if updated_condition and updated_condition.results:
                        if updated_condition.results.get("success", False):
                            row_count = updated_condition.results.get("row_count", 0)
                            print(f"✅ 執行成功，返回 {row_count} 行資料")
                            
                            # 顯示部分結果
                            data = updated_condition.results.get("data", [])
                            if data and len(data) > 0:
                                print("📊 查詢結果預覽:")
                                print(json.dumps(data[:3], ensure_ascii=False, indent=2))
                                if len(data) > 3:
                                    print(f"... 還有 {len(data) - 3} 行資料")
                        else:
                            error = updated_condition.results.get("error", "未知錯誤")
                            print(f"❌ 執行失敗: {error}")
                else:
                    print("❌ 執行失敗")
                    
        except Exception as e:
            print(f"❌ 執行條件失敗: {e}")
    
    def execute_all(self) -> None:
        """執行所有條件"""
        print("🚀 執行所有條件")
        print("=" * 50)
        
        try:
            result = self.query_service.process_all_conditions()
            
            print(f"📊 執行結果:")
            print(f"  總條件數: {result.get('total_conditions', 0)}")
            print(f"  處理成功: {result.get('successful', 0)}")
            print(f"  處理失敗: {result.get('failed', 0)}")
            print(f"  執行時間: {result.get('duration_seconds', 0):.2f} 秒")
            
            if result.get("error"):
                print(f"❌ 錯誤: {result['error']}")
            
        except Exception as e:
            print(f"❌ 執行所有條件失敗: {e}")
    
    def test_sql(self, sql_query: str) -> None:
        """
        測試 SQL 查詢
        
        Args:
            sql_query: 要測試的 SQL 查詢
        """
        print(f"🧪 測試 SQL: {sql_query}")
        print("-" * 50)
        
        try:
            # 驗證 SQL
            is_valid, error_msg = self.query_service.validate_sql_query(sql_query)
            if not is_valid:
                print(f"❌ SQL 驗證失敗: {error_msg}")
                return
            
            print("✅ SQL 驗證通過")
            
            # 執行查詢
            result = self.query_service.execute_sql_query(sql_query)
            
            if result.get("success", False):
                row_count = result.get("row_count", 0)
                print(f"✅ 查詢成功，返回 {row_count} 行資料")
                
                # 顯示結果
                data = result.get("data", [])
                if data:
                    print("📊 查詢結果:")
                    print(json.dumps(data[:5], ensure_ascii=False, indent=2))
                    if len(data) > 5:
                        print(f"... 還有 {len(data) - 5} 行資料")
                else:
                    print("📋 查詢無返回資料")
            else:
                error = result.get("error", "未知錯誤")
                print(f"❌ 查詢失敗: {error}")
                
        except Exception as e:
            print(f"❌ 測試 SQL 失敗: {e}")


def main():
    """主函式"""
    parser = argparse.ArgumentParser(description="任務條件查詢命令列工具")
    parser.add_argument(
        "--db-url", 
        default="postgresql+psycopg2://agvc:password@192.168.100.254/agvc",
        help="資料庫連接字串"
    )
    
    subparsers = parser.add_subparsers(dest="command", help="可用命令")
    
    # 列出條件
    subparsers.add_parser("list", help="列出所有條件")
    
    # 新增條件
    add_parser = subparsers.add_parser("add", help="新增條件")
    add_parser.add_argument("conditions", help="條件 SQL 語句")
    add_parser.add_argument("--description", help="條件描述")
    
    # 執行指定條件
    exec_parser = subparsers.add_parser("execute", help="執行指定條件")
    exec_parser.add_argument("condition_id", type=int, help="條件 ID")
    
    # 執行所有條件
    subparsers.add_parser("execute-all", help="執行所有條件")
    
    # 測試 SQL
    test_parser = subparsers.add_parser("test", help="測試 SQL 查詢")
    test_parser.add_argument("sql", help="要測試的 SQL 查詢")
    
    args = parser.parse_args()
    
    if not args.command:
        parser.print_help()
        return
    
    try:
        cli = TaskConditionQueryCLI(args.db_url)
        
        if args.command == "list":
            cli.list_conditions()
        elif args.command == "add":
            cli.add_condition(args.conditions, args.description or "")
        elif args.command == "execute":
            cli.execute_condition(args.condition_id)
        elif args.command == "execute-all":
            cli.execute_all()
        elif args.command == "test":
            cli.test_sql(args.sql)
            
    except KeyboardInterrupt:
        print("\n🛑 操作已取消")
    except Exception as e:
        print(f"❌ 執行失敗: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
