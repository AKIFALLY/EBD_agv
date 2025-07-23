"""
任務條件檢查器
提供基於 task_condition 表格的通用條件檢查功能
支援即時查詢模式和預存結果模式
"""

from db_proxy.crud.task_condition_crud import task_condition_crud
from sqlalchemy import text
from sqlalchemy.exc import SQLAlchemyError
import json
from datetime import datetime, timezone
from typing import Dict, Any, List, Optional, Tuple
from rclpy.node import Node
from .task_condition_config import get_current_config, TaskConditionConfig


class TaskConditionChecker:
    """
    任務條件檢查器

    提供基於 task_condition 表格的通用條件檢查功能，
    支援即時查詢模式和預存結果模式。
    """

    def __init__(self, db_manager, logger, config: Optional[TaskConditionConfig] = None, **kwargs):
        """
        初始化條件檢查器

        Args:
            db_manager: 資料庫管理器
            logger: 日誌記錄器
            config: 任務條件配置物件，如果為 None 則使用全域配置
            **kwargs: 額外的配置參數（會覆蓋 config 中的設定）
        """
        self.db_manager = db_manager
        self.logger = logger

        # 載入配置
        if config is None:
            config = get_current_config()

        # 應用配置
        self.real_time_mode = kwargs.get('real_time_mode', config.real_time_mode)
        self.query_timeout = kwargs.get('query_timeout', config.query_timeout)
        self.max_iterations = kwargs.get('max_iterations', config.max_iterations)
        self.enable_sql_validation = kwargs.get('enable_sql_validation', config.enable_sql_validation)
        self.log_sql_queries = kwargs.get('log_sql_queries', config.log_sql_queries)

        # List 狀態記憶（用於回溯）
        self.current_list = None          # 當前正在處理的 List
        self.current_list_index = 0       # 當前 List 的處理位置
        self.list_stack = []              # List 堆疊，支援嵌套 List

        mode_name = "即時查詢" if self.real_time_mode else "預存結果"
        self.logger.info(f"🔧 TaskConditionChecker 初始化完成，模式: {mode_name}")
        self.logger.debug(f"   查詢超時: {self.query_timeout}s, 最大迭代: {self.max_iterations}, SQL驗證: {self.enable_sql_validation}")

        # 簡化的 List 處理狀態
        self.processed_lists = set()  # 記錄已處理過的 List（避免重複處理）
    
    def check_conditions_from_id(self, start_id: int = 1) -> Tuple[bool, Dict[str, Any]]:
        """
        從指定 ID 開始進行條件檢查
        
        Args:
            start_id: 起始條件 ID
            
        Returns:
            Tuple[bool, Dict[str, Any]]: (是否成功, 收集的資料)
        """
        try:
            collected_data = {}
            current_id = start_id
            iteration_count = 0

            # 重置已處理的 List 記錄（每次新的條件檢查會話）
            self.reset_processed_lists()

            self.logger.info(f"🔍 開始條件檢查，起始 ID: {current_id}")
            
            while iteration_count < self.max_iterations:
                iteration_count += 1
                
                # 取得當前 ID 的條件資料
                condition_result = self.get_task_condition_results(current_id)
                if not condition_result:
                    self.logger.warning(f"⚠️ 無法取得 ID {current_id} 的條件資料，回到起始點")
                    current_id = start_id
                    continue
                
                # 解析結果
                parse_result = self.parse_condition_results(current_id, condition_result)
                if not parse_result:
                    self.logger.warning(f"⚠️ ID {current_id} 解析失敗，回到起始點")
                    current_id = start_id
                    continue
                
                success, data_list = parse_result
                
                if not success:
                    self.logger.info(f"📋 ID {current_id} 查詢未成功，回到起始點")
                    current_id = start_id
                    continue
                
                # 處理資料列表中的每一筆資料
                next_id = None
                for data_item in data_list:
                    # 檢查是否為結束條件
                    end_value = data_item.get("end")
                    self.logger.debug(f"🔍 ID {current_id} end 欄位: {end_value} (類型: {type(end_value)})")

                    # 支援多種結束條件格式
                    if (end_value is True or
                        end_value == "True" or
                        end_value == "true" or
                        end_value == 1):
                        # 先收集資料，再返回
                        self.collect_data(current_id, data_item, collected_data)
                        self.logger.info(f"✅ 遇到結束條件 (end={end_value})，條件檢查完成")
                        return True, collected_data
                    
                    # 檢查 result 欄位
                    result_value = data_item.get("result")
                    if result_value == "True":
                        # 收集資料
                        self.collect_data(current_id, data_item, collected_data)

                        # 取得 next_id 並確保是字串格式
                        next_id_raw = data_item.get("next_id")
                        if next_id_raw:
                            next_id = str(next_id_raw)  # 確保轉換為字串
                            self.logger.info(f"📝 ID {current_id} 條件滿足，next_id: {next_id}")

                            # 檢查是否為結束標記
                            if next_id.lower() == "end":
                                self.logger.info(f"✅ 遇到結束標記 'end'，條件檢查完成")
                                return True, collected_data

                            break
                    elif result_value == "False":
                        # 檢查是否有 next_id，支援 OR 邏輯和多路徑探索
                        next_id_raw = data_item.get("next_id")
                        if next_id_raw:
                            next_id = str(next_id_raw)
                            self.logger.info(f"📋 ID {current_id} 條件不滿足，但有 next_id: {next_id}，繼續探索")

                            # 檢查是否為結束標記（雖然條件不滿足，但可能是容錯結束）
                            if next_id.lower() == "end":
                                self.logger.info(f"⚠️ 條件不滿足但遇到結束標記，條件檢查結束")
                                return False, collected_data

                            break
                        else:
                            # 沒有 next_id，繼續檢查下一筆資料
                            continue
                
                # 處理 next_id
                if next_id:
                    next_id_result = self.process_next_id(next_id)
                    if next_id_result == -1:
                        # List 已完成但無滿足條件，直接結束
                        self.logger.info(f"📋 List 遍歷完成，無滿足條件，結束檢查")
                        return False, collected_data
                    elif next_id_result is not None:
                        current_id = next_id_result
                    else:
                        # next_id 處理失敗，回到起始點
                        current_id = start_id
                else:
                    # 沒有 next_id，檢查是否應該結束
                    # 如果當前條件的所有資料都是 result=False 且沒有 next_id，應該結束
                    if data_list:
                        all_false_no_next = all(
                            item.get("result") == "False" and not item.get("next_id")
                            for item in data_list
                        )
                        if all_false_no_next:
                            # 檢查是否可以回溯到 List 繼續處理
                            backtrack_id = self._try_backtrack_to_list()
                            if backtrack_id is not None:
                                self.logger.info(f"🔄 ID {current_id} 失敗，回溯到 List 繼續檢查 ID {backtrack_id}")
                                current_id = backtrack_id
                                continue
                            else:
                                self.logger.info(f"📋 ID {current_id} 所有條件都不滿足且無 next_id，結束檢查")
                                return False, collected_data

                    # 其他情況回到起始點
                    current_id = start_id
            
            # 達到最大迭代次數
            self.logger.warning(f"⚠️ 達到最大迭代次數 {self.max_iterations}，停止條件檢查")
            return False, collected_data
            
        except Exception as e:
            self.logger.error(f"❌ 條件檢查失敗: {e}")
            return False, {}

    def get_task_condition_results(self, condition_id: int) -> Optional[Dict[str, Any]]:
        """
        取得指定 ID 的 task_condition results 資料
        根據模式選擇即時查詢或讀取預存結果

        Args:
            condition_id: 條件 ID

        Returns:
            Optional[Dict[str, Any]]: results 資料，如果不存在則返回 None
        """
        try:
            if self.real_time_mode:
                # 即時查詢模式：執行 SQL 並返回結果
                return self._execute_condition_query_real_time(condition_id)
            else:
                # 預存結果模式：讀取 results 欄位
                return self._get_stored_results(condition_id)

        except Exception as e:
            self.logger.error(f"❌ 取得 ID {condition_id} 條件資料失敗: {e}")
            return None

    def _get_stored_results(self, condition_id: int) -> Optional[Dict[str, Any]]:
        """
        從 results 欄位取得預存的查詢結果

        Args:
            condition_id: 條件 ID

        Returns:
            Optional[Dict[str, Any]]: 預存的 results 資料
        """
        try:
            with self.db_manager.get_session() as session:
                condition = task_condition_crud.get_by_id(session, condition_id)
                if condition and condition.results:
                    self.logger.debug(f"📋 讀取 ID {condition_id} 的預存結果")
                    return condition.results
                else:
                    self.logger.warning(f"⚠️ ID {condition_id} 的條件記錄不存在或無 results 資料")
                    return None
        except Exception as e:
            self.logger.error(f"❌ 讀取 ID {condition_id} 預存結果失敗: {e}")
            return None

    def _execute_condition_query_real_time(self, condition_id: int) -> Optional[Dict[str, Any]]:
        """
        即時執行條件查詢

        Args:
            condition_id: 條件 ID

        Returns:
            Optional[Dict[str, Any]]: 查詢結果
        """
        try:
            # 取得條件記錄
            with self.db_manager.get_session() as session:
                condition = task_condition_crud.get_by_id(session, condition_id)
                if not condition or not condition.conditions:
                    self.logger.warning(f"⚠️ ID {condition_id} 的條件記錄不存在或無 conditions 內容")
                    return None

                sql_query = condition.conditions.strip()
                self.logger.debug(f"🔍 即時執行 ID {condition_id} 的查詢: {sql_query}")

                # 執行 SQL 查詢
                return self._execute_sql_query(session, sql_query)

        except Exception as e:
            self.logger.error(f"❌ 即時執行 ID {condition_id} 查詢失敗: {e}")
            return None

    def _execute_sql_query(self, session, sql_query: str) -> Dict[str, Any]:
        """
        執行 SQL 查詢並返回結果

        Args:
            session: 資料庫會話
            sql_query: 要執行的 SQL 查詢

        Returns:
            Dict[str, Any]: 查詢結果或錯誤資訊
        """
        try:
            # 記錄 SQL 查詢（如果啟用）
            if self.log_sql_queries:
                self.logger.info(f"🔍 執行 SQL 查詢: {sql_query}")

            # 驗證 SQL 安全性（如果啟用）
            if self.enable_sql_validation:
                is_valid, error_msg = self._validate_sql_query(sql_query)
                if not is_valid:
                    return {
                        "success": False,
                        "error": f"SQL 驗證失敗: {error_msg}",
                        "timestamp": datetime.now(timezone.utc).isoformat()
                    }

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

    def parse_condition_results(self, condition_id: int, results: Dict[str, Any]) -> Optional[Tuple[bool, List[Dict[str, Any]]]]:
        """
        解析條件結果資料
        
        Args:
            condition_id: 條件 ID
            results: results 資料
            
        Returns:
            Optional[Tuple[bool, List[Dict[str, Any]]]]: (success, data_list) 或 None
        """
        try:
            success = results.get("success", False)
            data = results.get("data", [])
            
            if not isinstance(data, list):
                self.logger.warning(f"⚠️ ID {condition_id} 的 data 不是列表格式")
                return None
            
            return (success, data)
            
        except Exception as e:
            self.logger.error(f"❌ 解析 ID {condition_id} 結果失敗: {e}")
            return None

    def collect_data(self, condition_id: int, data_item: Dict[str, Any], collected_data: Dict[str, Any]):
        """
        收集條件檢查過程中的資料
        
        Args:
            condition_id: 條件 ID
            data_item: 資料項目
            collected_data: 收集資料的字典（會被修改）
        """
        try:
            # 記錄來源
            data_item_copy = data_item.copy()
            data_item_copy["_condition_id"] = condition_id
            
            # 將資料加入收集清單，相同名稱的欄位會被覆蓋
            for key, value in data_item_copy.items():
                if key not in ["result", "next_id", "end"]:  # 排除控制欄位
                    collected_data[key] = value
            
            self.logger.debug(f"📝 收集 ID {condition_id} 的資料: {data_item_copy}")
            
        except Exception as e:
            self.logger.error(f"❌ 收集 ID {condition_id} 資料失敗: {e}")

    def process_next_id(self, next_id: str) -> Optional[int]:
        """
        處理 next_id 邏輯

        Args:
            next_id: next_id 字串（可能是單一值或列表格式）

        Returns:
            Optional[int]: 下一個要檢查的 ID，如果處理失敗則返回 None
        """
        try:
            # 確保 next_id 是字串格式
            if isinstance(next_id, int):
                next_id = str(next_id)
            elif not isinstance(next_id, str):
                self.logger.error(f"❌ next_id 格式錯誤: {type(next_id)} - {next_id}")
                return None
            # 檢查是否包含 "|" 分隔符（表示有多個選項）
            if "|" in next_id:
                # 分割格式：如 "1|2,3,4,5"
                parts = next_id.split("|", 1)
                if len(parts) == 2:
                    single_id = parts[0].strip()
                    list_ids = parts[1].strip()
                    
                    # 先檢查單一 ID
                    if single_id.isdigit():
                        single_result = self.check_single_id_condition(int(single_id))
                        if single_result:
                            return int(single_id)
                    
                    # 如果單一 ID 失敗，處理列表 ID
                    return self.process_id_list(list_ids)
            else:
                # 檢查是否為列表格式（方括號包圍或逗號分隔）
                if next_id.startswith("[") and next_id.endswith("]"):
                    # 移除方括號並處理
                    clean_list = next_id[1:-1]
                    return self.process_id_list(clean_list)
                elif "," in next_id:
                    return self.process_id_list(next_id)
                else:
                    # 單一 ID
                    if next_id.strip().isdigit():
                        return int(next_id.strip())
            
            return None
            
        except Exception as e:
            self.logger.error(f"❌ 處理 next_id '{next_id}' 失敗: {e}")
            return None

    def process_id_list(self, id_list_str: str) -> Optional[int]:
        """
        處理 ID 列表，支援回溯功能

        Args:
            id_list_str: ID 列表字串（逗號分隔）

        Returns:
            Optional[int]: 找到的 ID 或 next_id，如果都失敗則返回 -1
        """
        try:
            id_list = [int(id_str.strip()) for id_str in id_list_str.split(",") if id_str.strip().isdigit()]

            # 記住當前 List 狀態
            self.current_list = id_list
            self.current_list_index = 0

            return self._process_list_from_index()

        except Exception as e:
            self.logger.error(f"❌ 處理 ID 列表 '{id_list_str}' 失敗: {e}")
            return None

    def _process_list_from_index(self) -> Optional[int]:
        """
        從當前索引開始處理 List

        Returns:
            Optional[int]: 找到的 ID 或 next_id，如果都失敗則返回 -1
        """
        try:
            if not self.current_list or self.current_list_index >= len(self.current_list):
                # List 已完成，清空狀態
                self.current_list = None
                self.current_list_index = 0
                return -1  # List 已完成

            for i in range(self.current_list_index, len(self.current_list)):
                check_id = self.current_list[i]
                self.current_list_index = i + 1  # 更新索引
                # 檢查條件是否存在
                if not self.check_single_id_condition(check_id):
                    continue  # 條件不存在，檢查下一個

                # 條件存在，取得詳細結果
                condition_results = self.get_task_condition_results(check_id)
                success, data_list = self.parse_condition_results(check_id, condition_results)

                if success and data_list:
                    for data_item in data_list:
                        result_value = data_item.get("result")
                        next_id_value = data_item.get("next_id")

                        if result_value == "True":
                            # 條件滿足，返回此 ID
                            self.logger.info(f"✅ ID {check_id} 條件滿足，選擇此 ID")
                            return check_id
                        elif result_value == "False" and next_id_value:
                            # 條件不滿足但有 next_id，立即跳轉
                            self.logger.info(f"🔄 ID {check_id} 條件不滿足但有 next_id: {next_id_value}，立即跳轉")
                            return int(str(next_id_value))  # 直接返回 next_id

            # 所有條件都不滿足且無跳轉
            self.logger.info(f"📋 List {self.current_list} 遍歷完成，所有條件都不滿足且無跳轉")
            # 清空 List 狀態
            self.current_list = None
            self.current_list_index = 0
            return -1  # 特殊值：表示 List 已完成但無滿足條件

        except Exception as e:
            self.logger.error(f"❌ 處理 List 失敗: {e}")
            return -1

    def _try_backtrack_to_list(self) -> Optional[int]:
        """
        嘗試回溯到 List 繼續處理

        Returns:
            Optional[int]: 下一個要檢查的 ID，如果無法回溯則返回 None
        """
        try:
            # 首先檢查 List 是否已經完全處理完畢
            if not self.current_list or self.current_list_index >= len(self.current_list):
                # List 已完全處理完畢，無法回溯
                self.logger.info(f"📋 List 已完全處理完畢，無法回溯")
                self.current_list = None
                self.current_list_index = 0
                return None

            # List 還有未處理的項目，繼續處理
            self.logger.info(f"🔄 回溯到 List，從索引 {self.current_list_index} 繼續")
            result = self._process_list_from_index()

            # 如果 _process_list_from_index 返回 -1，表示 List 已完成
            if result == -1:
                self.logger.info(f"📋 List 回溯處理完成，無更多選項")
                self.current_list = None
                self.current_list_index = 0
                return None

            return result

        except Exception as e:
            self.logger.error(f"❌ 回溯到 List 失敗: {e}")
            return None

    def check_single_id_condition(self, check_id: int) -> bool:
        """
        檢查單一 ID 的條件是否滿足
        
        Args:
            check_id: 要檢查的 ID
            
        Returns:
            bool: 條件是否滿足
        """
        try:
            condition_result = self.get_task_condition_results(check_id)
            if not condition_result:
                return False
            
            parse_result = self.parse_condition_results(check_id, condition_result)
            if not parse_result:
                return False
            
            success, data_list = parse_result
            if not success:
                return False
            
            # 檢查是否有任何 data 項目的 result 為 "True" 或有 next_id（支援 OR 邏輯）
            for data_item in data_list:
                result_value = data_item.get("result")
                next_id_value = data_item.get("next_id")

                if result_value == "True":
                    return True
                elif result_value == "False" and next_id_value:
                    # 支援 OR 邏輯：即使 result 為 False，但有 next_id 也算成功
                    self.logger.info(f"📋 ID {check_id} 條件不滿足但有 next_id: {next_id_value}，視為可繼續")
                    return True

            return False
            
        except Exception as e:
            self.logger.error(f"❌ 檢查 ID {check_id} 條件失敗: {e}")
            return False

    def reset_processed_lists(self):
        """重置已處理的 List 記錄（用於新的條件檢查會話）"""
        self.processed_lists.clear()
        self.logger.debug("🔄 重置已處理的 List 記錄")

    def _validate_sql_query(self, sql_query: str) -> Tuple[bool, str]:
        """
        驗證 SQL 查詢的安全性

        Args:
            sql_query: 要驗證的 SQL 查詢

        Returns:
            Tuple[bool, str]: (是否有效, 錯誤訊息)
        """
        try:
            sql_upper = sql_query.upper().strip()

            # 檢查是否為 SELECT 語句
            if not sql_upper.startswith('SELECT'):
                return False, "只允許 SELECT 查詢"

            # 檢查危險關鍵字
            dangerous_keywords = [
                'DROP', 'DELETE', 'INSERT', 'UPDATE', 'ALTER', 'CREATE',
                'TRUNCATE', 'EXEC', 'EXECUTE', 'DECLARE', 'CURSOR',
                'GRANT', 'REVOKE', 'COMMIT', 'ROLLBACK'
            ]

            for keyword in dangerous_keywords:
                if keyword in sql_upper:
                    return False, f"包含危險關鍵字: {keyword}"

            # 檢查是否包含分號（防止 SQL 注入）
            if ';' in sql_query.rstrip(';'):
                return False, "不允許多重 SQL 語句"

            return True, ""

        except Exception as e:
            return False, f"驗證過程發生錯誤: {str(e)}"

    def set_max_iterations(self, max_iterations: int):
        """
        設定最大迭代次數

        Args:
            max_iterations: 最大迭代次數
        """
        self.max_iterations = max_iterations
        self.logger.info(f"🔧 設定最大迭代次數: {max_iterations}")

    def set_query_timeout(self, timeout: int):
        """
        設定查詢超時時間

        Args:
            timeout: 超時時間（秒）
        """
        self.query_timeout = timeout
        self.logger.info(f"🔧 設定查詢超時時間: {timeout} 秒")

    def set_mode(self, real_time_mode: bool):
        """
        設定檢查模式

        Args:
            real_time_mode: True=即時查詢模式，False=預存結果模式
        """
        self.real_time_mode = real_time_mode
        mode_name = "即時查詢" if real_time_mode else "預存結果"
        self.logger.info(f"🔧 切換到 {mode_name} 模式")

    def get_mode_info(self) -> Dict[str, Any]:
        """
        取得當前模式資訊

        Returns:
            Dict[str, Any]: 模式資訊
        """
        return {
            "real_time_mode": self.real_time_mode,
            "mode_name": "即時查詢" if self.real_time_mode else "預存結果",
            "query_timeout": self.query_timeout,
            "max_iterations": self.max_iterations
        }
