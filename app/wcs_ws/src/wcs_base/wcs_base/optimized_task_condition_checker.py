"""
優化的任務條件檢查器
增強了快取機制，避免重複查詢已獲取的變數

主要優化：
1. 變數快取機制 - 保存已查詢的變數避免重複查詢
2. 條件結果繼承 - 後續條件可以使用前面條件的查詢結果
3. 智能 SQL 優化 - 檢測並跳過重複的資料庫查詢
4. 位置計算快取 - 快取房間號、目標位置等計算結果
"""

from db_proxy.crud.task_condition_crud import task_condition_crud
from sqlalchemy import text
from sqlalchemy.exc import SQLAlchemyError
import json
from datetime import datetime, timezone
from typing import Dict, Any, List, Optional, Tuple
from rclpy.node import Node
from .task_condition_config import get_current_config, TaskConditionConfig
import re


class OptimizedTaskConditionChecker:
    """
    優化的任務條件檢查器
    
    提供基於 task_condition 表格的通用條件檢查功能，
    支援變數快取和條件結果繼承以避免重複查詢。
    """

    def __init__(self, db_manager, logger, config: Optional[TaskConditionConfig] = None, **kwargs):
        """
        初始化優化條件檢查器

        Args:
            db_manager: 資料庫管理器
            logger: 日誌記錄器
            config: 任務條件配置物件
            **kwargs: 額外的配置參數
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

        # === 快取機制 ===
        self.variable_cache = {}          # 變數快取：保存已查詢的變數
        self.query_result_cache = {}      # 查詢結果快取：保存 SQL 查詢結果
        self.calculation_cache = {}       # 計算結果快取：保存位置、房間號等计算
        self.collected_data = {}          # 收集的條件資料
        
        # === List 狀態記憶 ===
        self.current_list = None
        self.current_list_index = 0
        self.list_stack = []
        self.processed_lists = set()

        mode_name = "即時查詢(快取優化)" if self.real_time_mode else "預存結果(快取優化)"
        self.logger.info(f"🚀 OptimizedTaskConditionChecker 初始化完成，模式: {mode_name}")
        self.logger.debug(f"   快取功能已啟用：變數快取、查詢快取、計算快取")

    def check_conditions_from_id(self, start_id: int = 1) -> Tuple[bool, Dict[str, Any]]:
        """
        從指定 ID 開始進行條件檢查（優化版本）
        
        Args:
            start_id: 起始條件 ID
            
        Returns:
            Tuple[bool, Dict[str, Any]]: (是否成功, 收集的資料)
        """
        try:
            # 重置快取（新的檢查會話）
            self.reset_caches()
            current_id = start_id
            iteration_count = 0

            self.logger.info(f"🔍 開始優化條件檢查，起始 ID: {current_id}")
            
            while iteration_count < self.max_iterations:
                iteration_count += 1
                
                # 取得當前 ID 的條件資料
                condition_result = self.get_task_condition_results(current_id)
                if not condition_result:
                    self.logger.warning(f"⚠️ 無法取得 ID {current_id} 的條件資料")
                    current_id = start_id
                    continue
                
                # 優化解析：檢查快取並避免重複查詢
                parse_result = self.parse_condition_results_optimized(current_id, condition_result)
                if not parse_result:
                    self.logger.warning(f"⚠️ ID {current_id} 優化解析失敗")
                    current_id = start_id
                    continue
                
                success, data_list = parse_result
                
                if not success:
                    self.logger.info(f"📋 ID {current_id} 查詢未成功，回到起始點")
                    current_id = start_id
                    continue
                
                # 處理資料列表
                next_id = None
                for data_item in data_list:
                    # 檢查結束條件
                    end_value = data_item.get("end")
                    if (end_value is True or end_value == "True" or 
                        end_value == "true" or end_value == 1):
                        # 收集資料並返回
                        self.collect_data_optimized(current_id, data_item)
                        self.logger.info(f"✅ 遇到結束條件，優化條件檢查完成")
                        self.logger.info(f"📊 快取統計: 變數 {len(self.variable_cache)}, 查詢 {len(self.query_result_cache)}, 計算 {len(self.calculation_cache)}")
                        return True, self.collected_data
                    
                    # 檢查 result 欄位
                    result_value = data_item.get("result")
                    if result_value == "True":
                        # 收集資料
                        self.collect_data_optimized(current_id, data_item)

                        # 取得 next_id
                        next_id_raw = data_item.get("next_id")
                        if next_id_raw:
                            next_id = str(next_id_raw)
                            self.logger.info(f"📝 ID {current_id} 條件滿足，next_id: {next_id}")

                            if next_id.lower() == "end":
                                self.logger.info(f"✅ 遇到結束標記，優化條件檢查完成")
                                return True, self.collected_data
                            break
                    elif result_value == "False":
                        # 支援 OR 邏輯
                        next_id_raw = data_item.get("next_id")
                        if next_id_raw:
                            next_id = str(next_id_raw)
                            self.logger.info(f"📋 ID {current_id} 條件不滿足，探索 next_id: {next_id}")
                            if next_id.lower() == "end":
                                return False, self.collected_data
                            break
                        else:
                            continue
                
                # 處理 next_id
                if next_id:
                    next_id_result = self.process_next_id(next_id)
                    if next_id_result == -1:
                        self.logger.info(f"📋 List 遍歷完成，無滿足條件，結束檢查")
                        return False, self.collected_data
                    elif next_id_result is not None:
                        current_id = next_id_result
                    else:
                        current_id = start_id
                else:
                    # 沒有 next_id 的處理邏輯
                    if data_list:
                        all_false_no_next = all(
                            item.get("result") == "False" and not item.get("next_id")
                            for item in data_list
                        )
                        if all_false_no_next:
                            backtrack_id = self._try_backtrack_to_list()
                            if backtrack_id is not None:
                                current_id = backtrack_id
                                continue
                            else:
                                return False, self.collected_data
                    current_id = start_id
            
            # 達到最大迭代次數
            self.logger.warning(f"⚠️ 達到最大迭代次數 {self.max_iterations}，停止優化條件檢查")
            return False, self.collected_data
            
        except Exception as e:
            self.logger.error(f"❌ 優化條件檢查失敗: {e}")
            return False, {}

    def parse_condition_results_optimized(self, condition_id: int, condition_data: Dict[str, Any]) -> Optional[Tuple[bool, List[Dict[str, Any]]]]:
        """
        優化的條件結果解析
        
        檢查快取，避免重複查詢，並利用已有變數進行計算
        """
        try:
            conditions_sql = condition_data.get("conditions", "")
            
            # 檢查是否可以從快取中獲取結果
            cache_key = f"condition_{condition_id}_{hash(conditions_sql)}"
            if cache_key in self.query_result_cache:
                self.logger.debug(f"🎯 條件 {condition_id} 命中查詢快取")
                return self.query_result_cache[cache_key]
            
            # 檢查是否可以使用變數快取優化 SQL
            optimized_sql = self.optimize_sql_with_cache(conditions_sql, condition_id)
            
            if not self.real_time_mode:
                # 預存結果模式
                results_data = condition_data.get("results", {})
                if results_data:
                    result = (True, [results_data])
                    self.query_result_cache[cache_key] = result
                    return result
            
            # 即時查詢模式（使用優化的 SQL）
            if optimized_sql:
                with self.db_manager.get_session() as session:
                    try:
                        if self.log_sql_queries:
                            self.logger.debug(f"🔍 執行優化 SQL (條件 {condition_id}): {optimized_sql[:200]}...")
                        
                        query_result = session.execute(text(optimized_sql))
                        rows = query_result.fetchall()
                        
                        if rows:
                            data_list = []
                            for row in rows:
                                row_dict = dict(row._mapping)
                                # 將結果存入變數快取
                                self.update_variable_cache(row_dict)
                                data_list.append(row_dict)
                            
                            result = (True, data_list)
                            self.query_result_cache[cache_key] = result
                            self.logger.debug(f"✅ 條件 {condition_id} 優化查詢成功，獲得 {len(data_list)} 筆結果")
                            return result
                        else:
                            result = (False, [])
                            self.query_result_cache[cache_key] = result
                            return result
                            
                    except SQLAlchemyError as e:
                        self.logger.error(f"❌ 條件 {condition_id} 優化 SQL 執行失敗: {e}")
                        return None
            
            return None
            
        except Exception as e:
            self.logger.error(f"❌ 條件 {condition_id} 優化解析失敗: {e}")
            return None

    def optimize_sql_with_cache(self, sql: str, condition_id: int) -> str:
        """
        使用快取優化 SQL 查詢
        
        檢查 SQL 中需要的變數是否已在快取中，如果是則直接替換為快取值
        """
        try:
            optimized_sql = sql
            
            # 檢查常見的重複計算模式
            patterns = {
                # 房間號計算: location_id / 10000
                r'(\w+\.)?location_id\s*/\s*10000': lambda m: self._get_cached_room_id(),
                
                # 目標位置計算: room_id * 10000 + 1/2
                r'\(\s*(\w+\.)?location_id\s*/\s*10000\s*\)\s*\*\s*10000\s*\+\s*(\d+)': 
                    lambda m: self._get_cached_target_location(int(m.group(2))),
                    
                # 來源位置: MIN(location_id) 如果已快取
                r'MIN\((\w+\.)?location_id\)': lambda m: self._get_cached_source_location(),
                
                # 料架ID: MIN(id) 如果已快取  
                r'MIN\((\w+\.)?id\)': lambda m: self._get_cached_rack_id(),
            }
            
            # 應用優化模式
            for pattern, replacement_func in patterns.items():
                if re.search(pattern, optimized_sql):
                    try:
                        cached_value = replacement_func(re.search(pattern, optimized_sql))
                        if cached_value is not None:
                            optimized_sql = re.sub(pattern, str(cached_value), optimized_sql)
                            self.logger.debug(f"🎯 條件 {condition_id} 使用快取值替換: {pattern[:30]}... → {cached_value}")
                    except Exception as e:
                        self.logger.debug(f"⚠️ 快取替換失敗: {e}")
                        continue
            
            return optimized_sql
            
        except Exception as e:
            self.logger.error(f"❌ SQL 快取優化失敗: {e}")
            return sql

    def _get_cached_room_id(self) -> Optional[int]:
        """從快取中獲取房間號"""
        return self.variable_cache.get('room_id') or self.calculation_cache.get('room_id')
    
    def _get_cached_target_location(self, suffix: int) -> Optional[int]:
        """從快取中獲取目標位置"""
        cache_key = f'target_location_{suffix}'
        cached = self.calculation_cache.get(cache_key)
        if cached:
            return cached
            
        # 嘗試計算
        room_id = self._get_cached_room_id()
        if room_id:
            target_location = room_id * 10000 + suffix
            self.calculation_cache[cache_key] = target_location
            return target_location
        return None
    
    def _get_cached_source_location(self) -> Optional[int]:
        """從快取中獲取來源位置"""
        return self.variable_cache.get('source_location')
    
    def _get_cached_rack_id(self) -> Optional[int]:
        """從快取中獲取料架ID"""
        return self.variable_cache.get('rack_id')

    def collect_data_optimized(self, condition_id: int, data_item: Dict[str, Any]):
        """
        優化的資料收集
        
        收集資料並更新變數快取
        """
        # 收集到 collected_data
        for key, value in data_item.items():
            if key not in ["result", "next_id", "end"] and value is not None:
                self.collected_data[key] = value
        
        # 標記條件 ID
        self.collected_data["_condition_id"] = condition_id
        
        # 更新變數快取
        self.update_variable_cache(data_item)
        
        # 自動計算常見的衍生變數
        self.calculate_derived_variables(data_item)
        
        self.logger.debug(f"📊 條件 {condition_id} 收集資料: {len(data_item)} 項")

    def update_variable_cache(self, data_item: Dict[str, Any]):
        """
        更新變數快取
        """
        for key, value in data_item.items():
            if key not in ["result", "next_id", "end"] and value is not None:
                self.variable_cache[key] = value

    def calculate_derived_variables(self, data_item: Dict[str, Any]):
        """
        計算衍生變數並快取
        """
        # 從 source_location 計算 room_id
        if 'source_location' in data_item and 'room_id' not in self.variable_cache:
            source_location = data_item['source_location']
            if isinstance(source_location, int) and source_location > 10000:
                room_id = source_location // 10000
                self.variable_cache['room_id'] = room_id
                self.calculation_cache['room_id'] = room_id
                self.logger.debug(f"🧮 自動計算 room_id: {source_location} // 10000 = {room_id}")
        
        # 從 room_id 計算常見的目標位置
        room_id = self.variable_cache.get('room_id')
        if room_id:
            for suffix in [1, 2]:  # 入口和出口
                cache_key = f'target_location_{suffix}'
                if cache_key not in self.calculation_cache:
                    target_location = room_id * 10000 + suffix
                    self.calculation_cache[cache_key] = target_location
                    self.logger.debug(f"🧮 自動計算 {cache_key}: {room_id} * 10000 + {suffix} = {target_location}")

    def reset_caches(self):
        """重置所有快取"""
        self.variable_cache.clear()
        self.query_result_cache.clear()
        self.calculation_cache.clear()
        self.collected_data.clear()
        self.processed_lists.clear()
        self.logger.debug("🔄 快取已重置")

    def get_task_condition_results(self, condition_id: int) -> Optional[Dict[str, Any]]:
        """
        取得任務條件結果（继承原始實作）
        """
        try:
            with self.db_manager.get_session() as session:
                condition = task_condition_crud.get_by_id(session, condition_id)
                if condition:
                    return {
                        "id": condition.id,
                        "conditions": condition.conditions,
                        "results": condition.results or {},
                        "description": condition.description
                    }
                else:
                    self.logger.warning(f"⚠️ 找不到條件 ID: {condition_id}")
                    return None
        except Exception as e:
            self.logger.error(f"❌ 取得條件 {condition_id} 失敗: {e}")
            return None

    def process_next_id(self, next_id: str) -> Optional[int]:
        """
        處理 next_id（继承原始實作）
        """
        try:
            # 檢查是否為 List 格式
            if next_id.startswith('[') and next_id.endswith(']'):
                # 解析 List
                try:
                    id_list = json.loads(next_id)
                    if isinstance(id_list, list) and id_list:
                        # 設定 List 狀態
                        self.current_list = id_list
                        self.current_list_index = 0
                        # 返回第一個 ID
                        return id_list[0]
                except json.JSONDecodeError:
                    self.logger.error(f"❌ 無法解析 next_id List: {next_id}")
                    return None
            else:
                # 單一 ID
                try:
                    return int(next_id)
                except ValueError:
                    self.logger.error(f"❌ 無法轉換 next_id 為整數: {next_id}")
                    return None
        except Exception as e:
            self.logger.error(f"❌ 處理 next_id 失敗: {e}")
            return None

    def _try_backtrack_to_list(self) -> Optional[int]:
        """
        嘗試回溯到 List 繼續處理（继承原始實作）
        """
        if self.current_list and self.current_list_index < len(self.current_list) - 1:
            self.current_list_index += 1
            return self.current_list[self.current_list_index]
        return None

    def get_cache_statistics(self) -> Dict[str, Any]:
        """
        取得快取統計資訊
        """
        return {
            'variable_cache_size': len(self.variable_cache),
            'query_result_cache_size': len(self.query_result_cache),
            'calculation_cache_size': len(self.calculation_cache),
            'collected_data_size': len(self.collected_data),
            'variable_cache_keys': list(self.variable_cache.keys()),
            'calculation_cache_keys': list(self.calculation_cache.keys())
        }