"""
DSL 函數註冊器 - WCS 函數與 DSL 系統整合
將現有的 WCS 函數註冊到 DSL 執行環境中，提供統一的函數調用介面
"""

import logging
import inspect
from typing import Dict, Any, Callable, List, Optional
from dataclasses import dataclass


@dataclass
class DSLFunction:
    """DSL 函數定義"""
    name: str
    source: str
    function: Callable
    description: str
    parameters: Dict[str, Any]
    return_type: str
    category: str  # condition, logic, action
    
    def __post_init__(self):
        """自動獲取函數簽名資訊"""
        if self.function and not self.parameters:
            sig = inspect.signature(self.function)
            self.parameters = {
                param_name: {
                    'type': param.annotation.__name__ if param.annotation != inspect.Parameter.empty else 'any',
                    'default': param.default if param.default != inspect.Parameter.empty else None,
                    'required': param.default == inspect.Parameter.empty
                }
                for param_name, param in sig.parameters.items()
                if param_name != 'self'
            }


class DSLFunctionRegistry:
    """DSL 函數註冊器 - 管理所有可用的 WCS 函數"""
    
    def __init__(self):
        self.functions: Dict[str, DSLFunction] = {}
        self.sources: Dict[str, Any] = {}
        self.logger = logging.getLogger('yaml_dsl.function_registry')
    
    def register_source(self, source_name: str, source_object: Any):
        """註冊函數來源物件"""
        self.sources[source_name] = source_object
        self.logger.info(f"註冊函數來源: {source_name}")
    
    def register_function(self, 
                         name: str, 
                         source: str, 
                         function: Callable,
                         description: str = "",
                         category: str = "action",
                         parameters: Dict[str, Any] = None,
                         return_type: str = "any"):
        """註冊單個函數"""
        dsl_function = DSLFunction(
            name=name,
            source=source,
            function=function,
            description=description,
            parameters=parameters or {},
            return_type=return_type,
            category=category
        )
        
        self.functions[name] = dsl_function
        self.logger.debug(f"註冊 DSL 函數: {name} (來源: {source})")
    
    def register_source_functions(self, source_name: str, source_object: Any, function_mappings: Dict[str, Dict[str, Any]]):
        """批量註冊來源物件的函數"""
        self.register_source(source_name, source_object)
        
        for func_name, func_config in function_mappings.items():
            try:
                # 獲取函數物件
                function = getattr(source_object, func_name)
                
                self.register_function(
                    name=func_config.get('name', func_name),
                    source=source_name,
                    function=function,
                    description=func_config.get('description', ''),
                    category=func_config.get('category', 'action'),
                    parameters=func_config.get('parameters'),
                    return_type=func_config.get('return_type', 'any')
                )
                
            except AttributeError:
                self.logger.warning(f"函數 {func_name} 在 {source_name} 中不存在")
    
    def get_function(self, name: str, source: str = None) -> Optional[DSLFunction]:
        """取得註冊的函數"""
        if source:
            # 尋找特定來源的函數
            for func_name, dsl_func in self.functions.items():
                if func_name == name and dsl_func.source == source:
                    return dsl_func
        else:
            # 直接尋找函數名稱
            return self.functions.get(name)
        
        return None
    
    def call_function(self, name: str, source: str, **kwargs) -> Any:
        """調用註冊的函數"""
        dsl_function = self.get_function(name, source)
        if not dsl_function:
            raise ValueError(f"函數未註冊: {name} (來源: {source})")
        
        try:
            # 調用實際函數
            result = dsl_function.function(**kwargs)
            self.logger.debug(f"調用函數成功: {name} -> {type(result)}")
            return result
            
        except Exception as e:
            self.logger.error(f"調用函數失敗 {name}: {e}")
            raise
    
    def list_functions(self, category: str = None, source: str = None) -> List[DSLFunction]:
        """列出註冊的函數"""
        functions = list(self.functions.values())
        
        if category:
            functions = [f for f in functions if f.category == category]
        
        if source:
            functions = [f for f in functions if f.source == source]
        
        return functions
    
    def get_function_signature(self, name: str, source: str = None) -> Dict[str, Any]:
        """取得函數簽名資訊"""
        dsl_function = self.get_function(name, source)
        if not dsl_function:
            return {}
        
        return {
            'name': dsl_function.name,
            'source': dsl_function.source,
            'description': dsl_function.description,
            'parameters': dsl_function.parameters,
            'return_type': dsl_function.return_type,
            'category': dsl_function.category
        }


# 預定義的 WCS 函數註冊配置
WCS_FUNCTION_MAPPINGS = {
    # === 統一決策引擎函數 ===
    'unified_decision_engine': {
        'check_agv_rotation_flow': {
            'name': 'check_agv_rotation_flow',
            'description': 'AGV旋轉流程檢查',
            'category': 'condition',
            'return_type': 'List[TaskDecision]'
        },
        'check_ng_rack_recycling_flow': {
            'name': 'check_ng_rack_recycling_flow', 
            'description': 'NG料架回收流程檢查',
            'category': 'condition',
            'return_type': 'List[TaskDecision]'
        },
        'check_full_rack_to_manual_flow': {
            'name': 'check_full_rack_to_manual_flow',
            'description': '滿料架到人工區流程檢查', 
            'category': 'condition',
            'return_type': 'List[TaskDecision]'
        },
        'check_manual_area_transport_flow': {
            'name': 'check_manual_area_transport_flow',
            'description': '人工收料區搬運流程檢查',
            'category': 'condition', 
            'return_type': 'List[TaskDecision]'
        },
        'check_system_to_room_flow': {
            'name': 'check_system_to_room_flow',
            'description': '系統到房間流程檢查',
            'category': 'condition',
            'return_type': 'List[TaskDecision]'
        },
        'check_empty_rack_transfer_flow': {
            'name': 'check_empty_rack_transfer_flow',
            'description': '空料架轉移流程檢查',
            'category': 'condition',
            'return_type': 'List[TaskDecision]'
        },
        'check_manual_empty_recycling_flow': {
            'name': 'check_manual_empty_recycling_flow',
            'description': '人工回收空料架流程檢查',
            'category': 'condition',
            'return_type': 'List[TaskDecision]'
        },
        'check_opui_requests_flow': {
            'name': 'check_opui_requests_flow',
            'description': 'OPUI請求流程檢查',
            'category': 'condition',
            'return_type': 'List[TaskDecision]'
        },
        'prioritize_and_schedule': {
            'name': 'prioritize_and_schedule',
            'description': '優先級排序和調度',
            'category': 'logic',
            'return_type': 'List[TaskDecision]'
        },
        'get_room_location_info': {
            'name': 'get_room_location_info',
            'description': '房間位置資訊處理',
            'category': 'logic',
            'return_type': 'Dict[str, int]'
        },
        'generate_rotation_nodes': {
            'name': 'generate_rotation_nodes',
            'description': 'AGV旋轉路徑生成',
            'category': 'logic',
            'return_type': 'List[int]'
        },
        'run_unified_decision_cycle': {
            'name': 'run_unified_decision_cycle',
            'description': '執行統一決策週期',
            'category': 'action',
            'return_type': 'List[TaskDecision]'
        },
        # === 新增 Simple WCS Engine 整合函數 ===
        'check_rack_rotation_flow': {
            'name': 'check_rack_rotation_flow',
            'description': '料架旋轉流程檢查',
            'category': 'condition',
            'return_type': 'List[TaskDecision]'
        },
        'check_rack_rotation_exit_flow': {
            'name': 'check_rack_rotation_exit_flow', 
            'description': '料架旋轉出口流程檢查',
            'category': 'condition',
            'return_type': 'List[TaskDecision]'
        },
        'check_transport_to_manual_flow': {
            'name': 'check_transport_to_manual_flow',
            'description': '運送到人工區流程檢查',
            'category': 'condition',
            'return_type': 'List[TaskDecision]'
        },
        'check_single_room_rotation': {
            'name': 'check_single_room_rotation',
            'description': '檢查單個房間旋轉',
            'category': 'condition',
            'return_type': 'Optional[TaskDecision]'
        },
        'check_single_room_exit_rotation': {
            'name': 'check_single_room_exit_rotation',
            'description': '檢查單個房間出口旋轉',
            'category': 'condition',
            'return_type': 'Optional[TaskDecision]'
        },
        'check_single_location_transport': {
            'name': 'check_single_location_transport',
            'description': '檢查單個位置運輸',
            'category': 'condition',
            'return_type': 'Optional[TaskDecision]'
        },
        'evaluate_trigger_condition': {
            'name': 'evaluate_trigger_condition',
            'description': '評估觸發條件',
            'category': 'condition',
            'return_type': 'bool'
        },
        'execute_task_decision': {
            'name': 'execute_task_decision',
            'description': '執行任務決策',
            'category': 'action',
            'return_type': 'None'
        },
        'publish_task_decision': {
            'name': 'publish_task_decision',
            'description': '發布任務決策',
            'category': 'action',
            'return_type': 'None'
        },
        'publish_system_status': {
            'name': 'publish_system_status',
            'description': '發布系統狀態',
            'category': 'action',
            'return_type': 'None'
        },
        'decision_cycle_callback': {
            'name': 'decision_cycle_callback',
            'description': '決策週期回調',
            'category': 'logic',
            'return_type': 'None'
        }
    },
    
    # === 增強資料庫客戶端函數 ===
    'enhanced_database_client': {
        'check_locations_available': {
            'name': 'check_locations_available',
            'description': '檢查位置可用性',
            'category': 'condition',
            'return_type': 'List[Dict[str, Any]]'
        },
        'check_ng_rack_at_location': {
            'name': 'check_ng_rack_at_location',
            'description': '檢查位置NG料架',
            'category': 'condition',
            'return_type': 'bool'
        },
        'check_carriers_in_room': {
            'name': 'check_carriers_in_room',
            'description': '檢查房間Carrier',
            'category': 'condition',
            'return_type': 'bool'
        },
        'check_racks_at_location': {
            'name': 'check_racks_at_location',
            'description': '檢查位置料架狀態',
            'category': 'condition',
            'return_type': 'List[Dict[str, Any]]'
        },
        'create_task_from_decision_dict': {
            'name': 'create_task_from_decision_dict',
            'description': '從決策字典創建任務',
            'category': 'action',
            'return_type': 'Optional[int]'
        },
        'update_existing_opui_task': {
            'name': 'update_existing_opui_task',
            'description': '更新現有OPUI任務',
            'category': 'action',
            'return_type': 'bool'
        },
        'update_machine_parking_status': {
            'name': 'update_machine_parking_status',
            'description': '更新機台停車格狀態',
            'category': 'action',
            'return_type': 'bool'
        },
        'batch_update_parking_status': {
            'name': 'batch_update_parking_status',
            'description': '批次更新停車格狀態',
            'category': 'action',
            'return_type': 'Dict[str, Any]'
        }
    },
    
    # === 統一任務管理器函數 ===
    'unified_task_manager': {
        'create_tasks_from_decisions': {
            'name': 'create_tasks_from_decisions',
            'description': '從決策創建任務',
            'category': 'action',
            'return_type': 'List[TaskCreationResult]'
        },
        'create_task_from_decision': {
            'name': 'create_task_from_decision',
            'description': '從單一決策創建任務',
            'category': 'action',
            'return_type': 'TaskCreationResult'
        }
    },
    
    # === 位置管理器函數 (基於 LocationManager) ===
    'location_manager': {
        'get_room_inlet_point': {
            'name': 'get_room_inlet_point',
            'description': '取得房間入口點',
            'category': 'logic',
            'return_type': 'int'
        },
        'get_inlet_rotation_point': {
            'name': 'get_inlet_rotation_point',
            'description': '取得入口旋轉點',
            'category': 'logic',
            'return_type': 'int'
        },
        'get_room_exit_point': {
            'name': 'get_room_exit_point',
            'description': '取得房間出口點',
            'category': 'logic',
            'return_type': 'int'
        },
        'get_exit_rotation_point': {
            'name': 'get_exit_rotation_point',
            'description': '取得出口旋轉點',
            'category': 'logic',
            'return_type': 'int'
        },
        'load_config': {
            'name': 'load_config',
            'description': '載入位置配置',
            'category': 'logic',
            'return_type': 'Dict[str, Any]'
        }
    }
}


def create_default_registry() -> DSLFunctionRegistry:
    """創建預設的 DSL 函數註冊器"""
    registry = DSLFunctionRegistry()
    
    # 注意：實際函數物件需要在 Simple WCS 啟動時註冊
    # 這裡只是定義了函數映射配置
    
    registry.logger.info("DSL 函數註冊器已創建，等待實際函數物件註冊")
    return registry


def register_wcs_functions(registry: DSLFunctionRegistry, 
                          unified_decision_engine=None,
                          enhanced_database_client=None, 
                          unified_task_manager=None,
                          location_manager=None):
    """註冊 WCS 系統的實際函數物件到 DSL 註冊器"""
    
    if unified_decision_engine:
        registry.register_source_functions(
            'unified_decision_engine',
            unified_decision_engine,
            WCS_FUNCTION_MAPPINGS['unified_decision_engine']
        )
    
    if enhanced_database_client:
        registry.register_source_functions(
            'enhanced_database_client',
            enhanced_database_client,
            WCS_FUNCTION_MAPPINGS['enhanced_database_client']
        )
    
    if unified_task_manager:
        registry.register_source_functions(
            'unified_task_manager',
            unified_task_manager,
            WCS_FUNCTION_MAPPINGS['unified_task_manager']
        )
    
    if location_manager:
        registry.register_source_functions(
            'location_manager',
            location_manager,
            WCS_FUNCTION_MAPPINGS['location_manager']
        )
    
    registry.logger.info("WCS 函數註冊完成")


# DSL 內建函數
class DSLBuiltinFunctions:
    """DSL 內建函數集合"""
    
    @staticmethod
    def len(obj) -> int:
        """取得物件長度"""
        return len(obj) if obj else 0
    
    @staticmethod
    def append_to_list(target_list: List, new_item: Any) -> List:
        """添加項目到列表"""
        if target_list is None:
            target_list = []
        target_list.append(new_item)
        return target_list
    
    @staticmethod
    def log_error(message: str, context: Dict[str, Any] = None):
        """記錄錯誤"""
        logger = logging.getLogger('yaml_dsl.runtime')
        logger.error(f"{message} - Context: {context}")
    
    @staticmethod
    def log_info(message: str, context: Dict[str, Any] = None):
        """記錄資訊"""
        logger = logging.getLogger('yaml_dsl.runtime')
        logger.info(f"{message} - Context: {context}")


def register_builtin_functions(registry: DSLFunctionRegistry):
    """註冊 DSL 內建函數"""
    builtin = DSLBuiltinFunctions()
    
    builtin_mappings = {
        'len': {
            'name': 'len',
            'description': '取得物件長度',
            'category': 'logic',
            'return_type': 'int'
        },
        'append_to_list': {
            'name': 'append_to_list',
            'description': '添加項目到列表',
            'category': 'script',
            'return_type': 'list'
        },
        'log_error': {
            'name': 'log_error',
            'description': '記錄錯誤',
            'category': 'script',
            'return_type': 'None'
        },
        'log_info': {
            'name': 'log_info',
            'description': '記錄資訊',
            'category': 'script', 
            'return_type': 'None'
        }
    }
    
    registry.register_source_functions('dsl_runtime', builtin, builtin_mappings)


# 測試範例
if __name__ == "__main__":
    # 創建註冊器
    registry = create_default_registry()
    
    # 註冊內建函數
    register_builtin_functions(registry)
    
    # 列出所有函數
    print("註冊的 DSL 函數:")
    for category in ['condition', 'logic', 'action', 'script']:
        functions = registry.list_functions(category=category)
        if functions:
            print(f"\n{category.upper()} 函數:")
            for func in functions:
                print(f"  - {func.name} ({func.source}): {func.description}")
    
    # 測試函數調用
    try:
        result = registry.call_function('len', 'dsl_runtime', obj=[1, 2, 3])
        print(f"\n測試調用 len: {result}")
        
        registry.call_function('log_info', 'dsl_runtime', 
                             message="DSL 函數註冊器測試",
                             context={'test': True})
        
    except Exception as e:
        print(f"函數調用測試失敗: {e}")