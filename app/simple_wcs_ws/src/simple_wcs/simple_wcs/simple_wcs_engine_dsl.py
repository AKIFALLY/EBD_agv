"""
Simple WCS Engine DSL Extension
擴展 Simple WCS Engine 以支援 YAML DSL 執行

這個模組將 YAML DSL 解析器整合到 Simple WCS Engine 中，
提供統一的業務流程執行環境。
"""

import logging
import asyncio
from typing import List, Dict, Any, Optional
from pathlib import Path

# Simple WCS 原有模組
from .flow_parser import FlowParser, BusinessFlow
from .yaml_dsl_parser import YAMLDSLParser, YAMLDSLExecutor, DSLScript
from .dsl_function_registry import DSLFunctionRegistry, register_builtin_functions, register_wcs_functions


class SimpleWCSEngineDSL:
    """Simple WCS Engine DSL 擴展版本
    
    在原有 Simple WCS Engine 基礎上添加 DSL 支援：
    1. 解析 DSL 格式的流程檔案
    2. 執行 DSL 腳本邏輯
    3. 與現有 WCS 函數整合
    4. 提供統一的執行環境
    """
    
    def __init__(self, flows_dir: str = "/app/config/wcs/flows"):
        self.flows_dir = flows_dir
        self.logger = logging.getLogger('simple_wcs.engine_dsl')
        
        # 初始化解析器和執行器
        self.flow_parser = FlowParser(flows_dir)
        self.dsl_parser = YAMLDSLParser()
        self.dsl_executor = YAMLDSLExecutor()
        
        # 初始化函數註冊器
        self.function_registry = DSLFunctionRegistry()
        register_builtin_functions(self.function_registry)
        
        # 業務流程和 DSL 腳本快取
        self.business_flows: List[BusinessFlow] = []
        self.dsl_scripts: List[DSLScript] = []
        
        self.logger.info("Simple WCS Engine DSL 已初始化")
    
    def register_wcs_functions(self, 
                              unified_decision_engine=None,
                              enhanced_database_client=None,
                              unified_task_manager=None,
                              location_manager=None):
        """註冊 WCS 系統函數到 DSL 環境"""
        register_wcs_functions(
            self.function_registry,
            unified_decision_engine,
            enhanced_database_client, 
            unified_task_manager,
            location_manager
        )
        
        # 將函數註冊器設定到 DSL 執行器
        self.dsl_executor.function_registry = self.function_registry
    
    def load_flows(self) -> Dict[str, Any]:
        """載入所有業務流程（包含 DSL 和傳統格式）"""
        load_results = {
            'business_flows': 0,
            'dsl_scripts': 0,
            'errors': [],
            'total_enabled': 0
        }
        
        try:
            # 清空快取
            self.business_flows = []
            self.dsl_scripts = []
            
            # 使用 FlowParser 載入（會自動處理 DSL 轉換）
            flows = self.flow_parser.parse(self.flows_dir)
            self.business_flows = flows
            load_results['business_flows'] = len(flows)
            
            # 直接載入 DSL 腳本（用於原生 DSL 執行）
            dsl_scripts = self._load_dsl_scripts()
            self.dsl_scripts = dsl_scripts
            load_results['dsl_scripts'] = len(dsl_scripts)
            
            # 統計啟用的流程
            enabled_business_flows = [f for f in self.business_flows if f.enabled]
            enabled_dsl_scripts = [s for s in self.dsl_scripts if s.enabled]
            load_results['total_enabled'] = len(enabled_business_flows) + len(enabled_dsl_scripts)
            
            self.logger.info(f"載入完成 - BusinessFlow: {len(flows)}, DSL Scripts: {len(dsl_scripts)}")
            
        except Exception as e:
            error_msg = f"載入業務流程失敗: {e}"
            self.logger.error(error_msg)
            load_results['errors'].append(error_msg)
        
        return load_results
    
    def _load_dsl_scripts(self) -> List[DSLScript]:
        """直接載入 DSL 腳本檔案"""
        scripts = []
        flows_path = Path(self.flows_dir)
        
        if not flows_path.exists():
            return scripts
        
        # 查找所有 YAML 檔案
        yaml_files = list(flows_path.glob("*.yaml"))
        
        for yaml_file in yaml_files:
            try:
                script = self.dsl_parser.parse_dsl_script(str(yaml_file))
                if script:
                    scripts.append(script)
                    self.logger.debug(f"載入 DSL 腳本: {script.name}")
                    
            except Exception as e:
                self.logger.warning(f"載入 DSL 腳本失敗 '{yaml_file.name}': {e}")
        
        return scripts
    
    def execute_business_flows(self) -> Dict[str, Any]:
        """執行所有業務流程（傳統格式）"""
        execution_results = {
            'executed_flows': 0,
            'generated_tasks': 0,
            'errors': [],
            'flow_results': []
        }
        
        enabled_flows = [f for f in self.business_flows if f.enabled]
        
        for flow in enabled_flows:
            try:
                # 檢查是否為 DSL 轉換的流程
                if flow.debug.get('dsl_converted', False):
                    # 尋找對應的原始 DSL 腳本並執行
                    original_script = self._find_corresponding_dsl_script(flow)
                    if original_script:
                        dsl_result = self.dsl_executor.execute_script(original_script)
                        flow_result = {
                            'flow_name': flow.name,
                            'flow_type': 'dsl_converted',
                            'execution_status': dsl_result['execution_status'],
                            'step_results': len(dsl_result['step_results']),
                            'generated_tasks': 1 if dsl_result['execution_status'] == 'completed' else 0
                        }
                    else:
                        # 使用傳統方式執行
                        flow_result = self._execute_traditional_flow(flow)
                else:
                    # 傳統業務流程執行
                    flow_result = self._execute_traditional_flow(flow)
                
                execution_results['flow_results'].append(flow_result)
                execution_results['executed_flows'] += 1
                execution_results['generated_tasks'] += flow_result.get('generated_tasks', 0)
                
            except Exception as e:
                error_msg = f"執行業務流程失敗 '{flow.name}': {e}"
                self.logger.error(error_msg)
                execution_results['errors'].append(error_msg)
        
        return execution_results
    
    def execute_dsl_scripts(self) -> Dict[str, Any]:
        """執行所有 DSL 腳本（原生 DSL 執行）"""
        execution_results = {
            'executed_scripts': 0,
            'successful_scripts': 0,
            'errors': [],
            'script_results': []
        }
        
        enabled_scripts = [s for s in self.dsl_scripts if s.enabled]
        
        for script in enabled_scripts:
            try:
                result = self.dsl_executor.execute_script(script)
                
                script_result = {
                    'script_name': script.name,
                    'execution_status': result['execution_status'],
                    'steps_executed': len(result['step_results']),
                    'variables_final': result['variables'],
                    'errors': result['errors']
                }
                
                execution_results['script_results'].append(script_result)
                execution_results['executed_scripts'] += 1
                
                if result['execution_status'] == 'completed':
                    execution_results['successful_scripts'] += 1
                
            except Exception as e:
                error_msg = f"執行 DSL 腳本失敗 '{script.name}': {e}"
                self.logger.error(error_msg)
                execution_results['errors'].append(error_msg)
        
        return execution_results
    
    def _find_corresponding_dsl_script(self, flow: BusinessFlow) -> Optional[DSLScript]:
        """尋找對應的原始 DSL 腳本"""
        for script in self.dsl_scripts:
            if script.name == flow.name or script.work_id == flow.work_id:
                return script
        return None
    
    def _execute_traditional_flow(self, flow: BusinessFlow) -> Dict[str, Any]:
        """執行傳統業務流程邏輯"""
        # 模擬傳統流程執行
        self.logger.info(f"執行傳統業務流程: {flow.name}")
        
        flow_result = {
            'flow_name': flow.name,
            'flow_type': 'traditional',
            'execution_status': 'completed',
            'conditions_checked': len(flow.trigger_conditions),
            'action_executed': flow.action.function,
            'generated_tasks': 1
        }
        
        # 在實際實作中，這裡會：
        # 1. 檢查 trigger_conditions
        # 2. 調用對應的 WCS 函數
        # 3. 生成任務決策
        # 4. 創建資料庫任務
        
        return flow_result
    
    async def run_decision_cycle(self) -> Dict[str, Any]:
        """執行完整的決策週期（異步版本）"""
        cycle_results = {
            'cycle_start_time': asyncio.get_event_loop().time(),
            'business_flow_results': {},
            'dsl_script_results': {},
            'total_generated_tasks': 0,
            'errors': []
        }
        
        try:
            # 執行業務流程
            business_results = self.execute_business_flows()
            cycle_results['business_flow_results'] = business_results
            cycle_results['total_generated_tasks'] += business_results['generated_tasks']
            
            # 執行 DSL 腳本  
            dsl_results = self.execute_dsl_scripts()
            cycle_results['dsl_script_results'] = dsl_results
            # DSL 腳本的任務生成需要從執行結果中統計
            
            cycle_results['cycle_end_time'] = asyncio.get_event_loop().time()
            cycle_results['cycle_duration'] = cycle_results['cycle_end_time'] - cycle_results['cycle_start_time']
            
            self.logger.info(f"決策週期完成 - 生成任務: {cycle_results['total_generated_tasks']}, "
                           f"耗時: {cycle_results['cycle_duration']:.3f}s")
            
        except Exception as e:
            error_msg = f"決策週期執行失敗: {e}"
            self.logger.error(error_msg)
            cycle_results['errors'].append(error_msg)
        
        return cycle_results
    
    def get_engine_status(self) -> Dict[str, Any]:
        """取得引擎狀態資訊"""
        return {
            'flows_dir': self.flows_dir,
            'business_flows_loaded': len(self.business_flows),
            'dsl_scripts_loaded': len(self.dsl_scripts),
            'enabled_business_flows': len([f for f in self.business_flows if f.enabled]),
            'enabled_dsl_scripts': len([s for s in self.dsl_scripts if s.enabled]),
            'registered_functions': len(self.function_registry.functions),
            'registered_sources': list(self.function_registry.sources.keys()),
            'dsl_support': True,
            'legacy_support': True
        }
    
    def validate_configuration(self) -> Dict[str, Any]:
        """驗證引擎配置"""
        validation_results = {
            'flows_dir_exists': Path(self.flows_dir).exists(),
            'flows_validation': {},
            'dsl_validation': {},
            'function_registry_status': {},
            'errors': [],
            'warnings': []
        }
        
        try:
            # 驗證業務流程
            if self.business_flows:
                flows_validation = self.flow_parser.validate_flows(self.business_flows)
                validation_results['flows_validation'] = flows_validation
            
            # 驗證 DSL 腳本
            dsl_validation = self._validate_dsl_scripts()
            validation_results['dsl_validation'] = dsl_validation
            
            # 驗證函數註冊狀態
            registry_status = {
                'total_functions': len(self.function_registry.functions),
                'condition_functions': len(self.function_registry.list_functions(category='condition')),
                'logic_functions': len(self.function_registry.list_functions(category='logic')),
                'action_functions': len(self.function_registry.list_functions(category='action')),
                'script_functions': len(self.function_registry.list_functions(category='script'))
            }
            validation_results['function_registry_status'] = registry_status
            
        except Exception as e:
            error_msg = f"配置驗證失敗: {e}"
            validation_results['errors'].append(error_msg)
            self.logger.error(error_msg)
        
        return validation_results
    
    def _validate_dsl_scripts(self) -> Dict[str, Any]:
        """驗證 DSL 腳本"""
        validation = {
            'total_scripts': len(self.dsl_scripts),
            'valid_scripts': 0,
            'invalid_scripts': 0,
            'validation_errors': []
        }
        
        for script in self.dsl_scripts:
            try:
                # 基本驗證
                if not script.steps:
                    validation['validation_errors'].append(f"DSL 腳本 '{script.name}' 沒有執行步驟")
                    validation['invalid_scripts'] += 1
                    continue
                
                # 檢查步驟中使用的函數是否已註冊
                for step in script.steps:
                    if step.function and step.source:
                        dsl_function = self.function_registry.get_function(step.function, step.source)
                        if not dsl_function:
                            validation['validation_errors'].append(
                                f"DSL 腳本 '{script.name}' 步驟 '{step.step_id}' "
                                f"使用未註冊函數: {step.function} (來源: {step.source})"
                            )
                
                validation['valid_scripts'] += 1
                
            except Exception as e:
                validation['validation_errors'].append(f"驗證 DSL 腳本 '{script.name}' 失敗: {e}")
                validation['invalid_scripts'] += 1
        
        return validation


# 整合到現有 Simple WCS 系統的介面類別
class DSLIntegrationManager:
    """DSL 整合管理器 - 管理 DSL 與現有系統的整合"""
    
    def __init__(self, engine_dsl: SimpleWCSEngineDSL):
        self.engine_dsl = engine_dsl
        self.logger = logging.getLogger('simple_wcs.dsl_integration')
    
    def create_dsl_from_business_flow(self, flow: BusinessFlow) -> Optional[DSLScript]:
        """將現有業務流程轉換為 DSL 腳本"""
        try:
            dsl_script = self.engine_dsl.dsl_parser.convert_flow_to_dsl(flow)
            return dsl_script
        except Exception as e:
            self.logger.error(f"轉換業務流程為 DSL 失敗: {e}")
            return None
    
    def export_dsl_script(self, script: DSLScript, output_path: str) -> bool:
        """匯出 DSL 腳本為 YAML 檔案"""
        try:
            yaml_content = self.engine_dsl.dsl_parser.generate_dsl_yaml(script)
            
            with open(output_path, 'w', encoding='utf-8') as f:
                f.write(yaml_content)
            
            self.logger.info(f"DSL 腳本已匯出: {output_path}")
            return True
            
        except Exception as e:
            self.logger.error(f"匯出 DSL 腳本失敗: {e}")
            return False
    
    def import_dsl_script(self, file_path: str) -> Optional[DSLScript]:
        """匯入 DSL 腳本檔案"""
        try:
            script = self.engine_dsl.dsl_parser.parse_dsl_script(file_path)
            if script:
                self.engine_dsl.dsl_scripts.append(script)
                self.logger.info(f"DSL 腳本已匯入: {script.name}")
            return script
            
        except Exception as e:
            self.logger.error(f"匯入 DSL 腳本失敗: {e}")
            return None
    
    def sync_with_flow_designer(self) -> Dict[str, Any]:
        """與 Flow Designer 同步"""
        # 這個方法將在第三階段實作
        # 用於與 Flow Designer 的雙向同步
        sync_results = {
            'flows_synced': 0,
            'dsl_scripts_synced': 0,
            'conflicts': [],
            'sync_status': 'not_implemented'
        }
        
        self.logger.info("Flow Designer 同步功能將在第三階段實作")
        return sync_results


# 工廠函數
def create_simple_wcs_engine_dsl(flows_dir: str = "/app/config/wcs/flows",
                                unified_decision_engine=None,
                                enhanced_database_client=None,
                                unified_task_manager=None,
                                location_manager=None) -> SimpleWCSEngineDSL:
    """創建 Simple WCS Engine DSL 實例"""
    engine = SimpleWCSEngineDSL(flows_dir)
    
    # 註冊 WCS 函數
    if any([unified_decision_engine, enhanced_database_client, unified_task_manager, location_manager]):
        engine.register_wcs_functions(
            unified_decision_engine,
            enhanced_database_client,
            unified_task_manager,
            location_manager
        )
    
    return engine


# 測試範例
if __name__ == "__main__":
    async def test_dsl_engine():
        # 創建 DSL 引擎
        engine = create_simple_wcs_engine_dsl()
        
        # 載入流程
        load_results = engine.load_flows()
        print(f"載入結果: {load_results}")
        
        # 驗證配置
        validation = engine.validate_configuration()
        print(f"\n驗證結果: {validation}")
        
        # 取得引擎狀態
        status = engine.get_engine_status()
        print(f"\n引擎狀態: {status}")
        
        # 執行決策週期
        cycle_results = await engine.run_decision_cycle()
        print(f"\n決策週期結果: {cycle_results}")
    
    # 執行測試
    asyncio.run(test_dsl_engine())