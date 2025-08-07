"""
YAML DSL Parser - WCS 業務流程 DSL 解析器
基於現有 flow_parser.py 擴展，支援 DSL 語法解析和執行

這個解析器將 YAML DSL 程式語言轉換為可執行的業務流程邏輯。
"""

import logging
import yaml
import re
from dataclasses import dataclass, field
from typing import List, Dict, Any, Optional, Union
from pathlib import Path
# Avoid circular import - will import FlowParser classes when needed


@dataclass
class Variable:
    """DSL 變數定義"""
    name: str
    type: str
    value: Any
    description: str = ""
    scope: str = "local"  # local, global, input, output


@dataclass
class Expression:
    """DSL 表達式定義"""
    expression: str
    variables: List[str] = field(default_factory=list)
    functions: List[str] = field(default_factory=list)


@dataclass
class DSLStep:
    """DSL 執行步驟定義"""
    step_id: str
    step_type: str  # condition, logic, action, script
    description: str
    function: str
    source: str
    parameters: Dict[str, Any] = field(default_factory=dict)
    variables: Dict[str, Variable] = field(default_factory=dict)
    conditions: List[Expression] = field(default_factory=list)
    next_steps: List[str] = field(default_factory=list)
    error_handling: Dict[str, Any] = field(default_factory=dict)


@dataclass
class DSLScript:
    """DSL 腳本定義（程式化業務流程）"""
    name: str
    description: str
    version: str
    author: str
    variables: Dict[str, Variable] = field(default_factory=dict)
    steps: List[DSLStep] = field(default_factory=list)
    priority: int = 50
    work_id: str = "000000"
    enabled: bool = True
    metadata: Dict[str, Any] = field(default_factory=dict)


class VariableResolver:
    """變數解析器 - 處理 DSL 變數系統"""
    
    def __init__(self):
        self.global_variables: Dict[str, Variable] = {}
        self.local_variables: Dict[str, Variable] = {}
        self.logger = logging.getLogger('yaml_dsl.variable_resolver')
    
    def define_variable(self, variable: Variable, scope: str = "local"):
        """定義變數"""
        variable.scope = scope
        if scope == "global":
            self.global_variables[variable.name] = variable
        else:
            self.local_variables[variable.name] = variable
        
        self.logger.debug(f"定義變數 {variable.name} = {variable.value} (scope: {scope})")
    
    def get_variable(self, name: str) -> Optional[Variable]:
        """取得變數值"""
        # 優先查找本地變數，再查找全域變數
        if name in self.local_variables:
            return self.local_variables[name]
        elif name in self.global_variables:
            return self.global_variables[name]
        else:
            return None
    
    def resolve_expression(self, expression: str) -> Any:
        """解析表達式中的變數"""
        # 簡單的變數替換解析
        pattern = r'\$\{(\w+)\}'
        
        def replace_var(match):
            var_name = match.group(1)
            var = self.get_variable(var_name)
            if var:
                return str(var.value)
            else:
                raise ValueError(f"未定義變數: {var_name}")
        
        resolved = re.sub(pattern, replace_var, expression)
        return self._evaluate_expression(resolved)
    
    def _evaluate_expression(self, expression: str) -> Any:
        """評估表達式（安全評估）"""
        # 基本的算術和邏輯表達式評估
        # 在生產環境中應該使用更安全的表達式評估器
        try:
            # 簡單的安全檢查
            if any(dangerous in expression for dangerous in ['import', 'exec', 'eval', '__']):
                raise ValueError(f"不安全的表達式: {expression}")
            
            # 基本算術和比較運算
            return eval(expression)
        except Exception as e:
            self.logger.error(f"表達式評估失敗 '{expression}': {e}")
            return None


class ExpressionEvaluator:
    """表達式評估器 - 處理條件判斷和邏輯運算"""
    
    def __init__(self, variable_resolver: VariableResolver):
        self.resolver = variable_resolver
        self.logger = logging.getLogger('yaml_dsl.expression_evaluator')
    
    def evaluate_condition(self, condition: Expression) -> bool:
        """評估條件表達式"""
        try:
            result = self.resolver.resolve_expression(condition.expression)
            return bool(result) if result is not None else False
        except Exception as e:
            self.logger.error(f"條件評估失敗 '{condition.expression}': {e}")
            return False
    
    def evaluate_conditions(self, conditions: List[Expression], operator: str = "AND") -> bool:
        """評估多個條件"""
        if not conditions:
            return True
        
        results = [self.evaluate_condition(cond) for cond in conditions]
        
        if operator.upper() == "AND":
            return all(results)
        elif operator.upper() == "OR":
            return any(results)
        else:
            raise ValueError(f"不支援的條件運算子: {operator}")


class YAMLDSLParser:
    """YAML DSL 解析器 - 將 DSL 語法轉換為可執行腳本"""
    
    def __init__(self):
        self.logger = logging.getLogger('yaml_dsl.parser')
        self.flow_parser = None  # Lazy initialization to avoid circular import
        
    def parse_dsl_script(self, file_path: str) -> Optional[DSLScript]:
        """解析 DSL 腳本檔案"""
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                dsl_data = yaml.safe_load(f)
            
            return self._parse_dsl_data(dsl_data)
            
        except Exception as e:
            self.logger.error(f"解析 DSL 腳本失敗 '{file_path}': {e}")
            return None
    
    def _parse_dsl_data(self, data: Dict[str, Any]) -> Optional[DSLScript]:
        """解析 DSL 資料結構"""
        try:
            # 解析變數定義
            variables = {}
            for var_name, var_config in data.get('variables', {}).items():
                var = Variable(
                    name=var_name,
                    type=var_config.get('type', 'any'),
                    value=var_config.get('value'),
                    description=var_config.get('description', ''),
                    scope=var_config.get('scope', 'local')
                )
                variables[var_name] = var
            
            # 解析執行步驟
            steps = []
            for step_config in data.get('steps', []):
                step = self._parse_step(step_config)
                if step:
                    steps.append(step)
            
            # 建立 DSL 腳本物件
            script = DSLScript(
                name=data.get('name', 'Unnamed Script'),
                description=data.get('description', ''),
                version=data.get('version', '1.0'),
                author=data.get('author', 'Unknown'),
                variables=variables,
                steps=steps,
                priority=data.get('priority', 50),
                work_id=data.get('work_id', '000000'),
                enabled=data.get('enabled', True),
                metadata=data.get('metadata', {})
            )
            
            return script
            
        except Exception as e:
            self.logger.error(f"解析 DSL 資料結構失敗: {e}")
            return None
    
    def _parse_step(self, step_config: Dict[str, Any]) -> Optional[DSLStep]:
        """解析單個執行步驟"""
        try:
            # 解析步驟變數
            step_variables = {}
            for var_name, var_config in step_config.get('variables', {}).items():
                var = Variable(
                    name=var_name,
                    type=var_config.get('type', 'any'),
                    value=var_config.get('value'),
                    description=var_config.get('description', ''),
                    scope=var_config.get('scope', 'local')
                )
                step_variables[var_name] = var
            
            # 解析條件表達式
            conditions = []
            for cond_config in step_config.get('conditions', []):
                condition = Expression(
                    expression=cond_config.get('expression', 'true'),
                    variables=cond_config.get('variables', []),
                    functions=cond_config.get('functions', [])
                )
                conditions.append(condition)
            
            # 建立步驟物件
            step = DSLStep(
                step_id=step_config.get('id', f"step_{len(step_config)}"),
                step_type=step_config.get('type', 'action'),
                description=step_config.get('description', ''),
                function=step_config.get('function', ''),
                source=step_config.get('source', ''),
                parameters=step_config.get('parameters', {}),
                variables=step_variables,
                conditions=conditions,
                next_steps=step_config.get('next_steps', []),
                error_handling=step_config.get('error_handling', {})
            )
            
            return step
            
        except Exception as e:
            self.logger.error(f"解析執行步驟失敗: {e}")
            return None
    
    def convert_flow_to_dsl(self, flow) -> DSLScript:
        """將現有業務流程轉換為 DSL 腳本"""
        # 建立變數定義
        variables = {
            'flow_name': Variable('flow_name', 'string', flow.name),
            'priority': Variable('priority', 'integer', flow.priority),
            'work_id': Variable('work_id', 'string', flow.work_id)
        }
        
        # 轉換觸發條件為條件步驟
        steps = []
        for i, condition in enumerate(flow.trigger_conditions):
            condition_step = DSLStep(
                step_id=f"condition_{i}",
                step_type="condition",
                description=condition.description,
                function=condition.condition,
                source="enhanced_database_client",
                parameters=condition.parameters,
                conditions=[Expression(f"check_{condition.condition}")]
            )
            steps.append(condition_step)
        
        # 轉換執行動作為動作步驟
        action_step = DSLStep(
            step_id="main_action",
            step_type="action",
            description=f"執行 {flow.action.type} 動作",
            function=flow.action.function,
            source=flow.action.task_type,
            parameters={
                'type': flow.action.type,
                'model': flow.action.model,
                'api': flow.action.api,
                'mission_type': flow.action.mission_type,
                'path': flow.action.path
            }
        )
        steps.append(action_step)
        
        # 建立 DSL 腳本
        dsl_script = DSLScript(
            name=f"DSL_{flow.name}",
            description=f"自動轉換: {flow.description}",
            version="1.0",
            author="FlowParser",
            variables=variables,
            steps=steps,
            priority=flow.priority,
            work_id=flow.work_id,
            enabled=flow.enabled,
            metadata={
                'converted_from': 'BusinessFlow',
                'original_name': flow.name,
                'applicable_rooms': flow.applicable_rooms,
                'applicable_locations': flow.applicable_locations
            }
        )
        
        return dsl_script
    
    def generate_dsl_yaml(self, script: DSLScript) -> str:
        """產生 DSL YAML 檔案內容"""
        dsl_data = {
            'name': script.name,
            'description': script.description,
            'version': script.version,
            'author': script.author,
            'priority': script.priority,
            'work_id': script.work_id,
            'enabled': script.enabled,
            'metadata': script.metadata,
            
            'variables': {
                var_name: {
                    'type': var.type,
                    'value': var.value,
                    'description': var.description,
                    'scope': var.scope
                }
                for var_name, var in script.variables.items()
            },
            
            'steps': [
                {
                    'id': step.step_id,
                    'type': step.step_type,
                    'description': step.description,
                    'function': step.function,
                    'source': step.source,
                    'parameters': step.parameters,
                    'variables': {
                        var_name: {
                            'type': var.type,
                            'value': var.value,
                            'description': var.description,
                            'scope': var.scope
                        }
                        for var_name, var in step.variables.items()
                    },
                    'conditions': [
                        {
                            'expression': cond.expression,
                            'variables': cond.variables,
                            'functions': cond.functions
                        }
                        for cond in step.conditions
                    ],
                    'next_steps': step.next_steps,
                    'error_handling': step.error_handling
                }
                for step in script.steps
            ]
        }
        
        return yaml.dump(dsl_data, default_flow_style=False, allow_unicode=True, indent=2)


class YAMLDSLExecutor:
    """YAML DSL 執行器 - 執行 DSL 腳本邏輯"""
    
    def __init__(self):
        self.variable_resolver = VariableResolver()
        self.expression_evaluator = ExpressionEvaluator(self.variable_resolver)
        self.logger = logging.getLogger('yaml_dsl.executor')
        self.execution_context = {}
        
    def execute_script(self, script: DSLScript) -> Dict[str, Any]:
        """執行 DSL 腳本"""
        self.logger.info(f"開始執行 DSL 腳本: {script.name}")
        
        # 初始化執行環境
        self._initialize_execution_context(script)
        
        # 執行腳本步驟
        execution_results = {
            'script_name': script.name,
            'execution_status': 'started',
            'step_results': [],
            'variables': {},
            'errors': []
        }
        
        try:
            for step in script.steps:
                step_result = self._execute_step(step)
                execution_results['step_results'].append(step_result)
                
                # 檢查步驟執行是否成功
                if not step_result.get('success', False):
                    self.logger.error(f"步驟執行失敗: {step.step_id}")
                    execution_results['execution_status'] = 'failed'
                    break
            
            if execution_results['execution_status'] != 'failed':
                execution_results['execution_status'] = 'completed'
            
            # 收集最終變數狀態
            execution_results['variables'] = self._get_variable_state()
            
        except Exception as e:
            self.logger.error(f"DSL 腳本執行異常: {e}")
            execution_results['execution_status'] = 'error'
            execution_results['errors'].append(str(e))
        
        return execution_results
    
    def _initialize_execution_context(self, script: DSLScript):
        """初始化執行環境"""
        # 載入腳本變數
        for var_name, variable in script.variables.items():
            self.variable_resolver.define_variable(variable)
        
        # 設定執行上下文
        self.execution_context = {
            'script_name': script.name,
            'work_id': script.work_id,
            'priority': script.priority,
            'metadata': script.metadata
        }
    
    def _execute_step(self, step: DSLStep) -> Dict[str, Any]:
        """執行單個步驟"""
        self.logger.debug(f"執行步驟: {step.step_id} ({step.step_type})")
        
        step_result = {
            'step_id': step.step_id,
            'step_type': step.step_type,
            'success': False,
            'result': None,
            'variables_updated': {},
            'errors': []
        }
        
        try:
            # 檢查步驟條件
            if step.conditions:
                conditions_met = self.expression_evaluator.evaluate_conditions(step.conditions)
                if not conditions_met:
                    self.logger.info(f"步驟 {step.step_id} 條件不滿足，跳過執行")
                    step_result['success'] = True
                    step_result['result'] = 'conditions_not_met'
                    return step_result
            
            # 載入步驟變數
            for var_name, variable in step.variables.items():
                self.variable_resolver.define_variable(variable)
                step_result['variables_updated'][var_name] = variable.value
            
            # 根據步驟類型執行對應邏輯
            if step.step_type == "condition":
                result = self._execute_condition_step(step)
            elif step.step_type == "logic":
                result = self._execute_logic_step(step)
            elif step.step_type == "action":
                result = self._execute_action_step(step)
            elif step.step_type == "script":
                result = self._execute_script_step(step)
            else:
                raise ValueError(f"不支援的步驟類型: {step.step_type}")
            
            step_result['success'] = True
            step_result['result'] = result
            
        except Exception as e:
            self.logger.error(f"步驟執行失敗 {step.step_id}: {e}")
            step_result['errors'].append(str(e))
            
            # 處理錯誤恢復
            if step.error_handling:
                self._handle_step_error(step, e)
        
        return step_result
    
    def _execute_condition_step(self, step: DSLStep) -> Any:
        """執行條件檢查步驟"""
        # 模擬條件檢查邏輯
        self.logger.info(f"執行條件檢查: {step.function}")
        
        # 在實際實作中，這裡會調用對應的 WCS 函數
        # 例如：enhanced_database_client.check_locations_available()
        
        return {"condition_result": True, "checked": step.function}
    
    def _execute_logic_step(self, step: DSLStep) -> Any:
        """執行邏輯處理步驟"""
        # 模擬邏輯處理
        self.logger.info(f"執行邏輯處理: {step.function}")
        
        # 在實際實作中，這裡會調用對應的邏輯函數
        # 例如：unified_decision_engine.prioritize_and_schedule()
        
        return {"logic_result": "processed", "function": step.function}
    
    def _execute_action_step(self, step: DSLStep) -> Any:
        """執行動作步驟"""
        # 模擬動作執行
        self.logger.info(f"執行動作: {step.function}")
        
        # 在實際實作中，這裡會調用對應的動作函數
        # 例如：unified_task_manager.create_task_from_decision()
        
        return {"action_result": "executed", "function": step.function}
    
    def _execute_script_step(self, step: DSLStep) -> Any:
        """執行腳本步驟"""
        # 模擬腳本執行
        self.logger.info(f"執行腳本: {step.function}")
        
        # 在實際實作中，這裡會執行自定義腳本邏輯
        
        return {"script_result": "executed", "function": step.function}
    
    def _handle_step_error(self, step: DSLStep, error: Exception):
        """處理步驟執行錯誤"""
        error_config = step.error_handling
        
        if error_config.get('retry', False):
            retry_count = error_config.get('retry_count', 1)
            self.logger.info(f"步驟 {step.step_id} 錯誤重試，最多 {retry_count} 次")
        
        if error_config.get('skip_on_error', False):
            self.logger.info(f"步驟 {step.step_id} 錯誤跳過")
        
        if error_config.get('fallback_step'):
            fallback_step_id = error_config['fallback_step']
            self.logger.info(f"步驟 {step.step_id} 錯誤，執行備用步驟: {fallback_step_id}")
    
    def _get_variable_state(self) -> Dict[str, Any]:
        """取得當前變數狀態"""
        state = {}
        
        # 收集全域變數
        for var_name, variable in self.variable_resolver.global_variables.items():
            state[f"global.{var_name}"] = variable.value
        
        # 收集本地變數
        for var_name, variable in self.variable_resolver.local_variables.items():
            state[f"local.{var_name}"] = variable.value
        
        return state


# 測試用的 DSL 腳本範例
if __name__ == "__main__":
    # 建立 DSL 解析器
    parser = YAMLDSLParser()
    
    # 測試範例 DSL 腳本資料
    test_dsl_data = {
        'name': 'Test AGV Rotation Flow',
        'description': 'AGV 旋轉流程的 DSL 測試',
        'version': '1.0',
        'author': 'YAML DSL System',
        'priority': 100,
        'work_id': '220001',
        'enabled': True,
        
        'variables': {
            'target_location': {
                'type': 'integer',
                'value': 10001,
                'description': '目標位置ID'
            },
            'current_location': {
                'type': 'integer',
                'value': 10002,
                'description': '當前位置ID'
            }
        },
        
        'steps': [
            {
                'id': 'check_rotation_needed',
                'type': 'condition',
                'description': '檢查是否需要旋轉',
                'function': 'check_agv_rotation_flow',
                'source': 'unified_decision_engine',
                'conditions': [
                    {'expression': '${target_location} != ${current_location}'}
                ]
            },
            {
                'id': 'create_rotation_task',
                'type': 'action',
                'description': '建立旋轉任務',
                'function': 'create_task_from_decision',
                'source': 'unified_task_manager',
                'parameters': {
                    'work_id': '${work_id}',
                    'priority': '${priority}'
                }
            }
        ]
    }
    
    # 解析 DSL 腳本
    script = parser._parse_dsl_data(test_dsl_data)
    if script:
        print(f"✅ DSL 腳本解析成功: {script.name}")
        print(f"   變數數量: {len(script.variables)}")
        print(f"   步驟數量: {len(script.steps)}")
        
        # 測試執行器
        executor = YAMLDSLExecutor()
        result = executor.execute_script(script)
        print(f"\n✅ DSL 腳本執行結果: {result['execution_status']}")
        print(f"   執行步驟: {len(result['step_results'])}")
    else:
        print("❌ DSL 腳本解析失敗")