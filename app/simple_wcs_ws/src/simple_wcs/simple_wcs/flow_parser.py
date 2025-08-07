"""
Simple WCS Flow Parser
解析 YAML 格式的業務流程配置檔案 - 支援多檔案結構
"""

import logging
import yaml
import os
import glob
from dataclasses import dataclass
from typing import List, Dict, Any, Optional, Union
from pathlib import Path


@dataclass
class TriggerCondition:
    """觸發條件定義"""
    condition: str
    description: str
    parameters: Dict[str, Any]


@dataclass
class FlowAction:
    """流程執行動作定義"""
    type: str
    task_type: str
    function: str
    model: str
    api: str
    mission_type: str
    path: Dict[str, Any]


@dataclass
class BusinessFlow:
    """業務流程定義"""
    name: str
    description: str
    priority: int
    work_id: str
    enabled: bool
    trigger_conditions: List[TriggerCondition]
    action: FlowAction
    applicable_rooms: List[int]
    applicable_locations: List[int]  # 新增適用位置列表
    debug: Dict[str, Any]
    
    def __str__(self):
        return f"BusinessFlow(name='{self.name}', priority={self.priority}, work_id='{self.work_id}', enabled={self.enabled})"


class FlowParser:
    """業務流程解析器 - 多檔案 YAML 格式 (flows/ 目錄)
    
    現在支援兩種格式：
    1. 原始 BusinessFlow 格式 (向下相容)
    2. 新的 YAML DSL 格式 (支援程式化步驟)
    """
    
    def __init__(self, flows_dir: str = None):
        self.flows_dir = flows_dir  # flows 目錄路徑
        self.logger = logging.getLogger('simple_wcs.flow_parser')
        self.dsl_parser = None  # 延遲載入以避免循環導入
    
    def parse(self, flows_dir: str = None) -> List[BusinessFlow]:
        """解析 flows/ 目錄中的所有業務流程配置
        
        Args:
            flows_dir: flows 目錄路徑
            
        Returns:
            解析出的業務流程列表
        """
        dir_path = flows_dir or self.flows_dir
        if not dir_path:
            raise ValueError("未指定 flows 目錄路徑")
        
        path_obj = Path(dir_path)
        
        if not path_obj.is_dir():
            raise ValueError(f"flows 目錄不存在: {dir_path}")
        
        try:
            flows = self._parse_flows_directory(str(path_obj))
            enabled_flows = [f for f in flows if f.enabled]
            
            self.logger.info(f"成功解析 {len(flows)} 個業務流程，其中 {len(enabled_flows)} 個已啟用")
            return enabled_flows
            
        except Exception as e:
            self.logger.error(f"解析業務流程失敗: {e}")
            return []
    
    def _parse_flows_directory(self, dir_path: str) -> List[BusinessFlow]:
        """解析 flows/ 目錄中的所有 YAML 檔案"""
        flows = []
        
        # 查找目錄中所有的 .yaml 檔案
        yaml_files = glob.glob(os.path.join(dir_path, "*.yaml"))
        
        if not yaml_files:
            self.logger.warning(f"flows 目錄中未找到 YAML 檔案: {dir_path}")
            return flows
        
        self.logger.info(f"發現 {len(yaml_files)} 個流程檔案")
        
        for yaml_file in sorted(yaml_files):
            try:
                flow_id = Path(yaml_file).stem  # 檔案名作為 flow_id
                
                with open(yaml_file, 'r', encoding='utf-8') as f:
                    flow_data = yaml.safe_load(f)
                
                # 解析單個流程檔案
                flow = self._parse_flow_file(flow_id, flow_data)
                if flow:
                    flows.append(flow)
                    self.logger.debug(f"✅ 解析流程: {flow.name} (檔案: {Path(yaml_file).name})")
                    
            except Exception as e:
                self.logger.error(f"❌ 解析流程檔案 '{Path(yaml_file).name}' 失敗: {e}")
                continue
        
        return flows
    
    def _parse_flow_file(self, flow_id: str, data: Dict[str, Any]) -> Optional[BusinessFlow]:
        """解析單個流程檔案的資料結構
        
        支援兩種格式：
        1. 原始 BusinessFlow 格式 (有 trigger_conditions)
        2. YAML DSL 格式 (有 steps)
        """
        try:
            # 檢查檔案格式類型
            if self._is_dsl_format(data):
                return self._convert_dsl_to_business_flow(flow_id, data)
            else:
                return self._parse_legacy_format(flow_id, data)
                
        except Exception as e:
            self.logger.error(f"解析單個流程檔案失敗: {e}")
            return None
    
    def _is_dsl_format(self, data: Dict[str, Any]) -> bool:
        """檢查是否為 DSL 格式
        
        DSL 格式的特徵：
        1. 有 steps 且為列表
        2. 有 variables 且為字典
        3. 有 metadata 欄位（可選）
        4. 欄位 flow_type 為 'dsl_script'（可選）
        """
        # 必要特徵：steps 和 variables
        has_steps = 'steps' in data and isinstance(data['steps'], list)
        has_variables = 'variables' in data and isinstance(data['variables'], dict)
        
        # 可選特徵：metadata 中的 flow_type
        metadata = data.get('metadata', {})
        is_dsl_type = metadata.get('flow_type') == 'dsl_script'
        
        # DSL 格式需要同時有 steps 和 variables，或明確標示為 dsl_script
        return (has_steps and has_variables) or is_dsl_type
    
    def _parse_legacy_format(self, flow_id: str, data: Dict[str, Any]) -> Optional[BusinessFlow]:
        """解析原始 BusinessFlow 格式"""
        # 解析觸發條件
        trigger_conditions = []
        for condition_config in data.get('trigger_conditions', []):
            trigger_condition = TriggerCondition(
                condition=condition_config['condition'],
                description=condition_config.get('description', ''),
                parameters=condition_config.get('parameters', {})
            )
            trigger_conditions.append(trigger_condition)
        
        # 解析執行動作
        action_config = data.get('action', {})
        action = FlowAction(
            type=action_config.get('type', 'create_task'),
            task_type=action_config.get('task_type', 'unknown'),
            function=action_config.get('function', ''),
            model=action_config.get('model', ''),
            api=action_config.get('api', ''),
            mission_type=action_config.get('mission_type', ''),
            path=action_config.get('path', {})
        )
        
        # 建立業務流程物件
        flow = BusinessFlow(
            name=data.get('name', flow_id),
            description=data.get('description', ''),
            priority=data.get('priority', 50),
            work_id=data.get('work_id', '000000'),
            enabled=data.get('enabled', True),
            trigger_conditions=trigger_conditions,
            action=action,
            applicable_rooms=data.get('applicable_rooms', []),
            applicable_locations=data.get('applicable_locations', []),
            debug=data.get('debug', {})
        )
        
        return flow
    
    def _convert_dsl_to_business_flow(self, flow_id: str, data: Dict[str, Any]) -> Optional[BusinessFlow]:
        """將 DSL 格式轉換為 BusinessFlow 格式（向下相容）
        
        增強的轉換邏輯：
        1. 更完整的 DSL 解析
        2. 更精確的步驟類型映射
        3. 更好的錯誤處理
        4. 向下相容性支援
        """
        try:
            # 延遲載入 DSL 解析器以避免循環導入
            if self.dsl_parser is None:
                try:
                    from .yaml_dsl_parser import YAMLDSLParser
                    self.dsl_parser = YAMLDSLParser()
                except ImportError:
                    # 使用絕對路徑導入作為備選方案
                    import sys
                    import os
                    current_dir = os.path.dirname(__file__)
                    if current_dir not in sys.path:
                        sys.path.insert(0, current_dir)
                    
                    from yaml_dsl_parser import YAMLDSLParser
                    self.dsl_parser = YAMLDSLParser()
            
            # 解析 DSL 腳本
            dsl_script = self.dsl_parser._parse_dsl_data(data)
            if not dsl_script:
                self.logger.warning(f"DSL 腳本解析失敗: {flow_id}")
                return None
            
            # 智能步驟分析和轉換
            conversion_result = self._analyze_dsl_steps(dsl_script.steps)
            trigger_conditions = conversion_result['trigger_conditions']
            action = conversion_result['action']
            
            # 提取適用位置（從 DSL legacy_format 或 metadata）
            applicable_rooms = []
            applicable_locations = []
            
            # 從 legacy_format 提取（如果存在）
            legacy_format = data.get('legacy_format', {})
            if legacy_format:
                applicable_rooms = legacy_format.get('applicable_rooms', [])
                applicable_locations = legacy_format.get('applicable_locations', [])
            
            # 從 metadata 提取（如果存在）
            if not applicable_rooms:
                applicable_rooms = dsl_script.metadata.get('applicable_rooms', [])
            if not applicable_locations:
                applicable_locations = dsl_script.metadata.get('applicable_locations', [])
            
            # 建立 BusinessFlow 物件
            flow = BusinessFlow(
                name=dsl_script.name,
                description=dsl_script.description,
                priority=dsl_script.priority,
                work_id=dsl_script.work_id,
                enabled=dsl_script.enabled,
                trigger_conditions=trigger_conditions,
                action=action,
                applicable_rooms=applicable_rooms,
                applicable_locations=applicable_locations,
                debug={
                    'dsl_converted': True, 
                    'original_format': 'dsl',
                    'step_count': len(dsl_script.steps),
                    'variable_count': len(dsl_script.variables),
                    'has_legacy_format': bool(legacy_format),
                    'conversion_timestamp': self._get_timestamp()
                }
            )
            
            self.logger.info(f"✅ DSL 格式轉換為 BusinessFlow: {flow.name} (步驟: {len(dsl_script.steps)})")
            return flow
            
        except Exception as e:
            self.logger.error(f"DSL 格式轉換失敗 '{flow_id}': {e}")
            return None
    
    def _analyze_dsl_steps(self, steps: List) -> Dict[str, Any]:
        """分析 DSL 步驟並轉換為 BusinessFlow 結構
        
        Args:
            steps: DSL 步驟列表
            
        Returns:
            包含 trigger_conditions 和 action 的字典
        """
        trigger_conditions = []
        action_steps = []
        
        for step in steps:
            step_type = step.step_type
            
            if step_type in ["condition", "logic"]:
                # 條件和邏輯步驟轉換為觸發條件
                condition = TriggerCondition(
                    condition=step.function or step.step_id,
                    description=step.description,
                    parameters=step.parameters
                )
                trigger_conditions.append(condition)
                
            elif step_type in ["action", "script"]:
                # 收集動作步驟
                action_steps.append(step)
        
        # 選擇主要動作（優先選擇 action 類型，然後是第一個 script）
        main_action_step = None
        for step in action_steps:
            if step.step_type == "action":
                main_action_step = step
                break
        
        if not main_action_step and action_steps:
            main_action_step = action_steps[0]
        
        # 建立 FlowAction
        if main_action_step:
            action = FlowAction(
                type=main_action_step.parameters.get('type', 'create_task'),
                task_type=main_action_step.parameters.get('task_type', 'dsl_converted'),
                function=main_action_step.function or 'execute_dsl_step',
                model=main_action_step.parameters.get('model', ''),
                api=main_action_step.parameters.get('api', ''),
                mission_type=main_action_step.parameters.get('mission_type', ''),
                path=main_action_step.parameters.get('path', {})
            )
        else:
            # 預設動作（當沒有找到任何動作步驟時）
            action = FlowAction(
                type='dsl_script',
                task_type='dsl_execution', 
                function='execute_dsl_script',
                model='',
                api='',
                mission_type='',
                path={}
            )
        
        return {
            'trigger_conditions': trigger_conditions,
            'action': action,
            'step_analysis': {
                'total_steps': len(steps),
                'condition_steps': len([s for s in steps if s.step_type in ["condition", "logic"]]),
                'action_steps': len(action_steps)
            }
        }
    
    def _get_timestamp(self) -> str:
        """取得當前時間戳"""
        from datetime import datetime
        return datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    
    def validate_flows(self, flows: List[BusinessFlow]) -> Dict[str, List[str]]:
        """驗證業務流程配置的完整性（增強版本支援 DSL）"""
        validation_results = {
            'errors': [],
            'warnings': [],
            'info': [],
            'dsl_analysis': []
        }
        
        if not flows:
            validation_results['errors'].append("未找到任何業務流程")
            return validation_results
        
        # 分類統計
        legacy_flows = [f for f in flows if not f.debug.get('dsl_converted', False)]
        dsl_flows = [f for f in flows if f.debug.get('dsl_converted', False)]
        
        # 檢查重複的優先級
        priorities = [flow.priority for flow in flows]
        duplicate_priorities = set([p for p in priorities if priorities.count(p) > 1])
        if duplicate_priorities:
            validation_results['warnings'].append(f"發現重複的優先級: {duplicate_priorities}")
        
        # 檢查重複的 work_id
        work_ids = [flow.work_id for flow in flows if flow.work_id]
        duplicate_work_ids = set([w for w in work_ids if work_ids.count(w) > 1])
        if duplicate_work_ids:
            validation_results['errors'].append(f"發現重複的 work_id: {duplicate_work_ids}")
        
        # 檢查每個流程的完整性
        for flow in flows:
            flow_type = "DSL" if flow.debug.get('dsl_converted') else "Legacy"
            
            # 基本欄位檢查
            if not flow.name:
                validation_results['errors'].append(f"業務流程缺少名稱 (work_id: {flow.work_id})")
            
            if not flow.work_id:
                validation_results['errors'].append(f"業務流程 '{flow.name}' 缺少 work_id")
            
            # 觸發條件檢查
            if not flow.trigger_conditions:
                if flow.debug.get('dsl_converted'):
                    validation_results['warnings'].append(f"DSL 流程 '{flow.name}' 轉換後沒有觸發條件")
                else:
                    validation_results['warnings'].append(f"傳統流程 '{flow.name}' 沒有觸發條件")
            
            # 動作檢查
            if not flow.action.type:
                validation_results['warnings'].append(f"{flow_type} 流程 '{flow.name}' 沒有定義動作類型")
            
            if not flow.action.function:
                validation_results['warnings'].append(f"{flow_type} 流程 '{flow.name}' 沒有定義動作函數")
            
            # DSL 特殊檢查
            if flow.debug.get('dsl_converted'):
                step_count = flow.debug.get('step_count', 0)
                variable_count = flow.debug.get('variable_count', 0)
                
                if step_count == 0:
                    validation_results['errors'].append(f"DSL 流程 '{flow.name}' 沒有執行步驟")
                
                validation_results['dsl_analysis'].append(
                    f"DSL 流程 '{flow.name}': {step_count} 步驟, {variable_count} 變數"
                )
        
        # 統計資訊
        validation_results['info'].extend([
            f"總共解析了 {len(flows)} 個業務流程",
            f"傳統格式: {len(legacy_flows)} 個",
            f"DSL 格式: {len(dsl_flows)} 個",
            f"已啟用: {len([f for f in flows if f.enabled])} 個"
        ])
        
        # DSL 轉換品質分析
        if dsl_flows:
            avg_steps = sum(f.debug.get('step_count', 0) for f in dsl_flows) / len(dsl_flows)
            validation_results['info'].append(f"DSL 流程平均步驟數: {avg_steps:.1f}")
            
            high_complexity_flows = [f for f in dsl_flows if f.debug.get('step_count', 0) > 8]
            if high_complexity_flows:
                validation_results['info'].append(
                    f"高複雜度 DSL 流程 (>8步驟): {len(high_complexity_flows)} 個"
                )
        
        return validation_results
    
    def analyze_format_compatibility(self, flows_dir: str = None) -> Dict[str, Any]:
        """分析目錄中所有檔案的格式相容性
        
        提供詳細的格式分析報告：
        1. 檔案格式分類
        2. DSL 轉換成功率
        3. 相容性問題識別
        4. 建議的處理方案
        """
        dir_path = flows_dir or self.flows_dir
        if not dir_path:
            raise ValueError("未指定 flows 目錄路徑")
        
        path_obj = Path(dir_path)
        if not path_obj.is_dir():
            raise ValueError(f"flows 目錄不存在: {dir_path}")
        
        compatibility_report = {
            'total_files': 0,
            'legacy_format_files': 0,
            'dsl_format_files': 0,
            'successful_conversions': 0,
            'failed_conversions': 0,
            'file_analysis': [],
            'recommendations': [],
            'warnings': []
        }
        
        import glob
        yaml_files = glob.glob(os.path.join(str(path_obj), "*.yaml"))
        compatibility_report['total_files'] = len(yaml_files)
        
        for yaml_file in sorted(yaml_files):
            file_name = Path(yaml_file).name
            file_analysis = {
                'file_name': file_name,
                'format_type': 'unknown',
                'parse_success': False,
                'conversion_success': False,
                'issues': [],
                'features': []
            }
            
            try:
                with open(yaml_file, 'r', encoding='utf-8') as f:
                    flow_data = yaml.safe_load(f)
                
                file_analysis['parse_success'] = True
                
                # 格式檢測
                if self._is_dsl_format(flow_data):
                    file_analysis['format_type'] = 'dsl'
                    compatibility_report['dsl_format_files'] += 1
                    
                    # DSL 特徵分析
                    dsl_features = self._analyze_dsl_features(flow_data)
                    file_analysis['features'] = dsl_features
                    
                    # 嘗試轉換
                    try:
                        converted_flow = self._convert_dsl_to_business_flow(Path(yaml_file).stem, flow_data)
                        if converted_flow:
                            file_analysis['conversion_success'] = True
                            compatibility_report['successful_conversions'] += 1
                        else:
                            file_analysis['issues'].append("DSL 轉換返回 None")
                            compatibility_report['failed_conversions'] += 1
                            
                    except Exception as e:
                        file_analysis['issues'].append(f"DSL 轉換失敗: {e}")
                        compatibility_report['failed_conversions'] += 1
                else:
                    file_analysis['format_type'] = 'legacy'
                    compatibility_report['legacy_format_files'] += 1
                    
                    # 傳統格式特徵分析
                    legacy_features = self._analyze_legacy_features(flow_data)
                    file_analysis['features'] = legacy_features
                    
                    # 檢查是否可以解析
                    try:
                        legacy_flow = self._parse_legacy_format(Path(yaml_file).stem, flow_data)
                        if legacy_flow:
                            file_analysis['conversion_success'] = True
                        else:
                            file_analysis['issues'].append("傳統格式解析返回 None")
                    except Exception as e:
                        file_analysis['issues'].append(f"傳統格式解析失敗: {e}")
                
            except Exception as e:
                file_analysis['issues'].append(f"檔案解析失敗: {e}")
            
            compatibility_report['file_analysis'].append(file_analysis)
        
        # 生成建議
        self._generate_compatibility_recommendations(compatibility_report)
        
        return compatibility_report
    
    def _analyze_dsl_features(self, data: Dict[str, Any]) -> List[str]:
        """分析 DSL 格式檔案的特徵"""
        features = []
        
        if 'variables' in data:
            var_count = len(data['variables'])
            features.append(f"變數系統: {var_count} 個變數")
        
        if 'steps' in data:
            step_count = len(data['steps'])
            features.append(f"執行步驟: {step_count} 個步驟")
            
            # 步驟類型統計
            step_types = {}
            for step in data['steps']:
                step_type = step.get('type', 'unknown')
                step_types[step_type] = step_types.get(step_type, 0) + 1
            
            if step_types:
                type_summary = ", ".join([f"{t}: {c}" for t, c in step_types.items()])
                features.append(f"步驟類型: {type_summary}")
        
        if 'control_structures' in data:
            features.append("高級控制結構")
        
        if 'node_type_mapping' in data:
            features.append("節點類型映射")
        
        if 'legacy_format' in data:
            features.append("向下相容性支援")
        
        metadata = data.get('metadata', {})
        if metadata.get('compatible_with'):
            compatible_systems = metadata['compatible_with']
            features.append(f"相容系統: {compatible_systems}")
        
        return features
    
    def _analyze_legacy_features(self, data: Dict[str, Any]) -> List[str]:
        """分析傳統格式檔案的特徵"""
        features = []
        
        if 'trigger_conditions' in data:
            condition_count = len(data['trigger_conditions'])
            features.append(f"觸發條件: {condition_count} 個")
        
        if 'action' in data:
            action = data['action']
            if action.get('function'):
                features.append(f"執行函數: {action['function']}")
            if action.get('model'):
                features.append(f"機器人型號: {action['model']}")
        
        if 'applicable_rooms' in data:
            room_count = len(data['applicable_rooms'])
            features.append(f"適用房間: {room_count} 個")
        
        return features
    
    def _generate_compatibility_recommendations(self, report: Dict[str, Any]) -> None:
        """根據相容性分析結果生成建議"""
        recommendations = []
        warnings = []
        
        # 轉換成功率分析
        total_files = report['total_files']
        if total_files > 0:
            success_rate = report['successful_conversions'] / total_files
            
            if success_rate < 0.8:
                warnings.append(f"檔案轉換成功率較低: {success_rate:.1%}")
                recommendations.append("建議檢查失敗的檔案並修正格式問題")
        
        # DSL 格式比例分析
        dsl_ratio = report['dsl_format_files'] / total_files if total_files > 0 else 0
        if dsl_ratio < 0.3:
            recommendations.append("考慮將更多傳統格式檔案轉換為 DSL 格式以獲得更好的可維護性")
        elif dsl_ratio > 0.7:
            recommendations.append("DSL 格式採用率良好，建議完善 DSL 功能和工具")
        
        # 失敗檔案分析
        failed_files = [f for f in report['file_analysis'] if not f['conversion_success']]
        if failed_files:
            warnings.append(f"有 {len(failed_files)} 個檔案轉換失敗")
            for file_info in failed_files:
                if file_info['issues']:
                    main_issue = file_info['issues'][0]
                    recommendations.append(f"修復檔案 '{file_info['file_name']}': {main_issue}")
        
        report['recommendations'] = recommendations
        report['warnings'] = warnings
    
    def flows_to_dict(self, flows: List[BusinessFlow]) -> List[Dict[str, Any]]:
        """將業務流程轉換為字典格式"""
        return [
            {
                'name': flow.name,
                'description': flow.description,
                'priority': flow.priority,
                'work_id': flow.work_id,
                'enabled': flow.enabled,
                'trigger_conditions': [
                    {
                        'condition': tc.condition,
                        'description': tc.description,
                        'parameters': tc.parameters
                    }
                    for tc in flow.trigger_conditions
                ],
                'action': {
                    'type': flow.action.type,
                    'task_type': flow.action.task_type,
                    'function': flow.action.function,
                    'model': flow.action.model,
                    'api': flow.action.api,
                    'mission_type': flow.action.mission_type,
                    'path': flow.action.path
                },
                'applicable_rooms': flow.applicable_rooms,
                'applicable_locations': flow.applicable_locations,
                'debug': flow.debug
            }
            for flow in flows
        ]


# 測試用的範例解析
if __name__ == "__main__":
    import tempfile
    import os
    
    # 建立測試 YAML 內容
    test_yaml = """
flows:
  rack_rotation_inlet:
    name: "Rack旋轉檢查-房間入口"
    description: "當 Rack A面完成後，檢查是否需要旋轉處理 B面"
    priority: 100
    work_id: "220001"
    enabled: true
    
    trigger_conditions:
      - condition: "rack_at_location_exists"
        description: "房間入口位置有 Rack"
        parameters:
          location_type: "room_inlet"
        
      - condition: "rack_side_completed" 
        description: "Rack A面已完成"
        parameters:
          side: "A"
    
    action:
      type: "create_task"
      task_type: "rack_rotation"
      function: "rack_move"
      model: "KUKA400i"
      api: "submit_mission"
      mission_type: "RACK_MOVE"
      
      path:
        type: "inlet_rotation"
        
    applicable_rooms: [1, 2, 3, 4, 5]
    
    debug:
      enabled: false
      log_conditions: true
"""
    
    # 建立暫時檔案進行測試
    with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
        f.write(test_yaml)
        temp_file = f.name
    
    try:
        parser = FlowParser()
        flows = parser.parse(temp_file)
        
        print(f"解析結果：{len(flows)} 個業務流程")
        for flow in flows:
            print(f"- {flow}")
            print(f"  觸發條件: {len(flow.trigger_conditions)} 個")
            print(f"  適用房間: {flow.applicable_rooms}")
            
        # 驗證結果
        validation = parser.validate_flows(flows)
        print(f"\n驗證結果:")
        for category, messages in validation.items():
            if messages:
                print(f"  {category}: {messages}")
                
    finally:
        os.unlink(temp_file)