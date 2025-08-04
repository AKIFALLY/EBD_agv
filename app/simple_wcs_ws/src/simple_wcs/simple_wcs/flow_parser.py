"""
Simple WCS Flow Parser
解析 YAML 格式的業務流程配置檔案 - 支援多檔案結構
"""

import logging
import yaml
import os
import glob
from dataclasses import dataclass
from typing import List, Dict, Any, Optional
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
    """業務流程解析器 - 多檔案 YAML 格式 (flows/ 目錄)"""
    
    def __init__(self, flows_dir: str = None):
        self.flows_dir = flows_dir  # flows 目錄路徑
        self.logger = logging.getLogger('simple_wcs.flow_parser')
    
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
        """解析單個流程檔案的資料結構"""
        try:
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
                applicable_locations=data.get('applicable_locations', []),  # 新增適用位置
                debug=data.get('debug', {})
            )
            
            return flow
            
        except Exception as e:
            self.logger.error(f"解析單個流程檔案失敗: {e}")
            return None
    
    def validate_flows(self, flows: List[BusinessFlow]) -> Dict[str, List[str]]:
        """驗證業務流程配置的完整性"""
        validation_results = {
            'errors': [],
            'warnings': [],
            'info': []
        }
        
        if not flows:
            validation_results['errors'].append("未找到任何業務流程")
            return validation_results
        
        # 檢查重複的優先級
        priorities = [flow.priority for flow in flows]
        duplicate_priorities = set([p for p in priorities if priorities.count(p) > 1])
        if duplicate_priorities:
            validation_results['warnings'].append(f"發現重複的優先級: {duplicate_priorities}")
        
        # 檢查每個流程的完整性
        for flow in flows:
            if not flow.trigger_conditions:
                validation_results['warnings'].append(f"業務流程 '{flow.name}' 沒有觸發條件")
            
            if not flow.action.type:
                validation_results['warnings'].append(f"業務流程 '{flow.name}' 沒有定義動作")
        
        validation_results['info'].append(f"總共解析了 {len(flows)} 個業務流程")
        
        return validation_results
    
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