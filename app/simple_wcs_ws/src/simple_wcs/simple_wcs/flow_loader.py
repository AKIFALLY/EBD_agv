"""
Flow Loader - 流程載入器
只載入符合 FLOW_FORMAT_STANDARD.yaml 規範的 YAML 檔案
"""

import yaml
import logging
from pathlib import Path
from typing import Dict, List, Any, Optional
from dataclasses import dataclass, field


@dataclass
class NodeDefinition:
    """節點定義"""
    id: str
    type: str
    name: str
    function: str
    parameters: Dict[str, Any] = field(default_factory=dict)
    inputs: Dict[str, Dict[str, Any]] = field(default_factory=dict)
    outputs: Dict[str, Dict[str, Any]] = field(default_factory=dict)


@dataclass
class ConnectionDefinition:
    """連接定義"""
    from_node: str
    from_output: str
    to_node: str
    to_input: str
    
    @classmethod
    def from_string(cls, from_str: str, to_str: str):
        """從字串格式建立連接"""
        from_parts = from_str.split('.')
        to_parts = to_str.split('.')
        
        return cls(
            from_node=from_parts[0],
            from_output=from_parts[1] if len(from_parts) > 1 else 'output',
            to_node=to_parts[0],
            to_input=to_parts[1] if len(to_parts) > 1 else 'input'
        )


@dataclass
class FlowDefinition:
    """流程定義"""
    name: str
    description: str
    version: str
    author: str
    priority: int
    work_id: str
    enabled: bool
    nodes: List[NodeDefinition]
    connections: List[ConnectionDefinition]
    metadata: Dict[str, Any] = field(default_factory=dict)
    
    # 統計資訊
    node_count: int = 0
    connection_count: int = 0
    condition_nodes: int = 0
    action_nodes: int = 0
    logic_nodes: int = 0
    
    def __post_init__(self):
        """計算統計資訊"""
        self.node_count = len(self.nodes)
        self.connection_count = len(self.connections)
        
        for node in self.nodes:
            if node.type == 'condition':
                self.condition_nodes += 1
            elif node.type == 'action':
                self.action_nodes += 1
            elif node.type == 'logic':
                self.logic_nodes += 1


class FlowLoader:
    """流程載入器"""
    
    def __init__(self, flows_dir: str = '/app/config/wcs/flows'):
        self.flows_dir = Path(flows_dir)
        self.logger = logging.getLogger('simple_wcs.flow_loader')
        self.flows: Dict[str, FlowDefinition] = {}
        
    def load_all_flows(self) -> Dict[str, FlowDefinition]:
        """載入所有流程"""
        if not self.flows_dir.exists():
            self.logger.error(f"流程目錄不存在: {self.flows_dir}")
            return {}
        
        self.flows.clear()
        yaml_files = list(self.flows_dir.glob('*.yaml'))
        
        self.logger.info(f"掃描流程目錄: {self.flows_dir}")
        self.logger.info(f"發現 {len(yaml_files)} 個 YAML 檔案")
        
        for yaml_file in yaml_files:
            try:
                flow = self.load_flow_file(yaml_file)
                if flow:
                    self.flows[flow.name] = flow
                    self.logger.info(f"✅ 載入流程: {flow.name}")
                    self._log_flow_info(flow)
            except Exception as e:
                self.logger.error(f"❌ 載入流程失敗 {yaml_file.name}: {e}")
        
        self._log_summary()
        return self.flows
    
    def load_flow_file(self, file_path: Path) -> Optional[FlowDefinition]:
        """載入單個流程檔案"""
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
            
            # 驗證必要欄位
            if not self._validate_flow_format(data):
                self.logger.warning(f"檔案 {file_path.name} 格式不符合標準")
                return None
            
            # 解析節點
            nodes = []
            for node_data in data.get('nodes', []):
                node = NodeDefinition(
                    id=node_data['id'],
                    type=node_data['type'],
                    name=node_data.get('name', node_data['id']),
                    function=node_data.get('function', ''),
                    parameters=node_data.get('parameters', {}),
                    inputs=node_data.get('inputs', {}),
                    outputs=node_data.get('outputs', {})
                )
                nodes.append(node)
            
            # 解析連接
            connections = []
            for conn_data in data.get('connections', []):
                connection = ConnectionDefinition.from_string(
                    conn_data['from'],
                    conn_data['to']
                )
                connections.append(connection)
            
            # 建立流程定義
            flow = FlowDefinition(
                name=data.get('name', file_path.stem),
                description=data.get('description', ''),
                version=data.get('version', '1.0'),
                author=data.get('author', 'Unknown'),
                priority=data.get('priority', 50),
                work_id=data.get('work_id', '000000'),
                enabled=data.get('enabled', True),
                nodes=nodes,
                connections=connections,
                metadata=data.get('metadata', {})
            )
            
            return flow
            
        except Exception as e:
            self.logger.error(f"解析流程檔案 {file_path.name} 失敗: {e}")
            return None
    
    def _validate_flow_format(self, data: Dict[str, Any]) -> bool:
        """驗證流程格式是否符合標準"""
        # 必要欄位
        required_fields = ['name', 'nodes']
        
        for field in required_fields:
            if field not in data:
                self.logger.warning(f"缺少必要欄位: {field}")
                return False
        
        # 檢查節點格式
        nodes = data.get('nodes', [])
        if not isinstance(nodes, list):
            self.logger.warning("nodes 必須是列表")
            return False
        
        for node in nodes:
            if not isinstance(node, dict):
                self.logger.warning("節點必須是字典")
                return False
            
            if 'id' not in node or 'type' not in node:
                self.logger.warning("節點缺少 id 或 type")
                return False
            
            if node['type'] not in ['condition', 'action', 'logic']:
                self.logger.warning(f"未知節點類型: {node['type']}")
                return False
        
        return True
    
    def _log_flow_info(self, flow: FlowDefinition):
        """記錄流程資訊"""
        self.logger.debug(f"  - 優先級: {flow.priority}")
        self.logger.debug(f"  - 工作ID: {flow.work_id}")
        self.logger.debug(f"  - 節點數: {flow.node_count} (條件: {flow.condition_nodes}, 動作: {flow.action_nodes}, 邏輯: {flow.logic_nodes})")
        self.logger.debug(f"  - 連接數: {flow.connection_count}")
        self.logger.debug(f"  - 啟用: {flow.enabled}")
    
    def _log_summary(self):
        """記錄載入摘要"""
        total_flows = len(self.flows)
        enabled_flows = sum(1 for f in self.flows.values() if f.enabled)
        
        total_nodes = sum(f.node_count for f in self.flows.values())
        total_connections = sum(f.connection_count for f in self.flows.values())
        
        self.logger.info("=" * 50)
        self.logger.info(f"流程載入完成:")
        self.logger.info(f"  總流程數: {total_flows}")
        self.logger.info(f"  啟用流程: {enabled_flows}")
        self.logger.info(f"  總節點數: {total_nodes}")
        self.logger.info(f"  總連接數: {total_connections}")
        
        if self.flows:
            # 按優先級排序顯示
            sorted_flows = sorted(self.flows.values(), key=lambda f: f.priority, reverse=True)
            self.logger.info("  流程優先級:")
            for flow in sorted_flows[:5]:  # 只顯示前5個
                status = "✓" if flow.enabled else "✗"
                self.logger.info(f"    [{status}] {flow.name}: {flow.priority}")
    
    def get_enabled_flows(self) -> List[FlowDefinition]:
        """取得所有啟用的流程"""
        return [f for f in self.flows.values() if f.enabled]
    
    def get_flow_by_name(self, name: str) -> Optional[FlowDefinition]:
        """根據名稱取得流程"""
        return self.flows.get(name)
    
    def get_flows_by_priority(self, min_priority: int = 0) -> List[FlowDefinition]:
        """取得高於指定優先級的流程"""
        return [f for f in self.flows.values() if f.priority >= min_priority]
    
    def reload_flows(self) -> Dict[str, FlowDefinition]:
        """重新載入所有流程"""
        self.logger.info("重新載入流程...")
        return self.load_all_flows()


# 測試
if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    
    loader = FlowLoader()
    flows = loader.load_all_flows()
    
    print(f"\n載入了 {len(flows)} 個流程")
    
    for name, flow in flows.items():
        print(f"\n流程: {name}")
        print(f"  描述: {flow.description}")
        print(f"  節點: {flow.node_count}")
        print(f"  連接: {flow.connection_count}")