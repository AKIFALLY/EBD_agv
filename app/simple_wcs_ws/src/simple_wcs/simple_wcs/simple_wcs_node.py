#!/usr/bin/env python3
"""
Simple WCS Node - 基於統一 YAML 格式的並行流程執行引擎
根據 FLOW_FORMAT_STANDARD.yaml 規範執行所有流程
"""

import os
import asyncio
import yaml
import logging
from pathlib import Path
from typing import Dict, List, Any, Optional
from dataclasses import dataclass, field

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


@dataclass
class FlowNode:
    """流程節點定義"""
    id: str
    type: str  # condition, action, logic
    name: str
    function: str
    parameters: Dict[str, Any] = field(default_factory=dict)
    inputs: Dict[str, Dict[str, Any]] = field(default_factory=dict)
    outputs: Dict[str, Dict[str, Any]] = field(default_factory=dict)
    
    # 執行狀態
    is_ready: bool = False
    has_executed: bool = False
    result: Any = None


@dataclass
class FlowConnection:
    """節點連接定義"""
    from_node: str
    from_output: str
    to_node: str
    to_input: str


@dataclass 
class Flow:
    """完整流程定義"""
    name: str
    description: str
    enabled: bool
    priority: int
    work_id: str
    nodes: List[FlowNode]
    connections: List[FlowConnection]
    
    # 執行狀態
    is_executing: bool = False
    last_execution_time: float = 0.0


class SimpleWCSNode(Node):
    """Simple WCS ROS 2 Node - 並行執行所有流程"""
    
    def __init__(self):
        super().__init__('simple_wcs_node')
        
        # 設定日誌
        self.logger = self.get_logger()
        self.logger.info("🚀 Simple WCS Node 啟動中...")
        
        # 載入配置
        self.flows_dir = Path('/app/config/wcs/flows')
        self.flows: Dict[str, Flow] = {}
        
        # 載入所有流程
        self.load_all_flows()
        
        # 建立定時器 - 每5秒執行一次所有流程
        self.execution_timer = self.create_timer(5.0, self.execute_all_flows)
        
        # ROS 2 發布者
        self.task_publisher = self.create_publisher(String, '/simple_wcs/tasks', 10)
        self.status_publisher = self.create_publisher(String, '/simple_wcs/status', 10)
        
        # 執行器池 (用於並行執行)
        self.executor_tasks = []
        
        self.logger.info(f"✅ Simple WCS Node 啟動完成，載入了 {len(self.flows)} 個流程")
    
    def load_all_flows(self):
        """載入所有 YAML 流程檔案"""
        if not self.flows_dir.exists():
            self.logger.error(f"Flows 目錄不存在: {self.flows_dir}")
            return
        
        # 載入所有 .yaml 檔案
        yaml_files = list(self.flows_dir.glob('*.yaml'))
        self.logger.info(f"發現 {len(yaml_files)} 個流程檔案")
        
        for yaml_file in yaml_files:
            try:
                flow = self.load_flow_file(yaml_file)
                if flow and flow.enabled:
                    self.flows[flow.name] = flow
                    self.logger.info(f"✅ 載入流程: {flow.name} (優先級: {flow.priority})")
            except Exception as e:
                self.logger.error(f"載入流程失敗 {yaml_file.name}: {e}")
    
    def load_flow_file(self, file_path: Path) -> Optional[Flow]:
        """載入單個流程檔案"""
        with open(file_path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)
        
        # 只讀取業務邏輯部分 (忽略 flow_designer_data)
        nodes = []
        for node_data in data.get('nodes', []):
            node = FlowNode(
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
            # 解析 from 和 to 格式: "node_id.output_name"
            from_parts = conn_data['from'].split('.')
            to_parts = conn_data['to'].split('.')
            
            connection = FlowConnection(
                from_node=from_parts[0],
                from_output=from_parts[1] if len(from_parts) > 1 else 'output',
                to_node=to_parts[0],
                to_input=to_parts[1] if len(to_parts) > 1 else 'input'
            )
            connections.append(connection)
        
        # 建立流程物件
        flow = Flow(
            name=data.get('name', file_path.stem),
            description=data.get('description', ''),
            enabled=data.get('enabled', True),
            priority=data.get('priority', 50),
            work_id=data.get('work_id', '000000'),
            nodes=nodes,
            connections=connections
        )
        
        return flow
    
    def execute_all_flows(self):
        """並行執行所有流程 (定時器回調)"""
        self.logger.info("🔄 開始執行所有流程...")
        
        # 發布系統狀態
        self.publish_status()
        
        # 使用 asyncio 並行執行所有流程
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        
        try:
            # 建立所有流程的執行任務
            tasks = []
            for flow_name, flow in self.flows.items():
                if not flow.is_executing:  # 避免重複執行
                    task = loop.create_task(self.execute_flow_async(flow))
                    tasks.append(task)
            
            # 並行執行所有任務
            if tasks:
                loop.run_until_complete(asyncio.gather(*tasks))
                self.logger.info(f"✅ 完成執行 {len(tasks)} 個流程")
            else:
                self.logger.debug("💤 沒有流程需要執行")
                
        except Exception as e:
            self.logger.error(f"執行流程時發生錯誤: {e}")
        finally:
            loop.close()
    
    async def execute_flow_async(self, flow: Flow):
        """異步執行單個流程"""
        flow.is_executing = True
        self.logger.info(f"▶️ 開始執行流程: {flow.name}")
        
        try:
            # 重置所有節點狀態
            for node in flow.nodes:
                node.has_executed = False
                node.result = None
                node.is_ready = False
            
            # 標記沒有輸入的節點為就緒
            for node in flow.nodes:
                if not self.node_has_inputs(node, flow):
                    node.is_ready = True
            
            # 執行直到所有節點完成
            iterations = 0
            max_iterations = 100  # 防止無限循環
            
            while iterations < max_iterations:
                iterations += 1
                
                # 找出所有就緒但未執行的節點
                ready_nodes = [n for n in flow.nodes if n.is_ready and not n.has_executed]
                
                if not ready_nodes:
                    # 檢查是否所有節點都已執行
                    unexecuted = [n for n in flow.nodes if not n.has_executed]
                    if unexecuted:
                        self.logger.debug(f"流程 {flow.name} 有 {len(unexecuted)} 個節點無法執行")
                    break
                
                # 並行執行所有就緒節點
                tasks = []
                for node in ready_nodes:
                    task = self.execute_node_async(node, flow)
                    tasks.append(task)
                
                await asyncio.gather(*tasks)
                
                # 更新下游節點的就緒狀態
                for node in ready_nodes:
                    self.update_downstream_nodes(node, flow)
            
            self.logger.info(f"✅ 流程 {flow.name} 執行完成 (迭代: {iterations})")
            
        except Exception as e:
            self.logger.error(f"執行流程 {flow.name} 失敗: {e}")
        finally:
            flow.is_executing = False
            flow.last_execution_time = self.get_clock().now().seconds_nanoseconds()[0]
    
    async def execute_node_async(self, node: FlowNode, flow: Flow):
        """異步執行單個節點"""
        self.logger.debug(f"  執行節點: {node.name} ({node.type})")
        
        try:
            # 根據節點類型執行不同邏輯
            if node.type == 'condition':
                result = await self.execute_condition_node(node, flow)
            elif node.type == 'action':
                result = await self.execute_action_node(node, flow)
            elif node.type == 'logic':
                result = await self.execute_logic_node(node, flow)
            else:
                self.logger.warning(f"未知節點類型: {node.type}")
                result = None
            
            node.result = result
            node.has_executed = True
            
            if result:
                self.logger.debug(f"    ✓ 節點 {node.name} 執行成功")
            else:
                self.logger.debug(f"    ✗ 節點 {node.name} 條件不滿足")
                
        except Exception as e:
            self.logger.error(f"執行節點 {node.name} 失敗: {e}")
            node.has_executed = True
            node.result = None
    
    async def execute_condition_node(self, node: FlowNode, flow: Flow) -> bool:
        """執行條件節點"""
        # 這裡應該調用實際的條件檢查函數
        # 目前只是模擬
        function_name = node.function
        parameters = node.parameters
        
        self.logger.debug(f"    檢查條件: {function_name} 參數: {parameters}")
        
        # 模擬條件檢查
        # 實際實現時應該調用對應的函數
        import random
        result = random.random() > 0.7  # 30% 機率滿足條件
        
        return result
    
    async def execute_action_node(self, node: FlowNode, flow: Flow) -> Any:
        """執行動作節點"""
        # 檢查所有輸入是否滿足
        inputs_satisfied = self.check_node_inputs(node, flow)
        
        if not inputs_satisfied:
            self.logger.debug(f"    動作節點 {node.name} 輸入條件不滿足")
            return None
        
        function_name = node.function
        parameters = node.parameters
        
        self.logger.info(f"    🎯 執行動作: {function_name} 參數: {parameters}")
        
        # 發布任務到 ROS topic
        task_msg = String()
        task_msg.data = f"Task: {function_name} | WorkID: {flow.work_id} | Priority: {flow.priority}"
        self.task_publisher.publish(task_msg)
        
        return {'task_created': True, 'function': function_name}
    
    async def execute_logic_node(self, node: FlowNode, flow: Flow) -> Any:
        """執行邏輯節點"""
        function_name = node.function
        parameters = node.parameters
        
        self.logger.debug(f"    處理邏輯: {function_name} 參數: {parameters}")
        
        # 模擬邏輯處理
        return {'logic_result': True}
    
    def node_has_inputs(self, node: FlowNode, flow: Flow) -> bool:
        """檢查節點是否有輸入連接"""
        for connection in flow.connections:
            if connection.to_node == node.id:
                return True
        return False
    
    def check_node_inputs(self, node: FlowNode, flow: Flow) -> bool:
        """檢查節點的所有輸入是否滿足"""
        for connection in flow.connections:
            if connection.to_node == node.id:
                # 找到上游節點
                upstream_node = next((n for n in flow.nodes if n.id == connection.from_node), None)
                if upstream_node:
                    # 檢查上游節點是否已執行且結果為真
                    if not upstream_node.has_executed or not upstream_node.result:
                        return False
        return True
    
    def update_downstream_nodes(self, node: FlowNode, flow: Flow):
        """更新下游節點的就緒狀態"""
        for connection in flow.connections:
            if connection.from_node == node.id:
                # 找到下游節點
                downstream_node = next((n for n in flow.nodes if n.id == connection.to_node), None)
                if downstream_node and not downstream_node.has_executed:
                    # 檢查下游節點的所有輸入是否就緒
                    if self.check_node_inputs(downstream_node, flow):
                        downstream_node.is_ready = True
    
    def publish_status(self):
        """發布系統狀態"""
        active_flows = sum(1 for f in self.flows.values() if f.is_executing)
        total_flows = len(self.flows)
        
        status_msg = String()
        status_msg.data = f"Simple WCS | 流程: {total_flows} | 執行中: {active_flows}"
        self.status_publisher.publish(status_msg)


def main(args=None):
    """主函數"""
    rclpy.init(args=args)
    
    try:
        node = SimpleWCSNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n Simple WCS Node 正在關閉...")
    except Exception as e:
        print(f"Simple WCS Node 錯誤: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()