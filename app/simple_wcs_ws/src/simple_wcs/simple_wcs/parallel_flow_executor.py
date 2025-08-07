"""
Parallel Flow Executor - 並行流程執行器
負責管理和執行多個流程的並行處理
"""

import asyncio
import logging
from typing import Dict, List, Any, Optional, Callable
from dataclasses import dataclass, field
from datetime import datetime
import time


@dataclass
class ExecutionContext:
    """執行上下文"""
    flow_name: str
    start_time: float
    variables: Dict[str, Any] = field(default_factory=dict)
    results: Dict[str, Any] = field(default_factory=dict)
    errors: List[str] = field(default_factory=list)
    
    @property
    def duration(self) -> float:
        return time.time() - self.start_time
    
    def set_variable(self, name: str, value: Any):
        self.variables[name] = value
    
    def get_variable(self, name: str, default=None):
        return self.variables.get(name, default)
    
    def add_result(self, node_id: str, result: Any):
        self.results[node_id] = result
    
    def add_error(self, error: str):
        self.errors.append(f"[{datetime.now().strftime('%H:%M:%S')}] {error}")


class ParallelFlowExecutor:
    """並行流程執行器"""
    
    def __init__(self):
        self.logger = logging.getLogger('simple_wcs.parallel_executor')
        self.function_registry: Dict[str, Callable] = {}
        self.execution_contexts: Dict[str, ExecutionContext] = {}
        self.running_flows: set = set()
        
    def register_function(self, name: str, function: Callable):
        """註冊可執行函數"""
        self.function_registry[name] = function
        self.logger.debug(f"註冊函數: {name}")
    
    def register_builtin_functions(self):
        """註冊內建函數"""
        # 條件檢查函數
        self.register_function('check_rack_side_status', self._check_rack_side_status)
        self.register_function('check_rack_has_carrier', self._check_rack_has_carrier)
        self.register_function('check_rack_has_space', self._check_rack_has_space)
        self.register_function('check_task_not_exists', self._check_task_not_exists)
        
        # 邏輯處理函數
        self.register_function('and_gate', self._and_gate)
        self.register_function('or_gate', self._or_gate)
        self.register_function('not_gate', self._not_gate)
        
        # 動作執行函數
        self.register_function('create_rotation_task', self._create_rotation_task)
        self.register_function('update_rack_status', self._update_rack_status)
        
        self.logger.info(f"註冊了 {len(self.function_registry)} 個內建函數")
    
    async def execute_flows_parallel(self, flows: List[Any]) -> Dict[str, Any]:
        """並行執行多個流程"""
        self.logger.info(f"開始並行執行 {len(flows)} 個流程")
        
        # 建立執行任務
        tasks = []
        for flow in flows:
            if flow.name not in self.running_flows:
                self.running_flows.add(flow.name)
                context = ExecutionContext(
                    flow_name=flow.name,
                    start_time=time.time()
                )
                self.execution_contexts[flow.name] = context
                
                task = asyncio.create_task(self.execute_single_flow(flow, context))
                tasks.append(task)
            else:
                self.logger.warning(f"流程 {flow.name} 已在執行中，跳過")
        
        # 等待所有任務完成
        if tasks:
            results = await asyncio.gather(*tasks, return_exceptions=True)
            
            # 處理結果
            execution_summary = {
                'total_flows': len(flows),
                'executed': len(tasks),
                'successful': 0,
                'failed': 0,
                'results': {}
            }
            
            for i, result in enumerate(results):
                flow_name = flows[i].name
                if isinstance(result, Exception):
                    execution_summary['failed'] += 1
                    execution_summary['results'][flow_name] = {
                        'status': 'error',
                        'error': str(result)
                    }
                    self.logger.error(f"流程 {flow_name} 執行失敗: {result}")
                else:
                    execution_summary['successful'] += 1
                    execution_summary['results'][flow_name] = result
                
                # 清理執行狀態
                self.running_flows.discard(flow_name)
            
            return execution_summary
        else:
            return {
                'total_flows': len(flows),
                'executed': 0,
                'successful': 0,
                'failed': 0,
                'results': {}
            }
    
    async def execute_single_flow(self, flow: Any, context: ExecutionContext) -> Dict[str, Any]:
        """執行單個流程"""
        self.logger.debug(f"執行流程: {flow.name}")
        
        try:
            # 初始化流程變數
            context.set_variable('flow_name', flow.name)
            context.set_variable('work_id', flow.work_id)
            context.set_variable('priority', flow.priority)
            
            # 執行節點 (資料流驅動)
            executed_nodes = set()
            max_iterations = 100
            iteration = 0
            
            while iteration < max_iterations:
                iteration += 1
                progress = False
                
                for node in flow.nodes:
                    if node.id not in executed_nodes:
                        # 檢查節點是否準備就緒
                        if await self.is_node_ready(node, flow, context):
                            # 執行節點
                            result = await self.execute_node(node, context)
                            context.add_result(node.id, result)
                            executed_nodes.add(node.id)
                            progress = True
                            
                            # 如果是條件節點且結果為假，可能需要停止某些路徑
                            if node.type == 'condition' and not result:
                                self.logger.debug(f"條件節點 {node.name} 不滿足，相關路徑停止")
                
                # 如果沒有進展，結束執行
                if not progress:
                    break
            
            # 返回執行結果
            return {
                'status': 'completed',
                'flow_name': flow.name,
                'duration': context.duration,
                'executed_nodes': len(executed_nodes),
                'total_nodes': len(flow.nodes),
                'results': context.results,
                'errors': context.errors
            }
            
        except Exception as e:
            self.logger.error(f"執行流程 {flow.name} 時發生錯誤: {e}")
            context.add_error(str(e))
            return {
                'status': 'error',
                'flow_name': flow.name,
                'duration': context.duration,
                'error': str(e),
                'errors': context.errors
            }
    
    async def is_node_ready(self, node: Any, flow: Any, context: ExecutionContext) -> bool:
        """檢查節點是否準備就緒（所有輸入都已就緒）"""
        # 找出所有指向此節點的連接
        incoming_connections = [c for c in flow.connections if c.to_node == node.id]
        
        # 如果沒有輸入連接，節點立即就緒
        if not incoming_connections:
            return True
        
        # 檢查所有輸入是否已有值
        for connection in incoming_connections:
            # 檢查上游節點是否已執行
            upstream_result = context.results.get(connection.from_node)
            if upstream_result is None:
                return False
            
            # 對於條件節點，如果結果為假，下游節點可能不執行
            if isinstance(upstream_result, bool) and not upstream_result:
                # 這裡可以加入更複雜的邏輯判斷
                pass
        
        return True
    
    async def execute_node(self, node: Any, context: ExecutionContext) -> Any:
        """執行單個節點"""
        self.logger.debug(f"  執行節點: {node.name} ({node.type})")
        
        # 查找對應的函數
        function = self.function_registry.get(node.function)
        
        if function:
            # 準備參數
            kwargs = dict(node.parameters) if node.parameters else {}
            
            # 對於邏輯節點，需要從 context 中獲取輸入值
            if node.type == 'logic' and hasattr(node, 'inputs'):
                # 處理邏輯節點的輸入
                if node.function == 'and_gate':
                    # AND 閘需要多個輸入
                    input_values = []
                    for input_node_id in node.inputs:
                        value = context.results.get(input_node_id)
                        if value is not None:
                            input_values.append(value)
                    
                    # 如果有足夠的輸入，設置參數
                    if len(input_values) >= 2:
                        kwargs['input1'] = input_values[0]
                        kwargs['input2'] = input_values[1]
                        # 如果有第三個輸入，可以擴展為三輸入 AND
                        if len(input_values) >= 3:
                            # 先計算前兩個的 AND，再與第三個 AND
                            temp_result = input_values[0] and input_values[1]
                            kwargs['input1'] = temp_result
                            kwargs['input2'] = input_values[2]
                    
                elif node.function == 'or_gate':
                    # OR 閘需要多個輸入
                    input_values = []
                    for input_node_id in node.inputs:
                        value = context.results.get(input_node_id)
                        if value is not None:
                            input_values.append(value)
                    
                    if len(input_values) >= 2:
                        kwargs['input1'] = input_values[0]
                        kwargs['input2'] = input_values[1]
                
                elif node.function == 'not_gate':
                    # NOT 閘只需要一個輸入
                    if node.inputs and len(node.inputs) > 0:
                        input_node_id = node.inputs[0]
                        value = context.results.get(input_node_id)
                        if value is not None:
                            kwargs['input'] = value
            
            kwargs['context'] = context
            
            # 執行函數
            try:
                result = await self._call_function(function, **kwargs)
                self.logger.debug(f"    節點 {node.name} 執行結果: {result}")
                return result
            except Exception as e:
                self.logger.error(f"    節點 {node.name} 執行失敗: {e}")
                context.add_error(f"節點 {node.name}: {e}")
                return None
        else:
            self.logger.warning(f"    函數未註冊: {node.function}")
            return None
    
    async def _call_function(self, function: Callable, **kwargs) -> Any:
        """調用函數（支援同步和異步）"""
        if asyncio.iscoroutinefunction(function):
            return await function(**kwargs)
        else:
            return function(**kwargs)
    
    # === 內建函數實現 ===
    
    def _check_rack_side_status(self, side: str, check_type: str, context: ExecutionContext) -> bool:
        """檢查料架側面狀態"""
        # 模擬檢查邏輯
        import random
        result = random.random() > 0.5
        self.logger.debug(f"檢查料架 {side} 面 {check_type}: {result}")
        return result
    
    def _check_rack_has_carrier(self, side: str, context: ExecutionContext) -> bool:
        """檢查料架是否有載具"""
        # 模擬檢查邏輯
        import random
        result = random.random() > 0.5
        self.logger.debug(f"檢查料架 {side} 面是否有載具: {result}")
        return result
    
    def _check_rack_has_space(self, side: str, context: ExecutionContext) -> bool:
        """檢查料架是否有空間"""
        # 模擬檢查邏輯
        import random
        result = random.random() > 0.5
        self.logger.debug(f"檢查料架 {side} 面是否有空間: {result}")
        return result
    
    def _check_task_not_exists(self, task_type: str, location_id: int, context: ExecutionContext) -> bool:
        """檢查任務是否不存在"""
        # 模擬檢查邏輯
        return True
    
    def _and_gate(self, input1: bool, input2: bool, context: ExecutionContext) -> bool:
        """AND 邏輯閘"""
        return bool(input1 and input2)
    
    def _or_gate(self, input1: bool, input2: bool, context: ExecutionContext) -> bool:
        """OR 邏輯閘"""
        return bool(input1 or input2)
    
    def _not_gate(self, input: bool, context: ExecutionContext) -> bool:
        """NOT 邏輯閘"""
        return not bool(input)
    
    async def _create_rotation_task(self, location_id: int, context: ExecutionContext) -> Dict[str, Any]:
        """建立旋轉任務"""
        self.logger.info(f"建立旋轉任務 - 位置: {location_id}, 工作ID: {context.get_variable('work_id')}")
        
        task = {
            'type': 'rotation',
            'location_id': location_id,
            'work_id': context.get_variable('work_id'),
            'priority': context.get_variable('priority'),
            'created_at': datetime.now().isoformat()
        }
        
        return task
    
    async def _update_rack_status(self, location_id: int, status: str, context: ExecutionContext) -> bool:
        """更新料架狀態"""
        self.logger.info(f"更新料架狀態 - 位置: {location_id}, 狀態: {status}")
        return True