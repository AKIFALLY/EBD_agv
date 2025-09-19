#!/usr/bin/env python3
"""
Control flow functions for Flow WCS with systematic default value handling
"""

from typing import Dict, Any, List, Optional
import time
from .base import FlowFunctionBase
from ..decorators import flow_function


class ControlFunctions(FlowFunctionBase):
    """Control flow functions"""
    
    @flow_function("control", "等待指定時間", ["seconds"], "boolean",
                   defaults={"seconds": 1})
    def wait(self, params: Dict) -> bool:
        """
        Wait for specified time with systematic default value handling
        """
        # Extract parameter with default
        seconds = self.get_param(params, 'seconds', default=1)
        
        # Type casting with safety
        seconds = self.safe_cast(seconds, float, default=1.0)
        
        # Ensure reasonable wait time (max 60 seconds)
        if seconds < 0:
            seconds = 0
        elif seconds > 60:
            self.logger.warning(f"Wait time {seconds}s exceeds max 60s, capping at 60s")
            seconds = 60
        
        self.logger.info(f"Waiting for {seconds} seconds")
        time.sleep(seconds)
        
        self.log_execution('control.wait', params, True)
        return True
    
    @flow_function("control", "停止流程執行", ["reason"], "boolean",
                   defaults={"reason": "Flow stopped by control function"})
    def stop_flow(self, params: Dict) -> bool:
        """
        Stop flow execution with systematic default value handling
        """
        # Extract parameter with default
        reason = self.get_param(params, 'reason', default='Flow stopped by control function')
        
        self.logger.info(f"Stopping flow: {reason}")
        
        # Set stop flag in executor context
        if hasattr(self.executor, 'context'):
            self.executor.context['flow_stopped'] = True
            self.executor.context['stop_reason'] = reason
        
        self.log_execution('control.stop_flow', params, True)
        return True
    
    @flow_function("control", "計算項目數量", ["variable"], "number",
                   defaults={"variable": []})
    def count_items(self, params: Dict) -> int:
        """
        Count items in a variable with systematic default value handling
        """
        # Extract parameter
        variable = self.get_param(params, 'variable', default=[])
        
        # Count based on type
        if isinstance(variable, (list, tuple)):
            count = len(variable)
        elif isinstance(variable, dict):
            count = len(variable)
        elif isinstance(variable, str):
            count = len(variable)
        elif variable is None:
            count = 0
        else:
            count = 1  # Single item
        
        self.logger.info(f"Counted {count} items")
        
        self.log_execution('control.count_items', params, count)
        return count
    
    @flow_function("control", "Switch case 控制", ["value", "cases", "default"], "any",
                   defaults={"value": None, "cases": {}, "default": None})
    def switch_case(self, params: Dict) -> Any:
        """
        Switch case control with systematic default value handling
        """
        # Extract parameters
        value = self.get_param(params, 'value', default=None)
        cases = self.get_param(params, 'cases', default={})
        default = self.get_param(params, 'default', default=None)
        
        # Ensure cases is a dict
        cases = self.safe_cast(cases, dict, default={})
        
        self.logger.info(f"Switch case for value: {value}")
        
        # Find matching case
        result = cases.get(str(value), default)
        
        self.log_execution('control.switch_case', params, result)
        return result
    
    @flow_function("control", "迴圈遍歷", ["items", "var", "steps", "max_iterations"], "array",
                   defaults={"items": [], "var": "item", "steps": [], "max_iterations": 1000})
    def foreach(self, params: Dict) -> List[Any]:
        """
        For-each loop with systematic default value handling and proper variable scoping
        """
        # Extract parameters
        items = self.get_param(params, 'items', default=[])
        var_name = self.get_param(params, 'var', default='item')
        steps = self.get_param(params, 'steps', default=[])
        max_iterations = self.get_param(params, 'max_iterations', default=1000)
        
        # Type validation
        items = self.safe_cast(items, list, default=[])
        steps = self.safe_cast(steps, list, default=[])
        max_iterations = self.safe_cast(max_iterations, int, default=1000)
        
        self.logger.info(f"For-each loop over {len(items)} items")
        
        results = []
        
        # Save current context to restore later (proper scoping)
        saved_context = None
        if hasattr(self.executor, 'context') and 'variables' in self.executor.context:
            saved_context = self.executor.context['variables'].copy()
        
        try:
            for i, item in enumerate(items):
                if i >= max_iterations:
                    self.logger.warning(f"Reached max iterations {max_iterations}")
                    break
                
                # Set loop variable in context
                if hasattr(self.executor, 'context'):
                    if 'variables' not in self.executor.context:
                        self.executor.context['variables'] = {}
                    self.executor.context['variables'][var_name] = item
                    self.executor.context['variables']['loop_index'] = i
                
                # Execute steps for this item
                step_results = []
                for step in steps:
                    # In real implementation, would execute the step
                    # For now, just collect the step definition
                    step_results.append({
                        'item': item,
                        'index': i,
                        'step': step
                    })
                
                results.extend(step_results)
        
        finally:
            # Restore original context (proper scoping)
            if saved_context is not None and hasattr(self.executor, 'context'):
                self.executor.context['variables'] = saved_context
        
        self.log_execution('control.foreach', params, f"{len(results)} results")
        return results
    
    @flow_function("control", "更新變數值", ["variable", "operation", "value"], "any",
                   defaults={"variable": "", "operation": "set", "value": None})
    def update_variable(self, params: Dict) -> Any:
        """
        Update variable value with systematic default value handling
        """
        # Extract parameters
        variable = self.get_param(params, 'variable', default='')
        operation = self.get_param(params, 'operation', default='set')
        value = self.get_param(params, 'value', default=None)
        
        if not variable:
            self.logger.error("Variable name is required for update")
            return None
        
        self.logger.info(f"Updating variable {variable} with operation {operation}")
        
        # Get current value from context
        current_value = None
        if hasattr(self.executor, 'context') and 'variables' in self.executor.context:
            current_value = self.executor.context['variables'].get(variable, None)
        
        # Apply operation
        if operation == 'set':
            new_value = value
        elif operation == 'append' and isinstance(current_value, list):
            new_value = current_value + [value]
        elif operation == 'prepend' and isinstance(current_value, list):
            new_value = [value] + current_value
        elif operation == 'increment' and isinstance(current_value, (int, float)):
            increment = self.safe_cast(value, float, default=1)
            new_value = current_value + increment
        elif operation == 'decrement' and isinstance(current_value, (int, float)):
            decrement = self.safe_cast(value, float, default=1)
            new_value = current_value - decrement
        elif operation == 'multiply' and isinstance(current_value, (int, float)):
            multiplier = self.safe_cast(value, float, default=1)
            new_value = current_value * multiplier
        elif operation == 'divide' and isinstance(current_value, (int, float)):
            divisor = self.safe_cast(value, float, default=1)
            if divisor != 0:
                new_value = current_value / divisor
            else:
                self.logger.error("Division by zero")
                new_value = current_value
        else:
            new_value = value
        
        # Update context
        if hasattr(self.executor, 'context'):
            if 'variables' not in self.executor.context:
                self.executor.context['variables'] = {}
            self.executor.context['variables'][variable] = new_value
        
        self.log_execution('control.update_variable', params, new_value)
        return new_value
    
    @flow_function("control", "條件分支控制", ["value", "cases"], "any",
                   defaults={"value": None, "cases": []})
    def if_else(self, params: Dict) -> Any:
        """
        If-else branching control with systematic default value handling
        """
        # Extract parameters
        value = self.get_param(params, 'value', default=None)
        cases = self.get_param(params, 'cases', default=[])
        
        # Ensure cases is a list
        cases = self.safe_cast(cases, list, default=[])
        
        self.logger.info(f"If-else branching based on value: {value}")
        
        # Evaluate conditions
        for case in cases:
            if not isinstance(case, dict):
                continue
            
            condition = case.get('condition', 'true')
            then_value = case.get('then', None)
            else_value = case.get('else', None)
            
            # Evaluate condition
            if condition == 'true' or (condition == value) or \
               (condition == 'truthy' and bool(value)) or \
               (condition == 'falsy' and not bool(value)):
                result = then_value
            else:
                result = else_value
            
            if result is not None:
                self.log_execution('control.if_else', params, result)
                return result
        
        # No matching condition
        self.log_execution('control.if_else', params, None)
        return None
