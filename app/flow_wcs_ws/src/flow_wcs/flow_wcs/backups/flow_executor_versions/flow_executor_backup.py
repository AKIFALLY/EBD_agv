#!/usr/bin/env python3
"""
Flow Executor - Refactored with modular functions and systematic default value handling

This is a much smaller, cleaner version that:
1. Uses modular function organization
2. Implements systematic default value handling
3. Prevents circular variable overrides
4. Maintains all original functionality
"""

import re
import time
import logging
import copy
import json
import asyncio
from typing import Dict, Any, List, Optional, Union
from datetime import datetime

# Import modular functions
from .functions import (
    QueryFunctions,
    CheckFunctions,
    TaskFunctions,
    ActionFunctions,
    ControlFunctions
)

# Import database manager
from .database import db_manager

# Import decorator system
from .decorators import flow_function

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class FlowExecutor:
    """
    Linear Flow v2 Executor with modular functions and systematic default value handling.
    
    Key improvements:
    - Modular function organization for maintainability
    - Systematic default value handling to prevent issues
    - Protected variable scoping to prevent circular overrides
    - Cleaner, more maintainable codebase
    """
    
    def __init__(self, flow_config: Dict[str, Any] = None, context: Dict = None, ros_node=None, flow_data: Dict[str, Any] = None):
        """
        Initialize FlowExecutor with flow configuration.
        
        Args:
            flow_config: The flow configuration dictionary (new parameter name)
            context: Optional initial context (for compatibility)
            ros_node: Optional ROS 2 node for logging
            flow_data: Alias for flow_config (for backward compatibility)
        """
        # Handle both parameter names for backward compatibility
        if flow_data is not None and flow_config is None:
            flow_config = flow_data
        elif flow_config is None:
            raise ValueError("Either flow_config or flow_data must be provided")
            
        self.flow_config = flow_config
        self.ros_node = ros_node  # Store ROS node reference for logging
        self.flow_id = flow_config.get('flow', {}).get('id', 'unknown')
        self.flow_name = flow_config.get('flow', {}).get('name', 'Unnamed Flow')
        self.work_id = flow_config.get('flow', {}).get('work_id', None)
        
        # Initialize variable stack for proper scoping
        self.variable_stack = []
        
        # Initialize context with proper structure (use provided context if available)
        if context:
            self.context = context
            # Ensure required fields exist
            self.context.setdefault('variables', flow_config.get('variables', {}))
            self.context.setdefault('flow_stopped', False)
            self.context.setdefault('stop_reason', None)
            self.context.setdefault('current_section', None)
            self.context.setdefault('current_step', None)
            self.context.setdefault('execution_history', [])
            self.context.setdefault('error_count', 0)
        else:
            self.context = {
                'variables': flow_config.get('variables', {}),
                'flow_stopped': False,
                'stop_reason': None,
                'current_section': None,
                'current_step': None,
                'execution_history': [],
                'error_count': 0
            }
        
        # Initialize database manager
        self.db_manager = db_manager
        
        # Initialize function modules with reference to executor
        self.query_functions = QueryFunctions(self)
        self.check_functions = CheckFunctions(self)
        self.task_functions = TaskFunctions(self)
        self.action_functions = ActionFunctions(self)
        self.control_functions = ControlFunctions(self)
        
        # Map function categories to their handlers
        self.function_handlers = {
            'query': self.query_functions,
            'check': self.check_functions,
            'task': self.task_functions,
            'action': self.action_functions,
            'control': self.control_functions,
            'flow': self  # Flow functions are handled by executor itself
        }
        
        logger.info(f"FlowExecutor initialized for flow: {self.flow_name} (ID: {self.flow_id})")
    
    def execute(self) -> Dict[str, Any]:
        """
        Execute the complete flow.
        
        Returns:
            Execution result dictionary
        """
        start_time = time.time()
        logger.info(f"Starting flow execution: {self.flow_name}")
        
        try:
            # Execute workflow sections
            workflow = self.flow_config.get('workflow', [])
            
            for section in workflow:
                if self.context['flow_stopped']:
                    logger.info(f"Flow stopped: {self.context['stop_reason']}")
                    break
                
                self.execute_section(section)
            
            # Calculate execution time
            execution_time = time.time() - start_time
            
            # Prepare result
            result = {
                'success': not self.context['flow_stopped'] and self.context['error_count'] == 0,
                'flow_id': self.flow_id,
                'flow_name': self.flow_name,
                'execution_time': execution_time,
                'variables': self.context['variables'],
                'error_count': self.context['error_count'],
                'stopped': self.context['flow_stopped'],
                'stop_reason': self.context['stop_reason']
            }
            
            logger.info(f"Flow execution completed in {execution_time:.2f}s")
            return result
            
        except Exception as e:
            logger.error(f"Flow execution error: {e}", exc_info=True)
            self.context['error_count'] += 1
            
            return {
                'success': False,
                'flow_id': self.flow_id,
                'flow_name': self.flow_name,
                'error': str(e),
                'error_count': self.context['error_count']
            }
    
    async def execute_in_test_mode(self, dry_run: bool = False) -> Dict:
        """
        Execute flow in test mode with enhanced tracking.
        Compatible with Linear Flow Designer's test mode.
        
        Args:
            dry_run: If True, only simulate execution without actual database operations
            
        Returns:
            Detailed execution report with step tracking and variable snapshots
        """
        # Set test mode flags
        self.context['test_mode'] = True
        self.context['dry_run'] = dry_run
        
        # Initialize detailed tracking
        self.context['execution_trace'] = []
        self.context['variable_snapshots'] = []
        self.context['detailed_logs'] = []
        
        # Convert synchronous execute to async execution
        # Run in executor to avoid blocking
        loop = asyncio.get_event_loop()
        result = await loop.run_in_executor(None, self.execute)
        
        # Debug log the final trace before returning
        logger.info(f"[TRACE DEBUG] Final trace before returning to Linear Flow Designer:")
        for i, trace_item in enumerate(self.context.get('execution_trace', [])):
            logger.info(f"[TRACE DEBUG] [{i}] step_id: {trace_item.get('step_id')}, "
                       f"function: {trace_item.get('function')}, "
                       f"status: {trace_item.get('status')}, "
                       f"message: {trace_item.get('message', '')[:50]}")
        
        # Return detailed report compatible with Linear Flow Designer
        return {
            'status': 'completed' if result.get('success') else 'error',
            'success': result.get('success', False),
            'duration': result.get('execution_time', 0),
            'trace': self.context.get('execution_trace', []),
            'snapshots': self.context.get('variable_snapshots', []),
            'final_variables': self.context.get('variables', {}),
            'errors': [result.get('error')] if result.get('error') else [],
            'detailed_logs': self.context.get('detailed_logs', []),
            'logs': self.context.get('execution_history', []),
            'flow_id': self.flow_id,
            'flow_name': self.flow_name,
            'error_count': self.context.get('error_count', 0),
            'stopped': self.context.get('flow_stopped', False),
            'stop_reason': self.context.get('stop_reason')
        }
    
    def execute_section(self, section: Dict[str, Any]) -> None:
        """
        Execute a workflow section.
        
        Args:
            section: Section configuration
        """
        section_id = section.get('id', 'unknown')
        section_name = section.get('name', 'Unnamed Section')
        condition = section.get('condition')
        
        self.context['current_section'] = section_id
        
        # Check section condition
        if condition and not self.evaluate_condition(condition):
            logger.info(f"Skipping section {section_id}: condition not met")
            return
        
        logger.info(f"Executing section: {section_name} ({section_id})")
        
        # Handle different section types
        section_type = section.get('type', 'sequential')
        
        if section_type == 'foreach':
            self.execute_foreach_section(section)
        elif section_type == 'parallel':
            self.execute_parallel_section(section)
        elif section_type == 'conditional':
            self.execute_conditional_section(section)
        else:
            # Default sequential execution
            self.execute_sequential_section(section)
    
    def execute_sequential_section(self, section: Dict[str, Any]) -> None:
        """
        Execute steps sequentially.
        
        Args:
            section: Section configuration
        """
        steps = section.get('steps', [])
        
        for step in steps:
            if self.context['flow_stopped']:
                break
            
            self.execute_step(step)
    
    def execute_foreach_section(self, section: Dict[str, Any]) -> None:
        """
        Execute foreach loop section with proper variable scoping.
        
        Args:
            section: Section configuration
        """
        collection_expr = section.get('collection', [])
        var_name = section.get('var', 'item')
        steps = section.get('steps', [])
        
        # Resolve collection
        collection = self.resolve_value(collection_expr)
        if not isinstance(collection, (list, tuple)):
            logger.warning(f"Foreach collection is not iterable: {type(collection)}")
            return
        
        logger.info(f"Foreach section processing {len(collection)} items")
        
        for index, item in enumerate(collection):
            if self.context['flow_stopped']:
                break
            
            # Push current context to stack and create new scope
            self.push_variable_scope()
            
            # Set loop variables (compatible with old version)
            self.context['variables'][var_name] = item
            self.context['variables']['_item'] = item  # Also set _item for compatibility
            self.context['variables'][f'_{var_name}'] = item  # _${var_name} for template compatibility
            self.context['variables']['_index'] = index  # Add index for tracking
            self.context['variables']['loop_index'] = index
            
            # Log current item being processed
            if isinstance(item, dict) and 'id' in item:
                logger.info(f"Processing item {index+1}/{len(collection)}: ID={item.get('id')}")
            else:
                logger.info(f"Processing item {index+1}/{len(collection)}: {item}")
            
            # Execute steps
            for step in steps:
                if self.context['flow_stopped']:
                    break
                self.execute_step(step)
            
            # Pop scope to restore previous context
            self.pop_variable_scope()
    
    def execute_parallel_section(self, section: Dict[str, Any]) -> None:
        """
        Execute steps in parallel (currently sequential, can be enhanced).
        
        Args:
            section: Section configuration
        """
        # For now, execute sequentially
        # In future, could use threading or async
        self.execute_sequential_section(section)
    
    def execute_conditional_section(self, section: Dict[str, Any]) -> None:
        """
        Execute conditional section based on conditions.
        
        Args:
            section: Section configuration
        """
        cases = section.get('cases', [])
        
        for case in cases:
            condition = case.get('condition', 'true')
            
            if self.evaluate_condition(condition):
                steps = case.get('steps', [])
                for step in steps:
                    if self.context['flow_stopped']:
                        break
                    self.execute_step(step)
                break  # Only execute first matching case
    
    def execute_step(self, step: Dict[str, Any]) -> Any:
        """
        Execute a single step.
        
        Args:
            step: Step configuration
            
        Returns:
            Step execution result
        """
        step_id = step.get('id', 'unknown')
        self.context['current_step'] = step_id
        
        # Track step execution in test mode
        if self.context.get('test_mode'):
            # Determine the function name based on step type
            step_type = step.get('type', 'function')
            if step_type == 'foreach':
                function_name = 'foreach'
            elif step_type == 'if':
                function_name = 'if'
            elif step_type == 'switch':
                function_name = 'switch'
            else:
                function_name = step.get('exec', '')
            
            step_trace = {
                'step_id': step_id,
                'step': step.copy(),
                'timestamp': datetime.now().isoformat(),
                'variables_before': copy.deepcopy(self.context.get('variables', {})),
                'function': function_name,  # Set function field immediately
                'status': 'pending'  # Initial status
            }
            self.context.setdefault('execution_trace', []).append(step_trace)
        
        # Check skip condition
        skip_if = step.get('skip_if')
        if skip_if and self.evaluate_condition(skip_if):
            logger.info(f"Skipping step {step_id}: skip condition met")
            if self.context.get('test_mode'):
                # Update trace with skip info
                if self.context.get('execution_trace'):
                    self.context['execution_trace'][-1]['skipped'] = True
                    self.context['execution_trace'][-1]['skip_reason'] = 'condition'
                    # Don't overwrite function field, it's already set
                    self.context['execution_trace'][-1]['status'] = 'skipped'
                    self.context['execution_trace'][-1]['message'] = f"條件不符，跳過執行"
            return None
        
        # Execute based on step type
        step_type = step.get('type', 'function')
        
        # Handle different step types - check explicit types first
        if step_type == 'foreach':
            # Add debug logging for foreach execution
            logger.info(f"[FOREACH DEBUG] Starting foreach execution for step_id: {step_id}")
            logger.info(f"[FOREACH DEBUG] Test mode: {self.context.get('test_mode')}")
            logger.info(f"[FOREACH DEBUG] Trace length before: {len(self.context.get('execution_trace', []))}")
            
            # Execute the foreach step
            result = self.execute_foreach_step(step)
            
            logger.info(f"[FOREACH DEBUG] Foreach execution completed with result: {result}")
            logger.info(f"[FOREACH DEBUG] Trace length after: {len(self.context.get('execution_trace', []))}")
            
            # Update the trace entry for this foreach step
            if self.context.get('test_mode') and self.context.get('execution_trace'):
                logger.info(f"[FOREACH DEBUG] Looking for trace entry with step_id: {step_id}")
                
                # Find the trace entry for this foreach step by step_id
                foreach_trace = None
                foreach_index = -1
                for i, trace in enumerate(self.context['execution_trace']):
                    logger.debug(f"[FOREACH DEBUG] Checking trace[{i}] with step_id: {trace.get('step_id')}")
                    if trace.get('step_id') == step_id:
                        foreach_trace = trace
                        foreach_index = i
                        logger.info(f"[FOREACH DEBUG] Found foreach trace at index {i}")
                        break
                
                if foreach_trace:
                    logger.info(f"[FOREACH DEBUG] Current trace status: {foreach_trace.get('status')}")
                    
                    collection_expr = step.get('collection', '')
                    collection = self.resolve_value(collection_expr)
                    item_count = len(collection) if isinstance(collection, (list, tuple)) else 0
                    
                    logger.info(f"[FOREACH DEBUG] Updating trace: collection has {item_count} items")
                    
                    # Update the foreach trace entry
                    foreach_trace['status'] = 'completed'
                    foreach_trace['message'] = f"執行 {item_count} 次迴圈"
                    foreach_trace['result'] = result
                    foreach_trace['variables_after'] = copy.deepcopy(self.context.get('variables', {}))
                    
                    logger.info(f"[FOREACH DEBUG] Updated trace at index {foreach_index}:")
                    logger.info(f"[FOREACH DEBUG]   status: {foreach_trace['status']}")
                    logger.info(f"[FOREACH DEBUG]   message: {foreach_trace['message']}")
                    logger.info(f"[FOREACH DEBUG]   function: {foreach_trace.get('function')}")
                    
                    # Double-check the update worked
                    actual_trace = self.context['execution_trace'][foreach_index]
                    logger.info(f"[FOREACH DEBUG] Verification - actual trace status: {actual_trace.get('status')}")
                else:
                    logger.error(f"[FOREACH DEBUG] Could not find foreach trace entry for step_id: {step_id}")
                    logger.error(f"[FOREACH DEBUG] Available step_ids in trace: {[t.get('step_id') for t in self.context['execution_trace']]}")
            else:
                logger.warning(f"[FOREACH DEBUG] Not updating trace - test_mode: {self.context.get('test_mode')}, trace exists: {bool(self.context.get('execution_trace'))}")
            
            return result
        elif step_type == 'if':
            # Execute the if step
            result = self.execute_if_step(step)
            
            # Update the trace entry for this if step
            if self.context.get('test_mode') and self.context.get('execution_trace'):
                # Find the trace entry for this if step by step_id
                for trace in self.context['execution_trace']:
                    if trace.get('step_id') == step_id:
                        trace['status'] = 'completed'
                        trace['message'] = f"條件判斷執行"
                        trace['result'] = result
                        trace['variables_after'] = copy.deepcopy(self.context.get('variables', {}))
                        break
            return result
        elif step_type == 'switch':
            # Execute the switch step
            result = self.execute_switch_step(step)
            
            # Update the trace entry for this switch step
            if self.context.get('test_mode') and self.context.get('execution_trace'):
                # Find the trace entry for this switch step by step_id
                for trace in self.context['execution_trace']:
                    if trace.get('step_id') == step_id:
                        trace['status'] = 'completed'
                        trace['message'] = f"多重條件判斷執行"
                        trace['result'] = result
                        trace['variables_after'] = copy.deepcopy(self.context.get('variables', {}))
                        break
            return result
        elif step_type == 'function' or 'exec' in step:
            # Regular function execution
            return self.execute_function(step)
        else:
            logger.warning(f"Unknown step type: {step_type}")
            return None
    
    def execute_function(self, step: Dict[str, Any]) -> Any:
        """
        Execute a function step.
        
        Args:
            step: Step configuration with 'exec' and 'params'
            
        Returns:
            Function execution result
        """
        step_id = step.get('id', 'unknown')  # Get step_id from step
        function_name = step.get('exec', '')
        params = step.get('params', {})
        
        # Resolve parameters
        resolved_params = self.resolve_parameters(params)
        
        # Parse function name (e.g., "query.locations")
        parts = function_name.split('.')
        if len(parts) != 2:
            logger.error(f"Invalid function name format: {function_name}")
            return None
        
        category, method = parts
        
        # Get handler for category
        handler = self.function_handlers.get(category)
        if not handler:
            logger.error(f"Unknown function category: {category}")
            return None
        
        # Get method from handler
        if not hasattr(handler, method):
            logger.error(f"Function {method} not found in {category}")
            return None
        
        # Execute function
        try:
            func = getattr(handler, method)
            result = func(resolved_params)
            
            # Store result if variable name specified
            result_var = step.get('result')
            if result_var:
                self.context['variables'][result_var] = result
                logger.debug(f"[VARIABLE DEBUG] Stored variable '{result_var}' = {result}")
            
            # Log execution
            self.log_execution(step_id, function_name, resolved_params, result)
            
            # Track result in test mode
            if self.context.get('test_mode') and self.context.get('execution_trace'):
                self.context['execution_trace'][-1]['result'] = result
                self.context['execution_trace'][-1]['variables_after'] = copy.deepcopy(self.context.get('variables', {}))
                self.context['execution_trace'][-1]['success'] = True
                # Don't overwrite function field, it's already set
                self.context['execution_trace'][-1]['status'] = 'completed'
                # Add a descriptive message based on the function and result
                if category == 'query' and isinstance(result, list):
                    self.context['execution_trace'][-1]['message'] = f"查詢到 {len(result)} 筆資料"
                elif category == 'check' and isinstance(result, bool):
                    self.context['execution_trace'][-1]['message'] = f"檢查結果: {result}"
                elif category == 'task' and result:
                    self.context['execution_trace'][-1]['message'] = f"建立任務: {result}"
                elif category == 'action':
                    self.context['execution_trace'][-1]['message'] = f"執行動作完成"
                elif category == 'control':
                    self.context['execution_trace'][-1]['message'] = f"控制流程執行"
                else:
                    self.context['execution_trace'][-1]['message'] = str(result) if result else ""
            
            return result
            
        except Exception as e:
            logger.error(f"Error executing {function_name}: {e}", exc_info=True)
            self.context['error_count'] += 1
            
            # Track error in test mode
            if self.context.get('test_mode') and self.context.get('execution_trace'):
                self.context['execution_trace'][-1]['error'] = str(e)
                self.context['execution_trace'][-1]['success'] = False
                self.context['execution_trace'][-1]['variables_after'] = copy.deepcopy(self.context.get('variables', {}))
            
            return None
    
    def execute_foreach_step(self, step: Dict[str, Any]) -> List[Any]:
        """
        Execute a foreach step with proper scoping.
        
        Args:
            step: Foreach step configuration
            
        Returns:
            List of results from each iteration
        """
        collection_expr = step.get('collection', [])
        var_name = step.get('var', 'item')
        body = step.get('body', [])
        
        # Debug logging
        logger.debug(f"[FOREACH] collection_expr: {collection_expr}")
        logger.debug(f"[FOREACH] variables: {list(self.context['variables'].keys())}")
        
        # Resolve collection
        collection = self.resolve_value(collection_expr)
        logger.debug(f"[FOREACH] resolved collection: {collection}")
        
        if not isinstance(collection, (list, tuple)):
            logger.warning(f"[FOREACH] Collection is not a list/tuple: {type(collection)}")
            return []
        
        results = []
        
        # Track foreach details in test mode
        if self.context.get('test_mode'):
            logger.info(f"Foreach loop starting with {len(collection)} items")
            # Add detailed log for foreach start
            if self.context.get('detailed_logs') is not None:
                self.context['detailed_logs'].append({
                    'timestamp': datetime.now().isoformat(),
                    'level': 'info',
                    'message': f"開始 foreach 迴圈，共 {len(collection)} 個項目"
                })
        
        for index, item in enumerate(collection):
            if self.context['flow_stopped']:
                break
            
            # Push current context to stack and create new scope
            self.push_variable_scope()
            
            # Set loop variables (compatible with old version)
            self.context['variables'][var_name] = item
            self.context['variables']['_item'] = item  # Also set _item for compatibility
            # Also set _${var_name} for template compatibility (e.g., _room when var is "room")
            self.context['variables'][f'_{var_name}'] = item
            self.context['variables']['_index'] = index  # Add index for tracking
            self.context['variables']['loop_index'] = index
            
            # Log each iteration in test mode
            if self.context.get('test_mode'):
                iteration_info = f"迴圈第 {index + 1}/{len(collection)} 次"
                if isinstance(item, dict) and 'id' in item:
                    iteration_info += f" - 處理項目 ID: {item['id']}"
                logger.info(f"Foreach iteration {index + 1}/{len(collection)}")
                
                # Add detailed log for each iteration
                if self.context.get('detailed_logs') is not None:
                    self.context['detailed_logs'].append({
                        'timestamp': datetime.now().isoformat(),
                        'level': 'info',
                        'message': iteration_info
                    })
            
            # Execute body
            for body_step in body:
                result = self.execute_step(body_step)
                if result is not None:
                    results.append(result)
                    
                if self.context['flow_stopped']:
                    # Pop scope before breaking
                    self.pop_variable_scope()
                    break
            
            # Pop scope to restore previous context
            self.pop_variable_scope()
        
        # Log foreach completion in test mode
        if self.context.get('test_mode'):
            logger.info(f"Foreach loop completed, processed {len(results)} items with results")
            if self.context.get('detailed_logs') is not None:
                self.context['detailed_logs'].append({
                    'timestamp': datetime.now().isoformat(),
                    'level': 'info',
                    'message': f"完成 foreach 迴圈，共產生 {len(results)} 個結果"
                })
        
        return results
    
    def execute_if_step(self, step: Dict[str, Any]) -> Any:
        """
        Execute an if-then-else step.
        
        Args:
            step: If step configuration
            
        Returns:
            Result from executed branch
        """
        condition = step.get('condition', 'false')
        
        if self.evaluate_condition(condition):
            then_steps = step.get('then', [])
            for then_step in then_steps:
                result = self.execute_step(then_step)
                if result is not None:
                    return result
        else:
            else_steps = step.get('else', [])
            for else_step in else_steps:
                result = self.execute_step(else_step)
                if result is not None:
                    return result
        
        return None
    
    def execute_switch_step(self, step: Dict[str, Any]) -> Any:
        """
        Execute a switch-case step.
        
        Args:
            step: Switch step configuration
            
        Returns:
            Result from executed case
        """
        value_expr = step.get('value', '')
        cases = step.get('cases', {})
        default = step.get('default', [])
        
        # Resolve switch value
        value = self.resolve_value(value_expr)
        
        # Find matching case
        case_steps = cases.get(str(value), default)
        
        # Execute case steps
        for case_step in case_steps:
            result = self.execute_step(case_step)
            if result is not None:
                return result
        
        return None
    
    def evaluate_condition(self, condition: str) -> bool:
        """
        Evaluate a condition expression with full logical operator support.
        
        Args:
            condition: Condition expression
            
        Returns:
            Boolean evaluation result
        """
        if not condition or condition == 'true':
            return True
        if condition == 'false':
            return False
        
        # Handle logical operators
        if '||' in condition:
            # OR operator
            parts = condition.split('||')
            return any(self.evaluate_single_condition(p.strip()) for p in parts)
        
        if '&&' in condition:
            # AND operator
            parts = condition.split('&&')
            return all(self.evaluate_single_condition(p.strip()) for p in parts)
        
        if condition.startswith('!'):
            # NOT operator
            return not self.evaluate_single_condition(condition[1:].strip())
        
        return self.evaluate_single_condition(condition)
    
    def evaluate_single_condition(self, condition: str) -> bool:
        """
        Evaluate a single condition without logical operators.
        
        Args:
            condition: Single condition expression
            
        Returns:
            Boolean evaluation result
        """
        # Handle comparisons
        for op in ['==', '!=', '>=', '<=', '>', '<']:
            if op in condition:
                left, right = condition.split(op, 1)
                left_val = self.resolve_value(left.strip())
                right_val = self.resolve_value(right.strip())
                
                if op == '==':
                    return left_val == right_val
                elif op == '!=':
                    return left_val != right_val
                elif op == '>=':
                    return left_val >= right_val
                elif op == '<=':
                    return left_val <= right_val
                elif op == '>':
                    return left_val > right_val
                elif op == '<':
                    return left_val < right_val
        
        # Simple boolean check
        value = self.resolve_value(condition)
        return bool(value)
    
    def resolve_value(self, value: Any) -> Any:
        """
        Resolve a value that may contain variable references or expressions.
        
        Args:
            value: Value to resolve
            
        Returns:
            Resolved value
        """
        if value is None:
            return None
        
        if isinstance(value, str):
            # Debug logging
            logger.debug(f"[RESOLVE_VALUE] Input string: {value}")
            
            # Check for variable reference
            if value.startswith('${') and value.endswith('}'):
                expr = value[2:-1]
                logger.debug(f"[RESOLVE_VALUE] Resolving expression: {expr}")
                result = self.resolve_expression(expr)
                logger.debug(f"[RESOLVE_VALUE] Expression result: {result}")
                return result
            
            # Check for inline variables
            if '${' in value:
                result = self.resolve_template(value)
                logger.debug(f"[RESOLVE_VALUE] Template result: {result}")
                return result
        
        elif isinstance(value, dict):
            # Recursively resolve dictionary values
            return {k: self.resolve_value(v) for k, v in value.items()}
        
        elif isinstance(value, list):
            # Recursively resolve list items
            return [self.resolve_value(item) for item in value]
        
        return value
    
    def resolve_expression(self, expr: str) -> Any:
        """
        Resolve an expression with variable references and math operations.
        
        Args:
            expr: Expression to resolve
            
        Returns:
            Resolved value
        """
        # Handle mathematical operations
        if any(op in expr for op in ['+', '-', '*', '/']):
            return self.resolve_math_expression(expr)
        
        # Handle property access (e.g., location.id)
        if '.' in expr:
            parts = expr.split('.')
            value = self.context['variables'].get(parts[0])
            
            for part in parts[1:]:
                if value is None:
                    break
                if isinstance(value, dict):
                    value = value.get(part)
                elif hasattr(value, part):
                    value = getattr(value, part)
                else:
                    value = None
            
            return value
        
        # Simple variable lookup
        value = self.context['variables'].get(expr)
        logger.debug(f"[RESOLVE DEBUG] Resolving '{expr}' from variables: {list(self.context['variables'].keys())} = {value}")
        return value
    
    def resolve_math_expression(self, expr: str) -> Any:
        """
        Resolve mathematical expressions safely.
        
        Args:
            expr: Mathematical expression
            
        Returns:
            Calculated result
        """
        # Split by operators while preserving them
        parts = re.split(r'([+\-*/])', expr)
        resolved_parts = []
        
        for part in parts:
            part = part.strip()
            if part in ['+', '-', '*', '/']:
                resolved_parts.append(part)
            else:
                # Resolve variable reference
                if part.startswith('${') and part.endswith('}'):
                    value = self.resolve_expression(part[2:-1])
                elif '.' in part:
                    value = self.resolve_expression(part)
                else:
                    value = self.context['variables'].get(part, part)
                
                resolved_parts.append(str(value))
        
        # Safely evaluate the expression
        try:
            expression = ''.join(resolved_parts)
            result = eval(expression, {"__builtins__": {}}, {})
            return result
        except Exception as e:
            logger.warning(f"Failed to evaluate math expression {expr}: {e}")
            return None
    
    def resolve_template(self, template: str) -> str:
        """
        Resolve a template string with variable placeholders.
        
        Args:
            template: Template string
            
        Returns:
            Resolved string
        """
        def replacer(match):
            expr = match.group(1)
            value = self.resolve_expression(expr)
            return str(value) if value is not None else ''
        
        return re.sub(r'\$\{([^}]+)\}', replacer, template)
    
    def resolve_parameters(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """
        Resolve all parameters in a dictionary.
        
        Args:
            params: Parameters dictionary
            
        Returns:
            Resolved parameters
        """
        return {key: self.resolve_value(value) for key, value in params.items()}
    
    def log_execution(self, step_id: str, function: str, params: Dict, result: Any) -> None:
        """
        Log step execution to database.
        
        Args:
            step_id: Step identifier
            function: Function name
            params: Parameters used
            result: Execution result
        """
        # Skip database operations in dry_run mode
        if self.context.get('dry_run', False):
            logger.debug(f"[DRY RUN] Would log execution: {function} with params {params}")
            return
            
        try:
            self.db_manager.log_flow_execution(
                flow_id=self.flow_id,
                flow_name=self.flow_name,
                work_id=self.work_id,
                section=self.context.get('current_section', 'unknown'),
                step_id=step_id,
                function=function,
                params=params,
                result=result,
                status='success' if result is not None else 'failed'
            )
        except Exception as e:
            logger.warning(f"Failed to log execution: {e}")
    
    # Flow control functions (handled directly by executor)
    
    def set_variable(self, params: Dict) -> Any:
        """
        Set a variable in the context.
        
        Args:
            params: Parameters with 'name' and 'value'
            
        Returns:
            The set value
        """
        name = params.get('name')
        value = params.get('value')
        
        if name:
            resolved_value = self.resolve_value(value)
            self.context['variables'][name] = resolved_value
            logger.info(f"Set variable {name} = {resolved_value}")
            return resolved_value
        
        return None
    
    def get_variable(self, params: Dict) -> Any:
        """
        Get a variable from the context.
        
        Args:
            params: Parameters with 'name'
            
        Returns:
            The variable value
        """
        name = params.get('name')
        default = params.get('default')
        
        if name:
            value = self.context['variables'].get(name, default)
            return value
        
        return default
    
    def push_variable_scope(self):
        """Push current variables to stack and create new scope with inheritance"""
        # Create a deep copy of current variables
        import copy
        current_vars = copy.deepcopy(self.context['variables'])
        self.variable_stack.append(current_vars)
        logger.debug(f"Pushed variable scope, stack depth: {len(self.variable_stack)}")
    
    def pop_variable_scope(self):
        """Pop variable scope from stack and restore previous scope"""
        if self.variable_stack:
            # Restore previous scope
            self.context['variables'] = self.variable_stack.pop()
            logger.debug(f"Popped variable scope, stack depth: {len(self.variable_stack)}")
        else:
            logger.warning("Tried to pop empty variable stack")


# Direct function exports for compatibility
def execute_flow(flow_config: Dict[str, Any], context: Dict = None, ros_node=None) -> Dict[str, Any]:
    """
    Execute a flow configuration.
    
    Args:
        flow_config: Flow configuration dictionary
        context: Optional initial context
        ros_node: Optional ROS 2 node for logging
        
    Returns:
        Execution result
    """
    executor = FlowExecutor(flow_config, context, ros_node)
    return executor.execute()
