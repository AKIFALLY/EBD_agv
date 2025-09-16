#!/usr/bin/env python3
"""
TAFL Executor Wrapper - Enhanced with error handling, stack trace, and rollback
"""

import sys
import os
import traceback
import json
from typing import Dict, Any, Optional, List, Tuple
import asyncio
from datetime import datetime
from dataclasses import dataclass, field
from enum import Enum

# Add TAFL parser to path
sys.path.insert(0, '/app/tafl_ws/src/tafl')

from tafl.parser import TAFLParser
from tafl.executor import TAFLExecutor
from tafl.validator import TAFLValidator

# For testing, try to import from current directory first
try:
    from tafl_db_bridge import TAFLDatabaseBridge
    from tafl_functions import TAFLFunctions
except ImportError:
    # If that fails, try relative import (for production)
    from .tafl_db_bridge import TAFLDatabaseBridge
    from .tafl_functions import TAFLFunctions


class ExecutionState(Enum):
    """Execution state enumeration"""
    PENDING = "pending"
    RUNNING = "running"
    COMPLETED = "completed"
    FAILED = "failed"
    ROLLED_BACK = "rolled_back"


@dataclass
class ExecutionStep:
    """Single execution step record"""
    step_id: str
    verb: str
    params: Dict[str, Any]
    start_time: datetime
    end_time: Optional[datetime] = None
    status: ExecutionState = ExecutionState.PENDING
    result: Optional[Any] = None
    error: Optional[str] = None
    stack_trace: Optional[str] = None
    rollback_action: Optional[str] = None


@dataclass
class ExecutionContext:
    """Enhanced execution context with tracking"""
    flow_id: str
    started_at: datetime
    variables: Dict[str, Any] = field(default_factory=dict)
    steps: List[ExecutionStep] = field(default_factory=list)
    current_step: Optional[ExecutionStep] = None
    rollback_queue: List[Tuple[str, Dict]] = field(default_factory=list)
    metadata: Dict[str, Any] = field(default_factory=dict)
    
    def add_step(self, step: ExecutionStep):
        """Add execution step to history"""
        self.steps.append(step)
        self.current_step = step
    
    def get_execution_stack(self) -> List[str]:
        """Get current execution stack"""
        stack = []
        for step in self.steps:
            status_emoji = {
                ExecutionState.COMPLETED: "✅",
                ExecutionState.FAILED: "❌",
                ExecutionState.RUNNING: "🔄",
                ExecutionState.PENDING: "⏸️",
                ExecutionState.ROLLED_BACK: "↩️"
            }.get(step.status, "❓")
            
            stack.append(f"{status_emoji} {step.step_id}: {step.verb} - {step.status.value}")
        return stack


class TAFLExecutorWrapper:
    """Enhanced TAFL executor wrapper with error handling"""
    
    def __init__(self, database_url: str = None, logger=None):
        """Initialize TAFL executor wrapper
        
        Args:
            database_url: Database connection string
            logger: Logger instance
        """
        self.logger = logger
        self.parser = TAFLParser()
        self.validator = TAFLValidator()
        
        # Initialize database bridge and functions
        self.db_bridge = TAFLDatabaseBridge(database_url)
        self.functions = TAFLFunctions(self.db_bridge)
        
        # Track execution state
        self.current_context: Optional[ExecutionContext] = None
        self.execution_history: List[ExecutionContext] = []
        self.max_history_size = 100
        
        # Error handling configuration
        self.enable_rollback = True
        self.enable_stack_trace = True
        self.max_retry_attempts = 3
        
        # Performance metrics
        self.metrics = {
            'total_executions': 0,
            'successful_executions': 0,
            'failed_executions': 0,
            'rolled_back_executions': 0,
            'average_execution_time': 0,
            'total_execution_time': 0
        }
    
    def execute_flow_sync(self, flow_content: str, flow_id: str = None) -> Dict[str, Any]:
        """Execute a TAFL flow synchronously (like RCS dispatch)
        
        Args:
            flow_content: TAFL flow content (YAML string)
            flow_id: Optional flow identifier
        
        Returns:
            Execution result dictionary
        """
        # Create a new event loop for this execution
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        
        try:
            # Run the async execution in this new loop
            result = loop.run_until_complete(
                self.execute_flow(flow_content, flow_id)
            )
            return result
        finally:
            # Clean up the loop
            loop.close()
    
    async def execute_flow(self, flow_content: str, flow_id: str = None) -> Dict[str, Any]:
        """Execute a TAFL flow with enhanced error handling following v1.1 4-Phase model
        
        Args:
            flow_content: TAFL flow content (YAML string)
            flow_id: Optional flow identifier
        
        Returns:
            Execution result dictionary with detailed information
        """
        flow_id = flow_id or f"flow_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        start_time = datetime.now()
        
        # Initialize execution context with 5-Level scope support
        self.current_context = ExecutionContext(
            flow_id=flow_id,
            started_at=start_time
        )
        
        # Initialize 5-Level variable scopes
        self.scopes = {
            'rules': {},      # Level 1: Rules Scope (read-only)
            'preload': {},    # Level 2: Preload Scope (cached data)
            'global': {},     # Level 3: Global Scope
            'flow': {},       # Level 4: Flow Scope
            'loop': {}        # Level 5: Loop Scope (local)
        }
        
        try:
            # Parse TAFL flow content as YAML first for 4-Phase execution
            self._log_info(f"Parsing TAFL flow: {flow_id}")
            import yaml
            flow_data = yaml.safe_load(flow_content)
            
            # Phase 1: Settings (formerly initialization)
            self._log_info("Phase 1: Processing settings")
            await self._execute_settings(flow_data.get('settings', {}))
            
            # Phase 2: Preload - Skip here, let TAFLExecutor handle it
            # The TAFLExecutor will execute preload with proper context
            self._log_info("Phase 2: Preload will be handled by TAFLExecutor")
            
            # Phase 3: Rules - Rule definitions (read-only)
            self._log_info("Phase 3: Processing rules")
            self._process_rules(flow_data.get('rules', {}))
            
            # Phase 4: Variables - Variable initialization
            self._log_info("Phase 4: Processing variables")
            self._process_variables(flow_data.get('variables', {}))
            
            # Parse for TAFL executor (for flow execution)
            ast = self._parse_with_error_handling(flow_content)
            
            # Validate TAFL flow
            validation_result = self._validate_with_error_handling(ast)
            if not validation_result['valid']:
                return self._create_error_response(
                    'Validation failed',
                    validation_result['errors'],
                    flow_id
                )
            
            # Create executor with enhanced function registry
            executor = TAFLExecutor(function_registry=self._create_enhanced_registry())
            
            # Prepare initial context
            initial_context = {
                'flow_id': flow_id,
                'functions': self._create_enhanced_registry(),
                'execution_context': self.current_context
            }
            
            # Execute flow with monitoring
            self._log_info(f"Executing TAFL flow: {flow_id}")
            result = await self._execute_with_monitoring(executor, ast, initial_context)
            
            # Calculate metrics
            execution_time = (datetime.now() - start_time).total_seconds()
            self._update_metrics(True, execution_time)
            
            # Add to history
            self._add_to_history(self.current_context)
            
            return self._create_success_response(result, flow_id, execution_time)
            
        except Exception as e:
            # Handle execution error
            error_msg = str(e)
            stack_trace = traceback.format_exc() if self.enable_stack_trace else None
            
            self._log_error(f"Execution failed: {error_msg}")
            if stack_trace:
                self._log_error(f"Stack trace:\n{stack_trace}")
            
            # Attempt rollback if enabled
            if self.enable_rollback:
                rollback_result = await self._perform_rollback()
                if rollback_result['success']:
                    self._log_info("Rollback completed successfully")
                else:
                    self._log_error(f"Rollback failed: {rollback_result['error']}")
            
            # Calculate metrics
            execution_time = (datetime.now() - start_time).total_seconds()
            self._update_metrics(False, execution_time)
            
            # Add to history
            self._add_to_history(self.current_context)
            
            return self._create_error_response(error_msg, stack_trace, flow_id)
    
    def _parse_with_error_handling(self, flow_content: str) -> Any:
        """Parse TAFL with enhanced error handling"""
        try:
            return self.parser.parse_string(flow_content)
        except Exception as e:
            error_msg = f"Parse error: {str(e)}"
            self._log_error(error_msg)
            
            # Add parsing error to context
            if self.current_context:
                self.current_context.metadata['parse_error'] = {
                    'error': str(e),
                    'content_preview': flow_content[:200] if len(flow_content) > 200 else flow_content
                }
            
            raise ValueError(error_msg)
    
    def _validate_with_error_handling(self, ast: Any) -> Dict[str, Any]:
        """Validate TAFL with enhanced error handling"""
        try:
            is_valid = self.validator.validate(ast)
            errors = self.validator.get_errors() if not is_valid else []
            
            # Add validation info to context
            if self.current_context:
                self.current_context.metadata['validation'] = {
                    'valid': is_valid,
                    'errors': errors,
                    'timestamp': datetime.now().isoformat()
                }
            
            return {'valid': is_valid, 'errors': errors}
            
        except Exception as e:
            self._log_error(f"Validation error: {str(e)}")
            return {'valid': False, 'errors': [str(e)]}
    
    async def _execute_settings(self, settings: Dict[str, Any]):
        """Phase 1: Execute settings configuration
        
        Args:
            settings: Settings dictionary from TAFL flow
        """
        if not settings:
            return
            
        # Apply settings to execution context
        if self.current_context:
            self.current_context.metadata['settings'] = settings
        
        # Apply specific settings
        if 'timeout' in settings:
            self.current_context.metadata['timeout'] = settings['timeout']
        if 'max_retries' in settings:
            self.max_retry_attempts = settings.get('max_retries', 3)
        if 'log_level' in settings:
            self.current_context.metadata['log_level'] = settings['log_level']
    
    async def _execute_preload(self, preload: Dict[str, Any]):
        """Phase 2: Execute preload queries and cache data
        
        Args:
            preload: Preload dictionary with data queries
        """
        if not preload:
            return
            
        for key, query_def in preload.items():
            if isinstance(query_def, dict) and 'query' in query_def:
                # Execute query and store in preload scope
                query_params = query_def['query']
                try:
                    # Execute query using database bridge
                    if 'target' in query_params:
                        target = query_params['target']
                        where = query_params.get('where', {})
                        
                        # Call appropriate query method
                        if target == 'racks':
                            result = await self.db_bridge.query_racks(where)
                        elif target == 'agvs':
                            result = await self.db_bridge.query_agvs(where)
                        elif target == 'tasks':
                            result = await self.db_bridge.query_tasks(where)
                        elif target == 'locations':
                            result = await self.db_bridge.query_locations(where)
                        elif target == 'works':
                            result = await self.db_bridge.query_works(where)
                        else:
                            result = None
                            self._log_warning(f"Unknown preload target: {target}. Valid targets: racks, agvs, tasks, locations, works")
                        
                        # Store in preload scope
                        if result is not None:
                            self.scopes['preload'][key] = result
                            self._log_info(f"Preloaded {key}: {len(result) if isinstance(result, list) else 1} items")
                
                except Exception as e:
                    self._log_error(f"Preload failed for {key}: {str(e)}")
    
    def _process_rules(self, rules: Dict[str, Any]):
        """Phase 3: Process rule definitions (read-only)
        
        Args:
            rules: Rules dictionary from TAFL flow
        """
        if not rules:
            return
            
        # Store rules in read-only scope
        for rule_name, rule_def in rules.items():
            if isinstance(rule_def, dict):
                # Store complete rule definition
                self.scopes['rules'][rule_name] = {
                    'value': rule_def.get('value'),
                    'condition': rule_def.get('condition'),
                    'description': rule_def.get('description'),
                    'read_only': True  # Mark as read-only
                }
            else:
                # Simple value rule
                self.scopes['rules'][rule_name] = {
                    'value': rule_def,
                    'read_only': True
                }
            
            self._log_info(f"Registered rule: {rule_name}")
    
    def _process_variables(self, variables: Dict[str, Any]):
        """Phase 4: Process variable initialization
        
        Args:
            variables: Variables dictionary from TAFL flow
        """
        if not variables:
            return
            
        # Initialize variables in global scope
        for var_name, var_value in variables.items():
            self.scopes['global'][var_name] = var_value
            self._log_info(f"Initialized variable: {var_name} = {var_value}")
        
        # Store in execution context
        if self.current_context:
            self.current_context.variables = self.scopes['global'].copy()
    
    def _resolve_variable(self, var_ref: str):
        """Resolve variable from 5-Level scope hierarchy
        
        Args:
            var_ref: Variable reference (e.g., ${variable_name})
        
        Returns:
            Variable value or None if not found
        """
        # Remove ${ and } if present
        var_name = var_ref.strip('${}')
        
        # Search in scope hierarchy (from most specific to least)
        scope_order = ['loop', 'flow', 'global', 'preload', 'rules']
        
        for scope_name in scope_order:
            if var_name in self.scopes[scope_name]:
                value = self.scopes[scope_name][var_name]
                # For rules, return the value field
                if scope_name == 'rules' and isinstance(value, dict):
                    return value.get('value')
                return value
        
        return None
    
    async def _execute_with_monitoring(self, executor: TAFLExecutor, ast: Any, 
                                      initial_context: Dict) -> Any:
        """Execute with step-by-step monitoring"""
        # Track each step execution
        step_count = 0
        
        # Override executor methods to add monitoring
        original_execute_verb = executor.execute_verb if hasattr(executor, 'execute_verb') else None
        
        async def monitored_execute_verb(verb: str, params: Dict, context: Dict):
            """Monitored verb execution"""
            nonlocal step_count
            step_count += 1
            step_id = f"step_{step_count}"
            
            # Create step record
            step = ExecutionStep(
                step_id=step_id,
                verb=verb,
                params=params,
                start_time=datetime.now(),
                status=ExecutionState.RUNNING
            )
            
            if self.current_context:
                self.current_context.add_step(step)
            
            try:
                # Log step execution
                self._log_info(f"Executing step {step_id}: {verb}")
                
                # Execute original verb
                if original_execute_verb:
                    result = await original_execute_verb(verb, params, context)
                else:
                    result = await self._execute_verb_with_retry(verb, params, context)
                
                # Update step record
                step.end_time = datetime.now()
                step.status = ExecutionState.COMPLETED
                step.result = result
                
                # Add rollback action if applicable
                if self.enable_rollback:
                    rollback_action = self._get_rollback_action(verb, params, result)
                    if rollback_action:
                        step.rollback_action = rollback_action
                        if self.current_context:
                            self.current_context.rollback_queue.append((verb, rollback_action))
                
                return result
                
            except Exception as e:
                # Update step record with error
                step.end_time = datetime.now()
                step.status = ExecutionState.FAILED
                step.error = str(e)
                step.stack_trace = traceback.format_exc() if self.enable_stack_trace else None
                
                # Log error with context
                self._log_error(f"Step {step_id} failed: {str(e)}")
                if self.enable_stack_trace:
                    self._log_execution_stack()
                
                raise
        
        # Replace executor method
        if hasattr(executor, 'execute_verb'):
            executor.execute_verb = monitored_execute_verb
        
        # Execute with monitoring
        return await executor.execute(ast, initial_context)
    
    async def _execute_verb_with_retry(self, verb: str, params: Dict, context: Dict) -> Any:
        """Execute verb with retry logic"""
        last_error = None
        
        for attempt in range(self.max_retry_attempts):
            try:
                # Get function from registry
                func = self._create_enhanced_registry().get(verb)
                if not func:
                    raise ValueError(f"Unknown verb: {verb}")
                
                # Execute function
                result = await func(**params) if asyncio.iscoroutinefunction(func) else func(**params)
                
                # Success - return result
                return result
                
            except Exception as e:
                last_error = e
                self._log_warning(f"Attempt {attempt + 1}/{self.max_retry_attempts} failed: {str(e)}")
                
                # Wait before retry (exponential backoff)
                if attempt < self.max_retry_attempts - 1:
                    await asyncio.sleep(2 ** attempt)
        
        # All attempts failed
        raise last_error
    
    async def _perform_rollback(self) -> Dict[str, Any]:
        """Perform rollback of executed steps"""
        if not self.current_context:
            return {'success': False, 'error': 'No execution context'}
        
        self._log_info(f"Starting rollback for {len(self.current_context.rollback_queue)} actions")
        
        rolled_back = []
        failed_rollbacks = []
        
        # Execute rollback actions in reverse order
        for verb, rollback_action in reversed(self.current_context.rollback_queue):
            try:
                self._log_info(f"Rolling back: {verb}")
                
                # Execute rollback action
                if isinstance(rollback_action, dict):
                    # Rollback action is a function call
                    func_name = rollback_action.get('function')
                    params = rollback_action.get('params', {})
                    
                    func = self._create_enhanced_registry().get(func_name)
                    if func:
                        await func(**params) if asyncio.iscoroutinefunction(func) else func(**params)
                        rolled_back.append(verb)
                    else:
                        failed_rollbacks.append(f"{verb}: Unknown rollback function {func_name}")
                else:
                    failed_rollbacks.append(f"{verb}: Invalid rollback action")
                    
            except Exception as e:
                failed_rollbacks.append(f"{verb}: {str(e)}")
                self._log_error(f"Rollback failed for {verb}: {str(e)}")
        
        # Update execution state
        if self.current_context:
            for step in self.current_context.steps:
                if step.status == ExecutionState.COMPLETED and step.verb in rolled_back:
                    step.status = ExecutionState.ROLLED_BACK
        
        # Update metrics
        if rolled_back:
            self.metrics['rolled_back_executions'] += 1
        
        return {
            'success': len(failed_rollbacks) == 0,
            'rolled_back': rolled_back,
            'failed': failed_rollbacks,
            'error': '; '.join(failed_rollbacks) if failed_rollbacks else None
        }
    
    def _get_rollback_action(self, verb: str, params: Dict, result: Any) -> Optional[Dict]:
        """Get rollback action for a verb"""
        rollback_map = {
            'create_task': {
                'function': 'delete_task',
                'params': {'task_id': result} if isinstance(result, str) else {}
            },
            'update_task_status': {
                'function': 'update_task_status',
                'params': {'task_id': params.get('task_id'), 'status': 'PENDING'}
            },
            'update_location_status': {
                'function': 'update_location_status',
                'params': {'location_id': params.get('location_id'), 'status': 'AVAILABLE'}
            },
            # Add more rollback mappings as needed
        }
        
        return rollback_map.get(verb)
    
    def _create_enhanced_registry(self) -> Dict[str, Any]:
        """Create enhanced function registry with monitoring"""
        base_registry = self.functions.get_registry()
        
        # Add enhanced functions
        enhanced_registry = {}
        for name, func in base_registry.items():
            enhanced_registry[name] = self._wrap_function_with_monitoring(name, func)
        
        return enhanced_registry
    
    def _wrap_function_with_monitoring(self, name: str, func):
        """Wrap function with monitoring and error handling"""
        async def monitored_async_func(*args, **kwargs):
            start_time = datetime.now()
            try:
                result = await func(*args, **kwargs)
                duration = (datetime.now() - start_time).total_seconds()
                self._log_debug(f"Function {name} completed in {duration:.3f}s")
                return result
            except Exception as e:
                duration = (datetime.now() - start_time).total_seconds()
                self._log_error(f"Function {name} failed after {duration:.3f}s: {str(e)}")
                raise
        
        def monitored_sync_func(*args, **kwargs):
            start_time = datetime.now()
            try:
                result = func(*args, **kwargs)
                duration = (datetime.now() - start_time).total_seconds()
                self._log_debug(f"Function {name} completed in {duration:.3f}s")
                return result
            except Exception as e:
                duration = (datetime.now() - start_time).total_seconds()
                self._log_error(f"Function {name} failed after {duration:.3f}s: {str(e)}")
                raise
        
        if asyncio.iscoroutinefunction(func):
            return monitored_async_func
        else:
            return monitored_sync_func
    
    def _log_execution_stack(self):
        """Log current execution stack"""
        if not self.current_context:
            return
        
        stack = self.current_context.get_execution_stack()
        self._log_info("Execution Stack:")
        for entry in stack:
            self._log_info(f"  {entry}")
    
    def _create_success_response(self, result: Any, flow_id: str, 
                                execution_time: float) -> Dict[str, Any]:
        """Create success response with detailed information"""
        # Handle new executor return format with execution_log
        if isinstance(result, dict) and 'execution_log' in result:
            execution_log = result.get('execution_log', [])
            final_variables = result.get('variables', {})
        else:
            # Fallback for old format
            execution_log = []
            final_variables = result if isinstance(result, dict) else {}
        
        response = {
            'status': 'completed',
            'flow_id': flow_id,
            'result': result,
            'execution_log': execution_log,  # Include execution log
            'final_variables': final_variables,  # Include final variables
            'total_steps': len(execution_log),  # Count from execution log
            'execution_time': execution_time,
            'timestamp': datetime.now().isoformat()
        }
        
        if self.current_context:
            response['execution_summary'] = {
                'total_steps': len(self.current_context.steps),
                'completed_steps': sum(1 for s in self.current_context.steps 
                                     if s.status == ExecutionState.COMPLETED),
                'failed_steps': sum(1 for s in self.current_context.steps 
                                  if s.status == ExecutionState.FAILED),
                'execution_stack': self.current_context.get_execution_stack()
            }
        
        return response
    
    def _create_error_response(self, error: str, details: Any, flow_id: str) -> Dict[str, Any]:
        """Create error response with detailed information"""
        response = {
            'status': 'failed',
            'flow_id': flow_id,
            'error': error,
            'timestamp': datetime.now().isoformat()
        }
        
        if details:
            response['details'] = details
        
        if self.current_context:
            response['execution_summary'] = {
                'total_steps': len(self.current_context.steps),
                'completed_steps': sum(1 for s in self.current_context.steps 
                                     if s.status == ExecutionState.COMPLETED),
                'failed_steps': sum(1 for s in self.current_context.steps 
                                  if s.status == ExecutionState.FAILED),
                'rolled_back_steps': sum(1 for s in self.current_context.steps 
                                       if s.status == ExecutionState.ROLLED_BACK),
                'execution_stack': self.current_context.get_execution_stack(),
                'last_error': self.current_context.current_step.error 
                            if self.current_context.current_step else None
            }
        
        return response
    
    def _update_metrics(self, success: bool, execution_time: float):
        """Update performance metrics"""
        self.metrics['total_executions'] += 1
        self.metrics['total_execution_time'] += execution_time
        
        if success:
            self.metrics['successful_executions'] += 1
        else:
            self.metrics['failed_executions'] += 1
        
        # Update average execution time
        self.metrics['average_execution_time'] = (
            self.metrics['total_execution_time'] / self.metrics['total_executions']
        )
    
    def _add_to_history(self, context: ExecutionContext):
        """Add execution context to history"""
        self.execution_history.append(context)
        
        # Limit history size
        if len(self.execution_history) > self.max_history_size:
            self.execution_history = self.execution_history[-self.max_history_size:]
    
    def get_execution_history(self, flow_id: str = None) -> List[Dict[str, Any]]:
        """Get execution history"""
        history = []
        
        for context in self.execution_history:
            if flow_id and context.flow_id != flow_id:
                continue
            
            history.append({
                'flow_id': context.flow_id,
                'started_at': context.started_at.isoformat(),
                'total_steps': len(context.steps),
                'completed_steps': sum(1 for s in context.steps 
                                     if s.status == ExecutionState.COMPLETED),
                'failed_steps': sum(1 for s in context.steps 
                                  if s.status == ExecutionState.FAILED),
                'metadata': context.metadata
            })
        
        return history
    
    def get_metrics(self) -> Dict[str, Any]:
        """Get performance metrics"""
        return {
            **self.metrics,
            'success_rate': (self.metrics['successful_executions'] / 
                           self.metrics['total_executions'] * 100) 
                          if self.metrics['total_executions'] > 0 else 0,
            'failure_rate': (self.metrics['failed_executions'] / 
                           self.metrics['total_executions'] * 100) 
                          if self.metrics['total_executions'] > 0 else 0
        }
    
    # Logging helpers
    def _log_info(self, message: str):
        if self.logger:
            self.logger.info(message)
    
    def _log_error(self, message: str):
        if self.logger:
            self.logger.error(message)
    
    def _log_warning(self, message: str):
        if self.logger:
            self.logger.warning(message)
    
    def _log_debug(self, message: str):
        if self.logger:
            self.logger.debug(message)