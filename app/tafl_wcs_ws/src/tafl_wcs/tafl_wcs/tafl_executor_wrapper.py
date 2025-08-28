#!/usr/bin/env python3
"""
TAFL Executor Wrapper - Integrates TAFL parser/executor with database functions
"""

import sys
import os
from typing import Dict, Any, Optional, List
import asyncio
from datetime import datetime

# Add TAFL parser to path
sys.path.insert(0, '/app/tafl_ws/src/tafl')

from tafl.parser import TAFLParser
from tafl.executor import TAFLExecutor
from tafl.validator import TAFLValidator

from .tafl_db_bridge import TAFLDatabaseBridge
from .tafl_functions import TAFLFunctions


class TAFLExecutorWrapper:
    """Wrapper for TAFL executor with database integration"""
    
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
        self.current_flow_id = None
        self.execution_stats = {}
    
    async def execute_flow(self, flow_content: str, flow_id: str = None) -> Dict[str, Any]:
        """Execute a TAFL flow
        
        Args:
            flow_content: TAFL flow content (YAML string)
            flow_id: Optional flow identifier
        
        Returns:
            Execution result dictionary
        """
        self.current_flow_id = flow_id or 'unknown'
        start_time = datetime.now()
        
        try:
            # Parse TAFL flow
            if self.logger:
                self.logger.info(f"Parsing TAFL flow: {self.current_flow_id}")
            
            ast = self.parser.parse_string(flow_content)
            
            # Validate TAFL flow
            if not self.validator.validate(ast):
                errors = self.validator.get_errors()
                if self.logger:
                    self.logger.error(f"TAFL validation failed: {errors}")
                return {
                    'status': 'failed',
                    'error': 'Validation failed',
                    'details': errors,
                    'flow_id': self.current_flow_id
                }
            
            # Create executor with context
            executor = TAFLExecutor(ast)
            
            # Inject database functions into executor context
            self._inject_functions(executor)
            
            # Execute flow
            if self.logger:
                self.logger.info(f"Executing TAFL flow: {self.current_flow_id}")
            
            result = await executor.execute()
            
            # Calculate execution time
            execution_time = (datetime.now() - start_time).total_seconds()
            
            # Update statistics
            self.execution_stats[self.current_flow_id] = {
                'last_execution': datetime.now().isoformat(),
                'execution_time': execution_time,
                'status': 'completed'
            }
            
            if self.logger:
                self.logger.info(f"TAFL flow completed: {self.current_flow_id} in {execution_time:.2f}s")
            
            return {
                'status': 'completed',
                'result': result,
                'flow_id': self.current_flow_id,
                'execution_time': execution_time
            }
            
        except Exception as e:
            execution_time = (datetime.now() - start_time).total_seconds()
            
            # Update statistics
            self.execution_stats[self.current_flow_id] = {
                'last_execution': datetime.now().isoformat(),
                'execution_time': execution_time,
                'status': 'failed',
                'error': str(e)
            }
            
            if self.logger:
                self.logger.error(f"TAFL flow failed: {self.current_flow_id} - {e}")
            
            return {
                'status': 'failed',
                'error': str(e),
                'flow_id': self.current_flow_id,
                'execution_time': execution_time
            }
    
    def _inject_functions(self, executor: TAFLExecutor):
        """Inject database functions into executor context
        
        Args:
            executor: TAFL executor instance
        """
        # Initialize context if needed
        if not hasattr(executor, 'context'):
            executor.context = {}
        
        if not hasattr(executor.context, 'functions'):
            executor.context['functions'] = {}
        
        # Inject all functions from TAFLFunctions
        executor.context['functions'].update(self.functions.functions)
        
        # Map TAFL verb functions
        self._map_verb_functions(executor)
    
    def _map_verb_functions(self, executor: TAFLExecutor):
        """Map TAFL verbs to function implementations
        
        Args:
            executor: TAFL executor instance
        """
        # Map query verb
        executor.context['functions']['query'] = self._create_query_dispatcher()
        
        # Map check verb
        executor.context['functions']['check'] = self._create_check_dispatcher()
        
        # Map create verb
        executor.context['functions']['create'] = self._create_create_dispatcher()
        
        # Map update verb
        executor.context['functions']['update'] = self._create_update_dispatcher()
        
        # Map notify verb
        executor.context['functions']['notify'] = self.functions.notify
        
        # Map set/for/if control verbs
        executor.context['functions']['set'] = self._handle_set
        executor.context['functions']['for'] = self._handle_for
        executor.context['functions']['if'] = self._handle_if
    
    def _create_query_dispatcher(self):
        """Create query verb dispatcher
        
        Returns:
            Query dispatcher function
        """
        def query_dispatcher(target: str, **kwargs):
            func_name = f"query_{target}"
            if self.functions.has_function(func_name):
                return self.functions.call_function(func_name, **kwargs)
            else:
                # Default query behavior
                if self.logger:
                    self.logger.warning(f"Query function not found: {func_name}")
                return []
        
        return query_dispatcher
    
    def _create_check_dispatcher(self):
        """Create check verb dispatcher
        
        Returns:
            Check dispatcher function
        """
        def check_dispatcher(**kwargs):
            # Determine check type
            if 'condition' in kwargs:
                # Direct condition evaluation
                return self._evaluate_condition(kwargs['condition'])
            elif 'data' in kwargs:
                return self.functions.call_function('check_empty', kwargs['data'])
            elif 'rack_id' in kwargs:
                return self.functions.call_function('check_rack_status', **kwargs)
            elif 'task_type' in kwargs or 'location_id' in kwargs:
                return self.functions.call_function('check_task_exists', **kwargs)
            else:
                return False
        
        return check_dispatcher
    
    def _create_create_dispatcher(self):
        """Create create verb dispatcher
        
        Returns:
            Create dispatcher function
        """
        def create_dispatcher(target: str, **params):
            func_name = f"create_{target}"
            if self.functions.has_function(func_name):
                return self.functions.call_function(func_name, **params)
            else:
                if self.logger:
                    self.logger.warning(f"Create function not found: {func_name}")
                return None
        
        return create_dispatcher
    
    def _create_update_dispatcher(self):
        """Create update verb dispatcher
        
        Returns:
            Update dispatcher function
        """
        def update_dispatcher(target: str, **params):
            func_name = f"update_{target}"
            
            # Extract ID from params
            id_field = f"{target}_id"
            if 'id' in params:
                entity_id = params.pop('id')
            elif id_field in params:
                entity_id = params.pop(id_field)
            elif 'where' in params:
                # Handle where clause
                where = params.pop('where')
                if isinstance(where, dict) and 'id' in where:
                    entity_id = where['id']
                else:
                    entity_id = where
            else:
                if self.logger:
                    self.logger.error(f"No ID provided for update_{target}")
                return False
            
            # Extract set clause if present
            if 'set' in params:
                updates = params.pop('set')
            else:
                updates = params
            
            if self.functions.has_function(func_name):
                return self.functions.call_function(func_name, entity_id, **updates)
            else:
                if self.logger:
                    self.logger.warning(f"Update function not found: {func_name}")
                return False
        
        return update_dispatcher
    
    def _handle_set(self, **kwargs):
        """Handle set verb
        
        Returns:
            Set result
        """
        # Set verb is handled internally by TAFL executor
        # This is just a placeholder
        return kwargs.get('value')
    
    def _handle_for(self, **kwargs):
        """Handle for verb
        
        Returns:
            For loop result
        """
        # For verb is handled internally by TAFL executor
        # This is just a placeholder
        return []
    
    def _handle_if(self, **kwargs):
        """Handle if verb
        
        Returns:
            If result
        """
        # If verb is handled internally by TAFL executor
        # This is just a placeholder
        return None
    
    def _evaluate_condition(self, condition: str) -> bool:
        """Evaluate a condition expression
        
        Args:
            condition: Condition expression string
        
        Returns:
            Boolean result
        """
        # Simple condition evaluation
        # In production, would use a proper expression evaluator
        try:
            # Handle simple comparisons
            if '>=' in condition:
                parts = condition.split('>=')
                return float(parts[0].strip()) >= float(parts[1].strip())
            elif '<=' in condition:
                parts = condition.split('<=')
                return float(parts[0].strip()) <= float(parts[1].strip())
            elif '>' in condition:
                parts = condition.split('>')
                return float(parts[0].strip()) > float(parts[1].strip())
            elif '<' in condition:
                parts = condition.split('<')
                return float(parts[0].strip()) < float(parts[1].strip())
            elif '==' in condition:
                parts = condition.split('==')
                return parts[0].strip() == parts[1].strip()
            elif '!=' in condition:
                parts = condition.split('!=')
                return parts[0].strip() != parts[1].strip()
            else:
                # Try to evaluate as boolean
                return bool(condition)
        except:
            return False
    
    def get_stats(self) -> Dict[str, Any]:
        """Get execution statistics
        
        Returns:
            Statistics dictionary
        """
        return {
            'current_flow': self.current_flow_id,
            'execution_stats': self.execution_stats
        }
    
    def shutdown(self):
        """Shutdown executor and clean up resources"""
        self.db_bridge.shutdown()
