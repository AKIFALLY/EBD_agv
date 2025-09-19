#!/usr/bin/env python3
"""
TAFL Integration Module for Flow WCS
Integrates TAFL v1.0 executor with existing flow_wcs system
"""

import sys
import logging
from typing import Dict, Any, Optional, Union
from pathlib import Path

# Add TAFL workspace to path
tafl_path = Path('/home/ct/RosAGV/app/tafl_ws/src/tafl')
if tafl_path.exists() and str(tafl_path) not in sys.path:
    sys.path.insert(0, str(tafl_path))

try:
    from tafl.parser import TAFLParser
    from tafl.executor import TAFLExecutor
    from tafl.validator import TAFLValidator
    TAFL_AVAILABLE = True
except ImportError as e:
    TAFL_AVAILABLE = False
    print(f"Warning: TAFL not available: {e}")

logger = logging.getLogger(__name__)


class TAFLIntegration:
    """
    Bridge between TAFL v1.0 and Flow WCS system
    """
    
    def __init__(self, flow_executor=None):
        """
        Initialize TAFL integration
        
        Args:
            flow_executor: Optional reference to FlowExecutor for function access
        """
        if not TAFL_AVAILABLE:
            raise ImportError("TAFL modules not available. Check tafl_ws installation.")
        
        self.parser = TAFLParser()
        self.executor = TAFLExecutor()
        self.validator = TAFLValidator()
        self.flow_executor = flow_executor
        
        # Register flow_wcs functions if available
        if flow_executor:
            self._register_flow_functions()
    
    def _register_flow_functions(self):
        """
        Register flow_wcs functions with TAFL executor
        Maps TAFL verbs to flow_wcs functions
        """
        if not self.flow_executor:
            return
        
        # TODO: In TAFL v1.0, external functions are handled differently
        # For now, TAFL executor uses its own function registry
        # Future implementation: Bridge TAFL verbs to flow_wcs functions
        
        logger.info(f"TAFL integration initialized (function mapping pending)")
    
    def _wrap_flow_function(self, function_name: str):
        """
        Wrap a flow_wcs function for TAFL executor
        
        Args:
            function_name: Name of the flow_wcs function
            
        Returns:
            Wrapped async function compatible with TAFL
        """
        async def wrapped_function(**params):
            # Get the flow_wcs function
            func = self.flow_executor.functions.get(function_name)
            if not func:
                raise ValueError(f"Flow function '{function_name}' not found")
            
            # Execute with flow_wcs context
            result = await self.flow_executor.execute_function(function_name, params)
            return result
        
        return wrapped_function
    
    def parse_tafl_file(self, file_path: str):
        """
        Parse a TAFL YAML file
        
        Args:
            file_path: Path to TAFL YAML file
            
        Returns:
            TAFLProgram AST
        """
        return self.parser.parse_file(file_path)
    
    def parse_tafl_string(self, yaml_content: str):
        """
        Parse TAFL YAML from string
        
        Args:
            yaml_content: TAFL YAML content as string
            
        Returns:
            TAFLProgram AST
        """
        return self.parser.parse_string(yaml_content)
    
    def validate_tafl(self, program) -> list:
        """
        Validate a TAFL program
        
        Args:
            program: TAFLProgram AST
            
        Returns:
            List of validation errors (empty if valid)
        """
        is_valid = self.validator.validate(program)
        if not is_valid:
            return self.validator.get_errors()
        return []
    
    async def execute_tafl(self, program, initial_context: Optional[Dict] = None) -> Dict[str, Any]:
        """
        Execute a TAFL program
        
        Args:
            program: TAFLProgram AST
            initial_context: Optional initial variables
            
        Returns:
            Final execution context with all variables
        """
        # Set initial context if provided
        if initial_context:
            for key, value in initial_context.items():
                self.executor.context.set_variable(key, value)
        
        # Execute the program
        result = await self.executor.execute(program)
        return result
    
    async def execute_tafl_file(self, file_path: str, initial_context: Optional[Dict] = None) -> Dict[str, Any]:
        """
        Parse and execute a TAFL file
        
        Args:
            file_path: Path to TAFL YAML file
            initial_context: Optional initial variables
            
        Returns:
            Final execution context
        """
        # Parse
        program = self.parse_tafl_file(file_path)
        
        # Validate
        errors = self.validate_tafl(program)
        if errors:
            raise ValueError(f"TAFL validation errors: {errors}")
        
        # Execute
        return await self.execute_tafl(program, initial_context)
    
    async def execute_tafl_string(self, yaml_content: str, initial_context: Optional[Dict] = None) -> Dict[str, Any]:
        """
        Parse and execute TAFL from string
        
        Args:
            yaml_content: TAFL YAML content
            initial_context: Optional initial variables
            
        Returns:
            Final execution context
        """
        # Parse
        program = self.parse_tafl_string(yaml_content)
        
        # Validate
        errors = self.validate_tafl(program)
        if errors:
            raise ValueError(f"TAFL validation errors: {errors}")
        
        # Execute
        return await self.execute_tafl(program, initial_context)
    
    def detect_format(self, yaml_content: str) -> str:
        """
        Detect if YAML is TAFL or Linear Flow v2 format
        
        Args:
            yaml_content: YAML content as string
            
        Returns:
            'tafl' or 'linear_flow_v2'
        """
        # Simple heuristic: TAFL uses 'flow:' with verbs
        # Linear Flow v2 uses 'flow:' with 'exec:' statements
        
        import yaml
        try:
            data = yaml.safe_load(yaml_content)
            if 'flow' in data and isinstance(data['flow'], list):
                # Check first flow item
                if data['flow']:
                    first_item = data['flow'][0]
                    # TAFL uses verb keys
                    tafl_verbs = {'query', 'check', 'create', 'update', 'if', 'for', 'switch', 'set', 'stop', 'notify'}
                    if any(verb in first_item for verb in tafl_verbs):
                        return 'tafl'
                    # Linear Flow v2 uses 'exec' or 'type'
                    if 'exec' in first_item or 'type' in first_item:
                        return 'linear_flow_v2'
        except:
            pass
        
        return 'unknown'


class HybridFlowExecutor:
    """
    Hybrid executor that can run both TAFL and Linear Flow v2
    """
    
    def __init__(self, flow_executor):
        """
        Initialize hybrid executor
        
        Args:
            flow_executor: Existing FlowExecutor instance
        """
        self.flow_executor = flow_executor
        self.tafl_integration = TAFLIntegration(flow_executor)
    
    async def execute(self, yaml_content: str, initial_context: Optional[Dict] = None) -> Dict[str, Any]:
        """
        Execute YAML flow, automatically detecting format
        
        Args:
            yaml_content: YAML content (TAFL or Linear Flow v2)
            initial_context: Optional initial variables
            
        Returns:
            Execution result
        """
        format_type = self.tafl_integration.detect_format(yaml_content)
        
        if format_type == 'tafl':
            logger.info("Detected TAFL format, using TAFL executor")
            return await self.tafl_integration.execute_tafl_string(yaml_content, initial_context)
        elif format_type == 'linear_flow_v2':
            logger.info("Detected Linear Flow v2 format, using FlowExecutor")
            # Parse and execute with existing flow executor
            import yaml
            flow_data = yaml.safe_load(yaml_content)
            
            # FlowExecutor expects 'workflow' key, but Linear Flow v2 uses 'flow'
            # Transform the structure to match FlowExecutor expectations
            flow_config = {
                'variables': flow_data.get('variables', {}),
                'workflow': flow_data.get('flow', [])
            }
            
            # Create a new FlowExecutor instance with the transformed config
            from flow_wcs.flow_executor import FlowExecutor
            executor = FlowExecutor(flow_config, initial_context or {})
            return await executor.execute()
        else:
            raise ValueError("Unable to detect flow format (TAFL or Linear Flow v2)")


# Convenience functions for standalone use
async def execute_tafl_file(file_path: str, initial_context: Optional[Dict] = None) -> Dict[str, Any]:
    """
    Standalone function to execute a TAFL file
    
    Args:
        file_path: Path to TAFL YAML file
        initial_context: Optional initial variables
        
    Returns:
        Execution result
    """
    integration = TAFLIntegration()
    return await integration.execute_tafl_file(file_path, initial_context)


async def execute_tafl_string(yaml_content: str, initial_context: Optional[Dict] = None) -> Dict[str, Any]:
    """
    Standalone function to execute TAFL from string
    
    Args:
        yaml_content: TAFL YAML content
        initial_context: Optional initial variables
        
    Returns:
        Execution result
    """
    integration = TAFLIntegration()
    return await integration.execute_tafl_string(yaml_content, initial_context)