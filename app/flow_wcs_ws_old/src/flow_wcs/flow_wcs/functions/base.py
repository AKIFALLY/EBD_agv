#!/usr/bin/env python3
"""
Base class for Flow WCS functions with systematic default value handling
"""

from typing import Dict, Any, Optional, List
import logging
from ..decorators import flow_function


class FlowFunctionBase:
    """Base class for all flow function modules"""
    
    def __init__(self, executor=None):
        """Initialize with reference to FlowExecutor"""
        self.executor = executor
        self.logger = logging.getLogger(self.__class__.__name__)
        
    def get_param(self, params: Dict, key: str, default: Any = None, 
                  required: bool = False) -> Any:
        """
        Systematic parameter extraction with proper default handling.
        Prevents circular variable overrides and ensures consistent defaults.
        
        Args:
            params: Parameter dictionary
            key: Parameter key to extract
            default: Default value if key not found or None
            required: If True, raises error when key is missing
            
        Returns:
            Parameter value with proper type and default
        """
        # Get the raw value
        value = params.get(key)
        
        # Handle None, empty string, or unresolved variables
        if value is None or value == '' or \
           (isinstance(value, str) and value.startswith('${') and value.endswith('}')):
            if required and default is None:
                raise ValueError(f"Required parameter '{key}' is missing")
            return default
            
        return value
    
    def validate_params(self, params: Dict, required_keys: List[str]) -> None:
        """
        Validate that all required parameters are present
        
        Args:
            params: Parameter dictionary
            required_keys: List of required parameter keys
            
        Raises:
            ValueError: If any required key is missing
        """
        missing = []
        for key in required_keys:
            if key not in params or params[key] is None:
                missing.append(key)
        
        if missing:
            raise ValueError(f"Missing required parameters: {', '.join(missing)}")
    
    def safe_cast(self, value: Any, target_type: type, default: Any = None) -> Any:
        """
        Safely cast value to target type with fallback to default
        
        Args:
            value: Value to cast
            target_type: Target type (int, float, bool, str, list, dict)
            default: Default value if casting fails
            
        Returns:
            Casted value or default
        """
        if value is None:
            return default
            
        try:
            if target_type == bool:
                # Special handling for boolean
                if isinstance(value, bool):
                    return value
                if isinstance(value, str):
                    return value.lower() in ('true', '1', 'yes', 'on')
                return bool(value)
            elif target_type == list:
                # Handle list conversion
                if isinstance(value, list):
                    return value
                if isinstance(value, str):
                    import json
                    try:
                        return json.loads(value)
                    except:
                        return [value]  # Single item list
                return list(value) if hasattr(value, '__iter__') else [value]
            elif target_type == dict:
                # Handle dict conversion
                if isinstance(value, dict):
                    return value
                if isinstance(value, str):
                    import json
                    return json.loads(value)
                return dict(value)
            else:
                # Standard type casting
                return target_type(value)
        except (ValueError, TypeError, Exception) as e:
            self.logger.warning(f"Failed to cast {value} to {target_type.__name__}: {e}")
            return default
    
    def log_execution(self, function_name: str, params: Dict, result: Any) -> None:
        """
        Log function execution for debugging
        
        Args:
            function_name: Name of the function executed
            params: Parameters passed
            result: Execution result
        """
        self.logger.debug(f"Executed {function_name} with params: {params}, result: {result}")
