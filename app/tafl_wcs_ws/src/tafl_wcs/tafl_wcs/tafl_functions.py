#!/usr/bin/env python3
"""
TAFL Functions - Simplified function interfaces for TAFL verbs
"""

from typing import Dict, Any, List, Optional
from datetime import datetime


class TAFLFunctions:
    """Function registry for TAFL verbs"""
    
    def __init__(self, db_bridge):
        """Initialize with database bridge
        
        Args:
            db_bridge: TAFLDatabaseBridge instance
        """
        self.db_bridge = db_bridge
        self.functions = self._register_functions()
    
    def _register_functions(self) -> Dict[str, callable]:
        """Register all TAFL verb functions
        
        Returns:
            Dictionary mapping function names to implementations
        """
        functions = {}
        
        # Register db_bridge functions directly
        if self.db_bridge:
            functions.update({
                'query_locations': self.db_bridge.query_locations,
                'query_racks': self.db_bridge.query_racks,
                'query_agvs': self.db_bridge.query_agvs,  # Added AGV query support
                'query_tasks': self.db_bridge.query_tasks,
                'query_works': self.db_bridge.query_works,
                'query_carriers': self.db_bridge.query_carriers,  # Added carrier query support
                'create_task': self._wrap_create_task,
                'create_tasks': self._wrap_create_task,  # Plural alias for TAFL compatibility
                'create_rack': self.db_bridge.create_rack,
                'create_racks': self.db_bridge.create_rack,  # Plural alias for TAFL compatibility
                'update_task': self.db_bridge.update_task,  # Generic task update
                'update_tasks': self.db_bridge.update_task,  # Plural alias
                'update_task_status': self.db_bridge.update_task_status,
                'update_rack': self.db_bridge.update_rack,  # Generic rack update
                'update_racks': self.db_bridge.update_rack,  # Plural alias
                'update_rack_side_completed': self.db_bridge.update_rack_side_completed,
                'update_location_status': self.db_bridge.update_location_status,
            })
        
        # Add additional TAFL-specific functions
        functions.update({
            # Utility functions
            'now': self.now,
            'format_date': self.format_date,
            'parse_json': self.parse_json,
            'to_json': self.to_json,
            
            # Math functions
            'sum': self.sum_values,
            'avg': self.avg_values,
            'min': self.min_value,
            'max': self.max_value,
            
            # String functions
            'concat': self.concat,
            'split': self.split,
            'replace': self.replace,
            'upper': self.upper,
            'lower': self.lower,
            
            # List functions
            'filter': self.filter_list,
            'map': self.map_list,
            'sort': self.sort_list,
            'unique': self.unique_list,
            'first': self.first_item,
            'last': self.last_item,
            
            # Special TAFL functions
            'notify': self.notify,
            'set': self.set_variable,
            'get': self.get_variable,
            
            # TAFL v1.1 verb support
            'check_condition': self.check_condition,
            'evaluate_switch': self.evaluate_switch,
            'generate_id': self.generate_id,
            'validate_range': self.validate_range,
        })
        
        return functions
    
    def _wrap_create_task(self, **kwargs):
        """Wrapper for create_task to handle return format
        
        The db_bridge.create_task returns (task_id, details_dict),
        but TAFL expects a dict with accessible properties like 'id'.
        """
        task_id, details = self.db_bridge.create_task(**kwargs)
        if task_id and details:
            # Ensure the dict has 'id' property for ${new_task.id} interpolation
            details['id'] = task_id
            details['task_id'] = task_id  # Keep both for compatibility
            return details
        return None
    
    # ========== Utility Functions ==========
    
    def now(self) -> str:
        """Get current timestamp"""
        return datetime.now().isoformat()
    
    def format_date(self, date_str: str, format: str = '%Y-%m-%d %H:%M:%S') -> str:
        """Format date string"""
        dt = datetime.fromisoformat(date_str)
        return dt.strftime(format)
    
    def parse_json(self, json_str: str) -> Any:
        """Parse JSON string"""
        import json
        return json.loads(json_str)
    
    def to_json(self, obj: Any) -> str:
        """Convert object to JSON string"""
        import json
        return json.dumps(obj, ensure_ascii=False)
    
    # ========== Math Functions ==========

    def sum_values(self, values: List) -> float:
        """Sum of values with proper type conversion

        Handles mixed-type arrays by converting to numeric values.
        Non-numeric values are skipped.
        """
        if not values:
            return 0

        numeric_values = []
        for v in values:
            try:
                # Try to convert to float
                numeric_values.append(float(v))
            except (TypeError, ValueError):
                # Skip non-numeric values
                pass

        return sum(numeric_values) if numeric_values else 0

    def avg_values(self, values: List) -> float:
        """Average of values with proper type conversion

        Handles mixed-type arrays by converting to numeric values.
        Non-numeric values are skipped.
        """
        if not values:
            return 0

        numeric_values = []
        for v in values:
            try:
                # Try to convert to float
                numeric_values.append(float(v))
            except (TypeError, ValueError):
                # Skip non-numeric values
                pass

        return sum(numeric_values) / len(numeric_values) if numeric_values else 0

    def min_value(self, values: List) -> float:
        """Minimum value with proper type conversion

        Handles mixed-type arrays by converting to numeric values.
        Non-numeric values are skipped.
        """
        if not values:
            return 0

        numeric_values = []
        for v in values:
            try:
                # Try to convert to float
                numeric_values.append(float(v))
            except (TypeError, ValueError):
                # Skip non-numeric values
                pass

        return min(numeric_values) if numeric_values else 0

    def max_value(self, values: List) -> float:
        """Maximum value with proper type conversion

        Handles mixed-type arrays by converting to numeric values.
        Non-numeric values are skipped.
        """
        if not values:
            return 0

        numeric_values = []
        for v in values:
            try:
                # Try to convert to float
                numeric_values.append(float(v))
            except (TypeError, ValueError):
                # Skip non-numeric values
                pass

        return max(numeric_values) if numeric_values else 0
    
    # ========== String Functions ==========
    
    def concat(self, *args) -> str:
        """Concatenate strings"""
        return ''.join(str(arg) for arg in args)
    
    def split(self, text: str, delimiter: str = ',') -> List[str]:
        """Split string"""
        return text.split(delimiter)
    
    def replace(self, text: str, old: str, new: str) -> str:
        """Replace string"""
        return text.replace(old, new)
    
    def upper(self, text: str) -> str:
        """Convert to uppercase"""
        return text.upper()
    
    def lower(self, text: str) -> str:
        """Convert to lowercase"""
        return text.lower()
    
    # ========== List Functions ==========
    
    def filter_list(self, items: List[Any], condition: callable) -> List[Any]:
        """Filter list by condition"""
        return [item for item in items if condition(item)]
    
    def map_list(self, items: List[Any], transform: callable) -> List[Any]:
        """Map transformation over list"""
        return [transform(item) for item in items]
    
    def sort_list(self, items: List[Any], key: str = None, reverse: bool = False) -> List[Any]:
        """Sort list"""
        if key and isinstance(items[0], dict):
            return sorted(items, key=lambda x: x.get(key), reverse=reverse)
        return sorted(items, reverse=reverse)
    
    def unique_list(self, items: List[Any]) -> List[Any]:
        """Get unique items from list"""
        seen = set()
        result = []
        for item in items:
            # For dict items, use frozenset of items
            if isinstance(item, dict):
                key = frozenset(item.items())
            else:
                key = item
            
            if key not in seen:
                seen.add(key)
                result.append(item)
        return result
    
    def first_item(self, items: List[Any]) -> Any:
        """Get first item from list"""
        return items[0] if items else None
    
    def last_item(self, items: List[Any]) -> Any:
        """Get last item from list"""
        return items[-1] if items else None
    
    # ========== Special TAFL Functions ==========
    
    def notify(self, **kwargs) -> Dict[str, Any]:
        """Send notification according to TAFL v1.1 standard
        
        Args:
            level: Notification level (info, warning, alarm, error)
            message: Notification message
            recipients: Optional list of recipients
            **kwargs: Additional parameters
        
        Returns:
            Dictionary with notification details
        """
        # Get standard TAFL v1.1 attributes
        level = kwargs.get('level', 'info')
        message = kwargs.get('message', '')
        recipients = kwargs.get('recipients', [])
        
        # Validate level
        valid_levels = ['info', 'warning', 'alarm', 'error']
        if level not in valid_levels:
            level = 'info'
        
        # Log the notification
        if recipients:
            recipients_str = f" to {', '.join(recipients)}"
        else:
            recipients_str = ""
        print(f"[{level.upper()}]{recipients_str}: {message}")
        
        # Return detailed notification result
        result = {
            'notified': True,
            'level': level,
            'message': message
        }
        
        # Only include recipients if provided
        if recipients:
            result['recipients'] = recipients
            
        return result
    
    def set_variable(self, context: Dict, name: str, value: Any) -> Any:
        """Set variable in context
        
        Args:
            context: Execution context
            name: Variable name
            value: Variable value
        
        Returns:
            The value that was set
        """
        context[name] = value
        return value
    
    def get_variable(self, context: Dict, name: str, default: Any = None) -> Any:
        """Get variable from context
        
        Args:
            context: Execution context
            name: Variable name
            default: Default value if not found
        
        Returns:
            Variable value or default
        """
        return context.get(name, default)
    
    def get_function(self, name: str) -> Optional[callable]:
        """Get function by name
        
        Args:
            name: Function name
        
        Returns:
            Function implementation or None
        """
        return self.functions.get(name)
    
    def has_function(self, name: str) -> bool:
        """Check if function exists
        
        Args:
            name: Function name
        
        Returns:
            True if function exists
        """
        return name in self.functions
    
    def call_function(self, name: str, *args, **kwargs) -> Any:
        """Call function by name
        
        Args:
            name: Function name
            *args: Positional arguments
            **kwargs: Keyword arguments
        
        Returns:
            Function result
        
        Raises:
            ValueError: If function not found
        """
        func = self.get_function(name)
        if not func:
            raise ValueError(f"Function '{name}' not found")
        return func(*args, **kwargs)
    
    def get_registry(self) -> Dict[str, callable]:
        """Get function registry
        
        Returns:
            Function registry dictionary
        """
        return self.functions.copy()
    
    # ========== TAFL v1.1 Verb Support Functions ==========
    
    def check_condition(self, condition: str, context: Dict) -> bool:
        """Evaluate a condition expression
        
        Args:
            condition: Condition expression string
            context: Variable context
        
        Returns:
            Boolean result of condition evaluation
        """
        try:
            # Simple evaluation (in production, use safer expression evaluator)
            # Replace variable references with actual values
            expr = condition
            for var_name, var_value in context.items():
                expr = expr.replace(f'${{{var_name}}}', str(var_value))
                expr = expr.replace(f'${var_name}', str(var_value))
            
            # Evaluate the expression
            return eval(expr, {"__builtins__": {}}, {})
        except Exception as e:
            print(f"Error evaluating condition '{condition}': {e}")
            return False
    
    def evaluate_switch(self, expression: Any, case_value: str) -> bool:
        """Evaluate switch case condition
        
        Args:
            expression: The switch expression value
            case_value: The case condition (can be value, range, or comparison)
        
        Returns:
            True if case matches
        """
        try:
            # Handle range notation (e.g., "50..80")
            if '..' in str(case_value):
                min_val, max_val = case_value.split('..')
                return float(min_val) <= float(expression) <= float(max_val)
            
            # Handle comparison operators (e.g., "> 80", "< 20")
            if case_value.startswith('>'):
                if case_value.startswith('>='):
                    return float(expression) >= float(case_value[2:].strip())
                return float(expression) > float(case_value[1:].strip())
            elif case_value.startswith('<'):
                if case_value.startswith('<='):
                    return float(expression) <= float(case_value[2:].strip())
                return float(expression) < float(case_value[1:].strip())
            elif case_value.startswith('=='):
                return str(expression) == case_value[2:].strip()
            elif case_value.startswith('!='):
                return str(expression) != case_value[2:].strip()
            
            # Direct value comparison
            return str(expression) == str(case_value)
            
        except Exception as e:
            print(f"Error evaluating switch case '{case_value}' against '{expression}': {e}")
            return False
    
    def generate_id(self, prefix: str = "id") -> str:
        """Generate unique ID
        
        Args:
            prefix: ID prefix
        
        Returns:
            Unique ID string
        """
        import uuid
        return f"{prefix}_{uuid.uuid4().hex[:8]}"
    
    def validate_range(self, value: Any, min_val: Any = None, max_val: Any = None) -> bool:
        """Validate if value is within range
        
        Args:
            value: Value to check
            min_val: Minimum value (inclusive)
            max_val: Maximum value (inclusive)
        
        Returns:
            True if within range
        """
        try:
            val = float(value)
            if min_val is not None and val < float(min_val):
                return False
            if max_val is not None and val > float(max_val):
                return False
            return True
        except (ValueError, TypeError):
            return False
