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
        functions.update(self.db_bridge.functions)
        
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
        })
        
        return functions
    
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
    
    def sum_values(self, values: List[float]) -> float:
        """Sum of values"""
        return sum(values)
    
    def avg_values(self, values: List[float]) -> float:
        """Average of values"""
        return sum(values) / len(values) if values else 0
    
    def min_value(self, values: List[float]) -> float:
        """Minimum value"""
        return min(values) if values else 0
    
    def max_value(self, values: List[float]) -> float:
        """Maximum value"""
        return max(values) if values else 0
    
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
    
    def notify(self, channel: str, message: str, **kwargs) -> bool:
        """Send notification to channel
        
        Args:
            channel: Notification channel (system, kuka_fleet, info, warning, error)
            message: Notification message
            **kwargs: Additional parameters
        
        Returns:
            Success status
        """
        level = 'info'
        if channel in ['warning', 'error']:
            level = channel
        elif channel == 'kuka_fleet':
            # Special handling for KUKA Fleet notifications
            print(f"[KUKA Fleet] {message}")
            # In production, would send to KUKA Fleet API
            return True
        
        return self.db_bridge.send_notification(message, level, channel=channel, **kwargs)
    
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
