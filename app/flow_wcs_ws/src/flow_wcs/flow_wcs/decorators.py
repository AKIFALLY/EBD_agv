#!/usr/bin/env python3
"""
Flow Function Decorators
用於自動註冊和管理 flow 函數的裝飾器系統
"""

from typing import Dict, List, Any, Callable, Optional
from functools import wraps

# 全局函數註冊表
function_registry: Dict[str, Dict[str, Dict]] = {}


def flow_function(category: str, description: str, params: Optional[List[str]] = None, 
                  returns: str = "any", defaults: Optional[Dict[str, Any]] = None,
                  also_register_as: Optional[str] = None):
    """
    裝飾器：自動註冊函數到函數庫
    
    Args:
        category: 函數類別 (query, check, task, action, control, special)
        description: 函數描述
        params: 參數列表
        returns: 返回值類型
        defaults: 參數預設值字典
        also_register_as: 額外註冊的簡短名稱 (不含 category 前綴)
    
    Example:
        @flow_function("query", "查詢位置", ["type", "rooms", "has_rack"], "array",
                      defaults={"type": "rack", "rooms": ["room01", "room02"], "has_rack": True})
        def locations(self, params):
            # 實現...
    """
    def decorator(func: Callable) -> Callable:
        # 取得函數名稱
        func_name = func.__name__
        
        # 處理特殊函數名稱 (如 query_locations -> locations)
        if func_name.startswith(f"{category}_"):
            func_name = func_name[len(category) + 1:]
        
        # 完整函數名稱
        full_name = f"{category}.{func_name}"
        
        # 註冊函數到註冊表
        if category not in function_registry:
            function_registry[category] = {}
            
        function_registry[category][func_name] = {
            "name": full_name,
            "description": description,
            "params": params or [],
            "returns": returns,
            "defaults": defaults or {},  # 加入預設值
            "handler": func,
            "category": category,
            "func_name": func_name
        }
        
        # 如果指定了額外的簡短名稱，也註冊到 special 類別
        if also_register_as:
            if "special" not in function_registry:
                function_registry["special"] = {}
            
            function_registry["special"][also_register_as] = {
                "name": also_register_as,  # 使用簡短名稱
                "description": description,
                "params": params or [],
                "returns": returns,
                "defaults": defaults or {},
                "handler": func,
                "category": "special",  # 註冊為 special 類別
                "func_name": also_register_as
            }
        
        # 為函數添加元數據屬性
        func._flow_metadata = {
            "category": category,
            "name": full_name,
            "description": description,
            "params": params or [],
            "returns": returns,
            "defaults": defaults or {}  # 加入預設值
        }
        
        # 保持原函數不變
        @wraps(func)
        def wrapper(*args, **kwargs):
            return func(*args, **kwargs)
        
        # 保留元數據在 wrapper 上
        wrapper._flow_metadata = func._flow_metadata
        
        return wrapper
    
    return decorator


def get_function_library() -> Dict[str, List[Dict]]:
    """
    獲取完整的函數庫（用於 API）
    
    Returns:
        按類別組織的函數庫字典
    """
    library = {}
    
    for category, funcs in function_registry.items():
        library[category] = []
        for func_name, func_info in funcs.items():
            # 只返回必要的元數據，不包含 handler
            library[category].append({
                "name": func_info["name"],
                "description": func_info["description"],
                "params": func_info["params"],
                "returns": func_info["returns"],
                "defaults": func_info.get("defaults", {})  # 包含預設值
            })
    
    return library


def get_function_handler(function_name: str) -> Optional[Callable]:
    """
    根據函數名稱獲取處理函數
    
    Args:
        function_name: 完整函數名稱 (如 "query.locations")
        
    Returns:
        函數處理器，如果不存在則返回 None
    """
    try:
        category, func_name = function_name.split('.', 1)
        if category in function_registry and func_name in function_registry[category]:
            return function_registry[category][func_name]["handler"]
    except ValueError:
        pass
    
    return None


def get_function_metadata(function_name: str) -> Optional[Dict]:
    """
    獲取函數的元數據
    
    Args:
        function_name: 完整函數名稱 (如 "query.locations")
        
    Returns:
        函數元數據字典，如果不存在則返回 None
    """
    try:
        category, func_name = function_name.split('.', 1)
        if category in function_registry and func_name in function_registry[category]:
            func_info = function_registry[category][func_name].copy()
            # 不返回 handler
            func_info.pop("handler", None)
            return func_info
    except ValueError:
        pass
    
    return None


def list_registered_functions() -> List[str]:
    """
    列出所有已註冊的函數名稱
    
    Returns:
        函數名稱列表
    """
    functions = []
    for category, funcs in function_registry.items():
        for func_name in funcs:
            functions.append(f"{category}.{func_name}")
    
    return sorted(functions)


def clear_registry():
    """
    清空函數註冊表（主要用於測試）
    """
    global function_registry
    function_registry.clear()