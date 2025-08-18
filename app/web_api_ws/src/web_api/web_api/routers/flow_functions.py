"""
Flow Functions API Router
提供 flow_wcs 函數庫資訊給 Linear Flow Designer
"""

from fastapi import APIRouter, HTTPException
from typing import Dict, List, Any
import sys
from pathlib import Path

# Add flow_wcs to path to import FlowExecutor
flow_wcs_path = Path("/app/flow_wcs_ws/src/flow_wcs")
if str(flow_wcs_path) not in sys.path:
    sys.path.insert(0, str(flow_wcs_path))

router = APIRouter()

@router.get("/api/flow/functions")
async def get_flow_functions() -> Dict[str, Any]:
    """
    Get available functions from flow_wcs FlowExecutor
    
    Returns:
        Dict containing function library metadata
    """
    try:
        # Import FlowExecutor to get function library
        from flow_wcs.flow_executor import FlowExecutor
        
        # Get function library from FlowExecutor
        functions = FlowExecutor.get_function_library()
        
        return {
            "success": True,
            "functions": functions,
            "source": "flow_wcs",
            "version": "2.0.0"
        }
        
    except ImportError as e:
        raise HTTPException(
            status_code=500,
            detail=f"Failed to import flow_wcs: {str(e)}"
        )
    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail=f"Error getting function library: {str(e)}"
        )

@router.get("/api/flow/functions/flat")
async def get_functions_flat() -> Dict[str, Any]:
    """
    Get all functions in flat format (for validation mode)
    
    Returns:
        Dict with function names as keys
    """
    try:
        from flow_wcs.flow_executor import FlowExecutor
        
        # Get categorized functions
        categorized_functions = FlowExecutor.get_function_library()
        
        # Flatten the structure
        flat_functions = {}
        for category, funcs in categorized_functions.items():
            for func in funcs:
                func_name = func["name"]
                flat_functions[func_name] = {
                    "category": category,
                    "description": func.get("description", ""),
                    "params": func.get("params", []),
                    "returns": func.get("returns", "any"),
                    "defaults": func.get("defaults", {})
                }
        
        return {
            "success": True,
            "functions": flat_functions,
            "count": len(flat_functions),
            "source": "flow_wcs",
            "format": "flat",
            "version": "2.0.0"
        }
        
    except ImportError as e:
        raise HTTPException(
            status_code=500,
            detail=f"Failed to import flow_wcs: {str(e)}"
        )
    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail=f"Error getting function library: {str(e)}"
        )

@router.get("/api/flow/functions/categories")
async def get_function_categories() -> Dict[str, Any]:
    """
    Get function categories and counts
    
    Returns:
        Dict containing category information
    """
    try:
        from flow_wcs.flow_executor import FlowExecutor
        
        functions = FlowExecutor.get_function_library()
        
        categories = {}
        for category, funcs in functions.items():
            categories[category] = {
                "count": len(funcs),
                "functions": [f["name"] for f in funcs]
            }
        
        return {
            "success": True,
            "categories": categories
        }
        
    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail=f"Error getting categories: {str(e)}"
        )

@router.get("/api/flow/functions/{category}")
async def get_functions_by_category(category: str) -> Dict[str, Any]:
    """
    Get functions for a specific category
    
    Args:
        category: Function category (query, check, task, action, control, special)
        
    Returns:
        Dict containing functions for the category
    """
    try:
        from flow_wcs.flow_executor import FlowExecutor
        
        functions = FlowExecutor.get_function_library()
        
        if category not in functions:
            raise HTTPException(
                status_code=404,
                detail=f"Category '{category}' not found"
            )
        
        return {
            "success": True,
            "category": category,
            "functions": functions[category]
        }
        
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail=f"Error getting category functions: {str(e)}"
        )

@router.get("/api/flow/functions/search/{keyword}")
async def search_functions(keyword: str) -> Dict[str, Any]:
    """
    Search functions by keyword
    
    Args:
        keyword: Search keyword
        
    Returns:
        Dict containing matching functions
    """
    try:
        from flow_wcs.flow_executor import FlowExecutor
        
        functions = FlowExecutor.get_function_library()
        
        results = []
        keyword_lower = keyword.lower()
        
        for category, funcs in functions.items():
            for func in funcs:
                if (keyword_lower in func["name"].lower() or 
                    keyword_lower in func["description"].lower()):
                    results.append({
                        **func,
                        "category": category
                    })
        
        return {
            "success": True,
            "keyword": keyword,
            "count": len(results),
            "results": results
        }
        
    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail=f"Error searching functions: {str(e)}"
        )

@router.post("/api/flow/execute")
async def execute_flow_function(request: Dict[str, Any]) -> Dict[str, Any]:
    """
    Execute a single flow function for testing purposes
    
    Args:
        request: Dict containing:
            - function_name: Full function name (e.g., "query.locations")
            - params: Function parameters
            - variables: Optional variables context
            
    Returns:
        Dict containing execution result
    """
    try:
        from flow_wcs.flow_executor import FlowExecutor
        
        function_name = request.get("function_name")
        params = request.get("params", {})
        variables = request.get("variables", {})
        
        if not function_name:
            raise HTTPException(
                status_code=400,
                detail="function_name is required"
            )
        
        # Execute the function using the test method
        result = FlowExecutor.execute_function_test(
            function_name=function_name,
            params=params,
            variables=variables
        )
        
        return {
            "success": True,
            "function": function_name,
            "result": result,
            "type": type(result).__name__
        }
        
    except ValueError as e:
        # Function not found
        raise HTTPException(
            status_code=404,
            detail=str(e)
        )
    except Exception as e:
        # Execution error
        raise HTTPException(
            status_code=500,
            detail=f"Error executing function: {str(e)}"
        )