#!/usr/bin/env python3
"""
TAFL Editor Direct Integration - Phase 3 Simplified
Directly integrates with TAFL_WCS Node without HTTP service
"""

from fastapi import APIRouter, Request, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.responses import JSONResponse
import yaml
import json
import asyncio
from datetime import datetime
from typing import Dict, Any, Optional, List
import logging
import sys

# Add TAFL_WCS to path for direct import
# After colcon build, modules are in install directory
sys.path.insert(0, '/app/tafl_wcs_ws/install/tafl_wcs/lib/python3.12/site-packages')
sys.path.insert(0, '/app/tafl_ws/install/tafl/lib/python3.12/site-packages')

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

router = APIRouter(prefix="/tafl", tags=["tafl_editor_direct"])

# Import Phase 2 enhanced modules directly
# Global variables for module availability
ENHANCED_MODULES_AVAILABLE = False
TAFLExecutorWrapper = None
TAFLDatabaseBridge = None
TAFLParser = None

def check_and_import_modules():
    """Check and import enhanced modules dynamically"""
    global ENHANCED_MODULES_AVAILABLE, TAFLExecutorWrapper, TAFLDatabaseBridge, TAFLParser
    
    logger.info(f"=== check_and_import_modules called ===")
    logger.info(f"Current ENHANCED_MODULES_AVAILABLE: {ENHANCED_MODULES_AVAILABLE}")
    
    if ENHANCED_MODULES_AVAILABLE:
        logger.info("Modules already available, returning True")
        return True
    
    try:
        logger.info("Attempting to import enhanced modules...")
        from tafl_wcs.tafl_executor_wrapper import TAFLExecutorWrapper as TEW
        from tafl_wcs.tafl_db_bridge import TAFLDatabaseBridge as TDB
        from tafl.parser import TAFLParser as TP
        
        TAFLExecutorWrapper = TEW
        TAFLDatabaseBridge = TDB
        TAFLParser = TP
        ENHANCED_MODULES_AVAILABLE = True
        
        logger.info(f"✅ Enhanced modules loaded successfully")
        logger.info(f"  TAFLExecutorWrapper: {TAFLExecutorWrapper}")
        logger.info(f"  TAFLDatabaseBridge: {TAFLDatabaseBridge}")
        logger.info(f"  TAFLParser: {TAFLParser}")
        return True
    except ImportError as e:
        logger.warning(f"❌ Enhanced modules not available: {e}")
        logger.warning(f"Python path: {sys.path}")
        ENHANCED_MODULES_AVAILABLE = False
        return False

# Try to import on module load
check_and_import_modules()

# TAFL Verbs definition - Updated to match TAFL v1.1 specification (10 core verbs)
TAFL_VERBS = {
    "query": {
        "description": "Query data from database or system",
        "params": ["target", "where", "order", "limit", "as", "store_as"]
    },
    "check": {
        "description": "Check conditions or status",
        "params": ["condition", "where", "as", "target"]
    },
    "create": {
        "description": "Create new resources or records",
        "params": ["target", "with", "params", "as"]
    },
    "update": {
        "description": "Update existing data",
        "params": ["target", "where", "set", "as"]
    },
    "if": {
        "description": "Conditional execution",
        "params": ["condition", "then", "else"]
    },
    "for": {
        "description": "Loop through collection",
        "params": ["each", "in", "as", "filter", "do"]
    },
    "switch": {
        "description": "Multi-branch conditional",
        "params": ["expression", "cases"]  # TAFL v1.1: Only expression and cases at switch level
    },
    "set": {
        "description": "Set variable values",
        "params": ["variable", "value", "expression", "multi"]
    },
    "stop": {
        "description": "Stop flow execution",
        "params": ["reason", "code", "message"]
    },
    "notify": {
        "description": "Send notifications or output messages",
        "params": ["type", "message", "recipients", "level"]
    }
}

# WebSocket connection manager
class ConnectionManager:
    def __init__(self):
        self.active_connections: Dict[str, WebSocket] = {}
    
    async def connect(self, websocket: WebSocket, flow_id: str):
        await websocket.accept()
        self.active_connections[flow_id] = websocket
    
    def disconnect(self, flow_id: str):
        if flow_id in self.active_connections:
            del self.active_connections[flow_id]
    
    async def send_progress(self, flow_id: str, data: dict):
        if flow_id in self.active_connections:
            try:
                await self.active_connections[flow_id].send_json(data)
            except:
                pass

manager = ConnectionManager()

# Note: Page route is provided by tafl_editor.py
# This module only provides enhanced API endpoints

@router.get("/verbs")
async def get_supported_verbs():
    """Get list of supported TAFL verbs"""
    return {"verbs": TAFL_VERBS}

@router.post("/execute")
async def execute_tafl_flow(request: Request):
    """
    Execute TAFL flow - Direct integration with enhanced backend
    No HTTP service needed - direct Python calls
    """
    try:
        data = await request.json()
        flow_content = data.get("flow", [])
        metadata = data.get("metadata", {})
        settings = data.get("settings", {})  # Get settings from top level
        preload = data.get("preload", {})    # Get preload from top level
        rules = data.get("rules", {})        # Get rules from top level
        variables = data.get("variables", {})  # Get variables from top level
        flow_id = metadata.get("id", f"flow_{datetime.now().strftime('%Y%m%d_%H%M%S')}")
        execution_mode = data.get("mode", "real")
        
        # Direct print to stderr for debugging
        print(f"[DEBUG] ==> EXECUTE request: mode={execution_mode}, flow_id={flow_id}", file=sys.stderr)
        print(f"[DEBUG] ==> Variables received: {variables}", file=sys.stderr)
        
        # Re-check modules at runtime
        modules_available = check_and_import_modules()
        print(f"[DEBUG] ==> Modules available: {ENHANCED_MODULES_AVAILABLE}", file=sys.stderr)
        
        # Debug: Return current state (disabled - real execution working)
        # if "debug" in flow_id:
        #     return {
        #         "debug": True,
        #         "flow_id_received": flow_id,
        #         "ENHANCED_MODULES_AVAILABLE": ENHANCED_MODULES_AVAILABLE,
        #         "modules_available": modules_available,
        #         "execution_mode": execution_mode,
        #         "sys_path_count": len(sys.path),
        #         "has_tafl_wcs": any("tafl_wcs" in p for p in sys.path),
        #         "has_tafl": any("tafl" in p and "tafl_wcs" not in p for p in sys.path)
        #     }
        
        # Check if enhanced modules are available
        if execution_mode == "real" and not ENHANCED_MODULES_AVAILABLE:
            print(f"[DEBUG] ==> Real mode but no modules, returning error", file=sys.stderr)
            return {
                "success": False,
                "message": "Enhanced modules not available. Please ensure TAFL_WCS is built.",
                "mode": "simulation_fallback"
            }
        
        # Decision point
        use_simulation = execution_mode == "simulation" or not ENHANCED_MODULES_AVAILABLE
        print(f"[DEBUG] ==> Decision: mode={execution_mode}, modules={ENHANCED_MODULES_AVAILABLE}, use_sim={use_simulation}", file=sys.stderr)
        
        if use_simulation:
            # Simulation mode
            print(f"[DEBUG] ==> USING SIMULATION MODE", file=sys.stderr)
            return await execute_simulation(flow_content, flow_id)
        
        # Real execution using direct integration
        print(f"[DEBUG] ==> USING REAL EXECUTION MODE", file=sys.stderr)
        return await execute_real_direct(flow_content, flow_id, metadata, variables, settings, preload, rules)
        
    except Exception as e:
        logger.error(f"Execution failed: {str(e)}")
        return {
            "success": False,
            "message": f"Execution failed: {str(e)}"
        }

async def execute_simulation(flow_content: List, flow_id: str) -> Dict:
    """Execute in simulation mode (unchanged)"""
    logger.info(f"⚠️ SIMULATION MODE CALLED for flow {flow_id}")
    logger.info(f"  ENHANCED_MODULES_AVAILABLE={ENHANCED_MODULES_AVAILABLE}")
    execution_log = []
    
    for i, statement in enumerate(flow_content):
        for verb in TAFL_VERBS.keys():
            if verb in statement:
                execution_log.append({
                    "step": i + 1,
                    "verb": verb,
                    "status": "simulated",
                    "message": f"Would execute {verb}: {statement[verb]}",
                    "timestamp": datetime.now().isoformat()
                })
                
                # Send progress
                await manager.send_progress(flow_id, {
                    "type": "progress",
                    "step": i + 1,
                    "total": len(flow_content),
                    "verb": verb,
                    "status": "simulated"
                })
                
                await asyncio.sleep(0.2)
                break
    
    return {
        "success": True,
        "flow_id": flow_id,
        "message": "Flow executed successfully (simulation)",
        "execution_log": execution_log,
        "mode": "simulation"
    }

async def execute_real_direct(flow_content: List, flow_id: str, metadata: Dict, variables: Dict, 
                               settings: Dict = None, preload: Dict = None, rules: Dict = None) -> Dict:
    """
    Execute using complete TAFLExecutorWrapper for full 4-phase execution
    This ensures complete compatibility with actual tafl_wcs execution
    """
    try:
        # Build complete YAML structure for TAFL v1.1 4-phase execution
        flow_yaml = {
            'metadata': metadata,
            'settings': settings or metadata.get('settings', {
                'timeout': 30,
                'log_level': 'INFO'
            }),
            'preload': preload or metadata.get('preload', {}),
            'rules': rules or metadata.get('rules', {}),
            'variables': variables,  # Use variables from top level
            'flow': flow_content
        }
        
        # Convert to YAML string for executor
        import yaml
        flow_yaml_str = yaml.dump(flow_yaml, default_flow_style=False, allow_unicode=True)
        
        logger.info(f"Executing flow {flow_id} with TAFLExecutorWrapper")
        logger.debug(f"Variables: {variables}")
        
        # Initialize TAFLExecutorWrapper with database connection
        DATABASE_URL = "postgresql://agvc:password@192.168.100.254:5432/agvc"
        executor = TAFLExecutorWrapper(DATABASE_URL)
        
        # Configure executor with Phase 2 enhancements
        executor.enable_rollback = True
        executor.enable_stack_trace = True
        executor.max_retry_attempts = 3
        
        # Execute using the complete executor (includes 4-phase execution)
        result = await executor.execute_flow(flow_yaml_str, flow_id)
        
        # Return the execution result
        return result
        
    except Exception as e:
        import traceback
        error_details = traceback.format_exc()
        logger.error(f"Real execution failed: {e}")
        logger.error(f"Traceback:\n{error_details}")
        
        # Return error details instead of falling back silently
        return {
            "success": False,
            "message": f"Real execution failed: {str(e)}",
            "error_details": error_details,
            "mode": "real_failed",
            "fallback_available": True
        }

@router.post("/validate")
async def validate_tafl_flow(request: Request):
    """Validate TAFL flow syntax"""
    try:
        data = await request.json()
        flow_content = data.get("flow", [])
        
        errors = []
        warnings = []
        
        # Basic validation
        for i, statement in enumerate(flow_content):
            has_verb = False
            for verb in TAFL_VERBS.keys():
                if verb in statement:
                    has_verb = True
                    verb_data = statement[verb]
                    
                    # Check required parameters
                    if verb == "query" and isinstance(verb_data, dict) and "target" not in verb_data:
                        errors.append(f"Step {i+1}: 'query' requires 'target'")
                    elif verb == "create" and isinstance(verb_data, dict) and "target" not in verb_data:
                        errors.append(f"Step {i+1}: 'create' requires 'target'")
                    elif verb == "update" and isinstance(verb_data, dict):
                        if "target" not in verb_data or "id" not in verb_data:
                            errors.append(f"Step {i+1}: 'update' requires 'target' and 'id'")
                    break
            
            if not has_verb:
                errors.append(f"Step {i+1}: No recognized verb")
        
        # Use enhanced parser if available
        if ENHANCED_MODULES_AVAILABLE:
            try:
                parser = TAFLParser()
                # Additional validation using parser
                tafl_doc = {"flow": flow_content}
                ast = parser.parse_string(yaml.dump(tafl_doc))
                # Parser validation passed
            except Exception as e:
                errors.append(f"Parser validation: {str(e)}")
        
        return {
            "valid": len(errors) == 0,
            "errors": errors,
            "warnings": warnings,
            "statement_count": len(flow_content),
            "enhanced_validation": ENHANCED_MODULES_AVAILABLE
        }
        
    except Exception as e:
        return {
            "valid": False,
            "errors": [f"Validation failed: {str(e)}"],
            "warnings": []
        }

@router.websocket("/ws/execution/{flow_id}")
async def execution_websocket(websocket: WebSocket, flow_id: str):
    """WebSocket for real-time execution updates"""
    await manager.connect(websocket, flow_id)
    
    try:
        await websocket.send_json({
            "type": "connected",
            "flow_id": flow_id,
            "timestamp": datetime.now().isoformat()
        })
        
        while True:
            data = await websocket.receive_text()
            if data == "ping":
                await websocket.send_json({"type": "pong"})
                
    except WebSocketDisconnect:
        manager.disconnect(flow_id)

@router.get("/status")
async def get_integration_status():
    """Check integration status"""
    # Re-check modules at runtime
    check_and_import_modules()
    
    return {
        "enhanced_modules": ENHANCED_MODULES_AVAILABLE,
        "database": "connected" if ENHANCED_MODULES_AVAILABLE else "not_available",
        "execution_modes": ["simulation", "real"] if ENHANCED_MODULES_AVAILABLE else ["simulation"],
        "phase2_features": {
            "rollback": ENHANCED_MODULES_AVAILABLE,
            "stack_trace": ENHANCED_MODULES_AVAILABLE,
            "metrics": ENHANCED_MODULES_AVAILABLE,
            "history": ENHANCED_MODULES_AVAILABLE
        },
        "message": "Direct integration ready" if ENHANCED_MODULES_AVAILABLE else "Build TAFL_WCS first"
    }

@router.get("/test-db")
async def test_database_direct():
    """Test direct database connection bypassing execution logic"""
    # Re-check modules at runtime
    check_and_import_modules()
    
    result = {
        "enhanced_modules": ENHANCED_MODULES_AVAILABLE,
        "test_time": datetime.now().isoformat()
    }
    
    if ENHANCED_MODULES_AVAILABLE:
        try:
            # Try to query database directly
            DATABASE_URL = "postgresql://agvc:password@192.168.100.254:5432/agvc"
            db_bridge = TAFLDatabaseBridge(DATABASE_URL)
            
            # Test query
            locations = db_bridge.query_locations(limit=2)
            result["database_test"] = "success"
            result["query_result"] = locations
            result["result_type"] = type(locations).__name__
            # Check if it's a list or something else
            if isinstance(locations, list) and len(locations) > 0:
                result["location_count"] = len(locations)
                result["first_location"] = locations[0]
            elif isinstance(locations, int):
                result["location_count"] = locations
                result["note"] = "query_locations returned count, not data"
            else:
                result["location_count"] = 0
                result["note"] = f"Unexpected return type: {type(locations)}"
            
        except Exception as e:
            result["database_test"] = "failed"
            result["error"] = str(e)
            import traceback
            result["traceback"] = traceback.format_exc()
    else:
        result["database_test"] = "skipped"
        result["reason"] = "Enhanced modules not available"
    
    return result

# Export for agvcui integration
__all__ = ['router']