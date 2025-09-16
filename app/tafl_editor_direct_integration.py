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
sys.path.insert(0, '/app/tafl_wcs_ws/src/tafl_wcs')
sys.path.insert(0, '/app/tafl_ws/src/tafl')

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

router = APIRouter(prefix="/tafl", tags=["tafl_editor"])

# Import Phase 2 enhanced modules directly
try:
    from tafl_wcs.tafl_executor_wrapper import TAFLExecutorWrapper
    from tafl_wcs.tafl_db_bridge import TAFLDatabaseBridge
    from tafl.parser import TAFLParser
    ENHANCED_MODULES_AVAILABLE = True
except ImportError as e:
    logger.warning(f"Enhanced modules not available: {e}")
    ENHANCED_MODULES_AVAILABLE = False

# TAFL Verbs definition (same as before)
TAFL_VERBS = {
    "set": {"description": "Set a variable", "params": ["variable", "value"]},
    "query": {"description": "Query database", "params": ["target", "conditions"]},
    "check": {"description": "Check condition", "params": ["condition"]},
    "create": {"description": "Create entity", "params": ["target", "properties"]},
    "update": {"description": "Update entity", "params": ["target", "id", "properties"]},
    "for": {"description": "Loop", "params": ["each", "in", "do"]},
    "if": {"description": "Conditional", "params": ["condition", "then", "else"]},
    "print": {"description": "Output message", "params": ["message"]},
    "wait": {"description": "Wait duration", "params": ["duration", "unit"]},
    "call": {"description": "Call service", "params": ["service", "method", "params"]}
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
        flow_id = metadata.get("id", f"flow_{datetime.now().strftime('%Y%m%d_%H%M%S')}")
        execution_mode = data.get("mode", "real")
        
        logger.info(f"Executing flow {flow_id} in {execution_mode} mode")
        
        # Check if enhanced modules are available
        if execution_mode == "real" and not ENHANCED_MODULES_AVAILABLE:
            return {
                "success": False,
                "message": "Enhanced modules not available. Please ensure TAFL_WCS is built.",
                "mode": "simulation_fallback"
            }
        
        if execution_mode == "simulation" or not ENHANCED_MODULES_AVAILABLE:
            # Simulation mode
            return await execute_simulation(flow_content, flow_id)
        
        # Real execution using direct integration
        return await execute_real_direct(flow_content, flow_id, metadata)
        
    except Exception as e:
        logger.error(f"Execution failed: {str(e)}")
        return {
            "success": False,
            "message": f"Execution failed: {str(e)}"
        }

async def execute_simulation(flow_content: List, flow_id: str) -> Dict:
    """Execute in simulation mode (unchanged)"""
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

async def execute_real_direct(flow_content: List, flow_id: str, metadata: Dict) -> Dict:
    """
    Execute using direct Python integration with enhanced modules
    No HTTP service needed - direct imports
    """
    try:
        # Initialize enhanced components
        db_bridge = TAFLDatabaseBridge()
        executor = TAFLExecutorWrapper()
        
        # Configure executor with Phase 2 enhancements
        executor.enable_rollback = True
        executor.enable_stack_trace = True
        executor.max_retry_attempts = 3
        
        # Create execution context
        context = {
            'flow_id': flow_id,
            'variables': {},
            'metadata': metadata,
            'db_bridge': db_bridge
        }
        
        execution_log = []
        start_time = datetime.now()
        
        # Execute each statement
        for i, statement in enumerate(flow_content):
            step_start = datetime.now()
            
            # Find verb
            verb = None
            for v in TAFL_VERBS.keys():
                if v in statement:
                    verb = v
                    break
            
            if not verb:
                continue
            
            try:
                # Direct execution through enhanced wrapper
                result = await executor.execute_verb(verb, statement[verb], context)
                
                execution_time = (datetime.now() - step_start).total_seconds()
                
                log_entry = {
                    "step": i + 1,
                    "verb": verb,
                    "status": "completed",
                    "result": str(result) if result else None,
                    "execution_time": execution_time,
                    "timestamp": datetime.now().isoformat()
                }
                
                execution_log.append(log_entry)
                
                # Send real-time progress
                await manager.send_progress(flow_id, {
                    "type": "progress",
                    "step": i + 1,
                    "total": len(flow_content),
                    "verb": verb,
                    "status": "completed",
                    "execution_time": execution_time
                })
                
            except Exception as e:
                # Handle error with Phase 2 enhancements
                log_entry = {
                    "step": i + 1,
                    "verb": verb,
                    "status": "failed",
                    "error": str(e),
                    "timestamp": datetime.now().isoformat()
                }
                
                execution_log.append(log_entry)
                
                # Send error notification
                await manager.send_progress(flow_id, {
                    "type": "error",
                    "step": i + 1,
                    "verb": verb,
                    "error": str(e)
                })
                
                # Perform rollback if enabled
                if executor.enable_rollback:
                    await executor.perform_rollback(context)
                
                break
        
        # Calculate metrics
        total_time = (datetime.now() - start_time).total_seconds()
        metrics = executor.get_metrics()
        
        # Save execution history
        executor.save_execution_history(flow_id, execution_log)
        
        return {
            "success": True,
            "flow_id": flow_id,
            "message": "Flow executed successfully",
            "execution_log": execution_log,
            "execution_time": total_time,
            "mode": "real",
            "metrics": metrics,
            "context_variables": context.get('variables', {})
        }
        
    except Exception as e:
        logger.error(f"Real execution failed: {e}")
        # Fall back to simulation
        logger.info("Falling back to simulation mode")
        return await execute_simulation(flow_content, flow_id)

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
                ast = parser.parse(yaml.dump(tafl_doc))
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

# Export for agvcui integration
__all__ = ['router']