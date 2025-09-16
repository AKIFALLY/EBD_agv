#!/usr/bin/env python3
"""
Enhanced TAFL Editor API with Real Execution Support
Phase 3 - System Integration
"""

from fastapi import APIRouter, Request, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.responses import JSONResponse
import yaml
import json
import httpx
import asyncio
from datetime import datetime
from typing import Dict, Any, Optional, List
import logging

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

router = APIRouter(prefix="/tafl", tags=["tafl_editor"])

# Connection manager for WebSocket
class ConnectionManager:
    def __init__(self):
        self.active_connections: Dict[str, List[WebSocket]] = {}
    
    async def connect(self, websocket: WebSocket, flow_id: str):
        await websocket.accept()
        if flow_id not in self.active_connections:
            self.active_connections[flow_id] = []
        self.active_connections[flow_id].append(websocket)
    
    def disconnect(self, websocket: WebSocket, flow_id: str):
        if flow_id in self.active_connections:
            self.active_connections[flow_id].remove(websocket)
            if not self.active_connections[flow_id]:
                del self.active_connections[flow_id]
    
    async def send_progress(self, flow_id: str, message: Dict):
        if flow_id in self.active_connections:
            for connection in self.active_connections[flow_id]:
                try:
                    await connection.send_json(message)
                except:
                    pass

manager = ConnectionManager()

# TAFL Execution Service Configuration
TAFL_WCS_SERVICE_URL = "http://localhost:9000"  # TAFL_WCS HTTP service
EXECUTION_TIMEOUT = 60.0  # seconds

# Enhanced TAFL Verbs with execution support
TAFL_VERBS = {
    "set": {
        "description": "Set a variable to a value",
        "params": ["variable", "value"],
        "execution": "supported"
    },
    "query": {
        "description": "Query data from database",
        "params": ["target", "conditions"],
        "execution": "supported"
    },
    "check": {
        "description": "Check a condition",
        "params": ["condition"],
        "execution": "supported"
    },
    "create": {
        "description": "Create a new task or entity",
        "params": ["target", "properties"],
        "execution": "supported"
    },
    "update": {
        "description": "Update an existing entity",
        "params": ["target", "id", "properties"],
        "execution": "supported"
    },
    "for": {
        "description": "Loop through items",
        "params": ["each", "in", "do"],
        "execution": "supported"
    },
    "if": {
        "description": "Conditional execution",
        "params": ["condition", "then", "else"],
        "execution": "supported"
    },
    "print": {
        "description": "Output message to log",
        "params": ["message"],
        "execution": "supported"
    },
    "wait": {
        "description": "Wait for a duration",
        "params": ["duration", "unit"],
        "execution": "supported"
    },
    "call": {
        "description": "Call an external service",
        "params": ["service", "method", "params"],
        "execution": "supported"
    }
}

@router.get("/verbs", response_class=JSONResponse)
async def get_supported_verbs():
    """Get list of supported TAFL verbs with execution status"""
    return {"verbs": TAFL_VERBS}

@router.post("/execute", response_class=JSONResponse)
async def execute_tafl_flow(request: Request):
    """
    Execute TAFL flow with real backend integration
    Phase 3 Enhancement - Real Execution Mode
    """
    try:
        data = await request.json()
        flow_content = data.get("flow", [])
        metadata = data.get("metadata", {})
        flow_id = metadata.get("id", f"flow_{datetime.now().strftime('%Y%m%d_%H%M%S')}")
        execution_mode = data.get("mode", "real")  # "real" or "simulation"
        
        logger.info(f"Executing flow {flow_id} in {execution_mode} mode")
        
        if execution_mode == "simulation":
            # Keep simulation mode for testing
            return await execute_simulation(flow_content, flow_id)
        
        # Real execution through TAFL_WCS service
        execution_request = {
            "flow_id": flow_id,
            "flow_content": yaml.dump({"flow": flow_content, "metadata": metadata}),
            "mode": "real",
            "enable_progress": True
        }
        
        # Call TAFL_WCS execution service
        async with httpx.AsyncClient() as client:
            try:
                response = await client.post(
                    f"{TAFL_WCS_SERVICE_URL}/tafl_wcs/execute",
                    json=execution_request,
                    timeout=EXECUTION_TIMEOUT
                )
                
                if response.status_code == 200:
                    result = response.json()
                    return {
                        "success": True,
                        "flow_id": flow_id,
                        "message": "Flow executed successfully",
                        "result": result.get("result"),
                        "execution_time": result.get("execution_time"),
                        "execution_log": result.get("execution_log", [])
                    }
                else:
                    return {
                        "success": False,
                        "flow_id": flow_id,
                        "message": f"Execution failed with status {response.status_code}",
                        "error": response.text
                    }
                    
            except httpx.TimeoutException:
                return {
                    "success": False,
                    "flow_id": flow_id,
                    "message": "Execution timeout exceeded",
                    "timeout": EXECUTION_TIMEOUT
                }
            except httpx.ConnectError:
                # If TAFL_WCS service is not available, fall back to local execution
                logger.warning("TAFL_WCS service not available, using local execution")
                return await execute_local(flow_content, flow_id)
                
    except Exception as e:
        logger.error(f"Execution failed: {str(e)}")
        return {
            "success": False,
            "message": f"Execution failed: {str(e)}"
        }

async def execute_simulation(flow_content: List, flow_id: str) -> Dict:
    """Execute flow in simulation mode"""
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
                
                # Send progress via WebSocket
                await manager.send_progress(flow_id, {
                    "type": "progress",
                    "step": i + 1,
                    "total": len(flow_content),
                    "verb": verb,
                    "status": "simulated"
                })
                
                # Simulate execution delay
                await asyncio.sleep(0.5)
                break
    
    return {
        "success": True,
        "flow_id": flow_id,
        "message": "Flow executed successfully (simulation)",
        "execution_log": execution_log,
        "mode": "simulation"
    }

async def execute_local(flow_content: List, flow_id: str) -> Dict:
    """
    Execute flow locally using enhanced TAFL executor
    This connects to our Phase 2 enhanced backend
    """
    try:
        # Import enhanced executor
        import sys
        sys.path.insert(0, '/app/tafl_wcs_ws/src/tafl_wcs')
        from tafl_wcs.tafl_executor_wrapper import TAFLExecutorWrapper
        from tafl_wcs.tafl_db_bridge import TAFLDatabaseBridge
        
        # Initialize components
        db_bridge = TAFLDatabaseBridge()
        executor = TAFLExecutorWrapper()
        
        # Prepare execution context
        context = {
            "flow_id": flow_id,
            "variables": {},
            "db_bridge": db_bridge,
            "execution_log": []
        }
        
        # Execute each statement
        execution_log = []
        for i, statement in enumerate(flow_content):
            step_start = datetime.now()
            
            # Find verb
            verb = None
            for v in TAFL_VERBS.keys():
                if v in statement:
                    verb = v
                    break
            
            if verb:
                try:
                    # Execute through wrapper
                    result = await executor.execute_statement(verb, statement[verb], context)
                    
                    execution_time = (datetime.now() - step_start).total_seconds()
                    
                    log_entry = {
                        "step": i + 1,
                        "verb": verb,
                        "status": "completed",
                        "result": result,
                        "execution_time": execution_time,
                        "timestamp": datetime.now().isoformat()
                    }
                    
                    execution_log.append(log_entry)
                    
                    # Send progress
                    await manager.send_progress(flow_id, {
                        "type": "progress",
                        "step": i + 1,
                        "total": len(flow_content),
                        "verb": verb,
                        "status": "completed",
                        "execution_time": execution_time
                    })
                    
                except Exception as e:
                    # Handle execution error
                    log_entry = {
                        "step": i + 1,
                        "verb": verb,
                        "status": "failed",
                        "error": str(e),
                        "timestamp": datetime.now().isoformat()
                    }
                    
                    execution_log.append(log_entry)
                    
                    # Send error
                    await manager.send_progress(flow_id, {
                        "type": "error",
                        "step": i + 1,
                        "verb": verb,
                        "error": str(e)
                    })
                    
                    # Rollback if enabled
                    if executor.enable_rollback:
                        await executor.perform_rollback(context)
                    
                    break
        
        # Save execution history
        executor.save_execution_history(flow_id, execution_log)
        
        return {
            "success": True,
            "flow_id": flow_id,
            "message": "Flow executed successfully (local)",
            "execution_log": execution_log,
            "mode": "local",
            "metrics": executor.get_metrics()
        }
        
    except ImportError as e:
        logger.error(f"Failed to import enhanced modules: {e}")
        return {
            "success": False,
            "flow_id": flow_id,
            "message": "Enhanced modules not available",
            "error": str(e)
        }
    except Exception as e:
        logger.error(f"Local execution failed: {e}")
        return {
            "success": False,
            "flow_id": flow_id,
            "message": f"Local execution failed: {str(e)}"
        }

@router.websocket("/ws/execution/{flow_id}")
async def execution_websocket(websocket: WebSocket, flow_id: str):
    """
    WebSocket endpoint for real-time execution progress
    Phase 3 Enhancement - Real-time Updates
    """
    await manager.connect(websocket, flow_id)
    
    try:
        # Send initial connection message
        await websocket.send_json({
            "type": "connected",
            "flow_id": flow_id,
            "timestamp": datetime.now().isoformat()
        })
        
        # Keep connection alive
        while True:
            # Wait for messages from client
            data = await websocket.receive_text()
            
            # Handle client commands
            if data == "ping":
                await websocket.send_json({"type": "pong"})
            elif data == "status":
                # Get execution status
                await websocket.send_json({
                    "type": "status",
                    "flow_id": flow_id,
                    "active": True
                })
                
    except WebSocketDisconnect:
        manager.disconnect(websocket, flow_id)
        logger.info(f"WebSocket disconnected for flow {flow_id}")

@router.get("/execution/status/{flow_id}", response_class=JSONResponse)
async def get_execution_status(flow_id: str):
    """Get execution status for a flow"""
    try:
        # Query execution status from TAFL_WCS
        async with httpx.AsyncClient() as client:
            response = await client.get(
                f"{TAFL_WCS_SERVICE_URL}/tafl_wcs/status/{flow_id}",
                timeout=5.0
            )
            
            if response.status_code == 200:
                return response.json()
            else:
                return {
                    "flow_id": flow_id,
                    "status": "unknown",
                    "message": "Status not available"
                }
                
    except Exception as e:
        return {
            "flow_id": flow_id,
            "status": "error",
            "message": str(e)
        }

@router.get("/execution/history", response_class=JSONResponse)
async def get_execution_history(limit: int = 10):
    """Get execution history"""
    try:
        # Load history from file
        import os
        import json
        
        history_file = "/tmp/tafl_execution_history.json"
        
        if os.path.exists(history_file):
            with open(history_file, 'r') as f:
                history = json.load(f)
                # Return last N entries
                return {
                    "history": history[-limit:],
                    "total": len(history)
                }
        else:
            return {
                "history": [],
                "total": 0
            }
                
    except Exception as e:
        logger.error(f"Failed to load history: {e}")
        return {
            "history": [],
            "error": str(e)
        }

@router.post("/validate", response_class=JSONResponse)
async def validate_tafl_flow(request: Request):
    """Validate TAFL flow syntax and structure"""
    try:
        data = await request.json()
        flow_content = data.get("flow", [])
        
        errors = []
        warnings = []
        
        # Validate each statement
        for i, statement in enumerate(flow_content):
            # Check for recognized verb
            has_verb = False
            for verb in TAFL_VERBS.keys():
                if verb in statement:
                    has_verb = True
                    
                    # Validate required parameters
                    params = TAFL_VERBS[verb].get("params", [])
                    verb_data = statement[verb]
                    
                    # Check critical parameters
                    if verb == "query" and "target" not in verb_data:
                        errors.append(f"Step {i+1}: 'query' requires 'target' parameter")
                    elif verb == "create" and "target" not in verb_data:
                        errors.append(f"Step {i+1}: 'create' requires 'target' parameter")
                    elif verb == "update" and ("target" not in verb_data or "id" not in verb_data):
                        errors.append(f"Step {i+1}: 'update' requires 'target' and 'id' parameters")
                    
                    break
            
            if not has_verb:
                errors.append(f"Step {i+1}: No recognized TAFL verb found")
        
        # Check for unused variables (warning)
        defined_vars = set()
        used_vars = set()
        
        for statement in flow_content:
            if "set" in statement:
                var_name = statement["set"].get("variable")
                if var_name:
                    defined_vars.add(var_name)
            
            # Check variable usage
            for verb, data in statement.items():
                if isinstance(data, dict):
                    for value in data.values():
                        if isinstance(value, str) and value.startswith("$"):
                            used_vars.add(value[1:])
        
        unused = defined_vars - used_vars
        if unused:
            warnings.append(f"Unused variables: {', '.join(unused)}")
        
        undefined = used_vars - defined_vars
        if undefined:
            warnings.append(f"Undefined variables: {', '.join(undefined)}")
        
        return {
            "valid": len(errors) == 0,
            "errors": errors,
            "warnings": warnings,
            "statement_count": len(flow_content)
        }
        
    except Exception as e:
        return {
            "valid": False,
            "errors": [f"Validation failed: {str(e)}"],
            "warnings": []
        }

# Export router for integration
__all__ = ['router']