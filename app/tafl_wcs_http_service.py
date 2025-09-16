#!/usr/bin/env python3
"""
TAFL_WCS HTTP Service Wrapper
Provides HTTP interface for TAFL execution service
Phase 3 - System Integration
"""

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import asyncio
import yaml
import json
import logging
from datetime import datetime
from typing import Dict, Any, Optional, List
import os
import sys

# Add TAFL_WCS to path
sys.path.insert(0, '/app/tafl_wcs_ws/src/tafl_wcs')
sys.path.insert(0, '/app/tafl_ws/src/tafl')

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Create FastAPI app
app = FastAPI(
    title="TAFL_WCS HTTP Service",
    description="HTTP interface for TAFL execution with enhanced features",
    version="1.0.0"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Request/Response models
class ExecutionRequest(BaseModel):
    flow_id: str
    flow_content: str
    mode: str = "real"
    enable_progress: bool = True
    enable_rollback: bool = True
    max_retry: int = 3

class ExecutionResponse(BaseModel):
    flow_id: str
    status: str
    message: str
    result: Optional[Dict] = None
    execution_time: Optional[float] = None
    execution_log: List[Dict] = []
    metrics: Optional[Dict] = None

class ExecutionStatus(BaseModel):
    flow_id: str
    status: str
    progress: int
    total_steps: int
    current_step: Optional[str] = None
    message: Optional[str] = None

# Global execution manager
class ExecutionManager:
    def __init__(self):
        self.active_executions: Dict[str, Dict] = {}
        self.execution_history: List[Dict] = []
        self.load_history()
    
    def load_history(self):
        """Load execution history from file"""
        history_file = "/tmp/tafl_execution_history.json"
        if os.path.exists(history_file):
            try:
                with open(history_file, 'r') as f:
                    self.execution_history = json.load(f)
            except Exception as e:
                logger.error(f"Failed to load history: {e}")
                self.execution_history = []
    
    def save_history(self):
        """Save execution history to file"""
        history_file = "/tmp/tafl_execution_history.json"
        try:
            with open(history_file, 'w') as f:
                json.dump(self.execution_history[-1000:], f, indent=2)
        except Exception as e:
            logger.error(f"Failed to save history: {e}")
    
    def add_execution(self, flow_id: str, status: Dict):
        """Add execution to active list"""
        self.active_executions[flow_id] = status
    
    def update_execution(self, flow_id: str, updates: Dict):
        """Update execution status"""
        if flow_id in self.active_executions:
            self.active_executions[flow_id].update(updates)
    
    def complete_execution(self, flow_id: str, result: Dict):
        """Mark execution as complete"""
        if flow_id in self.active_executions:
            execution = self.active_executions.pop(flow_id)
            execution.update(result)
            execution['completed_at'] = datetime.now().isoformat()
            
            # Add to history
            self.execution_history.append(execution)
            self.save_history()
    
    def get_status(self, flow_id: str) -> Optional[Dict]:
        """Get execution status"""
        return self.active_executions.get(flow_id)

# Initialize manager
manager = ExecutionManager()

@app.get("/health")
async def health_check():
    """Health check endpoint"""
    return {
        "status": "healthy",
        "service": "TAFL_WCS HTTP Service",
        "timestamp": datetime.now().isoformat()
    }

@app.post("/tafl_wcs/execute", response_model=ExecutionResponse)
async def execute_flow(request: ExecutionRequest):
    """
    Execute TAFL flow with enhanced features
    """
    try:
        # Parse YAML content
        flow_data = yaml.safe_load(request.flow_content)
        
        # Extract flow and metadata
        flow_statements = flow_data.get('flow', [])
        metadata = flow_data.get('metadata', {})
        
        # Initialize execution status
        manager.add_execution(request.flow_id, {
            'flow_id': request.flow_id,
            'status': 'executing',
            'started_at': datetime.now().isoformat(),
            'total_steps': len(flow_statements),
            'current_step': 0,
            'mode': request.mode
        })
        
        # Execute based on mode
        if request.mode == "real":
            result = await execute_real_flow(
                request.flow_id,
                flow_statements,
                metadata,
                request.enable_rollback,
                request.max_retry
            )
        else:
            result = await execute_simulation_flow(
                request.flow_id,
                flow_statements
            )
        
        # Complete execution
        manager.complete_execution(request.flow_id, result)
        
        return ExecutionResponse(
            flow_id=request.flow_id,
            status=result.get('status', 'completed'),
            message=result.get('message', 'Execution completed'),
            result=result.get('result'),
            execution_time=result.get('execution_time'),
            execution_log=result.get('execution_log', []),
            metrics=result.get('metrics')
        )
        
    except yaml.YAMLError as e:
        raise HTTPException(status_code=400, detail=f"Invalid YAML: {str(e)}")
    except Exception as e:
        logger.error(f"Execution failed: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Execution failed: {str(e)}")

async def execute_real_flow(
    flow_id: str,
    flow_statements: List[Dict],
    metadata: Dict,
    enable_rollback: bool,
    max_retry: int
) -> Dict:
    """Execute flow with real backend integration"""
    try:
        # Import enhanced modules
        from tafl_wcs.tafl_executor_wrapper import TAFLExecutorWrapper
        from tafl_wcs.tafl_db_bridge import TAFLDatabaseBridge
        from tafl.parser import TAFLParser
        
        # Initialize components
        db_bridge = TAFLDatabaseBridge()
        executor = TAFLExecutorWrapper()
        parser = TAFLParser()
        
        # Configure executor
        executor.enable_rollback = enable_rollback
        executor.max_retry_attempts = max_retry
        
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
        for i, statement in enumerate(flow_statements):
            step_start = datetime.now()
            
            # Update progress
            manager.update_execution(flow_id, {
                'current_step': i + 1,
                'progress': int((i + 1) / len(flow_statements) * 100)
            })
            
            try:
                # Parse and execute statement
                result = await executor.execute_statement_dict(statement, context)
                
                execution_time = (datetime.now() - step_start).total_seconds()
                
                execution_log.append({
                    'step': i + 1,
                    'statement': statement,
                    'status': 'completed',
                    'result': result,
                    'execution_time': execution_time,
                    'timestamp': datetime.now().isoformat()
                })
                
            except Exception as e:
                # Handle execution error
                execution_log.append({
                    'step': i + 1,
                    'statement': statement,
                    'status': 'failed',
                    'error': str(e),
                    'timestamp': datetime.now().isoformat()
                })
                
                # Perform rollback if enabled
                if enable_rollback:
                    await executor.perform_rollback(context)
                
                return {
                    'status': 'failed',
                    'message': f'Execution failed at step {i+1}: {str(e)}',
                    'execution_log': execution_log,
                    'execution_time': (datetime.now() - start_time).total_seconds()
                }
        
        # Calculate total execution time
        total_time = (datetime.now() - start_time).total_seconds()
        
        # Get metrics
        metrics = executor.get_metrics()
        
        return {
            'status': 'completed',
            'message': 'Flow executed successfully',
            'result': context.get('variables'),
            'execution_log': execution_log,
            'execution_time': total_time,
            'metrics': metrics
        }
        
    except ImportError as e:
        logger.error(f"Failed to import modules: {e}")
        return {
            'status': 'error',
            'message': f'Module import error: {str(e)}',
            'execution_log': []
        }
    except Exception as e:
        logger.error(f"Execution error: {e}")
        return {
            'status': 'error',
            'message': f'Execution error: {str(e)}',
            'execution_log': []
        }

async def execute_simulation_flow(flow_id: str, flow_statements: List[Dict]) -> Dict:
    """Execute flow in simulation mode"""
    execution_log = []
    start_time = datetime.now()
    
    for i, statement in enumerate(flow_statements):
        # Update progress
        manager.update_execution(flow_id, {
            'current_step': i + 1,
            'progress': int((i + 1) / len(flow_statements) * 100)
        })
        
        # Simulate execution
        await asyncio.sleep(0.5)
        
        execution_log.append({
            'step': i + 1,
            'statement': statement,
            'status': 'simulated',
            'message': f'Simulated execution of: {statement}',
            'timestamp': datetime.now().isoformat()
        })
    
    total_time = (datetime.now() - start_time).total_seconds()
    
    return {
        'status': 'completed',
        'message': 'Flow executed successfully (simulation)',
        'execution_log': execution_log,
        'execution_time': total_time,
        'mode': 'simulation'
    }

@app.get("/tafl_wcs/status/{flow_id}", response_model=ExecutionStatus)
async def get_execution_status(flow_id: str):
    """Get execution status for a flow"""
    status = manager.get_status(flow_id)
    
    if status:
        return ExecutionStatus(
            flow_id=flow_id,
            status=status.get('status', 'unknown'),
            progress=status.get('progress', 0),
            total_steps=status.get('total_steps', 0),
            current_step=str(status.get('current_step', '')),
            message=status.get('message')
        )
    else:
        # Check history
        for execution in reversed(manager.execution_history):
            if execution.get('flow_id') == flow_id:
                return ExecutionStatus(
                    flow_id=flow_id,
                    status='completed',
                    progress=100,
                    total_steps=execution.get('total_steps', 0),
                    message='Execution completed'
                )
        
        raise HTTPException(status_code=404, detail=f"Flow {flow_id} not found")

@app.get("/tafl_wcs/history")
async def get_execution_history(limit: int = 10):
    """Get execution history"""
    history = manager.execution_history[-limit:]
    return {
        'history': history,
        'total': len(manager.execution_history),
        'limit': limit
    }

@app.delete("/tafl_wcs/history")
async def clear_execution_history():
    """Clear execution history"""
    manager.execution_history = []
    manager.save_history()
    return {'message': 'History cleared'}

@app.get("/tafl_wcs/metrics")
async def get_system_metrics():
    """Get system metrics"""
    try:
        from tafl_wcs.tafl_executor_wrapper import TAFLExecutorWrapper
        executor = TAFLExecutorWrapper()
        metrics = executor.get_metrics()
        
        return {
            'metrics': metrics,
            'active_executions': len(manager.active_executions),
            'history_count': len(manager.execution_history)
        }
    except Exception as e:
        return {
            'error': str(e),
            'active_executions': len(manager.active_executions),
            'history_count': len(manager.execution_history)
        }

# Helper class for statement execution
class TAFLExecutorWrapper:
    """Extended wrapper for Phase 3 integration"""
    
    def __init__(self):
        # Import the actual enhanced wrapper
        from tafl_wcs.tafl_executor_wrapper import TAFLExecutorWrapper as BaseWrapper
        self.base_wrapper = BaseWrapper()
    
    async def execute_statement_dict(self, statement: Dict, context: Dict) -> Any:
        """Execute a statement dictionary"""
        # Find the verb
        for verb, params in statement.items():
            if verb in ['set', 'query', 'check', 'create', 'update', 'for', 'if', 'print', 'wait', 'call']:
                return await self.base_wrapper.execute_verb(verb, params, context)
        
        raise ValueError(f"Unknown statement: {statement}")
    
    def __getattr__(self, name):
        """Delegate to base wrapper"""
        return getattr(self.base_wrapper, name)

if __name__ == "__main__":
    import uvicorn
    
    # Run the service
    uvicorn.run(
        app,
        host="0.0.0.0",
        port=9000,
        log_level="info"
    )