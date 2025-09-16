"""
TAFL Editor Router
TAFL (Task Automation Flow Language) Visual Editor - Card-based UI for TAFL flow creation

IMPORTANT: Main route is /tafl/editor (NOT /tafl-editor)
- Base prefix: /tafl  
- Editor endpoint: /editor
- Full path: /tafl/editor

This is replacing the Linear Flow Designer and will be the primary flow editor.
"""

import json
import yaml
import asyncio
from pathlib import Path
from typing import List, Dict, Any, Optional
from datetime import datetime
import traceback

from fastapi import APIRouter, Request, HTTPException, Form, UploadFile, File
from fastapi.responses import HTMLResponse, JSONResponse
from fastapi.templating import Jinja2Templates
from pydantic import BaseModel

# Create router - IMPORTANT: Route is /tafl/editor (prefix + endpoint)
router = APIRouter(prefix="/tafl", tags=["tafl_editor"])  # Base: /tafl

# Templates instance (will be set from main app)
_templates = None

# TAFL flows storage directory
TAFL_FLOWS_DIR = Path("/app/config/tafl/flows")
TAFL_FLOWS_DIR.mkdir(parents=True, exist_ok=True)

def clean_card_ids(data):
    """
    Remove editor-generated card IDs while preserving TAFL syntax-required ids.
    Only removes top-level 'id' fields that start with 'card_'.
    Preserves ids within verb parameters like 'where', 'with', etc.
    
    This function recursively processes all nested structures.
    """
    if isinstance(data, list):
        # Process each item in the list
        return [clean_card_ids(item) for item in data]
    elif isinstance(data, dict):
        # Process dictionary
        cleaned = {}
        for key, value in data.items():
            # Skip card IDs at the statement level
            if key == 'id' and isinstance(value, str) and value.startswith('card_'):
                continue  # Skip this card ID
            # Recursively clean all other values
            cleaned[key] = clean_card_ids(value)
        return cleaned
    else:
        # Return other data types as-is (strings, numbers, booleans, etc.)
        return data

def set_templates(tmpl: Jinja2Templates):
    """Set templates instance"""
    global _templates
    _templates = tmpl

def get_router(templates: Jinja2Templates = None) -> APIRouter:
    """Get router instance"""
    if templates:
        set_templates(templates)
    return router

# Pydantic models for request/response
class TAFLFlow(BaseModel):
    """TAFL Flow model"""
    id: str
    name: str
    description: Optional[str] = ""
    version: str = "1.1.2"
    created_at: str
    updated_at: str
    flow: List[Dict[str, Any]]

class TAFLValidationResult(BaseModel):
    """TAFL validation result"""
    valid: bool
    errors: List[str] = []
    warnings: List[str] = []

class TAFLExecutionResult(BaseModel):
    """TAFL execution result"""
    success: bool
    message: str
    result: Optional[Dict[str, Any]] = None

# TAFL verb definitions with metadata - Updated for TAFL v1.1 compliance
# TAFL v1.1.1 compliant verb definitions
TAFL_VERBS = {
    "query": {
        "name": "Query",
        "description": "Query data from database or system",
        "color": "is-info",
        "icon": "fas fa-database",
        "params": ["target", "where", "order", "limit", "as"]  # TAFL v1.1.1: use 'as' not 'store_as'
    },
    "check": {
        "name": "Check", 
        "description": "Check conditions or status",
        "color": "is-success",
        "icon": "fas fa-check-circle",
        "params": ["target", "condition", "as"]  # TAFL v1.1.1: target + condition + as
    },
    "create": {
        "name": "Create",
        "description": "Create new resources or records", 
        "color": "is-primary",
        "icon": "fas fa-plus-circle",
        "params": ["target", "with", "as"]  # TAFL v1.1.1: use 'with' not 'params'
    },
    "update": {
        "name": "Update",
        "description": "Update existing data",
        "color": "is-warning",
        "icon": "fas fa-edit",
        "params": ["target", "where", "set"]  # TAFL v1.1.1: target + where + set
    },
    "if": {
        "name": "If",
        "description": "Conditional execution",
        "color": "is-danger",
        "icon": "fas fa-question-circle",
        "params": ["condition", "then", "else"]
    },
    "for": {
        "name": "For",
        "description": "Loop through collection",
        "color": "is-purple",
        "icon": "fas fa-redo",
        "params": ["in", "as", "do", "filter"]  # TAFL v1.1.1: in + as + do (not 'each')
    },
    "switch": {
        "name": "Switch",
        "description": "Multi-branch conditional",
        "color": "is-orange", 
        "icon": "fas fa-code-branch",
        "params": ["expression", "cases", "default"]  # TAFL v1.1.1: expression + cases array + default
    },
    "set": {
        "name": "Set",
        "description": "Set variable values",
        "color": "is-dark",
        "icon": "fas fa-equals",
        "params": []  # TAFL v1.1.1: Set is a single assignment expression string, not object
    },
    "stop": {
        "name": "Stop",
        "description": "Stop flow execution",
        "color": "is-danger-dark",
        "icon": "fas fa-stop-circle",
        "params": ["reason", "condition"]  # TAFL v1.1.1: reason + optional condition
    },
    "notify": {
        "name": "Notify",
        "description": "Send notifications",
        "color": "is-link",
        "icon": "fas fa-bell",
        "params": ["level", "message", "recipients", "details"]  # TAFL v1.1.1: level is primary field
    }
}

# Main TAFL Editor route - Full path: /tafl/editor
@router.get("/editor", response_class=HTMLResponse)
async def tafl_editor(request: Request):
    """TAFL Editor main page"""
    if not _templates:
        raise HTTPException(status_code=500, detail="Templates not configured")
    
    return _templates.TemplateResponse("tafl_editor.html", {
        "request": request,
        "tafl_verbs": TAFL_VERBS
    })

@router.get("/flows", response_class=JSONResponse) 
async def list_tafl_flows():
    """Get list of all TAFL flows"""
    try:
        flows = []
        for flow_file in TAFL_FLOWS_DIR.glob("*.yaml"):
            try:
                with open(flow_file, 'r', encoding='utf-8') as f:
                    flow_data = yaml.safe_load(f)
                    # Support both TAFL v1.1 format (with metadata) and old format
                    metadata = flow_data.get("metadata", {})
                    flows.append({
                        "id": flow_file.stem,
                        "name": metadata.get("name") or flow_data.get("name", flow_file.stem),
                        "description": metadata.get("description") or flow_data.get("description", ""),
                        "version": metadata.get("version") or flow_data.get("version", "1.0"),
                        "created_at": flow_data.get("created_at", ""),
                        "updated_at": flow_data.get("updated_at", "")
                    })
            except Exception as e:
                print(f"Error reading flow {flow_file}: {e}")
                continue
                
        return {"flows": flows}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error listing flows: {str(e)}")

@router.get("/flows/{flow_id}", response_class=JSONResponse)
async def get_tafl_flow(flow_id: str):
    """Get specific TAFL flow"""
    try:
        flow_file = TAFL_FLOWS_DIR / f"{flow_id}.yaml"
        if not flow_file.exists():
            raise HTTPException(status_code=404, detail="Flow not found")
            
        with open(flow_file, 'r', encoding='utf-8') as f:
            flow_data = yaml.safe_load(f)
            
        return {"flow": flow_data}
    except FileNotFoundError:
        raise HTTPException(status_code=404, detail="Flow not found")
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error loading flow: {str(e)}")

@router.post("/flows", response_class=JSONResponse)
async def save_tafl_flow(request: Request):
    """Save TAFL flow"""
    try:
        data = await request.json()
        
        # Handle both old format (with separate id field) and new TAFL v1.1 format
        if "metadata" in data:
            # TAFL v1.1 format with 6-segment structure
            flow_id = data["metadata"].get("id")
            if not flow_id:
                raise HTTPException(status_code=400, detail="Flow ID required in metadata")
            
            # Clean card IDs from flow section before saving
            cleaned_flow = clean_card_ids(data.get("flow", []))
            
            # Save complete v1.1 structure
            flow_data = {
                "metadata": data.get("metadata", {}),
                "settings": data.get("settings", {}),
                "preload": data.get("preload", {}),
                "rules": data.get("rules", {}),
                "variables": data.get("variables", {}),
                "flow": cleaned_flow
            }
            # Add timestamps
            flow_data["metadata"]["updated_at"] = datetime.now().isoformat()
            if "created_at" not in flow_data["metadata"]:
                flow_data["metadata"]["created_at"] = datetime.now().isoformat()
        else:
            # Old format (backward compatibility)
            flow_id = data.get("id")
            if not flow_id:
                raise HTTPException(status_code=400, detail="Flow ID required")
            
            # Clean card IDs from flow section before saving
            cleaned_flow = clean_card_ids(data.get("flow", []))
            
            # Prepare flow data in old format
            flow_data = {
                "id": flow_id,
                "name": data.get("name", flow_id),
                "description": data.get("description", ""),
                "version": data.get("version", "1.0"),
                "created_at": data.get("created_at", datetime.now().isoformat()),
                "updated_at": datetime.now().isoformat(),
                "flow": cleaned_flow
            }
        
        # Save to file
        flow_file = TAFL_FLOWS_DIR / f"{flow_id}.yaml"
        with open(flow_file, 'w', encoding='utf-8') as f:
            yaml.dump(flow_data, f, default_flow_style=False, allow_unicode=True, sort_keys=False)
            
        return {"success": True, "message": "Flow saved successfully"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error saving flow: {str(e)}")

@router.delete("/flows/{flow_id}", response_class=JSONResponse)
async def delete_tafl_flow(flow_id: str):
    """Delete TAFL flow"""
    try:
        flow_file = TAFL_FLOWS_DIR / f"{flow_id}.yaml"
        if not flow_file.exists():
            raise HTTPException(status_code=404, detail="Flow not found")
            
        flow_file.unlink()
        return {"success": True, "message": "Flow deleted successfully"}
    except FileNotFoundError:
        raise HTTPException(status_code=404, detail="Flow not found")
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error deleting flow: {str(e)}")

@router.post("/validate", response_class=JSONResponse)
async def validate_tafl_flow(request: Request):
    """Validate TAFL flow structure and syntax - TAFL v1.1 compliant"""
    try:
        data = await request.json()
        
        errors = []
        warnings = []
        
        # Handle both v1.1 structure and legacy format
        if "metadata" in data:
            # TAFL v1.1 format - validate 6-segment structure
            # Validate metadata
            metadata = data.get("metadata", {})
            if not metadata.get("id"):
                warnings.append("Metadata should have an 'id' field")
            if not metadata.get("name"):
                warnings.append("Metadata should have a 'name' field")
            
            # Get flow section
            flow = data.get("flow", [])
            
            # Optional: validate preload section
            preload = data.get("preload", {})
            if preload:
                # Check if preload is in the correct format (should be dict for v1.1.1)
                if isinstance(preload, list):
                    errors.append("Preload must be a dictionary (key-value pairs) in TAFL v1.1.1, not a list. Each key should be the variable name, and value should contain a 'query' object.")
                elif isinstance(preload, dict):
                    for key, value in preload.items():
                        if not isinstance(value, dict):
                            errors.append(f"Preload item '{key}' must be an object containing a 'query'")
                        elif "query" not in value:
                            warnings.append(f"Preload item '{key}' should contain a 'query' field")
                        elif not isinstance(value.get("query"), dict):
                            errors.append(f"Preload item '{key}': 'query' must be an object")
                else:
                    errors.append("Preload must be a dictionary (key-value pairs)")
            
            # Optional: validate settings section
            settings = data.get("settings", {})
            if settings and not isinstance(settings, dict):
                errors.append("Settings must be a dictionary (key-value pairs) in TAFL v1.1.1, not a list")
            
            # Optional: validate rules section
            rules = data.get("rules", {})
            if rules and not isinstance(rules, dict):
                errors.append("Rules must be a dictionary (key-value pairs) in TAFL v1.1.1, not a list")
            
            # Optional: validate variables section
            variables = data.get("variables", {})
            if variables and not isinstance(variables, dict):
                errors.append("Variables must be a dictionary (key-value pairs) in TAFL v1.1.1, not a list")
            
        else:
            # Legacy format - just get flow
            flow = data.get("flow", [])
        
        # Validate flow section
        if not isinstance(flow, list):
            errors.append("Flow must be an array of statements")
        else:
            for i, statement in enumerate(flow):
                if not isinstance(statement, dict):
                    errors.append(f"Statement {i+1}: Must be an object")
                    continue
                    
                # Check if statement has a recognized verb
                verbs = [v for v in TAFL_VERBS.keys() if v in statement]
                if not verbs:
                    errors.append(f"Statement {i+1}: No recognized verb found")
                elif len(verbs) > 1:
                    errors.append(f"Statement {i+1}: Multiple verbs not allowed in single statement")
                else:
                    # Verb-specific validation
                    verb = verbs[0]
                    verb_content = statement[verb]
                    
                    # Validate required parameters for certain verbs
                    if verb == "set":
                        # Set can be string (single assignment) or dict (multiple assignments)
                        # But CANNOT be a list
                        if isinstance(verb_content, list):
                            errors.append(f"Statement {i+1}: 'set' must be a string (single assignment) or dictionary (multiple assignments), not a list")
                        elif not isinstance(verb_content, (str, dict)):
                            errors.append(f"Statement {i+1}: 'set' must be a string or dictionary")
                    elif verb == "for" and isinstance(verb_content, dict):
                        if "do" not in verb_content:
                            warnings.append(f"Statement {i+1}: 'for' should have a 'do' section")
                    elif verb == "if" and isinstance(verb_content, dict):
                        if "condition" not in verb_content and not isinstance(verb_content, str):
                            warnings.append(f"Statement {i+1}: 'if' should have a 'condition'")
                    elif verb == "switch" and isinstance(verb_content, dict):
                        if "cases" not in verb_content:
                            warnings.append(f"Statement {i+1}: 'switch' should have 'cases'")
            
        return {
            "valid": len(errors) == 0,
            "errors": errors,
            "warnings": warnings,
            "version": "1.1" if "metadata" in data else "1.0"
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error validating flow: {str(e)}")

@router.post("/convert-to-yaml", response_class=JSONResponse)
async def convert_to_yaml(request: Request):
    """Convert TAFL flow structure to YAML"""
    try:
        data = await request.json()
        
        # Handle TAFL v1.1 format with 6-segment structure
        if "metadata" in data:
            # Clean card IDs from flow section before converting
            cleaned_flow = clean_card_ids(data.get("flow", []))
            
            # Full v1.1 structure
            flow_yaml = {
                "metadata": data.get("metadata", {}),
                "settings": data.get("settings", {}),
                "preload": data.get("preload", {}),
                "rules": data.get("rules", {}),
                "variables": data.get("variables", {}),
                "flow": cleaned_flow
            }
        else:
            # Clean card IDs from flow section before converting
            cleaned_flow = clean_card_ids(data.get("flow", []))
            
            # Old format or partial structure
            flow_yaml = {
                "name": data.get("name", "Untitled Flow"),
                "description": data.get("description", ""),
                "version": data.get("version", "1.0"),
                "flow": cleaned_flow
            }
        
        yaml_content = yaml.dump(flow_yaml, default_flow_style=False, allow_unicode=True, sort_keys=False)
        return {"yaml": yaml_content}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error converting to YAML: {str(e)}")

@router.post("/import-yaml", response_class=JSONResponse)
async def import_yaml(request: Request):
    """Import YAML content as TAFL flow"""
    try:
        data = await request.json()
        yaml_content = data.get("yaml", "")
        
        if not yaml_content:
            raise HTTPException(status_code=400, detail="YAML content required")
            
        # Parse YAML
        flow_data = yaml.safe_load(yaml_content)
        
        if not isinstance(flow_data, dict):
            raise HTTPException(status_code=400, detail="YAML must contain a flow object")
            
        return {"flow": flow_data}
    except yaml.YAMLError as e:
        raise HTTPException(status_code=400, detail=f"Invalid YAML: {str(e)}")
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error importing YAML: {str(e)}")

@router.get("/verbs", response_class=JSONResponse)
async def get_tafl_verbs():
    """Get TAFL verb definitions"""
    return {"verbs": TAFL_VERBS}

@router.post("/testrun", response_class=JSONResponse)
async def test_run_flow_simulation(request: Request):
    """
    Run a TAFL flow in simulation mode
    This simulates the execution without affecting real systems
    """
    try:
        data = await request.json()
        flow_data = data.get("flow", [])
        metadata = data.get("metadata", {})
        variables = data.get("variables", {})
        
        # Simulation execution log
        execution_log = []
        step_number = 0
        
        # Process each card in the flow
        for card in flow_data:
            step_number += 1
            
            # Get the verb from the card
            verb = None
            for key in card:
                if key != "id" and key in TAFL_VERBS:
                    verb = key
                    break
            
            if not verb:
                continue
                
            # Simulate execution based on verb type
            step_result = {
                "step": step_number,
                "verb": verb,
                "card_id": card.get("id", f"card_{step_number}")
            }
            
            # Simulate different verb behaviors
            if verb == "query":
                params = card[verb]
                target = params.get("target", "unknown")
                step_result["action"] = f"Query {target}"
                # Return structured result for query
                step_result["result"] = {
                    "type": "query_result",
                    "target": target,
                    "count": 5,
                    "records": [
                        {"id": f"sim_{i}", "name": f"Simulated {target} {i}"} 
                        for i in range(1, min(6, 3))  # Show first 2 records
                    ],
                    "message": f"Simulated: Found 5 {target} records"
                }
                step_result["status"] = "success"
                
            elif verb == "check":
                params = card[verb]
                condition = params.get("condition", "true")
                step_result["action"] = f"Check condition: {condition}"
                step_result["result"] = "Simulated: Condition evaluated to true"
                step_result["status"] = "success"
                
            elif verb == "set":
                params = card[verb]
                variable = params.get("variable", "var")
                value = params.get("value", "")
                step_result["action"] = f"Set {variable} = {value}"
                step_result["result"] = f"Simulated: Variable '{variable}' set to '{value}'"
                step_result["status"] = "success"
                # Update simulated variables
                variables[variable] = value
                
            elif verb == "create":
                params = card[verb]
                target = params.get("target", "resource")
                step_result["action"] = f"Create {target}"
                step_result["result"] = {
                    "type": "create_result",
                    "target": target,
                    "created_id": "sim_123",
                    "message": f"Simulated: Created {target} with ID sim_123"
                }
                step_result["status"] = "success"
                
            elif verb == "update":
                params = card[verb]
                target = params.get("target", "resource")
                step_result["action"] = f"Update {target}"
                step_result["result"] = f"Simulated: Updated {target}"
                step_result["status"] = "success"
                
            elif verb == "notify":
                params = card[verb]
                message = params.get("message", "notification")
                step_result["action"] = f"Send notification"
                step_result["result"] = f"Simulated: Notification sent - {message}"
                step_result["status"] = "success"
                
            elif verb == "if":
                params = card[verb]
                condition = params.get("condition", "true")
                step_result["action"] = f"If {condition}"
                step_result["result"] = "Simulated: Conditional branch - took 'then' path"
                step_result["status"] = "success"
                
            elif verb == "for":
                params = card[verb]
                each = params.get("each", "item")
                in_collection = params.get("in", "collection")
                step_result["action"] = f"For {each} in {in_collection}"
                step_result["result"] = "Simulated: Loop executed 3 times"
                step_result["status"] = "success"
                
            elif verb == "switch":
                params = card[verb]
                expression = params.get("expression", "$var")
                step_result["action"] = f"Switch on {expression}"
                step_result["result"] = "Simulated: Matched case 1"
                step_result["status"] = "success"
                
            elif verb == "stop":
                params = card[verb]
                reason = params.get("reason", "completed")
                step_result["action"] = "Stop execution"
                step_result["result"] = f"Simulated: Flow stopped - {reason}"
                step_result["status"] = "completed"
                execution_log.append(step_result)
                break
                
            else:
                step_result["action"] = f"Execute {verb}"
                step_result["result"] = f"Simulated: {verb} executed"
                step_result["status"] = "success"
            
            execution_log.append(step_result)
            
            # Simulate processing delay
            await asyncio.sleep(0.1)
        
        return {
            "success": True,
            "mode": "simulation",
            "message": "Flow simulation completed successfully",
            "execution_log": execution_log,
            "final_variables": variables,
            "total_steps": step_number,
            "flow_id": metadata.get("id", "unnamed_flow"),
            "flow_name": metadata.get("name", "Unnamed Flow")
        }
        
    except Exception as e:
        logger.error(f"Simulation error: {str(e)}")
        logger.error(traceback.format_exc())
        return {
            "success": False,
            "mode": "simulation",
            "message": f"Simulation failed: {str(e)}",
            "error": str(e)
        }

# DISABLED: Using enhanced version from tafl_editor_direct.py instead
# @router.post("/execute", response_class=JSONResponse)
async def execute_tafl_flow_disabled(request: Request):
    """Execute TAFL flow (dry run) - DISABLED"""
    try:
        data = await request.json()
        flow = data.get("flow", [])
        
        # This is a simulation - actual execution would integrate with TAFL interpreter
        execution_log = []
        
        for i, statement in enumerate(flow):
            for verb in TAFL_VERBS.keys():
                if verb in statement:
                    execution_log.append({
                        "step": i + 1,
                        "verb": verb,
                        "status": "simulated",
                        "message": f"Would execute {verb}: {statement[verb]}"
                    })
                    break
        
        return {
            "success": True,
            "message": "Flow executed successfully (simulation)",
            "execution_log": execution_log
        }
    except Exception as e:
        return {
            "success": False,
            "message": f"Execution failed: {str(e)}"
        }