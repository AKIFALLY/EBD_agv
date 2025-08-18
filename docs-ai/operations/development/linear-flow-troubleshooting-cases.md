# Linear Flow Troubleshooting Cases

## 🎯 Use Cases
- Learning from real-world Linear Flow Designer issues
- Understanding common pitfalls and solutions
- Debugging complex flow execution problems
- Applying lessons learned to future development

## 📋 Case Study Overview

This document captures real-world troubleshooting cases from Linear Flow Designer development, providing valuable insights for future debugging and development efforts.

## 🚨 Case 1: Rack Rotation Task Over-Creation

### Problem Description
**Symptom**: System creating rotation tasks for all 15 racks instead of only the 2-3 that actually needed rotation.

**Context**: Implementing rack rotation logic for racks that completed side A processing and needed to rotate for side B processing.

### Root Cause Analysis
1. **Logical Operator Misunderstanding**: The skip_if condition wasn't properly evaluating complex logical expressions
2. **Missing Operator Support**: System didn't support `||` (OR) and `&&` (AND) operators
3. **Condition Evaluation**: All conditions were being evaluated as simple equality checks

### Solution Implementation
```python
def evaluate_condition(self, condition: str) -> bool:
    """Enhanced condition evaluation with logical operators"""
    # Handle OR operator
    if '||' in condition:
        parts = condition.split('||')
        return any(self.evaluate_single_condition(p.strip()) for p in parts)
    
    # Handle AND operator
    if '&&' in condition:
        parts = condition.split('&&')
        return all(self.evaluate_single_condition(p.strip()) for p in parts)
    
    # Handle NOT operator
    if condition.startswith('!'):
        return not self.evaluate_single_condition(condition[1:].strip())
    
    return self.evaluate_single_condition(condition)
```

### Verification
- **Before**: 15 tasks created
- **After**: 2 tasks created (only Rack 101 and 105)
- **Test**: Confirmed only racks with `side_a_completed=true` and `side_b_completed=false` created tasks

## 🚨 Case 2: Work ID vs Work Code Confusion

### Problem Description
**Symptom**: Tasks being created with wrong work reference, causing database foreign key violations.

**User Feedback**: "什麼時候改的?! 我需要的就是 work.id = task.work_id 不是work_code"

### Root Cause Analysis
1. **Incorrect Field Usage**: System was using `work.work_code` (string) instead of `work.id` (integer)
2. **Database Schema Mismatch**: Task table expects integer work_id as foreign key
3. **Unnecessary Lookup**: Code was doing extra database query to find work by code

### Solution Implementation
```python
def create_task(self, params: Dict) -> str:
    # BEFORE (Wrong)
    # work_code = params.get('work_code')
    # work = session.query(Work).filter_by(work_code=work_code).first()
    # task.work_id = work.id
    
    # AFTER (Correct)
    work_id = params.get('work_id')  # Direct integer ID
    task = Task(
        work_id=int(work_id),  # Use integer directly
        # ... other fields
    )
```

### Lesson Learned
- Always verify database schema expectations
- Use direct IDs when available, avoid unnecessary lookups
- Integer foreign keys should receive integers, not strings

## 🚨 Case 3: Variable Scoping in Nested Foreach

### Problem Description
**Symptom**: Inner foreach loop variables overwriting outer loop variables, causing incorrect data access.

**Context**: Processing locations with multiple racks, nested foreach loops losing parent context.

### Root Cause Analysis
1. **Shared Context**: All foreach loops sharing same context dictionary
2. **Variable Overwrite**: Inner loop's `location` variable overwriting outer loop's
3. **No Scope Isolation**: Missing context stack implementation

### Solution Implementation
```python
def _execute_foreach(self, step: Dict) -> Any:
    """Execute foreach with proper variable scoping"""
    # Save current context (push to stack)
    saved_context = self.context.copy()
    
    try:
        collection = self._resolve_value(step['collection'])
        var_name = step.get('var', 'item')
        
        for item in collection:
            # Set loop variable in current context
            self.context[var_name] = item
            
            # Execute loop body
            for body_step in step.get('body', []):
                self._execute_step(body_step)
    
    finally:
        # Restore original context (pop from stack)
        self.context = saved_context
```

### Impact
- Nested loops now maintain proper variable isolation
- Parent scope variables remain accessible
- No pollution between loop iterations

## 🚨 Case 4: Mathematical Expression Resolution

### Problem Description
**Symptom**: System couldn't evaluate expressions like `${location.node_id + 1}` for calculating navigation points.

**Context**: KUKA AGV needed turning point calculation for rack rotation (node_id + 1).

### Root Cause Analysis
1. **No Math Support**: Expression resolver only handled simple variable substitution
2. **String Concatenation**: System treating `+ 1` as string instead of math operation
3. **Evaluation Safety**: No safe way to evaluate mathematical expressions

### Solution Implementation
```python
def _resolve_complex_expression(self, expr: str) -> Any:
    """Resolve expressions with math operations"""
    if '+' in expr or '-' in expr or '*' in expr or '/' in expr:
        # Split by operators
        parts = re.split(r'(\+|\-|\*|/)', expr)
        resolved_parts = []
        
        for part in parts:
            part = part.strip()
            if part in ['+', '-', '*', '/']:
                resolved_parts.append(part)
            elif part.startswith('${') and part.endswith('}'):
                # Resolve variable
                var_path = part[2:-1]
                value = self._resolve_variable_path(var_path)
                resolved_parts.append(str(value))
            else:
                resolved_parts.append(part)
        
        # Safely evaluate
        resolved_expr = ''.join(resolved_parts)
        return eval(resolved_expr)
```

### Use Case Example
```yaml
metadata:
  nodes: 
    - "${location.node_id}"        # 10001
    - "${location.node_id + 1}"    # 10002 (calculated)
    - "${location.node_id}"        # 10001
```

## 🚨 Case 5: Parameter Merging Strategy

### Problem Description
**Symptom**: Tasks missing important configuration from Work parameters when metadata was provided.

**Context**: Need to preserve Work-level default parameters while allowing task-specific overrides.

### Root Cause Analysis
1. **Parameter Override**: Metadata completely replacing Work parameters
2. **Lost Configuration**: Important Work settings being discarded
3. **No Merge Strategy**: Missing logic to combine parameter sources

### Solution Implementation
```python
def create_task(self, params: Dict) -> str:
    work_id = params.get('work_id')
    work = session.query(Work).filter_by(id=work_id).first()
    
    # Start with work parameters as base
    task_parameters = {}
    if work and work.parameters:
        task_parameters.update(work.parameters)
    
    # Override/extend with metadata
    metadata = params.get('metadata')
    if metadata:
        task_parameters.update(metadata)
    
    # Create task with merged parameters
    task = Task(
        parameters=task_parameters,
        # ... other fields
    )
```

### Result
- Work default parameters preserved
- Task-specific metadata can override
- All required parameters included

## 💡 Best Practices Learned

### Variable Resolution
1. **Resolve Before Use**: Always resolve variables before evaluation
2. **Handle Null Values**: Check for None/undefined before operations
3. **Type Consistency**: Ensure resolved values have expected types
4. **Deep Property Access**: Support nested object property access

### Condition Evaluation
1. **Operator Support**: Implement full logical operator support (||, &&, !)
2. **Precedence Rules**: Evaluate in correct order (NOT → AND → OR)
3. **Short Circuit**: Optimize with short-circuit evaluation
4. **Clear Syntax**: Use consistent operator syntax

### Parameter Management
1. **Merge Strategy**: Define clear parameter inheritance rules
2. **Document Sources**: Track where each parameter originated
3. **Validation**: Verify all required parameters present
4. **Type Checking**: Ensure parameter types match expectations

### Database Integration
1. **Schema First**: Always check database schema requirements
2. **Foreign Keys**: Use correct types for foreign key references
3. **Transaction Safety**: Wrap multi-step operations in transactions
4. **Error Handling**: Provide clear error messages for violations

## 🔧 Debugging Techniques

### Logging Strategy
```python
# Add strategic logging
self.logger.debug(f"Evaluating condition: {condition}")
self.logger.debug(f"Resolved expression: {resolved_expr}")
self.logger.debug(f"Merged parameters: {task_parameters}")
```

### Test Data Creation
```python
# Create minimal test cases
test_data = {
    "locations": [
        {"id": 1, "node_id": 10001, "room_id": 1},
    ],
    "racks": [
        {"id": 101, "side_a_completed": True, "side_b_completed": False},
    ]
}
```

### Validation Scripts
```python
# Verify expected behavior
def verify_task_creation(task):
    assert task.work_id == 220001  # Integer
    assert "model" in task.parameters
    assert task.parameters["model"] == "KUKA400i"
    assert len(task.parameters["nodes"]) == 3
```

## 🔗 Cross References
- Linear Flow Advanced Features: @docs-ai/knowledge/system/linear-flow-advanced-features.md
- Flow WCS System: @docs-ai/knowledge/system/flow-wcs-system.md
- KUKA AGV Integration: @docs-ai/knowledge/protocols/kuka-agv-rack-rotation.md
- Core Development Principles: @docs-ai/operations/development/core-principles.md