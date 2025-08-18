# Linear Flow Advanced Features

## 🎯 Use Cases
- Understanding advanced expression resolution in Linear Flow Designer
- Implementing complex conditional logic in YAML flows
- Managing variable scoping in nested foreach loops
- Merging parameters from multiple sources

## 📋 Core Concepts

### Expression Resolution Enhancement
Linear Flow Designer supports advanced expression resolution including mathematical operations and complex variable references.

### Supported Features
- **Mathematical Operations**: `+`, `-`, `*`, `/` in expressions
- **Logical Operators**: `||` (OR), `&&` (AND), `!` (NOT)
- **Variable References**: `${variable.property}` notation
- **Nested Properties**: Deep object property access
- **Context Stack**: Isolated variable scoping in loops

## 🔧 Implementation Details

### Mathematical Expression Resolution
```python
def _resolve_complex_expression(self, expr: str) -> Any:
    """Resolve expressions with math operations"""
    # Support for expressions like: ${location.node_id + 1}
    if '+' in expr or '-' in expr or '*' in expr or '/' in expr:
        # Split expression by operators
        parts = re.split(r'(\+|\-|\*|/)', expr)
        resolved_parts = []
        
        for part in parts:
            part = part.strip()
            if part in ['+', '-', '*', '/']:
                resolved_parts.append(part)
            elif part.startswith('${') and part.endswith('}'):
                # Resolve variable reference
                var_path = part[2:-1]
                value = self._resolve_variable_path(var_path)
                resolved_parts.append(str(value))
            else:
                resolved_parts.append(part)
        
        # Safely evaluate mathematical expression
        resolved_expr = ''.join(resolved_parts)
        return eval(resolved_expr)
```

### Logical Operators in Conditions
```python
def evaluate_condition(self, condition: str) -> bool:
    """Evaluate conditions with logical operators"""
    # Handle OR operator (||)
    if '||' in condition:
        parts = condition.split('||')
        return any(self.evaluate_single_condition(p.strip()) for p in parts)
    
    # Handle AND operator (&&)
    if '&&' in condition:
        parts = condition.split('&&')
        return all(self.evaluate_single_condition(p.strip()) for p in parts)
    
    # Handle NOT operator (!)
    if condition.startswith('!'):
        return not self.evaluate_single_condition(condition[1:].strip())
    
    return self.evaluate_single_condition(condition)
```

### Context Stack for Variable Scoping
```python
def _execute_foreach(self, step: Dict) -> Any:
    """Execute foreach with proper variable scoping"""
    # Save current context (push to stack)
    saved_context = self.context.copy()
    
    try:
        collection = self._resolve_value(step['collection'])
        var_name = step.get('var', 'item')
        
        for item in collection:
            # Set loop variable in isolated scope
            self.context[var_name] = item
            
            # Execute loop body
            for body_step in step.get('body', []):
                result = self._execute_step(body_step)
                if result is not None and body_step.get('return_on_result'):
                    return result
    
    finally:
        # Restore original context (pop from stack)
        self.context = saved_context
```

## 💡 Best Practices

### Expression Writing
1. **Use Clear Variable Names**: Make expressions self-documenting
2. **Validate Math Operations**: Ensure operands are numeric
3. **Handle Edge Cases**: Consider null/undefined values
4. **Test Complex Expressions**: Verify results with test data

### Conditional Logic
1. **Operator Precedence**: Use parentheses for clarity when mixing operators
2. **Short-Circuit Evaluation**: Place most likely conditions first
3. **Readable Conditions**: Break complex conditions into named variables
4. **Consistent Style**: Use consistent operator spacing

### Variable Scoping
1. **Minimize Scope**: Keep variables in smallest necessary scope
2. **Avoid Name Conflicts**: Use descriptive variable names in loops
3. **Document Dependencies**: Clear comments for context requirements
4. **Test Nested Loops**: Verify proper variable isolation

## 📊 Real Examples

### Math Expression in YAML Flow
```yaml
- id: "calculate_turning_point"
  exec: "task.create_task"
  params:
    metadata:
      nodes: 
        - "${location.node_id}"           # Start point
        - "${location.node_id + 1}"        # Turning point (calculated)
        - "${location.node_id}"           # End point (same as start)
```

### Complex Conditional Logic
```yaml
- id: "check_rack_rotation_needed"
  exec: "flow.set_variable"
  params:
    name: "rack_needs_rotation"
    value: "${rack.side_a_completed && !rack.side_b_completed || rack.requires_flip}"
  skip_if: "!${rack.is_active} || ${rack.is_locked}"
```

### Nested Foreach with Scoping
```yaml
- id: "process_locations"
  type: "foreach"
  collection: "${locations}"
  var: "location"
  body:
    - id: "process_racks"
      type: "foreach"
      collection: "${location.racks}"
      var: "rack"
      body:
        - id: "create_task"
          exec: "task.create_task"
          params:
            location_id: "${location.id}"  # Parent scope variable
            rack_id: "${rack.id}"          # Current loop variable
```

## 🚨 Common Issues and Solutions

### Issue: Variable Overwrites in Nested Loops
**Problem**: Inner loop variables overwriting outer loop variables
**Solution**: Implemented context stack to isolate each loop's scope

### Issue: Math Expression Parsing Errors
**Problem**: Direct evaluation of unresolved variables
**Solution**: Resolve all variables before mathematical evaluation

### Issue: Logical Operator Precedence
**Problem**: Unexpected evaluation order with mixed operators
**Solution**: Evaluate operators in correct precedence: NOT → AND → OR

### Issue: Null Value in Expressions
**Problem**: NoneType errors when variables are undefined
**Solution**: Add null checks and default values in resolution

## 🔗 Cross References
- Flow WCS System: @docs-ai/knowledge/system/flow-wcs-system.md
- Flow Functions: @docs-ai/knowledge/system/flow-wcs-function-system.md
- YAML DSL Design: @docs-ai/operations/development/yaml-dsl-design-plan.md
- Troubleshooting Cases: @docs-ai/operations/development/linear-flow-troubleshooting-cases.md