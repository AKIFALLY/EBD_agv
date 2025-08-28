# Migration Guide: Linear Flow v2 to TAFL

## Overview

This guide helps you migrate from Linear Flow v2 to TAFL (Task Automation Flow Language).

## Key Differences

### 1. Simplified Structure

**Linear Flow v2:**
```yaml
flow:
  - id: "step_1"
    type: "action"
    exec: "flow.set_variable"
    params:
      name: "counter"
      value: 10
```

**TAFL:**
```yaml
flow:
  - set:
      counter: 10
```

### 2. Core Verbs vs Functions

Linear Flow v2 uses function calls like `flow.set_variable`, `task.create_task`, etc.
TAFL uses 10 core verbs that cover all operations.

### 3. Expression System

**Linear Flow v2:**
```yaml
params:
  value: "${variables.counter + 1}"
```

**TAFL:**
```yaml
- set:
    counter: "${counter} + 1"
```

## Migration Mapping

### Variable Operations

| Linear Flow v2 | TAFL |
|---------------|------|
| `flow.set_variable` | `set` |
| `flow.get_variable` | Direct reference `${var}` |
| `flow.check_condition` | `check` |

### Task Operations

| Linear Flow v2 | TAFL |
|---------------|------|
| `task.create_task` | `create: target: task` |
| `task.update_task` | `update: target: task` |
| `task.query_tasks` | `query: target: tasks` |

### Flow Control

| Linear Flow v2 | TAFL |
|---------------|------|
| `type: "condition"` | `if` |
| `type: "foreach"` | `for` |
| `type: "switch"` | `switch` |

### Database Operations

| Linear Flow v2 | TAFL |
|---------------|------|
| `location.query_locations` | `query: target: locations` |
| `location.update_location` | `update: target: location` |
| `rack.query_racks` | `query: target: racks` |

## Complete Example Migration

### Linear Flow v2 Example
```yaml
flow:
  - id: "query_locations"
    type: "action"
    exec: "location.query_locations"
    params:
      filters:
        room_id: 1
        is_parking_spot: false
    store_result: "locations"

  - id: "process_locations"
    type: "foreach"
    collection: "${locations}"
    var: "location"
    body:
      - id: "check_racks"
        type: "condition"
        condition: "${location.racks.length > 0}"
        true_branch:
          - id: "create_task"
            type: "action"
            exec: "task.create_task"
            params:
              work_id: 220001
              location_id: "${location.id}"
            store_result: "task"
```

### TAFL Equivalent
```yaml
flow:
  - query:
      target: locations
      where:
        room_id: 1
        is_parking_spot: false
      store_as: locations

  - for:
      each: location
      in: "${locations}"
      do:
        - check:
            condition: "${location.racks.length} > 0"
            store_as: has_racks
        
        - if:
            condition: "${has_racks}"
            then:
              - create:
                  target: task
                  params:
                    work_id: 220001
                    location_id: "${location.id}"
                  store_as: task
```

## Migration Steps

### Step 1: Identify Flow Structure
1. Map out your existing Linear Flow v2 structure
2. Identify the main operations (queries, updates, conditions, loops)
3. Group related operations

### Step 2: Convert Variables Section
```yaml
# Linear Flow v2
initial_variables:
  counter: 0
  status: "ready"

# TAFL
variables:
  counter: 0
  status: "ready"
```

### Step 3: Convert Flow Statements

For each Linear Flow v2 statement:
1. Identify the operation type
2. Find the corresponding TAFL verb
3. Convert parameters to TAFL format
4. Simplify expressions

### Step 4: Simplify Expressions

Linear Flow v2 often uses complex property paths.
TAFL simplifies these:

```yaml
# Linear Flow v2
"${variables.locations[0].id}"

# TAFL
"${locations[0].id}"
```

### Step 5: Test and Validate

Use the TAFL validator to check your migrated flow:

```python
from tafl.parser import TAFLParser
from tafl.validator import TAFLValidator

parser = TAFLParser()
validator = TAFLValidator()

program = parser.parse_file("my_flow.yaml")
errors = validator.validate(program)

if errors:
    for error in errors:
        print(f"Validation error: {error}")
else:
    print("Flow is valid!")
```

## Common Patterns

### Pattern 1: Query and Process
```yaml
# Query data and process each item
- query:
    target: items
    where:
      status: pending
    store_as: pending_items

- for:
    each: item
    in: "${pending_items}"
    do:
      - update:
          target: item
          where:
            id: "${item.id}"
          set:
            status: processing
```

### Pattern 2: Conditional Task Creation
```yaml
# Create tasks based on conditions
- check:
    condition: "${location.needs_service}"
    store_as: needs_service

- if:
    condition: "${needs_service}"
    then:
      - create:
          target: task
          params:
            type: service
            location_id: "${location.id}"
```

### Pattern 3: Multi-way Branching
```yaml
# Handle different states
- switch:
    on: "${task.status}"
    cases:
      pending:
        - notify:
            channel: system
            message: "Task pending"
      running:
        - set:
            timeout: 3600
      completed:
        - update:
            target: task
            where:
              id: "${task.id}"
            set:
              archived: true
```

## Troubleshooting

### Issue: Complex Expressions
If your Linear Flow v2 has complex expressions, break them down:

```yaml
# Instead of
- set:
    result: "${(locations.filter(l => l.status == 'available').length > 0) ? 'has_available' : 'none_available'}"

# Use
- query:
    target: locations
    where:
      status: available
    store_as: available_locations

- check:
    condition: "${available_locations.length} > 0"
    store_as: has_available

- if:
    condition: "${has_available}"
    then:
      - set:
          result: "has_available"
    else:
      - set:
          result: "none_available"
```

### Issue: Missing Functions
If a Linear Flow v2 function doesn't have a direct TAFL equivalent, use external functions:

```yaml
# Register in executor
executor.register_function("custom_operation", my_custom_function)

# Use in TAFL
- create:
    target: custom_operation
    params:
      param1: value1
    store_as: result
```

## Benefits After Migration

1. **Simpler Syntax**: Less verbose, more readable
2. **Better Validation**: Type checking at parse time
3. **Clearer Semantics**: 10 verbs are easier to understand
4. **Improved Scoping**: Proper variable scoping in loops
5. **Native Expressions**: No need for complex function calls

## Support

For migration assistance:
1. Check the TAFL specification: `docs/specification.md`
2. Review examples in `examples/` directory
3. Use the validator to check migrated flows
4. Test with the executor before deploying