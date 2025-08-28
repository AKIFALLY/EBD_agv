# TAFL (Task Automation Flow Language) Documentation

## Overview
TAFL v1.0 is a simplified YAML-based Domain Specific Language (DSL) designed to replace Linear Flow v2 in the RosAGV system. It provides a human-readable format for defining automated workflows with 10 core verbs.

## Implementation Status

### ✅ Completed Features
- Core language parser (AST-based)
- Expression evaluation system with `${}` interpolation
- All 10 core verbs implemented
- Variable scoping for loops
- Basic validator
- Comprehensive test suite

### 🔄 In Progress  
- Integration with flow_wcs (without Converter)
- Linear Flow Designer update to generate TAFL

### ⏳ Not Yet Implemented
- Standard library functions (empty, exists, count, sum, avg, etc.)
- Try-catch-finally error handling
- Break/continue flow control
- External function registry integration

## Syntax Support

TAFL supports both simplified and structured syntax formats:

### Simplified Syntax (Specification Style)
```yaml
flow:
  - set: counter = 10
  - set: message = Hello World
  - if: "${counter} > 5"
    then:
      - set: status = large
```

### Structured Syntax (Current Implementation)
```yaml
flow:
  - set:
      counter: 10
  - set:
      message: "Hello World"
  - if:
      condition: "${counter} > 5"
      then:
        - set:
            status: "large"
```

Both formats are fully supported and can be mixed within the same file.

## Core Verbs

### 1. query
Query data from database or external source.
```yaml
- query:
    target: locations
    where:
      status: available
    store_as: available_locations
```

### 2. check
Check condition and store result.
```yaml
- check:
    condition: "${count} > 0"
    store_as: has_items
```

### 3. create
Create new task or resource.
```yaml
- create:
    target: task
    params:
      work_id: 220001
      location_id: "${location.id}"
    store_as: new_task
```

### 4. update
Update existing resource.
```yaml
- update:
    target: location
    where:
      id: "${location_id}"
    set:
      status: occupied
```

### 5. if
Conditional execution.
```yaml
- if:
    condition: "${status} == 'ready'"
    then:
      - set:
          result: "proceed"
    else:
      - set:
          result: "wait"
```

### 6. for
Loop over collection.
```yaml
- for:
    each: item
    in: "${items}"
    do:
      - create:
          target: task
          params:
            item_id: "${item.id}"
```

### 7. switch
Multi-way branching.
```yaml
- switch:
    on: "${status}"
    cases:
      ready:
        - set:
            action: "start"
      busy:
        - set:
            action: "wait"
    default:
      - set:
          action: "error"
```

### 8. set
Set variable value.
```yaml
# Simplified format
- set: counter = 10

# Structured format  
- set:
    counter: 10
```

### 9. stop
Stop execution with optional reason.
```yaml
- stop:
    reason: "Task completed"
```

### 10. notify
Send notification.
```yaml
- notify:
    channel: "system"
    message: "Task ${task_id} completed"
```

## Expression System

### Variable Interpolation
Use `${}` to reference variables:
```yaml
- set:
    message: "Count is ${count}"
- set:
    total: "${price} * ${quantity}"
```

### Supported Operations
- Arithmetic: `+`, `-`, `*`, `/`
- Comparison: `==`, `!=`, `>`, `<`, `>=`, `<=`
- Logical: `and`, `or`, `not`
- Property access: `${object.property}`

## Variable Scoping

Variables in loops are properly scoped:
```yaml
variables:
  outer: "unchanged"

flow:
  - for:
      each: item
      in: [1, 2, 3]
      do:
        - set:
            inner: "${item}"
  # 'item' is not accessible here (proper scoping)
  # 'inner' retains last value (3)
  # 'outer' remains "unchanged"
```

## File Structure
```
tafl_ws/
├── src/
│   └── tafl/
│       ├── tafl/
│       │   ├── __init__.py
│       │   ├── parser.py      # YAML to AST parser
│       │   ├── ast_nodes.py   # AST node definitions
│       │   ├── executor.py    # Execution engine
│       │   └── validator.py   # Validation logic
│       ├── test/
│       │   ├── test_parser.py
│       │   ├── test_executor.py
│       │   └── test_validator.py
│       └── setup.cfg
├── docs/
│   ├── README.md              # This file
│   ├── specification.md       # Language specification
│   └── migration-guide.md     # Migration from Linear Flow v2
└── examples/
    └── *.yaml                 # Example TAFL files
```

## Testing

Run the test suite:
```bash
cd /home/ct/RosAGV/app/tafl_ws/src/tafl
python3 -m pytest test/ -v
```

Current test results: **19 passed, 1 skipped**
- Skipped test: `test_external_functions` (requires flow_wcs integration)

## Next Steps

1. Complete flow_wcs integration (without Converter)
2. Update Linear Flow Designer to generate TAFL format
3. Implement standard library functions
4. Add try-catch-finally support
5. Add break/continue support

## Related Documentation
- Language Specification: `/home/ct/RosAGV/docs-ai/knowledge/system/tafl-language-specification.md`
- Implementation Plan: `/home/ct/RosAGV/docs-ai/operations/development/tafl-implementation-plan.md`