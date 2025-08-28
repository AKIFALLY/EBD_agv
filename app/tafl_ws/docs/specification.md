# TAFL Language Specification v1.1

## Language Overview

TAFL (Task Automation Flow Language) is a YAML-based DSL for defining automated workflows in the RosAGV system.

### Design Principles
1. **Simplicity**: Human-readable YAML format
2. **Expressiveness**: 10 core verbs cover all workflow needs
3. **Type Safety**: Strong typing with validation
4. **Extensibility**: Support for external functions
5. **Maintainability**: Clear structure and documentation
6. **Efficiency**: Data preloading and rule-based optimization
7. **Separation of Concerns**: Clear separation between data, rules, and execution logic

## Program Structure

A TAFL program consists of six main sections:

```yaml
metadata:         # Optional: Program metadata
  id: flow_001
  name: Sample Flow
  version: 1.1
  author: System
  description: A sample TAFL flow with preload and rules
  tags: [sample, test, v1.1]

settings:         # Optional: Execution settings
  timeout: 3600
  retry_count: 3
  log_level: debug

preload:          # Optional: Data preloading
  - query:
      target: locations
      where: {room_id: [1, 2, 3]}
      store_as: target_locations
  - query:
      target: racks
      where: {status: "available"}
      store_as: available_racks

rules:            # Optional: Business rules
  rotation_needed:
    condition: "${rack.side_a_completed} and not ${rack.side_b_completed}"
    description: "Check if rack needs rotation"
  target_rooms: [1, 2, 3, 4, 5]
  task_template:
    work_id: 220001
    model: "KUKA400i"

variables:        # Optional: Initial variables
  counter: 0
  items: []
  status: ready

flow:            # Required: Flow statements (simplified with preloaded data)
  - for:
      each: location
      in: ${target_locations}
      filter: ${rules.rotation_needed}
      do:
        - create:
            target: task
            params: ${rules.task_template}
```

## New Sections in v1.1

### Preload Section
The preload section allows you to fetch and cache data before the main flow execution, improving performance and reducing redundant queries.

```yaml
preload:
  - query:
      target: <string>      # Target to query
      where: <dict>         # Optional: Query conditions
      limit: <int>          # Optional: Result limit
      store_as: <string>    # Required: Variable to store results
      comment: <string>     # Optional: Comment
```

**Benefits:**
- **Performance**: Avoid repeated database queries
- **Consistency**: Ensure data consistency throughout execution
- **Clarity**: Separate data loading from business logic

**Example:**
```yaml
preload:
  # Load all target rooms' locations
  - query:
      target: locations
      where: 
        type: "room_inlet"
        room_id: [1, 2, 3, 4, 5]
      store_as: inlet_locations
      comment: "Preload inlet locations for processing"
  
  # Load racks that need rotation
  - query:
      target: racks
      where:
        side_a_completed: true
        side_b_completed: false
      store_as: rotation_racks
      comment: "Preload racks requiring rotation"
```

### Rules Section
The rules section defines business logic, conditions, and configuration values that can be reused throughout the flow.

```yaml
rules:
  # Condition rules
  rule_name:
    condition: <expression>     # Boolean expression
    description: <string>       # Optional: Rule description
  
  # Configuration values
  config_name: <value>          # Any value type
  
  # Complex configurations
  template_name:
    field1: <value>
    field2: <value>
```

**Benefits:**
- **Reusability**: Define once, use multiple times
- **Maintainability**: Centralized business logic
- **Readability**: Clear separation of rules and execution
- **Testing**: Easy to test business rules independently

**Example:**
```yaml
rules:
  # Business condition rules
  rack_needs_rotation:
    condition: "${rack.side_a_completed} and not ${rack.side_b_completed}"
    description: "Determines if a rack needs rotation"
  
  location_is_available:
    condition: "${location.status} == 'available' and ${location.agv_id} != null"
    description: "Checks if location is available for task"
  
  # Configuration values
  target_rooms: [1, 2, 3, 4, 5]
  rotation_angle: 180
  max_tasks_per_location: 3
  
  # Task template
  task_template:
    work_id: 220001
    model: "KUKA400i"
    priority: 5
    metadata:
      rotation_angle: 180
      created_by: "tafl_executor"
```

**Using Rules in Flow:**
```yaml
flow:
  - for:
      each: rack
      in: ${rotation_racks}
      filter: ${rules.rack_needs_rotation}  # Reference rule condition
      do:
        - create:
            target: task
            params: ${rules.task_template}    # Reference rule template
```

## Core Language Elements

### 1. Statements

Each statement in the flow is one of the 10 core verbs:

#### query
Retrieve data from external source.
```yaml
- query:
    target: <string>      # Target to query (e.g., "locations")
    where: <dict>         # Optional: Query conditions
    limit: <int>          # Optional: Result limit
    store_as: <string>    # Variable to store results
    comment: <string>     # Optional: Statement comment
```

#### check
Evaluate condition and store result.
```yaml
- check:
    condition: <expression>  # Condition to evaluate
    store_as: <string>       # Variable to store boolean result
    comment: <string>        # Optional: Statement comment
```

#### create
Create new resource.
```yaml
- create:
    target: <string>         # Target type to create
    params: <dict>           # Creation parameters
    store_as: <string>       # Variable to store created resource
    comment: <string>        # Optional: Statement comment
```

#### update
Update existing resource.
```yaml
- update:
    target: <string>         # Target type to update
    where: <dict>            # Selection conditions
    set: <dict>              # Fields to update
    store_as: <string>       # Optional: Variable for result
    comment: <string>        # Optional: Statement comment
```

#### if
Conditional execution.
```yaml
- if:
    condition: <expression>  # Condition to evaluate
    then: <statements>       # Statements if true
    else: <statements>       # Optional: Statements if false
    comment: <string>        # Optional: Statement comment
```

#### for
Iterate over collection.
```yaml
- for:
    each: <string>           # Loop variable name
    in: <expression>         # Collection to iterate
    filter: <expression>     # Optional: Boolean filter condition
    do: <statements>         # Loop body statements
    comment: <string>        # Optional: Statement comment
```

**New in v1.1: Filter Support**
The `filter` parameter allows you to filter items before processing them:

```yaml
- for:
    each: rack
    in: ${all_racks}
    filter: ${rules.rack_needs_rotation}  # Only process racks that need rotation
    do:
      - create:
          target: task
          params: {rack_id: "${rack.id}"}
```

#### switch
Multi-way branching.
```yaml
- switch:
    on: <expression>         # Expression to evaluate
    cases: <dict>            # Case mappings
      value1: <statements>
      value2: <statements>
    default: <statements>    # Optional: Default case
    comment: <string>        # Optional: Statement comment
```

#### set
Set variable value.
```yaml
# Format 1: Structured
- set:
    variable_name: <expression>
    comment: <string>        # Optional: Statement comment

# Format 2: Simplified
- set: variable_name = <expression>
```

#### stop
Stop execution.
```yaml
- stop:
    reason: <string>         # Optional: Stop reason
    comment: <string>        # Optional: Statement comment
```

#### notify
Send notification.
```yaml
- notify:
    channel: <string>        # Notification channel
    message: <string>        # Message content
    level: <string>          # Optional: info/warning/error
    comment: <string>        # Optional: Statement comment
```

### 2. Expressions

Expressions can be literals or complex expressions with operations.

#### Literals
- Numbers: `42`, `3.14`, `-10`
- Strings: `"hello"`, `'world'`
- Booleans: `true`, `false`
- Lists: `[1, 2, 3]`
- Dicts: `{key: value}`

#### Variable References
Use `${}` for variable interpolation:
- Simple: `${counter}`
- Property: `${location.id}`
- Nested: `${data.user.name}`

#### Binary Operations
- Arithmetic: `${a} + ${b}`, `${a} - ${b}`, `${a} * ${b}`, `${a} / ${b}`
- Comparison: `${a} == ${b}`, `${a} != ${b}`, `${a} > ${b}`, `${a} < ${b}`, `${a} >= ${b}`, `${a} <= ${b}`
- Logical: `${a} and ${b}`, `${a} or ${b}`

#### Unary Operations
- Logical NOT: `not ${condition}`

#### String Interpolation
Embed variables in strings:
```yaml
- set:
    message: "Count is ${count}"
    result: "${name} has ${items} items"
```

### 3. Variable Scoping

Variables follow lexical scoping rules:

1. **Rules Scope**: Variables and conditions defined in `rules` section (global, read-only)
2. **Preload Scope**: Data loaded in `preload` section (global, cached)
3. **Global Scope**: Variables defined in `variables` section
4. **Flow Scope**: Variables set in flow statements
5. **Loop Scope**: Loop variables (`each`) are scoped to loop body

Example:
```yaml
rules:
  max_count: 5                    # Rules scope (global, read-only)
  is_valid:
    condition: "${item} > 0"

preload:
  - query:
      target: items
      store_as: all_items         # Preload scope (global, cached)

variables:
  global_var: 100                 # Global scope

flow:
  - set:
      flow_var: 200               # Flow scope
  
  - for:
      each: item                  # Loop scope (only exists in loop)
      in: ${all_items}            # Reference preloaded data
      filter: ${rules.is_valid}   # Reference rule condition
      do:
        - set:
            loop_var: "${item}"
  
  # After loop, available variables:
  # - rules.max_count: 5 (rules scope, read-only)
  # - all_items: [...] (preload scope, cached)
  # - global_var: 100 (global scope)
  # - flow_var: 200 (flow scope)
  # - loop_var: 3 (flow scope, last value from loop)
  # - item: undefined (out of scope)
```

## Type System

### Primitive Types
- `int`: Integer numbers
- `float`: Floating-point numbers
- `str`: Text strings
- `bool`: Boolean values (true/false)
- `null`: Null value

### Composite Types
- `list`: Ordered collection
- `dict`: Key-value mappings

### Type Coercion
Automatic type conversion in expressions:
- Numbers in strings: `"10"` → `10` (when used in arithmetic)
- String concatenation: `10 + " items"` → `"10 items"`

## Execution Model

### Execution Order (Updated in v1.1)
TAFL v1.1 follows a structured execution order:

1. **Preload Phase**: Execute all queries in the `preload` section and cache results
2. **Rules Evaluation**: Process and cache all rules and conditions
3. **Variable Initialization**: Initialize variables from the `variables` section
4. **Flow Execution**: Execute statements in the `flow` section sequentially

```yaml
# Execution flow example
preload:          # Phase 1: Data loading
  - query: ...
  
rules:            # Phase 2: Rules processing
  condition: ...
  
variables:        # Phase 3: Variable initialization
  counter: 0
  
flow:             # Phase 4: Main execution
  - for: ...
```

### Performance Benefits
- **Reduced Query Overhead**: Preload eliminates redundant database queries
- **Rule Caching**: Business rules are evaluated once and cached
- **Optimized Execution**: Clear separation allows for execution optimization

### Error Handling
Currently, errors stop execution. Future versions will support try-catch-finally.

### Async Execution
All operations are async-capable for non-blocking I/O.

## Comments

Comments can be added to any statement:
```yaml
- set:
    counter: 10
    comment: "Initialize counter"

# YAML comments are also supported
- query:  # This queries all locations
    target: locations
```

## Standard Library Functions (Planned)

These functions will be available in expressions:

### Data Functions
- `empty(value)`: Check if empty
- `exists(value)`: Check if exists
- `count(list)`: Count items
- `sum(list)`: Sum numbers
- `avg(list)`: Average
- `min(list)`: Minimum
- `max(list)`: Maximum

### String Functions
- `upper(str)`: Uppercase
- `lower(str)`: Lowercase
- `trim(str)`: Trim whitespace
- `split(str, sep)`: Split string
- `join(list, sep)`: Join strings

### Date/Time Functions
- `now()`: Current timestamp
- `today()`: Current date
- `format_date(date, fmt)`: Format date

## External Functions

External functions can be registered and called:
```yaml
- create:
    target: "external_function_name"
    params:
      param1: value1
      param2: value2
    store_as: result
```

## Validation Rules

1. **Required Sections**: `flow` section must exist
2. **Variable Names**: Must be valid identifiers (letters, numbers, underscore)
3. **Expression Syntax**: Must follow expression grammar
4. **Statement Structure**: Must match verb schema
5. **Type Consistency**: Operations must have compatible types
6. **Preload Rules** (New in v1.1):
   - All preload queries must have `store_as` parameter
   - `store_as` names must not conflict with variables
7. **Rules Validation** (New in v1.1):
   - Rule conditions must be valid boolean expressions
   - Rule names must not conflict with preload or variable names
   - Rules are read-only and cannot be modified in flow

## Migration from Linear Flow v2

Key differences:
1. **Simplified Syntax**: More concise YAML structure
2. **10 Core Verbs**: Reduced from many functions to 10 verbs
3. **Native Expressions**: Built-in expression evaluation
4. **Strong Typing**: Type validation at parse time
5. **Better Scoping**: Proper variable scoping in loops

## Version History

- **v1.1** (2025-08): Enhanced with preload and rules
  - Added `preload` section for data optimization
  - Added `rules` section for business logic separation
  - Enhanced `for` loop with `filter` support
  - Improved variable scoping (5-level hierarchy)
  - Structured execution model (4-phase)
  - Performance optimizations

- **v1.0** (2025-08): Initial implementation
  - Core parser and executor
  - 10 core verbs
  - Expression system
  - Basic validation