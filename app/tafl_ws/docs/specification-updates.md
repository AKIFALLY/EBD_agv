# TAFL Specification Updates Log

## Overview
This document tracks updates to the TAFL specification based on implementation experience and practical considerations.

## Update History

### 2025-09-11: TAFL v1.1.2 Syntax Standardization

#### Background
Based on the official TAFL specification in docs-ai and implementation consistency requirements, TAFL has been updated to v1.1.2 with syntax standardization for SWITCH and SET statements.

#### Major Changes

##### 1. SWITCH Statement - Default Case Format
**Previous (v1.1.1)**:
```yaml
- switch:
    on: <expression>
    cases: <dict>
    default: <statements>  # Separate key at same level
```

**New (v1.1.2)**:
```yaml
- switch:
    expression: <expression>  # Use 'expression' not 'on'
    cases:                    # Array format
      - when: <value>
        do: <statements>
      - when: "default"       # Default as special case
        do: <statements>
```

**Key Changes**:
- Use `expression` key instead of `on`
- Cases are now an array, not a dictionary
- Default is `when: "default"` within cases array
- Default must be placed at the end of cases
- More consistent with overall switch structure

##### 2. SET Statement - Format Restriction
**Previous (v1.1.x)**:
```yaml
# Two formats were supported
- set: variable = value        # String format
- set:                         # Object format
    variable: value
```

**New (v1.1.2)**:
```yaml
# Only object format allowed
- set:
    variable: value
# Or multiple variables
- set:
    var1: value1
    var2: value2
```

**Key Changes**:
- String format `"variable = value"` no longer supported
- Only object/dictionary format allowed
- Ensures consistency and clarity
- Eliminates parsing ambiguities

#### Implementation Status
- ✅ Parser updated (parser.py line 329-334 for switch, 473-482 for set)
- ✅ Validator updated with v1.1.2 requirements
- ✅ Example flows updated (complete_v11_example.yaml)
- ✅ Test flows validated
- ✅ **Backward compatibility removed** (2025-09-11): Parser now only accepts v1.1.2 format

#### Migration Guide
For existing TAFL flows:

1. **Update SWITCH statements**:
   - Change `on:` to `expression:`
   - Convert cases from dictionary to array format
   - Move `default:` into cases as `when: "default"`

2. **Update SET statements**:
   - Convert any `set: "var = value"` to `set: {var: value}`
   - Use multi-line YAML for better readability

#### Rationale
- **Consistency**: All verbs now use consistent syntax patterns
- **Clarity**: Clearer structure reduces confusion
- **YAML Best Practices**: Better alignment with YAML conventions
- **Parser Simplification**: Reduces special cases in parser
- **Editor Support**: Easier for TAFL Editor to generate correct syntax

### 2025-08-21: TAFL v1.1 Major Update - Preload and Rules Integration

#### Background
Based on analysis of Linear Flow v4 architecture and current TAFL implementation gaps, TAFL has been upgraded from v1.0 to v1.1 with significant architectural enhancements.

#### Major Changes

##### 1. Program Structure Expansion (4 → 6 Sections)
**Previous v1.0 Structure**:
```yaml
metadata:    # Optional
settings:    # Optional  
variables:   # Optional
flow:        # Required
```

**New v1.1 Structure**:
```yaml
metadata:    # Optional: Program identification
settings:    # Optional: Execution settings
preload:     # NEW: Data preloading for performance
rules:       # NEW: Business rules and configuration
variables:   # Optional: Initial variables
flow:        # Required: Main execution logic
```

##### 2. Preload Section - Data Caching System
**Purpose**: Eliminate redundant database queries and improve performance
**Syntax**:
```yaml
preload:
  - query:
      target: <string>          # Target to query
      where: <dict>             # Optional: Query conditions
      limit: <int>              # Optional: Result limit
      store_as: <string>        # Required: Variable to store results
      comment: <string>         # Optional: Description
```

**Benefits**:
- **Performance**: Avoid repeated database queries in loops
- **Consistency**: Ensure data consistency throughout execution
- **Clarity**: Separate data loading from business logic

##### 3. Rules Section - Business Logic Separation
**Purpose**: Centralize business logic, conditions, and configuration values
**Syntax**:
```yaml
rules:
  # Condition rules
  rule_name:
    condition: <expression>     # Boolean expression
    description: <string>       # Optional: Rule description
  
  # Configuration values
  config_name: <value>          # Any value type
  
  # Complex templates
  template_name:
    field1: <value>
    field2: <value>
```

**Benefits**:
- **Reusability**: Define once, use multiple times
- **Maintainability**: Centralized business logic
- **Readability**: Clear separation of rules and execution
- **Testing**: Easy to test business rules independently

##### 4. Enhanced For Loop with Filter Support
**New Feature**: `filter` parameter in for loops
```yaml
- for:
    each: <string>           # Loop variable name
    in: <expression>         # Collection to iterate
    filter: <expression>     # NEW: Boolean filter condition
    do: <statements>         # Loop body statements
```

**Usage Example**:
```yaml
- for:
    each: rack
    in: ${all_racks}
    filter: ${rules.rack_needs_rotation}  # Only process racks that need rotation
    do:
      - create:
          target: task
          params: ${rules.task_template}
```

##### 5. Structured Execution Model (4 Phases)
**v1.0 Execution**: Variables initialization → Flow execution
**v1.1 Execution**: 
1. **Preload Phase**: Execute all queries and cache results
2. **Rules Evaluation**: Process and cache business rules
3. **Variable Initialization**: Initialize variables from variables section
4. **Flow Execution**: Execute statements using preloaded data and rules

##### 6. Enhanced Variable Scoping (5 Levels)
**v1.0 Scoping**: Global → Flow → Loop
**v1.1 Scoping**:
1. **Rules Scope**: Variables defined in rules section (global, read-only)
2. **Preload Scope**: Data loaded in preload section (global, cached)
3. **Global Scope**: Variables defined in variables section
4. **Flow Scope**: Variables set in flow statements
5. **Loop Scope**: Loop variables (each) scoped to loop body

#### Implementation Changes

##### AST Node Additions
- **RuleDefinition**: Represents business rules and configuration
- **PreloadStatement**: Represents data preloading queries
- **Enhanced ForStatement**: Added filter parameter support
- **Updated TAFLProgram**: Added preload and rules fields

##### Example File Updates
- **example_flow.yaml**: Updated to demonstrate v1.1 features with preload and rules
- **rack_rotation_example.yaml**: Created comprehensive real-world example showing KUKA AGV integration

#### Design Decisions

##### 1. Backward Compatibility
**Decision**: No backward compatibility maintained
**Rationale**: TAFL is still in development phase, focus on optimal architecture

##### 2. Filter vs Where Clause
**Decision**: Use `filter` in for loops instead of separate where clause
**Rationale**: More intuitive and consistent with existing patterns

##### 3. Rules Section Structure
**Decision**: Support both condition rules and configuration values in same section
**Rationale**: Provides flexibility while maintaining simplicity

##### 4. Preload Variable Storage
**Decision**: Store preloaded data in dedicated scope (preload scope)
**Rationale**: Prevents naming conflicts with variables section

#### Performance Benefits
- **Query Optimization**: Preload eliminates redundant database queries
- **Rule Caching**: Business rules evaluated once and cached
- **Memory Efficiency**: Structured execution allows for memory optimization
- **Execution Speed**: Clear phases enable targeted optimizations

### 2025-08-21: Initial Implementation Insights

#### 1. Syntax Flexibility
**Original Spec**: Only showed simplified syntax (`set: counter = 10`)
**Implementation Finding**: Both simplified and structured syntax are useful
**Decision**: Support both formats for flexibility
**Rationale**: 
- Simplified is good for simple operations
- Structured is clearer for complex parameters
- Users can choose based on preference

#### 2. String Quote Handling in Simplified Syntax
**Issue**: `set: message = "Hello"` includes quotes in the value
**Current Behavior**: Quotes become part of the string
**Consideration**: May need special handling or documentation
**Workaround**: Use structured format for strings with specific quote requirements

#### 3. Loop Variable Scoping
**Original Spec**: Not explicitly defined
**Implementation**: Loop variables (`each`) are properly scoped to loop body only
**Decision**: This is the correct behavior and should be in spec
**Rationale**: Prevents variable pollution and follows modern language practices

#### 4. External Function Integration
**Original Spec**: Mentioned but not detailed
**Implementation Challenge**: Need clear interface for flow_wcs integration
**Current Status**: Using `create` verb with `target` as function name
**Future Consideration**: May need dedicated `call` verb for clarity

## Pending Specification Decisions

### 1. Standard Library Functions
**Question**: Should these be built-in or external functions?
**Options**:
- A: Built into executor (faster, no registration needed)
- B: External functions (more flexible, consistent interface)
- C: Hybrid (common ones built-in, others external)

### 2. Error Handling
**Question**: How much detail for try-catch-finally?
**Considerations**:
- Need to define error types
- Scope of catch blocks
- Finally execution guarantees
- May defer to Phase 2

### 3. Type System
**Question**: How strict should type checking be?
**Current**: Basic type checking in validator
**Options**:
- Runtime type coercion (current)
- Strict compile-time checking
- Optional type hints

### 4. Async Operations
**Implementation**: All operations are async
**Question**: Should spec distinguish sync vs async operations?
**Current Decision**: Keep all async for consistency

## Implementation-Driven Improvements

### 1. Comment Field
**Added**: Optional `comment` field on all statements
**Reason**: Better documentation and debugging
**Example**:
```yaml
- set:
    counter: 10
    comment: "Initialize counter for loop"
```

### 2. Metadata Section
**Enhanced**: Added more fields based on need
```yaml
metadata:
  id: flow_001
  name: Flow Name
  version: 1.0
  author: System
  description: Description
  tags: [tag1, tag2]
  created_at: 2025-08-21  # Auto-generated
  updated_at: 2025-08-21  # Auto-generated
```

### 3. Settings Section
**Added**: Execution settings
```yaml
settings:
  timeout: 3600        # Max execution time
  retry_count: 3       # Retry failed operations
  log_level: debug     # Logging verbosity
  parallel: false      # Future: parallel execution
```

## Deferred Features

These features are intentionally deferred to keep initial implementation simple:

### Phase 2 Features
1. **Try-Catch-Finally**: Complex error handling
2. **Break/Continue**: Loop control flow
3. **While Loop**: Condition-based looping
4. **Parallel Execution**: Concurrent statement execution
5. **Transaction Support**: Rollback capability

### Phase 3 Features
1. **Custom Types**: User-defined data types
2. **Functions**: User-defined functions in TAFL
3. **Modules**: Import/include other TAFL files
4. **Debugging**: Breakpoints and step execution

## Specification Clarifications

### 1. Variable Resolution Order
When resolving `${variable}`:
1. Check loop scope (if in loop)
2. Check flow scope (set variables)
3. Check initial variables
4. Error if not found

### 2. Expression Evaluation
Order of operations:
1. Parentheses (future)
2. Unary operators (not, -)
3. Multiplication/Division
4. Addition/Subtraction
5. Comparison operators
6. Logical AND
7. Logical OR

### 3. Null Handling
- Null in arithmetic: Error
- Null in comparison: `null == null` is true
- Null in string interpolation: Converts to "null"

## Backward Compatibility

### Principle
Maintain backward compatibility when updating spec:
1. New features are additive
2. Existing syntax remains valid
3. Deprecation requires migration path

### Version Strategy
- v1.0: Current implementation
- v1.1: Standard library functions
- v1.2: Error handling
- v2.0: Breaking changes (if needed)

## Feedback Integration Process

1. **Implementation Discovery**: Find issue during coding
2. **Document in This File**: Record the finding
3. **Discuss Options**: Consider alternatives
4. **Update Implementation**: Apply decision
5. **Update Spec**: Reflect in main specification
6. **Update Tests**: Ensure tests cover new behavior

## Open Questions for Discussion

1. Should we add a `call` verb specifically for external functions?
2. How should we handle async function timeouts?
3. Should variables be typed or dynamic?
4. Do we need a debugging mode with variable watches?
5. Should we support YAML anchors and aliases?

## Notes for Spec Writers

When updating the specification:
1. Keep examples simple and clear
2. Document both what works and what doesn't
3. Include rationale for design decisions
4. Consider implementation complexity
5. Think about error messages and debugging
6. Maintain consistency with existing patterns