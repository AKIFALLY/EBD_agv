# Flow WCS Workspace CLAUDE.md

## 📚 Context Loading
@docs-ai/knowledge/system/flow-wcs-system.md
@docs-ai/knowledge/system/flow-wcs-function-system.md
@docs-ai/knowledge/system/linear-flow-advanced-features.md
@docs-ai/knowledge/protocols/kuka-agv-rack-rotation.md
@docs-ai/operations/development/flow-wcs-development.md
@docs-ai/operations/development/linear-flow-troubleshooting-cases.md
@docs-ai/knowledge/agv-domain/wcs-system-design.md
@docs-ai/knowledge/agv-domain/wcs-database-design.md
@docs-ai/knowledge/agv-domain/wcs-workid-system.md

## 🎯 Module Overview
**Flow WCS** is the unified warehouse control system for RosAGV, implementing Linear Flow v2 architecture. It is the production system replacing all previous experimental WCS implementations.

## 🔧 Core Features
- **Linear Flow v2**: Declarative YAML-based flow execution
- **43 Built-in Functions**: Comprehensive function library for WCS operations
- **Direct Database Access**: PostgreSQL integration without db_proxy dependency
- **Advanced Expression Resolution**: Mathematical operations and complex logical expressions
- **Context Stack**: Proper variable scoping in nested foreach loops
- **Parameter Merging**: Work parameters inheritance with task-specific overrides

## 🚀 Recent Enhancements
- **Expression Resolution**: Support for `${variable + 1}` mathematical operations
- **Logical Operators**: Full support for `||`, `&&`, `!` in conditions
- **Variable Scoping**: Context stack implementation for nested foreach isolation
- **KUKA Integration**: Special parameters for KUKA AGV rack rotation tasks
- **Room ID Assignment**: Tasks now include room identification

## 📁 Project Structure
```
flow_wcs_ws/
├── src/flow_wcs/
│   ├── flow_executor.py      # Core execution engine with enhanced capabilities
│   ├── database.py           # Direct PostgreSQL access with parameter merging
│   ├── decorators.py         # Function registration system
│   ├── flow_monitor.py       # Flow execution monitoring
│   └── flow_validator.py     # YAML flow validation
├── test_rack_rotation_*.py   # Rack rotation test suites
└── deploy.sh                  # Deployment script
```

## 🔍 Key Technical Details

### Expression Resolution
- Mathematical operations: `+`, `-`, `*`, `/`
- Deep property access: `${object.nested.property}`
- Array indexing: `${array[0]}`

### Logical Operators
- OR: `${condition1 || condition2}`
- AND: `${condition1 && condition2}`
- NOT: `!${condition}`

### KUKA AGV Parameters
```yaml
metadata:
  model: "KUKA400i"
  nodes: ["${start}", "${start + 1}", "${start}"]  # 3-point rotation
  room_id: "${location.room_id}"
```

### Parameter Merging Strategy
1. Start with Work.parameters as base
2. Override/extend with task metadata
3. Preserve all required configurations

## 🛠️ Development Workflow

### Testing Rack Rotation
```bash
# Direct execution test
python test_rack_rotation_execution.py

# Integration test
python test_integration.py
```

### Building and Deploying
```bash
cd /app/flow_wcs_ws
colcon build --packages-select flow_wcs
./deploy.sh full
```

## 🚨 Common Issues and Solutions
See @docs-ai/operations/development/linear-flow-troubleshooting-cases.md for detailed cases:
- Task over-creation issues
- Work ID vs work_code confusion
- Variable scoping problems
- Mathematical expression errors
- Parameter merging strategies

## 🔗 Related Documentation
- Linear Flow Designer: Located in web_api_ws/src/agvcui/
- Work ID System: @docs-ai/knowledge/agv-domain/wcs-workid-system.md
- Database Design: @docs-ai/knowledge/agv-domain/wcs-database-design.md