# RosAGV AI Coding Assistant Instructions

## 🎯 Core Development Philosophy

### Linus Torvalds Principles
- **Data structures over code**: Focus on clean, simple data structures
- **Eliminate special cases**: Unify edge cases into normal operations
- **Never break userspace**: Maintain backward compatibility at all costs
- **Pragmatism over theory**: Solve real problems, not imaginary ones
- **Simplicity is key**: If it needs more than 3 levels of indentation, redesign it

See: `docs-ai/operations/development/linus-torvalds-ai-agent-principles.md`

## 📁 Project Architecture

### Dual-Environment System
```
🚗 AGV On-board (`rosagv` container)
├── Network: host mode (direct hardware access)
├── Deploy: On AGV edge computing devices
├── Config: docker-compose.yml
└── Purpose: Real-time control, PLC/sensor integration

🖥️ AGVC Management (`agvc_server` container)
├── Network: bridge mode (192.168.100.0/24)
├── Deploy: Central server or cloud
├── Config: docker-compose.agvc.yml
└── Purpose: Fleet management, Web UI, database
```

### Communication Layer
- **ROS 2 Jazzy** with **rmw_zenoh_cpp**
- **Zenoh Router**: Port 7447, config in `/app/routerconfig.json5` (JSON5 format)
- **Cross-container**: Automatic service discovery via Zenoh

## 🔧 Development Workflow

### Quick Commands (90% of Daily Operations)
```bash
# Enter AGVC container and rebuild web services
cd ~/RosAGV
docker compose -f docker-compose.agvc.yml exec agvc_server bash -i -c \
  "source /app/setup.bash && agvc_source && \
   manage_web_api_launch stop && ba && sa && manage_web_api_launch start"

# Common aliases (after sourcing setup.bash)
ba                    # build_all - Build all workspaces
sa                    # all_source - Source all workspaces
agvc_source          # Load AGVC workspaces
agv_source           # Load AGV workspaces
manage_web_api_launch {start|stop|restart|status}  # Web service control
```

### Essential Tools
```bash
# Unified tool system (from host)
r                    # Show all available tools
r agvc-check        # AGVC health check
r quick-diag        # Quick system diagnostics
r tafl-validate     # TAFL file validation

# Search tools (prefer ripgrep over grep)
rg "pattern" --type py    # Fast code search
json5 file.json5 | jq     # Process JSON5 configs
yq '.key' file.yaml       # Process YAML configs
```

## ⚠️ Critical Rules

### 1. Code-First Development
- **NEVER assume or guess** - Use `rg`, `ls`, `cat` to verify everything exists
- **NEVER create fictional features** - Only reference actual code
- **ALWAYS check imports** - Trace variable origins through import statements
- **ALWAYS use existing tools** - Check `scripts/` and `r` command before creating new ones

### 2. Container Execution
**ALL ROS 2 code MUST run inside Docker containers - host has NO ROS 2 environment**

```bash
# ✅ Correct: Load environment first
docker compose -f docker-compose.agvc.yml exec agvc_server bash -c \
  "source /app/setup.bash && ros2 topic list"

# ❌ Wrong: Direct execution fails
docker compose -f docker-compose.agvc.yml exec agvc_server ros2 topic list
```

### 3. Test File Management
**ALL temporary test files MUST go in `~/RosAGV/agents/`**
- ✅ `agents/test_*.py`, `agents/test_*.html`
- ❌ Never in `app/`, `docs-ai/`, or production directories

## 🏗️ Key Workspaces

### Web API (`web_api_ws`)
```
Tech Stack:
├── FastAPI + Uvicorn (from /opt/pyvenv_env)
├── Socket.IO for real-time updates
├── Jinja2 templates + static assets
└── Three services:
    ├── Port 8000: Core API (web_api)
    ├── Port 8001: Admin UI (agvcui)
    └── Port 8002: Operator UI (opui)

Key Patterns:
├── Router-based API (src/web_api/web_api/routers/)
├── Pydantic models for validation
├── miniStore for state management (frontend)
└── Module separation (tafl-editor-*.js files)
```

### Database Operations
- **Central proxy**: All DB operations through `db_proxy_ws` ROS services
- **No direct connections**: Never connect to PostgreSQL directly from nodes
- **Schema location**: `db_proxy_ws/src/db_proxy/sql/`

### Hardware Integration
- **PLC**: `keyence_plc_ws` + `plc_proxy_ws` for Keyence PLC communication
- **Sensors**: Direct access via host network mode (AGV container)
- **SICK SLAM**: Independent software, accessed directly

## 📚 Documentation Requirements

### When Modifying Code
1. **Read workspace README.md first** - Contains critical dependencies and setup
2. **Update README.md after changes** - Document new features/dependencies
3. **Follow status icons**: ✅ active, ⚠️ manual, ❌ deprecated
4. **Reference @docs-ai/** - Use knowledge base for system understanding

### Key Documentation
- **System overview**: `docs-ai/context/system/dual-environment.md`
- **Development principles**: `docs-ai/operations/development/core-principles.md`
- **Tool usage**: `docs-ai/operations/tools/unified-tools.md`
- **Docker workflows**: `docs-ai/operations/development/docker-development.md`

## 🚀 Development Checklist

### Before Starting
- [ ] Identify target environment (AGV vs AGVC)
- [ ] Read relevant workspace README.md
- [ ] Check existing implementations with `rg`
- [ ] Verify dependencies are available

### During Development
- [ ] Follow existing code patterns and style
- [ ] Use miniStore for frontend state (don't add Redux/MobX)
- [ ] Keep functions under 30 lines
- [ ] Eliminate special cases where possible

### After Changes
- [ ] Run `colcon build --packages-select <package>`
- [ ] Execute tests with `colcon test`
- [ ] Update README.md documentation
- [ ] Verify backward compatibility
- [ ] Run lint/typecheck if provided by user

## 💡 Problem-Solving Decision Tree

```
1. Is this a real problem? → Search with `rg` to confirm
2. Does a solution exist? → Check @docs-ai and scripts/
3. Can it be simplified? → Review data structures
4. Will it break anything? → Check APIs and interfaces
5. Is complexity justified? → Evaluate cost/benefit
```

## 🔗 Quick Reference

| Issue Type | Location | Debug Tool |
|------------|----------|------------|
| State machine | `agv_ws/src/agv_base/agv_states/` | `log_analyze agv` |
| API errors | `web_api_ws/src/web_api/routers/` | `r agvc-check` |
| Database | `db_proxy_ws/src/db_proxy/crud/` | `pgAdmin` on :5050 |
| PLC comm | `keyence_plc_ws/src/keyence_plc/` | Network diagnostics |
| TAFL flows | `app/config/tafl/` | `r tafl-validate` |
| Web UI | `agvcui/static/js/` | Browser DevTools |