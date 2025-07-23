# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

RosAGV is an enterprise-grade Automated Guided Vehicle (AGV) fleet management system built on ROS 2 Jazzy with Zenoh RMW middleware. The system uses a dual-environment Docker-based architecture supporting both AGV onboard systems and AGVC management systems for comprehensive warehouse automation.

### Key Architecture Features
- **Dual Environment**: AGV onboard systems + AGVC management systems
- **Modern Tech Stack**: ROS 2 Jazzy + Zenoh RMW + Docker containerization
- **Multi-Vehicle Support**: Cargo, Loader, Unloader vehicle types
- **External Integration**: KUKA Fleet system integration
- **Complete Web Interface**: AGVCUI management console + OPUI operator interface

## Development Commands

### Container Operations
```bash
# AGV onboard system
docker compose -f docker-compose.yml up -d
docker compose -f docker-compose.yml exec rosagv bash

# AGVC management system  
docker compose -f docker-compose.agvc.yml up -d
docker compose -f docker-compose.agvc.yml exec agvc_server bash
```

### Build Commands (Inside Container)
```bash
# Load development environment
source /app/setup.bash

# Build all workspaces
build_all                    # or ba

# Build specific workspace
build_ws agv_ws             # or build_single agv_ws

# Build with symlink install
build_all_symlink_install

# Clean workspaces
clean_all                   # or ca
clean_ws agv_ws            # or clean_single agv_ws
```

### Testing Commands
```bash
# Test all workspaces
test_all                    # or ta

# Test specific workspace
test_ws agv_ws             # or test_single agv_ws
```

### Environment Management
```bash
# Load all workspaces (dependency-ordered)
all_source                  # or sa or load_all

# Check system status
check_system_status         # or status
check_zenoh_status         # or zenoh
check_ros_env              # or rosenv

# Service management
manage_zenoh start|stop|restart|status
manage_ssh start|stop|restart|status
```

### AGVC-Specific Commands (AGVC environment only)
```bash
# Load AGVC workspaces
agvc_source

# Database management
start_db
stop_db

# Start ECS service
start_ecs

# Check AGVC status
check_agvc_status
```

## Code Architecture

### High-Level System Structure

The system is organized into two deployment environments:

**🚗 AGV Onboard System** (docker-compose.yml):
- Runs on vehicle-mounted computers
- Handles real-time vehicle control and sensor integration
- Auto-loads: keyence_plc_ws, plc_proxy_ws, agv_cmd_service_ws, joystick_ws, agv_ws, path_algorithm

**🖥️ AGVC Management System** (docker-compose.agvc.yml):
- Runs on management servers 
- Handles fleet management, task scheduling, web interfaces
- Includes: db_proxy_ws, ecs_ws, rcs_ws, wcs_ws, web_api_ws, kuka_fleet_ws

### Core Workspace Architecture

```
app/
├── agv_ws/                    # 🚗 AGV core control (3-layer state architecture)
├── agv_cmd_service_ws/        # 🚗 Manual command services
├── joystick_ws/               # 🚗 USB joystick control
├── keyence_plc_ws/            # 🚗🖥️ Keyence PLC communication
├── plc_proxy_ws/              # 🚗🖥️ ROS 2 PLC service proxy
├── path_algorithm/            # 🚗🖥️ A* pathfinding
├── db_proxy_ws/               # 🖥️ PostgreSQL proxy + ORM
├── ecs_ws/                    # 🖥️ Equipment Control System
├── rcs_ws/                    # 🖥️ Robot Control System  
├── wcs_ws/                    # 🖥️ Warehouse Control System
├── web_api_ws/                # 🖥️ Web APIs + Socket.IO
├── kuka_fleet_ws/             # 🖥️ KUKA Fleet integration
└── sensorpart_ws/             # 🚗 Sensor data processing
```

### Critical Design Patterns

#### 1. AGV State Machine Architecture (agv_ws)
Three-layer state control pattern:
- **Base Layer**: Common logic states (all vehicle types)
- **AGV Layer**: Vehicle-type specific states (Cargo/Loader/Unloader)  
- **Robot Layer**: Robotic arm task execution states

Key implementation files:
- `agv_ws/src/agv_base/agv_base/agv_node_base.py` - Base state machine
- `agv_ws/src/*/robot_context.py` - Vehicle-specific contexts

#### 2. ROS 2 + Zenoh Integration
- **RMW Implementation**: rmw_zenoh_cpp (pre-configured)
- **Configuration**: `/app/routerconfig.json5`
- **Auto-startup**: Zenoh Router launches automatically in containers
- **Network**: Port 7447 for Zenoh Router communication

#### 3. Web Service Architecture (web_api_ws)
Multi-service FastAPI architecture:
- **Port 8000**: Core API service (PLC, door, traffic, KUKA Fleet APIs)
- **Port 8001**: AGVCUI fleet management (admin interface + Socket.IO)
- **Port 8002**: OPUI operator interface (machine operations + Socket.IO)

Socket.IO systems:
- **AGVCUI**: Full system monitoring with change tracking (modify_log based)
- **OPUI**: Event-driven machine operations with real-time responses

#### 4. Database Integration (db_proxy_ws)
- **ORM**: SQLModel + SQLAlchemy
- **Connection Pool**: Managed PostgreSQL connections
- **Models**: Comprehensive fleet management data models

#### 5. PLC Communication Stack
```
ECS (Equipment Control) 
    ↓
plc_proxy_ws (ROS 2 Services)
    ↓  
keyence_plc_ws (TCP/IP Communication)
    ↓
Keyence PLC Hardware
```

## Development Guidelines

### Code Patterns to Follow

#### ROS 2 Node Structure
```python
import rclpy
from rclpy.node import Node
from typing import Optional

class ExampleNode(Node):
    def __init__(self):
        super().__init__('example_node')
        
        # Parameter declarations
        self.declare_parameter('param_name', default_value)
        
        # Publishers/Subscribers
        self.publisher = self.create_publisher(MsgType, 'topic', 10)
        self.subscription = self.create_subscription(
            MsgType, 'topic', self.callback, 10
        )
        
        # Timers
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.get_logger().info('Node initialized')
    
    def callback(self, msg: MsgType) -> None:
        """Handle incoming messages"""
        pass
    
    def timer_callback(self) -> None:
        """Timer callback"""  
        pass

def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = ExampleNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

#### FastAPI Router Structure
```python
from fastapi import APIRouter, HTTPException, Depends
from pydantic import BaseModel
from sqlmodel import Session
from ..database import get_session

router = APIRouter()

class ItemResponse(BaseModel):
    id: int
    name: str

@router.post("/items/", response_model=ItemResponse)
async def create_item(
    item: ItemCreate,
    session: Session = Depends(get_session)
) -> ItemResponse:
    """Create new item with proper error handling"""
    try:
        # Implementation
        pass
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))
    except Exception as e:
        raise HTTPException(status_code=500, detail="Internal server error")
```

#### Socket.IO Event Handler Structure  
```python
import socketio
from typing import Dict, Any

class SocketHandler:
    def __init__(self, sio: socketio.AsyncServer):
        self.sio = sio
        self.init_socketio()
    
    def init_socketio(self) -> None:
        """Register event handlers"""
        self.sio.on('connect')(self.connect)
        self.sio.on('custom_event')(self.handle_custom_event)
    
    async def connect(self, sid: str, environ: dict) -> None:
        """Handle client connections"""
        pass
    
    async def handle_custom_event(self, sid: str, data: Dict[str, Any]) -> Dict[str, Any]:
        """Handle custom events with proper response format"""
        try:
            # Event processing
            return {"success": True, "message": "Success"}
        except Exception as e:
            return {"success": False, "message": str(e)}
```

### Environment and Dependencies

#### Python Environment
- **Version**: Python 3.12+ (container environment)
- **Virtual Environment**: `/opt/pyvenv_env/` (auto-activated)
- **Key Packages**: ROS 2 Jazzy, SQLModel, FastAPI, Socket.IO

#### ROS 2 Environment Variables
```bash
ROS_DISTRO=jazzy
RMW_IMPLEMENTATION=rmw_zenoh_cpp
ZENOH_ROUTER_CONFIG_URI=/app/routerconfig.json5
```

#### Build Dependencies
- **ROS 2**: colcon build system
- **Python**: setup.py based packages  
- **Database**: PostgreSQL 12+
- **Containerization**: Docker + Docker Compose V2

### Testing Strategy

#### ROS 2 Testing
```bash
# Individual workspace testing
cd /app/agv_ws && colcon test --event-handlers console_direct+

# Comprehensive testing  
test_all  # Tests all built workspaces
```

#### Web API Testing
- FastAPI automatic OpenAPI documentation at `/docs`
- Socket.IO event testing via browser developer tools
- Database integration tests using pytest

### Configuration Management

#### Key Configuration Files
- `/app/config/hardware_mapping.yaml` - Device identity mapping
- `/app/config/agv/*.yaml` - Vehicle-specific configurations
- `/app/config/agvc/*.yaml` - Management system configurations  
- `/app/routerconfig.json5` - Zenoh Router network configuration

#### Environment Detection
The system automatically detects deployment environment:
- **AGV Environment**: CONTAINER_TYPE="agv"
- **AGVC Environment**: CONTAINER_TYPE="agvc"

### Common Development Workflows

#### Adding New AGV States
1. Extend base state machine in `agv_base/agv_states/`
2. Implement vehicle-specific states in appropriate `*_agv/` workspace
3. Update state constants and validation logic
4. Add comprehensive logging and error handling

#### Creating New API Endpoints
1. Define Pydantic models in `web_api_ws/src/web_api/models/`
2. Implement CRUD operations in `web_api_ws/src/web_api/crud/`
3. Create FastAPI router in `web_api_ws/src/web_api/routers/`
4. Add to main application in `api_server.py`

#### Extending Socket.IO Events
1. Add event handlers to appropriate socket classes
2. Update frontend JavaScript event listeners
3. Test event flow using browser developer tools
4. Document event format and responses

### Important Notes

- **Never commit sensitive data**: Use environment variables for credentials
- **Follow existing naming conventions**: Study existing code patterns before implementing
- **Zenoh auto-configuration**: The system automatically configures Zenoh RMW - avoid manual RMW setup
- **Container-aware development**: Always develop inside the appropriate container environment
- **State machine safety**: AGV state transitions must include comprehensive validation and logging
- **Database migrations**: Use proper SQLModel schema evolution for database changes

### Debugging and Troubleshooting

#### Common Issues
- **Zenoh connectivity**: Check `manage_zenoh status` and port 7447 availability
- **ROS 2 environment**: Verify with `check_ros_env` command
- **Container services**: Use `check_system_status` for overall health
- **Database connectivity**: AGVC environment only - check PostgreSQL service

#### Log Locations
- Zenoh Router: `/tmp/zenoh_router.log`
- AGV Launch: `/tmp/agv.log`  
- Service logs: Check with `docker compose logs -f <service>`

This architecture enables robust, scalable AGV fleet management with clear separation of concerns between vehicle control and fleet management systems.

## Language Configuration

### Language Settings
- **CLI User Interaction Language**: 使用繁體中文回應
- **Default Language for Code Comments**: 繁體中文
```