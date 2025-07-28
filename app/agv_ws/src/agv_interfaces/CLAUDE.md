# agv_interfaces - AGV ROS 2訊息接口定義

## 📚 Context Loading
@docs-ai/context/system/technology-stack.md
@docs-ai/knowledge/protocols/ros2-interfaces.md
@docs-ai/operations/development/docker-development.md
@docs-ai/operations/maintenance/system-diagnostics.md

## 📋 專案概述
agv_interfaces 定義 RosAGV 系統中 AGV 相關的 ROS 2 訊息類型，提供標準化的通訊接口。

詳細接口開發指導請參考: @docs-ai/knowledge/protocols/ros2-interfaces.md

## 📂 關鍵檔案位置

### 接口定義
```
agv_interfaces/
├─ msg/
│  ├─ AgvStatus.msg           # AGV狀態數據結構
│  └─ AgvStateChange.msg      # 狀態變更事件結構
├─ CMakeLists.txt             # CMake構建配置
└─ package.xml                # ROS 2套件元數據
```

## 🔧 開發環境
@docs-ai/operations/development/docker-development.md

## 🚀 接口測試
@docs-ai/knowledge/protocols/ros2-interfaces.md

### 基本功能測試
```bash
# 進入AGV容器並構建接口
agv_enter
cd /app/agv_ws
colcon build --packages-select agv_interfaces

# 檢查生成的接口
ros2 interface show agv_interfaces/msg/AgvStatus
ros2 interface show agv_interfaces/msg/AgvStateChange
```

### 接口驗證
```bash
# 發布測試訊息
ros2 topic pub /agv/status agv_interfaces/msg/AgvStatus "{
  agv_id: 'test_agv_01',
  agv_type: 'cargo',
  battery_level: 85,
  position_x: 1.5,
  position_y: 2.3,
  current_state: 'idle',
  robot_state: 'ready',
  is_busy: false
}"

# 監聽狀態訊息
ros2 topic echo /agv/status
ros2 topic echo /agv/state_change
```

## 📊 配置設定

### CMakeLists.txt 核心配置
```cmake
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AgvStatus.msg"
  "msg/AgvStateChange.msg"
  DEPENDENCIES builtin_interfaces
)
```

### package.xml 依賴管理
```xml
<depend>builtin_interfaces</depend>
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

## 🔗 系統整合
@docs-ai/knowledge/protocols/ros2-interfaces.md

### 使用此接口的專案
- **agv_base**: 發布 AgvStatus，處理 AgvStateChange
- **cargo_mover_agv**: 使用 AgvStatus 發布車輛狀態
- **loader_agv / unloader_agv**: 狀態同步和變更通知
- **agvcui**: 接收狀態資訊顯示 AGV 即時狀態
- **web_api_ws**: 透過 WebSocket 轉發狀態給前端

## 🚨 故障排除
@docs-ai/operations/maintenance/system-diagnostics.md
@docs-ai/knowledge/protocols/ros2-interfaces.md