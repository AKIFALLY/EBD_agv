# launch_ws CLAUDE.md

## 模組概述
簡單的 ROS 2 啟動檔案管理工作空間，包含 web API 服務和 ECS 系統的基本啟動配置。

## 實際專案結構 (基於實際程式碼)
```
src/
├── web_api_launch/          # Web API 服務啟動包
│   ├── launch/
│   │   └── launch.py        # Web API 啟動檔案
│   ├── web_api_launch/
│   │   └── __init__.py
│   ├── setup.py            # 套件設定 (無 entry_points)
│   └── package.xml
└── ecs_launch/             # ECS 系統啟動包
    ├── launch/
    │   └── launch.py        # ECS 啟動檔案  
    ├── ecs_launch/
    │   └── __init__.py
    ├── setup.py            # 套件設定 (無 entry_points)
    └── package.xml
```

## 核心功能 (基於實際實現)

### web_api_launch - Web API 服務啟動
啟動所有 Web 相關服務：

**啟動的節點**:
- `agvc_ui_server` (agvcui 套件) - AGV 車隊管理界面
- `op_ui_server` (opui 套件) - 操作員界面
- `web_api_server` (web_api 套件) - 核心 Web API 服務

**配置**:
- 所有節點使用 `agvc` 命名空間
- 配置檔案路徑: `/app/config/web_api_config.yaml`

### ecs_launch - ECS 系統啟動
啟動設備控制系統相關服務：

**啟動的節點**:
- `plc_service` (plc_proxy 套件) - PLC 通訊服務
- `ecs_core` (ecs 套件) - ECS 核心節點

**配置**:
- 所有節點使用 `agvc` 命名空間
- 配置檔案路徑: `/app/config/ecs_config.yaml`
- 包含已註解的 `door_controller_node_mqtt` (目前未啟用)

## 開發指令

### 環境設定 (AGVC容器內)
```bash
source /app/setup.bash
agvc_source  # 載入AGVC工作空間
cd /app/launch_ws
```

### 建置工作空間
```bash
# 建置整個工作空間
build_ws launch_ws

# 或使用 colcon 直接建置
colcon build
```

### 啟動服務

#### Web API 服務啟動
```bash
# 啟動所有 Web 服務
ros2 launch web_api_launch launch.py

# 檢查啟動的節點
ros2 node list | grep agvc
# 預期輸出:
# /agvc/agvc_ui_server
# /agvc/op_ui_server  
# /agvc/web_api_server
```

#### ECS 系統啟動
```bash
# 啟動 ECS 系統
ros2 launch ecs_launch launch.py

# 檢查啟動的節點
ros2 node list | grep agvc
# 預期輸出:
# /agvc/plc_service
# /agvc/ecs_core
```

## 實際啟動檔案內容

### web_api_launch/launch/launch.py
```python
def generate_launch_description():
    param_file = '/app/config/web_api_config.yaml'
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'param_file', default_value=param_file, 
            description='Path to parameter file'
        ),
        Node(
            package='agvcui',
            executable='agvc_ui_server',
            name='agvc_ui_server',
            namespace='agvc',
        ),
        Node(
            package='opui',
            executable='op_ui_server', 
            name='op_ui_server',
            namespace='agvc',
        ),
        Node(
            package='web_api',
            executable='api_server',
            name='web_api_server',
            namespace='agvc',
        )
    ])
```

### ecs_launch/launch/launch.py  
```python
def generate_launch_description():
    param_file = '/app/config/ecs_config.yaml'
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'param_file', default_value=param_file,
            description='Path to parameter file'
        ),
        Node(
            package='plc_proxy',
            executable='plc_service',
            name='plc_service',
            namespace='agvc',
            parameters=[LaunchConfiguration('param_file')],
            output="screen"
        ),
        Node(
            package='ecs',
            executable='ecs_core', 
            name='ecs_core',
            namespace='agvc',
            parameters=[LaunchConfiguration('param_file')],
            output="screen"
        ),
    ])
```

## 配置管理

### 配置檔案位置
- **Web API 配置**: `/app/config/web_api_config.yaml`
- **ECS 配置**: `/app/config/ecs_config.yaml`

### 參數傳遞
- ECS 啟動檔案使用參數檔案配置節點
- Web API 啟動檔案目前不使用參數檔案 (節點自行處理配置)

## 測試與驗證

### 檢查啟動狀態
```bash
# 檢查所有 ROS 2 節點
ros2 node list

# 檢查特定命名空間的節點
ros2 node list | grep agvc

# 檢查節點資訊
ros2 node info /agvc/web_api_server
ros2 node info /agvc/ecs_core
```

### 服務驗證
```bash
# 檢查可用服務
ros2 service list | grep agvc

# 檢查主題
ros2 topic list | grep agvc

# 測試 Web API 可用性 (如果配置正確)
curl http://localhost:8000/health
```

## 整合點

### 依賴套件
- **web_api_launch** 依賴:
  - `agvcui` - AGV 車隊管理界面
  - `opui` - 操作員界面  
  - `web_api` - 核心 Web API 服務

- **ecs_launch** 依賴:
  - `plc_proxy` - PLC 通訊代理
  - `ecs` - 設備控制系統

### 系統整合
- 兩個啟動包獨立運行，分別負責不同功能
- 所有節點使用 `agvc` 命名空間統一管理
- 透過配置檔案管理系統參數

## 故障排除

### 常見問題

#### 套件找不到
```bash
# 檢查套件是否存在
ros2 pkg list | grep -E "(web_api|agvcui|opui|ecs|plc_proxy)"

# 確認工作空間已正確載入
echo $ROS_PACKAGE_PATH
```

#### 節點啟動失敗  
```bash
# 檢查詳細錯誤訊息
ros2 launch web_api_launch launch.py --debug

# 檢查配置檔案是否存在
ls -la /app/config/web_api_config.yaml
ls -la /app/config/ecs_config.yaml
```

#### 服務無法連接
```bash
# 檢查網路端口
netstat -tlnp | grep -E "(8000|8001|8002)"

# 檢查服務狀態
ros2 node info /agvc/web_api_server
```

## 重要提醒
- **必須在AGVC容器內運行**: 需要正確的 ROS 2 環境
- **簡單啟動設計**: 相比複雜的啟動管理系統，實際實現較為簡單
- **命名空間統一**: 所有節點使用 `agvc` 命名空間
- **配置檔案依賴**: 確認 `/app/config/` 目錄下的配置檔案存在
- **無 console_scripts**: 兩個套件都沒有定義 entry_points，僅提供啟動檔案