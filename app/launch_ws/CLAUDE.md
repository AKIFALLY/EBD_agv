# launch_ws - ROS 2 啟動配置工作空間

## 📚 Context Loading
../../CLAUDE.md  # 引用根目錄系統文档

## 📋 工作空間概述

**ROS 2 啟動配置工作空間** 專注於提供統一的服務啟動管理，整合 Web API 服務群組和 ECS 設備控制系統的 ROS 2 Launch 檔案。

### ROS 2 啟動配置工作空間特有功能
- **🚀 統一啟動管理**: 提供 Web API 和 ECS 系統的統一啟動入口
- **📦 服務群組編排**: 整合多個相關服務的啟動順序
- **⚙️ 參數配置**: 統一的啟動參數管理
- **🔄 依賴管理**: 處理服務間的啟動依賴關係

### 核心定位
- **服務編排**: 統一管理多個 ROS 2 節點的啟動順序和依賴關係
- **配置集中**: 透過 Launch 檔案集中管理系統參數和配置
- **環境隔離**: 使用命名空間 (`agvc`) 實現服務隔離
- **部署簡化**: 提供一鍵啟動複雜服務群組的能力

詳細容器環境說明請參考: @docs-ai/context/system/dual-environment.md

## 📂 專案結構 (實際驗證)

```
src/
├── web_api_launch/          # 🌐 Web API 服務群組啟動包
│   ├── launch/
│   │   └── launch.py        # Web API 服務群組啟動檔案
│   ├── web_api_launch/
│   │   └── __init__.py      # Python 模組初始化
│   ├── resource/           # ROS 2 資源檔案
│   │   └── web_api_launch  # 套件標記檔案
│   ├── setup.py            # 套件設定 (Launch-only package)
│   ├── setup.cfg           # 建置配置
│   └── package.xml         # ROS 2 套件資訊
├── web_agv_launch/          # 🚗 AGV Web 服務啟動包
│   ├── launch/
│   │   └── launch.py        # AGV Web 服務啟動檔案
│   ├── web_agv_launch/
│   │   └── __init__.py      # Python 模組初始化
│   ├── resource/           # ROS 2 資源檔案
│   │   └── web_agv_launch  # 套件標記檔案
│   ├── test/               # 標準測試檔案
│   │   ├── test_copyright.py  # 版權檢查
│   │   ├── test_flake8.py     # 程式碼風格檢查
│   │   └── test_pep257.py     # 文檔字串檢查
│   ├── setup.py            # 套件設定 (Launch-only package)
│   ├── setup.cfg           # 建置配置
│   └── package.xml         # ROS 2 套件資訊
└── ecs_launch/             # 🔧 ECS 設備控制系統啟動包
    ├── launch/
    │   └── launch.py        # ECS 系統啟動檔案
    ├── ecs_launch/
    │   └── __init__.py      # Python 模組初始化
    ├── resource/           # ROS 2 資源檔案
    │   └── ecs_launch      # 套件標記檔案
    ├── setup.py            # 套件設定 (Launch-only package)
    ├── setup.cfg           # 建置配置
    └── package.xml         # ROS 2 套件資訊
```

### 套件特性
- **純 Launch 套件**: 三個套件都專注於 Launch 檔案，無可執行節點
- **標準結構**: 遵循 ROS 2 套件標準結構和命名慣例
- **測試支援**: web_agv_launch 包含標準的 ROS 2 測試檔案

## 🚀 核心功能 (基於實際代碼驗證)

### 🌐 web_api_launch - Web API 服務群組
提供完整的 Web 服務堆疊啟動，整合用戶界面和 API 服務：

**🔧 啟動的節點**:
- **`agvc_ui_server`** (agvcui 套件) - AGV 車隊管理界面 (Port 8001)
- **`op_ui_server`** (opui 套件) - 操作員任務管理界面 (Port 8002)
- **`web_api_server`** (web_api 套件) - 核心 Web API Gateway (Port 8000)

**⚙️ 配置特性**:
- **命名空間**: 統一使用 `agvc` 命名空間
- **配置檔案**: `/app/config/web_api_config.yaml` (已定義但節點自行處理配置)
- **服務群組**: 三個服務協同提供完整的 Web 功能棧

### 🔧 ecs_launch - ECS 設備控制系統
專注於設備控制和 PLC 通訊的核心服務啟動：

**🔧 啟動的節點**:
- **`plc_service`** (plc_proxy 套件) - PLC 通訊代理服務
- **`ecs_core`** (ecs 套件) - 設備控制系統核心節點

**⚙️ 配置特性**:
- **命名空間**: 統一使用 `agvc` 命名空間
- **配置檔案**: `/app/config/ecs_config.yaml` (參數化配置)
- **備用節點**: `door_controller_node_mqtt` (已註解，待需要時啟用)
- **輸出設定**: 所有節點輸出到螢幕 (`output="screen"`)

### 🚗 web_agv_launch - AGV Web 服務
專注於 AGV 車載系統的 Web 監控介面啟動：

**🔧 啟動的節點**:
- **`agv_ui_server`** (agvui 套件) - AGV 車載監控介面 (Port 8003)

**⚙️ 配置特性**:
- **命名空間**: 使用 `agv` 命名空間 (與 AGVC 區隔)
- **車載系統**: 專門為 AGV 車載環境設計
- **輸出設定**: 輸出到螢幕 (`output="screen"`)

## 🚀 ROS 2 啟動配置專用開發

**⚠️ 通用開發環境請參考**: ../../CLAUDE.md 開發指導章節

### ROS 2 啟動配置特定啟動
```bash
# 【推薦方式】透過根目錄統一工具
# 參考: ../../CLAUDE.md 開發指導

# 【直接啟動】Launch 配置
cd /app/launch_ws
build_ws launch_ws
ros2 launch launch_ws web_api_group.launch.py    # Web API 服務群組
ros2 launch launch_ws ecs_system.launch.py       # ECS 系統啟動
```

### 建置工作空間
```bash
# 建置整個工作空間 (容器內)
build_ws launch_ws

# 或使用 colcon 直接建置
colcon build --packages-select web_api_launch ecs_launch

# 檢查建置結果
ls -la install/
```

### 🚀 服務啟動

#### 🌐 Web API 服務群組啟動
```bash
# 啟動完整 Web 服務群組 (在 AGVC 容器內)
ros2 launch web_api_launch launch.py

# 檢查啟動的節點
ros2 node list | grep agvc
# 預期輸出:
# /agvc/agvc_ui_server    # AGV 車隊管理界面
# /agvc/op_ui_server      # 操作員界面
# /agvc/web_api_server    # 核心 Web API Gateway

# 驗證 Web 服務可用性
curl http://localhost:8000/health     # Web API 健康檢查
curl http://localhost:8001/           # AGVCUI 界面
curl http://localhost:8002/           # OPUI 界面
```

#### 🔧 ECS 設備控制系統啟動
```bash
# 啟動 ECS 設備控制系統 (在 AGVC 容器內)
ros2 launch ecs_launch launch.py

# 檢查啟動的節點
ros2 node list | grep agvc
# 預期輸出:
# /agvc/plc_service       # PLC 通訊代理
# /agvc/ecs_core          # ECS 核心節點

# 檢查服務狀態
ros2 service list | grep agvc
ros2 topic list | grep agvc
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

## 🚨 ROS 2 啟動配置專項故障排除

**⚠️ 通用故障排除請參考**: ../../CLAUDE.md 故障排除章節

### ROS 2 啟動配置特有問題
- @docs-ai/operations/guides/system-diagnostics.md - 系統診斷工具
- @docs-ai/operations/tools/unified-tools.md - 統一工具系統

### 常見問題排查

#### 🔧 Launch 套件找不到
```bash
# 檢查 Launch 套件是否存在
ros2 pkg list | grep -E "(web_api_launch|ecs_launch)"

# 檢查依賴套件是否存在
ros2 pkg list | grep -E "(web_api|agvcui|opui|ecs|plc_proxy)"

# 確認工作空間已正確載入
echo $ROS_PACKAGE_PATH
```

#### 🚨 節點啟動失敗
```bash
# 檢查詳細錯誤訊息
ros2 launch web_api_launch launch.py --debug
ros2 launch ecs_launch launch.py --debug

# 檢查配置檔案是否存在
ls -la /app/config/web_api_config.yaml
ls -la /app/config/ecs_config.yaml

# 檢查個別節點是否可以啟動
ros2 run web_api api_server
ros2 run ecs ecs_core
```

#### 🌐 Web 服務無法連接
```bash
# 檢查網路端口佔用
netstat -tlnp | grep -E "(8000|8001|8002)"

# 檢查服務狀態
ros2 node info /agvc/web_api_server
ros2 node info /agvc/agvc_ui_server

# 檢查服務健康狀態
curl -v http://localhost:8000/health
```

## 💡 重要提醒

### 開發環境要求
- **⚠️ 容器執行要求**: 所有 ROS 2 程式必須在 AGVC Docker 容器內執行
- **📋 工作空間載入**: 必須正確載入 AGVC 工作空間 (`agvc_source` 或 `all_source`)
- **🔧 配置檔案依賴**: 確認 `/app/config/` 目錄下的配置檔案存在

### 工具使用策略
詳細工具指導請參考: @docs-ai/operations/tools/unified-tools.md

### 系統架構特點
- **🏗️ 純 Launch 設計**: 兩個套件專注於啟動編排，無獨立可執行檔案
- **🏷️ 命名空間統一**: 所有節點使用 `agvc` 命名空間，避免衝突
- **📦 模組化服務**: Web 服務群組和 ECS 系統獨立管理，可分別啟動
- **⚙️ 參數化配置**: ECS 系統使用 YAML 配置檔案，Web 服務由節點自行管理配置

## 🔗 系統整合

### 與其他模組整合
- **web_api_ws**: 提供 Web API Gateway 的統一啟動入口
- **agvcui/opui**: 整合用戶界面服務的集中啟動
- **ecs_ws**: 設備控制系統的服務編排
- **plc_proxy_ws**: PLC 通訊服務的統一管理

### 外部系統依賴
- **ROS 2 Jazzy**: 核心 Launch 系統支援
- **AGVC Docker 環境**: 完整的容器化運行環境
- **Zenoh RMW**: 跨容器通訊機制
- **配置檔案系統**: `/app/config/` 下的 YAML 配置檔案

## 🔗 交叉引用

### 相關模組
- **Web API 服務**: `../web_api_ws/CLAUDE.md` - Web API 服務群組啟動
- **ECS 系統**: `../ecs_ws/CLAUDE.md` - 設備控制系統啟動

### 通用支援
詳細指導請參考: ../../CLAUDE.md 交叉引用章節
- 統一工具: @docs-ai/operations/tools/unified-tools.md