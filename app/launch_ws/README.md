# 啟動工作空間 (launch_ws)

## 📋 基本資訊

**啟動狀態**: ⚠️ 手動啟動 (未在容器啟動腳本中自動啟動)
**運行環境**: 🖥️ AGVC 管理系統 (主要)
**主要功能**: Launch 檔案管理和系統啟動配置
**依賴狀態**: 純系統套件，無其他工作空間依賴
**手動啟動**: 可使用 `ros2 launch ecs_launch launch.py` 或 `ros2 launch web_api_launch launch.py` 啟動

## 📋 專案概述

啟動工作空間是 RosAGV 系統的統一啟動和配置管理核心，提供各種系統組件的 Launch 檔案和配置模板。該工作空間實現了模組化的啟動管理，支援 ECS 系統和 Web API 系統的統一啟動，簡化了複雜系統的部署流程。

此工作空間作為 AGVC 管理系統的重要組件，提供了完整的啟動控制邏輯，包括 ECS 啟動、Web API 啟動、參數配置管理等。系統採用標準的 ROS 2 Launch 架構，支援多種啟動模式和配置選項，並提供完整的參數管理和依賴處理。

**重要特點**: 實現了完整的 ECS Launch 和 Web API Launch 檔案，支援統一的系統啟動和參數配置管理，並提供靈活的啟動選項和配置模板。

## 🔗 依賴關係

### 系統套件依賴
- **ROS 2**: `launch`, `launch_ros`, `launch.actions`, `launch.substitutions`
- **Python 標準庫**: `setuptools`, `glob`, `os`

### 被依賴的工作空間
- **ecs_ws**: 透過 `ecs_launch` 啟動 ECS 系統
- **web_api_ws**: 透過 `web_api_launch` 啟動 Web API 系統
- **plc_proxy_ws**: 透過 Launch 檔案啟動 PLC 服務
- **agvcui**: 透過 Launch 檔案啟動 AGVC UI 服務
- **opui**: 透過 Launch 檔案啟動 OP UI 服務

### 外部依賴
- **配置檔案**: `/app/config/ecs_config.yaml`, `/app/config/web_api_config.yaml`

## 🏗️ 專案結構

```
launch_ws/
├── src/                           # 原始碼
│   ├── ecs_launch/               # ECS 系統啟動套件
│   │   ├── launch/
│   │   │   └── launch.py         # ECS 系統啟動檔案 (完整實作)
│   │   ├── resource/             # 資源檔案
│   │   │   └── ecs_launch        # 套件資源標記
│   │   ├── package.xml           # 套件配置
│   │   ├── setup.py              # Python 套件設定 (僅系統套件)
│   │   └── setup.cfg             # 建置配置
│   └── web_api_launch/           # Web API 系統啟動套件
│       ├── launch/
│       │   └── launch.py         # Web API 系統啟動檔案 (完整實作)
│       ├── resource/             # 資源檔案
│       │   └── web_api_launch    # 套件資源標記
│       ├── package.xml           # 套件配置
│       ├── setup.py              # Python 套件設定 (僅系統套件)
│       └── setup.cfg             # 建置配置
├── build/                         # 建置輸出目錄
├── install/                       # 安裝目錄
└── log/                          # 日誌目錄
```

## ⚙️ 主要功能

### 1. ECS Launch (ecs_launch/launch.py)
**ECS 系統啟動管理**:
- **PLC 服務啟動**: 啟動 plc_proxy 套件的 plc_service 節點
- **ECS 核心啟動**: 啟動 ecs 套件的 ecs_core 節點
- **參數配置**: 使用 `/app/config/ecs_config.yaml` 配置檔案
- **命名空間管理**: 統一使用 `agvc` 命名空間

**啟動的節點**:
- `plc_service` (plc_proxy 套件) - PLC 通訊服務
- `ecs_core` (ecs 套件) - ECS 核心控制

### 2. Web API Launch (web_api_launch/launch.py)
**Web API 系統啟動管理**:
- **AGVC UI 服務**: 啟動 agvcui 套件的 agvc_ui_server 節點
- **OP UI 服務**: 啟動 opui 套件的 op_ui_server 節點
- **Web API 服務**: 啟動 web_api 套件的 api_server 節點
- **參數配置**: 使用 `/app/config/web_api_config.yaml` 配置檔案
- **命名空間管理**: 統一使用 `agvc` 命名空間

**啟動的節點**:
- `agvc_ui_server` (agvcui 套件) - AGVC 管理介面
- `op_ui_server` (opui 套件) - 操作員介面
- `web_api_server` (web_api 套件) - Web API 服務

### 3. 配置管理功能
**參數配置支援**:
- **LaunchConfiguration**: 支援動態參數配置
- **DeclareLaunchArgument**: 支援啟動參數宣告
- **配置檔案路徑**: 統一的配置檔案路徑管理
- **參數傳遞**: 完整的參數傳遞機制

## 🔧 核心 API

### ECS Launch 使用
```bash
# 使用預設配置啟動 ECS 系統
ros2 launch ecs_launch launch.py

# 使用自訂配置檔案啟動
ros2 launch ecs_launch launch.py param_file:=/path/to/custom_config.yaml

# 檢查啟動的節點
ros2 node list | grep agvc
```

### Web API Launch 使用
```bash
# 使用預設配置啟動 Web API 系統
ros2 launch web_api_launch launch.py

# 使用自訂配置檔案啟動
ros2 launch web_api_launch launch.py param_file:=/path/to/custom_config.yaml

# 檢查啟動的節點
ros2 node list | grep agvc
```

### 自訂 Launch 檔案範例
```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 定義配置檔案參數
    param_file_arg = DeclareLaunchArgument(
        'param_file',
        default_value='/app/config/custom_config.yaml',
        description='Path to parameter file'
    )

    # 定義節點
    custom_node = Node(
        package='your_package',
        executable='your_executable',
        name='your_node_name',
        namespace='agvc',
        parameters=[LaunchConfiguration('param_file')],
        output='screen'
    )

    return LaunchDescription([
        param_file_arg,
        custom_node
    ])
```

### 參數配置範例
```yaml
# ECS 配置檔案範例 (/app/config/ecs_config.yaml)
ecs_core:
  ros__parameters:
    update_rate: 10.0
    enable_monitoring: true

plc_service:
  ros__parameters:
    plc_ip: "192.168.1.100"
    port: 502
```

## 🧪 測試方法

### 1. 建置和測試
```bash
# 建置工作空間
source /opt/ros/jazzy/setup.bash && source /opt/ws_rmw_zenoh/install/setup.bash && cd /app/launch_ws && colcon build
source install/setup.bash

# 執行測試
colcon test
colcon test-result --verbose
```

### 2. 系統套件測試
```bash
# 測試 ROS 2 Launch 套件
python3 -c "
import launch
import launch_ros
print('✅ ROS 2 Launch 套件可用')
print(f'launch 位置: {launch.__file__}')
print(f'launch_ros 位置: {launch_ros.__file__}')
"

# 測試 setuptools 套件
python3 -c "
import setuptools
print('✅ setuptools 可用')
print(f'setuptools 版本: {setuptools.__version__}')
"
```

### 3. Launch 套件測試
```bash
# 測試 ECS Launch 套件
ros2 pkg list | grep ecs_launch
ros2 launch ecs_launch --help

# 測試 Web API Launch 套件
ros2 pkg list | grep web_api_launch
ros2 launch web_api_launch --help

# 檢查 Launch 檔案安裝位置
ls -la /app/launch_ws/install/ecs_launch/share/ecs_launch/launch/
ls -la /app/launch_ws/install/web_api_launch/share/web_api_launch/launch/
```

### 4. Launch 檔案語法測試
```bash
# 測試 ECS Launch 檔案語法
ros2 launch ecs_launch launch.py --show-args

# 測試 Web API Launch 檔案語法
ros2 launch web_api_launch launch.py --show-args

# 檢查 Launch 檔案內容
cat /app/launch_ws/src/ecs_launch/launch/launch.py
cat /app/launch_ws/src/web_api_launch/launch/launch.py
```

### 5. ECS Launch 功能測試 (需要相關套件)
```bash
# 檢查 ECS Launch 依賴套件
ros2 pkg list | grep -E "(plc_proxy|ecs)"

# 測試 ECS Launch 啟動 (乾跑模式)
ros2 launch ecs_launch launch.py --show-args

# 如果相關套件存在，可以測試實際啟動
# ros2 launch ecs_launch launch.py &
# sleep 10
# ros2 node list | grep agvc
# pkill -f ecs_launch
```

### 6. Web API Launch 功能測試 (需要相關套件)
```bash
# 檢查 Web API Launch 依賴套件
ros2 pkg list | grep -E "(agvcui|opui|web_api)"

# 測試 Web API Launch 啟動 (乾跑模式)
ros2 launch web_api_launch launch.py --show-args

# 如果相關套件存在，可以測試實際啟動
# ros2 launch web_api_launch launch.py &
# sleep 10
# ros2 node list | grep agvc
# pkill -f web_api_launch
```

## 🚀 使用方法

### 1. 建置工作空間
```bash
# 載入 ROS 2 環境並建置
source /opt/ros/jazzy/setup.bash && source /opt/ws_rmw_zenoh/install/setup.bash && cd /app/launch_ws && colcon build
source install/setup.bash
```

### 2. 啟動 ECS 系統 (手動啟動)
```bash
# 方法 1: 使用預設配置啟動
ros2 launch ecs_launch launch.py

# 方法 2: 使用自訂配置啟動
ros2 launch ecs_launch launch.py param_file:=/path/to/custom_config.yaml

# 檢查啟動狀態
ros2 node list | grep agvc
```

### 3. 啟動 Web API 系統 (手動啟動)
```bash
# 方法 1: 使用預設配置啟動
ros2 launch web_api_launch launch.py

# 方法 2: 使用自訂配置啟動
ros2 launch web_api_launch launch.py param_file:=/path/to/custom_config.yaml

# 檢查啟動狀態
ros2 node list | grep agvc
```

### 4. 檢查 Launch 系統狀態
```bash
# 檢查 Launch 相關進程
ps aux | grep -E "(ecs_launch|web_api_launch)"

# 檢查啟動的 ROS 2 節點
ros2 node list

# 檢查節點詳細資訊
ros2 node info /agvc/ecs_core
ros2 node info /agvc/agvc_ui_server
```

### 5. 停止 Launch 系統
```bash
# 使用 Ctrl+C 優雅關閉
# 或強制終止
pkill -f ecs_launch
pkill -f web_api_launch

# 檢查是否已停止
ps aux | grep -E "(ecs_launch|web_api_launch)"
```

## 🔧 故障排除

### 1. Launch 套件建置失敗
**症狀**: `colcon build` 失敗或套件無法找到
**解決方法**:
```bash
# 檢查工作空間是否正確建置
cd /app/launch_ws
colcon build

# 確認環境已載入
source install/setup.bash

# 檢查套件是否正確安裝
ros2 pkg list | grep -E "(ecs_launch|web_api_launch)"

# 檢查 Python 路徑
python3 -c "import sys; print('\\n'.join(sys.path))"
```

### 2. Launch 檔案啟動失敗
**症狀**: `ros2 launch ecs_launch launch.py` 無法啟動
**解決方法**:
```bash
# 檢查 Launch 檔案語法
python3 -m py_compile /app/launch_ws/src/ecs_launch/launch/launch.py
python3 -m py_compile /app/launch_ws/src/web_api_launch/launch/launch.py

# 檢查 Launch 檔案安裝位置
ls -la /app/launch_ws/install/ecs_launch/share/ecs_launch/launch/
ls -la /app/launch_ws/install/web_api_launch/share/web_api_launch/launch/

# 測試 Launch 檔案語法
ros2 launch ecs_launch launch.py --show-args
ros2 launch web_api_launch launch.py --show-args
```

### 3. 配置檔案問題
**症狀**: Launch 檔案無法載入配置檔案
**解決方法**:
```bash
# 檢查配置檔案是否存在
ls -la /app/config/ecs_config.yaml
ls -la /app/config/web_api_config.yaml

# 驗證配置檔案格式
python3 -c "
import yaml
try:
    with open('/app/config/ecs_config.yaml', 'r') as f:
        config = yaml.safe_load(f)
    print('✅ ecs_config.yaml 格式正確')
except Exception as e:
    print(f'❌ ecs_config.yaml 格式錯誤: {e}')
"

# 檢查配置檔案權限
ls -la /app/config/*.yaml
```

### 4. 目標套件不存在
**症狀**: Launch 檔案啟動但節點無法找到
**解決方法**:
```bash
# 檢查目標套件是否存在
ros2 pkg list | grep -E "(plc_proxy|ecs|agvcui|opui|web_api)"

# 檢查節點可執行檔案
ros2 pkg executables plc_proxy
ros2 pkg executables ecs
ros2 pkg executables agvcui

# 手動測試節點啟動
ros2 run plc_proxy plc_service --help
ros2 run ecs ecs_core --help
```

### 5. 命名空間問題
**症狀**: 節點啟動但無法在預期命名空間中找到
**解決方法**:
```bash
# 檢查所有節點
ros2 node list

# 檢查特定命名空間
ros2 node list | grep agvc

# 檢查節點詳細資訊
ros2 node info /agvc/plc_service
ros2 node info /agvc/ecs_core

# 檢查主題和服務
ros2 topic list | grep agvc
ros2 service list | grep agvc
## ⚙️ 配置說明

### ECS Launch 配置
```python
# ecs_launch/launch.py 配置參數
param_file = '/app/config/ecs_config.yaml'  # ECS 配置檔案路徑
namespace = 'agvc'                          # 統一命名空間

# 啟動的節點
nodes = [
    'plc_service',    # PLC 通訊服務 (plc_proxy 套件)
    'ecs_core'        # ECS 核心控制 (ecs 套件)
]
```

### Web API Launch 配置
```python
# web_api_launch/launch.py 配置參數
param_file = '/app/config/web_api_config.yaml'  # Web API 配置檔案路徑
namespace = 'agvc'                              # 統一命名空間

# 啟動的節點
nodes = [
    'agvc_ui_server',  # AGVC 管理介面 (agvcui 套件)
    'op_ui_server',    # 操作員介面 (opui 套件)
    'web_api_server'   # Web API 服務 (web_api 套件)
]
```

### Launch 參數配置
```bash
# 使用自訂配置檔案
ros2 launch ecs_launch launch.py param_file:=/path/to/custom_config.yaml
ros2 launch web_api_launch launch.py param_file:=/path/to/custom_config.yaml

# 檢查可用參數
ros2 launch ecs_launch launch.py --show-args
ros2 launch web_api_launch launch.py --show-args
```

### 配置檔案範例
```yaml
# /app/config/ecs_config.yaml
ecs_core:
  ros__parameters:
    update_rate: 10.0
    enable_monitoring: true

plc_service:
  ros__parameters:
    plc_ip: "192.168.12.224"
    plc_port: 8501
## 🔗 相關文檔

- **ecs_ws**: ECS 系統，透過 `ecs_launch` 啟動 ECS 核心和 PLC 服務
- **web_api_ws**: Web API 系統，透過 `web_api_launch` 啟動 Web 服務
- **plc_proxy_ws**: PLC 代理服務，被 Launch 檔案啟動
- **agvcui**: AGVC 管理介面，被 Web API Launch 啟動
- **opui**: 操作員介面，被 Web API Launch 啟動
- **ROS 2 Launch 文檔**: [ROS 2 Launch Documentation](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Launch-Main.html)

## 📋 ToDo 清單

### 🔴 高優先級 (緊急)
- [ ] 完善 Launch 檔案錯誤處理機制
- [ ] 新增 Launch 檔案參數驗證功能
- [ ] 最佳化節點啟動順序和依賴關係

### 🟡 中優先級 (重要)
- [ ] 新增更多系統組件的 Launch 檔案
- [ ] 實作組合式 Launch 檔案 (多系統同時啟動)
- [ ] 新增條件式啟動邏輯和環境檢測
- [ ] 完善 Launch 檔案測試覆蓋率

### 🟢 低優先級 (改善)
- [ ] 新增 Launch 狀態監控和健康檢查
- [ ] 支援動態參數調整和熱重載
- [ ] 新增 Launch 檔案版本管理
- [ ] 實作 Launch 效能監控和分析

### 🔧 技術債務
- [ ] 重構 Launch 檔案結構，提高可維護性
- [ ] 統一 Launch 檔案註解和文檔格式
- [ ] 改善 Launch 檔案最佳實踐指南

### 📊 完成度追蹤
- ✅ ECS Launch 檔案 (100%)
- ✅ Web API Launch 檔案 (100%)
- ✅ 基礎 Launch 架構 (100%)
- ✅ 參數配置支援 (100%)
- ⚠️ 錯誤處理機制 (70% - 需要改善)
- ⚠️ 測試覆蓋率 (60% - 基礎測試)
- ❌ 進階 Launch 功能 (0% - 未開始)

### 🎯 里程碑
- **v1.0.0**: ✅ 基礎 Launch 功能完成 (當前版本)
- **v1.1.0**: 🚧 錯誤處理和測試改善
- **v2.0.0**: 📋 進階 Launch 功能和監控

### 🏆 重要成就
- ✅ 成功整合到 RosAGV 系統
- ✅ 提供完整的系統啟動管理
- ✅ 實現標準化的 Launch 架構
- ✅ 支援靈活的參數配置
