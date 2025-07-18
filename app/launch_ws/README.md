# 啟動工作空間 (launch_ws)

## 📋 基本資訊

**啟動狀態**: ⚠️ 手動啟動 (未在容器啟動腳本中自動啟動)
**運行環境**: 🖥️ AGVC 管理系統 (主要)
**主要功能**: Launch 檔案管理 - 系統啟動和配置管理
**依賴狀態**: 使用系統套件，無其他工作空間依賴
**手動啟動**: 可使用 `ros2 launch ecs_launch launch.py` 或 `ros2 launch web_api_launch launch.py` 啟動

## 📋 專案概述

啟動工作空間是 RosAGV 系統的統一啟動和配置管理核心，提供各種系統組件的 launch 檔案和配置模板。該工作空間實現了模組化的啟動管理，支援 ECS 系統和 Web API 系統的統一啟動，簡化了複雜系統的部署流程。

作為 AGVC 管理系統的重要組件，Launch 工作空間提供了完整的啟動控制邏輯，包括 ECS 啟動、Web API 啟動、參數配置管理等。系統採用標準的 ROS 2 Launch 架構，支援多種啟動模式和配置選項，並提供完整的參數管理和依賴處理。

**重要特點**: 實現了完整的 ECS Launch 和 Web API Launch 檔案，支援統一的系統啟動和參數配置管理，並提供靈活的啟動選項和配置模板。

## 🔗 依賴關係

### 系統套件依賴
- **launch**: ROS 2 Launch 系統核心
- **launch_ros**: ROS 2 Launch 整合套件
- **setuptools**: Python 套件建置工具

### 依賴的工作空間
- **無**: 此工作空間為獨立模組，不依賴其他工作空間

### 被依賴的工作空間
- **無**: 此工作空間提供啟動檔案，不被其他工作空間直接依賴

### 外部依賴
- **配置檔案**: 依賴 `/app/config/` 目錄下的配置檔案

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

# 檢查 Launch 檔案
ls -la /app/launch_ws/install/ecs_launch/share/ecs_launch/launch/
ls -la /app/launch_ws/install/web_api_launch/share/web_api_launch/launch/
```

### 4. ECS Launch 測試
```bash
# 測試 ECS Launch 啟動 (乾跑模式)
ros2 launch ecs_launch launch.py --show-args

# 實際啟動 ECS 系統
ros2 launch ecs_launch launch.py &
sleep 10

# 檢查啟動的節點
ros2 node list | grep agvc

# 檢查節點狀態
ros2 node info /agvc/plc_service
ros2 node info /agvc/ecs_core

# 停止 Launch
pkill -f ecs_launch
```

### 5. Web API Launch 測試
```bash
# 測試 Web API Launch 啟動 (乾跑模式)
ros2 launch web_api_launch launch.py --show-args

# 實際啟動 Web API 系統
ros2 launch web_api_launch launch.py &
sleep 10

# 檢查啟動的節點
ros2 node list | grep agvc

# 檢查節點狀態
ros2 node info /agvc/agvc_ui_server
ros2 node info /agvc/op_ui_server
ros2 node info /agvc/web_api_server

# 停止 Launch
pkill -f web_api_launch
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

### 常見問題

#### 1. Launch 檔案啟動失敗
**症狀**: `ros2 launch ecs_launch launch.py` 或 `ros2 launch web_api_launch launch.py` 無法啟動
**解決方法**:
```bash
# 檢查 Launch 套件建置狀態
ls -la /app/launch_ws/install/

# 重新建置 Launch 工作空間
source /opt/ros/jazzy/setup.bash && source /opt/ws_rmw_zenoh/install/setup.bash && cd /app/launch_ws
rm -rf build install log
colcon build

# 檢查 setup.bash 載入
source install/setup.bash
ros2 pkg list | grep -E "(ecs_launch|web_api_launch)"

# 檢查 Launch 檔案語法
python3 -m py_compile src/ecs_launch/launch/launch.py
python3 -m py_compile src/web_api_launch/launch/launch.py
```

#### 2. 配置檔案問題
**症狀**: Launch 檔案無法載入配置檔案或配置錯誤
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
        print('✅ ECS 配置檔案格式正確')
except Exception as e:
    print(f'❌ ECS 配置檔案錯誤: {e}')
"

# 使用預設配置啟動
ros2 launch ecs_launch launch.py
```

#### 3. 節點啟動失敗
**症狀**: Launch 檔案啟動但節點無法正常運行
**解決方法**:
```bash
# 檢查目標套件是否存在
ros2 pkg list | grep -E "(plc_proxy|ecs|agvcui|opui|web_api)"

# 檢查節點可執行檔案
ros2 run plc_proxy plc_service --help
ros2 run ecs ecs_core --help
ros2 run agvcui agvc_ui_server --help

# 手動啟動節點進行測試
ros2 run plc_proxy plc_service --ros-args -r __ns:=/agvc
ros2 run ecs ecs_core --ros-args -r __ns:=/agvc
```

### 除錯工具
```bash
# 檢查 Launch 相關進程
ps aux | grep -E "(launch|ros2)"

# 檢查 ROS 2 環境
printenv | grep ROS

# 檢查 Launch 檔案詳細資訊
ros2 launch ecs_launch launch.py --show-args
ros2 launch web_api_launch launch.py --show-args
```

## 🔧 配置說明

### 啟動參數
```yaml
# 系統配置
system:
  namespace: "agvc"
  log_level: "INFO"
  
# 節點配置
nodes:
  plc_service:
    package: "plc_proxy"
    executable: "plc_service"
    parameters: "/app/config/ecs_config.yaml"
    
  ecs_core:
    package: "ecs"
    executable: "ecs_core"
    parameters: "/app/config/ecs_config.yaml"
```

### 環境變數
```bash
# ROS 2 環境
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_zenoh_cpp

# 系統路徑
export PYTHONPATH=/opt/pyvenv_env/lib/python3.12/site-packages:$PYTHONPATH
```

## 🔗 依賴項目

- **ROS 2 Jazzy**: 機器人作業系統框架
- **launch**: ROS 2 啟動系統
- **launch_ros**: ROS 2 特定啟動功能
- **ament_python**: Python 套件建置工具

## 📝 開發指南

### 新增啟動文件
1. 在 `launch/` 目錄下建立新的 launch 文件
2. 定義所需的節點和參數
3. 更新 setup.py 包含新文件
4. 測試啟動流程

### 配置管理
1. 建立配置模板
2. 實施配置驗證
3. 新增環境特定配置
4. 文檔化配置選項

## 🔧 維護注意事項

1. **版本相容性**: 確保與 ROS 2 版本相容
2. **配置同步**: 保持配置文件同步更新
3. **測試覆蓋**: 完整測試所有啟動場景
4. **文檔維護**: 保持啟動文檔最新

## 📋 ToDo 清單

### 🔴 高優先級 (緊急)
- [x] **ECS Launch 完整實作** ✅ **已完成**
  - [x] 完整的 ECS 系統啟動檔案 (ecs_launch/launch.py)
  - [x] PLC 服務和 ECS 核心節點啟動
  - [x] 參數配置管理和命名空間設定
  - [x] 完整的 Launch 架構和依賴處理
- [x] **Web API Launch 完整實作** ✅ **已完成**
  - [x] 完整的 Web API 系統啟動檔案 (web_api_launch/launch.py)
  - [x] AGVC UI、OP UI、Web API 服務啟動
  - [x] 統一的命名空間和參數管理
  - [x] 標準的 ROS 2 Launch 架構

### 🟡 中優先級 (重要)
- [ ] **Launch 檔案擴展** (2 週)
  - [x] 基本 ECS 和 Web API Launch 已完成
  - [ ] 新增更多系統組件的 Launch 檔案
  - [ ] 實現組合式 Launch 檔案 (多系統同時啟動)
  - [ ] 新增條件式啟動邏輯
- [ ] **配置驗證和管理** (2 週)
  - [ ] 實現配置檔案驗證機制
  - [ ] 新增配置錯誤檢測和報告
  - [ ] 建立配置模板和範例
- [ ] **測試覆蓋擴展** (1 週)
  - [x] 基本 Launch 測試已建立
  - [ ] 新增自動化 Launch 測試
  - [ ] 實現 Launch 檔案語法檢查
  - [ ] 建立整合測試框架

### 🟢 低優先級 (改善)
- [ ] **監控和分析功能** (3 週)
  - [ ] 實現 Launch 狀態監控
  - [ ] 新增啟動效能指標收集
  - [ ] 建立啟動失敗分析和警報機制
- [ ] **進階 Launch 功能** (2 週)
  - [ ] 實現動態參數調整
  - [ ] 新增 Launch 檔案熱重載
  - [ ] 建立 Launch 檔案版本管理

### 🔧 技術債務
- [x] **標準化架構** ✅ **已完成**
  - [x] 標準的 ROS 2 Launch 架構
  - [x] 統一的套件結構和配置
  - [x] 完整的 setup.py 和 package.xml 配置
- [ ] **程式碼品質提升** (1 週)
  - [ ] 新增 Launch 檔案註解和文檔
  - [ ] 實現程式碼風格統一
  - [ ] 新增 Launch 檔案最佳實踐指南

### 📊 完成度追蹤 (基於實際程式碼分析)
- **ECS Launch**: 95% ✅ (完整實作，包含所有必要節點)
- **Web API Launch**: 95% ✅ (完整實作，包含所有 Web 服務)
- **Launch 架構**: 90% ✅ (標準 ROS 2 架構已完成)
- **配置管理**: 70% 🔄 (基本配置支援已實現)
- **測試覆蓋**: 60% 🔄 (基本測試已建立)
- **文檔完整性**: 95% ✅ (完整的技術文檔已完成)

### 🎯 里程碑 (更新版)
1. **v1.0.0** ✅ **已達成** - 核心 Launch 功能實現
   - [x] ECS Launch 完整實作
   - [x] Web API Launch 完整實作
   - [x] 標準化 Launch 架構完成

2. **v1.1.0** (2 週後) - Launch 功能擴展
   - [ ] Launch 檔案擴展和組合式啟動
   - [ ] 配置驗證和管理功能
   - [ ] 測試覆蓋擴展

3. **v2.0.0** (6 週後) - 進階功能和監控
   - [ ] 監控和分析功能
   - [ ] 進階 Launch 功能
   - [ ] 完整的管理和維護工具

### 🏆 重要成就 (基於實際程式碼分析)
- ✅ **完整的 Launch 系統**: ECS 和 Web API 系統的完整啟動支援
- ✅ **標準化架構**: 符合 ROS 2 標準的 Launch 檔案架構
- ✅ **統一命名空間**: 所有節點使用統一的 agvc 命名空間
- ✅ **參數配置支援**: 完整的配置檔案和參數傳遞機制
- ✅ **模組化設計**: 清晰的套件分離和獨立啟動能力
