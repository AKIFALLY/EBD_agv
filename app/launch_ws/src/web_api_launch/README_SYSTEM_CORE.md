# RosAGV 系統核心啟動指南

## 概述
此目錄包含啟動 RosAGV 四個核心系統節點的 Launch 檔案：
- **agvc_database_node** - 資料庫代理服務
- **rcs_core** - RCS 車隊控制核心
- **wcs_base_node** - WCS 基礎決策引擎
- **task_condition_query_node** - 任務條件查詢節點

## 可用的 Launch 檔案

### 1. system_core_launch.py (生產環境推薦)
**功能特性：**
- ✅ 依序啟動，確保依賴關係
- ✅ 自動重啟機制 (最多3次)
- ✅ system 命名空間避免衝突
- ✅ 詳細的狀態日誌和診斷指令

**啟動順序：**
1. T+0s: agvc_database_node (資料庫服務)
2. T+3s: rcs_core (等待資料庫就緒)
3. T+5s: wcs_base_node (等待前面服務就緒)
4. T+7s: task_condition_query_node (等待 WCS 基礎服務就緒)
5. T+10s: 系統就緒通知

### 2. simple_core_launch.py (開發測試版本)
**功能特性：**
- 🔧 同時啟動所有四個節點，無延遲
- 🔧 統一輸出到螢幕便於調試
- 🔧 無自動重啟，簡化測試
- 🔧 適合快速開發和問題排除

## 使用方法

### 前置要求
確保在 AGVC 容器內且已載入工作空間：
```bash
# 進入 AGVC 容器
docker compose -f docker-compose.agvc.yml exec agvc_server bash

# 載入工作空間
source /app/setup.bash
agvc_source  # 或使用 all_source
```

### 生產環境啟動 (推薦)
```bash
# 啟動核心系統 - 生產版本
ros2 launch web_api_launch system_core_launch.py

# 檢查系統狀態
ros2 node list | grep system
ros2 topic list | grep system

# 測試資料庫連接
ros2 service call /system/sql_query db_proxy_interfaces/srv/SqlQuery "sql: 'SELECT 1'"
```

### 開發測試啟動
```bash
# 啟動核心系統 - 開發版本 (需要 xterm)
ros2 launch web_api_launch simple_core_launch.py

# 或者不使用 xterm 的簡化版本
ros2 launch web_api_launch simple_core_launch.py --ros-args -p use_xterm:=false
```

### 自訂參數啟動
```bash
# 使用 debug 日誌等級
ros2 launch web_api_launch system_core_launch.py log_level:=debug

# 檢查可用參數
ros2 launch web_api_launch system_core_launch.py --show-args
```

## 系統診斷

### 啟動後檢查
```bash
# 檢查所有節點狀態
ros2 node list
ros2 node info /system/agvc_database_node
ros2 node info /system/rcs_core  
ros2 node info /system/wcs_base_node
ros2 node info /system/task_condition_query_node

# 檢查服務和主題
ros2 service list | grep system
ros2 topic list | grep system

# 測試資料庫功能
ros2 service call /system/sql_query db_proxy_interfaces/srv/SqlQuery "sql: 'SELECT version()'"

# 使用統一診斷工具
r agvc-check
r quick-diag
r containers-status
```

### 常見問題排除

#### 節點啟動失敗
```bash
# 檢查詳細錯誤
ros2 launch web_api_launch system_core_launch.py --debug

# 檢查個別節點
ros2 run db_proxy agvc_database_node
ros2 run rcs rcs_core
ros2 run wcs_base wcs_base_node

# 檢查依賴服務
r network-check
docker compose -f docker-compose.agvc.yml ps postgres
```

#### 資料庫連接問題
```bash
# 檢查資料庫容器
docker compose -f docker-compose.agvc.yml ps postgres

# 檢查網路連接
ping 192.168.100.254
telnet 192.168.100.254 5432

# 手動測試資料庫連接
python3 -c "
from db_proxy.connection_pool_manager import ConnectionPoolManager
pool = ConnectionPoolManager('postgresql+psycopg2://agvc:password@192.168.100.254/agvc')
print('資料庫連接成功')
"
```

## 停止系統

### 正常停止
```bash
# 使用 Ctrl+C 優雅停止
# Launch 檔案會自動清理所有節點
```

### 強制停止
```bash
# 停止所有 system 命名空間節點
ros2 lifecycle set /system/agvc_database_node shutdown
ros2 lifecycle set /system/rcs_core shutdown
ros2 lifecycle set /system/wcs_base_node shutdown

# 或使用系統工具
pkill -f agvc_database_node
pkill -f rcs_core
pkill -f wcs_base_node
```

## 整合說明

### 與現有系統整合
這個核心系統 Launch 檔案與現有的 launch_ws 系統整合：

- **web_api_launch/launch/launch.py** - Web API 服務群組
- **ecs_launch/launch/launch.py** - ECS 設備控制系統
- **system_core_launch.py** - 核心系統 (新增)
- **simple_core_launch.py** - 開發測試版本 (新增)

### 啟動順序建議
完整系統啟動的建議順序：
1. **核心系統**: `ros2 launch web_api_launch system_core_launch.py`
2. **Web API**: `ros2 launch web_api_launch launch.py`
3. **ECS 系統**: `ros2 launch ecs_launch launch.py`

### 命名空間設計
- **system** - 核心系統節點 (database, rcs, wcs_base)
- **agvc** - Web API 和 ECS 系統節點
- 避免命名空間衝突，便於管理和診斷

## 技術備註

### 依賴關係
- **agvc_database_node** 必須先啟動，其他節點依賴資料庫服務
- **rcs_core** 需要資料庫連接進行車隊管理
- **wcs_base_node** 需要資料庫進行任務查詢和狀態更新
- **task_condition_query_node** 需要資料庫和 WCS 基礎服務支援

### 自動重啟策略
- **respawn=True**: 啟用自動重啟
- **respawn_delay=5.0**: 重啟延遲5秒
- **respawn_max=3**: 最多重啟3次，避免無限重啟

### 效能考量
- 使用 `system` 命名空間避免與其他服務衝突
- 節點間使用 ROS 2 服務和主題進行通訊
- 資料庫連接池確保高效的資料存取