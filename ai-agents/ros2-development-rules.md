# AI Agent ROS2 開發原則

## 工作空間配置
**AGV 工作空間 (11個含共用)**
- 專用: agv_ws, agv_cmd_service_ws, joystick_ws, sensorpart_ws, uno_gpio_ws
- 共用基礎: shared_constants_ws, keyence_plc_ws, plc_proxy_ws, path_algorithm
- 共用應用: db_proxy_ws, web_api_ws (AGVUI), launch_ws

**AGVC 工作空間 (13個含共用)**
- 專用: web_api_ws, db_proxy_ws, ecs_ws, rcs_ws, kuka_wcs_ws, wcs_ws, kuka_fleet_ws
- 共用基礎: shared_constants_ws, keyence_plc_ws, plc_proxy_ws, path_algorithm
- 共用應用: agv_ws (監控用), launch_ws
- ~~已棄用: tafl_ws, tafl_wcs_ws~~

## ROS2 指令執行
```bash
# ⚠️ 必須在 ~/EBD_agv 目錄執行
cd ~/EBD_agv

# ❌ 錯誤：宿主機無 ROS2
ros2 topic list

# ✅ AGV 容器內執行
docker compose -f docker-compose.yml exec rosagv bash -c "source /app/setup.bash && ros2 topic list"

# ✅ AGVC 容器內執行
docker compose -f docker-compose.agvc.yml exec agvc_server bash -c "source /app/setup.bash && ros2 topic list"
```

## 建置指令
```bash
# [容器內] 建置命令
build_all               # 建置所有工作空間 (別名: ba)
all_source              # 載入所有工作空間 (別名: sa)

# [容器內] 特定套件建置
colcon build --packages-select package_name
colcon build --packages-up-to package_name
colcon build --parallel-workers 4

# [宿主機] 透過 docker exec 在容器內建置
docker compose -f docker-compose.agvc.yml exec agvc_server bash -c "source /app/setup.bash && colcon build --packages-select web_api"
```

## 常用 ROS2 指令
```bash
# 節點管理
ros2 node list
ros2 node info /node_name
ros2 run package_name node_name

# 主題操作
ros2 topic list
ros2 topic echo /topic_name
ros2 topic pub /topic_name msg_type "data: value"
ros2 topic hz /topic_name

# 服務操作
ros2 service list
ros2 service type /service_name
ros2 service call /service_name srv_type "{param: value}"

# 參數操作
ros2 param list
ros2 param get /node_name param_name
ros2 param set /node_name param_name value

# 介面查詢
ros2 interface list
ros2 interface show msg_type
```

## 測試執行
```bash
# 執行測試
colcon test --packages-select package_name
colcon test-result --verbose

# Pytest 測試 (Python 套件)
python3 -m pytest tests/
python3 -m pytest tests/test_file.py::test_function
```

## 創建新套件
```bash
# Python 套件
ros2 pkg create --build-type ament_python package_name

# C++ 套件
ros2 pkg create --build-type ament_cmake package_name

# 混合套件
ros2 pkg create --build-type ament_cmake package_name --dependencies rclpy rclcpp
```

## 環境檢查
```bash
# 容器內檢查
check_system_status     # 整體系統狀態
check_ros_env          # ROS2 環境驗證
echo $ROS_DISTRO       # 應顯示: jazzy
echo $RMW_IMPLEMENTATION  # 應顯示: rmw_zenohd
```

## 常見問題解決

### ModuleNotFoundError: No module named 'rclpy'
```bash
# 原因：未載入環境
# 解決：加上 source /app/setup.bash
docker compose -f docker-compose.agvc.yml exec agvc_server bash -c "source /app/setup.bash && python3 -c 'import rclpy'"
```

### 跨容器通訊失敗
```bash
# 檢查 Zenoh Router
check_zenoh_status
ps aux | grep zenoh

# 重啟 ROS2 daemon
ros2 daemon stop
ros2 daemon start
```

### 建置失敗
```bash
# 清理建置目錄
rm -rf build/ install/ log/
colcon build
```

## 服務管理函數開發 (manage_*)

### ⚠️ 重要：所有 manage_* 函數必須遵循標準化規範

**必讀文檔**:
- [manage-function-standard.md](../docs-ai/operations/development/ros2/manage-function-standard.md) - 開發標準與最佳實踐
- [manage-function-template.md](../docs-ai/operations/development/ros2/manage-function-template.md) - 詳細實作模板

### 核心要求

#### 4 階段啟動流程
```bash
階段 1: 啟動前檢查（幂等性驗證）
階段 2: 依賴檢查（資料庫、上游服務、工作空間建置）
階段 3: 啟動服務（記錄父進程、子進程、服務進程 PID）
階段 4: 驗證啟動（多層驗證 + 診斷建議）
```

#### 6 階段停止流程
```bash
階段 1: 優雅停止 (SIGTERM)
階段 2: 強制終止 (SIGKILL)
階段 3: 備份清理（僵屍進程）
階段 4: 殘留進程清理
階段 5: 端口資源釋放
階段 6: 清理臨時文件
```

### 必須支援的子命令
```bash
manage_service_name start    # 啟動服務
manage_service_name stop     # 停止服務
manage_service_name restart  # 重啟服務
manage_service_name status   # 狀態檢查
manage_service_name logs     # 查看日誌
```

### 驗證方法決策
- **ROS2 節點** → 使用 `verify_ros2_node_startup()`
- **Web 服務** → 使用 `wait_for_port_with_retry()`
- **一般進程** → 使用 `verify_process_startup()`

### PID 追蹤規範
```bash
# ✅ 正確：記錄所有相關進程
echo "$PARENT_PID" > "$PID_FILE"      # 父進程
echo "$CHILD_PIDS" >> "$PID_FILE"     # 子進程
echo "$SERVICE_PIDS" >> "$PID_FILE"   # 服務進程

# ❌ 錯誤：只記錄父進程
echo "$PARENT_PID" > "$PID_FILE"
```

### 已標準化的函數範例
- `manage_web_api_launch` - Web 服務管理（最佳實踐）
- `manage_plc_service_agvc` - PLC 服務管理
- `manage_tafl_wcs` - TAFL WCS 流程控制
- `manage_room_task_build` - Room Task Build 節點

## 關鍵規則
1. **ROS2 只在容器內**: 宿主機無 ROS2 環境
2. **載入環境優先**: 執行前必須 `source /app/setup.bash`
3. **工作目錄**: 在 ~/EBD_agv 執行 docker compose
4. **Zenoh 通訊**: 跨容器通訊依賴 Zenoh Router
5. **自動載入**: 使用 `all_source` 自動檢測環境
6. **服務管理標準化**: 所有 `manage_*` 函數必須遵循 4+6 階段標準