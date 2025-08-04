# ROS 2 容器內指令執行格式

## 🎯 適用場景
- 在 Docker 容器內執行 ROS 2 相關指令
- 避免 `ModuleNotFoundError: No module named 'rclpy'` 錯誤
- 提供標準的容器指令執行模式

## ⚠️ 重要原則
**所有 ROS 2 程式必須在 Docker 容器內執行，宿主機無 ROS 2 環境。**

## 🔧 標準指令格式

### AGV 車載容器 (rosagv)
```bash
# ✅ 正確格式：先載入環境再執行指令
docker compose -f docker-compose.yml exec rosagv bash -c "source /app/setup.bash && [你的指令]"

# 範例
docker compose -f docker-compose.yml exec rosagv bash -c "source /app/setup.bash && ros2 topic list"
docker compose -f docker-compose.yml exec rosagv bash -c "source /app/setup.bash && colcon build --packages-select agv_base"
docker compose -f docker-compose.yml exec rosagv bash -c "source /app/setup.bash && python3 -c 'import rclpy; print(\"ROS 2 可用\")'"

# ❌ 錯誤格式：直接執行會發生 ModuleNotFoundError
docker compose -f docker-compose.yml exec rosagv bash -c "ros2 topic list"
docker compose -f docker-compose.yml exec rosagv bash -c "python3 -c 'import rclpy'"
```

### AGVC 管理容器 (agvc_server)
```bash
# ✅ 正確格式：先載入環境再執行指令
docker compose -f docker-compose.agvc.yml exec agvc_server bash -c "source /app/setup.bash && [你的指令]"

# 範例
docker compose -f docker-compose.agvc.yml exec agvc_server bash -c "source /app/setup.bash && ros2 topic list"
docker compose -f docker-compose.agvc.yml exec agvc_server bash -c "source /app/setup.bash && colcon build --packages-select rcs"
docker compose -f docker-compose.agvc.yml exec agvc_server bash -c "source /app/setup.bash && python3 -m db_proxy.sql.db_install"

# ❌ 錯誤格式：直接執行會發生 ModuleNotFoundError
docker compose -f docker-compose.agvc.yml exec agvc_server bash -c "ros2 topic list"
docker compose -f docker-compose.agvc.yml exec agvc_server bash -c "python3 -m db_proxy.sql.db_install"
```

## 🚀 常用開發指令模式

### 建置相關
```bash
# AGV 環境建置
docker compose -f docker-compose.yml exec rosagv bash -c "source /app/setup.bash && colcon build --packages-select [package_name]"

# AGVC 環境建置
docker compose -f docker-compose.agvc.yml exec agvc_server bash -c "source /app/setup.bash && colcon build --packages-select [package_name]"
```

### 測試相關
```bash
# AGV 環境測試
docker compose -f docker-compose.yml exec rosagv bash -c "source /app/setup.bash && colcon test --packages-select [package_name]"

# AGVC 環境測試
docker compose -f docker-compose.agvc.yml exec agvc_server bash -c "source /app/setup.bash && python3 -m pytest [test_file]"
```

### 資料庫相關 (僅 AGVC)
```bash
# 資料庫初始化
docker compose -f docker-compose.agvc.yml exec agvc_server bash -c "source /app/setup.bash && python3 -m db_proxy.sql.db_install"

# 資料庫服務啟動
docker compose -f docker-compose.agvc.yml exec agvc_server bash -c "source /app/setup.bash && ros2 run db_proxy agvc_database_node"
```

## 💡 記憶技巧

### 檢查清單
執行容器內 ROS 2 指令前，檢查：
- [ ] 是否在正確的容器中？(AGV: `rosagv`, AGVC: `agvc_server`)
- [ ] 是否使用了正確的 docker-compose 檔案？
- [ ] 是否加了 `source /app/setup.bash &&`？
- [ ] 指令是否包含 ROS 2 或 Python 模組導入？

### 常見錯誤信號
看到以下錯誤時，通常是忘記載入環境：
- `ModuleNotFoundError: No module named 'rclpy'`
- `ModuleNotFoundError: No module named 'db_proxy'`
- `command not found: ros2`
- `command not found: colcon`

## 🔗 交叉引用
- 雙環境架構: @docs-ai/context/system/dual-environment.md
- Docker 開發指導: @docs-ai/operations/development/docker-development.md
- ROS 2 開發指導: @docs-ai/operations/development/ros2-development.md