# Docker 容器開發指導

## 🎯 適用場景
- 在 Docker 容器內進行 ROS 2 開發
- 容器環境配置和管理
- 跨容器開發工作流程
- 容器內工具使用和最佳實踐

## 📋 RosAGV 容器開發環境

### 雙容器架構
RosAGV 採用雙容器架構，每個容器有不同的開發環境和工具：

```
🚗 AGV 車載容器 (rosagv)
├── 容器名稱: rosagv
├── 網路模式: host
├── 工作空間: 9個 AGV 專用工作空間
└── 主要用途: 即時控制、硬體整合

🖥️ AGVC 管理容器 (agvc_server)
├── 容器名稱: agvc_server
├── 網路模式: bridge (192.168.100.100)
├── 工作空間: 11個 AGVC 專用工作空間
└── 主要用途: 車隊管理、Web 服務
```

### 容器進入方式
```bash
# 進入 AGV 車載容器
docker compose -f docker-compose.yml exec rosagv bash

# 進入 AGVC 管理容器
docker compose -f docker-compose.agvc.yml exec agvc_server bash

# 使用專業工具進入 (推薦)
source scripts/docker-tools/docker-tools.sh
agv_enter      # 進入 AGV 容器
agvc_enter     # 進入 AGVC 容器 (自動載入 agvc_source)
```

### AGVC 容器管理工具
```bash
# 載入 Docker 工具集
source scripts/docker-tools/docker-tools.sh

# AGVC 系統生命週期管理
agvc_start                   # 啟動 AGVC 系統 (所有服務)
agvc_stop                    # 停止 AGVC 系統
agvc_restart                 # 重啟 AGVC 系統
agvc_status                  # 查看 AGVC 系統狀態
agvc_logs                    # 查看 AGVC 系統日誌
agvc_health                  # AGVC 系統健康檢查
agvc_services                # 檢查所有 AGVC 服務狀態

# 快速執行容器內指令
quick_agvc "command"         # 快速執行 AGVC 容器內指令
quick_agvc "build_ws web_api_ws"      # 建置 Web API 工作空間
quick_agvc "check_agvc_status"        # 檢查 AGVC 狀態
```

## 🔧 容器內開發環境

### 環境載入
```bash
# 智能載入 (自動檢測環境)
all_source              # 或別名: sa

# 強制載入特定環境
agv_source             # 載入 AGV 工作空間
agvc_source            # 載入 AGVC 工作空間

# 檢查載入狀態
echo $ROS_WORKSPACE    # 顯示當前載入的工作空間
printenv | rg ROS   # 檢查 ROS 2 環境變數
```

### 基本開發工具
```bash
# ROS 2 開發工具
ros2                   # ROS 2 CLI 工具
colcon                 # 建置系統
rosdep                 # 依賴管理

# Python 開發工具
python3                # Python 3.12
pip3                   # 套件管理
/opt/pyvenv_env/bin/pip3  # 虛擬環境套件管理

# 系統工具
git                    # 版本控制
vim/nano               # 文字編輯器
htop                   # 系統監控
```

### 虛擬環境配置
```bash
# 虛擬環境位置
/opt/pyvenv_env/

# 自動載入機制
echo $PYTHONPATH
# 應該包含: /opt/pyvenv_env/lib/python3.12/site-packages

# 安裝套件到虛擬環境
/opt/pyvenv_env/bin/pip3 install package_name

# 檢查虛擬環境套件
/opt/pyvenv_env/bin/pip3 list
```

## 🚀 開發工作流程

### 標準開發流程
```bash
# 1. 進入容器
agvc_enter             # 或 agv_enter

# 2. 載入工作空間
all_source

# 3. 檢查環境
check_system_status    # 整體系統狀態
check_ros_env          # ROS 2 環境驗證

# 4. 開發工作
cd /app/workspace/src/your_package
# 進行程式碼修改

# 5. 建置測試
colcon build --packages-select your_package
ros2 run your_package your_node

# 6. 測試驗證
ros2 topic list
ros2 node list
```

### 跨容器開發
```bash
# 在 AGV 容器中開發車載功能
agv_enter
all_source
cd /app/workspace/src/agv_base
# 開發 AGV 狀態機邏輯

# 在 AGVC 容器中開發管理功能
agvc_enter
all_source
cd /app/workspace/src/web_api
# 開發 Web API 功能

# 測試跨容器通訊
# 在一個容器中發布主題
ros2 topic pub /test_topic std_msgs/String "data: 'Hello'"

# 在另一個容器中訂閱
ros2 topic echo /test_topic
```

## 🔧 容器內工具使用

### 統一工具 (r 命令)
```bash
# 在容器內也可以使用統一工具
r dev-build            # 快速建置
r dev-test             # 快速測試
r dev-check            # 代碼檢查

# 注意：某些工具需要在容器內執行
r dev-status           # 在宿主機執行
r dev-build            # 在容器內執行
```

### 專業開發工具
```bash
# 載入開發工具集
source scripts/dev-tools/dev-tools.sh

# 使用專業工具
dev_build              # 智能建置
dev_test               # 執行測試
dev_check_style        # 代碼風格檢查
dev_check_lint         # 靜態分析
```

### 容器內診斷工具
```bash
# 系統狀態檢查
check_system_status    # 整體系統狀態
check_zenoh_status     # Zenoh 通訊狀態
check_ros_env          # ROS 2 環境驗證
check_agvc_status      # AGVC 系統狀態 (僅 AGVC 容器)

# 網路診斷
ping 192.168.100.254   # 測試資料庫連接 (AGVC 容器)
telnet localhost 7447  # 測試 Zenoh Router
```

## 📂 容器內檔案系統

### 重要目錄結構
```
/app/                          # 應用程式根目錄
├── workspace/                 # ROS 2 工作空間
│   ├── src/                  # 原始碼目錄
│   ├── build/                # 建置目錄
│   ├── install/              # 安裝目錄
│   └── log/                  # 日誌目錄
├── config/                   # 配置檔案
├── startup.*.bash            # 啟動腳本
├── setup.bash               # 環境設定腳本
└── routerconfig.json5       # Zenoh 配置
```

### 檔案權限和掛載
```bash
# 檢查檔案權限
ls -la /app/workspace/src/

# 檢查掛載點
mount | rg /app

# 同步檔案變更 (自動同步)
# 宿主機的變更會自動反映到容器內
```

## 🔍 除錯和診斷

### 容器內除錯
```bash
# ROS 2 除錯
ros2 node list
ros2 topic list
ros2 service list

# 查看節點資訊
ros2 node info /node_name

# 查看主題資料
ros2 topic echo /topic_name

# 查看服務介面
ros2 service type /service_name
```

### 日誌查看
```bash
# ROS 2 日誌
ros2 log list
ros2 log view

# 系統日誌
tail -f /tmp/agv.log
tail -f /tmp/zenoh_router.log

# 建置日誌
cat /app/workspace/log/latest_build/events.log
```

### 效能監控
```bash
# 系統資源
top
htop
free -h
df -h

# ROS 2 效能
ros2 topic hz /topic_name
ros2 topic bw /topic_name

# 網路效能  
iftop
netstat -i                  # 網路介面統計 (ss 無法替代此功能)
```

## 🔧 容器指令執行技巧

### Interactive Bash Mode (`bash -i`) 
**⚠️ 重要發現**: 使用 `bash -i` 可以解決容器內指令執行的問題

#### 問題背景
在使用 Bash tool 執行容器內指令時，經常遇到以下問題：
- 指令超時 (timeout after 2 minutes)
- Alias 無法載入 (如 `ba`, `sa`, `manage_web_api_launch` 等)
- 非互動式環境導致的指令失敗

#### 解決方案: bash -i 參數
```bash
# ✅ 推薦方式：使用 bash -i (interactive flag)
docker compose -f docker-compose.agvc.yml exec agvc_server bash -i -c "source /app/setup.bash && agvc_source && manage_web_api_launch stop"

docker compose -f docker-compose.agvc.yml exec agvc_server bash -i -c "source /app/setup.bash && agvc_source && ba && sa && manage_web_api_launch start"

docker compose -f docker-compose.agvc.yml exec agvc_server bash -i -c "source /app/setup.bash && agvc_source && manage_web_api_launch stop && ba && sa && manage_web_api_launch start"

# ❌ 問題方式：不使用 -i 參數
docker compose -f docker-compose.agvc.yml exec agvc_server bash -c "manage_web_api_launch stop"  # 會超時或失敗
```

#### 技術原理
- **Interactive Mode**: `-i` 參數啟用互動式 bash，確保 alias 和函數正確載入
- **Alias Loading**: 互動式模式會正確執行 `.bashrc` 和相關配置檔案
- **Timeout Prevention**: 避免非互動式環境造成的指令掛起
- **完整環境**: 確保容器內的完整 shell 環境被正確初始化

#### 最佳實踐模式
```bash
# 標準容器指令執行模式
docker compose -f docker-compose.agvc.yml exec agvc_server bash -i -c "
source /app/setup.bash && 
agvc_source && 
[your_commands_here]
"

# 實際應用範例
# 1. 停止服務
bash -i -c "source /app/setup.bash && agvc_source && manage_web_api_launch stop"

# 2. 建置和重啟
bash -i -c "source /app/setup.bash && agvc_source && ba && sa && manage_web_api_launch start"

# 3. 完整重啟流程
bash -i -c "source /app/setup.bash && agvc_source && manage_web_api_launch stop && ba && sa && manage_web_api_launch start"
```

#### 應用場景
- **Web 服務管理**: 使用 `manage_web_api_launch` 工具
- **建置工作流**: 執行 `ba` (build all) 和 `sa` (source all) 指令
- **系統重啟**: 完整的停止-建置-啟動流程
- **複雜指令序列**: 需要多個步驟協同執行的操作

## 🛠️ 開發最佳實踐

### 程式碼開發
1. **環境確認**: 確保在正確的容器環境中開發
2. **工作空間載入**: 使用 `all_source` 載入對應工作空間
3. **增量建置**: 使用 `--packages-select` 建置特定套件
4. **即時測試**: 開發過程中持續測試功能
5. **容器指令**: 使用 `bash -i` 執行容器內的複雜指令序列

### 依賴管理
```bash
# 檢查依賴
rosdep check --from-paths src --ignore-src -r

# 安裝依賴
rosdep install --from-paths src --ignore-src -r -y

# 更新 package.xml
# 確保依賴聲明正確
```

### 版本控制
```bash
# 在容器內使用 Git
git status
git add .
git commit -m "feat: 新增功能"

# 注意：Git 配置在容器內可能需要重新設定
git config --global user.name "Your Name"
git config --global user.email "your.email@example.com"
```

## 🚨 常見問題和解決方案

### 環境問題
```bash
# 問題：ROS 2 環境未載入
# 解決：重新載入工作空間
all_source

# 問題：Python 套件找不到
# 解決：檢查虛擬環境
echo $PYTHONPATH
/opt/pyvenv_env/bin/pip3 list

# 問題：建置失敗
# 解決：清理建置目錄
rm -rf /app/workspace/build /app/workspace/install
colcon build
```

### 通訊問題
```bash
# 問題：跨容器通訊失敗
# 解決：檢查 Zenoh Router
check_zenoh_status
ps aux | rg zenoh

# 問題：主題無法發現
# 解決：重啟 ROS 2 daemon
ros2 daemon stop
ros2 daemon start
```

### 效能問題
```bash
# 問題：建置速度慢
# 解決：使用並行建置
colcon build --parallel-workers 4

# 問題：記憶體不足
# 解決：檢查系統資源
free -h
docker stats
```

## 📋 開發檢查清單

### 開發前檢查
- [ ] 確認在正確的容器環境中
- [ ] 工作空間已正確載入
- [ ] ROS 2 環境變數已設定
- [ ] 依賴套件已安裝

### 開發中檢查
- [ ] 程式碼符合風格規範
- [ ] 建置無錯誤和警告
- [ ] 單元測試通過
- [ ] 功能測試正常

### 開發後檢查
- [ ] 跨容器通訊正常
- [ ] 效能符合要求
- [ ] 文檔已更新
- [ ] 版本控制提交完整

## 🔗 交叉引用
- 雙環境架構: @docs-ai/context/system/dual-environment.md
- ROS 2 開發: @docs-ai/operations/development/ros2-development.md
- 容器管理: @docs-ai/operations/deployment/container-management.md
- 系統診斷: @docs-ai/operations/maintenance/system-diagnostics.md
- 技術棧: @docs-ai/context/system/technology-stack.md
