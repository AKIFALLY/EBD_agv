# 開發環境設定

## 🎯 RosAGV 開發環境配置指南

本指南協助開發者建立完整的 RosAGV 開發環境，包括容器化開發、ROS 2 工作空間、除錯工具和開發工作流程。

## 📋 開發環境概覽

### 雙環境開發架構
```
開發環境架構
├── 🖥️ 宿主機環境
│   ├── Docker 和 Docker Compose
│   ├── IDE 和編輯器 (VS Code)
│   ├── Git 版本控制
│   └── 統一工具 (r 命令)
└── 🐳 容器化開發環境
    ├── AGV 開發容器 (rosagv)
    ├── AGVC 開發容器 (agvc_server)
    ├── ROS 2 工作空間
    └── 開發工具鏈
```

## 🛠️ 環境安裝

### 基礎環境要求
```yaml
系統要求:
  作業系統: Ubuntu 24.04 LTS (推薦)
  記憶體: 16GB RAM (最少 8GB)
  儲存: 256GB SSD
  處理器: x86_64 (4核心以上)

軟體要求:
  Docker: 24.0+
  Docker Compose: V2
  Git: 2.40+
  VS Code: 最新版本
```

### 安裝開發工具

#### 1. Docker 環境安裝
```bash
# 安裝 Docker
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh

# 安裝 Docker Compose V2
sudo curl -L "https://github.com/docker/compose/releases/latest/download/docker-compose-linux-$(uname -m)" -o /usr/local/bin/docker-compose
sudo chmod +x /usr/local/bin/docker-compose

# 設定用戶權限
sudo usermod -aG docker $USER
newgrp docker
```

#### 2. VS Code 和擴展安裝
```bash
# 安裝 VS Code
wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
sudo install -o root -g root -m 644 packages.microsoft.gpg /etc/apt/trusted.gpg.d/
sudo sh -c 'echo "deb [arch=amd64,arm64,armhf signed-by=/etc/apt/trusted.gpg.d/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" > /etc/apt/sources.list.d/vscode.list'
sudo apt update
sudo apt install code

# 推薦的 VS Code 擴展
code --install-extension ms-python.python
code --install-extension ms-vscode.cpptools
code --install-extension ms-vscode-remote.remote-containers
code --install-extension ms-vscode.cmake-tools
code --install-extension redhat.vscode-yaml
```

#### 3. 統一工具配置
```bash
# 設定 RosAGV 工具路徑
echo 'export PATH="/home/ct/RosAGV:$PATH"' >> ~/.bashrc
source ~/.bashrc

# 驗證工具可用性
r --help
r agvc-check
```

## 🚀 專案設定

### 克隆和配置專案
```bash
# 克隆專案
git clone https://github.com/your-org/RosAGV.git
cd RosAGV

# 設定 Git 配置
git config user.name "Your Name"
git config user.email "your.email@example.com"

# 設定開發分支
git checkout -b feature/your-feature-name
```

### 環境變數配置
```bash
# 複製環境配置模板
cp .env.example .env.dev

# 編輯開發環境變數
vim .env.dev

# 開發環境變數範例
ENVIRONMENT=development
DEBUG=true
POSTGRES_DB=agvc_dev
POSTGRES_USER=agvc_dev
POSTGRES_PASSWORD=dev_password
WEB_API_PORT=8000
LOG_LEVEL=DEBUG
```

## 🐳 容器化開發

### 開發容器啟動

#### AGV 開發環境
```bash
# 啟動 AGV 開發容器
docker compose -f docker-compose.yml up -d

# 進入 AGV 開發容器
docker compose -f docker-compose.yml exec rosagv bash

# 載入開發環境
source /app/setup.bash
all_source  # 或簡寫: sa
```

#### AGVC 開發環境
```bash
# 啟動 AGVC 開發容器
docker compose -f docker-compose.agvc.yml up -d

# 進入 AGVC 開發容器
docker compose -f docker-compose.agvc.yml exec agvc_server bash

# 載入開發環境
source /app/setup.bash
all_source  # 或簡寫: sa
```

### 工作空間管理

#### AGV 工作空間 (9個)
```bash
# AGV 工作空間列表
agv_ws/                    # 核心 AGV 控制
agv_cmd_service_ws/        # 手動指令服務
joystick_ws/               # 搖桿控制
sensorpart_ws/             # 感測器處理
keyence_plc_ws/            # PLC 通訊
plc_proxy_ws/              # PLC 代理
path_algorithm/            # 路徑規劃

# 建置特定工作空間
cd /app/agv_ws
colcon build --packages-select agv_base
source install/setup.bash
```

#### AGVC 工作空間 (11個)
```bash
# AGVC 工作空間列表
web_api_ws/                # Web API 服務
db_proxy_ws/               # 資料庫代理
ecs_ws/                    # 設備控制系統
rcs_ws/                    # 機器人控制系統
tafl_wcs_ws/               # TAFL 流程執行引擎
kuka_fleet_ws/             # KUKA Fleet 整合

# 建置特定工作空間
cd /app/web_api_ws
colcon build --packages-select web_api
source install/setup.bash
```

## 🔧 開發工具使用

### ROS 2 開發工具

#### 節點開發和測試
```bash
# 檢查 ROS 2 環境
ros2 doctor

# 節點列表
ros2 node list

# 主題監控
ros2 topic list
ros2 topic echo /agv_status
ros2 topic hz /agv_status

# 服務測試
ros2 service list
ros2 service call /plc_read plc_interfaces/PLCRead "address: 'DM100'"
```

#### 建置和測試
```bash
# 建置所有套件
colcon build

# 建置特定套件
colcon build --packages-select agv_base

# 執行測試
colcon test
colcon test --packages-select agv_base

# 查看測試結果
colcon test-result --verbose
```

### Python 開發工具

#### 程式碼品質檢查
```bash
# 進入容器內的虛擬環境
source /opt/pyvenv_env/bin/activate

# 程式碼格式化
black src/
isort src/

# 靜態分析
pylint src/
mypy src/

# 安全掃描
bandit -r src/
```

#### 測試和覆蓋率
```bash
# 執行單元測試
pytest tests/

# 測試覆蓋率
pytest --cov=src tests/
pytest --cov=src --cov-report=html tests/

# 效能測試
pytest tests/performance/ -v
```

### 資料庫開發

#### 資料庫操作
```bash
# 進入 AGVC 容器
docker compose -f docker-compose.agvc.yml exec agvc_server bash

# 連接資料庫
psql -h postgres -U agvc -d agvc

# 執行資料庫遷移
python -m db_proxy.sql.db_install

# 檢查資料庫狀態
python -c "from db_proxy.crud.base import check_db_connection; print(check_db_connection())"
```

## 🔍 除錯和診斷

### 系統診斷工具
```bash
# 統一診斷工具
r agvc-check              # AGVC 系統健康檢查
r agv-check               # AGV 系統健康檢查
r containers-status       # 容器狀態檢查
r network-check           # 網路連接檢查
r quick-diag              # 快速綜合診斷
```

### 容器內診斷
```bash
# 系統狀態檢查
check_system_status       # 整體系統狀態
check_zenoh_status        # Zenoh 通訊狀態
check_ros_env             # ROS 2 環境驗證

# 日誌分析
tail -f /tmp/agv.log
tail -f /tmp/zenoh_router.log
```

### VS Code 遠端開發

#### 設定容器開發
```json
// .devcontainer/devcontainer.json
{
    "name": "RosAGV Development",
    "dockerComposeFile": ["../docker-compose.yml"],
    "service": "rosagv",
    "workspaceFolder": "/app",
    "extensions": [
        "ms-python.python",
        "ms-vscode.cpptools",
        "redhat.vscode-yaml"
    ],
    "settings": {
        "python.defaultInterpreterPath": "/opt/pyvenv_env/bin/python",
        "python.linting.enabled": true,
        "python.linting.pylintEnabled": true
    }
}
```

## 📝 開發工作流程

### 日常開發流程
```bash
# 1. 同步最新程式碼
git pull origin main
git checkout -b feature/new-feature

# 2. 啟動開發環境
docker compose -f docker-compose.agvc.yml up -d
docker compose -f docker-compose.agvc.yml exec agvc_server bash

# 3. 開發和測試
source /app/setup.bash && all_source
# 進行程式碼開發...
colcon build --packages-select your_package
pytest tests/

# 4. 程式碼品質檢查
black src/
pylint src/
mypy src/

# 5. 提交變更
git add .
git commit -m "feat: 新增功能描述"
git push origin feature/new-feature
```

### 團隊協作
```bash
# 程式碼審查流程
git push origin feature/your-feature
# 建立 Pull Request
# 等待程式碼審查
# 修正審查意見
# 合併到主分支

# 同步主分支變更
git checkout main
git pull origin main
git checkout feature/your-feature
git rebase main
```

## 🚨 常見問題解決

### 容器問題
```bash
# 容器無法啟動
docker compose down
docker system prune -f
docker compose up -d --build

# 容器內權限問題
docker compose exec rosagv bash
chown -R $(id -u):$(id -g) /app/workspace
```

### ROS 2 問題
```bash
# ROS 2 環境問題
export RMW_IMPLEMENTATION=rmw_zenohd
source /opt/ros/jazzy/setup.bash
all_source

# 節點發現問題
ros2 daemon stop
ros2 daemon start
```

### 資料庫問題
```bash
# 資料庫連接問題
docker compose restart postgres
sleep 30
docker compose restart agvc_server

# 資料庫重置
docker compose down
docker volume rm rosagv_postgres_data
docker compose up -d
```

## 📚 開發資源

### 文檔參考
- [ROS 2 官方文檔](https://docs.ros.org/en/jazzy/)
- [FastAPI 文檔](https://fastapi.tiangolo.com/)
- [Docker 文檔](https://docs.docker.com/)
- [PostgreSQL 文檔](https://www.postgresql.org/docs/)

### 內部資源
- [系統架構](../system-architecture/dual-environment.md)
- [技術棧詳解](../system-architecture/technology-stack.md)
- [API 文檔](http://localhost:8000/docs) (開發環境)
- [故障排除](troubleshooting.md)

## 📋 開發檢查清單

### 環境設定檢查
- [ ] Docker 和 Docker Compose 已安裝
- [ ] Git 配置完成
- [ ] VS Code 和擴展已安裝
- [ ] 統一工具 (r 命令) 可用
- [ ] 專案已克隆並設定完成

### 開發前檢查
- [ ] 容器正常啟動
- [ ] ROS 2 環境載入成功
- [ ] 資料庫連接正常
- [ ] 所有服務運行正常
- [ ] 測試通過

### 提交前檢查
- [ ] 程式碼格式化完成
- [ ] 靜態分析通過
- [ ] 單元測試通過
- [ ] 文檔已更新
- [ ] 提交訊息清晰

---

**相關文檔：**
- [部署指導](deployment.md) - 生產環境部署
- [維護操作](maintenance.md) - 日常維護指導
- [故障排除](troubleshooting.md) - 問題診斷和解決
- [技術整合](../technical-details/ros2-integration.md) - ROS 2 開發詳解