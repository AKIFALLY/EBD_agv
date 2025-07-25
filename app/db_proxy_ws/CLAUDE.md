# db_proxy_ws CLAUDE.md

## 模組概述
PostgreSQL資料庫代理服務，提供連線池管理、ORM整合與ROS 2服務介面

## 專案結構  
```
src/
└── db_proxy/           # PostgreSQL代理服務
    ├── db_proxy/       # 核心代理邏輯
    ├── models/         # SQLModel資料模型
    ├── services/       # 資料庫服務介面
    └── config/         # 連線配置管理
```

## 核心功能

### 資料庫代理
- **連線池管理**: 最佳化資料庫連線使用
- **ORM整合**: SQLModel現代Python ORM
- **ROS 2介面**: 提供ROS服務訪問資料庫
- **異步支援**: AsyncIO非同步資料庫操作

### 技術架構
- **SQLModel**: FastAPI原生ORM (Pydantic + SQLAlchemy)
- **PostgreSQL**: 主要資料庫引擎  
- **連線池**: 管理資料庫連線生命週期
- **ROS 2服務**: 提供標準化資料庫訪問

## 🔧 開發工具指南

### 宿主機操作 (Docker 和資料庫管理)

#### 資料庫容器管理工具
```bash
# PostgreSQL 容器基本操作
docker compose -f docker-compose.agvc.yml up -d postgres     # 啟動資料庫
docker compose -f docker-compose.agvc.yml stop postgres      # 停止資料庫  
docker compose -f docker-compose.agvc.yml restart postgres   # 重啟資料庫
docker compose -f docker-compose.agvc.yml ps postgres        # 查看資料庫狀態
docker compose -f docker-compose.agvc.yml logs postgres      # 查看資料庫日誌
```

#### 資料庫診斷工具 (宿主機執行)
```bash
# 載入系統診斷工具
source scripts/system-tools/system-tools.sh

# 資料庫狀態檢查
scripts/system-tools/service-monitor.sh postgres    # PostgreSQL 服務監控
scripts/network-tools/port-check.sh --port 5432 --host localhost  # 資料庫端口檢查

# AGVC 系統狀態 (包含資料庫)
source scripts/docker-tools/docker-tools.sh
agvc_health                          # AGVC 系統健康檢查 (包含資料庫)
agvc_services                        # 所有服務狀態檢查

# 資料庫連接測試
timeout 3 bash -c "echo > /dev/tcp/localhost/5432" 2>/dev/null && echo "✅ 資料庫可連接" || echo "❌ 資料庫無法連接"
```

#### 資料庫日誌分析 (宿主機執行)
```bash
# PostgreSQL 專項日誌分析
scripts/log-tools/log-analyzer.sh postgres --stats     # PostgreSQL 日誌統計
scripts/log-tools/log-analyzer.sh agvc | grep -i database  # AGVC 中的資料庫相關日誌

# 資料庫性能監控
docker stats postgres_container     # 資料庫容器資源使用
```

#### 開發工作流工具 (宿主機執行)
```bash
# 載入開發工具集
source scripts/dev-tools/dev-tools.sh

# db_proxy 工作空間開發
dev_build --workspace db_proxy_ws    # 建置 db_proxy 工作空間
dev_test --workspace db_proxy_ws     # 測試 db_proxy 工作空間
dev_check --workspace db_proxy_ws --severity warning  # 代碼品質檢查

# 完整開發流程
scripts/dev-tools/build-helper.sh fast --workspace db_proxy_ws    # 快速建置
scripts/dev-tools/test-runner.sh unit --workspace db_proxy_ws     # 單元測試
scripts/dev-tools/code-analyzer.sh security --workspace db_proxy_ws  # 安全檢查
```

### 容器內操作 (ROS 2 和資料庫開發)

#### 環境設定 (AGVC容器內)
```bash
source /app/setup.bash
agvc_source  # 載入AGVC工作空間 (或使用 all_source 自動檢測)
cd /app/db_proxy_ws
```

#### 服務管理 (容器內執行)
```bash
# 【方法1: 透過宿主機工具】(推薦)
# 在宿主機執行：
source scripts/docker-tools/docker-tools.sh
agvc_enter  # 自動進入 AGVC 容器並載入環境

# 或使用快速執行
quick_agvc "start_db"                # 檢查資料庫連接狀態
quick_agvc "check_agvc_status"       # 檢查包含資料庫的系統狀態
quick_agvc "ros2 run db_proxy db_proxy_node"  # 啟動 db_proxy 服務

# 【方法2: 手動進入容器】
# 啟動資料庫服務檢查
start_db

# 停止資料庫服務指導  
stop_db

# 啟動db_proxy服務
ros2 run db_proxy db_proxy_node

# 資料庫狀態檢查
check_agvc_status  # 包含資料庫狀態
```

#### 構建與測試

##### 宿主機建置和測試 (推薦)
```bash
# 使用開發工具進行建置
source scripts/dev-tools/dev-tools.sh
dev_build --workspace db_proxy_ws    # 建置 db_proxy 工作空間
dev_test --workspace db_proxy_ws     # 測試 db_proxy 工作空間

# 或直接使用工具腳本
scripts/dev-tools/build-helper.sh fast --workspace db_proxy_ws
scripts/dev-tools/test-runner.sh unit --workspace db_proxy_ws
```

##### 容器內建置
```bash
# 【方法1: 透過宿主機工具】(推薦)
quick_agvc "build_ws db_proxy_ws"    # 在 AGVC 容器內建置
quick_agvc "ros2 test db_proxy"      # 資料庫測試

# 【方法2: 手動進入容器】
agvc_enter  # 進入容器
build_ws db_proxy_ws
ros2 test db_proxy  # 資料庫測試
```

## 資料模型開發

### SQLModel模型定義
```python
# models/agv_model.py
class AGVModel(SQLModel, table=True):
    __tablename__ = "agvs"
    
    id: Optional[int] = Field(primary_key=True)
    agv_id: str = Field(unique=True, index=True)
    status: str
    position_x: float
    position_y: float
    created_at: datetime = Field(default_factory=datetime.utcnow)
```

### 資料庫遷移
```bash
# 創建遷移檔案 (如果有alembic支援)
alembic revision --autogenerate -m "新增AGV表格"

# 執行遷移
alembic upgrade head
```

## 服務介面開發

### ROS 2服務定義
```python
# services/agv_service.py
class AGVDatabaseService:
    async def get_agv_status(self, request):
        async with get_db_session() as session:
            result = await session.execute(
                select(AGVModel).where(AGVModel.agv_id == request.agv_id)
            )
            return result.scalar_one_or_none()
```

### 新增資料庫服務
1. **定義模型**: `models/`目錄下創建SQLModel類別
2. **實現服務**: `services/`目錄下實現資料庫操作
3. **註冊ROS服務**: 在主節點中註冊服務端點
4. **測試驗證**: 編寫單元測試驗證功能

## 資料庫配置

### 連線設定
```yaml
# /app/config/agvc/database.yaml
database:
  host: postgres
  port: 5432
  name: agvc
  user: agvc
  password: password
  pool_size: 20
  max_overflow: 30
```

### 環境變數
- `DB_HOST`: 資料庫主機 (預設: postgres)
- `DB_PORT`: 資料庫端口 (預設: 5432)  
- `DB_NAME`: 資料庫名稱 (預設: agvc)
- `DB_USER`: 資料庫使用者 (預設: agvc)
- `DB_PASSWORD`: 資料庫密碼 (預設: password)

## 性能最佳化

### 連線池管理
- **pool_size**: 基礎連線池大小
- **max_overflow**: 最大溢出連線數
- **pool_timeout**: 連線超時時間
- **pool_recycle**: 連線回收時間

### 查詢最佳化
- 適當使用索引(index=True)
- 避免N+1查詢問題
- 使用異步查詢處理大量資料
- 實現資料分頁機制

## 測試架構

### 單元測試
```python
# test/test_agv_service.py
@pytest.mark.asyncio
async def test_get_agv_status():
    # 資料庫服務測試
    service = AGVDatabaseService()
    result = await service.get_agv_status(mock_request)
    assert result.agv_id == "AGV001"
```

### 整合測試
- 測試資料庫連線
- 驗證ORM模型映射
- ROS 2服務呼叫測試

## 🛠️ 故障排除

### 系統診斷工作流程

#### 第一步：快速系統檢查 (宿主機執行)
```bash
# 完整系統健康檢查
scripts/system-tools/health-check.sh --quick

# 資料庫系統狀態檢查
source scripts/docker-tools/docker-tools.sh
agvc_health                          # AGVC 系統健康檢查 (包含資料庫)
agvc_services                        # 所有服務狀態檢查
```

#### 第二步：資料庫專項診斷 (宿主機執行)
```bash
# 資料庫容器狀態
docker compose -f docker-compose.agvc.yml ps postgres
docker compose -f docker-compose.agvc.yml logs postgres

# 資料庫連接測試
scripts/network-tools/port-check.sh --port 5432 --host localhost
timeout 3 bash -c "echo > /dev/tcp/localhost/5432" 2>/dev/null && echo "✅ 可連接" || echo "❌ 無法連接"

# 資料庫日誌分析
scripts/log-tools/log-analyzer.sh postgres --stats       # PostgreSQL 日誌統計
scripts/log-tools/log-analyzer.sh agvc | grep -i database # AGVC 資料庫相關日誌

# 資料庫性能監控
docker stats postgres_container     # 資源使用監控
```

### 常見問題及解決方案

#### 1. **PostgreSQL 容器無法啟動**
```bash
# 宿主機診斷步驟
docker compose -f docker-compose.agvc.yml ps postgres    # 容器狀態
docker compose -f docker-compose.agvc.yml logs postgres  # 啟動日誌
docker volume ls | grep postgres    # 檢查資料卷
```

#### 2. **資料庫連線失敗**
```bash
# 宿主機連接測試
scripts/network-tools/port-check.sh --port 5432 --host localhost
quick_agvc "start_db"                # 檢查資料庫連接狀態

# 容器內連接測試
agvc_enter
start_db  # 提供詳細連接指導
```

#### 3. **ORM錯誤和模型問題**
```bash
# 代碼品質檢查
scripts/dev-tools/code-analyzer.sh style --workspace db_proxy_ws
scripts/dev-tools/code-analyzer.sh security --workspace db_proxy_ws

# 容器內模型驗證
quick_agvc "python3 -c \"from db_proxy.models import *; print('Models imported successfully')\""
```

#### 4. **資料庫效能問題**
```bash
# 資源監控
docker stats postgres_container     # CPU, 記憶體使用
quick_agvc "check_agvc_status"       # 包含資料庫統計

# 連線池調整 (容器內)
agvc_enter
# 檢查連線池配置並根據負載調整
```

#### 5. **權限錯誤**
```bash
# 檢查資料庫使用者權限
docker compose -f docker-compose.agvc.yml exec postgres psql -U postgres -d postgres -c "\\du"

# 確認 agvc 使用者權限
docker compose -f docker-compose.agvc.yml exec postgres psql -U agvc -d agvc -c "SELECT current_user, current_database();"
```

### 進階診斷指令

#### 資料庫直接連線 (宿主機執行)
```bash
# 檢查資料庫連線（系統管理員）
docker compose -f docker-compose.agvc.yml exec postgres psql -U postgres -d postgres

# 連線到agvc資料庫
docker compose -f docker-compose.agvc.yml exec postgres psql -U agv -d agvc

# 檢查資料庫狀態
docker compose -f docker-compose.agvc.yml exec postgres psql -U postgres -c "SELECT datname, numbackends, xact_commit, xact_rollback FROM pg_stat_database WHERE datname='agvc';"
```

### 日誌位置和分析
```bash
# 宿主機日誌分析 (推薦)
scripts/log-tools/log-analyzer.sh postgres --stats      # PostgreSQL 日誌統計
scripts/log-tools/log-analyzer.sh agvc --severity 3     # AGVC 中的嚴重錯誤

# 容器日誌位置
# - db_proxy日誌: ROS 2節點輸出 (透過 agvc_logs 查看)
# - PostgreSQL日誌: 容器內 /var/log/postgresql/ (透過 docker logs 查看)
# - 連線池統計: 透過監控API查看
```

## 安全注意事項

### 資料保護
- 敏感資料使用環境變數
- 資料庫密碼不得硬編碼
- 實施適當的訪問權限控制
- 定期備份重要資料

### 連線安全
- 使用SSL連線(生產環境)
- 限制資料庫網路訪問
- 實施連線數量限制
- 監控異常連線活動

## 💡 重要提醒

### 開發環境使用原則
- **🖥️ 宿主機**: 使用 `scripts/` 工具進行資料庫容器管理、連接測試、日誌分析
- **🐳 容器內**: 執行 db_proxy 服務、ROS 2 相關指令、ORM 模型開發
- **📡 推薦方式**: 使用 `agvc_enter` 進入容器，使用 `quick_agvc` 執行容器內指令

### 資料庫開發最佳實踐
- **資料庫變更**: 所有變更需透過 db_proxy 進行，避免直接操作
- **SQLModel模型**: 模型變更需謹慎處理遷移和向後兼容
- **連線池管理**: 根據實際負載調整連線池參數
- **服務運行**: db_proxy 服務必須在 AGVC 容器內運行

### 故障排除最佳實踐
1. **優先使用宿主機工具**: 快速檢查容器狀態和連接性
2. **分層診斷**: 容器→網路→連接→ORM→業務邏輯
3. **日誌分析為主**: 使用 `scripts/log-tools/` 進行智能分析
4. **資料安全**: 確保敏感資料使用環境變數，定期備份