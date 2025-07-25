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

## 開發指令

### 環境設定 (AGVC容器內)
```bash
source /app/setup.bash
agvc_source  # 載入AGVC工作空間
cd /app/db_proxy_ws
```

### 服務管理
```bash
# 啟動資料庫服務
start_db

# 停止資料庫服務  
stop_db

# 啟動db_proxy服務
ros2 run db_proxy db_proxy_node

# 資料庫狀態檢查
check_agvc_status  # 包含資料庫狀態
```

### 構建與測試
```bash
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

## 故障排除

### 常見問題
1. **連線失敗**: 檢查PostgreSQL容器狀態
2. **ORM錯誤**: 確認模型定義與資料表結構
3. **效能問題**: 調整連線池參數
4. **權限錯誤**: 確認資料庫使用者權限

### 診斷指令
```bash
# 檢查資料庫連線（系統管理員）
docker compose -f docker-compose.agvc.yml exec postgres psql -U postgres -d postgres

# 連線到agvc資料庫
docker compose -f docker-compose.agvc.yml exec postgres psql -U agvc -d agvc

# 查看容器狀態和日誌
docker compose -f docker-compose.agvc.yml ps postgres
docker compose -f docker-compose.agvc.yml logs postgres

# 資料庫效能監控
check_agvc_status  # 包含資料庫統計
```

### 日誌位置
- db_proxy日誌: ROS 2節點輸出
- PostgreSQL日誌: 容器內`/var/log/postgresql/`
- 連線池統計: 透過監控API查看

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

## 重要提醒
- 所有資料庫變更需透過此代理進行
- SQLModel模型變更需謹慎處理遷移
- 連線池參數根據負載調整
- 必須在AGVC容器內運行