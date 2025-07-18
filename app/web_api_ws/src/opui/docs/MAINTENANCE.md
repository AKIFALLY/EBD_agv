# OPUI 維護指南

## 🛠️ 日常維護

### 系統監控

#### 檢查服務狀態
```bash
# 檢查 OPUI 服務是否運行
ps aux | grep op_ui_server

# 檢查埠號是否被佔用
netstat -tulpn | grep :8002

# 檢查 ROS2 節點
ros2 node list | grep opui
```

#### 查看日誌
```bash
# 查看服務日誌
journalctl -u opui -f

# 查看 ROS2 日誌
ros2 log info
```

### 常見問題排除

#### 1. 服務無法啟動

**症狀**: `ros2 run opui op_ui_server` 失敗

**排除步驟**:
1. 檢查是否已執行 `all_source`
2. 確認 colcon build 是否成功
3. 檢查 Python 依賴是否安裝完整
4. 查看錯誤訊息中的具體原因

```bash
# 重新建置
cd /app/web_api_ws
colcon build --packages-select opui --symlink-install
source install/setup.bash

# 檢查依賴
pip list | grep -E "(fastapi|uvicorn|socketio)"
```

#### 2. 前端無法連線

**症狀**: 瀏覽器顯示連線錯誤

**排除步驟**:
1. 確認服務是否正常運行
2. 檢查防火牆設定
3. 確認 CORS 設定
4. 查看瀏覽器開發者工具的 Network 標籤

#### 3. 資料庫連線失敗

**症狀**: 後端日誌顯示資料庫錯誤

**排除步驟**:
1. 檢查 PostgreSQL 服務狀態
2. 確認資料庫連線字串正確
3. 測試資料庫連線

```bash
# 檢查 PostgreSQL 狀態
systemctl status postgresql

# 測試連線
psql -h 192.168.100.254 -U agvc -d agvc
```

#### 4. Socket.IO 連線問題

**症狀**: 前端顯示 Socket 連線中斷

**排除步驟**:
1. 檢查網路連線
2. 確認 Socket.IO 版本相容性
3. 查看瀏覽器 Console 錯誤訊息
4. 重新整理頁面

## 🔧 系統配置

### 環境變數設定

在 `/app/web_api_ws/src/opui/config/settings.py` 中可以調整以下設定：

```python
# 伺服器設定
HOST = "0.0.0.0"          # 監聽地址
PORT = 8002               # 監聽埠號

# 資料庫設定
DATABASE_URL = "postgresql+psycopg2://agvc:password@192.168.100.254/agvc"

# CORS 設定
CORS_ALLOWED_ORIGINS = "*"  # 允許的來源

# 任務監控設定
TASK_MONITOR_INTERVAL = 1.0  # 監控間隔（秒）
```

### 效能調整

#### 資料庫連線池
```python
# 在 database/operations.py 中調整
connection_pool = ConnectionPoolManager(
    db_url_agvc,
    pool_size=10,        # 連線池大小
    max_overflow=20      # 最大溢出連線數
)
```

#### Socket.IO 設定
```python
# 在 core/server.py 中調整
self.sio = socketio.AsyncServer(
    async_mode="asgi",
    cors_allowed_origins="*",
    ping_timeout=60,     # ping 超時時間
    ping_interval=25     # ping 間隔時間
)
```

## 📊 效能監控

### 系統資源監控

```bash
# CPU 和記憶體使用率
top -p $(pgrep -f op_ui_server)

# 磁碟使用率
df -h

# 網路連線狀態
ss -tulpn | grep :8002
```

### 應用程式監控

#### 前端效能
- 使用瀏覽器開發者工具的 Performance 標籤
- 監控 Network 標籤中的請求時間
- 檢查 Console 中的錯誤訊息

#### 後端效能
- 監控資料庫查詢時間
- 檢查 Socket.IO 連線數量
- 觀察記憶體使用趨勢

## 🔄 更新與部署

### 程式碼更新

```bash
# 1. 停止服務
pkill -f op_ui_server

# 2. 更新程式碼
git pull origin main

# 3. 重新建置
cd /app/web_api_ws
colcon build --packages-select opui --symlink-install

# 4. 重新啟動服務
source install/setup.bash
ros2 run opui op_ui_server
```

### 資料庫遷移

如果有資料庫結構變更：

```bash
# 1. 備份資料庫
pg_dump -h 192.168.100.254 -U agvc agvc > backup_$(date +%Y%m%d).sql

# 2. 執行遷移腳本（如果有）
psql -h 192.168.100.254 -U agvc -d agvc -f migration.sql

# 3. 驗證資料完整性
```

## 🧪 測試

### 功能測試

```bash
# 運行所有測試
cd /app/web_api_ws/src/opui
python -m pytest tests/ -v

# 運行特定測試
python -m pytest tests/test_db.py -v

# 運行整合測試
python -m pytest tests/test_integration.py -v
```

### 手動測試檢查清單

- [ ] 網頁能正常載入
- [ ] Socket.IO 連線正常
- [ ] 叫空車功能正常
- [ ] 派滿車功能正常
- [ ] 設定頁面功能正常
- [ ] 資料同步正常
- [ ] 錯誤處理正常

## 📋 維護檢查清單

### 每日檢查
- [ ] 服務運行狀態
- [ ] 錯誤日誌檢查
- [ ] 系統資源使用率
- [ ] 使用者回報問題

### 每週檢查
- [ ] 資料庫效能分析
- [ ] 系統備份驗證
- [ ] 安全性更新檢查
- [ ] 效能趨勢分析

### 每月檢查
- [ ] 系統全面測試
- [ ] 文件更新
- [ ] 依賴套件更新
- [ ] 容量規劃評估

## 🆘 緊急處理

### 服務完全無法使用

1. **立即行動**:
   - 檢查系統資源（CPU、記憶體、磁碟）
   - 重新啟動服務
   - 檢查資料庫連線

2. **如果重啟無效**:
   - 回滾到上一個穩定版本
   - 檢查最近的程式碼變更
   - 聯繫開發團隊

3. **恢復步驟**:
   ```bash
   # 緊急回滾
   git checkout last-stable-tag
   colcon build --packages-select opui --symlink-install
   source install/setup.bash
   ros2 run opui op_ui_server
   ```

### 資料庫問題

1. **資料庫無法連線**:
   - 檢查 PostgreSQL 服務狀態
   - 確認網路連線
   - 檢查認證資訊

2. **資料損壞**:
   - 停止所有寫入操作
   - 從最近的備份恢復
   - 驗證資料完整性

## 📞 聯繫資訊

- **開發團隊**: [聯繫方式]
- **系統管理員**: [聯繫方式]
- **緊急聯絡**: [24小時聯繫方式]

記住：遇到問題時，先查看日誌，再嘗試重啟，最後才考慮回滾。保持冷靜，按步驟排除問題。
