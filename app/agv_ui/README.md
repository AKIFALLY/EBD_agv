# Cargo AGV 即時監控系統

## 🎯 系統概述

Cargo AGV 即時監控系統是一個基於 Web 的即時監控界面，用於監控 cargo_mover_agv 系統的運行狀態。系統通過讀取 AGV 狀態 JSON 文件，並透過 WebSocket 提供即時資料更新。

### 主要功能
- **即時狀態監控**: 監控 AGV 系統的即時運行狀態
- **3層狀態機顯示**: 清晰展示 Base/Cargo/Robot 三層狀態機
- **設備狀態追蹤**: 追蹤機械臂、Hokuyo 設備等硬體狀態
- **PLC 資料監控**: 詳細顯示 PLC 通訊資料和變數
- **歷史資料圖表**: 即時圖表顯示電壓、移動狀態等變化
- **系統日誌**: 即時系統日誌和錯誤追蹤
- **響應式設計**: 支援桌面和行動裝置瀏覽

## 🏗️ 系統架構

### 技術棧
- **後端**: FastAPI + WebSocket + Uvicorn
- **前端**: HTML5 + Bootstrap 5 + Chart.js + WebSocket
- **資料來源**: JSON 狀態檔案 (來自 CargoAgvStatusJsonRecorder)
- **即時通訊**: WebSocket 雙向通訊

### 檔案結構
```
/home/ct/RosAGV/app/agv_ui/
├── app.py                  # FastAPI 後端服務
├── requirements.txt        # Python 依賴列表
├── venv/                   # Python 虛擬環境
├── templates/              # Jinja2 模板
│   └── index.html         # 主監控頁面
├── static/                # 靜態資源
│   ├── css/
│   │   └── style.css      # 自訂樣式
│   └── js/
│       └── main.js        # 前端 JavaScript
└── README.md              # 本文檔
```

### 資料流程
```
AGV 系統 → JSON 狀態文件 → FastAPI 後端 → WebSocket → 前端界面
    ↓              ↓              ↓            ↓          ↓
Cargo AGV      狀態記錄器      API 服務     即時通訊    監控界面
```

## 🚀 快速開始

### 環境需求
- Python 3.12+
- 運行中的 Cargo AGV 系統 (產生 JSON 狀態文件)
- 現代瀏覽器 (支援 WebSocket)

### 1. 確認 AGV 狀態文件
確認 Cargo AGV 系統正在運行並產生狀態文件：
```bash
ls -la /home/ct/RosAGV/app/agv_status_json/
# 應該看到：current_status.json 文件
```

### 2. 啟動監控服務
```bash
cd /home/ct/RosAGV/app/agv_ui
source venv/bin/activate
python3 app.py
```

### 3. 開啟監控界面
在瀏覽器中訪問：http://localhost:8080

## 🖥️ 界面功能說明

### 系統概覽
- **AGV ID**: 顯示當前 AGV 識別碼
- **電源電壓**: 即時電壓監控 (24V 正常)
- **運行模式**: 自動/手動模式指示
- **移動狀態**: AGV 是否正在移動
- **警報狀態**: 系統警報監控
- **歷史記錄**: 狀態變更歷史數量

### 狀態機監控
- **Base Context**: 基礎層狀態 (AutoState, ManualState 等)
- **Cargo Context**: 貨物層狀態 (WaitRobotState, CompleteState 等)  
- **Robot Context**: 機械臂層狀態 (ExecutingState, IdleState 等)

### 設備狀態
- **Robot**: 機械臂控制器狀態
- **HokuyoDMS8Bit_1/2**: 左右側光通訊模組狀態

### PLC 狀態詳情
顯示重要的 PLC 變數：
- **AGV_ID**: AGV 識別碼
- **POWER**: 電源電壓
- **AGV_X_SPEED/AGV_Y_SPEED**: X/Y 軸速度
- **AGV_SLAM_X/AGV_SLAM_Y**: SLAM 定位座標
- **AGV_Auto**: 自動模式狀態
- **AGV_MOVING**: 移動狀態
- **AGV_ALARM**: 警報狀態
- **AGV_INPUT/OUTPUT**: I/O 狀態

### 狀態歷史圖表
即時圖表顯示：
- **電源電壓**: 綠色線圖 (左 Y 軸)
- **移動狀態**: 藍色線圖 (右 Y 軸)
- 保持最近 20 個資料點的歷史

### 系統日誌
- **即時日誌**: 系統事件和狀態變更
- **日誌類型**: Info (藍), Warning (黃), Error (紅), Success (綠)
- **自動清理**: 最多保存 100 條日誌記錄

## 🔧 API 端點說明

### REST API
- `GET /` - 主監控頁面
- `GET /api/status` - 取得完整 AGV 狀態資料
- `GET /api/summary` - 取得狀態摘要資料  
- `GET /api/history` - 取得狀態歷史記錄

### WebSocket API
- `ws://localhost:8080/ws` - WebSocket 連接端點

#### WebSocket 訊息格式
**伺服器 → 客戶端:**
```json
{
  "type": "full_data",
  "data": { /* 完整 AGV 狀態資料 */ },
  "summary": { /* 狀態摘要 */ }
}
```

```json
{
  "type": "summary", 
  "data": { /* 狀態摘要資料 */ }
}
```

## ⚙️ 配置說明

### 主要配置參數 (app.py)
```python
AGV_STATUS_DIR = "/home/ct/RosAGV/app/agv_status_json"  # AGV 狀態文件目錄
UPDATE_INTERVAL = 1.0  # 更新間隔 (秒)
MAX_HISTORY = 100      # 最大歷史記錄數量
```

### 環境變數 (可選)
```bash
export AGV_UI_HOST="0.0.0.0"     # 監聽主機 (預設: 0.0.0.0)
export AGV_UI_PORT=8080          # 監聽端口 (預設: 8080)  
export AGV_UI_DEBUG=false        # 除錯模式 (預設: false)
```

## 🔍 故障排除

### 常見問題

**1. 找不到 AGV 狀態文件**
```
錯誤: 找不到 AGV 狀態文件
解決: 確認 Cargo AGV 系統正在運行並產生 JSON 文件
檢查: ls -la /home/ct/RosAGV/app/agv_status_json/current_status.json
```

**2. WebSocket 連接失敗**
```
錯誤: WebSocket 連接中斷
解決: 
- 檢查服務器是否運行: ps aux | grep "python3 app.py"
- 檢查端口是否開放: ss -tulpn | grep 8080
- 重啟服務: Ctrl+C 停止，然後重新啟動
```

**3. 虛擬環境問題**
```
錯誤: ImportError: No module named 'fastapi'
解決: 
cd /home/ct/RosAGV/app/agv_ui
source venv/bin/activate
pip install -r requirements.txt
```

**4. 端口被佔用**
```
錯誤: [Errno 98] Address already in use
解決:
# 查找佔用端口的進程
lsof -i :8080
# 停止進程或更改端口
```

### 日誌檢查
```bash
# 檢查服務器進程
ps aux | grep "python3 app.py"

# 檢查網路連接
ss -tulpn | grep 8080

# 測試 API 端點
curl http://localhost:8080/api/summary

# 檢查狀態文件
ls -la /home/ct/RosAGV/app/agv_status_json/current_status.json
cat /home/ct/RosAGV/app/agv_status_json/current_status.json
```

## 🔄 維護和更新

### 定期維護
- **日誌清理**: 界面提供日誌清除功能
- **狀態文件**: 系統自動處理過期的狀態文件
- **連接監控**: 自動重連機制確保穩定性

### 效能監控
- **記憶體使用**: 系統限制歷史記錄數量避免記憶體溢出
- **CPU 使用**: 1秒更新間隔平衡即時性和效能
- **網路頻寬**: WebSocket 只傳送變更的資料

### 擴展建議
- **多 AGV 支援**: 修改後端支援多個 AGV 同時監控
- **資料庫儲存**: 增加資料庫支援長期歷史記錄
- **警報系統**: 實現警報通知功能
- **使用者權限**: 增加身份驗證和權限管理

## 📊 系統監控指標

### 關鍵效能指標 (KPI)
- **連接穩定性**: WebSocket 連接持續時間
- **資料延遲**: 狀態更新的延遲時間
- **更新頻率**: 每秒狀態更新次數
- **錯誤率**: API 請求錯誤比例

### 健康檢查
```bash
# API 健康檢查
curl -f http://localhost:8080/api/summary > /dev/null && echo "✅ API 正常" || echo "❌ API 異常"

# 狀態文件檢查
test -f /home/ct/RosAGV/app/agv_status_json/current_status.json && echo "✅ 狀態文件正常" || echo "❌ 狀態文件缺失"

# 服務進程檢查  
pgrep -f "python3 app.py" > /dev/null && echo "✅ 服務運行中" || echo "❌ 服務未運行"
```

## 📝 更新日誌

### v1.0.0 (2024-08-08)
- ✅ 初始版本發布
- ✅ 基礎監控功能實現
- ✅ WebSocket 即時通訊
- ✅ 響應式界面設計
- ✅ 狀態機監控
- ✅ PLC 資料顯示
- ✅ 歷史圖表功能
- ✅ 系統日誌功能

## 🤝 技術支援

### 聯絡資訊
- **系統**: RosAGV Cargo Mover AGV 監控系統
- **版本**: 1.0.0
- **技術棧**: FastAPI + WebSocket + Bootstrap 5
- **相關文件**: 
  - AGV 狀態記錄器: `/home/ct/RosAGV/app/agv_ws/src/cargo_mover_agv/cargo_mover_agv/status_json_recorder.py`
  - AGV 核心節點: `/home/ct/RosAGV/app/agv_ws/src/cargo_mover_agv/cargo_mover_agv/agv_core_node.py`

### 開發指導
- **AGV 整合**: 確保 CargoAgvStatusJsonRecorder 正常運行
- **狀態文件**: 監控系統讀取 JSON 文件格式的狀態資料
- **即時更新**: 透過定時器 (1秒間隔) 檢查文件變更
- **WebSocket**: 使用 FastAPI WebSocket 實現雙向通訊

---

**🚀 享受即時 AGV 監控體驗！**