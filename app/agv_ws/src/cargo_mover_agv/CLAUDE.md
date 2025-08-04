# cargo_mover_agv - 貨物搬運車AGV控制系統

## 📚 Context Loading
../CLAUDE.md  # 引用上層 agv_ws 工作空間文档

## 📋 專案概述
cargo_mover_agv 實現 Cargo Mover AGV 的完整控制邏輯，支援 Hokuyo 8bit 光通訊模組管理、架台搬運操作、入口/出口流程控制等。負責房間門口的 Rack ↔ 傳送箱轉移作業，並執行 SensorPart 3D 視覺定位 + OCR 產品識別的智能品質檢查。

詳細 Cargo Mover 開發指導請參考: @docs-ai/knowledge/agv-domain/vehicle-types.md

## 🏭 核心業務流程 (基於眼鏡生產系統)

### 主要職責
- **工作區域**: 眼鏡房間門口
- **核心任務**: Rack ↔ 傳送箱 轉移作業 + 品質檢查 + 製程適配性驗證
- **關鍵設備**: SensorPart 相機 (3D視覺定位 + OCR識別)
- **通訊整合**: Hokuyo 8bit 光通訊模組 (左右側)

### 完整操作流程
#### 階段1：3D視覺定位 (Rack整體掃描)
```
SensorPart 相機對整個 Rack 進行 3D 掃描
├── 識別所有 Carrier 的位置座標
├── 更新定位基準點到機械臂座標系
├── 建立 Carrier 抓取路徑規劃
└── 準備逐一 Carrier 處理循環
```

#### 階段2：逐一 Carrier 處理循環
```
For 每個 Carrier:
1. 【移動定位】機械臂移動到 Carrier 前方
2. 【OCR 識別】SensorPart 相機執行 OCR 掃描
   ├── 讀取產品編號/條碼
   ├── 識別產品類型
   └── 取得產品製程需求資訊
3. 【製程匹配檢查】
   ├── 查詢產品的 process_settings_id
   ├── 比對當前房間的製程能力
   └── 判斷製程適配性 (泡藥1次 vs 2次)
4. 【條件分支處理】
   ✅ 製程匹配：
   ├── 機械臂抓取 Carrier
   ├── 8bit 通訊開啟傳送箱門
   ├── 將 Carrier 放入入口傳送箱
   └── 更新系統狀態
   ❌ 製程不匹配：
   ├── 標記 Carrier 為 NG 狀態
   ├── 機械臂回到 Home 位置
   ├── 記錄異常日誌
   └── 繼續處理下一個 Carrier
```

### 智能品質管制特色
- **製程適配性檢查**: Room1/Room2 只接受泡藥1次產品，泡藥2次產品標記NG
- **產品識別驗證**: OCR 確保產品正確性，防止混料誤送
- **異常處理機制**: NG 產品保留在 Rack 上，不影響其他產品處理
- **品質追溯**: 完整記錄每個 Carrier 的處理狀態和異常原因

### A/B 雙面處理流程
```
完整 Rack 處理循環
1. KUKA AGV → 將滿載 Rack 送到房間門口 (A面朝向)
2. Cargo AGV → 處理 A 面所有 Carrier
3. A 面處理完成後，WCS → 產生 Rack 轉向任務
4. KUKA AGV → 執行轉向任務，將 Rack 轉180度 (B面朝向)
5. Cargo AGV → 處理 B 面所有 Carrier (重複流程)
6. 雙面處理完成，判定最終狀態 (正常/NG)
```

### 技術實作特點
- **Hokuyo 8bit 光通訊模組**: 左右側配置，提供 PLC 資料通訊功能
- **麥克納姆輪**: 全向移動能力，精確定位於房間門口
- **機械臂整合**: 單一 Carrier 精密抓取操作
- **3層狀態機**: 複雜的狀態管理和控制邏輯 (Base → AGV → Robot)

## 📂 關鍵檔案位置

### 核心控制
```
cargo_mover_agv/
├─ agv_core_node.py          # Cargo AGV 核心控制節點
├─ cargo_context.py          # Cargo 狀態管理上下文 (AGV層)
├─ robot_context.py          # 機器人狀態控制 (Robot層)
└─ launch/launch.py          # ROS 2 啟動配置
```

### 完整測試套件
```
test/
├─ FINAL_TEST_REPORT.md                          # 完整測試報告
├─ async_update_task_analysis_report.md          # 非同步任務分析報告
├─ test_idle_state_hokuyo.py                     # Idle狀態Hokuyo測試
├─ test_complete_state_delayed_reset.py          # 延遲重置測試
├─ test_hokuyo_busy_states.py                    # Hokuyo忙碌狀態測試
└─ test_wait_rotation_async_update_task.py       # 等待旋轉非同步測試
```

## 🚀 Cargo AGV 專用測試

### 車型特定測試
```bash
# 【推薦方式】透過上層工作空間工具
# 參考: ../CLAUDE.md 開發環境設定

# 【直接測試】Cargo AGV 功能
cd /app/agv_ws/src/cargo_mover_agv
python3 -m pytest test/ -v
```

## 🧪 Cargo AGV 專項功能測試

### Hokuyo 8bit 光通訊測試
```bash
# Hokuyo 設備專項測試
python3 -m pytest test/test_idle_state_hokuyo.py -v              # 初始化測試
python3 -m pytest test/test_hokuyo_busy_states.py -v             # 忙碌狀態測試

# 非同步任務測試
python3 -m pytest test/test_wait_rotation_async_update_task.py -v # 等待旋轉非同步測試
python3 -m pytest test/test_complete_state_delayed_reset.py -v   # 延遲重置測試

# 查看詳細測試報告
cat test/FINAL_TEST_REPORT.md
cat test/async_update_task_analysis_report.md
```

## 📊 配置設定

### AGV 配置檔案
- `/app/config/agv/cargo01_config.yaml` - cargo01 配置
- `/app/config/agv/cargo02_config.yaml` - cargo02 配置

### 關鍵配置參數
```yaml
agv_id: "cargo01"
agv_type: "cargo"

hokuyo_devices:
  hokuyo_1: {ip: "192.168.1.101", port: 8000}
  hokuyo_2: {ip: "192.168.1.102", port: 8000}

rotation_params:
  wait_timeout_ms: 30000      # 等待旋轉逾時
  async_task_interval_ms: 500 # 非同步任務間隔
```

## 🔗 系統整合

### ROS 2 通訊
```bash
# 發布話題
/<agv_id>/status              # Cargo AGV 狀態
/<agv_id>/hokuyo_status       # Hokuyo 設備狀態
/<agv_id>/vision_result       # 視覺定位結果

# 訂閱話題
/<agv_id>/cmd                 # 任務指令
/system/rack_status           # 架台狀態
```

### 外部整合
- **agv_base**: 繼承 3層狀態機架構，使用 Hokuyo DMS 8-bit 光通訊模組
- **plc_proxy_ws**: 架台和傳送箱 PLC 控制
- **sensorpart_ws**: 視覺定位和 Hokuyo 8bit 光通訊模組整合

## 🚨 Cargo AGV 專項故障排除

**⚠️ 通用故障排除請參考**: ../CLAUDE.md 故障排除章節

### Cargo AGV 特有問題
- **Hokuyo 8bit 光通訊問題**: 檢查左右側光通訊模組狀態
- **SensorPart 3D 視覺問題**: 驗證 3D 掃描和 OCR 識別功能
- **A/B 雙面處理異常**: 檢查 Rack 轉向邏輯和狀態同步
- **製程適配性檢查錯誤**: 驗證產品 OCR 識別和製程匹配邏輯