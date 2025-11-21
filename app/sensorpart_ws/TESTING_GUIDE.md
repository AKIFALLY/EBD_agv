# SensorPart 測試指南

本指南說明如何使用模擬器測試 sensorpart_publisher_node，不需要實際連接相機。

---

## 🎯 測試架構

```
┌─────────────────────────────────┐
│ sensorpart_simulator_node       │  模擬相機 TCP 伺服器
│ (0.0.0.0:2005)                  │  發送測試資料
└────────────┬────────────────────┘
             │ TCP
             ↓
┌─────────────────────────────────┐
│ sensorpart_publisher_node       │  TCP 客戶端
│ (連接到 127.0.0.1:2005)         │  解析並發布 topics
└────────────┬────────────────────┘
             │ ROS2 Topics
             ↓
┌─────────────────────────────────┐
│ ros2 topic echo                 │  查看發布的資料
│ /sensor/ocr                     │
│ /sensor/position_3d             │
└─────────────────────────────────┘
```

---

## 🚀 快速開始

### 步驟 1：建置套件

```bash
# 進入 AGV 容器（如果尚未啟動）
cd ~/RosAGV
docker compose -f docker-compose.yml up -d rosagv
docker compose -f docker-compose.yml exec rosagv bash

# 載入環境
source /app/setup.bash && agv_source

# 建置 sensorpart
cd /app/sensorpart_ws
colcon build --packages-select sensorpart
source install/setup.bash
```

### 步驟 2：啟動模擬器（終端機 1）

```bash
# 啟動模擬相機伺服器
ros2 run sensorpart sensorpart_simulator_node

# 預期輸出：
# [INFO] [sensorpart_simulator]: 🌐 TCP 伺服器啟動: 0.0.0.0:2005
# [INFO] [sensorpart_simulator]: ✅ 自動發送已啟用，間隔: 5.0 秒
# [INFO] [sensorpart_simulator]: ✅ SensorPart 模擬器已啟動
# [INFO] [sensorpart_simulator]: ⏳ 等待客戶端連接...
```

### 步驟 3：啟動 Publisher 節點（終端機 2）

```bash
# 新開一個終端機
docker compose -f docker-compose.yml exec rosagv bash
source /app/setup.bash && agv_source
source /app/sensorpart_ws/install/setup.bash

# 啟動 publisher（連接到本地模擬器）
ros2 run sensorpart sensorpart_publisher_node --ros-args -p host:=127.0.0.1

# 預期輸出：
# [INFO] [sensorpart_publisher]: 📡 SensorPart 設定: 127.0.0.1:2005, 發布頻率: 10.0 Hz
# [INFO] [sensorpart_publisher]: ✅ SensorPart Publisher 節點已啟動
```

**此時終端機 1 應該顯示**：
```
[INFO] [sensorpart_simulator]: ✅ 客戶端已連接: ('127.0.0.1', xxxxx)
```

### 步驟 4：監聽 Topics（終端機 3）

```bash
# 新開第三個終端機
docker compose -f docker-compose.yml exec rosagv bash
source /app/setup.bash && agv_source
source /app/sensorpart_ws/install/setup.bash

# 監聽 OCR topic
ros2 topic echo /sensor/ocr

# 預期輸出（每 5 秒一次）：
# data: ABC12345
# ---
# data: DEF67890
# ---
# data: ABC54321
# ---
```

或監聽 3D 位置：
```bash
ros2 topic echo /sensor/position_3d

# 預期輸出：
# header:
#   stamp:
#     sec: 1737280000
#     nanosec: 123456789
#   frame_id: sensor_frame
# pose:
#   position:
#     x: 1523.0
#     y: 892.0
#     z: 234.0
#   orientation:
#     x: 0.0
#     y: 0.0
#     z: 0.012
#     w: 0.999
# ---
```

---

## 🔧 進階配置

### 自訂模擬器參數

```bash
# 自訂 Port
ros2 run sensorpart sensorpart_simulator_node --ros-args -p port:=3005

# 關閉自動發送（手動控制）
ros2 run sensorpart sensorpart_simulator_node --ros-args -p auto_send:=false

# 自訂發送間隔（秒）
ros2 run sensorpart sensorpart_simulator_node --ros-args -p send_interval:=2.0
```

### 使用 Namespace

```bash
# 終端機 1：模擬器（無需 namespace）
ros2 run sensorpart sensorpart_simulator_node

# 終端機 2：Publisher（帶 namespace）
ros2 run sensorpart sensorpart_publisher_node --ros-args \
    -r __ns:=/cargo_agv_1 \
    -p host:=127.0.0.1

# 終端機 3：監聽（注意 namespace）
ros2 topic echo /cargo_agv_1/sensor/ocr
```

---

## 📊 測試資料集

模擬器會循環發送以下測試產品名稱：

```python
測試資料集：
1. ABC12345
2. DEF67890
3. ABC54321
4. DEF09876
5. TEST0001
6. PROD1234
```

每次自動發送會：
1. 發送 3D 位置（隨機座標）
2. 發送 OCR 結果（按順序輪流）

---

## 🧪 測試場景

### 場景 1：基本功能測試

**目的**：驗證 regex 解析正確性

```bash
# 1. 啟動模擬器和 publisher
# 2. 監聽 topics
ros2 topic echo /sensor/ocr

# 3. 確認：
# ✅ OCR 資料格式正確
# ✅ 每 5 秒收到新資料
# ✅ 產品名稱按順序輪流
```

### 場景 2：Regex 格式測試

**目的**：測試各種資料格式

模擬器發送的格式：
```
3D 定位成功: (005,P,1523,892,234,1.2,-0.5,3.4)
3D 定位失敗: (005,F,0,0,0,0.0,0.0,0.0)
OCR 結果: (OCR,ABC12345)
```

**驗證點**：
- ✅ 3D 定位成功時，position_3d topic 有資料
- ✅ 3D 定位失敗時，publisher 應忽略（不發布）
- ✅ OCR 資料正確解析產品名稱

### 場景 3：多次連接測試

**目的**：測試斷線重連

```bash
# 1. 啟動模擬器
# 2. 啟動 publisher
# 3. Ctrl+C 停止 publisher
# 4. 再次啟動 publisher

# 確認：
# ✅ Publisher 能重新連接
# ✅ 資料繼續正常傳輸
```

### 場景 4：壓力測試

**目的**：測試高頻發送

```bash
# 啟動模擬器（1 秒間隔）
ros2 run sensorpart sensorpart_simulator_node --ros-args -p send_interval:=1.0

# 確認：
# ✅ Publisher 能正確處理高頻資料
# ✅ 沒有資料遺失
# ✅ Topic 發布正常
```

---

## 🐛 除錯技巧

### 檢查 TCP 連接

```bash
# 查看 Port 2005 是否被監聽
ss -tulpn | grep 2005

# 預期輸出：
# tcp   LISTEN  0  1  0.0.0.0:2005  0.0.0.0:*
```

### 手動測試 TCP 伺服器

```bash
# 使用 telnet 連接模擬器
telnet 127.0.0.1 2005

# 應該看到模擬器日誌顯示「客戶端已連接」
```

### 檢查 ROS2 Topics

```bash
# 列出所有 topics
ros2 topic list

# 預期看到：
# /sensor/ocr
# /sensor/position_3d

# 查看 topic 資訊
ros2 topic info /sensor/ocr

# 預期輸出：
# Type: std_msgs/msg/String
# Publisher count: 1
# Subscription count: 0
```

### 檢查節點狀態

```bash
# 列出所有節點
ros2 node list

# 預期看到：
# /sensorpart_simulator
# /sensorpart_publisher

# 查看節點資訊
ros2 node info /sensorpart_publisher
```

---

## 📝 常見問題

### Q1: 模擬器顯示「等待客戶端連接」但 Publisher 無法連接

**原因**：IP 地址配置錯誤

**解決**：
```bash
# 確保 Publisher 使用正確的 IP
# 模擬器在容器內：使用 127.0.0.1
ros2 run sensorpart sensorpart_publisher_node --ros-args -p host:=127.0.0.1

# 如果模擬器在其他機器：使用該機器的 IP
ros2 run sensorpart sensorpart_publisher_node --ros-args -p host:=192.168.x.x
```

### Q2: Topic 沒有資料發布

**檢查**：
1. 模擬器是否正在發送資料？（查看日誌）
2. Publisher 是否已連接？（模擬器顯示「客戶端已連接」）
3. Topic 名稱是否正確？（注意 namespace）

```bash
# 查看 publisher 日誌
ros2 run sensorpart sensorpart_publisher_node --ros-args -p host:=127.0.0.1

# 應該看到類似輸出：
# [INFO] [sensorpart_publisher]: 📄 發布 OCR: ABC12345
```

### Q3: OCR 資料重複或不更新

**原因**：Publisher 的去重機制

**說明**：Publisher 會檢查資料是否與上次相同，相同則不重複發布。這是正常行為。

### Q4: 如何手動發送特定資料？

**目前限制**：模擬器目前為自動模式

**替代方案**：
1. 修改 `sensorpart_simulator_node.py` 中的 `test_products` 列表
2. 或關閉自動發送，未來可實作手動控制介面

---

## 🎓 進階：實際相機測試

當模擬測試通過後，可以連接實際相機：

```bash
# 停止模擬器（Ctrl+C）

# 啟動 Publisher 連接實際相機
ros2 run sensorpart sensorpart_publisher_node --ros-args -p host:=192.168.2.111

# 預期：
# ✅ 連接到實際相機
# ✅ 收到真實的 OCR 和 3D 定位資料
```

---

## ✅ 測試檢查清單

- [ ] 模擬器能成功啟動並監聽 Port 2005
- [ ] Publisher 能連接到模擬器
- [ ] `/sensor/ocr` topic 有資料發布
- [ ] `/sensor/position_3d` topic 有資料發布
- [ ] OCR 資料格式正確（產品名稱）
- [ ] 3D 位置資料格式正確（x, y, z, 方向）
- [ ] 去重機制正常運作（相同資料不重複發布）
- [ ] 支援 namespace（多 AGV 場景）
- [ ] 斷線重連功能正常
- [ ] 準備連接實際相機測試

---

## 📚 相關檔案

- 模擬器節點：`sensorpart_ws/src/sensorpart/sensorpart/sensorpart_simulator_node.py`
- Publisher 節點：`sensorpart_ws/src/sensorpart/sensorpart/sensorpart_publisher_node.py`
- 完整計劃：`sensorpart_ws/PRODUCT_VERIFICATION_PLAN.md`
