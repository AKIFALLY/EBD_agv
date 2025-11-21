# Cargo AGV 產品驗證功能完整計劃

**計劃版本**: 1.0
**建立日期**: 2025-01-19
**狀態**: 第一階段（sensorpart_ws）已完成，待整合至 agv_ws

---

## 📋 需求確認

### 業務需求
從資料庫中取得目前 AGV 所在房間生產的產品名稱，使用 sensorpart OCR 相機讀取實際產品名稱，比較是否相同，將驗證結果寫入 AGV 的 PLC。

### 技術需求
- ✅ **實作位置**：WaitRobotState（不新增狀態）
- ✅ **房間識別**：從 work_id 第1碼判斷房間（例如：work_id=1234 → 房間1）
- ✅ **檢查時機**：在 handle() 中執行（本來就定時執行）
- ✅ **SensorPart**：獨立 ROS2 節點持續運行，發布 OCR topic
- ✅ **驗證結果**：Pass/Fail 寫入 PLC MR 位元
- ✅ **錯誤處理**：驗證失敗時停止並等待人工處理

---

## 🏗️ 架構設計

### 整體架構圖
```
┌─────────────────────────────────────────────┐
│  SensorPart 相機 (192.168.2.111:2005)      │
│  - 3D 定位                                  │
│  - OCR 識別                                 │
└──────────────┬──────────────────────────────┘
               │ TCP
               ↓
┌─────────────────────────────────────────────┐
│  sensorpart_publisher_node (ROS2 節點)     │
│  - TCP 客戶端接收資料                       │
│  - 發布 topics                              │
│    ├─ {namespace}/sensor/ocr (String)      │
│    └─ {namespace}/sensor/position_3d (Pose)│
└──────────────┬──────────────────────────────┘
               │ ROS2 Topic
               ↓
┌─────────────────────────────────────────────┐
│  WaitRobotState (agv_ws)                   │
│  - 訂閱 OCR topic                           │
│  - 從 work_id 取得房間 ID                   │
│  - 查詢資料庫產品清單                       │
│  - 比對產品名稱                             │
│  - 寫入 PLC 驗證結果                        │
└──────────────┬──────────────────────────────┘
               │ PLC Write
               ↓
┌─────────────────────────────────────────────┐
│  PLC (Keyence)                              │
│  - MR 7101: 驗證 Pass                       │
│  - MR 7102: 驗證 Fail                       │
└─────────────────────────────────────────────┘
```

### Namespace 設計
```
多 AGV 場景支援：
/cargo_agv_1/sensor/ocr           → Cargo AGV 1 的 OCR
/cargo_agv_1/sensor/position_3d   → Cargo AGV 1 的 3D 位置

/cargo_agv_2/sensor/ocr           → Cargo AGV 2 的 OCR
/cargo_agv_2/sensor/position_3d   → Cargo AGV 2 的 3D 位置
```

---

## 📝 實作狀態

### ✅ 第一階段：sensorpart_ws（已完成）

#### 新建檔案
1. **sensorpart_publisher_node.py** - ✅ 完成
   - 路徑：`sensorpart_ws/src/sensorpart/sensorpart/sensorpart_publisher_node.py`
   - 功能：
     - TCP 客戶端連接相機（192.168.2.111:2005）
     - 發布 `/sensor/ocr` topic（String）
     - 發布 `/sensor/position_3d` topic（PoseStamped）
     - 支援參數配置（host, port, publish_rate）
     - 支援 namespace
     - 避免重複發布相同資料

2. **sensorpart_publisher.launch.py** - ✅ 完成
   - 路徑：`sensorpart_ws/src/sensorpart/launch/sensorpart_publisher.launch.py`
   - 功能：
     - 啟動 sensorpart_publisher_node
     - 支援 namespace 參數（多 AGV）
     - 支援 host/port/publish_rate 參數

#### 修改檔案
3. **setup.py** - ✅ 完成
   - 路徑：`sensorpart_ws/src/sensorpart/setup.py`
   - 修改：
     - 新增 entry point: `sensorpart_publisher_node`
     - 新增 launch 檔案安裝

### ⏳ 第二階段：agv_ws 整合（待實作）

#### 修改檔案
1. **wait_robot_state.py** - ⏳ 待實作
   - 路徑：`agv_ws/src/agv_base/agv_base/agv_states/wait_robot_state.py`
   - 功能：
     - 訂閱 `/sensor/ocr` topic
     - 收到新 OCR 時觸發驗證
     - 從 work_id 解析房間 ID
     - 查詢資料庫產品清單
     - 比對產品名稱
     - 寫入 PLC Pass/Fail 結果

2. **package.xml** - ⏳ 待實作
   - 路徑：`agv_ws/src/agv_base/package.xml`
   - 新增依賴：`<depend>sensorpart</depend>`

3. **CLAUDE.md** - ⏳ 待實作
   - 路徑：`agv_ws/CLAUDE.md`
   - 記錄新增的 PLC 地址定義

---

## 📊 資料流程

### 1. OCR 資料發布流程
```
1. SensorPart 相機透過 TCP 發送 OCR 資料
   格式：(OCR,產品名稱)
   範例：(OCR,ABC12345)

2. sensorpart_publisher_node 接收並解析
   tcp_client.ocr_result = "ABC12345"

3. 發布到 ROS2 topic
   Topic: /cargo_agv_1/sensor/ocr
   Type: std_msgs/String
   Data: "ABC12345"

4. WaitRobotState 訂閱並收到資料
   _ocr_callback(msg) 觸發驗證
```

### 2. 產品驗證流程
```
1. 收到 OCR 資料（例如："ABC12345"）

2. 從 work_id 解析房間
   work_id = 1234 → 第1碼 = 1 → 房間1

3. 查詢資料庫
   SQL: SELECT p.name FROM product p
        JOIN room r ON p.process_settings_id = r.process_settings_id
        WHERE r.id = 1

   結果：["ABC12345", "ABC54321"]

4. 比對產品名稱
   "ABC12345" in ["ABC12345", "ABC54321"] → Pass

5. 寫入 PLC
   Pass → MR 7101 = ON, MR 7102 = OFF
   Fail → MR 7101 = OFF, MR 7102 = ON
```

---

## 🔧 配置說明

### SensorPart 相機配置
```yaml
相機 IP: 192.168.2.111
相機 Port: 2005
協議: TCP
編碼: UTF-8
```

### ROS2 Topic 配置
```yaml
OCR Topic:
  名稱: {namespace}/sensor/ocr
  類型: std_msgs/String
  頻率: 10 Hz（有新資料時發布）

Position Topic:
  名稱: {namespace}/sensor/position_3d
  類型: geometry_msgs/PoseStamped
  頻率: 10 Hz（有新資料時發布）
```

### PLC 地址配置
```yaml
驗證結果：
  MR 7101: 驗證 Pass（AGV → PLC）
  MR 7102: 驗證 Fail（AGV → PLC）
```

---

## 🧪 測試計劃

### 階段一：sensorpart_ws 節點測試（✅ 可執行）

#### 測試 1：建置套件
```bash
# 進入 AGV 容器
cd ~/RosAGV
docker compose -f docker-compose.yml exec rosagv bash

# 載入環境
source /app/setup.bash && agv_source

# 建置 sensorpart
cd /app/sensorpart_ws
colcon build --packages-select sensorpart
source install/setup.bash
```

#### 測試 2：啟動節點（不帶 namespace）
```bash
# 方式 1：直接執行
ros2 run sensorpart sensorpart_publisher_node

# 方式 2：使用 launch
ros2 launch sensorpart sensorpart_publisher.launch.py

# 預期結果：
# ✅ 節點啟動成功
# ✅ 連接到相機 192.168.2.111:2005
# ✅ 日誌顯示 topic 路徑
```

#### 測試 3：啟動節點（帶 namespace）
```bash
# 使用 namespace
ros2 launch sensorpart sensorpart_publisher.launch.py namespace:=cargo_agv_1

# 預期結果：
# ✅ Topics 在 /cargo_agv_1/ namespace 下
```

#### 測試 4：查看 Topics
```bash
# 列出所有 topics
ros2 topic list

# 預期結果（不帶 namespace）：
# /sensor/ocr
# /sensor/position_3d

# 預期結果（帶 namespace = cargo_agv_1）：
# /cargo_agv_1/sensor/ocr
# /cargo_agv_1/sensor/position_3d
```

#### 測試 5：監聽 OCR Topic
```bash
# 監聽 OCR 資料
ros2 topic echo /sensor/ocr

# 或（帶 namespace）
ros2 topic echo /cargo_agv_1/sensor/ocr

# 預期結果：
# 當相機識別到產品時，會輸出產品名稱
# data: "ABC12345"
```

#### 測試 6：監聽 3D 位置 Topic
```bash
# 監聽 3D 位置
ros2 topic echo /sensor/position_3d

# 預期結果：
# 當相機定位成功時，會輸出位置資料
# header:
#   stamp: ...
#   frame_id: sensor_frame
# pose:
#   position:
#     x: 1250.0
#     y: 890.0
#     z: 120.0
```

#### 測試 7：自訂參數
```bash
# 自訂相機 IP 和發布頻率
ros2 launch sensorpart sensorpart_publisher.launch.py \
    host:=192.168.2.111 \
    port:=2005 \
    publish_rate:=5.0

# 預期結果：
# ✅ 使用自訂參數啟動
```

### 階段二：agv_ws 整合測試（⏳ 待實作）

#### 測試 8：WaitRobotState 訂閱測試
```bash
# 測試 WaitRobotState 是否正確訂閱 OCR topic
# 待實作
```

#### 測試 9：產品驗證邏輯測試
```bash
# 測試完整的產品驗證流程
# 待實作
```

#### 測試 10：PLC 寫入測試
```bash
# 測試 Pass/Fail 是否正確寫入 PLC
# 待實作
```

---

## 🚀 部署指南

### 開發環境部署

#### 1. 建置套件
```bash
cd ~/RosAGV
docker compose -f docker-compose.yml exec rosagv bash
source /app/setup.bash && agv_source
cd /app/sensorpart_ws
colcon build --packages-select sensorpart
```

#### 2. 啟動節點（測試模式）
```bash
# 不帶 namespace
ros2 run sensorpart sensorpart_publisher_node

# 帶 namespace
ros2 run sensorpart sensorpart_publisher_node --ros-args -r __ns:=/cargo_agv_1
```

### 生產環境部署

#### 整合到現有 Launch 檔案
```python
# 在 agv_ws 的 launch 檔案中加入
Node(
    package='sensorpart',
    executable='sensorpart_publisher_node',
    name='sensorpart_publisher',
    namespace='cargo_agv_1',  # 根據 AGV 設定
    parameters=[{
        'host': '192.168.2.111',
        'port': 2005,
        'publish_rate': 10.0,
    }],
    output='screen',
)
```

---

## 📖 使用範例

### 訂閱 OCR 資料（Python）
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class OCRSubscriberNode(Node):
    def __init__(self):
        super().__init__('ocr_subscriber')
        self.subscription = self.create_subscription(
            String,
            'sensor/ocr',  # 或 '/cargo_agv_1/sensor/ocr'
            self.ocr_callback,
            10
        )

    def ocr_callback(self, msg):
        self.get_logger().info(f'收到 OCR: {msg.data}')

def main():
    rclpy.init()
    node = OCRSubscriberNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

### 在 WaitRobotState 中訂閱（待實作範例）
```python
class WaitRobotState(State):
    def __init__(self, node):
        super().__init__(node)

        # 訂閱 OCR topic
        self.ocr_subscription = self.node.create_subscription(
            String,
            'sensor/ocr',
            self._ocr_callback,
            10
        )
        self.last_verified_ocr = None

    def _ocr_callback(self, msg):
        """收到新 OCR 時觸發驗證"""
        ocr_result = msg.data

        # 避免重複驗證
        if ocr_result == self.last_verified_ocr:
            return

        self.node.get_logger().info(f'🔍 收到 OCR: {ocr_result}')
        self._verify_product(ocr_result)
        self.last_verified_ocr = ocr_result
```

---

## 🔗 相關文檔

### 已完成
- `sensorpart_ws/src/sensorpart/sensorpart/sensorpart_publisher_node.py`
- `sensorpart_ws/src/sensorpart/launch/sensorpart_publisher.launch.py`
- `sensorpart_ws/src/sensorpart/setup.py`
- `sensorpart_ws/CLAUDE.md` - 感測器工作空間文檔

### 待更新
- `agv_ws/src/agv_base/agv_base/agv_states/wait_robot_state.py`
- `agv_ws/src/agv_base/package.xml`
- `agv_ws/CLAUDE.md`

---

## 📊 待辦事項

### sensorpart_ws（✅ 已完成）
- [x] 建立 sensorpart_publisher_node.py
- [x] 建立 sensorpart_publisher.launch.py
- [x] 修改 setup.py
- [x] 撰寫完整計劃文檔
- [ ] 測試節點功能和 topic 發布

### agv_ws（⏳ 待實作）
- [ ] 修改 WaitRobotState 訂閱 OCR topic
- [ ] 實作 work_id → room_id 解析
- [ ] 實作資料庫查詢產品邏輯
- [ ] 實作產品名稱比對邏輯
- [ ] 實作 PLC 寫入 Pass/Fail
- [ ] 整合測試
- [ ] 更新文檔

---

## 📝 變更記錄

### v1.0 (2025-01-19)
- ✅ 完成 sensorpart_ws 獨立節點實作
- ✅ 支援 namespace 多 AGV 場景
- ✅ 支援參數配置
- ✅ 更新相機 IP 為 192.168.2.111
- ✅ 撰寫完整計劃文檔
