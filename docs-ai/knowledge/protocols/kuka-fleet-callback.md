# KUKA Fleet 任務狀態回調規格

## 🎯 適用場景
- 理解 KUKA Fleet 系統的任務狀態回調機制
- 實作 KUKA Fleet 狀態回調的接收和處理
- 設計任務狀態監控和事件處理邏輯
- 為上游系統提供 KUKA Fleet 任務狀態整合

## 📋 回調系統概覽

### 回調機制說明
KUKA Fleet Manager 透過 `missionStateCallback` API 主動向上游系統回報任務執行狀態，實現即時的任務狀態同步和監控。

### 基本資訊
- **API 名稱**: `missionStateCallback`
- **HTTP 方法**: `POST`
- **Content-Type**: `application/json`
- **回調方向**: KUKA Fleet → 上游系統 (RosAGV)
- **觸發時機**: 任務狀態變更時自動觸發

### 回調 URL 格式
```
http://[上游系統IP:Port]/interfaces/api/amr/missionStateCallback
```

## 📋 回調請求規格

### 請求參數結構
```json
{
    "missionCode": "mission202309250005",
    "viewBoardType": "",
    "containerCode": "1000002",
    "currentPosition": "M001-A001-31",
    "slotCode": "",
    "robotId": "14",
    "missionStatus": "MOVE_BEGIN",
    "message": "",
    "missionData": {}
}
```

### 參數詳細說明
| 參數名稱 | 類型 | 必填 | 最大長度 | 說明 |
|----------|------|------|----------|------|
| `missionCode` | String | 是 | 32 | 任務代碼 ID，唯一識別任務 |
| `viewBoardType` | String | 否 | - | 任務類型分類 |
| `containerCode` | String | 否 | - | 關聯的容器代碼 |
| `currentPosition` | String | 否 | - | 容器或機器人當前位置 |
| `slotCode` | String | 否 | - | 所在槽位代碼 |
| `robotId` | String | 否 | - | 執行任務的機器人 ID |
| `missionStatus` | String | 是 | - | 任務當前狀態 (詳見狀態說明) |
| `message` | String | 否 | - | 額外的狀態說明或錯誤訊息 |
| `missionData` | Object | 否 | - | 任務自訂資料，可為空物件 |

## 🔄 任務狀態定義

### 核心狀態流程
```
任務狀態流程
┌─────────────┐    ┌─────────────┐    ┌─────────────┐
│ MOVE_BEGIN  │───▶│   ARRIVED   │───▶│  COMPLETED  │
│   開始移動   │    │  到達節點    │    │   任務完成   │
└─────────────┘    └─────────────┘    └─────────────┘
       │                   │                   │
       │                   ▼                   │
       │            ┌─────────────┐            │
       │            │ 設備操作狀態  │            │
       │            │ (各種操作)    │            │
       │            └─────────────┘            │
       │                                       │
       ▼                                       │
┌─────────────┐                               │
│  CANCELED   │◀──────────────────────────────┘
│  任務取消    │
└─────────────┘
```

### 詳細狀態說明

#### 基本移動狀態
- **`MOVE_BEGIN`**: 開始移動
  - **觸發時機**: 機器人開始執行移動指令
  - **用途**: 標記任務開始執行
  - **後續狀態**: 通常轉為 `ARRIVED`

- **`ARRIVED`**: 到達任務節點
  - **觸發時機**: 機器人到達指定位置
  - **用途**: 確認位置到達
  - **後續狀態**: 可能進入設備操作或完成

#### 容器操作狀態
- **`UP_CONTAINER`**: 升箱完成
  - **觸發時機**: 機器人完成容器升起動作
  - **適用車型**: Cargo Mover, Loader
  - **操作說明**: 機械臂或升降裝置將容器抬起

- **`DOWN_CONTAINER`**: 放下完成
  - **觸發時機**: 機器人完成容器放下動作
  - **適用車型**: Cargo Mover, Unloader
  - **操作說明**: 機械臂或升降裝置將容器放下

#### 輸送設備操作狀態
- **`ROLLER_RECEIVE`**: 滾筒上料完成
  - **觸發時機**: 透過滾筒系統接收物料
  - **設備類型**: 滾筒輸送機
  - **操作方向**: 從輸送系統到 AGV

- **`ROLLER_SEND`**: 滾筒下料完成
  - **觸發時機**: 透過滾筒系統發送物料
  - **設備類型**: 滾筒輸送機
  - **操作方向**: 從 AGV 到輸送系統

#### 料箱操作狀態
- **`PICKER_RECEIVE`**: 料箱取料完成
  - **觸發時機**: 從料箱系統取得物料
  - **設備類型**: 料箱分揀系統
  - **適用車型**: Loader

- **`PICKER_SEND`**: 料箱下料完成
  - **觸發時機**: 向料箱系統投遞物料
  - **設備類型**: 料箱分揀系統
  - **適用車型**: Unloader

#### 叉車操作狀態
- **`FORK_UP`**: 叉車叉取完成
  - **觸發時機**: 叉車叉起物料或容器
  - **設備類型**: 叉車式 AGV
  - **操作說明**: 叉取動作完成

- **`FORK_DOWN`**: 叉車放下完成
  - **觸發時機**: 叉車放下物料或容器
  - **設備類型**: 叉車式 AGV
  - **操作說明**: 放下動作完成

#### 終結狀態
- **`COMPLETED`**: 任務完成
  - **觸發時機**: 所有任務步驟成功完成
  - **最終狀態**: 任務正常結束
  - **後續動作**: 機器人進入空閒狀態

- **`CANCELED`**: 任務取消完成
  - **觸發時機**: 任務被手動或系統取消
  - **最終狀態**: 任務異常結束
  - **後續動作**: 機器人釋放資源

## 🔧 回調處理實作

### 回調接收服務
```python
from flask import Flask, request, jsonify
import logging

app = Flask(__name__)
logger = logging.getLogger(__name__)

@app.route('/interfaces/api/amr/missionStateCallback', methods=['POST'])
def mission_state_callback():
    """接收 KUKA Fleet 任務狀態回調"""
    try:
        # 解析回調資料
        callback_data = request.get_json()
        
        # 驗證必要參數
        if not callback_data.get('missionCode'):
            return jsonify({
                "success": False,
                "message": "missionCode is required"
            }), 400
        
        if not callback_data.get('missionStatus'):
            return jsonify({
                "success": False,
                "message": "missionStatus is required"
            }), 400
        
        # 處理狀態回調
        process_mission_callback(callback_data)
        
        # 回傳成功回應
        return jsonify({
            "success": True,
            "message": "Callback processed successfully"
        }), 200
        
    except Exception as e:
        logger.error(f"回調處理錯誤: {str(e)}")
        return jsonify({
            "success": False,
            "message": "Internal server error"
        }), 500

def process_mission_callback(callback_data):
    """處理任務狀態回調邏輯"""
    mission_code = callback_data.get('missionCode')
    status = callback_data.get('missionStatus')
    robot_id = callback_data.get('robotId')
    container_code = callback_data.get('containerCode')
    position = callback_data.get('currentPosition')
    
    logger.info(f"任務 {mission_code} 狀態更新: {status}")
    
    # 根據狀態執行對應處理
    if status == 'MOVE_BEGIN':
        handle_move_begin(mission_code, robot_id)
    elif status == 'ARRIVED':
        handle_arrived(mission_code, robot_id, position)
    elif status == 'COMPLETED':
        handle_mission_completed(mission_code, robot_id)
    elif status == 'CANCELED':
        handle_mission_canceled(mission_code, robot_id)
    else:
        handle_equipment_operation(mission_code, status, callback_data)
```

### 狀態處理邏輯
```python
def handle_move_begin(mission_code, robot_id):
    """處理移動開始狀態"""
    # 更新任務狀態為執行中
    update_mission_status(mission_code, 'RUNNING')
    # 更新機器人狀態為任務中
    update_robot_status(robot_id, 'WORKING')
    # 記錄任務開始時間
    log_mission_event(mission_code, 'MOVE_BEGIN')

def handle_arrived(mission_code, robot_id, position):
    """處理到達節點狀態"""
    # 更新機器人當前位置
    update_robot_position(robot_id, position)
    # 檢查是否需要等待操作完成
    check_operation_required(mission_code, position)
    # 記錄到達事件
    log_mission_event(mission_code, 'ARRIVED', {'position': position})

def handle_mission_completed(mission_code, robot_id):
    """處理任務完成狀態"""
    # 更新任務狀態為完成
    update_mission_status(mission_code, 'COMPLETED')
    # 更新機器人狀態為空閒
    update_robot_status(robot_id, 'IDLE')
    # 釋放相關資源
    release_mission_resources(mission_code)
    # 記錄完成事件
    log_mission_event(mission_code, 'COMPLETED')

def handle_equipment_operation(mission_code, status, callback_data):
    """處理設備操作狀態"""
    equipment_operations = {
        'UP_CONTAINER': '升箱完成',
        'DOWN_CONTAINER': '放下完成',
        'ROLLER_RECEIVE': '滾筒上料完成',
        'ROLLER_SEND': '滾筒下料完成',
        'PICKER_RECEIVE': '料箱取料完成',
        'PICKER_SEND': '料箱下料完成',
        'FORK_UP': '叉車叉取完成',
        'FORK_DOWN': '叉車放下完成'
    }
    
    operation_name = equipment_operations.get(status, f'未知操作: {status}')
    logger.info(f"任務 {mission_code} 設備操作: {operation_name}")
    
    # 記錄設備操作事件
    log_mission_event(mission_code, status, callback_data)
    
    # 檢查是否需要發送操作完成確認
    if requires_operation_feedback(status):
        send_operation_feedback(mission_code, callback_data)
```

### ROS 2 整合
```python
import rclpy
from rclpy.node import Node
from agv_interfaces.msg import MissionStatus

class KukaCallbackHandler(Node):
    """KUKA Fleet 回調處理的 ROS 2 節點"""
    
    def __init__(self):
        super().__init__('kuka_callback_handler')
        
        # 建立狀態發布者
        self.status_publisher = self.create_publisher(
            MissionStatus, 
            'kuka_mission_status', 
            10
        )
        
    def handle_callback(self, callback_data):
        """處理回調並發布 ROS 2 訊息"""
        # 轉換為 ROS 2 訊息格式
        status_msg = MissionStatus()
        status_msg.mission_code = callback_data.get('missionCode', '')
        status_msg.robot_id = callback_data.get('robotId', '')
        status_msg.status = callback_data.get('missionStatus', '')
        status_msg.position = callback_data.get('currentPosition', '')
        status_msg.container_code = callback_data.get('containerCode', '')
        status_msg.timestamp = self.get_clock().now().to_msg()
        
        # 發布狀態訊息
        self.status_publisher.publish(status_msg)
        
        self.get_logger().info(
            f"發布 KUKA 任務狀態: {status_msg.mission_code} - {status_msg.status}"
        )
```

## 🔍 監控和除錯

### 回調日誌記錄
```python
import logging
from datetime import datetime

def setup_callback_logging():
    """設定回調日誌記錄"""
    logger = logging.getLogger('kuka_callback')
    logger.setLevel(logging.INFO)
    
    # 檔案處理器
    file_handler = logging.FileHandler('/tmp/kuka_callback.log')
    file_handler.setLevel(logging.INFO)
    
    # 格式設定
    formatter = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    file_handler.setFormatter(formatter)
    
    logger.addHandler(file_handler)
    return logger

def log_callback_received(callback_data):
    """記錄接收到的回調"""
    logger = logging.getLogger('kuka_callback')
    logger.info(f"接收回調: {callback_data}")

def log_callback_processed(mission_code, status, processing_time):
    """記錄回調處理完成"""
    logger = logging.getLogger('kuka_callback')
    logger.info(f"回調處理完成: {mission_code} - {status} (耗時: {processing_time}ms)")
```

### 回調統計和監控
```python
from collections import defaultdict
import time

class CallbackMonitor:
    """回調監控統計"""
    
    def __init__(self):
        self.callback_count = defaultdict(int)
        self.last_callback_time = {}
        self.processing_times = []
    
    def record_callback(self, mission_code, status, processing_time):
        """記錄回調統計"""
        self.callback_count[status] += 1
        self.last_callback_time[mission_code] = time.time()
        self.processing_times.append(processing_time)
        
        # 保持最近 1000 筆處理時間記錄
        if len(self.processing_times) > 1000:
            self.processing_times = self.processing_times[-1000:]
    
    def get_statistics(self):
        """獲取統計資訊"""
        avg_processing_time = (
            sum(self.processing_times) / len(self.processing_times)
            if self.processing_times else 0
        )
        
        return {
            'total_callbacks': sum(self.callback_count.values()),
            'status_distribution': dict(self.callback_count),
            'average_processing_time': avg_processing_time,
            'active_missions': len(self.last_callback_time)
        }
```

## 🚨 錯誤處理和容錯

### 回調重試機制
```python
import time
from functools import wraps

def retry_on_failure(max_retries=3, delay=1):
    """回調處理失敗重試裝飾器"""
    def decorator(func):
        @wraps(func)
        def wrapper(*args, **kwargs):
            for attempt in range(max_retries):
                try:
                    return func(*args, **kwargs)
                except Exception as e:
                    if attempt == max_retries - 1:
                        raise
                    logger.warning(f"回調處理失敗 (嘗試 {attempt + 1}/{max_retries}): {str(e)}")
                    time.sleep(delay * (2 ** attempt))  # 指數退避
            return None
        return wrapper
    return decorator

@retry_on_failure(max_retries=3)
def process_callback_with_retry(callback_data):
    """帶重試機制的回調處理"""
    return process_mission_callback(callback_data)
```

### 異常狀態處理
```python
def handle_callback_error(callback_data, error):
    """處理回調錯誤"""
    mission_code = callback_data.get('missionCode', 'UNKNOWN')
    
    logger.error(f"回調處理錯誤 - 任務: {mission_code}, 錯誤: {str(error)}")
    
    # 記錄錯誤事件
    log_mission_event(mission_code, 'CALLBACK_ERROR', {
        'error': str(error),
        'callback_data': callback_data
    })
    
    # 檢查是否需要通知相關系統
    if is_critical_error(error):
        notify_system_administrators(mission_code, error)
```

## 💡 最佳實踐

### 回調服務設計原則
1. **響應速度**: 快速回應 KUKA Fleet，避免超時
2. **冪等性**: 同一回調多次接收應產生相同結果
3. **錯誤處理**: 完善的錯誤處理和恢復機制
4. **日誌記錄**: 詳細記錄回調事件用於除錯

### 狀態同步策略
1. **即時更新**: 收到回調立即更新系統狀態
2. **狀態驗證**: 驗證狀態轉換的合理性
3. **資料一致性**: 確保多個系統間狀態一致
4. **失敗恢復**: 回調失敗時的狀態恢復機制

### 效能最佳化
1. **非同步處理**: 使用非同步方式處理回調
2. **批次操作**: 適當時機進行批次資料庫操作
3. **快取機制**: 快取常用資料減少資料庫查詢
4. **連接池**: 使用資料庫連接池提高效率

## 🔗 交叉引用
- KUKA Fleet API 規格: @docs-ai/knowledge/protocols/kuka-fleet-api.md
- KUKA Fleet 適配器實作: @app/kuka_fleet_ws/CLAUDE.md
- ROS 2 訊息介面: @app/agv_ws/src/agv_interfaces/CLAUDE.md
- 任務狀態管理: @docs-ai/knowledge/agv-domain/mission-management.md