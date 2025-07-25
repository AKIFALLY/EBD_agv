# agv_interfaces - AGV ROS 2訊息接口定義

## 專案概述
agv_interfaces定義RosAGV系統中AGV相關的ROS 2訊息類型，提供標準化的通訊接口。確保系統內各節點間的數據交換格式統一，支援狀態同步和事件通知機制。

## 核心模組

### 訊息定義
- **AgvStatus.msg**: AGV完整狀態資訊
- **AgvStateChange.msg**: AGV狀態變更事件

## 關鍵檔案

### 訊息檔案
- `/msg/AgvStatus.msg` - AGV狀態數據結構
- `/msg/AgvStateChange.msg` - 狀態變更事件結構
- `/CMakeLists.txt` - CMake構建配置
- `/package.xml` - ROS 2套件元數據

### 訊息格式詳解

#### AgvStatus.msg 結構
```bash
# AGV基本資訊
string agv_id              # AGV編號
string agv_type            # AGV類型 (cargo/loader/unloader)
int32 battery_level        # 電池電量百分比
float64 position_x         # X座標位置
float64 position_y         # Y座標位置  
float64 rotation           # 旋轉角度

# 狀態資訊
string current_state       # 當前狀態名稱
string robot_state         # 機器人狀態
bool is_busy              # 是否繁忙
bool is_error             # 是否錯誤
string error_message      # 錯誤訊息

# 任務資訊
string current_task_id    # 當前任務ID
string destination        # 目標位置
int32 progress           # 任務進度百分比

# 時間戳記
builtin_interfaces/Time timestamp
```

#### AgvStateChange.msg 結構
```bash
# 狀態變更資訊
string agv_id             # AGV編號
string from_state         # 來源狀態
string to_state           # 目標狀態
string reason             # 狀態變更原因
bool success              # 變更是否成功
string error_message      # 錯誤訊息(如有)

# 時間戳記
builtin_interfaces/Time timestamp
```

## 開發指令

### 構建接口
```bash
# 進入AGV容器
docker compose -f docker-compose.yml exec rosagv bash
source /app/setup.bash && all_source

# 構建agv_interfaces
cd /app/agv_ws
colcon build --packages-select agv_interfaces

# 檢查生成的接口
ros2 interface show agv_interfaces/msg/AgvStatus
ros2 interface show agv_interfaces/msg/AgvStateChange
```

### 測試接口
```bash
# 發布測試訊息
ros2 topic pub /agv/status agv_interfaces/msg/AgvStatus "{
  agv_id: 'agv01',
  agv_type: 'cargo',
  battery_level: 85,
  position_x: 1.5,
  position_y: 2.3,
  rotation: 0.785,
  current_state: 'idle',
  robot_state: 'ready',
  is_busy: false,
  is_error: false
}"

# 監聽狀態訊息
ros2 topic echo /agv/status
ros2 topic echo /agv/state_change
```

## 配置設定

### CMakeLists.txt 重點
```cmake
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AgvStatus.msg"
  "msg/AgvStateChange.msg"
  DEPENDENCIES builtin_interfaces
)
```

### package.xml 依賴
```xml
<depend>builtin_interfaces</depend>
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

## 整合點

### 使用此接口的專案
- **agv_base**: 發布AgvStatus，處理AgvStateChange
- **cargo_mover_agv**: 使用AgvStatus發布車輛狀態
- **loader_agv**: 實現狀態更新和事件通知
- **unloader_agv**: 狀態同步和變更通知
- **agvcui**: 接收狀態資訊顯示AGV即時狀態
- **web_api_ws**: 透過WebSocket轉發狀態給前端

### ROS 2話題規範
```bash
# 標準話題命名
/agv/status               # AGV狀態發布
/agv/state_change        # 狀態變更事件
/<agv_id>/status         # 特定AGV狀態
/<agv_id>/state_change   # 特定AGV狀態變更
```

## 測試方法

### 接口驗證
```bash
# 檢查訊息格式
ros2 interface show agv_interfaces/msg/AgvStatus
ros2 interface show agv_interfaces/msg/AgvStateChange

# 驗證欄位類型  
ros2 interface proto agv_interfaces/msg/AgvStatus
```

### 通訊測試
```bash
# 發布者測試
ros2 topic pub /agv/status agv_interfaces/msg/AgvStatus "{agv_id: 'test'}"

# 訂閱者測試
ros2 topic echo /agv/status

# 訊息頻率測試
ros2 topic hz /agv/status
```

### 資料完整性測試
```bash
# 檢查欄位完整性
ros2 topic echo /agv/status --csv

# 時間戳記驗證
ros2 topic echo /agv/status | grep timestamp
```

## 故障排除

### 常見問題

#### 接口未找到
```bash
# 重新構建接口
cd /app/agv_ws
colcon build --packages-select agv_interfaces --cmake-clean-cache

# 重新載入環境
source install/setup.bash
```

#### 訊息格式錯誤
```bash
# 檢查訊息定義語法
ros2 interface show agv_interfaces/msg/AgvStatus

# 驗證欄位名稱和類型
cat /app/agv_ws/src/agv_interfaces/msg/AgvStatus.msg
```

#### 依賴問題
```bash
# 檢查依賴套件
rosdep check agv_interfaces

# 安裝缺失依賴
rosdep install --from-paths . --ignore-src -r -y
```

### 除錯技巧
- 使用`ros2 interface list`確認接口已正確生成
- 檢查`builtin_interfaces`依賴是否正確
- 驗證CMakeLists.txt中的`rosidl_generate_interfaces`配置
- 確保package.xml包含必要的構建和執行依賴

### 版本兼容性
- 確保與ROS 2 Jazzy版本兼容
- 檢查`builtin_interfaces`版本一致性
- 驗證訊息序列化和反序列化正確性
- 測試與其他ROS 2發行版的交互性