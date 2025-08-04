# Simple WCS 開發操作指導

## 🎯 適用場景
- Simple WCS 系統的開發、建置、測試操作
- 業務流程配置的開發和調試
- AI Agent 自動化配置管理開發

## 📋 Simple WCS 開發環境

### 開發環境要求
- **容器環境**: 必須在 AGVC 容器內開發
- **資料庫依賴**: 需要 PostgreSQL 和 db_proxy 系統
- **ROS 2 環境**: ROS 2 Jazzy + 適當的 RMW 實作
- **Python 環境**: Python 3.12 + SQLModel + 虛擬環境

### 基本開發流程
```bash
# 1. 進入 AGVC 容器 (Simple WCS 需要資料庫存取)
docker compose -f docker-compose.agvc.yml exec agvc_server bash

# 2. 載入環境 (自動設定 RMW 和 PYTHONPATH)
all_source  # 或 agvc_source

# 3. 進入工作空間
cd /app/simple_wcs_ws

# 4. 建置套件 (首次或修改後)
colcon build --packages-select simple_wcs

# 5. 載入建置結果
source install/setup.bash
```

## 🔧 Simple WCS 工作空間結構

### 實際專案目錄結構
```
simple_wcs_ws/
├── src/simple_wcs/
│   ├── package.xml                        # ROS 2 套件配置
│   ├── setup.py                          # Python 套件安裝
│   ├── setup.cfg                         # 建置配置
│   ├── launch/
│   │   └── simple_wcs_launch.py          # ROS 2 Launch 檔案
│   ├── resource/
│   └── simple_wcs/                       # Python 套件
│       ├── __init__.py
│       ├── wcs_engine.py                 # 核心決策引擎 (ROS 2 節點)
│       ├── flow_parser.py                # 多檔案 YAML 解析器
│       └── database_client.py            # SQLModel 資料庫客戶端
├── build/                                # 建置目錄 (自動生成)
├── install/                              # 安裝目錄 (自動生成)
├── log/                                  # 建置日誌 (自動生成)
├── test_simple_wcs.py                    # 基本功能測試
├── test_exit_rotation.py                 # 出口旋轉測試
└── create_exit_test_data.py              # 建立出口測試資料
```

### 配置檔案結構 (待建立)
```
/app/config/wcs/
├── flows/                                # 業務流程目錄 (需要建立)
│   ├── rack_rotation_inlet.yaml          # Rack旋轉-入口 (優先級100)
│   ├── rack_rotation_exit.yaml           # Rack旋轉-出口 (優先級90)
│   └── full_rack_to_manual_area.yaml     # 滿料架運輸 (優先級80)
└── locations.yaml                        # 靜態位置配置 (需要建立)
```

## 🚀 開發操作指導

### 建置和測試
```bash
# 建置 Simple WCS 套件
cd /app/simple_wcs_ws
colcon build --packages-select simple_wcs

# 載入建置結果
source install/setup.bash

# 執行現有測試檔案
python3 test_simple_wcs.py                    # 基本功能測試
python3 test_exit_rotation.py                 # 出口旋轉測試

# 建立測試資料 (如果需要)
python3 create_exit_test_data.py              # 建立出口測試資料

# 手動測試解析器 (需要先建立配置檔案)
python3 -c "
import sys
sys.path.append('/app/simple_wcs_ws/src/simple_wcs')
from simple_wcs.flow_parser import FlowParser

# 測試解析器
parser = FlowParser('/app/config/wcs/flows')
flows = parser.parse()
print(f'載入 {len(flows)} 個業務流程')
for flow in flows:
    print(f'- {flow.name} (優先級: {flow.priority})')
    
# 驗證配置
validation = parser.validate_flows(flows)
print('驗證結果:', validation)
"

# 測試資料庫客戶端
python3 -c "
import sys
sys.path.append('/app/simple_wcs_ws/src/simple_wcs')
sys.path.append('/app/db_proxy_ws/src')
from simple_wcs.database_client import DatabaseClient

db = DatabaseClient()
print('✅ 資料庫連接成功')

# 獲取統計資訊
stats = db.get_rack_statistics()
print(f'Rack 統計: {stats}')
"
```

### 啟動 Simple WCS 服務
```bash
# 確保在 AGVC 容器內且已載入環境
all_source
cd /app/simple_wcs_ws
source install/setup.bash

# 方法 1: 使用 ROS 2 run (推薦)
ros2 run simple_wcs simple_wcs_node

# 方法 2: 使用 Launch 檔案
ros2 launch simple_wcs simple_wcs_launch.py

# 方法 3: 直接執行主函數
python3 -c "
import sys
sys.path.append('src/simple_wcs')
from simple_wcs.wcs_engine import main
main()
"

# 方法 4: 背景執行
nohup ros2 run simple_wcs simple_wcs_node > /tmp/simple_wcs.log 2>&1 &
```

## 📊 業務流程開發

### 建立配置檔案目錄
```bash
# 1. 建立配置目錄結構
mkdir -p /app/config/wcs/flows

# 2. 建立位置配置檔案
cat > /app/config/wcs/locations.yaml << 'EOF'
# 房間位置配置
rooms:
  "1":
    inlet:
      stop_point: 10001      # 房間1入口停靠點
      rotation_point: 10101  # 入口旋轉中間點
    exit:
      stop_point: 10002      # 房間1出口停靠點
      rotation_point: 10102  # 出口旋轉中間點
  "2":
    inlet:
      stop_point: 20001
      rotation_point: 20101
    exit:
      stop_point: 20002
      rotation_point: 20102
  # 可擴展到房間3-5
EOF

# 3. 建立 Rack 旋轉入口流程
cat > /app/config/wcs/flows/rack_rotation_inlet.yaml << 'EOF'
name: "Rack旋轉檢查-房間入口"
description: "當 Rack A面完成後，檢查是否需要旋轉處理 B面"
priority: 100
work_id: "220001"
enabled: true

trigger_conditions:
  - condition: "rack_at_location_exists"
    description: "房間入口位置有 Rack"
    parameters:
      location_type: "room_inlet"
      
  - condition: "rack_side_completed" 
    description: "Rack A面已完成"
    parameters:
      side: "A"
      
  - condition: "rack_has_b_side_work"
    description: "Rack B面有待處理工作"
    parameters: {}
      
  - condition: "rack_needs_rotation_for_b_side"
    description: "Rack 需要旋轉以處理 B面"
    parameters:
      location_type: "room_inlet"
      
  - condition: "no_active_task"
    description: "該位置無衝突任務"
    parameters:
      work_id: "220001"

action:
  type: "create_task"
  task_type: "rack_rotation"
  function: "rack_move"
  model: "KUKA400i"
  api: "submit_mission"
  mission_type: "RACK_MOVE"
  
  path:
    type: "inlet_rotation"
    
applicable_rooms: [1, 2, 3, 4, 5]

debug:
  enabled: false
  log_conditions: true
EOF

# 4. 建立 Rack 旋轉出口流程
cat > /app/config/wcs/flows/rack_rotation_exit.yaml << 'EOF'
name: "Rack旋轉檢查-房間出口"
description: "當 Rack B面完成後，檢查是否需要旋轉處理 A面"
priority: 90
work_id: "220001"
enabled: true

trigger_conditions:
  - condition: "rack_at_location_exists"
    description: "房間出口位置有 Rack"
    parameters:
      location_type: "room_exit"
      
  - condition: "rack_side_completed" 
    description: "Rack B面已完成"
    parameters:
      side: "B"
      
  - condition: "rack_has_a_side_work"
    description: "Rack A面有待處理工作"
    parameters: {}
      
  - condition: "rack_needs_rotation_for_a_side"
    description: "Rack 需要旋轉以處理 A面"
    parameters:
      location_type: "room_exit"
      
  - condition: "no_active_task"
    description: "該位置無衝突任務"
    parameters:
      work_id: "220001"

action:
  type: "create_task"
  task_type: "rack_rotation"
  function: "rack_move"
  model: "KUKA400i"
  api: "submit_mission"
  mission_type: "RACK_MOVE"
  
  path:
    type: "exit_rotation"
    
applicable_rooms: [1, 2, 3, 4, 5]

debug:
  enabled: false
  log_conditions: true
EOF

# 5. 建立滿料架運輸流程
cat > /app/config/wcs/flows/full_rack_to_manual_area.yaml << 'EOF'
name: "滿料架到人工收料區-傳送箱出口"
description: "檢查傳送箱出口的滿料架並運輸到人工收料區"
priority: 80
work_id: "220001"
enabled: true

trigger_conditions:
  - condition: "transfer_exit_has_full_rack"
    description: "傳送箱出口有滿料架"
    parameters: {}
      
  - condition: "rack_is_full"
    description: "Rack 已滿載"
    parameters: {}
      
  - condition: "manual_collection_area_available"
    description: "人工收料區有可用空間"
    parameters: {}
      
  - condition: "no_active_task_to_destination"
    description: "目的地無衝突任務"
    parameters:
      destination_type: "manual_collection_area"
      work_id: "220001"
      
  - condition: "no_active_task_from_source"
    description: "來源地無衝突任務"
    parameters:
      source_type: "transfer_exit"
      work_id: "220001"

action:
  type: "create_task"
  task_type: "rack_transport"
  function: "rack_move"
  model: "KUKA400i"
  api: "submit_mission"
  mission_type: "RACK_MOVE"
  
  path:
    type: "transport_to_manual"
    
applicable_locations: [20001]  # 傳送箱出口位置

debug:
  enabled: false
  log_conditions: true
EOF
```

### 修改現有流程
```bash
# 使用 yq 工具修改配置
cd /app/config/wcs

# 修改優先級
yq '.priority = 90' flows/rack_rotation_inlet.yaml > flows/rack_rotation_inlet_updated.yaml
mv flows/rack_rotation_inlet_updated.yaml flows/rack_rotation_inlet.yaml

# 啟用調試模式
yq '.debug.enabled = true' flows/rack_rotation_inlet.yaml > flows/temp.yaml
mv flows/temp.yaml flows/rack_rotation_inlet.yaml

# 新增觸發條件
yq '.trigger_conditions += [{"condition": "new_condition", "description": "新條件", "parameters": {"key": "value"}}]' flows/rack_rotation_inlet.yaml > flows/temp.yaml
mv flows/temp.yaml flows/rack_rotation_inlet.yaml
```

## 🔍 除錯和診斷

### 配置驗證
```bash
# 驗證 YAML 語法
python3 -c "
import yaml
with open('/app/config/wcs/flows/rack_rotation_inlet.yaml', 'r') as f:
    data = yaml.safe_load(f)
    print('✅ YAML 語法正確')
    print(f'流程名稱: {data[\"name\"]}')
    print(f'觸發條件數量: {len(data[\"trigger_conditions\"])}')
"

# 測試解析器
python3 -c "
import sys
sys.path.append('/app/simple_wcs_ws/src/simple_wcs')
from simple_wcs.flow_parser import FlowParser

parser = FlowParser('/app/config/wcs/flows')
flows = parser.parse()

# 執行驗證
validation = parser.validate_flows(flows)
print('驗證結果:')
for category, messages in validation.items():
    if messages:
        print(f'  {category}: {messages}')
"
```

### 系統監控
```bash
# 監控 ROS 2 主題
ros2 topic list | grep simple_wcs
ros2 topic echo /simple_wcs/system_status
ros2 topic echo /simple_wcs/task_decisions

# 檢查節點狀態
ros2 node list | grep simple_wcs
ros2 node info /simple_wcs_engine

# 查看系統日誌
tail -f /tmp/simple_wcs.log
```

### 資料庫除錯
```bash
# 測試資料庫連接
python3 -c "
import sys
sys.path.append('/app/simple_wcs_ws/src/simple_wcs')
sys.path.append('/app/db_proxy_ws/src')
from simple_wcs.database_client import DatabaseClient

db = DatabaseClient()
print('✅ 資料庫連接成功')

# 測試查詢方法
result = db.rack_at_location_exists(1001)
print(f'位置 1001 是否有 Rack: {result}')

# 測試統計查詢
stats = db.get_rack_statistics()
print(f'Rack 統計: {stats}')
"
```

## 🛠️ 開發最佳實踐

### 程式碼開發
1. **模組化設計**: 每個功能獨立的類別和方法
2. **錯誤處理**: 完善的異常處理和日誌記錄
3. **型別提示**: 使用 Python 型別提示提高程式碼品質
4. **文檔字串**: 詳細的函數和類別文檔

### 配置管理
1. **檔案命名**: 使用描述性的檔案名稱
2. **優先級管理**: 合理設定業務流程優先級
3. **調試配置**: 開發階段啟用詳細日誌
4. **版本控制**: 配置檔案納入版本控制

### 測試策略
1. **單元測試**: 各組件獨立測試
2. **整合測試**: 完整系統流程測試
3. **配置測試**: YAML 解析和驗證測試
4. **效能測試**: 決策循環效能測試

## 📋 故障排除

### 常見問題和解決方案

#### 建置失敗
```bash
# 檢查依賴
pip3 list | grep -E "(sqlmodel|pydantic|sqlalchemy)"

# 重新安裝依賴
/opt/pyvenv_env/bin/pip3 install sqlmodel pydantic sqlalchemy

# 清理重建
cd /app/simple_wcs_ws
rm -rf build install log
colcon build --packages-select simple_wcs
```

#### ROS 2 環境問題
```bash
# 檢查 RMW 設定
echo $RMW_IMPLEMENTATION

# 重設環境
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

#### 資料庫連接問題
```bash
# 檢查 PostgreSQL 狀態
docker compose -f docker-compose.agvc.yml ps postgres_container

# 檢查網路連接
ping postgres_container

# 檢查 PYTHONPATH
echo $PYTHONPATH | grep db_proxy
```

#### 配置解析問題
```bash
# 檢查配置檔案權限
ls -la /app/config/wcs/flows/

# 檢查 YAML 語法
python3 -c "import yaml; yaml.safe_load(open('/app/config/wcs/flows/rack_rotation_inlet.yaml'))"

# 檢查配置目錄
ls -la /app/config/wcs/
```

## 🚀 效能最佳化

### 系統效能調整
1. **決策週期**: 根據業務需求調整 timer 間隔
2. **資料庫查詢**: 最佳化查詢邏輯和索引使用
3. **記憶體使用**: 監控系統記憶體使用情況
4. **ROS 2 通訊**: 調整 QoS 設定和主題設計

### 配置最佳化
1. **條件順序**: 將最常滿足的條件放在前面
2. **房間範圍**: 限制 `applicable_rooms` 減少不必要檢查
3. **優先級設定**: 合理設定優先級避免資源競爭
4. **調試選項**: 生產環境關閉詳細日誌

## 🔗 交叉引用
- Simple WCS 系統設計: @docs-ai/knowledge/system/simple-wcs-system.md
- ROS 2 開發: @docs-ai/operations/development/ros2-development.md
- 資料庫操作: @docs-ai/operations/development/database-operations.md
- 容器開發: @docs-ai/operations/development/docker-development.md
- 系統診斷: @docs-ai/operations/maintenance/system-diagnostics.md