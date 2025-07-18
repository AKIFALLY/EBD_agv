# 路徑演算法工作空間 (path_algorithm)

## 📋 基本資訊

**啟動狀態**: ✅ 實際啟動 (容器啟動時自動載入)  
**運行環境**: 🚗🖥️ 共用 (AGV 車載系統 + AGVC 管理系統)  
**主要功能**: A* 路徑規劃演算法和地圖處理  
**依賴狀態**: 使用虛擬環境套件 (networkx)，提供路徑計算核心功能

## 📋 專案概述

路徑演算法工作空間提供 RosAGV 系統的核心路徑規劃功能，實現了基於 A* 演算法的最短路徑計算。該工作空間使用 NetworkX 圖論庫進行圖形建構和路徑搜尋，支援複雜的工廠地圖拓撲結構，並提供座標轉換、站點管理和路徑最佳化等功能。作為 AGV 導航的核心組件，它被 agv_ws 工作空間廣泛使用。

## 🔗 依賴關係

### 虛擬環境套件依賴
- **networkx**: 圖論演算法庫，用於 A* 路徑搜尋和圖形處理

### 系統套件依賴
- **yaml**: YAML 配置檔案解析
- **json**: JSON 資料檔案處理

### 被依賴的工作空間
- **agv_ws**: 在 `write_path_state.py` 中使用 `AStarAlgorithm` 進行路徑計算
- **外部系統**: 任何需要路徑規劃功能的模組

### 外部依賴
- **Python 標準庫**: `os`, `math`, `struct`
- **配置檔案**: `/app/config/path.yaml`, `/app/config/stationID.yaml`

## 🏗️ 專案結構

```
path_algorithm/
├── src/astar_algorithm/           # A* 演算法套件
│   ├── astar_algorithm/
│   │   ├── astar_algorithm.py     # A* 演算法核心實作 (使用虛擬環境 networkx)
│   │   └── data_tool.py           # 資料轉換工具
│   ├── package.xml                # 套件配置
│   ├── setup.py                   # Python 套件設定
│   └── setup.cfg                  # 安裝配置
└── README.md                      # 本檔案
```

## ⚙️ 主要功能

### 1. A* 路徑規劃演算法
**AStarAlgorithm 核心類別**:
- **圖形建構**: 從 JSON 資料建立有向圖 (NetworkX DiGraph)
- **路徑搜尋**: 使用 NetworkX 內建 A* 演算法進行最短路徑計算
- **啟發函數**: 歐式距離啟發函數，提高搜尋效率
- **權重計算**: 基於座標的歐式距離權重

### 2. 地圖資料處理
**資料轉換功能**:
- **JSON 解析**: 讀取工廠地圖 JSON 資料檔案
- **格式轉換**: 將原始 Tag 資料轉換為圖形友好格式
- **鄰接關係**: 處理 `CanToMoveSet` 建立節點鄰接關係
- **座標管理**: Tag 座標 (Tag_X, Tag_Y) 處理和查詢

### 3. 站點管理系統
**站點對應功能**:
- **站點映射**: 站點 ID 與 Tag 編號的雙向對應
- **配置載入**: 從 `stationID.yaml` 載入站點配置
- **查詢介面**: 提供站點名稱與 Tag 編號的查詢方法
- **快取機制**: 類別層級共享的站點對應表

### 4. 資料轉換工具
**PLC 資料處理**:
- **32-bit 整數分割**: 將 32-bit 整數分割為兩個 16-bit 字串
- **位元組序處理**: 小端序資料封裝和解封裝
- **PLC 相容性**: 符合 PLC 記憶體格式要求

## 🔧 核心 API

### AStarAlgorithm 類別
```python
from astar_algorithm.astar_algorithm import AStarAlgorithm

# 初始化演算法 (指定起點和終點)
astar = AStarAlgorithm(start_node=1, end_node=10)

# 執行路徑計算
try:
    path = astar.run()
    print(f"最短路徑: {path}")  # 例如: [1, 3, 7, 10]
except ValueError as e:
    print(f"路徑計算失敗: {e}")

# 取得節點座標
x, y = astar.getXY(tag_id=5)
print(f"Tag 5 座標: X={x}, Y={y}")

# 存取圖形資料
print(f"圖形節點數: {astar.graph.number_of_nodes()}")
print(f"圖形邊數: {astar.graph.number_of_edges()}")
```

### 站點管理 API
```python
# 站點名稱 → Tag 編號
tag_id = AStarAlgorithm.get_tag_by_station("Soaking01")
print(f"Soaking01 對應的 Tag: {tag_id}")

# Tag 編號 → 站點名稱
station_name = AStarAlgorithm.get_station_by_tag(4)
print(f"Tag 4 對應的站點: {station_name}")

# 載入站點對應表
site_map = AStarAlgorithm.load_site_map()
print(f"所有站點: {list(site_map.keys())}")
```

### 資料轉換工具 API
```python
from astar_algorithm.data_tool import int32_to_2_words, words_to_int32

# 32-bit 整數轉換為兩個 16-bit 字串
low_str, high_str = int32_to_2_words(123456)
print(f"123456 → 低位: {low_str}, 高位: {high_str}")

# 兩個 16-bit 字串合併為 32-bit 整數
original = words_to_int32(high_str, low_str)
print(f"合併結果: {original}")
```

## 🚀 使用方法

### 1. 建置工作空間
```bash
# 載入 ROS 2 環境並建置
source /opt/ros/jazzy/setup.bash && source /opt/ws_rmw_zenoh/install/setup.bash && cd /app/path_algorithm && colcon build
source install/setup.bash
```

### 2. 虛擬環境套件檢查
```bash
# networkx 已透過 Docker 建置時安裝在虛擬環境中
# 檢查 networkx 安裝位置
/opt/pyvenv_env/bin/python3 -c "import networkx; print(networkx.__file__)"

# 檢查 networkx 版本
/opt/pyvenv_env/bin/python3 -c "import networkx; print(networkx.__version__)"

# 如需重新安裝 networkx
/opt/pyvenv_env/bin/pip3 install networkx
```

### 3. 配置檔案設定
```bash
# 檢查路徑配置檔案
cat /app/config/path.yaml

# 檢查站點配置檔案
cat /app/config/stationID.yaml

# 檢查地圖資料檔案 (根據 path.yaml 中的設定)
ls /app/config/*.json
```

### 4. 測試演算法功能
```bash
# 執行演算法測試
cd /app/path_algorithm/src/astar_algorithm/astar_algorithm
python3 astar_algorithm.py

# 檢查圖形建構
python3 -c "
from astar_algorithm import AStarAlgorithm
astar = AStarAlgorithm()
print(f'節點數: {astar.graph.number_of_nodes()}')
print(f'邊數: {astar.graph.number_of_edges()}')
"
```

### 5. 在 AGV 系統中使用
```python
# 在 agv_ws 中的使用範例 (write_path_state.py)
from astar_algorithm.astar_algorithm import AStarAlgorithm

# 初始化並計算路徑
astar = AStarAlgorithm(start_tag, target_tag)
path = astar.run()

# 取得路徑資料用於 PLC 寫入
source_data = astar.source_data
for i, tag_id in enumerate(path):
    x, y = astar.getXY(tag_id)
    # 處理路徑資料...
```

## 🧪 測試方法

### 1. 建置和測試
```bash
# 建置工作空間
cd /app/path_algorithm
colcon build

# 執行測試
colcon test
colcon test-result --verbose
```

### 2. 演算法功能測試
```bash
# 測試 A* 演算法核心功能
cd /app/path_algorithm/src/astar_algorithm/astar_algorithm
python3 astar_algorithm.py

# 測試資料轉換工具
python3 -c "
from data_tool import int32_to_2_words, words_to_int32
low, high = int32_to_2_words(123456)
print(f'分割結果: {low}, {high}')
result = words_to_int32(high, low)
print(f'合併結果: {result}')
"
```

### 3. 圖形建構測試
```python
# 測試圖形建構和路徑計算
import sys
sys.path.append('/app/path_algorithm/src/astar_algorithm')

from astar_algorithm import AStarAlgorithm

# 初始化演算法
astar = AStarAlgorithm()

# 檢查圖形結構
print(f"✅ 圖形節點數: {astar.graph.number_of_nodes()}")
print(f"✅ 圖形邊數: {astar.graph.number_of_edges()}")

# 檢查轉換後的資料
print(f"✅ 轉換資料節點數: {len(astar.converted_data)}")

# 測試座標查詢
if 2 in astar.converted_data:
    x, y = astar.getXY(2)
    print(f"✅ Tag 2 座標: X={x}, Y={y}")
```

### 4. 配置檔案測試
```bash
# 測試 YAML 配置檔案
python3 -c "
import yaml
with open('/app/config/path.yaml', 'r') as f:
    config = yaml.safe_load(f)
    print('✅ path.yaml 載入成功')
    print(f'JSON 檔案路徑: {config.get(\"path_data_file\", {}).get(\"file_path\")}')
"

# 測試站點配置檔案
python3 -c "
from astar_algorithm.astar_algorithm import AStarAlgorithm
site_map = AStarAlgorithm.load_site_map()
print(f'✅ 站點配置載入成功，共 {len(site_map)} 個站點')
"
```

### 5. 路徑計算測試
```python
# 測試完整路徑計算流程
from astar_algorithm.astar_algorithm import AStarAlgorithm

# 測試路徑計算 (需要有效的起點和終點)
try:
    astar = AStarAlgorithm(start_node=1, end_node=5)
    path = astar.run()
    print(f"✅ 路徑計算成功: {path}")

    # 計算路徑總距離
    total_distance = 0
    for i in range(len(path) - 1):
        edge_data = astar.graph.get_edge_data(path[i], path[i+1])
        if edge_data:
            total_distance += edge_data['weight']
    print(f"✅ 路徑總距離: {total_distance:.2f}")

except ValueError as e:
    print(f"❌ 路徑計算失敗: {e}")
except Exception as e:
    print(f"❌ 測試過程發生錯誤: {e}")
```

## 🔧 故障排除

### 常見問題

#### 1. networkx 模組找不到
**症狀**: `ModuleNotFoundError: No module named 'networkx'`
**解決方法**:
```bash
# 檢查虛擬環境套件
/opt/pyvenv_env/bin/pip3 list | grep networkx

# 重新安裝 networkx
/opt/pyvenv_env/bin/pip3 uninstall networkx
/opt/pyvenv_env/bin/pip3 install networkx

# 檢查 networkx 安裝位置
python3 -c "import networkx; print(networkx.__file__)"
```

#### 2. 配置檔案找不到
**症狀**: `FileNotFoundError: ❌ YAML 設定檔不存在` 或 `❌ 指定的 JSON 檔案不存在`
**解決方法**:
```bash
# 檢查配置檔案是否存在
ls -la /app/config/path.yaml
ls -la /app/config/stationID.yaml

# 檢查 JSON 資料檔案
cat /app/config/path.yaml | grep file_path
ls -la /app/config/*.json

# 如果檔案不存在，檢查掛載點
mount | grep /app/config
```

#### 3. 路徑計算失敗
**症狀**: `❌ 找不到從 X 到 Y 的路徑` 或 `NetworkXNoPath`
**解決方法**:
```bash
# 檢查圖形連通性
python3 -c "
from astar_algorithm.astar_algorithm import AStarAlgorithm
astar = AStarAlgorithm()
import networkx as nx
print(f'圖形是否連通: {nx.is_weakly_connected(astar.graph)}')
print(f'連通分量數: {nx.number_weakly_connected_components(astar.graph)}')
"

# 檢查節點是否存在
python3 -c "
from astar_algorithm.astar_algorithm import AStarAlgorithm
astar = AStarAlgorithm()
start, end = 1, 5  # 替換為實際的起點和終點
print(f'起點 {start} 存在: {start in astar.graph}')
print(f'終點 {end} 存在: {end in astar.graph}')
"
```

#### 4. 記憶體或效能問題
**症狀**: 路徑計算過慢或記憶體不足
**解決方法**:
```bash
# 檢查圖形大小
python3 -c "
from astar_algorithm.astar_algorithm import AStarAlgorithm
astar = AStarAlgorithm()
print(f'節點數: {astar.graph.number_of_nodes()}')
print(f'邊數: {astar.graph.number_of_edges()}')
print(f'平均度數: {astar.graph.number_of_edges() / astar.graph.number_of_nodes():.2f}')
"

# 監控記憶體使用
top -p $(pgrep -f python3)

# 優化建議：考慮使用更高效的圖形表示或限制搜尋範圍
```

### 除錯工具
```bash
# 檢查 Python 路徑
python3 -c "import sys; print('\\n'.join(sys.path))"

# 檢查虛擬環境狀態
echo $PYTHONPATH
echo $VIRTUAL_ENV

# 檢查套件版本
python3 -c "
import networkx, yaml, json
print(f'NetworkX: {networkx.__version__}')
print(f'Python: {sys.version}')
"

# 檢查檔案權限
ls -la /app/config/
ls -la /app/path_algorithm/src/astar_algorithm/astar_algorithm/
```

### 日誌和診斷
```bash
# 啟用詳細日誌
export PYTHONPATH=/app/path_algorithm/src/astar_algorithm:$PYTHONPATH

# 檢查演算法執行日誌
python3 -c "
import logging
logging.basicConfig(level=logging.DEBUG)
from astar_algorithm.astar_algorithm import AStarAlgorithm
astar = AStarAlgorithm()
"

# 檢查系統資源
ps aux | grep python3
free -h
df -h /app
```

## ⚙️ 配置說明

### path.yaml 配置
```yaml
# /app/config/path.yaml
path_data_file:
  file_path: "/app/config/20250616_path.json"  # JSON 地圖資料檔案路徑
```

### stationID.yaml 配置
```yaml
# /app/config/stationID.yaml
StationID:
  Soaking01: 4      # 站點名稱對應 Tag 編號
  Soaking02: 5
  Loading01: 10
  # ... 更多站點配置
```

### JSON 地圖資料格式
```json
[
  {
    "TagNo": 1,
    "Tag_X": 1000,
    "Tag_Y": 2000,
    "CanToMoveSet": [
      {"CanToMoveTag": 2, "PGV": 1, "Act": [1,2,3], "Speed": [100,200,300]},
      {"CanToMoveTag": 3, "PGV": 2, "Act": [1,2,3], "Speed": [150,250,350]}
    ]
  }
]
```

### 演算法參數配置
```python
# A* 演算法參數
HEURISTIC_FUNCTION = "euclidean"    # 啟發函數類型
WEIGHT_ATTRIBUTE = "weight"         # 邊權重屬性名稱
GRAPH_TYPE = "DiGraph"              # 有向圖類型
COORDINATE_SCALE = 1.0              # 座標縮放因子
```

## 🔗 相關文檔

- **agv_ws**: AGV 核心系統，在 write_path_state.py 中使用本工作空間進行路徑計算
- **NetworkX 官方文檔**: [NetworkX Documentation](https://networkx.org/documentation/stable/)
- **A* 演算法**: [A* Search Algorithm](https://en.wikipedia.org/wiki/A*_search_algorithm)
- **圖論基礎**: [Graph Theory Concepts](https://networkx.org/documentation/stable/tutorial.html)
