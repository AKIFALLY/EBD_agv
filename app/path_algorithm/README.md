# 路徑演算法工作空間 (path_algorithm)

## 📋 基本資訊

**啟動狀態**: ✅ 自動載入 (容器啟動腳本中自動載入但不執行特定節點)
**運行環境**: 🚗🖥️ 共用 (AGV 車載系統 + AGVC 管理系統)
**主要功能**: A* 路徑規劃演算法和地圖處理
**依賴狀態**: 使用虛擬環境套件 (networkx)，提供路徑計算核心功能

## 📋 專案概述

路徑演算法工作空間提供 RosAGV 系統的核心路徑規劃功能，實現了基於 A* 演算法的最短路徑計算。該工作空間使用 NetworkX 圖論庫進行圖形建構和路徑搜尋，支援複雜的工廠地圖拓撲結構，並提供座標轉換、站點管理和路徑最佳化等功能。

此工作空間作為 AGV 導航的核心組件，被 agv_ws 工作空間廣泛使用。它採用標準的 A* 演算法實作，支援歐式距離啟發函數，能夠處理複雜的工廠地圖拓撲結構，並提供高效的路徑計算服務。工作空間還包含完整的站點管理系統和資料轉換工具，確保與 PLC 系統的完美整合。

## 🔗 依賴關係

### 虛擬環境套件依賴
- **networkx**: 圖論演算法庫，用於 A* 路徑搜尋和圖形處理 (安裝在 `/opt/pyvenv_env`)

### 系統套件依賴
- **Python 標準庫**: `os`, `math`, `struct`, `json`
- **yaml**: YAML 配置檔案解析

### 被依賴的工作空間
- **agv_ws**: 在 `write_path_state.py` 中使用 `AStarAlgorithm` 進行路徑計算

### 外部依賴
- **配置檔案**: `/app/config/path.yaml` (地圖資料檔案路徑)
- **站點配置**: `/app/config/stationID.yaml` (站點 ID 對應表)
- **地圖資料**: JSON 格式的工廠地圖檔案

## 🏗️ 專案結構

```
path_algorithm/
├── src/astar_algorithm/           # A* 演算法套件 (完整實作)
│   ├── astar_algorithm/
│   │   ├── astar_algorithm.py     # A* 演算法核心實作 (AStarAlgorithm 類別)
│   │   └── data_tool.py           # 資料轉換工具 (PLC 資料處理)
│   ├── package.xml                # ROS 2 套件配置
│   ├── setup.py                   # Python 套件設定
│   └── setup.cfg                  # 安裝配置
├── build/                         # 建置輸出目錄
├── install/                       # 安裝輸出目錄
├── log/                          # 建置日誌目錄
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
# 檢查虛擬環境是否正確設定
echo $PYTHONPATH
echo $VIRTUAL_ENV

# 檢查 networkx 安裝位置和版本
/opt/pyvenv_env/bin/python3 -c "
import networkx
print(f'NetworkX 版本: {networkx.__version__}')
print(f'安裝位置: {networkx.__file__}')
"

# 測試 networkx 基本功能
/opt/pyvenv_env/bin/python3 -c "
import networkx as nx
G = nx.DiGraph()
G.add_edge(1, 2, weight=1.0)
print(f'✅ NetworkX 功能正常，圖形節點數: {G.number_of_nodes()}')
"

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
# 載入 ROS 2 環境並建置
source /opt/ros/jazzy/setup.bash && source /opt/ws_rmw_zenoh/install/setup.bash && cd /app/path_algorithm && colcon build
source install/setup.bash

# 執行測試
colcon test
colcon test-result --verbose
```

### 2. 虛擬環境套件測試
```bash
# 測試 networkx 套件
/opt/pyvenv_env/bin/python3 -c "
import networkx as nx
print('✅ NetworkX 可用')
print(f'版本: {nx.__version__}')
print(f'位置: {nx.__file__}')
"

# 測試 A* 演算法功能
/opt/pyvenv_env/bin/python3 -c "
import networkx as nx
G = nx.DiGraph()
G.add_edge(1, 2, weight=1.0)
G.add_edge(2, 3, weight=1.5)
try:
    path = nx.astar_path(G, 1, 3, weight='weight')
    print(f'✅ A* 演算法測試成功: {path}')
except Exception as e:
    print(f'❌ A* 演算法測試失敗: {e}')
"
```

### 3. 系統套件測試
```bash
# 測試 Python 標準庫
python3 -c "
import os, math, struct, json
print('✅ Python 標準庫可用')
"

# 測試 YAML 套件
python3 -c "
import yaml
print('✅ YAML 套件可用')
print(f'版本: {yaml.__version__}')
"
```

### 4. 模組功能測試
```bash
# 測試 A* 演算法模組載入
python3 -c "
from astar_algorithm.astar_algorithm import AStarAlgorithm
from astar_algorithm.data_tool import int32_to_2_words, words_to_int32
print('✅ 路徑演算法模組載入成功')
"

# 測試資料轉換工具
python3 -c "
from astar_algorithm.data_tool import int32_to_2_words, words_to_int32
low, high = int32_to_2_words(123456)
result = words_to_int32(high, low)
assert result == 123456
print(f'✅ 資料轉換工具測試通過: {123456} -> [{low}, {high}] -> {result}')
"
```

### 5. 演算法功能測試
```bash
# 測試 A* 演算法核心功能 (需要配置檔案)
cd /app/path_algorithm/src/astar_algorithm/astar_algorithm
python3 astar_algorithm.py

# 測試圖形建構
python3 -c "
import sys
sys.path.append('/app/path_algorithm/src/astar_algorithm')
from astar_algorithm import AStarAlgorithm

try:
    astar = AStarAlgorithm()
    print(f'✅ 圖形建構成功')
    print(f'節點數: {astar.graph.number_of_nodes()}')
    print(f'邊數: {astar.graph.number_of_edges()}')
except Exception as e:
    print(f'❌ 圖形建構失敗: {e}')
"
```

### 6. 路徑計算測試 (需要有效地圖資料)
```bash
# 測試路徑計算功能
python3 -c "
import sys
sys.path.append('/app/path_algorithm/src/astar_algorithm')
from astar_algorithm import AStarAlgorithm

try:
    # 假設有節點 1 和 2
    astar = AStarAlgorithm(1, 2)
    path = astar.run()
    print(f'✅ 路徑計算成功: {path}')
except Exception as e:
    print(f'⚠️ 路徑計算測試: {e} (可能需要有效的地圖資料)')
"

# 測試站點管理功能
python3 -c "
import sys
sys.path.append('/app/path_algorithm/src/astar_algorithm')
from astar_algorithm import AStarAlgorithm

try:
    site_map = AStarAlgorithm.load_site_map()
    print(f'✅ 站點管理功能正常，載入 {len(site_map)} 個站點')
except Exception as e:
    print(f'⚠️ 站點管理測試: {e} (可能需要 stationID.yaml)')
## 🔧 故障排除

### 1. 模組載入問題
**症狀**: `ModuleNotFoundError: No module named 'astar_algorithm'`
**解決方法**:
```bash
# 檢查工作空間是否正確建置
cd /app/path_algorithm
colcon build

# 確認環境已載入
source install/setup.bash

# 檢查 Python 路徑
python3 -c "import sys; print('\\n'.join(sys.path))"

# 手動添加路徑 (臨時解決方案)
export PYTHONPATH=/app/path_algorithm/src/astar_algorithm:$PYTHONPATH
```

### 2. NetworkX 套件問題
**症狀**: `ModuleNotFoundError: No module named 'networkx'`
**解決方法**:
```bash
# 檢查虛擬環境是否正確設定
echo $PYTHONPATH
echo $VIRTUAL_ENV

# 檢查 networkx 是否安裝
/opt/pyvenv_env/bin/python3 -c "import networkx; print('NetworkX 可用')"

# 重新安裝 networkx
/opt/pyvenv_env/bin/pip3 install networkx

# 檢查安裝位置
/opt/pyvenv_env/bin/python3 -c "import networkx; print(networkx.__file__)"
```

### 3. 配置檔案問題
**症狀**: `FileNotFoundError` 或配置載入失敗
**解決方法**:
```bash
# 檢查配置檔案是否存在
ls -la /app/config/path.yaml
ls -la /app/config/stationID.yaml

# 檢查 JSON 地圖檔案
cat /app/config/path.yaml | grep file_path
ls -la /app/config/*.json

# 測試配置檔案格式
python3 -c "
import yaml
try:
    with open('/app/config/path.yaml', 'r') as f:
        config = yaml.safe_load(f)
    print('✅ path.yaml 格式正確')
except Exception as e:
    print(f'❌ path.yaml 格式錯誤: {e}')
"
```

### 4. 路徑計算失敗
**症狀**: `ValueError: 找不到路徑` 或 `NetworkXNoPath`
**解決方法**:
```bash
# 檢查圖形連通性
python3 -c "
import sys
sys.path.append('/app/path_algorithm/src/astar_algorithm')
from astar_algorithm import AStarAlgorithm

astar = AStarAlgorithm()
print(f'節點數: {astar.graph.number_of_nodes()}')
print(f'邊數: {astar.graph.number_of_edges()}')

# 檢查特定節點是否存在
start_node = 1
end_node = 2
if start_node in astar.graph and end_node in astar.graph:
    print(f'✅ 節點 {start_node} 和 {end_node} 都存在')
else:
    print(f'❌ 節點不存在: {start_node} in graph: {start_node in astar.graph}, {end_node} in graph: {end_node in astar.graph}')
"

# 檢查圖形連通性
python3 -c "
import networkx as nx
import sys
sys.path.append('/app/path_algorithm/src/astar_algorithm')
from astar_algorithm import AStarAlgorithm

astar = AStarAlgorithm()
if nx.is_weakly_connected(astar.graph):
    print('✅ 圖形是弱連通的')
else:
    print('❌ 圖形不是連通的，可能存在孤立節點')
    components = list(nx.weakly_connected_components(astar.graph))
    print(f'連通分量數: {len(components)}')
"
```

### 5. 記憶體或效能問題
**症狀**: 路徑計算過慢或記憶體不足
**解決方法**:
```bash
# 檢查圖形大小
python3 -c "
import sys
sys.path.append('/app/path_algorithm/src/astar_algorithm')
from astar_algorithm import AStarAlgorithm

astar = AStarAlgorithm()
nodes = astar.graph.number_of_nodes()
edges = astar.graph.number_of_edges()
avg_degree = edges / nodes if nodes > 0 else 0

print(f'節點數: {nodes}')
print(f'邊數: {edges}')
print(f'平均度數: {avg_degree:.2f}')

if nodes > 10000:
    print('⚠️ 圖形較大，可能影響效能')
if avg_degree > 10:
    print('⚠️ 平均度數較高，可能影響記憶體使用')
"

# 監控記憶體使用
top -p $(pgrep -f python3) -n 1

# 效能測試
python3 -c "
import time
import sys
sys.path.append('/app/path_algorithm/src/astar_algorithm')
from astar_algorithm import AStarAlgorithm

start_time = time.time()
astar = AStarAlgorithm()
init_time = time.time() - start_time
print(f'圖形初始化耗時: {init_time:.3f} 秒')
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
  Unloading01: 15
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
  },
  {
    "TagNo": 2,
    "Tag_X": 1500,
    "Tag_Y": 2000,
    "CanToMoveSet": [
      {"CanToMoveTag": 1, "PGV": 1, "Act": [1,2,3], "Speed": [100,200,300]},
      {"CanToMoveTag": 3, "PGV": 1, "Act": [1,2,3], "Speed": [100,200,300]}
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

# 資料轉換參數
WORD_SIZE = 2                       # PLC Word 大小 (bytes)
ENDIAN = "little"                   # 位元組序 (小端序)
## 🔗 相關文檔

- **agv_ws**: AGV 核心系統，在 `write_path_state.py` 中使用本工作空間的 `AStarAlgorithm` 進行路徑計算
- **NetworkX 官方文檔**: [NetworkX Documentation](https://networkx.org/documentation/stable/)
- **A* 演算法原理**: [A* Search Algorithm](https://en.wikipedia.org/wiki/A*_search_algorithm)
- **ROS 2 Jazzy 文檔**: [ROS 2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)

## 📋 ToDo 清單

### 🔴 高優先級 (緊急)
- [ ] 完善錯誤處理機制，提高演算法穩定性
- [ ] 新增路徑驗證功能，確保計算結果正確性
- [ ] 最佳化大型地圖的載入和處理效能

### 🟡 中優先級 (重要)
- [ ] 新增多種啟發函數支援 (曼哈頓距離、切比雪夫距離)
- [ ] 實作動態障礙物避障功能
- [ ] 新增路徑平滑化演算法
- [ ] 完善單元測試和效能測試覆蓋率

### 🟢 低優先級 (改善)
- [ ] 新增路徑視覺化工具
- [ ] 支援多目標路徑規劃
- [ ] 新增路徑快取機制，提高重複查詢效能
- [ ] 實作路徑品質評估指標

### 🔧 技術債務
- [ ] 重構資料載入邏輯，提高程式碼可讀性
- [ ] 統一錯誤訊息格式和多語言支援
- [ ] 改善程式碼文檔和演算法說明完整性

### 📊 完成度追蹤
- ✅ A* 演算法核心實作 (100%)
- ✅ 圖形建構和管理 (100%)
- ✅ 座標轉換功能 (100%)
- ✅ 資料轉換工具 (100%)
- ✅ 配置檔案支援 (100%)
- ⚠️ 錯誤處理機制 (70% - 需要改善)
- ⚠️ 效能最佳化 (60% - 基礎實作)
- ❌ 動態障礙物支援 (0% - 未開始)

### 🎯 里程碑
- **v1.0.0**: ✅ 基礎路徑規劃功能完成 (當前版本)
- **v1.1.0**: 🚧 錯誤處理和效能改善
- **v2.0.0**: 📋 動態障礙物和多目標支援

### 🏆 重要成就
- ✅ 成功整合到 RosAGV 系統
- ✅ 提供高效的 A* 路徑規劃
- ✅ 支援複雜工廠地圖拓撲
- ✅ 實現完整的座標轉換系統
