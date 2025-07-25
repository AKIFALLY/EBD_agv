# CLAUDE.md

## 系統概述
基於NetworkX的A*路徑規劃工具，專為工業AGV系統設計，使用標籤(Tag)和站點(Station)進行路徑規劃。

**🗺️ 圖形架構**: YAML/JSON配置 → NetworkX有向圖 → A*路徑搜尋

## 核心架構
```
path_algorithm/
└── astar_algorithm/              # A*演算法核心
    ├── astar_algorithm.py        # 主要A*實現
    ├── data_tool.py             # 數據轉換工具
    └── __init__.py
```

## 主要組件

### 1. AStarAlgorithm類別 (astar_algorithm.py)
**NetworkX基礎A*路徑規劃器**，處理工業環境中的標籤路徑規劃:
```python
class AStarAlgorithm:
    def __init__(self, start_node=None, end_node=None):
        # 步驟1: 讀取JSON路徑數據
        self.source_data = self.load_path_test_json()
        
        # 步驟2: 轉換為圖形友好格式
        self.converted_data = self.convert_tag_data_to_graph_format(self.source_data)
        
        # 步驟3: 建立NetworkX有向圖
        self.graph = self.build_graph_from_converted_data(self.converted_data)
```

**配置檔案載入**:
```python
@classmethod
def load_path_test_json(cls, config_path="/app/config/path.yaml"):
    """從YAML配置載入JSON路徑數據"""
    with open(config_path, 'r', encoding='utf-8') as yaml_file:
        config = yaml.safe_load(yaml_file)
    
    file_path = config.get("path_data_file", {}).get("file_path", None)
    with open(file_path, 'r', encoding='utf-8') as file:
        data = json.load(file)
        return data
```

**站點映射管理**:
```python
@classmethod
def load_site_map(cls, path="/app/config/stationID.yaml"):
    """載入站點ID對應表"""
    with open(path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f)
        cls._site_map = data.get("StationID", {})
    return cls._site_map

@classmethod
def get_tag_by_station(cls, station_id):
    """根據站點ID查詢對應的Tag"""
    site_map = cls.load_site_map()
    return site_map.get(station_id)

@classmethod  
def get_station_by_tag(cls, tag):
    """根據Tag查詢對應的站點ID"""
    site_map = cls.load_site_map()
    for site_id, t in site_map.items():
        if t == tag:
            return site_id
    return None
```

### 2. 圖形構建和轉換
**數據格式轉換**:
```python
def convert_tag_data_to_graph_format(self, tag_data: list) -> dict:
    """將原始Tag數據轉換為圖形格式"""
    converted = {}
    for entry in tag_data:
        tag_no = entry["TagNo"]
        x = entry["Tag_X"]
        y = entry["Tag_Y"]
        neighbors = [
            item["CanToMoveTag"]
            for item in entry.get("CanToMoveSet", [])
            if item["CanToMoveTag"] != 0
        ]
        converted[tag_no] = {
            "x": x,
            "y": y,
            "neighbors": neighbors
        }
    return converted
```

**NetworkX圖形建立**:
```python
def build_graph_from_converted_data(self, converted: dict) -> nx.DiGraph:
    """建立NetworkX有向圖"""
    G = nx.DiGraph()
    
    # 添加節點和位置信息
    for tag_id, data in converted.items():
        G.add_node(tag_id, pos=(data["x"], data["y"]))
    
    # 添加邊和權重(歐式距離)
    for to_tag, data in converted.items():
        for from_tag in data["neighbors"]:
            if from_tag in converted:
                x1, y1 = converted[from_tag]["x"], converted[from_tag]["y"]
                x2, y2 = data["x"], data["y"]
                dist = math.hypot(x2 - x1, y2 - y1)
                G.add_edge(from_tag, to_tag, weight=dist)
    return G
```

### 3. A*路徑搜尋
**啟發式函數**:
```python
def heuristic(self, u, v):
    """歐式距離啟發函數"""
    ux, uy = self.graph.nodes[u]['pos']
    vx, vy = self.graph.nodes[v]['pos']
    return math.hypot(ux - vx, uy - vy)
```

**路徑規劃執行**:
```python
def run(self):
    """使用NetworkX內建A*演算法進行路徑規劃"""
    try:
        path = nx.astar_path(
            self.graph,              # 有向圖
            self.start_node,         # 起始點
            self.end_node,           # 結束點
            heuristic=self.heuristic, # 啟發函數
            weight="weight"          # 邊權重
        )
        return path
    except nx.NetworkXNoPath:
        raise ValueError(f"❌ 找不到從 {self.start_node} 到 {self.end_node} 的路徑")
```

### 4. 輔助功能
**座標查詢**:
```python
def getXY(self, tag_id):
    """獲取指定Tag的X,Y座標"""
    if tag_id in self.converted_data:
        x = self.converted_data[tag_id]['x']
        y = self.converted_data[tag_id]['y']
        return x, y
```

**數據轉換工具 (data_tool.py)**:
```python
def int32_to_2_words(value):
    """將32位整數轉為兩個16位字串"""
    packed = struct.pack('<i', value)
    low_word, high_word = struct.unpack('<HH', packed)
    return [str(low_word), str(high_word)]

def words_to_int32(high_str, low_str):
    """將兩個16位字串合併為32位整數"""
    high = int(high_str)
    low = int(low_str)
    packed = struct.pack('<HH', low, high)
    return struct.unpack('<i', packed)[0]
```

## 配置檔案格式

### 路徑配置 (path.yaml)
```yaml
path_data_file:
  file_path: "/app/config/path_data.json"
```

### 站點映射 (stationID.yaml)
```yaml
StationID:
  Soaking01: 4
  Soaking02: 8
  LoadStation01: 12
  UnloadStation01: 16
  ChargeStation01: 20
```

### 路徑數據 (JSON格式)
```json
[
  {
    "TagNo": 1,
    "Tag_X": 1000,
    "Tag_Y": 2000,
    "Station": 0,
    "CanToMoveSet": [
      {
        "CanToMoveTag": 2,
        "PGV": 100,
        "Act": [1, 2],
        "Speed": [50, 60],
        "SHIFT": [0, 0, 0],
        "Inposition": [true, false],
        "SafeSensorSetting": [1, 2, 3]
      }
    ]
  }
]
```

## 開發指令

### 環境設定 (容器內執行)
```bash
# AGV或AGVC容器內
source /app/setup.bash && all_source  # 自動檢測環境
cd /app/path_algorithm
```

### 構建與測試
```bash
build_ws path_algorithm
```

### 程式執行 (容器內)
```bash
# 注意：setup.py未定義entry_points，需直接執行
cd /app/path_algorithm/src/astar_algorithm/astar_algorithm
python3 astar_algorithm.py
```

## 使用範例

### 1. 基本路徑規劃
```python
from astar_algorithm.astar_algorithm import AStarAlgorithm

# 使用Tag ID進行路徑規劃
start_tag = 1
end_tag = 10
astar = AStarAlgorithm(start_tag, end_tag)

try:
    path = astar.run()
    print(f"路徑: {path}")
    # 輸出範例: [1, 3, 7, 10]
except ValueError as e:
    print(f"路徑規劃失敗: {e}")
```

### 2. 使用站點名稱規劃
```python
# 使用站點名稱進行路徑規劃
start_station = "Soaking01"
end_station = "LoadStation01"

start_tag = AStarAlgorithm.get_tag_by_station(start_station)
end_tag = AStarAlgorithm.get_tag_by_station(end_station)

if start_tag and end_tag:
    astar = AStarAlgorithm(start_tag, end_tag)
    path = astar.run()
    print(f"從 {start_station} 到 {end_station} 的路徑: {path}")
```

### 3. 獲取路徑座標
```python
astar = AStarAlgorithm()

# 獲取路徑中每個點的座標
path = [1, 3, 7, 10]
coordinates = []

for tag in path:
    x, y = astar.getXY(tag)
    coordinates.append((x, y))
    print(f"Tag {tag}: X={x}, Y={y}")

print(f"路徑座標: {coordinates}")
```

### 4. 站點和Tag互相查詢
```python
# 查詢站點對應的Tag
tag = AStarAlgorithm.get_tag_by_station("Soaking01")
print(f"Soaking01 對應Tag: {tag}")

# 查詢Tag對應的站點
station = AStarAlgorithm.get_station_by_tag(4)
print(f"Tag 4 對應站點: {station}")
```

## 數據結構說明

### Tag數據結構
- **TagNo**: 標籤編號 (整數)
- **Tag_X, Tag_Y**: 標籤座標 (整數，通常以mm為單位)
- **Station**: 站點編號 (0表示非站點)
- **CanToMoveSet**: 可移動到的標籤集合，包含:
  - **CanToMoveTag**: 目標標籤
  - **PGV**: 程序控制值
  - **Act**: 動作序列
  - **Speed**: 速度設定
  - **SHIFT**: 偏移設定
  - **Inposition**: 定位狀態
  - **SafeSensorSetting**: 安全感測器設定

### 圖形特性
- **有向圖**: 支援單向通行路徑
- **權重邊**: 使用歐式距離作為權重
- **節點屬性**: 每個節點包含(x,y)座標信息
- **鄰接關係**: 基於CanToMoveSet定義

## 故障排除

### 常見問題
1. **配置檔案未找到**: 檢查YAML和JSON檔案路徑
   ```bash
   ls -la /app/config/path.yaml
   ls -la /app/config/stationID.yaml
   ```

2. **找不到路徑**: 檢查起終點Tag是否存在且連通
   ```python
   # 檢查Tag是否存在於圖中
   if start_tag not in astar.graph:
       print(f"起始Tag {start_tag} 不存在")
   ```

3. **JSON數據格式錯誤**: 驗證JSON檔案格式
   ```bash
   python3 -m json.tool /app/config/path_data.json
   ```

4. **站點映射錯誤**: 檢查stationID.yaml格式
   ```python
   site_map = AStarAlgorithm.load_site_map()
   print(site_map)
   ```

### 調試工具
```python
# 檢查圖形結構
print(f"節點數量: {astar.graph.number_of_nodes()}")
print(f"邊數量: {astar.graph.number_of_edges()}")

# 檢查特定節點的鄰居
neighbors = list(astar.graph.neighbors(tag_id))
print(f"Tag {tag_id} 的鄰居: {neighbors}")

# 檢查節點座標
pos = astar.graph.nodes[tag_id]['pos']
print(f"Tag {tag_id} 座標: {pos}")
```

### 性能考量
- **圖形大小**: NetworkX適合中等規模圖形 (< 10000節點)
- **記憶體使用**: 約每1000節點需要10-50MB記憶體
- **搜尋速度**: 典型路徑長度<100步時，搜尋時間<1ms

## 系統整合

### 在RosAGV系統中的角色
```
配置檔案 (YAML/JSON)
    ↓ 數據載入
AStarAlgorithm (NetworkX圖形)
    ↓ 路徑規劃
AGV控制系統 (路徑執行)
```

### 擴展可能性
- **ROS 2整合**: 可封裝為ROS 2服務節點
- **動態更新**: 支援圖形動態修改
- **多目標規劃**: 擴展為多點順序訪問
- **約束規劃**: 添加時間窗、容量等約束

## 重要提醒
- path_algorithm基於NetworkX提供純路徑規劃功能
- 適用於工業AGV系統的標籤導航環境
- 當前版本需要預先定義的靜態圖形結構
- 支援AGV和AGVC雙環境使用
- 所有操作需在對應容器內執行
- 依賴準確的配置檔案和圖形數據