# path_algorithm CLAUDE.md

## 模組概述
A*路徑規劃演算法實現，提供高效能的路徑規劃與導航功能，支援AGV與AGVC雙環境使用

## 專案結構
```
src/
└── path_algorithm/     # A*路徑規劃核心
    ├── astar/          # A*演算法實現
    ├── map_manager/    # 地圖管理
    ├── path_optimizer/ # 路徑最佳化
    └── collision_detection/ # 碰撞檢測
```

## 核心功能

### 路徑規劃
- **A*演算法**: 高效能A*路徑搜尋實現
- **動態避障**: 即時障礙物迴避
- **路徑最佳化**: 平滑化與最短路徑最佳化
- **多目標規劃**: 支援多點路徑規劃

### 地圖管理
- **佔據格地圖**: 基於佔據格的地圖表示
- **動態更新**: 即時地圖更新機制
- **多層地圖**: 支援多層地圖結構
- **地圖編輯**: 地圖編輯與維護工具

## 開發指令

### 環境設定
```bash
# AGV容器內
source /app/setup.bash && all_source
cd /app/path_algorithm

# AGVC容器內
source /app/setup.bash && agvc_source
cd /app/path_algorithm
```

### 服務啟動
```bash
# 啟動路徑規劃服務
ros2 run path_algorithm path_planner_node

# 啟動地圖管理服務
ros2 run path_algorithm map_manager_node

# 測試路徑規劃
ros2 run path_algorithm test_path_planning
```

### 構建與測試
```bash
build_ws path_algorithm
ros2 test path_algorithm  # 路徑規劃測試
```

## A*演算法實現

### 核心A*演算法
```python
# astar/astar_planner.py
import heapq
import numpy as np
from typing import List, Tuple, Optional

class AStarPlanner:
    def __init__(self, map_data: np.ndarray, resolution: float = 0.1):
        self.map_data = map_data  # 2D佔據格地圖
        self.resolution = resolution
        self.height, self.width = map_data.shape
        
        # 移動方向(8方向)
        self.directions = [
            (-1, -1), (-1, 0), (-1, 1),
            (0, -1),           (0, 1),
            (1, -1),  (1, 0),  (1, 1)
        ]
        
        # 移動成本
        self.costs = [
            1.414, 1.0, 1.414,  # 對角線移動成本較高
            1.0,        1.0,
            1.414, 1.0, 1.414
        ]
    
    def plan_path(self, start: Tuple[float, float], 
                  goal: Tuple[float, float]) -> Optional[List[Tuple[float, float]]]:
        """A*路徑規劃主函數"""
        # 轉換為格子座標
        start_grid = self.world_to_grid(start)
        goal_grid = self.world_to_grid(goal)
        
        # 驗證起點和終點
        if not self.is_valid_point(start_grid) or not self.is_valid_point(goal_grid):
            return None
            
        # A*搜尋
        path_grid = self.astar_search(start_grid, goal_grid)
        
        if path_grid is None:
            return None
            
        # 轉換回世界座標
        path_world = [self.grid_to_world(point) for point in path_grid]
        
        return path_world
    
    def astar_search(self, start: Tuple[int, int], 
                     goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
        """A*搜尋演算法"""
        open_set = []
        heapq.heappush(open_set, (0, start))
        
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}
        
        open_set_hash = {start}
        
        while open_set:
            current = heapq.heappop(open_set)[1]
            open_set_hash.remove(current)
            
            if current == goal:
                return self.reconstruct_path(came_from, current)
                
            for i, direction in enumerate(self.directions):
                neighbor = (current[0] + direction[0], current[1] + direction[1])
                
                if not self.is_valid_point(neighbor):
                    continue
                    
                tentative_g_score = g_score[current] + self.costs[i]
                
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    
                    if neighbor not in open_set_hash:
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))
                        open_set_hash.add(neighbor)
        
        return None  # 無法找到路徑
    
    def heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """啟發式函數(歐氏距離)"""
        return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
    
    def is_valid_point(self, point: Tuple[int, int]) -> bool:
        """檢查點是否有效"""
        x, y = point
        if x < 0 or x >= self.height or y < 0 or y >= self.width:
            return False
        return self.map_data[x, y] == 0  # 0表示空閒空間
    
    def world_to_grid(self, world_point: Tuple[float, float]) -> Tuple[int, int]:
        """世界座標轉換為格子座標"""
        x, y = world_point
        grid_x = int(x / self.resolution)
        grid_y = int(y / self.resolution)
        return (grid_x, grid_y)
    
    def grid_to_world(self, grid_point: Tuple[int, int]) -> Tuple[float, float]:
        """格子座標轉換為世界座標"""
        grid_x, grid_y = grid_point
        world_x = (grid_x + 0.5) * self.resolution
        world_y = (grid_y + 0.5) * self.resolution
        return (world_x, world_y)
    
    def reconstruct_path(self, came_from: dict, 
                        current: Tuple[int, int]) -> List[Tuple[int, int]]:
        """重建路徑"""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path
```

### 動態A*實現
```python
# astar/dynamic_astar.py
class DynamicAStarPlanner(AStarPlanner):
    def __init__(self, map_data: np.ndarray, resolution: float = 0.1):
        super().__init__(map_data, resolution)
        self.dynamic_obstacles = set()
        self.obstacle_buffer = 2  # 障礙物緩衝區(格子數)
        
    def add_dynamic_obstacle(self, obstacle: Tuple[float, float], radius: float):
        """添加動態障礙物"""
        center_grid = self.world_to_grid(obstacle)
        radius_grid = int(radius / self.resolution)
        
        # 在障礙物周圍創建緩衝區
        for dx in range(-radius_grid - self.obstacle_buffer, 
                       radius_grid + self.obstacle_buffer + 1):
            for dy in range(-radius_grid - self.obstacle_buffer,
                           radius_grid + self.obstacle_buffer + 1):
                x, y = center_grid[0] + dx, center_grid[1] + dy
                if 0 <= x < self.height and 0 <= y < self.width:
                    distance = np.sqrt(dx**2 + dy**2)
                    if distance <= radius_grid + self.obstacle_buffer:
                        self.dynamic_obstacles.add((x, y))
    
    def clear_dynamic_obstacles(self):
        """清除動態障礙物"""
        self.dynamic_obstacles.clear()
    
    def is_valid_point(self, point: Tuple[int, int]) -> bool:
        """檢查點是否有效(包含動態障礙物)"""
        if not super().is_valid_point(point):
            return False
        return point not in self.dynamic_obstacles
```

## 路徑最佳化

### 路徑平滑化
```python
# path_optimizer/path_smoother.py
class PathSmoother:
    def __init__(self, map_data: np.ndarray, resolution: float):
        self.map_data = map_data
        self.resolution = resolution
        self.planner = AStarPlanner(map_data, resolution)
        
    def smooth_path(self, path: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """路徑平滑化"""
        if len(path) < 3:
            return path
            
        smoothed_path = [path[0]]  # 起點
        
        i = 0
        while i < len(path) - 1:
            j = len(path) - 1
            
            # 找到最遠的可直達點
            while j > i + 1:
                if self.is_line_clear(path[i], path[j]):
                    smoothed_path.append(path[j])
                    i = j
                    break
                j -= 1
            else:
                # 如果沒有找到可直達點，移動到下一個點
                i += 1
                if i < len(path):
                    smoothed_path.append(path[i])
        
        return smoothed_path
    
    def is_line_clear(self, start: Tuple[float, float], 
                      end: Tuple[float, float]) -> bool:
        """檢查兩點間直線是否無障礙"""
        start_grid = self.planner.world_to_grid(start)
        end_grid = self.planner.world_to_grid(end)
        
        # Bresenham直線演算法
        points = self.bresenham_line(start_grid, end_grid)
        
        for point in points:
            if not self.planner.is_valid_point(point):
                return False
                
        return True
    
    def bresenham_line(self, start: Tuple[int, int], 
                      end: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Bresenham直線演算法"""
        x0, y0 = start
        x1, y1 = end
        
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        
        err = dx - dy
        
        while True:
            points.append((x0, y0))
            
            if x0 == x1 and y0 == y1:
                break
                
            e2 = 2 * err
            
            if e2 > -dy:
                err -= dy
                x0 += sx
                
            if e2 < dx:
                err += dx
                y0 += sy
        
        return points
```

### 貝茲曲線平滑
```python
# path_optimizer/bezier_smoother.py
class BezierSmoother:
    def smooth_with_bezier(self, waypoints: List[Tuple[float, float]], 
                          num_points: int = 100) -> List[Tuple[float, float]]:
        """使用貝茲曲線平滑路徑"""
        if len(waypoints) < 2:
            return waypoints
            
        smooth_path = []
        
        for i in range(len(waypoints) - 1):
            # 為每段路徑生成貝茲曲線
            segment = self.generate_bezier_segment(
                waypoints[i], waypoints[i + 1], num_points // (len(waypoints) - 1)
            )
            smooth_path.extend(segment)
            
        return smooth_path
    
    def generate_bezier_segment(self, start: Tuple[float, float], 
                               end: Tuple[float, float], 
                               num_points: int) -> List[Tuple[float, float]]:
        """生成貝茲曲線段"""
        # 簡單的二次貝茲曲線
        control_point = (
            (start[0] + end[0]) / 2,
            (start[1] + end[1]) / 2
        )
        
        points = []
        for i in range(num_points):
            t = i / (num_points - 1)
            point = self.quadratic_bezier(start, control_point, end, t)
            points.append(point)
            
        return points
    
    def quadratic_bezier(self, p0: Tuple[float, float], 
                        p1: Tuple[float, float], 
                        p2: Tuple[float, float], 
                        t: float) -> Tuple[float, float]:
        """二次貝茲曲線計算"""
        x = (1-t)**2 * p0[0] + 2*(1-t)*t * p1[0] + t**2 * p2[0]
        y = (1-t)**2 * p0[1] + 2*(1-t)*t * p1[1] + t**2 * p2[1]
        return (x, y)
```

## 地圖管理

### 佔據格地圖管理器
```python
# map_manager/occupancy_grid_manager.py
class OccupancyGridManager:
    def __init__(self, width: int, height: int, resolution: float):
        self.width = width
        self.height = height
        self.resolution = resolution
        self.grid = np.zeros((height, width), dtype=np.int8)
        
        # 地圖值定義
        self.FREE_SPACE = 0
        self.OCCUPIED = 100
        self.UNKNOWN = -1
        
    def load_map_from_file(self, map_file: str):
        """從檔案載入地圖"""
        try:
            with open(map_file, 'r') as f:
                map_data = yaml.safe_load(f)
                
            # 載入地圖圖像
            image_path = os.path.join(os.path.dirname(map_file), map_data['image'])
            map_image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
            
            # 轉換為佔據格
            self.grid = self.image_to_occupancy_grid(
                map_image, 
                map_data['occupied_thresh'], 
                map_data['free_thresh']
            )
            
            self.resolution = map_data['resolution']
            
        except Exception as e:
            self.get_logger().error(f"載入地圖失敗: {e}")
            
    def image_to_occupancy_grid(self, image: np.ndarray, 
                               occupied_thresh: float, 
                               free_thresh: float) -> np.ndarray:
        """圖像轉換為佔據格"""
        normalized = image.astype(float) / 255.0
        
        occupancy_grid = np.full(image.shape, self.UNKNOWN, dtype=np.int8)
        
        # 自由空間
        occupancy_grid[normalized > free_thresh] = self.FREE_SPACE
        
        # 佔據空間
        occupancy_grid[normalized < occupied_thresh] = self.OCCUPIED
        
        return occupancy_grid
    
    def update_obstacle(self, x: float, y: float, radius: float, is_occupied: bool):
        """更新障礙物"""
        center_grid = self.world_to_grid((x, y))
        radius_grid = int(radius / self.resolution)
        
        value = self.OCCUPIED if is_occupied else self.FREE_SPACE
        
        for dx in range(-radius_grid, radius_grid + 1):
            for dy in range(-radius_grid, radius_grid + 1):
                if dx*dx + dy*dy <= radius_grid*radius_grid:
                    grid_x = center_grid[0] + dx
                    grid_y = center_grid[1] + dy
                    
                    if 0 <= grid_x < self.height and 0 <= grid_y < self.width:
                        self.grid[grid_x, grid_y] = value
    
    def world_to_grid(self, world_point: Tuple[float, float]) -> Tuple[int, int]:
        """世界座標轉格子座標"""
        x, y = world_point
        grid_x = int(x / self.resolution)
        grid_y = int(y / self.resolution)
        return (grid_x, grid_y)
    
    def get_occupancy_grid_msg(self) -> OccupancyGrid:
        """生成ROS OccupancyGrid訊息"""
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        
        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.origin.position.x = 0.0
        msg.info.origin.position.y = 0.0
        msg.info.origin.orientation.w = 1.0
        
        msg.data = self.grid.flatten().tolist()
        
        return msg
```

## ROS 2介面整合

### 路徑規劃服務節點
```python
# path_algorithm/path_planner_node.py
class PathPlannerNode(Node):
    def __init__(self):
        super().__init__('path_planner')
        
        # 載入地圖
        self.map_manager = OccupancyGridManager(1000, 1000, 0.1)
        self.map_manager.load_map_from_file('/app/config/maps/warehouse_map.yaml')
        
        # 創建路徑規劃器
        self.planner = DynamicAStarPlanner(
            self.map_manager.grid, 
            self.map_manager.resolution
        )
        
        # 路徑平滑器
        self.smoother = PathSmoother(
            self.map_manager.grid, 
            self.map_manager.resolution
        )
        
        # 創建服務
        self.plan_path_service = self.create_service(
            GetPlan, '/path_planner/get_plan', self.handle_plan_request
        )
        
        # 發布地圖
        self.map_publisher = self.create_publisher(OccupancyGrid, '/map', 1)
        self.publish_map_timer = self.create_timer(1.0, self.publish_map)
        
        # 發布路徑
        self.path_publisher = self.create_publisher(Path, '/planned_path', 1)
        
    def handle_plan_request(self, request, response):
        """處理路徑規劃請求"""
        try:
            start = (request.start.pose.position.x, request.start.pose.position.y)
            goal = (request.goal.pose.position.x, request.goal.pose.position.y)
            
            # 更新動態障礙物
            self.update_dynamic_obstacles(request.obstacles)
            
            # 規劃路徑
            raw_path = self.planner.plan_path(start, goal)
            
            if raw_path is None:
                response.success = False
                response.message = "無法找到路徑"
                return response
                
            # 路徑平滑化
            smoothed_path = self.smoother.smooth_path(raw_path)
            
            # 轉換為ROS Path訊息
            path_msg = self.create_path_message(smoothed_path)
            response.path = path_msg
            response.success = True
            response.message = "路徑規劃成功"
            
            # 發布路徑
            self.path_publisher.publish(path_msg)
            
        except Exception as e:
            response.success = False
            response.message = f"路徑規劃錯誤: {str(e)}"
            
        return response
    
    def update_dynamic_obstacles(self, obstacles):
        """更新動態障礙物"""
        self.planner.clear_dynamic_obstacles()
        
        for obstacle in obstacles:
            self.planner.add_dynamic_obstacle(
                (obstacle.position.x, obstacle.position.y),
                obstacle.radius
            )
    
    def create_path_message(self, path: List[Tuple[float, float]]) -> Path:
        """創建ROS Path訊息"""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"
        
        for point in path:
            pose_stamped = PoseStamped()
            pose_stamped.header = path_msg.header
            pose_stamped.pose.position.x = point[0]
            pose_stamped.pose.position.y = point[1]
            pose_stamped.pose.orientation.w = 1.0
            
            path_msg.poses.append(pose_stamped)
            
        return path_msg
    
    def publish_map(self):
        """發布地圖"""
        map_msg = self.map_manager.get_occupancy_grid_msg()
        self.map_publisher.publish(map_msg)
```

## 配置管理

### 路徑規劃參數
```yaml
# /app/config/path_planning_config.yaml
path_planner:
  # 地圖設定
  map:
    file: "/app/config/maps/warehouse_map.yaml"
    resolution: 0.1          # 地圖解析度(m/格子)
    
  # A*演算法參數
  astar:
    allow_diagonal: true     # 是否允許對角線移動
    heuristic_weight: 1.0    # 啟發式權重
    tie_breaker: 0.001       # 打破平局的小增量
    
  # 動態障礙物設定
  dynamic_obstacles:
    buffer_distance: 0.3     # 障礙物緩衝距離(m)
    update_rate: 10.0        # 更新頻率(Hz)
    
  # 路徑平滑化參數
  smoothing:
    enabled: true
    method: "line_simplification"  # "bezier" 或 "line_simplification"
    bezier_points: 50        # 貝茲曲線點數
    
  # 性能設定
  performance:
    max_planning_time: 5.0   # 最大規劃時間(秒)
    cache_size: 100          # 路徑快取大小
    parallel_processing: true # 是否啟用並行處理
```

### 地圖配置
```yaml
# /app/config/maps/warehouse_map.yaml  
image: warehouse_layout.pgm
resolution: 0.1
origin: [0.0, 0.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196

# 地圖元數據
metadata:
  width: 1000              # 地圖寬度(格子)
  height: 500              # 地圖高度(格子)
  real_width: 100.0        # 實際寬度(m)
  real_height: 50.0        # 實際高度(m)
  
# 預定義位置
locations:
  charging_stations:
    - {name: "CHARGE_01", x: 5.0, y: 45.0}
    - {name: "CHARGE_02", x: 15.0, y: 45.0}
    
  pickup_points:
    - {name: "PICKUP_A", x: 20.0, y: 10.0}
    - {name: "PICKUP_B", x: 30.0, y: 15.0}
    
  drop_points:
    - {name: "DROP_A", x: 70.0, y: 20.0}
    - {name: "DROP_B", x: 80.0, y: 25.0}
```

## 測試與調試

### 路徑規劃測試
```bash
# 測試基本路徑規劃
ros2 service call /path_planner/get_plan nav_msgs/srv/GetPlan "{
  start: {pose: {position: {x: 0.0, y: 0.0}}},
  goal: {pose: {position: {x: 10.0, y: 10.0}}}
}"

# 查看規劃的路徑
ros2 topic echo /planned_path

# 測試動態障礙物
ros2 run path_algorithm test_dynamic_obstacles
```

### 性能測試
```python
# tests/performance_test.py
class PathPlanningPerformanceTest:
    def __init__(self):
        self.planner = AStarPlanner(self.load_test_map(), 0.1)
        
    def test_planning_speed(self, num_tests: int = 1000):
        """測試路徑規劃速度"""
        total_time = 0
        successful_plans = 0
        
        for _ in range(num_tests):
            start = self.generate_random_point()
            goal = self.generate_random_point()
            
            start_time = time.time()
            path = self.planner.plan_path(start, goal)
            end_time = time.time()
            
            if path is not None:
                successful_plans += 1
                total_time += (end_time - start_time)
                
        avg_time = total_time / successful_plans if successful_plans > 0 else 0
        success_rate = successful_plans / num_tests
        
        return {
            'average_planning_time': avg_time,
            'success_rate': success_rate,
            'total_tests': num_tests
        }
```

### 視覺化工具
```bash
# 在RViz中視覺化路徑
ros2 run rviz2 rviz2 -d /app/config/path_planning_visualization.rviz

# 地圖編輯工具
ros2 run path_algorithm map_editor

# 路徑規劃調試工具
ros2 run path_algorithm path_planning_debugger
```

## 最佳化技巧

### 性能最佳化
- **預計算**: 預計算常用路徑片段
- **分層規劃**: 粗略規劃+精細規劃
- **快取機制**: 快取計算結果
- **並行處理**: 多執行緒路徑搜尋

### 記憶體最佳化
- **稀疏地圖**: 僅存儲非空區域
- **動態分配**: 根據需要動態分配記憶體
- **資料壓縮**: 壓縮地圖資料
- **記憶體池**: 使用記憶體池管理

## 故障排除

### 常見問題
1. **找不到路徑**: 檢查地圖數據與起終點有效性
2. **規劃時間過長**: 調整演算法參數或使用分層規劃
3. **路徑不平滑**: 啟用路徑平滑化或調整參數
4. **動態障礙物未生效**: 確認障礙物更新頻率與緩衝設定

### 診斷工具
```bash
# 路徑規劃診斷
ros2 run path_algorithm path_planner_diagnostics

# 地圖驗證
ros2 run path_algorithm map_validator

# 性能分析
ros2 run path_algorithm performance_analyzer
```

## 重要提醒
- 路徑規劃演算法直接影響AGV導航效率與安全
- 地圖數據需定期更新以反映環境變化  
- 動態障礙物檢測對避碰至關重要
- 支援AGV與AGVC雙環境，可用於不同場景的路徑規劃需求