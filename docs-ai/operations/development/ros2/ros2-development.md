# ROS 2 開發操作指導

## 🎯 適用場景
- ROS 2 節點和套件開發
- 工作空間管理和建置
- 跨工作空間依賴處理
- ROS 2 最佳實踐實施

## 📋 RosAGV ROS 2 開發環境

### 環境要求
- **容器內開發**: 所有 ROS 2 開發必須在 Docker 容器內進行
- **雙環境支援**: AGV 車載環境 (9個工作空間) 和 AGVC 管理環境 (11個工作空間)
- **自動載入**: 使用 `all_source` 自動檢測並載入對應環境的工作空間

### 基本開發流程
```bash
# [宿主機] 1. 進入對應容器環境
docker compose -f docker-compose.yml exec rosagv bash      # AGV 環境
docker compose -f docker-compose.agvc.yml exec agvc_server bash  # AGVC 環境

# [容器內] 2. 載入工作空間
all_source              # 自動載入 (推薦)
# 或
agv_source             # 強制載入 AGV 工作空間
agvc_source            # 強制載入 AGVC 工作空間

# [容器內] 3. 檢查環境狀態
check_system_status    # 整體系統狀態
check_ros_env          # ROS 2 環境驗證
```

## 🔧 工作空間管理

### 工作空間結構
```
AGV 車載工作空間 (9個):
├── agv_ws/                    # 核心 AGV 控制
├── agv_cmd_service_ws/        # 手動指令服務
├── joystick_ws/               # 搖桿控制
├── sensorpart_ws/             # 感測器處理
├── keyence_plc_ws/            # PLC 通訊
├── plc_proxy_ws/              # PLC 代理
├── path_algorithm/            # 路徑規劃
└── [2個預留工作空間]

AGVC 管理工作空間 (11個):
├── web_api_ws/                # Web API 服務
├── db_proxy_ws/               # 資料庫代理
├── ecs_ws/                    # 設備控制系統
├── rcs_ws/                    # 機器人控制系統
# (wcs_ws 已整合至 tafl_wcs_ws)
├── kuka_fleet_ws/             # KUKA Fleet 整合
├── tafl_ws/                   # TAFL 解析器和執行器
├── tafl_wcs_ws/               # TAFL WCS 整合
└── [共用工作空間: keyence_plc_ws, plc_proxy_ws, path_algorithm]
```

### 建置管理
```bash
# 建置所有工作空間
build_all              # 自動建置腳本

# 建置特定工作空間
colcon build --packages-select package_name
colcon build --packages-up-to package_name

# 並行建置 (加速)
colcon build --parallel-workers 4

# 僅建置變更的套件
colcon build --packages-select-modified
```

### 測試執行
```bash
# 執行所有測試
colcon test

# 執行特定套件測試
colcon test --packages-select package_name

# 查看測試結果
colcon test-result --verbose
```

## 🚀 ROS 2 開發最佳實踐

### 套件開發
```bash
# 建立新套件 (Python)
ros2 pkg create --build-type ament_python package_name

# 建立新套件 (C++)
ros2 pkg create --build-type ament_cmake package_name

# 建立新套件 (混合)
ros2 pkg create --build-type ament_cmake package_name --dependencies rclpy rclcpp
```

### 節點開發模式
```python
# ROS 2 Python 節點模板
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.get_logger().info('Node started')
        
        # 發布者
        self.publisher = self.create_publisher(MsgType, 'topic_name', 10)
        
        # 訂閱者
        self.subscription = self.create_subscription(
            MsgType, 'topic_name', self.callback, 10)
        
        # 服務
        self.service = self.create_service(SrvType, 'service_name', self.service_callback)
        
        # 定時器
        self.timer = self.create_timer(1.0, self.timer_callback)
    
    def callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')
    
    def service_callback(self, request, response):
        # 處理服務請求
        return response
    
    def timer_callback(self):
        # 定時執行的任務
        pass

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 介面定義
```bash
# 建立自定義訊息
# 在 msg/ 目錄下建立 .msg 檔案
# 例如: MyMessage.msg
string data
int32 count
float64 value

# 建立自定義服務
# 在 srv/ 目錄下建立 .srv 檔案
# 例如: MyService.srv
string request_data
---
bool success
string response_data

# 建立自定義動作
# 在 action/ 目錄下建立 .action 檔案
# 例如: MyAction.action
string goal_data
---
bool success
---
float32 progress
```

### 參數管理
```python
# 宣告參數
self.declare_parameter('param_name', default_value)

# 讀取參數
param_value = self.get_parameter('param_name').value

# 參數回調
self.add_on_set_parameters_callback(self.parameter_callback)

def parameter_callback(self, params):
    for param in params:
        if param.name == 'param_name':
            # 處理參數變更
            pass
    return SetParametersResult(successful=True)
```

## 🔍 除錯和診斷

### 常用除錯指令
```bash
# 查看節點列表
ros2 node list

# 查看主題列表
ros2 topic list

# 查看主題資訊
ros2 topic info /topic_name
ros2 topic echo /topic_name

# 查看服務列表
ros2 service list

# 呼叫服務
ros2 service call /service_name service_type "request_data"

# 查看參數
ros2 param list
ros2 param get /node_name param_name
```

### 日誌管理
```python
# 使用適當的日誌等級
self.get_logger().debug('Debug message')
self.get_logger().info('Info message')
self.get_logger().warn('Warning message')
self.get_logger().error('Error message')
self.get_logger().fatal('Fatal message')
```

### 效能監控
```bash
# 查看主題頻率
ros2 topic hz /topic_name

# 查看主題頻寬
ros2 topic bw /topic_name

# 系統監控
top
htop
```

## 🧪 測試策略

### 單元測試
```python
import unittest
from my_package.my_module import MyClass

class TestMyClass(unittest.TestCase):
    def setUp(self):
        self.my_object = MyClass()
    
    def test_functionality(self):
        result = self.my_object.some_method()
        self.assertEqual(result, expected_value)
    
    def tearDown(self):
        pass

if __name__ == '__main__':
    unittest.main()
```

### 整合測試
```python
import rclpy
import unittest
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_testing.actions import ReadyToTest

class TestIntegration(unittest.TestCase):
    def test_node_communication(self):
        # 測試節點間通訊
        pass
```

### 測試執行
```bash
# 執行 Python 測試
python3 -m pytest test/

# 執行 ROS 2 測試
colcon test --packages-select package_name

# 查看測試覆蓋率
colcon test --packages-select package_name --pytest-args --cov=package_name
```

## 📦 套件管理

### 依賴管理
```xml
<!-- package.xml 依賴聲明 -->
<depend>rclpy</depend>
<depend>std_msgs</depend>
<depend>geometry_msgs</depend>

<build_depend>ament_cmake</build_depend>
<exec_depend>launch</exec_depend>
<test_depend>ament_lint_auto</test_depend>
```

### 安裝和分發
```bash
# 安裝套件
colcon build --packages-select package_name
source install/setup.bash

# 建立分發包
colcon build --packages-select package_name --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## 🛡️ 節點生命週期管理

### 優雅關閉實作
ROS 2 節點應該實作優雅關閉機制，確保資源正確釋放：

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import signal
import sys

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.is_shutting_down = False
        self.init_resources()

    def cleanup(self):
        """清理資源"""
        self.get_logger().info("Starting graceful shutdown...")
        self.is_shutting_down = True

        # 取消定時器
        if hasattr(self, 'timer'):
            self.timer.cancel()
            self.destroy_timer(self.timer)

        # 銷毀發布者/訂閱者
        if hasattr(self, 'publisher'):
            self.destroy_publisher(self.publisher)

        self.get_logger().info("Graceful shutdown completed")

def main(args=None):
    rclpy.init(args=args)
    node = None

    def signal_handler(signum, frame):
        nonlocal node
        if node:
            node.cleanup()
        if rclpy.ok():
            rclpy.shutdown()
        sys.exit(0)

    # 註冊信號處理器
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        node = MyNode()
        rclpy.spin(node)
    finally:
        if node:
            node.cleanup()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 生命週期最佳實踐
- 註冊 SIGINT 和 SIGTERM 信號處理器
- 在 cleanup() 方法中釋放所有資源
- 使用 try-finally 確保清理執行
- 設置關閉標誌避免處理新請求
- 等待活動任務完成（帶超時）

## 🔧 開發工具整合

### 統一工具使用
```bash
# 開發環境檢查
r dev-status

# 快速建置
r dev-build

# 執行測試
r dev-test

# 代碼品質檢查
r dev-check
```

### 專業工具載入
```bash
# 載入開發工具集
source scripts/dev-tools/dev-tools.sh

# 使用專業工具
dev_build              # 自動建置
dev_test               # 執行測試
dev_check_style        # 代碼風格檢查
```

## 📋 檢查清單

### 開發前檢查
- [ ] 確認在正確的容器環境中
- [ ] 工作空間已正確載入
- [ ] ROS 2 環境變數已設定
- [ ] 依賴套件已安裝

### 開發中檢查
- [ ] 程式碼符合風格規範
- [ ] 單元測試已撰寫並通過
- [ ] 介面定義清晰且文檔完整
- [ ] 錯誤處理機制完善

### 開發後檢查
- [ ] 整合測試通過
- [ ] 效能符合要求
- [ ] 文檔已更新
- [ ] 版本控制提交完整

## 🔗 交叉引用
- AGV 工作空間詳細: docs-ai/context/workspaces/agv-workspaces.md
- AGVC 工作空間詳細: docs-ai/context/workspaces/agvc-workspaces.md
- 測試程序: docs-ai/operations/development/testing/testing-procedures.md
- 容器開發: docs-ai/operations/development/docker-development.md
- ROS 2 架構: docs-ai/knowledge/protocols/ros2-interfaces.md
