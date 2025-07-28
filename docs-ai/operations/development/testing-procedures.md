# 測試程序操作指導

## 🎯 適用場景
- ROS 2 套件單元測試和整合測試
- Web API 測試和驗證
- 系統端到端測試
- 自動化測試流程建立

## 📋 RosAGV 測試架構

### 測試分層
```
RosAGV 測試金字塔
├── 🔬 單元測試 (Unit Tests)
│   ├── Python 函數測試
│   ├── ROS 2 節點邏輯測試
│   ├── 資料模型測試
│   └── 工具函數測試
├── 🔗 整合測試 (Integration Tests)
│   ├── ROS 2 節點間通訊測試
│   ├── 資料庫操作測試
│   ├── API 端點測試
│   └── 服務整合測試
├── 🎯 系統測試 (System Tests)
│   ├── 端到端工作流程測試
│   ├── 效能測試
│   ├── 負載測試
│   └── 安全性測試
└── 🚀 驗收測試 (Acceptance Tests)
    ├── 用戶場景測試
    ├── 業務流程驗證
    └── 部署驗證測試
```

### 測試工具
- **pytest**: Python 單元測試框架
- **pytest-asyncio**: 異步測試支援
- **pytest-cov**: 測試覆蓋率
- **launch_testing**: ROS 2 測試框架
- **httpx**: HTTP 客戶端測試
- **pytest-mock**: Mock 和 Stub

## 🔬 單元測試

### ROS 2 節點測試
```python
# test_agv_node.py
import pytest
import rclpy
from rclpy.node import Node
from agv_base.agv_node_base import AGVNodeBase
from agv_interfaces.msg import AGVStatus

class TestAGVNode:
    @classmethod
    def setup_class(cls):
        rclpy.init()
    
    @classmethod
    def teardown_class(cls):
        rclpy.shutdown()
    
    def setup_method(self):
        self.node = AGVNodeBase()
    
    def teardown_method(self):
        self.node.destroy_node()
    
    def test_node_initialization(self):
        """測試節點初始化"""
        assert self.node.get_name() == 'agv_node_base'
        assert self.node.current_state is not None
    
    def test_state_transition(self):
        """測試狀態轉換"""
        initial_state = self.node.current_state
        self.node.transition_to('moving')
        assert self.node.current_state == 'moving'
        assert self.node.previous_state == initial_state
    
    def test_status_publication(self):
        """測試狀態發布"""
        # 設置訂閱者來接收狀態訊息
        received_messages = []
        
        def status_callback(msg):
            received_messages.append(msg)
        
        subscription = self.node.create_subscription(
            AGVStatus,
            '/agv_status',
            status_callback,
            10
        )
        
        # 發布狀態
        self.node.publish_status()
        
        # 等待訊息
        rclpy.spin_once(self.node, timeout_sec=1.0)
        
        assert len(received_messages) > 0
        assert received_messages[0].agv_id == self.node.agv_id
```

### 資料模型測試
```python
# test_models.py
import pytest
from sqlmodel import Session, create_engine
from agv_models import AGV, Task
from datetime import datetime

class TestAGVModel:
    @pytest.fixture
    def session(self):
        engine = create_engine("sqlite:///:memory:")
        with Session(engine) as session:
            yield session
    
    def test_agv_creation(self, session):
        """測試 AGV 模型創建"""
        agv = AGV(
            name="AGV001",
            agv_type="loader",
            status="idle",
            battery_level=0.85
        )
        session.add(agv)
        session.commit()
        session.refresh(agv)
        
        assert agv.id is not None
        assert agv.name == "AGV001"
        assert agv.created_at is not None
    
    def test_agv_validation(self):
        """測試 AGV 模型驗證"""
        # 測試電池電量範圍驗證
        with pytest.raises(ValueError):
            AGV(
                name="AGV001",
                agv_type="loader",
                battery_level=1.5  # 超出範圍
            )
    
    def test_agv_relationships(self, session):
        """測試 AGV 關聯關係"""
        agv = AGV(name="AGV001", agv_type="loader")
        task = Task(
            task_type="move",
            target_location="A1",
            agv=agv
        )
        
        session.add(agv)
        session.add(task)
        session.commit()
        
        assert len(agv.tasks) == 1
        assert agv.tasks[0].task_type == "move"
```

### 工具函數測試
```python
# test_utils.py
import pytest
from agv_utils import calculate_distance, validate_coordinates
from geometry_msgs.msg import Point

class TestUtils:
    def test_calculate_distance(self):
        """測試距離計算"""
        point1 = Point(x=0.0, y=0.0, z=0.0)
        point2 = Point(x=3.0, y=4.0, z=0.0)
        
        distance = calculate_distance(point1, point2)
        assert abs(distance - 5.0) < 0.001
    
    def test_validate_coordinates(self):
        """測試座標驗證"""
        # 有效座標
        assert validate_coordinates(1.0, 2.0) == True
        
        # 無效座標
        assert validate_coordinates(float('inf'), 2.0) == False
        assert validate_coordinates(1.0, float('nan')) == False
    
    @pytest.mark.parametrize("x,y,expected", [
        (0.0, 0.0, True),
        (1.5, -2.3, True),
        (float('inf'), 0.0, False),
        (0.0, float('nan'), False),
    ])
    def test_validate_coordinates_parametrized(self, x, y, expected):
        """參數化測試座標驗證"""
        assert validate_coordinates(x, y) == expected
```

## 🔗 整合測試

### ROS 2 節點通訊測試
```python
# test_node_communication.py
import pytest
import rclpy
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_testing.actions import ReadyToTest
import launch_testing

@pytest.mark.launch_test
def generate_test_description():
    """生成測試啟動描述"""
    return LaunchDescription([
        Node(
            package='agv_base',
            executable='agv_node',
            name='test_agv_node'
        ),
        Node(
            package='agv_base',
            executable='task_manager',
            name='test_task_manager'
        ),
        ReadyToTest()
    ])

class TestNodeCommunication:
    def test_agv_task_communication(self):
        """測試 AGV 節點與任務管理器通訊"""
        # 建立測試客戶端
        rclpy.init()
        test_node = rclpy.create_node('test_client')
        
        # 建立服務客戶端
        client = test_node.create_client(
            TaskAssignment,
            '/assign_task'
        )
        
        # 等待服務可用
        assert client.wait_for_service(timeout_sec=5.0)
        
        # 發送任務分配請求
        request = TaskAssignment.Request()
        request.agv_id = "AGV001"
        request.task_type = "move"
        request.target_location = "A1"
        
        future = client.call_async(request)
        rclpy.spin_until_future_complete(test_node, future)
        
        response = future.result()
        assert response.success == True
        assert response.task_id is not None
        
        test_node.destroy_node()
        rclpy.shutdown()
```

### Web API 測試
```python
# test_api.py
import pytest
from fastapi.testclient import TestClient
from httpx import AsyncClient
from main import app
from database import get_session
from sqlmodel import Session, create_engine

@pytest.fixture
def client():
    """測試客戶端"""
    return TestClient(app)

@pytest.fixture
async def async_client():
    """異步測試客戶端"""
    async with AsyncClient(app=app, base_url="http://test") as ac:
        yield ac

class TestAGVAPI:
    def test_health_check(self, client):
        """測試健康檢查端點"""
        response = client.get("/health")
        assert response.status_code == 200
        assert response.json()["status"] == "healthy"
    
    def test_get_agvs(self, client):
        """測試獲取 AGV 列表"""
        response = client.get("/api/v1/agvs/")
        assert response.status_code == 200
        assert isinstance(response.json(), list)
    
    def test_create_agv(self, client):
        """測試創建 AGV"""
        agv_data = {
            "name": "AGV001",
            "agv_type": "loader",
            "status": "idle",
            "battery_level": 0.85
        }
        response = client.post("/api/v1/agvs/", json=agv_data)
        assert response.status_code == 200
        
        created_agv = response.json()
        assert created_agv["name"] == "AGV001"
        assert created_agv["id"] is not None
    
    def test_get_agv_not_found(self, client):
        """測試獲取不存在的 AGV"""
        response = client.get("/api/v1/agvs/99999")
        assert response.status_code == 404
    
    @pytest.mark.asyncio
    async def test_async_operations(self, async_client):
        """測試異步操作"""
        response = await async_client.get("/api/v1/agvs/")
        assert response.status_code == 200
```

### 資料庫整合測試
```python
# test_database_integration.py
import pytest
from sqlmodel import Session, create_engine
from database import get_session
from crud import create_agv, get_agv, update_agv, delete_agv
from models import AGV

@pytest.fixture
def test_session():
    """測試資料庫會話"""
    engine = create_engine("sqlite:///:memory:")
    with Session(engine) as session:
        yield session

class TestDatabaseIntegration:
    @pytest.mark.asyncio
    async def test_agv_crud_operations(self, test_session):
        """測試 AGV CRUD 操作"""
        # Create
        agv_data = {
            "name": "AGV001",
            "agv_type": "loader",
            "status": "idle",
            "battery_level": 0.85
        }
        created_agv = await create_agv(test_session, agv_data)
        assert created_agv.id is not None
        
        # Read
        retrieved_agv = await get_agv(test_session, created_agv.id)
        assert retrieved_agv.name == "AGV001"
        
        # Update
        update_data = {"status": "moving", "battery_level": 0.80}
        updated_agv = await update_agv(test_session, created_agv.id, update_data)
        assert updated_agv.status == "moving"
        assert updated_agv.battery_level == 0.80
        
        # Delete
        deleted = await delete_agv(test_session, created_agv.id)
        assert deleted == True
        
        # Verify deletion
        deleted_agv = await get_agv(test_session, created_agv.id)
        assert deleted_agv is None
```

## 🎯 系統測試

### 端到端測試
```python
# test_e2e.py
import pytest
import time
from selenium import webdriver
from selenium.webdriver.common.by import By
from selenium.webdriver.support.ui import WebDriverWait
from selenium.webdriver.support import expected_conditions as EC

class TestE2E:
    @pytest.fixture
    def driver(self):
        """Web 驅動程式"""
        options = webdriver.ChromeOptions()
        options.add_argument('--headless')
        driver = webdriver.Chrome(options=options)
        yield driver
        driver.quit()
    
    def test_agvcui_dashboard(self, driver):
        """測試 AGVCUI 儀表板"""
        driver.get("http://localhost:8001/dashboard")
        
        # 等待頁面載入
        wait = WebDriverWait(driver, 10)
        dashboard = wait.until(
            EC.presence_of_element_located((By.CLASS_NAME, "dashboard"))
        )
        
        # 檢查關鍵元素
        assert driver.find_element(By.ID, "online-agvs")
        assert driver.find_element(By.ID, "active-tasks")
        assert driver.find_element(By.ID, "system-status")
    
    def test_opui_workflow(self, driver):
        """測試 OPUI 工作流程"""
        driver.get("http://localhost:8002/")
        
        # 測試叫空車功能
        call_button = driver.find_element(By.ID, "call-empty-agv")
        call_button.click()
        
        # 等待回應
        wait = WebDriverWait(driver, 10)
        notification = wait.until(
            EC.presence_of_element_located((By.CLASS_NAME, "notification"))
        )
        
        assert "成功" in notification.text or "失敗" in notification.text
```

### 效能測試
```python
# test_performance.py
import pytest
import time
import asyncio
from concurrent.futures import ThreadPoolExecutor
import requests

class TestPerformance:
    def test_api_response_time(self):
        """測試 API 回應時間"""
        start_time = time.time()
        response = requests.get("http://localhost:8000/api/v1/agvs/")
        end_time = time.time()
        
        response_time = end_time - start_time
        assert response_time < 1.0  # 回應時間應小於 1 秒
        assert response.status_code == 200
    
    def test_concurrent_requests(self):
        """測試並發請求"""
        def make_request():
            response = requests.get("http://localhost:8000/health")
            return response.status_code == 200
        
        # 並發 50 個請求
        with ThreadPoolExecutor(max_workers=50) as executor:
            futures = [executor.submit(make_request) for _ in range(50)]
            results = [future.result() for future in futures]
        
        # 所有請求都應該成功
        assert all(results)
    
    @pytest.mark.asyncio
    async def test_websocket_performance(self):
        """測試 WebSocket 效能"""
        import socketio
        
        sio = socketio.AsyncClient()
        messages_received = 0
        
        @sio.event
        async def agv_status_update(data):
            nonlocal messages_received
            messages_received += 1
        
        await sio.connect('http://localhost:8000')
        await sio.emit('subscribe_agv_status', {'agv_id': 'AGV001'})
        
        # 等待接收訊息
        await asyncio.sleep(5)
        
        await sio.disconnect()
        
        # 應該接收到狀態更新訊息
        assert messages_received > 0
```

## 🚀 自動化測試

### 測試執行腳本
```bash
#!/bin/bash
# run-tests.sh

echo "🧪 開始執行 RosAGV 測試套件..."

# 設定測試環境
export TESTING=true
export DATABASE_URL="sqlite:///:memory:"

# 執行單元測試
echo "📋 執行單元測試..."
python -m pytest tests/unit/ -v --cov=src --cov-report=html

# 執行整合測試
echo "🔗 執行整合測試..."
python -m pytest tests/integration/ -v

# 執行 ROS 2 測試
echo "🤖 執行 ROS 2 測試..."
colcon test --packages-select agv_base agv_interfaces
colcon test-result --verbose

# 執行 API 測試
echo "🌐 執行 API 測試..."
python -m pytest tests/api/ -v

# 生成測試報告
echo "📊 生成測試報告..."
coverage report
coverage html

echo "✅ 測試完成！"
echo "📄 測試報告: htmlcov/index.html"
```

### CI/CD 整合
```yaml
# .github/workflows/test.yml
name: RosAGV Tests

on: [push, pull_request]

jobs:
  test:
    runs-on: ubuntu-latest
    
    services:
      postgres:
        image: postgres:16
        env:
          POSTGRES_PASSWORD: postgres
        options: >-
          --health-cmd pg_isready
          --health-interval 10s
          --health-timeout 5s
          --health-retries 5
    
    steps:
    - uses: actions/checkout@v3
    
    - name: Set up Python
      uses: actions/setup-python@v3
      with:
        python-version: '3.12'
    
    - name: Install dependencies
      run: |
        pip install -r requirements.txt
        pip install pytest pytest-cov pytest-asyncio
    
    - name: Run tests
      run: |
        python -m pytest tests/ -v --cov=src
    
    - name: Upload coverage
      uses: codecov/codecov-action@v3
```

## 📋 測試最佳實踐

### 測試組織
1. **測試分類**: 按功能和層級組織測試
2. **命名規範**: 使用描述性的測試名稱
3. **測試隔離**: 每個測試獨立運行
4. **資料清理**: 測試後清理測試資料

### 測試覆蓋率
```bash
# 檢查測試覆蓋率
python -m pytest --cov=src --cov-report=term-missing

# 設定覆蓋率目標
# 單元測試: > 90%
# 整合測試: > 80%
# 系統測試: > 70%
```

### Mock 和 Stub
```python
# 使用 Mock 隔離外部依賴
from unittest.mock import Mock, patch

@patch('agv_node.hardware_interface')
def test_hardware_interaction(mock_hardware):
    mock_hardware.read_sensor.return_value = 0.85
    
    node = AGVNode()
    battery_level = node.get_battery_level()
    
    assert battery_level == 0.85
    mock_hardware.read_sensor.assert_called_once()
```

## 🔗 交叉引用
- ROS 2 開發: @docs-ai/operations/development/ros2-development.md
- Web 開發: @docs-ai/operations/development/web-development.md
- 資料庫操作: @docs-ai/operations/development/database-operations.md
- 系統診斷: @docs-ai/operations/maintenance/system-diagnostics.md
