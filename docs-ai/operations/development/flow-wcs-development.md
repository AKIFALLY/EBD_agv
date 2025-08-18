# Flow WCS 開發指導

## 🎯 適用場景
- Flow WCS 系統的開發和維護
- Linear Flow v2 流程設計和實作
- 自定義函數開發和整合
- 系統測試和故障排除

## 📋 開發環境設置

### 容器環境
```bash
# 進入 AGVC 容器
docker compose -f docker-compose.agvc.yml exec agvc_server bash

# 載入環境
source /app/setup.bash
agvc_source

# 切換到工作目錄
cd /app/flow_wcs_ws
```

### 依賴安裝
```bash
# Python 依賴
/opt/pyvenv_env/bin/pip3 install sqlalchemy psycopg2-binary pyyaml

# ROS 2 建置
colcon build --packages-select flow_wcs
source install/setup.bash
```

## 🔧 流程開發

### 創建新流程

#### 1. 使用 Linear Flow Designer
```
訪問: http://localhost:8001/linear-flow/designer
1. 點擊 "New Flow"
2. 設計流程
3. 設定 work_id
4. 匯出 YAML
```

#### 2. 手動創建 YAML
```yaml
# /app/config/wcs/flows/my_flow.yaml
meta:
  system: linear_flow_v2
  version: "2.0.0"
  author: "開發者"
  description: "流程描述"

flow:
  id: "my_flow"
  name: "我的流程"
  work_id: "300001"  # 唯一 Work ID
  enabled: true
  priority: 100

workflow:
  - section: "初始化"
    description: "流程初始化"
    steps:
      - id: "init"
        exec: "action.log"
        params:
          message: "流程開始"
          level: "info"
```

### 流程設計模式

#### 查詢-檢查-執行模式
```yaml
workflow:
  - section: "查詢階段"
    steps:
      - id: "query_data"
        exec: "query.locations"
        params:
          type: "rack"
          has_rack: true
        store: "locations"
  
  - section: "檢查階段"
    steps:
      - id: "check_empty"
        exec: "check.empty"
        params:
          data: "${locations}"
        store: "is_empty"
      
      - id: "skip_if_empty"
        exec: "action.log"
        params:
          message: "沒有可用位置"
        skip_if_not: "${is_empty}"
  
  - section: "執行階段"
    steps:
      - id: "create_tasks"
        exec: "foreach"
        items: "${locations}"
        var: "location"
        skip_if: "${is_empty}"
        steps:
          - id: "create_task"
            exec: "task.create"
            params:
              type: "MOVE"
              location_id: "${_location.id}"
```

#### 並行處理模式
```yaml
steps:
  - id: "parallel_process"
    exec: "parallel"
    branches:
      - name: "agv_tasks"
        steps:
          - id: "query_agvs"
            exec: "query.agvs"
            params:
              status: "idle"
            store: "idle_agvs"
      
      - name: "rack_tasks"
        steps:
          - id: "query_racks"
            exec: "query.racks"
            params:
              status: "ready"
            store: "ready_racks"
```

## 🎨 自定義函數開發

### 使用裝飾器註冊函數

#### 1. 創建函數模組
```python
# /app/flow_wcs_ws/src/flow_wcs/flow_wcs/functions/custom.py

from ..decorators import flow_function

@flow_function(
    category="custom",
    description="自定義功能",
    params=["param1", "param2"],
    returns="dict",
    defaults={"param2": "default_value"}
)
async def my_custom_function(self, params):
    """
    自定義函數實作
    
    Args:
        params: 包含 param1 和 param2 的字典
    
    Returns:
        處理結果字典
    """
    param1 = params.get('param1')
    param2 = params.get('param2', 'default_value')
    
    # 函數邏輯
    result = {
        'success': True,
        'data': f"處理 {param1} 和 {param2}"
    }
    
    return result
```

#### 2. 在執行器中註冊
```python
# /app/flow_wcs_ws/src/flow_wcs/flow_wcs/flow_executor.py

# 在 __init__ 或 register_functions 中加入
from .functions.custom import my_custom_function

# 函數會自動通過裝飾器註冊
```

### 資料庫操作函數
```python
@flow_function(
    category="query",
    description="查詢特定資料",
    params=["table", "filter"],
    returns="array"
)
async def query_custom_data(self, params):
    """查詢自定義資料"""
    from .database import db_manager
    
    table = params.get('table')
    filter_dict = params.get('filter', {})
    
    # 使用 db_manager 查詢
    with db_manager.get_session() as session:
        # 執行查詢
        results = session.query(table).filter_by(**filter_dict).all()
        
        # 轉換為字典列表
        return [row.to_dict() for row in results]
```

### 外部服務整合函數
```python
@flow_function(
    category="action",
    description="呼叫外部服務",
    params=["service", "method", "data"],
    returns="dict"
)
async def call_external_service(self, params):
    """呼叫外部服務"""
    import aiohttp
    
    service = params.get('service')
    method = params.get('method', 'GET')
    data = params.get('data', {})
    
    async with aiohttp.ClientSession() as session:
        if method == 'GET':
            async with session.get(service, params=data) as response:
                return await response.json()
        elif method == 'POST':
            async with session.post(service, json=data) as response:
                return await response.json()
```

## 🧪 測試開發

### 單元測試
```python
# /app/flow_wcs_ws/test/test_custom_function.py

import pytest
from flow_wcs.flow_executor import FlowExecutor

@pytest.mark.asyncio
async def test_custom_function():
    """測試自定義函數"""
    # 創建測試流程
    test_flow = {
        'meta': {'system': 'linear_flow_v2'},
        'flow': {'id': 'test'},
        'workflow': [{
            'section': 'Test',
            'steps': [{
                'id': 'test_custom',
                'exec': 'custom.my_custom_function',
                'params': {
                    'param1': 'test_value'
                },
                'store': 'result'
            }]
        }]
    }
    
    # 執行流程
    executor = FlowExecutor(test_flow)
    context = await executor.execute()
    
    # 驗證結果
    assert context['status'] == 'completed'
    assert 'result' in context['variables']
    assert context['variables']['result']['success'] == True
```

### 整合測試
```bash
# 執行整合測試
cd /app/flow_wcs_ws
python3 test_integration.py

# 執行特定測試
python3 -m pytest test/test_flow_executor.py -v
```

### 流程測試
```yaml
# 測試流程檔案
meta:
  system: linear_flow_v2
  version: "2.0.0"
  
flow:
  id: "test_flow"
  name: "測試流程"
  work_id: "999999"
  enabled: true
  
workflow:
  - section: "測試變數"
    steps:
      - id: "set_var"
        exec: "action.log"
        params:
          message: "設定變數"
        store: "test_var"
      
      - id: "use_var"
        exec: "action.log"
        params:
          message: "使用變數: ${test_var}"
```

## 🔍 除錯技巧

### 日誌除錯
```python
# 在函數中加入日誌
@flow_function(category="debug", description="除錯函數")
async def debug_function(self, params):
    import logging
    logger = logging.getLogger(__name__)
    
    logger.debug(f"接收參數: {params}")
    logger.info("執行除錯函數")
    
    try:
        # 函數邏輯
        result = process_data(params)
        logger.info(f"處理成功: {result}")
        return result
    except Exception as e:
        logger.error(f"處理失敗: {e}")
        raise
```

### ROS 2 主題監控
```bash
# 監控流程執行
ros2 topic echo /flow_wcs/events

# 監控函數呼叫
ros2 topic echo /flow_wcs/function_calls

# 監控錯誤
ros2 topic echo /flow_wcs/errors
```

### 資料庫查詢
```sql
-- 查看最近的流程執行
SELECT * FROM flow_logs 
ORDER BY created_at DESC 
LIMIT 10;

-- 查看特定流程的執行歷史
SELECT * FROM flow_logs 
WHERE flow_id = 'my_flow' 
ORDER BY created_at DESC;

-- 查看錯誤記錄
SELECT * FROM flow_logs 
WHERE status = 'failed' 
ORDER BY created_at DESC;
```

## 📦 部署流程

### 開發環境部署
```bash
# 建置
cd /app/flow_wcs_ws
colcon build --packages-select flow_wcs

# 部署流程檔案
cp my_flow.yaml /app/config/wcs/flows/

# 重啟服務
./deploy.sh restart
```

### 生產環境部署
```bash
# 完整部署
./deploy.sh full

# 驗證部署
./deploy.sh status

# 檢查流程載入
curl http://localhost:8000/api/flow/list
```

## 🛡️ 服務管理和生命週期

### 優雅關閉 (Graceful Shutdown)
Flow WCS 節點支援優雅關閉，可以正確處理 Ctrl+C 信號：

```python
# flow_wcs_node_simple.py 實作優雅關閉
import signal
import sys
import threading

class FlowWCSNode(Node):
    def __init__(self):
        super().__init__('flow_wcs_node')
        # 關閉標誌
        self.is_shutting_down = False
        self.shutdown_event = threading.Event()
    
    def cleanup(self):
        """清理資源"""
        self.get_logger().info("Starting graceful shutdown...")
        self.is_shutting_down = True
        self.shutdown_event.set()
        
        # 取消定時器
        if hasattr(self, 'scan_timer'):
            self.scan_timer.cancel()
            self.destroy_timer(self.scan_timer)
        
        # 等待活動流程完成（最多10秒）
        if self.active_executions:
            self.get_logger().info(f"Waiting for {len(self.active_executions)} active flows...")
            timeout = 10.0
            start_time = datetime.now()
            while self.active_executions and (datetime.now() - start_time).total_seconds() < timeout:
                time.sleep(0.1)
        
        # 關閉線程池
        if hasattr(self, 'thread_executor'):
            self.thread_executor.shutdown(wait=False)

def main(args=None):
    # 信號處理器
    def signal_handler(signum, frame):
        print("\n[INFO] Received shutdown signal, cleaning up...")
        if node:
            node.cleanup()
        if executor:
            executor.shutdown(timeout_sec=0.1)
        if rclpy.ok():
            rclpy.shutdown()
        sys.exit(0)
    
    # 註冊信號處理
    signal.signal(signal.SIGINT, signal_handler)   # Ctrl+C
    signal.signal(signal.SIGTERM, signal_handler)  # 終止信號
```

### 服務管理腳本
使用 `manage_flow_wcs` 腳本管理服務：

```bash
# 服務管理命令
manage_flow_wcs start    # 啟動服務
manage_flow_wcs stop     # 停止服務（使用 SIGTERM）
manage_flow_wcs restart  # 重啟服務
manage_flow_wcs status   # 檢查狀態

# 實作在 setup.bash 中：
manage_flow_wcs() {
    case "$1" in
        start)
            ros2 run flow_wcs flow_wcs_node &
            echo $! > /tmp/flow_wcs.pid
            ;;
        stop)
            if [ -f /tmp/flow_wcs.pid ]; then
                kill -TERM $(cat /tmp/flow_wcs.pid)
                rm /tmp/flow_wcs.pid
            fi
            ;;
        restart)
            manage_flow_wcs stop
            sleep 2
            manage_flow_wcs start
            ;;
        status)
            if [ -f /tmp/flow_wcs.pid ] && kill -0 $(cat /tmp/flow_wcs.pid) 2>/dev/null; then
                echo "✅ Flow WCS is running (PID: $(cat /tmp/flow_wcs.pid))"
            else
                echo "❌ Flow WCS is not running"
            fi
            ;;
    esac
}
```

### 變數解析和表達式處理
Flow WCS 支援複雜的變數表達式解析：

```python
# 變數表達式解析改進
def resolve_variable_expressions(self, value: str, context: Dict) -> Any:
    """解析變數表達式（支援陣列長度等）"""
    if not isinstance(value, str):
        return value
    
    pattern = r'\$\{([^}]+)\}'
    
    def replacer(match):
        expr = match.group(1)
        
        # 處理 .length 表達式
        if '.length' in expr:
            var_name = expr.replace('.length', '')
            if var_name in context.get('variables', {}):
                var_value = context['variables'][var_name]
                if isinstance(var_value, (list, tuple)):
                    return str(len(var_value))
        
        # 處理陣列索引
        if '[' in expr and ']' in expr:
            var_name = expr.split('[')[0]
            index = int(expr.split('[')[1].split(']')[0])
            if var_name in context.get('variables', {}):
                var_value = context['variables'][var_name]
                if isinstance(var_value, (list, tuple)) and index < len(var_value):
                    return json.dumps(var_value[index])
        
        # 直接變數替換
        if expr in context.get('variables', {}):
            return json.dumps(context['variables'][expr])
        
        return match.group(0)  # 保留原始表達式
    
    return re.sub(pattern, replacer, value)
```

## 🚨 常見問題

### 流程不執行
1. 檢查 `enabled: true`
2. 確認 `work_id` 唯一
3. 查看錯誤日誌
4. 檢查節點是否正在運行：`manage_flow_wcs status`

### 變數解析失敗
1. 確認變數名稱正確
2. 檢查變數是否已儲存
3. 使用正確格式：
   - 基本變數：`${variable}`
   - 陣列長度：`${array.length}`
   - 陣列索引：`${array[0]}`
4. 確保變數在使用前已經設定

### 函數找不到
1. 確認函數已註冊
2. 檢查函數名稱格式
3. 重新建置套件

### 資料庫連接失敗
1. 檢查連接字串
2. 確認資料庫服務運行
3. 驗證網路連接

### 節點無法優雅關閉
1. 檢查是否有長時間運行的流程
2. 查看是否有資源未正確釋放
3. 使用 `manage_flow_wcs stop` 而非 `kill -9`
4. 檢查日誌中的清理訊息

## 💡 最佳實踐

### 程式碼組織
- 函數分類放置
- 使用裝飾器註冊
- 保持函數單一職責
- 適當的錯誤處理

### 流程設計
- 模組化設計
- 重用通用流程
- 適當的日誌記錄
- 完整的錯誤處理

### 測試策略
- 單元測試覆蓋
- 整合測試驗證
- 流程端到端測試
- 效能測試

## 🔗 交叉引用
- 系統架構: @docs-ai/knowledge/system/flow-wcs-system.md
- WCS 設計: @docs-ai/knowledge/agv-domain/wcs-system-design.md
- 測試標準: @docs-ai/operations/development/testing-standards.md
- 模組索引: @docs-ai/context/structure/module-index.md