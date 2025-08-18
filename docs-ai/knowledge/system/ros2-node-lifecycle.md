# ROS 2 節點生命週期管理

## 🎯 適用場景
- 實作 ROS 2 節點的優雅關閉
- 處理系統信號（SIGINT, SIGTERM）
- 管理節點資源和清理
- 確保服務可靠性

## 📋 生命週期管理概念

### ROS 2 節點生命週期
```
節點生命週期
├── 初始化 (Initialization)
│   ├── rclpy.init()
│   ├── 創建節點
│   └── 設置資源
├── 運行 (Running)
│   ├── 處理回調
│   ├── 發布/訂閱
│   └── 執行任務
├── 關閉 (Shutdown)
│   ├── 信號處理
│   ├── 資源清理
│   └── 優雅退出
└── 銷毀 (Destruction)
    ├── destroy_node()
    └── rclpy.shutdown()
```

## 🛡️ 優雅關閉實作

### 基本結構
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import signal
import sys
import threading

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        
        # 關閉控制
        self.is_shutting_down = False
        self.shutdown_event = threading.Event()
        
        # 初始化資源
        self.init_resources()
    
    def init_resources(self):
        """初始化節點資源"""
        # 發布者/訂閱者
        self.publisher = self.create_publisher(MsgType, 'topic', 10)
        self.subscription = self.create_subscription(
            MsgType, 'topic', self.callback, 10)
        
        # 定時器
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        # 服務
        self.service = self.create_service(
            SrvType, 'service', self.service_callback)
    
    def cleanup(self):
        """清理資源"""
        self.get_logger().info("Starting graceful shutdown...")
        
        # 設置關閉標誌
        self.is_shutting_down = True
        self.shutdown_event.set()
        
        # 取消定時器
        if hasattr(self, 'timer'):
            self.timer.cancel()
            self.destroy_timer(self.timer)
        
        # 銷毀發布者/訂閱者
        if hasattr(self, 'publisher'):
            self.destroy_publisher(self.publisher)
        if hasattr(self, 'subscription'):
            self.destroy_subscription(self.subscription)
        
        # 銷毀服務
        if hasattr(self, 'service'):
            self.destroy_service(self.service)
        
        self.get_logger().info("Graceful shutdown completed")
```

### 主函數與信號處理
```python
def main(args=None):
    """主入口點"""
    rclpy.init(args=args)
    
    node = None
    executor = None
    
    def signal_handler(signum, frame):
        """處理關閉信號"""
        nonlocal node, executor
        
        print(f"\n[INFO] Received signal {signum}, cleaning up...")
        
        # 清理節點
        if node:
            node.cleanup()
        
        # 關閉執行器
        if executor:
            executor.shutdown(timeout_sec=0.1)
        
        # 關閉 ROS 2
        if rclpy.ok():
            rclpy.shutdown()
        
        sys.exit(0)
    
    # 註冊信號處理器
    signal.signal(signal.SIGINT, signal_handler)   # Ctrl+C
    signal.signal(signal.SIGTERM, signal_handler)  # 終止信號
    
    try:
        # 創建節點
        node = MyNode()
        
        # 創建執行器
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(node)
        
        # 運行節點
        try:
            executor.spin()
        except KeyboardInterrupt:
            print("\n[INFO] Keyboard interrupt received")
        except Exception as e:
            if node:
                node.get_logger().error(f"Unexpected error: {e}")
    
    finally:
        # 最終清理
        if node:
            try:
                node.cleanup()
                node.destroy_node()
            except Exception as e:
                print(f"[WARNING] Error during cleanup: {e}")
        
        if executor:
            try:
                executor.shutdown(timeout_sec=0.1)
            except Exception:
                pass
        
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass

if __name__ == '__main__':
    main()
```

## 🔧 進階模式

### 等待活動任務完成
```python
class TaskNode(Node):
    def __init__(self):
        super().__init__('task_node')
        self.active_tasks = {}
        self.task_executor = ThreadPoolExecutor(max_workers=5)
    
    def cleanup(self):
        """等待任務完成後清理"""
        self.get_logger().info("Waiting for active tasks...")
        
        # 設置關閉標誌
        self.is_shutting_down = True
        
        # 等待任務（帶超時）
        if self.active_tasks:
            timeout = 10.0  # 10秒超時
            start_time = time.time()
            
            while self.active_tasks:
                if time.time() - start_time > timeout:
                    self.get_logger().warning(
                        f"Timeout: Force stopping {len(self.active_tasks)} tasks")
                    break
                time.sleep(0.1)
        
        # 關閉執行器
        self.task_executor.shutdown(wait=False)
```

### 保存狀態
```python
class StatefulNode(Node):
    def __init__(self):
        super().__init__('stateful_node')
        self.state_file = '/tmp/node_state.json'
        self.load_state()
    
    def load_state(self):
        """載入之前的狀態"""
        try:
            with open(self.state_file, 'r') as f:
                self.state = json.load(f)
                self.get_logger().info("State loaded")
        except FileNotFoundError:
            self.state = {}
    
    def save_state(self):
        """保存當前狀態"""
        try:
            with open(self.state_file, 'w') as f:
                json.dump(self.state, f)
                self.get_logger().info("State saved")
        except Exception as e:
            self.get_logger().error(f"Failed to save state: {e}")
    
    def cleanup(self):
        """清理前保存狀態"""
        self.save_state()
        super().cleanup()
```

## 📊 服務管理腳本

### Bash 服務管理
```bash
#!/bin/bash
# manage_node.sh

NODE_NAME="my_node"
PID_FILE="/tmp/${NODE_NAME}.pid"
LOG_FILE="/tmp/${NODE_NAME}.log"

start() {
    if [ -f "$PID_FILE" ] && kill -0 $(cat "$PID_FILE") 2>/dev/null; then
        echo "✅ $NODE_NAME is already running"
        return 0
    fi
    
    echo "🚀 Starting $NODE_NAME..."
    ros2 run package_name node_name > "$LOG_FILE" 2>&1 &
    echo $! > "$PID_FILE"
    echo "✅ $NODE_NAME started (PID: $(cat $PID_FILE))"
}

stop() {
    if [ ! -f "$PID_FILE" ]; then
        echo "❌ $NODE_NAME is not running"
        return 1
    fi
    
    PID=$(cat "$PID_FILE")
    if kill -0 "$PID" 2>/dev/null; then
        echo "🛑 Stopping $NODE_NAME (PID: $PID)..."
        kill -TERM "$PID"  # 發送 SIGTERM 優雅關閉
        
        # 等待進程結束
        for i in {1..10}; do
            if ! kill -0 "$PID" 2>/dev/null; then
                break
            fi
            sleep 1
        done
        
        # 如果還在運行，強制終止
        if kill -0 "$PID" 2>/dev/null; then
            echo "⚠️ Force stopping $NODE_NAME"
            kill -9 "$PID"
        fi
    fi
    
    rm -f "$PID_FILE"
    echo "✅ $NODE_NAME stopped"
}

restart() {
    stop
    sleep 2
    start
}

status() {
    if [ -f "$PID_FILE" ] && kill -0 $(cat "$PID_FILE") 2>/dev/null; then
        echo "✅ $NODE_NAME is running (PID: $(cat $PID_FILE))"
        return 0
    else
        echo "❌ $NODE_NAME is not running"
        return 1
    fi
}

case "$1" in
    start|stop|restart|status)
        $1
        ;;
    *)
        echo "Usage: $0 {start|stop|restart|status}"
        exit 1
        ;;
esac
```

### Python 服務管理
```python
#!/usr/bin/env python3
# manage_node.py

import subprocess
import os
import signal
import time
import sys

class NodeManager:
    def __init__(self, node_name, package_name):
        self.node_name = node_name
        self.package_name = package_name
        self.pid_file = f"/tmp/{node_name}.pid"
        self.log_file = f"/tmp/{node_name}.log"
    
    def start(self):
        """啟動節點"""
        if self.is_running():
            print(f"✅ {self.node_name} is already running")
            return True
        
        print(f"🚀 Starting {self.node_name}...")
        
        # 啟動節點
        with open(self.log_file, 'w') as log:
            process = subprocess.Popen(
                ['ros2', 'run', self.package_name, self.node_name],
                stdout=log,
                stderr=subprocess.STDOUT
            )
        
        # 保存 PID
        with open(self.pid_file, 'w') as f:
            f.write(str(process.pid))
        
        print(f"✅ {self.node_name} started (PID: {process.pid})")
        return True
    
    def stop(self):
        """停止節點"""
        if not self.is_running():
            print(f"❌ {self.node_name} is not running")
            return False
        
        try:
            with open(self.pid_file, 'r') as f:
                pid = int(f.read())
            
            print(f"🛑 Stopping {self.node_name} (PID: {pid})...")
            
            # 發送 SIGTERM
            os.kill(pid, signal.SIGTERM)
            
            # 等待進程結束
            for _ in range(10):
                try:
                    os.kill(pid, 0)
                    time.sleep(1)
                except ProcessLookupError:
                    break
            
            # 強制終止
            try:
                os.kill(pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            
            # 刪除 PID 檔案
            os.remove(self.pid_file)
            print(f"✅ {self.node_name} stopped")
            return True
            
        except Exception as e:
            print(f"❌ Error stopping node: {e}")
            return False
    
    def restart(self):
        """重啟節點"""
        self.stop()
        time.sleep(2)
        self.start()
    
    def status(self):
        """檢查狀態"""
        if self.is_running():
            with open(self.pid_file, 'r') as f:
                pid = f.read()
            print(f"✅ {self.node_name} is running (PID: {pid})")
            return True
        else:
            print(f"❌ {self.node_name} is not running")
            return False
    
    def is_running(self):
        """檢查節點是否運行"""
        if not os.path.exists(self.pid_file):
            return False
        
        try:
            with open(self.pid_file, 'r') as f:
                pid = int(f.read())
            os.kill(pid, 0)
            return True
        except (ProcessLookupError, ValueError):
            return False

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print("Usage: manage_node.py {start|stop|restart|status}")
        sys.exit(1)
    
    manager = NodeManager('flow_wcs_node', 'flow_wcs')
    
    command = sys.argv[1]
    if command == 'start':
        manager.start()
    elif command == 'stop':
        manager.stop()
    elif command == 'restart':
        manager.restart()
    elif command == 'status':
        manager.status()
    else:
        print(f"Unknown command: {command}")
        sys.exit(1)
```

## 🧪 測試優雅關閉

### 測試腳本
```bash
#!/bin/bash
# test_graceful_shutdown.sh

echo "🧪 Testing graceful shutdown..."

# 啟動節點
echo "1. Starting node..."
ros2 run flow_wcs flow_wcs_node &
PID=$!
echo "   Node started (PID: $PID)"

# 等待節點穩定
sleep 3

# 發送 SIGINT (Ctrl+C)
echo "2. Sending SIGINT..."
kill -INT $PID

# 監控進程
echo "3. Monitoring shutdown..."
for i in {1..10}; do
    if kill -0 $PID 2>/dev/null; then
        echo "   Still running... ($i/10)"
        sleep 1
    else
        echo "   ✅ Node shut down gracefully"
        exit 0
    fi
done

echo "   ❌ Node did not shut down in time"
exit 1
```

## 💡 最佳實踐

### 設計原則
1. **快速響應**: 信號處理器應該快速執行
2. **資源清理**: 確保所有資源正確釋放
3. **狀態保存**: 關閉前保存重要狀態
4. **超時機制**: 設置合理的清理超時
5. **日誌記錄**: 記錄關閉過程

### 實作建議
- 使用 `try-finally` 確保清理執行
- 避免在信號處理器中執行長時間操作
- 測試各種關閉場景（Ctrl+C, SIGTERM, 異常）
- 實作健康檢查機制
- 使用 PID 檔案管理服務

### 常見陷阱
- 忘記註冊信號處理器
- 資源清理不完整
- 死鎖在關閉過程中
- 忽略子線程/進程清理
- 沒有處理異常情況

## 🔗 交叉引用
- Flow WCS 開發: @docs-ai/operations/development/flow-wcs-development.md
- ROS 2 開發: @docs-ai/operations/development/ros2-development.md
- 服務管理: @docs-ai/operations/tools/unified-tools.md
- 容器管理: @docs-ai/operations/deployment/container-management.md