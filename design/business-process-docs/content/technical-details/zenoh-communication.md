# Zenoh 通訊機制

## 🎯 Zenoh RMW 高效能通訊

本文檔詳細說明 RosAGV 中 Zenoh RMW 的通訊機制、配置最佳化、效能調優和故障排除。

## 📋 Zenoh 通訊概覽

### Zenoh 在 RosAGV 中的定位
```
Zenoh RMW 通訊架構
├── 🌐 網路通訊層
│   ├── Zenoh Router (Port 7447)
│   ├── 自動服務發現
│   ├── 跨網路透明通訊
│   └── QoS 服務品質保證
├── 🤖 ROS 2 中間件層
│   ├── rmw_zenohd 實作
│   ├── DDS 相容介面
│   ├── 節點生命週期管理
│   └── 主題和服務路由
└── 🔧 應用整合層
    ├── AGV 跨容器通訊
    ├── AGVC 分散式服務
    ├── 即時狀態同步
    └── 高效能資料傳輸
```

### 雙環境通訊模式
```
RosAGV 雙環境 Zenoh 通訊
AGV 車載環境 (Host 網路)
├── Zenoh Router: 0.0.0.0:7447
├── 直接硬體存取模式
├── 即時控制通訊
└── 低延遲要求 (< 1ms)
    ↕️ Zenoh 網路通訊
AGVC 管理環境 (Bridge 網路)
├── Zenoh Router: 192.168.100.100:7447
├── 企業級隔離模式
├── 管理服務通訊
└── 高吞吐量要求 (> 1GB/s)
```

## 🔧 Zenoh 配置最佳化

### 核心配置檔案
```json5
// /app/routerconfig.json5
{
  // 運行模式配置
  "mode": "router",
  
  // 網路監聽配置
  "listen": {
    "endpoints": [
      "tcp/0.0.0.0:7447"  // 監聽所有介面的 7447 端口
    ],
    "timeout": 10000,      // 連接超時 (毫秒)
    "backlog": 100         // 連接佇列長度
  },
  
  // 連接配置
  "connect": {
    "endpoints": [
      // 自動發現其他 Zenoh Router
      // 可以在這裡添加靜態端點
    ]
  },
  
  // 服務發現配置
  "scouting": {
    "multicast": {
      "enabled": true,
      "address": "224.0.0.224:7446",
      "interface": "auto",
      "ttl": 1
    },
    "gossip": {
      "enabled": true,
      "multihop": false
    }
  },
  
  // 路由配置
  "routing": {
    "face": {
      "unicast": {
        "accept_timeout": 10000,
        "max_sessions": 1000,
        "max_links": 1
      }
    },
    "peers": {
      // 對等節點配置
      "connect_timeout": 5000,
      "heartbeat": 1000
    }
  },
  
  // 傳輸層配置
  "transport": {
    "unicast": {
      "lowlatency": true,    // 啟用低延遲模式
      "qos": {
        "enabled": true
      },
      "compression": {
        "enabled": false     // 低延遲優先，關閉壓縮
      }
    },
    "multicast": {
      "enabled": true,
      "address": "224.0.0.225:7447",
      "interface": "auto"
    }
  },
  
  // 插件配置
  "plugins": {
    "zenoh-plugin-dds": {
      "shm": {
        "enabled": true      // 啟用共享記憶體
      }
    }
  }
}
```

### 效能最佳化配置
```json5
// 高效能配置 (生產環境)
{
  "mode": "router",
  
  // 高效能網路配置
  "transport": {
    "unicast": {
      "lowlatency": true,
      "qos": {
        "enabled": true,
        "unicast": {
          "reliability": "reliable",
          "congestion_control": "block"
        }
      },
      "compression": {
        "enabled": false    // 關閉壓縮以獲得最低延遲
      }
    }
  },
  
  // 最佳化路由配置
  "routing": {
    "face": {
      "unicast": {
        "accept_timeout": 5000,
        "max_sessions": 2000,  // 增加最大會話數
        "max_links": 4         // 增加連結數
      }
    }
  },
  
  // 記憶體最佳化
  "plugins": {
    "zenoh-plugin-dds": {
      "shm": {
        "enabled": true,
        "size": "1GB"        // 增加共享記憶體大小
      }
    }
  }
}
```

## 🚀 效能特性和調優

### 效能基準測試
```python
# Zenoh 效能測試工具
import time
import statistics
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ZenohPerformanceTest(Node):
    def __init__(self):
        super().__init__('zenoh_perf_test')
        
        # 延遲測試
        self.latency_publisher = self.create_publisher(String, 'latency_test', 10)
        self.latency_subscriber = self.create_subscription(
            String, 'latency_test', self.latency_callback, 10)
        
        # 吞吐量測試
        self.throughput_publisher = self.create_publisher(String, 'throughput_test', 10)
        
        self.latency_measurements = []
        self.throughput_counter = 0
        self.start_time = None
        
    def test_latency(self, iterations=1000):
        """測試往返延遲"""
        self.get_logger().info('開始延遲測試...')
        
        for i in range(iterations):
            start_time = time.time_ns()
            msg = String()
            msg.data = f"latency_test_{i}_{start_time}"
            self.latency_publisher.publish(msg)
            
            # 等待回應 (在實際測試中需要更複雜的同步機制)
            time.sleep(0.001)
        
        if self.latency_measurements:
            avg_latency = statistics.mean(self.latency_measurements)
            min_latency = min(self.latency_measurements)
            max_latency = max(self.latency_measurements)
            
            self.get_logger().info(f'延遲測試結果:')
            self.get_logger().info(f'  平均延遲: {avg_latency:.3f} ms')
            self.get_logger().info(f'  最小延遲: {min_latency:.3f} ms')
            self.get_logger().info(f'  最大延遲: {max_latency:.3f} ms')
    
    def test_throughput(self, duration=60, message_size=1024):
        """測試吞吐量"""
        self.get_logger().info(f'開始吞吐量測試 ({duration}秒)...')
        
        # 生成測試資料
        test_data = 'x' * message_size
        
        self.start_time = time.time()
        end_time = self.start_time + duration
        
        while time.time() < end_time:
            msg = String()
            msg.data = f"{self.throughput_counter}:{test_data}"
            self.throughput_publisher.publish(msg)
            self.throughput_counter += 1
        
        elapsed_time = time.time() - self.start_time
        messages_per_second = self.throughput_counter / elapsed_time
        bytes_per_second = (self.throughput_counter * message_size) / elapsed_time
        
        self.get_logger().info(f'吞吐量測試結果:')
        self.get_logger().info(f'  訊息/秒: {messages_per_second:.2f}')
        self.get_logger().info(f'  MB/秒: {bytes_per_second / 1024 / 1024:.2f}')
    
    def latency_callback(self, msg):
        """延遲測試回調"""
        if msg.data.startswith('latency_test_'):
            parts = msg.data.split('_')
            if len(parts) >= 3:
                start_time = int(parts[2])
                current_time = time.time_ns()
                latency_ns = current_time - start_time
                latency_ms = latency_ns / 1_000_000
                
                self.latency_measurements.append(latency_ms)
```

### 效能監控
```python
# Zenoh 效能監控
class ZenohMonitor:
    def __init__(self):
        self.metrics = {
            'connection_count': 0,
            'message_rate': 0.0,
            'byte_rate': 0.0,
            'error_rate': 0.0,
            'avg_latency': 0.0
        }
        
    def collect_metrics(self):
        """收集 Zenoh 效능指標"""
        # 檢查 Zenoh Router 狀態
        router_status = self.check_router_status()
        
        # 統計連接數
        self.metrics['connection_count'] = self.count_active_connections()
        
        # 統計訊息速率
        self.metrics['message_rate'] = self.calculate_message_rate()
        
        # 統計位元組速率
        self.metrics['byte_rate'] = self.calculate_byte_rate()
        
        # 計算錯誤率
        self.metrics['error_rate'] = self.calculate_error_rate()
        
        return self.metrics
    
    def check_router_status(self):
        """檢查 Zenoh Router 狀態"""
        try:
            # 檢查進程是否運行
            import psutil
            for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
                if 'zenohd' in str(proc.info['cmdline']):
                    return {
                        'status': 'running',
                        'pid': proc.info['pid'],
                        'memory': proc.memory_info().rss / 1024 / 1024,  # MB
                        'cpu': proc.cpu_percent()
                    }
            
            return {'status': 'not_running'}
            
        except Exception as e:
            return {'status': 'error', 'message': str(e)}
```

## 🔍 診斷和故障排除

### Zenoh 診斷工具
```bash
#!/bin/bash
# zenoh-diagnostic.sh - Zenoh 系統診斷工具

echo "=== Zenoh 系統診斷 ==="

# 1. 檢查 Zenoh Router 進程
echo "1. 檢查 Zenoh Router 進程..."
if pgrep -f zenohd > /dev/null; then
    echo "✅ Zenoh Router 正在運行"
    echo "   PID: $(pgrep -f zenohd)"
    echo "   記憶體使用: $(ps -o pid,ppid,rss,comm -p $(pgrep -f zenohd) | tail -n +2 | awk '{print $3/1024 " MB"}')"
else
    echo "❌ Zenoh Router 未運行"
fi

# 2. 檢查端口監聽
echo "2. 檢查端口監聽..."
if ss -tulpn | grep -q ":7447"; then
    echo "✅ 端口 7447 正在監聽"
    ss -tulpn | grep ":7447"
else
    echo "❌ 端口 7447 未監聽"
fi

# 3. 檢查配置檔案
echo "3. 檢查配置檔案..."
if [ -f "/app/routerconfig.json5" ]; then
    echo "✅ 配置檔案存在"
    if command -v json5 > /dev/null; then
        if json5 --validate /app/routerconfig.json5 2>/dev/null; then
            echo "✅ 配置檔案語法正確"
        else
            echo "❌ 配置檔案語法錯誤"
        fi
    else
        echo "⚠️  無法驗證 JSON5 語法 (json5 工具未安裝)"
    fi
else
    echo "❌ 配置檔案不存在"
fi

# 4. 檢查網路連接
echo "4. 檢查網路連接..."
if timeout 3 bash -c "echo > /dev/tcp/localhost/7447" 2>/dev/null; then
    echo "✅ 本地 Zenoh 連接正常"
else
    echo "❌ 本地 Zenoh 連接失敗"
fi

# 5. 檢查 RMW 設定
echo "5. 檢查 RMW 設定..."
if [ "$RMW_IMPLEMENTATION" = "rmw_zenohd" ]; then
    echo "✅ RMW 設定正確: $RMW_IMPLEMENTATION"
else
    echo "❌ RMW 設定錯誤: $RMW_IMPLEMENTATION (應該是 rmw_zenohd)"
fi

# 6. 檢查 ROS 2 通訊
echo "6. 檢查 ROS 2 通訊..."
if command -v ros2 > /dev/null; then
    if ros2 topic list > /dev/null 2>&1; then
        echo "✅ ROS 2 通訊正常"
        echo "   可用主題數: $(ros2 topic list | wc -l)"
    else
        echo "❌ ROS 2 通訊異常"
    fi
else
    echo "⚠️  ROS 2 未安裝或未載入"
fi

echo "=== 診斷完成 ==="
```

### 常見問題排除

#### 1. Zenoh Router 無法啟動
```bash
# 診斷步驟
ps aux | grep zenoh
cat /tmp/zenoh_router.pid

# 檢查端口衝突
ss -tulpn | grep 7447
lsof -i :7447

# 檢查配置檔案
json5 --validate /app/routerconfig.json5

# 手動啟動進行除錯
zenohd -c /app/routerconfig.json5 --log-level DEBUG
```

#### 2. 跨環境通訊失敗
```bash
# 網路連接測試
ping 192.168.100.100  # 從 AGV 測試 AGVC
ping <AGV_IP>          # 從 AGVC 測試 AGV

# 端口連接測試
telnet 192.168.100.100 7447
nc -zv 192.168.100.100 7447

# 防火牆檢查
sudo ufw status
sudo iptables -L | grep 7447
```

#### 3. ROS 2 服務發現問題
```bash
# 重置 ROS 2 daemon
ros2 daemon stop
ros2 daemon start

# 檢查環境變數
echo $RMW_IMPLEMENTATION
echo $ROS_DOMAIN_ID

# 測試基本通訊
ros2 topic pub /test_topic std_msgs/String "data: 'test'" &
ros2 topic echo /test_topic
```

### 效能問題診斷
```python
# Zenoh 效能問題診斷
class ZenohPerformanceDiagnostic:
    def __init__(self):
        self.baseline_metrics = self.load_baseline()
        
    def diagnose_performance_issue(self):
        """診斷效能問題"""
        current_metrics = self.collect_current_metrics()
        issues = []
        
        # 檢查延遲
        if current_metrics['avg_latency'] > self.baseline_metrics['avg_latency'] * 2:
            issues.append({
                'type': 'high_latency',
                'severity': 'warning',
                'description': f"平均延遲過高: {current_metrics['avg_latency']:.2f}ms",
                'recommendation': '檢查網路配置和系統負載'
            })
        
        # 檢查吞吐量
        if current_metrics['throughput'] < self.baseline_metrics['throughput'] * 0.5:
            issues.append({
                'type': 'low_throughput',
                'severity': 'critical',
                'description': f"吞吐量過低: {current_metrics['throughput']:.2f} MB/s",
                'recommendation': '檢查網路頻寬和 Zenoh 配置'
            })
        
        # 檢查連接數
        if current_metrics['connections'] > 1000:
            issues.append({
                'type': 'too_many_connections',
                'severity': 'warning',
                'description': f"連接數過多: {current_metrics['connections']}",
                'recommendation': '考慮增加 max_sessions 配置'
            })
        
        return issues
    
    def generate_optimization_suggestions(self, issues):
        """生成最佳化建議"""
        suggestions = []
        
        for issue in issues:
            if issue['type'] == 'high_latency':
                suggestions.extend([
                    '啟用 lowlatency 模式',
                    '關閉壓縮功能',
                    '調整 QoS 設定',
                    '檢查網路設定'
                ])
            elif issue['type'] == 'low_throughput':
                suggestions.extend([
                    '增加 max_sessions 數量',
                    '啟用共享記憶體',
                    '調整網路緩衝區大小',
                    '使用批量傳輸'
                ])
        
        return list(set(suggestions))  # 去重
```

## 🔧 進階配置

### 多網路介面配置
```json5
// 多網路介面 Zenoh 配置
{
  "mode": "router",
  
  // 多端點監聽
  "listen": {
    "endpoints": [
      "tcp/0.0.0.0:7447",        // 主要介面
      "tcp/192.168.1.100:7448",  // 內部網路
      "tcp/10.0.0.100:7449"      // 管理網路
    ]
  },
  
  // 多網路發現
  "scouting": {
    "multicast": {
      "enabled": true,
      "address": "224.0.0.224:7446",
      "interface": "eth0"  // 指定網路介面
    }
  }
}
```

### 安全配置
```json5
// Zenoh 安全配置
{
  "mode": "router",
  
  // TLS 加密
  "transport": {
    "unicast": {
      "tls": {
        "enabled": true,
        "server_private_key": "/etc/zenoh/server.key",
        "server_certificate": "/etc/zenoh/server.crt",
        "ca_certificate": "/etc/zenoh/ca.crt"
      }
    }
  },
  
  // 存取控制
  "access_control": {
    "enabled": true,
    "rules": [
      {
        "permission": "allow",
        "flows": ["egress", "ingress"],
        "key_expr": "/agv/**"
      },
      {
        "permission": "deny",
        "flows": ["egress", "ingress"],
        "key_expr": "/admin/**"
      }
    ]
  }
}
```

## 📊 監控和日誌

### Zenoh 日誌配置
```bash
# 設定 Zenoh 日誌等級
export RUST_LOG=zenoh=debug,zenoh_transport=info

# 啟動 Zenoh 並輸出日誌
zenohd -c /app/routerconfig.json5 2>&1 | tee /tmp/zenoh_router.log

# 日誌分析
tail -f /tmp/zenoh_router.log | grep -E "(ERROR|WARN|INFO)"
```

### 整合監控系統
```python
# Prometheus 監控整合
from prometheus_client import start_http_server, Counter, Histogram, Gauge

class ZenohPrometheusExporter:
    def __init__(self):
        # 定義監控指標
        self.message_counter = Counter('zenoh_messages_total', 'Total messages processed')
        self.latency_histogram = Histogram('zenoh_latency_seconds', 'Message latency')
        self.connection_gauge = Gauge('zenoh_connections_active', 'Active connections')
        
        # 啟動 HTTP 服務器
        start_http_server(8090)
    
    def record_message(self, latency):
        """記錄訊息處理"""
        self.message_counter.inc()
        self.latency_histogram.observe(latency)
    
    def update_connections(self, count):
        """更新連接數"""
        self.connection_gauge.set(count)
```

---

**相關文檔：**
- [雙環境架構](../system-architecture/dual-environment.md) - Zenoh 網路架構
- [故障排除](../operations/troubleshooting.md) - Zenoh 問題診斷
- [效能調優](../operations/performance-tuning.md) - 系統效能最佳化
- [技術架構](../system-architecture/technology-stack.md) - 技術選型背景