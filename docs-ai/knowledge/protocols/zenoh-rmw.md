# Zenoh RMW 通訊機制

## 🎯 適用場景
- 理解 RosAGV 中 Zenoh RMW 的通訊機制
- 解決跨容器和跨網路的 ROS 2 通訊問題
- 最佳化 Zenoh 配置和效能調整

## 📋 Zenoh RMW 概述

### 什麼是 Zenoh RMW
Zenoh RMW (Robot Middleware) 是 ROS 2 的一個高效能中間件實作，基於 Zenoh 協定提供：
- **高效能通訊**: 低延遲、高吞吐量的資料傳輸
- **網路透明**: 無縫的跨網路通訊能力
- **自動發現**: 自動的服務和節點發現機制
- **可擴展性**: 支援大規模分散式系統

### 在 RosAGV 中的角色
```
RosAGV 通訊架構
├── AGV 車載系統 (Host 網路)
│   ├── Zenoh Router: 0.0.0.0:7447
│   └── ROS 2 節點 (rmw_zenohd)
├── AGVC 管理系統 (Bridge 網路)
│   ├── Zenoh Router: 192.168.100.100:7447
│   └── ROS 2 節點 (rmw_zenohd)
└── 跨環境通訊
    ├── 自動服務發現
    ├── 主題資料同步
    └── 服務呼叫轉發
```

## 🔧 Zenoh 配置

### 配置檔案位置
- **主配置**: `/app/routerconfig.json5`
- **格式**: JSON5 (支援註解的 JSON)
- **作用範圍**: 兩個環境共用相同配置

### 基本配置結構
```json5
{
  // Zenoh Router 配置
  mode: "router",
  
  // 監聽配置
  listen: {
    endpoints: [
      "tcp/0.0.0.0:7447"
    ]
  },
  
  // 連接配置
  connect: {
    endpoints: [
      // 自動發現其他 Zenoh Router
    ]
  },
  
  // 路由配置
  routing: {
    peers: {
      // 對等節點配置
    }
  },
  
  // 插件配置
  plugins: {
    // ROS 2 DDS 橋接
    "zenoh-plugin-dds": {
      // DDS 相關配置
    }
  }
}
```

### 關鍵配置參數
```json5
{
  // 網路配置
  "listen": {
    "endpoints": ["tcp/0.0.0.0:7447"],
    "timeout": 10000,
    "backlog": 100
  },
  
  // 發現配置
  "scouting": {
    "multicast": {
      "enabled": true,
      "address": "224.0.0.224:7446",
      "interface": "auto"
    }
  },
  
  // 效能配置
  "transport": {
    "unicast": {
      "lowlatency": true,
      "qos": {
        "enabled": true
      }
    }
  }
}
```

## 🌐 網路通訊機制

### 通訊模式
```
Zenoh 通訊模式
├── Pub/Sub (發布/訂閱)
│   ├── ROS 2 Topic → Zenoh Key Expression
│   ├── 自動資料路由
│   └── QoS 保證
├── Query/Reply (查詢/回應)
│   ├── ROS 2 Service → Zenoh Query
│   ├── 同步/異步呼叫
│   └── 錯誤處理
└── Storage (儲存)
    ├── 資料持久化
    ├── 歷史資料查詢
    └── 狀態同步
```

### 資料路由
```
資料路由流程
AGV 節點發布 → Zenoh Router (AGV) → 網路傳輸 → Zenoh Router (AGVC) → AGVC 節點接收
     ↑                                                                              ↓
ROS 2 Topic                                                                ROS 2 Topic
```

### 服務發現
```
服務發現機制
1. 節點啟動 → 註冊到本地 Zenoh Router
2. Zenoh Router → 廣播服務資訊
3. 遠端 Router → 接收並轉發
4. 遠端節點 → 自動發現可用服務
```

## 🚀 效能特性

### 延遲特性
- **本地通訊**: < 100μs
- **跨容器通訊**: < 1ms
- **跨網路通訊**: < 10ms (區域網路)
- **廣域網路**: 取決於網路延遲

### 吞吐量特性
- **小訊息**: > 100k msg/s
- **大訊息**: > 1GB/s (區域網路)
- **並發連接**: > 10k 同時連接
- **記憶體使用**: 低記憶體佔用

### QoS 支援
```
QoS 等級對應
ROS 2 QoS → Zenoh QoS
├── Reliability
│   ├── RELIABLE → Reliable
│   └── BEST_EFFORT → BestEffort
├── Durability
│   ├── TRANSIENT_LOCAL → Cached
│   └── VOLATILE → Volatile
└── History
    ├── KEEP_LAST → KeepLast(n)
    └── KEEP_ALL → KeepAll
```

## 🔍 診斷和監控

### Zenoh 狀態檢查
```bash
# 檢查 Zenoh Router 運行狀態
ps aux | rg zenoh
cat /tmp/zenoh_router.pid
pgrep -f rmw_zenohd

# 檢查 Zenoh 連接 (推薦使用 ss)
ss -tulpn | rg 7447

# 備選：netstat (舊工具)
netstat -tulpn | rg 7447
```

### 連接診斷
```bash
# 測試 Zenoh 端口連接
telnet localhost 7447
nc -zv localhost 7447

# 跨環境連接測試
# 在 AGV 容器中
telnet 192.168.100.100 7447

# 在 AGVC 容器中
telnet <AGV_IP> 7447
```

### ROS 2 通訊測試
```bash
# 檢查 RMW 實作
echo $RMW_IMPLEMENTATION
# 應該顯示: rmw_zenohd

# 測試跨環境主題通訊
# 在一個環境中發布
ros2 topic pub /test_topic std_msgs/String "data: 'Hello Zenoh'"

# 在另一個環境中訂閱
ros2 topic echo /test_topic

# 檢查主題列表
ros2 topic list
```

## 🛠️ 故障排除

### 常見問題

#### Zenoh Router 無法啟動
```bash
# 檢查端口佔用
ss -tulpn | rg 7447
lsof -i :7447

# 檢查配置檔案
cat /app/routerconfig.json5
# 驗證 JSON5 語法正確性

# 手動啟動 Zenoh Router
zenohd -c /app/routerconfig.json5
```

#### 跨環境通訊失敗
```bash
# 檢查網路連接
ping <target_ip>
telnet <target_ip> 7447

# 檢查防火牆設定
iptables -L
ufw status

# 檢查 Docker 網路
docker network ls
docker network inspect <network_name>
```

#### ROS 2 節點無法發現
```bash
# 檢查 RMW 設定
echo $RMW_IMPLEMENTATION

# 重新設定 RMW
export RMW_IMPLEMENTATION=rmw_zenohd

# 檢查 Zenoh 服務發現
ros2 daemon stop
ros2 daemon start
```

### 效能問題診斷
```bash
# 檢查 Zenoh 效能
# 測試延遲
ros2 topic hz /topic_name

# 測試頻寬
ros2 topic bw /topic_name

# 檢查系統資源
top
htop
iotop
```

## ⚙️ 最佳化配置

### 低延遲最佳化
```json5
{
  "transport": {
    "unicast": {
      "lowlatency": true,
      "qos": {
        "enabled": true
      }
    }
  },
  "routing": {
    "face": {
      "unicast": {
        "accept_timeout": 1000,
        "max_sessions": 1000
      }
    }
  }
}
```

### 高吞吐量最佳化
```json5
{
  "transport": {
    "unicast": {
      "qos": {
        "enabled": true
      },
      "compression": {
        "enabled": true
      }
    }
  },
  "plugins": {
    "zenoh-plugin-dds": {
      "shm": {
        "enabled": true
      }
    }
  }
}
```

### 可靠性最佳化
```json5
{
  "transport": {
    "unicast": {
      "qos": {
        "enabled": true
      }
    }
  },
  "routing": {
    "face": {
      "unicast": {
        "max_retries": 3,
        "retry_period": 1000
      }
    }
  }
}
```

## 🔧 開發指導

### ROS 2 節點開發
```python
# 確保使用 Zenoh RMW
import os
os.environ['RMW_IMPLEMENTATION'] = 'rmw_zenohd'

import rclpy
from rclpy.node import Node

class ZenohNode(Node):
    def __init__(self):
        super().__init__('zenoh_node')
        
        # 設定 QoS
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            depth=10
        )
        
        # 建立發布者和訂閱者
        self.publisher = self.create_publisher(MsgType, 'topic', qos)
        self.subscription = self.create_subscription(
            MsgType, 'topic', self.callback, qos)
```

### 跨環境服務
```python
# 服務伺服器 (在一個環境中)
class ServiceServer(Node):
    def __init__(self):
        super().__init__('service_server')
        self.service = self.create_service(
            ServiceType, 'cross_env_service', self.service_callback)
    
    def service_callback(self, request, response):
        # 處理跨環境服務請求
        return response

# 服務客戶端 (在另一個環境中)
class ServiceClient(Node):
    def __init__(self):
        super().__init__('service_client')
        self.client = self.create_client(ServiceType, 'cross_env_service')
    
    async def call_service(self, request):
        # 呼叫跨環境服務
        future = self.client.call_async(request)
        return await future
```

## 📋 最佳實踐

### 配置管理
1. **統一配置**: 使用相同的 Zenoh 配置檔案
2. **版本控制**: 將配置檔案納入版本控制
3. **環境適配**: 根據部署環境調整網路參數
4. **監控配置**: 定期檢查配置的有效性

### 效能最佳化
1. **QoS 設定**: 根據應用需求設定適當的 QoS
2. **網路調整**: 最佳化網路參數和防火牆設定
3. **資源監控**: 監控 CPU、記憶體和網路使用
4. **負載均衡**: 在多個 Router 間分散負載

### 故障恢復
1. **自動重連**: 實作自動重連機制
2. **健康檢查**: 定期檢查 Zenoh 連接狀態
3. **降級策略**: 在通訊失敗時的降級處理
4. **日誌記錄**: 詳細記錄通訊事件和錯誤

## 🔗 交叉引用
- 雙環境架構: docs-ai/context/system/dual-environment.md
- 網路診斷: docs-ai/operations/guides/system-diagnostics.md
- ROS 2 開發: docs-ai/operations/development/ros2/ros2-development.md
- 容器管理: docs-ai/operations/deployment/container-management.md
- 技術棧: docs-ai/context/system/technology-stack.md
