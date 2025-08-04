# 效能調優

## 概述

RosAGV 系統基於 Zenoh RMW 通訊機制，提供高效能的跨容器和跨網路通訊能力。本指南涵蓋 Zenoh 通訊最佳化、系統資源調整和整體效能調優策略。

## Zenoh RMW 效能最佳化

### 基礎效能特性

#### 延遲特性基準
```
Zenoh RMW 延遲效能
├── 本地通訊: < 100μs
├── 跨容器通訊: < 1ms
├── 跨網路通訊: < 10ms (區域網路)
└── 廣域網路: 取決於網路延遲
```

#### 吞吐量特性基準
```
Zenoh RMW 吞吐量效能
├── 小訊息 (< 1KB): > 100k msg/s
├── 大訊息 (> 1MB): > 1GB/s (區域網路)
├── 並發連接: > 10k 同時連接
└── 記憶體使用: 低記憶體佔用設計
```

### Zenoh 配置最佳化

#### 低延遲配置
```json5
{
  // 低延遲最佳化配置
  "transport": {
    "unicast": {
      "lowlatency": true,           // 啟用低延遲模式
      "qos": {
        "enabled": true             // 啟用 QoS 保證
      }
    }
  },
  "routing": {
    "face": {
      "unicast": {
        "accept_timeout": 1000,     // 1秒連接超時
        "max_sessions": 1000        // 最大會話數
      }
    }
  },
  // 發現機制最佳化
  "scouting": {
    "multicast": {
      "enabled": true,
      "address": "224.0.0.224:7446",
      "interface": "auto"
    }
  }
}
```

#### 高吞吐量配置
```json5
{
  // 高吞吐量最佳化配置
  "transport": {
    "unicast": {
      "qos": {
        "enabled": true
      },
      "compression": {
        "enabled": true             // 啟用資料壓縮
      }
    }
  },
  "plugins": {
    "zenoh-plugin-dds": {
      "shm": {
        "enabled": true             // 啟用共享記憶體
      }
    }
  },
  // 緩衝區最佳化
  "transport": {
    "unicast": {
      "tx_buffer_size": 65536,      // 傳送緩衝區 64KB
      "rx_buffer_size": 65536       // 接收緩衝區 64KB
    }
  }
}
```

#### 可靠性配置
```json5
{
  // 可靠性最佳化配置
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
        "max_retries": 3,           // 最大重試次數
        "retry_period": 1000        // 重試間隔 1秒
      }
    }
  },
  // 心跳檢測
  "transport": {
    "unicast": {
      "keep_alive": 30000           // 30秒心跳
    }
  }
}
```

### QoS 最佳化策略

#### ROS 2 QoS 映射最佳化
```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

# 即時控制最佳化 QoS
REALTIME_CONTROL_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,  # 低延遲優先
    durability=DurabilityPolicy.VOLATILE,       # 不需要持久化
    history=HistoryPolicy.KEEP_LAST,           # 只保留最新
    depth=1                                    # 最小佇列深度
)

# 關鍵資料最佳化 QoS
CRITICAL_DATA_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,     # 可靠傳輸
    durability=DurabilityPolicy.TRANSIENT_LOCAL, # 本地持久化
    history=HistoryPolicy.KEEP_ALL,            # 保留所有資料
    depth=100                                  # 適中佇列大小
)

# 高頻感測器最佳化 QoS
HIGH_FREQ_SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5                                    # 小佇列，防止累積
)
```

#### 應用場景 QoS 選擇
```python
class RosAGVQoSManager:
    @staticmethod
    def get_agv_status_qos():
        """AGV 狀態發布 QoS"""
        return QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
    
    @staticmethod
    def get_sensor_data_qos():
        """感測器資料 QoS"""
        return QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
    
    @staticmethod
    def get_command_qos():
        """控制指令 QoS"""
        return QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_ALL,
            depth=50
        )
```

## 系統資源最佳化

### 容器資源配置

#### Docker Compose 資源限制
```yaml
# docker-compose.agvc.yml 資源最佳化
services:
  agvc_server:
    deploy:
      resources:
        limits:
          cpus: '4.0'              # CPU 限制
          memory: 8G               # 記憶體限制
        reservations:
          cpus: '2.0'              # CPU 保留
          memory: 4G               # 記憶體保留
    environment:
      # JVM 記憶體最佳化 (如果使用 Java 組件)
      - JAVA_OPTS=-Xmx4g -Xms2g -XX:+UseG1GC
      # Python 最佳化
      - PYTHONUNBUFFERED=1
      - PYTHONOPTIMIZE=1
    
  postgres:
    deploy:
      resources:
        limits:
          cpus: '2.0'
          memory: 4G
        reservations:
          cpus: '1.0'
          memory: 2G
    environment:
      # PostgreSQL 效能參數
      - POSTGRES_SHARED_BUFFERS=1GB
      - POSTGRES_EFFECTIVE_CACHE_SIZE=3GB
      - POSTGRES_WORK_MEM=16MB
```

#### 系統核心參數調整
```bash
# /etc/sysctl.conf 網路最佳化
# TCP 緩衝區最佳化
net.core.rmem_max = 268435456          # 256MB 接收緩衝區
net.core.wmem_max = 268435456          # 256MB 發送緩衝區
net.ipv4.tcp_rmem = 4096 87380 268435456
net.ipv4.tcp_wmem = 4096 65536 268435456

# 連接數最佳化
net.core.somaxconn = 65535             # 最大連接佇列
net.ipv4.tcp_max_syn_backlog = 65535   # SYN 佇列大小
net.core.netdev_max_backlog = 5000     # 網路設備佇列

# 應用立即生效
sudo sysctl -p
```

### PostgreSQL 效能調整

#### 核心配置最佳化
```sql
-- postgresql.conf 效能參數
-- 記憶體配置
shared_buffers = 1GB                    -- 共享緩衝區
effective_cache_size = 3GB              -- 有效快取大小
work_mem = 16MB                         -- 工作記憶體
maintenance_work_mem = 256MB            -- 維護記憶體

-- 寫入最佳化
checkpoint_completion_target = 0.9      -- 檢查點完成目標
wal_buffers = 16MB                      -- WAL 緩衝區
max_wal_size = 2GB                      -- WAL 最大大小

-- 連接最佳化
max_connections = 200                   -- 最大連接數

-- 查詢最佳化
random_page_cost = 1.1                  -- SSD 隨機讀取成本
effective_io_concurrency = 200          -- I/O 並發度
```

#### 索引策略最佳化
```sql
-- 自動建立效能索引
CREATE INDEX CONCURRENTLY IF NOT EXISTS idx_agv_status_location 
ON agvs(status, current_location_id);

CREATE INDEX CONCURRENTLY IF NOT EXISTS idx_task_priority_status_created 
ON tasks(priority DESC, status, created_at);

CREATE INDEX CONCURRENTLY IF NOT EXISTS idx_carrier_rack_status 
ON carrier(rack_id, status);

-- 部分索引最佳化
CREATE INDEX CONCURRENTLY IF NOT EXISTS idx_pending_tasks 
ON tasks(priority DESC, created_at) 
WHERE status = 'pending';

-- 表達式索引
CREATE INDEX CONCURRENTLY IF NOT EXISTS idx_location_coordinates 
ON location((x + y));  -- 用於距離計算
```

#### PostgreSQL 效能監控
```sql
-- 檢查活躍連接和鎖定情況
SELECT 
    pid,
    usename,
    application_name,
    client_addr,
    state,
    query_start,
    query
FROM pg_stat_activity 
WHERE state = 'active'
ORDER BY query_start;

-- 檢查資料庫統計資訊 (含詳細說明)
SELECT 
    datname,                           -- 資料庫名稱
    numbackends as active_connections, -- 當前活躍連接數
    xact_commit,                       -- 提交的交易總數
    xact_rollback,                     -- 回滾的交易總數  
    blks_read,                         -- 從磁碟讀取的區塊數 (慢)
    blks_hit,                          -- 從快取命中的區塊數 (快)
    temp_files,                        -- 建立的臨時檔案數
    temp_bytes,                        -- 臨時檔案使用的位元組數
    -- 計算快取命中率 (重要效能指標)
    ROUND(100.0 * blks_hit / NULLIF(blks_hit + blks_read, 0), 2) as cache_hit_ratio,
    -- 計算回滾率 (穩定性指標)  
    ROUND(100.0 * xact_rollback / NULLIF(xact_commit + xact_rollback, 0), 2) as rollback_ratio
FROM pg_stat_database 
WHERE datname = 'agvc';

-- 檢查表使用統計 (含詳細說明)
SELECT 
    schemaname,                        -- 綱要名稱
    tablename,                         -- 表格名稱
    seq_scan,                          -- 順序掃描次數 (全表掃描，效能較差)
    seq_tup_read,                      -- 順序掃描讀取的資料列數
    idx_scan,                          -- 索引掃描次數 (效能較好)
    idx_tup_fetch,                     -- 索引掃描取得的資料列數
    n_tup_ins,                         -- 插入的資料列數
    n_tup_upd,                         -- 更新的資料列數
    n_tup_del,                         -- 刪除的資料列數
    -- 計算索引使用率 (重要效能指標)
    CASE 
        WHEN (seq_scan + idx_scan) > 0 
        THEN ROUND(100.0 * idx_scan / (seq_scan + idx_scan), 2) 
        ELSE 0 
    END as index_usage_ratio
FROM pg_stat_user_tables
ORDER BY seq_scan DESC;

-- 💡 表格效能分析說明：
-- seq_scan 高 = 經常全表掃描 (需要建立索引)
-- idx_scan 高 = 索引使用良好 (效能佳)
-- index_usage_ratio > 95% = 索引使用率優秀
-- index_usage_ratio < 80% = 需要檢查索引設計

-- 檢查索引使用效率
SELECT 
    schemaname,
    tablename,
    indexname,
    idx_scan,
    idx_tup_read,
    idx_tup_fetch
FROM pg_stat_user_indexes
WHERE idx_scan = 0  -- 找出未使用的索引
ORDER BY schemaname, tablename;
```

### ROS 2 節點最佳化

#### 節點效能調整
```python
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

class OptimizedAGVNode(Node):
    def __init__(self):
        super().__init__('optimized_agv_node')
        
        # 回調群組最佳化
        self.reentrant_group = ReentrantCallbackGroup()
        self.exclusive_group = MutuallyExclusiveCallbackGroup()
        
        # 高頻感測器訂閱 (並行處理)
        self.sensor_subscription = self.create_subscription(
            SensorMsg, 'sensor_data', 
            self.sensor_callback,
            HIGH_FREQ_SENSOR_QOS,
            callback_group=self.reentrant_group
        )
        
        # 控制指令訂閱 (序列處理)
        self.command_subscription = self.create_subscription(
            CommandMsg, 'agv_commands',
            self.command_callback,
            CRITICAL_DATA_QOS,
            callback_group=self.exclusive_group
        )
        
        # 狀態發布器最佳化
        self.status_publisher = self.create_publisher(
            AGVStatus, 'agv_status',
            self.get_agv_status_qos()
        )
        
        # 定時器最佳化 (減少系統調用)
        self.status_timer = self.create_timer(
            0.1,  # 10Hz 狀態發布
            self.publish_status,
            callback_group=self.reentrant_group
        )
    
    def sensor_callback(self, msg):
        """感測器回調 - 非阻塞處理"""
        # 使用執行緒池處理耗時操作
        self.executor.create_task(self.process_sensor_data(msg))
    
    async def process_sensor_data(self, msg):
        """異步處理感測器資料"""
        # 非阻塞的資料處理邏輯
        pass
```

#### 執行器最佳化
```python
def main():
    rclpy.init()
    
    # 多執行緒執行器最佳化
    executor = MultiThreadedExecutor(num_threads=4)
    
    # 建立最佳化節點
    node = OptimizedAGVNode()
    executor.add_node(node)
    
    try:
        # 使用 executor 進行最佳化調度
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
```

## 網路最佳化

### Zenoh Router 網路調整

#### 路由器配置最佳化
```json5
{
  // 網路最佳化配置
  "listen": {
    "endpoints": [
      "tcp/0.0.0.0:7447"
    ],
    "timeout": 5000,                    // 5秒連接超時
    "backlog": 1024                     // 連接佇列大小
  },
  
  // 多播發現最佳化
  "scouting": {
    "multicast": {
      "enabled": true,
      "address": "224.0.0.224:7446",
      "interface": "auto",
      "ttl": 1,                         // TTL 最佳化
      "enabled": true
    }
  },
  
  // 路由效能最佳化
  "routing": {
    "face": {
      "unicast": {
        "accept_timeout": 5000,
        "max_sessions": 2000,           // 增加最大會話數
        "max_links": 100                // 最大連結數
      }
    }
  }
}
```

### 防火牆和網路設定

#### iptables 最佳化
```bash
#!/bin/bash
# zenoh-firewall-optimize.sh

# Zenoh 端口最佳化
iptables -A INPUT -p tcp --dport 7447 -m conntrack --ctstate NEW -m limit --limit 1000/sec -j ACCEPT
iptables -A INPUT -p udp --dport 7446 -j ACCEPT  # 多播發現

# 連接追蹤最佳化
echo 'net.netfilter.nf_conntrack_max = 1048576' >> /etc/sysctl.conf
echo 'net.netfilter.nf_conntrack_tcp_timeout_established = 7200' >> /etc/sysctl.conf

# TCP 最佳化
echo 'net.ipv4.tcp_congestion_control = bbr' >> /etc/sysctl.conf
echo 'net.core.default_qdisc = fq' >> /etc/sysctl.conf

sysctl -p
```

## 應用程式最佳化

### FastAPI 效能調整

#### Web API 最佳化
```python
from fastapi import FastAPI, Depends
from fastapi.middleware.gzip import GZipMiddleware
from fastapi.middleware.cors import CORSMiddleware
import uvicorn

# FastAPI 應用最佳化
app = FastAPI(
    title="RosAGV API",
    docs_url="/docs",
    redoc_url="/redoc",
    openapi_url="/openapi.json"
)

# 中間件最佳化
app.add_middleware(GZipMiddleware, minimum_size=1000)  # 壓縮中間件
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# 資料庫連線池最佳化
from sqlalchemy.pool import QueuePool

DATABASE_CONFIG = {
    "pool_size": 20,          # 連線池大小
    "max_overflow": 30,       # 最大溢出連線
    "pool_timeout": 30,       # 連線超時
    "pool_recycle": 3600,     # 連線回收時間
    "pool_pre_ping": True,    # 連線檢查
}

# 異步操作最佳化
@app.get("/agvs/status")
async def get_agv_status():
    """最佳化的 AGV 狀態查詢"""
    async with get_db() as session:
        # 使用索引最佳化查詢
        result = await session.exec(
            select(AGV)
            .where(AGV.status.in_(["idle", "busy"]))
            .options(selectinload(AGV.current_location))  # 預載入關聯
        )
        return result.all()

# Uvicorn 伺服器最佳化
if __name__ == "__main__":
    uvicorn.run(
        "main:app",
        host="0.0.0.0",
        port=8000,
        workers=4,              # 工作進程數
        worker_class="uvicorn.workers.UvicornWorker",
        loop="uvloop",          # 高效能事件循環
        http="httptools",       # 高效能 HTTP 解析器
        access_log=False,       # 生產環境關閉存取日誌
        reload=False            # 生產環境關閉自動重載
    )
```

### 快取策略

#### Redis 快取整合
```python
import redis.asyncio as redis
from functools import wraps
import json
import pickle

class RosAGVCache:
    def __init__(self):
        self.redis_client = redis.Redis(
            host='localhost',
            port=6379,
            db=0,
            encoding='utf-8',
            decode_responses=False,
            socket_connect_timeout=5,
            socket_timeout=5,
            retry_on_timeout=True,
            health_check_interval=30
        )
    
    async def cache_agv_status(self, agv_id: int, status_data: dict, expire: int = 60):
        """快取 AGV 狀態"""
        key = f"agv_status:{agv_id}"
        await self.redis_client.setex(
            key, 
            expire, 
            json.dumps(status_data, default=str)
        )
    
    async def get_cached_agv_status(self, agv_id: int) -> dict:
        """取得快取的 AGV 狀態"""
        key = f"agv_status:{agv_id}"
        data = await self.redis_client.get(key)
        if data:
            return json.loads(data)
        return None

# 快取裝飾器
def cache_result(expire: int = 300):
    def decorator(func):
        @wraps(func)
        async def wrapper(*args, **kwargs):
            # 生成快取鍵
            cache_key = f"{func.__name__}:{hash(str(args) + str(kwargs))}"
            
            # 嘗試從快取取得
            cached = await cache.redis_client.get(cache_key)
            if cached:
                return pickle.loads(cached)
            
            # 執行函數並快取結果
            result = await func(*args, **kwargs)
            await cache.redis_client.setex(
                cache_key, 
                expire, 
                pickle.dumps(result)
            )
            return result
        return wrapper
    return decorator

# 使用快取的範例
@cache_result(expire=600)  # 快取 10 分鐘
async def get_available_agvs(agv_type: str = None):
    """取得可用 AGV (含快取)"""
    # 執行資料庫查詢
    return await agv_crud.get_available_agvs(session, agv_type)
```

## 監控和診斷

### 效能監控指標

#### 系統效能監控
```python
import psutil
import time
from dataclasses import dataclass
from typing import Dict, List

@dataclass
class SystemMetrics:
    timestamp: float
    cpu_percent: float
    memory_percent: float
    disk_io: Dict
    network_io: Dict
    zenoh_connections: int

class PerformanceMonitor:
    def __init__(self):
        self.metrics_history: List[SystemMetrics] = []
        
    async def collect_metrics(self) -> SystemMetrics:
        """收集系統效能指標"""
        
        # CPU 使用率
        cpu_percent = psutil.cpu_percent(interval=1)
        
        # 記憶體使用率
        memory = psutil.virtual_memory()
        memory_percent = memory.percent
        
        # 磁碟 I/O
        disk_io = psutil.disk_io_counters()._asdict()
        
        # 網路 I/O
        network_io = psutil.net_io_counters()._asdict()
        
        # Zenoh 連接數 (透過端口檢查)
        zenoh_connections = len([
            conn for conn in psutil.net_connections() 
            if conn.laddr.port == 7447 and conn.status == 'ESTABLISHED'
        ])
        
        metrics = SystemMetrics(
            timestamp=time.time(),
            cpu_percent=cpu_percent,
            memory_percent=memory_percent,
            disk_io=disk_io,
            network_io=network_io,
            zenoh_connections=zenoh_connections
        )
        
        # 保留最近 1000 筆記錄
        self.metrics_history.append(metrics)
        if len(self.metrics_history) > 1000:
            self.metrics_history.pop(0)
            
        return metrics
    
    def get_performance_summary(self) -> Dict:
        """取得效能摘要"""
        if not self.metrics_history:
            return {}
            
        recent_metrics = self.metrics_history[-60:]  # 最近 60 筆
        
        return {
            "avg_cpu": sum(m.cpu_percent for m in recent_metrics) / len(recent_metrics),
            "avg_memory": sum(m.memory_percent for m in recent_metrics) / len(recent_metrics),
            "max_zenoh_connections": max(m.zenoh_connections for m in recent_metrics),
            "current_zenoh_connections": recent_metrics[-1].zenoh_connections if recent_metrics else 0
        }
```

### Zenoh 效能監控

#### Zenoh 通訊診斷
```bash
#!/bin/bash
# zenoh-performance-check.sh

echo "🔍 Zenoh 效能診斷開始..."

# 檢查 Zenoh 進程
echo "📊 Zenoh 進程狀態:"
ps aux | grep zenoh | grep -v grep

# 檢查 Zenoh 端口
echo "🌐 Zenoh 端口連接:"
ss -tuln | grep 7447
ss -s | grep tcp

# 檢查網路效能
echo "📈 網路介面統計:"
cat /proc/net/dev | grep -E "(eth0|wlan0|docker0)"

# 檢查 ROS 2 主題頻率
echo "🔄 ROS 2 主題效能:"
timeout 10 ros2 topic hz /agv_status 2>/dev/null || echo "主題不可用"

# 檢查記憶體使用
echo "💾 記憶體使用狀況:"
free -h
echo "🔍 Zenoh 記憶體使用:"
pmap $(pgrep zenohd) | tail -1

echo "✅ Zenoh 效能診斷完成"
```

### 自動化效能調整

#### 自適應 QoS 調整
```python
class AdaptiveQoSManager:
    def __init__(self):
        self.latency_history = []
        self.message_loss_rate = 0.0
        
    async def measure_latency(self, topic_name: str) -> float:
        """測量主題延遲"""
        start_time = time.time()
        
        # 發送測試訊息並等待回應
        test_msg = TestMessage(timestamp=start_time)
        await self.test_publisher.publish(test_msg)
        
        # 等待回應 (簡化實作)
        await asyncio.sleep(0.001)  # 1ms 預估延遲
        
        latency = time.time() - start_time
        self.latency_history.append(latency)
        
        # 保留最近 100 筆記錄
        if len(self.latency_history) > 100:
            self.latency_history.pop(0)
            
        return latency
    
    def get_adaptive_qos(self, topic_type: str) -> QoSProfile:
        """根據網路狀況調整 QoS"""
        avg_latency = sum(self.latency_history) / len(self.latency_history) if self.latency_history else 0.001
        
        if avg_latency < 0.001:  # < 1ms，網路良好
            if topic_type == "sensor":
                return QoSProfile(
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                    history=HistoryPolicy.KEEP_LAST,
                    depth=1
                )
        elif avg_latency > 0.01:  # > 10ms，網路擁塞
            if topic_type == "sensor":
                return QoSProfile(
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                    history=HistoryPolicy.KEEP_LAST,
                    depth=5  # 增加緩衝
                )
        
        # 預設 QoS
        return QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
```

## 故障排除

### 效能問題診斷

#### 常見效能瓶頸
```bash
# 1. Zenoh 連接問題診斷
r zenoh-check                          # 檢查 Zenoh 狀態
netstat -tuln | grep 7447             # 檢查端口監聽

# 2. 資料庫效能診斷
# 檢查活躍連接數
docker compose -f docker-compose.agvc.yml exec postgres psql -U agvc -d agvc -c "SELECT count(*) FROM pg_stat_activity WHERE state = 'active';"

# 檢查資料庫大小
docker compose -f docker-compose.agvc.yml exec postgres psql -U agvc -d agvc -c "SELECT pg_size_pretty(pg_database_size('agvc')) as db_size;"

# 檢查表大小統計
docker compose -f docker-compose.agvc.yml exec postgres psql -U agvc -d agvc -c "SELECT schemaname, tablename, pg_size_pretty(pg_total_relation_size(schemaname||'.'||tablename)) as size FROM pg_tables WHERE schemaname = 'public' ORDER BY pg_total_relation_size(schemaname||'.'||tablename) DESC LIMIT 10;"

# 3. 系統資源診斷
top -p $(pgrep zenohd)                # Zenoh 進程資源
docker stats                         # 容器資源使用

# 4. 網路效能診斷
iftop -i eth0                         # 網路流量監控
ss -i                                 # TCP 資訊詳細
```

#### 自動效能調整腳本
```bash
#!/bin/bash
# auto-performance-tune.sh

LOG_FILE="/tmp/performance-tune.log"

log_message() {
    echo "$(date '+%Y-%m-%d %H:%M:%S') $1" | tee -a $LOG_FILE
}

# 檢查 CPU 使用率
CPU_USAGE=$(top -bn1 | grep "Cpu(s)" | awk '{print $2}' | sed 's/%us,//')
if (( $(echo "$CPU_USAGE > 80" | bc -l) )); then
    log_message "⚠️ 高 CPU 使用率: $CPU_USAGE%"
    
    # 降低 ROS 2 節點頻率
    pkill -SIGUSR1 agv_node  # 發送信號降頻
fi

# 檢查記憶體使用率
MEM_USAGE=$(free | grep Mem | awk '{printf("%.1f"), $3/$2 * 100.0}')
if (( $(echo "$MEM_USAGE > 85" | bc -l) )); then
    log_message "⚠️ 高記憶體使用率: $MEM_USAGE%"
    
    # 清理快取
    echo 1 > /proc/sys/vm/drop_caches
fi

# 檢查 Zenoh 連接數
ZENOH_CONNECTIONS=$(ss -tuln | grep 7447 | wc -l)
if [ $ZENOH_CONNECTIONS -eq 0 ]; then
    log_message "❌ Zenoh Router 未運行，嘗試重啟"
    systemctl restart zenoh-router
fi

log_message "✅ 效能檢查完成"
```

## 相關文檔

- [Zenoh 通訊](../technical-details/zenoh-communication.md) - Zenoh RMW 詳細配置
- [系統架構](../system-architecture/dual-environment.md) - 雙環境架構設計
- [資料庫設計](../technical-details/database-design.md) - 資料庫最佳化
- [監控設定](../technical-details/monitoring-setup.md) - 系統監控配置
- [故障排除](troubleshooting.md) - 問題診斷和解決