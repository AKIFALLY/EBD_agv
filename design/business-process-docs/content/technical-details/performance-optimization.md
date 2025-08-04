# 效能調優

## 🎯 RosAGV 系統效能最佳化指南

本文檔提供 RosAGV 系統的完整效能調優策略，包括系統層級最佳化、應用程式調優、資料庫最佳化和網路效能提升。

## 📊 效能調優概覽

### 最佳化層次
```
效能最佳化架構
├── 🖥️ 系統層級最佳化
│   ├── 作業系統調優
│   ├── 硬體資源最佳化
│   └── 核心參數調整
├── 🐳 容器層級最佳化
│   ├── Docker 設定調優
│   ├── 資源限制最佳化
│   └── 映像最佳化
├── 🚀 應用程式最佳化
│   ├── ROS 2 效能調優
│   ├── Python 應用最佳化
│   └── Web 服務調優
└── 🌐 網路和資料最佳化
    ├── 網路通訊最佳化
    ├── 資料庫效能調優
    └── 快取策略實施
```

## 🖥️ 系統層級最佳化

### 作業系統調優
```bash
# Linux 核心參數調優
# /etc/sysctl.conf
# 網路效能最佳化
net.core.rmem_max = 67108864
net.core.wmem_max = 67108864
net.ipv4.tcp_rmem = 4096 87380 67108864
net.ipv4.tcp_wmem = 4096 65536 67108864

# 記憶體管理最佳化
vm.swappiness = 10
vm.vfs_cache_pressure = 50
vm.dirty_ratio = 15
vm.dirty_background_ratio = 5

# 檔案描述符限制
fs.file-max = 100000
net.core.somaxconn = 1024

# 套用設定
sudo sysctl -p
```

### CPU 效能最佳化
```bash
# CPU 調度器最佳化
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor

# CPU 親和性設定
#!/bin/bash
# set-cpu-affinity.sh
# 為關鍵進程設定 CPU 親和性
AGVC_PID=$(pgrep -f "agvc_server")
POSTGRES_PID=$(pgrep -f "postgres")

# AGVC 服務使用 CPU 0-3
taskset -cp 0-3 $AGVC_PID

# PostgreSQL 使用 CPU 4-7
taskset -cp 4-7 $POSTGRES_PID
```

### 記憶體最佳化
```bash
# 記憶體分配最佳化
# 設定透明大頁面
echo madvise | sudo tee /sys/kernel/mm/transparent_hugepage/enabled

# NUMA 最佳化 (多 CPU 系統)
echo 0 | sudo tee /proc/sys/kernel/numa_balancing
```

## 🐳 容器層級最佳化

### Docker 設定調優
```yaml
# docker-compose.agvc.yml - 效能最佳化版本
version: '3.8'
services:
  agvc_server:
    deploy:
      resources:
        reservations:
          memory: 2G
          cpus: '2.0'
        limits:
          memory: 4G
          cpus: '4.0'
    # 使用 host 網路模式提升網路效能 (生產環境)
    # network_mode: host
    
    # 優化容器設定
    tmpfs:
      - /tmp:size=1G,noexec,nosuid,nodev
    shm_size: 2g
    
    # CPU 設定
    cpuset: "0-3"
    cpu_shares: 1024
    
    # 記憶體設定
    mem_swappiness: 0
    oom_kill_disable: false

  postgres:
    deploy:
      resources:
        reservations:
          memory: 1G
          cpus: '1.0'
        limits:
          memory: 2G
          cpus: '2.0'
    
    # PostgreSQL 特定最佳化
    command: >
      postgres
      -c shared_buffers=256MB
      -c effective_cache_size=1GB
      -c maintenance_work_mem=64MB
      -c checkpoint_completion_target=0.7
      -c wal_buffers=16MB
      -c default_statistics_target=100
```

### 映像最佳化
```dockerfile
# Dockerfile 最佳化範例
FROM ubuntu:24.04 as base

# 使用多階段建置減少映像大小
FROM base as builder
RUN apt-get update && apt-get install -y build-essential
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

FROM base as runtime
# 只複製必要檔案
COPY --from=builder /usr/local/lib/python3.12/site-packages /usr/local/lib/python3.12/site-packages
COPY app/ /app/

# 最佳化設定
ENV PYTHONUNBUFFERED=1
ENV PYTHONDONTWRITEBYTECODE=1
```

## 🚀 應用程式最佳化

### ROS 2 效能調優
```bash
# ROS 2 環境變數最佳化
export RMW_IMPLEMENTATION=rmw_zenohd
export ROS_LOCALHOST_ONLY=0
export ROS_DISABLE_LOANED_MESSAGES=0

# DDS 最佳化設定
export FASTRTPS_DEFAULT_PROFILES_FILE=/app/config/fastrtps_profile.xml
```

```xml
<!-- fastrtps_profile.xml - FastDDS 效能最佳化 -->
<?xml version="1.0" encoding="UTF-8"?>
<profiles>
    <transport_descriptors>
        <transport_descriptor>
            <transport_id>udp_transport</transport_id>
            <type>UDPv4</type>
            <sendBufferSize>262144</sendBufferSize>
            <receiveBufferSize>262144</receiveBufferSize>
        </transport_descriptor>
    </transport_descriptors>
    
    <participant profile_name="high_performance_participant">
        <rtps>
            <userTransports>
                <transport_id>udp_transport</transport_id>
            </userTransports>
            <use_builtin_transports>false</use_builtin_transports>
        </rtps>
    </participant>
</profiles>
```

### Python 應用最佳化
```python
# Python 效能最佳化技巧
import asyncio
import uvloop
from functools import lru_cache
import multiprocessing

# 1. 使用 uvloop 提升異步效能
asyncio.set_event_loop_policy(uvloop.EventLoopPolicy())

# 2. 快取頻繁使用的函數
@lru_cache(maxsize=128)
def expensive_computation(param):
    # 昂貴的計算操作
    return result

# 3. 使用多進程處理 CPU 密集任務
class PerformanceOptimizer:
    def __init__(self):
        self.process_pool = multiprocessing.Pool(
            processes=multiprocessing.cpu_count())
    
    async def process_heavy_task(self, data):
        """異步處理 CPU 密集任務"""
        loop = asyncio.get_event_loop()
        result = await loop.run_in_executor(
            self.process_pool, self.cpu_intensive_task, data)
        return result
```

### Web 服務調優
```python
# FastAPI 效能最佳化
from fastapi import FastAPI
from fastapi.middleware.gzip import GZipMiddleware
from fastapi.middleware.trustedhost import TrustedHostMiddleware
import uvicorn

app = FastAPI()

# 1. 啟用 GZIP 壓縮
app.add_middleware(GZipMiddleware, minimum_size=1000)

# 2. 信任主機中間件
app.add_middleware(
    TrustedHostMiddleware, allowed_hosts=["localhost", "*.company.com"])

# 3. 最佳化 Uvicorn 設定
if __name__ == "__main__":
    uvicorn.run(
        "main:app",
        host="0.0.0.0",
        port=8000,
        workers=4,  # 多工作進程
        loop="uvloop",  # 使用 uvloop
        http="h11",  # HTTP 解析器
        access_log=False,  # 關閉存取日誌提升效能
        log_level="warning"
    )
```

## 🗄️ 資料庫效能調優

### PostgreSQL 設定最佳化
```sql
-- postgresql.conf 最佳化設定
-- 記憶體設定
shared_buffers = 256MB                    -- 25% of RAM
effective_cache_size = 1GB                -- 75% of RAM
work_mem = 32MB                           -- 根據並發查詢調整
maintenance_work_mem = 64MB               -- 維護操作記憶體

-- 檢查點設定
checkpoint_completion_target = 0.7        -- 檢查點完成目標
wal_buffers = 16MB                        -- WAL 緩衝區
checkpoint_timeout = 10min                -- 檢查點超時

-- 查詢最佳化
default_statistics_target = 100           -- 統計目標
random_page_cost = 1.1                    -- SSD 硬碟設定
effective_io_concurrency = 200            -- SSD 並發 I/O

-- 連接設定
max_connections = 200                     -- 最大連接數
shared_preload_libraries = 'pg_stat_statements'  -- 載入統計擴展
```

### 索引最佳化
```sql
-- 分析查詢效能
CREATE EXTENSION IF NOT EXISTS pg_stat_statements;

-- 查找慢查詢
SELECT 
    query,
    mean_time,
    calls,
    total_time,
    rows,
    100.0 * shared_blks_hit / nullif(shared_blks_hit + shared_blks_read, 0) AS hit_percent
FROM pg_stat_statements 
ORDER BY mean_time DESC 
LIMIT 10;

-- 創建複合索引最佳化查詢
CREATE INDEX CONCURRENTLY idx_agv_status_timestamp 
ON agv_status (agv_id, created_at DESC);

-- 定期重建索引
REINDEX INDEX CONCURRENTLY idx_agv_status_timestamp;
```

### 連接池最佳化
```python
# SQLAlchemy 連接池最佳化
from sqlalchemy import create_engine
from sqlalchemy.pool import QueuePool

engine = create_engine(
    DATABASE_URL,
    poolclass=QueuePool,
    pool_size=10,              # 連接池大小
    max_overflow=20,           # 最大溢出連接
    pool_pre_ping=True,        # 連接前測試
    pool_recycle=300,          # 連接回收時間 (秒)
    echo=False                 # 關閉 SQL 日誌提升效能
)
```

## 🌐 網路和通訊最佳化

### Zenoh 通訊最佳化
```json5
// routerconfig.json5 - 效能最佳化版本
{
  "mode": "router",
  
  // 高效能傳輸設定
  "transport": {
    "unicast": {
      "lowlatency": true,        // 啟用低延遲模式
      "qos": {
        "enabled": true,
        "unicast": {
          "reliability": "reliable",
          "congestion_control": "block"
        }
      },
      "compression": {
        "enabled": false         // 關閉壓縮減少 CPU 使用
      }
    }
  },
  
  // 最佳化路由設定
  "routing": {
    "face": {
      "unicast": {
        "accept_timeout": 5000,
        "max_sessions": 2000,    // 增加最大會話數
        "max_links": 4           // 增加連結數
      }
    }
  },
  
  // 記憶體最佳化
  "plugins": {
    "zenoh-plugin-dds": {
      "shm": {
        "enabled": true,
        "size": "1GB"           // 增加共享記憶體大小
      }
    }
  }
}
```

### HTTP 服務最佳化
```nginx
# nginx.conf 效能最佳化
worker_processes auto;
worker_rlimit_nofile 100000;

events {
    worker_connections 4000;
    use epoll;
    multi_accept on;
}

http {
    # 快取設定
    open_file_cache max=200000 inactive=20s;
    open_file_cache_valid 30s;
    open_file_cache_min_uses 2;
    open_file_cache_errors on;
    
    # GZIP 壓縮
    gzip on;
    gzip_min_length 10240;
    gzip_comp_level 1;
    gzip_vary on;
    gzip_types text/plain text/css application/json application/javascript;
    
    # Keep-alive 設定
    keepalive_timeout 30;
    keepalive_requests 100;
    
    # 緩衝區設定
    client_body_buffer_size 128k;
    client_max_body_size 10m;
    client_header_buffer_size 1k;
    large_client_header_buffers 4 4k;
    
    upstream agvc_backend {
        server 127.0.0.1:8000;
        keepalive 32;
    }
}
```

## 📊 效能監控和基準測試

### 效能基準測試
```bash
#!/bin/bash
# performance-benchmark.sh
echo "=== RosAGV 效能基準測試 ==="

# 1. API 效能測試
echo "1. API 效能測試"
ab -n 1000 -c 10 http://localhost:8000/health
wrk -t4 -c100 -d30s http://localhost:8000/health

# 2. 資料庫效能測試
echo "2. 資料庫效能測試"
docker compose -f docker-compose.agvc.yml exec postgres pgbench -i agvc
docker compose -f docker-compose.agvc.yml exec postgres pgbench -c 10 -j 2 -T 60 agvc

# 3. ROS 2 通訊效能測試
echo "3. ROS 2 通訊效能測試"
ros2 topic hz /agv_status --window 100
ros2 topic bw /agv_status

# 4. 系統負載測試
echo "4. 系統負載測試"
stress --cpu 4 --io 2 --vm 2 --timeout 60s
```

### 效能監控指標
```python
class PerformanceMetrics:
    def __init__(self):
        self.metrics = {
            'api_response_time': [],
            'database_query_time': [],
            'ros2_message_latency': [],
            'system_resource_usage': {},
            'network_throughput': []
        }
    
    def collect_metrics(self):
        """收集效能指標"""
        # API 響應時間
        api_latency = self.measure_api_latency()
        self.metrics['api_response_time'].append(api_latency)
        
        # 資料庫查詢時間
        db_latency = self.measure_database_latency()
        self.metrics['database_query_time'].append(db_latency)
        
        # ROS 2 訊息延遲
        ros2_latency = self.measure_ros2_latency()
        self.metrics['ros2_message_latency'].append(ros2_latency)
        
        return self.generate_performance_report()
```

## 🎯 效能調優檢查清單

### 系統層級
- [ ] 核心參數已最佳化
- [ ] CPU 調度器設為 performance 模式
- [ ] 記憶體交換設定已調整
- [ ] 檔案描述符限制已提高

### 容器層級
- [ ] 資源限制已合理設定
- [ ] CPU 親和性已配置
- [ ] 共享記憶體已最佳化
- [ ] 映像大小已最小化

### 應用程式層級
- [ ] ROS 2 設定已調優
- [ ] Python 效能已最佳化
- [ ] Web 服務設定已調整
- [ ] 快取機制已實施

### 資料庫層級
- [ ] PostgreSQL 參數已調優
- [ ] 索引已最佳化
- [ ] 連接池已配置
- [ ] 查詢效能已分析

---

**相關文檔：**
- [系統診斷](../operations/system-diagnostics.md) - 效能問題診斷
- [維護操作](../operations/maintenance.md) - 效能監控指導  
- [雙環境架構](../system-architecture/dual-environment.md) - 架構最佳化
- [Zenoh 通訊](zenoh-communication.md) - 通訊效能調優