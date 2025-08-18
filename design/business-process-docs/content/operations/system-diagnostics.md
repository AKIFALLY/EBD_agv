# 系統診斷工具

## 🎯 RosAGV 系統診斷指南

本指南提供 RosAGV 系統的完整診斷工具集，包括自動化診斷、效能監控、健康檢查和問題分析工具。

## 📋 診斷工具概覽

### 診斷工具分類
```
系統診斷工具體系
├── 🔍 自動化診斷工具
│   ├── 統一診斷命令 (r 系列)
│   ├── 健康檢查腳本
│   └── 自動問題檢測
├── 📊 效能監控工具
│   ├── 資源使用監控
│   ├── 服務效能分析
│   └── 網路連接診斷
├── 🛠️ 專業診斷工具
│   ├── ROS 2 診斷工具
│   ├── 容器診斷工具
│   └── 資料庫診斷工具
└── 📈 報告和分析工具
    ├── 診斷報告生成
    ├── 趨勢分析
    └── 預測性分析
```

## 🔍 統一診斷工具 (r 命令系列)

### 核心診斷命令
```bash
# === 系統整體診斷 ===
r quick-diag           # 快速綜合診斷 (2分鐘完成)
r system-health        # 完整系統健康檢查 (5分鐘完成)
r agvc-check          # AGVC 管理系統專項檢查
r agv-check           # AGV 車載系統專項檢查

# === 組件專項診斷 ===
r containers-status   # Docker 容器狀態檢查
r network-check       # 網路連接和端口檢查
r zenoh-check         # Zenoh 通訊診斷
r database-check      # 資料庫連接和效能檢查

# === 日誌和錯誤分析 ===
r log-scan            # 快速日誌錯誤掃描
r log-errors          # 深度日誌錯誤分析
r log-timeline        # 錯誤時間軸分析
r error-patterns      # 錯誤模式識別
```

### 診斷結果解讀
```bash
# 典型診斷輸出格式
=== RosAGV 快速診斷報告 ===
📅 診斷時間: 2025-08-01 10:30:15
🖥️  系統狀態: [🟢 健康] [🟡 警告] [🔴 異常]

容器狀態:
  ✅ agvc_server    - 運行正常 (CPU: 15%, MEM: 45%)
  ✅ postgres       - 運行正常 (CPU: 8%, MEM: 25%)
  ✅ nginx          - 運行正常 (CPU: 2%, MEM: 10%)
  ⚠️  rosagv         - 高記憶體使用 (CPU: 25%, MEM: 85%)

服務檢查:
  ✅ Web API        - 響應正常 (平均延遲: 45ms)
  ✅ 資料庫連接     - 連接正常 (查詢時間: 12ms)
  ⚠️  Zenoh 通訊    - 延遲偏高 (平均延遲: 150ms)

發現問題:
  🔴 嚴重: PLC 連接失敗 (192.168.2.101:8501)
  🟡 警告: AGV 容器記憶體使用率過高 (85%)
  🟡 警告: Zenoh 通訊延遲增加

建議措施:
  1. 檢查 PLC 網路連接和設備狀態
  2. 重啟 AGV 容器釋放記憶體
  3. 檢查 Zenoh Router 配置和網路狀況
```

## 📊 效能監控工具

### 系統資源監控
```bash
# 系統資源即時監控
class SystemResourceMonitor:
    def __init__(self):
        self.cpu_monitor = CPUMonitor()
        self.memory_monitor = MemoryMonitor()
        self.disk_monitor = DiskMonitor()
        self.network_monitor = NetworkMonitor()
    
    def get_system_status(self):
        """獲取系統資源狀態"""
        return {
            'cpu': self.cpu_monitor.get_usage(),
            'memory': self.memory_monitor.get_usage(),
            'disk': self.disk_monitor.get_usage(),
            'network': self.network_monitor.get_traffic()
        }

# 實際監控命令
r system-resources    # 系統資源概覽
r cpu-analysis        # CPU 使用分析
r memory-analysis     # 記憶體使用分析
r disk-analysis       # 磁碟使用分析
r network-analysis    # 網路流量分析
```

### 服務效能監控
```bash
# Web 服務效能監控
r web-performance     # Web 服務效能分析
r api-latency         # API 回應時間分析
r database-performance # 資料庫效能分析
r ros2-performance    # ROS 2 通訊效能分析

# 效能基準測試
r benchmark-api       # API 效能基準測試
r benchmark-database  # 資料庫效能基準測試
r benchmark-zenoh     # Zenoh 通訊效能測試
```

## 🛠️ 專業診斷工具

### ROS 2 診斷工具
```bash
# ROS 2 系統診斷
r ros2-health         # ROS 2 整體健康檢查
r ros2-nodes          # 節點狀態診斷
r ros2-topics         # 主題通訊診斷
r ros2-services       # 服務可用性診斷

# 詳細的 ROS 2 診斷腳本
#!/bin/bash
# ros2-diagnostic.sh
echo "=== ROS 2 系統診斷 ==="

# 1. 檢查 ROS 2 環境
echo "1. ROS 2 環境檢查"
echo "   ROS_DISTRO: $ROS_DISTRO"
echo "   RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"

# 2. 檢查節點狀態
echo "2. 節點狀態檢查"
active_nodes=$(ros2 node list | wc -l)
echo "   活躍節點數: $active_nodes"

# 3. 檢查主題通訊
echo "3. 主題通訊檢查"
active_topics=$(ros2 topic list | wc -l)
echo "   活躍主題數: $active_topics"

# 4. 檢查服務可用性
echo "4. 服務可用性檢查"
available_services=$(ros2 service list | wc -l)
echo "   可用服務數: $available_services"
```

### 容器診斷工具
```bash
# Docker 容器診斷
r container-health    # 容器健康狀態檢查
r container-logs      # 容器日誌分析
r container-resources # 容器資源使用分析
r container-network   # 容器網路診斷

# 容器專項診斷腳本
#!/bin/bash
# container-diagnostic.sh
echo "=== Docker 容器診斷 ==="

# 1. 容器運行狀態
echo "1. 容器運行狀態"
docker ps -a --format "table {{.Names}}\t{{.Status}}\t{{.Ports}}"

# 2. 容器資源使用
echo "2. 容器資源使用"
docker stats --no-stream --format "table {{.Name}}\t{{.CPUPerc}}\t{{.MemUsage}}"

# 3. 容器網路連接
echo "3. 容器網路檢查"
docker network ls
docker network inspect rosagv_agvc_network | jq -r '.[] | .Containers | keys[]'

# 4. 容器健康檢查
echo "4. 容器健康檢查"
for container in agvc_server postgres nginx rosagv; do
    if docker ps --format '{{.Names}}' | grep -q "^${container}$"; then
        echo "   ✅ $container - 運行中"
    else
        echo "   ❌ $container - 未運行"
    fi
done
```

### 資料庫診斷工具
```bash
# PostgreSQL 資料庫診斷
r database-health     # 資料庫健康檢查
r database-performance # 資料庫效能分析
r database-connections # 資料庫連接診斷
r database-queries    # 慢查詢分析

# 資料庫診斷腳本
#!/bin/bash
# database-diagnostic.sh
echo "=== PostgreSQL 資料庫診斷 ==="

# 1. 資料庫連接測試
echo "1. 資料庫連接測試"
if docker compose -f docker-compose.agvc.yml exec postgres pg_isready -U agvc; then
    echo "   ✅ 資料庫連接正常"
else
    echo "   ❌ 資料庫連接失敗"
fi

# 2. 資料庫狀態檢查
echo "2. 資料庫狀態檢查"
docker compose -f docker-compose.agvc.yml exec postgres psql -U agvc -d agvc -c "
SELECT 
    version() as version,
    current_database() as database,
    current_user as user;
"

# 3. 資料庫效能指標
echo "3. 資料庫效能指標"
docker compose -f docker-compose.agvc.yml exec postgres psql -U agvc -d agvc -c "
SELECT 
    datname,
    numbackends,
    xact_commit,
    xact_rollback,
    blks_read,
    blks_hit
FROM pg_stat_database 
WHERE datname = 'agvc';
"

# 4. 慢查詢檢查
echo "4. 慢查詢分析"
docker compose -f docker-compose.agvc.yml exec postgres psql -U agvc -d agvc -c "
SELECT 
    query,
    mean_time,
    calls,
    total_time
FROM pg_stat_statements 
ORDER BY mean_time DESC 
LIMIT 5;
" 2>/dev/null || echo "   pg_stat_statements 擴展未啟用"
```

## 📈 自動化監控和告警

### 監控腳本設定
```bash
# 自動化監控腳本
#!/bin/bash
# rosagv-monitor.sh - 持續系統監控
MONITOR_LOG="/var/log/rosagv-monitor.log"
ALERT_THRESHOLD_CPU=80
ALERT_THRESHOLD_MEM=85

while true; do
    timestamp=$(date '+%Y-%m-%d %H:%M:%S')
    
    # 執行快速診斷
    diag_result=$(r quick-diag 2>&1)
    diag_status=$?
    
    if [ $diag_status -ne 0 ]; then
        echo "[$timestamp] 🔴 系統診斷發現問題" >> $MONITOR_LOG
        echo "$diag_result" >> $MONITOR_LOG
        
        # 發送告警 (可配置 email、Slack 等)
        echo "RosAGV 系統異常: $diag_result" | mail -s "RosAGV Alert" chieu@ms43.hinet.net
    else
        echo "[$timestamp] ✅ 系統運行正常" >> $MONITOR_LOG
    fi
    
    # 每 5 分鐘檢查一次
    sleep 300
done
```

### 預測性監控
```python
# 預測性問題檢測
class PredictiveMonitor:
    def __init__(self):
        self.historical_data = []
        self.alert_thresholds = {
            'cpu_trend': 0.8,
            'memory_trend': 0.85,
            'error_rate_trend': 0.1
        }
    
    def analyze_trends(self, current_metrics):
        """分析系統趋势"""
        self.historical_data.append(current_metrics)
        
        # 保留最近 100 個數據點
        if len(self.historical_data) > 100:
            self.historical_data.pop(0)
        
        # 趨勢分析
        cpu_trend = self.calculate_trend('cpu_usage')
        memory_trend = self.calculate_trend('memory_usage')
        error_trend = self.calculate_trend('error_rate')
        
        # 預測性告警
        alerts = []
        if cpu_trend > self.alert_thresholds['cpu_trend']:
            alerts.append("CPU 使用率呈上升趨勢，建議檢查系統負載")
        
        if memory_trend > self.alert_thresholds['memory_trend']:
            alerts.append("記憶體使用率持續增長，可能存在內存洩漏")
        
        if error_trend > self.alert_thresholds['error_rate_trend']:
            alerts.append("錯誤率增加，建議檢查系統日誌")
        
        return alerts
```

## 📋 診斷報告生成

### 自動報告生成
```bash
# 生成診斷報告
r generate-report     # 生成完整診斷報告
r health-report       # 生成健康狀態報告
r performance-report  # 生成效能分析報告
r security-report     # 生成安全狀態報告

# 報告範例
#!/bin/bash
# generate-diagnostic-report.sh
REPORT_DATE=$(date '+%Y-%m-%d_%H-%M-%S')
REPORT_FILE="/tmp/rosagv-diagnostic-report-$REPORT_DATE.txt"

echo "=== RosAGV 系統診斷報告 ===" > $REPORT_FILE
echo "生成時間: $(date)" >> $REPORT_FILE
echo "" >> $REPORT_FILE

echo "=== 系統概覽 ===" >> $REPORT_FILE
r system-health >> $REPORT_FILE 2>&1

echo "" >> $REPORT_FILE
echo "=== 容器狀態 ===" >> $REPORT_FILE
r containers-status >> $REPORT_FILE 2>&1

echo "" >> $REPORT_FILE
echo "=== 網路診斷 ===" >> $REPORT_FILE
r network-check >> $REPORT_FILE 2>&1

echo "" >> $REPORT_FILE
echo "=== 效能分析 ===" >> $REPORT_FILE
r system-resources >> $REPORT_FILE 2>&1

echo "診斷報告已生成: $REPORT_FILE"
```

### 趨勢分析報告
```python
# 系統趨勢分析
class TrendAnalyzer:
    def generate_trend_report(self, days=7):
        """生成趨勢分析報告"""
        report = {
            'period': f'過去 {days} 天',
            'summary': self.generate_summary(days),
            'performance_trends': self.analyze_performance_trends(days),
            'error_patterns': self.analyze_error_patterns(days),
            'recommendations': self.generate_recommendations(days)
        }
        
        return report
    
    def analyze_performance_trends(self, days):
        """分析效能趨勢"""
        metrics = self.collect_historical_metrics(days)
        
        return {
            'cpu_trend': self.calculate_trend(metrics, 'cpu'),
            'memory_trend': self.calculate_trend(metrics, 'memory'),
            'response_time_trend': self.calculate_trend(metrics, 'response_time'),
            'throughput_trend': self.calculate_trend(metrics, 'throughput')
        }
```

## 🚀 診斷最佳實踐

### 日常診斷流程
1. **每日快速檢查**: 使用 `r quick-diag` 進行日常健康檢查
2. **週週深度診斷**: 使用 `r system-health` 進行全面檢查
3. **問題追蹤**: 使用 `r log-errors` 分析錯誤模式
4. **效能監控**: 定期使用效能監控工具評估系統狀態

### 問題診斷策略
1. **分層診斷**: 從整體到具體，逐層深入診斷
2. **關聯分析**: 分析不同組件間的關聯性
3. **趨勢監控**: 關注指標變化趨勢而非單點數值
4. **預防性檢查**: 實施預防性監控避免問題發生

---

**相關文檔：**
- [故障排除](troubleshooting.md) - 具體問題解決方案
- [維護操作](maintenance.md) - 日常維護指導
- [效能調優](../technical-details/performance-optimization.md) - 系統效能最佳化
- [監控配置](../technical-details/monitoring-setup.md) - 監控系統配置