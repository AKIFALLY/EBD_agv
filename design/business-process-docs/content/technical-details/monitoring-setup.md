# 監控系統配置

## 🎯 RosAGV 監控系統設定指南

本文檔提供 RosAGV 系統完整的監控系統配置，包括指標收集、告警設定、視覺化儀表板和自動化監控流程。

## 📊 監控系統架構

### 監控架構概覽
```
RosAGV 監控系統架構
├── 📊 指標收集層
│   ├── 系統指標收集 (Node Exporter)
│   ├── 容器指標收集 (cAdvisor)
│   ├── 應用指標收集 (Custom Metrics)
│   └── ROS 2 指標收集 (ROS Metrics)
├── 💾 指標儲存層
│   ├── Prometheus (時序資料庫)
│   ├── InfluxDB (高頻資料)
│   └── 日誌聚合 (ELK Stack)
├── 🎨 視覺化層
│   ├── Grafana 儀表板
│   ├── Web 監控界面
│   └── 行動端監控
└── 🚨 告警通知層
    ├── AlertManager
    ├── Email/SMS 通知
    └── Slack/Teams 整合
```

## 📈 Prometheus 監控設定

### Prometheus 配置
```yaml
# prometheus.yml
global:
  scrape_interval: 15s
  evaluation_interval: 15s

rule_files:
  - "rosagv_alerts.yml"

alerting:
  alertmanagers:
    - static_configs:
        - targets:
          - alertmanager:9093

scrape_configs:
  # RosAGV 系統監控
  - job_name: 'rosagv-system'
    static_configs:
      - targets: ['localhost:9100']  # Node Exporter
    scrape_interval: 5s
    
  # Docker 容器監控
  - job_name: 'rosagv-containers'
    static_configs:
      - targets: ['localhost:8080']  # cAdvisor
    scrape_interval: 10s
    
  # RosAGV 應用監控
  - job_name: 'rosagv-app'
    static_configs:
      - targets: ['localhost:8000', 'localhost:8001', 'localhost:8002']
    metrics_path: '/metrics'
    scrape_interval: 15s
    
  # PostgreSQL 監控
  - job_name: 'postgres'
    static_configs:
      - targets: ['localhost:9187']  # Postgres Exporter
    scrape_interval: 30s
    
  # ROS 2 節點監控
  - job_name: 'ros2-nodes'
    static_configs:
      - targets: ['localhost:9200']  # ROS 2 Metrics Exporter
    scrape_interval: 10s
```

### RosAGV 自定義指標
```python
# RosAGV 應用指標收集
from prometheus_client import Counter, Histogram, Gauge, start_http_server
import time

class RosAGVMetrics:
    def __init__(self):
        # 計數器指標
        self.task_counter = Counter(
            'rosagv_tasks_total', 
            'Total number of tasks processed',
            ['task_type', 'status']
        )
        
        # 直方圖指標 (延遲分佈)
        self.task_duration = Histogram(
            'rosagv_task_duration_seconds',
            'Task processing duration',
            ['task_type']
        )
        
        # 儀表指標 (當前狀態)
        self.active_agvs = Gauge(
            'rosagv_active_vehicles',
            'Number of active AGV vehicles',
            ['vehicle_type']
        )
        
        self.system_health = Gauge(
            'rosagv_system_health_score',
            'Overall system health score (0-100)'
        )
        
        # 啟動 HTTP 服務器供 Prometheus 抓取
        start_http_server(8090)
    
    def record_task_completion(self, task_type, duration, status):
        """記錄任務完成"""
        self.task_counter.labels(task_type=task_type, status=status).inc()
        self.task_duration.labels(task_type=task_type).observe(duration)
    
    def update_vehicle_count(self, vehicle_type, count):
        """更新車輛數量"""
        self.active_agvs.labels(vehicle_type=vehicle_type).set(count)
    
    def update_health_score(self, score):
        """更新健康分數"""
        self.system_health.set(score)

# 在 FastAPI 應用中整合
from fastapi import FastAPI
app = FastAPI()
metrics = RosAGVMetrics()

@app.get("/metrics")
async def get_metrics():
    """Prometheus 指標端點"""
    # Prometheus client 會自動處理
    pass
```

## 🎨 Grafana 儀表板設定

### RosAGV 主儀表板
```json
{
  "dashboard": {
    "id": null,
    "title": "RosAGV System Overview",
    "tags": ["rosagv", "monitoring"],
    "timezone": "browser",
    "panels": [
      {
        "id": 1,
        "title": "System Health Score",
        "type": "stat",
        "targets": [
          {
            "expr": "rosagv_system_health_score",
            "legendFormat": "Health Score"
          }
        ],
        "fieldConfig": {
          "defaults": {
            "min": 0,
            "max": 100,
            "unit": "percent",
            "thresholds": {
              "steps": [
                {"color": "red", "value": 0},
                {"color": "yellow", "value": 70},
                {"color": "green", "value": 85}
              ]
            }
          }
        }
      },
      {
        "id": 2,
        "title": "Active AGV Vehicles",
        "type": "graph",
        "targets": [
          {
            "expr": "sum by (vehicle_type) (rosagv_active_vehicles)",
            "legendFormat": "{{vehicle_type}}"
          }
        ]
      },
      {
        "id": 3,
        "title": "Task Processing Rate",
        "type": "graph",
        "targets": [
          {
            "expr": "rate(rosagv_tasks_total[5m])",
            "legendFormat": "Tasks/sec"
          }
        ]
      },
      {
        "id": 4,
        "title": "System Resource Usage",
        "type": "graph",
        "targets": [
          {
            "expr": "100 - (avg(irate(node_cpu_seconds_total{mode=\"idle\"}[5m])) * 100)",
            "legendFormat": "CPU Usage %"
          },
          {
            "expr": "100 * (1 - node_memory_MemAvailable_bytes / node_memory_MemTotal_bytes)",
            "legendFormat": "Memory Usage %"
          }
        ]
      }
    ]
  }
}
```

### 容器監控儀表板
```json
{
  "dashboard": {
    "title": "RosAGV Container Monitoring",
    "panels": [
      {
        "title": "Container CPU Usage",
        "type": "graph",
        "targets": [
          {
            "expr": "rate(container_cpu_usage_seconds_total{name=~\"agvc_server|postgres|nginx|rosagv\"}[5m]) * 100",
            "legendFormat": "{{name}}"
          }
        ]
      },
      {
        "title": "Container Memory Usage",
        "type": "graph", 
        "targets": [
          {
            "expr": "container_memory_usage_bytes{name=~\"agvc_server|postgres|nginx|rosagv\"} / 1024 / 1024",
            "legendFormat": "{{name}} MB"
          }
        ]
      },
      {
        "title": "Container Network I/O",
        "type": "graph",
        "targets": [
          {
            "expr": "rate(container_network_receive_bytes_total{name=~\"agvc_server|postgres|nginx|rosagv\"}[5m])",
            "legendFormat": "{{name}} RX"
          },
          {
            "expr": "rate(container_network_transmit_bytes_total{name=~\"agvc_server|postgres|nginx|rosagv\"}[5m])",
            "legendFormat": "{{name}} TX"
          }
        ]
      }
    ]
  }
}
```

## 🚨 告警規則配置

### Prometheus 告警規則
```yaml
# rosagv_alerts.yml
groups:
  - name: rosagv.rules
    rules:
      # 系統健康告警
      - alert: SystemHealthLow
        expr: rosagv_system_health_score < 70
        for: 5m
        labels:
          severity: warning
        annotations:
          summary: "RosAGV system health is low"
          description: "System health score is {{ $value }}%"
      
      - alert: SystemHealthCritical
        expr: rosagv_system_health_score < 50
        for: 2m
        labels:
          severity: critical
        annotations:
          summary: "RosAGV system health is critical"
          description: "System health score is {{ $value }}%"
      
      # 容器資源告警
      - alert: HighCPUUsage
        expr: rate(container_cpu_usage_seconds_total[5m]) * 100 > 80
        for: 10m
        labels:
          severity: warning
        annotations:
          summary: "High CPU usage detected"
          description: "Container {{ $labels.name }} CPU usage is {{ $value }}%"
      
      - alert: HighMemoryUsage
        expr: container_memory_usage_bytes / container_spec_memory_limit_bytes * 100 > 85
        for: 5m
        labels:
          severity: warning
        annotations:
          summary: "High memory usage detected"
          description: "Container {{ $labels.name }} memory usage is {{ $value }}%"
      
      # 應用服務告警
      - alert: ServiceDown
        expr: up{job="rosagv-app"} == 0
        for: 1m
        labels:
          severity: critical
        annotations:
          summary: "RosAGV service is down"
          description: "Service {{ $labels.instance }} is not responding"
      
      # 資料庫告警
      - alert: DatabaseConnectionsHigh
        expr: pg_stat_database_numbackends > 80
        for: 5m
        labels:
          severity: warning
        annotations:
          summary: "High database connections"
          description: "PostgreSQL has {{ $value }} active connections"
      
      # ROS 2 節點告警
      - alert: ROS2NodeDown
        expr: ros2_node_status == 0
        for: 2m
        labels:
          severity: critical
        annotations:
          summary: "ROS 2 node is down"
          description: "ROS 2 node {{ $labels.node_name }} is not active"
```

### AlertManager 配置
```yaml
# alertmanager.yml
global:
  smtp_smarthost: 'smtp.company.com:587'
  smtp_from: 'alerts@company.com'
  smtp_auth_username: 'alerts@company.com'
  smtp_auth_password: 'password'

route:
  group_by: ['alertname', 'severity']
  group_wait: 10s
  group_interval: 10s
  repeat_interval: 1h
  receiver: 'web.hook'
  routes:
    - match:
        severity: critical
      receiver: 'critical-alerts'
    - match:
        severity: warning
      receiver: 'warning-alerts'

receivers:
  - name: 'web.hook'
    email_configs:
      - to: 'chieu@ms43.hinet.net'
        subject: 'RosAGV Alert: {{ .GroupLabels.alertname }}'
        body: |
          {{ range .Alerts }}
          Alert: {{ .Annotations.summary }}
          Description: {{ .Annotations.description }}
          {{ end }}

  - name: 'critical-alerts'
    email_configs:
      - to: 'chieu@ms43.hinet.net'
        subject: '🚨 CRITICAL: RosAGV {{ .GroupLabels.alertname }}'
    slack_configs:
      - api_url: 'https://hooks.slack.com/services/...'
        channel: '#rosagv-alerts'
        title: 'Critical RosAGV Alert'
        text: '{{ range .Alerts }}{{ .Annotations.summary }}{{ end }}'

  - name: 'warning-alerts'
    email_configs:
      - to: 'team@company.com'
        subject: '⚠️ WARNING: RosAGV {{ .GroupLabels.alertname }}'
```

## 📱 行動端監控應用

### 監控 Web 應用
```html
<!-- mobile-monitor.html -->
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>RosAGV Mobile Monitor</title>
    <style>
        body { font-family: Arial, sans-serif; margin: 0; padding: 20px; }
        .metric-card { 
            background: #f5f5f5; 
            padding: 15px; 
            margin: 10px 0; 
            border-radius: 8px; 
        }
        .metric-value { font-size: 24px; font-weight: bold; }
        .metric-label { color: #666; font-size: 14px; }
        .status-ok { color: #28a745; }
        .status-warning { color: #ffc107; }
        .status-critical { color: #dc3545; }
    </style>
</head>
<body>
    <h1>RosAGV System Status</h1>
    
    <div class="metric-card">
        <div class="metric-label">System Health</div>
        <div class="metric-value" id="health-score">--</div>
    </div>
    
    <div class="metric-card">
        <div class="metric-label">Active AGVs</div>
        <div class="metric-value" id="active-agvs">--</div>
    </div>
    
    <div class="metric-card">
        <div class="metric-label">Tasks/Hour</div>
        <div class="metric-value" id="task-rate">--</div>
    </div>
    
    <script>
        async function updateMetrics() {
            try {
                const response = await fetch('/api/metrics/summary');
                const data = await response.json();
                
                document.getElementById('health-score').textContent = 
                    data.health_score + '%';
                document.getElementById('active-agvs').textContent = 
                    data.active_agvs;
                document.getElementById('task-rate').textContent = 
                    data.tasks_per_hour;
                    
                // 更新狀態顏色
                const healthElement = document.getElementById('health-score');
                if (data.health_score >= 85) {
                    healthElement.className = 'metric-value status-ok';
                } else if (data.health_score >= 70) {
                    healthElement.className = 'metric-value status-warning';
                } else {
                    healthElement.className = 'metric-value status-critical';
                }
            } catch (error) {
                console.error('Failed to update metrics:', error);
            }
        }
        
        // 每 30 秒更新一次
        setInterval(updateMetrics, 30000);
        updateMetrics(); // 初始載入
    </script>
</body>
</html>
```

## 🔧 自動化監控部署

### Docker Compose 監控堆疊
```yaml
# monitoring-stack.yml
version: '3.8'
services:
  prometheus:
    image: prom/prometheus:latest
    container_name: prometheus
    ports:
      - "9090:9090"
    volumes:
      - ./monitoring/prometheus.yml:/etc/prometheus/prometheus.yml
      - ./monitoring/rosagv_alerts.yml:/etc/prometheus/rosagv_alerts.yml
    command:
      - '--config.file=/etc/prometheus/prometheus.yml'
      - '--storage.tsdb.path=/prometheus'
      - '--web.console.libraries=/etc/prometheus/console_libraries'
      - '--web.console.templates=/etc/prometheus/consoles'
      - '--web.enable-lifecycle'
      - '--web.enable-admin-api'

  grafana:
    image: grafana/grafana:latest
    container_name: grafana
    ports:
      - "3000:3000"
    environment:
      - GF_SECURITY_ADMIN_PASSWORD=admin123
    volumes:
      - grafana-storage:/var/lib/grafana
      - ./monitoring/grafana/dashboards:/var/lib/grafana/dashboards
      - ./monitoring/grafana/provisioning:/etc/grafana/provisioning

  alertmanager:
    image: prom/alertmanager:latest
    container_name: alertmanager
    ports:
      - "9093:9093"
    volumes:
      - ./monitoring/alertmanager.yml:/etc/alertmanager/alertmanager.yml

  node-exporter:
    image: prom/node-exporter:latest
    container_name: node-exporter
    ports:
      - "9100:9100"
    volumes:
      - /proc:/host/proc:ro
      - /sys:/host/sys:ro
      - /:/rootfs:ro
    command:
      - '--path.procfs=/host/proc'
      - '--path.rootfs=/rootfs'
      - '--path.sysfs=/host/sys'
      - '--collector.filesystem.mount-points-exclude=^/(sys|proc|dev|host|etc)($$|/)'

  cadvisor:
    image: gcr.io/cadvisor/cadvisor:latest
    container_name: cadvisor
    ports:
      - "8080:8080"
    volumes:
      - /:/rootfs:ro
      - /var/run:/var/run:ro
      - /sys:/sys:ro
      - /var/lib/docker/:/var/lib/docker:ro
      - /dev/disk/:/dev/disk:ro

volumes:
  grafana-storage:
```

### 監控部署腳本
```bash
#!/bin/bash
# setup-monitoring.sh
echo "設定 RosAGV 監控系統..."

# 1. 創建監控目錄
mkdir -p monitoring/{prometheus,grafana/{dashboards,provisioning},alertmanager}

# 2. 複製配置檔案
cp config/prometheus.yml monitoring/prometheus/
cp config/rosagv_alerts.yml monitoring/prometheus/
cp config/alertmanager.yml monitoring/alertmanager/

# 3. 啟動監控堆疊
docker compose -f monitoring-stack.yml up -d

# 4. 等待服務啟動
echo "等待服務啟動..."
sleep 30

# 5. 驗證服務
echo "驗證監控服務..."
curl -f http://localhost:9090 && echo "✅ Prometheus OK"
curl -f http://localhost:3000 && echo "✅ Grafana OK"
curl -f http://localhost:9093 && echo "✅ AlertManager OK"

echo "監控系統設定完成！"
echo "Prometheus: http://localhost:9090"
echo "Grafana: http://localhost:3000 (admin/admin123)"
echo "AlertManager: http://localhost:9093"
```

## 📈 監控最佳實踐

### 指標設計原則
1. **相關性**: 只收集與業務相關的指標
2. **可操作性**: 指標應該能指導具體行動
3. **及時性**: 重要指標應該實時更新
4. **可擴展性**: 監控系統應該能隨系統成長

### 告警設計原則
1. **可操作的告警**: 每個告警都應該有明確的處理步驟
2. **適當的閾值**: 避免過多或過少的告警
3. **分級告警**: 按照嚴重程度分級處理
4. **告警抑制**: 避免告警風暴

---

**相關文檔：**
- [系統診斷](../operations/system-diagnostics.md) - 監控數據分析
- [效能調優](performance-optimization.md) - 監控指標最佳化
- [維護操作](../operations/maintenance.md) - 基於監控的維護
- [故障排除](../operations/troubleshooting.md) - 監控告警處理