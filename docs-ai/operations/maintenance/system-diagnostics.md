# 系統診斷操作指導

## 🎯 適用場景
- 日常系統健康檢查
- 故障排除和問題診斷
- 效能監控和分析
- 預防性維護檢查

## 📋 RosAGV 診斷工具體系

### 統一診斷入口 (r 命令)
RosAGV 提供統一的診斷工具入口，簡化日常維護操作：

```bash
# 🔍 核心診斷命令
r agvc-check         # AGVC 管理系統健康檢查
r agv-check          # AGV 車載系統健康檢查
r system-health      # 完整系統健康檢查
r quick-diag         # 快速綜合診斷

# 🐳 容器狀態檢查
r containers-status  # 檢查所有容器狀態

# 🌐 網路診斷
r network-check      # 系統端口檢查
r zenoh-check        # Zenoh 連接檢查

# 📋 日誌分析
r log-scan           # 日誌錯誤掃描
r log-errors         # 高級錯誤分析
```

### 專業診斷工具集
```bash
# 載入系統診斷工具集
source scripts/system-tools/system-tools.sh

# 專業診斷函數
system_health_check     # 完整健康檢查
quick_diagnose          # 快速診斷
all_health              # 智能健康檢查
all_status              # 容器狀態

# Web 服務診斷工具 (宿主機執行)
scripts/system-tools/service-monitor.sh status    # 所有服務監控
scripts/docker-tools/container-status.sh agvc     # AGVC 容器詳細狀態

# 日誌分析工具
scripts/log-tools/log-analyzer.sh agvc --stats     # AGVC 日誌統計
scripts/log-tools/log-analyzer.sh agvc --timeline  # 錯誤時間軸
scripts/log-tools/log-analyzer.sh agvc --suggestions # 解決建議

# 網路和端口診斷
scripts/network-tools/port-check.sh system         # 系統端口檢查
scripts/network-tools/connectivity-test.sh performance --target localhost
scripts/network-tools/port-check.sh --port 8000-8002 # Web 服務端口檢查
```

## 🔍 日常健康檢查流程

### 每日例行檢查
```bash
# 標準每日檢查流程
r agvc-check              # 1. AGVC 系統健康檢查
r containers-status       # 2. 容器運行狀態
r network-check          # 3. 網路連接檢查

# 組合檢查 (一行執行)
r agvc-check && r containers-status && r network-check
```

### 每週深度檢查
```bash
# 深度系統檢查
r system-health          # 完整系統健康檢查
r log-scan              # 日誌錯誤掃描

# 組合深度檢查
r system-health && r log-scan
```

### 故障排除檢查
```bash
# 遇到問題時的診斷流程
r quick-diag             # 1. 快速綜合診斷
r log-errors             # 2. 深度日誌分析
r zenoh-check           # 3. Zenoh 連接專項檢查

# 組合故障診斷
r quick-diag && r log-errors && r zenoh-check
```

## 🐳 容器診斷

### 容器狀態檢查
```bash
# 檢查容器運行狀態
r containers-status

# 詳細容器資訊
docker compose -f docker-compose.agvc.yml ps
docker compose -f docker-compose.yml ps

# 容器資源使用
docker stats

# 容器健康檢查
docker compose -f docker-compose.agvc.yml exec agvc_server bash -c "check_system_status"
```

### 容器日誌檢查
```bash
# 查看容器日誌
docker compose -f docker-compose.agvc.yml logs -f agvc_server
docker compose -f docker-compose.agvc.yml logs -f postgres_container
docker compose -f docker-compose.agvc.yml logs -f nginx

# 查看最近日誌
docker compose -f docker-compose.agvc.yml logs --tail=100 agvc_server
```

### 容器網路診斷
```bash
# 檢查容器網路
docker network ls
docker network inspect rosagv_agvc_network

# 容器間連接測試
docker compose -f docker-compose.agvc.yml exec agvc_server ping postgres_container
docker compose -f docker-compose.agvc.yml exec agvc_server ping nginx
```

## 🌐 網路診斷

### 基礎網路檢查
```bash
# 端口連接檢查
r network-check

# 專業網路診斷工具
scripts/network-tools/port-check.sh system           # 系統端口檢查
scripts/network-tools/port-check.sh --port 8000-8002 # Web 服務端口
scripts/network-tools/connectivity-test.sh performance --target localhost

# 網路端口檢查 (推薦使用 ss)
ss -tulpn | rg "(8000|8001|8002|5432|5050|80|7447)"

# 備選：netstat (舊工具，但仍可用)
netstat -tulpn | rg "(8000|8001|8002|5432|5050|80|7447)"

# 端口可達性測試
telnet localhost 8000
telnet localhost 5432
telnet localhost 7447
```

### Zenoh 通訊診斷
```bash
# Zenoh 連接檢查
r zenoh-check

# 手動 Zenoh 檢查
ps aux | rg zenoh
cat /tmp/zenoh_router.pid
pgrep -f rmw_zenohd

# Zenoh 配置檢查
cat /app/routerconfig.json5
```

### 跨環境通訊測試
```bash
# AGV 和 AGVC 環境通訊測試
# 在 AGV 容器中
ros2 topic list
ros2 topic echo /agv_status

# 在 AGVC 容器中
ros2 topic list
ros2 topic echo /agv_status
```

## 📊 效能監控

### 系統資源監控
```bash
# CPU 和記憶體使用
top
htop
free -h

# 磁碟使用
df -h
du -sh /app/*

# 網路使用
iftop
nethogs
```

### 容器資源監控
```bash
# 容器資源使用統計
docker stats

# 特定容器資源監控
docker stats agvc_server postgres_container nginx

# 容器內資源檢查
docker compose -f docker-compose.agvc.yml exec agvc_server bash -c "top -bn1 | head -20"
docker compose -f docker-compose.agvc.yml exec agvc_server bash -c "free -h"
```

### 服務效能監控
```bash
# Web 服務效能
curl -w "@curl-format.txt" -o /dev/null -s "http://localhost:8000/health"

# 資料庫效能
docker compose -f docker-compose.agvc.yml exec postgres_container psql -U postgres -c "SELECT * FROM pg_stat_activity;"

# ROS 2 主題效能
ros2 topic hz /topic_name
ros2 topic bw /topic_name
```

## 🔧 服務診斷

### Web 服務診斷
```bash
# API 服務檢查
curl http://localhost:8000/health
curl http://localhost:8000/docs

# AGVCUI 檢查
curl http://localhost:8001/

# OPUI 檢查
curl http://localhost:8002/
```

### 資料庫診斷
```bash
# PostgreSQL 連接測試
docker compose -f docker-compose.agvc.yml exec postgres_container psql -U postgres -c "SELECT version();"

# 資料庫狀態檢查
docker compose -f docker-compose.agvc.yml exec postgres_container psql -U postgres -c "SELECT * FROM pg_stat_database;"

# 資料庫大小檢查
docker compose -f docker-compose.agvc.yml exec postgres_container psql -U postgres -c "SELECT pg_size_pretty(pg_database_size('postgres'));"
```

### ROS 2 服務診斷
```bash
# ROS 2 環境檢查
echo $ROS_DISTRO
echo $RMW_IMPLEMENTATION

# 節點和服務檢查
ros2 node list
ros2 service list
ros2 topic list

# 服務可用性測試
ros2 service call /service_name service_type
```

## 🚨 故障排除指導

### 常見問題診斷

#### 容器啟動失敗
```bash
# 診斷步驟
1. r containers-status    # 檢查容器狀態
2. r log-errors          # 檢查錯誤日誌
3. r agvc-stop           # 停止異常服務
4. r agvc-start          # 重新啟動
```

#### 網路連接問題
```bash
# 診斷步驟
1. r network-check       # 檢查端口狀態
2. r zenoh-check        # 檢查 Zenoh 連接
3. r quick-diag         # 綜合診斷
```

#### 系統效能問題
```bash
# 診斷步驟
1. r agvc-check         # 系統健康狀態
2. r quick-diag         # 快速診斷
3. r log-scan           # 檢查系統日誌
4. 分析系統資源使用情況
```

### 診斷決策樹
```
用戶報告問題
├── 容器相關 → r containers-status → 如果異常 → r agvc-start
├── 網路相關 → r network-check → 如果異常 → r zenoh-check
├── 效能相關 → r agvc-check → 如果異常 → r quick-diag
└── 未知問題 → r quick-diag → r log-errors → 深度分析
```

## 📋 診斷報告

### 自動報告生成
```bash
# 生成系統健康報告
r system-health > system-report-$(date +%Y%m%d).txt

# 生成錯誤分析報告
r log-errors > error-analysis-$(date +%Y%m%d).log

# 生成網路診斷報告
r network-check > network-report-$(date +%Y%m%d).txt
```

### 報告內容結構
```
系統診斷報告
├── 容器狀態
├── 服務狀態
├── 網路連接
├── 資源使用
├── 錯誤日誌
└── 建議措施
```

## 💡 最佳實踐

### 預防性維護
1. **定期檢查**: 每日執行基本健康檢查
2. **趨勢監控**: 週期性分析系統效能趨勢
3. **日誌輪轉**: 設置適當的日誌輪轉策略
4. **備份驗證**: 定期驗證備份的完整性

### 故障響應
1. **快速診斷**: 使用統一工具快速定位問題
2. **分層分析**: 從整體到具體的分層診斷
3. **文檔記錄**: 記錄故障現象和解決方案
4. **預防措施**: 分析根因並實施預防措施

### 監控自動化
```bash
# 創建自動監控腳本
#!/bin/bash
# daily-health-check.sh
echo "開始每日健康檢查..."
r agvc-check > /tmp/daily-health-$(date +%Y%m%d).txt
r containers-status >> /tmp/daily-health-$(date +%Y%m%d).txt
r network-check >> /tmp/daily-health-$(date +%Y%m%d).txt
echo "健康檢查完成，報告已產生"
```

## 🔗 交叉引用
- 故障排除流程: @docs-ai/operations/maintenance/troubleshooting.md
- 日誌分析方法: @docs-ai/operations/maintenance/log-analysis.md
- 效能監控: @docs-ai/operations/maintenance/performance-monitoring.md
- 容器管理: @docs-ai/operations/deployment/container-management.md
- 網路診斷: @docs-ai/knowledge/protocols/zenoh-rmw.md
