# 維護操作

## 🎯 RosAGV 系統維護指南

本指南提供 RosAGV 系統的日常維護操作，包括監控、備份、更新、效能調優和預防性維護。

## 📋 維護概覽

### 維護層級分類
```
維護作業分類
├── 🔍 日常監控 (每日)
│   ├── 系統健康檢查
│   ├── 服務狀態監控
│   └── 關鍵指標追蹤
├── 🛠️ 定期維護 (每週)
│   ├── 日誌清理和分析
│   ├── 效能評估
│   └── 安全更新
├── 🔧 深度維護 (每月)
│   ├── 系統最佳化
│   ├── 備份驗證
│   └── 容量規劃
└── 🚨 緊急維護 (按需)
    ├── 故障排除
    ├── 災難恢復
    └── 緊急修復
```

## 🔍 日常監控

### 系統健康檢查

#### 使用統一診斷工具
```bash
# 每日健康檢查流程
r agvc-check              # AGVC 系統健康檢查
r agv-check               # AGV 系統健康檢查
r containers-status       # 容器運行狀態
r network-check           # 網路連接檢查

# 綜合診斷
r quick-diag              # 快速綜合診斷
r system-health           # 完整系統健康檢查
```

#### 容器狀態監控
```bash
# 檢查容器運行狀態
docker ps -a

# 檢查容器資源使用
docker stats --no-stream

# 檢查容器健康狀態
docker compose -f docker-compose.agvc.yml ps
docker compose -f docker-compose.yml ps
```

### 服務狀態監控

#### Web 服務檢查
```bash
# API 服務狀態
curl -f http://localhost:8000/health || echo "API 服務異常"

# 管理介面檢查
curl -f http://localhost:8001/ || echo "AGVCUI 異常"
curl -f http://localhost:8002/ || echo "OPUI 異常"

# 資料庫連接檢查
docker compose -f docker-compose.agvc.yml exec postgres pg_isready -U agvc
```

#### ROS 2 服務檢查
```bash
# 進入容器檢查 ROS 2 服務
docker compose -f docker-compose.yml exec rosagv bash -c "
source /app/setup.bash && 
ros2 node list && 
ros2 topic list && 
ros2 service list
"
```

### 關鍵指標監控

#### 系統資源指標
```bash
# CPU 和記憶體使用率
top -bn1 | head -20
free -h

# 磁碟使用情況
df -h
du -sh /var/lib/docker

# 網路流量
ifconfig
netstat -i
```

#### 應用指標
```bash
# 資料庫效能指標
docker compose -f docker-compose.agvc.yml exec postgres psql -U agvc -d agvc -c "
SELECT 
    schemaname,
    tablename,
    n_tup_ins as inserts,
    n_tup_upd as updates,
    n_tup_del as deletes
FROM pg_stat_user_tables;
"

# ROS 2 主題頻率
docker compose -f docker-compose.yml exec rosagv bash -c "
source /app/setup.bash && 
ros2 topic hz /agv_status --window 10
"
```

## 🛠️ 定期維護

### 日誌管理

#### 日誌收集和分析
```bash
# 收集系統日誌
journalctl --since="1 week ago" > /tmp/system_logs_$(date +%Y%m%d).log

# 收集 Docker 日誌
docker compose -f docker-compose.agvc.yml logs --since=24h > /tmp/agvc_logs_$(date +%Y%m%d).log
docker compose -f docker-compose.yml logs --since=24h > /tmp/agv_logs_$(date +%Y%m%d).log

# 使用統一工具分析錯誤
r log-errors              # 深度日誌分析
r log-scan                # 快速錯誤掃描
```

#### 日誌清理
```bash
# 清理舊日誌 (保留最近 30 天)
journalctl --vacuum-time=30d

# 清理 Docker 日誌
docker system prune -f
docker volume prune -f

# 清理應用日誌
find /tmp -name "*.log" -mtime +7 -delete
```

### 系統更新

#### 安全更新
```bash
# 系統安全更新
sudo apt update
sudo apt list --upgradable
sudo apt upgrade -y

# Docker 更新
sudo apt update docker.io docker-compose-plugin
sudo systemctl restart docker
```

#### 應用更新
```bash
# 備份當前版本
docker compose -f docker-compose.agvc.yml down
cp -r /home/ct/EBD_agv /home/ct/EBD_agv.backup.$(date +%Y%m%d)

# 更新程式碼
cd /home/ct/EBD_agv
git fetch origin
git checkout main
git pull origin main

# 重新建置和部署
docker compose -f docker-compose.agvc.yml build
docker compose -f docker-compose.agvc.yml up -d
```

### 效能監控

#### 效能基準測試
```bash
# API 效能測試
curl -w "@curl-format.txt" -o /dev/null -s "http://localhost:8000/health"

# 資料庫效能測試
docker compose -f docker-compose.agvc.yml exec postgres psql -U agvc -d agvc -c "
EXPLAIN ANALYZE SELECT * FROM agv_status ORDER BY created_at DESC LIMIT 100;
"

# 系統負載測試
stress --cpu 4 --timeout 60s
```

## 🔧 深度維護

### 資料庫維護

#### 資料庫最佳化
```bash
# 進入資料庫容器
docker compose -f docker-compose.agvc.yml exec postgres psql -U agvc -d agvc

# 資料庫統計更新
ANALYZE;

# 重建索引
REINDEX DATABASE agvc;

# 清理無用資料
VACUUM FULL;

# 檢查資料庫大小
SELECT pg_size_pretty(pg_database_size('agvc'));
```

#### 資料備份和恢復
```bash
# 資料庫備份
docker compose -f docker-compose.agvc.yml exec postgres pg_dump -U agvc agvc > backup_$(date +%Y%m%d_%H%M%S).sql

# 資料庫恢復 (測試)
docker compose -f docker-compose.agvc.yml exec postgres psql -U agvc -d agvc_test < backup_20241201_120000.sql

# 設定自動備份
cat << 'EOF' > /etc/cron.daily/rosagv-backup
#!/bin/bash
cd /home/ct/EBD_agv
docker compose -f docker-compose.agvc.yml exec postgres pg_dump -U agvc agvc > /backup/rosagv_$(date +%Y%m%d).sql
find /backup -name "rosagv_*.sql" -mtime +30 -delete
EOF
chmod +x /etc/cron.daily/rosagv-backup
```

### 容器最佳化

#### 映像清理
```bash
# 清理未使用的映像
docker image prune -f

# 清理所有未使用資源
docker system prune -a -f

# 重建映像 (如需要)
docker compose -f docker-compose.agvc.yml build --no-cache
```

#### 容器資源調整
```yaml
# docker-compose.agvc.yml 資源限制調整
services:
  agvc_server:
    deploy:
      resources:
        reservations:
          memory: 2G
          cpus: '1.0'
        limits:
          memory: 4G
          cpus: '2.0'
```

### 安全維護

#### 安全性檢查
```bash
# 檢查開放端口
sudo nmap -sT -O localhost

# 檢查容器安全性
docker run --rm -v /var/run/docker.sock:/var/run/docker.sock \
  aquasec/trivy image rosagv:latest

# 檢查檔案權限
find /home/ct/EBD_agv -type f -perm /o+w 2>/dev/null
```

#### 憑證和密鑰管理
```bash
# 更新資料庫密碑 (每季)
NEW_PASSWORD=$(openssl rand -base64 32)
echo "新密碼: $NEW_PASSWORD"

# 更新 .env 文件
sed -i "s/POSTGRES_PASSWORD=.*/POSTGRES_PASSWORD=$NEW_PASSWORD/" .env

# 重新部署
docker compose -f docker-compose.agvc.yml down
docker compose -f docker-compose.agvc.yml up -d
```

## 📊 監控和告警

### 監控腳本設定
```bash
# 健康監控腳本
cat << 'EOF' > /usr/local/bin/rosagv-monitor.sh
#!/bin/bash
LOG_FILE="/var/log/rosagv-monitor.log"
ERROR_COUNT=0

# 檢查服務狀態
services=("rosagv" "agvc_server" "postgres" "nginx")
for service in "${services[@]}"; do
    if ! docker ps | grep -q "$service"; then
        echo "$(date): 服務 $service 未運行" >> $LOG_FILE
        ((ERROR_COUNT++))
    fi
done

# 檢查 API 服務
if ! curl -f http://localhost:8000/health >/dev/null 2>&1; then
    echo "$(date): API 服務異常" >> $LOG_FILE
    ((ERROR_COUNT++))
fi

# 如果有錯誤，發送告警
if [ $ERROR_COUNT -gt 0 ]; then
    echo "$(date): 發現 $ERROR_COUNT 個問題" >> $LOG_FILE
    # 可以在這裡添加郵件或通知邏輯
fi
EOF

chmod +x /usr/local/bin/rosagv-monitor.sh

# 設定 cron 作業 (每 5 分鐘檢查一次)
echo "*/5 * * * * /usr/local/bin/rosagv-monitor.sh" | crontab -
```

### 告警通知設定
```bash
# 安裝郵件工具
sudo apt install mailutils -y

# 配置簡單告警
cat << 'EOF' > /usr/local/bin/rosagv-alert.sh
#!/bin/bash
if [ "$1" = "error" ]; then
    echo "RosAGV 系統告警: $2" | mail -s "RosAGV Alert" chieu@ms43.hinet.net
fi
EOF

chmod +x /usr/local/bin/rosagv-alert.sh
```

## 🚨 緊急維護

### 緊急回復程序

#### 服務緊急重啟
```bash
# 快速重啟所有服務
docker compose -f docker-compose.agvc.yml restart

# 強制重建並重啟
docker compose -f docker-compose.agvc.yml down
docker compose -f docker-compose.agvc.yml up -d --force-recreate

# 回復到備份版本
cd /home/ct/EBD_agv.backup.20241201
docker compose -f docker-compose.agvc.yml up -d
```

#### 資料恢復
```bash
# 緊急資料恢復
docker compose -f docker-compose.agvc.yml down
docker volume rm rosagv_postgres_data
docker compose -f docker-compose.agvc.yml up -d postgres
sleep 30

# 恢復資料庫
docker compose -f docker-compose.agvc.yml exec postgres psql -U agvc -d agvc < /backup/latest_backup.sql
```

### 災難恢復

#### 完整系統恢復
```bash
# 1. 恢復程式碼
git clone https://github.com/your-org/RosAGV.git
cd RosAGV
git checkout last-stable-tag

# 2. 恢復配置
cp /backup/config/* app/config/
cp /backup/.env .env

# 3. 恢復資料
docker compose -f docker-compose.agvc.yml up -d postgres
sleep 30
docker compose -f docker-compose.agvc.yml exec postgres psql -U agvc -d agvc < /backup/full_backup.sql

# 4. 啟動系統
docker compose -f docker-compose.agvc.yml up -d
```

## 📋 維護檢查清單

### 每日檢查
- [ ] 系統健康檢查 (`r agvc-check`)
- [ ] 容器狀態檢查
- [ ] 關鍵服務可用性檢查
- [ ] 資源使用率檢查
- [ ] 錯誤日誌檢查

### 每週檢查
- [ ] 深度日誌分析
- [ ] 效能指標評估
- [ ] 安全更新檢查
- [ ] 備份驗證
- [ ] 磁碟空間清理

### 每月檢查
- [ ] 資料庫最佳化
- [ ] 系統效能調優
- [ ] 安全性審查
- [ ] 容量規劃評估
- [ ] 災難恢復測試

### 每季檢查
- [ ] 密碼和憑證更新
- [ ] 系統架構評估
- [ ] 備份策略檢討
- [ ] 監控告警測試
- [ ] 維護程序更新

## 📚 維護工具參考

### 統一工具 (r 命令)
- `r agvc-check` - AGVC 健康檢查
- `r containers-status` - 容器狀態
- `r network-check` - 網路檢查
- `r log-errors` - 錯誤日誌分析
- `r quick-diag` - 快速診斷

### 專業工具
- `docker stats` - 容器資源監控
- `psql` - 資料庫管理
- `ros2` - ROS 2 系統管理
- `systemctl` - 系統服務管理

---

**相關文檔：**
- [故障排除](troubleshooting.md) - 問題診斷和解決
- [系統診斷](../operations/system-diagnostics.md) - 診斷工具詳解
- [部署指導](deployment.md) - 系統部署操作
- [開發環境](development.md) - 開發環境設定