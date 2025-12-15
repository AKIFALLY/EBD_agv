# 故障排除

## 🎯 RosAGV 故障排除指南

本指南提供 RosAGV 系統常見問題的診斷方法和解決方案，協助快速定位和修復系統故障。

## 📋 故障分類體系

### 故障類型分類
```
故障分類
├── 🐳 容器相關故障
│   ├── 容器無法啟動
│   ├── 容器異常退出
│   └── 容器資源不足
├── 🌐 網路通訊故障
│   ├── Zenoh 連接失敗
│   ├── PLC 通訊中斷
│   └── Web 服務無回應
├── 💾 資料庫故障
│   ├── 連接失敗
│   ├── 查詢超時
│   └── 資料損壞
├── 🤖 ROS 2 故障
│   ├── 節點無法啟動
│   ├── 主題通訊異常
│   └── 服務呼叫失敗
└── 🚗 AGV 功能故障
    ├── 狀態機異常
    ├── 硬體通訊失敗
    └── 導航定位問題
```

## 🚨 緊急故障處理流程

### 第一階段：快速評估 (1-2分鐘)
```bash
# 1. 系統整體狀態檢查
r quick-diag

# 2. 容器運行狀態
r containers-status

# 3. 關鍵服務檢查
curl -f http://localhost:8000/health || echo "API 異常"
curl -f http://localhost:8001/ || echo "AGVCUI 異常"
```

### 第二階段：問題定位 (3-5分鐘)
```bash
# 根據第一階段結果選擇對應診斷
# 如果容器異常
r log-errors

# 如果網路異常
r network-check
r zenoh-check

# 如果服務異常
docker compose -f docker-compose.agvc.yml logs --tail=100
```

### 第三階段：問題解決 (5-15分鐘)
根據診斷結果執行對應的解決方案（詳見下方具體故障處理）

## 🐳 容器相關故障

### 容器無法啟動

#### 症狀識別
- `docker compose up` 命令失敗
- 容器狀態為 `Exited` 或 `Restarting`
- 服務無法存取

#### 診斷步驟
```bash
# 1. 檢查容器狀態
docker ps -a
docker compose -f docker-compose.agvc.yml ps

# 2. 查看啟動日誌
docker compose -f docker-compose.agvc.yml logs agvc_server

# 3. 檢查端口衝突
ss -tulpn | grep -E "(8000|8001|8002|5432|7447)"

# 4. 檢查磁碟空間
df -h
```

#### 常見原因和解決方案

**原因1: 端口被佔用**
```bash
# 診斷
sudo lsof -i :8000

# 解決
sudo kill -9 <PID>
# 或修改 docker-compose.agvc.yml 中的端口配置
```

**原因2: 磁碟空間不足**
```bash
# 診斷
df -h
du -sh /var/lib/docker

# 解決
docker system prune -f
docker volume prune -f
sudo apt autoremove -y
```

**原因3: 映像損壞**
```bash
# 診斷
docker images | grep rosagv

# 解決
docker compose -f docker-compose.agvc.yml build --no-cache
docker compose -f docker-compose.agvc.yml up -d
```

### 容器異常退出

#### 症狀識別
- 容器運行一段時間後自動停止
- 日誌顯示異常退出碼
- 服務間歇性不可用

#### 診斷和解決
```bash
# 檢查退出碼
docker ps -a

# 常見退出碼含義
# 0: 正常退出
# 1: 一般錯誤
# 125: Docker daemon 錯誤
# 126: 容器命令不可執行
# 127: 容器命令未找到
# 137: 被 SIGKILL 終止 (通常是 OOM)

# OOM Kill 解決方案
docker compose -f docker-compose.agvc.yml down
# 編輯 docker-compose.agvc.yml 增加記憶體限制
docker compose -f docker-compose.agvc.yml up -d
```

## 🌐 網路通訊故障

### Zenoh 連接失敗

#### 症狀識別
- AGV 和 AGVC 無法通訊
- ROS 2 主題無法跨容器傳輸
- `ros2 topic list` 只顯示本地主題

#### 診斷步驟
```bash
# 1. 檢查 Zenoh Router 狀態
r zenoh-check

# 2. 檢查進程
ps aux | grep zenoh
cat /tmp/zenoh_router.pid

# 3. 檢查端口監聽
ss -tulpn | grep 7447

# 4. 測試連接
telnet localhost 7447
telnet 192.168.100.100 7447  # 從 AGV 測試 AGVC
```

#### 解決方案
```bash
# 重啟 Zenoh Router
docker compose -f docker-compose.agvc.yml restart agvc_server
docker compose -f docker-compose.yml restart rosagv

# 檢查配置檔案
cat /app/routerconfig.json5
json5 --validate /app/routerconfig.json5

# 防火牆問題
sudo ufw allow 7447
sudo iptables -A INPUT -p tcp --dport 7447 -j ACCEPT
```

### PLC 通訊中斷

#### 症狀識別
- AGV 無法控制機械臂
- PLC 狀態讀取失敗
- 硬體操作無回應

#### 診斷和解決
```bash
# 1. 測試 PLC 網路連接
ping 192.168.2.101
telnet 192.168.2.101 8501

# 2. 檢查 PLC 服務
docker compose -f docker-compose.yml exec rosagv bash -c "
source /app/setup.bash && 
ros2 service call /plc_read plc_interfaces/PLCRead 'address: \"DM100\"'
"

# 3. 重啟 PLC 通訊服務
docker compose -f docker-compose.yml exec rosagv bash -c "
source /app/setup.bash && 
ros2 lifecycle set /plc_node shutdown && 
sleep 5 && 
ros2 run keyence_plc plc_node
"
```

### Web 服務無回應

#### 症狀識別
- 瀏覽器無法存取管理介面
- API 呼叫返回 5xx 錯誤
- 連接超時

#### 診斷和解決
```bash
# 1. 檢查服務狀態
curl -I http://localhost:8000/health
curl -I http://localhost:8001/
curl -I http://localhost:8002/

# 2. 檢查 Nginx 配置
docker compose -f docker-compose.agvc.yml exec nginx nginx -t

# 3. 重啟 Web 服務
docker compose -f docker-compose.agvc.yml restart nginx
docker compose -f docker-compose.agvc.yml restart agvc_server
```

## 💾 資料庫故障

### 資料庫連接失敗

#### 症狀識別
- 應用程式無法啟動
- 資料庫連接錯誤訊息
- Web API 返回資料庫錯誤

#### 診斷步驟
```bash
# 1. 檢查資料庫容器
docker compose -f docker-compose.agvc.yml ps postgres

# 2. 測試連接
docker compose -f docker-compose.agvc.yml exec postgres pg_isready -U agvc

# 3. 檢查日誌
docker compose -f docker-compose.agvc.yml logs postgres
```

#### 解決方案
```bash
# 重啟資料庫
docker compose -f docker-compose.agvc.yml restart postgres
sleep 30

# 檢查資料庫狀態
docker compose -f docker-compose.agvc.yml exec postgres psql -U agvc -d agvc -c "SELECT version();"

# 如果資料損壞，從備份恢復
docker compose -f docker-compose.agvc.yml down
docker volume rm rosagv_postgres_data
docker compose -f docker-compose.agvc.yml up -d postgres
sleep 30
docker compose -f docker-compose.agvc.yml exec postgres psql -U agvc -d agvc < /backup/latest_backup.sql
```

### 資料庫效能問題

#### 症狀識別
- 查詢響應緩慢
- Web 介面載入慢
- 系統整體效能下降

#### 診斷和最佳化
```bash
# 檢查資料庫效能指標
docker compose -f docker-compose.agvc.yml exec postgres psql -U agvc -d agvc -c "
SELECT 
    datname,                           -- 資料庫名稱
    numbackends as active_connections, -- 當前活躍連接數
    xact_commit,                       -- 提交的交易總數
    xact_rollback,                     -- 回滾的交易總數
    blks_read,                         -- 從磁碟讀取的區塊數
    blks_hit,                          -- 從快取命中的區塊數
    temp_files,                        -- 建立的臨時檔案數
    temp_bytes                         -- 臨時檔案使用的位元組數
FROM pg_stat_database 
WHERE datname = 'agvc';
"

# 💡 欄位說明與健康標準：
echo "
📊 資料庫效能指標解讀：
┌─────────────────────┬──────────────────────────────────┬─────────────────┐
│ 欄位                │ 說明                              │ 健康標準        │
├─────────────────────┼──────────────────────────────────┼─────────────────┤
│ active_connections  │ 當前活躍連接數                    │ < 50 (正常)     │
│ xact_commit         │ 成功提交的交易總數                │ 持續增長        │
│ xact_rollback       │ 回滾的交易總數                    │ < 10% 總交易數  │
│ blks_read           │ 磁碟讀取區塊數 (效能較差)        │ -               │
│ blks_hit            │ 快取命中區塊數 (效能較好)        │ -               │
│ 快取命中率          │ blks_hit/(blks_hit+blks_read)*100 │ > 95% (優秀)    │
│ temp_files          │ 臨時檔案數 (記憶體不足指標)      │ = 0 (理想)      │
│ temp_bytes          │ 臨時檔案大小                      │ = 0 (理想)      │
└─────────────────────┴──────────────────────────────────┴─────────────────┘

🔍 效能分析範例 (基於您的實際數據)：
  - 活躍連接數: 4 ✅ (正常範圍)
  - 交易成功率: 1801/(1801+16774) = 9.7% ⚠️ (回滾率過高，需檢查)
  - 快取命中率: 167548/(167548+741) = 99.6% ✅ (優秀)
  - 臨時檔案: 0 ✅ (無記憶體不足問題)
"

# 資料庫最佳化
docker compose -f docker-compose.agvc.yml exec postgres psql -U agvc -d agvc -c "
ANALYZE;
REINDEX DATABASE agvc;
VACUUM FULL;
"
```

## 🤖 ROS 2 故障

### 節點無法啟動

#### 症狀識別
- `ros2 node list` 缺少預期節點
- 節點啟動後立即退出
- 相關功能無法使用

#### 診斷和解決
```bash
# 1. 檢查 ROS 2 環境
echo $ROS_DISTRO
echo $RMW_IMPLEMENTATION

# 2. 手動啟動節點進行除錯
docker compose -f docker-compose.yml exec rosagv bash -c "
source /app/setup.bash && 
ros2 run agv_base agv_node --ros-args --log-level DEBUG
"

# 3. 檢查依賴
docker compose -f docker-compose.yml exec rosagv bash -c "
source /app/setup.bash && 
ros2 pkg list | grep agv
"

# 解決環境問題
export RMW_IMPLEMENTATION=rmw_zenohd
source /opt/ros/jazzy/setup.bash
all_source
```

### 主題通訊異常

#### 症狀識別
- 主題資料無法傳輸
- `ros2 topic echo` 無輸出
- 跨容器通訊失敗

#### 診斷和解決
```bash
# 1. 檢查主題列表
ros2 topic list -t

# 2. 測試發布和訂閱
ros2 topic pub /test_topic std_msgs/String "data: 'test'" &
ros2 topic echo /test_topic

# 3. 重啟 ROS 2 daemon
ros2 daemon stop
ros2 daemon start

# 4. 檢查 Zenoh 通訊
r zenoh-check
```

## 🚗 AGV 功能故障

### 狀態機異常

#### 症狀識別
- AGV 卡在某個狀態
- 狀態轉換不正常
- 任務執行中斷

#### 診斷和解決
```bash
# 1. 檢查 AGV 狀態
ros2 topic echo /agv_status

# 2. 檢查狀態機日誌
docker compose -f docker-compose.yml exec rosagv bash -c "
tail -f /tmp/agv.log | grep -i state
"

# 3. 發送重置命令
ros2 topic pub /agv_command agv_interfaces/Command "command_type: 'reset'"

# 4. 重啟 AGV 節點
docker compose -f docker-compose.yml exec rosagv bash -c "
source /app/setup.bash && 
ros2 lifecycle set /agv_node shutdown
sleep 5
ros2 run agv_base agv_node
"
```

### 硬體通訊失敗

#### 症狀識別
- 機械臂無回應
- 感測器資料異常
- 硬體狀態錯誤

#### 診斷和解決
```bash
# 1. 檢查硬體連接
ping 192.168.2.101  # PLC IP
lsusb  # USB 裝置

# 2. 測試 PLC 通訊
docker compose -f docker-compose.yml exec rosagv bash -c "
source /app/setup.bash && 
python3 -c '
from keyence_plc import KeyencePlcCom
plc = KeyencePlcCom(\"192.168.2.101\", 8501)
print(plc.connect())
print(plc.send_command(\"?K\\r\\n\"))
'
"

# 3. 重啟硬體服務
docker compose -f docker-compose.yml restart rosagv
```

## 📊 故障排除決策樹

### 系統無回應決策樹
```
系統無回應
├── Web 介面無法存取？
│   ├── Yes → 檢查 Nginx 和 Web 服務
│   └── No → 繼續下一步
├── 容器是否運行？
│   ├── No → 重啟容器服務
│   └── Yes → 檢查應用程式日誌
└── 網路是否正常？
    ├── No → 檢查網路配置
    └── Yes → 檢查系統資源
```

### AGV 功能異常決策樹
```
AGV 功能異常
├── ROS 2 節點是否運行？
│   ├── No → 重啟 ROS 2 服務
│   └── Yes → 繼續下一步
├── 硬體通訊是否正常？
│   ├── No → 檢查 PLC 連接
│   └── Yes → 檢查狀態機邏輯
└── Zenoh 通訊是否正常？
    ├── No → 重啟 Zenoh Router
    └── Yes → 檢查應用程式邏輯
```

## 🛠️ 故障預防措施

### 監控和預警
```bash
# 設置系統監控
cat << 'EOF' > /usr/local/bin/rosagv-health-monitor.sh
#!/bin/bash
# 每5分鐘檢查系統健康狀態
while true; do
    if ! r quick-diag > /tmp/health_check.log 2>&1; then
        echo "$(date): 系統健康檢查失敗" >> /var/log/rosagv-alerts.log
        # 發送告警通知
        mail -s "RosAGV Health Check Failed" chieu@ms43.hinet.net < /tmp/health_check.log
    fi
    sleep 300
done
EOF

chmod +x /usr/local/bin/rosagv-health-monitor.sh
nohup /usr/local/bin/rosagv-health-monitor.sh &
```

### 自動恢復機制
```bash
# 服務自動重啟腳本
cat << 'EOF' > /usr/local/bin/rosagv-auto-recovery.sh
#!/bin/bash
services=("rosagv" "agvc_server" "postgres" "nginx")

for service in "${services[@]}"; do
    if ! docker ps | grep -q "$service"; then
        echo "$(date): 重啟服務 $service" >> /var/log/rosagv-recovery.log
        docker compose -f /home/ct/EBD_agv/docker-compose.agvc.yml restart "$service"
    fi
done
EOF

chmod +x /usr/local/bin/rosagv-auto-recovery.sh
# 加入 cron 每分鐘檢查
echo "* * * * * /usr/local/bin/rosagv-auto-recovery.sh" | crontab -
```

## 📋 故障排除檢查清單

### 基本診斷檢查
- [ ] 執行 `r quick-diag` 快速診斷
- [ ] 檢查所有容器運行狀態
- [ ] 驗證網路連接和端口
- [ ] 查看系統資源使用情況
- [ ] 檢查關鍵服務日誌

### 深度診斷檢查
- [ ] 分析錯誤日誌模式
- [ ] 測試各組件獨立功能
- [ ] 驗證配置檔案正確性
- [ ] 檢查硬體連接狀態
- [ ] 測試資料庫完整性

### 修復後驗證
- [ ] 所有服務正常運行
- [ ] 功能測試通過
- [ ] 效能指標正常
- [ ] 無錯誤日誌產生
- [ ] 用戶可以正常使用

## 📞 緊急聯絡資訊

### 擎添工業 (Ching-Tech)
- **公司電話**: (02)2903-2788
- **傳真**: (02)2903-9518
- **電子郵件**: chieu@ms43.hinet.net
- **公司地址**: 248 新北市五股區成泰路一段194-8號 J棟
- **網站**: https://ching-tech.com/

### 系統管理員
- **緊急聯絡**: chieu@ms43.hinet.net
- **電話**: (02)2903-2788
- **值班時間**: 週一至週五 09:00-18:00

### 技術支援
- **RosAGV 開發團隊**: chieu@ms43.hinet.net
- **PLC 技術支援**: chieu@ms43.hinet.net
- **網路技術支援**: chieu@ms43.hinet.net

---

**相關文檔：**
- [系統診斷](../operations/system-diagnostics.md) - 診斷工具詳解
- [維護操作](maintenance.md) - 日常維護指導
- [部署指導](deployment.md) - 系統部署操作
- [開發環境](development.md) - 開發環境故障排除