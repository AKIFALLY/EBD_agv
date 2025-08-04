# 快速上手指導

## 🚀 立即開始使用 RosAGV

這個指導將幫助您在 15 分鐘內快速上手 RosAGV 系統，從環境準備到基本操作。

## ⚠️ 前提條件

### 系統要求
- **作業系統**：Ubuntu 24.04 LTS
- **Docker**：Docker Compose V2
- **硬體**：最少 8GB RAM，推薦 16GB
- **網路**：穩定的網路連接

### 環境準備
```bash
# 1. 確保 Docker 和 Docker Compose V2 已安裝
docker --version          # 應顯示 Docker 版本
docker compose version    # 應顯示 Compose V2 版本

# 2. 設定 RosAGV 工具路徑 (重要！)
echo 'export PATH="/home/ct/RosAGV:$PATH"' >> ~/.bashrc
source ~/.bashrc

# 3. 驗證工具可用性
which r                   # 應顯示 /home/ct/RosAGV/r
r                        # 顯示可用工具列表
```

## 🏗️ 第一步：啟動系統

### 啟動 AGVC 管理系統
```bash
# 進入 RosAGV 目錄
cd /home/ct/RosAGV

# 啟動 AGVC 管理系統（包含資料庫、Web 服務）
docker compose -f docker-compose.agvc.yml up -d

# 檢查啟動狀態
r containers-status
```

**預期結果**：
```
✅ agvc_server - 運行中
✅ postgres - 運行中  
✅ nginx - 運行中
✅ pgadmin - 運行中
```

### 啟動 AGV 車載系統（可選）
```bash
# 如果有實際 AGV 硬體或需要測試車載功能
docker compose -f docker-compose.yml up -d

# 檢查狀態
docker ps
```

## 🌐 第二步：存取 Web 介面

### 管理員介面 (AGVCUI)
```
URL: http://localhost:8001
功能: 完整的系統管理和監控
用途: 系統管理員使用
```

### 操作員介面 (OPUI)  
```
URL: http://localhost:8002
功能: 簡化的叫車和狀態監控
用途: 生產線操作員使用
```

### 文檔系統
```
URL: http://localhost/docs/
功能: 完整的技術文檔和指導
用途: 開發和維護人員使用
```

### 資料庫管理 (pgAdmin)
```
URL: http://localhost:5050
帳號: yazelin@ching-tech.com
密碼: password
用途: 資料庫管理和查詢
```

## 🔧 第三步：基本操作

### 系統健康檢查
```bash
# 完整系統健康檢查
r agvc-check

# 快速診斷
r quick-diag

# 網路連接檢查
r network-check
```

### 進入容器進行開發
```bash
# 進入 AGVC 管理容器
docker compose -f docker-compose.agvc.yml exec agvc_server bash

# 載入 ROS 2 環境
source /app/setup.bash && all_source

# 檢查 ROS 2 環境
ros2 node list
ros2 topic list
```

### 檢查資料庫
```bash
# 進入 AGVC 容器
docker compose -f docker-compose.agvc.yml exec agvc_server bash

# 載入環境並測試資料庫連接
source /app/setup.bash && python3 -c "
from db_proxy.database import get_database
db = get_database()
print('資料庫連接成功！')
print(f'資料庫 URL: {db.url}')
"
```

## 📊 第四步：監控系統狀態

### Web 介面監控
1. **開啟 AGVCUI**：http://localhost:8001
2. **查看 AGV 狀態**：系統會顯示所有 AGV 的即時狀態
3. **任務監控**：可以看到正在執行和排隊中的任務
4. **系統告警**：任何異常都會在界面上顯示

### 命令列監控
```bash
# 即時系統狀態
watch -n 5 'r agvc-check'

# 查看容器日誌
docker compose -f docker-compose.agvc.yml logs -f agvc_server

# 查看特定服務日誌
docker compose -f docker-compose.agvc.yml logs -f postgres
```

## 🚗 第五步：模擬 AGV 操作（進階）

### 手動發送測試指令
```bash
# 進入 AGVC 容器
docker compose -f docker-compose.agvc.yml exec agvc_server bash
source /app/setup.bash && all_source

# 發送測試 AGV 狀態
ros2 topic pub /agv_status agv_interfaces/AGVStatus "
agv_id: 'test_agv_01'
status: 'IDLE'
position: {x: 0.0, y: 0.0, theta: 0.0}
battery_level: 95
" --once

# 查看系統回應
ros2 topic echo /agv_status
```

### 測試任務分配
```bash
# 發送測試任務
ros2 service call /create_task ai_wcs_interfaces/CreateTask "
task: {
  task_id: 'test_task_001'
  agv_type: 'cargo_mover'
  work_id: 1
  priority: 1
}
"
```

## 🔍 第六步：診斷和故障排除

### 常用診斷指令
```bash
# 檢查所有服務狀態
r agvc-check

# 檢查網路連接
r network-check

# 檢查 Zenoh 通訊
r zenoh-check

# 查看系統日誌錯誤
r log-errors
```

### 常見問題解決

#### 問題1：容器無法啟動
```bash
# 檢查端口佔用
ss -tulpn | rg "(8000|8001|8002|5432)"

# 檢查磁碟空間
df -h

# 重新建置映像
docker compose -f docker-compose.agvc.yml build --no-cache
```

#### 問題2：Web 介面無法存取
```bash
# 檢查 nginx 配置
docker compose -f docker-compose.agvc.yml exec nginx nginx -t

# 重啟 nginx
docker compose -f docker-compose.agvc.yml restart nginx

# 檢查服務端口
curl http://localhost:8001/
```

#### 問題3：資料庫連接失敗
```bash
# 檢查 PostgreSQL 狀態
docker compose -f docker-compose.agvc.yml exec postgres pg_isready -U agvc

# 測試連接
docker compose -f docker-compose.agvc.yml exec postgres psql -U agvc -d agvc -c "SELECT version();"
```

## 📚 第七步：探索更多功能

### 開發相關
- [開發環境設定](../operations/development.md)
- [ROS 2 開發指導](../technical-details/ros2-integration.md)
- [資料庫操作](../technical-details/database-design.md)

### 業務流程  
- [眼鏡生產流程](../business-processes/eyewear-production.md)
- [AGV 車型介紹](../agv-vehicles/vehicle-types.md)
- [工作流程設計](../business-processes/indoor-process.md)

### 系統管理
- [部署指導](../operations/deployment.md)
- [維護操作](../operations/maintenance.md)
- [故障排除](../operations/troubleshooting.md)

## 🎯 成功檢查清單

完成以下檢查項目，確保系統正常運行：

### 基礎環境
- [ ] Docker 和 Docker Compose V2 正常運行
- [ ] RosAGV 工具路徑已設定（`r` 指令可用）
- [ ] 系統有足夠的記憶體和磁碟空間

### 服務啟動
- [ ] AGVC 所有容器正常運行（agvc_server、postgres、nginx、pgAdmin）
- [ ] Web 介面可以正常存取（8001、8002、5050）
- [ ] 資料庫連接正常

### 基本功能
- [ ] 系統健康檢查通過（`r agvc-check`）
- [ ] ROS 2 環境在容器內正常工作
- [ ] 可以查看 AGV 狀態和任務資訊

### 進階功能（可選）
- [ ] AGV 車載系統正常運行（如果需要）
- [ ] 跨環境 ROS 2 通訊正常
- [ ] 可以發送和接收測試指令

## 🚀 下一步行動

### 根據您的角色：

**👨‍💼 產品經理**
→ [眼鏡生產流程](../business-processes/eyewear-production.md)

**👨‍💻 系統工程師** 
→ [系統架構詳解](../system-architecture/dual-environment.md)

**🔧 維運工程師**
→ [維護操作指導](../operations/maintenance.md)

**🚗 AGV 專家**
→ [AGV 車型技術](../agv-vehicles/vehicle-types.md)

## 💡 實用提示

### 開發技巧
- 使用 `watch` 指令即時監控系統狀態
- 善用 `r` 工具集進行快速診斷
- 定期檢查容器日誌了解系統運行情況

### 除錯技巧
- 遇到問題先執行 `r quick-diag` 快速診斷
- 使用 `docker compose logs` 查看詳細錯誤資訊
- 檢查網路連接和端口佔用情況

### 效能最佳化
- 定期清理 Docker 映像和容器：`docker system prune`
- 監控系統資源使用：`htop`, `docker stats`
- 最佳化資料庫查詢和索引設計

---

🎉 **恭喜！** 您已經成功完成 RosAGV 快速上手指導。現在可以開始探索更深入的功能和進行實際開發了！

如有任何問題，請參考 [故障排除指南](../operations/troubleshooting.md) 或查看系統日誌進行診斷。