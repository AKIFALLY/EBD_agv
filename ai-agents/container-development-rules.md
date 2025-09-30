# AI Agent 容器開發原則

## 容器架構
- **AGV 容器**: `rosagv`, host 網路, 硬體直接存取
- **AGVC 容器**: `agvc_server`, bridge 網路 (192.168.100.100)
- **配置檔案**:
  - AGV: `docker-compose.yml`
  - AGVC: `docker-compose.agvc.yml`

## 容器進入
```bash
# [宿主機] 必須在 ~/RosAGV 目錄執行
cd ~/RosAGV

# [宿主機] 進入 AGV 容器
docker compose -f docker-compose.yml exec rosagv bash

# [宿主機] 進入 AGVC 容器
docker compose -f docker-compose.agvc.yml exec agvc_server bash
```

## 環境載入
```bash
# [容器內] 環境載入
all_source    # 或 sa (智能載入)
agv_source    # AGV 工作空間
agvc_source   # AGVC 工作空間
```

## 指令執行規則

### ROS2 指令必須在容器內
```bash
# ❌ 錯誤：宿主機執行
ros2 topic list

# ✅ 正確：[宿主機] 透過 docker exec 執行容器內命令
docker compose -f docker-compose.agvc.yml exec agvc_server bash -c "source /app/setup.bash && ros2 topic list"
```

### bash -i 技巧（解決 alias 和超時問題）
```bash
# ✅ [宿主機] 使用 bash -i 執行複雜指令
docker compose -f docker-compose.agvc.yml exec agvc_server bash -i -c "source /app/setup.bash && agvc_source && manage_web_api_launch stop && ba && sa && manage_web_api_launch start"
```

## 服務管理
```bash
# [容器內] 服務管理（載入 setup.bash 後可用）
manage_web_api_launch {start|stop|restart|status}
manage_zenoh {start|stop|restart|status}
```

## 工具使用
```bash
# [宿主機] 統一工具（需設定 PATH）
r containers-status    # 容器狀態
r agvc-check          # AGVC 健康檢查
r quick-diag          # 快速診斷

# [容器內] 系統檢查
check_system_status
check_zenoh_status
```

## 網路配置
- **Zenoh Router**: Port 7447
- **Web API**: Port 8000-8002
- **PostgreSQL**: Port 5432
- **跨容器通訊**: 透過 Zenoh RMW

## 環境識別
```bash
pwd
# /home/ct/RosAGV = 宿主機
# /app = 容器內
```

## 常見問題快速解決

### 容器無法啟動時診斷步驟
```bash
# 1. [宿主機] 檢查容器狀態
docker compose -f docker-compose.agvc.yml ps

# 2. [宿主機] 查看錯誤日誌
docker compose -f docker-compose.agvc.yml logs agvc_server

# 3. 檢查端口衝突
ss -tulpn | grep -E "(8000|8001|8002|5432|7447)"
# 如有衝突，停止佔用的服務或修改端口

# 4. 檢查磁碟空間
df -h
# 如空間不足：docker system prune -f

# 5. [宿主機] 嘗試重新啟動
docker compose -f docker-compose.agvc.yml up -d
```

### Zenoh 連接失敗
```bash
# [宿主機] 重啟 Zenoh Router
docker compose -f docker-compose.agvc.yml restart agvc_server
```

### Web 服務無回應
```bash
# [宿主機] 透過 docker exec 重啟容器內服務
docker compose -f docker-compose.agvc.yml exec agvc_server bash -i -c "source /app/setup.bash && agvc_source && manage_web_api_launch restart"
```

## 關鍵規則
1. **ROS2 只在容器內**: 宿主機無 ROS2 環境
2. **使用 bash -i**: 解決 alias 載入和超時
3. **工作目錄**: 必須在 ~/RosAGV 執行 docker compose
4. **環境載入**: 進入容器後先執行 `all_source`
5. **跨容器通訊**: 確保 Zenoh Router 運行 (Port 7447)