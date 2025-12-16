# RosAGV 本地設備配置

## 概述

此目錄包含設備配置範本。每台 AGV/AGVC 設備需要在 `/app/config/local/` 創建自己的本地配置文件。

**重要**: `/app/config/local/` 目錄不被 git 追蹤，確保每台設備的配置獨立維護。

## 快速開始

### 1. 創建本地配置目錄

```bash
mkdir -p /app/config/local
```

### 2. 複製範本

```bash
cp /app/config/local.example/device.yaml /app/config/local/device.yaml
```

### 3. 編輯配置

```bash
nano /app/config/local/device.yaml
# 修改 agv_name 為實際設備名稱
```

### 4. 重啟容器

```bash
# AGV 容器
docker compose -f docker-compose.yml restart rosagv

# AGVC 容器
docker compose -f docker-compose.agvc.yml restart agvc_server
```

### 5. 驗證

```bash
# 進入容器後檢查
echo $AGV_NAME
# 應輸出: loader02（或你設置的名稱）

# 檢查 ROS2 節點
ros2 node list
# 應顯示 /loader02/... 開頭的節點
```

## 支援的設備名稱

| 名稱 | 類型 | 說明 | 自動衍生的 Launch Package |
|------|------|------|---------------------------|
| `loader01`, `loader02` | Loader AGV | 裝載型 AGV | `loader_agv` |
| `cargo01`, `cargo02` | Cargo AGV | 貨物搬運型 AGV | `cargo_mover_agv` |
| `unloader01`, `unloader02` | Unloader AGV | 卸載型 AGV | `unloader_agv` |
| `agvc01` | AGVC | 控制系統 | - |

## 自動衍生值

只需設置 `agv_name`，以下值會自動衍生：

| 衍生值 | 規則 | 範例 |
|--------|------|------|
| ROS Namespace | `/${agv_name}` | `/loader02` |
| Launch Package | 根據名稱前綴 | `loader_agv` |
| Config File | `${agv_name}_config.yaml` | `loader02_config.yaml` |
| Room ID | 名稱末兩位數字 | `2` |

## 配置文件格式

最簡配置（推薦）：

```yaml
agv_name: "loader02"
```

完整配置（可選覆蓋）：

```yaml
agv_name: "loader02"

overrides:
  launch_package: "loader_agv"
  config_file: "loader02_config.yaml"
  room_id: 2
```

## 向後兼容性

- 若 `/app/config/local/device.yaml` 不存在，系統會降級到 MAC 地址識別
- `AGV_ID` 環境變量仍然可用（等同於 `AGV_NAME`）
- 現有設備無需立即遷移

## 故障排除

### 配置未生效

1. 確認配置文件路徑正確：`/app/config/local/device.yaml`
2. 確認 YAML 格式正確（無語法錯誤）
3. 重新啟動容器

### 環境變量為空

```bash
# 檢查配置文件是否存在
cat /app/config/local/device.yaml

# 手動執行識別腳本查看錯誤
source /app/scripts/config_driven_device_detector.bash
```

### 節點啟動失敗

```bash
# 檢查對應的設備配置文件是否存在
ls -la /app/config/agv/${AGV_NAME}_config.yaml

# 查看啟動日誌
tail -f /tmp/agv_launch.log
```
