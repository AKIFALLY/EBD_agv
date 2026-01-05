# AGV 電腦初始設定指南

本文件說明如何將開發環境的 RosAGV 系統部署到新的 AGV 電腦上。

## 目錄

1. [資料同步](#1-資料同步)
2. [設備識別配置](#2-設備識別配置)
3. [AGV 參數配置](#3-agv-參數配置)
4. [路徑檔案配置](#4-路徑檔案配置)
5. [AGVC 連線配置](#5-agvc-連線配置)
6. [驗證與啟動](#6-驗證與啟動)

---

## 1. 資料同步

### 從開發機同步程式碼到 AGV 電腦

使用 rsync 進行差異同步，排除不必要的編譯產物：

```bash
rsync -avz --partial --progress \
    --exclude='*/log/' \
    --exclude='*/build/' \
    --exclude='*/install/' \
    --exclude='*/__pycache__/' \
    --exclude='*.pyc' \
    --exclude='.git/' \
    -e "ssh -p 2222 -o ServerAliveInterval=60 -o ServerAliveCountMax=3" \
    /home/akifally/EBD_agv ct@192.168.2.10:/home/ct
```

**參數說明：**

| 參數 | 說明 |
|------|------|
| `-avz` | 歸檔模式、詳細輸出、壓縮傳輸 |
| `--partial` | 支援斷點續傳 |
| `--progress` | 顯示傳輸進度 |
| `--exclude` | 排除編譯產物和快取 |
| `-e "ssh -p 2222 ..."` | 指定 SSH 端口和 keep-alive |

**根據實際環境修改：**
- `/home/akifally/EBD_agv` → 來源路徑
- `ct@192.168.2.10` → 目標主機用戶和 IP
- `-p 2222` → SSH 端口（預設 22 可省略 `-p` 參數）

---

## 2. 設備識別配置

### 修改 `/app/config/local/device.yaml`

此檔案定義 AGV 的唯一識別名稱：

```yaml
# 設定 AGV 名稱（必填）
# 支援格式：
#   - loader01, loader02     → 裝載型 AGV
#   - cargo01, cargo02       → 貨物搬運型 AGV
#   - unloader01, unloader02 → 卸載型 AGV
#   - FORK01, FORK02         → 堆高機型 AGV

agv_name: "FORK01"    # ⬅️ 修改為實際 AGV 名稱
```

**命名規則：**
- 名稱必須與 AGVC 資料庫中的 AGV 名稱一致
- 系統會根據名稱前綴自動選擇對應的啟動套件

---

## 3. AGV 參數配置

### 新增 `/app/config/agv/{agv_name}_config.yaml`

根據 `device.yaml` 中設定的 `agv_name`，建立對應的配置檔案。

**範例：`FORK01_config.yaml`**

```yaml
FORK01:
  agv_core_node:
    ros__parameters:
      device_id: "FORK01"
      device_type: "forklift"
      description: "堆高機型 AGV #1"
      room_id: 1
      max_speed: 1.0
      max_payload: 1000
      safety_distance: 0.5

  plc_service:
    ros__parameters:
      plc_ip: "192.168.2.100"    # ⬅️ AGV 車載 PLC IP
      plc_port: 8501             # ⬅️ PLC 通訊端口
```

**重要參數說明：**

| 參數 | 說明 |
|------|------|
| `device_id` | 必須與 `agv_name` 一致 |
| `device_type` | AGV 類型：loader / cargo / unloader / forklift |
| `room_id` | 作業區域 ID |
| `plc_ip` | 車載 PLC 的 IP 位址 |
| `plc_port` | PLC 通訊端口（Keyence 預設 8501） |

---

## 4. 路徑檔案配置

### 修改 `/app/config/path.yaml`

指定 AGV 使用的路徑資料檔案：

```yaml
path_data_file:
  file_path: "/app/config/path/EBD_20260102_A.json"    # ⬅️ 修改為實際路徑檔
```

### 路徑檔案位置

路徑 JSON 檔案存放於 `/app/config/path/` 目錄：

```
/app/config/path/
├── EBD_20260102_A.json           # 現場路徑檔（依專案命名）
├── alan_room2_loader_path_*.json # 測試用路徑檔
└── ...
```

**注意事項：**
- 路徑檔案由路徑編輯工具產生
- 不同現場/專案使用不同的路徑檔
- 確保檔案路徑正確且檔案存在

---

## 5. AGVC 連線配置

### 修改 `/app/config/agvc/base_config.yaml`

設定 AGV 連接到 AGVC 伺服器的網路參數：

```yaml
base_agvc_parameters:
  system:
    update_rate: 10.0
    timeout_threshold: 30.0

  network:
    agvc_server_ip: "192.168.12.99"   # ⬅️ AGVC 伺服器 IP
    zenoh_router_port: 7447           # Zenoh 通訊端口
    web_api_port: 8000                # Web API 端口

database_base_config:
  postgresql:
    host: "192.168.12.99"             # ⬅️ 資料庫主機 IP（通常與 AGVC 相同）
    port: 5432
    database: "agvc"
    username: "agvc"
    password: "password"
```

**重要參數：**

| 參數 | 說明 |
|------|------|
| `agvc_server_ip` | AGVC 伺服器的 IP 位址 |
| `zenoh_router_port` | Zenoh RMW 通訊端口 |
| `web_api_port` | AGVC Web API 端口 |
| `postgresql.host` | PostgreSQL 資料庫主機 |

---

## 6. 驗證與啟動

### 6.1 檢查配置檔案

```bash
# 進入 AGV 容器
cd ~/EBD_agv
docker compose -f docker-compose.yml exec rosagv bash

# 檢查配置
cat /app/config/local/device.yaml
cat /app/config/path.yaml
ls -la /app/config/agv/
```

### 6.2 重建工作空間

```bash
# 在容器內執行
source /app/setup.bash
ba    # build_all
sa    # all_source
```

### 6.3 啟動 AGV 節點

```bash
# 在容器內執行
ros2 launch launch_ws agv_launch.py
```

### 6.4 驗證連線

```bash
# 檢查 ROS2 節點
ros2 node list

# 檢查與 AGVC 的通訊
ros2 topic list | grep agv

# 檢查 PLC 連線狀態
ros2 topic echo /plc_status
```

---

## 配置檔案總覽

| 檔案 | 用途 | 必須修改 |
|------|------|:--------:|
| `/app/config/local/device.yaml` | AGV 識別名稱 | ✅ |
| `/app/config/agv/{name}_config.yaml` | AGV 參數（PLC IP 等） | ✅ |
| `/app/config/path.yaml` | 路徑檔案指定 | ✅ |
| `/app/config/agvc/base_config.yaml` | AGVC 伺服器連線 | ✅ |

---

## 常見問題

### Q: rsync 傳輸中斷怎麼辦？
A: 使用 `--partial` 參數支援斷點續傳，重新執行相同指令即可繼續。

### Q: AGV 無法連接 AGVC？
A: 檢查以下項目：
1. `base_config.yaml` 中的 `agvc_server_ip` 是否正確
2. 網路連通性：`ping <agvc_ip>`
3. Zenoh Router 是否運行：端口 7447

### Q: PLC 通訊失敗？
A: 確認 `{agv_name}_config.yaml` 中的 `plc_ip` 和 `plc_port` 設定正確。

### Q: 找不到路徑檔案？
A: 確認 `path.yaml` 中指定的檔案路徑存在於 `/app/config/path/` 目錄。
