# AGV 名稱變更設定指南

本文件說明當需要變更 AGV 設備名稱 (AGV_NAME) 時，所需修改的所有設定項目。

## 概述

AGV_NAME 是設備識別系統的唯一識別變數，用於：
- ROS2 節點命名空間 (`/AGV_NAME/...`)
- Launch 套件自動選擇
- 配置檔案自動載入
- AGVC 資料庫任務匹配

---

## 必要設定項目

### 1. 本地設備配置（AGV 端）

**檔案位置：** `/app/config/local/device.yaml`

```yaml
# 設定 AGV 名稱
agv_name: "loader02"

# 可選覆蓋項（一般無需設置）
overrides:
  # launch_package: "loader_agv"      # 預設自動判斷
  # config_file: "loader02_config.yaml"
  # room_id: 2
```

**名稱前綴與 Launch 套件對應：**
| 名稱前綴 | Launch 套件 |
|---------|------------|
| `loader*` | `loader_agv` |
| `cargo*` | `cargo_mover_agv` |
| `unloader*` | `unloader_agv` |

**範例：**
- `loader01` → 使用 `loader_agv` 套件
- `cargo02` → 使用 `cargo_mover_agv` 套件
- `unloader01` → 使用 `unloader_agv` 套件

---

### 2. AGV 配置檔案

**檔案位置：** `/app/config/agv/<AGV_NAME>_config.yaml`

若使用新的 AGV_NAME，需確保對應的配置檔存在：

```bash
# 範例：新增 loader03 配置
cp /app/config/agv/loader01_config.yaml /app/config/agv/loader03_config.yaml
```

**配置檔內容範例：**
```yaml
agv_id: "loader03"
agv_type: "loader"
robot_arm_enabled: true
vision_system_enabled: true

# 端口配置
agv_ports:
  port_1: {x: 0.5, y: 0.3, z: 0.1}
  port_2: {x: 0.5, y: -0.3, z: 0.1}
```

---

### 3. AGVC 資料庫設定（AGVC 端）

AGV 需要在 AGVC 資料庫的 `agv` 表中註冊才能接收任務。

**透過 Web API 新增 AGV：**
```bash
curl -X POST "http://<AGVC_IP>:8000/api/v1/agv/" \
  -H "Content-Type: application/json" \
  -d '{
    "name": "loader03",
    "model": "loader",
    "description": "Loader AGV #3",
    "enable": 1
  }'
```

**透過 SQL 直接新增：**
```sql
INSERT INTO agv (name, model, description, enable, x, y, heading)
VALUES ('loader03', 'loader', 'Loader AGV #3', 1, 0.0, 0.0, 0.0);
```

**重要：** `agv.name` 必須與 `AGV_NAME` 完全一致，Web API 任務過濾依賴此欄位。

---

## AGVC 網路配置

**檔案位置：** `/app/config/agvc/base_config.yaml`

AGV 透過此配置連接 AGVC Web API：

```yaml
base_agvc_parameters:
  network:
    agvc_server_ip: "192.168.10.3"   # AGVC 伺服器 IP
    zenoh_router_port: 7447
    web_api_port: 8000
```

---

## 變更流程

### 步驟 1：修改本地設備配置

```bash
# 編輯設備配置
nano /app/config/local/device.yaml

# 設定新的 AGV 名稱
agv_name: "loader03"
```

### 步驟 2：確認 AGV 配置檔存在

```bash
# 檢查配置檔是否存在
ls /app/config/agv/loader03_config.yaml

# 若不存在，複製並修改
cp /app/config/agv/loader01_config.yaml /app/config/agv/loader03_config.yaml
```

### 步驟 3：在 AGVC 資料庫註冊（首次使用新名稱時）

透過 AGVCUI 或 API 在資料庫新增 AGV 記錄。

### 步驟 4：重啟 AGV 容器

```bash
# 重啟容器使設定生效
docker compose restart rosagv

# 或重啟 AGV Launch
docker compose exec rosagv bash -i -c "source /app/setup.bash && agv_source && manage_agv_launch restart"
```

### 步驟 5：驗證

```bash
# 檢查節點命名空間
docker compose exec rosagv bash -c "source /app/setup.bash && ros2 node list"

# 預期輸出：
# /loader03/agv_core_node
# /loader03/joy_linux_node
# /loader03/plc_service
```

---

## 環境變數說明

| 變數名 | 說明 | 來源 |
|--------|------|------|
| `AGV_NAME` | AGV 識別名稱 | `/app/config/local/device.yaml` |
| `AGV_ID` | 向後兼容別名（與 AGV_NAME 相同） | 自動衍生 |
| `AGV_LAUNCH_PACKAGE` | Launch 套件名稱 | 根據名稱前綴自動判斷 |
| `ROS_NAMESPACE` | ROS2 命名空間 | 自動衍生為 `/${AGV_NAME}` |

---

## 常見問題

### Q1: 變更 AGV_NAME 後節點無法啟動
**原因：** 配置檔不存在
**解決：** 確認 `/app/config/agv/<AGV_NAME>_config.yaml` 存在

### Q2: AGV 無法接收任務
**原因：** AGVC 資料庫未註冊該 AGV
**解決：** 在 AGVC 資料庫 `agv` 表中新增對應記錄

### Q3: Web API 連線失敗
**原因：** AGVC IP 設定錯誤
**解決：** 檢查 `/app/config/agvc/base_config.yaml` 中的 `agvc_server_ip`

### Q4: Launch 套件選擇錯誤
**原因：** AGV_NAME 前綴不符合命名規則
**解決：** 使用正確的前綴（loader/cargo/unloader）或在 `device.yaml` 中明確指定 `launch_package`

---

## 相關檔案一覽

| 檔案 | 位置 | 說明 |
|------|------|------|
| 本地設備配置 | `/app/config/local/device.yaml` | AGV_NAME 主要設定位置 |
| 配置範本 | `/app/config/local.example/device.yaml` | 設定檔範本（Git 追蹤） |
| AGV 配置 | `/app/config/agv/<name>_config.yaml` | 各 AGV 專屬配置 |
| AGVC 網路配置 | `/app/config/agvc/base_config.yaml` | AGVC 伺服器 IP 設定 |
| 設備識別腳本 | `/app/scripts/config_driven_device_detector.bash` | 讀取並設定環境變數 |
