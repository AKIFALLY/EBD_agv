# RosAGV 統一設備身份識別和配置管理系統部署指南

## 📋 **系統概述**

本系統實現了 6 台 AGV 車輛 + 1 台 AGVC 管理節點的統一設備身份識別、差異化配置和服務啟動。

### 支援的設備
- **AGV 車輛 (6 台)**：cargo01/02, loader01/02, unloader01/02
- **AGVC 管理節點 (1 台)**：agvc01

### 核心特性
- ✅ 基於 MAC 地址的可靠硬體識別機制
- ✅ 手動覆蓋設定支援（最高優先級）
- ✅ 動態 ROS 2 launch 配置
- ✅ 雙環境統一管理但配置隔離
- ✅ 完善的錯誤處理和降級機制

## 🚀 **快速部署步驟**

### 1. 收集硬體 MAC 地址

在每台實際設備上執行以下指令收集 MAC 地址：

```bash
# 在每台 AGV/AGVC 設備上執行
cat /sys/class/net/enp4s0/address
# 或者
cat /sys/class/net/eth0/address
```

### 2. 更新硬體映射配置

編輯 `/app/config/hardware_mapping.yaml`，將收集到的真實 MAC 地址更新到對應設備配置中：

```yaml
agv_devices:
  cargo01:
    mac_addresses:
      primary: "實際的MAC地址1"    # 替換為真實值
      backup: "備用MAC地址1"      # 替換為真實值
  # ... 其他設備
```

### 3. 測試身份識別

在每台設備上測試身份識別功能：

```bash
# AGV 環境測試
export CONTAINER_TYPE="agv"
bash /app/scripts/simple_unified_detector.bash

# AGVC 環境測試
export CONTAINER_TYPE="agvc"
bash /app/scripts/simple_unified_detector.bash
```

### 4. 驗證配置載入

檢查身份檔案和環境變數：

```bash
# 檢查統一設備身份
cat /app/.device_identity

# 檢查 AGV 專屬身份
cat /app/.agv_identity

# 檢查 AGVC 專屬身份
cat /app/.agvc_identity
```

## 🔧 **手動設定模式**

當自動識別失敗時，可使用手動設定模式：

### AGV 手動設定
```bash
export CONTAINER_TYPE="agv"
export MANUAL_DEVICE_ID="cargo01"  # 或其他有效的 AGV ID
bash /app/scripts/simple_unified_detector.bash
```

### AGVC 手動設定
```bash
export CONTAINER_TYPE="agvc"
export MANUAL_DEVICE_ID="agvc01"
bash /app/scripts/simple_unified_detector.bash
```

### 有效的設備 ID
- **AGV**: cargo01, cargo02, loader01, loader02, unloader01, unloader02
- **AGVC**: agvc01

## 📁 **配置檔案結構**

```
/app/config/
├── hardware_mapping.yaml           # 統一硬體映射配置
├── agv/                            # AGV 配置目錄
│   ├── base_config.yaml            # AGV 基礎共用配置
│   ├── cargo01_config.yaml         # cargo01 專屬配置
│   ├── cargo02_config.yaml         # cargo02 專屬配置
│   ├── loader01_config.yaml        # loader01 專屬配置
│   ├── loader02_config.yaml        # loader02 專屬配置
│   ├── unloader01_config.yaml      # unloader01 專屬配置
│   └── unloader02_config.yaml      # unloader02 專屬配置
└── agvc/                           # AGVC 配置目錄
    ├── base_config.yaml            # AGVC 基礎共用配置
    └── agvc01_config.yaml          # agvc01 專屬配置
```

## 🚀 **Launch 檔案動態配置**

所有 AGV launch 檔案已修改為支援動態配置：

### 環境變數支援
- `AGV_ID`: AGV 設備 ID
- `ROS_NAMESPACE`: ROS 2 命名空間
- `DEVICE_CONFIG_FILE`: 設備專屬配置檔案路徑

### Launch 檔案位置
- `app/agv_ws/src/cargo_mover_agv/launch/launch.py`
- `app/agv_ws/src/loader_agv/launch/launch.py`
- `app/agv_ws/src/unloader_agv/launch/launch.py`

## 🔍 **故障排除**

### 常見問題

#### 1. MAC 地址識別失敗
```bash
# 檢查網路介面
ls /sys/class/net/

# 檢查 MAC 地址
cat /sys/class/net/enp4s0/address
cat /sys/class/net/eth0/address
```

#### 2. 配置檔案不存在
```bash
# 檢查配置檔案
ls -la /app/config/agv/
ls -la /app/config/agvc/

# 驗證配置檔案語法
python3 -c "import yaml; yaml.safe_load(open('/app/config/agv/cargo01_config.yaml'))"
```

#### 3. 環境變數未設定
```bash
# 檢查環境變數
env | grep -E "AGV_ID|AGVC_ID|DEVICE_ID|CONTAINER_TYPE"

# 重新執行身份識別
source /app/scripts/simple_unified_detector.bash
```

### 除錯模式

啟用除錯模式獲取詳細日誌：

```bash
export DEVICE_DEBUG="true"
bash /app/scripts/simple_unified_detector.bash

# 檢查日誌
cat /tmp/device_identification.log
cat /tmp/device_hardware_info.log
```

## 🛠️ **管理指令**

在容器內使用以下指令管理設備身份：

```bash
# 載入 setup.bash
source /app/setup.bash

# 檢查設備身份資訊
check_device_identity

# 手動觸發設備身份識別
identify_device_manual

# 顯示設備配置資訊
show_device_config
```

## 📊 **系統監控**

### 身份識別狀態檢查
```bash
# 檢查身份識別成功狀態
grep "IDENTIFICATION_SUCCESS" /app/.device_identity

# 檢查識別方法
grep "IDENTIFICATION_METHOD" /app/.device_identity

# 檢查識別時間
grep "IDENTIFICATION_TIME" /app/.device_identity
```

### 日誌監控
```bash
# 監控識別日誌
tail -f /tmp/device_identification.log

# 監控硬體資訊日誌
tail -f /tmp/device_hardware_info.log
```

## 🔒 **安全注意事項**

1. **MAC 地址保護**: 確保 MAC 地址映射檔案的安全性
2. **配置檔案權限**: 適當設定配置檔案的讀寫權限
3. **手動覆蓋**: 謹慎使用手動覆蓋功能，避免誤配置

## 📝 **維護建議**

1. **定期備份**: 定期備份硬體映射配置和設備專屬配置
2. **版本控制**: 使用 Git 追蹤配置檔案變更
3. **測試驗證**: 在生產環境部署前充分測試
4. **文檔更新**: 保持部署文檔與實際配置同步

## 🆘 **技術支援**

如遇到問題，請提供以下資訊：
- 設備類型和 ID
- 錯誤訊息和日誌
- 硬體資訊（MAC 地址、網路介面）
- 環境變數設定

---

**最後更新**: 2025-01-18  
**版本**: v1.0.0
