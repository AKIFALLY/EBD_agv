# Scripts 目錄說明

此目錄包含 RosAGV 系統使用的各種工具腳本。

## 📂 目錄結構

### 核心系統腳本

#### config_driven_device_detector.bash
- **用途**: 配置驅動的統一設備身份識別系統
- **重要性**: ⭐⭐⭐⭐⭐ 核心系統腳本
- **功能**:
  - 基於 MAC 地址自動識別 AGV/AGVC 設備身份
  - 載入對應的硬體配置 (hardware_mapping.yaml)
  - 設定關鍵環境變數 (AGV_ID, AGV_TYPE, ROS_NAMESPACE 等)
  - 創建設備身份檔案
- **使用**: 由 `startup.agv.bash` 和 `startup.agvc.bash` 自動呼叫
- **注意**: 此腳本是容器啟動流程的關鍵部分，請勿移動或刪除

### test-data/ - 測試資料生成工具
用於生成 AGV 狀態測試資料的腳本，支援開發和測試環境。

## 📝 使用說明

- 核心系統腳本會在容器啟動時自動執行
- 測試資料生成腳本必須在 AGV 容器內執行。詳見 `test-data/README.md`