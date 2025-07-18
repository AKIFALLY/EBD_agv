# [工作空間名稱] ([workspace_name])

## 📋 基本資訊

**啟動狀態**: [✅/⚠️/❌] [狀態描述] ([詳細說明])
**運行環境**: [🚗/🖥️/🚗🖥️] [環境描述]
**主要功能**: [簡潔功能描述]
**依賴狀態**: [依賴摘要，特別標註虛擬環境套件]
**[特殊狀態]**: [如手動啟動方法、重要變更等]

## 📋 專案概述

[第一段：核心功能和定位]

[第二段：技術特點和實作方式，標註虛擬環境套件使用]

[第三段：在系統中的角色和重要性]

**[特殊說明]**: [重要變更、棄用功能、開發狀態等]

## 🔗 依賴關係

### 虛擬環境套件依賴 (如有)
- **套件名稱**: 功能說明

### 系統套件依賴
- **套件名稱**: 功能說明

### 依賴的工作空間
- **工作空間名稱**: 使用的功能 (✅/❌ 啟動狀態標記)

### 被依賴的工作空間
- **工作空間名稱**: 提供的功能

### 外部依賴
- **依賴名稱**: 連線資訊和用途

## 🏗️ 專案結構

```
workspace_name/
├── src/
│   └── package_name/              # 主要套件
│       ├── package_name/
│       │   ├── main_module.py     # 主要模組 (標註虛擬環境套件使用)
│       │   ├── ~~deprecated.py~~ # ❌ 已棄用：棄用模組
│       │   └── __init__.py
│       ├── package.xml            # 套件配置
│       └── setup.py               # Python 套件設定
├── test/                          # 測試檔案
├── config/                        # 配置檔案 (如有)
└── README.md                      # 本檔案
```

## ⚙️ 主要功能

### 1. 核心功能模組
**功能描述**:
- **子功能 1**: 詳細說明
- **子功能 2**: 詳細說明 (使用虛擬環境 package_name)
- ~~**已棄用功能**: 棄用說明~~ ❌ **已棄用**

### 2. 其他功能模組
**功能描述**:
- **子功能**: 詳細說明

## 🔧 核心 API

### 主要類別使用
```python
from package_name.module import MainClass

# 初始化
instance = MainClass(param1, param2)

# 基本操作
result = instance.method(args)

# 虛擬環境套件使用 (如適用)
# 注意：此功能使用虛擬環境套件 package_name
```

### ~~已棄用 API~~ ❌ **已棄用**
~~**OldClass** 已棄用，改用 NewClass~~

**新架構**: 功能已移至 **other_workspace** 專案

## 🚀 使用方法

### 1. 虛擬環境套件檢查 (如適用)
```bash
# 檢查虛擬環境套件
/opt/pyvenv_env/bin/pip3 list | grep package_name

# 檢查套件版本
python3 -c "
import package_name
print(f'✅ package_name 可用')
"
```

### 2. 依賴檢查
```bash
# 檢查依賴工作空間
python3 -c "
from dependency_workspace.module import Class
print('✅ 依賴檢查通過')
"
```

### 3. 建置工作空間
```bash
cd /app/workspace_name
colcon build
source install/setup.bash
```

### 4. 啟動服務
```bash
# 方法 1: 使用便利函數 (如有)
start_service_name

# 方法 2: 使用 launch 檔案
ros2 launch package_name launch.py

# 方法 3: 直接啟動節點
ros2 run package_name node_name
```

## 🧪 測試方法

### 1. 建置和測試
```bash
# 建置工作空間
cd /app/workspace_name
colcon build

# 執行測試
colcon test
colcon test-result --verbose
```

### 2. 虛擬環境套件測試 (如適用)
```bash
# 測試虛擬環境套件功能
python3 -c "
import package_name
# 測試程式碼
print('✅ 虛擬環境套件測試通過')
"
```

### 3. 功能測試
```bash
# 測試主要功能
ros2 run package_name test_node

# 檢查服務狀態
ros2 service list | grep service_name

# 檢查主題
ros2 topic list | grep topic_name
```

## 🔧 故障排除

### 常見問題

#### 1. 虛擬環境套件問題 (如適用)
**症狀**: `ModuleNotFoundError: No module named 'package_name'`
**解決方法**:
```bash
# 檢查虛擬環境套件
/opt/pyvenv_env/bin/pip3 list | grep package_name

# 重新安裝套件
/opt/pyvenv_env/bin/pip3 install package_name
```

#### 2. 依賴工作空間問題
**症狀**: 依賴模組載入失敗
**解決方法**:
```bash
# 檢查依賴工作空間載入
python3 -c "from dependency.module import Class"

# 手動載入依賴工作空間
cd /app/dependency_ws && source install/setup.bash
```

#### 3. 服務啟動失敗
**症狀**: 節點或服務無法啟動
**解決方法**:
```bash
# 檢查建置狀態
ls -la /app/workspace_name/install/

# 重新建置
cd /app/workspace_name
rm -rf build install log
colcon build
```

### 除錯工具
```bash
# 檢查節點狀態
ros2 node list | grep workspace_name

# 監控效能
top | grep process_name

# 檢查日誌
tail -f /tmp/workspace_name.log
```

## 🔗 相關文檔

- **dependency_ws**: 依賴工作空間說明
- **related_ws**: 相關工作空間說明
- **外部文檔**: [連結說明](URL)

## 📋 ToDo 清單

### 🔴 高優先級 (緊急)
- [ ] **重要任務** (時間估計)
  - 具體子任務
  - 實作細節

### 🟡 中優先級 (重要)
- [ ] **一般任務** (時間估計)
  - 具體子任務

### 🟢 低優先級 (改善)
- [ ] **改善任務** (時間估計)
  - 具體子任務

### 🔧 技術債務
- [ ] **重構任務** (時間估計)
  - 重構範圍

### 📊 完成度追蹤
- **核心功能**: XX% [✅/🔄/⏳]
- **虛擬環境整合**: XX% [✅/🔄/⏳] (如適用)
- **測試覆蓋**: XX% [✅/🔄/⏳]

### 🎯 里程碑
1. **vX.X.X** (時間) - 里程碑描述
2. **vX.X.X** (時間) - 里程碑描述

---

**最後更新**: YYYY-MM-DD  
**維護者**: 維護者資訊  
**版本**: vX.X.X
