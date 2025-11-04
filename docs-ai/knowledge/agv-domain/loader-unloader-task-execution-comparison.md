# Loader vs Unloader AGV 任務執行邏輯對比

## 🎯 適用場景
- 理解 Loader 和 Unloader AGV 的任務執行差異
- 設計和調試 TAFL 流程時選擇正確的 batch_size
- 優化任務調度策略
- 排查任務執行問題
- 為新車型設計提供參考

## 📊 核心對比表格

| 項目 | Loader AGV | Unloader AGV |
|------|-----------|--------------|
| **製程階段** | 前段製程（清潔、浸潤、預乾燥） | 後段製程（預烘機、烘箱、出口箱） |
| **工作模式** | 精密操作（機械臂限制） | 批量處理（效率優先） |
| **機械臂能力** | 一次 **1 格** | 一次 **2 格** |
| **設計原則** | **batch_size = Station port 數** | **batch_size = Station port 數** |
| **標準設備** | batch_size=2 (2 port/Station) | batch_size=4 (4 port/Station) |
| **特殊設備** | batch_size=1 (1 port/Station, Soaker) | - |
| **任務-動作對應** | **標準設備**: 1 任務 = 2 動作<br>**特殊設備**: 1 任務 = 1 動作 | **1 任務 = 2 動作** |
| **執行邏輯** | **標準**: 1任務 × (1格×2次) = 2格<br>**特殊**: 1任務 × (1格×1次) = 1格 | 1任務 × (2格×2次) = 4格 |
| **車載料架** | 4層垂直排列（Port 1-4） | 4格水平排列（上2格+下2格） |
| **Hokuyo位置** | 前方配置 | 前方配置 |
| **典型設備** | **標準**: 入口箱、清潔機、預乾機<br>**特殊**: Soaker（6台泡藥機） | 預烘機、烘箱、出口箱 |

## 🏗️ Loader AGV 詳細說明

### 核心設計原則
**batch_size = Station 的 port 數量**

Loader AGV 根據不同設備的 port 配置，使用不同的 batch_size：
- **標準設備**（入口箱、清潔機、預乾機）: 每個 Station 2個 port → batch_size=2
- **特殊設備**（Soaker 浸潤機）: 每個 Station 1個 port → batch_size=1

### 任務執行邏輯

#### TAFL 任務配置範例 - 標準設備
```yaml
# loader_take_boxin_transfer.yaml（入口箱）
stations:
  - station: 1
    work_id: 2010101
    ports: [2011, 2012]  # 2個 port
    batch_size: 2  # ✅ 標準設備: 2格操作（一個 Station 的2個 port）
```

#### TAFL 任務配置範例 - 特殊設備
```yaml
# loader_put_soaker.yaml（浸潤機）
stations:
  - station: 1  # 泡藥機A
    work_id: 2040102
    ports: [2041]  # 只有1個 port
    batch_size: 1  # ✅ 特殊設備: 1格操作（一個 Station 只有1個 port）
  - station: 2  # 泡藥機B
    work_id: 2040202
    ports: [2042]
    batch_size: 1
  # ... 共6個獨立的泡藥機 Station
```

#### 執行流程 - 標準設備（batch_size=2）
```
TAFL WCS 創建任務
├─ batch_size: 2
├─ ports: [2011, 2012]  # 2個 port，一次性處理
└─ work_id: 2010101

        ↓

AGV 接收任務並執行
├─ 第1次動作：從 port 2011 取1格
│   ├─ 機械臂動作：TAKE 1格 Carrier
│   └─ 放到 AGV Port 1 或 Port 3
│
└─ 第2次動作：從 port 2012 取1格
    ├─ 機械臂動作：TAKE 1格 Carrier
    └─ 放到 AGV Port 另一層

        ↓

任務完成
└─ 總共處理：2格 Carrier
```

#### 執行流程 - 特殊設備（batch_size=1）
```
TAFL WCS 創建任務
├─ batch_size: 1
├─ ports: [2041]  # 只有1個 port
└─ work_id: 2040102

        ↓

AGV 接收任務並執行
└─ 第1次動作：從 port 2041 取1格
    ├─ 機械臂動作：TAKE 1格 Carrier
    └─ 放到 AGV Port 1 或 Port 3

        ↓

任務完成
└─ 總共處理：1格 Carrier
```

#### 機械臂能力
- **一次取放能力**: 1格（精密操作）
- **車載料架**: 4層垂直排列
- **端口管理**: Port 1、3 用於前段流程
- **操作精度**: 高精度，確保每格品質

#### 設計考量
1. **精密操作需求**: 前段製程（清潔、浸潤）需要精確控制，機械臂一次只能處理1格
2. **靈活的 batch_size 設計**: 根據 Station 的 port 數量自動調整 batch_size
   - 標準設備: 2個 port → batch_size=2
   - 特殊設備: 1個 port → batch_size=1（Soaker）
3. **多工位支援**: 支援多次浸潤、清潔等重複操作
4. **平衡設計**: 平衡精密操作（1格/次）與批量處理（一個 Station/任務）的需求

#### 典型應用場景 - 標準設備
```
製程流程範例（一次處理入口箱 Station 1 的2格）：
1. TAKE_TRANSFER (batch_size=2)
   - 第1次動作: 從 port 2011 取1格
   - 第2次動作: 從 port 2012 取1格
   - 結果: AGV 車上有2格

2. PUT_CLEANER (batch_size=2)
   - 第1次動作: 從 AGV 取1格放入清潔機
   - 第2次動作: 從 AGV 取1格放入清潔機
   - 結果: 清潔機有2格
```

#### 典型應用場景 - 特殊設備
```
製程流程範例（處理6台泡藥機）：
1. PUT_SOAKER Station 1 (batch_size=1)
   - 第1次動作: 從 AGV 取1格放入泡藥機A (port 2041)
   - 結果: 泡藥機A有1格

2. PUT_SOAKER Station 2 (batch_size=1)
   - 第1次動作: 從 AGV 取1格放入泡藥機B (port 2042)
   - 結果: 泡藥機B有1格

... 依此類推，共6個獨立的 Station，每個處理1格
```

## 🏭 Unloader AGV 詳細說明

### 任務執行邏輯

#### TAFL 任務配置範例
```yaml
# unloader_take_pre_dryer.yaml
stations:
  - station: 1
    work_id: 2050101
    ports: [2051, 2052, 2055, 2056]  # 4個 port
    batch_size: 4  # ✅ Unloader: 4格批量操作
```

#### 執行流程
```
TAFL WCS 創建任務
├─ batch_size: 4
├─ ports: [2051, 2052, 2055, 2056]  # 4個 port
└─ work_id: 2050101

        ↓

AGV 接收任務並執行
├─ 第1次動作：從 port 2051, 2052 取2格
│   ├─ 機械臂動作：TAKE 2格 Carrier
│   └─ 放到 AGV 上排（或下排）
│
└─ 第2次動作：從 port 2055, 2056 取2格
    ├─ 機械臂動作：TAKE 2格 Carrier
    └─ 放到 AGV 下排（或上排）

        ↓

任務完成
└─ 總共處理：4格 Carrier
```

#### 機械臂能力
- **一次取放能力**: 2格
- **車載料架**: 4格水平排列（上2格+下2格）
- **批量處理**: 一次任務處理4格
- **效率優先**: 減少機械臂動作次數

#### 設計考量
1. **批量處理需求**: 後段製程（烘箱、預烘機）適合批量操作
2. **效率提升**: 減少 TAFL WCS 任務調度頻率
3. **充分利用料架**: 4格料架一次性裝滿
4. **平衡設計**: 平衡機械臂限制（2格）與任務效率（4格）

#### 典型應用場景
```
製程流程範例：
1. TAKE_PRE_DRYER (batch_size=4)
   - 第1次動作: 取2格（port 2051, 2052）
   - 第2次動作: 取2格（port 2055, 2056）
   - 總共: 4格

2. PUT_OVEN (batch_size=4)
   - 第1次動作: 放2格到烘箱上排
   - 第2次動作: 放2格到烘箱上排
   - 總共: 4格

3. TAKE_OVEN (batch_size=4)
   - 第1次動作: 從烘箱下排取2格
   - 第2次動作: 從烘箱下排取2格
   - 總共: 4格

4. PUT_BOXOUT_TRANSFER (batch_size=4)
   - 第1次動作: 放2格到出口箱
   - 第2次動作: 放2格到出口箱
   - 總共: 4格
```

## 🔍 關鍵差異分析

### 1. 任務粒度設計

**Loader AGV - 標準設備 (batch_size=2)**:
- ✅ 優點: 一次處理一個 Station（2個 port），符合設備配置
- ✅ 優點: 機械臂精密操作（一次1格），確保品質
- ✅ 優點: 平衡精密操作與批量處理
- ⚠️ 特點: 需要2次動作完成一個 Station

**Loader AGV - 特殊設備 (batch_size=1)**:
- ✅ 優點: 符合單 port 設備配置（Soaker）
- ✅ 優點: 6台獨立泡藥機，靈活調度
- ✅ 優點: 每個任務只需1次動作，簡化邏輯
- ⚠️ 特點: 需要6個任務完成6台泡藥機

**Unloader AGV (batch_size=4)**:
- ✅ 優點: 減少任務調度頻率
- ✅ 優點: 充分利用車載料架容量（4格）
- ✅ 優點: 提升整體效率
- ⚠️ 特點: 需要2次動作完成一個 Station（跨4個 port）

### 2. 機械臂動作次數

**Loader AGV - 標準設備**:
```
處理4格（2個 Station）= 2 個任務 = 4 次機械臂動作
（每個任務 batch_size=2，執行2次動作）
```

**Loader AGV - 特殊設備（Soaker）**:
```
處理6格（6台泡藥機）= 6 個任務 = 6 次機械臂動作
（每個任務 batch_size=1，執行1次動作）
```

**Unloader AGV**:
```
處理4格（1個 Station）= 1 個任務 = 2 次機械臂動作
（每個任務 batch_size=4，執行2次動作）
```

### 3. TAFL WCS 調度策略

**Loader AGV - 標準設備**:
- 等待一個 Station 的2格 Carrier 就緒後創建任務
- 一次性處理一個 Station（2個 port）
- 適合前段精密製程，機械臂一次1格的限制

**Loader AGV - 特殊設備（Soaker）**:
- 等待每個泡藥機的1格 Carrier 就緒後創建任務
- 一次性處理一台泡藥機（1個 port）
- 6台獨立泡藥機，需要6個獨立任務

**Unloader AGV**:
- 等待一個 Station 的4格 Carrier 就緒後才創建任務
- 一次性處理一個 Station（4個 port）
- 適合後段批量處理，機械臂一次2格的能力

**統一原則**:
- 所有 AGV 都是：一次任務處理一個 Station 的所有 port
- batch_size = Station 的 port 數量

## 🎨 設計最佳實踐

### 選擇 batch_size 的考量因素

1. **機械臂能力（關鍵因素）**:
   - 機械臂一次1格 → batch_size=2（需2次動作處理一個 Station）
   - 機械臂一次2格 → batch_size=4（需2次動作處理一個 Station）

2. **Station 配置**:
   - 一個 Station 有2個 port → batch_size=2 (Loader)
   - 一個 Station 有4個 port → batch_size=4 (Unloader)

3. **設計原則**:
   - batch_size = Station 的 port 數量
   - 執行動作 = batch_size ÷ 機械臂能力 = 2次（通用設計）

4. **料架配置**:
   - 垂直4層 → 適合 batch_size=2，分批管理
   - 水平4格 → 適合 batch_size=4，一次性裝滿

### TAFL 流程設計建議

#### Loader 流程模式 - 標準設備
```yaml
# Station 批量處理模式（2格）
stations:
  - station: 1
    batch_size: 2  # 一個 Station 的2個 port
    ports: [2011, 2012]  # 2個 port，一次性處理
```

#### Loader 流程模式 - 特殊設備（Soaker）
```yaml
# 單 port 處理模式（1格）
stations:
  - station: 1  # 泡藥機A
    batch_size: 1  # 一個 Station 只有1個 port
    ports: [2041]  # 只有1個 port
  - station: 2  # 泡藥機B
    batch_size: 1
    ports: [2042]
  # ... 共6台獨立泡藥機
```

#### Unloader 流程模式
```yaml
# Station 批量處理模式（4格）
stations:
  - station: 1
    batch_size: 4  # 一個 Station 的4個 port
    ports: [2051, 2052, 2055, 2056]  # 4個 port，一次性處理
```

**通用原則**:
- **batch_size = Station 的 port 數量**（最重要的設計原則）
- 機械臂會自動執行所需的動作次數（batch_size ÷ 機械臂能力）
- 每個任務處理一個 Station 的所有 port

## 🚨 常見問題

### Q1: 為什麼說「batch_size = Station port 數」是核心原則？

**答**: 這是系統的統一設計原則：
- **目的**: 一次任務處理一個 Station 的所有 port
- **Loader 標準設備**: 入口箱 Station 1 有2個 port [2011, 2012] → batch_size=2
- **Loader 特殊設備**: 泡藥機A Station 1 有1個 port [2041] → batch_size=1
- **Unloader**: 預烘機 Station 1 有4個 port [2051, 2052, 2055, 2056] → batch_size=4
- **優勢**: 靈活適應不同設備配置，簡化調度邏輯

### Q2: 為什麼 Soaker（泡藥機）是特殊設備？

**答**: Soaker 的物理配置與其他設備不同：
- **物理結構**: 6台獨立的泡藥機（A-F），每台只有1個 port
- **vs 標準設備**: 其他設備的 Station 有2個 port（上下2格）
- **batch_size**: 1（每個 Station 只有1個 port）
- **任務執行**: 一次任務 = 1次動作（不是2次）
- **調度**: 需要6個獨立任務完成6台泡藥機

### Q3: 標準設備為什麼是 batch_size=2？

**答**: 根據 Station 的 port 配置：
- 一個 Station 有2個 port（例如入口箱的上2格：port 2011, 2012）
- batch_size=2 可以一次性處理一個 Station
- 機械臂一次1格，需要執行2次動作
- 這樣設計平衡了精密操作與批量處理

### Q4: 任務失敗後如何處理？

**通用策略**（適用於兩者）:
- 記錄失敗在第幾次動作（第1次或第2次）
- 部分完成的可以繼續，完全失敗的重新規劃
- 兩者都需要處理2次動作的失敗情況

**Loader AGV** (batch_size=2):
- 影響範圍: 2格
- 第1次動作失敗: 整個任務重試
- 第2次動作失敗: 第1格已完成，只需處理第2格

**Unloader AGV** (batch_size=4):
- 影響範圍: 4格
- 第1次動作失敗: 整個任務重試（0格完成）
- 第2次動作失敗: 前2格已完成，只需處理後2格

### Q5: 如何在代碼中實現批量處理？

**參考文件**:
- Loader: `/app/agv_ws/src/loader_agv/loader_agv/robot_context.py`
- Unloader: `/app/agv_ws/src/unloader_agv/unloader_agv/robot_context.py`
- 參數計算: `LoaderRobotParameter` vs `UnloaderRobotParameter`

**關鍵實現**:
```python
# Unloader AGV 需要管理批量動作
class RobotContext:
    def update_port_parameters(self):
        self.robot_parameter.take_quantity = self.get_take_quantity  # 取料數量
        self.robot_parameter.calculate_parameter()  # 計算2次動作的參數
```

## 🔗 相關文檔

- **Loader AGV**: `/app/agv_ws/src/loader_agv/CLAUDE.md`
- **Unloader AGV**: `/app/agv_ws/src/unloader_agv/CLAUDE.md`
- **TAFL 流程**: `/app/config/tafl/flows/`
- **測試驗證**: `/app/tafl_wcs_ws/src/tafl_wcs/test/`
  - `test_loader_flows.py` - Loader 流程測試
  - `test_unloader_flows.py` - Unloader 流程測試
- **車型特性**: `docs-ai/knowledge/agv-domain/vehicle-types.md`
- **WCS 設計**: `docs-ai/knowledge/agv-domain/wcs-system-design.md`

## 📅 文檔歷史

- **2025-10-21**: 初始創建，記錄 Loader vs Unloader 任務執行邏輯對比
- **目的**: 作為 AGV 領域知識庫文檔，供開發和維護人員參考
