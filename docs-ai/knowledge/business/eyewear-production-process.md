# 眼鏡生產業務流程

## 🎯 適用場景
- 理解 RosAGV 在眼鏡（防風鏡、面罩、護目鏡）生產工廠中的實際應用
- 為業務邏輯開發和系統整合提供背景知識
- 理解射出機作業到AGV搬運的完整流程

## 🏭 系統實際部署狀況 (實驗階段)

### ⚠️ 系統管理前提條件
**在進行系統管理和監控之前，必須將 RosAGV 目錄加入 PATH 環境變數**

在 `~/.bashrc` 中添加以下設定：
```bash
# RosAGV 工具路徑配置
export PATH="/home/ct/RosAGV:$PATH"
```

設定完成後，重新載入環境：
```bash
source ~/.bashrc
```

驗證配置是否正確：
```bash
which r                    # 應該顯示 /home/ct/RosAGV/r
r agvc-check              # 執行 AGVC 系統健康檢查
r containers-status       # 檢查容器運行狀態
```

### 當前部署規模
- **KUKA AGV**: 2台 (負責房間外長距離搬運和 Rack 轉向)
- **CT AGV 系統**: 3台 (Cargo、Loader、Unloader 各1台)
- **活躍房間**: 僅房間2 (單房間實驗配置)
- **工廠狀態**: 建置中，實際生產量待確認

### 實驗階段特點
- **概念驗證**: 完整業務流程的技術可行性驗證
- **單車配置**: 每種 AGV 類型僅1台，無備援機制
- **人工介入**: 電池管理、NG 處理、車載料架調整均為人工模式
- **擴展準備**: 系統架構支援未來多房間和多車擴展

## 📋 工廠生產環境概述

### 生產設備配置
```
眼鏡生產工廠配置
├── 4台射出機 (machine表)
│   ├── 每台射出機：最多同時射出4種產品
│   ├── 每台射出機：配置2個停車格 (node_id對應KUKA地圖)
│   └── 每台射出機：2個作業員共用1台平板
├── 8個作業員 (每台射出機2人)
├── 4台操作平板 (client表，2人共用1台)
└── KUKA AGV車隊 (robot 400i，2輪差動車)
```

### 產品類型
- **S尺寸產品**：一般眼鏡
- **L尺寸產品**：面罩類產品

## 🏭 核心業務流程

### 射出機作業區配置
```
單個作業員 (OP) 工作區域配置：
├── 產品 Rack A (放射出的產品A類型)
├── 產品 Rack B (放射出的產品B類型)  
├── 停車格 (KUKA AGV 送空 Rack 的專用位置)
└── 作業員在這 3 個位置間管理產品裝載

重要概念：
- 每個作業員身邊通常有 3 個 Rack 空間
- 2個用於放置不同產品的 Rack
- 1個停車格供 KUKA AGV 送達空 Rack
```

### 射出機作業流程
```
射出機生產流程
├── 4台射出機並行生產
├── 每台射出機：
│   ├── 作業員A：負責產品1、產品2
│   ├── 作業員B：負責產品3、產品4
│   ├── 眼鏡射出完成
│   ├── 作業員將眼鏡掛載到carrier (眼鏡框架)
│   └── 將carrier安裝到對應產品的rack上
└── 根據產品尺寸選擇rack容量 (S/L規格)
```

### OPUI 完整操作流程
```
作業員完整操作流程
1. 【手動加入 Rack】當需要新空rack時：
   ├── 作業員從倉儲區手動搬運空rack到射出機旁
   ├── 在OPUI平板上選擇「Rack管理」→「加入Rack」
   ├── 輸入rack編號（如 "101"）進行識別
   ├── 系統驗證並將rack納入管理（分配location_id）
   └── 顯示「Rack已成功加入系統」確認訊息

2. 【產品裝載】射出機完成生產 → 作業員將眼鏡掛到carrier → 安裝到rack

3. 【OPUI資訊設定】在平板上設定：
   ├── 產品類型和數量 (告知AGVC這個rack裝的是什麼)
   ├── 目標房間 (告知WCS最終要送到哪個房間製程)
   └── 目的：讓系統知道rack內容，方便WCS調度

4. 【派滿車】當rack裝滿後：
   ├── 按「派車」按鈕產生任務請求
   ├── WCS系統根據房間製程需求決定調度
   └── 不會立刻派送，由WCS決定何時派哪台車搬運

註：已取消自動叫空車功能，改為人工搬運配合OPUI手動加入系統
```

## 📊 資料庫結構對應

### 設備和人員管理
```sql
-- machine表：射出機設備資訊
machine (4筆記錄)
├── 每筆記錄代表1台射出機
├── 包含2個停車格的node_id
├── node_id對應KUKA地圖上的定位點
└── 儲存停車格編號資訊

-- client表：操作平板設備
client (4筆記錄)  
├── 每筆記錄代表1台平板（2個作業員共用）
├── device_id欄位：android_id
├── op欄位：JSON格式，記錄左右兩邊作業員的操作狀態
└── 對應license表進行白名單驗證

-- license表：設備白名單
license
├── 儲存合法的device_id
├── 用於OPUI設備認證
└── 簡便的認證機制
```

### 生產物料管理
```sql
-- carrier表：眼鏡承載框架
carrier
├── 承載不定數量的眼鏡產品
├── 由作業員手動將眼鏡掛載到carrier上
└── 再將carrier安裝到rack上

-- rack表：載具架台
rack
├── 分為A面、B面兩個承載面
├── S產品規格：每面可承載16個carrier (4row × 4col)
├── L產品規格：每面可承載8個carrier (2row × 4col)
└── 由KUKA AGV負責搬運
```

## 🤖 AGV系統整合

### KUKA AGV車隊
- **車型**：KUKA robot 400i (2輪差動車)
- **主要任務**：空rack配送到射出機停車格
- **導航系統**：KUKA自有地圖系統，node_id對應地圖定位點
- **控制整合**：透過kuka_fleet_ws模組與RosAGV系統通訊

### AGVC系統職責
- **接收OPUI請求**：處理作業員的叫車需求
- **車隊調度**：選擇合適的KUKA AGV執行任務
- **rack管理**：追蹤空rack庫存和位置狀態
- **任務協調**：確保每個停車格的rack供應

## 🔧 操作介面系統

### 設備授權管理機制
系統採用嚴格的設備授權管理，確保生產安全性和權限控制：

**授權設備類型**：
1. **操作員工作站** (`op_station`)
   - 用於日常生產操作和任務管理
   - 權限：建立任務、呼叫AGV、查看生產狀態
   - 通常部署於控制室或辦公區
   - 介面：標準OPUI操作介面

2. **HMI人機介面終端** (`hmi_terminal`)
   - 用於現場特定區域的Rack管理
   - 權限：只能管理授權的位置（如人工收料區）
   - 通常部署於生產現場（射出機旁、收料區）
   - 介面：大按鈕觸控介面，適合現場操作

**授權控制特點**：
- 每個設備需要唯一的16位device_id識別
- 未授權設備無法存取系統功能
- 管理員可隨時啟用/停用設備授權
- HMI終端嚴格限制在指定區域內操作
- 所有操作都會記錄設備ID以供稽核

### OPUI操作平板
```
平板配置
├── 硬體：Android平板 (4台)
│   └── 需要授權的device_id才能使用
├── 應用：全螢幕webview開啟opui網站
├── 認證：URL參數帶入device_id
├── 介面分割：左右兩邊對應2個作業員
└── 功能：叫空車、狀態顯示
```

### 認證機制
```
設備認證流程
1. 平板啟動 → 取得android_id
2. OPUI網站 → URL參數帶入device_id
3. 系統檢查 → license表白名單驗證
4. 驗證通過 → 允許使用功能
5. 驗證失敗 → 拒絕存取
```

## 📋 產品承載規格

### Rack承載能力
| 產品類型 | 每面carrier數量 | 排列方式 | 總承載量 |
|---------|----------------|----------|----------|
| S尺寸眼鏡 | 16個carrier | 4行×4列 | 32個carrier (A+B面) |
| L尺寸面罩 | 8個carrier | 2行×4列 | 16個carrier (A+B面) |

### Carrier裝載
- **眼鏡掛載**：作業員手動將射出的眼鏡掛載到carrier框架上
- **數量彈性**：每個carrier可掛載不定數量的眼鏡
- **尺寸適配**：根據眼鏡或面罩尺寸選擇合適的carrier類型

## 🔧 OPUI 功能詳解

### 完整功能列表
基於程式碼分析，OPUI 提供以下操作功能：

#### 核心操作功能
1. **叫車功能** (Call Empty Car)
   - 按鈕：`data-call-empty="left/right"`
   - 用途：請求 KUKA AGV 送空 rack 到停車格

2. **派車功能** (Dispatch Car)
   - 按鈕：`data-dispatch-full="left/right"`  
   - 用途：當 rack 裝滿後，請求 WCS 派車搬運

3. **確認送達功能**
   - 用途：空 rack 送達後，作業員移走並通知系統停車格已空出
   - 重要性：只有確認後才能再次叫空車

4. **取消功能**
   - 用途：取消已發出的叫車或派車請求

#### 設定功能
5. **產品選擇** (`product-btn`)
   - 用途：設定 rack 裝載的產品類型 (S/L尺寸)
   - 目的：告知 AGVC 系統 rack 內容物

6. **數量設定** (`num-btn`)
   - 用途：設定 rack 中的產品數量
   - 目的：讓系統知道 rack 裝載情況

7. **房間選擇** (`room-btn`)
   - 用途：設定產品最終要送到的房間
   - 重要：由 WCS 根據製程需求決定實際派送時機

8. **料架選擇** (`rack-btn`)
   - 用途：選擇特定 rack 和面向 (A面/B面)

9. **機台設定** (`machine-btn`)
   - 用途：選擇對應的射出機台

10. **恢復原廠設定**
    - 用途：重置所有設定到預設值

### 業務邏輯修正
- **產品和數量選擇**：不是控制射出機生產，而是告知系統這個 rack 的內容物
- **房間選擇**：設定最終目標，但實際派送由 WCS 根據製程順序決定
- **確認送達**：關鍵功能，確保停車格管理的正確性

## 🔄 目前已知流程範圍

### 已確認理解的環節
1. **射出機生產** → 眼鏡產品射出
2. **人工作業** → 眼鏡掛載到carrier，carrier安裝到rack
3. **OPUI設定** → 設定rack內容物和目標房間
4. **叫車請求** → KUKA AGV 將空rack送到停車格
5. **確認取走** → 作業員移走空rack並通知系統
6. **派車請求** → 滿載rack請求WCS調度搬運

## 🏭 實際製程配置 (基於資料庫分析)

### 製程設定 (ProcessSettings表)
根據 `04_process_settings.py`，系統中有 **2種製程**：

1. **製程1 (ID=1)**: `{"soaking_times": 1, "description": "泡藥泡一次"}`
2. **製程2 (ID=2)**: `{"soaking_times": 2, "description": "泡藥泡兩次"}`

### 房間配置 (Room表)
根據 `07_rooms.py`，總共 **5個房間**，當前配置：

```
Room1 (ID=1) - 製程1 (泡藥1次) - 啟用 ✅
Room2 (ID=2) - 製程1 (泡藥1次) - 啟用 ✅  
Room3 (ID=3) - 製程1 (泡藥1次) - 停用 ❌
Room4 (ID=4) - 製程1 (泡藥1次) - 停用 ❌
Room5 (ID=5) - 製程1 (泡藥1次) - 停用 ❌
```

**注意**: 
- S/L尺寸 ≠ 製程1/製程2，產品規格決定製程需求
- 初始化資料為測試用範例，實際配置可調整
- 目前只有 Room1, Room2 啟用，都執行製程1

### 產品配置 (Product表)
根據 `09_products.py`，測試產品範例：
- 每個產品有自己的 `process_settings_id`
- S/L尺寸產品都可能需要製程1或製程2
- 由產品規格決定製程需求，非尺寸決定

## 🏭 房間內製程流程架構

### 🚛 KUKA AGV → 房間門口配送
```
KUKA AGV 配送流程
├── 將滿載 Rack 搬運到房間門口
├── Rack 的 A 面朝向 Cargo AGV 
├── 完成配送後 KUKA AGV 變回閒置狀態
└── 接下來由房間內的 AGV 系統接手
```

### 🔧 設備架構 (基於資料表分析)
```
房間設備配置
├── 入口傳送箱 (Eqp表)
│   ├── 4個空位 (EqpPort表) - 上2格同門，下2格同門
│   ├── 內外側雙門互鎖系統
│   └── 8bit通訊協調 (EqpSignal表)
├── 製程設備 (泡藥水設備)
│   ├── 製程1：泡藥1次
│   └── 製程2：泡藥2次 (需多次設備間轉移)
└── 出口傳送箱 (與入口傳送箱相同構造)
```

### 🤖 三種 AGV 分工 (完整理解)

#### Cargo AGV (貨物搬運車)
- **工作區域**: 房間門口
- **主要職責**: Rack ↔ 傳送箱 轉移作業 + 製程匹配驗證
- **關鍵設備**: SensorPart 相機 (3D視覺定位 + OCR識別)

##### 完整操作流程
```
階段1：3D視覺定位 (2D Mark定位)
├── SensorPart 相機對 Rack 上的 2D Mark 進行 3D 定位
├── 掃描識別 2D Mark 位置
├── 定位資訊傳送到 Robot 進行 base 更新
└── 建立 Carrier 抓取路徑規劃

階段2：逐一 Carrier 處理循環（房間入口卸載作業）
For 每個 Carrier:
1. 【移動定位】機械臂移動到 Carrier 前方
2. 【從Rack取出Carrier】
3. 【OCR 識別】SensorPart 相機執行 OCR 掃描
   ├── 讀取產品編號/條碼
   ├── 識別產品類型
   └── 取得產品製程需求資訊
4. 【製程匹配檢查】
   ├── 查詢產品的 process_settings_id
   ├── 比對當前房間的製程能力
   └── 判斷製程適配性 (泡藥1次 vs 2次)
5. 【條件分支處理】
   ✅ 製程匹配：
   ├── 8bit 通訊開啟傳送箱門
   ├── 將 Carrier 放入入口傳送箱
   └── 更新系統狀態
   ❌ OCR失敗或製程不匹配：
   ├── 立即停止處理
   ├── 發出 PLC Alarm 到 HMI
   ├── 等待人員現場處理
   └── 人員處理流程：
       • OCR失敗：處理問題後 Alarm Reset 重試
       • 錯誤產品：移除產品 → AGVCUI清除資料 → Alarm Reset

階段3：空Rack處理（由KUKA AGV執行）
├── A面所有Carrier處理完成
├── B面所有Carrier處理完成（轉向後）
├── 判斷房間出口是否有Rack停靠
├── 無Rack → 將空Rack搬到房間出口位置
└── 有Rack → 將空Rack搬到系統空料車停車區
```

##### 智能製程防錯
- **製程適配性檢查**: Room1/Room2 只接受泡藥1次產品，泡藥2次產品標記NG
- **產品識別驗證**: OCR 確保產品正確性，防止混料誤送
- **即時異常處理**: OCR NG 立即發出 Alarm，人員現場處理（非事後集中處理）
- **系統資料同步**: AGVCUI 提供資料清除功能，確保錯誤產品移除後資料一致性
- **Alarm Reset 機制**: 人員處理完成後可 Reset 重試，系統自動恢復

#### Loader AGV (裝載車)  
- **工作區域**: 房間內部
- **主要職責**: 入口傳送箱 → 製程設備 送料作業
- **詳細業務流程**: 7大工作流程，基於 work_id 智能路由系統

##### Work ID 路由系統 (基於實際程式碼分析)
Loader AGV 採用智能 work_id 路由系統，格式：`room_id + equipment_type + station_number + action_type`

**設備類型編碼**：
- `01` = TRANSFER (傳送箱)
- `03` = CLEANER (清潔機) 
- `04` = SOAKER (浸潤機)
- `05` = PRE_DRYER (預乾燥機)

**動作類型編碼**：
- `01` = TAKE (取料)
- `02` = PUT (放料)

##### 7大業務流程 (基於 idle_state.py 實作)

**取料流程 (TAKE Operations)**：
1. **TAKE_TRANSFER**: `room_id + "01" + "01" + "01"` - 從入口傳送箱取得 Cargo AGV 送來的載具
2. **TAKE_CLEANER**: `room_id + "03" + "01-02" + "01"` - 從清潔機取回完成清潔的載具 (2工位)
3. **TAKE_SOAKER**: `room_id + "04" + "01-06" + "01"` - 從浸潤機取回完成泡藥的載具 (6工位)
4. **TAKE_PRE_DRYER**: `room_id + "05" + "01-08" + "01"` - 從預乾燥機取回完成預乾燥的載具 (8工位)

**放料流程 (PUT Operations)**：
5. **PUT_CLEANER**: `room_id + "03" + "01-02" + "02"` - 將載具放入清潔機進行清潔製程 (2工位)
6. **PUT_SOAKER**: `room_id + "04" + "01-06" + "02"` - 將載具放入浸潤機進行泡藥製程 (6工位)
7. **PUT_PRE_DRYER**: `room_id + "05" + "01-08" + "02"` - 將載具放入預乾燥機進行預乾燥製程 (8工位)

##### 製程流程順序
**製程1 (泡藥1次)**: TAKE_TRANSFER → PUT_CLEANER → TAKE_CLEANER → PUT_SOAKER → TAKE_SOAKER → PUT_PRE_DRYER
- 注意：PUT_PRE_DRYER 後由 Unloader AGV 負責取出，Loader 不執行 TAKE_PRE_DRYER

**製程2 (泡藥2次)**: TAKE_TRANSFER → PUT_CLEANER → TAKE_CLEANER → PUT_SOAKER → TAKE_SOAKER → PUT_SOAKER → TAKE_SOAKER → PUT_PRE_DRYER
- 注意：PUT_PRE_DRYER 後由 Unloader AGV 負責取出

##### 預乾燥機 (PRE_DRYER) 設備特性
- **工位配置**: 總共8格 (上排4格 + 下排4格)
- **作業分工**: Loader AGV 負責放入 (PUT_PRE_DRYER)，Unloader AGV 負責取出
- **Unloader 取料方式**: 一次取2格的批量處理方式

##### Loader AGV 車載料架配置
- **物理結構**: 4格料架垂直排列 (第1層到第4層)
- **S尺寸產品配置**: 可使用全部4層 (1、2、3、4層)
- **L尺寸產品配置**: 只使用第1層和第3層 (2、4層掛勾由人工解下)
- **載重適配**: 根據產品尺寸調整車載料架配置，最佳化載運效率

##### 技術特性
- **Hokuyo 8-bit 光通訊模組**: 前方配備 `hokuyo_dms_8bit_1`，提供 PLC 8-bit 資料通訊
- **Work ID 動態計算**: 自動計算工作 ID 範圍，支援不同房間的動態配置
- **狀態機架構**: Idle State → Vision Position State 的智能狀態轉換
- **車載料架管理**: 支援 S/L 產品的不同料架配置需求
- **完整測試覆蓋**: 具備完整的測試套件和測試報告

#### Unloader AGV (卸載車)
- **工作區域**: 房間內部  
- **主要職責**: 製程設備 → 出口傳送箱 取料作業
- **詳細業務流程**: 4大工作流程，基於 work_id 智能路由系統

##### Work ID 路由系統 (基於實際程式碼分析)
Unloader AGV 採用智能 work_id 路由系統，格式：`room_id + equipment_type + station_number + action_type`

**設備類型編碼**：
- `02` = BOX_OUT_TRANSFER (出口傳送箱)
- `05` = PRE_DRYER (預烘機)
- `06` = OVEN (烘箱)

**動作類型編碼**：
- `01` = TAKE (取料)
- `02` = PUT (放料)

##### 4大業務流程 (基於 idle_state.py 實作)

**製程設備取料**：
1. **TAKE_PRE_DRYER**: `room_id + "05" + "01-04" + "01"` - 從預烘機取出完成預乾燥的載具
   - 工作站數量: 4個工作站 (每站包含2個 PORT，總計8格)
   - 取料方式: **一次取2格** (批量處理方式)
   - 計算邏輯: Port 1-4 → row=1, Port 5-8 → row=2, column=0

2. **TAKE_OVEN**: `room_id + "06" + "01-02" + "01"` - 從烘箱下排取出完成烘乾的載具
   - 烘箱配置: 總共8格 (上排4格 + 下排4格)
   - 取料位置: **下排4格** (烤完後的出料位置)
   - 取料方式: **一次取2格** (批量處理方式)
   - Work ID 計算: 2個工作站 (一次取2格的邏輯)

**製程設備放料**：
3. **PUT_OVEN**: `room_id + "06" + "01-02" + "02"` - 將載具放入烘箱上排進行烘乾製程
   - 烘箱配置: 總共8格 (上排4格 + 下排4格)
   - 放料位置: **上排4格** (烘乾製程的進料位置)
   - 放料方式: **一次放2格** (批量處理方式)
   - Work ID 計算: 2個工作站 (一次放2格的邏輯)

**出口轉移**：
4. **PUT_BOXOUT_TRANSFER**: `room_id + "02" + "01-02" + "02"` - 將處理完成的載具放入出口傳送箱
   - 出口傳送箱配置: 總共4格 (與入口傳送箱相同結構)
   - 放料方式: **一次放2格** (可選擇上2格或下2格)
   - Work ID 計算: 2個工作站 (一次放2格的邏輯)

##### 製程流程角色
- **預烘機取料**: Loader AGV 負責 PUT_PRE_DRYER，Unloader AGV 負責 TAKE_PRE_DRYER
- **烘箱雙向操作**: Unloader AGV 負責 PUT_OVEN 和 TAKE_OVEN
- **最終出料**: Unloader AGV 負責 PUT_BOXOUT_TRANSFER，等待 Cargo AGV 收集

##### 房間內設備物理配置詳解

**入口傳送箱 (TRANSFER)**：
- 物理結構: 4格配置 (上2格 + 下2格)
- Cargo AGV: 將 Carrier 從 Rack 放入
- Loader AGV: 一次取1格進行後續製程

**預烘機 (PRE_DRYER)**：
- 物理結構: 8格配置 (上排4格 + 下排4格)
- Loader AGV: 負責 PUT_PRE_DRYER (放入)
- Unloader AGV: 負責 TAKE_PRE_DRYER (一次取2格)

**烘箱 (OVEN)**：
- 物理結構: 8格配置 (上排4格 + 下排4格)
- 上排4格: **進料位置** - Unloader PUT_OVEN (一次放2格)
- 下排4格: **出料位置** - Unloader TAKE_OVEN (一次取2格)
- 烘乾流程: 上排進料 → 烘乾製程 → 下排出料

**出口傳送箱 (BOX_OUT_TRANSFER)**：
- 物理結構: 4格配置 (與入口傳送箱相同)
- Unloader AGV: 一次放2格 (可選上2格或下2格)
- Cargo AGV: 取出 Carrier 放回 Rack

##### 批量處理邏輯
- **Unloader AGV 特色**: 所有操作都採用 **一次2格** 的批量處理方式
- **效率提升**: 減少機械臂動作次數，提高整體製程效率
- **Work ID 設計**: 2個工作站對應一次處理2格的邏輯設計

##### Unloader AGV 車載料架配置
- **物理結構**: 4格料架，上排2格 + 下排2格
- **S尺寸產品配置**: 
  - 使用上排2格 + 下排2格 (全部4格可用)
  - 一次處理2格的批量操作
  - 車載容量: 最大4個 Carrier
- **L尺寸產品配置**: 
  - 只使用上排2格 (下排掛勾由人工解下)
  - 一次處理2格 (僅上排操作)
  - 車載容量: 最大2個 Carrier

##### 技術特性
- **Hokuyo 8-bit 光通訊模組**: 前方配備 `hokuyo_dms_8bit_1`，提供 PLC 8-bit 資料通訊
- **批量取料邏輯**: 預烘機、烘箱、出口傳送箱都採用一次處理2格的高效方式
- **參數化控制**: `UnloaderRobotParameter` 支援多工位參數管理和批量計算
- **車載料架管理**: 上下排分離設計，最佳化批量處理效率
- **完整測試覆蓋**: 具備預烘機計算邏輯和取料數量的專項測試

### 🔄 完整房間製程流程 (包含 A/B 面和 NG 處理)
```
完整 Rack 處理循環
1. KUKA AGV → 將滿載 Rack 送到房間門口 (A面朝向)
2. Cargo AGV → 處理 A 面所有 Carrier
   ├── 3D 掃描整個 A 面
   ├── 逐一 OCR 檢查每個 Carrier
   ├── 符合製程 → 送入口傳送箱 → 執行製程
   └── 不符合製程 → 標記 NG，留在 Rack 上
3. A 面處理完成後
4. WCS → 產生 Rack 轉向任務 (180度轉向)
5. KUKA AGV → 執行轉向任務，將 Rack 轉180度 (B面朝向)
6. Cargo AGV → 處理 B 面所有 Carrier (重複步驟2流程)
7. 雙面處理完成，Rack 送到人工收料區

注：所有 OCR NG 問題都在房間入口當下即時處理，不會有 NG Rack 事後處理
```

#### 製程執行詳細流程 (適用A/B雙面)
```
房間內製程流程 (每個符合的Carrier)
3. Loader AGV → 從入口傳送箱內側取出 Carrier (TAKE_TRANSFER)
4. 製程序列執行:
   製程1 (泡藥1次):
   ├── Loader: PUT_CLEANER → 清潔製程
   ├── Loader: TAKE_CLEANER → 取出清潔完成的載具
   ├── Loader: PUT_SOAKER → 浸潤製程 (泡藥1次)
   ├── Loader: TAKE_SOAKER → 取出浸潤完成的載具
   ├── Loader: PUT_PRE_DRYER → 放入預烘機
   ├── Unloader: TAKE_PRE_DRYER → 從預烘機取出 (一次2格)
   ├── Unloader: PUT_OVEN → 放入烘箱上排 (一次2格，進料位置)
   ├── [烘乾製程執行: 上排 → 下排]
   ├── Unloader: TAKE_OVEN → 從烘箱下排取出 (一次2格，出料位置)
   └── Unloader: PUT_BOXOUT_TRANSFER → 送到出口傳送箱 (一次2格)

   製程2 (泡藥2次):
   ├── Loader: PUT_CLEANER → 清潔製程
   ├── Loader: TAKE_CLEANER → 取出清潔完成的載具
   ├── Loader: PUT_SOAKER → 第1次浸潤製程
   ├── Loader: TAKE_SOAKER → 取出第1次浸潤完成的載具
   ├── Loader: PUT_SOAKER → 第2次浸潤製程
   ├── Loader: TAKE_SOAKER → 取出第2次浸潤完成的載具
   ├── Loader: PUT_PRE_DRYER → 放入預烘機
   └── [後續 Unloader 流程同製程1]

5. Cargo AGV → 房間出口裝載作業
   ├── 前提：房間出口已有空Rack停靠（從入口直接搬來或從系統空料車停車區調度）
   ├── 從出口傳送箱取出處理完成的Carrier
   ├── 透過8bit通訊協調門控
   ├── 將Carrier放回房間出口Rack的對應位置
   ├── 處理A面所有位置
   ├── Rack轉向180度
   ├── 處理B面所有位置
   └── 完成後由KUKA AGV搬運滿載Rack到人工收料區
```


### 🚛 AGV 車載料架配置對比

| AGV 類型 | 車載料架結構 | 載運方式 | S尺寸產品 | L尺寸產品 | 主要用途 |
|---------|------------|---------|----------|----------|---------|
| **Loader AGV** | 4層垂直排列 | 一次1格精密操作 | 使用全部4層 | 僅用1、3層 (2、4層人工解下) | 房間內精密製程轉移 |
| **Unloader AGV** | 上2格+下2格 | 一次2格批量操作 | 使用上排2格+下排2格 | 僅用上排2格 (下排掛勾人工解下) | 房間內批量取出和轉移 |
| **Cargo AGV** | 機械臂直接操作 | 單一 Carrier 精密抓取 | 直接操作 Carrier | 直接操作 Carrier | Rack ↔ 傳送箱轉移+品檢 |

### 🔧 車載料架設計優勢

**Loader AGV 垂直4層設計**：
- **產品適配性**: S/L尺寸產品彈性配置
- **人工介入**: L產品時人工調整掛勾配置
- **精密操作**: 一次1格確保製程精度
- **工位適應**: 適合各製程設備的單一操作需求

**Unloader AGV 上下2排設計**：
- **批量效率**: 一次2格提高後段製程效率
- **產品適配性**: S產品使用上下排全部4格，L產品僅用上排2格
- **人工介入**: L產品時下排掛勾由人工解下
- **分層管理**: 上下排獨立操作，靈活調度
- **預烘機適配**: 完美對應8格設備的批量取料需求
- **烘箱整合**: 上排進料、下排出料的直接對應

### 🏗️ 設計邏輯和優勢
- **分層責任制**: KUKA(長距離+轉向) → Cargo(轉運+品檢) → Loader/Unloader(精密操作)
- **車載料架最佳化**: 不同 AGV 的料架設計完美匹配其操作特性
- **產品尺寸適配**: S/L產品的不同料架配置策略
- **安全隔離**: 傳送箱雙門互鎖，房間內外操作獨立
- **通訊協調**: 8bit通訊確保門控安全
- **製程彈性**: 支援1次/2次泡藥的不同製程需求
- **Rack利用最大化**: A/B雙面處理，提高載具使用效率
- **製程防錯完整性**: OCR檢查 + NG處理 + 人工介入，確保正確製程
- **系統可恢復性**: NG Rack可重新設為空Rack循環使用
- **人機協作平衡**: 關鍵決策點保留人工介入，提高可靠性
- **麥克納姆輪精密操作**: 所有AGV都具備全向移動和機械臂精密操作能力

### 📋 WCS任務類型設計原理
基於實際業務需求，WCS系統設計了多種任務類型：
- **正常搬運任務**: 射出機 → 房間 → 下一製程
- **Rack轉向任務**: 180度轉向，確保A/B雙面都能處理
- **NG處理任務**: NG Rack → NG處理區，空Rack → 回放格
- **異常處理任務**: 設備故障、路徑阻塞等特殊情況

## 🏭 房間外 Rack 流程系統 (基於 AI WCS 七大業務流程)

### 🗂️ 系統區域配置 (基於 location_id)

根據 AI WCS 統一決策引擎分析，系統有以下主要區域：

```
🏭 系統準備區 (Location ID: 11-18)
├── 儲存等待送到房間的 Rack
├── OPUI 派車後的 Rack 暫存區
├── 作為房間製程的輸入緩衝區
└── 支援多個 Rack 同時準備

🏗️ 人工收料區 (Location ID: 51-55) 
├── 製程完成後的滿料架處理區
├── 需要人工取出 Carrier 的區域
├── 最終產品的收集點
└── 與空料架回收區部分重疊使用

🔄 空料架管理系統
├── 系統空架區 (Location ID: 31-34)
│   ├── 系統管理的空 Rack 庫存
│   ├── 用於滿料架到人工收料區流程
│   └── 空 Rack 的緩衝儲存區域
├── 系統空料車停車區
│   ├── 功能：當房間出口已有Rack時，存放從房間入口清空的Rack
│   ├── 決策觸發：房間出口占用檢查結果為「有Rack」
│   ├── 後續調度：等待房間出口空出後，調度到需要的位置
│   └── 避免房間入口堵塞，維持生產流暢
├── 空料架回收區 (Location ID: 51-54)
│   ├── 清空後的 Rack 回收暫存
│   ├── 準備重新投入系統循環
│   └── 與人工收料區共用部分空間
└── 人工回收空料架區 (Location ID: 91-92)
    ├── 作業員手動放置的空 Rack
    ├── 等待系統重新識別和回收
    └── 經 AGVCUI 重置後的 Rack 暫存

```

### 🔄 WCS 六級業務流程架構

基於 AI WCS 統一決策引擎的完整業務流程優先度系統（已移除 NG 料架回收）：

#### 🔴 第1級：AGV旋轉檢查 (Priority: 100)
```
Rack 180度轉向流程
房間X入口/出口 → 中間點 → 目標位置
├── A面處理完成後執行
├── 3節點移動方式實現
├── 防重複檢查機制  
├── 最高優先級確保機器手臂對接
└── Work ID: 220001 (kuka-移動貨架)
```


#### 🟠 第2級：滿料架到人工收料區 (Priority: 80)
```
製程完成後的 Rack 處理
房間X出口 → 人工收料區 (51-55)
├── 製程完成的滿料架搬運
├── 使用系統空架區 (31-34) 的空 Rack
├── 人工取出製程完成的 Carrier
├── Rack 清空後進入回收流程
├── 系統空架區可用性檢查
├── Carrier搬運需求判斷
└── Work ID: 220001 (kuka-移動貨架)
```

#### 🟠 第3級：人工收料區搬運 (Priority: 80)
```
人工收料區內部流程
人工收料區內的 Rack 重新分配和最佳化
├── Cargo 任務完成後的後續處理
├── 多種料架狀態的靈活調度
├── 確保人工收料區的高效利用
├── 避免資源衝突
└── Work ID: 220001 (kuka-移動貨架)
```

#### 🟡 第4級：系統準備區到房間 (Priority: 60)
```
系統投料流程
系統準備區 (11-18) → 房間X入口
├── OPUI 派車後的 Rack 調度
├── 檢查房間入口是否可用
├── 確保製程的連續供料
├── 一次只處理一個 Rack，避免衝突
├── 房間1-10全覆蓋支援
└── Work ID: 220001 (kuka-移動貨架)
```

#### 🟢 第5級：空料架搬運 (Priority: 40)
```
房間入口空 Rack 處理（兩種決策路徑）
房間X入口 (空Rack) → 判斷房間出口狀態 → 目標位置

決策邏輯：
├── 判斷：房間X出口是否有Rack停靠？
├── 路徑1：房間出口【無】Rack
│   ├── 將空Rack直接搬到房間X出口
│   ├── 準備收集處理完成的Carrier
│   └── 優化流程效率
├── 路徑2：房間出口【有】Rack
│   ├── 將空Rack搬到系統空料車停車區
│   ├── 避免房間出口堵塞
│   └── 等待後續調度使用
├── 房間1-10全覆蓋
└── Work ID: 220001 (kuka-移動貨架)
```

#### 🟢 第6級：人工回收空料架 (Priority: 40)
```
空 Rack 回收循環
人工回收空料架區 (91-92) → 空料架回收區 (51-54)
├── 人工手動放置的空 Rack 處理
├── 使用特殊 Work ID '230001' (kuka-流程觸發)
├── 三階段條件檢查確保安全
├── 唯一的 workflow 觸發流程
├── 重新進入系統循環使用
└── Work ID: 230001 (kuka-流程觸發)
```

### 🎯 房間外流程的關鍵特點

#### 完整的 Rack 生命週期管理
```
Rack 生命週期流程
1. 系統準備區 (11-18) → Rack 進入製程前的準備
2. 房間製程 → 實際的泡藥水處理（OCR NG 即時處理）
3. 人工收料區 (51-55) → 製程完成後的人工處理
4. 空料架回收 (91-92 → 51-54) → 重新循環使用
```

#### WCS 智能調度機制
- **六級優先度系統**: Priority 100(旋轉) → 80(滿料架/人工) → 60(系統投料) → 40(空料架)
- **Work ID 分類管理**: 220001(移動貨架主流程) + 230001(流程觸發特殊)
- **衝突避免邏輯**: 檢查位置佔用和重複任務執行
- **批次最佳化**: 減少70%資料庫查詢，提升系統效能
- **房間動態擴展**: 支援房間1-10的靈活配置

#### 人機協作設計原理
- **自動化主導**: WCS 統一決策引擎自動調度大部分流程
- **關鍵點人工介入**: OCR NG 即時處理、AGVCUI 資料清除、空 Rack 放置確認
- **狀態同步機制**: 確保系統狀態與實際情況一致
- **製程防錯完整性**: OCR 檢查 + 即時 Alarm 機制 + 人工現場處理

#### 資源循環利用系統
- **Rack 重複使用**: 完整的清空、重置、回收循環
- **區域彈性配置**: 多個區域可共用空間，提高利用率
- **智能資源分配**: 根據實際需求動態分配 Rack 和位置資源
- **異常即時處理**: OCR NG 當下處理完畢，Alarm Reset 後系統恢復

## 📊 資料表對應關係

### 設備管理 (Equipment)
```
eqp 表 → 傳送箱設備 (入口/出口傳送箱)
├── location_id → 設備所在位置
├── name → 設備名稱
└── description → 設備描述

eqp_port 表 → 傳送箱的 4 個空位
├── eqp_id → 所屬設備
├── name → 空位名稱 (上2格/下2格)
└── description → 空位描述

eqp_signal 表 → 通訊訊號 (門控、狀態)
├── eqp_id → 所屬設備
├── eqp_port_id → 相關空位
├── dm_address → PLC 通訊地址
├── value → 訊號值
└── type_of_value → 訊號類型
```

### 路徑管理 (Navigation)  
```
node 表 → 路徑點 (x, y 座標)
├── node_type_id → 節點類型
├── x, y → 座標位置
└── created_at, updated_at → 時間戳

edge 表 → 路徑連接 (from_id → to_id)
├── from_id, to_id → 連接的節點
├── weight → 路徑權重
└── heading → 行進方向

location 表 → 位置參考 (與 node 1:1 對應)
├── location_status_id → 位置狀態 (未知/未佔用/佔用)
├── room_id → 所屬房間
├── node_id → 對應路徑點
└── name → 位置名稱
```

## 🚦 交通管制系統

### 系統架構
在房間外的走廊區域，KUKA AGV 和 Cargo AGV 需要通過相同的交管區域，系統透過 Traffic 表記錄交管區域狀態，並透過 Web API 讓 KUKA 車呼叫取得交管區的使用權。

### 資料庫設計
```sql
-- TrafficZone 模型 (agvc_rcs.py)
traffic_zone 表：
├── id: 交管區 ID
├── owner_agv_id: 當前控制該區域的 AGV ID  
├── name: 交管區名稱
├── description: 區域描述
├── points: 多邊形座標 (JSON 字串)
├── status: 狀態 (free 或 controlled)
└── enable: 是否啟用該交管區
```

### Web API 整合
KUKA AGV 透過以下 API 端點管理交管區使用權：

```bash
# 申請交管區使用權
POST /traffic/acquire
{
  "trafficId": "1",
  "agvId": "kuka001"
}

# 釋放交管區使用權  
POST /traffic/release
{
  "trafficId": "1", 
  "agvId": "kuka001"
}
```

### 控制邏輯
- **TrafficController** (`traffic_controller.py`): 核心交通控制邏輯
  - `acquire_traffic_zone()`: 獲取交管區控制權
  - `release_traffic_zone()`: 釋放交管區控制權
  - 支援 ID 和名稱兩種識別方式
- **狀態管理**: `free` (空閒) ↔ `controlled` (被控制)
- **擁有者追蹤**: 記錄當前控制該區域的 AGV
- **衝突防護**: 同一時間只允許一台 AGV 控制特定區域

## 🚪 門控制系統

### 系統概述
KUKA AGV 在房間外各區域移動時會經過多個電動門，需要透過 Web API 呼叫門控制器來控制指定門的開關狀態。

### 門控制架構
```
門控制系統架構
├── Web API Layer (web_api/routers/door.py)
│   ├── POST /door/control - 門開關控制
│   └── POST /door/state - 門狀態查詢
├── 業務邏輯層 (ecs/door_logic.py)
│   ├── DoorLogic - 門控制邏輯
│   ├── async_control_door() - 異步門控制
│   └── async_state_door() - 異步門狀態查詢
├── 配置管理層 (ecs/door_controller_config.py)
│   ├── DoorControllerConfig - 門配置管理
│   └── door_config.yaml - 門配置檔案
└── PLC 通訊層 (plc_proxy/plc_client.py)
    ├── async_force_on() - PLC 強制 ON
    ├── async_force_off() - PLC 強制 OFF
    └── async_read_continuous_byte() - PLC 狀態讀取
```

### 門配置管理
```yaml
# /app/config/door_config.yaml
doors:
  - "1,MR,100,DM,5000"  # 門ID=1, 控制=MR100, 狀態=DM5000
  - "2,MR,101,DM,5001"  # 門ID=2, 控制=MR101, 狀態=DM5001  
  - "3,MR,102,DM,5002"  # 門ID=3, 控制=MR102, 狀態=DM5002
  - "4,MR,103,DM,5003"  # 門ID=4, 控制=MR103, 狀態=DM5003
  - "5,MR,104,DM,5004"  # 門ID=5, 控制=MR104, 狀態=DM5004
```

### Web API 接口
KUKA AGV 透過以下 API 端點控制電動門：

```bash
# 門開關控制
POST /door/control
{
  "doorId": "1",
  "isOpen": true    # true=開門, false=關門
}

# 門狀態查詢
POST /door/state  
{
  "doorId": "1"
}
# 回應: {"doorId": 1, "state": "OPEN", "isOpen": true, "success": true}
```

### 控制流程
1. **KUKA AGV 請求**: 透過 HTTP POST 發送門控制請求
2. **API 路由**: `door.py` 接收請求並解析參數
3. **業務邏輯**: `DoorLogic` 根據門ID查找配置
4. **PLC 通訊**: 透過 `PlcClient` 發送 PLC 指令
   - 開門: `async_force_on(MR, address)`
   - 關門: `async_force_off(MR, address)`
5. **狀態回饋**: 透過 PLC 狀態位 (DM) 確認門的實際狀態

### 技術特點
- **異步操作**: 支援非阻塞的門控制操作
- **配置化管理**: 透過 YAML 檔案管理門的 PLC 位址映射
- **狀態監控**: 實時監控門的開關狀態
- **錯誤處理**: 完整的異常處理和日誌記錄
- **批量控制**: 支援多個門的批量操作

## 🗺️ 地圖匯入系統

### 系統概述
RosAGV 支援兩套獨立的 AGV 地圖系統，透過 Web API 將地圖檔案匯入資料庫，並在 AGVCUI 上顯示對應的節點 (node/kuka_node) 和邊 (edge/kuka_edge)。

### 資料庫架構
```sql
-- CT 車系統地圖
node 表：
├── id: 節點 ID (TagNo)
├── x, y: 座標位置 (像素)
├── node_type_id: 節點類型
└── created_at, updated_at: 時間戳

edge 表：
├── from_id, to_id: 連接的節點 ID
├── weight: 路徑權重
├── name: 邊名稱 (格式: "from_id-to_id")
└── created_at, updated_at: 時間戳

-- KUKA 車系統地圖  
kuka_node 表：
├── id: 節點 ID (nodeNumber)
├── uuid: KUKA 節點 UUID
├── x, y: 座標位置 (像素)
├── node_type_id:節點類型
└── created_at, updated_at: 時間戳

kuka_edge 表：
├── from_id, to_id: 連接的 KUKA 節點 ID
├── weight: 路徑權重
├── name: 邊名稱 (格式: "from_id-to_id")
└── created_at, updated_at: 時間戳
```

### Web API 接口
地圖匯入透過 `map_importer.py` 提供的 API 端點：

```bash
# KUKA 地圖匯入
POST /map_importer/upload-kuka-map/
# 上傳 KUKA Fleet JSON 格式地圖檔案

# CT 地圖匯入
POST /map_importer/upload-ct-map/  
# 上傳 CT AGV 系統 JSON 格式地圖檔案

# KUKA 地圖清除
DELETE /map_importer/delete-kuka-map
# 清除所有 KUKA 地圖資料
```

### 座標轉換機制
系統具備智能座標轉換，將不同單位的座標統一轉換為像素：

```python
# KUKA 座標轉換 (公尺 → 像素)
def kuka_unit_2_px(y, x):
    # 0.0125 m => 1 px
    return y * 1000 / 12.5, x * 1000 / 12.5

# CT 座標轉換 (毫米 → 像素)
def ct_unit_2_px(y, x):
    # 12.5 mm => 1 px
    return y / 12.5, x / 12.5
```

### 地圖處理流程

#### KUKA 地圖處理
1. **JSON 解析**: 解析 `floorList` → `nodeList` + `edgeList`
2. **節點處理**: 使用 `nodeNumber` 作為 `kuka_node.id`，保存 `nodeUuid`
3. **映射建立**: 建立 `nodeLabel` → `nodeNumber` 映射關係
4. **邊處理**: 根據 `nodeLabel` 映射建立邊的 `from_id` → `to_id` 連接
5. **兩階段提交**: 先提交節點，再提交邊，確保外鍵約束

#### CT 地圖處理
1. **JSON 解析**: 解析節點陣列 (`TagNo`, `Tag_X`, `Tag_Y`)
2. **節點儲存**: 使用 `TagNo` 作為 `node.id`，轉換座標後儲存
3. **邊關係**: 處理 `CanToMoveSet` 建立路徑連接關係
4. **兩階段提交**: 先提交節點避免外鍵失敗，再建立邊關係

### AGVCUI 地圖顯示

#### 前端技術棧
```
地圖顯示技術架構
├── Leaflet.js - 地圖視覺化庫
├── map.html - 地圖頁面模板
├── mapDataSync.js - 資料同步管理器
├── mapPage.js - 地圖主要邏輯
└── Socket.IO - 即時資料更新
```

#### 資料流程
```
地圖資料流
資料庫 (node/kuka_node, edge/kuka_edge)
  ↓
Web API 查詢
  ↓  
Socket.IO 即時推送
  ↓
前端 Store 資料管理
  ↓
Leaflet 地圖視覺化
```

#### 地圖功能特性
- **雙地圖顯示**: 同時顯示 CT (藍色) 和 KUKA (綠色) 兩套地圖
- **即時同步**: 透過 Socket.IO 即時同步地圖資料變更
- **互動功能**: 點擊節點查看詳細資訊、任務管理、貨架管理
- **視覺化元素**: 
  - 節點 (node/kuka_node) - 圓形標記
  - 邊 (edge/kuka_edge) - 連接線
  - AGV 位置 - 即時動態顯示
  - 任務路徑 - 執行中任務的路徑標示

### 實際使用方法

#### 地圖匯入操作
```bash
# 匯入 KUKA 地圖
curl -X POST http://localhost:8000/map_importer/upload-kuka-map/ \
  -F "file=@kuka_fleet_map.json"

# 匯入 CT 地圖
curl -X POST http://localhost:8000/map_importer/upload-ct-map/ \
  -F "file=@ct_agv_map.json"

# 清除 KUKA 地圖 (重新匯入前)
curl -X DELETE http://localhost:8000/map_importer/delete-kuka-map
```

#### AGVCUI 地圖查看
1. **開啟地圖頁面**: 瀏覽器前往 `http://localhost:8001/map` 或 `http://agvc.ui/map`
2. **地圖載入**: 自動從資料庫載入所有節點和邊資料
3. **即時顯示**: 
   - 藍色系統: CT AGV 的 node 和 edge
   - 綠色系統: KUKA AGV 的 kuka_node 和 kuka_edge
   - 動態元素: AGV 位置、任務狀態、設備狀態

### 技術特點
- **雙系統支援**: 完整支援 CT 和 KUKA 兩套獨立 AGV 系統
- **座標標準化**: 自動轉換不同系統的座標單位 (公尺/毫米 → 像素)
- **資料一致性**: 兩階段提交確保節點和邊的外鍵關係正確
- **即時同步**: Socket.IO 提供地圖資料的即時更新
- **視覺化管理**: 基於 Leaflet.js 的直觀地圖界面
- **錯誤處理**: 完整的異常處理和資料庫事務回滾機制

## 🏭 房間內 Loader AGV 業務流程

### Loader AGV 工作概覽
在眼鏡生產的房間內，Loader AGV 負責前段製程的精密載料操作，包含 4 種設備類型共 7 種業務流程。

#### 🔧 Work ID 路由系統
Loader AGV 使用智能 work_id 路由系統，格式為：`room_id + equipment_type + station_number + action_type`

**設備類型編碼**：
- `01` = TRANSFER (入口傳送箱)
- `03` = CLEANER (清潔機) 
- `04` = SOAKER (浸潤機)
- `05` = PRE_DRYER (預乾燥機)

**動作類型編碼**：
- `01` = TAKE (取料)
- `02` = PUT (放料)

#### 🚀 7種核心業務流程

**1. TAKE_TRANSFER - 從入口傳送箱取料**
- Work ID: `room_id + "01" + "01" + "01"`
- 功能: 從房間入口的4格傳送箱取得待處理產品
- 設備: 入口傳送箱 (4格: 上2下2)

**2. TAKE_SOAKER - 從浸潤機取料** 
- Work ID 範圍: `room_id + "04" + "01-06" + "01"`
- 功能: 從6台浸潤機取得處理完成的產品
- 設備: 浸潤機站點1-6 (6個工位)

**3. TAKE_CLEANER - 從清潔機取料**
- Work ID 範圍: `room_id + "03" + "01-02" + "01"`  
- 功能: 從2台清潔機取得清潔完成的產品
- 設備: 清潔機站點1-2 (2個工位)

**4. TAKE_PRE_DRYER - 從預乾燥機取料**
- Work ID 範圍: `room_id + "05" + "01-08" + "01"`
- 功能: 從8台預乾燥機取得預乾完成的產品  
- 設備: 預乾燥機站點1-8 (8個工位)

**5. PUT_SOAKER - 放料到浸潤機**
- Work ID 範圍: `room_id + "04" + "01-06" + "02"`
- 功能: 將產品放入6台浸潤機進行藥水處理
- 設備: 浸潤機站點1-6 (6個工位)

**6. PUT_CLEANER - 放料到清潔機**
- Work ID 範圍: `room_id + "03" + "01-02" + "02"`
- 功能: 將產品放入2台清潔機進行清潔處理
- 設備: 清潔機站點1-2 (2個工位)

**7. PUT_PRE_DRYER - 放料到預乾燥機**
- Work ID 範圍: `room_id + "05" + "01-08" + "02"`
- 功能: 將產品放入8台預乾燥機進行預乾處理
- 設備: 預乾燥機站點1-8 (8個工位)

#### 🎯 製程設備配置 (實際驗證)
```
房間內前段製程設備
├── 入口傳送箱 (TRANSFER): 4格 (上2下2)
├── 清潔機 (CLEANER): 2台工位  
├── 浸潤機 (SOAKER): 6台工位
└── 預乾燥機 (PRE_DRYER): 8台工位
```

#### 🤖 技術實作特點
- **3層狀態機**: Base → AGV → Robot 層級控制
- **PGNO系統**: 使用程式編號(Program Number)控制機械臂精確動作
- **Hokuyo光通訊**: 前方8bit光通訊模組提供PLC資料通訊
- **智能端口管理**: 動態AGV端口狀態管理和分配
- **參數化控制**: LoaderRobotParameter 提供靈活的設備配置

#### 🔄 典型業務流程範例
**製程1 (泡藥1次) 流程**:
1. TAKE_TRANSFER → 從入口傳送箱取料
2. PUT_CLEANER → 放入清潔機清潔  
3. TAKE_CLEANER → 從清潔機取出
4. PUT_SOAKER → 放入浸潤機泡藥
5. TAKE_SOAKER → 從浸潤機取出
6. PUT_PRE_DRYER → 放入預乾燥機預乾

**製程2 (泡藥2次) 流程**:
1. TAKE_TRANSFER → 從入口傳送箱取料
2. PUT_CLEANER → 放入清潔機清潔
3. TAKE_CLEANER → 從清潔機取出  
4. PUT_SOAKER → 第1次放入浸潤機泡藥
5. TAKE_SOAKER → 第1次從浸潤機取出
6. PUT_SOAKER → 第2次放入浸潤機泡藥 
7. TAKE_SOAKER → 第2次從浸潤機取出
8. PUT_PRE_DRYER → 放入預乾燥機預乾

#### 🏗️ Loader AGV 車載料架配置
- **物理結構**: 4層垂直排列 (第1層到第4層)
- **S尺寸產品配置**: 可使用全部4層 (1、2、3、4層)
- **L尺寸產品配置**: 只使用第1層和第3層 (2、4層掛勾由人工解下)
- **載重適配**: 根據產品尺寸調整車載料架配置，最佳化載運效率

#### 🎛️ 製程設備工位詳解
**入口傳送箱 (TRANSFER)**:
- Work ID站點: `01` (單一站點)
- 物理配置: 4格 (上2下2)，與出口傳送箱相同結構
- Loader作業: 一次取1格進行後續製程

**清潔機 (CLEANER)**:
- Work ID站點: `01-02` (2個工位)
- 物理配置: 2台清潔設備
- Loader作業: PUT_CLEANER (放入) + TAKE_CLEANER (取出)

**浸潤機 (SOAKER)**:
- Work ID站點: `01-06` (6個工位)
- 物理配置: 6台浸潤設備 (主要製程設備)
- Loader作業: PUT_SOAKER (放入) + TAKE_SOAKER (取出)
- 製程特色: 支援泡藥1次/2次的靈活配置

**預乾燥機 (PRE_DRYER)**:
- Work ID站點: `01-08` (8個工位)
- 物理配置: 8格 (上排4格 + 下排4格)
- Loader作業: 僅負責 PUT_PRE_DRYER (放入)
- 分工設計: Unloader AGV 負責 TAKE_PRE_DRYER (取出)


## 🔄 人工收料區 Rack 管理流程

### 人工收料區完整回收流程
**背景**：目前人工收料區（51-55）的 rack 回收流程不完整，rack 無法真正離開系統管理。

**規劃新流程**：
```
完整的 Rack 回收與重新加入流程（待實作）

階段 1：滿 rack 到人工收料區
1. 製程完成的滿 rack
2. KUKA AGV 搬運到人工收料區（Location ID: 51-55）

階段 2：人工處理與系統移除（已在 OPUI-HMI 實作）
3. 👷 作業員手動取出所有 carrier（實體操作）
4. 📱 作業員使用 OPUI-HMI 操作：
   ├── 選擇該 rack（根據位置或 ID）
   ├── 執行「移出系統」功能
   ├── 系統自動：
   │   ├── 刪除該 rack 上所有 carrier 記錄
   │   └── 設置 rack.location_id = None（標記為系統外）
   └── 確認移除成功
5. 🏭 人員將空 rack 搬到系統外倉儲空間（非系統管理區域）

階段 3：Rack 按需重新進入系統
6. 射出機作業員需要空 rack
7. 👷 人員從倉儲空間手動搬運 rack 到射出機旁
8. 📱 使用 OPUI 輸入 rack 編號（如 "101"）
9. 系統處理：
   ├── 根據 rack 名稱查找系統中的 rack 記錄
   ├── 更新 rack.location_id 到指定停車格
   └── rack 重新成為系統管理的活躍 rack
10. ✅ 空 rack 可供系統調度使用
```

**已實作功能**：

#### OPUI-HMI Rack 移出功能（已實作）
- 在 OPUI-HMI 中已實作「移出系統」功能
- 可將人工收料區的 rack 設置為 location_id = None
- 自動清除相關 carrier 記錄
- 記錄操作日誌供追蹤

#### OPUI 手動加入 Rack 功能（已實作）
- OPUI 已有手動加入 rack 功能（`rackPage.js` 的 `handleAddRack`）
- 可以將 location_id = None 的 rack 重新分配到停車格
- 使用 rack 名稱（如 "101"）而非 ID 來識別
- 驗證 rack 存在性並分配適當的 location_id

### 3. 系統狀態管理更新
**Rack 的三種存在狀態**：
1. **系統內活躍**：`location_id = 具體位置`（如停車格、人工收料區等）
2. **系統內待命**：`location_id = 空料架區`（31-34）
3. **系統外儲存**：`location_id = None`（實體存在於倉儲但系統不追蹤）

**狀態轉換流程**：
```
系統內活躍 ←→ 系統內待命 ←→ 系統外儲存
     ↑                          ↓
     └──────── OPUI 加入 ←──────┘
```

## 🔄 Rack 生命週期管理 2.0

### Rack 的生命週期狀態
基於新的商業邏輯，Rack 現在有以下生命週期：

```
系統外儲存 (location=null) 
    ↓ [手動搬運到射出機]
OPUI 手動加入 → 系統內活躍 (location=停車格)
    ↓ [裝載產品]
製程處理中 (location=房間入口/出口)
    ↓ [製程完成]
人工收料區 (location=51-55)
    ↓ [OPUI-HMI 移出]
系統外儲存 (location=null)
```

### 狀態轉換詳解

#### 1. 系統外 → 系統內
- **觸發**：射出機作業員需要空 Rack
- **操作**：人工搬運 + OPUI「加入 Rack」
- **結果**：Rack 獲得 location_id，納入系統管理

#### 2. 系統內流轉
- **正常流程**：停車格 → 房間入口 → 房間出口 → 人工收料區
- **OCR NG 處理**：即時 PLC Alarm → 人員現場處理 → Alarm Reset 繼續

#### 3. 系統內 → 系統外
- **觸發**：人工收料完成
- **操作**：OPUI-HMI「移出系統」
- **結果**：location_id = null，Rack 脫離系統追蹤

### 關鍵變更點
1. **取消自動叫空車**：不再由 KUKA AGV 自動送空 Rack
2. **手動 Rack 管理**：人工搬運配合 OPUI 系統識別
3. **OCR NG 即時處理**：立即 PLC Alarm，人員現場處理，不再搬到 NG 區
4. **簡化回收流程**：處理完直接移出系統，不再自動搬運


## 🔗 交叉引用
- RosAGV 系統概覽: docs-ai/context/system/rosagv-overview.md
- 雙環境架構: docs-ai/context/system/dual-environment.md
- AGV 車型特性: docs-ai/knowledge/agv-domain/vehicle-types.md
- KUKA Fleet API: docs-ai/knowledge/protocols/kuka-fleet-api.md
- 交通管制技術實作: `app/rcs_ws/src/traffic_manager/`
- 門控制技術實作: `app/ecs_ws/src/ecs/door_logic.py`
- 地圖管理技術實作: `app/web_api_ws/src/web_api/web_api/routers/map_importer.py`
- Loader AGV 技術實作: `app/agv_ws/src/loader_agv/`
- 系統架構: docs-ai/context/system/rosagv-overview.md
- KUKA整合: docs-ai/knowledge/protocols/kuka-fleet-api.md
- 資料庫操作: docs-ai/operations/development/database-operations.md
- OPUI介面: `app/web_api_ws/src/opui/CLAUDE.md`