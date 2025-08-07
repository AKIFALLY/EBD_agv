# Flow Designer 雙向轉換使用指南

## 🎯 功能概覽
Flow Designer Phase 3.3 完成了雙向同步機制，實現視覺化流程圖與 YAML DSL 的完整互轉功能。

## 🚀 完整使用流程

### 1. 開啟 Flow Designer
```bash
# 啟動 AGVC 系統
agvc_start

# 開啟瀏覽器
http://localhost:8001/flows
```

### 2. 視覺化 → YAML DSL (Phase 3.2)

#### 創建視覺化流程
1. 點擊「新建流程」，輸入流程名稱
2. 從左側節點選板拖拽節點到編輯器：
   - **條件節點**: `check_agv_rotation_flow`
   - **邏輯節點**: `get_room_inlet_point`
   - **動作節點**: `create_task_from_decision`
3. 連接節點形成流程圖
4. 設置節點參數

#### 生成 YAML DSL
- 點擊紅色「生成 YAML DSL」按鈕
- 系統自動生成並下載 YAML 檔案

### 3. YAML DSL → 視覺化 (Phase 3.3)

#### 載入 YAML DSL
1. 點擊藍色「載入 YAML DSL」按鈕
2. 選擇 YAML DSL 檔案 (.yaml 或 .yml)
3. 系統自動解析並重建視覺化流程圖

#### 支援的 YAML 格式
```yaml
# 標準 YAML DSL 格式
variables:
  room_id:
    type: integer
    value: 1
    description: "房間ID"

steps:
  - step: 1
    name: "AGV旋轉流程檢查"
    type: condition_nodes
    function: check_agv_rotation_flow
    parameters:
      room_id: ${room_id}
      agv_id: "agv01"
    outputs:
      - decisions
```

## 🔄 雙向轉換驗證

### 往返一致性測試
1. **視覺化設計** → 創建複雜流程圖
2. **生成 DSL** → Phase 3.2 功能
3. **清空編輯器** → 準備測試載入
4. **載入 DSL** → Phase 3.3 功能
5. **比較結果** → 驗證流程圖一致性

### 支援的節點類型
- **condition_nodes**: 9個條件判斷函數
- **logic_nodes**: 5個邏輯處理函數
- **action_nodes**: 4個動作執行函數
- **script_nodes**: 控制結構 (if_else, for_loop, 變數操作)

## 🎨 視覺化特性

### 節點渲染效果
- 四種節點類型差異化顏色
- 拖拽移動和視覺反饋
- 懸停效果和選中狀態
- 智能位置計算和佈局

### 連接線系統
- SVG 動態連接線
- 貝塞爾曲線平滑效果
- 箭頭指向和動畫
- 自動重建和更新

## 🛠️ 高級功能

### 智能節點映射
- 根據 DSL 函數名自動找到對應節點類型
- 支援類別內查找和直接匹配
- 38個 WCS 函數完整支援

### 錯誤處理機制
- YAML 格式驗證
- 節點類型不匹配警告
- 檔案讀取錯誤處理
- 用戶友好的錯誤通知

## 📱 用戶體驗

### 操作便捷性
- 一鍵檔案選擇對話框
- 自動檔案格式支援
- 進度指示和狀態通知
- 載入動畫和視覺反饋

### 企業級特性
- 完整的元資料保存
- 流程名稱和描述維護
- 變數定義和類型推斷
- 版本資訊和時間戳記

## 🔍 故障排除

### 常見問題
1. **YAML 解析失敗**
   - 檢查 YAML 語法格式
   - 確認必要欄位存在
   - 驗證資料結構完整性

2. **節點創建失敗**
   - 確認函數名稱正確
   - 檢查節點類型映射
   - 驗證參數格式

3. **連接重建問題**
   - 檢查步驟順序
   - 確認節點存在
   - 驗證 DOM 元素狀態

### 調試方法
- 開啟瀏覽器開發工具
- 查看控制台日誌輸出
- 檢查 Phase 3.3 標記的日誌
- 驗證 YAML 檔案格式

## 💡 最佳實踐

### 檔案管理
- 使用描述性的檔案名稱
- 保持 YAML 檔案格式整潔
- 定期備份重要流程設計

### 流程設計
- 合理安排節點佈局
- 設置清晰的節點名稱
- 添加詳細的描述資訊

### 版本控制
- 使用 Git 管理 YAML DSL 檔案
- 記錄重要的流程變更
- 建立版本發布標準

## 🔗 整合應用

### Simple WCS 整合
生成的 YAML DSL 可直接用於：
- Simple WCS 系統執行
- OPUI 叫車流程配置
- 自動化測試場景
- 業務邏輯模板

### 開發工作流
1. **需求分析** → 理解業務流程
2. **視覺化設計** → 使用 Flow Designer 設計
3. **DSL 生成** → 轉換為可執行代碼
4. **測試驗證** → 在 Simple WCS 中測試
5. **部署應用** → 正式環境使用

Flow Designer 的雙向轉換功能為 WCS 業務邏輯開發提供了完整的視覺化解決方案，大幅提升開發效率和維護便利性。