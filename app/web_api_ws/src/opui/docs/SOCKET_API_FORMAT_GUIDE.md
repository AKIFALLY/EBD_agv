# OPUI Socket.IO API 扁平化格式統一指南

## 📋 概述

本文檔定義了 OPUI 專案中 Socket.IO API 的統一扁平化資料交換格式標準，確保前後端資料格式一致性。

## 🎯 格式統一原則

### 1. 統一扁平化格式
- **扁平化結構**：`data.clientId`, `data.machineId`, `data.userAgent` 等直接欄位
- **簡化架構**：避免巢狀結構，提高程式碼可讀性和維護性
- **資料庫相容**：直接對應資料庫欄位，減少轉換邏輯

### 2. 資料提取邏輯
```python
# 後端統一的扁平化資料提取
clientId = data.get("clientId") or sid
machineId = data.get("machineId") or 1
userAgent = data.get("userAgent") or ""
isConnected = data.get("isConnected") or False
```

## 📡 Socket.IO API 完整清單

### 🔐 認證相關 API

#### `login` 事件
**功能**：使用者登入驗證

**前端發送格式**：
```javascript
{
    // 統一扁平化格式
    clientId: "abc123",
    machineId: 1,
    userAgent: "Mozilla/5.0...",
    isConnected: true
}
```

**後端回應格式**：
```javascript
{
    success: true,
    message: "登入成功，clientId: abc123",
    client: { /* 客戶端資料 */ },
    clientId: "abc123"
}
```

#### `client_update` 事件
**功能**：客戶端資料更新

**前端發送格式**：
```javascript
{
    // 統一扁平化格式
    clientId: "abc123",
    machineId: 1,
    userAgent: "Mozilla/5.0...",
    isConnected: true,
    // 操作資料使用 op 欄位
    op: {
        left: { productSelected: 0, products: [...] },
        right: { productSelected: 1, products: [...] }
    }
}
```

**後端回應格式**：
```javascript
{
    success: true,
    message: "設定已儲存",
    client: { /* 更新後的客戶端資料 */ },
    clientId: "abc123"
}
```

### 🚛 任務操作 API

#### `dispatch_full` 事件
**功能**：派滿車操作

**前端發送格式**：
```javascript
{
    side: "left" | "right",
    productName: "ProductABC",  // 新格式
    name: "ProductABC",         // 舊格式（向後相容）
    count: 32,
    rackId: 456,
    room: 2
}
```

#### `cancel_task` 事件
**功能**：取消任務

**前端發送格式**：
```javascript
{
    side: "left" | "right"
}
```

### 🏷️ 料架管理 API

#### `add_rack` 事件
**功能**：新增料架到停車格

**前端發送格式**：
```javascript
{
    side: "left" | "right",
    rack: "RackName001"  // 注意：後端期望 "rack" 而非 "rackName"
}
```

#### `del_rack` 事件
**功能**：刪除料架

**前端發送格式**：
```javascript
{
    rackId: 456
}
```

### 📥 前端接收事件

#### 資料列表更新事件
- `product_list`: `{products: [...]}`
- `machine_list`: `{machines: [...]}`
- `room_list`: `{rooms: [...]}`
- `parking_list`: `{left: [...], right: [...]}`

#### 通知事件
- `notify_message`: `{message: "通知內容"}`
- `error_message`: `{message: "錯誤內容"}`
- `active_tasks`: 任務狀態物件

## ✅ 格式一致性檢查清單

### 前端檢查項目
- [ ] `login` 方法使用分離式架構格式
- [ ] `updateClient` 方法正確轉換資料格式
- [ ] 所有 API 調用使用統一的錯誤處理
- [ ] 向後相容性欄位正確設置

### 後端檢查項目
- [ ] `client_update` 支援分離式架構提取
- [ ] `login` 支援分離式架構提取
- [ ] 所有事件處理器有統一的錯誤回應格式
- [ ] 資料庫操作正確處理欄位映射

## 🔧 實作建議

### 1. 前端統一格式轉換
```javascript
// 統一使用 _convertToUnifiedFormat 方法
const unifiedData = this._convertToUnifiedFormat(separatedData);
this.socket.emit("event_name", unifiedData, callback);
```

### 2. 後端統一資料提取
```python
# 統一的資料提取模式
def extract_user_data(data):
    user_data = data.get("user", {})
    return {
        "clientId": user_data.get("clientId") or data.get("clientId"),
        "machineId": user_data.get("machineId") or data.get("machineId") or 1,
        "userAgent": user_data.get("userAgent") or data.get("userAgent") or "",
        "isConnected": user_data.get("isConnected") or data.get("isConnected") or False
    }
```

### 3. 錯誤處理統一格式
```javascript
// 統一的錯誤回應格式
{
    success: false,
    message: "具體錯誤描述",
    error_code?: "ERROR_CODE",  // 可選的錯誤代碼
    details?: { /* 詳細錯誤資訊 */ }
}
```

## 📈 遷移計劃

### 階段 1：立即修正（已完成）
- ✅ 修正 `client_update` 支援分離式架構
- ✅ 修正 `login` 使用統一格式
- ✅ 新增格式一致性測試

### 階段 2：漸進式改善
- [ ] 簡化前端格式轉換邏輯
- [ ] 統一所有 API 的錯誤處理格式
- [ ] 新增 API 文檔和型別定義

### 階段 3：長期優化
- [ ] 實作執行時格式驗證
- [ ] 新增 TypeScript 型別安全
- [ ] 建立自動化格式一致性檢查

## 🧪 測試驗證

執行格式一致性測試：
```bash
cd app/web_api_ws/src/opui
python -m pytest tests/test_socket_api_format_consistency.py -v
```

## 📚 相關文檔

- [OPUI 專案 README](../README.md)
- [Socket.IO 事件處理器實作](../core/op_ui_socket.py)
- [前端 API 通訊層](../frontend/static/js/api.js)
