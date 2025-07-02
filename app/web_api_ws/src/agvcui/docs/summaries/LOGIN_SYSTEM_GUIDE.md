# AGVC UI 登入系統指南

## 🔧 修復的問題

**錯誤**: `AttributeError: 'UserCRUD' object has no attribute 'get'`

**原因**: `UserCRUD` 類別中的方法調用了不存在的 `self.get()` 方法，應該使用 `self.get_by_id()`

**修復**: 已將所有 `self.get()` 調用更改為 `self.get_by_id()`

## 🚀 如何測試登入功能

### 1. 啟動服務器
```bash
cd /app/web_api_ws/src/agvcui
python3 -m agvcui.agvc_ui_server
```

### 2. 初始化管理員用戶
訪問: `http://localhost:8001/init-admin`

這會創建預設管理員帳號:
- 用戶名: `admin`
- 密碼: `admin123`

### 3. 測試登入流程
1. 訪問: `http://localhost:8001/login`
2. 輸入管理員帳號密碼
3. 登入成功後會重定向到首頁
4. 檢查導航欄是否顯示用戶信息

### 4. 測試頁面保護
1. 登出後嘗試訪問受保護頁面 (如 `/map`, `/tasks`)
2. 應該會自動重定向到登入頁面
3. 登入後會重定向回原始頁面

## 📁 系統架構

### 核心文件
- `agvcui/auth.py` - 認證功能 (密碼哈希、token 管理)
- `agvcui/middleware.py` - 認證中間件 (頁面保護)
- `agvcui/routers/auth.py` - 登入/登出路由
- `agvcui/templates/login.html` - 登入頁面
- `agvcui/static/js/auth.js` - 前端認證管理

### 資料庫相關
- `db_proxy/models/agvc_client.py` - User 模型
- `db_proxy/crud/user_crud.py` - 用戶 CRUD 操作
- `agvcui/db.py` - 資料庫接口函數

## 🔐 安全特性

### 密碼安全
- 使用 PBKDF2 進行密碼哈希
- 隨機鹽值防止彩虹表攻擊
- 100,000 次迭代增強安全性

### Token 安全
- 自定義 JWT 實現
- HMAC 簽名驗證
- 可配置過期時間 (預設 30 分鐘)

### 會話管理
- HttpOnly cookies 防止 XSS
- 自動過期檢查
- 安全的登出清理

## 👥 用戶角色

### 支援的角色
- `admin` - 管理員 (完整權限)
- `operator` - 操作員 (操作權限)
- `user` - 一般用戶 (檢視權限)

### 權限控制
- 中間件自動檢查認證狀態
- 路由層級的角色權限控制
- 前端權限狀態管理

## 🛠️ 自定義配置

### 環境變數
```bash
SECRET_KEY=your-secret-key-here  # JWT 簽名密鑰
ACCESS_TOKEN_EXPIRE_MINUTES=30   # Token 過期時間
```

### 受保護路徑
在 `middleware.py` 中修改 `protected_paths` 列表:
```python
protected_paths = [
    "/map", "/tasks", "/devices", "/signals",
    "/clients", "/racks", "/products", "/carriers",
    "/rosout_logs", "/runtime_logs"
]
```

## 🐛 故障排除

### 常見問題

1. **登入後立即登出**
   - 檢查系統時間是否正確
   - 確認 SECRET_KEY 設置一致

2. **無法訪問受保護頁面**
   - 檢查中間件是否正確註冊
   - 確認路徑是否在 protected_paths 中

3. **資料庫連接錯誤**
   - 確認資料庫服務正常運行
   - 檢查連接字符串配置

### 調試模式
啟用詳細日誌:
```python
import logging
logging.basicConfig(level=logging.DEBUG)
```

## 📝 API 端點

### 認證相關
- `GET /login` - 顯示登入頁面
- `POST /login` - 處理登入請求
- `GET /logout` - 登出
- `GET /init-admin` - 初始化管理員

### 用戶管理 (未來擴展)
- `GET /users` - 用戶列表 (管理員)
- `POST /users` - 創建用戶 (管理員)
- `PUT /users/{id}` - 更新用戶 (管理員)

## 🔄 下一步開發

### 建議的功能擴展
1. 用戶管理界面
2. 密碼重置功能
3. 多因素認證
4. 審計日誌
5. 會話管理界面

### 安全增強
1. 密碼複雜度要求
2. 登入嘗試限制
3. CSRF 保護
4. 內容安全策略

---

**注意**: 在生產環境中，請務必:
1. 更改預設管理員密碼
2. 設置強密碼的 SECRET_KEY
3. 啟用 HTTPS (設置 secure=True)
4. 定期更新依賴包
