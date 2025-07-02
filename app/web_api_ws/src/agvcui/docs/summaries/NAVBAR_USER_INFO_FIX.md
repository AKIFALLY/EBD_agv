# 🔧 Navbar 用戶信息顯示修復

## 🐛 問題描述
在 devices, signals 等頁面時，navbar 仍然顯示"登入"按鈕，而不是顯示"系統管理員"用戶信息。

## 🔍 根本原因
雖然中間件正確處理了認證並將用戶信息存儲在 `request.state.current_user` 中，但是各個路由沒有將這個用戶信息傳遞給模板，導致 navbar 無法顯示正確的用戶信息。

## ✅ 修復的路由

已修復以下所有路由，確保它們都傳遞 `current_user` 給模板：

### 1. `/devices` 路由
```python
# 修復前
return templates.TemplateResponse("devices.html", {
    "request": request,
    "devices": devices,
    "active_tab": "devices",
    # ❌ 缺少 current_user
})

# 修復後
from agvcui.middleware import get_current_user_from_request
current_user = get_current_user_from_request(request)

return templates.TemplateResponse("devices.html", {
    "request": request,
    "devices": devices,
    "active_tab": "devices",
    "current_user": current_user  # ✅ 添加用戶信息
})
```

### 2. `/signals` 路由
✅ 已修復，添加 `current_user` 參數

### 3. `/carriers` 路由
✅ 已修復，添加 `current_user` 參數

### 4. `/racks` 路由
✅ 已修復，添加 `current_user` 參數

### 5. `/products` 路由
✅ 已修復，添加 `current_user` 參數

### 6. `/rosout_logs` 路由
✅ 已修復，添加 `current_user` 參數

### 7. `/runtime_logs` 路由
✅ 已修復，添加 `current_user` 參數

### 已經正確的路由
- ✅ `/` (首頁)
- ✅ `/map`
- ✅ `/tasks`
- ✅ `/clients`

## 🔧 修復模式

每個路由都使用相同的修復模式：

```python
@router.get("/route_name", response_class=HTMLResponse)
async def route_function(request: Request, page: int = 1):
    from agvcui.middleware import get_current_user_from_request
    
    # ... 原有的業務邏輯 ...
    
    current_user = get_current_user_from_request(request)
    
    return templates.TemplateResponse("template.html", {
        "request": request,
        # ... 原有的模板參數 ...
        "current_user": current_user  # 添加用戶信息
    })
```

## 🎯 Navbar 顯示邏輯

Navbar 模板 (`navbar.html`) 使用以下邏輯：

```html
{% if current_user %}
<!-- 顯示用戶信息和下拉選單 -->
<div class="navbar-item has-dropdown is-hoverable">
    <a class="navbar-link">
        <span class="icon">
            <i class="mdi mdi-account-circle"></i>
        </span>
        <span>{{ current_user.full_name or current_user.username }}</span>
        {% if current_user.role == 'admin' %}
        <span class="tag is-small is-primary ml-1">管理員</span>
        {% endif %}
    </a>
    <!-- 下拉選單內容 -->
</div>
{% else %}
<!-- 顯示登入按鈕 -->
<div class="navbar-item">
    <div class="buttons">
        <a class="button is-primary" href="/login">
            <span class="icon">
                <i class="mdi mdi-login"></i>
            </span>
            <span>登入</span>
        </a>
    </div>
</div>
{% endif %}
```

## 📊 修復前後對比

### 修復前
- ❌ 用戶已登入，但 navbar 顯示"登入"按鈕
- ❌ 無法看到當前用戶信息
- ❌ 無法使用登出功能

### 修復後
- ✅ 用戶登入後，navbar 顯示用戶名稱
- ✅ 管理員用戶顯示"管理員"標籤
- ✅ 下拉選單顯示用戶詳細信息
- ✅ 可以正常使用登出功能

## 🚀 現在的完整功能

### 用戶信息顯示
- ✅ 顯示用戶名稱或全名
- ✅ 顯示用戶角色標籤（管理員）
- ✅ 顯示最後登入時間

### 下拉選單功能
- ✅ 用戶詳細信息
- ✅ 登出連結

### 所有頁面一致性
- ✅ 首頁 `/`
- ✅ 地圖 `/map`
- ✅ 任務 `/tasks`
- ✅ 設備 `/devices`
- ✅ 信號 `/signals`
- ✅ 客戶 `/clients`
- ✅ 貨架 `/racks`
- ✅ 產品 `/products`
- ✅ 載具 `/carriers`
- ✅ ROS 日誌 `/rosout_logs`
- ✅ 運行日誌 `/runtime_logs`

## 🔍 測試驗證

### 測試步驟
1. 登入系統（admin / admin123）
2. 訪問各個頁面
3. 檢查 navbar 是否正確顯示用戶信息

### 預期結果
- ✅ 所有頁面的 navbar 都顯示"系統管理員"
- ✅ 用戶下拉選單正常工作
- ✅ 登出功能正常

## 📝 最佳實踐

### 1. 一致的模板參數
所有路由都應該傳遞 `current_user` 參數給模板。

### 2. 使用輔助函數
可以考慮使用 `get_template_context()` 輔助函數來簡化代碼：

```python
from agvcui.utils.template_helpers import get_template_context

return templates.TemplateResponse("template.html", 
    get_template_context(request, "active_tab", 
        devices=devices,
        latest_query=latest_query,
        # ... 其他參數
    )
)
```

### 3. 中間件職責
- ✅ 中間件負責認證和設置 `request.state.current_user`
- ✅ 路由負責從 request 中獲取用戶信息並傳遞給模板
- ✅ 模板負責根據用戶信息顯示適當的 UI

---

**修復完成！** 現在所有頁面的 navbar 都會正確顯示用戶信息。🎉
