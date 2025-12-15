# Nginx 反向代理配置詳解

## 🎯 適用場景
- 理解 RosAGV 的 Nginx 反向代理架構
- 配置和管理 Web 服務路由
- 解決 Web 服務存取和網路問題
- 優化 Web 服務效能和安全性

## 📋 Nginx 在 RosAGV 中的角色

### 系統定位
Nginx 在 RosAGV AGVC 管理系統中扮演反向代理伺服器的角色，負責：
- 統一的 Web 服務入口 (Port 80)
- 將不同域名請求路由到對應的內部服務
- 提供 WebSocket 支援以實現即時通訊
- 托管靜態文檔系統

### 容器配置
```yaml
容器名稱: nginx
映像版本: nginx:latest
網路模式: Bridge (192.168.100.0/24)
固定 IP: 192.168.100.252
對外端口: 80
重啟策略: always
```

## 🔧 配置檔案結構

### 檔案位置
- **宿主機配置目錄**: `/home/ct/EBD_agv/nginx/`
- **容器內映射路徑**: `/etc/nginx/conf.d/`
- **主配置檔案**: `default.conf`
- **文檔目錄**: `/home/ct/EBD_agv/design/business-process-docs/`

### 掛載配置
```yaml
volumes:
  - /home/ct/EBD_agv/nginx:/etc/nginx/conf.d:ro
  - /home/ct/EBD_agv/design/business-process-docs:/usr/share/nginx/html/docs:ro
```

## 🌐 虛擬主機配置

### 1. Web API 服務 (agvc.webapi)
```nginx
server {
    listen 80;
    server_name agvc.webapi;
    
    location / {
        proxy_pass http://192.168.100.100:8000;
        proxy_set_header Host $host;
        proxy_set_header X-Real-IP $remote_addr;
        proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
    }
}
```
- **域名**: agvc.webapi
- **目標**: AGVC 容器的 FastAPI 服務 (Port 8000)
- **用途**: 核心 API 服務

### 2. AGVCUI 管理介面 (localhost/agvc.ui)
```nginx
server {
    listen 80;
    server_name localhost agvc.ui;
    
    # Socket.IO WebSocket 支援
    location /socket.io/ {
        proxy_pass http://192.168.100.100:8001;
        proxy_http_version 1.1;
        proxy_set_header Upgrade $http_upgrade;
        proxy_set_header Connection "upgrade";
        proxy_read_timeout 86400;
        proxy_send_timeout 86400;
    }
    
    location / {
        proxy_pass http://192.168.100.100:8001;
    }
}
```
- **域名**: localhost 或 agvc.ui
- **目標**: AGVCUI 服務 (Port 8001)
- **特性**: WebSocket 支援，長連接超時設定

### 3. OPUI 操作介面 (op.ui)
```nginx
server {
    listen 80;
    server_name op.ui;
    
    location / {
        proxy_pass http://192.168.100.100:8002;
        proxy_set_header Host $host;
        proxy_set_header X-Real-IP $remote_addr;
        proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
    }
}
```
- **域名**: op.ui
- **目標**: OPUI 服務 (Port 8002)
- **用途**: 操作員介面

## 📚 動態文檔系統

### 文檔路由配置
```nginx
location /docs/ {
    root /usr/share/nginx/html;
    index index.html;
    
    # SPA 路由支援
    try_files $uri $uri/ /docs/index.html;
    
    # API 端點 CORS 支援
    location ~ ^/docs/(content|api)/ {
        add_header Access-Control-Allow-Origin "*";
        add_header Access-Control-Allow-Methods "GET, OPTIONS";
        add_header Access-Control-Allow-Headers "Content-Type";
    }
}
```

### 檔案類型處理
- **Markdown (.md)**: `text/markdown; charset=utf-8`
- **JSON/JSON5**: `application/json; charset=utf-8`
- **JavaScript/CSS**: 24小時快取
- **圖片資源**: 1年快取，immutable 標記

## ⚡ 效能優化

### gzip 壓縮
```nginx
gzip on;
gzip_vary on;
gzip_min_length 1024;
gzip_types text/plain text/css text/markdown application/json application/javascript;
```
- 啟用 gzip 壓縮
- 最小壓縮檔案大小: 1024 bytes
- 支援文字、CSS、JSON、JavaScript 壓縮

### 快取策略
- **靜態資源**: 長期快取 (1年)
- **JS/CSS**: 中期快取 (24小時)
- **Markdown/JSON**: 短期快取 (1小時)

## 🔒 安全配置

### 安全標頭
```nginx
add_header X-Frame-Options "SAMEORIGIN" always;
add_header X-Content-Type-Options "nosniff" always;
add_header X-XSS-Protection "1; mode=block" always;
add_header Referrer-Policy "strict-origin-when-cross-origin" always;
```
- 防止點擊劫持 (X-Frame-Options)
- 防止 MIME 類型嗅探 (X-Content-Type-Options)
- 啟用 XSS 保護 (X-XSS-Protection)
- 控制 Referrer 資訊 (Referrer-Policy)

### CORS 配置
- 允許跨域存取文檔 API
- 支援 OPTIONS 預檢請求
- 限制允許的 HTTP 方法

## 🔍 診斷和測試

### 配置測試
```bash
# 測試 Nginx 配置語法
docker compose -f docker-compose.agvc.yml exec nginx nginx -t

# 重新載入配置
docker compose -f docker-compose.agvc.yml exec nginx nginx -s reload
```

### 連接測試
```bash
# 測試各服務路由
curl -H "Host: agvc.webapi" http://localhost/health
curl -H "Host: agvc.ui" http://localhost/
curl -H "Host: op.ui" http://localhost/

# 測試文檔系統
curl http://localhost/docs/
```

### 日誌查看
```bash
# 查看 Nginx 存取日誌
docker compose -f docker-compose.agvc.yml logs nginx

# 進入容器查看詳細日誌
docker compose -f docker-compose.agvc.yml exec nginx tail -f /var/log/nginx/access.log
docker compose -f docker-compose.agvc.yml exec nginx tail -f /var/log/nginx/error.log
```

## 🛠️ 常見問題處理

### 502 Bad Gateway
**原因**: 後端服務未啟動或無法連接
**解決**: 
```bash
# 檢查後端服務狀態
docker compose -f docker-compose.agvc.yml ps
# 確認服務端口
netstat -tulpn | grep -E "8000|8001|8002"
```

### WebSocket 連接失敗
**原因**: WebSocket 配置不正確或超時設定過短
**解決**: 確認 `/socket.io/` 路由配置正確，檢查 proxy_read_timeout 設定

### 域名無法解析
**原因**: 本地 hosts 檔案未配置
**解決**: 
```bash
# 添加到 /etc/hosts
192.168.100.252 agvc.webapi
192.168.100.252 agvc.ui
192.168.100.252 op.ui
```

## 📋 維護建議

### 定期檢查
1. 配置語法驗證: `nginx -t`
2. 存取日誌分析: 檢查異常請求
3. 錯誤日誌監控: 及時發現問題
4. 效能指標: 回應時間、吞吐量

### 配置備份
```bash
# 備份 Nginx 配置
cp -r /home/ct/EBD_agv/nginx /home/ct/EBD_agv/nginx.backup.$(date +%Y%m%d)
```

### 更新流程
1. 修改配置檔案
2. 測試配置: `nginx -t`
3. 重新載入: `nginx -s reload`
4. 驗證服務正常

## 🔗 交叉引用
- 雙環境架構: docs-ai/context/system/dual-environment.md
- 技術棧: docs-ai/context/system/technology-stack.md
- Web 開發: docs-ai/operations/development/web/web-development.md
- 容器管理: docs-ai/operations/deployment/container-management.md
- 系統診斷: docs-ai/operations/guides/system-diagnostics.md