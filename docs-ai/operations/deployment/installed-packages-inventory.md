# RosAGV 雙環境套件清單

## 🎯 適用場景
- 了解兩個環境實際安裝的軟體套件
- 進行套件相容性檢查和依賴分析
- 規劃新功能開發時的技術選型
- 解決套件相關的問題和衝突

## 📊 快速對比

### 兩環境主要差異
| 項目 | AGV 環境 | AGVC 環境 |
|------|----------|-----------|
| **基礎映像** | osrf/ros:jazzy-desktop-full | osrf/ros:jazzy-desktop-full |
| **特有套件** | ros-jazzy-joy-linux (搖桿) | Node.js + npm |
| **GPIO 支援** | ✅ gpiod, python3-libgpiod | ✅ 相同 |
| **網路模式** | Host (硬體直接存取) | Bridge (隔離網路) |
| **主要用途** | 硬體控制、感測器、搖桿 | Web 服務、資料庫、管理介面 |
| **json5 工具** | ❌ 未安裝 | ❌ 未安裝 (但可透過 npm 安裝) |
| **容器數量** | 1個 (rosagv) | 4個 (agvc_server, postgres, nginx, pgadmin) |

## 📋 基礎環境

### 基礎映像
兩個環境都使用相同的基礎映像：
- **基礎映像**: `osrf/ros:jazzy-desktop-full`
- **Ubuntu 版本**: 24.04 LTS (Noble Numbat)
- **ROS 2 版本**: Jazzy Jalapa
- **Python 版本**: 3.12

### 共同設定
- **語言環境**: zh_TW.UTF-8 (支援中文顯示)
- **Shell**: Bash
- **虛擬環境**: `/opt/pyvenv_env`
- **SSH 端口**: 2200
- **使用者**: root (密碼: 36274806), ct (密碼: 36274806, sudo 權限)

## 🔧 系統工具套件（兩環境共用）

### 開發工具
| 套件 | 用途 | 版本/備註 |
|------|------|-----------|
| git | 版本控制 | 最新版 |
| cargo | Rust 套件管理 | 用於建置 Zenoh |
| clang | C/C++ 編譯器 | 用於建置 ROS 2 套件 |
| software-properties-common | PPA 管理工具 | 用於添加套件源 |
| python3-venv | Python 虛擬環境 | Python 3.12 |

### 網路工具
| 套件 | 用途 | 版本/備註 |
|------|------|-----------|
| net-tools | 網路工具集 (ifconfig等) | 經典網路工具 |
| iputils-ping | ping 工具 | 網路連通性測試 |
| telnet | Telnet 客戶端 | 端口測試 |
| iproute2 | 現代網路工具 (ip, ss等) | 推薦使用 |
| ipset | IP 集合管理 | 防火牆相關 |
| iptables | 防火牆管理 | 網路安全 |
| dnsutils | DNS 工具 (dig, nslookup) | DNS 診斷 |
| curl | HTTP 客戶端 | 檔案下載和 API 測試 |

### 系統管理工具
| 套件 | 用途 | 版本/備註 |
|------|------|-----------|
| openssh-server | SSH 伺服器 | Port 2200 |
| rsync | 檔案同步工具 | 遠端同步 |
| lsof | 檔案和端口查看 | 系統診斷 |
| usbutils | USB 工具 (lsusb等) | USB 設備管理 |
| sudo | 權限管理 | 系統管理 |
| aggregate | 資料聚合工具 | 日誌分析 |

### GPIO 控制（AGV 環境特別需要）
| 套件 | 用途 | 版本/備註 |
|------|------|-----------|
| gpiod | GPIO 控制工具 | Linux GPIO 子系統 |
| python3-libgpiod | Python GPIO 綁定 | GPIO 程式開發 |

### 搜尋和處理工具
| 套件 | 用途 | 版本/備註 |
|------|------|-----------|
| ripgrep (rg) | 高速文字搜尋 | 替代 grep |
| fzf | 模糊搜尋工具 | 互動式搜尋 |
| jq | JSON 處理工具 | JSON 查詢和修改 |
| yq | YAML 處理工具 | YAML 查詢和修改 |
| less | 分頁檢視器 | 檔案檢視 |
| unzip | 解壓縮工具 | ZIP 檔案處理 |
| zstd | Zstandard 壓縮 | 高效壓縮 |
| ❌ json5 | JSON5 處理工具 | **未安裝** (需手動安裝) |

### 開發者工具
| 套件 | 用途 | 版本/備註 |
|------|------|-----------|
| python3-debugpy | Python 除錯器 | VSCode 遠端除錯 |
| gh | GitHub CLI | GitHub 操作 |
| man-db | 手冊頁系統 | 文檔查看 |
| zsh | Z Shell | 替代 shell |

## 🤖 ROS 2 相關套件（兩環境共用）

### ROS 2 核心
| 套件 | 用途 | 版本/備註 |
|------|------|-----------|
| ros-jazzy-desktop-full | ROS 2 完整桌面版 | 基礎映像提供 |
| ros-jazzy-joy-linux | 搖桿驅動 | Linux 搖桿支援 |
| colcon | ROS 2 建置工具 | 工作空間建置 |
| rosdep | ROS 依賴管理 | 套件依賴安裝 |

### Zenoh RMW
| 元件 | 位置 | 用途 |
|------|------|------|
| rmw_zenoh | /opt/ws_rmw_zenoh | Zenoh 中間件實作 |
| zenoh-c | 內建於 rmw_zenoh | Zenoh C 綁定 |
| zenoh-rust | 透過 cargo 建置 | Zenoh 核心實作 |

## 🐍 Python 套件（兩環境共用）

所有 Python 套件安裝在虛擬環境 `/opt/pyvenv_env` 中：

### Web 框架
| 套件 | 用途 | 使用場景 |
|------|------|-----------|
| fastapi | 現代 Web 框架 | REST API 開發 |
| uvicorn | ASGI 伺服器 | FastAPI 運行 |
| pydantic | 資料驗證 | API 模型定義 |
| jinja2 | 模板引擎 | HTML 模板渲染 |
| python-multipart | 表單處理 | 檔案上傳 |

### 資料庫
| 套件 | 用途 | 使用場景 |
|------|------|-----------|
| sqlalchemy | ORM 框架 | 資料庫操作 |
| sqlmodel | SQLAlchemy + Pydantic | 型別安全 ORM |
| psycopg2 | PostgreSQL 驅動 | PostgreSQL 連接 |

### 通訊
| 套件 | 用途 | 使用場景 |
|------|------|-----------|
| python-socketio | Socket.IO | 即時通訊 |
| paho-mqtt | MQTT 客戶端 | MQTT 通訊 |
| requests | HTTP 客戶端 | REST API 呼叫 |
| httpx | 現代 HTTP 客戶端 | 異步 HTTP 請求 |

### 其他
| 套件 | 用途 | 使用場景 |
|------|------|-----------|
| pygame | 遊戲開發框架 | 搖桿控制、圖形介面 |
| networkx | 圖論演算法 | 路徑規劃 |

## 🎯 AGVC 環境專有套件

### Node.js 生態系統
| 套件 | 用途 | 版本/備註 |
|------|------|-----------|
| nodejs | JavaScript 運行環境 | 最新 LTS 版本 |
| npm | Node 套件管理器 | 最新版本 |

### 額外的系統套件
| 套件 | 用途 | 版本/備註 |
|------|------|-----------|
| gnupg2 | GPG 加密工具 | 套件簽名驗證 |
| apt-transport-https | HTTPS APT 支援 | 安全套件源 |
| ca-certificates | CA 憑證 | SSL/TLS 驗證 |
| libldap2 | LDAP 函式庫 | 目錄服務支援 |

## 📊 套件統計摘要

### AGV 環境
- **系統套件總數**: 約 40 個
- **Python 套件總數**: 15 個
- **ROS 2 套件**: 基礎 + joy-linux
- **特色**: GPIO 支援、搖桿控制

### AGVC 環境
- **系統套件總數**: 約 45 個（含 Node.js 相關）
- **Python 套件總數**: 15 個（與 AGV 相同）
- **ROS 2 套件**: 基礎套件
- **特色**: Node.js 支援、Web 開發環境

## 🚫 已註解但未安裝的套件

### AGVC 環境中註解掉的套件
1. **Microsoft SQL Server 2022**
   - mssql-server
   - mssql-tools18
   - msodbcsql18
   - 原因: 可能改用 PostgreSQL

2. **Mosquitto MQTT Broker**
   - mosquitto
   - mosquitto-clients
   - 原因: 可能改用其他通訊方式

## 📦 套件版本管理策略

### 版本鎖定
- **基礎映像**: 使用特定的 ROS 2 版本標籤
- **Python 套件**: 未鎖定版本（使用最新版）
- **系統套件**: 使用 Ubuntu 倉庫的穩定版本

### 建議改進
1. **Python 套件版本鎖定**: 建議使用 requirements.txt 鎖定版本
2. **安全更新**: 定期更新基礎映像和套件
3. **最小化映像**: 考慮使用多階段建置減少映像大小

## 🔍 套件相容性注意事項

### Python 3.12 相容性
- 所有 Python 套件都與 Python 3.12 相容
- 虛擬環境路徑: `/opt/pyvenv_env/lib/python3.12/site-packages`

### ROS 2 Jazzy 相容性
- 使用 rmw_zenoh_cpp 作為 RMW 實作
- 所有 ROS 2 套件都是 Jazzy 版本

### 網路工具重疊
- 同時安裝了經典工具 (net-tools) 和現代工具 (iproute2)
- 建議優先使用現代工具：`ip` 替代 `ifconfig`，`ss` 替代 `netstat`

## 🛠️ 開發建議

### 套件使用優先級
1. **搜尋**: 使用 `rg` (ripgrep) 替代 `grep`
2. **網路**: 使用 `ss` 替代 `netstat`
3. **JSON 處理**: 使用 `jq` 處理 JSON 檔案
4. **YAML 處理**: 使用 `yq` 處理 YAML 檔案
5. **JSON5 處理**: 需要安裝 `json5` CLI 工具（見下方）

### 缺失但建議安裝的工具

#### json5 CLI 工具
用於處理 `/app/routerconfig.json5` 配置檔案：
```bash
# 在 AGVC 容器中安裝 (全域安裝)
npm install -g json5

# 使用範例
json5 /app/routerconfig.json5 | jq '.mode'
json5 --validate /app/routerconfig.json5
```

### 環境隔離
- Python 套件都安裝在虛擬環境中
- 避免污染系統 Python 環境
- 便於套件版本管理

## 🔗 交叉引用
- Docker Compose 配置: @docs-ai/operations/deployment/docker-compose-configuration.md
- 技術棧: @docs-ai/context/system/technology-stack.md
- 雙環境架構: @docs-ai/context/system/dual-environment.md
- 容器管理: @docs-ai/operations/deployment/container-management.md