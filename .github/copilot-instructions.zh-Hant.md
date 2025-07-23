# RosAGV AI 編碼助理指南

## 1. 宏觀架構

- **雙環境系統：** 專案分為兩個 Docker 環境：
    1.  **AGV 車載系統（`rosagv` 容器）：** 用於即時 AGV 控制。由 `docker-compose.yml` 管理。
    2.  **AGVC 管理系統（`agvc_server` 容器）：** 用於車隊管理、資料庫和 Web UI。由 `docker-compose.agvc.yml` 管理。
- **ROS 2 搭配 Zenoh：** 我們使用 ROS 2 Jazzy 與 `rmw_zenoh_cpp` 進行通訊。Zenoh 路由器的設定檔位於 `app/routerconfig.json5`。
- **Python 虛擬環境：** 在 `/opt/pyvenv_env` 有一個獨立的 Python 虛擬環境，用於非 ROS 套件，如 FastAPI 和 SQLAlchemy。

## 2. 關鍵開發工作流程

- **環境設定：** 在容器內，務必先執行環境腳本：`source /app/setup.bash`。此腳本提供必要的別名和函式。
- **建置工作空間：** 使用 `colcon build` 來編譯工作空間。別名 `build_all` 可以建置所有項目。
- **執行測試：** 使用 `colcon test` 來執行測試。別名 `test_all` 也可用。
- **Python 使用方式：**
    - 永遠使用 `python3` 和 `pip3`。
    - 對於虛擬環境中的套件（例如 `fastapi`），請使用 `/opt/pyvenv_env/bin/python3` 和 `/opt/pyvenv_env/bin/pip3`。

## 3. 專案特定慣例

- **README 是必讀文件：** 在編輯任何工作空間之前，你 **必須** 閱讀其 `README.md`。它包含關於依賴、啟動狀態和功能的關鍵資訊。
- **文件優先：** 任何會改變功能、依賴或使用方式的程式碼變更，都必須在相關的 `README.md` 檔案中同步更新。
- **狀態圖示：** 注意 README 中的狀態圖示（例如 ✅ 表示啟用，⚠️ 表示手動，❌ 表示已棄用），以了解工作空間或功能的當前狀態。

## 4. 關鍵整合點

- **PLC 通訊：** `plc_proxy_ws` 和 `keyence_plc_ws` 是與 PLC 硬體溝通的閘道。
- **資料庫存取：** 所有資料庫操作都透過 `db_proxy_ws` 提供的 ROS 2 服務處理。不要從其他節點直接連線資料庫。
- **Web API：** `web_api_ws` 提供一個基於 FastAPI 的 REST API。這是外部系統和前端應用的主要入口點。詳見下方說明。
- **KUKA 車隊整合：** `kuka_fleet_ws` 和 `wcs_ws` 負責與 KUKA 車隊管理系統通訊。

### Web API (`web_api_ws`) 詳細說明

- **技術棧：** 此工作空間使用來自 `/opt/pyvenv_env` 虛擬環境的 **FastAPI**、**Uvicorn** 和 **Pydantic**。
- **模組化結構：** 它包含三個獨立的服務：
    - `web_api`：核心的 RESTful API 後端。
    - `agvcui`：功能完整的 AGV 管理介面。
    - `opui`：簡化的操作員介面。
- **基於路由器的架構：** `web_api` 服務採用路由器模式。關鍵業務邏輯被封裝在 `src/web_api/web_api/routers/` 中。
    - **範例：** `routers/door.py` 透過從 `ecs_ws` 工作空間匯入並使用 `DoorLogic` 來處理門控。這取代了舊有的基於 MQTT 的系統。
    - 其他路由器則與 `plc_proxy_ws` (`plc.py`) 和 `rcs_ws` (`traffic.py`) 整合。
- **資料模型：** API 的請求與回應模型是使用 Pydantic 的 `BaseModel` 來定義的。
- **前端：** `agvcui` 和 `opui` 前端是使用 **Jinja2** 模板建立的，並從其各自的 `static/` 目錄提供靜態資源（JS/CSS）。它們使用 **Socket.IO** 來進行即時更新。

## 5. 如何處理任務

1.  **識別環境：** 判斷任務是與 AGV 車載系統還是 AGVC 管理系統相關。
2.  **查閱 README：** 找到相關的工作空間並仔細閱讀其 `README.md` 檔案。
3.  **檢查依賴：** 注意工作空間是使用系統的 ROS 2 套件還是 Python 虛擬環境中的套件。
4.  **實作變更：** 遵循現有的風格和慣例進行程式碼變更。
5.  **更新文件：** 更新 `README.md` 以反映你的變更。這是必要步驟。
6.  **建置與測試：** 使用 `colcon build` 和 `colcon test` 來驗證你的變更。