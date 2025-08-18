# 節點管理系統狀態顯示修復案例

## 🎯 適用場景
- ROS 2 節點狀態顯示不正確的問題診斷
- Web API 在容器內執行命令的正確方式
- 節點管理界面顯示與實際狀態不符的解決方案

## 📋 問題描述

### 症狀
- 節點管理頁面 (http://agvc.ui/nodes/) 只顯示 1 個節點為 "running"
- 實際上多個節點正在運行（通過 `ros2 node list` 確認）
- 大部分節點顯示為 "unknown" 或 "stopped" 狀態

### 影響
- 用戶無法正確了解系統運行狀態
- 可能誤以為系統故障而進行不必要的重啟操作
- 管理界面失去監控價值

## 🔍 根本原因分析

### 問題 1：API 超時
- **原因**：原始代碼使用 30 秒超時檢查每個節點狀態
- **影響**：導致 API 響應緩慢，瀏覽器可能超時
- **解決**：將超時時間降低到 5 秒

### 問題 2：容器內命令執行錯誤
- **錯誤代碼**：
```python
# 錯誤：嘗試在容器內載入環境
cmd = "bash -c 'source /app/setup.bash >/dev/null 2>&1 && agvc_source >/dev/null 2>&1 && ros2 node list 2>/dev/null'"
```
- **原因**：API 已經在容器內運行，環境已載入，重複載入導致命令執行失敗
- **正確代碼**：
```python
# 正確：直接執行 ros2 命令
cmd = "ros2 node list 2>/dev/null"
```

### 問題 3：節點名稱映射
- **需求**：ROS 2 節點名稱與註冊表名稱不一致
- **解決**：建立節點名稱映射表
```python
node_mapping = {
    "flow_wcs": "flow_wcs_node",
    "ecs_core": "ecs_core",
    "rcs_core": "rcs_core",
    "db_proxy": "db_proxy_node",
    "kuka_fleet": "kuka_adapter_demo_node",
    "plc_proxy_agvc": "agvc/plc_service"
}
```

## 💡 解決方案

### 完整的優化代碼
**檔案**: `/app/web_api_ws/src/web_api/web_api/routers/nodes.py`

```python
@router.get("/status")
async def get_all_status():
    """獲取所有節點狀態 - 優化版本，快速檢查實際狀態"""
    status_list = []
    
    # 快速獲取所有運行中的 ROS 2 節點
    running_nodes = set()
    try:
        # 直接在容器內執行 ros2 node list（API 本身就在容器內運行）
        cmd = "ros2 node list 2>/dev/null"
        result = await NodeManager.run_command(cmd, timeout=3)
        if result["success"]:
            # 解析輸出的節點列表
            lines = result["stdout"].strip().split('\n')
            for line in lines:
                if line and not line.startswith("WARNING"):
                    # 提取節點名稱，去掉前綴 /
                    node = line.strip().lstrip('/')
                    running_nodes.add(node)
            logger.info(f"Running nodes detected: {running_nodes}")
        else:
            logger.warning(f"Failed to get node list: {result.get('error', 'Unknown error')}")
    except Exception as e:
        logger.error(f"Failed to get running nodes: {e}")
    
    # 獲取本地節點狀態
    for node_name, node_info in node_registry.get("nodes", {}).items():
        status = {
            "name": node_name,
            "type": node_info.get("type"),
            "description": node_info.get("description"),
            "status": "unknown",
            "running": False,
            "details": {}
        }
        
        # 根據節點類型檢查狀態
        if node_info.get("type") == "launch":
            # Launch 類型節點特殊處理
            if node_name == "web_api_launch":
                # web_api_launch 包含多個子節點
                if "agvc/web_api_server" in running_nodes or "agv_ui_server_node" in running_nodes:
                    status["status"] = "running"
                    status["running"] = True
            elif node_name == "ecs_launch":
                if "ecs_core" in running_nodes:
                    status["status"] = "running"
                    status["running"] = True
        else:
            # 單一節點類型
            node_key = node_info.get("node_name", node_name)
            
            # 特殊節點名稱映射
            node_mapping = {
                "flow_wcs": "flow_wcs_node",
                "ecs_core": "ecs_core",
                "rcs_core": "rcs_core",
                "db_proxy": "db_proxy_node",
                "kuka_fleet": "kuka_adapter_demo_node",
                "plc_proxy_agvc": "agvc/plc_service"
            }
            
            check_name = node_mapping.get(node_name, node_key)
            
            # 檢查節點是否在運行列表中
            if check_name in running_nodes:
                status["status"] = "running"
                status["running"] = True
            elif any(check_name in node for node in running_nodes):
                # 部分匹配檢查
                status["status"] = "running"
                status["running"] = True
            else:
                status["status"] = "stopped"
        
        status_list.append(status)
    
    return {
        "timestamp": datetime.now().isoformat(),
        "nodes": status_list,
        "agvs": agv_status_list
    }
```

## 🔧 實施步驟

### 1. 修改源代碼
```bash
# 編輯節點管理 API
vim /app/web_api_ws/src/web_api/web_api/routers/nodes.py
```

### 2. 重建套件
```bash
cd /app/web_api_ws
colcon build --packages-select web_api
```

### 3. 重啟服務
```bash
# 使用管理函數重啟
manage_web_api_launch stop
sleep 2
manage_web_api_launch start
```

### 4. 驗證修復
```bash
# 檢查 API 返回
curl http://localhost:8000/api/nodes/status | python3 -m json.tool

# 檢查實際運行的節點
ros2 node list
```

## 📊 修復前後對比

### 修復前
- 顯示狀態：1 個 running, 6 個 stopped/unknown
- 實際狀態：7 個節點正在運行
- 問題：狀態檢測失敗

### 修復後
- 顯示狀態：6 個 running, 1 個 stopped
- 實際狀態：6 個節點正在運行，1 個未運行（db_proxy）
- 結果：狀態顯示準確

## 🚨 關鍵經驗教訓

### 1. 容器內命令執行
- **錯誤觀念**：在容器內需要重新載入環境
- **正確理解**：API 進程已在容器內運行，環境已載入
- **最佳實踐**：直接執行命令，避免不必要的環境載入

### 2. 調試技巧
- 使用獨立測試腳本驗證邏輯
- 添加日誌輸出追蹤執行流程
- 分離測試命令執行和業務邏輯

### 3. 節點名稱處理
- ROS 2 節點名稱可能包含命名空間（如 `/agvc/web_api_server`）
- 需要正確處理斜線前綴
- 建立明確的名稱映射關係

## 🔍 故障排除檢查清單

### 節點狀態顯示問題診斷
1. **檢查實際運行節點**
   ```bash
   ros2 node list
   ```

2. **測試 API 端點**
   ```bash
   curl http://localhost:8000/api/nodes/status
   ```

3. **檢查命令執行**
   ```bash
   # 在容器內測試命令
   ros2 node list 2>/dev/null
   ```

4. **驗證節點名稱映射**
   - 確認節點註冊表中的名稱
   - 確認 ROS 2 實際節點名稱
   - 更新映射表

5. **檢查日誌錯誤**
   ```bash
   docker compose logs agvc_server | grep -i error
   ```

## 💡 預防措施

### 設計原則
1. **簡化命令執行**：避免複雜的環境載入鏈
2. **快速失敗**：設置合理的超時時間
3. **緩存策略**：考慮緩存節點狀態以減少檢查頻率
4. **異步處理**：使用異步方式並行檢查多個節點

### 監控建議
1. 定期檢查節點狀態 API 的響應時間
2. 監控 `ros2 node list` 命令的執行時間
3. 設置節點狀態不一致的告警

## 🔗 相關參考
- ROS 2 容器開發: @docs-ai/operations/development/docker-development.md
- Web API 開發: @docs-ai/operations/development/web-development.md
- 系統診斷: @docs-ai/operations/maintenance/system-diagnostics.md
- 故障排除: @docs-ai/operations/maintenance/troubleshooting.md