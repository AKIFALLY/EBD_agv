---
type: "manual"
---

**重要提醒：ROS2 專案建置和測試指引**

1. **禁止修改 build 資料夾**：
   - 絕對不要修改工作目錄中任何 `build/` 資料夾下的檔案
   - `build/` 資料夾包含自動生成的編譯產物，手動修改會在下次建置時被覆蓋
   - 只能修改 `src/` 目錄下的原始碼檔案

2. **測試前的必要建置步驟**：
   - 在測試任何程式碼修改之前，必須先執行建置命令
   - 建置命令格式：
   ```bash
   cd [workspace_name]_ws  # 例如：cd agv_ws 或 cd robot_ws
   colcon build --symlink-install
   ```
   - `--symlink-install` 參數的作用：
     - 建立符號連結而非複製檔案
     - Python 檔案修改後無需重新建置即可生效
     - C++ 檔案仍需重新建置

3. **ROS2 環境設定和測試**：
   - 建置完成後，使用 `all_source` 指令載入 ROS2 環境變數
   - 測試流程：
     ```bash
     cd [workspace_name]_ws
     colcon build --symlink-install
     all_source
     # 然後執行測試命令
     ```

4. **檔案修改和建置原則**：
   - **只能修改**：`[workspace_name]_ws/src/` 下的原始碼檔案
   - **禁止修改**：
     - `build/` 資料夾中的任何檔案
     - `install/` 資料夾中的任何檔案
     - `log/` 資料夾中的任何檔案
   - **修改後必須**：重新執行 `colcon build --symlink-install` 以確保變更生效
   - **工作目錄**：確保在正確的 `[workspace_name]_ws` 目錄中執行所有建置命令

5. **常見錯誤避免**：
   - 不要在錯誤的目錄執行建置命令
   - 不要跳過建置步驟直接測試
   - 不要忘記執行 `all_source` 載入環境
   - 修改 C++ 檔案後必須重新建置，Python 檔案則不需要（因為使用了 --symlink-install）