#!/bin/bash
# RosAGV 專案環境設定腳本 (模組化版本)
# 支援互動式和非互動式 shell

# ============================================================================
# 模組化架構說明
# ============================================================================
# 本腳本已重構為模組化架構，各功能分散在獨立模組中：
#   - common.bash                : 顏色定義、日誌系統、基礎工具函數
#   - build-tools.bash           : 建置、清理相關函數
#   - workspace-loader.bash      : 工作空間載入、檔案同步、幫助文檔
#   - node-management.bash       : 節點管理、服務控制函數
#   - system-checks.bash         : 系統狀態檢查和診斷函數
#   - device-identity.bash       : 設備識別和配置生成函數
#   - remote-agv-management.bash : 遠程 AGV Launch 管理函數
#
# 修改日期: 2025-10-30
# 版本: 2.2.0 (新增 AGV 本地 Launch 管理功能)
# ============================================================================

# 啟用別名展開（對於 source 的腳本很重要）
shopt -s expand_aliases

# ============================================================================
# 載入所有模組 (按依賴順序)
# ============================================================================

MODULES_DIR="/app/setup_modules"

# 1. 載入 common.bash (最優先，提供基礎函數)
if [ -f "$MODULES_DIR/common.bash" ]; then
    source "$MODULES_DIR/common.bash"
    log_debug "✅ 載入 common 模組"
else
    echo "❌ 錯誤: common.bash 模組不存在"
    return 1
fi

# 2. 載入 build-tools.bash (建置工具)
if [ -f "$MODULES_DIR/build-tools.bash" ]; then
    source "$MODULES_DIR/build-tools.bash"
    log_debug "✅ 載入 build-tools 模組"
else
    log_error "build-tools.bash 模組不存在"
fi

# 3. 載入 workspace-loader.bash (工作空間載入)
if [ -f "$MODULES_DIR/workspace-loader.bash" ]; then
    source "$MODULES_DIR/workspace-loader.bash"
    log_debug "✅ 載入 workspace-loader 模組"
else
    log_error "workspace-loader.bash 模組不存在"
fi

# 4. 載入 node-management.bash (節點管理)
if [ -f "$MODULES_DIR/node-management.bash" ]; then
    source "$MODULES_DIR/node-management.bash"
    log_debug "✅ 載入 node-management 模組"
else
    log_error "node-management.bash 模組不存在"
fi

# 5. 載入 system-checks.bash (系統檢查)
if [ -f "$MODULES_DIR/system-checks.bash" ]; then
    source "$MODULES_DIR/system-checks.bash"
    log_debug "✅ 載入 system-checks 模組"
else
    log_error "system-checks.bash 模組不存在"
fi

# 6. 載入 device-identity.bash (設備識別)
if [ -f "$MODULES_DIR/device-identity.bash" ]; then
    source "$MODULES_DIR/device-identity.bash"
    log_debug "✅ 載入 device-identity 模組"
else
    log_error "device-identity.bash 模組不存在"
fi

# 7. 載入 remote-agv-management.bash (遠程 AGV 管理)
if [ -f "$MODULES_DIR/remote-agv-management.bash" ]; then
    source "$MODULES_DIR/remote-agv-management.bash"
    log_debug "✅ 載入 remote-agv-management 模組"
else
    log_error "remote-agv-management.bash 模組不存在"
fi

# ============================================================================
# ROS 2 環境載入
# ============================================================================

# 載入 ROS 2 基礎環境
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
    echo "✅ 載入 ROS 2 Jazzy 環境"
else
    echo "❌ ROS 2 Jazzy 環境不存在"
fi

# 載入 rmw_zenoh 環境
if [ -f "/opt/ws_rmw_zenoh/install/setup.bash" ]; then
    source /opt/ws_rmw_zenoh/install/setup.bash
    echo "✅ 載入 rmw_zenoh 環境"
else
    echo "❌ rmw_zenoh 環境不存在"
fi

# 設定虛擬環境 PYTHONPATH
if [ -d "/opt/pyvenv_env/lib/python3.12/site-packages" ]; then
    export PYTHONPATH="/opt/pyvenv_env/lib/python3.12/site-packages:$PYTHONPATH"
    echo "✅ 設定虛擬環境 PYTHONPATH"
else
    echo "⚠️ 虛擬環境路徑不存在"
fi

# 設定 Zenoh 相關環境變數
export ZENOH_ROUTER_CONFIG_URI="/app/routerconfig.json5"
export RMW_IMPLEMENTATION="rmw_zenoh_cpp"

# 確認環境變數設定
echo "✅ 設定 ZENOH_ROUTER_CONFIG_URI=$ZENOH_ROUTER_CONFIG_URI"
echo "✅ 設定 RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"

# ============================================================================
# 便捷別名定義
# ============================================================================

# 為常用指令創建別名
alias ll='ls -alF'
alias la='ls -A'
alias l='ls -CF'
alias ..='cd ..'
alias ...='cd ../..'

# ROS 2 相關別名
alias rn='ros2 node list'
alias rt='ros2 topic list'
alias rs='ros2 service list'
alias ri='ros2 interface list'

# 工作空間管理別名
alias ba='build_all'
alias bas='build_all_smart'    # 依賴解析建置 (使用 colcon 依賴解析)
alias ta='test_all'
alias ca='clean_all'
alias sa='all_source'
alias load_all='all_source'  # 簡化的載入指令

# 專用工作空間載入別名
alias agv='agv_source'       # AGV 專用工作空間載入
alias agvc='agvc_source'     # AGVC 專用工作空間載入

# 單一工作空間操作別名
alias build1='build_single'
alias test1='test_single'
alias clean1='clean_single'

# 幫助和狀態別名
alias help='show_help'
alias status='check_system_status'

# 狀態檢查別名
alias status='check_system_status'
alias zenoh='check_zenoh_status'
alias rosenv='check_ros_env'

# 容器特定的智能別名 (根據環境自動選擇命令)
if is_agvc_environment; then
    alias check_status='manage_all_nodes status'
else
    alias check_status='check_agv_status'
fi

# 臨時文件清理別名 (定義在 node-management.bash 中)
alias ct='cleanup_temp_files'
alias cleanup_temp='cleanup_temp_files'

# 別名載入提示 (僅在互動式 shell 中顯示)
if [[ $- == *i* ]]; then
    log_debug "別名已載入: status, zenoh, rosenv, help, build1, test1, clean1, agv, agvc, ct"
fi

# ============================================================================
# 初始化完成訊息
# ============================================================================

# 檢測環境並顯示對應訊息
if is_agvc_environment; then
    echo "🖥️ RosAGV AGVC 管理系統環境已載入"
    echo "📊 資料庫支援：PostgreSQL"
else
    echo "🚗 RosAGV AGV 車載系統環境已載入"
fi

echo "🔧 通用指令："
echo "  build_all/ba         - 自動建置工作空間 (根據容器類型自動選擇)"
echo "  build_agv            - 建置 AGV 車載系統專用工作空間"
echo "  build_agvc           - 建置 AGVC 管理系統專用工作空間"
echo "  build_all_workspaces - 建置所有工作空間 (傳統方式)"
echo "  build_all_smart/bas  - 依賴解析建置 (使用 colcon 依賴解析)"
echo "  build_ws <name>      - 建置指定工作空間"
echo "  test_all/ta          - 測試所有工作空間"
echo "  test_ws <name>       - 測試指定工作空間"
echo "  clean_all/ca         - 清理所有工作空間"
echo "  clean_ws <name>      - 清理指定工作空間"
echo "  all_source/sa        - 自動載入工作空間 (根據環境自動選擇)"
echo "  agv_source           - 載入 AGV 車載系統專用工作空間"
echo "  agvc_source          - 載入 AGVC 管理系統專用工作空間"
echo "  check_system_status/status - 檢查系統狀態"
echo "  check_zenoh_status/zenoh   - 檢查 Zenoh 狀態"
echo "  check_ros_env/rosenv       - 檢查 ROS 2 環境"
echo "  manage_zenoh <cmd>                - 管理 Zenoh Router"
echo "  manage_agvc_database_node <cmd>   - 管理 AGVC 資料庫節點"
echo "  manage_web_api_launch <cmd>       - 管理 Web API Launch"
echo "  manage_tafl_wcs <cmd>             - 管理 TAFL WCS 節點"  # 新一代 WCS 系統
echo "  manage_rcs_core <cmd>             - 管理 RCS 核心節點"
echo "  manage_room_task_build <cmd>      - 管理 WCS 房間任務節點"
echo "  manage_ssh <cmd>                  - 管理 SSH 服務"
echo ""
echo "🚗 AGV 本地管理指令："
echo "  manage_agv_launch <cmd>     - 管理 AGV 本地 Launch 服務（不通過 SSH）"
echo "  local_agv <cmd>                   - manage_agv_launch 的簡化別名"
echo "  lagv <cmd>                        - manage_agv_launch 的簡化別名"
echo ""
echo "🔧 統一設備身份管理指令："
echo "  check_device_identity    - 檢查設備身份資訊"
echo "  identify_device_manual   - 手動觸發設備身份識別"
echo "  show_device_config       - 顯示設備配置資訊"
echo "  show_device_mac_info     - 顯示設備 MAC 地址資訊和管理建議"
echo "  mac_info                 - show_device_mac_info 的簡化別名"

log_success "✅ RosAGV 模組化環境設定完成"
