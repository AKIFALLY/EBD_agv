#!/bin/bash
# RosAGV Build Tools Module
# 包含所有建置、清理相關的函數

# ============================================================================
# AGV/AGVC 工作空間建置函數
# ============================================================================

# AGV 專用工作空間建置函數
build_agv() {
    echo "🚗 開始建置 AGV 車載系統專用工作空間..."

    BASE_DIR="/app/"
    local success_count=0
    local total_count=0

    # AGV 車載系統專用工作空間 (按依賴順序排列)
    local agv_base_workspaces=(
        "shared_constants_ws"    # 共享常數 (最優先)
        "keyence_plc_ws"
        "plc_proxy_ws"
        "path_algorithm"
        "db_proxy_ws"
    )

    local agv_app_workspaces=(
        "agv_cmd_service_ws"
        "joystick_ws"
        "agv_ws"
        "sensorpart_ws"
        "uno_gpio_ws"
        "launch_ws"
    )

    # 建置 AGV 基礎工作空間
    echo "📦 建置 AGV 基礎工作空間..."
    for ws_name in "${agv_base_workspaces[@]}"; do
        local workspace_path="$BASE_DIR$ws_name"

        if [ -d "$workspace_path" ]; then
            ((total_count++))
            echo "🔨 建置工作空間: $ws_name (AGV 基礎)"

            cd "$workspace_path" || continue

            if colcon build --event-handlers console_direct+; then
                echo "✅ $ws_name 建置成功"
                ((success_count++))
            else
                echo "❌ $ws_name 建置失敗"
            fi

            cd "$BASE_DIR" || continue
        else
            echo "⚠️  工作空間不存在: $ws_name"
        fi
    done

    # 建置 AGV 應用工作空間
    echo "🚀 建置 AGV 應用工作空間..."
    for ws_name in "${agv_app_workspaces[@]}"; do
        local workspace_path="$BASE_DIR$ws_name"

        if [ -d "$workspace_path" ]; then
            ((total_count++))
            echo "🔨 建置工作空間: $ws_name (AGV 應用)"

            cd "$workspace_path" || continue

            if colcon build --event-handlers console_direct+; then
                echo "✅ $ws_name 建置成功"
                ((success_count++))
            else
                echo "❌ $ws_name 建置失敗"
            fi

            cd "$BASE_DIR" || continue
        else
            echo "⚠️  工作空間不存在: $ws_name"
        fi
    done

    echo "📊 AGV 建置完成: $success_count/$total_count 個工作空間建置成功"
}

# AGVC 專用工作空間建置函數
build_agvc() {
    echo "🖥️ 開始建置 AGVC 管理系統專用工作空間..."

    BASE_DIR="/app/"
    local success_count=0
    local total_count=0

    # AGVC 管理系統專用工作空間 (按依賴順序排列)
    local agvc_base_workspaces=(
        "shared_constants_ws"    # 共享常數 (最優先)
        "keyence_plc_ws"
        "plc_proxy_ws"
        "path_algorithm"
        "agv_ws"
        "db_proxy_ws"
    )

    local agvc_app_workspaces=(
        "ecs_ws"
        "rcs_ws"
        "tafl_ws"      # TAFL parser and executor (新一代 WCS 基礎)
        "tafl_wcs_ws"  # TAFL WCS integration (目前使用的 WCS 實作)
        "web_api_ws"
        "kuka_fleet_ws"
        "launch_ws"
        "wcs_ws"
    )

    # 建置 AGVC 基礎工作空間
    echo "📦 建置 AGVC 基礎工作空間..."
    for ws_name in "${agvc_base_workspaces[@]}"; do
        local workspace_path="$BASE_DIR$ws_name"

        if [ -d "$workspace_path" ]; then
            ((total_count++))
            echo "🔨 建置工作空間: $ws_name (AGVC 基礎)"

            cd "$workspace_path" || continue

            if colcon build --event-handlers console_direct+; then
                echo "✅ $ws_name 建置成功"
                ((success_count++))
            else
                echo "❌ $ws_name 建置失敗"
            fi

            cd "$BASE_DIR" || continue
        else
            echo "⚠️  工作空間不存在: $ws_name"
        fi
    done

    # 建置 AGVC 應用工作空間
    echo "🚀 建置 AGVC 應用工作空間..."
    for ws_name in "${agvc_app_workspaces[@]}"; do
        local workspace_path="$BASE_DIR$ws_name"

        if [ -d "$workspace_path" ]; then
            ((total_count++))
            echo "🔨 建置工作空間: $ws_name (AGVC 應用)"

            cd "$workspace_path" || continue

            if colcon build --event-handlers console_direct+; then
                echo "✅ $ws_name 建置成功"
                ((success_count++))
            else
                echo "❌ $ws_name 建置失敗"
            fi

            cd "$BASE_DIR" || continue
        else
            echo "⚠️  工作空間不存在: $ws_name"
        fi
    done

    echo "📊 AGVC 建置完成: $success_count/$total_count 個工作空間建置成功"
}

# 自動建置函數 (根據環境自動選擇)
build_all() {
    echo "🔧 自動建置工作空間 (根據容器環境自動選擇)..."

    # 檢測當前環境並選擇對應的建置策略
    if [ "$CONTAINER_TYPE" = "agv" ]; then
        echo "🚗 檢測到 AGV 車載環境，建置 AGV 專用工作空間"
        build_agv
    elif [ "$CONTAINER_TYPE" = "agvc" ]; then
        echo "🖥️ 檢測到 AGVC 管理環境，建置 AGVC 專用工作空間"
        build_agvc
    elif is_agvc_environment; then
        echo "🖥️ 檢測到 AGVC 管理環境，建置 AGVC 專用工作空間"
        build_agvc
    else
        echo "🔄 無法確定環境類型，建置 AGV 工作空間 (預設)"
        build_agv
    fi
}

# 建置所有工作空間 (傳統方式，包含所有工作空間)
build_all_workspaces() {
    echo "🔧 開始建置所有工作空間..."

    BASE_DIR="/app/"
    local success_count=0
    local total_count=0

    # 定義依賴順序的工作空間列表
    local ordered_workspaces=(
        # 共享常數 (最優先，所有工作空間的基礎依賴)
        "shared_constants_ws"

        # 基礎依賴工作空間 (其他工作空間的依賴)
        "keyence_plc_ws"
        "plc_proxy_ws"
        "path_algorithm"

        # 核心服務工作空間
        "db_proxy_ws"          # 資料庫服務，被 tafl_wcs_ws 等依賴

        # AGV 相關工作空間
        "agv_ws"               # 核心 AGV 控制
        "agv_cmd_service_ws"   # 手動指令服務
        "joystick_ws"          # 搖桿控制
        "sensorpart_ws"        # 感測器處理

        # AGVC 應用工作空間 (依賴 db_proxy_ws)
        "ecs_ws"               # 設備控制系統
        "rcs_ws"               # 機器人控制系統
        "tafl_ws"              # TAFL parser and executor (新一代 WCS 基礎)
        "tafl_wcs_ws"          # TAFL WCS integration (目前使用的 WCS 實作)
        "web_api_ws"           # Web API 服務
        "kuka_fleet_ws"        # KUKA Fleet 整合

        # 啟動配置工作空間 (最後建置)
        "launch_ws"            # Launch 配置

        # 其他工作空間
        "uno_gpio_ws"          # GPIO 控制
    )

    # 按順序建置工作空間
    for ws_name in "${ordered_workspaces[@]}"; do
        local workspace_path="$BASE_DIR$ws_name"

        if [ -d "$workspace_path" ]; then
            ((total_count++))
            echo "🔨 建置工作空間: $ws_name (按依賴順序)"

            # 進入資料夾並執行 colcon build
            cd "$workspace_path" || continue

            if colcon build --event-handlers console_direct+; then
                echo "✅ $ws_name 建置成功"
                ((success_count++))
            else
                echo "❌ $ws_name 建置失敗"
                # 可選：是否在依賴失敗時停止建置
                # echo "⚠️  由於 $ws_name 建置失敗，可能影響後續依賴工作空間"
            fi

            cd "$BASE_DIR" || continue
        else
            echo "⚠️  工作空間不存在: $ws_name"
        fi
    done

    echo "📊 建置完成: $success_count/$total_count 個工作空間建置成功"
}

# 使用 colcon 依賴解析的建置函數
build_all_smart() {
    echo "🧠 開始依賴解析建置所有工作空間 (使用 colcon 依賴解析)..."

    BASE_DIR="/app/"

    # 收集所有工作空間的源碼目錄
    local workspace_src_dirs=()
    for dir in "$BASE_DIR"/*_ws; do
        if [ -d "$dir/src" ]; then
            workspace_src_dirs+=("$dir/src")
        fi
    done

    if [ ${#workspace_src_dirs[@]} -eq 0 ]; then
        echo "❌ 未找到任何有效的工作空間源碼目錄"
        return 1
    fi

    echo "📦 找到 ${#workspace_src_dirs[@]} 個工作空間源碼目錄"

    # 在根目錄創建臨時統一建置空間
    local unified_build_dir="/tmp/rosagv_unified_build"
    rm -rf "$unified_build_dir"
    mkdir -p "$unified_build_dir/src"

    # 將所有套件鏈接到統一建置空間
    echo "🔗 建立統一建置空間..."
    for src_dir in "${workspace_src_dirs[@]}"; do
        for package_dir in "$src_dir"/*; do
            if [ -d "$package_dir" ] && [ -f "$package_dir/package.xml" ]; then
                local package_name=$(basename "$package_dir")
                ln -sf "$package_dir" "$unified_build_dir/src/$package_name"
                echo "  🔗 鏈接套件: $package_name"
            fi
        done
    done

    # 切換到統一建置空間
    cd "$unified_build_dir" || return 1

    echo "🚀 執行統一建置 (colcon 將自動解析依賴關係)..."

    # 使用 colcon 的依賴解析功能進行建置
    if colcon build \
        --event-handlers console_direct+ \
        --executor sequential \
        --continue-on-error; then
        echo "✅ 統一建置成功"

        # 將建置結果複製回各個工作空間
        echo "📋 複製建置結果回各工作空間..."
        copy_build_results_back "$unified_build_dir" "$BASE_DIR"

    else
        echo "❌ 統一建置失敗"
        cd "$BASE_DIR"
        return 1
    fi

    # 清理臨時目錄
    cd "$BASE_DIR"
    rm -rf "$unified_build_dir"

    echo "🎉 依賴解析建置完成"
}

# 複製建置結果回各工作空間的輔助函數
copy_build_results_back() {
    local unified_dir="$1"
    local base_dir="$2"

    # 遍歷每個套件的建置結果
    for package_install in "$unified_dir/install"/*; do
        if [ -d "$package_install" ]; then
            local package_name=$(basename "$package_install")

            # 找到該套件原始所屬的工作空間
            for ws_dir in "$base_dir"/*_ws; do
                if [ -d "$ws_dir/src/$package_name" ]; then
                    echo "  📋 複製 $package_name 建置結果到 $(basename "$ws_dir")"

                    # 複製 install 目錄
                    cp -r "$package_install" "$ws_dir/install/"

                    # 複製 build 目錄 (如果存在)
                    if [ -d "$unified_dir/build/$package_name" ]; then
                        mkdir -p "$ws_dir/build"
                        cp -r "$unified_dir/build/$package_name" "$ws_dir/build/"
                    fi

                    break
                fi
            done
        fi
    done
}

# ============================================================================
# 單一工作空間建置函數
# ============================================================================

# 建置單一工作空間
build_ws() {
    local workspace_name="$1"

    if [ -z "$workspace_name" ]; then
        log_error "請提供工作空間名稱"
        echo "用法: build_ws/build_single <workspace_name>"
        echo "範例: build_single agv_ws"
        return 1
    fi

    local workspace_path="/app/$workspace_name"

    if [ ! -d "$workspace_path" ]; then
        log_error "工作空間不存在: $workspace_path"
        return 1
    fi

    log_info "建置工作空間: $workspace_name"
    cd "$workspace_path" || return 1

    if colcon build --event-handlers console_direct+; then
        log_success "$workspace_name 建置成功"

        # 自動載入環境
        if [ -f "install/setup.bash" ]; then
            source install/setup.bash
            log_success "環境已載入"
        fi

        return 0
    else
        log_error "$workspace_name 建置失敗"
        return 1
    fi
}

# 建置單一工作空間 (別名)
build_single() {
    build_ws "$@"
}

# 函式：執行 colcon build --symlink-install所有 _ws 資料夾
build_all_symlink_install() {
    BASE_DIR="/app/"

    # 查找所有 _ws 結尾的資料夾並執行 colcon build
    for dir in "$BASE_DIR"/*_ws; do
        if [ -d "$dir" ]; then
            echo "開始建置 $dir ..."
            # 進入資料夾並執行 colcon build
            cd "$dir" || continue
            colcon build --symlink-install
            if [ $? -eq 0 ]; then
                echo "$dir 建置成功！"
            else
                echo "$dir 建置失敗！"
            fi
            cd "$BASE_DIR" || continue
        fi
    done
}

# ============================================================================
# 清理函數
# ============================================================================

# 清理所有工作空間
clean_all() {
    echo "🧹 開始清理所有工作空間..."

    BASE_DIR="/app/"
    local success_count=0
    local total_count=0

    # 查找所有 _ws 結尾的資料夾並執行清理
    for dir in "$BASE_DIR"/*_ws; do
        if [ -d "$dir" ]; then
            ((total_count++))
            local ws_name=$(basename "$dir")
            echo "🧹 清理工作空間: $ws_name"

            # 進入資料夾並執行清理
            cd "$dir" || continue

            if rm -rf build/ install/ log/; then
                echo "✅ $ws_name 清理成功"
                ((success_count++))
            else
                echo "❌ $ws_name 清理失敗"
            fi

            cd "$BASE_DIR" || continue
        fi
    done

    echo "📊 清理完成: $success_count/$total_count 個工作空間清理成功"
}

# 清理單一工作空間
clean_ws() {
    local workspace_name="$1"

    if [ -z "$workspace_name" ]; then
        log_error "請提供工作空間名稱"
        echo "用法: clean_ws/clean_single <workspace_name>"
        echo "範例: clean_single agv_ws"
        return 1
    fi

    local workspace_path="/app/$workspace_name"

    if [ ! -d "$workspace_path" ]; then
        log_error "工作空間不存在: $workspace_path"
        return 1
    fi

    log_info "清理工作空間: $workspace_name"
    cd "$workspace_path" || return 1

    if rm -rf build/ install/ log/; then
        log_success "$workspace_name 清理成功"
        return 0
    else
        log_error "$workspace_name 清理失敗"
        return 1
    fi
}

# 清理單一工作空間 (別名)
clean_single() {
    clean_ws "$@"
}
