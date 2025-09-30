#!/bin/bash

# 生成基於實際文件系統的樹狀結構
# 用於更新 docs-ai/README.md 中的文件列表

echo "生成 docs-ai 目錄的樹狀結構..."

# 函數：將路徑轉換為樹狀結構
generate_tree() {
    local base_dir="$1"
    local prefix="$2"

    # 找出所有文件並排序
    local files=$(find "$base_dir" -name "*.md" -o -type d | sort)

    # 用於追蹤目錄層級
    declare -A dir_level
    declare -A dir_printed

    while IFS= read -r path; do
        # 跳過基礎目錄本身
        if [ "$path" == "$base_dir" ]; then
            continue
        fi

        # 獲取相對路徑
        local rel_path="${path#$base_dir/}"

        # 計算層級
        local level=$(echo "$rel_path" | tr -cd '/' | wc -c)

        # 如果是文件
        if [[ "$path" == *.md ]]; then
            local dir_name=$(dirname "$rel_path")
            local file_name=$(basename "$rel_path")

            # 打印目錄結構（如果還沒打印過）
            if [ "$dir_name" != "." ] && [ -z "${dir_printed[$dir_name]}" ]; then
                local dir_parts=($(echo "$dir_name" | tr '/' ' '))
                local current_path=""

                for i in "${!dir_parts[@]}"; do
                    if [ -z "$current_path" ]; then
                        current_path="${dir_parts[$i]}"
                    else
                        current_path="$current_path/${dir_parts[$i]}"
                    fi

                    if [ -z "${dir_printed[$current_path]}" ]; then
                        local indent=""
                        for ((j=0; j<$((i+1)); j++)); do
                            if [ $j -eq $i ]; then
                                indent="${indent}├── "
                            else
                                indent="${indent}│   "
                            fi
                        done

                        # 添加註釋
                        local comment=""
                        case "${dir_parts[$i]}" in
                            "context") comment=" # AI Agent 背景知識庫";;
                            "system") comment=" # 系統層級文檔";;
                            "workspaces") comment=" # 工作空間文檔";;
                            "structure") comment=" # 結構化文檔";;
                            "knowledge") comment=" # 領域專業知識";;
                            "agv-domain") comment=" # AGV 領域知識";;
                            "business") comment=" # 業務流程知識";;
                            "protocols") comment=" # 通訊協定知識";;
                            "operations") comment=" # AI Agent 操作指導";;
                            "development") comment=" # 開發操作指導";;
                            "deployment") comment=" # 部署操作指導";;
                            "guides") comment=" # 操作指南";;
                            "tools") comment=" # 工具系統";;
                            "core") comment=" # 核心開發原則";;
                            "ros2") comment=" # ROS 2 開發指導";;
                            "testing") comment=" # 測試程序文檔";;
                            "web") comment=" # Web 開發指導";;
                            "tafl") comment=" # TAFL 語言相關";;
                        esac

                        echo "${indent}${dir_parts[$i]}/${comment}"
                        dir_printed[$current_path]=1
                    fi
                done
            fi

            # 打印文件
            local indent=""
            for ((i=0; i<=level; i++)); do
                if [ $i -eq $level ]; then
                    # 檢查是否是目錄中的最後一個文件
                    local next_file=$(find "$base_dir/$(dirname "$rel_path")" -maxdepth 1 -name "*.md" | sort | tail -1)
                    if [ "$path" == "$next_file" ]; then
                        indent="${indent}└── "
                    else
                        indent="${indent}├── "
                    fi
                else
                    indent="${indent}│   "
                fi
            done

            # 添加文件描述
            local desc=""
            case "$file_name" in
                "rosagv-overview.md") desc=" # RosAGV 專案整體概覽";;
                "dual-environment.md") desc=" # 雙環境架構詳解";;
                "technology-stack.md") desc=" # 技術棧和依賴關係";;
                "agv-workspaces.md") desc=" # AGV 車載工作空間概覽";;
                "agvc-workspaces.md") desc=" # AGVC 管理工作空間概覽";;
                "vehicle-types.md") desc=" # 車型特性和應用場景";;
                "agv-state-machine.md") desc=" # AGV 狀態機設計";;
                "eyewear-production-process.md") desc=" # 眼鏡生產業務流程";;
                "zenoh-rmw.md") desc=" # Zenoh RMW 通訊機制";;
                "ros2-interfaces.md") desc=" # ROS 2 介面設計";;
                "unified-tools.md") desc=" # 統一工具系統";;
            esac

            echo "${indent}${file_name}${desc}"
        fi
    done <<< "$files"
}

# Context 系列
echo -e "\n### 📚 Context 文件系列 - AI Agent 背景知識庫"
echo '```'
echo "docs-ai/context/"
generate_tree "docs-ai/context" ""
echo '```'

# Operations 系列
echo -e "\n### 🔧 Operations 文件系列 - AI Agent 操作指導"
echo '```'
echo "docs-ai/operations/"
generate_tree "docs-ai/operations" ""
echo '```'

# Knowledge 系列
echo -e "\n### 🧠 Knowledge 文件系列 - 領域專業知識"
echo '```'
echo "docs-ai/knowledge/"
generate_tree "docs-ai/knowledge" ""
echo '```'

echo -e "\n✅ 樹狀結構生成完成！"