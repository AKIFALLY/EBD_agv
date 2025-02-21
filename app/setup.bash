#!/bin/bash
# 如果是非交互式 shell，則跳過
[[ $- != *i* ]] && return

# 設定要載入的 workspace 路徑
workspaces=(
    "/app/keyence_plc_ws/install"
    "/app/ecs_ws/install"
    "/app/web_api_ws/install"
)

# 逐一檢查並 source
for ws in "${workspaces[@]}"; do
    if [ -d "$ws" ]; then
        echo "Sourcing $ws/setup.bash"
        source "$ws/setup.bash"
    else
        echo "Warning: $ws 不存在，略過"
    fi
done
