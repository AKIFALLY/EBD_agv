#!/bin/bash
# 啟動 Room Task Build Node
#
# 使用方式：
#   ./start_room_task_build.sh          # 只啟動 room_task_build (推薦)
#   ./start_room_task_build.sh full     # 啟動所有三個節點

cd ~/EBD_agv

MODE=${1:-standalone}

if [ "$MODE" = "full" ]; then
    echo "🚀 啟動完整 WCS 系統 (PLC + ECS + Room Task Build)..."
    docker compose -f docker-compose.agvc.yml exec agvc_server bash -c "
    source /app/setup.bash &&
    agvc_source &&
    source /app/wcs_ws/install/setup.bash &&
    ros2 launch alan_room_task_build wcs_system.launch.py
    "
else
    echo "🚀 啟動 Room Task Build Node (使用現有的 PLC 和 ECS 服務)..."
    docker compose -f docker-compose.agvc.yml exec agvc_server bash -c "
    source /app/setup.bash &&
    agvc_source &&
    source /app/wcs_ws/install/setup.bash &&
    ros2 launch alan_room_task_build wcs_system.launch.py \
      use_ecs:=false \
      use_plc:=false
    "
fi
