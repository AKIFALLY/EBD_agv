#!/bin/bash
# 启动 Room Task Build Node
#
# 使用方式：
#   ./start_room_task_build.sh          # 只启动 room_task_build (推荐)
#   ./start_room_task_build.sh full     # 启动所有三个节点

cd ~/RosAGV

MODE=${1:-standalone}

if [ "$MODE" = "full" ]; then
    echo "🚀 启动完整 WCS 系统 (PLC + ECS + Room Task Build)..."
    docker compose -f docker-compose.agvc.yml exec agvc_server bash -c "
    source /app/setup.bash &&
    agvc_source &&
    source /app/wcs_ws/install/setup.bash &&
    ros2 launch alan_room_task_build wcs_system.launch.py
    "
else
    echo "🚀 启动 Room Task Build Node (使用现有的 PLC 和 ECS 服务)..."
    docker compose -f docker-compose.agvc.yml exec agvc_server bash -c "
    source /app/setup.bash &&
    agvc_source &&
    source /app/wcs_ws/install/setup.bash &&
    ros2 launch alan_room_task_build wcs_system.launch.py \
      use_ecs:=false \
      use_plc:=false
    "
fi
