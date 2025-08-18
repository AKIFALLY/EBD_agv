#!/bin/bash
# 生成多個 AGV 的測試狀態檔案

echo "🚀 生成 6 個 AGV 的測試狀態檔案..."

# AGV 列表
AGVS=("loader01" "loader02" "cargo01" "cargo02" "unloader01" "unloader02")

# 生成每個 AGV 的狀態檔案
for i in "${!AGVS[@]}"; do
    agv_id="${AGVS[$i]}"
    
    # 生成隨機值
    x_pos=$(awk -v min=1000 -v max=5000 'BEGIN{srand(); print int(min+rand()*(max-min+1))}')
    y_pos=$(awk -v min=1000 -v max=5000 'BEGIN{srand(); print int(min+rand()*(max-min+1))}')
    power=$(awk -v min=20 -v max=100 'BEGIN{srand(); print int(min+rand()*(max-min+1))}')
    speed=$(awk -v min=0 -v max=150 'BEGIN{srand(); print min+rand()*(max-min)}')
    
    # 決定 AGV 狀態
    if [ $((i % 2)) -eq 0 ]; then
        auto_mode=1
        manual_mode=0
    else
        auto_mode=0
        manual_mode=1
    fi
    
    # 生成完整的 330+ 屬性狀態檔案
    cat > "/tmp/agv_status_${agv_id}.json" << EOF
{
  "AGV_ID": "${agv_id}",
  "MAGIC": 13243,
  "timestamp": "$(date -Iseconds)",
  "namespace": "${agv_id}",
  "AGV_Auto": ${auto_mode},
  "AGV_Manual": ${manual_mode},
  "AGV_MOVING": $((RANDOM % 2)),
  "AGV_TURN": $((RANDOM % 2)),
  "AGV_SLAM_X": ${x_pos},
  "AGV_SLAM_Y": ${y_pos},
  "AGV_SLAM_ANGLE": $(awk -v max=360 'BEGIN{srand(); print rand()*max}'),
  "AGV_PGV_X": ${x_pos},
  "AGV_PGV_Y": ${y_pos},
  "AGV_PGV_ANGLE": $(awk -v max=360 'BEGIN{srand(); print rand()*max}'),
  "POWER": ${power},
  "AGV_SPEED": ${speed},
  "AGV_TARGET_POINT": $((RANDOM % 100)),
  "AGV_CURRENT_ZONE": $((RANDOM % 10)),
EOF

    # 添加輸入狀態 (100個)
    for j in {1..100}; do
        echo "  \"Input_${j}\": $((RANDOM % 2))," >> "/tmp/agv_status_${agv_id}.json"
    done
    
    # 添加輸出狀態 (100個)
    for j in {1..100}; do
        echo "  \"Output_${j}\": $((RANDOM % 2))," >> "/tmp/agv_status_${agv_id}.json"
    done
    
    # 添加警報狀態 (50個)
    for j in {1..50}; do
        echo "  \"Alarm_${j}\": $((RANDOM % 2))," >> "/tmp/agv_status_${agv_id}.json"
    done
    
    # 添加 PLC 記憶體 (80個)
    for j in {1..80}; do
        value=$((RANDOM % 65536))
        if [ $j -eq 80 ]; then
            echo "  \"PLC_D${j}\": ${value}" >> "/tmp/agv_status_${agv_id}.json"
        else
            echo "  \"PLC_D${j}\": ${value}," >> "/tmp/agv_status_${agv_id}.json"
        fi
    done
    
    # 關閉 JSON
    echo "}" >> "/tmp/agv_status_${agv_id}.json"
    
    echo "✅ 生成 ${agv_id} 狀態檔案: /tmp/agv_status_${agv_id}.json"
done

# 複製所有檔案到容器內
echo "\n📋 複製檔案到 AGVC 容器..."
for agv_id in "${AGVS[@]}"; do
    docker cp "/tmp/agv_status_${agv_id}.json" agvc_server:"/tmp/agv_status_${agv_id}.json"
    echo "✅ 已複製 ${agv_id} 狀態到容器"
done

# 生成一個預設的 agv_status.json (用於相容舊版)
cp "/tmp/agv_status_loader01.json" "/tmp/agv_status.json"
docker cp "/tmp/agv_status.json" agvc_server:"/tmp/agv_status.json"

echo "\n✅ 完成! 已生成 6 個 AGV 的測試狀態檔案"
echo "📌 訪問 http://localhost:8003/test 選擇要監控的 AGV"
echo "📌 或直接訪問 http://localhost:8003/?agv_id=<agv_id> 監控特定 AGV"