#!/usr/bin/env python3
"""
生成測試日誌數據
"""

from sqlmodel import delete
from db_proxy.models import RosoutLog
from agvcui.db import connection_pool
import sys
import os
from datetime import datetime, timedelta
import random

# 添加必要的路徑
current_dir = os.path.dirname(os.path.abspath(__file__))
agvcui_src = os.path.join(current_dir, '..', '..')

sys.path.insert(0, agvcui_src)

# 添加 db_proxy 路徑
db_proxy_path = os.path.join(current_dir, '..', '..', '..', '..', 'db_proxy')
sys.path.insert(0, db_proxy_path)


def generate_test_logs():
    """生成測試日誌數據"""
    print("開始生成測試日誌數據...")

    # 日誌級別定義
    log_levels = {
        10: "DEBUG",
        20: "INFO",
        30: "WARN",
        40: "ERROR",
        50: "FATAL"
    }

    # 節點名稱列表
    node_names = [
        "agv_controller",
        "navigation_node",
        "sensor_manager",
        "task_scheduler",
        "device_monitor",
        "safety_controller",
        "battery_monitor",
        "communication_hub",
        "map_server",
        "path_planner"
    ]

    # 消息模板
    message_templates = {
        10: [  # DEBUG
            "Received sensor data: {sensor_value}",
            "Processing navigation waypoint: ({x}, {y})",
            "Battery level check: {battery}%",
            "Device status update: {device} - {status}",
            "Path calculation completed in {time}ms"
        ],
        20: [  # INFO
            "AGV started successfully",
            "Task {task_id} assigned to AGV",
            "Navigation to position ({x}, {y}) completed",
            "Device {device} connected",
            "System health check passed",
            "Task {task_id} completed successfully",
            "AGV reached charging station",
            "Map update received"
        ],
        30: [  # WARN
            "Low battery warning: {battery}% remaining",
            "Obstacle detected, replanning path",
            "Device {device} connection unstable",
            "Task {task_id} delayed due to traffic",
            "Sensor calibration drift detected",
            "Network latency high: {latency}ms",
            "Temperature warning: {temp}°C"
        ],
        40: [  # ERROR
            "Failed to connect to device {device}",
            "Navigation error: path blocked",
            "Task {task_id} failed: {error}",
            "Sensor {sensor} malfunction detected",
            "Communication timeout with {node}",
            "Emergency stop triggered",
            "Battery critical: {battery}%",
            "System overload detected"
        ],
        50: [  # FATAL
            "Critical system failure",
            "Safety system malfunction",
            "Complete navigation failure",
            "Power system critical error",
            "Emergency shutdown initiated"
        ]
    }

    # 生成時間範圍（過去7天到現在）
    from datetime import timezone
    end_time = datetime.now(timezone.utc)
    start_time = end_time - timedelta(days=7)

    logs_to_create = []

    print(f"生成時間範圍: {start_time} 到 {end_time}")

    # 生成不同時間段的日誌
    for day in range(7):
        day_start = start_time + timedelta(days=day)

        # 每天生成不同數量的日誌
        daily_log_count = random.randint(50, 150)

        for _ in range(daily_log_count):
            # 隨機時間
            random_seconds = random.randint(0, 24 * 60 * 60 - 1)
            log_time = day_start + timedelta(seconds=random_seconds)

            # 隨機選擇日誌級別（INFO 和 WARN 較多）
            level_weights = {10: 10, 20: 40, 30: 25, 40: 20, 50: 5}
            level = random.choices(
                list(level_weights.keys()),
                weights=list(level_weights.values())
            )[0]

            # 隨機選擇節點
            node_name = random.choice(node_names)

            # 生成消息
            template = random.choice(message_templates[level])

            # 替換模板變量
            message = template.format(
                sensor_value=random.randint(0, 100),
                x=random.randint(-50, 50),
                y=random.randint(-50, 50),
                battery=random.randint(10, 100),
                device=random.choice(["Camera", "Lidar", "Motor", "Encoder"]),
                status=random.choice(["Online", "Offline", "Error"]),
                time=random.randint(10, 500),
                task_id=f"TASK_{random.randint(1000, 9999)}",
                error=random.choice(
                    ["Timeout", "Connection Lost", "Invalid Data"]),
                sensor=random.choice(["IMU", "GPS", "Camera", "Lidar"]),
                node=random.choice(node_names),
                latency=random.randint(50, 500),
                temp=random.randint(60, 90)
            )

            # 創建日誌對象
            log = RosoutLog(
                timestamp=log_time,
                level=level,
                name=node_name,
                message=message,
                file="",
                function="",
                line=0
            )

            logs_to_create.append(log)

    # 批量插入數據庫
    print(f"準備插入 {len(logs_to_create)} 條日誌記錄...")

    try:
        with connection_pool.get_session() as session:
            # 清除現有的測試數據（可選）
            print("清除現有日誌數據...")
            delete_statement = delete(RosoutLog)
            session.exec(delete_statement)

            # 批量添加新數據
            session.add_all(logs_to_create)
            session.commit()

            print(f"✅ 成功插入 {len(logs_to_create)} 條日誌記錄")

            # 統計信息
            print("\n📊 數據統計:")
            for level, name in log_levels.items():
                count = len(
                    [log for log in logs_to_create if log.level == level])
                percentage = (count / len(logs_to_create)) * 100
                print(
                    f"  {name:5} (Level {level:2d}): {count:4d} 條 ({percentage:5.1f}%)")

            print(f"\n🏷️  節點統計:")
            for node in node_names:
                count = len(
                    [log for log in logs_to_create if log.name == node])
                percentage = (count / len(logs_to_create)) * 100
                print(f"  {node:20}: {count:3d} 條 ({percentage:4.1f}%)")

            print(f"\n📅 時間分布:")
            for day in range(7):
                day_start = start_time + timedelta(days=day)
                day_end = day_start + timedelta(days=1)
                count = len([log for log in logs_to_create
                             if day_start <= log.timestamp < day_end])
                print(f"  {day_start.strftime('%Y-%m-%d')}: {count:3d} 條")

    except Exception as e:
        print(f"❌ 插入數據失敗: {e}")
        return False

    print("\n🎯 測試建議:")
    print("現在您可以測試以下篩選功能:")
    print("  1. 日誌級別篩選 - 選擇 ERROR 查看錯誤日誌")
    print("  2. 節點名稱篩選 - 選擇 'agv_controller' 查看特定節點")
    print("  3. 時間範圍篩選 - 選擇最近1小時或今天")
    print("  4. 消息內容篩選 - 搜尋 'battery' 或 'task'")
    print("  5. 組合篩選 - 同時使用多個條件")
    print("  6. 快速篩選 - 點擊 '僅錯誤' 按鈕")

    return True


if __name__ == "__main__":
    success = generate_test_logs()
    if success:
        print("\n✅ 測試數據生成完成！")
        print("🌐 請訪問 http://localhost:8001/rosout_logs 測試篩選功能")
    else:
        print("\n❌ 測試數據生成失敗！")
