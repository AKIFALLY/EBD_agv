#!/usr/bin/env python3
"""
生成 Runtime Log 測試數據的腳本
用於測試 runtime logs 頁面的篩選和分頁功能
"""

import os
import sys
import random
from datetime import datetime, timedelta, timezone

# 添加項目路徑到 Python 路徑
current_dir = os.path.dirname(os.path.abspath(__file__))
agvcui_src = os.path.join(current_dir, '..')
sys.path.insert(0, agvcui_src)

# 添加 db_proxy 路徑
db_proxy_path = os.path.join(current_dir, '..', '..', '..', '..', 'db_proxy')
sys.path.insert(0, db_proxy_path)

from agvcui.db import connection_pool
from db_proxy.models import RuntimeLog
from sqlmodel import delete


def generate_runtime_test_logs():
    """生成 Runtime Log 測試數據"""
    
    # 日誌級別定義
    LOG_LEVELS = {
        10: "DEBUG",
        20: "INFO", 
        30: "WARN",
        40: "ERROR",
        50: "FATAL"
    }
    
    # 節點名稱列表（模擬不同的運行時組件）
    NODE_NAMES = [
        "task_manager",
        "motion_controller", 
        "safety_monitor",
        "battery_manager",
        "navigation_core",
        "sensor_fusion",
        "communication_hub",
        "database_manager",
        "file_handler",
        "system_monitor"
    ]
    
    # 消息模板
    MESSAGE_TEMPLATES = {
        10: [  # DEBUG
            "Debugging task execution step {step}",
            "Processing sensor data from {sensor}",
            "Memory usage: {memory}MB",
            "Cache hit rate: {rate}%",
            "Thread {thread_id} processing request"
        ],
        20: [  # INFO
            "Task {task_id} started successfully",
            "Battery level: {battery}%",
            "Navigation waypoint reached: {waypoint}",
            "File {filename} processed successfully",
            "System health check passed",
            "Connection established with {device}",
            "📦Configuration loaded from {config_file}"
        ],
        30: [  # WARN
            "Battery level low: {battery}%",
            "High CPU usage detected: {cpu}%",
            "Network latency increased: {latency}ms",
            "Disk space running low: {space}GB remaining",
            "Sensor {sensor} response time slow",
            "Task {task_id} taking longer than expected"
        ],
        40: [  # ERROR
            "Failed to connect to {device}",
            "Task {task_id} execution failed: {error}",
            "Sensor {sensor} malfunction detected",
            "Database connection lost",
            "File {filename} not found",
            "Memory allocation failed",
            "Network timeout occurred"
        ],
        50: [  # FATAL
            "System critical error: {error}",
            "Emergency stop triggered",
            "Hardware failure detected: {hardware}",
            "Safety system malfunction",
            "Critical battery failure"
        ]
    }
    
    # 生成時間範圍（過去7天到現在）
    from datetime import timezone
    end_time = datetime.now(timezone.utc)
    start_time = end_time - timedelta(days=7)
    
    print("🚀 開始生成 Runtime Log 測試數據...")
    print(f"📅 時間範圍: {start_time.strftime('%Y-%m-%d %H:%M:%S')} 到 {end_time.strftime('%Y-%m-%d %H:%M:%S')}")
    
    # 生成日誌記錄
    logs_to_create = []
    total_logs = 800  # 生成800條日誌
    
    for i in range(total_logs):
        # 隨機選擇日誌級別（INFO 和 WARN 較多，FATAL 較少）
        level_weights = [10, 40, 25, 20, 5]  # DEBUG, INFO, WARN, ERROR, FATAL 的權重
        level = random.choices(list(LOG_LEVELS.keys()), weights=level_weights)[0]
        
        # 隨機選擇節點名稱
        node_name = random.choice(NODE_NAMES)
        
        # 生成隨機時間
        time_range = end_time - start_time
        random_seconds = random.randint(0, int(time_range.total_seconds()))
        log_time = start_time + timedelta(seconds=random_seconds)
        
        # 生成消息內容
        message_template = random.choice(MESSAGE_TEMPLATES[level])
        
        # 填充消息模板中的變數
        message = message_template.format(
            step=random.randint(1, 10),
            sensor=random.choice(["lidar", "camera", "ultrasonic", "imu"]),
            memory=random.randint(100, 2000),
            rate=random.randint(60, 95),
            thread_id=random.randint(1000, 9999),
            task_id=f"T{random.randint(1000, 9999)}",
            battery=random.randint(10, 100),
            waypoint=f"WP{random.randint(1, 50)}",
            filename=f"data_{random.randint(1, 100)}.json",
            device=random.choice(["PLC", "AGV", "Scanner", "Printer"]),
            config_file=random.choice(["system.yaml", "navigation.json", "safety.xml"]),
            cpu=random.randint(70, 95),
            latency=random.randint(100, 500),
            space=random.randint(1, 10),
            error=random.choice(["timeout", "invalid_data", "connection_lost", "permission_denied"]),
            hardware=random.choice(["motor", "sensor", "controller", "power_supply"])
        )
        
        # 創建 RuntimeLog 對象
        runtime_log = RuntimeLog(
            timestamp=log_time,
            level=level,
            name=node_name,
            message=message,
            file="runtime_system.py",
            function="process_runtime_event",
            line=random.randint(100, 500)
        )
        
        logs_to_create.append(runtime_log)
    
    # 統計信息
    level_counts = {}
    node_counts = {}
    day_counts = {}
    
    for log in logs_to_create:
        # 統計級別分布
        level_name = LOG_LEVELS[log.level]
        level_counts[level_name] = level_counts.get(level_name, 0) + 1
        
        # 統計節點分布
        node_counts[log.name] = node_counts.get(log.name, 0) + 1
        
        # 統計每日分布
        day_key = log.timestamp.strftime('%Y-%m-%d')
        day_counts[day_key] = day_counts.get(day_key, 0) + 1
    
    print(f"📊 總記錄數: {len(logs_to_create)} 條")
    print("\n🎯 日誌級別分布:")
    for level_name, count in sorted(level_counts.items()):
        percentage = (count / len(logs_to_create)) * 100
        print(f"  - {level_name}: {count} 條 ({percentage:.1f}%)")
    
    print("\n🏷️ 節點分布:")
    for node_name, count in sorted(node_counts.items()):
        print(f"  - {node_name}: {count} 條")
    
    print("\n📅 時間分布:")
    for day, count in sorted(day_counts.items()):
        print(f"  - {day}: {count} 條")
    
    # 插入數據庫
    try:
        with connection_pool.get_session() as session:
            # 清除現有的測試數據（可選）
            print("\n🗑️ 清除現有 Runtime Log 數據...")
            delete_statement = delete(RuntimeLog)
            session.exec(delete_statement)

            # 批量添加新數據
            print("💾 插入新的測試數據...")
            session.add_all(logs_to_create)
            session.commit()
            
        print("✅ Runtime Log 測試數據生成成功！")
        print(f"🎉 已生成 {len(logs_to_create)} 條 Runtime Log 記錄")
        
    except Exception as e:
        print(f"❌ 插入數據失敗: {e}")
        print("❌ Runtime Log 測試數據生成失敗！")


if __name__ == "__main__":
    generate_runtime_test_logs()
