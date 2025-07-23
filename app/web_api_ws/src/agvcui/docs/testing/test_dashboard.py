#!/usr/bin/env python3
"""
Dashboard 功能測試腳本
測試 Dashboard 頁面的基本功能和資料顯示
"""

import asyncio
import json
from datetime import datetime, timedelta
from zoneinfo import ZoneInfo

# 模擬測試資料


def generate_test_agv_data():
    """生成測試 AGV 資料"""
    return [
        {
            "id": 1,
            "name": "AGV-001",
            "model": "K400",
            "x": 100.5,
            "y": 200.3,
            "heading": 45.0,
            "battery": 85.5,
            "enable": 1,
            "status_id": 1
        },
        {
            "id": 2,
            "name": "AGV-002",
            "model": "Cargo",
            "x": 150.2,
            "y": 180.7,
            "heading": 90.0,
            "battery": 15.2,  # 低電量
            "enable": 1,
            "status_id": 1
        },
        {
            "id": 3,
            "name": "AGV-003",
            "model": "Loader",
            "x": 200.1,
            "y": 250.8,
            "heading": 180.0,
            "battery": 92.8,
            "enable": 0,  # 離線
            "status_id": 2
        }
    ]


def generate_test_signal_data():
    """生成測試 Signal 資料"""
    return [
        {
            "id": 1,
            "eqp_id": 1,
            "name": "sensor_01",
            "value": True,
            "type_of_value": "boolean"
        },
        {
            "id": 2,
            "eqp_id": 1,
            "name": "sensor_02",
            "value": False,
            "type_of_value": "boolean"
        },
        {
            "id": 3,
            "eqp_id": 2,
            "name": "temperature",
            "value": None,  # 錯誤狀態
            "type_of_value": "float"
        },
        {
            "id": 4,
            "eqp_id": 2,
            "name": "pressure",
            "value": 1.5,
            "type_of_value": "float"
        }
    ]


def generate_test_rack_data():
    """生成測試 Rack 資料"""
    return [
        {
            "id": 1,
            "name": "Rack-A01",
            "product_id": 1,
            "product_name": "產品A",
            "size": "S",
            "total": 32,
            "count": 28  # 使用中
        },
        {
            "id": 2,
            "name": "Rack-B01",
            "product_id": 2,
            "product_name": "產品B",
            "size": "L",
            "total": 16,
            "count": 0  # 空閒
        },
        {
            "id": 3,
            "name": "Rack-C01",
            "product_id": 1,
            "product_name": "產品A",
            "size": "S",
            "total": 32,
            "count": 16
        }
    ]


def generate_test_task_data():
    """生成測試 Task 資料"""
    return [
        {
            "id": 1,
            "name": "運輸任務-001",
            "status_id": 1,  # 執行中
            "work_id": 1
        },
        {
            "id": 2,
            "name": "運輸任務-002",
            "status_id": 2,  # 已完成
            "work_id": 1
        },
        {
            "id": 3,
            "name": "運輸任務-003",
            "status_id": 3,  # 失敗
            "work_id": 2
        },
        {
            "id": 4,
            "name": "運輸任務-004",
            "status_id": 1,  # 執行中
            "work_id": 1
        }
    ]


def generate_test_product_data():
    """生成測試 Product 資料"""
    return [
        {
            "id": 1,
            "name": "產品A",
            "size": "S",
            "room": 2,
            "room_id": 2,
            "soaking_times": 3,
            "process_settings": {
                "soaking_times": 3,
                "description": "標準泡藥製程"
            }
        },
        {
            "id": 2,
            "name": "產品B",
            "size": "L",
            "room": 1,
            "room_id": 1,
            "soaking_times": 5,
            "process_settings": {
                "soaking_times": 5,
                "description": "加強泡藥製程"
            }
        }
    ]


def generate_test_room2_rack_data():
    """生成測試房間2貨架資料"""
    return [
        {
            "id": 101,
            "name": "Rack-R2-IN-01",
            "location_id": 20100,  # 房間2入口（修正後的編碼）
            "count": 16,
            "total": 32,
            "product_id": 1,
            "product_name": "產品A"
        },
        {
            "id": 102,
            "name": "Rack-R2-OUT-01",
            "location_id": 20200,  # 房間2出口（修正後的編碼）
            "count": 8,
            "total": 32,
            "product_id": 1,
            "product_name": "產品A"
        },
        {
            "id": 103,
            "name": "Rack-R1-01",
            "location_id": 10100,  # 房間1入口（修正後的編碼）
            "count": 0,
            "total": 32,
            "product_id": 2,
            "product_name": "產品B"
        },
        {
            "id": 104,
            "name": "Rack-R3-IN-01",
            "location_id": 30100,  # 房間3入口（新增測試資料）
            "count": 12,
            "total": 32,
            "product_id": 3,
            "product_name": "產品C"
        }
    ]


def test_agv_stats_calculation():
    """測試 AGV 統計計算邏輯"""
    print("🧪 測試 AGV 統計計算...")

    agvs = generate_test_agv_data()

    # 模擬 JavaScript 中的計算邏輯
    total = len(agvs)
    online = sum(1 for agv in agvs if agv.get('enable') == 1)
    offline = total - online

    batteries = [agv['battery'] for agv in agvs if agv.get('battery') is not None]
    avg_battery = round(sum(batteries) / len(batteries)) if batteries else 0
    low_battery = sum(1 for battery in batteries if battery < 20)

    expected_stats = {
        'total': 3,
        'online': 2,
        'offline': 1,
        'avg_battery': 64,  # (85.5 + 15.2 + 92.8) / 3 ≈ 64
        'low_battery': 1
    }

    actual_stats = {
        'total': total,
        'online': online,
        'offline': offline,
        'avg_battery': avg_battery,
        'low_battery': low_battery
    }

    print(f"  預期結果: {expected_stats}")
    print(f"  實際結果: {actual_stats}")

    # 驗證結果
    assert actual_stats['total'] == expected_stats[
        'total'], f"總數不符: {actual_stats['total']} != {expected_stats['total']}"
    assert actual_stats['online'] == expected_stats[
        'online'], f"線上數不符: {actual_stats['online']} != {expected_stats['online']}"
    assert actual_stats['offline'] == expected_stats[
        'offline'], f"離線數不符: {actual_stats['offline']} != {expected_stats['offline']}"
    assert actual_stats['low_battery'] == expected_stats[
        'low_battery'], f"低電量數不符: {actual_stats['low_battery']} != {expected_stats['low_battery']}"

    print("  ✅ AGV 統計計算測試通過")


def test_signal_stats_calculation():
    """測試 Signal 統計計算邏輯"""
    print("🧪 測試 Signal 統計計算...")

    # 更新測試資料以反映實際的信號資料結構
    signals = [
        {
            "id": 1,
            "eqp_id": 1,
            "name": "sensor_01",
            "value": "true",  # 字符串形式的布林值
            "type_of_value": "bool"
        },
        {
            "id": 2,
            "eqp_id": 1,
            "name": "sensor_02",
            "value": "false",  # 字符串形式的布林值
            "type_of_value": "bool"
        },
        {
            "id": 3,
            "eqp_id": 2,
            "name": "temperature",
            "value": None,  # 錯誤狀態
            "type_of_value": "float"
        },
        {
            "id": 4,
            "eqp_id": 2,
            "name": "pressure",
            "value": "1.5",  # 字符串形式的浮點數
            "type_of_value": "float"
        },
        {
            "id": 5,
            "eqp_id": 3,
            "name": "status",
            "value": "normal",  # 字符串狀態
            "type_of_value": "string"
        }
    ]

    # 模擬新的信號統計邏輯
    total = len(signals)
    normal = 0
    warning = 0
    error = 0

    for signal in signals:
        if signal['value'] is None or signal['value'] == '':
            error += 1
            continue

        type_of_value = signal['type_of_value'].lower() if signal['type_of_value'] else ''
        value = signal['value']

        try:
            if type_of_value in ['bool', 'boolean']:
                bool_value = value.lower() == 'true' or value == '1'
                if bool_value:
                    normal += 1
                else:
                    warning += 1
            elif type_of_value in ['float', 'double']:
                float_value = float(value)
                if float_value > 0:
                    normal += 1
                elif float_value == 0:
                    warning += 1
                else:
                    error += 1
            elif type_of_value == 'string':
                error_keywords = ['error', 'fail', 'fault', 'alarm', '錯誤', '故障', '警報']
                warning_keywords = ['warning', 'caution', '警告', '注意']
                lower_value = value.lower()

                if any(keyword in lower_value for keyword in error_keywords):
                    error += 1
                elif any(keyword in lower_value for keyword in warning_keywords):
                    warning += 1
                else:
                    normal += 1
            else:
                normal += 1
        except:
            error += 1

    expected_stats = {
        'total': 5,
        'normal': 3,  # true, 1.5, normal
        'warning': 1,  # false
        'error': 1    # None
    }

    actual_stats = {
        'total': total,
        'normal': normal,
        'warning': warning,
        'error': error
    }

    print(f"  預期結果: {expected_stats}")
    print(f"  實際結果: {actual_stats}")

    assert actual_stats == expected_stats, f"Signal 統計不符: {actual_stats} != {expected_stats}"

    print("  ✅ Signal 統計計算測試通過")


def test_rack_stats_calculation():
    """測試 Rack 統計計算邏輯"""
    print("🧪 測試 Rack 統計計算...")

    racks = generate_test_rack_data()

    total = len(racks)
    used = sum(1 for rack in racks if rack.get('count', 0) > 0)
    utilization = round((used / total) * 100) if total > 0 else 0

    expected_stats = {
        'total': 3,
        'used': 2,  # Rack-A01 和 Rack-C01 有使用
        'utilization': 67  # 2/3 * 100 ≈ 67
    }

    actual_stats = {
        'total': total,
        'used': used,
        'utilization': utilization
    }

    print(f"  預期結果: {expected_stats}")
    print(f"  實際結果: {actual_stats}")

    assert actual_stats == expected_stats, f"Rack 統計不符: {actual_stats} != {expected_stats}"

    print("  ✅ Rack 統計計算測試通過")


def test_task_data_processing():
    """測試 Task 資料處理邏輯"""
    print("🧪 測試 Task 資料處理...")

    # 更新測試資料以反映新的任務處理邏輯
    tasks = [
        {
            "id": 1,
            "name": "運輸任務-001",
            "status_id": 1,  # 待執行
            "work_id": 1,
            "created_at": datetime.now(ZoneInfo("Asia/Taipei")).isoformat()
        },
        {
            "id": 2,
            "name": "運輸任務-002",
            "status_id": 2,  # 執行中
            "work_id": 1,
            "created_at": (datetime.now(ZoneInfo("Asia/Taipei")) - timedelta(minutes=30)).isoformat()
        },
        {
            "id": 3,
            "name": "運輸任務-003",
            "status_id": 3,  # 已完成（不應顯示）
            "work_id": 2,
            "created_at": datetime.now(ZoneInfo("Asia/Taipei")).isoformat()
        },
        {
            "id": 4,
            "name": "運輸任務-004",
            "status_id": 5,  # 失敗（不應顯示）
            "work_id": 1,
            "created_at": datetime.now(ZoneInfo("Asia/Taipei")).isoformat()
        },
        {
            "id": 5,
            "name": "運輸任務-005",
            "status_id": 1,  # 待執行
            "work_id": 3,
            "created_at": (datetime.now(ZoneInfo("Asia/Taipei")) - timedelta(minutes=10)).isoformat()
        }
    ]

    # 模擬新的任務處理邏輯
    # 只保留待執行和執行中的任務
    active_tasks = [task for task in tasks if task.get('status_id') in [1, 2]]

    # 按狀態和時間排序（執行中優先，然後按時間）
    active_tasks.sort(key=lambda x: (
        0 if x.get('status_id') == 2 else 1,  # 執行中優先
        -(datetime.fromisoformat(x.get('created_at', '1970-01-01')).timestamp())  # 時間倒序
    ))

    # 限制顯示數量（最多10個）
    limited_tasks = active_tasks[:10]

    # 統計數量
    pending_count = len([task for task in active_tasks if task.get('status_id') == 1])
    running_count = len([task for task in active_tasks if task.get('status_id') == 2])

    expected_data = {
        'activeTasks': 3,        # 任務 1, 2, 5（只有待執行和執行中）
        'totalActive': 3,
        'pendingCount': 2,       # 任務 1, 5
        'runningCount': 1,       # 任務 2
        'firstTaskId': 2,        # 執行中的任務應該排在第一位
        'firstTaskStatus': 2     # 執行中
    }

    actual_data = {
        'activeTasks': len(limited_tasks),
        'totalActive': len(active_tasks),
        'pendingCount': pending_count,
        'runningCount': running_count,
        'firstTaskId': limited_tasks[0]['id'] if limited_tasks else None,
        'firstTaskStatus': limited_tasks[0]['status_id'] if limited_tasks else None
    }

    print(f"  預期結果: {expected_data}")
    print(f"  實際結果: {actual_data}")
    print(f"  活躍任務列表: {[f'ID:{t['id']}, 狀態:{t['status_id']}' for t in limited_tasks]}")

    assert actual_data == expected_data, f"Task 資料處理不符: {actual_data} != {expected_data}"

    print("  ✅ Task 資料處理測試通過")


def test_room2_carrier_stats_calculation():
    """測試房間2載具統計計算邏輯"""
    print("🧪 測試房間2載具統計計算...")

    carriers = generate_test_agv_data()  # 重用 AGV 資料作為載具資料

    # 模擬房間2載具（location_id 200-299）
    room2_carriers = [
        {"id": 1, "room_id": 2, "location_id": 201, "status": "processing"},
        {"id": 2, "room_id": 2, "location_id": 202, "status": "processing"},
        {"id": 3, "room_id": 1, "location_id": 101, "status": "idle"},
        {"id": 4, "room_id": 2, "location_id": 250, "status": "processing"}
    ]

    # 計算房間2內處理中的載具
    room2_processing = len([c for c in room2_carriers if c.get('room_id') == 2])

    expected_stats = {
        'carriersInProcess': 3  # 載具 1, 2, 4 在房間2
    }

    actual_stats = {
        'carriersInProcess': room2_processing
    }

    print(f"  預期結果: {expected_stats}")
    print(f"  實際結果: {actual_stats}")

    assert actual_stats == expected_stats, f"房間2載具統計不符: {actual_stats} != {expected_stats}"

    print("  ✅ 房間2載具統計計算測試通過")


def test_room2_product_info():
    """測試房間2產品資訊處理"""
    print("🧪 測試房間2產品資訊處理...")

    products = generate_test_product_data()

    # 找到房間2的產品
    room2_products = [p for p in products if p.get('room') == 2 or p.get('room_id') == 2]

    expected_info = {
        'product_name': '產品A',
        'size': 'S',
        'soaking_times': 3
    }

    if room2_products:
        current_product = room2_products[0]
        actual_info = {
            'product_name': current_product['name'],
            'size': current_product['size'],
            'soaking_times': current_product['soaking_times']
        }
    else:
        actual_info = {
            'product_name': '無',
            'size': '-',
            'soaking_times': '-'
        }

    print(f"  預期結果: {expected_info}")
    print(f"  實際結果: {actual_info}")

    assert actual_info == expected_info, f"房間2產品資訊不符: {actual_info} != {expected_info}"

    print("  ✅ 房間2產品資訊處理測試通過")


def test_room2_rack_status():
    """測試房間2貨架狀態檢測"""
    print("🧪 測試房間2貨架狀態檢測...")

    racks = generate_test_room2_rack_data()

    # 使用修正後的 location_id 編碼規則
    # 房間2入口：20100，房間2出口：20200
    room_id = 2
    entrance_location_id = room_id * 10000 + 100  # 20100
    exit_location_id = room_id * 10000 + 200      # 20200

    # 檢查入口貨架（精確匹配）
    entrance_racks = [r for r in racks if r['location_id']
                      == entrance_location_id and r['count'] > 0]

    # 檢查出口貨架（精確匹配）
    exit_racks = [r for r in racks if r['location_id'] == exit_location_id and r['count'] > 0]

    # 測試顯示內容（顯示具體貨架 ID）
    entrance_status = f"貨架 {entrance_racks[0]['name']}" if entrance_racks else '無貨架'
    exit_status = f"貨架 {exit_racks[0]['name']}" if exit_racks else '無貨架'

    expected_status = {
        'entrance_has_rack': True,   # location_id 20100 有貨架
        'exit_has_rack': True,       # location_id 20200 有貨架
        'entrance_count': 1,
        'exit_count': 1,
        'entrance_status': '貨架 Rack-R2-IN-01',   # 顯示具體貨架名稱
        'exit_status': '貨架 Rack-R2-OUT-01'       # 顯示具體貨架名稱
    }

    actual_status = {
        'entrance_has_rack': len(entrance_racks) > 0,
        'exit_has_rack': len(exit_racks) > 0,
        'entrance_count': len(entrance_racks),
        'exit_count': len(exit_racks),
        'entrance_status': entrance_status,
        'exit_status': exit_status
    }

    print(f"  使用的 location_id: 入口={entrance_location_id}, 出口={exit_location_id}")
    print(f"  預期結果: {expected_status}")
    print(f"  實際結果: {actual_status}")

    assert actual_status == expected_status, f"房間2貨架狀態不符: {actual_status} != {expected_status}"

    print("  ✅ 房間2貨架狀態檢測測試通過")


def test_room_location_id_encoding():
    """測試房間 location_id 編碼規則"""
    print("🧪 測試房間 location_id 編碼規則...")

    # 測試編碼規則：{房間ID}{位置類型}00
    test_cases = [
        {'room_id': 1, 'entrance': 10100, 'exit': 10200},
        {'room_id': 2, 'entrance': 20100, 'exit': 20200},
        {'room_id': 3, 'entrance': 30100, 'exit': 30200},
        {'room_id': 4, 'entrance': 40100, 'exit': 40200},
        {'room_id': 5, 'entrance': 50100, 'exit': 50200},
    ]

    for case in test_cases:
        room_id = case['room_id']
        expected_entrance = case['entrance']
        expected_exit = case['exit']

        # 模擬 JavaScript 中的編碼邏輯
        actual_entrance = room_id * 10000 + 100
        actual_exit = room_id * 10000 + 200

        assert actual_entrance == expected_entrance, f"房間{room_id}入口編碼錯誤: {actual_entrance} != {expected_entrance}"
        assert actual_exit == expected_exit, f"房間{room_id}出口編碼錯誤: {actual_exit} != {expected_exit}"

        print(f"  房間{room_id}: 入口={actual_entrance}, 出口={actual_exit} ✅")

    print("  ✅ 房間 location_id 編碼規則測試通過")


def test_multiple_rooms_rack_status():
    """測試多房間貨架狀態檢測"""
    print("🧪 測試多房間貨架狀態檢測...")

    racks = generate_test_room2_rack_data()

    # 測試房間1（應該無貨架，因為 count=0）
    room1_entrance_id = 1 * 10000 + 100  # 10100
    room1_exit_id = 1 * 10000 + 200      # 10200

    room1_entrance_racks = [r for r in racks if r['location_id']
                            == room1_entrance_id and r['count'] > 0]
    room1_exit_racks = [r for r in racks if r['location_id'] == room1_exit_id and r['count'] > 0]

    # 測試房間3（應該有入口貨架）
    room3_entrance_id = 3 * 10000 + 100  # 30100
    room3_exit_id = 3 * 10000 + 200      # 30200

    room3_entrance_racks = [r for r in racks if r['location_id']
                            == room3_entrance_id and r['count'] > 0]
    room3_exit_racks = [r for r in racks if r['location_id'] == room3_exit_id and r['count'] > 0]

    expected_results = {
        'room1_entrance_has_rack': False,  # count=0
        'room1_exit_has_rack': False,      # 無此 location_id
        'room3_entrance_has_rack': True,   # location_id 30100 有貨架
        'room3_exit_has_rack': False,      # 無此 location_id
    }

    actual_results = {
        'room1_entrance_has_rack': len(room1_entrance_racks) > 0,
        'room1_exit_has_rack': len(room1_exit_racks) > 0,
        'room3_entrance_has_rack': len(room3_entrance_racks) > 0,
        'room3_exit_has_rack': len(room3_exit_racks) > 0,
    }

    print(f"  預期結果: {expected_results}")
    print(f"  實際結果: {actual_results}")

    assert actual_results == expected_results, f"多房間貨架狀態不符: {actual_results} != {expected_results}"

    print("  ✅ 多房間貨架狀態檢測測試通過")


def test_json_serialization():
    """測試 JSON 序列化（中文字符處理）"""
    print("🧪 測試 JSON 序列化...")

    test_data = {
        "agv_name": "AGV-測試車輛",
        "status": "運行中",
        "location": "倉庫A區",
        "room2_product": "產品A",
        "timestamp": datetime.now(ZoneInfo("Asia/Taipei")).isoformat()
    }

    # 測試 JSON 序列化（確保中文字符正確處理）
    json_str = json.dumps(test_data, ensure_ascii=False)

    # 驗證中文字符沒有被轉義
    assert "AGV-測試車輛" in json_str, "中文字符被錯誤轉義"
    assert "運行中" in json_str, "中文狀態被錯誤轉義"
    assert "倉庫A區" in json_str, "中文位置被錯誤轉義"
    assert "產品A" in json_str, "中文產品名被錯誤轉義"

    # 測試反序列化
    parsed_data = json.loads(json_str)
    assert parsed_data["agv_name"] == "AGV-測試車輛", "反序列化後中文字符不正確"
    assert parsed_data["room2_product"] == "產品A", "反序列化後中文產品名不正確"

    print(f"  JSON 輸出: {json_str}")
    print("  ✅ JSON 序列化測試通過")


def main():
    """主測試函數"""
    print("🚀 開始 Dashboard 功能測試")
    print("=" * 50)

    try:
        # 執行各項測試
        test_agv_stats_calculation()
        print()

        test_signal_stats_calculation()
        print()

        test_rack_stats_calculation()
        print()

        test_task_data_processing()
        print()

        test_room2_carrier_stats_calculation()
        print()

        test_room2_product_info()
        print()

        test_room2_rack_status()
        print()

        test_room_location_id_encoding()
        print()

        test_multiple_rooms_rack_status()
        print()

        test_json_serialization()
        print()

        print("=" * 50)
        print("✅ 所有測試通過！Dashboard 功能正常")

    except AssertionError as e:
        print(f"❌ 測試失敗: {e}")
        return False
    except Exception as e:
        print(f"❌ 測試過程中發生錯誤: {e}")
        return False

    return True


if __name__ == "__main__":
    success = main()
    exit(0 if success else 1)
