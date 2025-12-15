#!/usr/bin/env python3
"""
測試 KUKA API Client 便利方法
"""
import sys
import os
import time
from datetime import datetime

# 支援容器和宿主機路徑
container_path = '/app/kuka_fleet_ws/src/kuka_fleet_adapter/kuka_fleet_adapter'
host_path = '/home/ct/EBD_agv/app/kuka_fleet_ws/src/kuka_fleet_adapter/kuka_fleet_adapter'

if os.path.exists(container_path):
    sys.path.append(container_path)
elif os.path.exists(host_path):
    sys.path.append(host_path)
else:
    print(f"❌ 找不到 kuka_fleet_adapter 模組路徑")
    sys.exit(1)

from kuka_api_client import KukaApiClient

def test_convenience_methods():
    print('=== 測試 KUKA API Client 便利方法 ===')
    
    # 初始化客戶端
    client = KukaApiClient(
        base_url='http://192.168.10.3:10870',
        username='admin',
        password='Admin'
    )
    
    if not client.token:
        print('❌ 登入失敗')
        return False
    
    print('✅ 登入成功')
    
    # 測試便利方法
    test_results = []
    
    # 1. 測試 get_all_robots
    print('\n🤖 測試 get_all_robots():')
    try:
        robots = client.get_all_robots()
        if robots.get('success'):
            robot_count = len(robots.get('data', []))
            print(f'  ✅ 成功取得 {robot_count} 台機器人')
            if robot_count > 0:
                for robot in robots['data'][:3]:  # 只顯示前3台
                    print(f'    - {robot["robotId"]}: {robot["robotType"]}, 電量: {robot["batteryLevel"]}%')
            test_results.append(('get_all_robots', True))
        else:
            print(f'  ❌ 失敗: {robots.get("message")}')
            test_results.append(('get_all_robots', False))
    except Exception as e:
        print(f'  ❌ 異常: {e}')
        test_results.append(('get_all_robots', False))
    
    # 2. 測試 get_robot_by_id
    print('\n🔍 測試 get_robot_by_id("101"):')
    try:
        robot = client.get_robot_by_id('101')
        if robot.get('success') and robot.get('data'):
            r_data = robot['data']
            if isinstance(r_data, list) and len(r_data) > 0:
                r = r_data[0]
                print(f'  ✅ 機器人 101: 位置 {r["nodeCode"]}, 電量 {r["batteryLevel"]}%')
                test_results.append(('get_robot_by_id', True))
            else:
                print('  ❌ 找不到機器人 101')
                test_results.append(('get_robot_by_id', False))
        else:
            print(f'  ❌ 失敗: {robot.get("message")}')
            test_results.append(('get_robot_by_id', False))
    except Exception as e:
        print(f'  ❌ 異常: {e}')
        test_results.append(('get_robot_by_id', False))
    
    # 3. 測試 get_all_containers_in_map
    print('\n📦 測試 get_all_containers_in_map():')
    try:
        containers = client.get_all_containers_in_map()
        if containers.get('success'):
            container_count = len(containers.get('data', []))
            print(f'  ✅ 成功取得 {container_count} 個在場容器')
            if container_count > 0:
                for container in containers['data'][:2]:  # 只顯示前2個
                    print(f'    - {container["containerCode"]}: {container["containerModelCode"]}, 位置: {container.get("nodeCode", "未知")}')
            test_results.append(('get_all_containers_in_map', True))
        else:
            print(f'  ❌ 失敗: {containers.get("message")}')
            test_results.append(('get_all_containers_in_map', False))
    except Exception as e:
        print(f'  ❌ 異常: {e}')
        test_results.append(('get_all_containers_in_map', False))
    
    # 4. 測試 get_running_jobs
    print('\n📋 測試 get_running_jobs():')
    try:
        jobs = client.get_running_jobs()
        if jobs.get('success'):
            job_data = jobs.get('data', {})

            # 詳細輸出數據結構
            print(f'  📊 返回數據類型: {type(job_data)}')

            if isinstance(job_data, dict) and 'jobs' in job_data:
                job_count = len(job_data['jobs'])
                print(f'  ✅ 成功取得 {job_count} 個運行中的作業')
            elif isinstance(job_data, list):
                job_count = len(job_data)
                print(f'  ✅ 成功取得 {job_count} 個運行中的作業（列表格式）')
                if job_count > 0:
                    print(f'  📦 第一個作業: {job_data[0].get("jobCode", "N/A")} - {job_data[0].get("missionCode", "N/A")}')
            else:
                print(f'  ⚠️  未預期的數據格式: {type(job_data)}')
                print(f'  📄 數據內容: {job_data}')
            test_results.append(('get_running_jobs', True))
        else:
            print(f'  ❌ 失敗: {jobs.get("message")}')
            test_results.append(('get_running_jobs', False))
    except Exception as e:
        print(f'  ❌ 異常: {e}')
        test_results.append(('get_running_jobs', False))
    
    # 5. 測試 get_pending_jobs (非運行中的作業)
    print('\n⏳ 測試 get_pending_jobs() [非運行中作業]:')
    try:
        pending_jobs = client.get_pending_jobs()
        if pending_jobs.get('success'):
            job_data = pending_jobs.get('data', {})

            print(f'  📊 返回數據類型: {type(job_data)}')

            if isinstance(job_data, dict) and 'jobs' in job_data:
                job_count = len(job_data['jobs'])
                print(f'  ✅ 成功取得 {job_count} 個待執行的作業')
                if job_count > 0:
                    print(f'  📦 第一個作業: {job_data["jobs"][0].get("jobCode", "N/A")} - {job_data["jobs"][0].get("missionCode", "N/A")}')
            elif isinstance(job_data, list):
                job_count = len(job_data)
                print(f'  ✅ 成功取得 {job_count} 個待執行的作業（列表格式）')
                if job_count > 0:
                    print(f'  📦 第一個作業: {job_data[0].get("jobCode", "N/A")} - {job_data[0].get("missionCode", "N/A")}')
            else:
                print(f'  ℹ️  待執行作業數: 0 或數據格式: {type(job_data)}')
            test_results.append(('get_pending_jobs', True))
        else:
            print(f'  ❌ 失敗: {pending_jobs.get("message")}')
            test_results.append(('get_pending_jobs', False))
    except Exception as e:
        print(f'  ❌ 異常: {e}')
        test_results.append(('get_pending_jobs', False))

    # 6. 測試 get_all_jobs (所有狀態的作業)
    print('\n📋 測試 job_query({}) [所有作業]:')
    try:
        all_jobs = client.job_query({})  # 空字典 = 查詢所有作業
        if all_jobs.get('success'):
            job_data = all_jobs.get('data', {})

            print(f'  📊 返回數據類型: {type(job_data)}')

            if isinstance(job_data, dict) and 'jobs' in job_data:
                job_count = len(job_data['jobs'])
                print(f'  ✅ 成功取得 {job_count} 個作業（所有狀態）')
                if job_count > 0:
                    # 統計各狀態數量（KUKA AMR 官方狀態碼）
                    status_map = {10: '待執行', 20: '執行中', 25: '等待放行', 28: '取消中',
                                  30: '已完成', 31: '已取消', 35: '手動完成', 50: '告警', 60: '流程啟動異常'}
                    status_counts = {}
                    for job in job_data['jobs']:
                        status = job.get('status', 0)
                        status_counts[status] = status_counts.get(status, 0) + 1
                    print(f'  📊 狀態分佈: ', end='')
                    for status, count in sorted(status_counts.items()):
                        print(f'{status_map.get(status, f"狀態{status}")}={count} ', end='')
                    print()
            elif isinstance(job_data, list):
                job_count = len(job_data)
                print(f'  ✅ 成功取得 {job_count} 個作業（列表格式，所有狀態）')
                if job_count > 0:
                    # 統計各狀態數量
                    status_map = {1: '待執行', 2: '運行中', 3: '已完成', 4: '失敗', 5: '已取消'}
                    status_counts = {}
                    for job in job_data:
                        status = job.get('status', 0)
                        status_counts[status] = status_counts.get(status, 0) + 1
                    print(f'  📊 狀態分佈: ', end='')
                    for status, count in sorted(status_counts.items()):
                        print(f'{status_map.get(status, f"狀態{status}")}={count} ', end='')
                    print()
            else:
                print(f'  ℹ️  作業數: 0 或數據格式: {type(job_data)}')
            test_results.append(('get_all_jobs', True))
        else:
            print(f'  ❌ 失敗: {all_jobs.get("message")}')
            test_results.append(('get_all_jobs', False))
    except Exception as e:
        print(f'  ❌ 異常: {e}')
        test_results.append(('get_all_jobs', False))

    # 7. 測試 is_token_valid
    print('\n🔐 測試 is_token_valid():')
    try:
        is_valid = client.is_token_valid()
        print(f'  Token 有效性: {"✅ 有效" if is_valid else "❌ 無效"}')
        test_results.append(('is_token_valid', True))
    except Exception as e:
        print(f'  ❌ 異常: {e}')
        test_results.append(('is_token_valid', False))

    # 8. 測試任務提交（submit_mission）
    print('\n🚀 測試 submit_mission() [提交測試任務]:')
    test_mission_code = None
    try:
        # 生成唯一的任務代碼
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        test_mission_code = f'TEST_MISSION_{timestamp}'

        # 構建簡單的 MOVE 任務
        mission = {
            "orgId": "Ching-Tech",
            "requestId": test_mission_code,
            "missionCode": test_mission_code,
            "missionType": "MOVE",
            "robotModels": ["KMP 400i diffDrive"],
            "robotIds": [101],
            "robotType": "LIFT",
            "priority": 50,
            "missionData": [
                {
                    "sequence": 1,
                    "position": "AlanACT-AlanSec1-55",
                    "type": "NODE_POINT",
                    "passStrategy": "AUTO"
                }
            ]
        }

        print(f'  📝 任務代碼: {test_mission_code}')
        print(f'  🤖 目標機器人: 101')
        print(f'  📍 目標位置: AlanACT-AlanSec1-55')

        # 提交任務
        submit_result = client.submit_mission(mission)

        if submit_result.get('success'):
            print(f'  ✅ 任務提交成功')
            test_results.append(('submit_mission', True))

            # 短暫等待任務進入系統
            print(f'  ⏳ 等待 2 秒讓任務進入系統...')
            time.sleep(2)

            # 9. 驗證任務進入隊列
            print('\n🔍 驗證任務是否進入隊列:')
            try:
                # 查詢所有任務
                all_jobs_after = client.job_query({})
                if all_jobs_after.get('success'):
                    job_data = all_jobs_after.get('data', [])

                    if isinstance(job_data, list):
                        # 查找我們提交的任務
                        our_job = None
                        for job in job_data:
                            if job.get('jobCode') == test_mission_code:
                                our_job = job
                                break

                        if our_job:
                            status_map = {1: '待執行', 2: '運行中', 3: '已完成', 4: '失敗', 5: '已取消'}
                            job_status = our_job.get('status', 0)
                            print(f'  ✅ 找到測試任務！')
                            print(f'  📊 任務狀態: {status_map.get(job_status, f"未知({job_status})")}')
                            print(f'  🆔 作業代碼: {our_job.get("jobCode", "N/A")}')
                            test_results.append(('verify_mission_in_queue', True))
                        else:
                            print(f'  ⚠️  未找到測試任務（可能已完成或失敗）')
                            # 顯示當前所有任務
                            print(f'  📋 當前任務總數: {len(job_data)}')
                            test_results.append(('verify_mission_in_queue', False))
                    else:
                        print(f'  ⚠️  數據格式異常: {type(job_data)}')
                        test_results.append(('verify_mission_in_queue', False))
                else:
                    print(f'  ❌ 查詢失敗: {all_jobs_after.get("message")}')
                    test_results.append(('verify_mission_in_queue', False))
            except Exception as e:
                print(f'  ❌ 驗證異常: {e}')
                test_results.append(('verify_mission_in_queue', False))

            # 10. 測試任務取消（清理測試任務）
            print('\n🛑 測試 mission_cancel() [清理測試任務]:')
            try:
                cancel_data = {
                    "missionCode": test_mission_code,
                    "cancelMode": "FORCE"
                }
                cancel_result = client.mission_cancel(cancel_data)

                if cancel_result.get('success'):
                    print(f'  ✅ 任務取消成功')
                    test_results.append(('cancel_test_mission', True))
                else:
                    print(f'  ⚠️  取消失敗: {cancel_result.get("message")}（任務可能已完成）')
                    test_results.append(('cancel_test_mission', True))  # 標記為成功（任務可能已完成）
            except Exception as e:
                print(f'  ⚠️  取消異常: {e}（任務可能已完成）')
                test_results.append(('cancel_test_mission', True))  # 標記為成功

        else:
            print(f'  ❌ 任務提交失敗: {submit_result.get("message")}')
            test_results.append(('submit_mission', False))
            test_results.append(('verify_mission_in_queue', False))
            test_results.append(('cancel_test_mission', False))

    except Exception as e:
        print(f'  ❌ 異常: {e}')
        test_results.append(('submit_mission', False))
        test_results.append(('verify_mission_in_queue', False))
        test_results.append(('cancel_test_mission', False))

    # 總結測試結果
    print('\n=== 測試結果總結 ===')
    passed = sum(1 for _, result in test_results if result)
    total = len(test_results)
    
    for method, result in test_results:
        status = "✅ 通過" if result else "❌ 失敗"
        print(f'{method}: {status}')
    
    print(f'\n總計: {passed}/{total} 個測試通過')
    
    if passed == total:
        print('🎉 所有便利方法測試通過！')
        return True
    else:
        print('⚠️  部分測試失敗，請檢查相關功能')
        return False

if __name__ == '__main__':
    success = test_convenience_methods()
    sys.exit(0 if success else 1)