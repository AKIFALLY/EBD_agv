#!/usr/bin/env python3
"""
測試 KUKA API Client 便利方法
"""
import sys
sys.path.append('/home/ct/RosAGV/app/kuka_fleet_ws/src/kuka_fleet_adapter/kuka_fleet_adapter')

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
            if isinstance(job_data, dict) and 'jobs' in job_data:
                job_count = len(job_data['jobs'])
                print(f'  ✅ 成功取得 {job_count} 個運行中的作業')
            else:
                print(f'  ✅ 成功調用，但回應格式不同: {type(job_data)}')
            test_results.append(('get_running_jobs', True))
        else:
            print(f'  ❌ 失敗: {jobs.get("message")}')
            test_results.append(('get_running_jobs', False))
    except Exception as e:
        print(f'  ❌ 異常: {e}')
        test_results.append(('get_running_jobs', False))
    
    # 5. 測試 is_token_valid
    print('\n🔐 測試 is_token_valid():')
    try:
        is_valid = client.is_token_valid()
        print(f'  Token 有效性: {"✅ 有效" if is_valid else "❌ 無效"}')
        test_results.append(('is_token_valid', True))
    except Exception as e:
        print(f'  ❌ 異常: {e}')
        test_results.append(('is_token_valid', False))
    
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