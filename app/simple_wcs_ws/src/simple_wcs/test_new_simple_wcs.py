#!/usr/bin/env python3
"""
測試新的 Simple WCS 實現
"""

import sys
import asyncio
import logging
from pathlib import Path

# Add the package to path for testing
sys.path.insert(0, '/home/ct/RosAGV/app/simple_wcs_ws/src/simple_wcs')

from simple_wcs.flow_loader import FlowLoader
from simple_wcs.parallel_flow_executor import ParallelFlowExecutor
from simple_wcs.wcs_functions import WCSFunctions, register_functions_to_executor


def test_flow_loader():
    """測試流程載入器"""
    print("\n" + "="*50)
    print("測試流程載入器")
    print("="*50)
    
    loader = FlowLoader('/home/ct/RosAGV/app/config/wcs/flows')
    flows = loader.load_all_flows()
    
    print(f"\n載入了 {len(flows)} 個流程:")
    for name, flow in flows.items():
        print(f"  - {name}")
        print(f"    優先級: {flow.priority}")
        print(f"    工作ID: {flow.work_id}")
        print(f"    節點數: {flow.node_count} (條件:{flow.condition_nodes}, 動作:{flow.action_nodes}, 邏輯:{flow.logic_nodes})")
        print(f"    連接數: {flow.connection_count}")
        print(f"    啟用: {flow.enabled}")
        print()
    
    return flows


def test_wcs_functions():
    """測試 WCS 函數"""
    print("\n" + "="*50)
    print("測試 WCS 函數")
    print("="*50)
    
    wcs_funcs = WCSFunctions()
    
    # 測試條件檢查函數
    print("\n條件檢查函數:")
    result = wcs_funcs.check_rack_side_status(side='A', check_type='completion')
    print(f"  check_rack_side_status(A, completion): {result}")
    
    result = wcs_funcs.check_rack_has_carrier(side='B')
    print(f"  check_rack_has_carrier(B): {result}")
    
    result = wcs_funcs.check_task_not_exists(task_type='rotation', location_id=10001)
    print(f"  check_task_not_exists(rotation, 10001): {result}")
    
    # 測試邏輯函數
    print("\n邏輯函數:")
    result = wcs_funcs.and_gate(True, False)
    print(f"  and_gate(True, False): {result}")
    
    result = wcs_funcs.or_gate(True, False)
    print(f"  or_gate(True, False): {result}")
    
    result = wcs_funcs.not_gate(True)
    print(f"  not_gate(True): {result}")
    
    # 測試動作函數
    print("\n動作函數:")
    task = wcs_funcs.create_rotation_task(location_id=10021)
    print(f"  create_rotation_task(10021): {task['task_id']}")
    
    return wcs_funcs


async def test_parallel_executor(flows):
    """測試並行執行器"""
    print("\n" + "="*50)
    print("測試並行執行器")
    print("="*50)
    
    executor = ParallelFlowExecutor()
    
    # 註冊 WCS 函數
    wcs_funcs = WCSFunctions()
    register_functions_to_executor(executor, wcs_funcs)
    
    # 也註冊內建函數
    executor.register_builtin_functions()
    
    # 執行所有流程
    enabled_flows = [f for f in flows.values() if f.enabled]
    print(f"\n準備執行 {len(enabled_flows)} 個啟用的流程...")
    
    results = await executor.execute_flows_parallel(enabled_flows)
    
    print(f"\n執行結果:")
    print(f"  總流程數: {results['total_flows']}")
    print(f"  執行數: {results['executed']}")
    print(f"  成功數: {results['successful']}")
    print(f"  失敗數: {results['failed']}")
    
    print(f"\n各流程結果:")
    for flow_name, result in results['results'].items():
        print(f"  {flow_name}:")
        print(f"    狀態: {result.get('status')}")
        if result.get('duration'):
            print(f"    耗時: {result['duration']:.3f}秒")
        if result.get('executed_nodes'):
            print(f"    執行節點: {result['executed_nodes']}/{result.get('total_nodes', 0)}")


def main():
    """主測試函數"""
    # 設定日誌
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    
    print("\n" + "="*70)
    print("Simple WCS 新架構測試")
    print("="*70)
    
    # 測試流程載入
    flows = test_flow_loader()
    
    if not flows:
        print("\n⚠️ 沒有載入到任何流程，請檢查流程檔案")
        return
    
    # 測試 WCS 函數
    test_wcs_functions()
    
    # 測試並行執行
    print("\n開始測試並行執行...")
    asyncio.run(test_parallel_executor(flows))
    
    print("\n" + "="*70)
    print("測試完成！")
    print("="*70)
    print("\n新的 Simple WCS 架構特點:")
    print("  ✅ 只支援統一的 YAML 格式")
    print("  ✅ 所有流程並行執行")
    print("  ✅ 基於資料流的節點執行")
    print("  ✅ 完整的 ROS 2 Node 支援")
    print("  ✅ 可用 ros2 launch 或 ros2 run 啟動")


if __name__ == "__main__":
    main()