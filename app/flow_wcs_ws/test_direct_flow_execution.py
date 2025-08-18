#!/usr/bin/env python3
"""
Test direct flow execution using FlowExecutor
"""

import sys
import asyncio
from pathlib import Path

# Add flow_wcs to path
flow_wcs_path = Path("/app/flow_wcs_ws/src/flow_wcs")
if str(flow_wcs_path) not in sys.path:
    sys.path.insert(0, str(flow_wcs_path))

async def test_direct_execution():
    print("=== 測試直接流程執行 ===\n")
    
    try:
        from flow_wcs.flow_executor import FlowExecutor
        
        # Test simple flow
        simple_flow = {
            'meta': {
                'system': 'linear_flow_v2',
                'version': '2.0.0'
            },
            'flow': {
                'id': 'test_simple',
                'name': 'Simple Test Flow',
                'work_id': '999999'
            },
            'workflow': [
                {
                    'section': 'Test Section',
                    'steps': [
                        {
                            'id': 'test_log',
                            'exec': 'action.log_message',
                            'params': {
                                'message': 'Simple test successful!',
                                'level': 'info'
                            }
                        },
                        {
                            'id': 'test_count',
                            'exec': 'control.count_items',
                            'params': {
                                'variable': 'test_items'
                            },
                            'store': 'count_result'
                        }
                    ]
                }
            ],
            'variables': {
                'test_items': [1, 2, 3, 4, 5]
            }
        }
        
        print("執行簡單測試流程...")
        executor = FlowExecutor(simple_flow)
        result = await executor.execute()
        
        print(f"✅ 流程執行成功!")
        print(f"狀態: {result['status']}")
        print(f"變數: {result['variables']}")
        print(f"日誌數量: {len(result['logs'])}")
        
        print("\n" + "="*50 + "\n")
        
        # Test foreach flow  
        foreach_flow = {
            'meta': {
                'system': 'linear_flow_v2',
                'version': '2.0.0'
            },
            'flow': {
                'id': 'test_foreach',
                'name': 'Foreach Test Flow',
                'work_id': '999999'
            },
            'workflow': [
                {
                    'section': 'Foreach Test',
                    'steps': [
                        {
                            'id': 'test_foreach',
                            'exec': 'foreach',
                            'params': {
                                'items': ['item1', 'item2', 'item3'],
                                'var': 'current_item',
                                'steps': [
                                    {
                                        'id': 'log_item',
                                        'exec': 'action.log_message',
                                        'params': {
                                            'message': 'Processing item: ${current_item}',
                                            'level': 'info'
                                        }
                                    }
                                ]
                            }
                        }
                    ]
                }
            ]
        }
        
        print("執行 foreach 測試流程...")
        executor = FlowExecutor(foreach_flow)
        result = await executor.execute()
        
        print(f"✅ Foreach 流程執行成功!")
        print(f"狀態: {result['status']}")
        print(f"日誌:")
        for log in result['logs']:
            if log['level'] == 'info':
                print(f"  - {log['message']}")
        
    except Exception as e:
        print(f"❌ 執行出錯: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    asyncio.run(test_direct_execution())