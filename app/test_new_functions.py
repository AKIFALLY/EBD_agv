#!/usr/bin/env python3
import sys
sys.path.insert(0, '/app/flow_wcs_ws/src/flow_wcs')

from flow_wcs.flow_executor import FlowExecutor

def test_new_functions():
    print('=' * 80)
    print('Testing new function registration')
    print('=' * 80)
    
    executor = FlowExecutor()
    new_functions = ['action.send_alarm', 'action.trigger_event', 'query.rooms']
    
    print('\nChecking new functions:')
    all_registered = True
    
    for func_name in new_functions:
        if func_name in executor.functions:
            print(f'  ✅ {func_name} - Registered')
        else:
            print(f'  ❌ {func_name} - NOT registered')
            all_registered = False
    
    print(f'\nTotal registered functions: {len(executor.functions)}')
    
    if all_registered:
        print('\n✅ All new functions successfully registered!')
    else:
        print('\n❌ Some functions not registered')
    
    return 0 if all_registered else 1

if __name__ == '__main__':
    sys.exit(test_new_functions())
