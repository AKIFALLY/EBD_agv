#!/usr/bin/env python3
import sys
sys.path.insert(0, '/app/flow_wcs_ws/src/flow_wcs')

from flow_wcs.flow_executor import FlowExecutor

def test_control_switch():
    executor = FlowExecutor()
    
    if 'control.switch' in executor.functions:
        print('✅ control.switch is registered')
        result = executor.functions['control.switch']({
            'value': 'urgent',
            'cases': {'urgent': 100, 'normal': 50, 'low': 10},
            'default': 0
        })
        print(f'   Test result: {result}')
    else:
        print('❌ control.switch is NOT registered')
        print('   Available control functions:')
        for name in sorted(executor.functions.keys()):
            if name.startswith('control.'):
                print(f'     - {name}')

def test_query_racks_array():
    executor = FlowExecutor()
    
    print('\nTesting query.racks with array parameter:')
    
    class MockDB:
        def query_racks(self, location_id=None, status=None):
            print(f'   DB query: location_id={location_id}, status={status}')
            return [{'id': 1, 'rack_id': f'RACK_{location_id}', 'status': status}]
    
    executor.db = MockDB()
    
    try:
        result = executor.functions['query.racks']({
            'location_id': [41, 42, 43],
            'status': 'available'
        })
        print('✅ Array parameter handled successfully')
        print(f'   Result: {result}')
    except Exception as e:
        print(f'❌ Error with array parameter: {e}')

print('=== Testing Flow Executor Functions ===\n')
test_control_switch()
test_query_racks_array()
print('\n=== Test Complete ===')
