#!/usr/bin/env python3
"""
Simple TAFL integration test for flow_wcs
"""

import asyncio
import sys
sys.path.insert(0, '/app/flow_wcs_ws/src/flow_wcs')
sys.path.insert(0, '/app/tafl_ws/src/tafl')

from flow_wcs.flow_executor import execute_tafl, execute_hybrid_flow

# Simple TAFL test
TAFL_TEST = """
metadata:
  id: test_001
  name: Simple TAFL Test

variables:
  counter: 0
  message: ""

flow:
  - set: counter = 10
  - set: message = TAFL works!
  - if:
      condition: "${counter} > 5"
      then:
        - set: status = Success
"""

# Simple Linear Flow v2 test
LINEAR_FLOW_TEST = """
variables:
  counter:
    default: 0
  message:
    default: ""

flow:
  - id: "step1"
    exec: "flow.set_variable"
    params:
      name: "counter"
      value: 20
  
  - id: "step2"
    exec: "flow.set_variable"
    params:
      name: "message"
      value: "Linear Flow works!"
"""

async def main():
    print("=" * 60)
    print("Testing TAFL Integration in flow_wcs")
    print("=" * 60)
    
    # Test TAFL execution
    print("\n1. Testing TAFL execution...")
    try:
        result = await execute_tafl(TAFL_TEST)
        print(f"   ✅ TAFL result: counter={result.get('counter')}, message={result.get('message')}, status={result.get('status')}")
    except Exception as e:
        print(f"   ❌ TAFL failed: {e}")
    
    # Test hybrid execution with TAFL
    print("\n2. Testing hybrid execution with TAFL...")
    try:
        result = await execute_hybrid_flow(TAFL_TEST)
        print(f"   ✅ Hybrid (TAFL) result: counter={result.get('counter')}, message={result.get('message')}")
    except Exception as e:
        print(f"   ❌ Hybrid (TAFL) failed: {e}")
    
    # Test hybrid execution with Linear Flow v2
    print("\n3. Testing hybrid execution with Linear Flow v2...")
    try:
        result = await execute_hybrid_flow(LINEAR_FLOW_TEST)
        # FlowExecutor returns a context dict with 'variables' key
        if isinstance(result, dict) and 'variables' in result:
            counter = result['variables'].get('counter')
            message = result['variables'].get('message')
            print(f"   ✅ Hybrid (Linear) result: counter={counter}, message={message}")
        else:
            print(f"   ⚠️ Hybrid (Linear) returned unexpected format: {type(result)}")
    except Exception as e:
        print(f"   ❌ Hybrid (Linear) failed: {e}")
    
    print("\n" + "=" * 60)
    print("✨ TAFL Integration Test Complete!")
    print("=" * 60)

if __name__ == '__main__':
    asyncio.run(main())