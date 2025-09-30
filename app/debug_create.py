#!/usr/bin/env python3
"""Debug the create issue"""

import json
import urllib.request
import yaml

def test_create_syntax():
    """Test different create syntaxes"""

    # Test 1: Simple create without parameters
    flow1 = """
metadata:
  id: "test_create_simple"

flow:
  - create:
      target: task
      data:
        work_id: 210001
        name: "Test Task"
        rack_id: 1
        location_id: 11
        room_id: 1
        priority: 5
        status_id: 1
      as: new_task

  - if:
      condition: "${new_task}"
      then:
        - notify:
            message: "✅ Simple create SUCCESS"
      else:
        - notify:
            message: "❌ Simple create FAILED"
"""

    # Test 2: Create with parameters as JSON object
    flow2 = """
metadata:
  id: "test_create_with_params"

flow:
  - create:
      target: task
      with:
        work_id: 210001
        name: "Test Task with Params"
        rack_id: 1
        location_id: 11
        room_id: 1
        priority: 5
        status_id: 1
        parameters: {"source_location_id": 11, "target_location_id": 10001}
      as: new_task

  - if:
      condition: "${new_task}"
      then:
        - notify:
            message: "✅ Create with params SUCCESS"
      else:
        - notify:
            message: "❌ Create with params FAILED"
"""

    # Test 3: Query then create with dynamic values
    flow3 = """
metadata:
  id: "test_dynamic_create"

flow:
  - query:
      target: racks
      where:
        id: 1
      limit: 1
      as: rack

  - if:
      condition: "${rack}"
      then:
        - create:
            target: task
            with:
              work_id: 210001
              name: "Dynamic Task"
              rack_id: "${rack[0].id}"
              location_id: "${rack[0].location_id}"
              room_id: 1
              priority: 5
              status_id: 1
            as: new_task

        - if:
            condition: "${new_task}"
            then:
              - notify:
                  message: "✅ Dynamic create SUCCESS"
            else:
              - notify:
                  message: "❌ Dynamic create FAILED"
"""

    flows = [
        ("Simple create (data)", flow1),
        ("Create with params (with)", flow2),
        ("Dynamic create", flow3)
    ]

    for name, flow_yaml in flows:
        print(f"\n{'='*60}")
        print(f"Testing: {name}")
        print('='*60)

        flow_obj = yaml.safe_load(flow_yaml)

        url = 'http://localhost:8001/tafl/execute'
        data = json.dumps({
            'flow': flow_obj.get('flow', []),
            'metadata': flow_obj.get('metadata', {}),
            'variables': flow_obj.get('variables', {}),
            'mode': 'real'
        }).encode('utf-8')

        headers = {'Content-Type': 'application/json'}
        req = urllib.request.Request(url, data=data, headers=headers)

        try:
            with urllib.request.urlopen(req, timeout=10) as response:
                result = json.loads(response.read().decode('utf-8'))

                print(f"Status: {result.get('status')}")

                if result.get('error'):
                    print(f"Error: {result.get('error')}")

                # Show notifications
                if 'execution_log' in result:
                    for log in result['execution_log']:
                        if log.get('verb') == 'notify':
                            print(f"  {log['action']['message']}")
                        elif log.get('verb') == 'create':
                            print(f"  Create result: {log.get('result')}")
                            if log.get('error'):
                                print(f"  Create error: {log.get('error')}")

        except Exception as e:
            print(f"❌ Error: {e}")

if __name__ == "__main__":
    print("CREATE SYNTAX DEBUG TEST")
    print("========================")
    test_create_syntax()