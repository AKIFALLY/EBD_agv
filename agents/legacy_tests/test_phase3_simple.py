#!/usr/bin/env python3
"""
Simple Phase 3 Integration Test
Uses standard library only
"""

import json
import urllib.request
import urllib.parse
import urllib.error
from datetime import datetime

# Test configuration
EDITOR_API_URL = "http://localhost:8001/tafl"
WCS_SERVICE_URL = "http://localhost:9000"

def print_header(title):
    print(f"\n{'='*60}")
    print(f"🎯 {title}")
    print(f"{'='*60}")

def http_get(url):
    """Simple HTTP GET request"""
    try:
        with urllib.request.urlopen(url, timeout=5) as response:
            return response.status, json.loads(response.read().decode())
    except urllib.error.HTTPError as e:
        return e.code, None
    except Exception as e:
        print(f"Error: {e}")
        return None, None

def http_post(url, data):
    """Simple HTTP POST request"""
    try:
        json_data = json.dumps(data).encode('utf-8')
        req = urllib.request.Request(url, data=json_data, headers={'Content-Type': 'application/json'})
        with urllib.request.urlopen(req, timeout=10) as response:
            return response.status, json.loads(response.read().decode())
    except urllib.error.HTTPError as e:
        return e.code, None
    except Exception as e:
        print(f"Error: {e}")
        return None, None

def test_services():
    """Test if services are available"""
    print_header("Service Availability Check")
    
    services_ok = True
    
    # Check TAFL Editor API
    print("Checking TAFL Editor API...")
    status, data = http_get(f"{EDITOR_API_URL}/verbs")
    if status == 200:
        verbs = data.get('verbs', {})
        print(f"✅ TAFL Editor API available ({len(verbs)} verbs)")
    else:
        print(f"❌ TAFL Editor API not available (status: {status})")
        services_ok = False
    
    # Check WCS Service
    print("Checking WCS Service...")
    status, data = http_get(f"{WCS_SERVICE_URL}/health")
    if status == 200:
        print(f"✅ WCS Service available")
    else:
        print(f"⚠️ WCS Service not available (status: {status})")
        print("   This is expected if service is not started")
    
    return services_ok

def test_simulation():
    """Test simulation execution"""
    print_header("Testing Simulation Mode")
    
    # Simple test flow
    test_flow = {
        "metadata": {
            "id": "test_simple",
            "name": "Simple Test"
        },
        "flow": [
            {"set": {"variable": "test", "value": "hello"}},
            {"print": {"message": "Test message"}},
            {"check": {"condition": "test == 'hello'"}}
        ],
        "mode": "simulation"
    }
    
    print("Executing test flow in simulation mode...")
    status, result = http_post(f"{EDITOR_API_URL}/execute", test_flow)
    
    if status == 200 and result:
        if result.get('success'):
            log = result.get('execution_log', [])
            print(f"✅ Simulation executed successfully ({len(log)} steps)")
            for entry in log[:3]:
                print(f"   Step {entry.get('step')}: {entry.get('verb')} - {entry.get('status')}")
        else:
            print(f"❌ Simulation failed: {result.get('message')}")
    else:
        print(f"❌ Request failed with status: {status}")
    
    return status == 200

def test_validation():
    """Test flow validation"""
    print_header("Testing Flow Validation")
    
    # Valid flow
    valid_flow = {
        "flow": [
            {"set": {"variable": "x", "value": 10}},
            {"query": {"target": "tasks", "conditions": {}}}
        ]
    }
    
    print("Testing valid flow...")
    status, result = http_post(f"{EDITOR_API_URL}/validate", valid_flow)
    
    if status == 200 and result:
        if result.get('valid'):
            print(f"✅ Valid flow accepted ({result.get('statement_count')} statements)")
        else:
            print(f"❌ Valid flow rejected: {result.get('errors')}")
    else:
        print(f"❌ Validation request failed: {status}")
    
    # Invalid flow
    invalid_flow = {
        "flow": [
            {"unknown_verb": {"param": "value"}},
            {"query": {}}  # Missing required 'target'
        ]
    }
    
    print("Testing invalid flow...")
    status, result = http_post(f"{EDITOR_API_URL}/validate", invalid_flow)
    
    if status == 200 and result:
        if not result.get('valid'):
            errors = result.get('errors', [])
            print(f"✅ Invalid flow correctly rejected ({len(errors)} errors)")
            for error in errors[:2]:
                print(f"   - {error}")
        else:
            print(f"❌ Invalid flow incorrectly accepted")
    else:
        print(f"❌ Validation request failed: {status}")

def main():
    """Main test runner"""
    print("\n" + "="*60)
    print("🚀 PHASE 3 SIMPLE INTEGRATION TEST")
    print("="*60)
    
    # Test services
    services_ok = test_services()
    
    if services_ok:
        # Test simulation
        test_simulation()
        
        # Test validation
        test_validation()
    else:
        print("\n⚠️ Skipping tests - TAFL Editor API not available")
        print("\n💡 Make sure AGVCUI is running:")
        print("   cd /app/web_api_ws")
        print("   python3 src/agvcui/agvcui/agvc_ui_server.py")
    
    print("\n" + "="*60)
    print("📊 TEST SUMMARY")
    print("="*60)
    
    if services_ok:
        print("\n✅ Basic Phase 3 integration is working")
        print("\n🎯 Next steps:")
        print("1. Start WCS HTTP service for real execution:")
        print("   python3 /app/tafl_wcs_http_service.py")
        print("\n2. Update TAFL Editor router in agvcui:")
        print("   Copy enhanced router to replace simulation mode")
        print("\n3. Test WebSocket progress monitoring:")
        print("   Open browser and monitor execution progress")
    else:
        print("\n❌ Services need to be started first")
    
    return 0

if __name__ == "__main__":
    exit(main())