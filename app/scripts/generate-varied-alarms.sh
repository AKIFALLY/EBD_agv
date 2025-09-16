#!/bin/bash
# Generate test data with varied alarm patterns for better UI testing
# Run this inside the AGV container

echo "📝 Generating test data with varied alarm patterns..."

cat > /tmp/generate_varied_alarms.py << 'PYTHON_SCRIPT'
#!/usr/bin/env python3
import json
import random

def update_agv_alarms(filename, agv_id, alarm_pattern):
    """Update AGV test data with specific alarm pattern"""
    try:
        with open(filename, 'r') as f:
            data = json.load(f)
    except:
        print(f"❌ Could not read {filename}")
        return
    
    # Clear all alarms first
    for i in range(1, 97):
        data['alarms'][f'ALARM_STATUS_{i}'] = False
    
    # Apply alarm pattern
    if alarm_pattern == "none":
        data['agv_status']['AGV_ALARM'] = False
    elif alarm_pattern == "few":
        # Activate 3-5 random alarms
        alarm_indices = random.sample(range(1, 97), random.randint(3, 5))
        for idx in alarm_indices:
            data['alarms'][f'ALARM_STATUS_{idx}'] = True
        data['agv_status']['AGV_ALARM'] = True
    elif alarm_pattern == "many":
        # Activate 10-15 random alarms
        alarm_indices = random.sample(range(1, 97), random.randint(10, 15))
        for idx in alarm_indices:
            data['alarms'][f'ALARM_STATUS_{idx}'] = True
        data['agv_status']['AGV_ALARM'] = True
    elif alarm_pattern == "critical":
        # Activate specific critical alarms (1, 2, 3, 10, 20, 30)
        critical = [1, 2, 3, 10, 20, 30, 40, 50, 60, 70, 80, 90]
        for idx in critical:
            data['alarms'][f'ALARM_STATUS_{idx}'] = True
        data['agv_status']['AGV_ALARM'] = True
    elif alarm_pattern == "sequential":
        # Activate first 10 alarms
        for i in range(1, 11):
            data['alarms'][f'ALARM_STATUS_{i}'] = True
        data['agv_status']['AGV_ALARM'] = True
    
    # Save updated data
    with open(filename, 'w') as f:
        json.dump(data, f, indent=2)
    
    # Count active alarms
    active = sum(1 for k, v in data['alarms'].items() if v == True)
    print(f"✅ {agv_id}: {active} active alarms (pattern: {alarm_pattern})")
    
    return active

# Update each AGV with different alarm patterns
patterns = {
    '/tmp/agv_status_loader01.json': ('loader01', 'few'),      # Few alarms
    '/tmp/agv_status_loader02.json': ('loader02', 'sequential'), # Sequential alarms
    '/tmp/agv_status_cargo01.json': ('cargo01', 'none'),       # No alarms
    '/tmp/agv_status_unloader01.json': ('unloader01', 'critical'), # Critical alarms
    '/tmp/agv_status_unloader02.json': ('unloader02', 'many')   # Many alarms
}

print("Updating AGV test data with varied alarm patterns:")
print("="*50)

for filename, (agv_id, pattern) in patterns.items():
    update_agv_alarms(filename, agv_id, pattern)

print("\n📊 Summary of alarm patterns:")
print("  loader01: Few random alarms (3-5)")
print("  loader02: Sequential alarms (1-10)")
print("  cargo01: No alarms")
print("  unloader01: Critical alarms (specific important ones)")
print("  unloader02: Many random alarms (10-15)")

# Also copy loader02 to main status file for immediate testing
import shutil
shutil.copy('/tmp/agv_status_loader02.json', '/tmp/agv_status.json')
print("\n✅ Copied loader02 to /tmp/agv_status.json for testing")
PYTHON_SCRIPT

python3 /tmp/generate_varied_alarms.py

echo ""
echo "📊 Verification of alarm states:"
for agv in loader01 loader02 cargo01 unloader01 unloader02; do
    python3 -c "
import json
with open('/tmp/agv_status_${agv}.json', 'r') as f:
    data = json.load(f)
active = [int(k.split('_')[-1]) for k,v in data['alarms'].items() if v == True]
active.sort()
if active:
    print(f'${agv}: {len(active)} alarms active - [{active[0]}..{active[-1]}]')
else:
    print(f'${agv}: No alarms active')
"
done