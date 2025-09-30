#!/bin/bash
# Generate test data with some active alarms for demonstration
# Run this inside the AGV container

echo "📝 Generating test data with active alarms for loader02..."

cat > /tmp/update_loader02_alarms.py << 'PYTHON_SCRIPT'
#!/usr/bin/env python3
import json

# Load existing loader02 data
with open('/tmp/agv_status_loader02.json', 'r') as f:
    data = json.load(f)

# Add some active alarms for demonstration
# Let's activate alarms 1, 5, 12, 25, 48, 72 
data['alarms']['ALARM_STATUS_1'] = True
data['alarms']['ALARM_STATUS_5'] = True
data['alarms']['ALARM_STATUS_12'] = True
data['alarms']['ALARM_STATUS_25'] = True
data['alarms']['ALARM_STATUS_48'] = True
data['alarms']['ALARM_STATUS_72'] = True

# Also set AGV_ALARM to true
data['agv_status']['AGV_ALARM'] = True

# Save updated data
with open('/tmp/agv_status_loader02.json', 'w') as f:
    json.dump(data, f, indent=2)

print("✅ Updated loader02 with 6 active alarms:")
print("   ALARM_STATUS_1: True")
print("   ALARM_STATUS_5: True") 
print("   ALARM_STATUS_12: True")
print("   ALARM_STATUS_25: True")
print("   ALARM_STATUS_48: True")
print("   ALARM_STATUS_72: True")
print("   AGV_ALARM: True")

# Also update the main agv_status.json if it's using loader02 data
# or create a copy with alarms for testing
with open('/tmp/agv_status_loader02_with_alarms.json', 'w') as f:
    json.dump(data, f, indent=2)
    
print("\n✅ Also saved as /tmp/agv_status_loader02_with_alarms.json for testing")
PYTHON_SCRIPT

python3 /tmp/update_loader02_alarms.py

echo ""
echo "📊 Verification:"
python3 -c "
import json
with open('/tmp/agv_status_loader02.json', 'r') as f:
    data = json.load(f)
active = [k for k,v in data['alarms'].items() if v == True]
print(f'Active alarms in loader02: {len(active)} alarms')
print(f'AGV_ALARM status: {data[\"agv_status\"][\"AGV_ALARM\"]}')
"