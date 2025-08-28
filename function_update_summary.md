# Flow WCS Function Update Summary

## Date: 2025-08-19

### Problem Fixed
1. **control.switch function not found** - The function was manually registered but not exposed through the decorator system
2. **query.racks array parameter issue** - Fixed to handle both single values and arrays

### Changes Made

#### 1. Added Missing Functions to flow_executor.py
- `action.send_alarm` - Send system alarms
- `action.trigger_event` - Trigger system events  
- `query.rooms` - Query available rooms

#### 2. Fixed query_racks to Handle Arrays
Modified the function to iterate through array of location_ids:
```python
if isinstance(location_id, list):
    all_racks = []
    for loc_id in location_id:
        if isinstance(loc_id, str):
            try:
                loc_id = int(loc_id)
            except ValueError:
                continue
        racks = self.db.query_racks(location_id=loc_id, status=status)
        all_racks.extend(racks)
    return all_racks
```

#### 3. Exposed control.switch via Decorator
Added `also_register_as="switch"` to the @flow_function decorator:
```python
@flow_function("control", "進階條件分支控制", ["value", "cases"], "any",
               defaults={"value": 0, "cases": []}, 
               also_register_as="switch")
def switch_advanced(self, params: Dict) -> Any:
```

### Build and Deployment
```bash
# Built flow_wcs package
colcon build --packages-select flow_wcs

# Restarted Web API Launch service
manage_web_api_launch stop
manage_web_api_launch start
```

### Verification
✅ All 8 business process flows load successfully in Linear Flow Designer
✅ control.switch_advanced available in control category
✅ switch available in special category (accessible as control.switch)
✅ Functions properly exposed through API at /api/flow/functions

### Files Modified
- `/home/ct/RosAGV/app/flow_wcs_ws/src/flow_wcs/flow_wcs/flow_executor.py`

### Testing
- Playwright browser automation test: All flows load successfully
- API function availability test: Functions properly registered and exposed
