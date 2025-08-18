# KUKA AGV Rack Rotation Integration

## 🎯 Use Cases
- Implementing rack rotation tasks for KUKA AGV systems
- Understanding KUKA-specific navigation parameters
- Configuring proper task parameters for rack rotation
- Managing room-based task assignment

## 📋 Core Concepts

### KUKA AGV Rotation Requirements
KUKA AGV systems require specific parameters for executing rack rotation tasks, including a navigation model identifier and a precise node sequence for the rotation maneuver.

### Key Components
- **Model Identifier**: `KUKA400i` for rack rotation operations
- **Node Sequence**: 3-point navigation pattern
- **Room Assignment**: Task association with specific room entrances/exits
- **Parameter Inheritance**: Work parameters merged with task metadata

## 🔧 Implementation Details

### KUKA Navigation Parameters
```yaml
# Required parameters for KUKA AGV rack rotation
metadata:
  model: "KUKA400i"              # KUKA AGV model identifier
  nodes:                         # 3-point rotation pattern
    - "${start_node}"           # Entry point (inlet)
    - "${turning_node}"         # Rotation point (typically node_id + 1)
    - "${end_node}"             # Exit point (same as entry for rotation)
```

### Three-Point Rotation Pattern
The KUKA AGV uses a specific 3-point pattern for rack rotation:

```
Point 1 (Start)    Point 2 (Turn)    Point 3 (End)
     [A] ————————————> [B]
      ↑                 ↓
      |                 |
      ↑                 ↓
      [A] <———————————— [B]
      
Where: Point 1 == Point 3 (same physical location)
       Point 2 = Turning point for rotation maneuver
```

### Task Creation with KUKA Parameters
```python
def create_rack_rotation_task(location, rack, work):
    """Create a rack rotation task with KUKA parameters"""
    
    # Calculate navigation nodes
    start_node = location.node_id
    turning_node = location.node_id + 1  # Typically next sequential node
    end_node = location.node_id          # Return to start point
    
    # Prepare KUKA-specific metadata
    metadata = {
        "model": "KUKA400i",
        "nodes": [start_node, turning_node, end_node],
        "rotation_angle": 180,
        "rack_name": rack.rack_id,
        "location_name": location.name,
        "room_id": location.room_id,
        "reason": "A面完成，B面待作業"
    }
    
    # Merge with work parameters
    task_parameters = {}
    if work.parameters:
        task_parameters.update(work.parameters)
    task_parameters.update(metadata)
    
    return create_task(
        type="RACK_ROTATION",
        work_id=work.id,          # Use integer work ID
        location_id=location.id,
        rack_id=rack.id,
        room_id=location.room_id,  # Assign to room
        parameters=task_parameters
    )
```

## 💡 Best Practices

### Parameter Configuration
1. **Always Include Model**: The `model` field is mandatory for KUKA system recognition
2. **Validate Node Sequence**: Ensure nodes array has exactly 3 elements
3. **Match Start and End**: First and third nodes must be identical
4. **Calculate Turn Point**: Turn point should be reachable from start point

### Room Assignment
1. **Identify Room Context**: Each task should know its room association
2. **Use Room ID**: Include `room_id` in both params and metadata
3. **Track Entry/Exit**: Distinguish between room entry and exit tasks
4. **Route Planning**: Room ID helps in optimizing task routing

### Parameter Merging
1. **Preserve Work Parameters**: Always start with work.parameters as base
2. **Override with Metadata**: Task-specific metadata can override work defaults
3. **Document Overrides**: Clear comments when overriding work parameters
4. **Validate Final Parameters**: Ensure all required fields are present

## 📊 Real Examples

### Complete YAML Configuration
```yaml
- id: "create_rotation_task"
  exec: "task.create_task"
  params:
    type: "RACK_ROTATION"
    work_id: "220001"                    # Integer work ID
    location_id: "${location.id}"
    rack_id: "${racks_at_location[0].id}"
    room_id: "${location.room_id}"       # Room assignment
    priority: "high"
    metadata:
      rack_name: "${racks_at_location[0].rack_id}"
      location_name: "${location.name}"
      room_id: "${location.room_id}"
      rotation_angle: 180
      reason: "A面完成，B面待作業"
      model: "KUKA400i"                  # KUKA model
      nodes: [                           # 3-point pattern
        "${location.node_id}",
        "${location.node_id + 1}",
        "${location.node_id}"
      ]
```

### Actual Task Examples
```python
# Task for Rack 101 at Room 1 entrance
{
    "type": "RACK_ROTATION",
    "work_id": 220001,
    "location_id": 1,
    "rack_id": 101,
    "room_id": 1,
    "parameters": {
        # From work parameters
        "speed": 1.5,
        "safety_distance": 0.5,
        # From metadata (overrides/additions)
        "model": "KUKA400i",
        "nodes": [10001, 10002, 10001],
        "rotation_angle": 180,
        "rack_name": "RACK_101",
        "location_name": "Room1_Entrance",
        "reason": "A面完成，B面待作業"
    }
}

# Task for Rack 105 at Room 5 entrance
{
    "type": "RACK_ROTATION",
    "work_id": 220001,
    "location_id": 5,
    "rack_id": 105,
    "room_id": 5,
    "parameters": {
        "model": "KUKA400i",
        "nodes": [50001, 50002, 50001],
        "rotation_angle": 180,
        "rack_name": "RACK_105",
        "location_name": "Room5_Entrance",
        "reason": "A面完成，B面待作業"
    }
}
```

## 🚨 Common Issues and Solutions

### Issue: Missing KUKA Model Parameter
**Problem**: Task rejected by KUKA system due to missing model identifier
**Solution**: Always include `model: "KUKA400i"` in metadata

### Issue: Invalid Node Sequence
**Problem**: KUKA AGV cannot execute rotation with incorrect node pattern
**Solution**: Ensure exactly 3 nodes with first == third node

### Issue: Work ID Confusion
**Problem**: Using work_code string instead of work.id integer
**Solution**: Always use `work.id` (integer) for task.work_id field

### Issue: Lost Work Parameters
**Problem**: Task missing important work configuration parameters
**Solution**: Merge work.parameters first, then add/override with metadata

### Issue: Room Assignment Missing
**Problem**: Tasks cannot be properly routed without room context
**Solution**: Include room_id in both task creation params and metadata

## 🔗 Cross References
- Linear Flow Advanced Features: @docs-ai/knowledge/system/linear-flow-advanced-features.md
- Flow WCS System: @docs-ai/knowledge/system/flow-wcs-system.md
- KUKA Fleet API: @docs-ai/knowledge/protocols/kuka-fleet-api.md
- WCS Database Design: @docs-ai/knowledge/agv-domain/wcs-database-design.md
- Troubleshooting Cases: @docs-ai/operations/development/linear-flow-troubleshooting-cases.md