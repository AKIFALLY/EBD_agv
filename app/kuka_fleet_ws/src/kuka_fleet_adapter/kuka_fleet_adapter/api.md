# KUKA Fleet API Documentation

**Version**: 2.13.0  
**Host**: 192.168.10.3:10870  
**Base URL**: http://192.168.10.3:10870

## Authentication

### Login
- **Endpoint**: `POST /api/login`
- **Description**: User login to get access token
- **Request Body**:
```json
{
    "username": "admin",
    "password": "Admin"
}
```
- **Response**:
```json
{
    "code": "SUCCESS",
    "data": {
        "token": "your_access_token",
        "expiresIn": 7200,
        "tokenType": "Bearer"
    },
    "msg": "Login successful",
    "success": true
}
```
- **Token Usage**: Include token in subsequent requests:
  - **Header**: `Authorization: <token>` (不需要 Bearer 前綴)
  - **Expiration**: Token expires in 2 hours (7200 seconds)
  - **Refresh**: Re-login when token expires

## Mission Related APIs

### Query Jobs
- **Endpoint**: `POST /api/amr/jobQuery`
- **Description**: Query AMR job list with filtering conditions
- **Request Body**:
```json
{
    "containerCode": "",
    "createUsername": "",
    "jobCode": "",
    "limit": 0,
    "maps": [],
    "robotId": "",
    "sourceValue": 0,
    "status": 0,
    "targetCellCode": "",
    "workflowCode": "",
    "workflowId": 0,
    "workflowName": ""
}
```
- **Parameters**:
  - `sourceValue`: 0=All sources, 1=Manual, 2=Auto, 3=External
  - `status`: 0=All, 1=Pending, 2=Running, 3=Completed, 4=Failed, 5=Cancelled
  - `limit`: Maximum number of results (0=no limit)
- **Response**:
```json
{
    "code": "SUCCESS",
    "data": {
        "jobs": [
            {
                "jobId": "12345",
                "jobCode": "JOB001",
                "status": 2,
                "priority": 50,
                "robotId": "AGV001",
                "containerCode": "CONT001",
                "workflowCode": "WF001",
                "workflowName": "Transport Workflow",
                "createTime": "2024-01-15T10:30:00Z",
                "startTime": "2024-01-15T10:35:00Z",
                "endTime": null,
                "progress": 65
            }
        ],
        "total": 25,
        "page": 1,
        "pageSize": 20,
        "hasNext": true
    },
    "msg": "Query successful",
    "success": true
}
```

### Cancel Mission
- **Endpoint**: `POST /api/amr/missionCancel`
- **Description**: Cancel executing AMR mission
- **Request Body**:
```json
{
    "cancelMode": "FORCE",
    "containerCode": "",
    "missionCode": "",
    "position": "",
    "reason": "",
    "requestId": ""
}
```
- **Parameters**:
  - `cancelMode`: "NORMAL" (finish current task), "FORCE" (immediate stop)
- **Response**:
```json
{
    "code": "SUCCESS",
    "data": {
        "missionCode": "MISSION001",
        "status": "CANCELLED",
        "cancelTime": "2024-01-15T10:30:00Z"
    },
    "msg": "Mission cancelled successfully",
    "success": true
}
```

### Operation Feedback
- **Endpoint**: `POST /api/amr/operationFeedback`
- **Description**: Release mission to continue after node operation completion
- **Request Body**:
```json
{
    "containerCode": "",
    "missionCode": "",
    "position": "",
    "requestId": ""
}
```

### Pause Mission
- **Endpoint**: `POST /api/amr/pauseMission`
- **Description**: Pause executing mission
- **Parameters**: 
  - `missionCode` (string, required): Mission code to pause

### Recover Mission
- **Endpoint**: `POST /api/amr/recoverMission`
- **Description**: Recover paused mission
- **Parameters**: 
  - `missionCode` (string, required): Mission code to recover

### Redirect Mission
- **Endpoint**: `POST /api/amr/redirectMission`
- **Description**: Redirect mission to new target node
- **Request Body**:
```json
{
    "missionCode": "",
    "requestId": "",
    "targetPosition": ""
}
```

### Submit Mission
- **Endpoint**: `POST /api/amr/submitMission`
- **Description**: Submit new AMR mission
- **Request Body**:
```json
{
    "orgId": "",
    "requestId": "",
    "missionCode": "",
    "missionType": "TRANSPORT",
    "viewBoardType": "",
    "robotModels": [],
    "robotIds": [],
    "robotType": "",
    "priority": 0,
    "containerModelCode": "",
    "containerCode": "",
    "templateCode": "",
    "lockRobotAfterFinish": true,
    "unlockRobotId": "",
    "unlockMissionCode": "",
    "idleNode": "",
    "missionData": [
        {
            "taskType": "PICK",
            "sourcePosition": "A001",
            "targetPosition": "B001",
            "containerModelCode": "PALLET_1200",
            "priority": 50,
            "parameters": {
                "waitTime": 5,
                "retryCount": 3
            }
        }
    ]
}
```
- **Parameters**:
  - `missionType`: "TRANSPORT", "PICK", "DROP", "CHARGE", "MAINTENANCE"
  - `priority`: 0-100 (higher number = higher priority)
  - `missionData`: Array of task objects with detailed execution steps
- **Response**:
```json
{
    "code": "SUCCESS",
    "data": {
        "missionId": "MISSION_12345",
        "missionCode": "MISSION001",
        "status": "SUBMITTED",
        "estimatedDuration": 300
    },
    "msg": "Mission submitted successfully",
    "success": true
}
```

## Map Node and Area APIs

### Query Area Nodes
- **Endpoint**: `POST /api/amr/areaNodesQuery`
- **Description**: Query nodes within specified areas
- **Request Body**:
```json
{
    "areaCodes": ["area1", "area2"]
}
```

### Query All Areas
- **Endpoint**: `GET /api/amr/areaQuery`
- **Description**: Get all WCS area information

### Query All Forbidden Areas
- **Endpoint**: `GET /api/amr/queryAllForbiddenAreas`
- **Description**: Get all forbidden areas information

### Query Function Node
- **Endpoint**: `POST /api/amr/queryFunctionNode`
- **Description**: Query function nodes by conditions
- **Request Body**:
```json
{
    "containerModel": "",
    "floorNumber": "",
    "functionType": 0,
    "mapCode": "",
    "robotTypeCode": ""
}
```
- **Parameters**:
  - `functionType`: 0=All, 1=Pick, 2=Drop, 3=Charge, 4=Maintenance, 5=Waiting
- **Response**:
```json
{
    "code": "SUCCESS",
    "data": [
        {
            "nodeCode": "PICK_001",
            "nodeName": "Pick Station 1",
            "functionType": 1,
            "functionTypeName": "PICK",
            "position": {
                "x": 10.5,
                "y": 5.2,
                "angle": 0.0
            },
            "mapCode": "WAREHOUSE_L1",
            "floorNumber": "1",
            "isAvailable": true,
            "supportedContainerModels": ["PALLET_1200"],
            "supportedRobotTypes": ["CARGO_AGV", "LOADER_AGV"]
        }
    ],
    "msg": "Query successful",
    "success": true
}
```

### Query One Forbidden Area
- **Endpoint**: `GET /api/amr/queryOneForbiddenArea`
- **Description**: Get specific forbidden area details
- **Parameters**: 
  - `forbiddenAreaCode` (string, optional): Forbidden area code
  - `forbiddenAreaId` (integer, optional): Forbidden area ID

### Query Area by Map Node
- **Endpoint**: `GET /api/amr/queryWCSAreaByMapNode`
- **Description**: Query area information by node UUID
- **Parameters**: 
  - `nodeUuid` (string, required): Node UUID

### Update Forbidden Area Status
- **Endpoint**: `POST /api/amr/updateForbiddenAreaStatus`
- **Description**: Update forbidden area enable/disable status
- **Request Body**:
```json
{
    "forbiddenAreaCode": "",
    "forbiddenAreaId": 0,
    "status": "0"
}
```

## Container Related APIs

### Container In
- **Endpoint**: `POST /api/amr/containerIn`
- **Description**: Insert container into map
- **Request Body**:
```json
{
    "containerCode": "",
    "containerModelCode": "",
    "containerValidationCode": "",
    "enterOrientation": "",
    "isNew": true,
    "position": "",
    "requestId": "",
    "withDefaultValidationCode": true
}
```

### Container Out
- **Endpoint**: `POST /api/amr/containerOut`
- **Description**: Remove container from map
- **Request Body**:
```json
{
    "containerCode": "",
    "containerType": "",
    "isDelete": true,
    "position": "",
    "requestId": ""
}
```

### Query Containers (In Map Only)
- **Endpoint**: `POST /api/amr/containerQuery`
- **Description**: Query containers that are currently in map
- **Request Body**:
```json
{
    "areaCode": "",
    "containerCode": "",
    "containerModelCode": "",
    "emptyFullStatus": 0,
    "nodeCode": ""
}
```
- **Parameters**:
  - `emptyFullStatus`: 0=All, 1=Empty, 2=Full, 3=Unknown
- **Response**:
```json
{
    "code": "SUCCESS",
    "data": [
        {
            "containerCode": "CONT001",
            "containerModelCode": "PALLET_1200",
            "position": "A001",
            "emptyFullStatus": 2,
            "emptyFullStatusName": "FULL",
            "inMapStatus": 1,
            "areaCode": "STORAGE_A",
            "nodeCode": "A001",
            "weight": 250.5,
            "lastUpdateTime": "2024-01-15T10:30:00Z"
        }
    ],
    "msg": "Query successful",
    "success": true
}
```

### Query All Containers
- **Endpoint**: `POST /api/amr/containerQueryAll`
- **Description**: Query all containers (both in and out of map)
- **Request Body**:
```json
{
    "areaCode": "",
    "containerCode": "",
    "containerModelCode": "",
    "districtCode": "",
    "emptyFullStatus": 0,
    "inMapStatus": 0,
    "mapCode": "",
    "nodeCode": ""
}
```
- **Parameters**:
  - `emptyFullStatus`: 0=All, 1=Empty, 2=Full, 3=Unknown
  - `inMapStatus`: 0=All, 1=In Map, 2=Out of Map
- **Response**:
```json
{
    "code": "SUCCESS",
    "data": [
        {
            "containerCode": "CONT001",
            "containerModelCode": "PALLET_1200",
            "position": "A001",
            "emptyFullStatus": 2,
            "emptyFullStatusName": "FULL",
            "inMapStatus": 1,
            "inMapStatusName": "IN_MAP",
            "areaCode": "STORAGE_A",
            "districtCode": "DIST_001",
            "mapCode": "WAREHOUSE_L1",
            "nodeCode": "A001",
            "weight": 250.5,
            "createTime": "2024-01-15T09:00:00Z",
            "lastUpdateTime": "2024-01-15T10:30:00Z"
        }
    ],
    "total": 150,
    "msg": "Query successful",
    "success": true
}
```

### Query All Container Models
- **Endpoint**: `GET /api/amr/queryAllContainerModelCode`
- **Description**: Get all available container model codes
- **Response**:
```json
{
    "code": "SUCCESS",
    "data": [
        {
            "containerModelCode": "PALLET_1200",
            "containerModelName": "Standard Pallet 1200x800",
            "dimensions": {
                "length": 1200,
                "width": 800,
                "height": 150
            },
            "maxWeight": 500.0,
            "supportedRobotTypes": ["CARGO_AGV", "LOADER_AGV"]
        },
        {
            "containerModelCode": "BOX_SMALL",
            "containerModelName": "Small Box Container",
            "dimensions": {
                "length": 400,
                "width": 300,
                "height": 200
            },
            "maxWeight": 50.0,
            "supportedRobotTypes": ["CARGO_AGV"]
        }
    ],
    "msg": "Query successful",
    "success": true
}
```

### Query Areas for Container Model
- **Endpoint**: `GET /api/amr/queryAreaCodeForContainerModel`
- **Description**: Query storage areas for specific container model
- **Parameters**: 
  - `containerModelCode` (string, required): Container model code
  - `noContainerFirst` (boolean, optional): Priority for empty areas
- **Response**:
```json
{
    "code": "SUCCESS",
    "data": [
        {
            "areaCode": "STORAGE_A",
            "areaName": "Storage Area A",
            "availableSlots": 25,
            "totalSlots": 50,
            "occupancyRate": 0.5,
            "priority": 1
        },
        {
            "areaCode": "STORAGE_B",
            "areaName": "Storage Area B",
            "availableSlots": 30,
            "totalSlots": 40,
            "occupancyRate": 0.25,
            "priority": 2
        }
    ],
    "msg": "Query successful",
    "success": true
}
```

### Update Container
- **Endpoint**: `POST /api/amr/updateContainer`
- **Description**: Update container information (empty/full status and position)
- **Request Body**:
```json
{
    "containerCode": "",
    "containerType": "",
    "emptyStatus": "",
    "originPosition": "",
    "reason": "",
    "requestId": "",
    "targetPosition": ""
}
```

## Robot Related APIs

### Charge Robot
- **Endpoint**: `POST /api/amr/chargeRobot`
- **Description**: Dispatch charge task to robot
- **Request Body**:
```json
{
    "lowestLevel": 0,
    "missionCode": "",
    "necessary": 0,
    "robotId": "",
    "targetLevel": 0
}
```

### Insert Robot
- **Endpoint**: `POST /api/amr/insertRobot`
- **Description**: Insert robot into map
- **Request Body**:
```json
{
    "cellCode": "",
    "robotId": "",
    "synchroContainer": 0
}
```

### Query Robot by Node
- **Endpoint**: `POST /api/amr/queryRobByNodeUuidOrForeignCode`
- **Description**: Query robot by node UUID or foreign code
- **Parameters**: 
  - `nodeCode` (string, required): Node code (support comma-separated multiple codes)

### Remove Robot
- **Endpoint**: `POST /api/amr/removeRobot`
- **Description**: Remove robot from map
- **Request Body**:
```json
{
    "robotId": "",
    "withContainer": 0
}
```

### Robot Move Carry
- **Endpoint**: `POST /api/amr/robotMoveCarry`
- **Description**: Dispatch move carry task to robot
- **Request Body**:
```json
{
    "containerCode": "",
    "missionCode": "",
    "robotId": "",
    "targetNodeCode": ""
}
```

### Query Robots
- **Endpoint**: `POST /api/amr/robotQuery`
- **Description**: Query robot status with different conditions
- **Request Body**:
```json
{
    "floorNumber": "",
    "mapCode": "",
    "robotId": "",
    "robotType": ""
}
```
- **Response**:
```json
{
    "code": "SUCCESS",
    "data": [
        {
            "robotId": "AGV001",
            "robotType": "CARGO_AGV",
            "status": 2,
            "statusName": "WORKING",
            "batteryLevel": 85.5,
            "position": {
                "x": 12.5,
                "y": 8.3,
                "angle": 90.0,
                "nodeCode": "A001"
            },
            "currentTask": "MISSION_001",
            "mapCode": "WAREHOUSE_L1",
            "floorNumber": "1",
            "speed": 1.2,
            "loadStatus": 1,
            "errorCode": "",
            "lastUpdateTime": "2024-01-15T10:30:00Z"
        }
    ],
    "msg": "Query successful",
    "success": true
}
```
- **Status Values**:
  - 0: OFFLINE, 1: IDLE, 2: WORKING, 3: CHARGING, 4: ERROR, 5: MAINTENANCE
- **Load Status**:
  - 0: EMPTY, 1: LOADED, 2: UNKNOWN

## Plugin Operation APIs

### Tag Plugin
- **Endpoint**: `POST /api/plugin/addTag/{pluginId}`
- **Description**: Add tag to specified plugin
- **Parameters**: 
  - `pluginId` (string, required): Plugin ID
  - `tag` (string, required): Tag name
  - `version` (string, required): Plugin version

### Delete Plugin
- **Endpoint**: `POST /api/plugin/delete/{pluginId}`
- **Description**: Delete specified plugin
- **Parameters**: 
  - `pluginId` (string, required): Plugin ID
  - `version` (string, optional): Plugin version

### Query Record
- **Endpoint**: `GET /api/plugin/getRecord`
- **Description**: Query plugin operation records
- **Parameters**: 
  - `beginTime` (string, optional): Begin time
  - `endTime` (string, optional): End time
  - `page` (integer, optional): Page number
  - `pageCount` (integer, optional): Records per page
  - `status` (integer, optional): Status
  - `taskCode` (string, optional): Task code
  - `url` (string, optional): URL

### Get Record Body
- **Endpoint**: `GET /api/plugin/getRecordHttpMessage`
- **Description**: Get detailed plugin record content
- **Parameters**: 
  - `id` (integer, required): Record ID

### List Plugins
- **Endpoint**: `GET /api/plugin/listPlugins`
- **Description**: List all available plugins
- **Parameters**: 
  - `pluginId` (string, optional): Plugin ID filter

### Reload Plugin
- **Endpoint**: `POST /api/plugin/reload/{pluginId}`
- **Description**: Reload specified plugin
- **Parameters**: 
  - `pluginId` (string, required): Plugin ID
  - `version` (string, optional): Plugin version

### Resend Record
- **Endpoint**: `POST /api/plugin/resend/{id}`
- **Description**: Resend specified record
- **Parameters**: 
  - `id` (integer, required): Record ID
- **Request Body**: Parameter map object

### Enable Plugin
- **Endpoint**: `POST /api/plugin/start/{pluginId}`
- **Description**: Enable specified plugin
- **Parameters**: 
  - `pluginId` (string, required): Plugin ID
  - `version` (string, optional): Plugin version

### Disable Plugin
- **Endpoint**: `POST /api/plugin/stop/{pluginId}`
- **Description**: Disable specified plugin
- **Parameters**: 
  - `pluginId` (string, required): Plugin ID

### Unload Plugin
- **Endpoint**: `POST /api/plugin/unload/{pluginId}`
- **Description**: Unload specified plugin
- **Parameters**: 
  - `pluginId` (string, required): Plugin ID

### Upload Plugin
- **Endpoint**: `POST /api/plugin/upload`
- **Description**: Upload plugin JAR file
- **Content-Type**: `multipart/form-data`
- **Form Data**: 
  - `jarFile` (file, required): JAR file to upload

## Response Formats

### Success Response
All APIs return responses in the following format:

```json
{
    "code": "SUCCESS",
    "data": "response_data",
    "msg": "response_message",
    "success": true
}
```

### Error Response
Error responses include detailed error information:

```json
{
    "code": "ERROR_CODE",
    "data": null,
    "msg": "Error description",
    "success": false,
    "errorDetails": {
        "errorCode": "VALIDATION_ERROR",
        "errorMessage": "Invalid parameter: robotId cannot be empty",
        "timestamp": "2024-01-15T10:30:00Z",
        "requestId": "REQ_12345"
    }
}
```

### Common Error Codes
| Error Code | Description |
|------------|-------------|
| VALIDATION_ERROR | Request parameter validation failed |
| AUTHENTICATION_ERROR | Invalid or expired token |
| AUTHORIZATION_ERROR | Insufficient permissions |
| RESOURCE_NOT_FOUND | Requested resource does not exist |
| RESOURCE_CONFLICT | Resource conflict (e.g., robot already assigned) |
| SYSTEM_ERROR | Internal system error |
| TIMEOUT_ERROR | Request timeout |
| NETWORK_ERROR | Network connectivity issue |

## Status Code Mappings

### HTTP Status Codes
| Status Code | Description | Common Scenarios |
|-------------|-------------|------------------|
| 200 | OK | Successful API call |
| 201 | Created | Resource created successfully |
| 400 | Bad Request | Invalid request parameters |
| 401 | Unauthorized | Missing or invalid token |
| 403 | Forbidden | Insufficient permissions |
| 404 | Not Found | Resource not found |
| 409 | Conflict | Resource conflict |
| 429 | Too Many Requests | Rate limit exceeded |
| 500 | Internal Server Error | System error |
| 503 | Service Unavailable | System maintenance |

### Job Status Codes
| Status | Name | Description |
|--------|------|-------------|
| 0 | ALL | All statuses (query filter) |
| 1 | PENDING | Job created, waiting for execution |
| 2 | RUNNING | Job currently executing |
| 3 | COMPLETED | Job finished successfully |
| 4 | FAILED | Job execution failed |
| 5 | CANCELLED | Job cancelled by user |

### Robot Status Codes
| Status | Name | Description |
|--------|------|-------------|
| 0 | OFFLINE | Robot not connected |
| 1 | IDLE | Robot available for tasks |
| 2 | WORKING | Robot executing task |
| 3 | CHARGING | Robot charging battery |
| 4 | ERROR | Robot in error state |
| 5 | MAINTENANCE | Robot in maintenance mode |

### Container Status Codes
| Status | Name | Description |
|--------|------|-------------|
| 0 | ALL | All statuses (query filter) |
| 1 | EMPTY | Container is empty |
| 2 | FULL | Container has cargo |
| 3 | UNKNOWN | Container status unknown |

## Usage Examples

### Python Client Usage

```python
from kuka_api_client import KukaApiClient

# Initialize client with auto-login
client = KukaApiClient(
    base_url="http://192.168.10.3:10870",
    username="admin", 
    password="Admin"
)

# Query robots
robots = client.robot_query({})

# Submit mission
mission_data = {
    "orgId": "ORG001",
    "requestId": "REQ001",
    "missionCode": "MISSION001",
    "missionType": "TRANSPORT",
    "robotIds": ["AGV001"],
    "priority": 50
}
result = client.submit_mission(mission_data)

# Query areas
areas = client.area_query()
```

### CURL Examples

```bash
# Login
curl -X POST "http://192.168.10.3:10870/api/login" \
     -H "Content-Type: application/json" \
     -d '{"username":"admin","password":"Admin"}'

# Query robots (with token)
curl -X POST "http://192.168.10.3:10870/api/amr/robotQuery" \
     -H "Authorization: <token>" \
     -H "Content-Type: application/json" \
     -d '{}'

# Submit mission
curl -X POST "http://192.168.10.3:10870/api/amr/submitMission" \
     -H "Authorization: <token>" \
     -H "Content-Type: application/json" \
     -d '{
         "orgId": "ORG001",
         "requestId": "REQ001", 
         "missionCode": "MISSION001",
         "missionType": "TRANSPORT",
         "robotIds": ["AGV001"],
         "priority": 50
     }'
```