/**
 * Linear Flow Designer v2 - Function Library with Explicit Defaults
 * 所有函數都有明確定義的預設值，不依賴推論
 */

// Export function library with complete explicit defaults
export const functionLibrary = {
    'query': [
        {
            name: 'query.locations',
            params: ['type', 'rooms', 'has_rack'],
            defaults: {
                type: 'rack',
                rooms: ['room01', 'room02'],
                has_rack: true
            },
            description: '查詢位置資料'
        },
        {
            name: 'query.work_rack_info',
            params: ['work_id', 'rack_type'],
            defaults: {
                work_id: 'W001',
                rack_type: 'standard'
            },
            description: '查詢工作架台資訊'
        },
        {
            name: 'query.agv_status',
            params: ['agv_id'],
            defaults: {
                agv_id: 'agv01'
            },
            description: '查詢 AGV 狀態'
        },
        {
            name: 'query.rack_status',
            params: ['rack_id'],
            defaults: {
                rack_id: 'rack01'
            },
            description: '查詢架台狀態'
        },
        {
            name: 'query.task_status',
            params: ['task_id'],
            defaults: {
                task_id: 'task_001'
            },
            description: '查詢任務狀態'
        },
        {
            name: 'query.available_agvs',
            params: ['zone', 'min_battery'],
            defaults: {
                zone: 'zone_A',
                min_battery: 30
            },
            description: '查詢可用 AGV'
        },
        {
            name: 'query.station_info',
            params: ['station_id', 'include_status'],
            defaults: {
                station_id: 'station01',
                include_status: true
            },
            description: '查詢站點資訊'
        }
    ],
    'check': [
        {
            name: 'check.empty',
            params: ['list'],
            defaults: {
                list: '${result}'
            },
            description: '檢查列表是否為空'
        },
        {
            name: 'check.agv_available',
            params: ['agv_id'],
            defaults: {
                agv_id: 'agv01'
            },
            description: '檢查 AGV 是否可用'
        },
        {
            name: 'check.location_accessible',
            params: ['location_id'],
            defaults: {
                location_id: 'station01'
            },
            description: '檢查位置是否可達'
        },
        {
            name: 'check.condition',
            params: ['expression'],
            defaults: {
                expression: '${result} > 0'
            },
            description: '檢查條件表達式'
        },
        {
            name: 'check.battery_level',
            params: ['agv_id', 'min_level'],
            defaults: {
                agv_id: 'agv01',
                min_level: 20
            },
            description: '檢查電池電量'
        },
        {
            name: 'check.rack_occupied',
            params: ['rack_id'],
            defaults: {
                rack_id: 'rack01'
            },
            description: '檢查架台是否被佔用'
        }
    ],
    'task': [
        {
            name: 'task.create',
            params: ['type', 'params'],
            defaults: {
                type: 'transport',
                params: {
                    from: 'rack01',
                    to: 'station01',
                    priority: 100,
                    item_id: 'item_001'
                }
            },
            description: '創建任務'
        },
        {
            name: 'task.assign',
            params: ['task_id', 'agv_id'],
            defaults: {
                task_id: '${task_id}',
                agv_id: 'agv01'
            },
            description: '分配任務給 AGV'
        },
        {
            name: 'task.cancel',
            params: ['task_id', 'reason'],
            defaults: {
                task_id: '${task_id}',
                reason: 'User cancelled'
            },
            description: '取消任務'
        },
        {
            name: 'task.update',
            params: ['task_id', 'status', 'message'],
            defaults: {
                task_id: '${task_id}',
                status: 'completed',
                message: 'Task completed successfully'
            },
            description: '更新任務狀態'
        },
        {
            name: 'task.prioritize',
            params: ['task_id', 'new_priority'],
            defaults: {
                task_id: '${task_id}',
                new_priority: 200
            },
            description: '調整任務優先級'
        }
    ],
    'action': [
        {
            name: 'action.move_agv',
            params: ['agv_id', 'target', 'speed'],
            defaults: {
                agv_id: 'agv01',
                target: 'station01',
                speed: 1.0
            },
            description: '移動 AGV'
        },
        {
            name: 'action.load',
            params: ['agv_id', 'item_id', 'position'],
            defaults: {
                agv_id: 'agv01',
                item_id: 'item_001',
                position: 'front'
            },
            description: '裝載物品'
        },
        {
            name: 'action.unload',
            params: ['agv_id', 'location', 'position'],
            defaults: {
                agv_id: 'agv01',
                location: 'station01',
                position: 'front'
            },
            description: '卸載物品'
        },
        {
            name: 'action.wait',
            params: ['seconds'],
            defaults: {
                seconds: 5
            },
            description: '等待指定時間'
        },
        {
            name: 'action.charge',
            params: ['agv_id', 'charging_station'],
            defaults: {
                agv_id: 'agv01',
                charging_station: 'charge_01'
            },
            description: '充電'
        },
        {
            name: 'action.rotate',
            params: ['agv_id', 'angle', 'direction'],
            defaults: {
                agv_id: 'agv01',
                angle: 90,
                direction: 'clockwise'
            },
            description: '旋轉 AGV'
        },
        {
            name: 'action.log',
            params: ['message', 'level'],
            defaults: {
                message: 'Step completed',
                level: 'info'
            },
            description: '記錄日誌'
        },
        {
            name: 'action.send_notification',
            params: ['recipient', 'message', 'priority'],
            defaults: {
                recipient: 'operator',
                message: 'Task status update',
                priority: 'normal'
            },
            description: '發送通知'
        },
        {
            name: 'action.optimize_batch',
            params: ['tasks', 'optimization_type'],
            defaults: {
                tasks: '${task_list}',
                optimization_type: 'distance'
            },
            description: '批次優化'
        },
        {
            name: 'action.update_metrics',
            params: ['metrics'],
            defaults: {
                metrics: {
                    throughput: 100,
                    efficiency: 0.95,
                    errors: 0,
                    timestamp: '${current_time}'
                }
            },
            description: '更新指標'
        },
        {
            name: 'action.log_message',
            params: ['message', 'level', 'tags'],
            defaults: {
                message: 'Flow step executed',
                level: 'info',
                tags: []
            },
            description: '記錄日誌訊息'
        }
    ],
    'control': [
        {
            name: 'control.if',
            params: ['condition', 'then', 'else'],
            defaults: {
                condition: '${result} == true',
                then: 'action.log',
                else: 'action.wait'
            },
            description: '條件控制'
        },
        {
            name: 'control.switch',
            params: ['variable', 'cases', 'default'],
            defaults: {
                variable: '${status}',
                cases: {
                    'success': 'action.log',
                    'error': 'action.send_notification',
                    'pending': 'action.wait'
                },
                default: 'action.wait'
            },
            description: '多條件分支'
        },
        {
            name: 'control.loop',
            params: ['items', 'action', 'max_iterations'],
            defaults: {
                items: '${list}',
                action: 'action.process_item',
                max_iterations: 100
            },
            description: '循環控制'
        },
        {
            name: 'control.while',
            params: ['condition', 'action', 'max_iterations'],
            defaults: {
                condition: '${counter} < 10',
                action: 'action.increment',
                max_iterations: 100
            },
            description: 'While 循環'
        },
        {
            name: 'control.parallel',
            params: ['actions', 'wait_all'],
            defaults: {
                actions: ['action.move_agv', 'action.load', 'action.send_notification'],
                wait_all: true
            },
            description: '並行執行'
        },
        {
            name: 'control.retry',
            params: ['action', 'max_retries', 'retry_delay'],
            defaults: {
                action: 'action.move_agv',
                max_retries: 3,
                retry_delay: 5
            },
            description: '重試執行'
        },
        {
            name: 'control.timeout',
            params: ['action', 'timeout_seconds', 'fallback'],
            defaults: {
                action: 'action.move_agv',
                timeout_seconds: 30,
                fallback: 'action.cancel'
            },
            description: '超時控制'
        },
        {
            name: 'control.count',
            params: ['items'],
            defaults: {
                items: '${list}'
            },
            description: '計數'
        },
        {
            name: 'control.count_items',
            params: ['variable'],
            defaults: {
                variable: '${query_result}'
            },
            description: '計算項目數量'
        }
    ],
    'special': [
        {
            name: 'foreach',
            params: ['items', 'action', 'index_var'],
            defaults: {
                items: '${list}',
                action: 'action.process_item',
                index_var: 'index'
            },
            description: '迭代處理'
        },
        {
            name: 'wait',
            params: ['seconds'],
            defaults: {
                seconds: 3
            },
            description: '等待'
        },
        {
            name: 'return',
            params: ['value'],
            defaults: {
                value: '${result}'
            },
            description: '返回值'
        },
        {
            name: 'break',
            params: [],
            defaults: {},
            description: '中斷循環'
        },
        {
            name: 'continue',
            params: [],
            defaults: {},
            description: '繼續循環'
        },
        {
            name: 'throw',
            params: ['error_code', 'error_message'],
            defaults: {
                error_code: 'ERROR_001',
                error_message: 'An error occurred'
            },
            description: '拋出錯誤'
        },
        {
            name: 'catch',
            params: ['error_handler'],
            defaults: {
                error_handler: 'action.log_error'
            },
            description: '捕獲錯誤'
        }
    ],
    'data': [
        {
            name: 'data.set',
            params: ['variable', 'value'],
            defaults: {
                variable: 'my_variable',
                value: 'default_value'
            },
            description: '設置變數'
        },
        {
            name: 'data.get',
            params: ['variable', 'default_value'],
            defaults: {
                variable: 'my_variable',
                default_value: null
            },
            description: '獲取變數'
        },
        {
            name: 'data.append',
            params: ['list', 'item'],
            defaults: {
                list: '${my_list}',
                item: 'new_item'
            },
            description: '添加到列表'
        },
        {
            name: 'data.remove',
            params: ['list', 'item'],
            defaults: {
                list: '${my_list}',
                item: 'item_to_remove'
            },
            description: '從列表移除'
        },
        {
            name: 'data.merge',
            params: ['object1', 'object2'],
            defaults: {
                object1: '${data1}',
                object2: '${data2}'
            },
            description: '合併資料'
        },
        {
            name: 'data.transform',
            params: ['input', 'transformation'],
            defaults: {
                input: '${raw_data}',
                transformation: 'uppercase'
            },
            description: '轉換資料'
        }
    ]
};

// Helper function to get all function names for validation
export function getAllFunctionNames() {
    const names = [];
    for (const category in functionLibrary) {
        for (const func of functionLibrary[category]) {
            names.push(func.name);
        }
    }
    return names;
}

// Helper function to get function by name
export function getFunctionByName(functionName) {
    for (const category in functionLibrary) {
        const func = functionLibrary[category].find(f => f.name === functionName);
        if (func) {
            return { ...func, category };
        }
    }
    return null;
}

// Helper function to validate function parameters
export function validateFunctionParams(functionName, params) {
    const func = getFunctionByName(functionName);
    if (!func) return { valid: false, error: 'Function not found' };
    
    const missingParams = [];
    for (const requiredParam of func.params) {
        if (!(requiredParam in params)) {
            missingParams.push(requiredParam);
        }
    }
    
    if (missingParams.length > 0) {
        return { 
            valid: false, 
            error: `Missing required parameters: ${missingParams.join(', ')}` 
        };
    }
    
    return { valid: true };
}

export default functionLibrary;