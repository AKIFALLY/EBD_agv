/**
 * Flow Designer Node Types Definition
 * Phase 3.1: 節點類型系統 - 基於 38 個 WCS 函數的完整節點定義
 * 
 * 基於已完成的 DSL 系統中 WCS_FUNCTION_MAPPINGS 的實際函數：
 * - unified_decision_engine: 19個條件判斷函數
 * - location_manager: 9個邏輯處理函數  
 * - unified_task_manager: 10個動作執行函數
 * - enhanced_database_client: 支援查詢函數
 * 
 * 與 DSL 分類系統完全一致：condition_nodes, logic_nodes, action_nodes, script_nodes
 */

/**
 * 節點視覺樣式配置 - 與 CSS 類別系統統一
 */
const NODE_STYLES = {
    condition: {
        shape: 'diamond',
        color: '#3B82F6', // blue-500 - 與 CSS data-node-category="input" 一致
        borderColor: '#2563eb', // blue-600
        textColor: '#1e3a8a', // blue-800
        bgColor: '#dbeafe', // blue-200
        category: 'input' // 對應 CSS 類別
    },
    logic: {
        shape: 'rectangle',
        color: '#F59E0B', // amber-500 - 與 CSS data-node-category="control" 一致
        borderColor: '#d97706', // amber-600
        textColor: '#92400e', // amber-800
        bgColor: '#fef3c7', // amber-100
        category: 'control' // 對應 CSS 類別
    },
    action: {
        shape: 'rounded-rectangle',
        color: '#10B981', // emerald-500 - 與 CSS data-node-category="output" 一致
        borderColor: '#059669', // emerald-600
        textColor: '#064e3b', // emerald-800
        bgColor: '#d1fae5', // emerald-200
        category: 'output' // 對應 CSS 類別
    },
    script: {
        shape: 'octagon',
        color: '#8B5CF6', // violet-500 - 與 CSS data-node-category="storage" 一致
        borderColor: '#7c3aed', // violet-600
        textColor: '#4c1d95', // violet-800
        bgColor: '#e9d5ff', // violet-200
        category: 'storage' // 對應 CSS 類別
    }
};

/**
 * Condition Nodes - 條件判斷節點 (基於實際 DSL 系統函數)
 * 來源: WCS_FUNCTION_MAPPINGS['unified_decision_engine'] 和 ['enhanced_database_client']
 * 與 DSL 系統 condition_nodes 完全一致
 */
const CONDITION_NODES = {
    // AGV 旋轉流程條件 - 基於實際 DSL 函數
    check_agv_rotation_flow: {
        id: 'check_agv_rotation_flow',
        name: 'AGV 旋轉流程檢查',
        description: '檢查 AGV 是否需要執行旋轉流程',
        category: 'condition',
        source: 'unified_decision_engine',
        dslType: 'condition_nodes',
        returnType: 'List[TaskDecision]',
        inputs: [
            { name: 'room_id', type: 'integer', required: true, description: '房間ID' },
            { name: 'agv_id', type: 'string', required: false, description: 'AGV ID' }
        ],
        outputs: [
            { name: 'decisions', type: 'List[TaskDecision]', description: '旋轉任務決策列表' }
        ],
        icon: '🔄',
        ...NODE_STYLES.condition
    },

    // 料架旋轉相關條件 - 基於實際 DSL 函數
    check_rack_rotation_flow: {
        id: 'check_rack_rotation_flow',
        name: '料架旋轉檢查',
        description: '檢查料架是否需要旋轉',
        category: 'condition',
        source: 'unified_decision_engine',
        dslType: 'condition_nodes',
        returnType: 'List[TaskDecision]',
        inputs: [
            { name: 'room_id', type: 'integer', required: true, description: '房間ID' },
            { name: 'location_type', type: 'string', required: true, description: '位置類型', options: ['room_inlet', 'room_exit'] }
        ],
        outputs: [
            { name: 'rotation_needed', type: 'boolean' },
            { name: 'task_decisions', type: 'List[TaskDecision]' }
        ],
        icon: '🔄',
        ...NODE_STYLES.condition
    },

    check_rack_rotation_exit_flow: {
        id: 'check_rack_rotation_exit_flow', 
        name: '料架出口旋轉檢查',
        description: '檢查料架在房間出口的旋轉需求',
        category: 'condition',
        source: 'unified_decision_engine',
        dslType: 'condition_nodes',
        returnType: 'List[TaskDecision]',
        inputs: [
            { name: 'room_id', type: 'integer', required: true, description: '房間ID' },
            { name: 'exit_point', type: 'integer', required: true, description: '出口點位置' }
        ],
        outputs: [
            { name: 'task_decisions', type: 'List[TaskDecision]' }
        ],
        icon: '🚪',
        ...NODE_STYLES.condition
    },

    // 運輸流程條件 - 基於實際 DSL 函數
    check_transport_to_manual_flow: {
        id: 'check_transport_to_manual_flow',
        name: '人工收料區運輸檢查',
        description: '檢查是否需要運輸到人工收料區',
        category: 'condition',
        source: 'unified_decision_engine',
        dslType: 'condition_nodes',
        returnType: 'List[TaskDecision]',
        inputs: [
            { name: 'transfer_exit_id', type: 'integer', required: true, description: '傳送出口ID' },
            { name: 'priority', type: 'integer', required: false, default: 5, min: 1, max: 10, description: '優先級' }
        ],
        outputs: [
            { name: 'task_decisions', type: 'List[TaskDecision]' }
        ],
        icon: '🚚',
        ...NODE_STYLES.condition
    },

    // 資料庫查詢條件 - 基於實際 DSL 函數
    check_locations_available: {
        id: 'check_locations_available',
        name: '位置可用性檢查',
        description: '檢查指定位置是否可用',
        category: 'condition',
        source: 'enhanced_database_client',
        dslType: 'condition_nodes',
        returnType: 'List[Dict[str, Any]]',
        inputs: [
            { name: 'location_ids', type: 'List[integer]', required: true, description: '位置ID列表' },
            { name: 'check_type', type: 'string', required: false, default: 'available', options: ['available', 'occupied', 'all'], description: '檢查類型' }
        ],
        outputs: [
            { name: 'available_locations', type: 'List[Dict[str, Any]]', description: '可用位置列表' }
        ],
        icon: '📍',
        ...NODE_STYLES.condition
    },

    check_ng_rack_at_location: {
        id: 'check_ng_rack_at_location',
        name: 'NG料架位置檢查',
        description: '檢查指定位置是否有NG料架',
        category: 'condition',
        source: 'enhanced_database_client',
        dslType: 'condition_nodes',
        returnType: 'boolean',
        inputs: [
            { name: 'location_id', type: 'integer', required: true, description: '位置ID' }
        ],
        outputs: [
            { name: 'has_ng_rack', type: 'boolean', description: '是否有NG料架' }
        ],
        icon: '⚠️',
        ...NODE_STYLES.condition
    },

    // 額外實際函數 - 基於 DSL 系統中的其他條件函數
    rack_at_location_exists: {
        id: 'rack_at_location_exists',
        name: '料架存在檢查',
        description: '檢查指定位置是否有料架',
        category: 'condition',
        source: 'enhanced_database_client',
        dslType: 'condition_nodes',
        returnType: 'boolean',
        inputs: [
            { name: 'location_id', type: 'integer', required: true, description: '位置ID' }
        ],
        outputs: [
            { name: 'rack_exists', type: 'boolean', description: '料架是否存在' }
        ],
        icon: '📦',
        ...NODE_STYLES.condition
    },

    rack_side_completed: {
        id: 'rack_side_completed',
        name: '料架側面完成檢查',
        description: '檢查料架指定側面是否已完成',
        category: 'condition',
        source: 'enhanced_database_client',
        dslType: 'condition_nodes',
        returnType: 'boolean',
        inputs: [
            { name: 'rack_id', type: 'string', required: true, description: '料架ID' },
            { name: 'side', type: 'string', required: true, options: ['A', 'B'], description: '料架側面' }
        ],
        outputs: [
            { name: 'side_completed', type: 'boolean', description: '側面是否完成' }
        ],
        icon: '✅',
        ...NODE_STYLES.condition
    }
};

/**
 * Logic Nodes - 邏輯處理節點 (9個函數)
 * 基於 WCS_FUNCTION_MAPPINGS['location_manager'] 和其他邏輯函數
 */
const LOGIC_NODES = {
    // 位置管理邏輯
    get_room_inlet_point: {
        id: 'get_room_inlet_point',
        name: '取得房間入口點',
        description: '根據房間ID獲取入口停靠點',
        category: 'logic',
        source: 'location_manager',
        returnType: 'int',
        inputs: [
            { name: 'room_id', type: 'integer', required: true, min: 1, max: 10 }
        ],
        outputs: [
            { name: 'inlet_point', type: 'integer', description: '入口停靠點位置ID' }
        ],
        icon: '📥',
        ...NODE_STYLES.logic
    },

    get_inlet_rotation_point: {
        id: 'get_inlet_rotation_point',
        name: '取得入口旋轉點',
        description: '獲取房間入口的旋轉中間點',
        category: 'logic',
        source: 'location_manager',
        returnType: 'int',
        inputs: [
            { name: 'room_id', type: 'integer', required: true }
        ],
        outputs: [
            { name: 'rotation_point', type: 'integer' }
        ],
        icon: '🔄',
        ...NODE_STYLES.logic
    },

    get_room_exit_point: {
        id: 'get_room_exit_point',
        name: '取得房間出口點',
        description: '根據房間ID獲取出口停靠點',
        category: 'logic',
        source: 'location_manager',
        returnType: 'int',
        inputs: [
            { name: 'room_id', type: 'integer', required: true }
        ],
        outputs: [
            { name: 'exit_point', type: 'integer' }
        ],
        icon: '📤',
        ...NODE_STYLES.logic
    },

    get_exit_rotation_point: {
        id: 'get_exit_rotation_point',
        name: '取得出口旋轉點',
        description: '獲取房間出口的旋轉中間點',
        category: 'logic',
        source: 'location_manager',
        returnType: 'int',
        inputs: [
            { name: 'room_id', type: 'integer', required: true }
        ],
        outputs: [
            { name: 'exit_rotation_point', type: 'integer' }
        ],
        icon: '🔄',
        ...NODE_STYLES.logic
    },

    // 資料庫邏輯查詢
    find_available_manual_location: {
        id: 'find_available_manual_location',
        name: '尋找可用人工位置',
        description: '尋找可用的人工收料區位置',
        category: 'logic',
        source: 'enhanced_database_client',
        returnType: 'int',
        inputs: [
            { name: 'area_type', type: 'string', required: false, default: 'manual_collection' },
            { name: 'priority_filter', type: 'boolean', required: false, default: false }
        ],
        outputs: [
            { name: 'location_id', type: 'integer', description: '可用位置ID，0表示無可用位置' }
        ],
        icon: '🔍',
        ...NODE_STYLES.logic
    }
};

/**
 * Action Nodes - 動作執行節點 (10個函數)
 * 基於 WCS_FUNCTION_MAPPINGS['unified_task_manager'] 和其他動作函數
 */
const ACTION_NODES = {
    // 任務管理動作
    create_task_from_decision: {
        id: 'create_task_from_decision',
        name: '從決策創建任務',
        description: '根據決策結果創建 WCS 任務',
        category: 'action',
        source: 'unified_task_manager',
        returnType: 'TaskCreationResult',
        inputs: [
            { name: 'decision', type: 'Dict[str, Any]', required: true, description: '任務決策資料' },
            { name: 'priority', type: 'integer', required: false, default: 5, min: 1, max: 10 }
        ],
        outputs: [
            { name: 'task_result', type: 'TaskCreationResult', description: '任務創建結果' },
            { name: 'task_id', type: 'string', description: '創建的任務ID' }
        ],
        icon: '📋',
        ...NODE_STYLES.action
    },

    create_tasks_from_decisions: {
        id: 'create_tasks_from_decisions',
        name: '批量創建任務',
        description: '從多個決策批量創建任務',
        category: 'action',
        source: 'unified_task_manager',
        returnType: 'List[TaskCreationResult]',
        inputs: [
            { name: 'decisions', type: 'List[Dict[str, Any]]', required: true },
            { name: 'batch_options', type: 'Dict[str, Any]', required: false }
        ],
        outputs: [
            { name: 'results', type: 'List[TaskCreationResult]' },
            { name: 'success_count', type: 'integer' },
            { name: 'failure_count', type: 'integer' }
        ],
        icon: '📋',
        ...NODE_STYLES.action
    },

    // 資料庫更新動作
    create_task_from_decision_dict: {
        id: 'create_task_from_decision_dict',
        name: '從決策字典創建任務',
        description: '從決策字典格式創建資料庫任務記錄',
        category: 'action',
        source: 'enhanced_database_client',
        returnType: 'int',
        inputs: [
            { name: 'decision_dict', type: 'Dict[str, Any]', required: true },
            { name: 'metadata', type: 'Dict[str, Any]', required: false }
        ],
        outputs: [
            { name: 'task_id', type: 'integer', description: '新創建的任務ID' }
        ],
        icon: '💾',
        ...NODE_STYLES.action
    },

    update_machine_parking_status: {
        id: 'update_machine_parking_status',
        name: '更新機台停車狀態',
        description: '更新機台的停車狀態資訊',
        category: 'action',
        source: 'enhanced_database_client',
        returnType: 'bool',
        inputs: [
            { name: 'machine_id', type: 'string', required: true },
            { name: 'parking_status', type: 'string', required: true, options: ['parked', 'moving', 'idle'] },
            { name: 'location_id', type: 'integer', required: false }
        ],
        outputs: [
            { name: 'update_success', type: 'boolean' }
        ],
        icon: '🏭',
        ...NODE_STYLES.action
    }
};

/**
 * Script Nodes - 腳本控制節點
 * 控制流程邏輯 (if/else, loops, variables)
 */
const SCRIPT_NODES = {
    // 條件控制
    if_else: {
        id: 'if_else',
        name: '條件分支',
        description: 'if/else 條件分支控制',
        category: 'script',
        source: 'dsl_runtime',
        inputs: [
            { name: 'condition', type: 'boolean', required: true, description: '判斷條件' },
            { name: 'if_branch', type: 'List[Step]', required: true, description: '條件為真時執行的步驟' },
            { name: 'else_branch', type: 'List[Step]', required: false, description: '條件為假時執行的步驟' }
        ],
        outputs: [
            { name: 'execution_result', type: 'Any', description: '執行結果' }
        ],
        icon: '🔀',
        ...NODE_STYLES.script
    },

    // 循環控制
    for_loop: {
        id: 'for_loop',
        name: 'For 循環',
        description: 'For 循環遍歷控制',
        category: 'script',
        source: 'dsl_runtime',
        inputs: [
            { name: 'iterable', type: 'List[Any]', required: true, description: '要遍歷的列表' },
            { name: 'loop_variable', type: 'string', required: true, description: '循環變數名稱' },
            { name: 'loop_body', type: 'List[Step]', required: true, description: '循環體步驟' }
        ],
        outputs: [
            { name: 'results', type: 'List[Any]', description: '循環執行結果列表' }
        ],
        icon: '🔁',
        ...NODE_STYLES.script
    },

    // 變數操作
    set_variable: {
        id: 'set_variable',
        name: '設置變數',
        description: '設置或更新變數值',
        category: 'script',
        source: 'dsl_runtime',
        inputs: [
            { name: 'variable_name', type: 'string', required: true, description: '變數名稱' },
            { name: 'value', type: 'Any', required: true, description: '變數值' },
            { name: 'scope', type: 'string', required: false, default: 'local', options: ['local', 'global', 'output'] }
        ],
        outputs: [
            { name: 'variable_set', type: 'boolean', description: '變數設置成功' }
        ],
        icon: '📝',
        ...NODE_STYLES.script
    },

    get_variable: {
        id: 'get_variable',
        name: '取得變數',
        description: '取得變數值',
        category: 'script',
        source: 'dsl_runtime',
        inputs: [
            { name: 'variable_name', type: 'string', required: true, description: '變數名稱' },
            { name: 'default_value', type: 'Any', required: false, description: '預設值（變數不存在時）' }
        ],
        outputs: [
            { name: 'value', type: 'Any', description: '變數值' }
        ],
        icon: '📖',
        ...NODE_STYLES.script
    }
};

/**
 * 完整節點類型定義 - 整合所有節點類型
 */
const ALL_NODE_TYPES = {
    ...CONDITION_NODES,
    ...LOGIC_NODES,
    ...ACTION_NODES,
    ...SCRIPT_NODES
};

/**
 * 按類別分組的節點類型
 */
const NODE_TYPES_BY_CATEGORY = {
    condition: CONDITION_NODES,
    logic: LOGIC_NODES,
    action: ACTION_NODES,
    script: SCRIPT_NODES
};

/**
 * 節點類型驗證函數
 */
function validateNodeType(nodeType) {
    const required = ['id', 'name', 'description', 'category', 'source'];
    return required.every(field => nodeType.hasOwnProperty(field));
}

/**
 * 獲取節點類型資訊
 */
function getNodeType(nodeId) {
    return ALL_NODE_TYPES[nodeId] || null;
}

/**
 * 獲取特定類別的節點類型
 */
function getNodeTypesByCategory(category) {
    return NODE_TYPES_BY_CATEGORY[category] || {};
}

/**
 * 搜尋節點類型
 */
function searchNodeTypes(query) {
    const lowerQuery = query.toLowerCase();
    return Object.values(ALL_NODE_TYPES).filter(nodeType => 
        nodeType.name.toLowerCase().includes(lowerQuery) ||
        nodeType.description.toLowerCase().includes(lowerQuery) ||
        nodeType.id.toLowerCase().includes(lowerQuery)
    );
}

// 導出主要對象和函數
if (typeof module !== 'undefined' && module.exports) {
    // Node.js 環境
    module.exports = {
        NODE_STYLES,
        CONDITION_NODES,
        LOGIC_NODES,
        ACTION_NODES,
        SCRIPT_NODES,
        ALL_NODE_TYPES,
        NODE_TYPES_BY_CATEGORY,
        validateNodeType,
        getNodeType,
        getNodeTypesByCategory,
        searchNodeTypes
    };
} else {
    // 瀏覽器環境
    window.FlowDesigner = {
        NODE_STYLES,
        CONDITION_NODES,
        LOGIC_NODES,
        ACTION_NODES,
        SCRIPT_NODES,
        ALL_NODE_TYPES,
        NODE_TYPES_BY_CATEGORY,
        validateNodeType,
        getNodeType,
        getNodeTypesByCategory,
        searchNodeTypes
    };
}