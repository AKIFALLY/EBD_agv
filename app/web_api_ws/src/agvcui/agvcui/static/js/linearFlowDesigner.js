/**
 * Linear Flow Designer JavaScript
 * For Flow WCS v2 Linear Format
 * Store-based Architecture
 */

// Import FlowStore
import { flowStore } from '/static/store/flowStore.js';

// Global variables
let yamlEditor = null;
let availableFunctions = {};
let currentSectionIndex = null;
let currentStepIndex = null;
let isUpdatingFromStore = false;  // Prevent circular updates

// Auto-save and validation variables
let autoSaveTimer = null;
let validationTimer = null;
let lastSavedData = null;
let lastSavedContent = null;  // Track last saved content for comparison
let isModified = false;
let validationStatus = 'unknown'; // unknown, validating, valid, invalid, warning

/**
 * Initialize the designer
 */
async function initializeDesigner(initialData, flowId) {
    console.log('🚀 Initializing Linear Flow Designer with Store');
    
    // Initialize flow data in store
    if (initialData) {
        flowStore.loadFlow(initialData);
    } else if (flowId) {
        // Load from server if flowId provided
        await loadFlowFromServer(flowId);
    } else {
        // Create new flow
        flowStore.loadFlow(createNewFlow());
    }
    
    // Setup store subscriptions
    setupStoreSubscriptions();
    
    // Get state and update UI
    const state = flowStore.getFullState();
    updateFormFieldsFromState(state);
    
    // Store initial state for change detection
    lastSavedData = JSON.stringify(state);
    
    // Initialize validation indicator
    initializeValidationIndicator();
    
    // Perform initial validation
    performBackgroundValidation();
    
    // Initialize YAML editor
    initializeYamlEditor();
    
    // Load available functions first, then render (default to flow_wcs for real defaults)
    await loadAvailableFunctions('flow_wcs');
    
    // Render workflow
    renderWorkflow();
    
    // Initialize and render visual editor
    initializeVisualEditor();
    
    // Update variables display
    updateVariablesDisplay();
}

/**
 * Setup store subscriptions
 */
function setupStoreSubscriptions() {
    // Subscribe to store changes
    flowStore.on('change', (state, source) => {
        console.log('📦 Store changed from source:', source);
        
        // Skip if update is from current component to prevent loops
        if (isUpdatingFromStore) return;
        
        // Update UI components
        renderWorkflow();
        renderVisualEditor();
        updateVariablesDisplay();
        
        // Update YAML editor if change is not from YAML
        if (source !== 'yaml' && yamlEditor) {
            const yamlContent = flowStore.toYAML();
            yamlEditor.setValue(yamlContent);
        }
        
        // Update form fields
        updateFormFieldsFromState(state);
        
        // Mark as modified
        markFlowAsModified();
    });
}

/**
 * Update form fields from state
 */
function updateFormFieldsFromState(state) {
    if (state.flow) {
        document.getElementById('flow-id').value = state.flow.id || '';
        document.getElementById('flow-name').value = state.flow.name || '';
        document.getElementById('work-id').value = state.flow.work_id || '';
        document.getElementById('flow-priority').value = state.flow.priority || 100;
        document.getElementById('flow-enabled').checked = state.flow.enabled !== false;
    }
    if (state.meta) {
        document.getElementById('flow-description').value = state.meta.description || '';
    }
}

/**
 * Create a new flow structure
 */
function createNewFlow() {
    return {
        meta: {
            system: "linear_flow_v2",
            version: "2.0.0",
            author: "Flow Designer",
            created: new Date().toISOString(),
            description: ""
        },
        flow: {
            id: "new_flow_" + Date.now(),
            name: "New Flow",
            work_id: "",
            enabled: true,
            priority: 100,
            trigger: {
                type: "manual",
                interval: 10
            }
        },
        workflow: [],
        config: {
            timeout: 60,
            retry_on_error: true,
            max_retries: 3,
            error_handling: "continue"
        },
        variables: {}
    };
}

/**
 * Initialize YAML editor with CodeMirror
 */
function initializeYamlEditor() {
    const textarea = document.getElementById('yaml-editor-textarea');
    if (textarea) {
        // Define custom overlay mode for variable and function highlighting
        CodeMirror.defineMode("yaml-with-variables", function(config, parserConfig) {
            var yamlMode = CodeMirror.getMode(config, "yaml");
            
            var variableOverlay = {
                startState: function() {
                    return { afterExec: false, lineHasExec: false };
                },
                token: function(stream, state) {
                    // At the beginning of each line, check if line contains exec:
                    if (stream.sol()) {
                        state.lineHasExec = stream.string.includes('exec:');
                        state.afterExec = false;
                    }
                    
                    // Match entire ${...} pattern as one token for variables
                    var match = stream.match(/\$\{[^}]*\}/);
                    if (match) {
                        return "custom-variable";
                    }
                    
                    // Check if we've passed 'exec:' on this line
                    if (state.lineHasExec && !state.afterExec) {
                        var beforeCursor = stream.string.substring(0, stream.pos);
                        if (beforeCursor.includes('exec:')) {
                            state.afterExec = true;
                        }
                    }
                    
                    // Match function names after exec: and validate against available functions
                    if (state.afterExec) {
                        // Try to match function pattern
                        var funcMatch = stream.match(/\b[a-zA-Z_][a-zA-Z0-9_]*\.[a-zA-Z_][a-zA-Z0-9_]*/);
                        if (funcMatch) {
                            // Check if this function actually exists in availableFunctions
                            var functionName = funcMatch[0];
                            var functionExists = false;
                            
                            // Check all categories for this function
                            for (var category in availableFunctions) {
                                if (availableFunctions[category]) {
                                    for (var i = 0; i < availableFunctions[category].length; i++) {
                                        if (availableFunctions[category][i].name === functionName) {
                                            functionExists = true;
                                            break;
                                        }
                                    }
                                }
                                if (functionExists) break;
                            }
                            
                            // Only highlight if function actually exists
                            if (functionExists) {
                                return "custom-function";
                            }
                            // If function doesn't exist, let base mode handle it
                            return null;
                        }
                        
                        // Consume any other word after exec: without highlighting
                        var wordMatch = stream.match(/[a-zA-Z_][a-zA-Z0-9_.]*/);
                        if (wordMatch) {
                            return null; // Let base mode handle it
                        }
                    }
                    
                    // Skip one character and continue
                    if (stream.next() == null) {
                        // End of line reached
                        state.afterExec = false;
                        state.lineHasExec = false;
                    }
                    
                    return null;
                }
            };
            // false means overlay tokens override base mode tokens at the same position
            return CodeMirror.overlayMode(yamlMode, variableOverlay, false);
        });
        
        yamlEditor = CodeMirror.fromTextArea(textarea, {
            mode: 'yaml-with-variables',      // Use custom overlay mode
            theme: 'default',                  // Use default light theme
            lineNumbers: true,
            indentUnit: 2,
            tabSize: 2,
            indentWithTabs: false,
            lineWrapping: true,
            autoCloseBrackets: true,
            matchBrackets: true,
            styleActiveLine: true,
            foldGutter: true,
            gutters: ["CodeMirror-linenumbers", "CodeMirror-foldgutter"],
            extraKeys: {
                "Ctrl-Q": function(cm) { 
                    cm.foldCode(cm.getCursor()); 
                },
                "Ctrl-Shift-Q": function(cm) {
                    // Fold all top-level items
                    var firstLine = cm.firstLine();
                    var lastLine = cm.lastLine();
                    for (var i = firstLine; i <= lastLine; i++) {
                        cm.foldCode({line: i, ch: 0}, null, "fold");
                    }
                },
                "Ctrl-Shift-A": function(cm) {
                    // Unfold all
                    var firstLine = cm.firstLine();
                    var lastLine = cm.lastLine();
                    for (var i = firstLine; i <= lastLine; i++) {
                        cm.foldCode({line: i, ch: 0}, null, "unfold");
                    }
                }
            }
        });
        
        // Add onChange handler to sync YAML changes
        let yamlUpdateTimer = null;
        yamlEditor.on('change', function(cm, change) {
            // Mark the flow as modified when YAML is edited
            markFlowAsModified();
            
            // Debounce update for performance
            if (yamlUpdateTimer) {
                clearTimeout(yamlUpdateTimer);
            }
            yamlUpdateTimer = setTimeout(() => {
                const yamlContent = cm.getValue();
                if (yamlContent.trim()) {
                    // Update store from YAML
                    const result = flowStore.updateFromYAML(yamlContent, 'yaml');
                    if (!result.success) {
                        console.debug('YAML parse error during typing:', result.error);
                    }
                }
            }, 500); // 500ms debounce
        });
        
        // Update YAML content from store
        const yamlContent = flowStore.toYAML();
        yamlEditor.setValue(yamlContent);
    }
}

/**
 * Load available functions from API
 */
async function loadAvailableFunctions(source = null) {
    try {
        // Determine source based on context if not specified
        if (!source) {
            // Check if we're in test modal and what mode is selected
            const testMode = document.getElementById('test-mode')?.value;
            if (testMode === 'api' || testMode === 'validation') {
                source = 'flow_wcs';  // Both API and validation modes use flow_wcs
            } else {
                source = 'config';    // Default to config for simulation
            }
        }
        
        // Always request categorized format now
        // Frontend will build index for O(1) lookup
        const response = await fetch(`/linear-flow/api/functions?source=${source}`);
        const data = await response.json();
        
        if (data.success) {
            // Always categorized format now
            availableFunctions = data.functions;
            
            // Show source indicator in function library panel
            const functionList = document.getElementById('function-list');
            if (functionList) {
                // Remove any existing source indicator
                const existingIndicator = functionList.querySelector('.source-indicator');
                if (existingIndicator) {
                    existingIndicator.remove();
                }
                
                // Add new source indicator
                const sourceIndicator = document.createElement('div');
                sourceIndicator.className = 'source-indicator notification is-info is-light mb-2';
                let sourceText = '';
                if (data.source === 'flow_wcs') {
                    sourceText = '🔄 Flow WCS API (即時同步)';
                } else if (data.source === 'config') {
                    sourceText = '📝 配置檔案 (flow_functions.yaml)';
                } else {
                    sourceText = '📦 本地靜態函數庫';
                }
                sourceIndicator.innerHTML = `
                    <button class="delete" onclick="this.parentElement.remove()"></button>
                    <strong>函數來源:</strong> ${sourceText}
                    ${data.version ? `<span class="tag is-light ml-2">v${data.version}</span>` : ''}
                `;
                functionList.insertBefore(sourceIndicator, functionList.firstChild);
            }
            
            // Convert to flat array for function library validation (include defaults if available)
            window.functionLibrary = [];
            // Build function index for O(1) lookup (replaces functionLibraryFlat)
            window.functionIndex = {};
            
            for (const category in availableFunctions) {
                for (const func of availableFunctions[category]) {
                    const funcData = {
                        name: func.name,
                        category: category,
                        params: func.params || [],
                        defaults: func.defaults || {},  // Include defaults from API
                        description: func.description || '',
                        returns: func.returns || 'any'
                    };
                    
                    // Add to array (for iteration)
                    window.functionLibrary.push(funcData);
                    
                    // Add to index (for O(1) lookup)
                    window.functionIndex[func.name] = funcData;
                }
            }
            
            // Show data source notification
            if (data.source) {
                const sourceLabels = {
                    'flow_wcs_live': '✅ 從 Flow WCS API 載入函數庫（最新版）',
                    'cache': '📦 從快取載入函數庫',
                    'static': '📋 使用靜態函數庫'
                };
                const message = sourceLabels[data.source] || `從 ${data.source} 載入函數庫`;
                showNotification(message, data.source === 'flow_wcs_live' ? 'success' : 'info');
            }
            
            console.log(`Function library loaded from ${data.source || source} with ${window.functionLibrary.length} functions`);
            
            // Render with 'all' category by default
            renderFunctionLibrary('all');
            populateFunctionSelect();
        }
    } catch (error) {
        console.error(`Failed to load functions from ${source}:`, error);
        
        // Try fallback to local if flow_wcs fails
        if (source === 'flow_wcs') {
            console.log('Attempting fallback to local functions...');
            await loadAvailableFunctions('local');
        } else {
            // Load function library (external or fallback)
            loadFunctionLibrary();
        }
    }
}

/**
 * Load function library from external module or use fallback
 */
async function loadFunctionLibrary() {
    try {
        // Try to load the external function library module
        const module = await import('./functionLibrary.js');
        availableFunctions = module.functionLibrary;
        console.log('✅ Loaded external function library with', Object.keys(availableFunctions).length, 'categories');
        
        // Convert to flat array for validation and build index
        window.functionLibrary = [];
        window.functionIndex = {};
        
        for (const category in availableFunctions) {
            for (const func of availableFunctions[category]) {
                const funcData = {
                    name: func.name,
                    category: category,
                    params: func.params || [],
                    defaults: func.defaults || {},
                    description: func.description || '',
                    returns: func.returns || 'any'
                };
                
                // Add to array (for iteration)
                window.functionLibrary.push(funcData);
                
                // Add to index (for O(1) lookup)
                window.functionIndex[func.name] = funcData;
            }
        }
        
        // Render the library
        renderFunctionLibrary('all');
        populateFunctionSelect();
        
    } catch (error) {
        console.warn('⚠️ Could not load external function library, using fallback:', error);
        initializeDefaultFunctionLibrary();
    }
}

/**
 * Initialize default function library (fallback)
 */
function initializeDefaultFunctionLibrary() {
    // Define the default function library with explicit default values
    availableFunctions = {
        'query': [
            { name: 'query.locations', params: ['type', 'rooms', 'has_rack'], 
              defaults: { 
                  type: 'rack',           // 位置類型: rack, station, parking 等
                  rooms: ['room01'],      // 房間列表範例
                  has_rack: true          // 是否有架台
              },
              description: '查詢位置資料' },
            { name: 'query.agv_status', params: ['agv_id'], 
              defaults: { agv_id: 'agv01' },
              description: '查詢 AGV 狀態' },
            { name: 'query.rack_status', params: ['rack_id'], 
              defaults: { rack_id: 'rack01' },
              description: '查詢架台狀態' },
            { name: 'query.task_status', params: ['task_id'], 
              defaults: { task_id: 'task_001' },
              description: '查詢任務狀態' }
        ],
        'check': [
            { name: 'check.empty', params: ['list'], 
              defaults: { list: '${result}' },
              description: '檢查列表是否為空' },
            { name: 'check.agv_available', params: ['agv_id'], 
              defaults: { agv_id: 'agv01' },
              description: '檢查 AGV 是否可用' },
            { name: 'check.location_accessible', params: ['location_id'], 
              defaults: { location_id: 'station01' },
              description: '檢查位置是否可達' },
            { name: 'check.condition', params: ['expression'], 
              defaults: { expression: '${result} > 0' },
              description: '檢查條件表達式' }
        ],
        'task': [
            { name: 'task.create', params: ['type', 'params'], 
              defaults: { 
                  type: 'transport',
                  params: {
                      from: 'rack01',
                      to: 'station01',
                      priority: 100
                  }
              },
              description: '創建任務' },
            { name: 'task.assign', params: ['task_id', 'agv_id'], 
              defaults: { task_id: '${task_id}', agv_id: 'agv01' },
              description: '分配任務給 AGV' },
            { name: 'task.cancel', params: ['task_id'], 
              defaults: { task_id: '${task_id}' },
              description: '取消任務' },
            { name: 'task.update', params: ['task_id', 'status'], 
              defaults: { task_id: '${task_id}', status: 'completed' },
              description: '更新任務狀態' }
        ],
        'action': [
            { name: 'action.move_agv', params: ['agv_id', 'target'], 
              defaults: { agv_id: 'agv01', target: 'station01' },
              description: '移動 AGV' },
            { name: 'action.load', params: ['agv_id', 'item_id'], 
              defaults: { agv_id: 'agv01', item_id: 'item_001' },
              description: '裝載物品' },
            { name: 'action.unload', params: ['agv_id', 'location'], 
              defaults: { agv_id: 'agv01', location: 'station01' },
              description: '卸載物品' },
            { name: 'action.wait', params: ['seconds'], 
              defaults: { seconds: 5 },
              description: '等待指定時間' },
            { name: 'action.log', params: ['message', 'level'], 
              defaults: { message: 'Step completed', level: 'info' },
              description: '記錄日誌' },
            { name: 'action.optimize_batch', params: ['tasks'], 
              defaults: { tasks: '${task_list}' },
              description: '批次優化' },
            { name: 'action.update_metrics', params: ['metrics'], 
              defaults: { 
                  metrics: {
                      throughput: 100,
                      efficiency: 0.95,
                      errors: 0
                  }
              },
              description: '更新指標' }
        ],
        'control': [
            { name: 'control.if', params: ['condition', 'then', 'else'], 
              defaults: { 
                  condition: '${result} == true',
                  then: 'action.log',
                  else: 'action.wait'
              },
              description: '條件控制' },
            { name: 'control.loop', params: ['items', 'action'], 
              defaults: { 
                  items: '${list}',
                  action: 'action.process_item'
              },
              description: '循環控制' },
            { name: 'control.parallel', params: ['actions'], 
              defaults: { 
                  actions: ['action.move_agv', 'action.load']
              },
              description: '並行執行' },
            { name: 'control.retry', params: ['action', 'max_retries'], 
              defaults: { 
                  action: 'action.move_agv',
                  max_retries: 3
              },
              description: '重試執行' },
            { name: 'control.count', params: ['items'], 
              defaults: { items: '${list}' },
              description: '計數' }
        ],
        'special': [
            { name: 'foreach', params: ['items', 'var', 'steps', 'max_iterations'], 
              defaults: { 
                  items: '${list}',
                  var: 'item',
                  steps: [],
                  max_iterations: 100
              },
              description: '迭代處理' },
            { name: 'wait', params: ['seconds'], 
              defaults: { seconds: 3 },
              description: '等待' },
            { name: 'return', params: ['value'], 
              defaults: { value: '${result}' },
              description: '返回值' },
            { name: 'break', params: [], 
              defaults: {},
              description: '中斷循環' },
            { name: 'continue', params: [], 
              defaults: {},
              description: '繼續循環' }
        ]
    };
    
    // Convert to flat array for function library validation and build index
    window.functionLibrary = [];
    window.functionIndex = {};
    
    for (const category in availableFunctions) {
        for (const func of availableFunctions[category]) {
            const funcData = {
                name: func.name,
                category: category,
                params: func.params || [],
                defaults: func.defaults || {},  // Include defaults from fallback
                description: func.description || '',
                returns: func.returns || 'any'
            };
            
            // Add to array (for iteration)
            window.functionLibrary.push(funcData);
            
            // Add to index (for O(1) lookup)
            window.functionIndex[func.name] = funcData;
        }
    }
    
    console.log('Default function library initialized with', window.functionLibrary.length, 'functions');
    
    // Render with 'all' category by default
    renderFunctionLibrary('all');
    populateFunctionSelect();
}

/**
 * Render the function library
 */
function renderFunctionLibrary(filterCategory = 'all') {
    const functionList = document.getElementById('function-list');
    if (!functionList) return;
    
    functionList.innerHTML = '';
    
    // Check if functions are loaded
    if (!availableFunctions || Object.keys(availableFunctions).length === 0) {
        functionList.innerHTML = '<p class="has-text-grey-light has-text-centered">載入函數中...</p>';
        return;
    }
    
    // Create a container for each category
    if (filterCategory === 'all') {
        // Show all categories with headers
        for (const [category, functions] of Object.entries(availableFunctions)) {
            const categoryDiv = document.createElement('div');
            categoryDiv.className = 'function-category';
            
            const categoryHeader = document.createElement('div');
            categoryHeader.className = 'function-category-header';
            categoryHeader.innerHTML = `<i class="fas fa-folder mr-2"></i>${category.charAt(0).toUpperCase() + category.slice(1)}`;
            categoryDiv.appendChild(categoryHeader);
            
            const itemsDiv = document.createElement('div');
            itemsDiv.className = 'function-items';
            
            functions.forEach(func => {
                const funcElement = createFunctionElement(func, category);
                itemsDiv.appendChild(funcElement);
            });
            
            categoryDiv.appendChild(itemsDiv);
            functionList.appendChild(categoryDiv);
        }
    } else {
        // Show only functions from selected category
        const functions = availableFunctions[filterCategory];
        if (functions && functions.length > 0) {
            functions.forEach(func => {
                const funcElement = createFunctionElement(func, filterCategory);
                functionList.appendChild(funcElement);
            });
        } else {
            functionList.innerHTML = '<p class="has-text-grey-light has-text-centered">此分類無函數</p>';
        }
    }
}

/**
 * Create a function element
 */
function createFunctionElement(func, category) {
    const funcElement = document.createElement('div');
    funcElement.className = 'function-item';
    funcElement.setAttribute('draggable', 'true');
    funcElement.setAttribute('data-function', func.name);
    funcElement.setAttribute('data-category', category);
    
    funcElement.innerHTML = `
        <div class="function-name">${func.name}</div>
        <div class="function-description">${func.description || ''}</div>
        <div class="function-params">
            ${func.params ? func.params.map(p => `<span class="function-param">${p}</span>`).join('') : ''}
        </div>
    `;
    
    // Add drag event handlers
    funcElement.addEventListener('dragstart', handleFunctionDragStart);
    funcElement.addEventListener('dragend', handleFunctionDragEnd);
    
    // Add click to insert - show modal with pre-filled values
    funcElement.addEventListener('click', () => showAddFunctionModal(func));
    
    return funcElement;
}

/**
 * Populate function select dropdown
 */
function populateFunctionSelect() {
    const select = document.getElementById('new-step-exec');
    if (!select) return;
    
    select.innerHTML = '<option value="">選擇函數...</option>';
    
    for (const [category, functions] of Object.entries(availableFunctions)) {
        const optgroup = document.createElement('optgroup');
        optgroup.label = category.charAt(0).toUpperCase() + category.slice(1);
        
        functions.forEach(func => {
            const option = document.createElement('option');
            option.value = func.name;
            option.textContent = `${func.name} - ${func.description}`;
            optgroup.appendChild(option);
        });
        
        select.appendChild(optgroup);
    }
}

/**
 * Render the workflow sections
 */
function renderWorkflow() {
    const currentFlow = flowStore.getState();
    const container = document.getElementById('workflow-sections');
    if (!container) return;
    
    container.innerHTML = '';
    
    // Get workflow from store
    const state = flowStore.getState();
    const workflow = state.workflow || [];
    
    if (workflow.length === 0) {
        container.innerHTML = `
            <div class="notification is-info is-light">
                <p>尚無工作流程區段。點擊「新增區段」開始建立流程。</p>
            </div>
        `;
        return;
    }
    
    workflow.forEach((section, sectionIndex) => {
        const sectionElement = createSectionElement(section, sectionIndex);
        container.appendChild(sectionElement);
    });
    
    // Mark as unsaved when workflow is modified
    if (typeof updateSyncStatus === 'function') {
        updateSyncStatus(false);
    }
}

/**
 * Create a section element
 */
function createSectionElement(section, sectionIndex) {
    const sectionDiv = document.createElement('div');
    sectionDiv.className = 'workflow-section';
    sectionDiv.setAttribute('data-section-index', sectionIndex);
    // Remove drag functionality from workflow-section (left panel)
    
    // Section header - clickable to select section
    const headerDiv = document.createElement('div');
    headerDiv.className = 'section-header';
    headerDiv.onclick = () => selectSection(sectionIndex);
    headerDiv.title = '點擊選擇此區段';  // Add tooltip for section
    
    headerDiv.innerHTML = `
        <div class="section-title">
            <span class="icon"><i class="fas fa-folder"></i></span>
            <span>${section.section || 'Unnamed Section'}</span>
        </div>
        <div class="section-actions">
            <button class="button is-small" onclick="event.stopPropagation(); moveSection(${sectionIndex}, -1)" 
                    title="上移" ${sectionIndex === 0 ? 'disabled' : ''}>
                <span class="icon"><i class="fas fa-arrow-up"></i></span>
            </button>
            <button class="button is-small" onclick="event.stopPropagation(); moveSection(${sectionIndex}, 1)" 
                    title="下移" ${sectionIndex === flowStore.getState().workflow.length - 1 ? 'disabled' : ''}>
                <span class="icon"><i class="fas fa-arrow-down"></i></span>
            </button>
            <button class="button is-small" onclick="event.stopPropagation(); addStepToSection(${sectionIndex})" title="新增步驟">
                <span class="icon"><i class="fas fa-plus"></i></span>
            </button>
            <button class="button is-small" onclick="event.stopPropagation(); deleteSection(${sectionIndex})" title="刪除區段">
                <span class="icon"><i class="fas fa-trash"></i></span>
            </button>
        </div>
    `;
    
    sectionDiv.appendChild(headerDiv);
    
    // Section steps (simplified for left panel)
    const stepsDiv = document.createElement('div');
    stepsDiv.className = 'section-steps';
    
    if (section.steps && section.steps.length > 0) {
        section.steps.forEach((step, stepIndex) => {
            const stepElement = createStepElementForWorkflow(step, sectionIndex, stepIndex);
            stepsDiv.appendChild(stepElement);
        });
    } else {
        stepsDiv.innerHTML = `
            <div class="empty-steps">
                <small class="has-text-grey">無步驟</small>
            </div>
        `;
    }
    
    sectionDiv.appendChild(stepsDiv);
    
    return sectionDiv;
}

/**
 * Create a simplified step element for workflow panel (left side)
 */
function createStepElementForWorkflow(step, sectionIndex, stepIndex) {
    const stepDiv = document.createElement('div');
    stepDiv.className = 'workflow-step-simple';
    stepDiv.setAttribute('data-step-index', stepIndex);
    stepDiv.setAttribute('data-section-index', sectionIndex);
    // Add click handler to highlight step and select for editing
    stepDiv.onclick = () => {
        // Remove active class from all workflow steps
        document.querySelectorAll('.workflow-step-simple').forEach(s => {
            s.classList.remove('active');
        });
        // Add active class to clicked step
        stepDiv.classList.add('active');
        // Highlight in visual editor and select for editing
        highlightStepInVisualEditor(sectionIndex, stepIndex);
        selectStep(sectionIndex, stepIndex);
    };
    
    // Build store variable HTML with purple color (like variable panel)
    let storeHtml = '';
    if (step.store) {
        storeHtml = `
            <div class="workflow-step-store">
                <span class="workflow-step-store-tag" style="background: #f3e7ff; color: #9b59b6;">
                    <i class="fas fa-save"></i> → ${step.store}
                </span>
            </div>
        `;
    }
    
    // Build compact params HTML for function line
    let paramsHtml = '';
    if (step.params) {
        let params = step.params;
        // Parse if it's a string
        if (typeof params === 'string') {
            try {
                params = JSON.parse(params);
            } catch (e) {
                params = {};
            }
        }
        
        const paramKeys = Object.keys(params);
        if (paramKeys.length > 0) {
            // Limit to first 3 params
            const displayParams = paramKeys.slice(0, 3);
            const moreCount = paramKeys.length - 3;
            
            paramsHtml = '<span class="workflow-step-params">';
            displayParams.forEach(key => {
                // Truncate long param names
                const displayKey = key.length > 8 ? key.substring(0, 6) + '..' : key;
                paramsHtml += `<span class="workflow-step-param">${displayKey}</span>`;
            });
            if (moreCount > 0) {
                paramsHtml += `<span class="workflow-step-param">+${moreCount}</span>`;
            }
            paramsHtml += '</span>';
        }
    }
    
    // Left panel: display only (no actions needed)
    stepDiv.innerHTML = `
        <div class="step-header-simple">
            <div class="step-icon">${stepIndex + 1}</div>
            <div class="step-details">
                <div class="step-id">${step.id || 'Unnamed Step'}</div>
                <div class="step-function-row">
                    <span class="step-function">${step.exec || 'No function'}</span>
                    ${paramsHtml}
                </div>
                ${storeHtml}
            </div>
            <div class="step-actions">
                <button class="button is-small delete-step-btn" 
                        onclick="event.stopPropagation(); confirmDeleteStep(${sectionIndex}, ${stepIndex})" 
                        title="刪除步驟">
                    <span class="icon">
                        <i class="fas fa-trash"></i>
                    </span>
                </button>
            </div>
        </div>
    `;
    
    return stepDiv;
}

/**
 * Create a full-featured step element for editor panel (center area)
 */
function createStepElementForEditor(step, sectionIndex, stepIndex) {
    const stepDiv = document.createElement('div');
    stepDiv.className = 'workflow-step-editor';
    stepDiv.setAttribute('data-step-index', stepIndex);
    stepDiv.setAttribute('data-section-index', sectionIndex);
    // REMOVED: Left panel steps should not be draggable
    stepDiv.onclick = () => selectStep(sectionIndex, stepIndex);
    
    // REMOVED: Drag event handlers - left panel doesn't support dragging
    
    let conditionHtml = '';
    if (step.skip_if) {
        conditionHtml = `<div class="step-condition">⚠️ Skip if: ${step.skip_if}</div>`;
    } else if (step.skip_if_not) {
        conditionHtml = `<div class="step-condition">⚠️ Skip if not: ${step.skip_if_not}</div>`;
    }
    
    stepDiv.innerHTML = `
        <div class="step-header-editor">
            <span class="step-drag-handle" title="拖動調整順序">
                <i class="fas fa-grip-vertical"></i>
            </span>
            <div class="step-icon">${stepIndex + 1}</div>
            <div class="step-details">
                <div class="step-id">${step.id || 'Unnamed Step'}</div>
                <div class="step-function">${step.exec || 'No function'}</div>
                ${conditionHtml}
            </div>
            <div class="step-actions">
                <button class="button is-small" onclick="event.stopPropagation(); moveStep(${sectionIndex}, ${stepIndex}, -1)" 
                        title="上移步驟" ${stepIndex === 0 ? 'disabled' : ''}>
                    <span class="icon"><i class="fas fa-arrow-up"></i></span>
                </button>
                <button class="button is-small" onclick="event.stopPropagation(); moveStep(${sectionIndex}, ${stepIndex}, 1)" 
                        title="下移步驟" ${stepIndex === currentFlow.workflow[sectionIndex].steps.length - 1 ? 'disabled' : ''}>
                    <span class="icon"><i class="fas fa-arrow-down"></i></span>
                </button>
                <button class="button is-small is-danger" onclick="event.stopPropagation(); deleteStep(${sectionIndex}, ${stepIndex})">
                    <span class="icon"><i class="fas fa-trash"></i></span>
                </button>
            </div>
        </div>
        ${step.store ? `<div class="step-store">→ ${step.store}</div>` : ''}
    `;
    
    return stepDiv;
}

/**
 * Legacy function for compatibility - now redirects to editor version
 */
function createStepElement(step, sectionIndex, stepIndex) {
    const currentFlow = flowStore.getState();
    return createStepElementForEditor(step, sectionIndex, stepIndex);
}


/**
 * Show notification message
 */
function showNotification(message, type = 'info') {
    const notification = document.createElement('div');
    notification.className = `notification is-${type}`;
    notification.style.cssText = 'position: fixed; top: 20px; right: 20px; z-index: 9999; min-width: 300px;';
    notification.innerHTML = `
        <button class="delete" onclick="this.parentElement.remove()"></button>
        ${message}
    `;
    document.body.appendChild(notification);
    
    // Auto remove after 3 seconds
    setTimeout(() => {
        if (notification.parentElement) {
            notification.remove();
        }
    }, 3000);
}

/**
 * Select a section for editing
 */
function selectSection(sectionIndex) {
    const currentFlow = flowStore.getState();
    currentSectionIndex = sectionIndex;
    currentStepIndex = null;
    
    // Update UI - remove active from all sections and steps
    document.querySelectorAll('.workflow-section').forEach(el => {
        el.classList.remove('active');
        el.classList.remove('section-highlight');
    });
    document.querySelectorAll('.workflow-step').forEach(el => el.classList.remove('is-active'));
    document.querySelectorAll('.workflow-step-simple').forEach(el => el.classList.remove('is-active'));
    document.querySelectorAll('.visual-section').forEach(el => {
        el.classList.remove('selected');
        el.classList.remove('section-highlight');
    });
    
    // Add active to left panel workflow-section and highlight animation
    const leftPanelSection = document.querySelector(`.workflow-section[data-section-index="${sectionIndex}"]`);
    if (leftPanelSection) {
        leftPanelSection.classList.add('active');
        
        // Add highlight animation
        leftPanelSection.classList.remove('section-highlight');
        void leftPanelSection.offsetWidth; // Force reflow
        leftPanelSection.classList.add('section-highlight');
        
        // Remove highlight after animation
        setTimeout(() => {
            leftPanelSection.classList.remove('section-highlight');
        }, 1000);
    }
    
    // Add selected to visual section in center panel with highlight animation
    const visualSection = document.querySelector(`.visual-section[data-section-index="${sectionIndex}"]`);
    if (visualSection) {
        visualSection.classList.add('selected');
        
        // Add highlight animation
        visualSection.classList.remove('section-highlight');
        void visualSection.offsetWidth; // Force reflow
        visualSection.classList.add('section-highlight');
        
        // Remove highlight after animation
        setTimeout(() => {
            visualSection.classList.remove('section-highlight');
        }, 1000);
        
        // Scroll into view
        visualSection.scrollIntoView({
            behavior: 'smooth',
            block: 'center'
        });
    }
    
    // Show section editor
    showSectionEditor(currentFlow.workflow[sectionIndex]);
}

/**
 * Select a step for editing
 */
function selectStep(sectionIndex, stepIndex) {
    const currentFlow = flowStore.getState();
    currentSectionIndex = sectionIndex;
    currentStepIndex = stepIndex;
    
    // Update UI
    document.querySelectorAll('.section-header').forEach(el => el.classList.remove('is-active'));
    document.querySelectorAll('.workflow-step').forEach(el => el.classList.remove('is-active'));
    
    const stepElement = document.querySelector(`[data-section-index="${sectionIndex}"] [data-step-index="${stepIndex}"]`);
    if (stepElement) {
        stepElement.classList.add('is-active');
    }
    
    // Show step editor
    showStepEditor(currentFlow.workflow[sectionIndex].steps[stepIndex]);
}

/**
 * Show section editor
 */
function showSectionEditor(section) {
    const editor = document.getElementById('section-editor');
    if (!editor) return;
    
    // Build steps list HTML
    let stepsHtml = '';
    if (section.steps && section.steps.length > 0) {
        stepsHtml = `
            <div class="field">
                <label class="label">區段內的步驟</label>
                <div class="box steps-list-box">
        `;
        section.steps.forEach((step, index) => {
            // Parse parameters to show them as tags
            let paramsHtml = '';
            if (step.params) {
                try {
                    const params = typeof step.params === 'string' ? JSON.parse(step.params) : step.params;
                    const paramKeys = Object.keys(params);
                    if (paramKeys.length > 0) {
                        paramsHtml = `
                            <div class="step-params-display">
                                ${paramKeys.map(key => `<span class="step-param-tag">${key}</span>`).join('')}
                            </div>
                        `;
                    }
                } catch (e) {
                    // If params is not valid JSON, just show as text
                    paramsHtml = `<div class="step-params-display"><span class="step-param-tag">params: ${step.params}</span></div>`;
                }
            }
            
            // Store variable display with purple color (like variable panel)
            let storeHtml = '';
            if (step.store) {
                storeHtml = `
                    <div class="step-store-display">
                        <span class="tag" style="background: #f3e7ff; color: #9b59b6;">
                            <span class="icon is-small"><i class="fas fa-save"></i></span>
                            <span>→ ${step.store}</span>
                        </span>
                    </div>
                `;
            }
            
            stepsHtml += `
                <div class="media step-media">
                    <div class="media-content">
                        <p class="is-size-7">
                            <strong>${index + 1}. ${step.id}</strong><br>
                            <span class="has-text-primary">${step.exec}</span>
                            ${paramsHtml}
                            ${storeHtml}
                            ${step.skip_if ? `<br><span class="has-text-warning">⚠️ Skip if: ${step.skip_if}</span>` : ''}
                            ${step.skip_if_not ? `<br><span class="has-text-warning">⚠️ Skip if not: ${step.skip_if_not}</span>` : ''}
                        </p>
                    </div>
                    <div class="media-right">
                        <button class="button is-small is-info" onclick="selectStep(${currentSectionIndex}, ${index})">
                            <span class="icon"><i class="fas fa-edit"></i></span>
                        </button>
                    </div>
                </div>
            `;
        });
        stepsHtml += `
                </div>
            </div>
        `;
    } else {
        stepsHtml = `
            <div class="field">
                <label class="label">區段內的步驟</label>
                <div class="notification is-light">
                    <p class="has-text-grey">此區段尚無步驟。點擊上方的 + 按鈕新增步驟。</p>
                </div>
            </div>
        `;
    }
    
    editor.innerHTML = `
        <div class="editor-form">
            <h3 class="title is-4">編輯區段</h3>
            <div class="field">
                <label class="label">區段名稱</label>
                <input class="input" type="text" id="edit-section-name" value="${section.section || ''}">
            </div>
            <div class="field">
                <label class="label">描述</label>
                <textarea class="textarea" id="edit-section-description">${section.description || ''}</textarea>
            </div>
            <div class="field">
                <label class="label">執行條件 <span class="has-text-grey has-text-weight-normal">(留空表示無條件執行)</span></label>
                <input class="input" type="text" id="edit-section-condition" value="${section.condition || ''}" placeholder="例如: \${has_task} 或 \${count} > 0">
                <p class="help">
                    • 如果設定條件，只有條件為 true 時才會執行此區段<br>
                    • 每個區段的條件獨立判斷，不影響其他區段執行<br>
                    • 可使用前面區段儲存的變數作為條件
                </p>
            </div>
            ${stepsHtml}
            <div class="field">
                <div class="buttons">
                    <button class="button is-primary" onclick="updateSection()">
                        <span class="icon"><i class="fas fa-save"></i></span>
                        <span>更新區段</span>
                    </button>
                    <button class="button is-info" onclick="addStepToSection(${currentSectionIndex})">
                        <span class="icon"><i class="fas fa-plus"></i></span>
                        <span>新增步驟</span>
                    </button>
                    <button class="button is-danger" onclick="deleteSection(${currentSectionIndex})">
                        <span class="icon"><i class="fas fa-trash"></i></span>
                        <span>刪除區段</span>
                    </button>
                </div>
            </div>
        </div>
    `;
}

/**
 * Show step editor
 */
function showStepEditor(step) {
    const editor = document.getElementById('section-editor');
    if (!editor) return;
    
    editor.innerHTML = `
        <div class="editor-form">
            <h3 class="title is-4">編輯步驟</h3>
            <div class="field">
                <label class="label">步驟 ID</label>
                <input class="input" type="text" id="edit-step-id" value="${step.id || ''}">
            </div>
            <div class="field">
                <label class="label">執行函數</label>
                <input class="input" type="text" id="edit-step-exec" value="${step.exec || ''}">
            </div>
            <div class="field">
                <label class="label">參數 <span class="has-text-grey has-text-weight-normal">(JSON 格式，儲存時自動轉為 YAML)</span></label>
                <textarea class="textarea params-editor" id="edit-step-params">${JSON.stringify(step.params || {}, null, 2)}</textarea>
                <p class="help">使用 JSON 格式編輯，系統會自動轉換為 YAML 儲存</p>
            </div>
            <div class="field">
                <label class="label">儲存結果至</label>
                <input class="input" type="text" id="edit-step-store" value="${step.store || ''}" placeholder="variable_name">
            </div>
            <div class="field">
                <label class="label">跳過條件 (Skip If)</label>
                <input class="input" type="text" id="edit-step-skip-if" value="${step.skip_if || ''}" placeholder="\${condition}">
            </div>
            <div class="field">
                <label class="label">跳過條件 (Skip If Not)</label>
                <input class="input" type="text" id="edit-step-skip-if-not" value="${step.skip_if_not || ''}" placeholder="\${condition}">
            </div>
            <div class="field">
                <button class="button is-primary" onclick="updateStep()">更新步驟</button>
                <button class="button is-danger" onclick="deleteStep(${currentSectionIndex}, ${currentStepIndex})">刪除步驟</button>
            </div>
        </div>
    `;
}

/**
 * Add a new section
 */
/**
 * Mark flow as modified (needs sync)
 */
function markFlowAsModified() {
    updateSyncStatus('modified');
}

function addSection() {
    document.getElementById('add-section-modal').classList.add('is-active');
}

/**
 * Confirm adding a new section
 */
function confirmAddSection() {
    const name = document.getElementById('new-section-name').value;
    const description = document.getElementById('new-section-description').value;
    const condition = document.getElementById('new-section-condition').value;
    
    if (!name) {
        showAlert('請輸入區段名稱', 'warning');
        return;
    }
    
    const newSection = {
        section: name,
        description: description,
        steps: []
    };
    
    if (condition) {
        newSection.condition = condition;
    }
    
    // Use flowStore to add section
    flowStore.addSection(newSection);
    
    // UI updates are handled by flowStore event
    closeModal('add-section-modal');
    
    // Clear form
    document.getElementById('new-section-name').value = '';
    document.getElementById('new-section-description').value = '';
    document.getElementById('new-section-condition').value = '';
}

/**
 * Add a step to a section
 */
function addStepToSection(sectionIndex) {
    currentSectionIndex = sectionIndex;
    currentStepIndex = null; // Clear editing state for adding new step
    
    // Show section selector for new steps
    const sectionField = document.getElementById('new-step-section').closest('.field');
    if (sectionField) {
        sectionField.style.display = 'block';
    }
    
    // Populate section dropdown
    const sectionSelect = document.getElementById('new-step-section');
    sectionSelect.innerHTML = '<option value="">選擇區段...</option>';
    sectionSelect.innerHTML += '<option value="new">📝 新增區段</option>';
    
    // Add existing sections and select the current one
    const currentFlow = flowStore.getState();
    if (currentFlow.workflow && currentFlow.workflow.length > 0) {
        currentFlow.workflow.forEach((section, index) => {
            const sectionName = section.section || `區段 ${index + 1}`;
            const option = document.createElement('option');
            option.value = index;
            option.textContent = sectionName;
            if (index === sectionIndex) {
                option.selected = true;  // Select the section where the button was clicked
            }
            sectionSelect.appendChild(option);
        });
    }
    
    // Ensure modal is in "add" mode
    document.querySelector('#add-step-modal .modal-card-title').textContent = '新增步驟';
    const submitButton = document.querySelector('#add-step-modal .button.is-success');
    if (submitButton) {
        submitButton.textContent = '新增';
        submitButton.onclick = () => confirmAddStep();
    }
    
    // Clear form fields
    document.getElementById('new-step-id').value = '';
    document.getElementById('new-step-exec').value = '';
    document.getElementById('new-step-params').value = '{}';
    document.getElementById('new-step-store').value = '';
    document.getElementById('new-step-skip-if-enabled').checked = false;
    document.getElementById('new-step-skip-if').value = '';
    document.getElementById('new-step-skip-if').style.display = 'none';
    
    document.getElementById('add-step-modal').classList.add('is-active');
}

/**
 * Confirm adding a new step
 */
function confirmAddStep() {
    const currentFlow = flowStore.getState();
    const stepId = document.getElementById('new-step-id').value;
    const exec = document.getElementById('new-step-exec').value;
    const paramsText = document.getElementById('new-step-params').value;
    const store = document.getElementById('new-step-store').value;
    const skipIfEnabled = document.getElementById('new-step-skip-if-enabled').checked;
    const skipIf = document.getElementById('new-step-skip-if').value;
    
    if (!stepId || !exec) {
        showAlert('請輸入步驟 ID 和執行函數', 'warning');
        return;
    }
    
    const stepData = {
        id: stepId,
        exec: exec
    };
    
    // Parse parameters
    if (paramsText) {
        try {
            stepData.params = JSON.parse(paramsText);
        } catch (e) {
            showAlert('參數格式錯誤，請輸入有效的 JSON', 'error');
            return;
        }
    }
    
    if (store) {
        stepData.store = store;
    }
    
    if (skipIfEnabled && skipIf) {
        stepData.skip_if = skipIf;
    }
    
    // Check if we're editing existing step or adding new one
    const isEditing = currentStepIndex !== null && currentStepIndex !== undefined;
    
    // For new steps, get the selected section from dropdown
    if (!isEditing) {
        const selectedSection = document.getElementById('new-step-section').value;
        
        if (!selectedSection) {
            showAlert('請選擇要加入的區段', 'warning');
            return;
        }
        
        if (selectedSection === 'new') {
            // Create a new section
            const newSection = {
                section: '新區段',
                description: '',
                steps: [stepData]
            };
            flowStore.addSection(newSection);
            showNotification(`✅ 已新增步驟到新區段: ${stepData.id}`, 'success');
        } else {
            // Add to existing section
            const sectionIndex = parseInt(selectedSection);
            if (!currentFlow.workflow[sectionIndex].steps) {
                currentFlow.workflow[sectionIndex].steps = [];
            }
            flowStore.addStep(sectionIndex, stepData);
            showNotification(`✅ 步驟已新增: ${stepData.id}`, 'success');
        }
    } else {
        // Update existing step (editing mode)
        if (!currentFlow.workflow[currentSectionIndex].steps) {
            currentFlow.workflow[currentSectionIndex].steps = [];
        }
        currentFlow.workflow[currentSectionIndex].steps[currentStepIndex] = stepData;
        showNotification(`✅ 步驟已更新: ${stepData.id}`, 'success');
        
        // Reset editing state
        currentStepIndex = null;
    }
    
    // Update all views
    renderWorkflow();
    renderVisualEditor();
    updateYamlEditor();
    updateVariablesDisplay();
    closeModal('add-step-modal');
    
    // Reset modal title and button text for next use
    document.querySelector('#add-step-modal .modal-card-title').textContent = '新增步驟';
    const submitButton = document.querySelector('#add-step-modal .button.is-success');
    if (submitButton) {
        submitButton.textContent = '新增';
    }
    
    // Show section field for next time (default state for new steps)
    const sectionField = document.getElementById('new-step-section').closest('.field');
    if (sectionField) {
        sectionField.style.display = 'block';
    }
    
    // Clear form
    document.getElementById('new-step-id').value = '';
    document.getElementById('new-step-exec').value = '';
    document.getElementById('new-step-params').value = '{}';
    document.getElementById('new-step-store').value = '';
    document.getElementById('new-step-skip-if-enabled').checked = false;
    document.getElementById('new-step-skip-if').value = '';
}

/**
 * Update current section
 */
function updateSection() {
    const currentFlow = flowStore.getState();
    if (currentSectionIndex === null) return;
    
    const section = currentFlow.workflow[currentSectionIndex];
    section.section = document.getElementById('edit-section-name').value;
    section.description = document.getElementById('edit-section-description').value;
    
    const condition = document.getElementById('edit-section-condition').value;
    if (condition) {
        section.condition = condition;
    } else {
        delete section.condition;
    }
    
    renderWorkflow();
    updateYamlEditor();
}

/**
 * Update current step
 */
function updateStep() {
    const currentFlow = flowStore.getState();
    if (currentSectionIndex === null || currentStepIndex === null) return;
    
    const step = currentFlow.workflow[currentSectionIndex].steps[currentStepIndex];
    step.id = document.getElementById('edit-step-id').value;
    step.exec = document.getElementById('edit-step-exec').value;
    
    const paramsText = document.getElementById('edit-step-params').value;
    if (paramsText) {
        try {
            step.params = JSON.parse(paramsText);
        } catch (e) {
            showAlert('參數格式錯誤', 'error');
            return;
        }
    } else {
        delete step.params;
    }
    
    const store = document.getElementById('edit-step-store').value;
    if (store) {
        step.store = store;
    } else {
        delete step.store;
    }
    
    const skipIf = document.getElementById('edit-step-skip-if').value;
    if (skipIf) {
        step.skip_if = skipIf;
    } else {
        delete step.skip_if;
    }
    
    const skipIfNot = document.getElementById('edit-step-skip-if-not').value;
    if (skipIfNot) {
        step.skip_if_not = skipIfNot;
    } else {
        delete step.skip_if_not;
    }
    
    renderWorkflow();
    updateYamlEditor();
}

/**
 * Delete Confirmation Modal Handler
 */
let deleteConfirmationCallback = null;

function showDeleteConfirmation(message, callback, options = {}) {
    const modal = document.getElementById('delete-confirmation-modal');
    const messageElement = document.getElementById('delete-confirmation-message');
    const confirmButton = document.getElementById('confirm-delete-button');
    const modalTitle = modal.querySelector('.modal-card-title span:last-child');
    const modalHeader = modal.querySelector('.modal-card-head');
    const warningMessage = modal.querySelector('.message.is-warning');
    
    // Set title and style based on operation type
    if (options.isImport) {
        modalTitle.textContent = '確認導入';
        modalHeader.style.background = '#3273dc';
        modalHeader.style.borderBottomColor = '#3273dc';
        confirmButton.className = 'button is-info';
        confirmButton.innerHTML = '<span class="icon"><i class="fas fa-file-import"></i></span><span>確定導入</span>';
        warningMessage.style.display = 'block';
    } else {
        modalTitle.textContent = '確認刪除';
        modalHeader.style.background = '#f14668';
        modalHeader.style.borderBottomColor = '#f14668';
        confirmButton.className = 'button is-danger';
        confirmButton.innerHTML = '<span class="icon"><i class="fas fa-trash"></i></span><span>確定刪除</span>';
        warningMessage.style.display = 'block';
    }
    
    // Set the message
    messageElement.innerHTML = message;
    
    // Store callback
    deleteConfirmationCallback = callback;
    
    // Set up confirm button
    confirmButton.onclick = function() {
        if (deleteConfirmationCallback) {
            deleteConfirmationCallback();
        }
        closeDeleteConfirmation();
    };
    
    // Show modal
    modal.classList.add('is-active');
}

function closeDeleteConfirmation() {
    const modal = document.getElementById('delete-confirmation-modal');
    
    // Remove modal immediately without fade-out animation
    modal.classList.remove('is-active');
    deleteConfirmationCallback = null;
}

/**
 * Show alert modal with custom styling
 */
function showAlert(message, type = 'info') {
    const modal = document.getElementById('alert-modal');
    const messageElement = document.getElementById('alert-modal-message');
    const titleElement = document.getElementById('alert-modal-title');
    const iconElement = document.getElementById('alert-modal-icon');
    const headerElement = document.getElementById('alert-modal-header');
    
    // Set message
    messageElement.innerHTML = message;
    
    // Set style based on type
    switch(type) {
        case 'error':
            titleElement.textContent = '錯誤';
            iconElement.innerHTML = '<i class="fas fa-exclamation-circle"></i>';
            headerElement.style.background = '#f14668';
            headerElement.style.borderBottomColor = '#f14668';
            break;
        case 'warning':
            titleElement.textContent = '警告';
            iconElement.innerHTML = '<i class="fas fa-exclamation-triangle"></i>';
            headerElement.style.background = '#ff9f43';
            headerElement.style.borderBottomColor = '#ff9f43';
            break;
        case 'success':
            titleElement.textContent = '成功';
            iconElement.innerHTML = '<i class="fas fa-check-circle"></i>';
            headerElement.style.background = '#48c774';
            headerElement.style.borderBottomColor = '#48c774';
            break;
        default:
            titleElement.textContent = '提示';
            iconElement.innerHTML = '<i class="fas fa-info-circle"></i>';
            headerElement.style.background = '#3273dc';
            headerElement.style.borderBottomColor = '#3273dc';
    }
    
    // Show modal
    modal.classList.add('is-active');
}

/**
 * Close alert modal
 */
function closeAlertModal() {
    const modal = document.getElementById('alert-modal');
    
    // Remove modal immediately without fade-out animation
    modal.classList.remove('is-active');
}

/**
 * Delete a section
 */
function deleteSection(sectionIndex) {
    const currentFlow = flowStore.getState();
    const state = flowStore.getState();
    const section = state.workflow[sectionIndex];
    const sectionName = section.section || '未命名區段';
    const stepCount = section.steps ? section.steps.length : 0;
    
    let message = `<strong>確定要刪除區段「${sectionName}」嗎？</strong>`;
    if (stepCount > 0) {
        message += `<br><br>此區段包含 <span class="tag is-warning">${stepCount} 個步驟</span>，刪除後這些步驟也會一併移除。`;
    }
    
    showDeleteConfirmation(message, function() {
        flowStore.deleteSection(sectionIndex);
        showNotification('✅ 區段已刪除', 'success');
    });
}

/**
 * Confirm delete step with better UI
 */
function confirmDeleteStep(sectionIndex, stepIndex) {
    const currentFlow = flowStore.getState();
    const step = currentFlow.workflow[sectionIndex].steps[stepIndex];
    const stepName = step.id || '未命名步驟';
    const functionName = step.exec || '無';
    
    let message = `<strong>確定要刪除步驟「${stepName}」嗎？</strong>`;
    message += `<br><br>執行函數：<code>${functionName}</code>`;
    if (step.store_to) {
        message += `<br>儲存變數：<code>${step.store_to}</code>`;
    }
    
    showDeleteConfirmation(message, function() {
        deleteStep(sectionIndex, stepIndex);
    });
}

/**
 * Delete a step
 */
function deleteStep(sectionIndex, stepIndex) {
    const currentFlow = flowStore.getState();
    // Delete step from store
    flowStore.deleteStep(sectionIndex, stepIndex);
    
    // Clear editor if the deleted step was selected
    if (currentSectionIndex === sectionIndex && currentStepIndex === stepIndex) {
        document.getElementById('section-editor').innerHTML = `
            <div class="notification is-info">
                <p>選擇左側的區段或步驟進行編輯</p>
            </div>
        `;
        currentStepIndex = null;
    }
    
    showNotification(`✅ 步驟已刪除`, 'success');
}

/**
 * Update YAML editor content
 */
function updateYamlEditor() {
    if (yamlEditor) {
        // Update store from form
        flowStore.updateFlow({
            id: document.getElementById('flow-id').value,
            name: document.getElementById('flow-name').value,
            work_id: document.getElementById('work-id').value,
            priority: parseInt(document.getElementById('flow-priority').value) || 100,
            enabled: document.getElementById('flow-enabled').checked
        });
        
        flowStore.updateMeta({
            description: document.getElementById('flow-description').value || ''
        });
        
        // Get YAML from store
        const yamlContent = flowStore.toYAML();
        yamlEditor.setValue(yamlContent);
    }
}

/**
 * Update variables display
 */
function updateVariablesDisplay() {
    const container = document.getElementById('variables-list');
    if (!container) return;
    
    container.innerHTML = '';
    
    // Get state and computed variables from store
    const fullState = flowStore.getFullState();
    const variables = fullState.computed.variables;
    
    console.log('updateVariablesDisplay: Variables from store:', variables);
    
    // Check if we have variables
    if (!variables || variables.size === 0) {
        container.innerHTML = '<p class="has-text-grey-light is-size-7">無變數</p>';
        return;
    }
    
    // Categorize variables based on flowStore's computed variables
    const systemVars = new Set();
    const stepVars = new Map(); // Map of variable name to step info
    const queryVars = new Set();
    const checkVars = new Set();
    const taskVars = new Set();
    const foreachVars = new Set(); // Variables from foreach loops
    const referenceVars = new Set(); // Variables referenced but not yet defined
    
    // Process variables from flowStore
    variables.forEach((varInfo, varName) => {
        if (varInfo.type === 'system') {
            systemVars.add(varName);
        } else if (varInfo.type === 'foreach_var') {
            // Foreach iteration variable
            foreachVars.add(varName);
            stepVars.set(varName, {
                section: varInfo.source,
                stepId: 'foreach',
                exec: 'foreach 迭代變數'
            });
        } else if (varInfo.type === 'foreach_result') {
            // Foreach result variable
            stepVars.set(varName, {
                section: varInfo.source,
                stepId: 'foreach',
                exec: 'foreach 結果'
            });
        } else if (varInfo.type === 'step') {
            // Regular step variable
            const [sectionPart, ...stepParts] = varInfo.source.split(' > ');
            stepVars.set(varName, {
                section: sectionPart,
                stepId: stepParts.join(' > '),
                exec: varInfo.exec
            });
            
            // Categorize based on exec function
            if (varInfo.exec) {
                if (varInfo.exec.includes('query') || varInfo.exec.includes('get') || varInfo.exec.includes('find')) {
                    queryVars.add(varName);
                } else if (varInfo.exec.includes('check') || varInfo.exec.includes('validate') || varInfo.exec.includes('verify')) {
                    checkVars.add(varName);
                } else if (varInfo.exec.includes('task') || varInfo.exec.includes('create') || varInfo.exec.includes('assign')) {
                    taskVars.add(varName);
                }
            }
        } else if (varInfo.type === 'reference' && varInfo.warning) {
            // Only add to referenceVars if it has a warning (truly undefined)
            // But don't add it if it's actually defined elsewhere
            if (!stepVars.has(varName) && !systemVars.has(varName) && !foreachVars.has(varName)) {
                referenceVars.add(varName);
            }
        }
    });
    
    console.log('updateVariablesDisplay: System vars:', Array.from(systemVars));
    console.log('updateVariablesDisplay: Step vars:', Array.from(stepVars.keys()));
    console.log('updateVariablesDisplay: Reference vars:', Array.from(referenceVars));
    
    // Build categorized display
    let html = '';
    
    // System Variables
    if (systemVars.size > 0) {
        html += '<div class="function-category-header"><i class="fas fa-cog mr-2"></i>系統變數</div>';
        html += '<div class="variables-category">';
        Array.from(systemVars).sort().forEach(varName => {
            html += `
                <div class="variable-item">
                    <div class="variable-name">\${${varName}}</div>
                    <div class="variable-type">系統提供</div>
                </div>
            `;
        });
        html += '</div>';
    }
    
    // Query Variables
    if (queryVars.size > 0) {
        html += '<div class="function-category-header"><i class="fas fa-search mr-2"></i>查詢結果變數</div>';
        html += '<div class="variables-category">';
        Array.from(queryVars).sort().forEach(varName => {
            const info = stepVars.get(varName);
            html += `
                <div class="variable-item">
                    <div class="variable-name">\${${varName}}</div>
                    <div class="variable-type" title="${info.section} → ${info.stepId}">${info.exec}</div>
                </div>
            `;
        });
        html += '</div>';
    }
    
    // Check Variables
    if (checkVars.size > 0) {
        html += '<div class="function-category-header"><i class="fas fa-check-circle mr-2"></i>檢查結果變數</div>';
        html += '<div class="variables-category">';
        Array.from(checkVars).sort().forEach(varName => {
            const info = stepVars.get(varName);
            html += `
                <div class="variable-item">
                    <div class="variable-name">\${${varName}}</div>
                    <div class="variable-type" title="${info.section} → ${info.stepId}">${info.exec}</div>
                </div>
            `;
        });
        html += '</div>';
    }
    
    // Task Variables
    if (taskVars.size > 0) {
        html += '<div class="function-category-header"><i class="fas fa-tasks mr-2"></i>任務變數</div>';
        html += '<div class="variables-category">';
        Array.from(taskVars).sort().forEach(varName => {
            const info = stepVars.get(varName);
            html += `
                <div class="variable-item">
                    <div class="variable-name">\${${varName}}</div>
                    <div class="variable-type" title="${info.section} → ${info.stepId}">${info.exec}</div>
                </div>
            `;
        });
        html += '</div>';
    }
    
    // ForEach Variables
    if (foreachVars.size > 0) {
        html += '<div class="function-category-header"><i class="fas fa-repeat mr-2"></i>ForEach 變數</div>';
        html += '<div class="variables-category">';
        Array.from(foreachVars).sort().forEach(varName => {
            const info = stepVars.get(varName);
            html += `
                <div class="variable-item">
                    <div class="variable-name">\${${varName}}</div>
                    <div class="variable-type" title="${info.section}">${info.exec}</div>
                </div>
            `;
        });
        html += '</div>';
    }
    
    // Other Step Variables (not categorized)
    const otherStepVars = Array.from(stepVars.keys()).filter(v => 
        !queryVars.has(v) && !checkVars.has(v) && !taskVars.has(v) && !foreachVars.has(v)
    );
    if (otherStepVars.length > 0) {
        html += '<div class="function-category-header"><i class="fas fa-database mr-2"></i>其他步驟變數</div>';
        html += '<div class="variables-category">';
        otherStepVars.sort().forEach(varName => {
            const info = stepVars.get(varName);
            html += `
                <div class="variable-item">
                    <div class="variable-name">\${${varName}}</div>
                    <div class="variable-type" title="${info.section} → ${info.stepId}">${info.exec || '儲存值'}</div>
                </div>
            `;
        });
        html += '</div>';
    }
    
    // Referenced but undefined variables (warnings)
    if (referenceVars.size > 0) {
        // Create a map to track where each undefined variable is referenced
        const referenceLocations = new Map();
        
        // Find reference locations
        const currentFlow = flowStore.getState();
        if (currentFlow.workflow) {
            currentFlow.workflow.forEach(section => {
                if (section.steps) {
                    section.steps.forEach(step => {
                        // Check in parameters
                        if (step.params && typeof step.params === 'object') {
                            const paramString = JSON.stringify(step.params);
                            const variableMatches = paramString.match(/\$\{([^}]+)\}/g);
                            if (variableMatches) {
                                variableMatches.forEach(match => {
                                    const varName = match.slice(2, -1);
                                    if (referenceVars.has(varName)) {
                                        if (!referenceLocations.has(varName)) {
                                            referenceLocations.set(varName, []);
                                        }
                                        referenceLocations.get(varName).push({
                                            step: step.id || 'Unnamed Step',
                                            type: 'params'
                                        });
                                    }
                                });
                            }
                        }
                        
                        // Check in skip conditions
                        if (step.skip_if) {
                            const conditionMatches = step.skip_if.match(/\$\{([^}]+)\}/g);
                            if (conditionMatches) {
                                conditionMatches.forEach(match => {
                                    const varName = match.slice(2, -1);
                                    if (referenceVars.has(varName)) {
                                        if (!referenceLocations.has(varName)) {
                                            referenceLocations.set(varName, []);
                                        }
                                        referenceLocations.get(varName).push({
                                            step: step.id || 'Unnamed Step',
                                            type: 'skip_if'
                                        });
                                    }
                                });
                            }
                        }
                    });
                }
            });
        }
        
        html += '<div class="function-category-header" style="color: #e67e22;"><i class="fas fa-exclamation-triangle mr-2"></i>引用但未定義</div>';
        html += '<div class="variables-category">';
        Array.from(referenceVars).sort().forEach(varName => {
            const locations = referenceLocations.get(varName) || [];
            const locationText = locations.map(loc => 
                loc.type === 'params' ? `${loc.step} (參數)` : `${loc.step} (條件)`
            ).join(', ');
            
            html += `
                <div class="variable-item" style="border-color: #e67e22;">
                    <div class="variable-name" style="color: #e67e22;">\${${varName}}</div>
                    <div class="variable-type" style="color: #e67e22;" title="引用位置: ${locationText}">引用於: ${locationText}</div>
                </div>
            `;
        });
        html += '</div>';
    }
    
    // Display result
    if (html === '') {
        container.innerHTML = '<p class="has-text-grey-light is-size-7">無可用變數</p>';
    } else {
        container.innerHTML = html;
        
        // Add click handlers to variable items
        container.querySelectorAll('.variable-item').forEach(item => {
            item.addEventListener('click', function() {
                const varNameElement = this.querySelector('.variable-name');
                if (varNameElement) {
                    const varName = varNameElement.textContent.replace('${', '').replace('}', '');
                    highlightVariableRelatedSteps(varName);
                }
            });
        });
    }
}

/**
 * Highlight steps related to a variable
 */
function highlightVariableRelatedSteps(varName) {
    // Find all steps that either store or use this variable
    const relatedSteps = [];
    
    // Helper function to check if a step uses the variable
    function checkStepForVariable(step, varName, sectionIndex, stepIndex, parentIndices = []) {
        let isRelated = false;
        
        // Check if step stores this variable
        if (step.store === varName) {
            isRelated = true;
        }
        
        // Check if this is a foreach that defines this variable
        if ((step.exec === 'foreach' || step.exec === 'control.foreach') && 
            (step.params?.var === varName || step.var === varName)) {
            isRelated = true;
        }
        
        // Check if step uses this variable in parameters
        if (step.params && typeof step.params === 'object') {
            const paramString = JSON.stringify(step.params);
            if (paramString.includes('${' + varName + '}')) {
                isRelated = true;
            }
        }
        
        // Check if step uses this variable in skip conditions
        if (step.skip_if && step.skip_if.includes('${' + varName + '}')) {
            isRelated = true;
        }
        
        if (step.skip_if_not && step.skip_if_not.includes('${' + varName + '}')) {
            isRelated = true;
        }
        
        if (isRelated) {
            relatedSteps.push({
                sectionIndex: sectionIndex,
                stepIndex: stepIndex,
                parentIndices: parentIndices
            });
        }
        
        // Recursively check foreach inner steps
        if (step.exec === 'foreach' || step.exec === 'control.foreach') {
            const innerSteps = step.params?.steps || step.steps;
            if (innerSteps && Array.isArray(innerSteps)) {
                innerSteps.forEach((innerStep, innerIndex) => {
                    checkStepForVariable(innerStep, varName, sectionIndex, stepIndex, 
                        [...parentIndices, { type: 'foreach', index: innerIndex }]);
                });
            }
        }
    }
    
    const currentFlow = flowStore.getState();
    if (currentFlow.workflow) {
        currentFlow.workflow.forEach((section, sectionIndex) => {
            if (section.steps) {
                section.steps.forEach((step, stepIndex) => {
                    checkStepForVariable(step, varName, sectionIndex, stepIndex);
                });
            }
        });
    }
    
    // Highlight the visual steps
    relatedSteps.forEach(location => {
        const visualStep = document.querySelector(
            `.visual-step[data-section-index="${location.sectionIndex}"][data-step-index="${location.stepIndex}"]`
        );
        
        if (visualStep) {
            // Remove any existing animation class
            visualStep.classList.remove('variable-highlight');
            
            // Force reflow to restart animation
            void visualStep.offsetWidth;
            
            // Add animation class
            visualStep.classList.add('variable-highlight');
            
            // Remove class after animation completes
            setTimeout(() => {
                visualStep.classList.remove('variable-highlight');
            }, 1000);
            
            // Scroll the first related step into view
            if (relatedSteps[0] === location) {
                visualStep.scrollIntoView({
                    behavior: 'smooth',
                    block: 'center'
                });
            }
        }
    });
    
    // Also highlight in the left panel workflow
    relatedSteps.forEach(location => {
        const workflowStep = document.querySelector(
            `.workflow-step[data-section-index="${location.sectionIndex}"][data-step-index="${location.stepIndex}"]`
        );
        
        if (workflowStep) {
            // Add a temporary highlight class
            workflowStep.style.transition = 'background-color 0.5s';
            const originalBg = workflowStep.style.backgroundColor;
            workflowStep.style.backgroundColor = '#f3e5f5';
            
            setTimeout(() => {
                workflowStep.style.backgroundColor = originalBg || '';
            }, 1000);
        }
    });
}

/**
 * Highlight step in visual editor when clicking on workflow step
 */
function highlightStepInVisualEditor(sectionIndex, stepIndex) {
    // Find the corresponding step in the visual editor
    const visualStep = document.querySelector(
        `.visual-step[data-section-index="${sectionIndex}"][data-step-index="${stepIndex}"]`
    );
    
    if (visualStep) {
        // Remove any existing highlight class from all steps
        document.querySelectorAll('.visual-step.step-highlight').forEach(el => {
            el.classList.remove('step-highlight');
        });
        
        // Remove the class first to restart animation
        visualStep.classList.remove('step-highlight');
        
        // Force reflow to restart animation
        void visualStep.offsetWidth;
        
        // Add highlight animation class
        visualStep.classList.add('step-highlight');
        
        // Remove class after animation completes (1 second for 2 flashes)
        setTimeout(() => {
            visualStep.classList.remove('step-highlight');
        }, 1000);
        
        // Scroll the step into view
        visualStep.scrollIntoView({
            behavior: 'smooth',
            block: 'center'
        });
    }
}

/**
 * Switch function category
 */
function switchFunctionCategory(category) {
    // Update tab active state
    document.querySelectorAll('#function-category-tabs a').forEach(tab => {
        tab.classList.remove('is-active');
        if (tab.getAttribute('data-category') === category) {
            tab.classList.add('is-active');
        }
    });
    
    // Re-render function library with filter
    renderFunctionLibrary(category);
}

/**
 * Switch tabs - Using visibility to prevent reflow/jitter
 */
function switchTab(tabName) {
    console.log(`🔄 Switching to tab: ${tabName}`);
    
    // Check if we're leaving the YAML tab and need to sync changes
    const activeTabElement = document.querySelector('.editor-tabs a.is-active');
    const previousTab = activeTabElement ? activeTabElement.getAttribute('data-tab') : 'visual';
    
    if (previousTab === 'yaml' && tabName !== 'yaml' && yamlEditor) {
        // Parse YAML content and update store before switching away
        const yamlContent = yamlEditor.getValue();
        if (yamlContent.trim()) {
            const result = flowStore.updateFromYAML(yamlContent, 'yaml');
            if (!result.success) {
                showAlert('YAML 格式錯誤，無法同步變更: ' + result.error, 'error');
                // Don't switch tabs if there's an error
                return;
            }
            
            // Update form fields from store
            const state = flowStore.getState();
            updateFormFieldsFromState(state);
            
            // Clear any pending YAML update timer
            if (window.yamlUpdateTimer) {
                clearTimeout(window.yamlUpdateTimer);
                window.yamlUpdateTimer = null;
            }
            
            showNotification('✅ YAML 變更已同步', 'success');
        }
    }
    
    // Update tab buttons - fix for editor-tabs structure
    document.querySelectorAll('.editor-tabs a[data-tab]').forEach(tab => {
        tab.classList.remove('is-active');
    });
    const activeTab = document.querySelector(`.editor-tabs a[data-tab="${tabName}"]`);
    if (activeTab) {
        activeTab.classList.add('is-active');
    }
    
    // Switch tab contents using classes only (let CSS handle visibility)
    document.querySelectorAll('.tab-content').forEach(content => {
        content.classList.remove('is-active');
    });
    
    // Show the selected tab content
    const selectedTab = document.getElementById(`${tabName}-tab`);
    if (selectedTab) {
        selectedTab.classList.add('is-active');
        console.log(`✅ Tab ${tabName} is now active`);
    }
    
    // Special handling for different tabs
    if (tabName === 'yaml') {
        updateYamlEditor();
        if (yamlEditor) {
            setTimeout(() => yamlEditor.refresh(), 100);
        }
    } else if (tabName === 'preview') {
        renderPreview();
    }
}

/**
 * Render flow preview - Business logic focused visualization
 */
function renderPreview() {
    const container = document.getElementById('flow-preview');
    if (!container) return;
    
    const currentFlow = flowStore.getState();
    
    // Generate business logic focused preview
    let html = `
        <div class="preview-header">
            <h2 class="title is-4">
                <span class="icon has-text-primary">
                    <i class="fas fa-project-diagram"></i>
                </span>
                ${currentFlow.flow.name || '未命名流程'}
            </h2>
            <div class="tags">
                <span class="tag is-info">
                    <span class="icon is-small">
                        <i class="fas fa-fingerprint"></i>
                    </span>
                    <span>ID: ${currentFlow.flow.id || 'new_flow'}</span>
                </span>
                ${currentFlow.flow.work_id ? `
                    <span class="tag is-primary">
                        <span class="icon is-small">
                            <i class="fas fa-briefcase"></i>
                        </span>
                        <span>Work: ${currentFlow.flow.work_id}</span>
                    </span>
                ` : ''}
                <span class="tag ${currentFlow.flow.enabled ? 'is-success' : 'is-danger'}">
                    <span class="icon is-small">
                        <i class="fas ${currentFlow.flow.enabled ? 'fa-check-circle' : 'fa-times-circle'}"></i>
                    </span>
                    <span>${currentFlow.flow.enabled ? '啟用' : '停用'}</span>
                </span>
                <span class="tag is-warning">
                    <span class="icon is-small">
                        <i class="fas fa-sort-amount-up"></i>
                    </span>
                    <span>優先級: ${currentFlow.flow.priority || 100}</span>
                </span>
            </div>
            ${currentFlow.meta.description ? `
                <p class="has-text-grey-dark mt-2">
                    <em>${currentFlow.meta.description}</em>
                </p>
            ` : ''}
        </div>
        <hr>
    `;
    
    // Render workflow sections with business logic focus
    if (currentFlow.workflow && currentFlow.workflow.length > 0) {
        html += '<div class="workflow-overview">';
        
        currentFlow.workflow.forEach((section, sectionIndex) => {
            const sectionId = `preview-section-${sectionIndex}`;
            const hasCondition = section.condition ? true : false;
            const stepCount = section.steps ? section.steps.length : 0;
            
            html += `
                <div class="preview-section" id="${sectionId}">
                    <div class="preview-section-header">
                        <h3 class="title is-5">
                            <span class="icon has-text-info">
                                <i class="fas fa-layer-group"></i>
                            </span>
                            <span>${sectionIndex + 1}. ${section.section || '未命名區段'}</span>
                            <span class="tag is-light is-small ml-2">${stepCount} 步驟</span>
                        </h3>
                        ${hasCondition ? `
                            <div class="notification is-info is-light">
                                <span class="icon">
                                    <i class="fas fa-filter"></i>
                                </span>
                                <strong>執行條件：</strong>
                                <code>${formatCondition(section.condition)}</code>
                            </div>
                        ` : ''}
                        ${section.description ? `
                            <p class="has-text-grey mb-3">${section.description}</p>
                        ` : ''}
                    </div>
                    
                    <div class="preview-steps">
            `;
            
            // Render steps with business logic interpretation
            if (section.steps && section.steps.length > 0) {
                section.steps.forEach((step, stepIndex) => {
                    html += renderBusinessLogicStep(step, stepIndex, sectionIndex);
                });
            } else {
                html += `
                    <div class="notification is-warning is-light">
                        <span class="icon">
                            <i class="fas fa-exclamation-triangle"></i>
                        </span>
                        此區段尚無步驟
                    </div>
                `;
            }
            
            html += `
                    </div>
                </div>
            `;
        });
        
        html += '</div>';
    } else {
        html += `
            <div class="notification is-info is-light">
                <span class="icon">
                    <i class="fas fa-info-circle"></i>
                </span>
                <strong>尚未定義工作流程</strong>
                <p>請在左側新增區段和步驟來建立流程</p>
            </div>
        `;
    }
    
    // Add execution flow summary
    html += renderExecutionFlowSummary(currentFlow);
    
    container.innerHTML = html;
}

/**
 * Render a single step with business logic interpretation
 */
function renderBusinessLogicStep(step, stepIndex, sectionIndex) {
    const functionInfo = getFunctionInfo(step.exec);
    const stepNumber = stepIndex + 1;
    
    // Determine step type and icon
    let stepIcon = 'fa-cog';
    let stepColor = 'is-info';
    
    if (step.exec) {
        if (step.exec.startsWith('query.')) {
            stepIcon = 'fa-database';
            stepColor = 'is-primary';
        } else if (step.exec.startsWith('check.')) {
            stepIcon = 'fa-check-square';
            stepColor = 'is-warning';
        } else if (step.exec.startsWith('task.')) {
            stepIcon = 'fa-tasks';
            stepColor = 'is-success';
        } else if (step.exec.startsWith('action.')) {
            stepIcon = 'fa-bolt';
            stepColor = 'is-danger';
        } else if (step.exec.startsWith('control.')) {
            stepIcon = 'fa-sliders-h';
            stepColor = 'is-link';
        } else if (step.exec === 'foreach') {
            stepIcon = 'fa-repeat';
            stepColor = 'is-purple';
        }
    }
    
    // Determine step class for border color
    let stepClass = '';
    if (step.exec) {
        if (step.exec.startsWith('query.')) stepClass = 'is-query';
        else if (step.exec.startsWith('check.')) stepClass = 'is-check';
        else if (step.exec.startsWith('task.')) stepClass = 'is-task';
        else if (step.exec.startsWith('action.')) stepClass = 'is-action';
        else if (step.exec.startsWith('control.')) stepClass = 'is-control';
        else if (step.exec === 'foreach') stepClass = 'is-foreach';
    }
    
    let html = `
        <div class="preview-step ${stepClass}">
            <div class="preview-step-header">
                <div class="level">
                    <div class="level-left">
                        <div class="level-item">
                            <span class="icon ${stepColor}">
                                <i class="fas ${stepIcon}"></i>
                            </span>
                        </div>
                        <div class="level-item">
                            <div>
                                <p class="preview-step-id">
                                    <strong>步驟 ${stepNumber}:</strong> ${step.id || '未命名步驟'}
                                </p>
                                <p class="preview-step-exec">
                                    <span class="tag ${stepColor} is-light">
                                        ${functionInfo ? functionInfo.description : step.exec}
                                    </span>
                                </p>
                            </div>
                        </div>
                    </div>
                </div>
            </div>
    `;
    
    // Add business logic description based on function and parameters
    const businessDesc = getBusinessLogicDescription(step);
    if (businessDesc) {
        html += `
            <div class="preview-step-business-logic">
                <p class="has-text-dark">
                    <span class="icon is-small has-text-grey">
                        <i class="fas fa-angle-right"></i>
                    </span>
                    ${businessDesc}
                </p>
            </div>
        `;
    }
    
    // Show conditions in a user-friendly way
    if (step.skip_if) {
        html += `
            <div class="preview-step-condition">
                <span class="tag is-warning is-light">
                    <span class="icon is-small">
                        <i class="fas fa-code-branch"></i>
                    </span>
                    <span>跳過條件: ${formatConditionText(step.skip_if)}</span>
                </span>
            </div>
        `;
    }
    
    // Show result storage
    if (step.store) {
        html += `
            <div class="preview-step-store">
                <span class="tag is-success is-light">
                    <span class="icon is-small">
                        <i class="fas fa-save"></i>
                    </span>
                    <span>儲存結果至: <code>${step.store}</code></span>
                </span>
            </div>
        `;
    }
    
    // For foreach loops, show nested steps
    if (step.exec === 'foreach' && step.steps) {
        html += `
            <div class="preview-foreach-body">
                <p class="has-text-grey-dark mb-2">
                    <span class="icon is-small">
                        <i class="fas fa-level-down-alt"></i>
                    </span>
                    迴圈內容 (對每個 <code>${step.var || 'item'}</code> 執行):
                </p>
                <div class="ml-4 pl-3" style="border-left: 2px solid #dbdbdb;">
        `;
        
        step.steps.forEach((nestedStep, nestedIndex) => {
            html += renderBusinessLogicStep(nestedStep, nestedIndex, `${sectionIndex}-foreach`);
        });
        
        html += `
                </div>
            </div>
        `;
    }
    
    html += '</div>';
    return html;
}

/**
 * Get business logic description for a step
 */
function getBusinessLogicDescription(step) {
    const params = step.params || {};
    let desc = '';
    
    switch(step.exec) {
        case 'query.locations':
            desc = `查詢${params.type || '所有'}類型的位置`;
            if (params.rooms) desc += `，範圍限制在房間 ${Array.isArray(params.rooms) ? params.rooms.join(', ') : params.rooms}`;
            if (params.has_rack !== undefined) desc += params.has_rack ? '，必須有架台' : '，不需有架台';
            break;
            
        case 'query.racks':
            desc = `查詢架台資料`;
            if (params.location_id) desc += `，位置 ID: ${params.location_id}`;
            if (params.status) desc += `，狀態: ${params.status}`;
            break;
            
        case 'query.tasks':
            desc = `查詢任務`;
            if (params.status) desc += `，狀態: ${params.status}`;
            if (params.type) desc += `，類型: ${params.type}`;
            if (params.limit) desc += `，限制 ${params.limit} 筆`;
            break;
            
        case 'check.empty':
            desc = `檢查 ${params.data || '資料'} 是否為空`;
            break;
            
        case 'check.rack_status':
            desc = `檢查架台 ${params.rack_id || '未指定'} 的${params.side || ''}面狀態`;
            break;
            
        case 'task.create_task':
            desc = `建立 ${params.type || '未指定類型'} 任務`;
            if (params.work_id) desc += `，工作 ID: ${params.work_id}`;
            break;
            
        case 'action.log_message':
            desc = `記錄日誌訊息: "${params.message || '未指定訊息'}"`;
            break;
            
        case 'action.send_notification':
            desc = `發送${params.level || 'info'}級別通知: "${params.message || '未指定訊息'}"`;
            break;
            
        case 'control.wait_time':
            desc = `等待 ${params.seconds || 1} 秒`;
            break;
            
        case 'control.count_items':
            desc = `計算 ${params.variable || '變數'} 中的項目數量`;
            break;
            
        case 'foreach':
            desc = `對 ${step.items || '集合'} 中的每個項目執行迴圈`;
            break;
            
        default:
            // For other functions, try to generate a description from parameters
            if (Object.keys(params).length > 0) {
                const paramStrs = Object.entries(params).map(([key, value]) => {
                    if (typeof value === 'object') {
                        return `${key}: ${JSON.stringify(value)}`;
                    }
                    return `${key}: ${value}`;
                });
                desc = `執行 ${step.exec}，參數: ${paramStrs.join(', ')}`;
            }
    }
    
    return desc;
}

/**
 * Format condition for display (plain text version)
 */
function formatConditionText(condition) {
    if (!condition) return '';
    
    // Replace variable references with more readable format
    let formatted = condition;
    
    // Handle ${var} format
    formatted = formatted.replace(/\$\{([^}]+)\}/g, '【$1】');
    
    // Handle logical operators
    formatted = formatted.replace(/\|\|/g, ' 或 ');
    formatted = formatted.replace(/&&/g, ' 且 ');
    formatted = formatted.replace(/!/g, '非 ');
    
    // Handle comparisons
    formatted = formatted.replace(/==/g, ' 等於 ');
    formatted = formatted.replace(/!=/g, ' 不等於 ');
    formatted = formatted.replace(/>=/g, ' 大於等於 ');
    formatted = formatted.replace(/<=/g, ' 小於等於 ');
    formatted = formatted.replace(/>/g, ' 大於 ');
    formatted = formatted.replace(/</g, ' 小於 ');
    
    return formatted;
}

/**
 * Format condition for display (HTML version - currently not used)
 */
function formatCondition(condition) {
    if (!condition) return '';
    
    // Replace variable references with more readable format
    let formatted = condition;
    
    // Handle ${var} format
    formatted = formatted.replace(/\$\{([^}]+)\}/g, '<strong>$1</strong>');
    
    // Handle logical operators
    formatted = formatted.replace(/\|\|/g, ' <span class="has-text-info">或</span> ');
    formatted = formatted.replace(/&&/g, ' <span class="has-text-info">且</span> ');
    formatted = formatted.replace(/!/g, '<span class="has-text-danger">非</span>');
    
    // Handle comparisons
    formatted = formatted.replace(/==/g, ' <span class="has-text-primary">等於</span> ');
    formatted = formatted.replace(/!=/g, ' <span class="has-text-primary">不等於</span> ');
    formatted = formatted.replace(/>=/g, ' <span class="has-text-primary">大於等於</span> ');
    formatted = formatted.replace(/<=/g, ' <span class="has-text-primary">小於等於</span> ');
    formatted = formatted.replace(/>/g, ' <span class="has-text-primary">大於</span> ');
    formatted = formatted.replace(/</g, ' <span class="has-text-primary">小於</span> ');
    
    return formatted;
}

/**
 * Render execution flow summary
 */
function renderExecutionFlowSummary(flow) {
    let totalSteps = 0;
    let queryCount = 0;
    let checkCount = 0;
    let taskCount = 0;
    let actionCount = 0;
    let controlCount = 0;
    
    // Count step types
    if (flow.workflow) {
        flow.workflow.forEach(section => {
            if (section.steps) {
                section.steps.forEach(step => {
                    totalSteps++;
                    if (step.exec) {
                        if (step.exec.startsWith('query.')) queryCount++;
                        else if (step.exec.startsWith('check.')) checkCount++;
                        else if (step.exec.startsWith('task.')) taskCount++;
                        else if (step.exec.startsWith('action.')) actionCount++;
                        else if (step.exec.startsWith('control.')) controlCount++;
                    }
                });
            }
        });
    }
    
    return `
        <div class="preview-summary mt-5">
            <h3 class="title is-5">
                <span class="icon has-text-info">
                    <i class="fas fa-chart-pie"></i>
                </span>
                執行流程摘要
            </h3>
            <div class="columns is-multiline">
                <div class="column is-4">
                    <div class="box has-background-light">
                        <p class="heading">總步驟數</p>
                        <p class="title is-4">${totalSteps}</p>
                    </div>
                </div>
                ${queryCount > 0 ? `
                    <div class="column is-4">
                        <div class="box has-background-primary-light">
                            <p class="heading">查詢操作</p>
                            <p class="title is-4">${queryCount}</p>
                        </div>
                    </div>
                ` : ''}
                ${checkCount > 0 ? `
                    <div class="column is-4">
                        <div class="box has-background-warning-light">
                            <p class="heading">檢查操作</p>
                            <p class="title is-4">${checkCount}</p>
                        </div>
                    </div>
                ` : ''}
                ${taskCount > 0 ? `
                    <div class="column is-4">
                        <div class="box has-background-success-light">
                            <p class="heading">任務操作</p>
                            <p class="title is-4">${taskCount}</p>
                        </div>
                    </div>
                ` : ''}
                ${actionCount > 0 ? `
                    <div class="column is-4">
                        <div class="box has-background-danger-light">
                            <p class="heading">動作操作</p>
                            <p class="title is-4">${actionCount}</p>
                        </div>
                    </div>
                ` : ''}
                ${controlCount > 0 ? `
                    <div class="column is-4">
                        <div class="box has-background-link-light">
                            <p class="heading">控制操作</p>
                            <p class="title is-4">${controlCount}</p>
                        </div>
                    </div>
                ` : ''}
            </div>
        </div>
    `;
}

/**
 * Get function info from available functions
 */
function getFunctionInfo(funcName) {
    if (!funcName || !availableFunctions) return null;
    
    // Search in all categories
    for (const category in availableFunctions) {
        const functions = availableFunctions[category];
        if (Array.isArray(functions)) {
            const func = functions.find(f => f.name === funcName);
            if (func) return func;
        }
    }
    
    return null;
}

/**
 * Save flow
 */
async function saveFlow() {
    const currentFlow = flowStore.getState();
    // Update flow data from form
    currentFlow.flow.id = document.getElementById('flow-id').value;
    currentFlow.flow.name = document.getElementById('flow-name').value;
    currentFlow.meta.description = document.getElementById('flow-description').value || '';
    currentFlow.flow.work_id = document.getElementById('work-id').value;
    currentFlow.flow.priority = parseInt(document.getElementById('flow-priority').value) || 100;
    currentFlow.flow.enabled = document.getElementById('flow-enabled').checked;
    
    // If in YAML tab, parse YAML content
    const activeTabElement = document.querySelector('.editor-tabs a.is-active');
    const activeTab = activeTabElement ? activeTabElement.getAttribute('data-tab') : 'visual';
    if (activeTab === 'yaml' && yamlEditor) {
        try {
            currentFlow = jsyaml.load(yamlEditor.getValue());
            
            // Ensure workflow field exists (Linear Flow v2 uses 'workflow')
            if (!currentFlow.workflow) {
                currentFlow.workflow = [];
            }
        } catch (e) {
            showAlert('YAML 格式錯誤: ' + e.message, 'error');
            return;
        }
    }
    
    try {
        const response = await fetch('/linear-flow/api/flows/save', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            },
            body: JSON.stringify({
                flow_id: currentFlow.flow.id,
                flow_data: currentFlow
            })
        });
        
        const result = await response.json();
        
        if (result.success) {
            showNotification('流程已成功儲存', 'success');
            if (result.warnings && result.warnings.length > 0) {
                result.warnings.forEach(warning => {
                    showNotification('警告: ' + warning, 'warning');
                });
            }
        } else {
            if (result.errors) {
                result.errors.forEach(error => {
                    showNotification('錯誤: ' + error, 'danger');
                });
            } else {
                showNotification('儲存失敗: ' + (result.error || '未知錯誤'), 'danger');
            }
        }
    } catch (error) {
        showNotification('儲存失敗: ' + error.message, 'danger');
    }
}

/**
 * Validate flow
 */
async function validateFlow() {
    const currentFlow = flowStore.getState();
    // Update flow data
    currentFlow.flow.id = document.getElementById('flow-id').value;
    currentFlow.flow.name = document.getElementById('flow-name').value;
    currentFlow.meta.description = document.getElementById('flow-description').value || '';
    currentFlow.flow.work_id = document.getElementById('work-id').value;
    currentFlow.flow.priority = parseInt(document.getElementById('flow-priority').value) || 100;
    currentFlow.flow.enabled = document.getElementById('flow-enabled').checked;
    
    try {
        const response = await fetch('/linear-flow/api/flows/validate', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            },
            body: JSON.stringify({
                flow_data: currentFlow
            })
        });
        
        const result = await response.json();
        
        if (result.valid) {
            showNotification('✅ 流程驗證通過 - 結構正確，可以儲存', 'success');
        } else {
            showNotification('❌ 流程驗證失敗 - 請修正以下問題：', 'danger');
            if (result.errors) {
                result.errors.forEach(error => {
                    showNotification('錯誤: ' + error, 'danger');
                });
            }
        }
        
        if (result.warnings && result.warnings.length > 0) {
            showNotification('⚠️ 發現以下警告（不影響儲存）：', 'warning');
            result.warnings.forEach(warning => {
                showNotification('警告: ' + warning, 'warning');
            });
        }
    } catch (error) {
        showNotification('驗證失敗: ' + error.message, 'danger');
    }
}

/**
 * Build flow data from current editor state
 */
function buildFlowData() {
    const currentFlow = flowStore.getState();
    // Get metadata from form
    const flowData = {
        flow_id: document.getElementById('flow-id').value || 'unnamed_flow',
        flow_name: document.getElementById('flow-name').value || '',
        description: document.getElementById('flow-description').value || '',
        work_id: document.getElementById('work-id').value || '',
        priority: parseInt(document.getElementById('flow-priority').value) || 100,
        enabled: document.getElementById('flow-enabled').checked,
        workflow: currentFlow ? currentFlow.workflow : []
    };
    
    return flowData;
}

/**
 * Display detailed execution logs with expandable details
 */
function displayDetailedLogs(logs, logDiv) {
    logs.forEach((log, index) => {
        const logClass = log.level === 'error' ? 'log-error' : 
                        log.level === 'warning' ? 'log-warning' : 
                        log.level === 'success' ? 'log-success' : 'log-info';
        
        // Create log entry container
        const logEntry = document.createElement('div');
        logEntry.className = `log-entry ${logClass}`;
        logEntry.style.marginBottom = '5px';
        
        // Main log message
        const messageDiv = document.createElement('div');
        messageDiv.style.display = 'flex';
        messageDiv.style.alignItems = 'center';
        messageDiv.innerHTML = `
            <span style="color: #888; font-size: 0.8em; margin-right: 10px;">${log.timestamp ? log.timestamp.split('T')[1].substring(0, 8) : ''}</span>
            <span style="color: #666; margin-right: 10px;">[${log.step || '-'}]</span>
            <span>${log.message}</span>
        `;
        logEntry.appendChild(messageDiv);
        
        // If there are details, add expandable section
        if (log.details && Object.keys(log.details).length > 0) {
            const detailsId = `log-details-${index}`;
            
            // Add expand/collapse button
            const expandBtn = document.createElement('button');
            expandBtn.className = 'button is-small is-text';
            expandBtn.style.marginLeft = '10px';
            expandBtn.innerHTML = '📋 詳細';
            expandBtn.onclick = () => {
                const detailsDiv = document.getElementById(detailsId);
                if (detailsDiv.style.display === 'none') {
                    detailsDiv.style.display = 'block';
                    expandBtn.innerHTML = '📂 收起';
                } else {
                    detailsDiv.style.display = 'none';
                    expandBtn.innerHTML = '📋 詳細';
                }
            };
            messageDiv.appendChild(expandBtn);
            
            // Add details section
            const detailsDiv = document.createElement('div');
            detailsDiv.id = detailsId;
            detailsDiv.style.display = 'none';
            detailsDiv.style.marginLeft = '40px';
            detailsDiv.style.marginTop = '5px';
            detailsDiv.style.padding = '10px';
            detailsDiv.style.backgroundColor = '#f5f5f5';
            detailsDiv.style.borderLeft = '3px solid #3498db';
            detailsDiv.style.fontSize = '0.9em';
            
            // Format details based on content
            if (log.details.condition_expression) {
                // Condition evaluation details
                detailsDiv.innerHTML = `
                    <div><strong>條件表達式:</strong> <code>${log.details.condition_expression}</code></div>
                    <div><strong>評估結果:</strong> <span class="tag ${log.details.evaluation_result ? 'is-success' : 'is-danger'}">${log.details.evaluation_result ? 'TRUE' : 'FALSE'}</span></div>
                `;
                
                if (log.details.variables_used && Object.keys(log.details.variables_used).length > 0) {
                    detailsDiv.innerHTML += '<div><strong>使用的變數:</strong></div>';
                    const varsTable = document.createElement('table');
                    varsTable.className = 'table is-narrow';
                    varsTable.style.marginTop = '5px';
                    varsTable.innerHTML = '<tbody>';
                    
                    Object.entries(log.details.variables_used).forEach(([key, value]) => {
                        varsTable.innerHTML += `
                            <tr>
                                <td><code>${key}</code></td>
                                <td>=</td>
                                <td><code>${JSON.stringify(value)}</code></td>
                            </tr>
                        `;
                    });
                    
                    varsTable.innerHTML += '</tbody>';
                    detailsDiv.appendChild(varsTable);
                }
            } else if (log.details.function) {
                // Function execution details
                detailsDiv.innerHTML = `
                    <div><strong>函數:</strong> <code>${log.details.function}</code></div>
                    <div><strong>步驟 ID:</strong> ${log.details.step_id}</div>
                `;
                
                if (log.details.input_params) {
                    detailsDiv.innerHTML += `<div><strong>輸入參數:</strong></div>`;
                    detailsDiv.innerHTML += `<pre style="margin-top: 5px;">${JSON.stringify(log.details.input_params, null, 2)}</pre>`;
                }
                
                if (log.details.result_value !== undefined) {
                    detailsDiv.innerHTML += `<div><strong>返回結果:</strong></div>`;
                    detailsDiv.innerHTML += `<pre style="margin-top: 5px;">${JSON.stringify(log.details.result_value, null, 2)}</pre>`;
                }
            } else {
                // Generic details display
                detailsDiv.innerHTML = `<pre>${JSON.stringify(log.details, null, 2)}</pre>`;
            }
            
            logEntry.appendChild(detailsDiv);
        }
        
        logDiv.appendChild(logEntry);
    });
}

/**
 * Test flow execution - opens test modal
 */
function testFlow() {
    // Check if flow is valid first
    const flowData = buildFlowData();
    if (!flowData.workflow || flowData.workflow.length === 0) {
        showNotification('請先設計流程再進行測試', 'warning');
        return;
    }
    
    // Use existing modal from HTML
    const modal = document.getElementById('test-execution-modal');
    if (modal) {
        modal.classList.add('is-active');
        
        // Initialize test state
        initializeTestExecution(flowData);
    } else {
        showNotification('測試模態框未找到', 'error');
    }
}

/**
 * Create test execution modal
 */
function createTestExecutionModal() {
    const modal = document.createElement('div');
    modal.className = 'modal';
    modal.id = 'test-execution-modal';
    modal.innerHTML = `
        <div class="modal-background" onclick="closeTestModal()"></div>
        <div class="modal-card" style="width: 80%; max-width: 900px;">
            <header class="modal-card-head">
                <p class="modal-card-title">
                    <span class="icon"><i class="fas fa-flask"></i></span>
                    <span>流程測試執行</span>
                </p>
                <button class="delete" onclick="closeTestModal()"></button>
            </header>
            <section class="modal-card-body">
                <div class="tabs">
                    <ul>
                        <li class="is-active"><a onclick="switchTestTab('execution')">執行過程</a></li>
                        <li><a onclick="switchTestTab('variables')">變數狀態</a></li>
                        <li><a onclick="switchTestTab('report')">測試報告</a></li>
                    </ul>
                </div>
                
                <div id="test-execution-tab" class="test-tab-content is-active">
                    <div class="test-controls mb-3">
                        <div class="field is-grouped">
                            <div class="control">
                                <button class="button is-small is-primary" onclick="startTest()">
                                    <span class="icon"><i class="fas fa-play"></i></span>
                                    <span>開始測試</span>
                                </button>
                            </div>
                            <div class="control">
                                <button class="button is-small is-warning" onclick="pauseTest()">
                                    <span class="icon"><i class="fas fa-pause"></i></span>
                                    <span>暫停</span>
                                </button>
                            </div>
                            <div class="control">
                                <button class="button is-small is-danger" onclick="stopTest()">
                                    <span class="icon"><i class="fas fa-stop"></i></span>
                                    <span>停止</span>
                                </button>
                            </div>
                            <div class="control ml-4">
                                <label class="checkbox">
                                    <input type="checkbox" id="step-by-step" checked>
                                    逐步執行
                                </label>
                            </div>
                            <div class="control ml-4">
                                <div class="field has-addons">
                                    <div class="control">
                                        <span class="button is-static is-small">測試模式:</span>
                                    </div>
                                    <div class="control">
                                        <div class="select is-small">
                                            <select id="test-mode">
                                                <option value="simulation">模擬測試</option>
                                                <option value="validation">驗證測試</option>
                                                <option value="api">API測試(測試環境)</option>
                                            </select>
                                        </div>
                                    </div>
                                </div>
                            </div>
                        </div>
                    </div>
                    <div id="execution-log" class="box" style="max-height: 400px; overflow-y: auto; font-family: monospace; font-size: 0.9rem;">
                        <div class="has-text-grey">等待開始測試...</div>
                    </div>
                </div>
                
                <div id="test-variables-tab" class="test-tab-content" style="display: none;">
                    <div id="variables-state" class="box">
                        <table class="table is-fullwidth is-striped">
                            <thead>
                                <tr>
                                    <th>變數名稱</th>
                                    <th>當前值</th>
                                    <th>類型</th>
                                    <th>最後更新</th>
                                </tr>
                            </thead>
                            <tbody id="variables-tbody">
                                <tr>
                                    <td colspan="4" class="has-text-grey has-text-centered">尚無變數資料</td>
                                </tr>
                            </tbody>
                        </table>
                    </div>
                </div>
                
                <div id="test-report-tab" class="test-tab-content" style="display: none;">
                    <div id="test-report" class="content">
                        <p class="has-text-grey">測試完成後將顯示報告...</p>
                    </div>
                </div>
            </section>
            <footer class="modal-card-foot">
                <button class="button is-success" onclick="downloadTestReport()">下載報告</button>
                <button class="button" onclick="closeTestModal()">關閉</button>
            </footer>
        </div>
    `;
    return modal;
}

/**
 * Execute flow test
 */
async function executeFlowTest(flowData, modal) {
    const executionLog = modal.querySelector('#execution-log');
    const variablesState = {};
    let testRunning = false;
    let testPaused = false;
    
    // Store test state globally for control functions
    window.testState = {
        running: false,
        paused: false,
        variables: variablesState,
        log: [],
        currentStep: null
    };
    
    // Clear execution log
    executionLog.innerHTML = '';
    
    // Log function
    function logExecution(message, type = 'info') {
        const timestamp = new Date().toLocaleTimeString();
        const logEntry = document.createElement('div');
        logEntry.className = `mb-2 ${type === 'error' ? 'has-text-danger' : type === 'success' ? 'has-text-success' : type === 'warning' ? 'has-text-warning' : ''}`;
        logEntry.innerHTML = `<span class="has-text-grey">[${timestamp}]</span> ${message}`;
        executionLog.appendChild(logEntry);
        executionLog.scrollTop = executionLog.scrollHeight;
        
        // Add to test log
        window.testState.log.push({ timestamp, message, type });
    }
    
    // Update variables display (local function for old test modal)
    function updateLocalVariablesDisplay() {
        const tbody = modal.querySelector('#variables-tbody');
        const variables = Object.entries(variablesState);
        
        if (variables.length === 0) {
            tbody.innerHTML = '<tr><td colspan="4" class="has-text-grey has-text-centered">尚無變數資料</td></tr>';
            return;
        }
        
        tbody.innerHTML = variables.map(([name, data]) => `
            <tr>
                <td><code>${name}</code></td>
                <td><code>${JSON.stringify(data.value)}</code></td>
                <td>${typeof data.value}</td>
                <td>${data.lastUpdated}</td>
            </tr>
        `).join('');
    }
    
    // Simulate step execution
    async function executeStep(step, sectionName, stepIndex) {
        if (!window.testState.running || window.testState.paused) return false;
        
        window.testState.currentStep = step;
        
        logExecution(`執行步驟 [${stepIndex + 1}] ${step.id || 'Unnamed'} - 函數: ${step.exec}`, 'info');
        
        // Check skip conditions
        if (step.skip_if) {
            const condition = evaluateCondition(step.skip_if, variablesState);
            if (condition) {
                logExecution(`  ⏭️ 跳過步驟 (條件成立: ${step.skip_if})`, 'warning');
                return true;
            }
        }
        
        // Simulate function execution (now async)
        const result = await simulateFunctionExecution(step.exec, step.params, variablesState);
        
        if (result.success) {
            logExecution(`  ✅ 執行成功`, 'success');
            
            // Store result if specified
            if (step.store && result.value !== undefined) {
                variablesState[step.store] = {
                    value: result.value,
                    lastUpdated: new Date().toLocaleTimeString()
                };
                logExecution(`  💾 儲存結果到變數: ${step.store} = ${JSON.stringify(result.value)}`, 'info');
                updateLocalVariablesDisplay();
            }
        } else {
            logExecution(`  ❌ 執行失敗: ${result.error}`, 'error');
            return false;
        }
        
        // Wait if step-by-step mode
        if (document.getElementById('step-by-step').checked) {
            await new Promise(resolve => setTimeout(resolve, 1000));
        }
        
        return true;
    }
    
    // Start test execution
    window.startTest = async function() {
        if (window.testState.running) return;
        
        window.testState.running = true;
        window.testState.paused = false;
        
        logExecution('🚀 開始測試執行', 'info');
        logExecution(`流程 ID: ${flowData.flow_id || 'Unknown'}`, 'info');
        logExecution(`工作 ID: ${flowData.work_id || 'Unknown'}`, 'info');
        logExecution('---', 'info');
        
        // Execute workflow sections
        for (const section of flowData.workflow) {
            if (!window.testState.running) break;
            
            // Wait if paused
            while (window.testState.paused && window.testState.running) {
                await new Promise(resolve => setTimeout(resolve, 100));
            }
            
            logExecution(`📂 執行區段: ${section.name || 'Unnamed Section'}`, 'info');
            
            // Check section condition
            if (section.condition) {
                const condition = evaluateCondition(section.condition, variablesState);
                if (!condition) {
                    logExecution(`  ⏭️ 跳過區段 (條件不成立: ${section.condition})`, 'warning');
                    continue;
                }
            }
            
            // Execute steps in section
            if (section.steps && section.steps.length > 0) {
                for (let i = 0; i < section.steps.length; i++) {
                    if (!window.testState.running) break;
                    
                    // Wait if paused
                    while (window.testState.paused && window.testState.running) {
                        await new Promise(resolve => setTimeout(resolve, 100));
                    }
                    
                    const success = await executeStep(section.steps[i], section.name, i);
                    if (!success && !section.steps[i].continue_on_error) {
                        logExecution('⛔ 測試因錯誤而停止', 'error');
                        window.testState.running = false;
                        break;
                    }
                }
            }
        }
        
        if (window.testState.running) {
            logExecution('---', 'info');
            logExecution('✨ 測試執行完成', 'success');
            generateTestReport();
        }
        
        window.testState.running = false;
    };
    
    // Pause test
    window.pauseTest = function() {
        if (window.testState.running) {
            window.testState.paused = !window.testState.paused;
            logExecution(window.testState.paused ? '⏸️ 測試已暫停' : '▶️ 測試繼續執行', 'warning');
        }
    };
    
    // Stop test
    window.stopTest = function() {
        if (window.testState.running) {
            window.testState.running = false;
            window.testState.paused = false;
            logExecution('⏹️ 測試已停止', 'error');
        }
    };
}

/**
 * Evaluate condition
 */
function evaluateCondition(condition, variables) {
    // Simple condition evaluation (can be enhanced)
    // Format: ${variable_name} or ${variable_name} == value
    
    if (!condition) return true;
    
    // Replace variables with their values
    let evaluatedCondition = condition;
    for (const [name, data] of Object.entries(variables)) {
        const pattern = new RegExp(`\\$\\{${name}\\}`, 'g');
        evaluatedCondition = evaluatedCondition.replace(pattern, JSON.stringify(data.value));
    }
    
    // Try to evaluate (be careful with security)
    try {
        // Simple boolean check
        if (evaluatedCondition === 'true' || evaluatedCondition === true) return true;
        if (evaluatedCondition === 'false' || evaluatedCondition === false) return false;
        
        // For demo, just return true if variable exists
        return evaluatedCondition.includes('true') || !evaluatedCondition.includes('false');
    } catch (e) {
        console.error('Condition evaluation error:', e);
        return false;
    }
}

/**
 * Simulate function execution with validation
 */
async function simulateFunctionExecution(funcName, params, variables) {
    const testMode = document.getElementById('test-mode')?.value || 'simulation';
    
    // Mode 1: Validation Mode - Check if function exists
    if (testMode === 'validation' || testMode === 'api' || testMode === 'executor') {
        // Use function index for O(1) lookup (unified approach)
        const funcDef = window.functionIndex ? window.functionIndex[funcName] : null;
        
        if (!funcDef) {
            // Fallback to array search if index not available
            const availableFunctions = window.functionLibrary || [];
            const found = availableFunctions.find(f => f.name === funcName);
            
            if (!found) {
                return { 
                    success: false, 
                    error: `❌ 函數 '${funcName}' 不存在於函數庫中` 
                };
            }
        }
        
        // Validate required parameters
        if (funcDef.params && funcDef.params.length > 0) {
            const missingParams = [];
            
            // Parse params if it's a string
            let parsedParams = params;
            if (typeof params === 'string') {
                try {
                    parsedParams = JSON.parse(params);
                } catch (e) {
                    parsedParams = {};
                }
            }
            
            // Special handling for foreach function
            if (funcName === 'foreach' && parsedParams) {
                // Auto-fill max_iterations with default if not provided
                if (parsedParams.max_iterations === undefined) {
                    parsedParams.max_iterations = 100; // Default value
                    params = parsedParams; // Update params with default
                }
            }
            
            // Check for required parameters (assuming all listed params are required)
            for (const paramName of funcDef.params) {
                if (!parsedParams || parsedParams[paramName] === undefined) {
                    // Check if there's a default value for this parameter
                    if (funcDef.defaults && funcDef.defaults[paramName] !== undefined) {
                        // Use default value
                        if (!parsedParams) parsedParams = {};
                        parsedParams[paramName] = funcDef.defaults[paramName];
                        params = parsedParams; // Update params with default
                    } else {
                        missingParams.push(paramName);
                    }
                }
            }
            
            if (missingParams.length > 0) {
                return { 
                    success: false, 
                    error: `❌ 缺少必要參數: ${missingParams.join(', ')}` 
                };
            }
        }
    }
    
    // Mode 2: Executor Mode - Use FlowExecutor for complete execution
    if (testMode === 'executor') {
        try {
            // Build complete flow data for current step only
            const flowData = {
                flow_id: 'test_flow',
                work_id: 220001,  // Test work ID
                workflow: [{
                    section: 'test_section',
                    steps: [{
                        id: 'test_step',
                        exec: funcName,
                        params: params
                    }]
                }]
            };
            
            // Call the new FlowExecutor API endpoint
            const response = await fetch('/api/flows/execute', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json'
                },
                body: JSON.stringify({
                    flow_data: flowData,
                    dry_run: true  // Use dry-run mode for testing
                })
            });
            
            if (!response.ok) {
                const errorText = await response.text();
                return { 
                    success: false, 
                    error: `FlowExecutor 錯誤 (${response.status}): ${errorText}` 
                };
            }
            
            const result = await response.json();
            
            // Extract step result
            if (result.success && result.steps && result.steps.length > 0) {
                const stepResult = result.steps[0];
                return {
                    success: stepResult.status === 'completed',
                    value: stepResult.result,
                    error: stepResult.error,
                    variables: result.variables  // Include variable changes
                };
            } else {
                return {
                    success: false,
                    error: result.error || 'FlowExecutor 執行失敗'
                };
            }
        } catch (error) {
            return { 
                success: false, 
                error: `FlowExecutor 連接失敗: ${error.message}` 
            };
        }
    }
    
    // Mode 3: API Test Mode - Call real test API using flow_wcs execute endpoint
    if (testMode === 'api') {
        try {
            // Resolve variables in params before sending to API
            let resolvedParams = params;
            if (params && variables) {
                // Convert params to string if it's an object
                let paramsStr = typeof params === 'object' ? JSON.stringify(params) : params;
                
                // Replace ${var} patterns with actual values
                paramsStr = paramsStr.replace(/\$\{([^}]+)\}/g, (match, varName) => {
                    const varData = variables[varName];
                    if (varData && varData.value !== undefined) {
                        // If value is an array or object, return it as JSON string
                        if (typeof varData.value === 'object') {
                            return JSON.stringify(varData.value);
                        }
                        return varData.value;
                    }
                    return match; // Keep original if variable not found
                });
                
                // Parse back to object
                try {
                    resolvedParams = JSON.parse(paramsStr);
                } catch (e) {
                    resolvedParams = params; // Fallback to original if parsing fails
                }
            }
            
            // 使用 nginx proxy 呼叫 web_api 服務的執行端點
            const response = await fetch('http://agvc.webapi/api/flow/execute', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json'
                },
                body: JSON.stringify({
                    function_name: funcName,
                    params: resolvedParams,  // Use resolved params
                    variables: variables
                })
            });
            
            if (!response.ok) {
                const errorText = await response.text();
                return { 
                    success: false, 
                    error: `API 錯誤 (${response.status}): ${errorText}` 
                };
            }
            
            const result = await response.json();
            return {
                success: result.success !== false,
                value: result.result || result.value || result.data || result,
                error: result.error || result.message
            };
        } catch (error) {
            return { 
                success: false, 
                error: `API 連接失敗: ${error.message}` 
            };
        }
    }
    
    // Mode 3: Simulation Mode (default) - Use predefined simulations
    const simulations = {
        'query_agv_status': () => ({ 
            success: true, 
            value: { status: 'idle', battery: 85, position: { x: 100, y: 200 } } 
        }),
        'query_rack_status': () => ({ 
            success: true, 
            value: { occupied: false, items: 0 } 
        }),
        'check_agv_available': () => ({ 
            success: true, 
            value: Math.random() > 0.3 
        }),
        'create_task': () => ({ 
            success: true, 
            value: { task_id: 'TASK_' + Math.floor(Math.random() * 10000), status: 'created' } 
        }),
        'send_agv_to_location': () => ({ 
            success: Math.random() > 0.1, 
            value: true,
            error: 'AGV communication timeout'
        }),
        'wait': () => {
            const waitTime = params?.seconds || 1;
            return { success: true, value: `Waited ${waitTime} seconds` };
        },
        'log_message': () => ({ 
            success: true, 
            value: params?.message || 'Log entry created' 
        })
    };
    
    // In simulation mode, check if function exists in library for warning
    if (testMode === 'simulation') {
        // Check if function exists in categorized structure
        let funcExists = false;
        if (availableFunctions && typeof availableFunctions === 'object') {
            for (const category in availableFunctions) {
                if (availableFunctions[category] && Array.isArray(availableFunctions[category])) {
                    if (availableFunctions[category].some(f => f.name === funcName)) {
                        funcExists = true;
                        break;
                    }
                }
            }
        }
        
        if (!funcExists && Object.keys(availableFunctions).length > 0) {
            // Function doesn't exist but we'll simulate it anyway
            console.warn(`⚠️ 函數 '${funcName}' 不在函數庫中，使用預設模擬`);
        }
    }
    
    // Get simulation or use default
    const simulator = simulations[funcName] || (() => ({ 
        success: true, 
        value: `[模擬] ${funcName}` 
    }));
    
    return simulator();
}

/**
 * Generate test report
 */
function generateTestReport() {
    const reportDiv = document.querySelector('#test-report');
    const variables = Object.entries(window.testState.variables);
    const logs = window.testState.log;
    
    const successCount = logs.filter(l => l.type === 'success').length;
    const errorCount = logs.filter(l => l.type === 'error').length;
    const warningCount = logs.filter(l => l.type === 'warning').length;
    
    reportDiv.innerHTML = `
        <h4 class="title is-5">測試執行報告</h4>
        
        <div class="columns">
            <div class="column">
                <div class="box has-background-success-light">
                    <p class="heading">成功</p>
                    <p class="title">${successCount}</p>
                </div>
            </div>
            <div class="column">
                <div class="box has-background-warning-light">
                    <p class="heading">警告</p>
                    <p class="title">${warningCount}</p>
                </div>
            </div>
            <div class="column">
                <div class="box has-background-danger-light">
                    <p class="heading">錯誤</p>
                    <p class="title">${errorCount}</p>
                </div>
            </div>
        </div>
        
        <h5 class="subtitle is-6">最終變數狀態</h5>
        <table class="table is-fullwidth is-striped">
            <thead>
                <tr>
                    <th>變數</th>
                    <th>最終值</th>
                </tr>
            </thead>
            <tbody>
                ${variables.length > 0 ? 
                    variables.map(([name, data]) => `
                        <tr>
                            <td><code>${name}</code></td>
                            <td><code>${JSON.stringify(data.value)}</code></td>
                        </tr>
                    `).join('') :
                    '<tr><td colspan="2" class="has-text-centered has-text-grey">無變數</td></tr>'
                }
            </tbody>
        </table>
        
        <h5 class="subtitle is-6">執行時間軸</h5>
        <div class="content is-small">
            ${logs.slice(-10).map(log => {
                let message = '';
                if (log.step && log.function) {
                    // Format the message based on log content
                    message = `步驟 [${log.step}] - 函數: ${log.function}`;
                    if (log.type === 'success' && log.result !== undefined) {
                        // For success, show result if available
                        const resultStr = typeof log.result === 'object' ? 
                            JSON.stringify(log.result).substring(0, 50) + '...' : 
                            String(log.result);
                        message += ` ✓`;
                    } else if (log.type === 'error' && log.error) {
                        // For error, show error message
                        message += ` - 錯誤: ${log.error}`;
                    } else if (log.type === 'warning') {
                        message += ` - 警告`;
                    }
                } else if (log.message) {
                    // Fallback to message if available
                    message = log.message;
                } else {
                    // Default message
                    message = `${log.type || '未知操作'}`;
                }
                
                return `
                    <p class="${log.type === 'error' ? 'has-text-danger' : log.type === 'success' ? 'has-text-success' : log.type === 'warning' ? 'has-text-warning' : ''}">
                        <span class="has-text-grey">[${log.timestamp || new Date().toLocaleTimeString()}]</span> ${message}
                    </p>
                `;
            }).join('')}
        </div>
    `;
}

/**
 * Switch test tab
 */
function switchTestTab(tabName) {
    // Update tab buttons
    const modal = document.getElementById('test-execution-modal');
    const tabs = modal.querySelectorAll('.tabs li');
    const contents = modal.querySelectorAll('.test-tab-content');
    
    tabs.forEach(tab => tab.classList.remove('is-active'));
    contents.forEach(content => content.style.display = 'none');
    
    if (tabName === 'execution') {
        tabs[0].classList.add('is-active');
        document.getElementById('test-execution-tab').style.display = 'block';
    } else if (tabName === 'variables') {
        tabs[1].classList.add('is-active');
        document.getElementById('test-variables-tab').style.display = 'block';
    } else if (tabName === 'report') {
        tabs[2].classList.add('is-active');
        document.getElementById('test-report-tab').style.display = 'block';
    }
}

/**
 * Download test report
 */
function downloadTestReport() {
    if (!window.testState || !window.testState.log) {
        showNotification('尚無測試報告可下載', 'warning');
        return;
    }
    
    const report = {
        timestamp: new Date().toISOString(),
        flow_id: document.getElementById('flow-id').value || 'unknown',
        execution_log: window.testState.log,
        variables: window.testState.variables,
        summary: {
            total_steps: window.testState.log.filter(l => l.message.includes('執行步驟')).length,
            success: window.testState.log.filter(l => l.type === 'success').length,
            warnings: window.testState.log.filter(l => l.type === 'warning').length,
            errors: window.testState.log.filter(l => l.type === 'error').length
        }
    };
    
    const blob = new Blob([JSON.stringify(report, null, 2)], { type: 'application/json' });
    const url = URL.createObjectURL(blob);
    const link = document.createElement('a');
    link.href = url;
    link.download = `test_report_${new Date().toISOString().replace(/[:.]/g, '-')}.json`;
    document.body.appendChild(link);
    link.click();
    document.body.removeChild(link);
    URL.revokeObjectURL(url);
}

/**
 * Close test modal
 */
function closeTestModal() {
    const modal = document.getElementById('test-execution-modal');
    if (modal) {
        // Stop test if running
        if (window.testState && window.testState.running) {
            window.stopTest();
        }
        
        // Remove modal immediately without fade-out animation
        modal.classList.remove('is-active');
    }
}

/**
 * Update test mode description
 */
function updateTestModeDescription() {
    const modeSelect = document.getElementById('test-mode');
    const descriptionEl = document.getElementById('test-mode-description');
    const dryRunField = document.getElementById('dry-run-field');
    
    if (!modeSelect || !descriptionEl) return;
    
    const descriptions = {
        'simulation': '使用預定義的模擬值執行流程，不進行實際函數調用（使用配置檔案函數定義）',
        'validation': '驗證函數是否存在於函數庫中，並檢查必要參數是否提供（使用配置檔案函數定義）',
        'api': '調用真實的測試 API 端點，執行實際的函數測試（使用 flow_wcs 即時函數定義）',
        'executor': '使用 FlowExecutor 引擎完整執行流程，包含所有函數的實際執行和變數追蹤（生產環境執行）'
    };
    
    // Show/hide dry-run option based on mode
    if (dryRunField) {
        dryRunField.style.display = (modeSelect.value === 'executor') ? 'block' : 'none';
    }
    
    // Reload function library based on selected mode
    const mode = modeSelect.value;
    let functionSource = 'config';  // Default to config
    
    if (mode === 'api' || mode === 'executor') {
        functionSource = 'flow_wcs';  // API and executor modes use real flow_wcs
    }
    
    // Reload functions with appropriate source
    loadAvailableFunctions(functionSource).then(() => {
        console.log(`Functions reloaded for ${mode} mode from ${functionSource}`);
    });
    
    descriptionEl.textContent = descriptions[modeSelect.value] || descriptions['simulation'];
}

/**
 * Run test execution
 */
function runTest() {
    const flowData = buildFlowData();
    
    // Change button states
    const runBtn = document.getElementById('run-test-button');
    const stopBtn = document.getElementById('stop-test-button');
    if (runBtn) runBtn.style.display = 'none';
    if (stopBtn) stopBtn.style.display = 'inline-flex';
    
    // Clear previous test results
    const logDiv = document.getElementById('test-log');
    if (logDiv) {
        logDiv.innerHTML = '<div class="log-success">開始執行測試...</div>';
    }
    
    // Start test execution
    initializeTestExecution(flowData);
}

/**
 * Stop test execution
 */
function stopTest() {
    if (window.testState) {
        window.testState.running = false;
    }
    
    // Change button states
    const runBtn = document.getElementById('run-test-button');
    const stopBtn = document.getElementById('stop-test-button');
    if (runBtn) runBtn.style.display = 'inline-flex';
    if (stopBtn) stopBtn.style.display = 'none';
    
    // Add stop message to log
    const logDiv = document.getElementById('test-log');
    if (logDiv) {
        logDiv.innerHTML += '<div style="color: #ff9800;">⚠️ 測試已手動停止</div>';
    }
}

/**
 * Initialize test execution
 */
function initializeTestExecution(flowData) {
    // Initialize test state
    window.testState = {
        running: true,
        paused: false,
        currentSection: null,
        currentStep: null,
        variables: {},
        log: [],
        errors: [],
        warnings: []
    };
    
    // Clear test UI
    const logDiv = document.getElementById('test-log');
    const variablesDiv = document.getElementById('test-variables-tbody');
    const reportDiv = document.getElementById('test-report');
    
    if (logDiv) {
        logDiv.innerHTML = '<div style="color: #5bc0de;">開始執行流程測試...</div>';
    }
    
    if (variablesDiv) {
        variablesDiv.innerHTML = '<tr><td colspan="4" class="has-text-grey has-text-centered">尚無變數</td></tr>';
    }
    
    if (reportDiv) {
        reportDiv.innerHTML = '<div class="notification is-info"><p>測試執行中...</p></div>';
    }
    
    // Start execution
    executeFlowTestAsync(flowData);
}

/**
 * Execute flow with FlowExecutor (complete execution)
 */
async function executeFlowWithExecutor(flowData) {
    const logDiv = document.getElementById('test-log');
    const variablesDiv = document.getElementById('test-variables-tbody');
    
    if (logDiv) {
        logDiv.innerHTML += '<div class="log-info">🚀 使用 FlowExecutor 執行完整流程...</div>';
    }
    
    try {
        // Call the FlowExecutor API endpoint for complete flow execution
        const response = await fetch('/linear-flow/api/flows/execute', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            },
            body: JSON.stringify({
                flow_data: flowData,
                dry_run: document.getElementById('dry-run-checkbox')?.checked ?? true
            })
        });
        
        if (!response.ok) {
            const errorText = await response.text();
            if (logDiv) {
                logDiv.innerHTML += `<div class="log-error">❌ FlowExecutor 錯誤 (${response.status}): ${errorText}</div>`;
            }
            window.testState.running = false;
            updateTestButtons();
            return;
        }
        
        const result = await response.json();
        
        // Display execution results
        if (result.success) {
            if (logDiv) {
                logDiv.innerHTML += '<div class="log-success">✅ 流程執行成功</div>';
                
                // Display detailed logs if available
                if (result.detailed_logs && result.detailed_logs.length > 0) {
                    logDiv.innerHTML += '<div class="log-section">📋 詳細執行日誌:</div>';
                    displayDetailedLogs(result.detailed_logs, logDiv);
                } else if (result.raw_logs && result.raw_logs.length > 0) {
                    logDiv.innerHTML += '<div class="log-section">📋 執行日誌:</div>';
                    displayDetailedLogs(result.raw_logs, logDiv);
                } else if (result.logs && result.logs.length > 0) {
                    logDiv.innerHTML += '<div class="log-section">執行日誌:</div>';
                    result.logs.forEach(log => {
                        const logClass = log.level === 'error' ? 'log-error' : 
                                       log.level === 'warning' ? 'log-warning' : 
                                       log.level === 'success' ? 'log-success' : 'log-info';
                        logDiv.innerHTML += `<div class="${logClass}">${log.timestamp || ''} ${log.message}</div>`;
                    });
                }
                
                // Display steps execution details
                if (result.steps && result.steps.length > 0) {
                    logDiv.innerHTML += '<div class="log-section">步驟執行詳情:</div>';
                    result.steps.forEach(step => {
                        const stepClass = step.status === 'completed' ? 'log-success' :
                                        step.status === 'failed' ? 'log-error' :
                                        step.status === 'skipped' ? 'log-warning' : 'log-info';
                        logDiv.innerHTML += `<div class="${stepClass}">步驟 ${step.id}: ${step.status}`;
                        if (step.result) {
                            logDiv.innerHTML += ` - 結果: ${JSON.stringify(step.result)}`;
                        }
                        if (step.error) {
                            logDiv.innerHTML += ` - 錯誤: ${step.error}`;
                        }
                        if (step.variables_before || step.variables_after) {
                            logDiv.innerHTML += ' <span style="color: #666;">[變數已更新]</span>';
                        }
                        logDiv.innerHTML += '</div>';
                    });
                }
            }
            
            // Update variables display
            if (variablesDiv && result.variables) {
                variablesDiv.innerHTML = '';
                Object.entries(result.variables).forEach(([name, value]) => {
                    const row = document.createElement('tr');
                    row.innerHTML = `
                        <td>${name}</td>
                        <td>${typeof value}</td>
                        <td>${JSON.stringify(value)}</td>
                        <td>FlowExecutor</td>
                    `;
                    variablesDiv.appendChild(row);
                });
            }
            
            // Generate test report
            window.testState.log = result.logs || [];
            window.testState.executionTime = result.execution_time || 0;
            window.testState.errors = result.errors || [];
            generateTestReport();
            
        } else {
            if (logDiv) {
                logDiv.innerHTML += `<div class="log-error">❌ 流程執行失敗: ${result.error || '未知錯誤'}</div>`;
                if (result.traceback) {
                    logDiv.innerHTML += `<div class="log-error" style="font-family: monospace; font-size: 0.9em; white-space: pre-wrap;">${result.traceback}</div>`;
                }
            }
        }
        
    } catch (error) {
        if (logDiv) {
            logDiv.innerHTML += `<div class="log-error">❌ 連接錯誤: ${error.message}</div>`;
        }
    } finally {
        window.testState.running = false;
        updateTestButtons();
    }
}

/**
 * Update test buttons state
 */
function updateTestButtons() {
    const runBtn = document.getElementById('run-test-button');
    const stopBtn = document.getElementById('stop-test-button');
    if (runBtn) runBtn.style.display = 'inline-flex';
    if (stopBtn) stopBtn.style.display = 'none';
}

/**
 * Execute flow test asynchronously
 */
async function executeFlowTestAsync(flowData) {
    const testMode = document.getElementById('test-mode')?.value || 'simulation';
    const logDiv = document.getElementById('test-log');
    
    // Log test mode
    if (logDiv) {
        logDiv.innerHTML += `<div style="color: #5bc0de;">測試模式: ${testMode}</div>`;
    }
    
    // If executor mode, execute the entire flow at once
    if (testMode === 'executor') {
        await executeFlowWithExecutor(flowData);
        return;
    }
    
    // Execute workflow sections
    for (const section of flowData.workflow) {
        if (!window.testState.running) break;
        
        window.testState.currentSection = section.section || section.name;
        
        // Log section start
        if (logDiv) {
            logDiv.innerHTML += `<div class="log-section">區段: ${section.section || section.name || 'undefined'}</div>`;
        }
        
        // Check section condition
        if (section.condition) {
            const condResult = evaluateCondition(section.condition, window.testState.variables);
            if (!condResult) {
                if (logDiv) {
                    logDiv.innerHTML += `<div class="log-warning">跳過區段 (條件不滿足)</div>`;
                }
                continue;
            }
        }
        
        // Execute steps
        for (const step of section.steps) {
            if (!window.testState.running) break;
            
            await executeStepAsync(step, section.name);
        }
    }
    
    // Test completed
    window.testState.running = false;
    
    // Update UI
    const runBtn = document.getElementById('run-test-button');
    const stopBtn = document.getElementById('stop-test-button');
    if (runBtn) runBtn.style.display = 'inline-flex';
    if (stopBtn) stopBtn.style.display = 'none';
    
    // Generate report
    generateTestReport();
    
    if (logDiv) {
        logDiv.innerHTML += '<div class="log-success" style="margin-top: 10px;"><strong>測試完成！</strong></div>';
    }
}

/**
 * Execute a single step asynchronously
 */
async function executeStepAsync(step, sectionName) {
    const logDiv = document.getElementById('test-log');
    
    if (!window.testState.running) return;
    
    window.testState.currentStep = step.id;
    
    // Log step start
    if (logDiv) {
        logDiv.innerHTML += `<div class="log-step">步驟 ${step.id || 'undefined'}: ${step.exec || 'undefined'}</div>`;
    }
    
    // Check skip condition
    if (step.skip_if) {
        const skipResult = evaluateCondition(step.skip_if, window.testState.variables);
        if (skipResult) {
            if (logDiv) {
                logDiv.innerHTML += `<div class="log-warning" style="margin-left: 40px;">跳過 (條件滿足)</div>`;
            }
            return;
        }
    }
    
    // Execute function
    try {
        const result = await simulateFunctionExecution(step.exec, step.params, window.testState.variables);
        
        if (result.success) {
            // Store result in variable if specified
            if (step.store) {
                window.testState.variables[step.store] = {
                    value: result.value,
                    type: typeof result.value,
                    source: `${sectionName}.${step.id}`
                };
                
                // Update test variables display
                updateTestVariablesDisplay();
            }
            
            if (logDiv) {
                logDiv.innerHTML += `<div class="log-success" style="margin-left: 40px;">✓ 成功${step.store ? ` (儲存至 ${step.store})` : ''}</div>`;
            }
            
            // Add to log
            window.testState.log.push({
                type: 'success',
                timestamp: new Date().toLocaleTimeString(),
                step: step.id,
                function: step.exec,
                result: result.value
            });
        } else {
            if (logDiv) {
                logDiv.innerHTML += `<div class="log-error" style="margin-left: 40px;">✗ ${result.error || '執行失敗'}</div>`;
            }
            
            // Add to errors
            window.testState.errors.push({
                step: step.id,
                function: step.exec,
                error: result.error
            });
            
            // Add to log
            window.testState.log.push({
                type: 'error',
                timestamp: new Date().toLocaleTimeString(),
                step: step.id,
                function: step.exec,
                error: result.error
            });
        }
    } catch (error) {
        if (logDiv) {
            logDiv.innerHTML += `<div style="margin-left: 40px; color: #f44336;">✗ 錯誤: ${error.message}</div>`;
        }
        
        window.testState.errors.push({
            step: step.id,
            function: step.exec,
            error: error.message
        });
        
        window.testState.log.push({
            type: 'error',
            timestamp: new Date().toLocaleTimeString(),
            step: step.id,
            function: step.exec,
            error: error.message
        });
    }
    
    // Scroll log to bottom
    if (logDiv) {
        logDiv.scrollTop = logDiv.scrollHeight;
    }
}

/**
 * Update test variables display (for test modal)
 */
function updateTestVariablesDisplay() {
    const tbody = document.getElementById('test-variables-tbody');
    if (!tbody) return;
    
    const variables = window.testState ? window.testState.variables : {};
    
    if (Object.keys(variables).length === 0) {
        tbody.innerHTML = '<tr><td colspan="4" class="has-text-grey has-text-centered">尚無變數</td></tr>';
        return;
    }
    
    let html = '';
    for (const [name, data] of Object.entries(variables)) {
        const valueStr = typeof data.value === 'object' ? 
            JSON.stringify(data.value, null, 2) : 
            String(data.value);
        
        html += `
            <tr>
                <td><code>${name}</code></td>
                <td>${data.type}</td>
                <td><pre style="max-width: 300px; overflow: auto;">${valueStr}</pre></td>
                <td>${data.source}</td>
            </tr>
        `;
    }
    
    tbody.innerHTML = html;
}

// showNotification function already defined above (line 803)

/**
 * Close modal
 */
function closeModal(modalId) {
    const modal = document.getElementById(modalId);
    
    // Remove modal immediately without fade-out animation
    modal.classList.remove('is-active');
}

// ============================================
// Drag and Drop Functions for Reordering
// ============================================

let draggedSection = null;
let draggedStep = null;

/**
 * Handle section drag start
 */
function handleSectionDragStart(e) {
    draggedSection = this;
    this.classList.add('dragging');
    e.dataTransfer.effectAllowed = 'move';
    e.dataTransfer.setData('text/html', this.innerHTML);
}

/**
 * Handle section drag end
 */
function handleSectionDragEnd(e) {
    this.classList.remove('dragging');
    
    // Remove drag over styles from all sections
    document.querySelectorAll('.workflow-section').forEach(section => {
        section.classList.remove('drag-over');
    });
}

/**
 * Handle section drag over
 */
function handleSectionDragOver(e) {
    if (e.preventDefault) {
        e.preventDefault();
    }
    
    // Check if dragging a function from library
    const functionName = e.dataTransfer.types.includes('function');
    const draggedSection = document.querySelector('.workflow-section.dragging');
    
    if (functionName || draggedSection) {
        e.dataTransfer.dropEffect = functionName ? 'copy' : 'move';
        
        // Add visual feedback
        if (!this.classList.contains('dragging')) {
            this.classList.add('drag-over');
        }
    }
    
    return false;
}

/**
 * Handle section drag enter
 */
function handleSectionDragEnter(e) {
    if (!this.classList.contains('dragging')) {
        this.classList.add('drag-over');
    }
}

/**
 * Handle section drag leave
 */
function handleSectionDragLeave(e) {
    this.classList.remove('drag-over');
}

/**
 * Handle section drop
 */
function handleSectionDrop(e) {
    if (e.stopPropagation) {
        e.stopPropagation();
    }
    
    this.classList.remove('drag-over');
    
    // Check if dropping a function from library
    const functionData = e.dataTransfer.getData('function');
    if (functionData) {
        // Handle function drop - add as new step to this section
        const sectionElement = this.closest('.workflow-section');
        const sectionIndex = parseInt(sectionElement.dataset.sectionIndex);
        
        if (sectionIndex >= 0 && sectionIndex < workflowSections.length) {
            const newStep = {
                id: generateStepId(),
                exec: functionData,
                params: {},
                skip_if: '',
                skip_if_not: '',
                store: ''
            };
            
            // Add step to section
            if (!workflowSections[sectionIndex].steps) {
                workflowSections[sectionIndex].steps = [];
            }
            workflowSections[sectionIndex].steps.push(newStep);
            
            // Update UI
            renderWorkflow();
            renderVisualEditor();
            showNotification(`✅ 已新增步驟: ${functionData}`, 'success');
        }
    } else if (draggedSection && draggedSection !== this) {
        // Handle section reordering
        const sections = Array.from(document.querySelectorAll('.workflow-section'));
        const draggedIndex = sections.indexOf(draggedSection);
        const targetIndex = sections.indexOf(this);
        
        // Reorder in workflowSections data
        const movedSection = workflowSections.splice(draggedIndex, 1)[0];
        workflowSections.splice(targetIndex, 0, movedSection);
        
        // Update UI
        renderWorkflow();
        renderVisualEditor();
        showNotification('✅ 區段順序已更新', 'success');
    }
    
    return false;
}

/**
 * Handle step drag start - DEPRECATED (not for left panel)
 * This function should not be used for left panel steps
 */
function handleStepDragStart(e) {
    // Left panel steps should not be draggable
    e.preventDefault();
    e.stopPropagation();
    return false;
}

/**
 * Handle step drag end
 */
function handleStepDragEnd(e) {
    this.classList.remove('dragging');
    
    // Remove drag over styles and indicators from all steps
    document.querySelectorAll('.workflow-step').forEach(step => {
        step.classList.remove('drag-over');
    });
    
    // Clean up all drop indicators
    document.querySelectorAll('.drop-indicator').forEach(indicator => indicator.remove());
    
    // Clean up all insertion position data
    document.querySelectorAll('[data-insert-position]').forEach(elem => {
        delete elem.dataset.insertPosition;
    });
}

/**
 * Handle step drag over - REMOVED (not needed for left panel)
 * Left panel steps are not draggable, only visual editor steps
 */
function handleStepDragOver(e) {
    // This function is deprecated - left panel doesn't support dragging
    e.preventDefault();
    return false;
}

/**
 * Handle step drag enter
 */
function handleStepDragEnter(e) {
    if (!this.classList.contains('dragging')) {
        this.classList.add('drag-over');
    }
}

/**
 * Handle step drag leave
 */
function handleStepDragLeave(e) {
    this.classList.remove('drag-over');
    this.classList.remove('drop-insertion-indicator');
    
    // Remove drop indicator line
    const indicator = this.querySelector('.drop-indicator');
    if (indicator) {
        indicator.remove();
    }
    
    // Clean up dataset
    delete this.dataset.insertPosition;
}

/**
 * Handle step drop
 */
function handleStepDrop(e) {
    if (e.stopPropagation) {
        e.stopPropagation();
    }
    
    this.classList.remove('drag-over');
    
    // Check if it's a function from library
    const functionName = e.dataTransfer.getData('function');
    if (functionName) {
        // Insert function as new step after this step
        const targetSection = this.closest('.workflow-section');
        const sections = Array.from(document.querySelectorAll('.workflow-section'));
        const targetSectionIndex = sections.indexOf(targetSection);
        const targetSectionSteps = Array.from(targetSection.querySelectorAll('.workflow-step'));
        const targetStepIndex = targetSectionSteps.indexOf(this);
        
        const newStep = {
            id: generateStepId(),
            exec: functionName,
            params: {},
            store: ''
        };
        
        // Get current flow state
        const currentFlow = flowStore.getState();
        
        // Create updated workflow with new step inserted
        const updatedWorkflow = [...currentFlow.workflow];
        updatedWorkflow[targetSectionIndex] = {
            ...updatedWorkflow[targetSectionIndex],
            steps: [...updatedWorkflow[targetSectionIndex].steps]
        };
        updatedWorkflow[targetSectionIndex].steps.splice(targetStepIndex + 1, 0, newStep);
        
        // Update using flowStore
        flowStore.updateWorkflow(updatedWorkflow);
        
        showNotification(`✅ 已新增步驟: ${functionName}`, 'success');
        return false;
    }
    
    // Handle step reordering
    if (draggedStep && draggedStep !== this) {
        // Find section indices
        const draggedSection = draggedStep.closest('.workflow-section');
        const targetSection = this.closest('.workflow-section');
        
        const sections = Array.from(document.querySelectorAll('.workflow-section'));
        const draggedSectionIndex = sections.indexOf(draggedSection);
        const targetSectionIndex = sections.indexOf(targetSection);
        
        // Find step indices within their sections
        const draggedSectionSteps = Array.from(draggedSection.querySelectorAll('.workflow-step'));
        const targetSectionSteps = Array.from(targetSection.querySelectorAll('.workflow-step'));
        const draggedStepIndex = draggedSectionSteps.indexOf(draggedStep);
        let targetStepIndex = targetSectionSteps.indexOf(this);
        
        // Get current flow state
        const currentFlow = flowStore.getState();
        
        // Create updated workflow with step moved
        const updatedWorkflow = [...currentFlow.workflow];
        
        // Clone the sections being modified
        updatedWorkflow[draggedSectionIndex] = {
            ...updatedWorkflow[draggedSectionIndex],
            steps: [...updatedWorkflow[draggedSectionIndex].steps]
        };
        
        // Remove the step from source
        const movedStep = updatedWorkflow[draggedSectionIndex].steps.splice(draggedStepIndex, 1)[0];
        
        // Adjust target index if moving within same section and from before target
        if (draggedSectionIndex === targetSectionIndex && draggedStepIndex < targetStepIndex) {
            targetStepIndex--;
        }
        
        // Clone target section if different from source
        if (targetSectionIndex !== draggedSectionIndex) {
            updatedWorkflow[targetSectionIndex] = {
                ...updatedWorkflow[targetSectionIndex],
                steps: [...updatedWorkflow[targetSectionIndex].steps]
            };
        }
        
        // Insert after the target step (add 1 to index)
        updatedWorkflow[targetSectionIndex].steps.splice(targetStepIndex + 1, 0, movedStep);
        
        // Update using flowStore
        flowStore.updateWorkflow(updatedWorkflow);
        
        showNotification('✅ 步驟順序已更新', 'success');
    }
    
    return false;
}

/**
 * Move section up or down
 */
function moveSection(index, direction) {
    const currentFlow = flowStore.getState();
    const newIndex = index + direction;
    const state = flowStore.getState();
    
    if (newIndex >= 0 && newIndex < state.workflow.length) {
        // Move section in store
        flowStore.moveSection(index, newIndex);
        
        showNotification('✅ 區段已移動', 'success');
    }
}

/**
 * Move step up or down within a section
 */
function moveStep(sectionIndex, stepIndex, direction) {
    const currentFlow = flowStore.getState();
    const state = flowStore.getState();
    const section = state.workflow[sectionIndex];
    const newIndex = stepIndex + direction;
    
    if (newIndex >= 0 && newIndex < section.steps.length) {
        // Move step in store
        flowStore.moveStep(sectionIndex, stepIndex, newIndex);
        
        showNotification('✅ 步驟已移動', 'success');
    }
}

/**
 * Handle function drag start
 */
function handleFunctionDragStart(e) {
    e.dataTransfer.effectAllowed = 'copy';
    const functionName = e.currentTarget.getAttribute('data-function');
    
    // Set both formats for compatibility
    e.dataTransfer.setData('function', functionName);
    
    // Also set as JSON for drop zones that expect text/plain
    const dragData = {
        type: 'function',
        functionName: functionName
    };
    e.dataTransfer.setData('text/plain', JSON.stringify(dragData));
    
    e.currentTarget.classList.add('is-dragging');
}

/**
 * Handle function drag end
 */
function handleFunctionDragEnd(e) {
    e.currentTarget.classList.remove('is-dragging');
}

/**
 * Handle visual section drag over (for dropping functions from library)
 */
function handleVisualSectionDragOver(e) {
    if (e.preventDefault) {
        e.preventDefault();
    }
    
    // Check if dragging a function from library
    const functionData = e.dataTransfer.types.includes('text/plain') && 
                        document.querySelector('.function-item.is-dragging');
    
    if (functionData) {
        e.dataTransfer.dropEffect = 'copy';
        this.classList.add('drag-over');
    }
    
    return false;
}

/**
 * Handle visual section drag enter
 */
function handleVisualSectionDragEnter(e) {
    const functionData = e.dataTransfer.types.includes('text/plain') && 
                        document.querySelector('.function-item.is-dragging');
    
    if (functionData) {
        this.classList.add('drag-over');
    }
}

/**
 * Handle visual section drag leave
 */
function handleVisualSectionDragLeave(e) {
    // Check if we're actually leaving the section (not just moving to a child element)
    if (!this.contains(e.relatedTarget)) {
        this.classList.remove('drag-over');
    }
}

/**
 * Handle visual section drop (for functions from library)
 */
function handleVisualSectionDrop(e) {
    if (e.stopPropagation) {
        e.stopPropagation();
    }
    if (e.preventDefault) {
        e.preventDefault();
    }
    
    this.classList.remove('drag-over');
    
    // Get function data
    const functionName = e.dataTransfer.getData('function');
    if (functionName) {
        const sectionIndex = parseInt(this.dataset.sectionIndex);
        
        // Use currentFlow.workflow instead of workflowSections
        if (sectionIndex >= 0 && sectionIndex < currentFlow.workflow.length) {
            const newStep = {
                id: generateStepId(),
                exec: functionName,
                params: {},
                skip_if: '',
                skip_if_not: '',
                store: ''
            };
            
            // Add step to section in currentFlow
            if (!currentFlow.workflow[sectionIndex].steps) {
                currentFlow.workflow[sectionIndex].steps = [];
            }
            flowStore.addStep(sectionIndex, newStep);
            
            // Update UI
            renderWorkflow();
            renderVisualEditor();
            updateYamlEditor();
            
            // Select the new step for editing
            const stepIndex = currentFlow.workflow[sectionIndex].steps.length - 1;
            selectStepInEditor(sectionIndex, stepIndex);
            
            showNotification(`✅ 已新增步驟: ${functionName}`, 'success');
        }
    }
    
    return false;
}

/**
 * Render Visual Editor Area (Center Panel)
 */
function renderVisualEditor() {
    const currentFlow = flowStore.getState();
    const visualEditor = document.querySelector('.visual-editor');
    if (!visualEditor) return;
    
    let html = '<div class="visual-editor-content">';
    
    // Get workflow from store
    const state = flowStore.getState();
    const workflow = state.workflow || [];
    
    // Check if workflow is empty
    if (workflow.length === 0) {
        // Only show top drop zone when completely empty
        html += '<div class="visual-drop-zone visual-drop-zone-top" data-insert-position="start">將區段或步驟拖放到這裡開始設計流程</div>';
    } else {
        // When sections exist, don't show top drop zone
        workflow.forEach((section, sectionIndex) => {
            html += createVisualSectionElement(section, sectionIndex);
        });
        
        // Show bottom drop zone when sections exist
        html += '<div class="visual-drop-zone visual-drop-zone-bottom" data-insert-position="end">將新區段拖放到這裡</div>';
    }
    
    html += '</div>';
    
    visualEditor.innerHTML = html;
    
    // Add event listeners for drop zones
    visualEditor.querySelectorAll('.visual-drop-zone').forEach(zone => {
        zone.addEventListener('dragover', handleVisualDropZoneDragOver);
        zone.addEventListener('drop', handleVisualDropZoneDrop);
        zone.addEventListener('dragleave', handleVisualDropZoneDragLeave);
    });
    
    // Add event listeners for visual steps
    visualEditor.querySelectorAll('.visual-step').forEach(step => {
        step.addEventListener('dragstart', handleVisualStepDragStart);
        step.addEventListener('dragend', handleVisualStepDragEnd);
        step.addEventListener('dragover', handleVisualStepDragOver);
        step.addEventListener('drop', handleVisualStepDrop);
        step.addEventListener('dragenter', handleVisualStepDragEnter);
        step.addEventListener('dragleave', handleVisualStepDragLeave);
        step.addEventListener('click', (e) => {
            e.preventDefault();
            selectStepInEditor(step.dataset.sectionIndex, step.dataset.stepIndex);
        });
    });
    
    // Add event listeners for visual sections to accept dropped functions
    visualEditor.querySelectorAll('.visual-section').forEach(section => {
        section.addEventListener('dragover', handleVisualSectionDragOver);
        section.addEventListener('drop', handleVisualSectionDrop);
        section.addEventListener('dragleave', handleVisualSectionDragLeave);
        section.addEventListener('dragenter', handleVisualSectionDragEnter);
    });
}

/**
 * Create Visual Section Element for Center Panel
 */
function createVisualSectionElement(section, sectionIndex) {
    let html = '<div class="visual-section" data-section-index="' + sectionIndex + '" onclick="selectSection(' + sectionIndex + ')">';
    
    // Section header
    html += '<div class="visual-section-header" onclick="selectSection(' + sectionIndex + ')">';
    html += '<div class="section-icon"><i class="fas fa-layer-group"></i></div>';
    html += '<div class="section-title">' + (section.section || 'Untitled Section') + '</div>';
    html += '<div class="section-step-count">(' + (section.steps ? section.steps.length : 0) + ' 步驟)</div>';
    html += '</div>';
    
    // Steps container
    html += '<div class="visual-steps-container">';
    
    if (section.steps && section.steps.length > 0) {
        // Don't show between drop zones to save space
        section.steps.forEach((step, stepIndex) => {
            html += createVisualStepElement(step, sectionIndex, stepIndex);
        });
    } else {
        // Only show empty drop zone when section has no steps
        html += '<div class="visual-drop-zone visual-drop-zone-empty" data-section-index="' + sectionIndex + '" data-insert-position="0">拖放步驟到此區段</div>';
    }
    
    html += '</div>'; // visual-steps-container
    html += '</div>'; // visual-section
    
    return html;
}

/**
 * Create Visual Step Element for Center Panel
 */
function createVisualStepElement(step, sectionIndex, stepIndex) {
    let html = '<div class="visual-step" draggable="true" data-section-index="' + sectionIndex + '" data-step-index="' + stepIndex + '" onclick="event.stopPropagation(); editStep(' + sectionIndex + ', ' + stepIndex + ')">';
    
    // Combined drag handle and step number
    html += '<div class="visual-step-drag-handle">';
    html += '<span class="step-number-text">' + (stepIndex + 1) + '</span>';
    html += '<i class="fas fa-grip-vertical drag-icon"></i>';
    html += '</div>';
    
    // Step content
    html += '<div class="visual-step-content">';
    
    // Step header (without step number)
    html += '<div class="visual-step-header">';
    html += '<div class="step-id">' + (step.id || 'untitled_step') + '</div>';
    html += '</div>';
    
    // Step function
    if (step.exec) {
        // Check if function exists in any category
        let isValidFunction = false;
        if (availableFunctions) {
            for (const category in availableFunctions) {
                if (availableFunctions[category]) {
                    const found = availableFunctions[category].find(f => f.name === step.exec);
                    if (found) {
                        isValidFunction = true;
                        break;
                    }
                }
            }
        }
        html += '<div class="step-function ' + (isValidFunction ? 'valid' : 'invalid') + '">';
        html += '<i class="fas fa-cube"></i> ' + step.exec;
        html += '</div>';
    }
    
    // Step parameters preview (only param names like function-item)
    if (step.params) {
        let params = step.params;
        // Parse if it's a string
        if (typeof params === 'string') {
            try {
                params = JSON.parse(params);
            } catch (e) {
                // If parsing fails, treat as empty object
                params = {};
            }
        }
        
        const paramKeys = Object.keys(params);
        if (paramKeys.length > 0) {
            html += '<div class="visual-step-params">';
            paramKeys.forEach(key => {
                // Only show param name, not value (like function library)
                html += '<span class="visual-step-param">' + key + '</span>';
            });
            html += '</div>';
        }
    }
    
    // Store variable display with purple color (like variable panel)
    if (step.store) {
        html += '<div class="visual-step-store">';
        html += '<span class="visual-step-store-tag" style="background: #f3e7ff; color: #9b59b6;">';
        html += '<i class="fas fa-save"></i> → ' + step.store;
        html += '</span>';
        html += '</div>';
    }
    
    // Step conditions
    if (step.skip_if || step.condition || step.skip_if_not) {
        html += '<div class="step-conditions">';
        html += '<i class="fas fa-code-branch"></i> 條件';
        html += '</div>';
    }
    
    html += '</div>'; // visual-step-content
    
    // Step actions
    html += '<div class="visual-step-actions">';
    html += '<button class="button is-small" onclick="event.stopPropagation(); moveStepInEditor(' + sectionIndex + ', ' + stepIndex + ', -1)">';
    html += '<i class="fas fa-arrow-up"></i>';
    html += '</button>';
    html += '<button class="button is-small" onclick="event.stopPropagation(); moveStepInEditor(' + sectionIndex + ', ' + stepIndex + ', 1)">';
    html += '<i class="fas fa-arrow-down"></i>';
    html += '</button>';
    html += '<button class="button is-small" onclick="event.stopPropagation(); confirmDeleteStepFromVisual(' + sectionIndex + ', ' + stepIndex + ')">';
    html += '<i class="fas fa-trash"></i>';
    html += '</button>';
    html += '</div>';
    
    html += '</div>'; // visual-step
    
    return html;
}

/**
 * Handle Visual Drop Zone Events
 */
function handleVisualDropZoneDragOver(e) {
    e.preventDefault();
    e.stopPropagation();
    this.classList.add('drag-over');
    return false;
}

function handleVisualDropZoneDragLeave(e) {
    e.preventDefault();
    e.stopPropagation();
    this.classList.remove('drag-over');
    return false;
}

function handleVisualDropZoneDrop(e) {
    e.preventDefault();
    e.stopPropagation();
    this.classList.remove('drag-over');
    
    // Try to get drag data from text/plain first
    let dragData = null;
    try {
        const textData = e.dataTransfer.getData('text/plain');
        if (textData) {
            dragData = JSON.parse(textData);
        }
    } catch (err) {
        // If JSON parse fails, check for direct function data
        const functionName = e.dataTransfer.getData('function');
        if (functionName) {
            dragData = {
                type: 'function',
                functionName: functionName
            };
        }
    }
    
    if (dragData) {
        if (dragData.type === 'function') {
            // Handle function drop - create new step
            insertFunctionAsStep(dragData, this);
        } else if (dragData.type === 'step') {
            // Handle step move
            moveStepToDropZone(dragData, this);
        }
    }
    
    return false;
}

/**
 * Handle Visual Step Drag Events
 */
function handleVisualStepDragStart(e) {
    this.classList.add('dragging');
    
    const sectionIndex = parseInt(this.dataset.sectionIndex);
    const stepIndex = parseInt(this.dataset.stepIndex);
    
    // Get current flow data from flowStore
    const currentFlow = flowStore.getState();
    
    // Defensive check for workflow existence
    if (!currentFlow || !currentFlow.workflow || 
        !currentFlow.workflow[sectionIndex] || 
        !currentFlow.workflow[sectionIndex].steps ||
        !currentFlow.workflow[sectionIndex].steps[stepIndex]) {
        console.error('Invalid flow structure for drag operation', {
            currentFlow,
            sectionIndex,
            stepIndex
        });
        e.preventDefault();
        return;
    }
    
    const dragData = {
        type: 'step',
        sectionIndex: sectionIndex,
        stepIndex: stepIndex,
        step: currentFlow.workflow[sectionIndex].steps[stepIndex]
    };
    
    e.dataTransfer.setData('text/plain', JSON.stringify(dragData));
    e.dataTransfer.effectAllowed = 'move';
}

function handleVisualStepDragEnd(e) {
    this.classList.remove('dragging');
    
    // Clean up any drag-over states and indicators
    document.querySelectorAll('.visual-drop-zone.drag-over').forEach(zone => {
        zone.classList.remove('drag-over');
    });
    document.querySelectorAll('.visual-step.drag-over').forEach(step => {
        step.classList.remove('drag-over');
    });
    
    // Remove all drop indicators
    document.querySelectorAll('.drop-indicator').forEach(indicator => indicator.remove());
    
    // Clean up all insertion position data
    document.querySelectorAll('[data-insert-position]').forEach(elem => {
        delete elem.dataset.insertPosition;
    });
}

function handleVisualStepDragOver(e) {
    if (e.preventDefault) {
        e.preventDefault();
    }
    e.dataTransfer.dropEffect = 'move';
    
    // Add visual feedback with insertion indicator
    if (!this.classList.contains('dragging')) {
        // Calculate drop position
        const rect = this.getBoundingClientRect();
        const midpoint = rect.top + rect.height / 2;
        const insertBefore = e.clientY < midpoint;
        
        // Remove existing indicators
        document.querySelectorAll('.drop-indicator').forEach(indicator => indicator.remove());
        
        // Create and position the drop indicator line
        const indicator = document.createElement('div');
        indicator.className = 'drop-indicator';
        
        // Position the indicator line
        if (insertBefore) {
            indicator.style.top = '0';
            this.insertBefore(indicator, this.firstChild);
        } else {
            indicator.style.bottom = '0';
            indicator.style.top = 'auto';
            this.appendChild(indicator);
        }
        
        // Store insertion position
        this.dataset.insertPosition = insertBefore ? 'before' : 'after';
        
        this.classList.add('drag-over');
    }
    
    return false;
}

function handleVisualStepDragEnter(e) {
    if (!this.classList.contains('dragging')) {
        this.classList.add('drag-over');
    }
}

function handleVisualStepDragLeave(e) {
    this.classList.remove('drag-over');
    
    // Remove drop indicator line
    const indicator = this.querySelector('.drop-indicator');
    if (indicator) {
        indicator.remove();
    }
    
    // Clean up dataset
    delete this.dataset.insertPosition;
}

function handleVisualStepDrop(e) {
    if (e.stopPropagation) {
        e.stopPropagation();
    }
    if (e.preventDefault) {
        e.preventDefault();
    }
    
    this.classList.remove('drag-over');
    
    // Remove drop indicator
    const indicator = this.querySelector('.drop-indicator');
    if (indicator) {
        indicator.remove();
    }
    
    const targetSectionIndex = parseInt(this.dataset.sectionIndex);
    const targetStepIndex = parseInt(this.dataset.stepIndex);
    const insertPosition = this.dataset.insertPosition || 'after'; // Get insertion position
    
    // Check if it's a function from library
    const functionName = e.dataTransfer.getData('function');
    if (functionName) {
        // Insert function as new step
        const newStep = {
            id: generateStepId(),
            exec: functionName,
            params: {},
            store: ''
        };
        
        // Get current flow state
        const currentFlow = flowStore.getState();
        
        // Insert based on drop position indicator
        const insertIndex = insertPosition === 'before' ? targetStepIndex : targetStepIndex + 1;
        
        // Create a new workflow array with the new step inserted
        const updatedWorkflow = [...currentFlow.workflow];
        updatedWorkflow[targetSectionIndex] = {
            ...updatedWorkflow[targetSectionIndex],
            steps: [...updatedWorkflow[targetSectionIndex].steps]
        };
        updatedWorkflow[targetSectionIndex].steps.splice(insertIndex, 0, newStep);
        
        // Update using flowStore
        flowStore.updateWorkflow(updatedWorkflow);
        
        showNotification(`✅ 已新增步驟: ${functionName}`, 'success');
        return false;
    }
    
    // Handle step reordering (from text/plain data)
    try {
        const dragData = JSON.parse(e.dataTransfer.getData('text/plain'));
        if (dragData.type === 'step') {
            const sourceSectionIndex = dragData.sectionIndex;
            const sourceStepIndex = dragData.stepIndex;
            
            // Don't drop on self
            if (sourceSectionIndex === targetSectionIndex && sourceStepIndex === targetStepIndex) {
                return false;
            }
            
            // Get current flow state
            const currentFlow = flowStore.getState();
            
            // Create updated workflow with step moved
            const updatedWorkflow = [...currentFlow.workflow];
            
            // Clone the source section
            updatedWorkflow[sourceSectionIndex] = {
                ...updatedWorkflow[sourceSectionIndex],
                steps: [...updatedWorkflow[sourceSectionIndex].steps]
            };
            
            // Remove step from source
            const movedStep = updatedWorkflow[sourceSectionIndex].steps.splice(sourceStepIndex, 1)[0];
            
            // Calculate insertion index based on drop position
            let insertIndex = insertPosition === 'before' ? targetStepIndex : targetStepIndex + 1;
            
            // Adjust if moving within same section
            if (sourceSectionIndex === targetSectionIndex) {
                if (sourceStepIndex < targetStepIndex) {
                    // Moving from before to after in same section
                    insertIndex--;
                }
            } else {
                // Clone the target section if different from source
                updatedWorkflow[targetSectionIndex] = {
                    ...updatedWorkflow[targetSectionIndex],
                    steps: [...updatedWorkflow[targetSectionIndex].steps]
                };
            }
            
            // Insert at calculated position
            updatedWorkflow[targetSectionIndex].steps.splice(insertIndex, 0, movedStep);
            
            // Update using flowStore
            flowStore.updateWorkflow(updatedWorkflow);
            
            showNotification('✅ 步驟已移動', 'success');
        }
    } catch (e) {
        // Ignore if can't parse drag data
    }
    
    // Clean up dataset
    delete this.dataset.insertPosition;
    
    return false;
}

/**
 * Insert Function as New Step
 */
function insertFunctionAsStep(dragData, dropZone) {
    // Get default parameters for this function
    const defaultParams = getFunctionDefaults(dragData.functionName);
    
    const newStep = {
        id: generateStepId(dragData.functionName),
        exec: dragData.functionName,
        params: defaultParams || {},
        store: ''
    };
    
    const sectionIndex = dropZone.dataset.sectionIndex ? parseInt(dropZone.dataset.sectionIndex) : -1;
    const insertPosition = dropZone.dataset.insertPosition;
    
    // Get current flow state
    const currentFlow = flowStore.getState();
    
    if (insertPosition === 'start') {
        // Create new section at beginning with this step
        const newSection = { 
            section: 'New Section', 
            steps: [newStep] 
        };
        const updatedWorkflow = [newSection, ...currentFlow.workflow];
        flowStore.updateWorkflow(updatedWorkflow);
    } else if (insertPosition === 'end') {
        // Create new section at end with this step (for bottom zone)
        const newSection = { 
            section: 'New Section', 
            steps: [newStep] 
        };
        flowStore.addSection(newSection);
    } else if (sectionIndex >= 0) {
        // Insert at specific position in existing section
        const position = parseInt(insertPosition) || 0;
        const updatedWorkflow = [...currentFlow.workflow];
        updatedWorkflow[sectionIndex] = {
            ...updatedWorkflow[sectionIndex],
            steps: [...updatedWorkflow[sectionIndex].steps]
        };
        updatedWorkflow[sectionIndex].steps.splice(position, 0, newStep);
        flowStore.updateWorkflow(updatedWorkflow);
    }
    
    showNotification('✅ 已插入新步驟: ' + newStep.id, 'success');
}

/**
 * Move Step to Drop Zone
 */
function moveStepToDropZone(dragData, dropZone) {
    const sourceSectionIndex = dragData.sectionIndex;
    const sourceStepIndex = dragData.stepIndex;
    const targetSectionIndex = dropZone.dataset.sectionIndex ? parseInt(dropZone.dataset.sectionIndex) : 0;
    const insertPosition = dropZone.dataset.insertPosition;
    
    // Get current flow state
    const currentFlow = flowStore.getState();
    
    // Create updated workflow
    const updatedWorkflow = [...currentFlow.workflow];
    
    // Clone source section
    updatedWorkflow[sourceSectionIndex] = {
        ...updatedWorkflow[sourceSectionIndex],
        steps: [...updatedWorkflow[sourceSectionIndex].steps]
    };
    
    // Remove step from source
    const movedStep = updatedWorkflow[sourceSectionIndex].steps.splice(sourceStepIndex, 1)[0];
    
    // Insert at target
    if (insertPosition === 'start') {
        // Create new section at the beginning
        const newSection = {
            section: '新區段',
            description: '',
            steps: [movedStep]
        };
        updatedWorkflow.unshift(newSection);
        flowStore.updateWorkflow(updatedWorkflow);
    } else if (insertPosition === 'end') {
        // Create new section at the end when dropping to "將新區段拖放到這裡"
        const newSection = {
            section: '新區段',
            description: '',
            steps: [movedStep]
        };
        updatedWorkflow.push(newSection);
        flowStore.updateWorkflow(updatedWorkflow);
    } else {
        // Clone target section if different from source
        if (targetSectionIndex !== sourceSectionIndex) {
            updatedWorkflow[targetSectionIndex] = {
                ...updatedWorkflow[targetSectionIndex],
                steps: [...updatedWorkflow[targetSectionIndex].steps]
            };
        }
        // Insert into existing section at specific position
        const position = parseInt(insertPosition);
        updatedWorkflow[targetSectionIndex].steps.splice(position, 0, movedStep);
        flowStore.updateWorkflow(updatedWorkflow);
    }
    
    showNotification('✅ 步驟已移動到新區段', 'success');
}

/**
 * Move Step within Editor
 */
function moveStepInEditor(sectionIndex, stepIndex, direction) {
    const currentFlow = flowStore.getState();
    const section = currentFlow.workflow[sectionIndex];
    const newIndex = stepIndex + direction;
    
    if (newIndex >= 0 && newIndex < section.steps.length) {
        // Move step using flowStore method
        flowStore.moveStep(sectionIndex, stepIndex, newIndex);
        
        // Re-render
        renderVisualEditor();
        renderWorkflow();
        updateYamlEditor();
        
        showNotification('✅ 步驟順序已調整', 'success');
    }
}

/**
 * Select Step in Editor
 */
function selectStepInEditor(sectionIndex, stepIndex) {
    // Remove previous selection - only for workflow steps in left panel
    document.querySelectorAll('.workflow-step.selected').forEach(el => {
        el.classList.remove('selected');
    });
    
    // Don't add selection to visual editor step - only drag and drop needed
    
    // Add selection to workflow step in left panel
    const workflowStep = document.querySelector(`.workflow-step[data-section-index="${sectionIndex}"][data-step-index="${stepIndex}"]`);
    if (workflowStep) {
        workflowStep.classList.add('selected');
    }
    
    // Show step details in editor
    const state = flowStore.getState();
    const step = state.workflow[sectionIndex].steps[stepIndex];
    // Removed unnecessary notification - user already knows they selected a step
}

/**
 * Generate Step ID
 */
function generateStepId(functionName) {
    if (!functionName) return 'step_' + Date.now();
    
    // Extract meaningful part from function name
    const parts = functionName.split('.');
    const lastPart = parts[parts.length - 1];
    
    // Convert to snake_case
    const snakeCase = lastPart.replace(/([a-z])([A-Z])/g, '$1_$2').toLowerCase();
    
    return snakeCase + '_' + Math.floor(Math.random() * 1000);
}

/**
 * Auto-generate default value based on parameter name
 */
function generateDefaultForParam(paramName) {
    // Common parameter patterns and their default values
    const patterns = {
        // IDs
        'id$|_id$': (name) => {
            if (name.includes('agv')) return 'agv01';
            if (name.includes('task')) return 'task_001';
            if (name.includes('rack')) return 'rack01';
            if (name.includes('station')) return 'station01';
            if (name.includes('item')) return 'item_001';
            return name + '_001';
        },
        // Types
        '^type$|_type$': () => 'default',
        // Status
        'status$|state$': () => 'active',
        // Names
        'name$|_name$': (name) => 'example_' + name,
        // Counts/Numbers
        'count$|number$|num$|quantity$': () => 1,
        'seconds$|timeout$|duration$|delay$': () => 5,
        'retries$|max_retries$': () => 3,
        'priority$': () => 100,
        // Booleans
        '^is_|^has_|^can_|^should_|enabled$|disabled$': () => true,
        // Lists/Arrays
        'list$|items$|array$': () => [],
        '^rooms$': () => ['room01', 'room02'],
        'actions$': () => ['action1', 'action2'],
        // Objects/Params
        'params$|options$|config$|settings$|metrics$': () => ({}),
        // Locations
        'location$|position$|target$|destination$|^from$|^to$': () => 'station01',
        // Messages
        'message$|text$|description$': () => 'Default message',
        // Levels
        'level$': () => 'info',
        // Expressions/Conditions
        'expression$|condition$': () => '${result} == true',
        // Variables
        'variable$|store$': () => '${result}',
        // Values
        'value$': () => '${value}'
    };
    
    // Check each pattern
    for (const [pattern, generator] of Object.entries(patterns)) {
        const regex = new RegExp(pattern, 'i');
        if (regex.test(paramName)) {
            return generator(paramName);
        }
    }
    
    // Default fallback based on common patterns
    if (paramName.startsWith('$')) {
        return paramName; // Keep variable references as is
    }
    
    // Generic default
    return 'default_' + paramName;
}

/**
 * Get default parameters for a function
 */
function getFunctionDefaults(functionName) {
    // Search through all categories for the function
    for (const category in availableFunctions) {
        const funcs = availableFunctions[category];
        const func = funcs.find(f => f.name === functionName);
        if (func) {
            // If function has explicit defaults, use them
            if (func.defaults && Object.keys(func.defaults).length > 0) {
                // Return a deep copy of defaults
                return JSON.parse(JSON.stringify(func.defaults));
            }
            
            // Otherwise, auto-generate defaults based on parameter names
            if (func.params && func.params.length > 0) {
                const autoDefaults = {};
                for (const param of func.params) {
                    autoDefaults[param] = generateDefaultForParam(param);
                }
                return autoDefaults;
            }
        }
    }
    return {};
}

/**
 * Get function definition
 */
function getFunctionDefinition(functionName) {
    // Search through all categories for the function
    for (const category in availableFunctions) {
        const funcs = availableFunctions[category];
        const func = funcs.find(f => f.name === functionName);
        if (func) {
            return func;
        }
    }
    return null;
}

/**
 * Show parameter hints in the edit modal
 */
function showParameterHints(functionName) {
    const func = getFunctionDefinition(functionName);
    const paramsField = document.getElementById('new-step-params').closest('.field');
    
    // Remove existing hints
    const existingHint = paramsField.querySelector('.parameter-hints');
    if (existingHint) {
        existingHint.remove();
    }
    
    if (func && func.params && func.params.length > 0) {
        // Create hint container
        const hintContainer = document.createElement('div');
        hintContainer.className = 'parameter-hints notification is-info is-light mt-2';
        
        // Create hints content
        let hintsHTML = '<div class="content">';
        hintsHTML += '<strong>必要參數：</strong>';
        hintsHTML += '<ul>';
        func.params.forEach(param => {
            const defaultValue = func.defaults && func.defaults[param] !== undefined ? 
                JSON.stringify(func.defaults[param]) : '(無預設值)';
            hintsHTML += `<li><code>${param}</code> - 預設: ${defaultValue}</li>`;
        });
        hintsHTML += '</ul>';
        
        // Add reset button
        hintsHTML += '<button class="button is-small is-warning" onclick="resetToDefaultParams(\'' + functionName + '\')">';
        hintsHTML += '<span class="icon"><i class="fas fa-undo"></i></span>';
        hintsHTML += '<span>重置為預設值</span>';
        hintsHTML += '</button>';
        hintsHTML += '</div>';
        
        hintContainer.innerHTML = hintsHTML;
        
        // Insert after the params textarea
        paramsField.appendChild(hintContainer);
    }
}

/**
 * Reset parameters to default values
 */
function resetToDefaultParams(functionName) {
    const defaults = getFunctionDefaults(functionName);
    const paramsTextarea = document.getElementById('new-step-params');
    paramsTextarea.value = JSON.stringify(defaults, null, 2);
    
    // Show notification
    showNotification('參數已重置為預設值', 'success');
}

/**
 * Show parameter hints with reset button in edit modal
 */
function showParameterHintsWithReset(functionName) {
    if (!functionName) return;
    
    const functionDef = getFunctionDefinition(functionName);
    const defaults = getFunctionDefaults(functionName);
    
    // Remove any existing hint
    const existingHint = document.querySelector('#add-step-modal .parameter-hint-container');
    if (existingHint) {
        existingHint.remove();
    }
    
    // Show parameter hints with reset button
    if (functionDef) {
        const paramsField = document.querySelector('#add-step-modal .field:has(#new-step-params)');
        
        if (paramsField) {
            const hintContainer = document.createElement('div');
            hintContainer.className = 'parameter-hint-container';
            hintContainer.style.marginTop = '10px';
            
            // Create hint notification
            const hintDiv = document.createElement('div');
            hintDiv.className = 'notification is-info is-light';
            
            let hintContent = '<div style="display: flex; justify-content: space-between; align-items: start;">';
            hintContent += '<div>';
            
            if (functionDef.params && functionDef.params.length > 0) {
                hintContent += `<p><strong>必要參數:</strong> ${functionDef.params.join(', ')}</p>`;
            }
            
            if (functionDef.description) {
                hintContent += `<p><small>${functionDef.description}</small></p>`;
            }
            
            if (defaults) {
                hintContent += `<p><small><strong>預設值:</strong> <code>${JSON.stringify(defaults)}</code></small></p>`;
            }
            
            hintContent += '</div>';
            
            // Add reset button if defaults exist
            if (defaults) {
                hintContent += `
                    <button class="button is-small is-warning" onclick="resetEditStepParams('${functionName}')">
                        <span class="icon"><i class="fas fa-undo"></i></span>
                        <span>重置為預設值</span>
                    </button>
                `;
            }
            
            hintContent += '</div>';
            hintDiv.innerHTML = hintContent;
            
            hintContainer.appendChild(hintDiv);
            paramsField.appendChild(hintContainer);
        }
    }
}

/**
 * Reset parameters in edit modal to default values
 */
function resetEditStepParams(functionName) {
    const defaults = getFunctionDefaults(functionName);
    if (defaults) {
        const paramsTextarea = document.getElementById('new-step-params');
        if (paramsTextarea) {
            paramsTextarea.value = JSON.stringify(defaults, null, 2);
            showNotification('參數已重置為預設值', 'success');
        }
    }
}

/**
 * Handle function selection change in add step modal
 */
function onFunctionSelectChange(functionName) {
    if (!functionName) {
        // Clear parameters if no function selected
        const paramsTextarea = document.getElementById('new-step-params');
        if (paramsTextarea) {
            paramsTextarea.value = '{}';
        }
        // Remove any existing hint
        const existingHint = document.querySelector('#add-step-modal .parameter-hint');
        if (existingHint) {
            existingHint.remove();
        }
        return;
    }
    
    // Get function defaults
    const defaults = getFunctionDefaults(functionName);
    const functionDef = getFunctionDefinition(functionName);
    
    // Update parameters textarea with defaults
    const paramsTextarea = document.getElementById('new-step-params');
    if (paramsTextarea && defaults) {
        paramsTextarea.value = JSON.stringify(defaults, null, 2);
    } else if (paramsTextarea) {
        paramsTextarea.value = '{}';
    }
    
    // Remove any existing hint
    const existingHint = document.querySelector('#add-step-modal .parameter-hint');
    if (existingHint) {
        existingHint.remove();
    }
    
    // Show parameter hints
    if (functionDef && functionDef.params && functionDef.params.length > 0) {
        // Create hint element
        const paramsField = document.querySelector('#add-step-modal .field:has(#new-step-params)');
        
        if (paramsField) {
            const hintDiv = document.createElement('div');
            hintDiv.className = 'notification is-info is-light parameter-hint';
            hintDiv.style.marginTop = '10px';
            
            const hintContent = `
                <p><strong>必要參數:</strong> ${functionDef.params.join(', ')}</p>
                ${functionDef.description ? `<p><small>${functionDef.description}</small></p>` : ''}
            `;
            hintDiv.innerHTML = hintContent;
            
            // Insert after the params field
            paramsField.appendChild(hintDiv);
        }
    }
}

/**
 * Switch left panel tabs
 */
function switchLeftTab(tabName) {
    // Remove active class from all left panel tabs
    document.querySelectorAll('.panel-tabs a[data-tab]').forEach(tab => {
        tab.classList.remove('is-active');
    });
    
    // Remove active class from all left tab contents
    document.querySelectorAll('.left-tab-content').forEach(content => {
        content.classList.remove('is-active');
    });
    
    // Add active class to selected tab
    const activeTab = document.querySelector(`.panel-tabs a[data-tab="${tabName}"]`);
    if (activeTab) {
        activeTab.classList.add('is-active');
    }
    
    // Add active class to selected content
    const activeContent = document.getElementById(`${tabName}-tab`);
    if (activeContent) {
        activeContent.classList.add('is-active');
    }
    
    // Show notification
    const tabNames = {
        'settings': '設定',
        'workflow': '工作流程'
    };
    
    showNotification(`已切換到 ${tabNames[tabName] || tabName} 頁面`, 'info');
}

/**
 * Initialize Visual Editor
 */
function initializeVisualEditor() {
    renderVisualEditor();
    
    // Make function items draggable
    document.querySelectorAll('.function-item').forEach(item => {
        item.draggable = true;
        item.addEventListener('dragstart', (e) => {
            const functionName = item.querySelector('.function-name').textContent;
            const dragData = {
                type: 'function',
                functionName: functionName,
                description: item.querySelector('.function-description').textContent
            };
            
            e.dataTransfer.setData('text/plain', JSON.stringify(dragData));
            e.dataTransfer.effectAllowed = 'copy';
            
            item.classList.add('dragging');
        });
        
        item.addEventListener('dragend', () => {
            item.classList.remove('dragging');
        });
    });
}

/**
 * Insert function as step
 */
/**
 * Show add function modal with pre-filled values
 */
function showAddFunctionModal(func) {
    // Clear the current step index to indicate this is a new step
    currentStepIndex = null;
    
    // Show section selector for new steps
    const sectionField = document.getElementById('new-step-section').closest('.field');
    if (sectionField) {
        sectionField.style.display = 'block';
    }
    
    // Populate section dropdown
    const sectionSelect = document.getElementById('new-step-section');
    sectionSelect.innerHTML = '<option value="">選擇區段...</option>';
    sectionSelect.innerHTML += '<option value="new">📝 新增區段</option>';
    
    // Add existing sections
    const currentFlow = flowStore.getState();
    if (currentFlow.workflow && currentFlow.workflow.length > 0) {
        currentFlow.workflow.forEach((section, index) => {
            const sectionName = section.section || `區段 ${index + 1}`;
            const option = document.createElement('option');
            option.value = index;
            option.textContent = sectionName;
            if (index === currentSectionIndex) {
                option.selected = true;  // Select current section by default
            }
            sectionSelect.appendChild(option);
        });
    }
    
    // If no section is selected and there are sections, select the first one
    if ((currentSectionIndex === null || currentSectionIndex === undefined) && currentFlow.workflow.length > 0) {
        sectionSelect.value = "0";
    }
    
    // Prepare step data with function defaults
    document.getElementById('new-step-id').value = func.name.replace(/\./g, '_') + '_' + Date.now();
    document.getElementById('new-step-exec').value = func.name;
    
    // Prepare default params - use function defaults if available
    const defaultParams = {};
    if (func.params && func.params.length > 0) {
        // First try to get defaults from the function
        const funcDefaults = func.defaults || getFunctionDefaults(func.name);
        
        func.params.forEach(param => {
            // Use default value if available, otherwise empty string
            defaultParams[param] = funcDefaults && funcDefaults[param] !== undefined ? 
                funcDefaults[param] : '';
        });
    }
    document.getElementById('new-step-params').value = JSON.stringify(defaultParams, null, 2);
    
    // Clear store field
    document.getElementById('new-step-store').value = '';
    
    // Reset skip_if
    document.getElementById('new-step-skip-if-enabled').checked = false;
    document.getElementById('new-step-skip-if').style.display = 'none';
    document.getElementById('new-step-skip-if').value = '';
    
    // Set modal title for new step
    const modalTitle = document.querySelector('#add-step-modal .modal-card-title');
    if (modalTitle) {
        modalTitle.textContent = `新增步驟 - ${func.name}`;
    }
    
    // Change button text to "新增" for new step
    const submitButton = document.querySelector('#add-step-modal .button.is-success');
    if (submitButton) {
        submitButton.textContent = '新增';
        submitButton.onclick = () => confirmAddStep();  // Make sure it calls confirmAddStep
    }
    
    // Show the modal with fade-in
    const modal = document.getElementById('add-step-modal');
    modal.classList.add('is-active');
}

/**
 * Edit Step (restore missing popup functionality)
 */
function editStep(sectionIndex, stepIndex) {
    const currentFlow = flowStore.getState();
    // Store current step indices for editing
    currentSectionIndex = sectionIndex;
    currentStepIndex = stepIndex;
    
    console.log('editStep - setting currentSectionIndex:', sectionIndex, 'currentStepIndex:', stepIndex);
    
    const step = currentFlow.workflow[sectionIndex].steps[stepIndex];
    
    // Hide section selector when editing (step already belongs to a section)
    const sectionField = document.getElementById('new-step-section').closest('.field');
    if (sectionField) {
        sectionField.style.display = 'none';
    }
    
    // Populate step editing modal
    document.getElementById('new-step-id').value = step.id || '';
    document.getElementById('new-step-exec').value = step.exec || '';
    document.getElementById('new-step-params').value = JSON.stringify(step.params || {}, null, 2);
    document.getElementById('new-step-store').value = step.store || '';
    
    // Show parameter hints and reset button for this function
    showParameterHintsWithReset(step.exec);
    
    // Handle skip_if condition
    const skipIfCheckbox = document.getElementById('new-step-skip-if-enabled');
    const skipIfInput = document.getElementById('new-step-skip-if');
    
    if (step.skip_if) {
        skipIfCheckbox.checked = true;
        skipIfInput.style.display = 'block';
        skipIfInput.value = step.skip_if;
    } else {
        skipIfCheckbox.checked = false;
        skipIfInput.style.display = 'none';
        skipIfInput.value = '';
    }
    
    // Change modal title to indicate editing
    document.querySelector('#add-step-modal .modal-card-title').textContent = '編輯步驟';
    
    // Change button text to "更新" for editing
    const submitButton = document.querySelector('#add-step-modal .button.is-success');
    if (submitButton) {
        submitButton.textContent = '更新';
        submitButton.onclick = () => confirmAddStep();  // Still use confirmAddStep but it will update
    }
    
    // Show modal
    document.getElementById('add-step-modal').classList.add('is-active');
    
    // Removed unnecessary notification - modal title already shows "編輯步驟"
}

/**
 * Confirm Delete Step from Visual Editor
 */
function confirmDeleteStepFromVisual(sectionIndex, stepIndex) {
    const currentFlow = flowStore.getState();
    const step = currentFlow.workflow[sectionIndex].steps[stepIndex];
    const stepName = step.id || '未命名步驟';
    const functionName = step.exec || '無';
    
    let message = `<strong>確定要刪除步驟「${stepName}」嗎？</strong>`;
    message += `<br><br>執行函數：<code>${functionName}</code>`;
    if (step.store_to) {
        message += `<br>儲存變數：<code>${step.store_to}</code>`;
    }
    
    showDeleteConfirmation(message, function() {
        deleteStepFromVisual(sectionIndex, stepIndex);
    });
}

/**
 * Delete Step from Visual Editor
 */
function deleteStepFromVisual(sectionIndex, stepIndex) {
    const currentFlow = flowStore.getState();
    const step = currentFlow.workflow[sectionIndex].steps[stepIndex];
    const stepName = step.id || 'untitled_step';
    
    // Remove step from workflow using flowStore method
    flowStore.deleteStep(sectionIndex, stepIndex);
    
    // Update all views
    renderWorkflow();
    renderVisualEditor();
    updateYamlEditor();
    updateVariablesDisplay();
    
    showNotification(`🗑️ 已刪除步驟: ${stepName}`, 'success');
}

// Toggle skip-if input visibility
document.addEventListener('DOMContentLoaded', function() {
    const skipIfCheckbox = document.getElementById('new-step-skip-if-enabled');
    const skipIfInput = document.getElementById('new-step-skip-if');
    
    if (skipIfCheckbox && skipIfInput) {
        skipIfCheckbox.addEventListener('change', function() {
            skipIfInput.style.display = this.checked ? 'block' : 'none';
        });
    }
    
    // Initialize flow ID validation
    initializeFlowIdValidation();
    
    // Add keyboard shortcuts
    document.addEventListener('keydown', function(e) {
        // Ctrl+S or Cmd+S to save (now triggers auto-save immediately)
        if ((e.ctrlKey || e.metaKey) && e.key === 's') {
            e.preventDefault();
            performAutoSave();
        }
        // Ctrl+Enter to validate
        if ((e.ctrlKey || e.metaKey) && e.key === 'Enter') {
            e.preventDefault();
            performBackgroundValidation(true); // Force validation
        }
    });
});

/**
 * Initialize flow ID validation
 */
function initializeFlowIdValidation() {
    const flowIdInput = document.getElementById('flow-id');
    if (flowIdInput) {
        // Remove any existing blur listener first
        flowIdInput.removeEventListener('blur', validateFlowId);
        
        // Add new blur listener
        flowIdInput.addEventListener('blur', validateFlowId);
        
        // Also validate on input with debounce
        let validateTimeout;
        flowIdInput.addEventListener('input', function() {
            clearTimeout(validateTimeout);
            validateTimeout = setTimeout(validateFlowId, 500);
        });
    }
}

/**
 * Validate flow ID
 */
function validateFlowId() {
    const flowIdInput = document.getElementById('flow-id');
    const flowId = flowIdInput.value.trim();
    
    // Clear previous validation states
    flowIdInput.classList.remove('is-danger', 'is-success');
    
    if (!flowId) {
        flowIdInput.classList.add('is-danger');
        showNotification('流程 ID 不能為空', 'warning');
        return false;
    }
    
    // Check format (alphanumeric with underscores and hyphens)
    const validFormat = /^[a-zA-Z0-9_-]+$/.test(flowId);
    if (!validFormat) {
        flowIdInput.classList.add('is-danger');
        showNotification('流程 ID 只能包含字母、數字、底線和連字號', 'warning');
        return false;
    }
    
    flowIdInput.classList.add('is-success');
    return true;
}

/**
 * Initialize validation indicator
 */
function initializeValidationIndicator() {
    const indicator = document.getElementById('validation-indicator');
    if (indicator) {
        indicator.addEventListener('click', function() {
            showValidationDetails();
        });
    }
    updateValidationIndicator('unknown');
}

// Store validation results globally for display
let lastValidationResults = {
    errors: [],
    warnings: [],
    status: 'unknown'
};

/**
 * Show validation details in a modal or notification
 */
function showValidationDetails() {
    // Force validation and show results
    performBackgroundValidationWithDetails();
}

/**
 * Perform validation and show detailed results
 */
async function performBackgroundValidationWithDetails() {
    await performBackgroundValidation(true);
    
    // Show detailed results
    if (lastValidationResults.errors.length > 0 || lastValidationResults.warnings.length > 0) {
        let message = '';
        
        if (lastValidationResults.errors.length > 0) {
            message += '<div class="has-text-danger"><strong>錯誤:</strong><ul>';
            lastValidationResults.errors.forEach(error => {
                message += `<li>${error}</li>`;
            });
            message += '</ul></div>';
        }
        
        if (lastValidationResults.warnings.length > 0) {
            message += '<div class="has-text-warning"><strong>警告:</strong><ul>';
            lastValidationResults.warnings.forEach(warning => {
                message += `<li>${warning}</li>`;
            });
            message += '</ul></div>';
        }
        
        // Create a detailed notification
        const notification = document.createElement('div');
        notification.className = 'notification is-light';
        notification.style.cssText = 'position: fixed; top: 80px; right: 20px; z-index: 9999; max-width: 500px; max-height: 400px; overflow-y: auto;';
        
        if (lastValidationResults.errors.length > 0) {
            notification.classList.add('is-danger');
        } else {
            notification.classList.add('is-warning');
        }
        
        notification.innerHTML = `
            <button class="delete" onclick="this.parentElement.remove()"></button>
            <h5 class="title is-5">驗證結果詳情</h5>
            ${message}
        `;
        
        // Remove any existing validation details
        document.querySelectorAll('.validation-details').forEach(el => el.remove());
        notification.classList.add('validation-details');
        
        document.body.appendChild(notification);
        
        // Auto-remove after 10 seconds
        setTimeout(() => {
            notification.remove();
        }, 10000);
    } else {
        showNotification('✅ 流程驗證通過，沒有錯誤或警告', 'success');
    }
}

/**
 * Update validation indicator status
 */
function updateValidationIndicator(status, message) {
    const indicator = document.getElementById('validation-indicator');
    const icon = document.getElementById('validation-icon');
    
    if (!indicator || !icon) return;
    
    // Remove all status classes
    indicator.classList.remove('validating', 'valid', 'invalid', 'warning');
    
    // Update icon and status
    switch(status) {
        case 'validating':
            indicator.classList.add('validating');
            icon.className = 'fas fa-circle-notch fa-spin';
            indicator.title = '驗證中...';
            break;
        case 'valid':
            indicator.classList.add('valid');
            icon.className = 'fas fa-check-circle';
            indicator.title = message || '驗證通過';
            break;
        case 'invalid':
            indicator.classList.add('invalid');
            icon.className = 'fas fa-exclamation-circle';
            indicator.title = message || '驗證失敗，點擊查看詳情';
            break;
        case 'warning':
            indicator.classList.add('warning');
            icon.className = 'fas fa-exclamation-triangle';
            indicator.title = message || '有警告，點擊查看詳情';
            break;
        default:
            icon.className = 'fas fa-question-circle';
            indicator.title = '尚未驗證，點擊進行驗證';
    }
    
    validationStatus = status;
}

/**
 * Track changes for auto-save
 */
function trackChanges() {
    const currentData = JSON.stringify(currentFlow);
    if (currentData !== lastSavedData) {
        isModified = true;
        
        // Clear existing timers
        if (autoSaveTimer) clearTimeout(autoSaveTimer);
        if (validationTimer) clearTimeout(validationTimer);
        
        // Schedule auto-save (after 2 seconds of no changes)
        autoSaveTimer = setTimeout(performAutoSave, 2000);
        
        // Schedule validation (after 1 second of no changes)
        validationTimer = setTimeout(performBackgroundValidation, 1000);
    }
}

/**
 * Perform auto-save
 */
async function performAutoSave() {
    const currentFlow = flowStore.getState();
    if (!isModified) return;
    
    try {
        // For visual editor, changes are already in currentFlow
        // For YAML editor, we need to parse YAML first
        const activeTab = document.querySelector('.editor-tabs a.is-active').dataset.tab;
        if (activeTab === 'yaml' && yamlEditor) {
            try {
                currentFlow = jsyaml.load(yamlEditor.getValue());
                
                // Ensure workflow field exists (Linear Flow v2 uses 'workflow')
                if (!currentFlow.workflow) {
                    currentFlow.workflow = [];
                }
            } catch (e) {
                console.error('YAML parsing error during auto-save:', e);
                return;
            }
        }
        
        // Save to server (you can implement actual save here)
        console.log('Auto-saving flow...', currentFlow);
        
        // Update last saved state
        lastSavedData = JSON.stringify(currentFlow);
        isModified = false;
        
        // Show subtle notification
        showNotification('已自動儲存', 'info', 1000);
        
        // Trigger validation after save
        performBackgroundValidation();
        
    } catch (error) {
        console.error('Auto-save error:', error);
    }
}

/**
 * Perform background validation
 */
async function performBackgroundValidation(force = false) {
    // Don't validate if not modified and not forced
    if (!force && !isModified && validationStatus === 'valid') {
        return;
    }
    
    updateValidationIndicator('validating');
    
    try {
        // Simulate validation (replace with actual validation logic)
        await new Promise(resolve => setTimeout(resolve, 500));
        
        const errors = [];
        const warnings = [];
        
        // Validate YAML structure
        const activeTab = document.querySelector('.editor-tabs a.is-active')?.dataset.tab;
        if (activeTab === 'yaml' && yamlEditor) {
            try {
                // Test YAML parsing
                const testParse = jsyaml.load(yamlEditor.getValue());
                if (!testParse) {
                    errors.push('YAML 內容為空');
                }
            } catch (e) {
                errors.push(`YAML 格式錯誤: ${e.message}`);
                updateValidationIndicator('invalid', `YAML 格式錯誤: ${e.message}`);
                lastValidationResults.errors = [e.message];
                lastValidationResults.warnings = [];
                lastValidationResults.status = 'invalid';
                return;
            }
        }
        
        // Use flowStore's validation system for consistency
        const fullState = flowStore.getFullState();
        const storeValidation = fullState.computed?.validation || { errors: [], warnings: [] };
        
        // Merge flowStore validation results with YAML validation
        errors.push(...storeValidation.errors);
        warnings.push(...storeValidation.warnings);
        
        // Store results for display
        lastValidationResults.errors = errors;
        lastValidationResults.warnings = warnings;
        
        // Update indicator based on results
        if (errors.length > 0) {
            lastValidationResults.status = 'invalid';
            updateValidationIndicator('invalid', `${errors.length} 個錯誤: ${errors[0]}`);
        } else if (warnings.length > 0) {
            lastValidationResults.status = 'warning';
            updateValidationIndicator('warning', `${warnings.length} 個警告: ${warnings[0]}`);
        } else {
            lastValidationResults.status = 'valid';
            updateValidationIndicator('valid', '流程驗證通過');
        }
        
    } catch (error) {
        console.error('Validation error:', error);
        updateValidationIndicator('invalid', '驗證過程發生錯誤');
    }
}

// Validation now uses flowStore's validation system exclusively

/**
 * Save flow to server
 */
async function saveFlowToServer() {
    const currentFlow = flowStore.getState();
    try {
        // Update sync status to syncing
        updateSyncStatus('syncing');
        
        // Build flow data
        const flowData = buildFlowData();
        
        // Validate flow ID
        const flowId = document.getElementById('flow-id').value;
        if (!flowId || flowId.trim() === '') {
            showNotification('請輸入流程 ID', 'error');
            updateSyncStatus('error');
            return;
        }
        
        // Check if flow is new or existing
        const isNewFlow = !window.existingFlowId || window.existingFlowId !== flowId;
        
        // Prepare save data in the format expected by server
        const fullFlowData = {
            meta: {
                system: 'linear_flow_v2',
                version: '2.0.0',
                description: document.getElementById('flow-description').value || '',
                author: 'Linear Flow Designer',
                created_at: new Date().toISOString()
            },
            flow: {
                id: flowId,
                name: document.getElementById('flow-name').value || 'Unnamed Flow',
                work_id: document.getElementById('work-id').value || '',
                priority: parseInt(document.getElementById('flow-priority').value) || 100,
                enabled: document.getElementById('flow-enabled').checked
            },
            workflow: flowData.workflow || [],
            variables: {},
            config: {}
        };
        
        const saveData = {
            flow_id: flowId,
            flow_data: fullFlowData
        };
        
        // Save to server - Use the save endpoint
        const endpoint = '/linear-flow/api/flows/save';
        const method = 'POST';
        
        const response = await fetch(endpoint, {
            method: method,
            headers: {
                'Content-Type': 'application/json'
            },
            body: JSON.stringify(saveData)
        });
        
        if (response.ok) {
            const result = await response.json();
            
            if (result.success) {
                window.existingFlowId = flowId;
                showNotification('流程已同步到伺服器', 'success');
                
                // Update auto-save state
                lastSavedContent = JSON.stringify(flowData);
                updateSyncStatus('synced');
                
                // Show warnings if any
                if (result.warnings && result.warnings.length > 0) {
                    result.warnings.forEach(warning => {
                        showNotification(`警告: ${warning}`, 'warning');
                    });
                }
            } else {
                // Handle validation errors
                updateSyncStatus('error');
                if (result.errors && result.errors.length > 0) {
                    result.errors.forEach(error => {
                        showNotification(`驗證錯誤: ${error}`, 'error');
                    });
                } else {
                    showNotification('同步失敗: ' + (result.error || '未知錯誤'), 'error');
                }
            }
        } else {
            const error = await response.text();
            showNotification('同步失敗: ' + error, 'error');
            updateSyncStatus('error');
        }
    } catch (error) {
        console.error('Save error:', error);
        showNotification('同步失敗: ' + error.message, 'error');
        updateSyncStatus('error');
    }
}

/**
 * Update sync status indicator
 */
function updateSyncStatus(status = 'modified') {
    const syncButton = document.getElementById('sync-button');
    const syncIcon = document.getElementById('sync-icon');
    const syncText = document.getElementById('sync-text');
    
    if (!syncButton) return;
    
    // Remove all status classes
    syncButton.classList.remove('is-synced', 'is-syncing', 'is-modified', 'is-error');
    
    // Handle boolean for backward compatibility
    if (typeof status === 'boolean') {
        status = status ? 'synced' : 'modified';
    }
    
    switch (status) {
        case 'synced':
            syncButton.classList.add('is-synced');
            syncIcon.innerHTML = '<i class="fas fa-check"></i>';
            syncText.textContent = '已同步';
            break;
        case 'syncing':
            syncButton.classList.add('is-syncing');
            syncIcon.innerHTML = '<i class="fas fa-sync-alt"></i>';
            syncText.textContent = '同步中';
            break;
        case 'modified':
            syncButton.classList.add('is-modified');
            syncIcon.innerHTML = '<i class="fas fa-cloud-upload-alt"></i>';
            syncText.textContent = '同步';
            break;
        case 'error':
            syncButton.classList.add('is-error');
            syncIcon.innerHTML = '<i class="fas fa-exclamation-triangle"></i>';
            syncText.textContent = '錯誤';
            break;
    }
}

/**
 * Save flow settings
 */
function saveFlowSettings() {
    try {
        // Check if we're in YAML tab and need to parse YAML first
        const activeTabElement = document.querySelector('.editor-tabs a.is-active');
        const activeTab = activeTabElement ? activeTabElement.getAttribute('data-tab') : 'visual';
        
        if (activeTab === 'yaml' && yamlEditor) {
            // Parse YAML content first
            const yamlContent = yamlEditor.getValue();
            if (yamlContent.trim()) {
                const result = flowStore.updateFromYAML(yamlContent, 'yaml');
                if (!result.success) {
                    showAlert('YAML 格式錯誤: ' + result.error, 'error');
                    return;
                }
                
                // Update form fields from store
                const state = flowStore.getState();
                updateFormFieldsFromState(state);
            }
        } else {
            // Update store from form (when not in YAML tab)
            flowStore.updateFlow({
                id: document.getElementById('flow-id').value,
                name: document.getElementById('flow-name').value,
                work_id: document.getElementById('work-id').value,
                priority: parseInt(document.getElementById('flow-priority').value) || 100,
                enabled: document.getElementById('flow-enabled').checked
            });
            
            flowStore.updateMeta({
                description: document.getElementById('flow-description').value
            });
        }
        
        // The store automatically saves to localStorage
        const state = flowStore.getState();
        const settingsToSave = {
            flow_id: state.flow?.id || '',
            flow_name: state.flow?.name || '',
            description: state.meta?.description || '',
            work_id: state.flow?.work_id || '',
            priority: state.flow?.priority || 100,
            enabled: state.flow?.enabled !== false
        };
        localStorage.setItem('flowSettings', JSON.stringify(settingsToSave));
        
        // Update sync status to modified
        updateSyncStatus('modified');
        
        showNotification('已暫存到本地', 'info');
        
        // Trigger validation
        performBackgroundValidation(true);
    } catch (error) {
        console.error('Save settings error:', error);
        showNotification('儲存設定失敗: ' + error.message, 'error');
    }
}

/**
 * Clear local storage cache
 * Only clears the Linear Flow Designer's data
 */
function clearLocalCache() {
    try {
        // Show confirmation dialog
        if (!confirm('確定要清除本地快取嗎？\n\n這將清除編輯器中的暫存資料，但不會影響已儲存到伺服器的流程。')) {
            return;
        }
        
        // Clear only the Linear Flow Designer's localStorage
        localStorage.removeItem('linearFlowDesigner');
        
        // Also clear the flowStore (which uses the same key)
        if (flowStore && typeof flowStore.clear === 'function') {
            flowStore.clear();
        }
        
        // Reload the page to reset everything
        showNotification('本地快取已清除，頁面將重新載入...', 'success');
        
        // Reload after a short delay
        setTimeout(() => {
            // If we have a flow ID in the URL, preserve it
            const urlParams = new URLSearchParams(window.location.search);
            const flowId = urlParams.get('flow');
            
            if (flowId) {
                // Reload with the same flow ID
                window.location.href = `/linear-flow/designer?flow=${flowId}`;
            } else {
                // Just reload the page
                window.location.reload();
            }
        }, 1500);
        
    } catch (error) {
        console.error('Clear cache error:', error);
        showNotification('清除快取失敗: ' + error.message, 'error');
    }
}

/**
 * Export flow to YAML file
 */
function exportFlow() {
    try {
        // Get current flow data
        const flowData = buildFlowData();
        
        // Convert to YAML
        const yamlContent = jsyaml.dump(flowData, {
            indent: 2,
            lineWidth: -1,
            noRefs: true,
            sortKeys: false
        });
        
        // Create blob and download link
        const blob = new Blob([yamlContent], { type: 'text/yaml;charset=utf-8' });
        const url = URL.createObjectURL(blob);
        const link = document.createElement('a');
        
        // Set filename with timestamp
        const timestamp = new Date().toISOString().replace(/[:.]/g, '-').slice(0, -5);
        const flowId = document.getElementById('flow-id').value || 'flow';
        const filename = `${flowId}_${timestamp}.yaml`;
        
        link.href = url;
        link.download = filename;
        document.body.appendChild(link);
        link.click();
        document.body.removeChild(link);
        
        // Clean up
        URL.revokeObjectURL(url);
        
        showNotification('流程已成功導出', 'success');
    } catch (error) {
        console.error('Export error:', error);
        showNotification('導出失敗: ' + error.message, 'error');
    }
}

/**
 * Trigger file import dialog
 */
function importFlow() {
    const fileInput = document.getElementById('import-file-input');
    fileInput.click();
}

/**
 * Load flow data into the designer
 */
function loadFlowFromServer() {
    const currentFlow = flowStore.getState();
    try {
        // Initialize currentFlow if not exists
        if (!currentFlow) {
            currentFlow = {
                meta: {},
                flow: {},
                workflow: [],
                variables: {},
                config: {}
            };
        }
        
        // Clear existing workflow
        currentFlow.workflow = [];
        selectedSection = null;
        selectedStep = null;
        
        // Load workflow sections
        if (flowData.workflow && Array.isArray(flowData.workflow)) {
            currentFlow.workflow = flowData.workflow.map(section => {
                // Ensure each section has required properties
                return {
                    section: section.section || 'Unnamed Section',
                    condition: section.condition || '',
                    steps: (section.steps || []).map(step => ({
                        id: step.id || generateStepId(),
                        exec: step.exec || '',
                        params: step.params || {},
                        skip_if: step.skip_if || '',
                        skip_if_not: step.skip_if_not || '',
                        store: step.store || ''
                    }))
                };
            });
        }
        
        // Update metadata fields and currentFlow
        if (flowData.flow_id) {
            document.getElementById('flow-id').value = flowData.flow_id;
            currentFlow.flow.id = flowData.flow_id;
        }
        if (flowData.flow_name) {
            document.getElementById('flow-name').value = flowData.flow_name;
            currentFlow.flow.name = flowData.flow_name;
        }
        if (flowData.description) {
            document.getElementById('flow-description').value = flowData.description;
            currentFlow.meta.description = flowData.description;
        }
        if (flowData.work_id) {
            document.getElementById('work-id').value = flowData.work_id;
            currentFlow.flow.work_id = flowData.work_id;
        }
        if (flowData.priority !== undefined) {
            document.getElementById('flow-priority').value = flowData.priority;
            currentFlow.flow.priority = flowData.priority;
        }
        if (flowData.enabled !== undefined) {
            document.getElementById('flow-enabled').checked = flowData.enabled;
            currentFlow.flow.enabled = flowData.enabled;
        }
        
        // Refresh UI
        renderWorkflow();
        renderVisualEditor();
        performBackgroundValidation(true);
        
        return true;
    } catch (error) {
        console.error('Error loading flow data:', error);
        showNotification('載入流程資料失敗: ' + error.message, 'error');
        return false;
    }
}

/**
 * Handle file import
 */
function handleFileImport(event) {
    const currentFlow = flowStore.getState();
    const file = event.target.files[0];
    if (!file) return;
    
    // Check file extension
    const fileName = file.name.toLowerCase();
    if (!fileName.endsWith('.yaml') && !fileName.endsWith('.yml')) {
        showNotification('請選擇 YAML 檔案 (.yaml 或 .yml)', 'error');
        return;
    }
    
    const reader = new FileReader();
    reader.onload = function(e) {
        try {
            // Parse YAML content
            const yamlContent = e.target.result;
            const flowData = jsyaml.load(yamlContent);
            
            // Validate basic structure
            if (!flowData || typeof flowData !== 'object') {
                throw new Error('無效的 YAML 格式');
            }
            
            // Check if it's Linear Flow v2 format
            let processedData = {};
            
            // If the file has the full Linear Flow v2 structure
            if (flowData.meta && flowData.meta.system === 'linear_flow_v2') {
                // Extract flow information
                processedData.flow_id = flowData.flow?.id || '';
                processedData.flow_name = flowData.flow?.name || '';
                processedData.description = flowData.meta?.description || '';
                processedData.work_id = flowData.flow?.work_id || '';
                processedData.priority = flowData.flow?.priority || 100;
                processedData.enabled = flowData.flow?.enabled !== false;
                processedData.workflow = flowData.workflow || [];
            } else {
                // Try to process as simple workflow format
                processedData = flowData;
            }
            
            // Confirm before replacing current flow
            let importMessage = '<strong>導入將會取代當前的流程設計</strong>';
            
            // Check if current flow has content
            const currentFlow = flowStore.getState();
            if (currentFlow.workflow && currentFlow.workflow.length > 0) {
                const totalSteps = currentFlow.workflow.reduce((sum, section) => 
                    sum + (section.steps ? section.steps.length : 0), 0);
                importMessage += `<br><br>當前流程包含：`;
                importMessage += `<br>• <span class="tag is-info">${currentFlow.workflow.length} 個區段</span>`;
                importMessage += `<br>• <span class="tag is-info">${totalSteps} 個步驟</span>`;
                importMessage += `<br><br>這些內容將會被新的流程覆蓋。`;
            }
            
            showDeleteConfirmation(importMessage, function() {
                // Load the imported flow
                flowStore.loadFlow(processedData);
                renderWorkflow();
                
                showNotification('流程已成功導入', 'success');
                
                // Trigger validation
                validateFlow();
            });
        } catch (error) {
            console.error('Import error:', error);
            showNotification('導入失敗: ' + error.message, 'error');
        }
    };
    
    reader.onerror = function() {
        showNotification('檔案讀取失敗', 'error');
    };
    
    reader.readAsText(file);
    
    // Clear the input value to allow importing the same file again
    event.target.value = '';
}

/**
 * Clear cache and force reload (clear browser cache too)
 */
function clearCacheAndReload() {
    try {
        // Clear localStorage
        localStorage.removeItem('linearFlowDesigner');
        
        // Clear flowStore if available
        if (typeof flowStore !== 'undefined' && flowStore && typeof flowStore.clear === 'function') {
            flowStore.clear();
        }
        
        // Force reload with cache bypass
        // This tells the browser to reload the page and bypass cache
        window.location.reload(true);
        
    } catch (error) {
        console.error('Failed to clear cache and reload:', error);
        showNotification('❌ 清除快取失敗: ' + error.message, 'danger');
    }
}

// Export functions for external use
// These need to be immediately available for HTML onclick handlers
(function() {
    console.log('🚀 Exporting Linear Flow Designer functions to window object');
    
    // Safe export helper - only export if function exists
    function safeExport(name, func) {
        if (typeof func !== 'undefined') {
            window[name] = func;
            console.log(`  ✅ Exported: ${name}`);
            return true;
        } else {
            console.warn(`  ⚠️ Skipped: ${name} (not defined)`);
            return false;
        }
    }
    
    let exportCount = 0;
    let skipCount = 0;
    
    // Core functions
    if (safeExport('initializeDesigner', initializeDesigner)) exportCount++;
    if (safeExport('flowStore', flowStore)) exportCount++;
    
    // Tab switching
    if (safeExport('switchTab', switchTab)) exportCount++;
    if (safeExport('switchLeftTab', switchLeftTab)) exportCount++;
    
    // Section operations
    if (safeExport('addSection', addSection)) exportCount++;
    if (safeExport('confirmAddSection', confirmAddSection)) exportCount++;
    // editSection doesn't exist
    if (safeExport('deleteSection', deleteSection)) exportCount++;
    // confirmDeleteSection doesn't exist
    if (safeExport('selectSection', selectSection)) exportCount++;
    if (safeExport('moveSection', moveSection)) exportCount++;
    
    // Step operations
    if (safeExport('addStepToSection', addStepToSection)) exportCount++;
    if (safeExport('confirmAddStep', confirmAddStep)) exportCount++;
    if (safeExport('editStep', editStep)) exportCount++;
    if (safeExport('deleteStep', deleteStep)) exportCount++;
    if (safeExport('confirmDeleteStep', confirmDeleteStep)) exportCount++;
    if (safeExport('selectStep', selectStep)) exportCount++;
    if (safeExport('moveStep', moveStep)) exportCount++;
    // moveUp and moveDown don't exist
    
    // Flow operations
    if (safeExport('saveFlow', saveFlow)) exportCount++;
    // loadFlow doesn't exist
    if (safeExport('validateFlow', validateFlow)) exportCount++;
    if (safeExport('exportFlow', exportFlow)) exportCount++;
    if (safeExport('importFlow', importFlow)) exportCount++;
    if (safeExport('createNewFlow', createNewFlow)) exportCount++;
    if (safeExport('saveFlowSettings', saveFlowSettings)) exportCount++;
    if (safeExport('clearLocalCache', clearLocalCache)) exportCount++;
    if (safeExport('clearCacheAndReload', clearCacheAndReload)) exportCount++;
    if (safeExport('saveFlowToServer', saveFlowToServer)) exportCount++;
    if (safeExport('testFlow', testFlow)) exportCount++;
    
    // UI operations
    // showFunctionLibrary, selectFunction, closeFunctionModal don't exist
    // showSettings, saveSettings, closeSettingsModal don't exist
    // showYamlEditor, showVisualEditor don't exist
    if (safeExport('showValidationDetails', showValidationDetails)) exportCount++;
    if (safeExport('handleFileImport', handleFileImport)) exportCount++;
    if (safeExport('switchFunctionCategory', switchFunctionCategory)) exportCount++;
    if (safeExport('closeModal', closeModal)) exportCount++;
    if (safeExport('updateVariablesDisplay', updateVariablesDisplay)) exportCount++;
    
    // Visual editor functions
    if (safeExport('confirmDeleteStepFromVisual', confirmDeleteStepFromVisual)) exportCount++;
    if (safeExport('moveStepInEditor', moveStepInEditor)) exportCount++;
    
    // Drag and drop functions
    if (safeExport('handleFunctionDragStart', handleFunctionDragStart)) exportCount++;
    if (safeExport('handleFunctionDragEnd', handleFunctionDragEnd)) exportCount++;
    if (safeExport('insertFunctionAsStep', insertFunctionAsStep)) exportCount++;
    if (safeExport('generateStepId', generateStepId)) exportCount++;
    
    // Test modal functions
    if (safeExport('updateTestModeDescription', updateTestModeDescription)) exportCount++;
    if (safeExport('runTest', runTest)) exportCount++;
    if (safeExport('switchTestTab', switchTestTab)) exportCount++;
    
    // Modal functions
    if (safeExport('closeDeleteConfirmation', closeDeleteConfirmation)) exportCount++;
    if (safeExport('showDeleteConfirmation', showDeleteConfirmation)) exportCount++;
    if (safeExport('closeAlertModal', closeAlertModal)) exportCount++;
    if (safeExport('closeTestModal', closeTestModal)) exportCount++;
    
    // Parameter management functions
    if (safeExport('generateDefaultForParam', generateDefaultForParam)) exportCount++;
    if (safeExport('getFunctionDefaults', getFunctionDefaults)) exportCount++;
    if (safeExport('getFunctionDefinition', getFunctionDefinition)) exportCount++;
    if (safeExport('showParameterHints', showParameterHints)) exportCount++;
    if (safeExport('showParameterHintsWithReset', showParameterHintsWithReset)) exportCount++;
    if (safeExport('resetToDefaultParams', resetToDefaultParams)) exportCount++;
    if (safeExport('resetEditStepParams', resetEditStepParams)) exportCount++;
    if (safeExport('onFunctionSelectChange', onFunctionSelectChange)) exportCount++;
    
    // Notify that exports are ready
    window.linearFlowDesignerReady = true;
    console.log(`✅ Linear Flow Designer functions exported: ${exportCount} successful, ${skipCount} skipped`);
    
    // Dispatch custom event to notify that module is ready
    window.dispatchEvent(new Event('linearFlowDesignerReady'));
})();