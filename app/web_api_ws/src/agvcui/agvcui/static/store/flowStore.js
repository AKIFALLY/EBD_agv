/**
 * FlowStore - Linear Flow Designer 專用資料儲存
 * 基於 miniStore 提供進階功能
 */

import { createStore } from './miniStore.js';

class FlowStore {
    constructor() {
        // 預設流程結構
        const defaultFlow = {
            flow: {
                id: '',
                name: 'New Flow',
                work_id: '',
                priority: 100,
                enabled: true
            },
            meta: {
                description: '',
                version: '2.0',
                created_at: new Date().toISOString()
            },
            workflow: []
        };
        
        // 使用 miniStore 作為底層儲存
        this.baseStore = createStore('linearFlowDesigner', defaultFlow);
        
        // 計算屬性快取
        this.computed = {
            variables: new Map(),
            validation: { errors: [], warnings: [] },
            statistics: {
                sectionCount: 0,
                stepCount: 0,
                variableCount: 0
            }
        };
        
        // 事件監聽器
        this.listeners = new Map();
        
        // 更新來源追蹤（防止循環更新）
        this.updateSource = null;
        
        // 訂閱基礎 store 變更
        this.baseStore.on('change', (state) => {
            this.updateComputed(state);
            this.notifyListeners(state);
        });
        
        // 初始計算
        this.updateComputed(this.baseStore.getState());
    }
    
    /**
     * 取得當前狀態
     */
    getState() {
        return this.baseStore.getState();
    }
    
    /**
     * 取得完整狀態（包含計算值）
     */
    getFullState() {
        return {
            ...this.baseStore.getState(),
            computed: this.computed
        };
    }
    
    /**
     * 更新流程資訊
     */
    updateFlow(flowData) {
        const state = this.baseStore.getState();
        this.baseStore.setState({
            ...state,
            flow: {
                ...state.flow,
                ...flowData
            }
        });
    }
    
    /**
     * 更新元資料
     */
    updateMeta(metaData) {
        const state = this.baseStore.getState();
        this.baseStore.setState({
            ...state,
            meta: {
                ...state.meta,
                ...metaData
            }
        });
    }
    
    /**
     * 更新整個工作流程
     */
    updateWorkflow(workflow) {
        const state = this.baseStore.getState();
        this.baseStore.setState({
            ...state,
            workflow: workflow
        });
    }
    
    /**
     * 新增區段
     */
    addSection(section) {
        const state = this.baseStore.getState();
        const workflow = [...(state.workflow || [])];
        workflow.push({
            section: section.name || 'New Section',
            description: section.description || '',
            condition: section.condition || null,
            steps: []
        });
        this.updateWorkflow(workflow);
        return workflow.length - 1; // 返回新區段索引
    }
    
    /**
     * 更新區段
     */
    updateSection(index, sectionData) {
        const state = this.baseStore.getState();
        const workflow = [...(state.workflow || [])];
        if (workflow[index]) {
            workflow[index] = {
                ...workflow[index],
                ...sectionData
            };
            this.updateWorkflow(workflow);
        }
    }
    
    /**
     * 刪除區段
     */
    deleteSection(index) {
        const state = this.baseStore.getState();
        const workflow = [...(state.workflow || [])];
        workflow.splice(index, 1);
        this.updateWorkflow(workflow);
    }
    
    /**
     * 移動區段
     */
    moveSection(fromIndex, toIndex) {
        const state = this.baseStore.getState();
        const workflow = [...(state.workflow || [])];
        const [section] = workflow.splice(fromIndex, 1);
        workflow.splice(toIndex, 0, section);
        this.updateWorkflow(workflow);
    }
    
    /**
     * 新增步驟
     */
    addStep(sectionIndex, step) {
        const state = this.baseStore.getState();
        const workflow = [...(state.workflow || [])];
        if (workflow[sectionIndex]) {
            const steps = [...(workflow[sectionIndex].steps || [])];
            steps.push(step);
            workflow[sectionIndex] = {
                ...workflow[sectionIndex],
                steps: steps
            };
            this.updateWorkflow(workflow);
            return steps.length - 1; // 返回新步驟索引
        }
        return -1;
    }
    
    /**
     * 更新步驟
     */
    updateStep(sectionIndex, stepIndex, stepData) {
        const state = this.baseStore.getState();
        const workflow = [...(state.workflow || [])];
        if (workflow[sectionIndex] && workflow[sectionIndex].steps[stepIndex]) {
            const steps = [...workflow[sectionIndex].steps];
            steps[stepIndex] = {
                ...steps[stepIndex],
                ...stepData
            };
            workflow[sectionIndex] = {
                ...workflow[sectionIndex],
                steps: steps
            };
            this.updateWorkflow(workflow);
        }
    }
    
    /**
     * 刪除步驟
     */
    deleteStep(sectionIndex, stepIndex) {
        const state = this.baseStore.getState();
        const workflow = [...(state.workflow || [])];
        if (workflow[sectionIndex] && workflow[sectionIndex].steps) {
            const steps = [...workflow[sectionIndex].steps];
            steps.splice(stepIndex, 1);
            workflow[sectionIndex] = {
                ...workflow[sectionIndex],
                steps: steps
            };
            this.updateWorkflow(workflow);
        }
    }
    
    /**
     * 移動步驟
     */
    moveStep(sectionIndex, fromIndex, toIndex) {
        const state = this.baseStore.getState();
        const workflow = [...(state.workflow || [])];
        if (workflow[sectionIndex] && workflow[sectionIndex].steps) {
            const steps = [...workflow[sectionIndex].steps];
            const [step] = steps.splice(fromIndex, 1);
            steps.splice(toIndex, 0, step);
            workflow[sectionIndex] = {
                ...workflow[sectionIndex],
                steps: steps
            };
            this.updateWorkflow(workflow);
        }
    }
    
    /**
     * 從 YAML 更新
     */
    updateFromYAML(yamlContent, source = 'yaml') {
        try {
            const parsed = jsyaml.load(yamlContent);
            
            // 確保必要欄位存在
            if (!parsed.workflow) {
                parsed.workflow = [];
            }
            if (!parsed.flow) {
                parsed.flow = {};
            }
            if (!parsed.meta) {
                parsed.meta = {};
            }
            
            this.updateSource = source;
            this.baseStore.setState(parsed);
            this.updateSource = null;
            
            return { success: true };
        } catch (error) {
            return { success: false, error: error.message };
        }
    }
    
    /**
     * 轉換為 YAML
     */
    toYAML() {
        const state = this.baseStore.getState();
        return jsyaml.dump(state, {
            indent: 2,
            lineWidth: -1,
            noRefs: true,
            sortKeys: false
        });
    }
    
    /**
     * 更新計算屬性
     */
    updateComputed(state) {
        // 提取變數
        this.computed.variables = this.extractVariables(state);
        
        // 執行驗證
        this.computed.validation = this.validateFlow(state);
        
        // 更新統計
        this.computed.statistics = {
            sectionCount: state.workflow ? state.workflow.length : 0,
            stepCount: state.workflow ? 
                state.workflow.reduce((sum, section) => 
                    sum + (section.steps ? section.steps.length : 0), 0) : 0,
            variableCount: this.computed.variables.size
        };
    }
    
    /**
     * 提取變數
     */
    extractVariables(state) {
        const variables = new Map();
        
        // 系統變數 (與後端 flow_executor.py 保持一致)
        const systemVars = [
            '_flow_id',        // 流程 ID
            '_flow_name',      // 流程名稱
            '_work_id',        // 工作 ID
            '_priority',       // 流程優先級
            '_timestamp',      // 執行開始時間
            '_execution_time', // 執行時間
            '_errors'          // 錯誤列表
        ];
        systemVars.forEach(varName => {
            variables.set(varName, {
                type: 'system',
                source: 'built-in',
                value: null
            });
        });
        
        // 從工作流程提取變數
        if (state.workflow) {
            state.workflow.forEach((section, sectionIndex) => {
                if (section.steps) {
                    this.extractVariablesFromSteps(section.steps, variables, 
                        `${section.section}`, sectionIndex);
                }
            });
        }
        
        return variables;
    }
    
    /**
     * 從步驟中提取變數（支援 foreach 巢狀結構）
     */
    extractVariablesFromSteps(steps, variables, sectionName, sectionIndex, parentVars = new Set()) {
        steps.forEach((step, stepIndex) => {
            // 處理 foreach 特殊情況
            if (step.exec === 'foreach' || step.exec === 'control.foreach') {
                // 1. foreach 的迭代變數
                const iterVar = step.params?.var || step.var || 'item';
                variables.set(iterVar, {
                    type: 'foreach_var',
                    source: `${sectionName} > ${step.id || 'ForEach ' + (stepIndex + 1)}`,
                    exec: step.exec,
                    sectionIndex,
                    stepIndex
                });
                
                // 2. 創建新的變數作用域，包含父作用域的變數和迭代變數
                const foreachVars = new Set([...parentVars, iterVar]);
                
                // 3. 遞迴處理 foreach 內部的步驟
                if (step.params?.steps && Array.isArray(step.params.steps)) {
                    this.extractVariablesFromSteps(step.params.steps, variables, 
                        `${sectionName} > ${step.id || 'ForEach'}`, sectionIndex, foreachVars);
                } else if (step.steps && Array.isArray(step.steps)) {
                    this.extractVariablesFromSteps(step.steps, variables, 
                        `${sectionName} > ${step.id || 'ForEach'}`, sectionIndex, foreachVars);
                }
                
                // 4. 如果 foreach 有 store，記錄結果變數
                if (step.store) {
                    variables.set(step.store, {
                        type: 'foreach_result',
                        source: `${sectionName} > ${step.id || 'ForEach ' + (stepIndex + 1)}`,
                        exec: step.exec,
                        sectionIndex,
                        stepIndex
                    });
                }
            } else {
                // 一般步驟的變數儲存
                if (step.store) {
                    variables.set(step.store, {
                        type: 'step',
                        source: `${sectionName} > ${step.id || 'Step ' + (stepIndex + 1)}`,
                        exec: step.exec,
                        sectionIndex,
                        stepIndex
                    });
                }
            }
            
            // 提取參考的變數
            const refs = this.extractVariableReferences(step);
            refs.forEach(ref => {
                // 移除陣列索引部分（如 single_room_location[0] -> single_room_location）
                const baseRef = ref.split('[')[0].split('.')[0];
                
                // 檢查是否在父作用域中（foreach 變數）或全域變數中
                if (!parentVars.has(baseRef) && !variables.has(baseRef)) {
                    // 只記錄基礎變數名稱，而不是帶索引的版本
                    variables.set(baseRef, {
                        type: 'reference',
                        source: 'undefined',
                        warning: '變數尚未定義'
                    });
                }
            });
        });
    }
    
    /**
     * 提取變數參考
     */
    extractVariableReferences(step) {
        const refs = new Set();
        
        // Helper function to extract base variable name without array indices
        const extractBaseVarName = (varRef) => {
            // Remove array indices and property accessors
            // single_room_location[0] -> single_room_location
            // obj.prop -> obj
            return varRef.split('[')[0].split('.')[0];
        };
        
        // 從參數提取（但排除 foreach 的特殊參數）
        if (step.params) {
            // 如果是 foreach，排除 items、var、steps 等特殊參數
            if (step.exec === 'foreach' || step.exec === 'control.foreach') {
                // 只處理 items 參數中的變數引用
                if (step.params.items && typeof step.params.items === 'string') {
                    const matches = step.params.items.match(/\$\{([^}]+)\}/g);
                    if (matches) {
                        matches.forEach(match => {
                            const fullRef = match.slice(2, -1);
                            const baseRef = extractBaseVarName(fullRef);
                            refs.add(baseRef);
                        });
                    }
                }
                // 不處理 var 和 steps，它們是結構性參數
            } else {
                // 一般步驟，處理所有參數
                const paramString = JSON.stringify(step.params);
                const matches = paramString.match(/\$\{([^}]+)\}/g);
                if (matches) {
                    matches.forEach(match => {
                        const fullRef = match.slice(2, -1);
                        const baseRef = extractBaseVarName(fullRef);
                        refs.add(baseRef);
                    });
                }
            }
        }
        
        // 從條件提取 (skip_if 和 skip_if_not)
        if (step.skip_if) {
            const matches = step.skip_if.match(/\$\{([^}]+)\}/g);
            if (matches) {
                matches.forEach(match => {
                    const fullRef = match.slice(2, -1);
                    const baseRef = extractBaseVarName(fullRef);
                    refs.add(baseRef);
                });
            }
        }
        
        if (step.skip_if_not) {
            const matches = step.skip_if_not.match(/\$\{([^}]+)\}/g);
            if (matches) {
                matches.forEach(match => {
                    const fullRef = match.slice(2, -1);
                    const baseRef = extractBaseVarName(fullRef);
                    refs.add(baseRef);
                });
            }
        }
        
        // 從 items 提取（如果直接在步驟層級）
        if (step.items && typeof step.items === 'string') {
            const matches = step.items.match(/\$\{([^}]+)\}/g);
            if (matches) {
                matches.forEach(match => {
                    const fullRef = match.slice(2, -1);
                    const baseRef = extractBaseVarName(fullRef);
                    refs.add(baseRef);
                });
            }
        }
        
        return refs;
    }
    
    /**
     * 驗證流程
     */
    validateFlow(state) {
        const errors = [];
        const warnings = [];
        
        // 驗證基本資訊
        if (!state.flow || !state.flow.id) {
            errors.push('流程 ID 不能為空');
        }
        if (!state.flow || !state.flow.name) {
            warnings.push('建議設定流程名稱');
        }
        
        // 驗證工作流程
        if (!state.workflow || state.workflow.length === 0) {
            warnings.push('工作流程為空');
        }
        
        // 驗證變數
        this.computed.variables.forEach((varInfo, varName) => {
            if (varInfo.type === 'reference' && varInfo.warning) {
                warnings.push(`變數 ${varName} ${varInfo.warning}`);
            }
        });
        
        return { errors, warnings };
    }
    
    /**
     * 訂閱變更事件
     */
    on(event, listener) {
        if (!this.listeners.has(event)) {
            this.listeners.set(event, new Set());
        }
        this.listeners.get(event).add(listener);
    }
    
    /**
     * 取消訂閱
     */
    off(event, listener) {
        const listeners = this.listeners.get(event);
        if (listeners) {
            listeners.delete(listener);
        }
    }
    
    /**
     * 通知監聽器
     */
    notifyListeners(state) {
        const fullState = this.getFullState();
        
        // 通知 'change' 事件
        const listeners = this.listeners.get('change');
        if (listeners) {
            listeners.forEach(listener => {
                listener(fullState, this.updateSource);
            });
        }
        
        // 通知特定事件
        if (this.updateSource) {
            const sourceListeners = this.listeners.get(`change:${this.updateSource}`);
            if (sourceListeners) {
                sourceListeners.forEach(listener => {
                    listener(fullState, this.updateSource);
                });
            }
        }
    }
    
    /**
     * 清除所有資料
     */
    clear() {
        this.baseStore.clear();
    }
    
    /**
     * 載入流程
     */
    loadFlow(flowData) {
        this.baseStore.setState(flowData);
    }
}

// 創建單例實例
const flowStore = new FlowStore();

// 導出
export { flowStore, FlowStore };