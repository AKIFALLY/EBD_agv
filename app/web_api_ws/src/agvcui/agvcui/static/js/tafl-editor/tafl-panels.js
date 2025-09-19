/**
 * TAFL Panels Integration
 * 面板系統 - CSS 控制樣式，JS 控制功能
 */

import { taflFlowStore } from './tafl-editor-store.js';

class TAFLPanels {
    constructor() {
        this.panels = {};
        this.codeMirrorEditors = []; // Store CodeMirror instances for refresh
        this.isUpdatingPanel = false; // Flag to prevent infinite update loops
        this.initPanels();
        this.bindStoreEvents();
    }
    
    // ============================================
    // 初始化所有面板
    // ============================================
    
    initPanels() {
        // Variables 面板 - 現在在右側面板中
        const variablesContainer = document.getElementById('variables-list');
        if (variablesContainer) {
            this.panels.variables = variablesContainer;
            this.updateVariablesPanel();
            
            // 綁定 Add Variable 按鈕事件
            const addVariableBtn = document.getElementById('add-variable-btn');
            if (addVariableBtn) {
                addVariableBtn.onclick = () => this.showAddVariableModal();
            }
        }
        
        // Settings 面板  
        const settingsContainer = document.getElementById('settings-panel');
        if (settingsContainer) {
            this.panels.settings = settingsContainer;
            this.updateSettingsPanel();
        }
        
        // Rules 面板
        const rulesContainer = document.getElementById('rules-panel');
        if (rulesContainer) {
            this.panels.rules = rulesContainer;
            this.updateRulesPanel();
            
            // 綁定 Add Rule 按鈕事件
            const addRuleBtn = document.getElementById('add-rule-btn');
            if (addRuleBtn) {
                addRuleBtn.onclick = () => this.showAddRuleModal();
            }
        }
        
        // Preload 面板
        const preloadContainer = document.getElementById('preload-panel');
        if (preloadContainer) {
            this.panels.preload = preloadContainer;
            this.updatePreloadPanel();
            
            // 綁定 Add Preload 按鈕事件
            const addPreloadBtn = document.getElementById('add-preload-btn');
            if (addPreloadBtn) {
                addPreloadBtn.onclick = () => this.showAddPreloadModal();
            }
        }
    }
    
    // ============================================
    // Store 事件綁定
    // ============================================
    
    bindStoreEvents() {
        // Variables 更新
        taflFlowStore.on('variables:changed', () => {
            console.log('📌 TAFLPanels received variables:changed event');
            this.updateVariablesPanel();
        });
        
        // Flow 載入
        taflFlowStore.on('flow:loaded', () => {
            console.log('📌 TAFLPanels received flow:loaded event, updating all panels...');
            this.updateAllPanels();
        });
        
        // Flow 清除
        taflFlowStore.on('flow:cleared', () => {
            console.log('📌 TAFLPanels received flow:cleared event, clearing all panels...');
            this.updateAllPanels();  // This will update with empty flow data
        });
        
        // Flow 改變
        taflFlowStore.on('flow:changed', () => {
            console.log('📌 TAFLPanels received flow:changed event');
            this.updateSettingsPanel();
            this.updateRulesPanel();
            this.updatePreloadPanel();
        });
    }
    
    // ============================================
    // 更新所有面板
    // ============================================
    
    updateAllPanels() {
        this.updateVariablesPanel();
        this.updateSettingsPanel();
        this.updateRulesPanel();
        this.updatePreloadPanel();
    }
    
    // ============================================
    // Variables 面板
    // ============================================
    
    updateVariablesPanel() {
        // Prevent infinite update loops
        if (this.isUpdatingPanel) return;
        
        const container = this.panels.variables;
        console.log('📌 updateVariablesPanel called, container:', container);
        if (!container) {
            console.error('❌ Variables panel container not found!');
            return;
        }
        
        this.isUpdatingPanel = true;
        
        // Clear old variable editors from the array
        const oldEditors = container.querySelectorAll('.CodeMirror');
        oldEditors.forEach(cm => {
            const editor = cm.CodeMirror;
            if (editor) {
                const index = this.codeMirrorEditors.indexOf(editor);
                if (index > -1) {
                    this.codeMirrorEditors.splice(index, 1);
                }
            }
        });
        
        const variables = taflFlowStore.getFlow().variables || {};
        console.log('📌 Current variables:', variables);
        
        // 只清空變數列表，不清空整個容器
        // 保留標題、描述和按鈕
        let list = container;
        
        // 檢查是否已經有 title 和 description
        const hasTitle = container.querySelector('.title');
        if (hasTitle) {
            // 如果有標題，表示需要保留結構，只清空變數列表
            list = container.querySelector('#variables-list');
            if (!list) {
                // 創建變數列表容器
                list = document.createElement('div');
                list.id = 'variables-list';
                list.className = 'variables-list';
                container.appendChild(list);
            }
            list.innerHTML = '';
        } else {
            // 沒有標題的話，清空整個容器
            container.innerHTML = '';
            list = container;
        }
        
        // 添加每個變數
        Object.entries(variables).forEach(([key, value]) => {
            console.log(`📌 Adding variable: ${key} = ${value}`);
            const item = this.createVariableItem(key, value);
            list.appendChild(item);
        });
        
        // Add Variable button is now in the HTML template and bound in initPanels()
        
        console.log('📌 Panel update complete, total variables:', Object.keys(variables).length);
    }
    
    createVariableItem(key, value) {
        // 使用與 Rules 和 Preload 相同的 box 樣式
        const item = document.createElement('div');
        item.className = 'box';
        
        // 標題應該永遠顯示變數名稱 (key)，而不是值 (value)
        const heading = document.createElement('p');
        heading.className = 'heading';
        // 直接使用 key 作為標題
        heading.textContent = key;
        item.appendChild(heading);
        
        // 創建 CodeMirror 編輯器容器
        const control = document.createElement('div');
        control.className = 'control';
        
        const editorContainer = document.createElement('div');
        editorContainer.style.border = '1px solid #dbdbdb';
        editorContainer.style.borderRadius = '4px';
        control.appendChild(editorContainer);
        item.appendChild(control);
        
        // 立即初始化 CodeMirror 編輯器
        // 直接顯示原始值，不自動加引號
        let editorValue;
        if (typeof value === 'object' && value !== null) {
            editorValue = JSON.stringify(value, null, 2);
        } else {
            // 所有其他類型都直接轉換為字串，不自動加引號
            editorValue = String(value);
        }
        
        const editor = CodeMirror(editorContainer, {
            value: editorValue,
            mode: 'application/json',
            theme: 'default',
            lineNumbers: true,
            lineWrapping: true,
            matchBrackets: true,
            autoCloseBrackets: true,
            foldGutter: true,
            gutters: ['CodeMirror-linenumbers', 'CodeMirror-foldgutter'],
            extraKeys: {
                'Ctrl-Space': 'autocomplete',
                'Ctrl-F': function(cm) {
                    // 格式化值
                    try {
                        const currentValue = cm.getValue().trim();
                        let parsed;
                        
                        // 嘗試解析為 JSON
                        try {
                            parsed = JSON.parse(currentValue);
                        } catch {
                            // 如果不是有效的 JSON，嘗試將其視為原始值
                            if (currentValue === 'true' || currentValue === 'false') {
                                parsed = currentValue === 'true';
                            } else if (!isNaN(currentValue) && currentValue !== '') {
                                parsed = Number(currentValue);
                            } else {
                                parsed = currentValue;
                            }
                        }
                        
                        // 格式化值
                        if (typeof parsed === 'object' && parsed !== null) {
                            cm.setValue(JSON.stringify(parsed, null, 2));
                        } else {
                            cm.setValue(JSON.stringify(parsed));
                        }
                    } catch (e) {
                        console.error('Cannot format value:', e);
                    }
                }
            }
        });
        
        // 設置編輯器高度
        editor.setSize(null, '100px');
        
        // Store editor reference for later refresh
        this.codeMirrorEditors.push(editor);
        
        // Force refresh after rendering to ensure proper display
        requestAnimationFrame(() => {
            editor.refresh();
        });
        
        // 使用防抖來避免頻繁更新
        let updateTimeout = null;
        
        editor.on('change', (cm) => {
            // 清除之前的 timeout
            if (updateTimeout) {
                clearTimeout(updateTimeout);
            }
            
            // 立即更新錯誤狀態（不延遲）
            try {
                const newValue = cm.getValue().trim();
                // 嘗試解析為 JSON
                try {
                    JSON.parse(newValue);
                    editorContainer.classList.remove('has-error');
                } catch {
                    // 如果不是 JSON，當作字串也是有效的
                    editorContainer.classList.remove('has-error');
                }
            } catch (error) {
                editorContainer.classList.add('has-error');
                return;
            }
            
            // 延遲更新 store（防抖 500ms）
            updateTimeout = setTimeout(() => {
                try {
                    const newValue = cm.getValue().trim();
                    // 嘗試解析為 JSON，如果失敗則當作字串
                    let parsedValue;
                    try {
                        parsedValue = JSON.parse(newValue);
                    } catch {
                        parsedValue = newValue;
                    }
                    
                    const variables = taflFlowStore.getFlow().variables;
                    variables[key] = parsedValue;
                    taflFlowStore.updateVariables(variables);
                    
                    // 不更新標題 - 標題應該保持為變數名稱（key）
                    // heading.textContent 不應該在這裡被更新
                } catch (error) {
                    console.error('Error updating variable:', error);
                }
            }, 500);  // 500ms 防抖延遲
        });
        
        // 保存編輯器實例
        item.dataset.editor = 'cm';
        item.cmEditor = editor;
        
        // 確保 CodeMirror 正確渲染
        requestAnimationFrame(() => {
            editor.refresh();
        });
        
        // 按鈕組 (與 Rules/Preload 相同的樣式)
        const buttons = document.createElement('div');
        buttons.className = 'field is-grouped is-grouped-right';
        buttons.style.marginTop = '0.5rem';
        
        // 格式化按鈕 (for CodeMirror)
        const formatControl = document.createElement('p');
        formatControl.className = 'control';
        const formatBtn = document.createElement('button');
        formatBtn.className = 'button is-small is-info';
        formatBtn.onclick = () => {
            if (item.cmEditor) {
                try {
                    const currentValue = item.cmEditor.getValue().trim();
                    let parsed;
                    
                    // 嘗試解析為 JSON
                    try {
                        parsed = JSON.parse(currentValue);
                    } catch {
                        // 如果不是有效的 JSON，嘗試將其視為原始值
                        // 數字、布林值或字串
                        if (currentValue === 'true' || currentValue === 'false') {
                            parsed = currentValue === 'true';
                        } else if (!isNaN(currentValue) && currentValue !== '') {
                            parsed = Number(currentValue);
                        } else {
                            // 將其視為字串
                            parsed = currentValue;
                        }
                    }
                    
                    // 格式化值
                    if (typeof parsed === 'object' && parsed !== null) {
                        item.cmEditor.setValue(JSON.stringify(parsed, null, 2));
                    } else {
                        // 對於原始值，轉換為 JSON 字串表示
                        item.cmEditor.setValue(JSON.stringify(parsed));
                    }
                } catch (e) {
                    console.error('Cannot format value:', e);
                }
            }
        };
        const formatIcon = document.createElement('span');
        formatIcon.className = 'icon';
        const formatI = document.createElement('i');
        formatI.className = 'fas fa-magic';
        formatIcon.appendChild(formatI);
        formatBtn.appendChild(formatIcon);
        const formatText = document.createElement('span');
        formatText.textContent = 'Format';
        formatBtn.appendChild(formatText);
        formatControl.appendChild(formatBtn);
        
        // 刪除按鈕
        const deleteControl = document.createElement('p');
        deleteControl.className = 'control';
        const deleteBtn = document.createElement('button');
        deleteBtn.className = 'button is-small is-danger';
        deleteBtn.onclick = () => {
            if (confirm(`Delete variable "${key}"?`)) {
                const variables = { ...taflFlowStore.getFlow().variables };
                delete variables[key];
                taflFlowStore.updateVariables(variables);
            }
        };
        const deleteIcon = document.createElement('span');
        deleteIcon.className = 'icon';
        const deleteI = document.createElement('i');
        deleteI.className = 'fas fa-trash';
        deleteIcon.appendChild(deleteI);
        deleteBtn.appendChild(deleteIcon);
        const deleteText = document.createElement('span');
        deleteText.textContent = 'Delete';
        deleteBtn.appendChild(deleteText);
        deleteControl.appendChild(deleteBtn);
        
        buttons.appendChild(formatControl);
        buttons.appendChild(deleteControl);
        item.appendChild(buttons);
        
        return item;
    }
    
    // ============================================
    // Settings 面板
    // ============================================
    
    updateSettingsPanel() {
        const container = this.panels.settings;
        if (!container) return;
        
        const settings = taflFlowStore.getFlow().settings || {};
        
        // 更新現有的 HTML 欄位，而不是創建新的
        const executionIntervalInput = container.querySelector('#settings-execution-interval');
        if (executionIntervalInput) {
            executionIntervalInput.value = settings.execution_interval || 5;
            // 只綁定一次事件
            if (!executionIntervalInput.dataset.bound) {
                executionIntervalInput.dataset.bound = 'true';
                executionIntervalInput.addEventListener('change', (e) => {
                    const flow = taflFlowStore.getFlow();
                    const newSettings = {
                        ...(flow.settings || {}),
                        execution_interval: parseInt(e.target.value, 10)
                    };
                    taflFlowStore.updateFlow({ settings: newSettings });
                });
            }
        }
        
        // Reset the flag after update is complete
        this.isUpdatingPanel = false;
    }
    
    // ============================================
    // Rules 面板
    // ============================================
    
    updateRulesPanel() {
        // Prevent infinite update loops
        if (this.isUpdatingPanel) return;
        
        const container = this.panels.rules;
        if (!container) return;
        
        this.isUpdatingPanel = true;
        
        // Clear old rule editors from the array
        const oldEditors = container.querySelectorAll('.CodeMirror');
        oldEditors.forEach(cm => {
            const editor = cm.CodeMirror;
            if (editor) {
                const index = this.codeMirrorEditors.indexOf(editor);
                if (index > -1) {
                    this.codeMirrorEditors.splice(index, 1);
                }
            }
        });
        
        const rules = taflFlowStore.getFlow().rules || {};
        console.log('📌 Updating rules panel with:', rules);
        
        // 找到 property-editor div，保留標題和描述
        let propertyEditor = container.querySelector('.property-editor');
        if (!propertyEditor) {
            // 如果沒有 property-editor，使用整個容器
            propertyEditor = container;
        }
        
        // 只清空 rules-list 的內容，保留標題
        let list = propertyEditor.querySelector('.rules-list');
        if (!list) {
            // 如果沒有 list，創建一個
            list = document.createElement('div');
            list.className = 'rules-list';
            propertyEditor.appendChild(list);
        }
        list.innerHTML = '';
        
        // 添加所有規則項目
        Object.entries(rules).forEach(([key, value]) => {
            const item = this.createRuleItem(key, value);
            list.appendChild(item);
        });
        
        // Add Rule button is now in HTML and bound in initPanels()
        
        // Reset the flag after update is complete
        this.isUpdatingPanel = false;
    }
    
    createRuleItem(key, value) {
        // 使用與 Preload 相同的 box 樣式 - 不要添加內聯樣式
        const item = document.createElement('div');
        item.className = 'box';
        
        // 標題應該永遠顯示規則名稱 (key)，而不是值 (value)
        const heading = document.createElement('p');
        heading.className = 'heading';
        // 直接使用 key 作為標題
        heading.textContent = key;
        item.appendChild(heading);
        
        // 創建 CodeMirror 編輯器容器
        const control = document.createElement('div');
        control.className = 'control';
        
        const editorContainer = document.createElement('div');
        editorContainer.style.border = '1px solid #dbdbdb';
        editorContainer.style.borderRadius = '4px';
        control.appendChild(editorContainer);
        item.appendChild(control);
        
        // 立即初始化 CodeMirror 編輯器
        // 直接顯示原始值，不自動加引號
        let editorValue;
        if (typeof value === 'object' && value !== null) {
            editorValue = JSON.stringify(value, null, 2);
        } else {
            // 所有其他類型都直接轉換為字串，不自動加引號
            editorValue = String(value);
        }
        
        const editor = CodeMirror(editorContainer, {
            value: editorValue,
            mode: 'application/json',
            theme: 'default',
            lineNumbers: true,
            lineWrapping: true,
            matchBrackets: true,
            autoCloseBrackets: true,
            foldGutter: true,
            gutters: ['CodeMirror-linenumbers', 'CodeMirror-foldgutter'],
            extraKeys: {
                'Ctrl-Space': 'autocomplete',
                'Ctrl-F': function(cm) {
                    // 格式化值
                    try {
                        const currentValue = cm.getValue().trim();
                        let parsed;
                        
                        // 嘗試解析為 JSON
                        try {
                            parsed = JSON.parse(currentValue);
                        } catch {
                            // 如果不是有效的 JSON，嘗試將其視為原始值
                            if (currentValue === 'true' || currentValue === 'false') {
                                parsed = currentValue === 'true';
                            } else if (!isNaN(currentValue) && currentValue !== '') {
                                parsed = Number(currentValue);
                            } else {
                                parsed = currentValue;
                            }
                        }
                        
                        // 格式化值
                        if (typeof parsed === 'object' && parsed !== null) {
                            cm.setValue(JSON.stringify(parsed, null, 2));
                        } else {
                            cm.setValue(JSON.stringify(parsed));
                        }
                    } catch (e) {
                        console.error('Cannot format value:', e);
                    }
                }
            }
        });
        
        // 設置編輯器高度
        editor.setSize(null, '100px');
        
        // Store editor reference for later refresh
        this.codeMirrorEditors.push(editor);
        
        // Force refresh after rendering to ensure proper display
        requestAnimationFrame(() => {
            editor.refresh();
        });
        
        // 使用防抖來避免頻繁更新
        let updateTimeout = null;
        
        editor.on('change', (cm) => {
            // 清除之前的 timeout
            if (updateTimeout) {
                clearTimeout(updateTimeout);
            }
            
            // 立即更新錯誤狀態（不延遲）
            try {
                const newValue = cm.getValue().trim();
                // 嘗試解析為 JSON
                try {
                    JSON.parse(newValue);
                    editorContainer.classList.remove('has-error');
                } catch {
                    // 如果不是 JSON，當作字串或布林值也是有效的
                    editorContainer.classList.remove('has-error');
                }
            } catch (error) {
                editorContainer.classList.add('has-error');
                return;
            }
            
            // 延遲更新 store（防抖 500ms）
            updateTimeout = setTimeout(() => {
                try {
                    const newValue = cm.getValue().trim();
                    // 嘗試解析為 JSON，如果失敗則當作字串
                    let parsedValue;
                    try {
                        parsedValue = JSON.parse(newValue);
                    } catch {
                        parsedValue = newValue;
                    }
                    
                    const flow = taflFlowStore.getFlow();
                    flow.rules = flow.rules || {};
                    flow.rules[key] = parsedValue;
                    taflFlowStore.updateFlow(flow);
                    
                    // 不更新標題 - 標題應該保持為規則名稱（key）
                    // heading.textContent 不應該在這裡被更新
                } catch (error) {
                    console.error('Error updating rule:', error);
                }
            }, 500);  // 500ms 防抖延遲
        });
        
        // 保存編輯器實例
        item.dataset.editor = 'cm';
        item.cmEditor = editor;
        
        // 確保 CodeMirror 正確渲染
        requestAnimationFrame(() => {
            editor.refresh();
        });
        
        // 按鈕組 (與 Preload 相同的樣式)
        const buttons = document.createElement('div');
        buttons.className = 'field is-grouped is-grouped-right';
        buttons.style.marginTop = '0.5rem';
        
        // 格式化按鈕 (for CodeMirror)
        const formatControl = document.createElement('p');
        formatControl.className = 'control';
        const formatBtn = document.createElement('button');
        formatBtn.className = 'button is-small is-info';
        formatBtn.onclick = () => {
            if (item.cmEditor) {
                try {
                    const currentValue = item.cmEditor.getValue().trim();
                    let parsed;
                    
                    // 嘗試解析為 JSON
                    try {
                        parsed = JSON.parse(currentValue);
                    } catch {
                        // 如果不是有效的 JSON，嘗試將其視為原始值
                        // 數字、布林值或字串
                        if (currentValue === 'true' || currentValue === 'false') {
                            parsed = currentValue === 'true';
                        } else if (!isNaN(currentValue) && currentValue !== '') {
                            parsed = Number(currentValue);
                        } else {
                            // 將其視為字串
                            parsed = currentValue;
                        }
                    }
                    
                    // 格式化值
                    if (typeof parsed === 'object' && parsed !== null) {
                        item.cmEditor.setValue(JSON.stringify(parsed, null, 2));
                    } else {
                        // 對於原始值，轉換為 JSON 字串表示
                        item.cmEditor.setValue(JSON.stringify(parsed));
                    }
                } catch (e) {
                    console.error('Cannot format value:', e);
                }
            }
        };
        const formatIcon = document.createElement('span');
        formatIcon.className = 'icon';
        const formatI = document.createElement('i');
        formatI.className = 'fas fa-magic';
        formatIcon.appendChild(formatI);
        formatBtn.appendChild(formatIcon);
        const formatText = document.createElement('span');
        formatText.textContent = 'Format';
        formatBtn.appendChild(formatText);
        formatControl.appendChild(formatBtn);
        
        // 刪除按鈕
        const deleteControl = document.createElement('p');
        deleteControl.className = 'control';
        const deleteBtn = document.createElement('button');
        deleteBtn.className = 'button is-small is-danger';
        deleteBtn.onclick = () => {
            if (confirm(`Delete rule "${key}"?`)) {
                const flow = taflFlowStore.getFlow();
                delete flow.rules[key];
                taflFlowStore.updateFlow(flow);
            }
        };
        const deleteIcon = document.createElement('span');
        deleteIcon.className = 'icon';
        const deleteI = document.createElement('i');
        deleteI.className = 'fas fa-trash';
        deleteIcon.appendChild(deleteI);
        deleteBtn.appendChild(deleteIcon);
        const deleteText = document.createElement('span');
        deleteText.textContent = 'Delete';
        deleteBtn.appendChild(deleteText);
        deleteControl.appendChild(deleteBtn);
        
        buttons.appendChild(formatControl);
        buttons.appendChild(deleteControl);
        item.appendChild(buttons);
        
        return item;
    }
    
    // ============================================
    // Preload 面板
    // ============================================
    
    updatePreloadPanel() {
        // Prevent infinite update loops
        if (this.isUpdatingPanel) return;
        
        const container = this.panels.preload;
        if (!container) return;
        
        this.isUpdatingPanel = true;
        
        // Clear old preload editors from the array
        // Find and destroy old preload editors
        const oldEditors = container.querySelectorAll('.CodeMirror');
        oldEditors.forEach(cm => {
            const editor = cm.CodeMirror;
            if (editor) {
                const index = this.codeMirrorEditors.indexOf(editor);
                if (index > -1) {
                    this.codeMirrorEditors.splice(index, 1);
                }
            }
        });
        
        const preload = taflFlowStore.getFlow().preload || {};
        console.log('📌 Updating preload panel with:', preload);
        
        // 找到 property-editor div，保留標題和描述
        let propertyEditor = container.querySelector('.property-editor');
        if (!propertyEditor) {
            propertyEditor = container;
        }
        
        // 只清空 preload-list 的內容，保留標題
        let list = propertyEditor.querySelector('.preload-list');
        if (!list) {
            list = document.createElement('div');
            list.className = 'preload-list';
            propertyEditor.appendChild(list);
        }
        list.innerHTML = '';
        
        // 創建 preload 項目
        Object.entries(preload).forEach(([key, value]) => {
            const item = this.createPreloadItem(key, value);
            list.appendChild(item);
        });
        
        // Add Preload button is now in HTML and bound in initPanels()
        
        // Reset the flag after update is complete
        this.isUpdatingPanel = false;
    }
    
    createPreloadItem(key, value) {
        const item = document.createElement('div');
        item.className = 'box';
        
        // 標題應該永遠顯示預載項目名稱 (key)，而不是值 (value)
        const heading = document.createElement('p');
        heading.className = 'heading';
        // 直接使用 key 作為標題
        heading.textContent = key;
        item.appendChild(heading);
        
        // 創建 CodeMirror 編輯器容器
        const control = document.createElement('div');
        control.className = 'control';
        
        const editorContainer = document.createElement('div');
        editorContainer.style.border = '1px solid #dbdbdb';
        editorContainer.style.borderRadius = '4px';
        control.appendChild(editorContainer);
        item.appendChild(control);
        
        // 立即初始化 CodeMirror 編輯器（不延遲）
        const editor = CodeMirror(editorContainer, {
                value: JSON.stringify(value, null, 2),
                mode: 'application/json',
                theme: 'default',
                lineNumbers: true,
                lineWrapping: true,
                matchBrackets: true,
                autoCloseBrackets: true,
                foldGutter: true,
                gutters: ['CodeMirror-linenumbers', 'CodeMirror-foldgutter'],
                extraKeys: {
                    'Ctrl-Space': 'autocomplete',
                    'Ctrl-F': function(cm) {
                        // 格式化 JSON
                        try {
                            const parsed = JSON.parse(cm.getValue());
                            cm.setValue(JSON.stringify(parsed, null, 2));
                        } catch (e) {
                            console.error('Invalid JSON:', e);
                        }
                    }
                }
            });
            
            // 設置編輯器高度
            editor.setSize(null, '120px');
            
            // Store editor reference for later refresh
            this.codeMirrorEditors.push(editor);
            
            // Force refresh after a small delay to ensure proper rendering
            setTimeout(() => {
                editor.refresh();
            }, 100);
            
        // 使用防抖來避免頻繁更新
        let updateTimeout = null;
        let lastSavedValue = JSON.stringify(value);
        
        editor.on('change', (cm) => {
            // 清除之前的 timeout
            if (updateTimeout) {
                clearTimeout(updateTimeout);
            }
            
            // 立即更新錯誤狀態（不延遲）
            try {
                JSON.parse(cm.getValue());
                editorContainer.classList.remove('has-error');
            } catch (error) {
                editorContainer.classList.add('has-error');
                return;  // JSON 無效就不更新
            }
            
            // 延遲更新 store（防抖 800ms）
            updateTimeout = setTimeout(() => {
                try {
                    const newValue = JSON.parse(cm.getValue());
                    const newValueStr = JSON.stringify(newValue);
                    
                    // 只在值真正改變時更新
                    if (lastSavedValue !== newValueStr) {
                        lastSavedValue = newValueStr;
                        const flow = taflFlowStore.getFlow();
                        flow.preload = flow.preload || {};
                        flow.preload[key] = newValue;
                        
                        // 直接更新 store，不觸發完整 UI 重新渲染
                        taflFlowStore.setFlow(flow);
                        taflFlowStore.markDirty();
                        
                        // 不更新標題 - 標題應該保持為預載項目名稱（key）
                        // heading.textContent 不應該在這裡被更新
                    }
                } catch (error) {
                    // JSON 無效時不更新
                }
            }, 800);  // 800ms 防抖延遲
        });
        
        // 保存編輯器實例
        item.dataset.editor = 'cm';
        item.cmEditor = editor;
        
        // 確保 CodeMirror 正確渲染
        requestAnimationFrame(() => {
            editor.refresh();
        });
        
        // 按鈕組
        const buttons = document.createElement('div');
        buttons.className = 'field is-grouped is-grouped-right';
        buttons.style.marginTop = '0.5rem';
        
        // 格式化按鈕
        const formatControl = document.createElement('p');
        formatControl.className = 'control';
        const formatBtn = document.createElement('button');
        formatBtn.className = 'button is-small is-info';
        formatBtn.onclick = () => {
            if (item.cmEditor) {
                try {
                    const currentValue = item.cmEditor.getValue().trim();
                    let parsed;
                    
                    // 嘗試解析為 JSON
                    try {
                        parsed = JSON.parse(currentValue);
                    } catch {
                        // 如果不是有效的 JSON，嘗試將其視為原始值
                        // 數字、布林值或字串
                        if (currentValue === 'true' || currentValue === 'false') {
                            parsed = currentValue === 'true';
                        } else if (!isNaN(currentValue) && currentValue !== '') {
                            parsed = Number(currentValue);
                        } else {
                            // 將其視為字串
                            parsed = currentValue;
                        }
                    }
                    
                    // 格式化值
                    if (typeof parsed === 'object' && parsed !== null) {
                        item.cmEditor.setValue(JSON.stringify(parsed, null, 2));
                    } else {
                        // 對於原始值，轉換為 JSON 字串表示
                        item.cmEditor.setValue(JSON.stringify(parsed));
                    }
                } catch (e) {
                    console.error('Cannot format value:', e);
                }
            }
        };
        const formatIcon = document.createElement('span');
        formatIcon.className = 'icon';
        const formatI = document.createElement('i');
        formatI.className = 'fas fa-magic';
        formatIcon.appendChild(formatI);
        formatBtn.appendChild(formatIcon);
        const formatText = document.createElement('span');
        formatText.textContent = 'Format';
        formatBtn.appendChild(formatText);
        formatControl.appendChild(formatBtn);
        
        // 刪除按鈕
        const deleteControl = document.createElement('p');
        deleteControl.className = 'control';
        const deleteBtn = document.createElement('button');
        deleteBtn.className = 'button is-small is-danger';
        deleteBtn.onclick = () => {
            if (confirm(`Delete preload data "${key}"?`)) {
                const flow = taflFlowStore.getFlow();
                delete flow.preload[key];
                taflFlowStore.updateFlow(flow);
            }
        };
        const deleteIcon = document.createElement('span');
        deleteIcon.className = 'icon';
        const deleteI = document.createElement('i');
        deleteI.className = 'fas fa-trash';
        deleteIcon.appendChild(deleteI);
        deleteBtn.appendChild(deleteIcon);
        const deleteText = document.createElement('span');
        deleteText.textContent = 'Delete';
        deleteBtn.appendChild(deleteText);
        deleteControl.appendChild(deleteBtn);
        
        buttons.appendChild(formatControl);
        buttons.appendChild(deleteControl);
        item.appendChild(buttons);
        
        return item;
    }
    
    // ============================================
    // CodeMirror Refresh Method
    // ============================================
    
    refreshCodeMirrors() {
        // Refresh all CodeMirror instances
        this.codeMirrorEditors.forEach(editor => {
            if (editor && typeof editor.refresh === 'function') {
                editor.refresh();
            }
        });
    }
    
    // ============================================
    // 輔助方法 - 創建欄位
    // ============================================
    
    createTextField(label, value, onChange) {
        const field = document.createElement('div');
        field.className = 'field';
        
        const labelEl = document.createElement('label');
        labelEl.className = 'label';
        labelEl.textContent = label;
        
        const control = document.createElement('div');
        control.className = 'control';
        
        const textarea = document.createElement('textarea');
        textarea.className = 'textarea';
        textarea.value = value;
        textarea.addEventListener('change', (e) => onChange(e.target.value));
        
        control.appendChild(textarea);
        field.appendChild(labelEl);
        field.appendChild(control);
        
        return field;
    }
    
    createNumberField(label, value, onChange) {
        const field = document.createElement('div');
        field.className = 'field';
        
        const labelEl = document.createElement('label');
        labelEl.className = 'label';
        labelEl.textContent = label;
        
        const control = document.createElement('div');
        control.className = 'control';
        
        const input = document.createElement('input');
        input.className = 'input';
        input.type = 'number';
        input.value = value;
        input.addEventListener('change', (e) => onChange(e.target.value));
        
        control.appendChild(input);
        field.appendChild(labelEl);
        field.appendChild(control);
        
        return field;
    }
    
    createCheckboxField(label, checked, onChange) {
        const field = document.createElement('div');
        field.className = 'field';
        
        const control = document.createElement('div');
        control.className = 'control';
        
        const labelEl = document.createElement('label');
        labelEl.className = 'checkbox';
        
        const input = document.createElement('input');
        input.type = 'checkbox';
        input.checked = checked;
        input.addEventListener('change', (e) => onChange(e.target.checked));
        
        labelEl.appendChild(input);
        labelEl.appendChild(document.createTextNode(' ' + label));
        control.appendChild(labelEl);
        field.appendChild(control);
        
        return field;
    }
    
    createAddButton(text, onClick) {
        const div = document.createElement('div');
        div.className = 'panel-block';
        
        const btn = document.createElement('button');
        btn.className = 'button is-primary is-fullwidth';
        btn.onclick = onClick;
        
        const icon = document.createElement('span');
        icon.className = 'icon';
        const i = document.createElement('i');
        i.className = 'fas fa-plus';
        icon.appendChild(i);
        
        const span = document.createElement('span');
        span.textContent = text;
        
        btn.appendChild(icon);
        btn.appendChild(span);
        div.appendChild(btn);
        
        return div;
    }
    
    // ============================================
    // Modal Dialogs  
    // ============================================
    
    showAddVariableModal() {
        const modal = this.createModal('Add Variable', (modalEl) => {
            const body = modalEl.querySelector('.modal-card-body');
            
            // Name field
            const nameField = document.createElement('div');
            nameField.className = 'field';
            nameField.innerHTML = `
                <label class="label">Variable Name</label>
                <div class="control">
                    <input class="input" type="text" id="modal-var-name" placeholder="e.g., counter">
                </div>
            `;
            body.appendChild(nameField);
            
            // Value field
            const valueField = document.createElement('div');
            valueField.className = 'field';
            valueField.innerHTML = `
                <label class="label">Initial Value</label>
                <div class="control">
                    <input class="input" type="text" id="modal-var-value" placeholder="e.g., 0">
                </div>
            `;
            body.appendChild(valueField);
            
            // Save button
            const saveBtn = modalEl.querySelector('.modal-save-btn');
            saveBtn.onclick = () => {
                const name = document.getElementById('modal-var-name').value.trim();
                const value = document.getElementById('modal-var-value').value.trim();
                
                if (!name) {
                    alert('Please enter a variable name');
                    return;
                }
                
                const flow = taflFlowStore.getFlow();
                flow.variables = flow.variables || {};
                
                if (flow.variables[name] !== undefined) {
                    if (!confirm(`Variable "${name}" already exists. Replace it?`)) {
                        return;
                    }
                }
                
                flow.variables[name] = value || '';
                taflFlowStore.updateVariables(flow.variables);
                modalEl.remove();
            };
        });
    }
    
    showAddRuleModal() {
        const modal = this.createModal('Add Business Rule', (modalEl) => {
            const body = modalEl.querySelector('.modal-card-body');
            
            // Name field
            const nameField = document.createElement('div');
            nameField.className = 'field';
            nameField.innerHTML = `
                <label class="label">Rule Name</label>
                <div class="control">
                    <input class="input" type="text" id="modal-rule-name" placeholder="e.g., max_retries">
                </div>
            `;
            body.appendChild(nameField);
            
            // Type field
            const typeField = document.createElement('div');
            typeField.className = 'field';
            typeField.innerHTML = `
                <label class="label">Rule Type</label>
                <div class="control">
                    <div class="select is-fullwidth">
                        <select id="modal-rule-type">
                            <option value="string">String</option>
                            <option value="number">Number</option>
                            <option value="boolean">Boolean</option>
                            <option value="object">Object (JSON)</option>
                        </select>
                    </div>
                </div>
            `;
            body.appendChild(typeField);
            
            // Value field
            const valueField = document.createElement('div');
            valueField.className = 'field';
            valueField.innerHTML = `
                <label class="label">Rule Value</label>
                <div class="control">
                    <input class="input" type="text" id="modal-rule-value" placeholder="Enter value">
                </div>
            `;
            body.appendChild(valueField);
            
            // Object field (hidden by default)
            const objectField = document.createElement('div');
            objectField.className = 'field';
            objectField.style.display = 'none';
            objectField.innerHTML = `
                <label class="label">JSON Object</label>
                <div class="control">
                    <textarea class="textarea" id="modal-rule-object" rows="4" placeholder='{"key": "value"}'></textarea>
                </div>
            `;
            body.appendChild(objectField);
            
            // Type change handler
            document.getElementById('modal-rule-type').onchange = (e) => {
                if (e.target.value === 'object') {
                    valueField.style.display = 'none';
                    objectField.style.display = 'block';
                } else {
                    valueField.style.display = 'block';
                    objectField.style.display = 'none';
                }
            };
            
            // Save button
            const saveBtn = modalEl.querySelector('.modal-save-btn');
            saveBtn.onclick = () => {
                const name = document.getElementById('modal-rule-name').value.trim();
                const type = document.getElementById('modal-rule-type').value;
                
                if (!name) {
                    alert('Please enter a rule name');
                    return;
                }
                
                let value;
                if (type === 'object') {
                    const objText = document.getElementById('modal-rule-object').value.trim();
                    try {
                        value = objText ? JSON.parse(objText) : {};
                    } catch (e) {
                        alert('Invalid JSON format');
                        return;
                    }
                } else {
                    const rawValue = document.getElementById('modal-rule-value').value.trim();
                    if (type === 'number') {
                        value = Number(rawValue);
                        if (isNaN(value)) {
                            alert('Please enter a valid number');
                            return;
                        }
                    } else if (type === 'boolean') {
                        value = rawValue.toLowerCase() === 'true';
                    } else {
                        value = rawValue;
                    }
                }
                
                const flow = taflFlowStore.getFlow();
                flow.rules = flow.rules || {};
                
                if (flow.rules[name] !== undefined) {
                    if (!confirm(`Rule "${name}" already exists. Replace it?`)) {
                        return;
                    }
                }
                
                flow.rules[name] = value;
                taflFlowStore.updateFlow(flow);
                modalEl.remove();
            };
        });
    }
    
    showAddPreloadModal() {
        const modal = this.createModal('Add Preload Query', (modalEl) => {
            const body = modalEl.querySelector('.modal-card-body');
            
            // Variable name field
            const nameField = document.createElement('div');
            nameField.className = 'field';
            nameField.innerHTML = `
                <label class="label">Store As (Variable Name)</label>
                <div class="control">
                    <input class="input" type="text" id="modal-preload-name" placeholder="e.g., available_locations">
                </div>
                <p class="help">This will be the variable name you can use in the flow</p>
            `;
            body.appendChild(nameField);
            
            // Target field
            const targetField = document.createElement('div');
            targetField.className = 'field';
            targetField.innerHTML = `
                <label class="label">Query Target</label>
                <div class="control">
                    <div class="select is-fullwidth">
                        <select id="modal-preload-target">
                            <option value="locations">locations</option>
                            <option value="racks">racks</option>
                            <option value="tasks">tasks</option>
                            <option value="agvs">agvs</option>
                            <option value="products">products</option>
                        </select>
                    </div>
                </div>
            `;
            body.appendChild(targetField);
            
            // Where conditions field
            const whereField = document.createElement('div');
            whereField.className = 'field';
            whereField.innerHTML = `
                <label class="label">Where Conditions (Optional)</label>
                <div class="control">
                    <input class="input" type="text" id="modal-preload-where" placeholder="e.g., status: active, room_id: 1">
                </div>
                <p class="help">Format: key: value, key2: value2</p>
            `;
            body.appendChild(whereField);
            
            // Save button
            const saveBtn = modalEl.querySelector('.modal-save-btn');
            saveBtn.onclick = () => {
                const name = document.getElementById('modal-preload-name').value.trim();
                const target = document.getElementById('modal-preload-target').value;
                const whereStr = document.getElementById('modal-preload-where').value.trim();
                
                if (!name) {
                    alert('Please enter a variable name');
                    return;
                }
                
                // Parse where conditions
                const where = {};
                if (whereStr) {
                    const pairs = whereStr.split(',');
                    pairs.forEach(pair => {
                        const [key, value] = pair.split(':').map(s => s.trim());
                        if (key && value) {
                            where[key] = value;
                        }
                    });
                }
                
                const flow = taflFlowStore.getFlow();
                flow.preload = flow.preload || {};
                
                if (flow.preload[name] !== undefined) {
                    if (!confirm(`Preload "${name}" already exists. Replace it?`)) {
                        return;
                    }
                }
                
                flow.preload[name] = {
                    query: {
                        target: target,
                        ...(Object.keys(where).length > 0 && { where: where })
                    }
                };
                
                taflFlowStore.updateFlow(flow);
                modalEl.remove();
            };
        });
    }
    
    createModal(title, setupCallback) {
        const modal = document.createElement('div');
        modal.className = 'modal is-active';
        modal.innerHTML = `
            <div class="modal-background" onclick="this.parentElement.remove()"></div>
            <div class="modal-card">
                <header class="modal-card-head">
                    <p class="modal-card-title">${title}</p>
                    <button class="delete" aria-label="close" onclick="this.parentElement.parentElement.parentElement.remove()"></button>
                </header>
                <section class="modal-card-body">
                    <!-- Content will be added dynamically -->
                </section>
                <footer class="modal-card-foot">
                    <button class="button is-primary modal-save-btn">Save</button>
                    <button class="button" onclick="this.parentElement.parentElement.parentElement.remove()">Cancel</button>
                </footer>
            </div>
        `;
        
        document.body.appendChild(modal);
        
        // Call setup callback to populate modal content
        setupCallback(modal);
        
        // Focus first input
        setTimeout(() => {
            const firstInput = modal.querySelector('input, select, textarea');
            if (firstInput) firstInput.focus();
        }, 100);
        
        return modal;
    }
    
    /**
     * Show confirmation modal for flow deletion
     * @param {string} flowId - Flow ID to delete
     * @param {string} flowName - Flow name for display
     * @param {Function} onConfirm - Callback when user confirms deletion
     */
    showDeleteFlowModal(flowId, flowName, onConfirm) {
        const modal = document.createElement('div');
        modal.className = 'modal is-active';
        modal.innerHTML = `
            <div class="modal-background"></div>
            <div class="modal-card">
                <header class="modal-card-head">
                    <p class="modal-card-title">Delete Flow</p>
                    <button class="delete" aria-label="close"></button>
                </header>
                <section class="modal-card-body">
                    <div class="notification is-warning">
                        <p class="has-text-weight-semibold">Are you sure you want to delete this flow?</p>
                        <p class="mt-2">Flow: <strong>${flowName}</strong></p>
                        <p class="mt-2 has-text-danger">This action cannot be undone!</p>
                    </div>
                </section>
                <footer class="modal-card-foot">
                    <button class="button is-danger modal-delete-btn">
                        <span class="icon">
                            <i class="fas fa-trash"></i>
                        </span>
                        <span>Delete</span>
                    </button>
                    <button class="button modal-cancel-btn">Cancel</button>
                </footer>
            </div>
        `;
        
        document.body.appendChild(modal);
        
        // Event handlers
        const closeModal = () => modal.remove();
        
        // Close on background click
        modal.querySelector('.modal-background').addEventListener('click', closeModal);
        
        // Close on X button click
        modal.querySelector('.delete').addEventListener('click', closeModal);
        
        // Close on Cancel button click
        modal.querySelector('.modal-cancel-btn').addEventListener('click', closeModal);
        
        // Handle Delete button click
        modal.querySelector('.modal-delete-btn').addEventListener('click', async () => {
            closeModal();
            if (onConfirm) {
                await onConfirm(flowId);
            }
        });
        
        // ESC key to close
        const handleEsc = (e) => {
            if (e.key === 'Escape') {
                closeModal();
                document.removeEventListener('keydown', handleEsc);
            }
        };
        document.addEventListener('keydown', handleEsc);
        
        return modal;
    }
    
    /**
     * Show confirmation modal for flow execution
     * @param {string} mode - Execution mode ('real' or 'simulation')
     * @param {string} flowName - Flow name for display
     * @param {Function} onConfirm - Callback when user confirms execution
     */
    showExecuteFlowModal(mode, flowName, onConfirm) {
        const isRealExecution = mode === 'real';
        const modal = document.createElement('div');
        modal.className = 'modal is-active';
        modal.innerHTML = `
            <div class="modal-background"></div>
            <div class="modal-card">
                <header class="modal-card-head">
                    <p class="modal-card-title">${isRealExecution ? 'Execute Flow' : 'Test Run Flow'}</p>
                    <button class="delete" aria-label="close"></button>
                </header>
                <section class="modal-card-body">
                    <div class="notification ${isRealExecution ? 'is-warning' : 'is-info'}">
                        <p class="has-text-weight-semibold">
                            ${isRealExecution ? '⚠️ Real System Execution' : '🧪 Simulation Mode'}
                        </p>
                        <p class="mt-2">Flow: <strong>${flowName}</strong></p>
                        <p class="mt-2">
                            ${isRealExecution 
                                ? 'This will execute on the real system and affect actual resources.' 
                                : 'This is a simulation only. No real resources will be affected.'}
                        </p>
                        ${isRealExecution 
                            ? '<p class="mt-2 has-text-danger">Are you sure you want to proceed?</p>' 
                            : ''}
                    </div>
                </section>
                <footer class="modal-card-foot">
                    <button class="button ${isRealExecution ? 'is-danger' : 'is-primary'} modal-execute-btn">
                        <span class="icon">
                            <i class="fas ${isRealExecution ? 'fa-play-circle' : 'fa-vial'}"></i>
                        </span>
                        <span>${isRealExecution ? 'Execute' : 'Test Run'}</span>
                    </button>
                    <button class="button modal-cancel-btn">Cancel</button>
                </footer>
            </div>
        `;
        
        document.body.appendChild(modal);
        
        // Event handlers
        const closeModal = () => modal.remove();
        
        // Close on background click
        modal.querySelector('.modal-background').addEventListener('click', closeModal);
        
        // Close on X button click
        modal.querySelector('.delete').addEventListener('click', closeModal);
        
        // Close on Cancel button click
        modal.querySelector('.modal-cancel-btn').addEventListener('click', closeModal);
        
        // Handle Execute/Test Run button click
        modal.querySelector('.modal-execute-btn').addEventListener('click', async () => {
            closeModal();
            if (onConfirm) {
                await onConfirm();
            }
        });
        
        // ESC key to close
        const handleEsc = (e) => {
            if (e.key === 'Escape') {
                closeModal();
                document.removeEventListener('keydown', handleEsc);
            }
        };
        document.addEventListener('keydown', handleEsc);
        
        return modal;
    }
}

// 自動初始化
export const taflPanels = new TAFLPanels();
console.log('✅ TAFL Panels initialized - CSS for style, JS for function');