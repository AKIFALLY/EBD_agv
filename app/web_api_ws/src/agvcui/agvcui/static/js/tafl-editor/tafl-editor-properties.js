/**
 * TAFL Editor Properties Module
 * 事件驅動的屬性編輯核心引擎
 * 負責業務邏輯和內容生成，不直接操作 DOM
 */

import { taflFlowStore } from './tafl-editor-store.js';

class TAFLEditorProperties {
    constructor() {
        this.verbDefinitions = null;  // 將從主模組注入
        this.currentCardId = null;
        this.listeners = new Map();
        this.debouncedUpdater = new DebouncedUpdater(300);
        
        // 訂閱 Store 事件
        this.bindStoreEvents();
    }
    
    // ========================================
    // Store 事件訂閱
    // ========================================
    
    bindStoreEvents() {
        console.log('🎯 [INIT] Properties: Binding store events');
        // 響應選擇變化
        taflFlowStore.on('selection:changed', ({ cardId, cardData }) => {
            console.log('🎯 [STEP 15] Properties: Received selection:changed event for', cardId);
            this.handleSelectionChanged(cardId, cardData);
        });
        
        // 響應卡片更新
        taflFlowStore.on('card:updated', ({ cardId, cardData }) => {
            if (cardId === this.currentCardId) {
                console.log('📝 Properties: Current card updated', cardId);
                this.handleCardUpdated(cardData);
            }
        });
        
        // 響應屬性更新
        taflFlowStore.on('property:updated', ({ cardId, propertyPath, value }) => {
            if (cardId === this.currentCardId) {
                console.log('📝 Properties: Property updated', propertyPath, '=', value);
                this.handlePropertyUpdated(propertyPath, value);
            }
        });
        
        // 響應 Switch-Case 變更
        taflFlowStore.on('switch:case-added', ({ cardId, caseIndex }) => {
            if (cardId === this.currentCardId) {
                this.handleCaseAdded(caseIndex);
            }
        });
        
        taflFlowStore.on('switch:case-removed', ({ cardId, caseIndex }) => {
            if (cardId === this.currentCardId) {
                this.handleCaseRemoved(caseIndex);
            }
        });
    }
    
    // ========================================
    // 事件處理
    // ========================================
    
    handleSelectionChanged(cardId, cardData) {
        console.log('🎯 [STEP 16] Properties: handleSelectionChanged called');
        this.currentCardId = cardId;
        
        if (!cardId) {
            console.log('🎯 [STEP 16a] No cardId, emitting render:empty');
            this.emit('render:empty');
            return;
        }
        
        console.log('🎯 [STEP 17] Properties: Rendering property editor');
        const html = this.renderPropertyEditor(cardData);
        console.log('🎯 [STEP 18] Properties: HTML generated, length:', html?.length);
        
        console.log('🎯 [STEP 19] Properties: Emitting render:complete event');
        this.emit('render:complete', { 
            html, 
            cardData,
            cardId 
        });
        console.log('🎯 [STEP 20] Properties: render:complete emitted');
    }
    
    handleCardUpdated(cardData) {
        // 如果沒有 cardData，從 Store 獲取
        if (!cardData) {
            cardData = taflFlowStore.findCardById(this.currentCardId);
        }
        // 完整重繪
        const html = this.renderPropertyEditor(cardData);
        this.emit('render:update', { 
            html, 
            cardData,
            cardId: this.currentCardId 
        });
    }
    
    handlePropertyUpdated(propertyPath, value) {
        // 發出局部更新事件（UI可選擇性響應）
        this.emit('property:changed', { 
            propertyPath, 
            value,
            cardId: this.currentCardId 
        });
    }
    
    handleCaseAdded(caseIndex) {
        // 重繪 Switch 編輯器
        const cardData = taflFlowStore.findCardById(this.currentCardId);
        if (cardData) {
            this.handleCardUpdated(cardData);
        }
    }
    
    handleCaseRemoved(caseIndex) {
        // 重繪 Switch 編輯器
        const cardData = taflFlowStore.findCardById(this.currentCardId);
        if (cardData) {
            this.handleCardUpdated(cardData);
        }
    }
    
    // ========================================
    // 渲染方法（純函數，無副作用）
    // ========================================
    
    renderPropertyEditor(cardData) {
        console.log('🎯 [RENDER-1] renderPropertyEditor called with:', cardData);
        if (!cardData) {
            console.log('🎯 [RENDER-1a] No cardData, returning empty string');
            return '';
        }
        
        // Ensure verbDefinitions are loaded
        if (!this.verbDefinitions) {
            console.warn('🎯 [RENDER-2] verbDefinitions not set, attempting to load from store');
            this.verbDefinitions = taflFlowStore.getVerbDefinitions();
            console.log('🎯 [RENDER-2a] Loaded verbDefinitions:', this.verbDefinitions);
            if (!this.verbDefinitions) {
                console.warn('🎯 [RENDER-2b] Still no verbDefinitions, using empty object');
                this.verbDefinitions = {};
            }
        }
        
        const verb = this.getCardVerb(cardData);
        console.log('🎯 [RENDER-3] Found verb:', verb, 'for card:', cardData.id);
        if (!verb) {
            console.error('🎯 [RENDER-3a] No verb found for card:', cardData);
            return '<div class="has-text-danger">Invalid card data</div>';
        }
        
        // Switch 特殊處理
        if (verb === 'switch') {
            console.log('🎯 [RENDER-4] Rendering switch editor');
            return this.renderSwitchCasesEditor(cardData);
        }
        
        // 一般屬性編輯器
        console.log('🎯 [RENDER-5] Rendering standard editor for verb:', verb);
        const result = this.renderStandardEditor(cardData, verb);
        console.log('🎯 [RENDER-6] Generated HTML length:', result?.length);
        return result;
    }
    
    renderStandardEditor(cardData, verb) {
        console.log('🎯 [STD-1] renderStandardEditor called for verb:', verb);
        const params = cardData[verb];
        console.log('🎯 [STD-2] Card params:', params, 'type:', typeof params);
        const verbDef = this.verbDefinitions?.[verb];
        console.log('🎯 [STD-3] Verb definition:', verbDef);
        
        // Special handling for SET verb - support both single and multiple variables
        if (verb === 'set') {
            console.log('🎯 [STD-4] SET verb detected, using enhanced SET editor');
            return this.renderSetEditor(cardData, params);
        }
        
        // 處理參數 (TAFL 1.1.2: all verbs use object parameters)
        const objParams = params || {};
        
        let html = '<div class="property-editor">';
        
        // Add a nice title for the verb
        const verbTitle = this.getVerbTitle(verb);
        html += `<h4 class="title is-5 mb-4">${verbTitle}</h4>`;
        
        // Add description if available
        const verbDescription = this.getVerbDescription(verb);
        if (verbDescription) {
            html += `<div class="notification is-light mb-3">`;
            html += `<p class="is-size-7">${verbDescription}</p>`;
            html += `</div>`;
        }
        
        console.log('🎯 [STD-5] Starting HTML generation for object params');
        
        // Get all available parameters for this verb
        const availableParams = verbDef?.params || [];
        console.log('🎯 [STD-5a] Available params for verb:', availableParams);
        
        // 優先顯示的參數 (TAFL v1.1.1 compliant)
        const priorityParams = ['target', 'condition', 'expression', 'level', 'where', 'with', 'set', 'in', 'as'];
        const renderedParams = new Set();
        
        // 先渲染優先參數（如果它們在可用參數列表中）
        console.log('🎯 [STD-6] Rendering priority params:', priorityParams);
        for (const key of priorityParams) {
            if (availableParams.includes(key)) {
                // Look for value in objParams first, then in cardData directly
                const value = objParams[key] || cardData[key] || '';
                console.log('🎯 [STD-6a] Rendering priority param:', key, '=', value);
                html += this.renderPropertyField(cardData.id, verb, key, value);
                renderedParams.add(key);
            }
        }
        
        // 渲染其他可用參數
        console.log('🎯 [STD-7] Rendering other available params');
        for (const key of availableParams) {
            if (renderedParams.has(key)) continue;
            if (this.isInternalProperty(key)) continue;
            
            // Look for value in objParams first, then in cardData directly
            const value = objParams[key] || cardData[key] || '';
            console.log('🎯 [STD-7a] Rendering param:', key, '=', value);
            console.log('🎯 [STD-7a-pre] About to call renderPropertyField for:', key);
            const fieldHtml = this.renderPropertyField(cardData.id, verb, key, value);
            console.log('🎯 [STD-7a-post] renderPropertyField returned HTML length:', fieldHtml.length);
            html += fieldHtml;
            renderedParams.add(key);
        }
        
        // Also render any extra params that are in the data but not in the definition
        // Check both objParams and cardData for extra params
        const allParams = { ...objParams };
        
        // For Set verb (TAFL v1.1.1: single expression, not object)
        if (verb === 'set' && typeof params === 'object') {
            // Legacy format - migrate to single expression
            for (const key of ['variable', 'value', 'expression']) {
                if (cardData[key] !== undefined && !allParams[key]) {
                    allParams[key] = cardData[key];
                }
            }
        }
        
        // 只有當 allParams 是物件時才遍歷
        if (typeof allParams === 'object' && allParams !== null) {
            for (const [key, value] of Object.entries(allParams)) {
                if (renderedParams.has(key)) continue;
                if (this.isInternalProperty(key)) continue;
                
                console.log('🎯 [STD-7b] Rendering extra param:', key, '=', value);
                html += this.renderPropertyField(cardData.id, verb, key, value);
            }
        }
        
        html += '</div>';
        console.log('🎯 [STD-8] Final HTML length:', html.length);
        return html;
    }
    
    renderSetEditor(cardData, params) {
        const cardId = cardData.id;

        // Always convert to object format if it's a string
        let variables = {};
        if (typeof params === 'string') {
            // Try to parse single expression into key-value
            const match = params.match(/^\s*([^=]+?)\s*=\s*(.+)\s*$/);
            if (match) {
                const [, key, value] = match;
                variables = { [key.trim()]: value.trim() };
            } else {
                // Default if can't parse
                variables = { variable1: params || 'value' };
            }
        } else if (typeof params === 'object' && params !== null && !Array.isArray(params)) {
            variables = params;
        }

        let html = '<div class="property-editor">';

        html += '<h4 class="title is-5 mb-4">Variable Assignments</h4>';
        html += '<p class="is-size-7 has-text-grey mb-3">Define one or more variable assignments. Each assignment consists of a variable name and its value.</p>';
        html += '<div class="notification is-info is-light mb-3">';
        html += '<p class="is-size-7"><strong>Examples:</strong></p>';
        html += '<ul class="is-size-7">';
        html += '<li><code>simple_string: "Hello TAFL"</code></li>';
        html += '<li><code>calculated: ${initial_value * 2}</code></li>';
        html += '<li><code>array_data: [1, 2, 3]</code></li>';
        html += '<li><code>object_data: {key: "value"}</code></li>';
        html += '</ul>';
        html += '</div>';
        
        // Add Variable button at the top (like in Variables/Preload/Rules panels)
        html += `
            <button class="button is-small is-primary mb-3" 
                    onclick="window.taflEditorProperties.addSetVariable('${cardId}')">
                <span class="icon"><i class="fas fa-plus"></i></span>
                <span>Add Variable</span>
            </button>
        `;
        
        // Variables container
        html += `<div class="set-variables-container" id="set-vars-${cardId}">`;
        
        // Render existing variables
        const entries = Object.entries(variables);
        
        if (entries.length === 0) {
            html += '<p class="is-size-7 has-text-grey">No variables defined. Click "Add Variable" to start.</p>';
        } else {
            entries.forEach(([key, value], index) => {
                html += this.renderSetVariable(cardId, key, value, index);
            });
        }
        
        html += '</div>';  // Close set-variables-container
        html += '</div>';  // Close property-editor
        return html;
    }
    
    renderSetVariable(cardId, key, value, index) {
        const keyId = `prop-${cardId}-set-key-${index}`;
        const valueId = `prop-${cardId}-set-value-${index}`;
        
        return `
            <div class="field is-grouped set-variable-item" data-index="${index}">
                <div class="control is-expanded">
                    <input 
                        id="${keyId}"
                        class="input is-small property-input set-var-key" 
                        type="text"
                        data-card-id="${cardId}"
                        data-verb="set"
                        data-property="set.${key}"
                        data-var-index="${index}"
                        data-var-type="key"
                        value="${this.escapeHtml(key)}"
                        placeholder="e.g., counter, status, result"
                    />
                </div>
                <div class="control">
                    <span class="button is-static is-small">=</span>
                </div>
                <div class="control is-expanded">
                    <input 
                        id="${valueId}"
                        class="input is-small property-input set-var-value" 
                        type="text"
                        data-card-id="${cardId}"
                        data-verb="set"
                        data-property="set.${key}"
                        data-var-index="${index}"
                        data-var-type="value"
                        data-var-key="${key}"
                        value="${this.escapeHtml(String(value || ''))}"
                        placeholder='e.g., 42, "active", $\{counter + 1}'
                    />
                </div>
                <div class="control">
                    <button class="button is-small is-danger" 
                            onclick="window.taflEditorProperties.removeSetVariable('${cardId}', '${key}')">
                        <span class="icon"><i class="fas fa-trash"></i></span>
                    </button>
                </div>
            </div>
        `;
    }

    
    renderPropertyField(cardId, verb, key, value) {
        console.log('📋 renderPropertyField called:', { cardId, verb, key, value, valueType: typeof value });
        const fieldId = `prop-${cardId}-${key}`;
        const paramHelp = this.getParamHelp(verb, key);
        
        // 根據參數類型渲染不同的輸入元件
        let inputHTML = this.renderInputByType(fieldId, cardId, verb, key, value);
        console.log('📋 renderPropertyField inputHTML length:', inputHTML.length);
        
        return `
            <div class="field mb-4">
                <label class="label is-size-6" for="${fieldId}">
                    ${this.formatParamName(key)}
                </label>
                <div class="control">
                    ${inputHTML}
                </div>
                ${paramHelp ? `<p class="is-size-7 has-text-grey mt-1">${paramHelp}</p>` : ''}
            </div>
        `;
    }
    
    renderInputByType(fieldId, cardId, verb, key, value) {
        const paramType = this.getParamType(verb, key);

        // Debug logging
        console.log('🔍 renderInputByType called:', {
            fieldId,
            cardId,
            verb,
            key,
            value,
            valueType: typeof value,
            paramType,
            isObject: value !== null && typeof value === 'object'
        });

        // Get placeholder examples based on verb and key
        const placeholderExamples = this.getPlaceholderExample(verb, key);
        
        // Check if value is an object or array that needs special handling
        const isComplexValue = value !== null && typeof value === 'object';
        
        // For complex values (objects/arrays), always use multiline JSON editor
        if (isComplexValue && paramType !== 'select') {
            console.log('📝 Rendering JSON editor for:', key, value);
            const jsonValue = JSON.stringify(value, null, 2);
            return `
                <div class="json-editor-container">
                    <textarea 
                        id="${fieldId}"
                        class="textarea property-input json-property"
                        rows="6"
                        data-card-id="${cardId}"
                        data-verb="${verb}"
                        data-property="${key}"
                        data-type="json"
                        placeholder='${this.getJsonPlaceholder(verb, key)}'
                    >${this.escapeHtml(jsonValue)}</textarea>
                </div>
                <p class="help is-info">
                    <span class="icon is-small">
                        <i class="fas fa-info-circle"></i>
                    </span>
                    <span>JSON format - Edit as needed</span>
                </p>
            `;
        }
        
        switch (paramType) {
            case 'boolean':
                return `
                    <label class="checkbox">
                        <input 
                            id="${fieldId}"
                            type="checkbox"
                            class="property-input"
                            data-card-id="${cardId}"
                            data-verb="${verb}"
                            data-property="${key}"
                            ${value ? 'checked' : ''}
                        />
                        ${this.formatParamName(key)}
                    </label>
                `;
                
            case 'number':
                return `
                    <input 
                        id="${fieldId}"
                        class="input property-input" 
                        type="number"
                        data-card-id="${cardId}"
                        data-verb="${verb}"
                        data-property="${key}"
                        value="${this.escapeHtml(String(value || 0))}"
                    />
                `;
                
            case 'select':
                const options = this.getParamOptions(verb, key);
                return `
                    <div class="select">
                        <select 
                            id="${fieldId}"
                            class="property-input"
                            data-card-id="${cardId}"
                            data-verb="${verb}"
                            data-property="${key}"
                        >
                            ${options.map(opt => `
                                <option value="${opt.value}" ${value === opt.value ? 'selected' : ''}>
                                    ${opt.label}
                                </option>
                            `).join('')}
                        </select>
                    </div>
                `;
                
            case 'multiline':
                return `
                    <textarea 
                        id="${fieldId}"
                        class="textarea property-input"
                        rows="3"
                        data-card-id="${cardId}"
                        data-verb="${verb}"
                        data-property="${key}"
                        placeholder="${placeholderExamples || ''}"
                    >${this.escapeHtml(String(value || ''))}</textarea>
                `;
                
            case 'json':
                // Explicitly handle JSON type
                const jsonStr = typeof value === 'object' ? JSON.stringify(value, null, 2) : String(value || '{}');
                return `
                    <div class="json-editor-container">
                        <textarea 
                            id="${fieldId}"
                            class="textarea property-input json-property"
                            rows="6"
                            data-card-id="${cardId}"
                            data-verb="${verb}"
                            data-property="${key}"
                            data-type="json"
                            placeholder='${this.getJsonPlaceholder(verb, key)}'
                        >${this.escapeHtml(jsonStr)}</textarea>
                    </div>
                    <p class="help is-info">
                        <span class="icon is-small">
                            <i class="fas fa-info-circle"></i>
                        </span>
                        <span>JSON format - Edit as needed</span>
                    </p>
                `;
                
            default:
                return `
                    <input
                        id="${fieldId}"
                        class="input property-input"
                        type="text"
                        data-card-id="${cardId}"
                        data-verb="${verb}"
                        data-property="${key}"
                        value="${this.escapeHtml(String(value || ''))}"
                        placeholder="${placeholderExamples || ''}"
                    />
                `;
        }
    }
    
    renderSwitchCasesEditor(cardData) {
        const switchData = cardData.switch || {};
        const cases = switchData.cases || [];
        const cardId = cardData.id;
        
        let html = `
            <div class="property-editor">
                <h4 class="title is-5 mb-4">Switch Configuration</h4>
                <div class="notification is-light mb-3">
                    <p class="is-size-7">Branch execution based on the value of an expression. Each case matches a specific value, with an optional default case for unmatched values.</p>
                    <p class="is-size-7 mt-2"><strong>Example expressions:</strong> <code>\${task_type}</code>, <code>\${status}</code>, <code>\${priority}</code></p>
                </div>
                
                <!-- Expression field -->
                <div class="field mb-4">
                    <label class="label is-size-6">Expression</label>
                    <div class="control">
                        <input class="input property-input" 
                               type="text" 
                               data-card-id="${cardId}" 
                               data-verb="switch"
                               data-property="switch.expression"
                               value="${this.escapeHtml(switchData.expression || '')}"
                               placeholder="Examples: \${task_type}, \${status}, \${priority}">
                    </div>
                    <p class="is-size-7 has-text-grey mt-1">The variable or expression to evaluate. Use \${} syntax for variables.</p>
                </div>
                
                <hr class="my-3">

                <h5 class="title is-6 mb-3">Cases</h5>
                <div class="switch-cases">
        `;
        
        // 渲染每個 Case (TAFL v1.1.1: 包括普通 cases 和 when: "default" 的 case)
        cases.forEach((caseData, index) => {
            const isDefault = caseData.when === 'default';
            const normalCaseCount = cases.filter((c, i) => i < index && c.when !== 'default').length;
            const stepCount = (caseData.do || caseData.then || []).length;

            html += `
                <div class="case-item box p-3 mb-3${isDefault ? ' default-case-box' : ''}" data-case-index="${index}">
                    <!-- Header row with case label and buttons -->
                    <div class="level is-mobile mb-2">
                        <div class="level-left">
                            <span class="tag ${isDefault ? 'is-warning' : 'is-info'}">
                                ${isDefault ? 'Default' : `Case ${normalCaseCount + 1}`}
                            </span>
                        </div>
                        <div class="level-right">
                            <div class="buttons has-addons">
                                ${!isDefault && index > 0 ? `
                                    <button class="button is-small move-case-up"
                                            data-card-id="${cardId}"
                                            data-case-index="${index}"
                                            title="Move up">
                                        <span class="icon is-small"><i class="fas fa-arrow-up"></i></span>
                                    </button>
                                ` : ''}
                                ${!isDefault && index < cases.length - 1 && cases[index + 1].when !== 'default' ? `
                                    <button class="button is-small move-case-down"
                                            data-card-id="${cardId}"
                                            data-case-index="${index}"
                                            title="Move down">
                                        <span class="icon is-small"><i class="fas fa-arrow-down"></i></span>
                                    </button>
                                ` : ''}
                                <button class="button is-small is-danger ${isDefault ? 'remove-default-btn' : 'remove-case-btn'}"
                                        data-card-id="${cardId}"
                                        ${!isDefault ? `data-case-index="${index}"` : ''}
                                        title="Remove case">
                                    <span class="icon is-small"><i class="fas fa-times"></i></span>
                                </button>
                            </div>
                        </div>
                    </div>

                    ${!isDefault ? `
                        <!-- When field -->
                        <div class="field has-addons mb-1">
                            <div class="control">
                                <a class="button is-static is-small">When</a>
                            </div>
                            <div class="control is-expanded">
                                <input class="input is-small property-input case-when"
                                       type="text"
                                       data-card-id="${cardId}"
                                       data-property="switch.cases.${index}.when"
                                       value="${this.escapeHtml(caseData.when || '')}"
                                       placeholder="Value to match (e.g., \"A\", \"active\", 1)"
                                />
                            </div>
                        </div>
                    ` : ''}

                    <!-- Do/Actions field -->
                    <div class="field has-addons mb-0">
                        <div class="control">
                            <a class="button is-static is-small">${isDefault ? 'Actions' : 'Do'}</a>
                        </div>
                        <div class="control">
                            <a class="button is-static is-small">
                                <span class="${stepCount === 0 ? 'has-text-grey' : 'has-text-info'}">
                                    ${stepCount} ${stepCount === 1 ? 'step' : 'steps'}
                                </span>
                            </a>
                        </div>
                    </div>
                </div>
            `;
        });
        
        // 舊版 Default case (向下相容)
        // 如果有舊的 default 欄位且 cases 陣列中沒有 when: "default"，顯示遷移提示
        if (switchData.default && !cases.some(c => c.when === 'default')) {
            const legacyStepCount = switchData.default?.length || 0;
            html += `
                <div class="notification is-warning is-light mb-3">
                    <p class="is-size-7">⚠️ Legacy default format detected. Click "Add Default" to migrate to TAFL v1.1.1 format.</p>
                </div>
                <div class="case-item box p-3 mb-3 default-case-box">
                    <div class="level is-mobile mb-2">
                        <div class="level-left">
                            <span class="tag is-warning">Default (Legacy)</span>
                        </div>
                        <div class="level-right">
                            <button class="button is-small is-danger remove-default-btn"
                                    data-card-id="${cardId}"
                                    title="Remove default">
                                <span class="icon is-small"><i class="fas fa-times"></i></span>
                            </button>
                        </div>
                    </div>
                    <div class="field has-addons mb-0">
                        <div class="control">
                            <a class="button is-static is-small">Actions</a>
                        </div>
                        <div class="control">
                            <a class="button is-static is-small">
                                <span class="${legacyStepCount === 0 ? 'has-text-grey' : 'has-text-info'}">
                                    ${legacyStepCount} ${legacyStepCount === 1 ? 'step' : 'steps'}
                                </span>
                            </a>
                        </div>
                    </div>
                </div>
            `;
        }
        
        html += `
                </div>
                
                <div class="field is-grouped">
                    <button class="button is-small is-primary add-case-btn" 
                            data-card-id="${cardId}">
                        <span class="icon"><i class="fas fa-plus"></i></span>
                        <span>Add Case</span>
                    </button>
                    ${!cases.some(c => c.when === 'default') && !switchData.default ? `
                        <button class="button is-small add-default-btn" 
                                data-card-id="${cardId}">
                            <span class="icon"><i class="fas fa-plus"></i></span>
                            <span>Add Default</span>
                        </button>
                    ` : ''}
                </div>
            </div>
        `;
        
        return html;
    }
    
    // ========================================
    // SET Verb Special Methods
    // ========================================
    
    // This function is kept for backward compatibility but always converts to object format
    switchSetMode(cardId, mode) {
        console.log('🔄 Converting SET card to object format for card:', cardId);
        const cardData = taflFlowStore.findCardById(cardId);
        if (!cardData) return;
        
        const currentParams = cardData.set;
        let newParams;
        
        // Always convert to object format
        if (typeof currentParams === 'string') {
            // Try to parse single expression into key-value
            const match = currentParams.match(/^\s*([^=]+?)\s*=\s*(.+)\s*$/);
            if (match) {
                const [, key, value] = match;
                newParams = { [key.trim()]: value.trim() };
            } else {
                // Can't parse, use as-is
                newParams = { variable1: currentParams || 'value' };
            }
        } else if (typeof currentParams === 'object' && currentParams !== null) {
            // Already in object format
            newParams = currentParams;
        } else {
            // Default
            newParams = { variable1: 'value' };
        }
        
        // Update the card if format changed
        if (typeof currentParams === 'string' || !currentParams) {
            taflFlowStore.updateCard(cardId, {
                ...cardData,
                set: newParams
            });
        }
        
        // Refresh the properties panel
        const html = this.renderPropertyEditor(cardData);
        if (window.taflPanelsProperties) {
            window.taflPanelsProperties.updatePanel(html, cardId);
        }
    }
    
    addSetVariable(cardId) {
        console.log('➕ Adding SET variable for card:', cardId);
        const cardData = taflFlowStore.findCardById(cardId);
        if (!cardData) return;
        
        let params = cardData.set;
        
        // Ensure params is an object
        if (typeof params !== 'object' || params === null) {
            params = {};
        }
        
        // Find a unique variable name
        let varName = 'new_variable';
        let counter = 1;
        while (params[varName]) {
            varName = `new_variable_${counter}`;
            counter++;
        }
        
        // Add the new variable
        params[varName] = '';
        
        // Update the card
        taflFlowStore.updateCard(cardId, {
            ...cardData,
            set: params
        });
        
        // Refresh the properties panel
        const updatedCard = taflFlowStore.findCardById(cardId);
        const html = this.renderPropertyEditor(updatedCard);
        if (window.taflPanelsProperties) {
            window.taflPanelsProperties.updatePanel(html, cardId);
        }
    }
    
    removeSetVariable(cardId, key) {
        console.log('🗑️ Removing SET variable:', key, 'from card:', cardId);
        const cardData = taflFlowStore.findCardById(cardId);
        if (!cardData || typeof cardData.set !== 'object') return;
        
        const params = { ...cardData.set };
        delete params[key];
        
        // Update the card
        taflFlowStore.updateCard(cardId, {
            ...cardData,
            set: params
        });
        
        // Refresh the properties panel
        const updatedCard = taflFlowStore.findCardById(cardId);
        const html = this.renderPropertyEditor(updatedCard);
        if (window.taflPanelsProperties) {
            window.taflPanelsProperties.updatePanel(html, cardId);
        }
    }
    
    // ========================================
    // 資料更新方法（更新Store）
    // ========================================
    
    updateProperty(cardId, propertyPath, value, verb = null) {
        console.log('🎯 updateProperty called:', { cardId, propertyPath, value, verb });
        
        // 特殊處理 _root 屬性（用於簡單字串動詞如 set, stop）
        if (propertyPath === '_root') {
            // 取得卡片資料以獲取動詞
            const cardData = taflFlowStore.findCardById(cardId);
            if (cardData) {
                const actualVerb = verb || this.getCardVerb(cardData);
                if (actualVerb) {
                    console.log('🎯 Updating simple string verb:', actualVerb, 'with value:', value);
                    // 直接更新動詞的值（字串形式）
                    this.debouncedUpdater.update(`${cardId}-${actualVerb}`, () => {
                        taflFlowStore.updateCard(cardId, {
                            ...cardData,
                            [actualVerb]: value  // 直接設定字串值
                        });
                    });
                    return true;
                }
            }
            return false;
        }
        
        // 一般屬性處理
        // 類型轉換
        const processedValue = this.processPropertyValue(propertyPath, value);
        
        // 驗證
        if (!this.validateProperty(propertyPath, processedValue)) {
            this.emit('validation:failed', { 
                propertyPath, 
                value: processedValue,
                reason: 'Invalid value' 
            });
            return false;
        }
        
        console.log('🎯 Updating property path:', propertyPath, 'with value:', processedValue);
        
        // 檢查是否需要保持物件結構
        const cardData = taflFlowStore.findCardById(cardId);
        const currentVerb = verb || this.getCardVerb(cardData);
        
        // Debounced 更新
        this.debouncedUpdater.update(`${cardId}-${propertyPath}`, () => {
            // 對於有結構化參數的動詞（物件形式），屬性應該在動詞物件內
            if (currentVerb && typeof cardData[currentVerb] === 'object') {
                // 屬性路徑應該是 verb.property (例如 query.limit)
                const fullPath = `${currentVerb}.${propertyPath}`;
                console.log('🎯 Updating nested property:', fullPath, 'with value:', processedValue);
                taflFlowStore.updateCardProperty(cardId, fullPath, processedValue);
                
                // Clean up any duplicate property at root level if it exists
                if (cardData[propertyPath] !== undefined && propertyPath !== 'id') {
                    console.log('🧹 Cleaning up duplicate property at root:', propertyPath);
                    const cleanedCard = { ...cardData };
                    delete cleanedCard[propertyPath];
                    taflFlowStore.updateCard(cardId, cleanedCard);
                }
            } else {
                // 簡單動詞或頂層屬性
                taflFlowStore.updateCardProperty(cardId, propertyPath, processedValue);
            }
        });
        
        return true;
    }
    
    updatePropertyImmediate(cardId, propertyPath, value) {
        const processedValue = this.processPropertyValue(propertyPath, value);
        taflFlowStore.updateCardProperty(cardId, propertyPath, processedValue);
    }
    
    addSwitchCase(cardId) {
        taflFlowStore.addSwitchCase(cardId);
    }
    
    removeSwitchCase(cardId, caseIndex) {
        taflFlowStore.removeSwitchCase(cardId, caseIndex);
    }
    
    moveSwitchCase(cardId, fromIndex, direction) {
        const toIndex = direction === 'up' ? fromIndex - 1 : fromIndex + 1;
        taflFlowStore.moveSwitchCase(cardId, fromIndex, toIndex);
    }
    
    toggleDefault(cardId, add = true) {
        const card = taflFlowStore.findCardById(cardId);
        if (!card?.switch) return;
        
        if (add) {
            taflFlowStore.updateCard(cardId, {
                ...card,
                switch: {
                    ...card.switch,
                    default: []
                }
            });
        } else {
            const { default: _, ...switchWithoutDefault } = card.switch;
            taflFlowStore.updateCard(cardId, {
                ...card,
                switch: switchWithoutDefault
            });
        }
    }
    
    // ========================================
    // 輔助方法
    // ========================================
    
    getCardVerb(cardData) {
        console.log('🎯 [VERB-1] getCardVerb called with:', cardData);
        if (!cardData) {
            console.log('🎯 [VERB-1a] No cardData, returning null');
            return null;
        }
        
        console.log('🎯 [VERB-2] Card keys:', Object.keys(cardData));
        console.log('🎯 [VERB-3] verbDefinitions available:', !!this.verbDefinitions);
        
        // Use verbDefinitions if available (preferred method)
        if (this.verbDefinitions) {
            console.log('🎯 [VERB-4] Available verb definitions:', Object.keys(this.verbDefinitions));
            const foundVerb = Object.keys(cardData).find(key => {
                const hasVerb = this.verbDefinitions[key] !== undefined;
                console.log('🎯 [VERB-4a] Checking key:', key, 'is verb:', hasVerb);
                return hasVerb;
            });
            console.log('🎯 [VERB-5] Found verb from definitions:', foundVerb);
            return foundVerb;
        }
        
        // Fallback: check for known TAFL verbs
        console.log('🎯 [VERB-6] Using fallback verb check');
        const knownVerbs = [
            'query', 'check', 'create', 'update', 'delete', 
            'if', 'for', 'while', 'switch', 'set', 'get',
            'notify', 'wait', 'retry', 'parallel', 'move',
            'assign', 'call', 'return', 'break', 'continue'
        ];
        const fallbackVerb = Object.keys(cardData).find(key => knownVerbs.includes(key));
        console.log('🎯 [VERB-7] Found verb from fallback:', fallbackVerb);
        return fallbackVerb;
    }
    
    formatParamName(name) {
        return name
            .replace(/_/g, ' ')
            .replace(/\b\w/g, l => l.toUpperCase());
    }
    
    getVerbTitle(verb) {
        // Format verb name nicely
        const titles = {
            'query': 'Query Configuration',
            'check': 'Check Condition',
            'create': 'Create Action',
            'update': 'Update Action',
            'delete': 'Delete Action',
            'call': 'Call Service',
            'wait': 'Wait Configuration',
            'stop': 'Stop Configuration',
            'for': 'For Loop Configuration',
            'while': 'While Loop Configuration',
            'switch': 'Switch Configuration'
        };
        return titles[verb] || this.formatParamName(verb) + ' Configuration';
    }
    
    getVerbDescription(verb) {
        const descriptions = {
            'query': 'Query data from database tables like locations, racks, tasks, carriers. Store results in a variable for later use.',
            'check': 'Evaluate a condition and store the boolean result. Use ${} syntax for variables and expressions.',
            'create': 'Create a new resource (task, rack, carrier) with specified parameters. Returns the created object.',
            'update': 'Update existing records based on conditions. Use where to filter and set to specify new values.',
            'delete': 'Delete an entity or resource from the system',
            'call': 'Call an external service or function',
            'wait': 'Wait for a specified condition or duration',
            'stop': 'Stop the flow execution with optional reason and condition',
            'for': 'Loop through arrays or collections. Access each item with the loop variable.',
            'while': 'Loop while a condition is true',
            'switch': 'Branch execution based on expression value. Include default case for unmatched values.',
            'set': 'Assign values to variables. Supports strings, numbers, arrays, objects, and expressions.',
            'if': 'Conditional execution with then/else branches. Supports complex boolean expressions.',
            'notify': 'Send notifications with different severity levels (info/warning/error).'
        };
        return descriptions[verb] || '';
    }
    
    getParamHelp(verb, param) {
        // Provide detailed help with examples for each parameter
        const helpTexts = {
            query: {
                target: 'The table/resource to query. Examples: locations, racks, tasks, carriers, works',
                where: 'Filter conditions as object. Example: {type: "room_outlet", status_id: 1}',
                limit: 'Maximum number of results. Example: 10',
                order: 'Sort order field. Examples: id, created_at, priority',
                as: 'Variable name to store results. Example: outlet_locations'
            },
            check: {
                condition: 'Expression to evaluate. Examples: ${counter > threshold}, ${status == "active"}',
                target: 'Optional target to check. Example: battery_level, status',
                as: 'Variable to store boolean result. Example: is_valid, is_ready'
            },
            create: {
                target: 'Resource type to create. Examples: task, rack, carrier',
                with: 'Object with creation parameters. Example: {work_id: 100001, name: "Test Task", priority: 5}',
                as: 'Optional variable to store created resource. Example: new_task'
            },
            update: {
                target: 'Resource type to update. Examples: rack, task, carrier',
                where: 'Conditions to find records. Example: {id: ${rack_id}}',
                set: 'Fields to update. Example: {status_id: 2, is_carry: 1, updated_at: ${now()}}'
            },
            if: {
                condition: 'Boolean expression. Examples: ${a_side_full && b_side_empty}, ${count > 0}',
                then: 'Array of steps to execute if true',
                else: 'Array of steps to execute if false'
            },
            for: {
                in: 'Array or variable to iterate. Examples: ${outlet_locations}, [1, 2, 3], ${items}',
                as: 'Loop variable name. Examples: location, item, i',
                filter: 'Optional filter expression. Example: ${item.value >= 20}',
                do: 'Array of steps to execute in each iteration'
            },
            switch: {
                expression: 'Value to switch on. Examples: ${task_type}, ${status}, ${priority}',
                cases: 'Array of cases with when and do properties',
                default: 'Optional default case steps'
            },
            set: {
                _info: 'Define one or more variables. Examples:\n• simple_string: "Hello TAFL"\n• calculated: ${initial_value * 2}\n• array_data: [1, 2, 3]\n• object_data: {key: "value"}'
            },
            notify: {
                level: 'Message level: info, warning, or error',
                message: 'Notification text. Example: "Task ${task_id} completed"',
                recipients: 'Optional array of recipients. Example: ["admin", "operator"]',
                details: 'Optional additional data. Example: {task_id: 123, status: "done"}'
            },
            stop: {
                reason: 'Reason for stopping. Example: "Task completed successfully"',
                condition: 'Optional condition to stop. Example: ${error_count == 0}'
            }
        };

        const verbHelp = helpTexts[verb];
        if (!verbHelp) return '';

        // Special case for set verb
        if (verb === 'set' && param === '_info') {
            return verbHelp._info;
        }

        return verbHelp[param] || '';
    }
    
    getParamType(verb, param) {
        const verbDef = this.verbDefinitions?.[verb];
        const paramDef = verbDef?.params?.[param];
        return paramDef?.type || 'text';
    }
    
    getParamOptions(verb, param) {
        const verbDef = this.verbDefinitions?.[verb];
        const paramDef = verbDef?.params?.[param];
        return paramDef?.options || [];
    }
    
    processPropertyValue(propertyPath, value) {
        // 處理 checkbox
        if (typeof value === 'boolean') {
            return value;
        }
        
        // 處理數字
        if (!isNaN(value) && value !== '') {
            return Number(value);
        }
        
        // 處理空字串
        if (value === '') {
            return '';
        }
        
        return value;
    }
    
    validateProperty(propertyPath, value) {
        // 基本驗證
        // 可以根據 propertyPath 加入更多驗證規則
        return true;
    }
    
    isInternalProperty(key) {
        return ['then', 'else', 'do', 'cases', 'default'].includes(key);
    }
    
    escapeHtml(text) {
        const div = document.createElement('div');
        div.textContent = text;
        return div.innerHTML;
    }

    getPlaceholderExample(verb, key) {
        const examples = {
            query: {
                target: 'e.g., locations, racks, tasks',
                limit: 'e.g., 10',
                order: 'e.g., id, created_at',
                as: 'e.g., outlet_locations'
            },
            check: {
                condition: 'e.g., ${counter > 5}',
                target: 'e.g., battery_level',
                as: 'e.g., is_valid'
            },
            create: {
                target: 'e.g., task, rack',
                as: 'e.g., new_task'
            },
            update: {
                target: 'e.g., rack, task'
            },
            notify: {
                level: 'info, warning, or error',
                message: 'e.g., Task ${task_id} completed'
            },
            stop: {
                reason: 'e.g., Task completed successfully',
                condition: 'e.g., ${error_count == 0}'
            },
            for: {
                in: 'e.g., ${outlet_locations}',
                as: 'e.g., location',
                filter: 'e.g., ${item.value >= 20}'
            },
            switch: {
                expression: 'e.g., ${task_type}'
            }
        };

        const verbExamples = examples[verb];
        return verbExamples ? verbExamples[key] || '' : '';
    }

    getJsonPlaceholder(verb, key) {
        const jsonExamples = {
            query: {
                where: '{\"type\": \"room_outlet\", \"status_id\": 1}'
            },
            create: {
                with: '{\"work_id\": 100001, \"name\": \"Test Task\", \"priority\": 5}'
            },
            update: {
                where: '{\"id\": \"${rack_id}\"}',
                set: '{\"status_id\": 2, \"is_carry\": 1}'
            },
            notify: {
                details: '{\"task_id\": 123, \"status\": \"done\"}'
            }
        };

        const verbExamples = jsonExamples[verb];
        return verbExamples ? verbExamples[key] || '{\"key\": \"value\"}' : '{\"key\": \"value\"}';
    }
    
    // ========================================
    // 事件系統
    // ========================================
    
    on(event, callback) {
        if (!this.listeners.has(event)) {
            this.listeners.set(event, []);
        }
        this.listeners.get(event).push(callback);
    }
    
    off(event, callback) {
        const callbacks = this.listeners.get(event);
        if (callbacks) {
            const index = callbacks.indexOf(callback);
            if (index > -1) {
                callbacks.splice(index, 1);
            }
        }
    }
    
    emit(event, data) {
        const callbacks = this.listeners.get(event) || [];
        callbacks.forEach(cb => cb(data));
    }
}

// ========================================
// 輔助類
// ========================================

class DebouncedUpdater {
    constructor(delay = 300) {
        this.timers = new Map();
        this.delay = delay;
    }
    
    update(key, callback) {
        clearTimeout(this.timers.get(key));
        this.timers.set(key, setTimeout(() => {
            callback();
            this.timers.delete(key);
        }, this.delay));
    }
    
    cancel(key) {
        clearTimeout(this.timers.get(key));
        this.timers.delete(key);
    }
    
    cancelAll() {
        this.timers.forEach(timer => clearTimeout(timer));
        this.timers.clear();
    }
}

// 單例導出
const taflEditorProperties = new TAFLEditorProperties();
export default taflEditorProperties;
export { TAFLEditorProperties };
