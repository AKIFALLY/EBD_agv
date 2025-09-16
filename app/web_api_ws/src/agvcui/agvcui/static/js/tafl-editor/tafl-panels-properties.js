/**
 * TAFL Panels Properties Module
 * 響應事件的屬性面板 UI 管理
 * 負責 DOM 操作和使用者互動，不處理業務邏輯
 */

import { taflFlowStore } from './tafl-editor-store.js';
import taflEditorProperties from './tafl-editor-properties.js';
import taflEventBus from './tafl-event-bus.js';

class TAFLPanelsProperties {
    constructor() {
        this.panel = null;
        this.content = null;
        this.currentCardId = null;
        this.isInitialized = false;
        this.codeMirrorEditors = []; // Store CodeMirror instances for JSON editors
        
        // 事件處理器綁定
        this.handleInputChange = this.handleInputChange.bind(this);
        this.handleSwitchCaseAdd = this.handleSwitchCaseAdd.bind(this);
        this.handleSwitchCaseRemove = this.handleSwitchCaseRemove.bind(this);
        this.handleSwitchCaseAddDefault = this.handleSwitchCaseAddDefault.bind(this);
        this.handleSwitchCaseRemoveDefault = this.handleSwitchCaseRemoveDefault.bind(this);
        
        // 延遲初始化 - 等待 DOM 準備好
        if (document.readyState === 'loading') {
            document.addEventListener('DOMContentLoaded', () => this.init());
        } else {
            // DOM 已經準備好，但可能元素還沒有創建，延遲一下
            setTimeout(() => this.init(), 0);
        }
    }
    
    // ========================================
    // 初始化
    // ========================================
    
    init() {
        // 防止重複初始化
        if (this.isInitialized) {
            return;
        }
        
        this.panel = document.getElementById('properties-panel');
        // In TAFL Editor, properties-panel is both the container and content area
        this.content = this.panel;  // Use the same element for both
        
        if (!this.panel) {
            // 如果元素還不存在，稍後重試
            setTimeout(() => this.init(), 100);
            return;
        }
        
        // 訂閱 Properties Module 事件
        this.bindPropertyEvents();
        
        // 訂閱 Store 事件（直接監聽選擇變化）
        this.bindStoreEvents();
        
        this.isInitialized = true;
    }
    
    /**
     * 公開的重新初始化方法
     */
    reinitialize() {
        
        // If already initialized but store events not bound, rebind them
        if (this.isInitialized) {
            
            // Check if store events are actually bound
            if (window.taflFlowStore) {
                const listeners = window.taflFlowStore._listeners || {};
                const selectionListeners = listeners['selection:changed'] || [];
                
                if (selectionListeners.length === 0) {
                    this.bindStoreEvents();
                }
                
                // Also rebind property events if needed
                if (window.taflEditorProperties) {
                    this.bindPropertyEvents();
                }
            }
            return;
        }
        
        // Not initialized yet, do full init
        this.init();
    }
    
    // ========================================
    // 事件訂閱
    // ========================================
    
    bindPropertyEvents() {
        // 訂閱 Properties Module 的渲染事件
        if (window.taflEditorProperties) {
            window.taflEditorProperties.on('render:complete', ({ html, cardId }) => {
                this.updatePanel(html, cardId);
                this.showPanel();
            });
            
            window.taflEditorProperties.on('render:update', ({ html, cardId }) => {
                this.updatePanel(html, cardId);
            });
            
            window.taflEditorProperties.on('render:empty', () => {
                this.showEmptyState();
                this.hidePanel();
            });
            
            // 訂閱屬性變更事件
            window.taflEditorProperties.on('property:changed', ({ propertyPath, value }) => {
                // 可選：局部 UI 更新（如顯示儲存指示器）
            });
            
            // 訂閱驗證失敗事件
            window.taflEditorProperties.on('validation:failed', ({ propertyPath, reason }) => {
                this.showValidationError(propertyPath, reason);
            });
        } else {
        }
    }
    
    bindStoreEvents() {
        // 直接監聽選擇變化以控制面板顯示
        if (window.taflFlowStore) {
            window.taflFlowStore.on('selection:changed', (data) => {
                
                const { cardId, cardData } = data;
                
                if (cardId && cardData) {
                    this.currentCardId = cardId;
                    // Trigger properties rendering
                    if (window.taflEditorProperties) {
                        const html = window.taflEditorProperties.renderPropertyEditor(cardData);
                        this.updatePanel(html, cardId);
                    } else {
                    }
                } else {
                    this.currentCardId = null;
                    this.hidePanel();
                }
            });
        } else {
        }
    }
    
    // ========================================
    // UI 更新方法
    // ========================================
    
    updatePanel(html, cardId) {
        this.currentCardId = cardId;
        
        // Clean up existing CodeMirror instances
        this.cleanupCodeMirrors();
        
        // 保存捲動位置
        const scrollTop = this.content.scrollTop;
        
        // 更新內容
        this.content.innerHTML = html;
        
        // 恢復捲動位置
        this.content.scrollTop = scrollTop;
        
        // 重新綁定事件
        this.bindInputEvents();
        
        // Initialize CodeMirror for JSON properties
        this.initCodeMirrors();
        
        // Refresh CodeMirror editors after DOM is ready to ensure proper rendering
        requestAnimationFrame(() => {
            this.refreshCodeMirrors();
        });
    }
    
    showPanel() {
        if (!this.panel) return;
        // In TAFL Editor, panel visibility is controlled by tabs
        // Just ensure it's visible when a card is selected
        this.panel.style.display = 'block';
        this.panel.classList.add('is-active');
        
        // Ensure Properties tab is selected
        const propertiesTab = document.querySelector('[data-panel="properties"]');
        if (propertiesTab && !propertiesTab.classList.contains('is-active')) {
            // Trigger click to switch to properties tab
            propertiesTab.click();
        }
    }
    
    hidePanel() {
        if (!this.panel) return;
        // Don't actually hide the panel div, just show empty state
        // Panel visibility is controlled by tab system
        this.showEmptyState();
    }
    
    showEmptyState() {
        this.content.innerHTML = `
            <div class="no-selection">
                <div class="has-text-centered">
                    <span class="icon is-large has-text-grey-light">
                        <i class="fas fa-mouse-pointer fa-3x"></i>
                    </span>
                    <p class="has-text-grey">Select a card to edit properties</p>
                </div>
            </div>
        `;
    }
    
    showValidationError(propertyPath, reason) {
        const inputElement = this.content.querySelector(`[data-path="${propertyPath}"]`);
        if (!inputElement) return;
        
        // 添加錯誤樣式
        inputElement.classList.add('is-danger');
        
        // 顯示錯誤訊息
        const helpElement = inputElement.parentElement.querySelector('.help.is-danger');
        if (helpElement) {
            helpElement.textContent = reason;
            helpElement.style.display = 'block';
        }
        
        // 3秒後移除錯誤提示
        setTimeout(() => {
            inputElement.classList.remove('is-danger');
            if (helpElement) {
                helpElement.style.display = 'none';
            }
        }, 3000);
    }
    
    // ========================================
    // 事件綁定
    // ========================================
    
    bindInputEvents() {
        // 綁定所有輸入框
        const inputs = this.content.querySelectorAll('input, textarea, select');
        inputs.forEach(input => {
            input.addEventListener('change', this.handleInputChange);
            // 移除 blur 事件以避免重複觸發
            // input.addEventListener('blur', this.handleInputChange);
            
            // 文字區域自動調整高度
            if (input.tagName === 'TEXTAREA') {
                input.addEventListener('input', () => {
                    input.style.height = 'auto';
                    input.style.height = input.scrollHeight + 'px';
                });
            }
        });
        
        // 綁定 Switch Case 按鈕 (修正: 使用正確的 class names)
        
        // Add Case button
        const addCaseButtons = this.content.querySelectorAll('.add-case-btn');
        addCaseButtons.forEach(button => {
            button.addEventListener('click', this.handleSwitchCaseAdd);
        });
        
        // Add Default button
        const addDefaultButtons = this.content.querySelectorAll('.add-default-btn');
        addDefaultButtons.forEach(button => {
            button.addEventListener('click', this.handleSwitchCaseAddDefault);
        });
        
        // Remove Case buttons
        const removeCaseButtons = this.content.querySelectorAll('.remove-case-btn');
        removeCaseButtons.forEach(button => {
            button.addEventListener('click', this.handleSwitchCaseRemove);
        });
        
        // Remove Default button
        const removeDefaultButtons = this.content.querySelectorAll('.remove-default-btn');
        removeDefaultButtons.forEach(button => {
            button.addEventListener('click', this.handleSwitchCaseRemoveDefault);
        });
        
        // 綁定摺疊功能
        const collapsibles = this.content.querySelectorAll('.collapsible-header');
        collapsibles.forEach(header => {
            header.addEventListener('click', () => {
                const content = header.nextElementSibling;
                const icon = header.querySelector('.fa-chevron-down, .fa-chevron-right');
                
                if (content.style.display === 'none') {
                    content.style.display = 'block';
                    if (icon) {
                        icon.classList.remove('fa-chevron-right');
                        icon.classList.add('fa-chevron-down');
                    }
                } else {
                    content.style.display = 'none';
                    if (icon) {
                        icon.classList.remove('fa-chevron-down');
                        icon.classList.add('fa-chevron-right');
                    }
                }
            });
        });
    }
    
    // ========================================
    // 事件處理
    // ========================================
    
    handleInputChange(event) {
        const input = event.target;
        // Support both data-path and data-property for backward compatibility
        const propertyPath = input.dataset.path || input.dataset.property;
        const verb = input.dataset.verb;  // 取得 verb 資訊
        
        // Special handling for SET variable updates
        if (verb === 'set' && (input.classList.contains('set-var-key') || input.classList.contains('set-var-value'))) {
            this.handleSetVariableChange(input);
            return;
        }
        
        if (!propertyPath) {
            return;
        }
        let value = input.value;
        
        // 處理不同類型的輸入
        if (input.type === 'checkbox') {
            value = input.checked;
        } else if (input.type === 'number') {
            value = parseFloat(value) || 0;
        } else if (input.dataset.type === 'json') {
            try {
                value = JSON.parse(value);
            } catch (e) {
                // 保持原始字串，讓 Properties Module 驗證
            }
        }
        
        
        // 通知 Properties Module 更新（傳遞 verb 參數）
        taflEditorProperties.updateProperty(this.currentCardId, propertyPath, value, verb);
    }
    
    handleSetVariableChange(input) {
        const cardId = input.dataset.cardId;
        const varType = input.dataset.varType; // 'key' or 'value'
        const varKey = input.dataset.varKey; // current key name (for value updates)
        const varIndex = input.dataset.varIndex;
        
        
        const cardData = taflFlowStore.findCardById(cardId);
        if (!cardData || typeof cardData.set !== 'object') return;
        
        const currentParams = { ...cardData.set };
        
        if (varType === 'key') {
            // Renaming a variable key
            const oldKey = Object.keys(currentParams)[parseInt(varIndex)];
            if (oldKey && oldKey !== input.value) {
                const newKey = input.value || 'unnamed';
                const value = currentParams[oldKey];
                
                // Create new params object with renamed key
                const newParams = {};
                Object.entries(currentParams).forEach(([k, v]) => {
                    if (k === oldKey) {
                        newParams[newKey] = v;
                    } else {
                        newParams[k] = v;
                    }
                });
                
                // Update the card
                taflFlowStore.updateCard(cardId, {
                    ...cardData,
                    set: newParams
                });
                
                // Refresh panel to update data attributes
                setTimeout(() => {
                    const updatedCard = taflFlowStore.findCardById(cardId);
                    const html = taflEditorProperties.renderPropertyEditor(updatedCard);
                    this.updatePanel(html, cardId);
                }, 100);
            }
        } else if (varType === 'value') {
            // Updating a variable value
            if (varKey) {
                currentParams[varKey] = input.value;
                
                // Update the card
                taflFlowStore.updateCard(cardId, {
                    ...cardData,
                    set: currentParams
                });
            }
        }
    }
    
    handleSwitchCaseAdd(event) {
        const cardId = event.currentTarget.dataset.cardId || this.currentCardId;
        
        // 通知 Store 添加 Case
        taflFlowStore.addSwitchCase(cardId);
    }
    
    handleSwitchCaseAddDefault(event) {
        const cardId = event.currentTarget.dataset.cardId || this.currentCardId;
        
        // 通知 Store 添加 Default case
        taflFlowStore.addSwitchDefault(cardId);
    }
    
    handleSwitchCaseRemove(event) {
        const cardId = event.currentTarget.dataset.cardId || this.currentCardId;
        const caseIndex = parseInt(event.currentTarget.dataset.caseIndex);
        
        // 確認刪除
        if (confirm('Remove this case?')) {
            taflFlowStore.removeSwitchCase(cardId, caseIndex);
        }
    }
    
    handleSwitchCaseRemoveDefault(event) {
        const cardId = event.currentTarget.dataset.cardId || this.currentCardId;
        
        // 確認刪除
        if (confirm('Remove default case?')) {
            taflFlowStore.removeSwitchDefault(cardId);
        }
    }
    
    // ========================================
    // CodeMirror 管理
    // ========================================
    
    cleanupCodeMirrors() {
        // Clean up existing CodeMirror instances
        this.codeMirrorEditors.forEach(editor => {
            if (editor && editor.toTextArea) {
                editor.toTextArea();
            }
        });
        this.codeMirrorEditors = [];
    }
    
    refreshCodeMirrors() {
        // Refresh all CodeMirror instances to fix rendering
        this.codeMirrorEditors.forEach(editor => {
            if (editor && editor.refresh) {
                editor.refresh();
            }
        });
    }
    
    initCodeMirrors() {
        // Find all JSON property containers that need CodeMirror
        const jsonContainers = this.content.querySelectorAll('.json-editor-container');
        
        jsonContainers.forEach(container => {
            const textarea = container.querySelector('textarea.json-property');
            if (!textarea) return;
            
            // Check if CodeMirror is available
            if (typeof CodeMirror === 'undefined') {
                return;
            }
            
            // Initialize CodeMirror with JSON mode
            const editor = CodeMirror.fromTextArea(textarea, {
                mode: 'application/json',
                theme: 'default',
                lineNumbers: true,
                lineWrapping: true,
                autoCloseBrackets: true,
                matchBrackets: true,
                indentUnit: 2,
                tabSize: 2,
                lint: true,
                gutters: ['CodeMirror-lint-markers'],
                extraKeys: {
                    'Ctrl-Space': 'autocomplete',
                    'Tab': (cm) => {
                        if (cm.somethingSelected()) {
                            cm.indentSelection('add');
                        } else {
                            cm.replaceSelection('  ', 'end');
                        }
                    }
                }
            });
            
            // Set initial size
            editor.setSize(null, 200);
            
            // Flag to prevent change events during initialization
            let isInitializing = true;
            
            // Store the editor instance first
            this.codeMirrorEditors.push(editor);
            
            // Auto-format JSON on initialization BEFORE setting up change handler
            try {
                const value = JSON.parse(textarea.value);
                // Set value without triggering change event
                editor.setValue(JSON.stringify(value, null, 2));
            } catch (e) {
                // Keep original value if not valid JSON
            }
            
            // Refresh editor after it becomes visible
            requestAnimationFrame(() => {
                editor.refresh();
            });
            
            // Allow change events after initialization and formatting
            setTimeout(() => {
                isInitializing = false;
                
                // Now set up the change handler after initialization is complete
                editor.on('change', (cm) => {
                    // Update the underlying textarea
                    textarea.value = cm.getValue();
                    
                    // Trigger change event for property update
                    const event = new Event('change', { bubbles: true });
                    textarea.dispatchEvent(event);
                });
            }, 100);
        });
        
    }
    
    // ========================================
    // 公開方法
    // ========================================
    
    /**
     * 重新整理目前顯示的屬性
     */
    refresh() {
        if (this.currentCardId) {
            const cardData = taflFlowStore.findCardById(this.currentCardId);
            if (cardData) {
                const html = taflEditorProperties.renderPropertyEditor(cardData);
                this.updatePanel(html, this.currentCardId);
            }
        }
    }
    
    /**
     * 清理資源
     */
    dispose() {
        // 移除所有事件監聽
        const inputs = this.content.querySelectorAll('input, textarea, select');
        inputs.forEach(input => {
            input.removeEventListener('change', this.handleInputChange);
            // input.removeEventListener('blur', this.handleInputChange);
        });
    }
}

// 建立單例並匯出
const taflPanelsProperties = new TAFLPanelsProperties();

export default taflPanelsProperties;
export { TAFLPanelsProperties };