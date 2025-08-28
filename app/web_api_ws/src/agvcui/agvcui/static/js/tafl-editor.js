/**
 * TAFL Editor - Professional Drag & Drop Visual Editor
 * Task Automation Flow Language Visual Editor v3.0
 * Complete rewrite with professional drag & drop experience
 * Now using miniStore for state management
 */

// Import store
import { taflFlowStore } from './tafl-editor/tafl-editor-store.js';
// Import notification system
import taflNotifications from './tafl-editor/tafl-notifications.js';
// Import API module
import taflAPI from './tafl-editor/tafl-editor-api.js';
// Import modals module
import taflModals from './tafl-editor/tafl-editor-modals.js';
// Import panels module
import { taflPanels } from './tafl-editor/tafl-panels.js';
// Import enhancements module
import TAFLEditorEnhancements, { injectEnhancementsStyles } from './tafl-editor/tafl-editor-enhancements.js';

class TAFLEditor {
    constructor() {
        // Store manages the flow state now
        this.selectedCard = null;
        this.draggedElement = null;
        this.dragData = null;
        this.yamlEditor = null;
        this.lastInsertionIndex = undefined;  // Track last drop position to prevent flashing
        this.cardOriginalPositions = new Map();  // 追蹤卡片的原始位置
        
        // Temporary verbDefinitions reference for compatibility
        this.verbDefinitions = {};
        
        // Make notifications available globally
        window.taflNotifications = taflNotifications;
        
        // Initialize panels module
        this.panels = taflPanels;
        window.taflPanels = taflPanels; // For compatibility with existing code
        
        // Initialize enhancements module
        this.enhancements = new TAFLEditorEnhancements(this);
        window.taflEnhancements = this.enhancements; // For compatibility
        
        // Inject enhancements styles
        injectEnhancementsStyles();
        
        // Subscribe to store events
        this.setupStoreSubscriptions();
        
        this.initializeEditor();
        
        console.log('✅ TAFL Editor Enhancements initialized - Press ? for keyboard shortcuts');
    }
    
    setupStoreSubscriptions() {
        // Flow changes
        taflFlowStore.on('flow:changed', () => {
            // 如果沒有其他更具體的事件處理，也需要更新畫布
            this.updateCanvas({
                fullRefresh: true,
                skipDuringDrag: false,
                source: 'flow:changed'
            });
            this.updateUI();
            this.refreshYAML();
        });
        
        // Card selection
        taflFlowStore.on('card:selected', (cardId) => {
            this.updateSelectionUI(cardId);
        });
        
        // Card additions
        taflFlowStore.on('card:added', (data) => {
            this.updateCanvas({ 
                fullRefresh: true, 
                skipDuringDrag: false,  // 添加卡片時不要跳過更新！
                source: 'card:added' 
            });
            const cardEl = document.querySelector(`[data-card-id="${data.card.id}"]`);
            if (cardEl) this.selectCard(cardEl);
        });
        
        // Card updates
        taflFlowStore.on('card:updated', (data) => {
            this.updateCardUI(data.cardId);
        });
        
        // Card deletions
        taflFlowStore.on('card:deleted', () => {
            this.updateCanvas({ 
                fullRefresh: true, 
                source: 'card:deleted' 
            });
        });
        
        // Card moves
        taflFlowStore.on('card:moved', (data) => {
            // 檢查是否需要完整更新（從嵌套移到外層或反之）
            const needsFullRefresh = this.checkIfStructureChanged(data);
            this.updateCanvas({ 
                fullRefresh: needsFullRefresh,  // 結構改變時需要完整更新
                source: 'card:moved' 
            });
        });
        
        // Variables changes
        taflFlowStore.on('variables:changed', (variables) => {
            this.renderVariables();
        });
        
        // Dirty state changes
        taflFlowStore.on('dirty:changed', (isDirty) => {
            this.updateSaveButton(isDirty);
        });
        
        // Flow loaded
        taflFlowStore.on('flow:loaded', () => {
            this.updateCanvas({ 
                fullRefresh: true, 
                preserveScroll: false,  // 新載入不保留捲動
                source: 'flow:loaded' 
            });
            this.updateUI();
        });
        
        // Verbs loaded
        taflFlowStore.on('verbs:loaded', (verbs) => {
            this.verbDefinitions = verbs;
        });
        
        // Subscribe to specific flow updates
        taflFlowStore.on('metadata:changed', () => {
            // SimpleTAFLPanels handles metadata UI updates
            // this.refreshMetadataUI();
        });
        
        // Re-render UI when flow changes
        taflFlowStore.on('flow:changed', () => {
            // SimpleTAFLPanels handles all UI updates via store events
            // this.refreshPreloadUI();
            // this.refreshRulesUI();
            // this.refreshSettingsUI();
        });
    }
    
    async initializeEditor() {
        await this.loadVerbDefinitions();
        this.setupEventListeners();
        this.setupGlobalEventDelegation();  // 新增：全域事件委派
        this.initializeCodeMirror();
        this.setupDragAndDrop();
        this.updateUI();
        this.initializeVariables();
    }
    
    async loadVerbDefinitions() {
        try {
            const verbs = await taflAPI.loadVerbs();
            this.verbDefinitions = verbs;
            taflFlowStore.setVerbDefinitions(verbs);
        } catch (error) {
            console.error('Error loading verb definitions:', error);
            this.showNotification(error.message, 'is-danger');
        }
    }
    
    /**
     * Helper method to extract verb from card data in TAFL v1.1 format
     * In TAFL v1.1, the verb is the key, not a property
     */
    getCardVerb(cardData) {
        if (!cardData) return null;
        
        // Exclude known non-verb keys
        const nonVerbKeys = ['id', 'comment', 'skip_if', 'store_as', 'as'];
        
        // Find the verb key (should be one of the TAFL verbs)
        for (const key of Object.keys(cardData)) {
            if (!nonVerbKeys.includes(key) && this.verbDefinitions && this.verbDefinitions[key]) {
                return key;
            }
        }
        
        // Fallback: find any key that's not in the exclusion list
        for (const key of Object.keys(cardData)) {
            if (!nonVerbKeys.includes(key)) {
                return key;
            }
        }
        
        return null;
    }
    
    setupEventListeners() {
        // Toolbar buttons
        document.getElementById('new-flow-btn').addEventListener('click', () => taflModals.newFlow());
        // Open button removed - using dropdown instead
        document.getElementById('save-flow-btn').addEventListener('click', () => taflModals.showSaveFlowModal());
        document.getElementById('validate-btn').addEventListener('click', () => this.validateFlow());
        document.getElementById('execute-btn').addEventListener('click', () => this.executeFlow());
        
        // Panel tabs
        document.querySelectorAll('.panel-tabs a').forEach(tab => {
            tab.addEventListener('click', (e) => {
                e.preventDefault();
                this.switchVerbCategory(tab.dataset.tab);
            });
        });
        
        // Property panel tabs
        document.querySelectorAll('.tabs li').forEach(tab => {
            tab.addEventListener('click', (e) => {
                e.preventDefault();
                this.switchPropertiesTab(tab.dataset.tab);
            });
        });
        
        // Verb items (toolbox) - click only for now
        document.querySelectorAll('.verb-item').forEach(item => {
            item.addEventListener('click', () => {
                const verb = item.dataset.verb;
                this.addCard(verb);
            });
        });
        
        // Metadata inputs
        document.getElementById('flow-id').addEventListener('input', (e) => {
            taflFlowStore.updateMetadata({ id: e.target.value });
            this.markDirty();
        });
        
        document.getElementById('flow-name').addEventListener('input', (e) => {
            taflFlowStore.updateMetadata({ name: e.target.value });
            this.markDirty();
        });
        
        document.getElementById('flow-version').addEventListener('input', (e) => {
            taflFlowStore.updateMetadata({ version: e.target.value });
            this.markDirty();
        });
        
        document.getElementById('flow-description').addEventListener('input', (e) => {
            taflFlowStore.updateMetadata({ description: e.target.value });
            this.markDirty();
        });
        
        // TAFL v1.1: Settings panel event listeners
        const settingsTimeout = document.getElementById('settings-timeout');
        if (settingsTimeout) {
            settingsTimeout.addEventListener('input', (e) => {
                const flow = taflFlowStore.getFlow();
                taflFlowStore.updateFlow({
                    ...flow,
                    settings: { ...flow.settings, timeout: parseInt(e.target.value) || 3600 }
                });
                this.markDirty();
            });
        }
        
        const settingsMaxRetries = document.getElementById('settings-max-retries');
        if (settingsMaxRetries) {
            settingsMaxRetries.addEventListener('input', (e) => {
                const flow = taflFlowStore.getFlow();
                taflFlowStore.updateFlow({
                    ...flow,
                    settings: { ...flow.settings, max_retries: parseInt(e.target.value) || 3 }
                });
                this.markDirty();
            });
        }
        
        const settingsRetryOnFailure = document.getElementById('settings-retry-on-failure');
        if (settingsRetryOnFailure) {
            settingsRetryOnFailure.addEventListener('change', (e) => {
                const flow = taflFlowStore.getFlow();
                taflFlowStore.updateFlow({
                    ...flow,
                    settings: { ...flow.settings, retry_on_failure: e.target.checked }
                });
                this.markDirty();
            });
        }
        
        // TAFL v1.1: Preload panel event listeners
        // NOTE: Add Preload button event is now handled by TAFLPanels.showAddPreloadModal()
        // @see app/web_api_ws/src/agvcui/agvcui/static/js/tafl-editor/tafl-panels.js (line 63)
        // The addPreload() method below is DEPRECATED - TAFLPanels handles modal display
        // Commenting out to avoid duplicate event binding:
        // const addPreloadBtn = document.getElementById('add-preload-btn');
        // if (addPreloadBtn) {
        //     addPreloadBtn.addEventListener('click', () => this.addPreload());
        // }
        
        // TAFL v1.1: Rules panel event listeners
        // NOTE: Add Rule button event is now handled by TAFLPanels.showAddRuleModal()
        // @see app/web_api_ws/src/agvcui/agvcui/static/js/tafl-editor/tafl-panels.js (line 50)
        // The addRule() method below is DEPRECATED - TAFLPanels handles modal display
        // Commenting out to avoid duplicate event binding:
        // const addRuleBtn = document.getElementById('add-rule-btn');
        // if (addRuleBtn) {
        //     addRuleBtn.addEventListener('click', () => this.addRule());
        // }
        
        // Rule type selector removed - now using modal dialogs in TAFLPanels
        
        // Variable management - bind the Add Variable button  
        // (handled by TAFLPanels in tafl-panels.js now)
        
        // Modal controls
        taflModals.setupModalControls();
        
        // Canvas click (deselect)
        document.getElementById('flow-canvas').addEventListener('click', (e) => {
            if (e.target.classList.contains('tafl-canvas') || e.target.classList.contains('canvas-drop-zone')) {
                this.deselectCard();
            }
        });
        
        // YAML editor controls
        const applyYamlBtn = document.getElementById('apply-yaml-btn');
        const refreshYamlBtn = document.getElementById('refresh-yaml-btn');
        
        console.log('Apply YAML button found:', !!applyYamlBtn);
        console.log('Refresh YAML button found:', !!refreshYamlBtn);
        
        if (applyYamlBtn) {
            applyYamlBtn.addEventListener('click', () => {
                console.log('Apply YAML button clicked');
                this.applyYAMLChanges();
            });
        }
        
        if (refreshYamlBtn) {
            refreshYamlBtn.addEventListener('click', () => {
                console.log('Refresh YAML button clicked');
                this.refreshYAML();
            });
        }
    }
    
    /**
     * GLOBAL EVENT DELEGATION SYSTEM
     * 統一處理所有層級的卡片事件，解決巢狀卡片無法操作的問題
     */
    setupGlobalEventDelegation() {
        const canvas = document.getElementById('flow-canvas');
        if (!canvas) return;
        
        // 1. 統一點擊處理（所有層級）
        canvas.addEventListener('click', (e) => {
            // 如果正在拖動，忽略點擊事件
            if (this.draggedElement || document.body.classList.contains('dragging')) {
                console.log('Ignoring click during drag');
                return;
            }
            
            const card = e.target.closest('.tafl-card');
            
            if (!card) {
                // 點擊空白處取消選擇
                if (e.target.id === 'flow-canvas' || e.target.classList.contains('canvas-drop-zone')) {
                    this.deselectCard();
                }
                return;
            }
            
            // 檢查是否點擊控制按鈕
            if (e.target.closest('.card-controls')) {
                e.stopPropagation();
                const btn = e.target.closest('button');
                if (btn?.classList.contains('card-duplicate-btn')) {
                    this.duplicateCard(card.dataset.cardId);
                } else if (btn?.classList.contains('card-delete-btn')) {
                    this.deleteCard(card.dataset.cardId);
                }
                return;
            }
            
            // 選擇卡片
            e.stopPropagation();
            this.selectCard(card);
        });
        
        // 2. 統一拖動開始
        canvas.addEventListener('dragstart', (e) => {
            const card = e.target.closest('.tafl-card');
            console.log('🔵 Dragstart event fired');
            console.log('  - target:', e.target);
            console.log('  - target tagName:', e.target.tagName);
            console.log('  - target classes:', e.target.className);
            console.log('  - card found:', card);
            console.log('  - isTrusted:', e.isTrusted);
            console.log('  - bubbles:', e.bubbles);
            console.log('  - cancelable:', e.cancelable);
            console.log('  - dataTransfer:', e.dataTransfer);
            console.log('  - currentTarget:', e.currentTarget);
            console.log('  - eventPhase:', e.eventPhase);
            console.log('  - defaultPrevented:', e.defaultPrevented);
            
            // 檢查是否有其他事件監聽器
            if (typeof getEventListeners !== 'undefined') {
                const listeners = getEventListeners(e.target);
                console.log('  - Event listeners on target:', listeners);
            }
            
            if (!card) {
                console.log('❌ No card found for dragstart');
                return;
            }
            
            if (!card.draggable) {
                console.log('❌ Card is not draggable:', card);
                console.log('  - card.draggable attribute:', card.getAttribute('draggable'));
                return;
            }
            
            console.log('📍 Card info:');
            console.log('  - cardId:', card.dataset.cardId);
            console.log('  - parent:', card.parentElement);
            console.log('  - parent tagName:', card.parentElement?.tagName);
            console.log('  - parent classes:', card.parentElement?.className);
            console.log('  - classList:', card.classList.toString());
            
            // 檢查是否有 CSS 阻止拖動
            const computedStyle = window.getComputedStyle(card);
            console.log('  - computed userSelect:', computedStyle.userSelect);
            console.log('  - computed pointerEvents:', computedStyle.pointerEvents);
            console.log('  - computed cursor:', computedStyle.cursor);
            console.log('  - computed touchAction:', computedStyle.touchAction);
            console.log('  - computed webkitUserDrag:', computedStyle.webkitUserDrag);
            
            // 檢查父元素樣式
            const parentStyle = card.parentElement ? window.getComputedStyle(card.parentElement) : null;
            if (parentStyle) {
                console.log('  - parent userSelect:', parentStyle.userSelect);
                console.log('  - parent pointerEvents:', parentStyle.pointerEvents);
            }
            
            // 檢查是否有其他東西阻止拖動
            const isDefaultPrevented = e.defaultPrevented;
            console.log('  - defaultPrevented before our handling:', isDefaultPrevented);
            
            // 不要 stopPropagation - 讓事件正常冒泡
            // e.stopPropagation();
            
            // 立即設置拖動數據，確保 dragover 可以訪問
            this.draggedElement = card;
            
            // 記錄原始位置（是否在嵌套中）
            const cardId = card.dataset.cardId;
            const isNested = card.closest('.nested-cards') !== null;
            this.cardOriginalPositions.set(cardId, { isNested });
            console.log(`📌 Recording original position for card ${cardId}: nested=${isNested}`);
            
            this.dragData = {
                type: 'card',
                cardId: cardId
            };
            
            console.log('📦 Setting dataTransfer:');
            console.log('  - effectAllowed: move');
            console.log('  - dropEffect: move');
            
            // 設置 dataTransfer
            e.dataTransfer.effectAllowed = 'move';
            e.dataTransfer.dropEffect = 'move';
            
            // 設定拖動時的視覺效果 - 直接使用原始元素
            // 避免創建複製元素可能導致的問題
            if (e.dataTransfer.setDragImage) {
                console.log('  - Setting drag image');
                console.log('  - offsetX:', e.offsetX);
                console.log('  - offsetY:', e.offsetY);
                e.dataTransfer.setDragImage(card, e.offsetX, e.offsetY);
            }
            
            try {
                e.dataTransfer.setData('text/plain', JSON.stringify(this.dragData));
                console.log('  - Data set successfully');
            } catch (err) {
                console.error('❌ Error setting drag data:', err);
            }
            
            card.classList.add('dragging');
            document.body.classList.add('dragging'); // 設置全域拖動狀態
            
            // 檢查卡片是否仍然可拖動
            setTimeout(() => {
                if (!card.draggable) {
                    console.log('⚠️ Card became undraggable after dragstart!');
                }
                if (!document.body.contains(card)) {
                    console.log('⚠️ Card was removed from DOM after dragstart!');
                }
                const stillDragging = card.classList.contains('dragging');
                console.log('  - Card still has dragging class after 10ms:', stillDragging);
            }, 10);
            
            console.log('✅ Drag start completed');
            console.log('  - draggedElement set:', !!this.draggedElement);
            console.log('  - dragData set:', this.dragData);
            console.log('  - Event propagation stopping?', e.cancelBubble);
            
            // 添加監聽器來追踪何時觸發 dragend
            let dragEventCount = 0;
            const dragHandler = (e) => {
                dragEventCount++;
                if (dragEventCount === 1 || dragEventCount % 50 === 0) {
                    console.log(`🟡 Drag event #${dragEventCount} (during drag)`);
                    console.log('  - clientX:', e.clientX);
                    console.log('  - clientY:', e.clientY);
                }
            };
            window.addEventListener('drag', dragHandler);
            this.onDrag = dragHandler;
            
            // 監聽各種可能導致 drag 結束的事件
            const mouseUpHandler = (e) => {
                console.log('🟠 MouseUp event during drag!');
                console.log('  - target:', e.target);
                console.log('  - button:', e.button);
            };
            window.addEventListener('mouseup', mouseUpHandler, { once: true });
            
            // 監聽 ESC 鍵
            const keyHandler = (e) => {
                if (e.key === 'Escape') {
                    console.log('🟠 ESC key pressed during drag!');
                }
            };
            window.addEventListener('keydown', keyHandler);
            
            // 監聽 document 上的 dragend 事件，以防在 canvas 外結束
            document.addEventListener('dragend', this.onDocDragEnd = (e) => {
                console.log('🔴 Document dragend event fired');
                console.log('  - target:', e.target);
                console.log('  - outside canvas?', !canvas.contains(e.target));
                
                // 清理事件監聽器
                window.removeEventListener('mouseup', mouseUpHandler);
                window.removeEventListener('keydown', keyHandler);
            }, { once: true });
        }, false);
        
        // 3. 統一拖動結束
        canvas.addEventListener('dragend', (e) => {
            console.log('🔴 Canvas dragend event fired');
            console.log('  - dropEffect:', e.dataTransfer.dropEffect);
            console.log('  - effectAllowed:', e.dataTransfer.effectAllowed);
            console.log('  - isTrusted:', e.isTrusted);
            console.log('  - target:', e.target);
            console.log('  - target tagName:', e.target.tagName);
            console.log('  - target classes:', e.target.className);
            console.log('  - currentTarget:', e.currentTarget);
            console.log('  - eventPhase:', e.eventPhase);
            console.log('  - clientX:', e.clientX);
            console.log('  - clientY:', e.clientY);
            console.log('  - screenX:', e.screenX);
            console.log('  - screenY:', e.screenY);
            
            // 移除 drag 事件監聽器
            if (this.onDrag) {
                window.removeEventListener('drag', this.onDrag);
                console.log('  - Removed drag event listener');
            }
            
            // 移除 document dragend 監聽器
            if (this.onDocDragEnd) {
                document.removeEventListener('dragend', this.onDocDragEnd);
                console.log('  - Removed document dragend listener');
            }
            
            // 檢查拖動是否真的成功
            console.log('  - Was drag successful?', e.dataTransfer.dropEffect !== 'none');
            
            const card = e.target.closest('.tafl-card');
            if (!card) {
                console.log('❌ No card found for dragend');
                return;
            }
            
            console.log('📍 Dragend card info:');
            console.log('  - cardId:', card.dataset.cardId);
            console.log('  - was dragging:', card.classList.contains('dragging'));
            console.log('  - had draggedElement:', this.draggedElement === card);
            
            card.classList.remove('dragging');
            document.body.classList.remove('dragging'); // 移除全域拖動狀態
            
            this.hideDropIndicators();
            // 立即清理拖曳狀態 - 不需要延遲！
            this.draggedElement = null;
            this.dragData = null;
            console.log('  - Cleared drag data immediately');
            
            console.log('✅ Dragend completed');
        }, false);
        
        // 4. 統一拖動經過
        let dragoverCount = 0;
        canvas.addEventListener('dragover', (e) => {
            dragoverCount++;
            if (dragoverCount === 1 || dragoverCount % 10 === 0) {
                console.log(`🟢 Dragover event #${dragoverCount}`);
                console.log('  - has dragData:', !!this.dragData);
                console.log('  - target:', e.target.tagName, e.target.className);
            }
            
            // 始終阻止默認行為，允許 drop
            e.preventDefault();
            
            // 根據拖動類型設置正確的 dropEffect
            if (this.dragData) {
                e.dataTransfer.dropEffect = this.dragData.type === 'verb' ? 'copy' : 'move';
            } else {
                e.dataTransfer.dropEffect = 'move';
            }
            
            // 如果沒有拖動數據，不更新視覺效果
            if (!this.dragData) {
                console.log('⚠️ Dragover without dragData');
                return;
            }
            
            // 處理巢狀 drop zone 的 hover 效果
            const nestedDropZone = e.target.closest('.nested-drop-zone');
            if (nestedDropZone) {
                nestedDropZone.classList.add('drag-over');
            }
            
            this.updateUnifiedDropIndicator(e);
        }, false);
        
        // 4.5. 統一拖動離開
        canvas.addEventListener('dragleave', (e) => {
            // 處理巢狀 drop zone 的 hover 移除
            const nestedDropZone = e.target.closest('.nested-drop-zone');
            if (nestedDropZone && !nestedDropZone.contains(e.relatedTarget)) {
                nestedDropZone.classList.remove('drag-over');
            }
        }, false);
        
        // 5. 統一放置處理
        canvas.addEventListener('drop', (e) => {
            console.log('📍 Drop event triggered, dragData:', this.dragData);
            
            // 確保有拖動數據
            // 注意：從 Verbs 拖動時，draggedElement 是 null，但 dragData 存在
            if (!this.dragData) {
                console.log('Drop event but no drag data');
                return;
            }
            
            e.preventDefault();
            e.stopPropagation();
            
            console.log('📍 Drop event at:', e.clientX, e.clientY, 'dragData type:', this.dragData.type);
            
            // 檢查是否點擊在特定的 drop zone 上
            const dropZone = e.target.closest('.drop-zone, .nested-drop-zone, .canvas-drop-zone');
            let position;
            
            if (dropZone) {
                // 如果是 drop zone，使用其特定位置
                position = this.getDropZonePosition(dropZone);
                console.log('Drop zone detected:', dropZone, 'position:', position);
            } else {
                // 否則計算一般位置
                position = this.calculateUnifiedDropPosition(e);
                console.log('Calculated position:', position);
            }
            
            if (position) {
                console.log('💧 Handling drop at position:', position);
                this.handleUnifiedDrop(position);
            } else {
                console.log('⚠️ Could not determine drop position');
            }
            
            this.hideDropIndicators();
            this.hideAllDropZones();
            document.body.classList.remove('dragging');
            
            // 立即清空 dragData 避免重複處理
            this.dragData = null;
            this.draggedElement = null;
        }, false);
    }
    
    /**
     * 獲取 drop zone 的位置信息
     */
    getDropZonePosition(dropZone) {
        // 檢查是否是嵌套 drop zone (使用正確的 class)
        const branch = dropZone.closest('.nested-cards');
        if (branch) {
            const branchName = branch.dataset.branch;
            const parentCard = branch.closest('.tafl-card');
            if (parentCard) {
                const parentId = parentCard.dataset.cardId;
                
                // 找到該 branch 內的現有卡片數量來決定 index
                const existingCards = branch.querySelectorAll('.tafl-card');
                const index = existingCards.length;
                
                return {
                    nested: {
                        parentCardId: parentId,  // 統一使用 parentCardId
                        branchType: branchName    // 統一使用 branchType
                    },
                    index: index
                };
            }
        }
        
        // 主畫布 drop zone
        const canvasDropZone = document.getElementById('canvas-drop-zone');
        const cards = Array.from(canvasDropZone.querySelectorAll(':scope > .tafl-card'));
        return {
            nested: null,
            index: cards.length
        };
    }
    
    /**
     * PROFESSIONAL DRAG & DROP SYSTEM
     * Complete rewrite based on successful patterns from linear flow designer
     */
    setupDragAndDrop() {
        const canvas = document.getElementById('flow-canvas');
        
        if (!canvas) {
            console.error('Canvas element not found!');
            return;
        }
        
        // Set up toolbox drag sources
        this.setupToolboxDragSources();
        
        // Set up canvas as drop target
        this.setupCanvasDropTarget(canvas);
        
        // Initialize drop zones
        this.createInitialDropZone();
        
        // Make sure existing cards are draggable
        this.refreshCardDragHandlers();
        
        console.log('✅ Drag and drop initialized');
    }
    
    setupToolboxDragSources() {
        // Make verb items draggable
        document.querySelectorAll('.verb-item').forEach(item => {
            item.draggable = true;
            
            // Visual feedback that item is draggable
            item.style.cursor = 'move';
            item.title = 'Drag to add to flow';
            
            item.addEventListener('dragstart', (e) => {
                e.stopPropagation();
                document.body.classList.add('dragging');
                
                const verb = item.dataset.verb;
                // Always set dragData as primary data storage
                this.dragData = {
                    type: 'verb',
                    verb: verb,
                    source: 'toolbox'
                };
                
                // Try to set data for drop detection (may fail in some browsers)
                try {
                    e.dataTransfer.setData('application/json', JSON.stringify(this.dragData));
                    e.dataTransfer.setData('text/plain', verb); // Fallback
                } catch (err) {
                    console.warn('Could not set dataTransfer data:', err);
                }
                e.dataTransfer.effectAllowed = 'copy';
                
                // Visual feedback
                item.classList.add('is-dragging');
                this.showAllDropZones();
                
                console.log('🎯 Started dragging verb:', verb);
            });
            
            item.addEventListener('dragend', (e) => {
                item.classList.remove('is-dragging');
                document.body.classList.remove('dragging');
                this.hideAllDropZones();
                // Don't clear dragData immediately, let drop handler use it first
                setTimeout(() => {
                    this.dragData = null;
                }, 100);
                
                console.log('🎯 Ended dragging');
            });
        });
    }
    
    setupCanvasDropTarget(canvas) {
        // Canvas-level drag events
        canvas.addEventListener('dragover', (e) => {
            e.preventDefault();
            
            // Always update drop indicators during dragover
            this.updateDropIndicators(e);
            
            const dragData = this.getDragData(e) || this.dragData;
            if (dragData) {
                e.dataTransfer.dropEffect = dragData.source === 'toolbox' ? 'copy' : 'move';
            }
        });
        
        canvas.addEventListener('dragleave', (e) => {
            // Only hide if truly leaving canvas
            if (!canvas.contains(e.relatedTarget)) {
                this.hideDropIndicators();
            }
        });
        
        // 移除重複的 drop 監聽器，已經在 setupUnifiedDragDrop 中處理
    }
    
    getDragData(e) {
        // During drag, we can't always read dataTransfer, so use stored dragData
        if (this.dragData) {
            return this.dragData;
        }
        
        try {
            const jsonData = e.dataTransfer.getData('application/json');
            if (jsonData) {
                return JSON.parse(jsonData);
            }
        } catch (err) {
            // Fallback to plain text
            const verb = e.dataTransfer.getData('text/plain');
            if (verb) {
                return { type: 'verb', verb: verb, source: 'toolbox' };
            }
        }
        return null;
    }
    
    handleCanvasDrop(e) {
        // 檢查是否已經被 drop zone 處理過
        if (e.defaultPrevented) {
            return;
        }
        
        const dragData = this.getDragData(e);
        if (!dragData) {
            return;
        }
        
        console.log('🎯 Drop on canvas:', dragData);
        
        this.hideAllDropZones();
        this.hideDropIndicators();
        
        if (dragData.type === 'verb' && dragData.source === 'toolbox') {
            // Adding new card from toolbox
            const dropPosition = this.calculateDropPosition(e);
            this.addCardAtPosition(dragData.verb, dropPosition);
        } else if (dragData.type === 'card') {
            // Moving existing card
            const dropPosition = this.calculateDropPosition(e);
            this.moveCardToPosition(dragData.cardId, dropPosition);
        }
    }
    
    calculateDropPosition(e) {
        const dropZone = document.getElementById('canvas-drop-zone');
        if (!dropZone) {
            // 如果 drop zone 不存在，返回默認位置
            return { index: 0, nested: null };
        }
        
        // Get all cards including the dragging one to calculate correct original index
        const allCards = Array.from(dropZone.querySelectorAll('.tafl-card'));
        const draggingCard = dropZone.querySelector('.tafl-card.dragging');
        const draggingCardIndex = draggingCard ? allCards.indexOf(draggingCard) : -1;
        
        // Get cards excluding the dragging one for position calculation
        const cards = Array.from(dropZone.querySelectorAll('.tafl-card:not(.dragging)'));
        
        if (cards.length === 0 && !draggingCard) {
            return { index: 0, nested: null };
        }
        
        const mouseY = e.clientY;
        let targetIndex = cards.length;
        
        // Find insertion position based on non-dragging cards
        for (let i = 0; i < cards.length; i++) {
            const card = cards[i];
            const rect = card.getBoundingClientRect();
            const cardMiddle = rect.top + rect.height / 2;
            
            if (mouseY < cardMiddle) {
                targetIndex = i;
                break;
            }
        }
        
        // Adjust index if dragging a card from before the target position
        // This is necessary because the dragging card will be removed first
        // and then inserted, which changes the indices
        if (draggingCard && draggingCardIndex !== -1 && draggingCardIndex < targetIndex) {
            // The target index should account for the fact that the dragging card
            // will be removed, which shifts all subsequent indices down by 1
            // But since we already excluded the dragging card from 'cards',
            // the targetIndex is already correct for the final position
            // No adjustment needed here
        }
        
        // Check for nested drop zones
        const nestedArea = this.getNestedDropTarget(e.target);
        if (nestedArea) {
            const parentCard = nestedArea.closest('.tafl-card');
            const parentCardId = parentCard?.dataset.cardId;
            const branchType = this.getBranchType(nestedArea);
            
            if (parentCardId && branchType) {
                return {
                    nested: {
                        parentCardId: parentCardId,
                        branchType: branchType
                    }
                };
            }
        }
        
        console.log('📍 Drop position calculated:', {
            targetIndex,
            draggingCardIndex,
            totalCards: allCards.length,
            mouseY
        });
        
        return { index: targetIndex, nested: null };
    }
    
    /**
     * UNIFIED DROP POSITION CALCULATION
     * 統一計算所有層級的放置位置
     */
    calculateUnifiedDropPosition(e) {
        // 1. 檢查是否在空的巢狀放置區
        const emptyNested = e.target.closest('.nested-drop-zone.empty-zone');
        if (emptyNested) {
            const parentCard = emptyNested.closest('.tafl-card');
            const branch = emptyNested.dataset.branch;
            
            if (parentCard && branch) {
                return {
                    nested: {
                        parentCardId: parentCard.dataset.cardId,
                        branchType: branch
                    },
                    index: 0
                };
            }
        }
        
        // 2. 檢查是否在巢狀卡片之間
        const nestedCards = e.target.closest('.nested-cards');
        if (nestedCards) {
            const parentCard = nestedCards.closest('.tafl-card');
            const branch = nestedCards.dataset.branch;
            
            if (parentCard && branch) {
                const cards = Array.from(nestedCards.querySelectorAll(':scope > .tafl-card'));
                const index = this.calculateIndexFromPosition(e, cards);
                
                return {
                    nested: {
                        parentCardId: parentCard.dataset.cardId,
                        branchType: branch
                    },
                    index: index
                };
            }
        }
        
        // 3. 主流程位置
        const dropZone = document.getElementById('canvas-drop-zone');
        if (!dropZone) {
            // 如果 drop zone 不存在，返回默認位置
            return {
                nested: null,
                index: 0
            };
        }
        const mainCards = Array.from(dropZone.querySelectorAll(':scope > .tafl-card'));
        const index = this.calculateIndexFromPosition(e, mainCards);
        
        return {
            nested: null,
            index: index
        };
    }
    
    /**
     * 輔助方法：根據滑鼠位置計算插入索引
     */
    calculateIndexFromPosition(e, cards) {
        const mouseY = e.clientY;
        
        // 如果沒有卡片，返回 0
        if (cards.length === 0) {
            return 0;
        }
        
        // 特殊處理：檢查是否在第一張卡片的上方（增加檢測區域）
        const firstCard = cards[0];
        if (!firstCard.classList.contains('dragging')) {
            const firstRect = firstCard.getBoundingClientRect();
            // 增加頂部檢測區域到卡片頂部 + 30px
            if (mouseY < firstRect.top + 30) {
                return 0;
            }
        }
        
        // 檢查每張卡片的中間位置
        for (let i = 0; i < cards.length; i++) {
            // 跳過正在拖動的卡片
            if (cards[i].classList.contains('dragging')) continue;
            
            const rect = cards[i].getBoundingClientRect();
            if (mouseY < rect.top + rect.height / 2) {
                return i;
            }
        }
        
        // 特殊處理：檢查是否在最後一張卡片的下方
        const lastCard = cards[cards.length - 1];
        if (!lastCard.classList.contains('dragging')) {
            const lastRect = lastCard.getBoundingClientRect();
            // 確保在最後一張卡片下方時返回正確的索引
            if (mouseY >= lastRect.bottom - 30) {
                return cards.length;
            }
        }
        
        return cards.length; // 插入到最後
    }
    
    /**
     * UNIFIED DROP HANDLER
     * 統一處理所有放置操作（新增和移動）
     */
    handleUnifiedDrop(position) {
        // 保存 dragData 因為可能會被清空
        const dragDataCopy = { ...this.dragData };
        
        if (!dragDataCopy || !dragDataCopy.type) {
            console.warn('No drag data available for drop');
            return;
        }
        
        if (dragDataCopy.type === 'verb') {
            // 新增卡片
            this.addCardAtPosition(dragDataCopy.verb, position);
        } else if (dragDataCopy.type === 'card') {
            // 移動現有卡片（使用 Store 的統一邏輯）
            console.log('🚀 Moving card:', dragDataCopy.cardId, 'to position:', position);
            
            // 在移動前清理拖曳狀態，確保更新不會被跳過
            this.draggedElement = null;
            this.dragData = null;
            document.body.classList.remove('dragging');
            
            taflFlowStore.moveCard(dragDataCopy.cardId, position);
            // 不再手動調用 refreshCanvas，由 card:moved 事件處理
            
            // 重新選擇移動的卡片
            setTimeout(() => {
                const movedCard = document.querySelector(`[data-card-id="${dragDataCopy.cardId}"]`);
                if (movedCard) {
                    movedCard.classList.add('just-moved');
                    setTimeout(() => movedCard.classList.remove('just-moved'), 500);
                    this.selectCard(movedCard);
                }
            }, 100);
            
            this.markDirty();
            this.showNotification('Card moved successfully', 'is-success');
        }
    }
    
    /**
     * UNIFIED DROP INDICATOR UPDATE
     * 統一更新放置指示器
     */
    updateUnifiedDropIndicator(e) {
        const position = this.calculateUnifiedDropPosition(e);
        if (!position) return;
        
        // 檢查是否需要更新（避免重複創建）
        const positionKey = JSON.stringify(position);
        if (this.lastIndicatorPosition === positionKey) {
            return; // 位置沒變，不需要更新
        }
        this.lastIndicatorPosition = positionKey;
        
        // 移除所有現有的插入線
        this.hideDropIndicators();
        
        // 創建新的插入線
        const insertionLine = this.createInsertionLine();
        
        if (position.nested) {
            // 在巢狀結構中顯示
            const parentCard = document.querySelector(`[data-card-id="${position.nested.parentCardId}"]`);
            const nestedContainer = parentCard?.querySelector(`.nested-cards[data-branch="${position.nested.branchType}"]`);
            
            if (nestedContainer) {
                const nestedCards = Array.from(nestedContainer.querySelectorAll(':scope > .tafl-card'));
                if (position.index >= nestedCards.length) {
                    nestedContainer.appendChild(insertionLine);
                } else {
                    nestedCards[position.index].insertAdjacentElement('beforebegin', insertionLine);
                }
            }
        } else {
            // 在主流程中顯示
            const dropZone = document.getElementById('canvas-drop-zone');
            if (!dropZone) {
                // 如果 drop zone 不存在，直接返回
                return;
            }
            const mainCards = Array.from(dropZone.querySelectorAll(':scope > .tafl-card'));
            
            if (mainCards.length === 0) {
                // 畫布為空，顯示在 drop zone
                // dropZone 已經在上面宣告了
                if (dropZone) {
                    dropZone.classList.add('drag-over');
                }
            } else if (position.index >= mainCards.length) {
                // 在最後位置顯示引導線
                // 確保引導線在最後一張卡片之後
                const lastCard = mainCards[mainCards.length - 1];
                if (lastCard) {
                    lastCard.insertAdjacentElement('afterend', insertionLine);
                } else {
                    dropZone.appendChild(insertionLine);
                }
            } else if (position.index === 0) {
                // 在第一個位置顯示引導線
                mainCards[0].insertAdjacentElement('beforebegin', insertionLine);
            } else {
                // 在中間位置顯示引導線
                mainCards[position.index].insertAdjacentElement('beforebegin', insertionLine);
            }
        }
        
        // 激活插入線動畫
        requestAnimationFrame(() => {
            const line = document.querySelector('.insertion-line');
            if (line) {
                line.classList.add('active');
            }
        });
    }
    
    showAllDropZones() {
        const canvas = document.getElementById('flow-canvas');
        const dropZones = canvas.querySelectorAll('.drop-zone');
        
        dropZones.forEach(zone => {
            zone.classList.add('visible');
        });
        
        // Show main drop zone if no cards
        const flow = taflFlowStore.getFlow();
        if (flow.flow.length === 0) {
            const mainDropZone = document.getElementById('canvas-drop-zone');
            mainDropZone.classList.add('drag-active');
        }
        
        // Show nested drop zones
        canvas.querySelectorAll('.nested-drop-zone').forEach(zone => {
            zone.classList.add('visible');
        });
    }
    
    hideAllDropZones() {
        const canvas = document.getElementById('flow-canvas');
        
        canvas.querySelectorAll('.drop-zone, .nested-drop-zone').forEach(zone => {
            zone.classList.remove('visible', 'drag-over');
        });
        
        const mainDropZone = document.getElementById('canvas-drop-zone');
        if (mainDropZone) {
            mainDropZone.classList.remove('drag-active', 'drag-over');
        }
    }
    
    // Generate unique ID for cards without IDs
    generateId() {
        // Use the store's generateId which properly manages the counter
        return taflFlowStore.generateId();
    }
    
    updateDropIndicators(e) {
        const dropZone = document.getElementById('canvas-drop-zone');
        if (!dropZone) {
            // 如果 drop zone 不存在，直接返回
            return;
        }
        const cards = Array.from(dropZone.querySelectorAll('.tafl-card:not(.dragging)'));
        
        // If no cards, show drop zone in canvas
        if (cards.length === 0) {
            // dropZone 已經在上面宣告了
            if (dropZone) {
                dropZone.classList.add('drag-over');
            }
            // Clear any existing insertion lines
            this.hideDropIndicators();
            return;
        }
        
        const mouseY = e.clientY;
        let insertionIndex = -1;
        
        // Find where to show insertion line
        for (let i = 0; i < cards.length; i++) {
            const card = cards[i];
            const rect = card.getBoundingClientRect();
            const cardMiddle = rect.top + rect.height / 2;
            
            if (mouseY < cardMiddle) {
                insertionIndex = i;
                break;
            }
        }
        
        // Check if position has changed
        if (this.lastInsertionIndex === insertionIndex) {
            // Position hasn't changed, don't update
            return;
        }
        
        this.lastInsertionIndex = insertionIndex;
        
        // Clear existing indicators only when position changes
        this.hideDropIndicators();
        
        // Create and show insertion line
        const insertionLine = this.createInsertionLine();
        
        if (insertionIndex === -1) {
            // Insert at end
            const lastCard = cards[cards.length - 1];
            lastCard.insertAdjacentElement('afterend', insertionLine);
        } else if (insertionIndex === 0) {
            // Insert at beginning
            const firstCard = cards[0];
            firstCard.insertAdjacentElement('beforebegin', insertionLine);
        } else {
            // Insert between cards
            const targetCard = cards[insertionIndex];
            targetCard.insertAdjacentElement('beforebegin', insertionLine);
        }
        
        // Add active class immediately (no setTimeout needed)
        insertionLine.classList.add('active');
    }
    
    createInsertionLine() {
        const line = document.createElement('div');
        line.className = 'insertion-line';
        line.innerHTML = `
            <div class="insertion-line-inner">
                <div class="insertion-dot"></div>
                <div class="insertion-text">Drop here</div>
                <div class="insertion-dot"></div>
            </div>
        `;
        return line;
    }
    
    hideDropIndicators() {
        document.querySelectorAll('.insertion-line').forEach(line => {
            line.remove();
        });
        // Also remove any drop-indicator class elements (legacy)
        document.querySelectorAll('.drop-indicator').forEach(line => {
            line.remove();
        });
        // Reset the last insertion index
        this.lastInsertionIndex = undefined;
        this.lastIndicatorPosition = undefined;
    }
    
    refreshCardDragHandlers() {
        // All cards are already draggable through their draggable="true" attribute
        // Global event delegation handles all drag events - no need for individual setup
        // Just ensure all cards have the draggable attribute set
        const canvas = document.getElementById('flow-canvas');
        const allCards = canvas.querySelectorAll('.tafl-card'); // Include ALL cards, not just non-nested
        
        allCards.forEach(cardEl => {
            // Ensure draggable is set (should already be set by createCardElementSimple)
            if (!cardEl.draggable) {
                cardEl.draggable = true;
            }
        });
    }
    
    createInitialDropZone() {
        const canvas = document.getElementById('flow-canvas');
        const dropZone = document.getElementById('canvas-drop-zone');
        
        if (!dropZone) {
            console.error('Canvas drop zone not found');
            return;
        }
        
        // Set up main drop zone events
        dropZone.addEventListener('dragover', (e) => {
            e.preventDefault();
            e.stopPropagation();
            dropZone.classList.add('drag-over');
        });
        
        dropZone.addEventListener('dragleave', (e) => {
            if (!dropZone.contains(e.relatedTarget)) {
                dropZone.classList.remove('drag-over');
            }
        });
        
        // Drop handler already exists in setupUnifiedDragDrop, no need for duplicate
    }
    
    /**
     * CARD MANAGEMENT WITH DRAG & DROP SUPPORT
     */
    addCard(verb) {
        // Always add to main canvas at the end (as requested)
        this.addCardAtPosition(verb, { index: taflFlowStore.getFlow().flow.length, nested: null });
    }
    
    addCardAtPosition(verb, position) {
        if (!this.verbDefinitions[verb]) {
            console.error('Unknown verb:', verb);
            return;
        }
        
        const cardId = this.generateId();
        
        // Create TAFL v1.1 compliant card data (verb as key, not property)
        const cardData = {
            id: cardId,
            [verb]: this.getDefaultParams(verb)[verb]
        };
        
        // Insert at specific position  
        if (position.nested) {
            // Add to nested structure
            this.addToNestedStructure(position.nested.parentCardId, position.nested.branchType, cardData);
        } else {
            // Insert at specific index
            const flow = taflFlowStore.getFlow();
            const newFlow = [...flow.flow];
            newFlow.splice(position.index, 0, cardData);
            taflFlowStore.updateFlow({ ...flow, flow: newFlow });
        }
        
        // Cards are now properly ordered - event will trigger refresh
        
        // Select and animate the new card
        const newCardElement = document.querySelector(`[data-card-id="${cardId}"]`);
        if (newCardElement) {
            newCardElement.classList.add('just-added');
            setTimeout(() => {
                newCardElement.classList.remove('just-added');
                newCardElement.classList.add('pulse-once');
            }, 100);
            setTimeout(() => newCardElement.classList.remove('pulse-once'), 600);
            
            this.selectCard(newCardElement);
        }
        
        this.markDirty();
        this.updateUI();
        
        // Ensure new cards are draggable
        setTimeout(() => {
            this.refreshCardDragHandlers();
            // 使用全域事件委派，不需要個別設置
            // this.setupNestedCardInteractions();
        }, 100);
        
        console.log('✅ Added card:', verb, 'at position:', position);
    }
    
    moveCardToPosition(cardId, position) {
        // 簡化版：直接使用 Store 的統一邏輯
        console.log('🚀 Moving card:', cardId, 'to position:', position);
        
        // 檢查是否從嵌套移動
        const cardEl = document.querySelector(`[data-card-id="${cardId}"]`);
        const wasNested = cardEl && cardEl.closest('.nested-cards') !== null;
        console.log(`  Was nested: ${wasNested}, Target nested: ${!!position.nested}`);
        
        // 在移動前清理拖曳狀態，確保更新不會被跳過
        this.draggedElement = null;
        this.dragData = null;
        document.body.classList.remove('dragging');
        
        taflFlowStore.moveCard(cardId, position);
        // 不再手動調用 refreshCanvas，由 card:moved 事件處理
        
        // 視覺回饋
        setTimeout(() => {
            const movedCard = document.querySelector(`[data-card-id="${cardId}"]`);
            if (movedCard) {
                movedCard.classList.add('just-moved');
                setTimeout(() => movedCard.classList.remove('just-moved'), 500);
                this.selectCard(movedCard);
            }
        }, 100);
        
        this.markDirty();
        this.showNotification('Card moved successfully', 'is-success');
    }
    
    /**
     * 簡化版卡片創建（不綁定個別事件，使用事件委派）
     */
    createCardElementSimple(step) {
        // TAFL v1.1 Fix: Extract verb from step object
        const verb = step ? Object.keys(step).find(key => 
            this.verbDefinitions && this.verbDefinitions[key]
        ) : null;
        
        if (!verb) {
            console.warn('No valid verb found in step:', step);
            const emptyCard = document.createElement('div');
            emptyCard.className = 'tafl-card empty-card';
            emptyCard.innerHTML = '<div class="notification is-warning">Invalid step</div>';
            return emptyCard;
        }
        
        const verbDef = this.verbDefinitions[verb];
        
        // 創建卡片元素（設置 draggable 屬性）
        const cardEl = document.createElement('div');
        cardEl.className = 'tafl-card';
        cardEl.dataset.cardId = step.id || this.generateId();
        cardEl.dataset.verb = verb;
        cardEl.draggable = true; // 確保所有卡片都可拖動
        
        cardEl.innerHTML = `
            <div class="tafl-card-header">
                <div class="card-drag-handle" title="Drag to reorder">
                    <i class="fas fa-grip-vertical"></i>
                </div>
                <span class="tag verb-badge is-${verb}">${verbDef.name}</span>
                <h3 class="card-title">${this.getCardTitle(step)}</h3>
                <div class="card-controls">
                    <button class="button is-small card-duplicate-btn" title="Duplicate">
                        <span class="icon is-small">
                            <i class="fas fa-copy"></i>
                        </span>
                    </button>
                    <button class="button is-small card-delete-btn" title="Delete">
                        <span class="icon is-small">
                            <i class="fas fa-trash"></i>
                        </span>
                    </button>
                </div>
            </div>
            <div class="tafl-card-body">
                ${this.renderCardParams(step)}
            </div>
            ${this.renderNestedCardsSimple(step)}
        `;
        
        // 設置 comment tooltip（如果有）
        if (step.comment) {
            cardEl.title = step.comment;
        }
        
        return cardEl;
    }
    
    /**
     * 簡化版巢狀卡片渲染（遞迴渲染所有層級）
     */
    renderNestedCardsSimple(cardData) {
        const verb = this.getCardVerb(cardData);
        const params = cardData[verb];
        let html = '';
        
        if (typeof params === 'object') {
            if (verb === 'if') {
                if (params.then && Array.isArray(params.then)) {
                    html += this.renderNestedBranchSimple('then', params.then);
                }
                if (params.else && Array.isArray(params.else)) {
                    html += this.renderNestedBranchSimple('else', params.else);
                }
            } else if (verb === 'for' && params.do && Array.isArray(params.do)) {
                html += this.renderNestedBranchSimple('do', params.do);
            } else if (verb === 'switch') {
                if (params.cases && Array.isArray(params.cases)) {
                    params.cases.forEach((caseItem, index) => {
                        html += this.renderNestedBranchSimple(`case-${index}`, caseItem.do || []);
                    });
                }
                if (params.default && Array.isArray(params.default)) {
                    html += this.renderNestedBranchSimple('default', params.default);
                }
            }
        }
        
        return html;
    }
    
    /**
     * 簡化版巢狀分支渲染
     */
    renderNestedBranchSimple(branchType, cards) {
        if (!Array.isArray(cards)) return '';
        
        const branchClass = branchType.replace(/[^a-zA-Z0-9]/g, '-');
        
        let html = `
            <div class="nested-cards ${branchClass}-cards" data-branch="${branchType}">
                <div class="branch-label">${branchType}</div>
        `;
        
        if (cards.length === 0) {
            // 空的巢狀放置區
            html += `
                <div class="nested-drop-zone empty-zone" data-branch="${branchType}">
                    <div class="nested-drop-content">
                        <i class="mdi mdi-plus"></i>
                        <span>Add cards here</span>
                    </div>
                </div>
            `;
        } else {
            // 遞迴渲染巢狀卡片
            cards.forEach(nestedCard => {
                const nestedElement = this.createCardElementSimple(nestedCard);
                html += nestedElement.outerHTML;
            });
        }
        
        html += `</div>`;
        return html;
    }
    
    // Removed unused createCardElement function - using createCardElementSimple instead
    
    setupCommentTooltip(cardEl, cardData) {
        // Check if card has a comment
        if (!cardData.comment) return;
        
        // Create tooltip element
        let tooltip = null;
        
        const showTooltip = (e) => {
            // Don't show tooltip when interacting with inputs
            if (e.target.closest('input') || e.target.closest('textarea') || e.target.closest('button')) {
                return;
            }
            
            // Create tooltip if not exists
            if (!tooltip) {
                tooltip = document.createElement('div');
                tooltip.className = 'tafl-comment-tooltip';
                tooltip.innerHTML = `
                    <div class="tooltip-arrow"></div>
                    <div class="tooltip-content">
                        <strong>Comment:</strong><br>
                        ${this.escapeHtml(cardData.comment)}
                    </div>
                `;
                tooltip.style.cssText = `
                    position: absolute;
                    background: #363636;
                    color: white;
                    padding: 8px 12px;
                    border-radius: 6px;
                    font-size: 14px;
                    max-width: 300px;
                    box-shadow: 0 4px 6px rgba(0,0,0,0.1);
                    z-index: 10000;
                    pointer-events: none;
                    opacity: 0;
                    transition: opacity 0.2s;
                `;
                document.body.appendChild(tooltip);
            }
            
            // Position tooltip above the card
            const rect = cardEl.getBoundingClientRect();
            tooltip.style.left = rect.left + (rect.width / 2) + 'px';
            tooltip.style.top = (rect.top - 10) + 'px';
            tooltip.style.transform = 'translate(-50%, -100%)';
            
            // Show tooltip with animation
            requestAnimationFrame(() => {
                tooltip.style.opacity = '1';
            });
        };
        
        const hideTooltip = () => {
            if (tooltip) {
                tooltip.style.opacity = '0';
                setTimeout(() => {
                    if (tooltip && tooltip.parentNode) {
                        tooltip.parentNode.removeChild(tooltip);
                        tooltip = null;
                    }
                }, 200);
            }
        };
        
        // Add hover event listeners
        cardEl.addEventListener('mouseenter', showTooltip);
        cardEl.addEventListener('mouseleave', hideTooltip);
        
        // Clean up on card removal
        cardEl.addEventListener('remove', hideTooltip);
    }
    
    /**
     * NESTED STRUCTURES SUPPORT
     */
    renderNestedCards(cardData) {
        // TAFL v1.1: Extract verb from object keys (verb is the key, not a property)
        const verb = this.getCardVerb(cardData);
        const params = cardData[verb];
        let html = '';
        
        if (typeof params === 'object') {
            if (verb === 'if') {
                if (params.then && Array.isArray(params.then)) {
                    html += this.renderNestedBranch('then', params.then);
                }
                if (params.else && Array.isArray(params.else)) {
                    html += this.renderNestedBranch('else', params.else);
                }
            } else if (verb === 'for' && params.do && Array.isArray(params.do)) {
                html += this.renderNestedBranch('do', params.do);
            } else if (verb === 'switch') {
                if (params.cases && Array.isArray(params.cases)) {
                    params.cases.forEach((caseItem, index) => {
                        html += this.renderNestedBranch(`case-${index}`, caseItem.do || []);
                    });
                }
                if (params.default && Array.isArray(params.default)) {
                    html += this.renderNestedBranch('default', params.default);
                }
            }
        }
        
        return html;
    }
    
    renderNestedBranch(branchType, cards) {
        if (!Array.isArray(cards)) return '';
        
        const branchClass = branchType.replace(/[^a-zA-Z0-9]/g, '-');
        
        let html = `
            <div class="nested-cards ${branchClass}-cards" data-branch="${branchType}">
                <div class="branch-label">${branchType}</div>
        `;
        
        // Only show drop zone if empty or during drag
        if (cards.length === 0) {
            html += `
                <div class="nested-drop-zone empty-zone" data-branch="${branchType}">
                    <div class="nested-drop-content">
                        <i class="mdi mdi-plus"></i>
                        <span>Add cards here</span>
                    </div>
                </div>
            `;
        } else {
            // Add horizontal drop indicator (initially hidden)
            html += `<div class="drop-indicator horizontal" data-branch="${branchType}" data-position="0"></div>`;
            
            cards.forEach((nestedCard, index) => {
                html += this.renderNestedCard(nestedCard);
                // Add drop indicator after each card
                html += `<div class="drop-indicator horizontal" data-branch="${branchType}" data-position="${index + 1}"></div>`;
            });
        }
        
        html += `</div>`;
        
        return html;
    }
    
    renderNestedCard(cardData) {
        // For nested cards, render a simplified version
        const verb = Object.keys(cardData).find(key => key !== 'id' && this.verbDefinitions[key]);
        if (!verb) return '';
        
        const verbDef = this.verbDefinitions[verb];
        
        // Ensure card has an ID
        if (!cardData.id) {
            cardData.id = this.generateId();
        }
        
        // Create the full card data structure for rendering
        const fullCardData = {
            [verb]: cardData[verb],
            id: cardData.id
        };
        
        return `
            <div class="tafl-card nested" data-card-id="${cardData.id}" data-verb="${verb}">
                <div class="tafl-card-header">
                    <div class="card-drag-handle" title="Drag to reorder">
                        <i class="fas fa-grip-vertical"></i>
                    </div>
                    <span class="tag verb-badge is-${verb}">${verbDef.name}</span>
                    <h3 class="card-title">${this.getCardTitle({verb, [verb]: cardData[verb]})}</h3>
                    <div class="card-controls">
                        <button class="button is-small card-duplicate-btn" title="Duplicate">
                            <span class="icon is-small">
                                <i class="fas fa-copy"></i>
                            </span>
                        </button>
                        <button class="button is-small card-delete-btn" title="Delete">
                            <span class="icon is-small">
                                <i class="fas fa-trash"></i>
                            </span>
                        </button>
                    </div>
                </div>
                <div class="tafl-card-body">
                    ${this.renderCardParams(fullCardData)}
                </div>
                ${this.renderNestedCards(fullCardData)}
            </div>
        `;
    }
    
    getNestedDropTarget(element) {
        return element.closest('.nested-drop-zone');
    }
    
    getBranchType(nestedArea) {
        return nestedArea?.dataset?.branch || null;
    }
    
    addToNestedStructure(parentCardId, branchType, cardData) {
        // Ensure the card has an ID
        if (!cardData.id) {
            cardData.id = this.generateId();
        }
        
        // Get the current flow
        const flow = taflFlowStore.getFlow();
        
        // Create position object
        const position = {
            nested: {
                parentCardId: parentCardId,
                branchType: branchType
            }
        };
        
        // Use the store's method to insert into nested structure
        const updatedFlow = taflFlowStore.insertCardInNested(flow.flow, cardData, position);
        
        if (!updatedFlow) {
            console.error('Failed to add card to nested structure');
            return;
        }
        
        // Update the flow in the store
        taflFlowStore.updateFlow({ ...flow, flow: updatedFlow });
        
        console.log('✅ Added card to nested structure:', parentCardId, branchType, cardData);
    }
    
    addToNestedStructureAtPosition(parentCardId, branchType, cardData, position) {
        // Ensure the card has an ID
        if (!cardData.id) {
            cardData.id = this.generateId();
        }
        
        // Get the current flow
        const flow = taflFlowStore.getFlow();
        
        // Create position object
        const positionObj = {
            nested: {
                parentCardId: parentCardId,
                branchType: branchType
            },
            index: position
        };
        
        // Use the store's method to insert into nested structure
        const updatedFlow = taflFlowStore.insertCardInNested(flow.flow, cardData, positionObj);
        
        if (!updatedFlow) {
            console.error('Failed to add card to nested structure at position');
            return;
        }
        
        // Update the flow in the store
        taflFlowStore.updateFlow({ ...flow, flow: updatedFlow });
        
        console.log('✅ Added card to nested structure at position:', parentCardId, branchType, position, cardData);
    }
    
    /**
     * REST OF THE CLASS METHODS (keeping existing functionality)
     */
    
    initializeVariables() {
        // Initialize with empty variables object
        // Users can add variables as needed
        taflFlowStore.getFlow().variables = {};
        // SimpleTAFLPanels handles UI updates via store events
        // this.refreshVariablesUI(); // Deprecated - handled by SimpleTAFLPanels
    }
    
    addVariable() {
        const name = prompt('Variable name:');
        if (name) {
            const flow = taflFlowStore.getFlow();
            if (!flow.variables) {
                flow.variables = {};
            }
            if (!flow.variables.hasOwnProperty(name)) {
                const value = prompt('Initial value:', '""');
                try {
                    flow.variables[name] = JSON.parse(value);
                } catch (e) {
                    flow.variables[name] = value;
                }
                // Emit the event that SimpleTAFLPanels listens to
                taflFlowStore.emit('variables:changed');
                this.markDirty();
                this.showNotification('Variable added successfully', 'success');
                
                // Switch to Variables tab to show the newly added variable
                this.switchPropertiesTab('variables');
            } else {
                this.showNotification('Variable already exists', 'warning');
            }
        }
    }
    
    removeVariable(name) {
        const flow = taflFlowStore.getFlow();
        if (flow.variables && flow.variables[name] !== undefined) {
            delete flow.variables[name];
            // Use the store's updateVariables method which will emit the event
            taflFlowStore.updateVariables(flow.variables);
            this.markDirty();
            this.showNotification('Variable removed successfully', 'success');
        }
    }
    
    // TAFL v1.1: Add preload data item
    async addPreload() {
        // Use simple prompts for consistency
        const key = prompt('Variable name to store the preloaded data:');
        if (!key) return;
        
        const target = prompt('Query target (locations/racks/tasks/agvs/products):');
        if (!target) return;
        
        const whereStr = prompt('Where conditions (optional, e.g. "status: active, room_id: 1"):', '');
        
        // Parse where conditions
        const where = whereStr ? this.parseWhereConditions(whereStr) : {};
        
        // Create preload config
        const preloadConfig = {
            query: {
                target: target,
                ...(Object.keys(where).length > 0 && { where: where })
            }
        };
        
        // Add to flow
        const flow = taflFlowStore.getFlow();
        if (!flow.preload) {
            flow.preload = {};
        }
        
        // Check if key already exists
        if (flow.preload[key]) {
            if (!await taflModals.confirmOverwrite('Preload', key)) {
                return;
            }
        }
        
        // Add preload query
        flow.preload[key] = preloadConfig;
        
        // Trigger update
        taflFlowStore.updateFlow(flow);
        this.markDirty();
        this.generateYAML();
    }
    
    // Save new preload
    async saveNewPreload() {
        const key = document.getElementById('new-preload-key').value.trim();
        const target = document.getElementById('new-preload-target').value;
        const whereStr = document.getElementById('new-preload-where').value.trim();
        const comment = document.getElementById('new-preload-comment').value.trim();
        
        if (!key || !target) {
            taflModals.showRequiredFieldsError(['Store As', 'Target']);
            return;
        }
        
        // Check if key already exists
        if (taflFlowStore.getFlow().preload && taflFlowStore.getFlow().preload[key]) {
            taflModals.alert(`A preload with the key "${key}" already exists`, 'warning');
            return;
        }
        
        // Parse where conditions
        const where = this.parseWhereConditions(whereStr);
        
        // Create preload config
        const preloadConfig = {
            query: {
                target: target,
                ...(Object.keys(where).length > 0 && { where: where }),
                ...(comment && { comment: comment })
            }
        };
        
        // Add to flow
        if (!taflFlowStore.getFlow().preload) {
            taflFlowStore.getFlow().preload = {};
        }
        
        // Add preload query
        taflFlowStore.getFlow().preload[key] = preloadConfig;
        
        // Close modal and trigger panel update via store event
        document.querySelector('.modal.is-active').remove();
        taflFlowStore.emit('flow:changed');
        this.markDirty();
        this.generateYAML();
    }
    
    // TAFL v1.1: Add business rule
    async addRule() {
        // Use simple prompts since we don't have input fields anymore
        const name = prompt('Rule name:');
        if (!name) return;
        
        const type = prompt('Rule type (string/number/boolean/object):', 'string');
        if (!type) return;
        
        let value;
        if (type === 'object') {
            const objText = prompt('Enter JSON object:', '{}');
            if (!objText) return;
            try {
                value = JSON.parse(objText);
            } catch (e) {
                taflModals.showValidationError('Invalid JSON format');
                return;
            }
        } else {
            const rawValue = prompt('Rule value:');
            if (rawValue === null) return;
            
            // Parse based on type
            if (type === 'number') {
                value = Number(rawValue);
                if (isNaN(value)) {
                    taflModals.showValidationError('Invalid number format');
                    return;
                }
            } else if (type === 'boolean') {
                value = rawValue.toLowerCase() === 'true';
            } else {
                value = rawValue;
            }
        }
        
        // Initialize rules if needed
        const flow = taflFlowStore.getFlow();
        if (!flow.rules) {
            flow.rules = {};
        }
        
        // Check if rule already exists
        if (flow.rules[name]) {
            if (!await taflModals.confirmOverwrite('Rule', name)) {
                return;
            }
        }
        
        // Store rule
        flow.rules[name] = value;
        
        // Trigger panel update via store events
        taflFlowStore.updateFlow(flow);
        this.markDirty();
        this.generateYAML();
    }
    
    // Save new rule
    async saveNewRule() {
        const name = document.getElementById('new-rule-name').value.trim();
        const value = document.getElementById('new-rule-value').value.trim();
        const description = document.getElementById('new-rule-description').value.trim();
        
        if (!name || !value) {
            taflModals.showRequiredFieldsError(['Rule name', 'value']);
            return;
        }
        
        // Check if rule already exists
        if (taflFlowStore.getFlow().rules && taflFlowStore.getFlow().rules[name]) {
            taflModals.alert(`A rule with the name "${name}" already exists`, 'warning');
            return;
        }
        
        // Parse value - try JSON first, then number, then boolean, finally string
        let parsedValue = value;
        try {
            // Try to parse as JSON (for objects and arrays)
            parsedValue = JSON.parse(value);
        } catch (e) {
            // Not JSON, try other types
            if (!isNaN(value)) {
                parsedValue = Number(value);
            } else if (value === 'true' || value === 'false') {
                parsedValue = value === 'true';
            }
            // Otherwise keep as string
        }
        
        // Initialize rules if needed
        if (!taflFlowStore.getFlow().rules) {
            taflFlowStore.getFlow().rules = {};
        }
        
        // Store rule with optional description
        if (description) {
            taflFlowStore.getFlow().rules[name] = {
                value: parsedValue,
                description: description
            };
        } else {
            taflFlowStore.getFlow().rules[name] = parsedValue;
        }
        
        // Close modal and trigger panel update via store events
        document.querySelector('.modal.is-active').remove();
        taflFlowStore.emit('flow:changed');
        this.markDirty();
        this.generateYAML();
    }
    
    // TAFL v1.1: Edit business rule
    editRule(name) {
        const rule = taflFlowStore.getFlow().rules[name];
        if (rule === undefined) return;
        
        // Extract value and description from rule
        let ruleValue = rule;
        let ruleDescription = '';
        if (typeof rule === 'object' && rule !== null && 'value' in rule) {
            ruleValue = rule.value;
            ruleDescription = rule.description || '';
        }
        
        // Format value for display
        let valueStr = ruleValue;
        if (typeof ruleValue === 'object') {
            valueStr = JSON.stringify(ruleValue, null, 2);
        } else if (typeof ruleValue === 'boolean') {
            valueStr = ruleValue ? 'true' : 'false';
        } else {
            valueStr = String(ruleValue);
        }
        
        // Show edit dialog
        const modal = document.createElement('div');
        modal.className = 'modal is-active';
        modal.innerHTML = `
            <div class="modal-background" onclick="this.parentElement.remove()"></div>
            <div class="modal-card">
                <header class="modal-card-head">
                    <p class="modal-card-title">Edit Business Rule</p>
                    <button class="delete" aria-label="close" onclick="this.closest('.modal').remove()"></button>
                </header>
                <section class="modal-card-body">
                    <div class="field">
                        <label class="label">Rule Name</label>
                        <div class="control">
                            <input class="input" type="text" id="edit-rule-name" value="${name}" placeholder="e.g., max_rack_capacity">
                        </div>
                    </div>
                    <div class="field">
                        <label class="label">Rule Value</label>
                        <div class="control">
                            <textarea class="textarea" id="edit-rule-value" rows="3" placeholder="Enter value">${valueStr}</textarea>
                        </div>
                        <p class="help">Examples: 10, true, "active", {"min": 1, "max": 5}</p>
                    </div>
                    <div class="field">
                        <label class="label">Description (Optional)</label>
                        <div class="control">
                            <input class="input" type="text" id="edit-rule-description" value="${ruleDescription}" placeholder="Description of this rule">
                        </div>
                    </div>
                </section>
                <footer class="modal-card-foot">
                    <button class="button is-success" onclick="taflEditor.saveRuleEdit('${name}')">Save Changes</button>
                    <button class="button" onclick="this.closest('.modal').remove()">Cancel</button>
                </footer>
            </div>
        `;
        document.body.appendChild(modal);
    }
    
    // Save rule edit
    saveRuleEdit(oldName) {
        const newName = document.getElementById('edit-rule-name').value.trim();
        const value = document.getElementById('edit-rule-value').value.trim();
        const description = document.getElementById('edit-rule-description').value.trim();
        
        if (!newName || !value) {
            taflModals.showRequiredFieldsError(['Rule name', 'value']);
            return;
        }
        
        // Parse value
        let parsedValue = value;
        try {
            parsedValue = JSON.parse(value);
        } catch (e) {
            if (!isNaN(value)) {
                parsedValue = Number(value);
            } else if (value === 'true' || value === 'false') {
                parsedValue = value === 'true';
            }
        }
        
        // If name changed, delete old one
        if (oldName !== newName && taflFlowStore.getFlow().rules[oldName] !== undefined) {
            delete taflFlowStore.getFlow().rules[oldName];
        }
        
        // Set new/updated rule
        if (!taflFlowStore.getFlow().rules) {
            taflFlowStore.getFlow().rules = {};
        }
        
        if (description) {
            taflFlowStore.getFlow().rules[newName] = {
                value: parsedValue,
                description: description
            };
        } else {
            taflFlowStore.getFlow().rules[newName] = parsedValue;
        }
        
        // Close modal and trigger panel update via store events
        document.querySelector('.modal.is-active').remove();
        taflFlowStore.emit('flow:changed');
        this.markDirty();
        this.generateYAML();
    }
    
    // Helper method to parse where condition string
    parseWhereCondition(whereStr) {
        const conditions = {};
        const parts = whereStr.split(',');
        parts.forEach(part => {
            const [key, value] = part.split(':').map(s => s.trim());
            if (key && value) {
                conditions[key] = value;
            }
        });
        return conditions;
    }
    
    // ============================================
    // Legacy Panel Functions (Deprecated)
    // All panel innerHTML updates now handled by SimpleTAFLPanels
    // These modal functions kept for backward compatibility
    // ============================================
    
    // TAFL v1.1: Edit preload item (Modal interaction)
    editPreload(key) {
        const preload = taflFlowStore.getFlow().preload[key];
        if (!preload) return;
        
        // Show edit dialog with current values
        const modal = document.createElement('div');
        modal.className = 'modal is-active';
        modal.innerHTML = `
            <div class="modal-background" onclick="this.parentElement.remove()"></div>
            <div class="modal-card">
                <header class="modal-card-head">
                    <p class="modal-card-title">Edit Preload Query</p>
                    <button class="delete" aria-label="close" onclick="this.closest('.modal').remove()"></button>
                </header>
                <section class="modal-card-body">
                    <div class="field">
                        <label class="label">Store As (Variable Name)</label>
                        <div class="control">
                            <input class="input" type="text" id="edit-preload-key" value="${key}" placeholder="e.g., available_locations">
                        </div>
                    </div>
                    <div class="field">
                        <label class="label">Target</label>
                        <div class="control">
                            <div class="select is-fullwidth">
                                <select id="edit-preload-target">
                                    <option value="locations" ${preload.query?.target === 'locations' ? 'selected' : ''}>locations</option>
                                    <option value="racks" ${preload.query?.target === 'racks' ? 'selected' : ''}>racks</option>
                                    <option value="tasks" ${preload.query?.target === 'tasks' ? 'selected' : ''}>tasks</option>
                                    <option value="agvs" ${preload.query?.target === 'agvs' ? 'selected' : ''}>agvs</option>
                                </select>
                            </div>
                        </div>
                    </div>
                    <div class="field">
                        <label class="label">Where Conditions</label>
                        <div class="control">
                            <textarea class="textarea" id="edit-preload-where" rows="3" placeholder="e.g., room_id: 1, status: available">${
                                typeof preload.query?.where === 'string' 
                                    ? preload.query.where 
                                    : typeof preload.query?.where === 'object'
                                        ? Object.entries(preload.query.where).map(([k, v]) => `${k}: ${v}`).join(', ')
                                        : ''
                            }</textarea>
                        </div>
                        <p class="help">Format: key: value, key2: value2 (comma-separated)</p>
                    </div>
                    <div class="field">
                        <label class="label">Comment (Optional)</label>
                        <div class="control">
                            <input class="input" type="text" id="edit-preload-comment" value="${preload.query?.comment || ''}" placeholder="Description of this preload query">
                        </div>
                    </div>
                </section>
                <footer class="modal-card-foot">
                    <button class="button is-success" onclick="taflEditor.savePreloadEdit('${key}')">Save changes</button>
                    <button class="button" onclick="this.closest('.modal').remove()">Cancel</button>
                </footer>
            </div>
        `;
        document.body.appendChild(modal);
    }
    
    // TAFL v1.1: Save preload edit
    savePreloadEdit(oldKey) {
        const newKey = document.getElementById('edit-preload-key').value.trim();
        const target = document.getElementById('edit-preload-target').value;
        const whereStr = document.getElementById('edit-preload-where').value.trim();
        const comment = document.getElementById('edit-preload-comment').value.trim();
        
        if (!newKey || !target) {
            taflModals.showRequiredFieldsError(['Store As', 'Target']);
            return;
        }
        
        // Parse where conditions
        const where = this.parseWhereConditions(whereStr);
        
        // Update or create new preload
        const preloadConfig = {
            query: {
                target: target,
                where: where,
                ...(comment && { comment: comment })
            }
        };
        
        // If key changed, delete old one
        if (oldKey !== newKey && taflFlowStore.getFlow().preload[oldKey]) {
            delete taflFlowStore.getFlow().preload[oldKey];
        }
        
        // Set new/updated preload
        if (!taflFlowStore.getFlow().preload) {
            taflFlowStore.getFlow().preload = {};
        }
        taflFlowStore.getFlow().preload[newKey] = preloadConfig;
        
        // Close modal and trigger panel update via store event
        document.querySelector('.modal.is-active').remove();
        taflFlowStore.emit('flow:changed');
        this.markDirty();
        this.generateYAML();
    }

    // TAFL v1.1: Remove preload item
    async removePreload(key) {
        if (await taflModals.confirmDelete('preload query', key)) {
            const flow = taflFlowStore.getFlow();
            if (flow.preload && flow.preload[key]) {
                delete flow.preload[key];
                // Panel update handled by SimpleTAFLPanels via store events
                taflFlowStore.emit('flow:changed');
                this.markDirty();
                this.generateYAML();
            }
        }
    }

    // Removed: refreshSettingsUI - handled by SimpleTAFLPanels

    // Enhanced expression parser for TAFL v1.1
    parseExpression(expr) {
        if (!expr || typeof expr !== 'string') return expr;
        
        // Support for variable references: ${variable}
        // Support for preload data: ${preload.key}
        // Support for rules: ${rules.ruleName}
        // Support for math operations: ${value + 1}
        // Support for nested properties: ${object.property.subproperty}
        
        return expr.replace(/\$\{([^}]+)\}/g, (match, path) => {
            // Check if it's a math expression
            if (/[+\-*/]/.test(path)) {
                // For now, return as is - actual evaluation would happen at runtime
                return match;
            }
            
            // Parse the path
            const parts = path.split('.');
            const scope = parts[0];
            
            // Check different scopes (5-Level variable scope)
            if (scope === 'preload' && parts.length > 1) {
                return taflFlowStore.getFlow().preload[parts[1]] || match;
            } else if (scope === 'rules' && parts.length > 1) {
                const rule = taflFlowStore.getFlow().rules[parts[1]];
                return rule ? rule.action : match;
            } else if (taflFlowStore.getFlow().variables[scope] !== undefined) {
                return taflFlowStore.getFlow().variables[scope];
            }
            
            return match;
        });
    }
    
    // TAFL v1.1: Remove rule
    removeRule(name) {
        const flow = taflFlowStore.getFlow();
        if (flow.rules && flow.rules[name]) {
            delete flow.rules[name];
            // Panel update handled by SimpleTAFLPanels via store events
            taflFlowStore.emit('flow:changed');
            this.markDirty();
            this.showNotification('Rule removed successfully', 'success');
        }
    }
    
    renderVariables() {
        // SimpleTAFLPanels handles UI updates via store events
        // this.refreshVariablesUI(); // Deprecated - handled by SimpleTAFLPanels
    }
    
    updateSaveButton(isDirty) {
        const saveBtn = document.getElementById('save-flow-btn');
        if (saveBtn) {
            if (isDirty) {
                saveBtn.classList.add('is-warning');
                const icon = saveBtn.querySelector('.icon i');
                if (icon) {
                    icon.classList.remove('fa-save');
                    icon.classList.add('fa-exclamation-triangle');
                }
            } else {
                saveBtn.classList.remove('is-warning');
                const icon = saveBtn.querySelector('.icon i');
                if (icon) {
                    icon.classList.remove('fa-exclamation-triangle');
                    icon.classList.add('fa-save');
                }
            }
        }
    }
    
    // Removed: refreshVariablesUI - handled by SimpleTAFLPanels
    
    // setupModalControls() moved to tafl-editor-modals.js
    
    initializeCodeMirror() {
        // Initialize main YAML editor (in center)
        const yamlTextareaMain = document.getElementById('yaml-editor-main');
        if (yamlTextareaMain) {
            this.yamlEditor = CodeMirror.fromTextArea(yamlTextareaMain, {
                mode: 'yaml',
                theme: 'default',
                lineNumbers: true,
                lineWrapping: true,
                foldGutter: true,
                gutters: ['CodeMirror-linenumbers', 'CodeMirror-foldgutter'],
                height: '100%'
            });
            
            this.yamlEditor.on('change', () => {
                this.markDirty();
            });
            
            // Set height to fill container
            this.yamlEditor.setSize(null, '100%');
        }
        
        // Initialize display mode switcher
        this.initializeDisplayModes();
    }
    
    initializeDisplayModes() {
        const modeButtons = document.querySelectorAll('.mode-button');
        const contentWrapper = document.querySelector('.tafl-content-wrapper');
        const toggleBtn = document.getElementById('toggle-properties-btn');
        const editorContainer = document.querySelector('.columns.is-gapless');
        
        modeButtons.forEach(button => {
            button.addEventListener('click', (e) => {
                e.preventDefault();
                e.stopPropagation();
                
                // Toggle properties panel - just toggle a class on the parent container
                if (button.classList.contains('toggle-properties')) {
                    editorContainer.classList.toggle('properties-collapsed');
                    
                    // Refresh CodeMirror if needed
                    const currentMode = document.querySelector('.mode-button.is-active[data-mode]');
                    if (currentMode && (currentMode.dataset.mode === 'yaml' || currentMode.dataset.mode === 'both') && this.yamlEditor) {
                        setTimeout(() => this.yamlEditor.refresh(), 100);
                    }
                    return;
                }
                
                // Normal mode buttons
                if (button.dataset.mode) {
                    modeButtons.forEach(btn => {
                        if (btn.dataset.mode) btn.classList.remove('is-active');
                    });
                    button.classList.add('is-active');
                    
                    const mode = button.dataset.mode;
                    contentWrapper.className = `tafl-content-wrapper mode-${mode}`;
                    
                    if ((mode === 'yaml' || mode === 'both') && this.yamlEditor) {
                        setTimeout(() => {
                            this.yamlEditor.refresh();
                            this.refreshYAML();
                        }, 100);
                    }
                }
            });
        });
    }
    
    getDefaultParams(verb) {
        // TAFL v1.1 compliant default parameters with enhanced features
        const defaults = {
            query: {
                target: 'table_name',
                where: '',
                order: '',
                limit: '',
                as: 'result'  // Changed from store_as to as for v1.1
            },
            check: {
                condition: 'true',
                where: '',
                as: 'check_result'  // Changed from store_as to as for v1.1
            },
            create: {
                target: 'resource',
                with: {},  // Changed from params to with for v1.1
                as: 'created_item'  // Changed from store_as to as for v1.1
            },
            update: {
                target: 'resource',
                where: '',
                set: {},
                as: ''  // Added as parameter for v1.1
            },
            if: {
                condition: '${condition}',
                then: [],
                else: []
            },
            for: {
                each: 'item',
                in: '${items}',
                as: 'current',  // Added as parameter for v1.1
                filter: '',  // New in v1.1: filter condition
                do: []
            },
            switch: {
                expression: '${value}',
                cases: []
                // TAFL v1.1 標準格式：expression 作為 switch 的屬性
            },
            set: {
                // Enhanced set with multi-variable support (v1.1)
                variable: 'var_name',
                value: '',
                expression: '',  // New in v1.1: expression evaluation
                multi: {}  // New in v1.1: multiple variable assignments
            },
            stop: {
                reason: 'Flow completed',
                code: 0  // Added code parameter for v1.1
            },
            notify: {
                type: 'info',  // Changed from channel to type for v1.1
                message: 'Notification message',
                recipients: [],  // New in v1.1
                level: 'normal'  // New in v1.1
            }
        };
        
        return { [verb]: defaults[verb] || {} };
    }
    
    getCardTitle(cardData) {
        // TAFL v1.1: Extract verb from object keys (verb is the key, not a property)
        const verb = this.getCardVerb(cardData);
        const params = cardData[verb];
        
        switch (verb) {
            case 'query':
                return `Query ${params.target || 'data'}`;
            case 'check':
                return `Check ${params.condition || 'condition'}`;
            case 'create':
                return `Create ${params.target || 'resource'}`;
            case 'update':
                return `Update ${params.target || 'resource'}`;
            case 'if':
                return `If ${params.condition || 'condition'}`;
            case 'for':
                return `For each ${params.each || 'item'}`;
            case 'switch':
                return `Switch ${params.expression || 'value'}`;
            case 'set':
                return `Set ${typeof params === 'string' ? params.split('=')[0].trim() : 'variable'}`;
            case 'stop':
                return `Stop: ${typeof params === 'string' ? params : 'reason'}`;
            case 'notify':
                return `Notify: ${params.message || 'message'}`;
            default:
                return verb.charAt(0).toUpperCase() + verb.slice(1);
        }
    }
    
    renderCardParams(cardData) {
        // TAFL v1.1: Extract verb from object keys (verb is the key, not a property)
        const verb = this.getCardVerb(cardData);
        const params = cardData[verb];
        let html = '';
        
        if (typeof params === 'string') {
            // Simple string parameter (like set, stop)
            html += `
                <div class="card-param">
                    <div class="param-value">${this.formatParamValue(params)}</div>
                </div>
            `;
        } else if (typeof params === 'object') {
            for (const [key, value] of Object.entries(params)) {
                if (key === 'then' || key === 'else' || key === 'do' || key === 'cases' || key === 'default') {
                    continue; // These are rendered as nested cards
                }
                
                html += `
                    <div class="card-param">
                        <label class="param-label">${key}:</label>
                        <div class="param-value editable" data-param="${key}">
                            ${this.formatParamValue(value)}
                        </div>
                    </div>
                `;
            }
        }
        
        return html;
    }
    
    formatParamValue(value) {
        if (typeof value === 'object') {
            return JSON.stringify(value, null, 2);
        }
        return String(value);
    }
    
    selectCard(cardElement) {
        // Remove previous selection
        document.querySelectorAll('.tafl-card.selected').forEach(card => {
            card.classList.remove('selected');
        });
        
        // Select new card
        cardElement.classList.add('selected');
        this.selectedCard = cardElement;
        
        // Update properties panel
        this.updatePropertiesPanel();
    }
    
    deselectCard() {
        document.querySelectorAll('.tafl-card.selected').forEach(card => {
            card.classList.remove('selected');
        });
        this.selectedCard = null;
        this.updatePropertiesPanel();
    }
    
    updatePropertiesPanel() {
        const propertiesPanel = document.getElementById('properties-panel');
        
        if (!this.selectedCard) {
            propertiesPanel.innerHTML = `
                <div class="no-selection">
                    <div class="has-text-centered">
                        <span class="icon is-large has-text-grey-light">
                            <i class="fas fa-mouse-pointer fa-3x"></i>
                        </span>
                        <p class="has-text-grey">Select a card to edit properties</p>
                    </div>
                </div>
            `;
            return;
        }
        
        const cardId = this.selectedCard.dataset.cardId;
        const cardData = this.findCardById(cardId);
        
        if (!cardData) {
            console.error('Card data not found for ID:', cardId);
            propertiesPanel.innerHTML = `
                <div class="no-selection">
                    <div class="has-text-centered">
                        <span class="icon is-large has-text-danger">
                            <i class="fas fa-exclamation-triangle fa-3x"></i>
                        </span>
                        <p class="has-text-danger">Card data not found</p>
                    </div>
                </div>
            `;
            return;
        }
        
        const verb = this.getCardVerb(cardData);
        const verbDef = this.verbDefinitions[verb];
        
        if (!verbDef) {
            console.error('Verb definition not found for:', verb);
            return;
        }
        
        propertiesPanel.innerHTML = `
            <div class="property-editor">
                <h4 class="title is-5">Properties</h4>
                
                <div class="field">
                    <label class="label">Type: ${verbDef.name}</label>
                </div>
                
                <div class="property-fields">
                    ${this.renderPropertyEditor(cardData)}
                </div>
                
                <hr>
                
                <div class="field">
                    <label class="label">Card ID</label>
                    <div class="control">
                        <input class="input is-small" type="text" value="${cardData.id || ''}" readonly>
                    </div>
                </div>
                
                <div class="field">
                    <label class="label">Comment</label>
                    <div class="control">
                        <textarea class="textarea is-small property-input" 
                                  data-card-id="${cardId}" 
                                  data-param="comment" 
                                  rows="2"
                                  placeholder="Add a comment...">${cardData.comment || ''}</textarea>
                    </div>
                    <p class="help">Optional comment for documentation</p>
                </div>
                
                <div class="field is-grouped">
                    <p class="control">
                        <button class="button is-small is-primary" id="apply-properties">
                            <span class="icon">
                                <i class="fas fa-check"></i>
                            </span>
                            <span>Apply All</span>
                        </button>
                    </p>
                    <p class="control">
                        <button class="button is-small is-danger" id="delete-card">
                            <span class="icon">
                                <i class="fas fa-trash"></i>
                            </span>
                            <span>Delete Card</span>
                        </button>
                    </p>
                </div>
            </div>
        `;
        
        // Add event listeners for property editing
        this.setupPropertyEditing(cardData);
    }
    
    renderPropertyEditor(cardData) {
        const verb = this.getCardVerb(cardData);
        const params = cardData[verb];
        const cardId = cardData.id;
        let html = '';
        
        // Special handling for switch card
        if (verb === 'switch') {
            return this.renderSwitchCasesEditor(cardData);
        }
        
        if (!params) {
            return '<p class="has-text-grey">No parameters</p>';
        }
        
        if (typeof params === 'string') {
            // Simple string parameter (set, stop)
            html += `
                <div class="field">
                    <label class="label">Value</label>
                    <div class="control">
                        <textarea class="textarea is-small property-input" 
                                  data-card-id="${cardId}" 
                                  data-param="_root" 
                                  rows="2">${this.escapeHtml(params)}</textarea>
                    </div>
                </div>
            `;
        } else if (typeof params === 'object') {
            // Render common parameters first
            const commonParams = ['target', 'condition', 'each', 'in', 'value', 'filter', 'limit', 'id'];
            const renderedParams = new Set();
            
            // Render common params in order
            for (const key of commonParams) {
                if (key in params) {
                    html += this.renderPropertyField(cardId, key, params[key]);
                    renderedParams.add(key);
                }
            }
            
            // Render remaining params
            for (const [key, value] of Object.entries(params)) {
                if (!renderedParams.has(key) && 
                    key !== 'then' && key !== 'else' && 
                    key !== 'do' && key !== 'cases' && key !== 'default') {
                    html += this.renderPropertyField(cardId, key, value);
                }
            }
        }
        
        return html || '<p class="has-text-grey">No editable parameters</p>';
    }
    
    renderSwitchCasesEditor(cardData) {
        const switchParams = cardData.switch;
        const cardId = cardData.id;
        const cases = switchParams.cases || [];
        const hasDefaultCase = cases.some(c => c.when === "default");
        
        let html = `
            <!-- Expression field -->
            <div class="field">
                <label class="label">Expression</label>
                <div class="control">
                    <input class="input is-small property-input" 
                           type="text" 
                           data-card-id="${cardId}" 
                           data-param="expression"
                           value="${this.escapeHtml(switchParams.expression || '')}"
                           placeholder="e.g., \${task_priority}">
                </div>
                <p class="help">The variable or expression to evaluate</p>
            </div>
            
            <!-- Cases management -->
            <div class="field">
                <label class="label">Cases</label>
                
                <!-- Add Case button at the top -->
                <div class="mb-2">
                    <button class="button is-small is-info" id="add-case-btn" data-card-id="${cardId}">
                        <span class="icon"><i class="fas fa-plus"></i></span>
                        <span>Add Case</span>
                    </button>
                    ${!hasDefaultCase ? `
                        <button class="button is-small is-warning ml-2" id="add-default-btn" data-card-id="${cardId}">
                            <span class="icon"><i class="fas fa-plus"></i></span>
                            <span>Add Default</span>
                        </button>
                    ` : ''}
                </div>
                
                <!-- Cases list -->
                <div class="cases-list" id="switch-cases-list">
        `;
        
        if (cases.length === 0) {
            html += '<p class="has-text-grey is-italic">No cases defined. Click "Add Case" to create one.</p>';
        } else {
            // Render each case's when condition editor
            cases.forEach((caseItem, index) => {
                const isDefault = caseItem.when === "default";
                const isFirst = index === 0;
                const isLast = index === cases.length - 1;
                const isBeforeDefault = !isLast && cases[index + 1]?.when === "default";
                
                html += `
                    <div class="case-item box mb-2" data-case-index="${index}" style="padding: 0.5rem;">
                        <div class="field has-addons mb-0">
                            <!-- Move controls -->
                            <p class="control">
                                <button class="button is-small ${isFirst || isDefault ? 'is-static' : ''}" 
                                        data-card-id="${cardId}"
                                        data-case-index="${index}"
                                        ${isFirst || isDefault ? 'disabled' : ''}
                                        onclick="window.taflEditor.moveSwitchCase('${cardId}', ${index}, 'up')"
                                        title="Move up">
                                    <span class="icon is-small"><i class="fas fa-arrow-up"></i></span>
                                </button>
                            </p>
                            <p class="control">
                                <button class="button is-small ${isLast || isBeforeDefault ? 'is-static' : ''}" 
                                        data-card-id="${cardId}"
                                        data-case-index="${index}"
                                        ${isLast || isBeforeDefault ? 'disabled' : ''}
                                        onclick="window.taflEditor.moveSwitchCase('${cardId}', ${index}, 'down')"
                                        title="Move down">
                                    <span class="icon is-small"><i class="fas fa-arrow-down"></i></span>
                                </button>
                            </p>
                            
                            <!-- Case label -->
                            <p class="control">
                                <span class="button is-static is-small">
                                    ${isDefault ? 
                                        '<span class="has-text-warning-dark"><strong>Default</strong></span>' : 
                                        'When'}
                                </span>
                            </p>
                            
                            <!-- Case input -->
                            <p class="control is-expanded">
                                ${isDefault ? 
                                    '<input class="input is-small" value="(always matches)" readonly disabled style="background: #fffbeb;">' :
                                    `<input class="input is-small case-when-input" 
                                           data-card-id="${cardId}"
                                           data-case-index="${index}"
                                           value="${this.escapeHtml(caseItem.when || '')}"
                                           placeholder='e.g., "> 8" or "5..8" or "urgent"'>`
                                }
                            </p>
                            
                            <!-- Remove button -->
                            <p class="control">
                                <button class="button is-small is-danger is-light remove-case-btn" 
                                        data-card-id="${cardId}"
                                        data-case-index="${index}"
                                        title="Remove this case">
                                    <span class="icon"><i class="fas fa-trash"></i></span>
                                </button>
                            </p>
                        </div>
                    </div>
                `;
            });
        }
        
        html += `
                </div>
            </div>
        `;
        
        return html;
    }
    
    renderPropertyField(cardId, key, value) {
        let inputHTML = '';
        
        if (typeof value === 'string') {
            if (value.length > 50 || value.includes('\n')) {
                inputHTML = `
                    <textarea class="textarea is-small property-input" 
                              data-card-id="${cardId}" 
                              data-param="${key}" 
                              rows="3">${this.escapeHtml(value)}</textarea>
                `;
            } else {
                inputHTML = `
                    <input class="input is-small property-input" 
                           type="text" 
                           data-card-id="${cardId}" 
                           data-param="${key}" 
                           value="${this.escapeHtml(value)}">
                `;
            }
        } else if (typeof value === 'number') {
            inputHTML = `
                <input class="input is-small property-input" 
                       type="number" 
                       data-card-id="${cardId}" 
                       data-param="${key}" 
                       value="${value}">
            `;
        } else if (typeof value === 'boolean') {
            inputHTML = `
                <label class="checkbox">
                    <input type="checkbox" 
                           class="property-input" 
                           data-card-id="${cardId}" 
                           data-param="${key}" 
                           ${value ? 'checked' : ''}>
                    <span class="ml-2">${value ? 'Yes' : 'No'}</span>
                </label>
            `;
        } else if (value === null || value === undefined) {
            inputHTML = `
                <input class="input is-small property-input" 
                       type="text" 
                       data-card-id="${cardId}" 
                       data-param="${key}" 
                       value="" 
                       placeholder="Not set">
            `;
        } else if (Array.isArray(value) || typeof value === 'object') {
            // For arrays and objects, show as JSON
            inputHTML = `
                <textarea class="textarea is-small property-input" 
                          data-card-id="${cardId}" 
                          data-param="${key}" 
                          rows="4">${JSON.stringify(value, null, 2)}</textarea>
            `;
        }
        
        return `
            <div class="field">
                <label class="label">${this.formatParamName(key)}</label>
                <div class="control">
                    ${inputHTML}
                </div>
                ${this.getParamHelp(key)}
            </div>
        `;
    }
    
    formatParamName(name) {
        // Convert snake_case to Title Case
        return name.replace(/_/g, ' ')
                  .replace(/\b\w/g, char => char.toUpperCase());
    }
    
    getParamHelp(param) {
        const helpTexts = {
            'target': 'The target resource or entity',
            'condition': 'The condition to evaluate',
            'filter': 'Filter criteria for the query',
            'limit': 'Maximum number of results',
            'each': 'The item variable name in the loop',
            'in': 'The collection to iterate over',
            'value': 'The value to set or use',
            'id': 'The identifier of the resource',
            'data': 'The data payload'
        };
        
        if (helpTexts[param]) {
            return `<p class="help">${helpTexts[param]}</p>`;
        }
        return '';
    }
    
    escapeHtml(text) {
        if (!text) return '';
        const div = document.createElement('div');
        div.textContent = text;
        return div.innerHTML;
    }
    
    setupPropertyEditing(cardData) {
        const verb = this.getCardVerb(cardData);
        
        // Apply properties button
        const applyBtn = document.getElementById('apply-properties');
        applyBtn?.addEventListener('click', () => {
            this.applyAllPropertyChanges();
        });
        
        // Delete card button
        const deleteBtn = document.getElementById('delete-card');
        deleteBtn?.addEventListener('click', () => {
            this.deleteCard(cardData.id);
        });
        
        // Special handling for switch card
        if (verb === 'switch') {
            // Add case button
            document.getElementById('add-case-btn')?.addEventListener('click', () => {
                this.addSwitchCase(cardData.id);
            });
            
            // Add default button
            document.getElementById('add-default-btn')?.addEventListener('click', () => {
                this.toggleDefault(cardData.id, true);
            });
            
            // Remove default button
            document.getElementById('remove-default-btn')?.addEventListener('click', () => {
                this.toggleDefault(cardData.id, false);
            });
            
            // Remove case buttons
            document.querySelectorAll('.remove-case-btn').forEach(btn => {
                btn.addEventListener('click', (e) => {
                    const caseIndex = parseInt(e.currentTarget.dataset.caseIndex);
                    this.removeSwitchCase(cardData.id, caseIndex);
                });
            });
            
            // When condition editing
            document.querySelectorAll('.case-when-input').forEach(input => {
                input.addEventListener('input', (e) => {
                    const caseIndex = parseInt(e.target.dataset.caseIndex);
                    this.updateCaseWhen(cardData.id, caseIndex, e.target.value);
                });
            });
            
            // Move case buttons
            document.querySelectorAll('.move-case-up:not([disabled])').forEach(btn => {
                btn.addEventListener('click', (e) => {
                    const caseIndex = parseInt(e.currentTarget.dataset.caseIndex);
                    this.moveSwitchCase(cardData.id, caseIndex, 'up');
                });
            });
            
            document.querySelectorAll('.move-case-down:not([disabled])').forEach(btn => {
                btn.addEventListener('click', (e) => {
                    const caseIndex = parseInt(e.currentTarget.dataset.caseIndex);
                    this.moveSwitchCase(cardData.id, caseIndex, 'down');
                });
            });
        }
        
        // Real-time property updates
        document.querySelectorAll('.property-input').forEach(input => {
            // Remove any existing listeners
            const newInput = input.cloneNode(true);
            input.parentNode.replaceChild(newInput, input);
            
            // Add debounced change listener
            let timeout;
            const updateValue = () => {
                clearTimeout(timeout);
                timeout = setTimeout(() => {
                    this.updatePropertyValue(newInput);
                }, 300);
            };
            
            if (newInput.type === 'checkbox') {
                newInput.addEventListener('change', updateValue);
            } else {
                newInput.addEventListener('input', updateValue);
            }
        });
    }
    
    updatePropertyValue(input) {
        const cardId = input.dataset.cardId;
        const param = input.dataset.param;
        const cardData = this.findCardById(cardId);
        
        if (!cardData) {
            console.error('Card not found:', cardId);
            return;
        }
        
        let value = input.type === 'checkbox' ? input.checked : input.value;
        
        // Try to parse as JSON if it looks like JSON
        if (typeof value === 'string' && (value.startsWith('[') || value.startsWith('{'))) {
            try {
                value = JSON.parse(value);
            } catch (e) {
                // Keep as string if not valid JSON
            }
        } else if (typeof value === 'string' && value.match(/^\d+$/)) {
            // Keep numeric strings as strings unless they're IDs
            if (param !== 'id' && param !== 'target') {
                value = parseInt(value);
            }
        } else if (typeof value === 'string' && value.match(/^\d+\.\d+$/)) {
            // Convert decimal strings to floats
            value = parseFloat(value);
        }
        
        const verb = this.getCardVerb(cardData);
        
        if (param === '_root') {
            // For simple string parameters
            cardData[verb] = value;
        } else if (param === 'comment') {
            // Update comment field
            if (value) {
                cardData.comment = value;
            } else {
                delete cardData.comment;
            }
        } else {
            // For object parameters
            if (typeof cardData[verb] === 'object') {
                if (value === '' || value === null) {
                    delete cardData[verb][param];
                } else {
                    cardData[verb][param] = value;
                }
            }
        }
        
        // Update card display immediately
        this.refreshCard(cardId);
        this.markDirty();
        
        // Show visual feedback
        input.classList.add('is-success');
        setTimeout(() => {
            input.classList.remove('is-success');
        }, 500);
    }
    
    applyAllPropertyChanges() {
        // Apply all pending changes
        document.querySelectorAll('.property-input').forEach(input => {
            this.updatePropertyValue(input);
        });
        
        this.showNotification('Properties updated', 'is-success');
        this.updateUI();
    }
    
    applyPropertyChanges(cardData) {
        // Legacy method for compatibility
        this.applyAllPropertyChanges();
    }
    
    refreshCard(cardId) {
        const cardElement = document.querySelector(`[data-card-id="${cardId}"]`);
        const cardData = this.findCardById(cardId);
        
        if (!cardElement || !cardData) return;
        
        // Update title
        const titleElement = cardElement.querySelector('.card-title');
        if (titleElement) {
            titleElement.textContent = this.getCardTitle(cardData);
        }
        
        // Update parameters display
        const bodyElement = cardElement.querySelector('.tafl-card-body');
        if (bodyElement) {
            bodyElement.innerHTML = this.renderCardParams(cardData);
        }
    }
    
    findCardById(cardId) {
        return taflFlowStore.findCardById(cardId);
    }
    
    findCardInArray(cards, cardId) {
        for (const card of cards) {
            if (card.id === cardId) {
                return card;
            }
            
            // Search in nested structures
            const verb = this.getCardVerb(card);
            const params = card[verb];
            if (typeof params === 'object') {
                const nested = ['then', 'else', 'do', 'cases', 'default'];
                for (const key of nested) {
                    if (params[key] && Array.isArray(params[key])) {
                        const found = this.findCardInArray(params[key], cardId);
                        if (found) return found;
                    }
                }
            }
        }
        return null;
    }
    
    duplicateCard(cardId) {
        const cardData = this.findCardById(cardId);
        if (!cardData) return;
        
        const duplicatedData = JSON.parse(JSON.stringify(cardData));
        duplicatedData.id = this.generateId();
        
        // Find insertion point (after the original card)
        const index = taflFlowStore.getFlow().flow.findIndex(card => card.id === cardId);
        if (index >= 0) {
            taflFlowStore.getFlow().flow.splice(index + 1, 0, duplicatedData);
        } else {
            taflFlowStore.getFlow().flow.push(duplicatedData);
        }
        
        // Manual refresh after duplication
        this.updateCanvas({
            fullRefresh: true,
            source: 'card:duplicate'
        });
        
        // Animate the duplicated card
        const duplicatedElement = document.querySelector(`[data-card-id="${duplicatedData.id}"]`);
        if (duplicatedElement) {
            duplicatedElement.classList.add('just-added');
            setTimeout(() => duplicatedElement.classList.remove('just-added'), 500);
        }
        
        this.markDirty();
    }
    
    getCardById(cardId) {
        // Helper method to find a card by ID in the flow
        for (const card of taflFlowStore.getFlow().flow) {
            if (card.id === cardId) {
                return card;
            }
            // Check nested structures (if, for, switch branches)
            if (card.then_branch) {
                const found = this.findCardInBranch(card.then_branch, cardId);
                if (found) return found;
            }
            if (card.else_branch) {
                const found = this.findCardInBranch(card.else_branch, cardId);
                if (found) return found;
            }
            if (card.body) {
                const found = this.findCardInBranch(card.body, cardId);
                if (found) return found;
            }
            if (card.cases) {
                for (const caseItem of card.cases) {
                    const found = this.findCardInBranch(caseItem.body, cardId);
                    if (found) return found;
                }
            }
        }
        return null;
    }
    
    findCardInBranch(branch, cardId) {
        if (!branch) return null;
        for (const card of branch) {
            if (card.id === cardId) {
                return card;
            }
        }
        return null;
    }
    
    async deleteCard(cardId) {
        if (!await taflModals.confirmDelete('card')) {
            return;
        }
        
        // Remove from flow data
        this.removeCardFromFlow(cardId);
        
        // Manual refresh after flow reset
        this.updateCanvas({
            fullRefresh: true,
            preserveScroll: false,
            source: 'flow:new'
        });
        
        // Show drop zone if no cards left
        const flow = taflFlowStore.getFlow();
        if (flow.flow.length === 0) {
            document.getElementById('canvas-drop-zone').style.display = 'flex';
        }
        
        // Deselect if this was the selected card
        if (this.selectedCard && this.selectedCard.dataset.cardId === cardId) {
            this.deselectCard();
        }
        
        this.markDirty();
        this.updateUI();
        this.showNotification('Card deleted', 'is-success');
    }
    
    removeCardFromFlow(cardId) {
        taflFlowStore.deleteCard(cardId);
    }
    
    removeFromNestedStructures(cards, cardId) {
        for (const card of cards) {
            const verb = this.getCardVerb(card);
            const params = card[verb];
            
            if (typeof params === 'object') {
                const nestedKeys = ['then', 'else', 'do', 'cases', 'default'];
                for (const key of nestedKeys) {
                    if (params[key] && Array.isArray(params[key])) {
                        params[key] = params[key].filter(nestedCard => nestedCard.id !== cardId);
                        this.removeFromNestedStructures(params[key], cardId);
                    }
                }
            }
        }
    }
    
    removeNestedCard(cardId) {
        // Remove from main flow and nested structures
        this.removeCardFromFlow(cardId);
        return true;
    }
    
    // ============================================
    // Switch Cases Management Functions
    // ============================================
    
    addSwitchCase(cardId) {
        const cardData = this.findCardById(cardId);
        if (!cardData || !cardData.switch) return;
        
        if (!cardData.switch.cases) {
            cardData.switch.cases = [];
        }
        
        // Find if there's a default case at the end
        const defaultIndex = cardData.switch.cases.findIndex(c => c.when === "default");
        
        // Create new case
        const newCase = {
            when: "",
            do: []
        };
        
        // If there's a default case, insert before it; otherwise append
        if (defaultIndex !== -1) {
            cardData.switch.cases.splice(defaultIndex, 0, newCase);
        } else {
            cardData.switch.cases.push(newCase);
        }
        
        // Refresh the properties panel to show the new case
        this.updatePropertiesPanel();
        
        // Refresh the card to show the new case drop zone
        this.refreshCard(cardId);
        
        console.log('Added new case to switch:', cardData);
    }
    
    removeSwitchCase(cardId, caseIndex) {
        const cardData = this.findCardById(cardId);
        if (!cardData || !cardData.switch || !cardData.switch.cases) return;
        
        // Remove the case at the specified index
        cardData.switch.cases.splice(caseIndex, 1);
        
        // Refresh the properties panel
        this.updatePropertiesPanel();
        
        // Refresh the card to update the display
        this.refreshCard(cardId);
        
        console.log(`Removed case ${caseIndex} from switch:`, cardData);
    }
    
    updateCaseWhen(cardId, caseIndex, newWhen) {
        const cardData = this.findCardById(cardId);
        if (!cardData || !cardData.switch || !cardData.switch.cases) return;
        
        if (cardData.switch.cases[caseIndex]) {
            cardData.switch.cases[caseIndex].when = newWhen;
            
            // Only update the card display, not the properties panel
            this.refreshCard(cardId);
            
            console.log(`Updated case ${caseIndex} when to "${newWhen}":`, cardData);
        }
    }
    
    toggleDefault(cardId, add = true) {
        const cardData = this.findCardById(cardId);
        if (!cardData || !cardData.switch) return;
        
        if (add) {
            // Add default as a special case at the end
            if (!cardData.switch.cases) {
                cardData.switch.cases = [];
            }
            // Check if default already exists
            const hasDefault = cardData.switch.cases.some(c => c.when === "default");
            if (!hasDefault) {
                cardData.switch.cases.push({
                    when: "default",
                    do: []
                });
            }
        } else {
            // Remove default case
            if (cardData.switch.cases) {
                cardData.switch.cases = cardData.switch.cases.filter(c => c.when !== "default");
            }
        }
        
        // Refresh the properties panel
        this.updatePropertiesPanel();
        
        // Refresh the card to show/hide the default drop zone
        this.refreshCard(cardId);
        
        console.log(`${add ? 'Added' : 'Removed'} default case:`, cardData);
    }
    
    moveSwitchCase(cardId, caseIndex, direction) {
        const cardData = this.findCardById(cardId);
        if (!cardData || !cardData.switch || !cardData.switch.cases) return;
        
        const cases = cardData.switch.cases;
        const targetIndex = direction === 'up' ? caseIndex - 1 : caseIndex + 1;
        
        // Boundary check
        if (targetIndex < 0 || targetIndex >= cases.length) return;
        
        // Check if trying to move default case or move something after default
        const isMovingDefault = cases[caseIndex].when === "default";
        const isTargetDefault = cases[targetIndex].when === "default";
        
        // Don't allow moving default up or moving other cases down past default
        if (isMovingDefault && direction === 'up') return;
        if (isTargetDefault && direction === 'down') return;
        
        // Swap positions (including the entire case structure with do array)
        [cases[caseIndex], cases[targetIndex]] = [cases[targetIndex], cases[caseIndex]];
        
        // Refresh the properties panel and card display
        this.updatePropertiesPanel();
        this.refreshCard(cardId);
        
        console.log(`Moved case ${caseIndex} ${direction}:`, cardData);
    }
    
    // ===== 統一 Canvas 更新系統 =====
    // 智能更新控制器 - 所有更新都通過此接口
    updateCanvas(options = {}) {
        const defaults = {
            fullRefresh: false,      // 是否完整重建
            preserveScroll: true,    // 保留捲動位置
            skipDuringDrag: true,    // 拖曳時跳過
            source: 'unknown'        // 呼叫來源（用於除錯）
        };
        
        const config = { ...defaults, ...options };
        
        // 拖曳中檢查
        if (config.skipDuringDrag && this.isDragging()) {
            console.log(`🚫 Skipping canvas update during drag (source: ${config.source})`);
            return;
        }
        
        console.log(`🔄 Canvas update (source: ${config.source}, full: ${config.fullRefresh})`);
        
        // 保存捲動位置
        const scrollState = config.preserveScroll ? this.saveScrollState() : null;
        
        // 執行更新
        if (config.fullRefresh) {
            this.performFullRefresh();
        } else {
            this.performPartialUpdate();
        }
        
        // 恢復捲動位置
        if (scrollState) {
            this.restoreScrollState(scrollState);
        }
    }
    
    // 檢查是否正在拖曳
    isDragging() {
        return this.draggedElement || 
               document.body.classList.contains('dragging');
    }
    
    // 檢查移動操作是否改變了結構（嵌套 <-> 外層）
    checkIfStructureChanged(moveData) {
        if (!moveData || !moveData.cardId) return true; // 安全起見，預設完整更新
        
        // 使用記錄的原始位置
        const originalPos = this.cardOriginalPositions.get(moveData.cardId);
        const wasNested = originalPos ? originalPos.isNested : false;
        
        // 檢查新位置是否嵌套
        const isNowNested = moveData.position && moveData.position.nested;
        
        // 如果從嵌套移到外層或從外層移到嵌套，需要完整更新
        if (wasNested && !isNowNested) {
            console.log('📤 Card moved from nested to top level - full refresh needed');
            return true;
        }
        if (!wasNested && isNowNested) {
            console.log('📦 Card moved from top level to nested - full refresh needed');
            return true;
        }
        
        // 檢查是否在不同的嵌套分支間移動（如 then -> else）
        if (wasNested && isNowNested) {
            // 都在嵌套中，但可能在不同分支
            // 為了安全起見，嵌套間的移動都需要完整更新
            console.log('🔄 Card moved between nested branches - full refresh needed');
            this.cardOriginalPositions.delete(moveData.cardId);
            return true;
        }
        
        // 清理記錄（拖曳結束）
        this.cardOriginalPositions.delete(moveData.cardId);
        
        // 只有頂層之間的移動才是部分更新
        console.log('↔️ Card moved within top level - partial update sufficient');
        return false;
    }
    
    // 保存捲動狀態
    saveScrollState() {
        const container = document.querySelector('.tafl-canvas-container');
        if (!container) return null;
        return {
            scrollTop: container.scrollTop,
            scrollLeft: container.scrollLeft
        };
    }
    
    // 恢復捲動狀態
    restoreScrollState(state) {
        if (!state) return;
        const container = document.querySelector('.tafl-canvas-container');
        if (container) {
            requestAnimationFrame(() => {
                container.scrollTop = state.scrollTop;
                container.scrollLeft = state.scrollLeft;
            });
        }
    }
    
    // 部分更新（僅更新位置，適用於拖曳）
    performPartialUpdate() {
        const dropZone = document.getElementById('canvas-drop-zone');
        if (!dropZone) return;
        
        console.log('⚡ Performing partial update (order only)');
        
        // 只更新頂層卡片的順序，不處理嵌套結構變化
        const flow = taflFlowStore.getFlow();
        const cards = flow.flow || [];
        cards.forEach((card, index) => {
            const element = dropZone.querySelector(`:scope > [data-card-id="${card.id}"]`);
            if (element) {
                // 使用 CSS order 屬性重新排序，避免 DOM 重建
                element.style.order = index;
            }
        });
    }
    
    // 完整重建（保留原 refreshCanvas 邏輯）
    performFullRefresh() {
        this.refreshCanvasCore();
    }
    
    // 舊的 refreshCanvas 保留為向後相容
    refreshCanvas() {
        // 直接調用新的統一更新系統
        this.updateCanvas({ 
            fullRefresh: true, 
            source: 'legacy-refresh' 
        });
    }
    
    // 原始的 refreshCanvas 邏輯（被 performFullRefresh 使用）
    refreshCanvasCore() {
        const canvas = document.getElementById('flow-canvas');
        const flow = taflFlowStore.getFlow();
        
        // 保存選中狀態
        const selectedId = this.selectedCard?.dataset?.cardId;
        
        // 清空畫布
        canvas.innerHTML = '';
        
        if (flow.flow.length === 0) {
            // 顯示空白提示 - 與 HTML 模板保持一致
            canvas.innerHTML = `
                <div class="canvas-drop-zone" id="canvas-drop-zone">
                    <div class="drop-zone-content">
                        <span class="icon is-large has-text-grey-light">
                            <i class="fas fa-grip-vertical fa-3x"></i>
                        </span>
                        <p class="has-text-grey">Drag TAFL verbs here to build your flow</p>
                        <p class="has-text-grey-light is-size-7">Or click on a verb from the toolbox</p>
                    </div>
                </div>
            `;
            // 重新設置空畫布的放置區
            this.createInitialDropZone();
        } else {
            // 創建 drop zone 容器
            const dropZone = document.createElement('div');
            dropZone.id = 'canvas-drop-zone';
            dropZone.className = 'canvas-drop-zone has-cards'; // 加上 has-cards class
            canvas.appendChild(dropZone);
            
            // 渲染所有卡片到 drop zone 內（不再綁定個別事件）
            flow.flow.forEach(cardData => {
                const cardElement = this.createCardElementSimple(cardData);
                dropZone.appendChild(cardElement);
            });
        }
        
        // 恢復選中狀態
        if (selectedId) {
            const card = document.querySelector(`[data-card-id="${selectedId}"]`);
            if (card) {
                this.selectCard(card);
            }
        }
        
        // Ensure all cards are draggable after refresh
        this.refreshCardDragHandlers();
    }
    
    // 已被全域事件委派取代 - cloneNode(true)會破壞事件冒泡！
    setupNestedCardInteractions() {
        return; // 直接返回，不執行任何操作
        // Set up click events for ALL nested cards (including deeply nested ones)
        // Use a more general selector that catches all nested cards at any depth
        document.querySelectorAll('.nested-cards .tafl-card').forEach(nestedCard => {
            // Skip if this is a direct child of flow-canvas (not actually nested)
            if (nestedCard.parentElement.id === 'flow-canvas') {
                return;
            }
            
            // Remove any existing click listeners to avoid duplicates
            const newCard = nestedCard.cloneNode(true);
            nestedCard.parentNode.replaceChild(newCard, nestedCard);
            
            // Add click listener that properly handles nested cards
            newCard.addEventListener('click', (e) => {
                // Prevent event from bubbling to parent cards
                e.stopPropagation();
                
                // Don't select if clicking on interactive elements
                if (e.target.closest('.card-controls') || 
                    e.target.closest('input') || 
                    e.target.closest('textarea') ||
                    e.target.closest('button')) {
                    return;
                }
                
                // Select this nested card
                this.selectCard(newCard);
            });
            
            // Set up control buttons for nested cards
            const cardId = newCard.dataset.cardId;
            const duplicateBtn = newCard.querySelector('.card-duplicate-btn:not(.bound)');
            const deleteBtn = newCard.querySelector('.card-delete-btn:not(.bound)');
            
            if (duplicateBtn) {
                // Mark as bound to avoid duplicate event listeners
                duplicateBtn.classList.add('bound');
                duplicateBtn.addEventListener('click', (e) => {
                    e.stopPropagation();
                    console.log('🔄 Duplicate button clicked for nested card:', cardId);
                    this.duplicateCard(cardId);
                });
            }
            
            if (deleteBtn) {
                // Mark as bound to avoid duplicate event listeners
                deleteBtn.classList.add('bound');
                deleteBtn.addEventListener('click', (e) => {
                    e.stopPropagation();
                    console.log('🗑️ Delete button clicked for nested card:', cardId);
                    this.deleteCard(cardId);
                });
            }
            
            // Setup drag functionality for nested cards
            // 註：拖放事件已經由 setupGlobalEventDelegation 統一處理
            // 這裡只需要確保卡片有 draggable 屬性
            const dragHandle = newCard.querySelector('.card-drag-handle');
            if (dragHandle && !dragHandle.classList.contains('bound')) {
                dragHandle.classList.add('bound');
                newCard.draggable = true;
                // 不再單獨綁定 dragstart/dragend 事件
                // 全局事件委派會處理所有拖放操作
            }
        });
        
        // All drop handling is done through unified drop handler
    }
    
    setupNestedDropTargets() {
        // Handle empty drop zones
        // Note: dragover and dragleave are now handled via event delegation in setupUnifiedDragDrop
        document.querySelectorAll('.nested-drop-zone.empty-zone').forEach(dropZone => {
            dropZone.addEventListener('drop', (e) => {
                e.preventDefault();
                e.stopPropagation();
                dropZone.classList.remove('drag-over');
                
                // Clear insertion lines when dropping
                this.hideDropIndicators();
                
                const dragData = this.getDragData(e) || this.dragData;
                if (!dragData) return;
                
                const nestedCards = dropZone.closest('.nested-cards');
                const parentCard = nestedCards?.closest('.tafl-card');
                const parentCardId = parentCard?.dataset.cardId;
                const branchType = dropZone.dataset.branch;
                
                if (parentCardId && branchType) {
                    if (dragData.type === 'verb') {
                        // Add new card to nested structure
                        const cardId = this.generateId();
                        // TAFL v1.1: verb as key, not property
                        const cardData = {
                            id: cardId,
                            [dragData.verb]: this.getDefaultParams(dragData.verb)[dragData.verb]
                        };
                        
                        this.addToNestedStructure(parentCardId, branchType, cardData);
                        this.updateCanvas({
                            fullRefresh: true,
                            source: 'nested:drag-add'
                        });
                        
                        // Select new card
                        const newElement = document.querySelector(`[data-card-id="${cardId}"]`);
                        if (newElement) {
                            newElement.classList.add('just-added');
                            setTimeout(() => newElement.classList.remove('just-added'), 500);
                            this.selectCard(newElement);
                        }
                        
                        this.markDirty();
                        this.showNotification(`Added ${dragData.verb} to ${branchType} branch`, 'is-success');
                    } else if (dragData.type === 'card') {
                        // Move existing card to nested structure
                        const cardData = this.findCardById(dragData.cardId);
                        if (cardData) {
                            this.removeCardFromFlow(dragData.cardId);
                            this.addToNestedStructure(parentCardId, branchType, cardData);
                            this.updateCanvas({
                                fullRefresh: true,
                                source: 'nested:drag-add-else'
                            });
                            
                            const movedElement = document.querySelector(`[data-card-id="${dragData.cardId}"]`);
                            if (movedElement) {
                                movedElement.classList.add('just-moved');
                                setTimeout(() => movedElement.classList.remove('just-moved'), 500);
                                this.selectCard(movedElement);
                            }
                            
                            this.markDirty();
                            this.showNotification(`Moved card to ${branchType} branch`, 'is-success');
                        }
                    }
                }
            });
        });
        
        // Set up horizontal drop indicators
        document.querySelectorAll('.drop-indicator.horizontal').forEach(indicator => {
            indicator.addEventListener('dragover', (e) => {
                e.preventDefault();
                e.stopPropagation();
                
                // Clear all other active indicators
                document.querySelectorAll('.drop-indicator.horizontal').forEach(ind => {
                    ind.classList.remove('active');
                });
                
                // Activate this indicator
                indicator.classList.add('active');
            });
            
            indicator.addEventListener('dragleave', (e) => {
                if (!indicator.contains(e.relatedTarget)) {
                    indicator.classList.remove('active');
                }
            });
            
            indicator.addEventListener('drop', (e) => {
                e.preventDefault();
                e.stopPropagation();
                indicator.classList.remove('active');
                
                // Clear insertion lines when dropping
                this.hideDropIndicators();
                
                const dragData = this.getDragData(e) || this.dragData;
                if (!dragData) return;
                
                const nestedCards = indicator.closest('.nested-cards');
                const parentCard = nestedCards?.closest('.tafl-card');
                const parentCardId = parentCard?.dataset.cardId;
                const branchType = indicator.dataset.branch;
                const position = parseInt(indicator.dataset.position);
                
                if (parentCardId && branchType && !isNaN(position)) {
                    if (dragData.type === 'verb') {
                        // Add new card to nested structure at specific position
                        const cardId = this.generateId();
                        const cardData = {
                            id: cardId,
                            [dragData.verb]: this.getDefaultParams(dragData.verb)[dragData.verb]
                        };
                        
                        this.addToNestedStructureAtPosition(parentCardId, branchType, cardData, position);
                        this.updateCanvas({
                            fullRefresh: true,
                            source: 'nested:drag-position'
                        });
                        
                        // Select new card
                        const newElement = document.querySelector(`[data-card-id="${cardId}"]`);
                        if (newElement) {
                            newElement.classList.add('just-added');
                            setTimeout(() => newElement.classList.remove('just-added'), 500);
                            this.selectCard(newElement);
                        }
                        
                        this.markDirty();
                        this.showNotification(`Added ${dragData.verb} at position ${position + 1}`, 'is-success');
                    } else if (dragData.type === 'card') {
                        // Move existing card to specific position
                        const cardData = this.findCardById(dragData.cardId);
                        if (cardData) {
                            this.removeCardFromFlow(dragData.cardId);
                            this.addToNestedStructureAtPosition(parentCardId, branchType, cardData, position);
                            this.updateCanvas({
                                fullRefresh: true,
                                source: 'nested:drag-position-else'
                            });
                            
                            const movedElement = document.querySelector(`[data-card-id="${dragData.cardId}"]`);
                            if (movedElement) {
                                movedElement.classList.add('just-moved');
                                setTimeout(() => movedElement.classList.remove('just-moved'), 500);
                                this.selectCard(movedElement);
                            }
                            
                            this.markDirty();
                            this.showNotification(`Moved card to position ${position + 1}`, 'is-success');
                        }
                    }
                }
            });
        });
    }
    
    switchVerbCategory(category) {
        // Update active tab
        document.querySelectorAll('.panel-tabs a').forEach(tab => {
            tab.classList.toggle('is-active', tab.dataset.tab === category);
        });
        
        // Show/hide verb categories
        document.querySelectorAll('.verb-category').forEach(cat => {
            cat.style.display = 'none';
        });
        
        const targetCategory = document.getElementById(`${category}-verbs`);
        if (targetCategory) {
            targetCategory.style.display = 'block';
        }
    }
    
    switchPropertiesTab(tab) {
        // Update active tab
        document.querySelectorAll('.tabs li').forEach(tabEl => {
            tabEl.classList.toggle('is-active', tabEl.dataset.tab === tab);
        });
        
        // Show/hide panels
        document.querySelectorAll('.properties-content').forEach(panel => {
            panel.style.display = 'none';
        });
        
        const targetPanel = document.getElementById(`${tab}-panel`);
        if (targetPanel) {
            targetPanel.style.display = 'block';
        }
        
        // If switching to variables tab, update the panel
        if (tab === 'variables' && window.taflPanels) {
            window.taflPanels.updateVariablesPanel();
        }
        
        // Refresh CodeMirror editors for tabs that use JSON editors
        if (['preload', 'rules', 'variables'].includes(tab)) {
            // Small delay to ensure panel is visible before refreshing
            setTimeout(() => {
                if (window.taflPanels && window.taflPanels.refreshCodeMirrors) {
                    window.taflPanels.refreshCodeMirrors();
                }
            }, 50);
        }
        
        // Refresh YAML if switching to YAML tab
        if (tab === 'yaml') {
            this.refreshYAML();
        }
    }
    
    // newFlow() moved to tafl-editor-modals.js
    
    // OpenFlowModal removed - using dropdown selector instead
    // The dropdown in the header now handles flow selection
    
    async openFlow(flowId) {
        try {
            const data = await taflAPI.getFlow(flowId);
            const normalizedFlow = data.flow;
            
            taflFlowStore.loadFlow(normalizedFlow);
            // flow:loaded 事件會觸發 refresh
            // UI updates handled by SimpleTAFLPanels via store events
            // this.refreshVariablesUI();
            // this.refreshPreloadUI();
            // this.refreshRulesUI();
            this.updateUI();
            taflFlowStore.setDirty(false);
            
            this.showNotification(`Opened flow: ${normalizedFlow.metadata.name}`, 'is-success');
        } catch (error) {
            this.showNotification(error.message, 'is-danger');
        }
    }
    
    /**
     * Recursively ensure all cards have IDs (including nested cards)
     */
    ensureAllCardsHaveIds(cards) {
        if (!Array.isArray(cards)) return cards;
        
        return cards.map(card => {
            // Ensure this card has an ID
            if (!card.id) {
                card = { ...card, id: this.generateId() };
            }
            
            // Get the verb to find nested structures
            const verb = this.getCardVerb(card);
            if (verb && card[verb]) {
                const params = card[verb];
                
                // Process if/then/else structures
                if (verb === 'if') {
                    if (params.then && Array.isArray(params.then)) {
                        card = {
                            ...card,
                            [verb]: {
                                ...params,
                                then: this.ensureAllCardsHaveIds(params.then)
                            }
                        };
                    }
                    if (params.else && Array.isArray(params.else)) {
                        card = {
                            ...card,
                            [verb]: {
                                ...card[verb],
                                else: this.ensureAllCardsHaveIds(params.else)
                            }
                        };
                    }
                }
                // Process for/do structures
                else if (verb === 'for' && params.do && Array.isArray(params.do)) {
                    card = {
                        ...card,
                        [verb]: {
                            ...params,
                            do: this.ensureAllCardsHaveIds(params.do)
                        }
                    };
                }
                // Process switch/cases structures
                else if (verb === 'switch') {
                    let updatedParams = { ...params };
                    
                    if (params.cases && Array.isArray(params.cases)) {
                        updatedParams.cases = params.cases.map(caseItem => {
                            if (caseItem.do && Array.isArray(caseItem.do)) {
                                return {
                                    ...caseItem,
                                    do: this.ensureAllCardsHaveIds(caseItem.do)
                                };
                            }
                            return caseItem;
                        });
                    }
                    
                    // Convert old format default to new format
                    if (params.default && typeof params.default === 'object') {
                        // Old format: default: { do: [...] }
                        // Convert to new format as a special case
                        if (!updatedParams.cases) {
                            updatedParams.cases = [];
                        }
                        // Check if default case already exists
                        const hasDefaultCase = updatedParams.cases.some(c => c.when === "default");
                        if (!hasDefaultCase) {
                            updatedParams.cases.push({
                                when: "default",
                                do: params.default.do ? this.ensureAllCardsHaveIds(params.default.do) : []
                            });
                        }
                        // Remove old format default
                        delete updatedParams.default;
                    }
                    
                    card = {
                        ...card,
                        [verb]: updatedParams
                    };
                }
            }
            
            return card;
        });
    }
    
    // Public method for loading flows from dropdown
    async loadFlow(flowDataOrId) {
        // Handle both flow data object and flow ID string
        if (typeof flowDataOrId === 'string') {
            // It's a flow ID, fetch the flow
            await this.openFlow(flowDataOrId);
        } else if (typeof flowDataOrId === 'object' && flowDataOrId !== null) {
            // It's already flow data
            let flowToLoad;
            if (flowDataOrId.flow) {
                // The response has a wrapper
                flowToLoad = flowDataOrId.flow;
            } else {
                // Direct flow data
                flowToLoad = flowDataOrId;
            }
            
            // Ensure flow has all required sections
            const normalizedFlow = {
                metadata: flowToLoad.metadata || { id: null, name: 'Untitled Flow', version: '1.1', description: '' },
                settings: flowToLoad.settings || { timeout: 3600, max_retries: 3, retry_on_failure: false },
                preload: flowToLoad.preload || {},
                rules: flowToLoad.rules || {},
                variables: flowToLoad.variables || {},
                flow: flowToLoad.flow || []
            };
            
            // Load the normalized flow
            taflFlowStore.loadFlow(normalizedFlow);
            
            // Ensure all cards have IDs for internal tracking (including nested cards)
            const flow = taflFlowStore.getFlow();
            if (flow.flow && Array.isArray(flow.flow)) {
                const updatedFlow = this.ensureAllCardsHaveIds(flow.flow);
                // Only update if IDs were added
                if (JSON.stringify(updatedFlow) !== JSON.stringify(flow.flow)) {
                    taflFlowStore.updateFlow({ flow: updatedFlow });
                }
            }
            
            // Manual refresh after open flow
            this.updateCanvas({
                fullRefresh: true,
                preserveScroll: false,
                source: 'flow:open'
            });
            // UI updates handled by SimpleTAFLPanels via store events
            // this.refreshVariablesUI();
            // this.refreshPreloadUI();
            // this.refreshRulesUI();
            this.updateUI();
            taflFlowStore.setDirty(false);
            
            const flowName = taflFlowStore.getFlow().metadata?.name || taflFlowStore.getFlow().name || 'Flow';
            this.showNotification(`Loaded flow: ${flowName}`, 'is-success');
        } else {
            console.error('Invalid flow data or ID provided');
        }
    }
    
    // saveFlowModal() and saveFlow() moved to tafl-editor-modals.js
    
    async validateFlow() {
        try {
            const result = await taflAPI.validateFlow({
                metadata: taflFlowStore.getFlow().metadata,
                variables: taflFlowStore.getFlow().variables,
                flow: taflFlowStore.getFlow().flow
            });
            
            this.displayValidationResults(result);
            this.switchPropertiesTab('validation');
        } catch (error) {
            this.showNotification(error.message, 'is-danger');
        }
    }
    
    displayValidationResults(result) {
        const resultsContainer = document.getElementById('validation-results');
        
        if (result.valid && result.errors.length === 0 && result.warnings.length === 0) {
            resultsContainer.innerHTML = `
                <div class="validation-item success">
                    <span class="icon validation-icon has-text-success">
                        <i class="mdi mdi-check-circle"></i>
                    </span>
                    <div class="validation-message">
                        <strong>TAFL Flow is valid!</strong><br>
                        No errors or warnings found.
                    </div>
                </div>
            `;
        } else {
            let html = '';
            
            result.errors.forEach(error => {
                html += `
                    <div class="validation-item error">
                        <span class="icon validation-icon has-text-danger">
                            <i class="mdi mdi-alert-circle"></i>
                        </span>
                        <div class="validation-message">
                            <strong>Error:</strong> ${error}
                        </div>
                    </div>
                `;
            });
            
            result.warnings.forEach(warning => {
                html += `
                    <div class="validation-item warning">
                        <span class="icon validation-icon has-text-warning">
                            <i class="mdi mdi-alert"></i>
                        </span>
                        <div class="validation-message">
                            <strong>Warning:</strong> ${warning}
                        </div>
                    </div>
                `;
            });
            
            resultsContainer.innerHTML = html;
        }
    }
    
    async executeFlow() {
        try {
            const result = await taflAPI.executeFlow({
                metadata: taflFlowStore.getFlow().metadata,
                variables: taflFlowStore.getFlow().variables,
                flow: taflFlowStore.getFlow().flow
            });
            
            if (result.success) {
                this.showNotification('TAFL flow executed successfully (test mode)', 'is-success');
                console.log('Execution log:', result.execution_log);
            } else {
                this.showNotification(`Execution failed: ${result.message}`, 'is-danger');
            }
        } catch (error) {
            this.showNotification(error.message, 'is-danger');
        }
    }
    
    refreshYAML() {
        if (!this.yamlEditor) return;
        
        this.yamlEditor.setValue(this.generateYAML());
    }
    
    /**
     * Clean card IDs from flow data recursively
     * Removes all 'id' fields that start with 'card_'
     * Preserves TAFL syntax-required IDs
     */
    cleanCardIds(data) {
        if (Array.isArray(data)) {
            // Process array
            return data.map(item => this.cleanCardIds(item));
        } else if (data && typeof data === 'object') {
            // Process object
            const cleaned = {};
            for (const [key, value] of Object.entries(data)) {
                // Skip card IDs at the statement level
                if (key === 'id' && typeof value === 'string' && value.startsWith('card_')) {
                    continue; // Skip this card ID
                }
                // Recursively clean all other values
                cleaned[key] = this.cleanCardIds(value);
            }
            return cleaned;
        } else {
            // Return other data types as-is
            return data;
        }
    }
    
    generateYAML() {
        // Generate TAFL v1.1 compliant YAML
        const flow = taflFlowStore.getFlow();
        
        // Create clean metadata without id or null values
        const metadata = {};
        // Only include non-null, non-empty values
        if (flow.metadata.name && flow.metadata.name !== 'Untitled Flow') {
            metadata.name = flow.metadata.name;
        }
        if (flow.metadata.version) {
            metadata.version = flow.metadata.version;
        }
        if (flow.metadata.description && flow.metadata.description.trim()) {
            metadata.description = flow.metadata.description;
        }
        // Never include id in YAML output
        
        const flowData = {
            metadata: metadata
        };
        
        // Add optional sections if they have content
        if (flow.rules && Object.keys(flow.rules).length > 0) {
            flowData.rules = flow.rules;
        }
        
        if (flow.preload && Object.keys(flow.preload).length > 0) {
            flowData.preload = flow.preload;
        }
        
        if (flow.settings) {
            // Only include non-default settings
            const settings = {};
            if (flow.settings.timeout && flow.settings.timeout !== 3600) {
                settings.timeout = flow.settings.timeout;
            }
            if (flow.settings.max_retries && flow.settings.max_retries !== 3) {
                settings.max_retries = flow.settings.max_retries;
            }
            if (flow.settings.retry_on_failure !== undefined && flow.settings.retry_on_failure !== false) {
                settings.retry_on_failure = flow.settings.retry_on_failure;
            }
            if (Object.keys(settings).length > 0) {
                flowData.settings = settings;
            }
        }
        
        if (flow.variables && Object.keys(flow.variables).length > 0) {
            flowData.variables = flow.variables;
        }
        
        // Add flow section - clean card IDs recursively
        flowData.flow = this.cleanCardIds(flow.flow);
        
        // Use js-yaml to convert to YAML
        try {
            const yamlOutput = jsyaml.dump(flowData, {
                indent: 2,
                lineWidth: -1,
                noRefs: true,
                sortKeys: false,
                skipInvalid: true,  // Skip undefined, null values
                quotingType: '"',   // Force double quotes for consistency
                forceQuotes: true   // Always quote string values
            });
            console.log('Generated YAML (no id):', yamlOutput.substring(0, 200));
            return yamlOutput;
        } catch (error) {
            console.error('Error generating YAML:', error);
            return '# Error generating YAML\n# ' + error.message;
        }
    }
    
    parseYAML(yamlString) {
        try {
            const parsed = jsyaml.load(yamlString);
            
            // Extract metadata from either top level or metadata object
            const metadata = parsed.metadata || {
                id: parsed.id || 'untitled',
                name: parsed.name || 'Untitled Flow',
                version: parsed.version || '1.0',
                description: parsed.description || ''
            };
            
            // Convert parsed YAML back to flow format
            const flow = {
                metadata: metadata,
                flow: [],  // Use 'flow' not 'steps' - this is TAFL standard!
                settings: parsed.settings || {},
                preload: parsed.preload || {},
                rules: parsed.rules || {},
                variables: parsed.variables || {}
            };
            
            // Convert flow array (not steps!)
            if (parsed.flow && Array.isArray(parsed.flow)) {
                flow.flow = parsed.flow;  // Keep the original format
            }
            
            return flow;
        } catch (error) {
            console.error('Error parsing YAML:', error);
            throw error;
        }
    }
    
    applyYAMLChanges() {
        // Set a flag for testing
        window._applyYAMLCalled = true;
        
        if (!this.yamlEditor) {
            console.warn('No YAML editor initialized');
            window._applyYAMLError = 'No YAML editor';
            return;
        }
        
        const yamlContent = this.yamlEditor.getValue();
        console.log('Applying YAML content:', yamlContent.substring(0, 200));
        window._yamlContent = yamlContent;
        
        try {
            // Parse YAML and update flow
            const flow = this.parseYAML(yamlContent);
            console.log('Parsed flow:', flow);
            
            // Update store with proper structure (flow.flow is already in correct format)
            const updateData = {
                metadata: flow.metadata,
                flow: flow.flow,  // Use 'flow' not 'steps'
                settings: flow.settings,
                preload: flow.preload,
                rules: flow.rules,
                variables: flow.variables
            };
            console.log('Updating store with:', updateData);
            taflFlowStore.updateFlow(updateData);
            
            // 手動觸發完整重繪（因為結構可能大幅改變）
            this.updateCanvas({ 
                fullRefresh: true, 
                preserveScroll: false,  // YAML 改變可能很大，不保留捲動
                source: 'yaml:apply' 
            });
            
            // Re-initialize drag handlers after rebuilding
            this.refreshCardDragHandlers();
            
            // Show success message
            window._applyYAMLSuccess = true;
            this.showNotification('YAML changes applied successfully', 'is-success');
        } catch (error) {
            console.error('Error applying YAML changes:', error);
            window._applyYAMLError = error.message;
            this.showNotification('Error parsing YAML: ' + error.message, 'is-danger');
        }
    }
    
    // Note: rebuildCardsFromFlow has been removed - use updateCanvas instead
    // This function was redundant with refreshCanvas/refreshCanvasCore
    
    markDirty() {
        taflFlowStore.setDirty(true);
        this.updateUI();
    }
    
    updateUI() {
        // Update metadata inputs
        document.getElementById('flow-id').value = taflFlowStore.getFlow().metadata.id || '';
        document.getElementById('flow-name').value = taflFlowStore.getFlow().metadata.name || '';
        document.getElementById('flow-version').value = taflFlowStore.getFlow().metadata.version || '1.0';
        document.getElementById('flow-description').value = taflFlowStore.getFlow().metadata.description || '';
        
        // Update save button state
        const saveBtn = document.getElementById('save-flow-btn');
        saveBtn.disabled = !taflFlowStore.isDirty() || taflFlowStore.getFlow().flow.length === 0;
        
        // 使用全域事件委派，不需要個別設置
        // setTimeout(() => {
        //     this.setupNestedCardInteractions();
        // }, 100);
        
        // Update title
        document.title = `TAFL Editor - ${taflFlowStore.getFlow().metadata.name}${taflFlowStore.isDirty() ? ' *' : ''}`;
    }
    
    showNotification(message, type = 'is-info') {
        // 使用新的通知模組
        if (window.taflNotifications) {
            // 轉換舊的 type 格式到新格式
            const typeMap = {
                'is-success': 'success',
                'is-warning': 'warning',
                'is-danger': 'danger',
                'is-error': 'error',
                'is-info': 'info',
                'success': 'success',
                'warning': 'warning',
                'danger': 'danger',
                'error': 'error',
                'info': 'info'
            };
            
            const notificationType = typeMap[type] || 'info';
            window.taflNotifications.show(message, notificationType);
        } else {
            // 備援方案：如果模組未載入，使用簡單通知
            console.warn('TAFLNotifications module not loaded, using fallback');
            const notification = document.createElement('div');
            notification.className = `notification ${type}`;
            notification.style.cssText = `
                position: fixed;
                top: 20px;
                right: 20px;
                z-index: 9999;
                min-width: 300px;
            `;
            notification.innerHTML = `
                <button class="delete"></button>
                ${message}
            `;
            document.body.appendChild(notification);
            
            setTimeout(() => notification.remove(), 5000);
            notification.querySelector('.delete').addEventListener('click', () => {
                notification.remove();
            });
        }
    }
}

// Initialize TAFL Editor when page loads
document.addEventListener('DOMContentLoaded', async () => {
    window.taflEditor = new TAFLEditor();
    
    // Setup event delegation for flow dropdown items
    const dropdownContent = document.getElementById('flows-dropdown-content');
    if (dropdownContent) {
        dropdownContent.addEventListener('click', function(e) {
            const flowItem = e.target.closest('.flow-dropdown-item');
            if (flowItem) {
                const flowId = flowItem.dataset.flowId;
                if (flowId) {
                    loadFlow(flowId);
                    // Save as last edited flow
                    localStorage.setItem('lastEditedFlowId', flowId);
                    // Close dropdown
                    const dropdown = flowItem.closest('.navbar-dropdown');
                    if (dropdown) {
                        dropdown.parentElement.classList.remove('is-active');
                    }
                }
            }
        });
    }
    
    // Load flows dropdown
    await loadFlowsDropdown();
    
    // 不要自動載入 localStorage 中的 flow
    // 使用者需要手動選擇要載入的 flow
    console.log('TAFL Editor initialized. Please select a flow from the dropdown or create a new one.');
});

// Load first available flow from the list
async function loadFirstAvailableFlow() {
    try {
        const flows = await taflAPI.getFlows();
        
        if (flows.length > 0) {
            // Load the first flow
            const firstFlow = flows[0];
            await loadFlow(firstFlow.id);
            localStorage.setItem('lastEditedFlowId', firstFlow.id);
        }
    } catch (error) {
        console.error('Failed to load first available flow:', error);
    }
}

// Load flows into dropdown
async function loadFlowsDropdown() {
    const dropdownContent = document.getElementById('flows-dropdown-content');
    if (!dropdownContent) return;
    
    try {
        const flows = await taflAPI.getFlows();
        
        if (flows.length === 0) {
            dropdownContent.innerHTML = `
                <div class="dropdown-item has-text-grey">
                    No flows available
                </div>
            `;
            return;
        }
        
        dropdownContent.innerHTML = flows.map(flow => `
            <a class="dropdown-item flow-dropdown-item" data-flow-id="${flow.id}">
                <div>
                    <strong>${flow.name || flow.id}</strong>
                    ${flow.description ? `<br><small class="has-text-grey">${flow.description}</small>` : ''}
                </div>
            </a>
        `).join('');
        
    } catch (error) {
        console.error('Failed to load flows:', error);
        dropdownContent.innerHTML = `
            <div class="dropdown-item has-text-danger">
                Failed to load flows
            </div>
        `;
    }
}

// Load a specific flow
async function loadFlow(flowId) {
    try {
        const flowData = await taflAPI.getFlow(flowId);
        
        if (window.taflEditor) {
            window.taflEditor.loadFlow(flowData);
            
            // Show success notification using the notification module
            const flowName = flowData.flow?.metadata?.name || flowData.flow?.name || flowId;
            if (window.taflNotifications) {
                window.taflNotifications.success(`Flow "${flowName}" loaded successfully`);
            } else {
                // Fallback if notification module not loaded
                console.log(`Flow "${flowName}" loaded successfully`);
            }
        }
    } catch (error) {
        console.error('Failed to load flow:', error);
        if (window.taflNotifications) {
            window.taflNotifications.error('Failed to load flow: ' + error.message);
        } else {
            alert('Failed to load flow: ' + error.message);
        }
    }
}