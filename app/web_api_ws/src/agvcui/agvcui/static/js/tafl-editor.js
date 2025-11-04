/**
 * TAFL Editor - Professional Drag & Drop Visual Editor
 * Task Automation Flow Language Visual Editor v3.0
 * Complete rewrite with professional drag & drop experience
 * Now using miniStore for state management
 */

// Import store
import { taflFlowStore } from './tafl-editor/tafl-editor-store.js';
// Import notifications module
import taflEditorNotifications, { TAFLEditorNotifications } from './tafl-editor/tafl-editor-notifications.js';
// Import API module
import taflAPI from './tafl-editor/tafl-editor-api.js';
// Import modals module
import taflModals, { TAFLEditorModals } from './tafl-editor/tafl-editor-modals.js';
// Import panels module
import { taflPanels } from './tafl-editor/tafl-panels.js';
// Import enhancements module
import TAFLEditorEnhancements, { injectEnhancementsStyles } from './tafl-editor/tafl-editor-enhancements.js';
// Import YAML module
import { TAFLEditorYaml } from './tafl-editor/tafl-editor-yaml.js';
import taflEditorValidator, { TAFLEditorValidator } from './tafl-editor/tafl-editor-validator.js';
// Import Cards module
import taflEditorCards, { TAFLEditorCards } from './tafl-editor/tafl-editor-cards.js';
// Import Utils module
import taflEditorUtils, { TAFLEditorUtils } from './tafl-editor/tafl-editor-utils.js';
// Import Flow module
import taflEditorFlow, { TAFLEditorFlow } from './tafl-editor/tafl-editor-flow.js';
// Import Variables module
import taflEditorVariables, { TAFLEditorVariables } from './tafl-editor/tafl-editor-variables.js';
import taflEditorPreload, { TAFLEditorPreload } from './tafl-editor/tafl-editor-preload.js';
// Import Rules module
import taflEditorRules, { TAFLEditorRules } from './tafl-editor/tafl-editor-rules.js';
// Import Canvas module - Linus style: simple and direct
import TAFLEditorCanvas from './tafl-editor/tafl-editor-canvas.js';
// Import Events module  
import taflEditorEvents, { TAFLEditorEvents } from './tafl-editor/tafl-editor-events.js';
// Import Properties modules
import taflEditorProperties, { TAFLEditorProperties } from './tafl-editor/tafl-editor-properties.js';
import taflPanelsProperties, { TAFLPanelsProperties } from './tafl-editor/tafl-panels-properties.js';
// Execution panel for showing test run and execution results
import taflExecutionPanel, { TAFLEditorExecutionPanel } from './tafl-editor/tafl-editor-execution.js';
// Import DragDrop module - Linus style: handles ALL drag-drop with ZERO special cases
import TAFLEditorDragDrop from './tafl-editor/tafl-editor-dragdrop.js';

class TAFLEditor {
    constructor() {
        // Store manages the flow state now
        // Removed: this.selectedCard - now managed by Store
        // Removed: this.draggedElement, this.dragData - now managed by DragDrop module
        this.yamlModule = null; // YAML module instance
        this.validatorModule = null; // Validator module instance
        this.lastInsertionIndex = undefined;  // Track last drop position to prevent flashing
        this.cardOriginalPositions = new Map();  // 追蹤卡片的原始位置
        
        // Initialize simplified drag system
        // Using built-in drag system
        
        // Temporary verbDefinitions reference for compatibility
        this.verbDefinitions = {};
        
        // Initialize panels module
        this.panels = taflPanels;
        
        // Initialize enhancements module
        this.enhancements = new TAFLEditorEnhancements(this);
        
        // Initialize notifications module
        this.notificationsModule = new TAFLEditorNotifications(this);
        taflEditorNotifications.editor = this; // Set editor reference for singleton
        
        // Initialize utils module
        this.utilsModule = new TAFLEditorUtils();
        taflEditorUtils.verbDefinitions = this.verbDefinitions; // Share verb definitions
        
        // Initialize flow module
        this.flowModule = new TAFLEditorFlow(this);
        taflEditorFlow.editor = this; // Set editor reference for singleton
        
        // Initialize variables module
        this.variablesModule = new TAFLEditorVariables(this);
        taflEditorVariables.editor = this; // Set editor reference for singleton
        
        // Initialize rules module
        this.rulesModule = new TAFLEditorRules(this);
        taflEditorRules.editor = this; // Set editor reference for singleton
        
        // Initialize canvas module - Linus style: simple and direct
        this.canvasModule = new TAFLEditorCanvas(this);
        
        // Initialize drag-drop module - Linus style: handles ALL drag-drop with ZERO special cases
        this.dragDropModule = new TAFLEditorDragDrop(this);
        
        // Initialize properties modules
        taflEditorProperties.verbDefinitions = this.verbDefinitions;
        
        // Properties panel UI (initialized after DOM ready)
        
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
            // Simple: just update
            this.canvasModule.update();
            this.updateUI();
            this.yamlModule.refreshYaml();
        });
        
        // Card selection
        taflFlowStore.on('card:selected', (cardId) => {
            console.log('🎯 [SELECTION] card:selected event received for:', cardId);
            // Update visual selection state
            this.updateCardSelectionVisuals(cardId);
        });
        
        // Card additions
        taflFlowStore.on('card:added', (data) => {
            console.log('🔍 DEBUG-15: card:added event received:', data);
            console.log('🔍 DEBUG-16: canvasModule exists:', !!this.canvasModule);
            console.log('🔍 DEBUG-17: canvasModule.update exists:', !!this.canvasModule?.update);
            
            if (this.canvasModule && this.canvasModule.update) {
                console.log('🔍 DEBUG-18: Calling canvasModule.update()');
                this.canvasModule.update();
                console.log('🔍 DEBUG-19: canvasModule.update() completed');
            } else {
                console.error('❌ DEBUG-ERROR: canvasModule or update method not available!');
            }
            
            const cardEl = document.querySelector(`[data-card-id="${data.card.id}"]`);
            if (cardEl) {
                console.log('🔍 DEBUG-20: Card element found, selecting:', data.card.id);
                this.selectCard(cardEl);
            } else {
                console.log('🔍 DEBUG-21: Card element NOT found in DOM:', data.card.id);
            }
        });
        
        // Card updates
        taflFlowStore.on('card:updated', (data) => {
            this.refreshCard(data.cardId);
        });
        
        // Card deletions
        taflFlowStore.on('card:deleted', (cardId) => {
            console.log('🎬 Event handler: card:deleted for', cardId);
            this.canvasModule.update();
            // Force YAML refresh on delete
            this.yamlModule.refreshYaml();
            this.updateUI();
        });
        
        // Card moves
        taflFlowStore.on('card:moved', (data) => {
            // Linus says: no special cases
            this.canvasModule.update();
        });
        
        // Variables changes
        taflFlowStore.on('variables:changed', (variables) => {
            this.variablesModule.renderVariables();
        });
        
        // Dirty state changes
        taflFlowStore.on('dirty:changed', (isDirty) => {
            this.updateSaveButton(isDirty);
        });
        
        // Flow loaded
        taflFlowStore.on('flow:loaded', (flowData) => {
            console.log('🔄 flow:loaded event received in tafl-editor.js');
            console.log('🔄 Event data flow.flow:', flowData?.flow, 'Length:', flowData?.flow?.length);
            console.log('🔄 Store flow.flow:', taflFlowStore.getFlow()?.flow, 'Length:', taflFlowStore.getFlow()?.flow?.length);
            this.canvasModule.update();
            this.updateUI();
            // Refresh YAML to sync with loaded flow - add delay to ensure flow is fully loaded
            setTimeout(() => {
                this.yamlModule.refreshYaml();
                console.log('🔄 YAML refreshed after flow load');
            }, 200);
            console.log('🔄 flow:loaded handler complete');
        });
        
        // Flow cleared (for New Flow action)
        taflFlowStore.on('flow:cleared', () => {
            console.log('🧹 flow:cleared event received in tafl-editor.js');
            const currentFlow = taflFlowStore.getFlow();
            console.log('🧹 Current flow after clear event:', currentFlow);
            console.log('🧹 Flow.flow after clear event:', currentFlow?.flow);
            console.log('🧹 Flow.flow length after clear event:', currentFlow?.flow?.length);
            
            this.canvasModule.update();
            this.updateUI();
            this.yamlModule.refreshYaml();
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
        
        // Switch case operations - ensure YAML and canvas update
        taflFlowStore.on('switch:case-added', (data) => {
            console.log('📝 switch:case-added event received:', data);
            this.canvasModule.update();
            this.yamlModule.refreshYaml();
        });
        
        taflFlowStore.on('switch:case-removed', (data) => {
            console.log('📝 switch:case-removed event received:', data);
            this.canvasModule.update();
            this.yamlModule.refreshYaml();
        });
        
        taflFlowStore.on('switch:default-added', (data) => {
            console.log('📝 switch:default-added event received:', data);
            this.canvasModule.update();
            this.yamlModule.refreshYaml();
        });
        
        taflFlowStore.on('switch:default-removed', (data) => {
            console.log('📝 switch:default-removed event received:', data);
            this.canvasModule.update();
            this.yamlModule.refreshYaml();
        });
        
        taflFlowStore.on('switch:case-moved', (data) => {
            console.log('📝 switch:case-moved event received:', data);
            this.canvasModule.update();
            this.yamlModule.refreshYaml();
        });
    }
    
    async initializeEditor() {
        // Register global variables first (must be before module initialization)
        this.registerGlobalVariables();
        
        // Log the initial state to see what's in the store
        console.log('🔍 Initial store state on editor init:', taflFlowStore.getFlow());
        
        await this.loadVerbDefinitions();
        this.initializeCards(); // Initialize cards module first
        this.initializeYAML();   // Initialize YAML and preload modules
        this.initializeValidator();
        this.initializeVariables();
        
        // Now initialize events after all modules are created
        this.initializeEvents();
        
        this.setupGlobalEventDelegation();  // 新增：全域事件委派
        this.initializeDisplayModes();  // Initialize display mode switcher
        // Initialize drag-drop system
        if (this.dragDropModule) {
            this.dragDropModule.init();
        }
        this.updateUI();
        
        // Check if store has existing flow data and render it
        const flow = taflFlowStore.getFlow();
        if (flow && flow.flow && flow.flow.length > 0) {
            console.log('🔍 Store has existing flow data, refreshing canvas with', flow.flow.length, 'cards');
            this.canvasModule.update();
        }
        
        // 初始化歷史記錄（在載入流程之後）
        this.initializeHistory();
        
        // Log state after initialization
        console.log('🔍 Store state after editor init:', taflFlowStore.getFlow());
    }
    
    /**
     * 初始化歷史記錄系統
     */
    initializeHistory() {
        // 保存初始狀態作為第一個歷史記錄
        taflFlowStore.saveSnapshot('Initial state');
        
        // 訂閱歷史變更事件
        taflFlowStore.on('history:changed', (state) => {
            this.updateUndoRedoButtons();
        });
        
        // 初始更新按鈕狀態
        this.updateUndoRedoButtons();
    }
    
    /**
     * Register global variables for modules that need them
     */
    registerGlobalVariables() {
        window.taflFlowStore = taflFlowStore;
        window.taflModals = taflModals;
        window.taflAPI = taflAPI;
        window.taflPanels = this.panels;
        window.taflEditor = this;
        window.TAFLEditor = TAFLEditor;
        
        // Properties modules need global references
        window.taflEditorProperties = taflEditorProperties;
        window.taflPanelsProperties = taflPanelsProperties;
        
        // Reinitialize properties panel after globals are registered
        console.log('🎯 [INIT] Reinitializing properties panel after globals registered');
        if (taflPanelsProperties && taflPanelsProperties.reinitialize) {
            taflPanelsProperties.reinitialize();
        }
    }
    
    initializeYAML() {
        // Create YAML module instance with reference to this editor
        this.yamlModule = new TAFLEditorYaml(this);
        // Initialize YAML editor
        this.yamlModule.initYamlEditor('yaml-editor-main');
        // Initial YAML refresh
        this.yamlModule.refreshYaml();
        
        // Initialize preload module
        this.preloadModule = new TAFLEditorPreload(this);
        taflEditorPreload.editor = this; // Set editor reference for singleton
    }

    initializeValidator() {
        // Create validator module instance
        this.validatorModule = taflEditorValidator;
        // Initialize validator with editor reference
        this.validatorModule.init(this);
    }
    
    initializeCards() {
        // Create cards module instance
        this.cardsModule = taflEditorCards;
        // Initialize cards module with editor reference and verb definitions
        this.cardsModule.init(this, this.verbDefinitions);
    }
    
    initializeEvents() {
        // Initialize events module after all other modules are created
        this.eventsModule = new TAFLEditorEvents(this);
        taflEditorEvents.editor = this;
        
        // Initialize execution panel
        taflExecutionPanel.editor = this;
        
        // Now initialize event listeners with all modules available
        this.eventsModule.initEventListeners();
    }
    
    async loadVerbDefinitions() {
        try {
            const verbs = await taflAPI.loadVerbs();
            this.verbDefinitions = verbs;
            taflFlowStore.setVerbDefinitions(verbs);
            // Share with Properties module
            taflEditorProperties.verbDefinitions = verbs;
            
            // Share verb definitions with properties module
            if (this.propertiesModule) {
                this.propertiesModule.verbDefinitions = verbs;
            }
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
                    // Call async deleteCard - no need to await since confirm is synchronous
                    this.deleteCard(card.dataset.cardId);
                }
                return;
            }
            
            // 選擇卡片
            e.stopPropagation();
            console.log('🎯 [STEP 1] Click event on card');
            console.log('🎯 [STEP 1a] Card element:', card);
            console.log('🎯 [STEP 1b] Card dataset:', card.dataset);
            console.log('🎯 [STEP 1c] Card ID from dataset:', card.dataset.cardId);
            console.log('🎯 [STEP 1d] Card verb from dataset:', card.dataset.verb);
            console.log('🎯 [STEP 1e] Card className:', card.className);
            console.log('🎯 [STEP 1f] Card outerHTML (first 200 chars):', card.outerHTML.substring(0, 200));
            this.selectCard(card);
        });
    }
    
    /**
     * 獲取 drop zone 的位置信息
     */
    getDropZonePosition(dropZone, event) {
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
        
        // 主畫佈 drop zone - 使用統一的計算邏輯
        const canvasDropZone = document.getElementById('canvas-drop-zone');
        const cards = Array.from(canvasDropZone.querySelectorAll(':scope > .tafl-card'));
        
        // 如果有 event，使用統一的位置計算方法
        if (event) {
            const result = this.calculateIndexFromPosition(event, cards);
            
            // 處理死區情況
            if (result && typeof result === 'object' && result.isDeadZone) {
                console.log('[getDropZonePosition] Dead zone detected');
                return {
                    nested: null,
                    index: result.index,
                    isDeadZone: true
                };
            }
            
            console.log('[getDropZonePosition] Calculated index:', result);
            return {
                nested: null,
                index: result
            };
        }
        
        // 沒有 event 時，預設放在最後
        return {
            nested: null,
            index: cards.length
        };
    }
    
    /**
     * PROFESSIONAL DRAG & DROP SYSTEM
     * DEPRECATED: Now handled by TAFLEditorDragDrop module
     * Professional drag and drop implementation with advanced interaction patterns
     */
    setupDragAndDrop_DEPRECATED() {
        const canvas = document.getElementById('flow-canvas');
        
        if (!canvas) {
            console.error('Canvas element not found!');
            return;
        }
        
        // DEPRECATED: Now handled by TAFLEditorDragDrop module
        // this.setupToolboxDragSources_DEPRECATED();
        // this.setupCanvasDropTarget_DEPRECATED(canvas);
        
        // Initialize drop zones
        this.createInitialDropZone();
        
        // Make sure existing cards are draggable
        this.refreshCardDragHandlers();
        
        console.log('✅ Drag and drop initialized');
    }
    
    setupToolboxDragSources_DEPRECATED() {
        // Make verb items draggable
        document.querySelectorAll('.verb-item').forEach(item => {
            item.draggable = true;
            
            // Visual feedback that item is draggable
            item.style.cursor = 'move';
            item.title = 'Drag to add to flow';
            
            item.addEventListener('dragstart', (e) => {
                e.stopPropagation();
                document.body.classList.add('dragging');
                this.canvasModule.startDrag();
                
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
                this.canvasModule.endDrag();
                this.hideAllDropZones();
                
                // 清理所有 placeholder
                document.querySelectorAll('.tafl-card-placeholder').forEach(p => p.remove());
                this.hideDropIndicators();
                
                // Don't clear dragData immediately, let drop handler use it first
                setTimeout(() => {
                    this.dragData = null;
                }, 100);
                
                console.log('🎯 Ended dragging');
            });
        });
    }
    
    setupCanvasDropTarget_DEPRECATED(canvas) {
        // Canvas-level drag events
        canvas.addEventListener('dragover', (e) => {
            e.preventDefault();
            
            // Always update drop indicators during dragover
            this.updateUnifiedDropIndicator(e);
            
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
            const dropPosition = this.calculateUnifiedDropPosition(e);
            this.addCardAtPosition(dragData.verb, dropPosition);
        } else if (dragData.type === 'card') {
            // Moving existing card
            const dropPosition = this.calculateUnifiedDropPosition(e);
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
        const result = this.calculateIndexFromPosition(e, mainCards);
        
        // 處理死區情況
        if (result && typeof result === 'object' && result.isDeadZone) {
            return {
                nested: null,
                index: result.index,
                isDeadZone: true
            };
        }
        
        return {
            nested: null,
            index: result
        };
    }
    
    /**
     * 創建 Placeholder 元素
     */
    createPlaceholder() {
        const placeholder = document.createElement('div');
        placeholder.className = 'tafl-card-placeholder';
        placeholder.dataset.placeholder = 'true';
        
        // 如果有拖動元素，匹配其高度
        if (this.draggedElement) {
            const height = this.draggedElement.offsetHeight;
            placeholder.style.minHeight = `${height}px`;
        }
        
        return placeholder;
    }
    
    /**
     * 輔助方法：根據滑鼠位置計算插入索引（30% 規則）
     */
    calculateIndexFromPosition(e, cards) {
        const mouseY = e.clientY;
        
        console.log('📏 calculateIndexFromPosition (30% threshold):');
        console.log('  - mouseY:', mouseY);
        console.log('  - cards count:', cards.length);
        
        // 如果沒有卡片，返回 0
        if (cards.length === 0) {
            console.log('  - No cards, returning 0');
            return 0;
        }
        
        // 獲取正在拖動的卡片 ID
        const draggingCardId = this.dragData?.cardId || this.draggedElement?.dataset?.cardId;
        
        // 使用 30% 規則進行更靈敏的檢測
        for (let i = 0; i < cards.length; i++) {
            const card = cards[i];
            
            // 跳過 placeholder 和正在拖動的卡片
            if (card.dataset.placeholder === 'true') continue;
            if (draggingCardId && card.dataset.cardId === draggingCardId) continue;
            
            const rect = card.getBoundingClientRect();
            const threshold = rect.top + rect.height * 0.3; // 30% 閾值
            
            console.log(`  - Card ${i}: top=${rect.top}, height=${rect.height}, threshold=${threshold}`);
            
            if (mouseY < threshold) {
                console.log(`  - Mouse is above 30% of card ${i}, target index = ${i}`);
                return i;
            }
        }
        
        // 如果沒有在任何卡片 30% 之前，則放在最後
        console.log('  - Mouse is below all cards, target index = end');
        return cards.length;
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
            // 新增卡片 - 使用 cards 模組的方法
            console.log('🎯 Adding new card from verb:', dragDataCopy.verb, 'at position:', position);
            taflEditorCards.addCardAtPosition(dragDataCopy.verb, position);
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
     * 更新 Placeholder 位置
     */
    updatePlaceholder(e) {
        // 移除現有的 placeholder (但保留 original-position-placeholder)
        const existingPlaceholder = document.querySelector('.tafl-card-placeholder:not(.original-position-placeholder)');
        if (existingPlaceholder) {
            existingPlaceholder.remove();
        }
        
        // 計算新位置
        const position = this.calculateUnifiedDropPosition(e);
        if (!position) return;
        
        // 創建新的 placeholder
        const placeholder = this.createPlaceholder();
        
        // 根據位置類型插入 placeholder
        if (position.nested) {
            // 巢狀結構中
            const parentCard = document.querySelector(`[data-card-id="${position.nested.parentCardId}"]`);
            const nestedContainer = parentCard?.querySelector(`.nested-cards[data-branch="${position.nested.branchType}"]`);
            
            if (nestedContainer) {
                const nestedCards = Array.from(nestedContainer.querySelectorAll(':scope > .tafl-card'));
                if (position.index >= nestedCards.length) {
                    nestedContainer.appendChild(placeholder);
                } else {
                    nestedCards[position.index].insertAdjacentElement('beforebegin', placeholder);
                }
            }
        } else {
            // 主流程中
            const dropZone = document.getElementById('canvas-drop-zone');
            if (!dropZone) return;
            
            const mainCards = Array.from(dropZone.querySelectorAll(':scope > .tafl-card:not(.tafl-card-placeholder)'));
            if (position.index >= mainCards.length) {
                dropZone.appendChild(placeholder);
            } else if (position.index === 0 && mainCards.length > 0) {
                mainCards[0].insertAdjacentElement('beforebegin', placeholder);
            } else if (position.index > 0 && position.index < mainCards.length) {
                mainCards[position.index].insertAdjacentElement('beforebegin', placeholder);
            } else {
                dropZone.appendChild(placeholder);
            }
        }
        
        // 記錄位置
        this.lastPlaceholderPosition = position;
    }
    
    /**
     * UNIFIED DROP INDICATOR UPDATE (已棄用，改用 updatePlaceholder)
     * 保留此函數以維持相容性，但改為調用 updatePlaceholder
     */
    updateUnifiedDropIndicator(e) {
        // 使用原來的 insertion-line 系統（水平引導線）
        this.updateDropIndicators(e);
        return;
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
        // Delegate to utils module
        return this.utilsModule.generateId();
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
        // 移除舊的 insertion-line
        document.querySelectorAll('.insertion-line').forEach(line => {
            line.remove();
        });
        // 移除舊的 drop-indicator
        document.querySelectorAll('.drop-indicator').forEach(line => {
            line.remove();
        });
        // 移除新的 placeholder (但在拖動期間保留 original-position-placeholder)
        const isDragging = document.body.classList.contains('dragging');
        if (isDragging) {
            // 拖動期間只移除非原始位置的 placeholder
            document.querySelectorAll('.tafl-card-placeholder:not(.original-position-placeholder)').forEach(p => {
                p.remove();
            });
        } else {
            // 拖動結束後移除所有 placeholder
            document.querySelectorAll('.tafl-card-placeholder').forEach(p => {
                p.remove();
            });
        }
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
        console.log('🔍 DEBUG-3: Editor.addCard called with verb:', verb);
        console.log('🔍 DEBUG-4: cardsModule exists:', !!this.cardsModule);
        // Delegate to cards module
        if (this.cardsModule) {
            console.log('🔍 DEBUG-5: Delegating to cardsModule.addCard');
            this.cardsModule.addCard(verb);
            console.log('🔍 DEBUG-6: cardsModule.addCard returned');
        } else {
            console.error('❌ DEBUG-ERROR: cardsModule is not initialized!');
        }
    }
    
    addCardAtPosition(verb, position) {
        // Delegate to cards module
        this.cardsModule.addCardAtPosition(verb, position);
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
        console.log('🎯 [CREATE-1] createCardElementSimple called with:', step);
        // TAFL v1.1 Fix: Extract verb from step object
        console.log('🎯 [CREATE-2] verbDefinitions available:', !!this.verbDefinitions);
        if (this.verbDefinitions) {
            console.log('🎯 [CREATE-2a] Available verbs:', Object.keys(this.verbDefinitions));
        }
        
        const verb = step ? Object.keys(step).find(key => {
            const isVerb = this.verbDefinitions && this.verbDefinitions[key];
            console.log('🎯 [CREATE-3] Checking key:', key, 'is verb:', !!isVerb);
            return isVerb;
        }) : null;
        
        console.log('🎯 [CREATE-4] Found verb:', verb);
        
        if (!verb) {
            console.warn('🎯 [CREATE-5] No valid verb found in step:', step);
            const emptyCard = document.createElement('div');
            emptyCard.className = 'tafl-card empty-card';
            emptyCard.innerHTML = '<div class="notification is-warning">Invalid step</div>';
            return emptyCard;
        }
        
        const verbDef = this.verbDefinitions[verb];
        
        // 創建卡片元素（設置 draggable 屬性）
        const cardEl = document.createElement('div');
        cardEl.className = 'tafl-card';
        // Fix: Handle case where step.id is an object (from backend)
        let cardId;
        if (step.id && typeof step.id === 'object') {
            // If ID is an object, extract the actual ID value
            cardId = step.id.id || step.id.value || JSON.stringify(step.id);
            console.log('🔧 [CREATE-CARD] ID is object, extracted:', cardId, 'from:', step.id);
        } else {
            cardId = step.id || this.generateId();
        }
        // Ensure cardId is always a string
        if (typeof cardId === 'object') {
            console.log('🔧 [CREATE-CARD] WARNING: cardId is still an object after extraction:', cardId);
            cardId = cardId.id || JSON.stringify(cardId);
        }
        console.log('🔧 [CREATE-CARD] Final card ID (should be string):', cardId, 'type:', typeof cardId);
        console.log('🔧 [CREATE-CARD] Step object:', JSON.stringify(step));
        console.log('🔧 [CREATE-CARD] Step has id?', !!step.id);
        cardEl.dataset.cardId = cardId;
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
        `;
        
        // Linus style: 統一使用 DOM 操作附加巢狀卡片
        const nestedElements = this.renderNestedCardsSimple(step);
        if (nestedElements && nestedElements.childNodes.length > 0) {
            // 如果有巢狀元素，直接附加到卡片上
            nestedElements.childNodes.forEach(child => {
                cardEl.appendChild(child.cloneNode(true));
            });
        }
        
        // 設置 comment tooltip（如果有）
        if (step.comment) {
            cardEl.title = step.comment;
        }
        
        return cardEl;
    }
    
    /**
     * 簡化版巢狀卡片渲染（Linus style: 統一使用 DOM 操作）
     */
    renderNestedCardsSimple(cardData) {
        const verb = this.getCardVerb(cardData);
        const params = cardData[verb];
        const container = document.createDocumentFragment();
        
        if (typeof params === 'object') {
            if (verb === 'if') {
                if (params.then && Array.isArray(params.then)) {
                    const thenBranch = this.renderNestedBranchSimple('then', params.then);
                    container.appendChild(thenBranch);
                }
                if (params.else && Array.isArray(params.else)) {
                    const elseBranch = this.renderNestedBranchSimple('else', params.else);
                    container.appendChild(elseBranch);
                }
            } else if (verb === 'for' && params.do && Array.isArray(params.do)) {
                const doBranch = this.renderNestedBranchSimple('do', params.do);
                container.appendChild(doBranch);
            } else if (verb === 'switch') {
                if (params.cases && Array.isArray(params.cases)) {
                    params.cases.forEach((caseItem, index) => {
                        if (caseItem.when === 'default') {
                            // 特殊處理 default case
                            const defaultBranch = this.renderNestedBranchSimple('default', caseItem.do || [], 'default');
                            container.appendChild(defaultBranch);
                        } else {
                            // 一般 case，顯示條件，沒有條件顯示 "case ?"
                            const labelText = caseItem.when ? `case ${caseItem.when}` : 'case ?';
                            const caseBranch = this.renderNestedBranchSimple(`case-${index}`, caseItem.do || [], labelText);
                            container.appendChild(caseBranch);
                        }
                    });
                }
                if (params.default && Array.isArray(params.default)) {
                    const defaultBranch = this.renderNestedBranchSimple('default', params.default, 'default');
                    container.appendChild(defaultBranch);
                }
            }
        }
        
        return container;
    }
    
    /**
     * 簡化版巢狀分支渲染 (Linus style: 消除特殊情況，統一使用 DOM)
     */
    renderNestedBranchSimple(branchType, cards, labelText = null) {
        if (!Array.isArray(cards)) {
            // Return empty DOM element instead of empty string
            return document.createElement('div');
        }
        
        const branchClass = branchType.replace(/[^a-zA-Z0-9]/g, '-');
        
        // Create container using DOM instead of HTML string
        const container = document.createElement('div');
        container.className = `nested-cards ${branchClass}-cards`;
        container.dataset.branch = branchType;
        
        // Add branch label
        const branchLabel = document.createElement('div');
        branchLabel.className = 'branch-label';
        branchLabel.textContent = labelText || branchType;
        container.appendChild(branchLabel);
        
        if (cards.length === 0) {
            // 空的巢狀放置區 - using DOM
            const dropZone = document.createElement('div');
            dropZone.className = 'nested-drop-zone empty-zone';
            dropZone.dataset.branch = branchType;
            
            const dropContent = document.createElement('div');
            dropContent.className = 'nested-drop-content';
            dropContent.innerHTML = '<i class="mdi mdi-plus"></i><span>Add cards here</span>';
            
            dropZone.appendChild(dropContent);
            container.appendChild(dropZone);
        } else {
            // 遞迴渲染巢狀卡片 - using DOM operations
            // Add drop indicator before first card
            const firstIndicator = document.createElement('div');
            firstIndicator.className = 'drop-indicator horizontal';
            firstIndicator.dataset.branch = branchType;
            firstIndicator.dataset.position = '0';
            container.appendChild(firstIndicator);
            
            cards.forEach((nestedCard, index) => {
                // Linus style: 直接附加 DOM 元素，不轉換為字串
                const nestedElement = this.createCardElementSimple(nestedCard);
                container.appendChild(nestedElement);  // 直接附加，保留所有屬性和事件
                
                // Add drop indicator after each card
                const indicator = document.createElement('div');
                indicator.className = 'drop-indicator horizontal';
                indicator.dataset.branch = branchType;
                indicator.dataset.position = String(index + 1);
                container.appendChild(indicator);
            });
        }
        
        return container;
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
                        if (caseItem.when === 'default') {
                            // 特殊處理 default case
                            html += this.renderNestedBranch('default', caseItem.do || [], 'default');
                        } else {
                            // 一般 case，顯示條件，沒有條件顯示 "case ?"
                            const labelText = caseItem.when ? `case ${caseItem.when}` : 'case ?';
                            html += this.renderNestedBranch(`case-${index}`, caseItem.do || [], labelText);
                        }
                    });
                }
                if (params.default && Array.isArray(params.default)) {
                    html += this.renderNestedBranch('default', params.default, 'default');
                }
            }
        }
        
        return html;
    }
    
    renderNestedBranch(branchType, cards, labelText = null) {
        if (!Array.isArray(cards)) return '';
        
        const branchClass = branchType.replace(/[^a-zA-Z0-9]/g, '-');
        
        let html = `
            <div class="nested-cards ${branchClass}-cards" data-branch="${branchType}">
                <div class="branch-label">${labelText || branchType}</div>
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
    
    addToNestedStructure(parentCardId, branchType, cardData, index = 0) {
        console.log('🔵 tafl-editor.js addToNestedStructure called with:');
        console.log('  parentCardId:', parentCardId);
        console.log('  branchType:', branchType);
        console.log('  index:', index);
        
        // Ensure the card has an ID
        if (!cardData.id) {
            cardData.id = this.generateId();
        }
        
        // Get the current flow
        const flow = taflFlowStore.getFlow();
        
        // Create position object with index
        const position = {
            nested: {
                parentCardId: parentCardId,
                branchType: branchType
            },
            index: index  // Add the index to position object
        };
        
        console.log('🔵 Position object with index:', position);
        
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
    
    // Delegate to variables module
    initializeVariables() {
        return this.variablesModule.initializeVariables();
    }
    
    // ============================================
    // Legacy Panel Functions (Deprecated)
    // All panel innerHTML updates now handled by SimpleTAFLPanels
    // These modal functions kept for backward compatibility
    // ============================================
    
    updateSaveButton(isDirty) {
        // Delegate to notifications module
        this.notificationsModule.updateSaveButton(isDirty);
    }
    
    // Removed: refreshVariablesUI - handled by SimpleTAFLPanels
    
    // setupModalControls() moved to tafl-editor-modals.js
    
    
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
                    if (currentMode && (currentMode.dataset.mode === 'yaml' || currentMode.dataset.mode === 'both') && this.yamlModule && this.yamlModule.yamlEditor) {
                        setTimeout(() => this.yamlModule.yamlEditor.refresh(), 100);
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
                    
                    if ((mode === 'yaml' || mode === 'both') && this.yamlModule) {
                        setTimeout(() => {
                            // Always refresh YAML content when switching to YAML view
                            this.yamlModule.refreshYaml();
                            // Refresh CodeMirror editor if it exists
                            if (this.yamlModule.yamlEditor) {
                                this.yamlModule.yamlEditor.refresh();
                            }
                            console.log('📝 YAML view refreshed on mode switch');
                        }, 100);
                    }
                }
            });
        });
    }
    
    getDefaultParams(verb) {
        // Update utils module with current verb definitions
        this.utilsModule.verbDefinitions = this.verbDefinitions;
        // Delegate to utils module
        return this.utilsModule.getDefaultParams(verb);
    }
    
    
    getCardTitle(cardData) {
        // Delegate to utils module (since cards module is not initialized yet)
        return this.utilsModule.getCardTitle(cardData);
    }
    
    renderCardParams(cardData) {
        // Delegate to cards module
        return this.cardsModule.renderCardParams(cardData);
    }
    
    formatParamValue(value) {
        // Delegate to utils module
        return this.utilsModule.formatParamValue(value);
    }
    
    selectCard(cardElement) {
        console.log('🎯 [STEP 2] TAFLEditor.selectCard called with:', cardElement?.dataset?.cardId);
        // Delegate to cards module
        this.cardsModule.selectCard(cardElement);
        console.log('🎯 [STEP 3] Delegated to cardsModule.selectCard');
        // Removed: this.selectedCard - now managed by Store
    }
    
    deselectCard() {
        // Delegate to Store
        taflFlowStore.deselectCard();
        // UI update handled by Store events
    }
    
    /**
     * Update visual selection state for cards
     * This method handles the visual aspects of card selection
     * @param {string} cardId - The ID of the selected card
     */
    updateCardSelectionVisuals(cardId) {
        console.log('🎯 [SELECTION] updateCardSelectionVisuals called for:', cardId);
        
        // Remove selected class from all cards
        document.querySelectorAll('.tafl-card.selected').forEach(card => {
            card.classList.remove('selected');
        });
        
        // Add selected class to the target card
        if (cardId) {
            const selectedCard = document.querySelector(`[data-card-id="${cardId}"]`);
            if (selectedCard) {
                selectedCard.classList.add('selected');
                console.log('🎯 [SELECTION] Added selected class to card:', cardId);
                
                // Scroll into view if needed
                selectedCard.scrollIntoView({ 
                    behavior: 'smooth', 
                    block: 'nearest' 
                });
            } else {
                console.warn('🎯 [SELECTION] Card element not found for ID:', cardId);
            }
        }
    }
    
    // Properties panel methods removed - handled by tafl-editor-properties.js and tafl-panels-properties.js modules
    // The following methods have been migrated to dedicated modules:
    // - updatePropertiesPanel() → tafl-panels-properties.js
    // - renderPropertyEditor() → tafl-editor-properties.js
    // - renderSwitchCasesEditor() → tafl-editor-properties.js
    // - renderPropertyField() → tafl-editor-properties.js
    // - setupPropertyEditing() → tafl-panels-properties.js
    // - updatePropertyValue() → tafl-editor-properties.js
    // - applyPropertyChanges() → tafl-editor-properties.js
    
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
        // Delegate to cards module
        this.cardsModule.duplicateCard(cardId);
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
        console.log('🗑️ TAFLEditor.deleteCard() called for:', cardId);
        
        if (!await taflModals.confirmDelete('card')) {
            return;
        }
        
        // Store the flow state before deletion for debugging
        const flowBefore = taflFlowStore.getFlow();
        console.log('🗑️ Flow before delete:', flowBefore.flow.length, 'cards');
        
        // Delegate to cards module which will update store and emit event
        this.cardsModule.deleteCard(cardId);
        
        // Force immediate refresh after delete
        // The event handler will also trigger, but this ensures immediate update
        setTimeout(() => {
            const flowAfter = taflFlowStore.getFlow();
            console.log('🗑️ Flow after delete:', flowAfter.flow.length, 'cards');
            
            // Force full canvas refresh
            this.canvasModule.forceUpdate();
            
            // Show drop zone if no cards left
            if (flowAfter.flow.length === 0) {
                const dropZone = document.getElementById('canvas-drop-zone');
                if (dropZone) {
                    dropZone.style.display = 'flex';
                }
            }
            
            // Update YAML view
            this.yamlModule.refreshYaml();
            
            // Update UI elements
            this.updateUI();
        }, 100);
        
        // Deselect if this was the selected card
        const selectedCardId = taflFlowStore.getSelectedCardId();
        if (selectedCardId === cardId) {
            this.deselectCard();
        }
        
        this.markDirty();
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
        // Delegate to Store
        taflFlowStore.addSwitchCase(cardId);
    }
    
    removeSwitchCase(cardId, caseIndex) {
        // Delegate to Store
        taflFlowStore.removeSwitchCase(cardId, caseIndex);
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
        // Delegate to Properties module
        taflEditorProperties.toggleDefault(cardId, add);
    }
    
    moveSwitchCase(cardId, caseIndex, direction) {
        // Delegate to Store
        const fromIndex = caseIndex;
        const toIndex = direction === 'up' ? caseIndex - 1 : caseIndex + 1;
        taflFlowStore.moveSwitchCase(cardId, fromIndex, toIndex);
    }
    
    // ===== Canvas Module Delegation =====
    // Linus says: "One function, one purpose. No special cases."
    
    // Legacy compatibility - all canvas operations delegate to canvas module
    updateCanvas() {
        // No special cases, no conditions, just delegate
        this.canvasModule.update();
    }
    
    refreshCanvas() {
        this.canvasModule.update();
    }
    
    refreshCanvasCore() {
        // For operations that need to force update even during drag
        this.canvasModule.forceUpdate();
    }
    
    // Simple drag state check (used by drag handlers)
    isDragging() {
        return this.draggedElement || 
               document.body.classList.contains('dragging');
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
            
            // Note: Control buttons for nested cards are handled by event delegation
            // No need to bind individual events - they're handled in setupEventListeners()
            
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
        // LINUS FIX: All drag-drop handling moved to tafl-editor-dragdrop.js
        // This function now only marks nested zones for identification
        // No special event handlers - unified handling only!
        
        document.querySelectorAll('.nested-drop-zone.empty-zone').forEach(dropZone => {
            // Just mark it, don't add special handlers
            dropZone.dataset.isNestedDropZone = 'true';
        });
        
        // Set up horizontal drop indicators
        document.querySelectorAll('.drop-indicator.horizontal').forEach(indicator => {
            // Prevent duplicate event registration
            if (indicator.dataset.eventsRegistered === 'true') {
                return;
            }
            indicator.dataset.eventsRegistered = 'true';
            
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
                        this.canvasModule.update();
                        
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
                            this.canvasModule.update();
                            
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
        
        // Update panel content when switching tabs
        if (tab === 'variables' && window.taflPanels) {
            // Update variables panel content
            window.taflPanels.updateVariablesPanel();
        } else if (tab === 'rules' && window.taflPanels) {
            // Update rules panel content
            window.taflPanels.updateRulesPanel();
        } else if (tab === 'preload' && window.taflPanels) {
            // Update preload panel content
            window.taflPanels.updatePreloadPanel();
        }
        
        // Refresh CodeMirror editors after updating content
        if (['preload', 'rules', 'variables'].includes(tab)) {
            // Use setTimeout to ensure DOM updates are complete
            setTimeout(() => {
                if (window.taflPanels && window.taflPanels.refreshCodeMirrors) {
                    window.taflPanels.refreshCodeMirrors();
                }
            }, 10);
        }
        
        // Refresh YAML if switching to YAML tab
        if (tab === 'yaml') {
            this.yamlModule.refreshYaml();
        }
    }
    
    // newFlow() moved to tafl-editor-modals.js
    
    // OpenFlowModal removed - using dropdown selector instead
    // The dropdown in the header now handles flow selection
    
    // Delegate to flow module
    async openFlow(flowId) {
        return this.flowModule.openFlow(flowId);
    }
    
    /**
     * Recursively ensure all cards have IDs (including nested cards)
     */
    // Delegate to flow module
    ensureAllCardsHaveIds(cards) {
        return this.flowModule.ensureAllCardsHaveIds(cards);
    }
    
    // Delegate to flow module
    async loadFlow(flowDataOrId) {
        return this.flowModule.loadFlow(flowDataOrId);
    }
    
    // saveFlowModal() and saveFlow() moved to tafl-editor-modals.js
    
    
    // Delegate to flow module for test run
    async testRun() {
        return this.flowModule.testRun();
    }
    
    // Delegate to flow module for real execution
    async executeFlow() {
        return this.flowModule.executeFlow();
    }
    
    
    // Note: rebuildCardsFromFlow has been removed - use updateCanvas instead
    // This function was redundant with refreshCanvas/refreshCanvasCore
    
    markDirty() {
        taflFlowStore.setDirty(true);
        this.notificationsModule.markDirty();
        this.notificationsModule.updateUI();
    }
    
    updateUI() {
        // Update metadata inputs
        document.getElementById('flow-id').value = taflFlowStore.getFlow().metadata.id || '';
        document.getElementById('flow-name').value = taflFlowStore.getFlow().metadata.name || '';
        document.getElementById('flow-version').value = taflFlowStore.getFlow().metadata.version || '1.0';
        document.getElementById('flow-description').value = taflFlowStore.getFlow().metadata.description || '';
        
        // Update enabled checkbox (default to true if not specified)
        const flowEnabled = document.getElementById('flow-enabled');
        if (flowEnabled) {
            flowEnabled.checked = taflFlowStore.getFlow().metadata.enabled !== false;
        }
        
        // Update save button state
        const saveBtn = document.getElementById('save-flow-btn');
        // Enable save button whenever there are changes (dirty state)
        // Allow saving metadata changes even without flow cards
        saveBtn.disabled = !taflFlowStore.isDirty();
        
        // Use notifications module for additional UI updates
        this.notificationsModule.updateUI();
        
        // Update title
        document.title = `TAFL Editor - ${taflFlowStore.getFlow().metadata.name}${taflFlowStore.isDirty() ? ' *' : ''}`;
    }
    
    showNotification(message, type = 'is-info') {
        // Delegate to notifications module
        this.notificationsModule.showNotification(message, type);
    }
    
    /**
     * 當流程改變時調用（供拖放系統使用）
     */
    onFlowChanged() {
        console.log('Flow structure changed');
        
        // 更新 store 中的流程資料
        if (this.store) {
            const flowData = this.exportFlowFromCanvas();
            this.store.updateFlow(flowData);
        }
        
        // 觸發任何需要的更新
        // 例如：更新 YAML 視圖、保存草稿等
    }
    
    /**
     * 撤銷 - Linus: Show me the code
     */
    undo() {
        if (taflFlowStore.undo()) {
            this.canvasModule.update();
            this.showNotification('Undo successful', 'is-info');
            this.updateUndoRedoButtons();
        }
    }
    
    /**
     * 重做
     */
    redo() {
        if (taflFlowStore.redo()) {
            this.canvasModule.update();
            this.showNotification('Redo successful', 'is-info');
            this.updateUndoRedoButtons();
        }
    }
    
    /**
     * 更新撤銷/重做按鈕狀態
     */
    updateUndoRedoButtons() {
        const undoBtn = document.getElementById('undo-btn');
        const redoBtn = document.getElementById('redo-btn');
        
        if (undoBtn) {
            undoBtn.disabled = !taflFlowStore.canUndo();
            undoBtn.title = taflFlowStore.canUndo() ? 'Undo (Ctrl+Z)' : 'Nothing to undo';
        }
        if (redoBtn) {
            redoBtn.disabled = !taflFlowStore.canRedo();
            redoBtn.title = taflFlowStore.canRedo() ? 'Redo (Ctrl+Y)' : 'Nothing to redo';
        }
    }
}

// Export the TAFLEditor class
export { TAFLEditor };

// Initialize TAFL Editor when page loads
document.addEventListener('DOMContentLoaded', async () => {
    // Don't clear localStorage on every page load - let the store manage persistence
    // localStorage.removeItem('taflEditor');
    
    const taflEditor = new TAFLEditor();
    
    // Use the singleton instance of Properties Panel UI (already initialized)
    // Don't create a new instance - the module already exports a singleton
    taflEditor.propertiesPanel = taflPanelsProperties;
    
    // Ensure the properties panel is properly initialized
    // The module might have tried to init before DOM was ready
    taflPanelsProperties.reinitialize();
    
    // Clear any stale selected card ID from previous sessions
    // This ensures we start with a clean selection state
    taflFlowStore.deselectCard();
    
    // Setup event delegation for flow dropdown items
    const dropdownContent = document.getElementById('flows-dropdown-content');
    if (dropdownContent) {
        dropdownContent.addEventListener('click', async function(e) {
            // Check if delete button was clicked
            const deleteBtn = e.target.closest('.flow-delete-btn');
            if (deleteBtn) {
                e.preventDefault();
                e.stopPropagation();
                
                const flowId = deleteBtn.dataset.flowId;
                const flowName = deleteBtn.dataset.flowName;
                
                // Show confirmation modal
                window.taflPanels.showDeleteFlowModal(flowId, flowName, async (confirmedFlowId) => {
                    try {
                        // Call API to delete flow
                        await window.taflAPI.deleteFlow(confirmedFlowId);
                        taflEditor.notificationsModule.success(`Flow "${flowName}" deleted successfully`);
                        
                        // Reload flows dropdown
                        await taflEditor.flowModule.loadFlowsDropdown();
                        
                        // If deleted flow was currently loaded, load first available flow
                        const currentFlow = taflFlowStore.getFlow();
                        if (currentFlow.metadata?.id === confirmedFlowId) {
                            await taflEditor.flowModule.loadFirstAvailableFlow();
                        }
                    } catch (error) {
                        console.error('Failed to delete flow:', error);
                        taflEditor.notificationsModule.error('Failed to delete flow: ' + error.message);
                    }
                });
                return;
            }
            
            // Handle regular flow item click
            const flowItem = e.target.closest('.flow-dropdown-item');
            if (flowItem && !deleteBtn) {
                const flowId = flowItem.dataset.flowId;
                if (flowId) {
                    try {
                        // Pass flowId to loadFlow so it can properly normalize IDs
                        await taflEditor.loadFlow(flowId);
                        
                        // Get the loaded flow data for the notification
                        const flowData = taflFlowStore.getFlow();
                        const flowName = flowData.metadata?.name || flowId;
                        taflEditor.notificationsModule.success(`Flow "${flowName}" loaded successfully`);
                        
                        // Save as last edited flow
                        localStorage.setItem('lastEditedFlowId', flowId);
                        // Close dropdown
                        const dropdown = flowItem.closest('.navbar-dropdown');
                        if (dropdown) {
                            dropdown.parentElement.classList.remove('is-active');
                        }
                    } catch (error) {
                        console.error('Failed to load flow:', error);
                        taflEditor.notificationsModule.error('Failed to load flow: ' + error.message);
                    }
                }
            }
        });
    }
    
    // Load flows dropdown using module method
    await taflEditor.flowModule.loadFlowsDropdown();
    
    // Make loadFlowsDropdown available globally for modals module
    window.loadFlowsDropdown = () => taflEditor.flowModule.loadFlowsDropdown();
    
    // Expose all modules globally so they can communicate
    window.taflEditorProperties = taflEditorProperties;
    window.taflPanelsProperties = taflPanelsProperties;
    window.taflEditorCards = taflEditorCards;
    window.taflFlowStore = taflFlowStore;
    
    // 不要自動載入 localStorage 中的 flow
    // 使用者需要手動選擇要載入的 flow
    console.log('TAFL Editor initialized. Please select a flow from the dropdown or create a new one.');
});

// Global functions have been moved to module methods:
// - loadFirstAvailableFlow -> taflEditor.flowModule.loadFirstAvailableFlow()
// - loadFlowsDropdown -> taflEditor.flowModule.loadFlowsDropdown()
// - loadFlow is handled inline in the event handler or via taflEditor.loadFlow()