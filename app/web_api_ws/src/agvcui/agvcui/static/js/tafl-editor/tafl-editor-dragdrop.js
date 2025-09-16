/**
 * TAFL Editor Drag & Drop Module
 * 
 * Linus Torvalds philosophy: "Good taste means removing special cases"
 * This module handles ALL drag-drop operations with ZERO special cases.
 */

import { taflFlowStore } from './tafl-editor-store.js';

class TAFLEditorDragDrop {
    constructor(editor) {
        this.editor = editor;
        
        // Drag state - simple and clear
        this.dragData = null;
        this.draggedElement = null;
        this.isDragging = false;
        
        // UI elements
        this.placeholder = null;
        this.dropIndicator = null;
        this.insertionLine = null;
        
        // Event counters for debugging
        this.dragEventCount = 0;
        this.dragoverCount = 0;
        
        // Auto-scroll properties
        this.autoScrollInterval = null;
        this.scrollContainer = null;
        this.scrollSpeed = 0;
        this.lastDragoverEvent = null;
    }

    /**
     * Initialize drag-drop system
     * Single entry point, no special cases
     */
    init() {
        console.log('🎯 Initializing DragDrop module');
        
        // Setup drag sources
        this.setupToolboxDrag();
        this.setupCanvasDrag();
        
        // Setup global event delegation
        this.setupGlobalHandlers();
        
        console.log('✅ DragDrop module initialized');
    }

    /**
     * Setup toolbox verb dragging
     */
    setupToolboxDrag() {
        const verbItems = document.querySelectorAll('.verb-item');
        
        verbItems.forEach(item => {
            item.draggable = true;
            
            item.addEventListener('dragstart', (e) => {
                const verb = item.dataset.verb;
                if (!verb) return;
                
                this.handleDragStart(e, {
                    type: 'verb',
                    verb: verb,
                    source: 'toolbox'
                });
                
                // Visual feedback
                item.classList.add('is-dragging');
            });
            
            item.addEventListener('dragend', (e) => {
                item.classList.remove('is-dragging');
                this.handleDragEnd(e);
            });
        });
    }

    /**
     * Setup canvas card dragging through event delegation
     */
    setupCanvasDrag() {
        const canvas = document.getElementById('flow-canvas');
        if (!canvas) return;
        
        // Use event delegation for dynamic elements
        canvas.addEventListener('dragstart', (e) => {
            const card = e.target.closest('.tafl-card');
            if (!card || !card.draggable) return;
            
            // Debug: Check if card has required data
            if (!card.dataset.cardId) {
                console.error('🚨 Card missing cardId:', card);
                return;
            }
            
            // LINUS FIX: Track if dragging from nested area
            const isFromNested = card.closest('.nested-cards, .nested-drop-zone') !== null;
            
            this.handleDragStart(e, {
                type: 'card',
                cardId: card.dataset.cardId,
                source: 'canvas',
                fromNested: isFromNested  // Track nested origin
            });
            
            // Store reference
            this.draggedElement = card;
            this.isFromNestedArea = isFromNested;  // Store for later checks
            card.classList.add('is-dragging');
            
            if (isFromNested) {
                console.log('🎯 Dragging from nested area - preventing immediate drop');
            }
        });
        
        canvas.addEventListener('dragend', (e) => {
            const card = e.target.closest('.tafl-card');
            if (!card) return;
            
            card.classList.remove('is-dragging');
            this.handleDragEnd(e);
        });
    }

    /**
     * Setup global drag-drop handlers
     */
    setupGlobalHandlers() {
        const canvas = document.getElementById('flow-canvas');
        if (!canvas) return;
        
        // Dragover - show drop indicators and handle auto-scroll
        canvas.addEventListener('dragover', (e) => {
            e.preventDefault(); // Allow drop
            
            if (!this.isDragging) return;
            
            // Store the event for use during scrolling
            this.lastDragoverEvent = e;
            
            // Update drop indicators
            this.updateDropIndicators(e);
            
            // Handle auto-scroll when near edges
            this.handleAutoScroll(e);
            
            // Throttle logging
            this.dragoverCount++;
            if (this.dragoverCount === 1 || this.dragoverCount % 50 === 0) {
                console.log(`🎯 Dragover event #${this.dragoverCount}`);
            }
        });
        
        // Dragleave - cleanup indicators
        canvas.addEventListener('dragleave', (e) => {
            // Only cleanup if leaving the canvas entirely
            if (!canvas.contains(e.relatedTarget)) {
                this.hideDropIndicators();
            }
        });
        
        // Drop - handle the drop
        canvas.addEventListener('drop', (e) => {
            e.preventDefault();
            e.stopPropagation();
            
            if (!this.isDragging) return;
            
            this.handleDrop(e);
        });
    }

    /**
     * Handle drag start
     * Single entry point for all drag operations
     */
    handleDragStart(e, dragData) {
        console.log('🚀 Drag started:', dragData);
        
        // Set drag state
        this.isDragging = true;
        this.dragData = dragData;
        this.dragEventCount = 0;
        this.dragoverCount = 0;
        
        // Store in dataTransfer for cross-browser support
        try {
            e.dataTransfer.effectAllowed = 'move';
            e.dataTransfer.setData('application/json', JSON.stringify(dragData));
        } catch (err) {
            console.warn('Could not set drag data:', err);
        }
        
        // Set drag image if element exists
        if (e.dataTransfer.setDragImage && this.draggedElement) {
            const rect = this.draggedElement.getBoundingClientRect();
            e.dataTransfer.setDragImage(
                this.draggedElement,
                e.clientX - rect.left,
                e.clientY - rect.top
            );
        }
        
        // Show drop zones
        this.showDropZones();
        
        // Notify editor
        if (this.editor.canvasModule) {
            this.editor.canvasModule.startDrag();
        }
    }

    /**
     * Handle drag end
     * Single exit point for all drag operations
     */
    handleDragEnd(e) {
        console.log('🏁 Drag ended');
        
        // Stop auto-scroll
        this.stopAutoScroll();
        
        // Clear drag state
        this.isDragging = false;
        this.dragData = null;
        this.draggedElement = null;
        this.dragEventCount = 0;
        this.dragoverCount = 0;
        this.isFromNestedArea = false;  // LINUS FIX: Clear nested tracking
        this.lastDragoverEvent = null;
        
        // Hide drop zones and indicators
        this.hideDropZones();
        this.hideDropIndicators();
        
        // Clear any placeholders
        if (this.placeholder) {
            this.placeholder.remove();
            this.placeholder = null;
        }
        
        // Notify editor
        if (this.editor.canvasModule) {
            this.editor.canvasModule.endDrag();
        }
    }

    /**
     * Handle drop
     * Process the actual drop operation
     */
    handleDrop(e) {
        console.log('💧 Drop event at:', e.clientX, e.clientY);
        
        try {
            // Get drop position
            const position = this.calculateDropPosition(e);
            if (!position) {
                console.warn('No valid drop position');
                return;
            }
            
            console.log('📍 Drop position:', position);
            
            // Get drag data
            const dragData = this.getDragData(e);
            if (!dragData) {
                console.warn('No drag data available');
                return;
            }
            
            // Process drop based on type
            if (dragData.type === 'verb') {
                this.handleVerbDrop(dragData, position);
            } else if (dragData.type === 'card') {
                this.handleCardDrop(dragData, position);
            }
        } catch (error) {
            console.error('❌ Error during drop:', error);
            // 確保即使發生錯誤也清理拖曳狀態
        } finally {
            // Cleanup - 總是執行清理
            this.hideDropIndicators();
            this.hideDropZones();
            
            // 強制重置所有拖放狀態
            this.isDragging = false;
            this.dragData = null;
            this.draggedElement = null;
            this.dragEventCount = 0;
            this.dragoverCount = 0;
            this.isFromNestedArea = false;
            this.lastDragoverEvent = null;
            
            // Clear any placeholders
            if (this.placeholder) {
                this.placeholder.remove();
                this.placeholder = null;
            }
            
            // 重新綁定拖放處理器以恢復功能
            // 直接調用，不使用延遲 - 事件委託確保功能正常
            this.refreshDragHandlers();
            // Also re-setup nested drop targets
            if (this.editor.setupNestedDropTargets) {
                this.editor.setupNestedDropTargets();
            }
        }
    }

    /**
     * Handle verb drop from toolbox
     */
    handleVerbDrop(dragData, position) {
        const cardId = this.editor.generateId();
        const cardData = {
            id: cardId,
            [dragData.verb]: this.editor.utilsModule.getDefaultParams(dragData.verb)
        };
        
        if (position.nested) {
            // Add to nested structure with proper index
            console.log('🟢 handleVerbDrop - position.nested:', position.nested);
            console.log('🟢 handleVerbDrop - position.index:', position.index);
            console.log('🟢 handleVerbDrop - position.index type:', typeof position.index);
            this.editor.addToNestedStructure(
                position.parentCardId,
                position.branchType,
                cardData,
                position.index  // Pass the index for correct positioning
            );
        } else {
            // Add to main flow - use addCardAtPosition with proper format
            this.editor.cardsModule.addCardAtPosition(dragData.verb, {
                index: position.index,
                nested: null
            });
        }
        
        // Update and notify
        this.editor.canvasModule.update();
        this.editor.markDirty();
        this.editor.showNotification(`Added ${dragData.verb} card`, 'is-success');
        
        // Linus principle: ensure ALL new cards are immediately draggable
        // Single RAF is sufficient - DOM will be ready by next frame
        requestAnimationFrame(() => {
            this.refreshDragHandlers();
            // Also re-setup nested drop targets
            if (this.editor.setupNestedDropTargets) {
                this.editor.setupNestedDropTargets();
            }
        });
    }

    /**
     * Handle card move
     */
    handleCardDrop(dragData, position) {
        const cardData = this.editor.findCardById(dragData.cardId);
        if (!cardData) return;
        
        if (position.nested) {
            // Move to nested structure with proper index
            console.log('🟢 handleCardDrop - position.nested:', position.nested);
            console.log('🟢 handleCardDrop - position.index:', position.index);
            console.log('🟢 handleCardDrop - position.index type:', typeof position.index);
            
            // Wrap removeCardFromFlow with error handling
            try {
                this.editor.removeCardFromFlow(dragData.cardId);
            } catch (error) {
                console.error('⚠️ Failed to remove card from flow:', error);
                // Continue anyway - the card might not be in the flow yet
            }
            
            this.editor.addToNestedStructure(
                position.parentCardId,
                position.branchType,
                cardData,
                position.index  // Pass the index for correct positioning
            );
            // Handlers will be refreshed at the end of the function
        } else {
            // Move to main flow (from anywhere, including nested)
            // First remove from current location (handles both nested and main flow)
            try {
                this.editor.removeCardFromFlow(dragData.cardId);
            } catch (error) {
                console.error('⚠️ Failed to remove card from flow:', error);
                // Continue anyway - the card might not be in the flow yet
            }
            
            // Then add to main flow at the specified position
            const flow = taflFlowStore.getFlow();
            flow.flow.splice(position.index, 0, cardData);
            // Use updateFlow instead of loadFlow to avoid complete state replacement
            taflFlowStore.updateFlow({ flow: flow.flow });
            // Handlers will be refreshed at the end of the function
        }
        
        // Update and notify
        this.editor.canvasModule.update();
        this.editor.markDirty();
        this.editor.showNotification('Card moved', 'is-success');
        
        // Ensure cards remain draggable after move
        // Single RAF is sufficient - DOM will be ready by next frame
        requestAnimationFrame(() => {
            this.refreshDragHandlers();
            // Also re-setup nested drop targets
            if (this.editor.setupNestedDropTargets) {
                this.editor.setupNestedDropTargets();
            }
        });
    }

    /**
     * Calculate drop position
     * Returns unified position object
     */
    calculateDropPosition(e) {
        // Check if we're over the main drop zone first
        const mainDropZone = document.getElementById('canvas-drop-zone');
        
        // Check if we're over ANY nested area (cards or empty zone)
        const nestedArea = e.target.closest('.nested-cards, .nested-drop-zone');
        if (nestedArea) {
            const parentCard = nestedArea.closest('.tafl-card');
            const branch = nestedArea.dataset.branch;
            
            if (parentCard && branch) {
                // LINUS FIX: Prevent immediate drop in same nested area when dragging
                if (this.isFromNestedArea && this.draggedElement) {
                    const draggedParent = this.draggedElement.closest('.tafl-card[data-card-id]');
                    if (draggedParent && draggedParent.dataset.cardId === parentCard.dataset.cardId) {
                        // Allow reordering within same nested area, but calculate proper index
                        const calculatedIndex = this.calculateNestedIndex(nestedArea, e);
                        console.log('📍 Reordering within same nested area', branch, 'at index', calculatedIndex);
                        return {
                            nested: true,
                            parentCardId: parentCard.dataset.cardId,
                            branchType: branch,
                            index: calculatedIndex
                        };
                    }
                }
                
                const calculatedIndex = this.calculateNestedIndex(nestedArea, e);
                console.log('📍 Drop position: nested area', branch, 'in card', parentCard.dataset.cardId, 'at index', calculatedIndex);
                return {
                    nested: true,
                    parentCardId: parentCard.dataset.cardId,
                    branchType: branch,
                    index: calculatedIndex
                };
            }
        }
        
        // Default to main drop zone
        if (!mainDropZone) return null;
        
        const cards = Array.from(mainDropZone.querySelectorAll(':scope > .tafl-card'));
        const index = this.calculateIndexFromPosition(e, cards);
        
        console.log('📍 Drop position: main canvas at index', index);
        return {
            nested: false,
            index: index
        };
    }

    /**
     * Calculate index for nested drop
     */
    calculateNestedIndex(nestedZone, e) {
        // For empty zones, always return 0
        if (nestedZone.classList.contains('empty-zone')) {
            console.log('📊 Empty nested zone, returning index 0');
            return 0;
        }
        
        // For nested-cards container, find cards within it
        const cards = Array.from(nestedZone.querySelectorAll(':scope > .tafl-card'));
        console.log('📊 Found', cards.length, 'cards in nested zone');
        
        const calculatedIndex = this.calculateIndexFromPosition(e, cards);
        console.log('📊 Calculated nested index:', calculatedIndex);
        return calculatedIndex;
    }

    /**
     * Calculate index from mouse position
     */
    calculateIndexFromPosition(e, cards) {
        if (cards.length === 0) {
            console.log('📊 No cards, returning index 0');
            return 0;
        }
        
        const mouseY = e.clientY;
        console.log('📊 Mouse Y position:', mouseY);
        
        // Filter out dragging card and build index map
        const visibleCards = [];
        let adjustedIndex = 0;
        
        for (let i = 0; i < cards.length; i++) {
            const card = cards[i];
            // Skip cards being dragged
            if (card.classList.contains('is-dragging')) {
                console.log(`📊 Skipping dragging card at original index ${i}`);
                continue;
            }
            visibleCards.push({ card, originalIndex: i });
        }
        
        // If only dragging card exists, return 0
        if (visibleCards.length === 0) {
            console.log('📊 Only dragging card exists, returning index 0');
            return 0;
        }
        
        // Check position against visible cards
        for (let i = 0; i < visibleCards.length; i++) {
            const { card, originalIndex } = visibleCards[i];
            const rect = card.getBoundingClientRect();
            const cardMiddle = rect.top + rect.height / 2;
            console.log(`📊 Visible card ${i} (original ${originalIndex}): top=${rect.top}, middle=${cardMiddle}, mouseY=${mouseY}`);
            
            if (mouseY < cardMiddle) {
                console.log(`📊 Mouse is above visible card ${i} middle, returning index ${originalIndex}`);
                return originalIndex;
            }
        }
        
        console.log(`📊 Mouse is below all visible cards, returning index ${cards.length}`);
        return cards.length;
    }

    /**
     * Update drop indicators based on drag position
     */
    updateDropIndicators(e) {
        // Remove existing indicators
        this.hideDropIndicators();
        
        // Add drag-over class to nested zones
        const nestedZone = e.target.closest('.nested-drop-zone');
        if (nestedZone) {
            nestedZone.classList.add('drag-over');
        }
        
        // Get target position
        const position = this.calculateDropPosition(e);
        if (!position) return;
        
        // Create and show insertion line
        this.showInsertionLine(position);
    }

    /**
     * Show insertion line at position
     */
    showInsertionLine(position) {
        // Create line if doesn't exist - match CSS structure
        if (!this.insertionLine) {
            this.insertionLine = document.createElement('div');
            this.insertionLine.className = 'insertion-line active';  // Add active class for visibility
            
            // Create inner structure matching CSS
            const lineInner = document.createElement('div');
            lineInner.className = 'insertion-line-inner';
            
            const lineDot = document.createElement('div');
            lineDot.className = 'insertion-dot';
            
            const lineText = document.createElement('div');
            lineText.className = 'insertion-text';
            lineText.textContent = 'Drop here';
            
            lineInner.appendChild(lineDot);
            lineInner.appendChild(lineText);
            this.insertionLine.appendChild(lineInner);
        }
        
        // Position the line
        if (position.nested) {
            // Fix: look for both .nested-cards and .nested-drop-zone
            const parentCard = document.querySelector(`.tafl-card[data-card-id="${position.parentCardId}"]`);
            if (parentCard) {
                // Look for both .nested-cards (with existing cards) and .nested-drop-zone (empty)
                const nestedZone = parentCard.querySelector(
                    `[data-branch="${position.branchType}"].nested-cards, [data-branch="${position.branchType}"].nested-drop-zone`
                );
                if (nestedZone) {
                    this.positionLineInContainer(nestedZone, position.index);
                }
            }
        } else {
            const dropZone = document.getElementById('canvas-drop-zone');
            if (dropZone) {
                this.positionLineInContainer(dropZone, position.index);
            }
        }
    }

    /**
     * Position insertion line in container
     */
    positionLineInContainer(container, index) {
        const cards = Array.from(container.querySelectorAll(':scope > .tafl-card'));
        
        // Ensure container has relative positioning for absolute children
        if (getComputedStyle(container).position === 'static') {
            container.style.position = 'relative';
        }
        
        // Add line to container if not already there
        if (this.insertionLine.parentElement !== container) {
            container.appendChild(this.insertionLine);
        }
        
        // Ensure active class is always present
        if (!this.insertionLine.classList.contains('active')) {
            this.insertionLine.classList.add('active');
        }
        
        // Apply positioning styles
        this.insertionLine.style.position = 'absolute';
        this.insertionLine.style.left = '0';
        this.insertionLine.style.right = '0';
        this.insertionLine.style.zIndex = '1000';
        
        if (cards.length === 0 || index === 0) {
            // Position at top
            this.insertionLine.style.top = '0px';
        } else if (index >= cards.length) {
            // Position at bottom
            const lastCard = cards[cards.length - 1];
            const rect = lastCard.getBoundingClientRect();
            const containerRect = container.getBoundingClientRect();
            this.insertionLine.style.top = `${rect.bottom - containerRect.top}px`;
        } else {
            // Position between cards - center the insertion line in the gap
            const prevCard = cards[index - 1];
            const nextCard = cards[index];
            const prevRect = prevCard.getBoundingClientRect();
            const nextRect = nextCard.getBoundingClientRect();
            const containerRect = container.getBoundingClientRect();
            // Center the 60px insertion line container between cards
            // Subtract 30px (half of container height) to center it properly
            const middleY = prevRect.bottom - containerRect.top - 30;
            this.insertionLine.style.top = `${middleY}px`;
        }
    }

    /**
     * Hide drop indicators
     */
    hideDropIndicators() {
        // Remove insertion line
        if (this.insertionLine) {
            this.insertionLine.remove();
            this.insertionLine = null;
        }
        
        // Remove drag-over class from all nested zones
        document.querySelectorAll('.nested-drop-zone.drag-over').forEach(zone => {
            zone.classList.remove('drag-over');
        });
        
        // Remove drag-over class from all drop zones
        document.querySelectorAll('.drop-zone.drag-over').forEach(zone => {
            zone.classList.remove('drag-over');
        });
    }

    /**
     * Show drop zones
     */
    showDropZones() {
        const canvas = document.getElementById('flow-canvas');
        if (!canvas) return;
        
        // Show all drop zones
        canvas.querySelectorAll('.drop-zone, .nested-drop-zone').forEach(zone => {
            zone.classList.add('drop-active');
        });
    }

    /**
     * Hide drop zones
     */
    hideDropZones() {
        const canvas = document.getElementById('flow-canvas');
        if (!canvas) return;
        
        // Hide all drop zones
        canvas.querySelectorAll('.drop-zone, .nested-drop-zone').forEach(zone => {
            zone.classList.remove('drop-active', 'drag-over');
        });
    }

    /**
     * Get drag data from event
     */
    getDragData(e) {
        // First try our stored data
        if (this.dragData) {
            return this.dragData;
        }
        
        // Try dataTransfer
        try {
            const jsonData = e.dataTransfer.getData('application/json');
            if (jsonData) {
                return JSON.parse(jsonData);
            }
        } catch (err) {
            console.warn('Could not get drag data from dataTransfer:', err);
        }
        
        return null;
    }

    /**
     * Refresh drag handlers after DOM updates
     * Called after canvas rebuild
     */
    refreshDragHandlers() {
        // Prevent multiple queued refreshes
        if (this.refreshPending) {
            return;
        }
        this.refreshPending = true;
        
        // Linus principle: eliminate special cases - ALL cards must be draggable
        // Use requestAnimationFrame to ensure DOM is fully ready
        requestAnimationFrame(() => {
            // Clear the pending flag
            this.refreshPending = false;
            
            const allCards = document.querySelectorAll('.tafl-card');
            allCards.forEach(card => {
                // Only set draggable if not already set
                if (!card.draggable) {
                    card.draggable = true;
                }
                // Also ensure the card has proper dataset attributes
                if (!card.dataset.cardId && card.id) {
                    card.dataset.cardId = card.id;
                }
            });
            
            // Count nested vs root cards for debugging
            const nestedCards = document.querySelectorAll('.nested-cards .tafl-card, .nested-drop-zone .tafl-card');
            const rootCards = document.querySelectorAll('#canvas-drop-zone > .tafl-card');
            
            console.log(`🔄 Drag handlers refreshed: ${allCards.length} total cards (${rootCards.length} root, ${nestedCards.length} nested)`);
        });
    }

    /**
     * Handle auto-scroll when dragging near edges
     */
    handleAutoScroll(e) {
        // Find the scrollable container
        const container = document.querySelector('.tafl-canvas-container');
        if (!container) return;
        
        const rect = container.getBoundingClientRect();
        const threshold = 50; // Distance from edge to trigger scroll
        const maxSpeed = 15;  // Maximum scroll speed
        
        // Check if near top edge
        if (e.clientY < rect.top + threshold) {
            const distance = rect.top + threshold - e.clientY;
            const speed = -Math.min(maxSpeed, distance / 3);
            this.startAutoScroll(container, speed);
        }
        // Check if near bottom edge
        else if (e.clientY > rect.bottom - threshold) {
            const distance = e.clientY - (rect.bottom - threshold);
            const speed = Math.min(maxSpeed, distance / 3);
            this.startAutoScroll(container, speed);
        }
        // Stop scrolling if not near edges
        else {
            this.stopAutoScroll();
        }
    }

    /**
     * Start auto-scrolling
     */
    startAutoScroll(container, speed) {
        // Don't restart if already scrolling at same speed
        if (this.autoScrollInterval && this.scrollSpeed === speed) return;
        
        // Stop existing scroll if any
        this.stopAutoScroll();
        
        this.scrollContainer = container;
        this.scrollSpeed = speed;
        
        // Start scrolling interval
        this.autoScrollInterval = setInterval(() => {
            if (this.scrollContainer && this.isDragging) {
                this.scrollContainer.scrollTop += this.scrollSpeed;
                
                // Update drop indicators while scrolling
                // This ensures the insertion line follows the scroll
                const lastEvent = this.lastDragoverEvent;
                if (lastEvent) {
                    this.updateDropIndicators(lastEvent);
                }
            }
        }, 16); // ~60fps
    }

    /**
     * Stop auto-scrolling
     */
    stopAutoScroll() {
        if (this.autoScrollInterval) {
            clearInterval(this.autoScrollInterval);
            this.autoScrollInterval = null;
            this.scrollContainer = null;
            this.scrollSpeed = 0;
        }
    }

    /**
     * Cleanup and destroy
     */
    destroy() {
        this.stopAutoScroll();
        this.hideDropZones();
        this.hideDropIndicators();
        this.dragData = null;
        this.draggedElement = null;
        this.isDragging = false;
        this.lastDragoverEvent = null;
    }
}

// ES6 Module export
export default TAFLEditorDragDrop;
export { TAFLEditorDragDrop };