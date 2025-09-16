/**
 * TAFL Editor Canvas Module
 * 
 * Linus Torvalds philosophy: "Good taste means removing special cases"
 * This module manages canvas updates with ZERO special cases.
 */

class TAFLEditorCanvas {
    constructor(editor) {
        this.editor = editor;
        this.isDragging = false;
        this.dropZoneId = 'canvas-drop-zone';
    }

    /**
     * The ONLY update function you need
     * No special cases, no complex conditions, just update
     */
    update() {
        console.log('🔍 DEBUG-C1: Canvas.update() called');
        // Simple rule: don't update during drag
        if (this.isDragging) {
            console.log('🚫 Canvas update blocked during drag');
            return;
        }

        const canvas = document.getElementById('flow-canvas');
        console.log('🔍 DEBUG-C2: Canvas element found:', !!canvas);
        if (!canvas) {
            console.error('❌ DEBUG-ERROR: Canvas element not found!');
            return;
        }

        console.log('🔍 DEBUG-C3: Calling rebuild()');
        this.rebuild(canvas);
        console.log('🔍 DEBUG-C4: rebuild() completed');
    }

    /**
     * Core rebuild logic - clean and simple
     */
    rebuild(canvas) {
        console.log('🔍 DEBUG-C5: rebuild() started');
        const flow = window.taflFlowStore.getFlow();
        console.log('🔍 DEBUG-C6: Flow data:', flow);
        console.log('🔍 DEBUG-C7: Flow cards count:', flow?.flow?.length || 0);
        
        const dropZone = this.getOrCreateDropZone(canvas);
        console.log('🔍 DEBUG-C8: Drop zone element:', !!dropZone);
        
        // Clear everything
        this.clearDropZone(dropZone);
        console.log('🔍 DEBUG-C9: Drop zone cleared');
        
        // Rebuild from data
        if (flow?.flow?.length > 0) {
            console.log(`🔍 DEBUG-C10: Rendering ${flow.flow.length} cards`);
            
            // Hide the static drop-zone-content from HTML template
            const dropZoneContent = dropZone.querySelector('.drop-zone-content');
            if (dropZoneContent) {
                dropZoneContent.style.display = 'none';
            }
            
            flow.flow.forEach((card, index) => {
                console.log(`🔍 DEBUG-C11: Creating card element ${index}:`, card);
                const element = this.createCardElement(card, index);
                if (element) {
                    console.log(`🔍 DEBUG-C12: Appending card ${index} to drop zone`);
                    dropZone.appendChild(element);
                } else {
                    console.error(`❌ DEBUG-ERROR: Failed to create card element ${index}`);
                }
            });
            // Add has-cards class for proper CSS styling
            dropZone.classList.add('has-cards');
        } else {
            console.log('🔍 DEBUG-C13: No cards to render, showing empty message');
            
            // Show the static drop-zone-content from HTML template
            const dropZoneContent = dropZone.querySelector('.drop-zone-content');
            if (dropZoneContent) {
                dropZoneContent.style.display = 'block';
            }
            
            // Remove has-cards class when no cards
            dropZone.classList.remove('has-cards');
            // Removed duplicate showEmptyMessage() - using HTML template message only
        }

        // Re-setup drag handlers after rebuild
        console.log('🔍 DEBUG-C14: Setting up drag handlers');
        this.setupDragHandlers(dropZone);
        
        // Refresh drag-drop module handlers for new elements
        if (this.editor && this.editor.dragDropModule) {
            console.log('🔍 DEBUG-C15: Refreshing drag-drop handlers');
            this.editor.dragDropModule.refreshDragHandlers();
        }
        
        // Re-attach event handlers to newly created nested elements
        // This is critical - new DOM elements don't have event handlers!
        if (this.editor && this.editor.setupNestedDropTargets) {
            console.log('🔍 DEBUG-C16: Re-attaching nested drop target handlers');
            this.editor.setupNestedDropTargets();
        }
    }

    /**
     * Get or create the drop zone
     */
    getOrCreateDropZone(canvas) {
        let dropZone = document.getElementById(this.dropZoneId);
        
        if (!dropZone) {
            dropZone = document.createElement('div');
            dropZone.id = this.dropZoneId;
            dropZone.className = 'canvas-drop-zone';
            
            // Find or create cards container
            let cardsContainer = canvas.querySelector('.cards-container');
            if (!cardsContainer) {
                cardsContainer = document.createElement('div');
                cardsContainer.className = 'cards-container';
                canvas.appendChild(cardsContainer);
            }
            
            cardsContainer.appendChild(dropZone);
        }
        
        return dropZone;
    }

    /**
     * Clear the drop zone
     */
    clearDropZone(dropZone) {
        // Remove all cards but preserve the drop zone itself
        const cards = dropZone.querySelectorAll('.tafl-card');
        cards.forEach(card => card.remove());
        
        // Remove any placeholders
        const placeholders = dropZone.querySelectorAll('.tafl-card-placeholder');
        placeholders.forEach(placeholder => placeholder.remove());
        
        // Remove any drop indicators
        const indicators = dropZone.querySelectorAll('.drop-indicator');
        indicators.forEach(indicator => indicator.remove());
        
        // Note: We don't remove .drop-zone-content here as it's handled in rebuild()
    }

    /**
     * Create a card element
     */
    createCardElement(cardData, index) {
        console.log(`🔍 DEBUG-C15: createCardElement called for index ${index}:`, cardData);
        console.log(`🔍 DEBUG-C16: Editor reference:`, this.editor);
        console.log(`🔍 DEBUG-C17: createCardElementSimple exists:`, !!this.editor?.createCardElementSimple);
        
        // Delegate to editor's existing card creation
        // This keeps rendering logic in one place
        if (this.editor && this.editor.createCardElementSimple) {
            console.log(`🔍 DEBUG-C18: Using editor.createCardElementSimple`);
            const element = this.editor.createCardElementSimple(cardData);
            if (element) {
                element.style.order = index;
                element.dataset.index = index;
                console.log(`🔍 DEBUG-C19: Card element created via editor method`);
            } else {
                console.error(`❌ DEBUG-ERROR: createCardElementSimple returned null/undefined`);
            }
            return element;
        }
        
        console.log(`🔍 DEBUG-C20: Using fallback card creation`);
        // Fallback: create basic card
        const card = document.createElement('div');
        card.className = 'tafl-card';
        card.dataset.cardId = cardData.id || `card-${index}`;
        card.dataset.index = index;
        card.style.order = index;
        
        // Get verb from card data
        const verb = Object.keys(cardData).find(key => key !== 'id' && key !== 'comment');
        console.log(`🔍 DEBUG-C21: Card verb detected:`, verb);
        if (verb) {
            card.dataset.verb = verb;
            card.innerHTML = `
                <div class="tafl-card-header">
                    <span class="tafl-card-title">${verb}</span>
                </div>
                <div class="tafl-card-body">
                    ${JSON.stringify(cardData[verb] || {}, null, 2)}
                </div>
            `;
        }
        
        console.log(`🔍 DEBUG-C22: Fallback card element created`);
        return card;
    }

    /**
     * Setup drag handlers after rebuild
     */
    setupDragHandlers(dropZone) {
        // Prevent multiple queued setups
        if (this.setupPending) {
            return;
        }
        this.setupPending = true;
        
        // Linus principle: No special cases - ALL cards should be draggable
        // Use requestAnimationFrame to ensure DOM is ready
        requestAnimationFrame(() => {
            // Clear the pending flag
            this.setupPending = false;
            
            const allCards = document.querySelectorAll('.tafl-card');
            allCards.forEach(card => {
                // Only set if not already set
                if (!card.draggable) {
                    card.draggable = true;
                }
                // Ensure card has proper ID in dataset
                if (!card.dataset.cardId && card.id) {
                    card.dataset.cardId = card.id;
                }
            });
            console.log(`🔧 Set draggable=true for ${allCards.length} cards (including nested)`);
        });
    }


    /**
     * Drag state management - simple and direct
     */
    startDrag() {
        this.isDragging = true;
        document.body.classList.add('dragging');
    }

    endDrag() {
        this.isDragging = false;
        document.body.classList.remove('dragging');
        // Always update after drag ends
        this.update();
    }

    /**
     * Force update (ignores drag state)
     */
    forceUpdate() {
        const wasDragging = this.isDragging;
        this.isDragging = false;
        this.update();
        this.isDragging = wasDragging;
    }

    /**
     * Check if currently dragging
     */
    isCurrentlyDragging() {
        return this.isDragging || document.body.classList.contains('dragging');
    }
}

// ES6 Module exports - class only, no singleton (Linus style: one way to do things)
export default TAFLEditorCanvas;
export { TAFLEditorCanvas };