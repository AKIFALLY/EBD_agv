/**
 * TAFL Editor Card Operations Module
 * Handles all card-related operations including creation, deletion, duplication, and selection
 */

import { taflFlowStore } from './tafl-editor-store.js';
import taflNotifications from './tafl-editor-notifications.js';
import taflEventBus from './tafl-event-bus.js';

class TAFLEditorCards {
    constructor() {
        this.verbDefinitions = null;
        this.editor = null;
        this._isAddingCard = false;  // Flag to prevent selection during card addition
        this._preventAutoSelect = false;  // Flag to disable auto-selection
        
        // Bind Store events for UI updates
        this.bindStoreEvents();
    }

    /**
     * Initialize card operations
     */
    init(editor, verbDefinitions) {
        this.editor = editor;
        this.verbDefinitions = verbDefinitions;
    }
    
    /**
     * Subscribe to Store events for UI updates
     */
    bindStoreEvents() {
        // Respond to selection changes to update UI
        taflFlowStore.on('card:selected', (cardId) => {
            // Clear all selection styles
            document.querySelectorAll('.tafl-card').forEach(card => {
                card.classList.remove('selected');
            });
            
            // Add selection style to selected card
            const selectedCard = document.querySelector(`[data-card-id="${cardId}"]`);
            if (selectedCard) {
                selectedCard.classList.add('selected');
                // Scroll into view if needed
                this.scrollCardIntoView(selectedCard);
            }
        });
        
        taflFlowStore.on('card:deselected', () => {
            document.querySelectorAll('.tafl-card').forEach(card => {
                card.classList.remove('selected');
            });
        });
    }

    /**
     * Generate unique ID for cards
     * Delegates to utils for consistent ID generation
     */
    generateId() {
        // Use utils.generateId() which properly delegates to store
        return this.utils.generateId();
    }

    /**
     * Add a new card to the flow
     */
    addCard(verb) {
        // Always add to main canvas at the end
        return this.addCardAtPosition(verb, { 
            index: taflFlowStore.getFlow().flow.length, 
            nested: null 
        });
    }

    /**
     * Add card at specific position
     */
    addCardAtPosition(verb, position) {
        // More flexible verb checking with better logging
        if (!this.verbDefinitions) {
            this.verbDefinitions = taflFlowStore.getVerbDefinitions() || {};
        }
        
        if (!this.verbDefinitions[verb]) {
            // Don't return - continue with default params
        }

        // Set flag to prevent auto-selection that could cause infinite loops
        this._isAddingCard = true;
        
        let cardId;
        
        // Insert at specific position  
        if (position.nested) {
            // For nested cards, we still need to generate our own ID
            cardId = this.generateId();
            
            // Create TAFL v1.1 compliant card data
            const cardData = {
                id: cardId,
                [verb]: this.editor.getDefaultParams(verb)
            };
            
            // Add to nested structure with proper index
            this.addToNestedStructure(position.nested.parentCardId, position.nested.branchType, cardData, position.nested.index || 0);
        } else {
            // Use the store's addCard method which properly emits 'card:added' event
            // The store returns the generated cardId
            const params = this.editor.getDefaultParams(verb);
            cardId = taflFlowStore.addCard(verb, position, params);
        }
        
        // Select and animate the new card (only if not prevented)
        setTimeout(() => {
            // Clear the flag after event processing
            this._isAddingCard = false;
            
            const newCardElement = document.querySelector(`[data-card-id="${cardId}"]`);
            if (newCardElement) {
                newCardElement.classList.add('just-added');
                setTimeout(() => {
                    newCardElement.classList.remove('just-added');
                    newCardElement.classList.add('pulse-once');
                }, 100);
                setTimeout(() => newCardElement.classList.remove('pulse-once'), 600);
                
                // Only auto-select if not prevented (to avoid infinite loops)
                if (!this._preventAutoSelect) {
                    // Add a small delay to ensure setState from addCard has completed
                    setTimeout(() => {
                        this.selectCard(newCardElement);
                    }, 150);
                }
            }
        }, 100);
        
        this.editor.markDirty();
        this.editor.updateUI();
        
        // Ensure new cards are draggable
        setTimeout(() => {
            this.editor.refreshCardDragHandlers();
        }, 100);
        
        return cardId;
    }

    /**
     * Move card to new position
     */
    moveCardToPosition(cardId, position) {
        
        // Check if card was nested
        const cardEl = document.querySelector(`[data-card-id="${cardId}"]`);
        const wasNested = cardEl && cardEl.closest('.nested-cards') !== null;
        
        // Clear drag state before moving
        this.editor.draggedElement = null;
        this.editor.dragData = null;
        document.body.classList.remove('dragging');
        
        // Use store to move card
        taflFlowStore.moveCard(cardId, position);
        
        // Visual feedback
        setTimeout(() => {
            const movedCard = document.querySelector(`[data-card-id="${cardId}"]`);
            if (movedCard) {
                movedCard.classList.add('just-moved');
                setTimeout(() => movedCard.classList.remove('just-moved'), 500);
                this.selectCard(movedCard);
            }
        }, 100);
        
        this.editor.markDirty();
        taflNotifications.success('Card moved successfully');
    }

    /**
     * Delete a card
     */
    deleteCard(cardId) {
        
        // Use the store's deleteCard method which emits the proper event
        taflFlowStore.deleteCard(cardId);
        
        // Clear selection if deleted card was selected
        if (taflFlowStore.getSelectedCardId() === cardId) {
            taflFlowStore.deselectCard();
        }
        
        this.editor.markDirty();
        this.editor.updateUI();
        
        taflNotifications.success('Card deleted');
    }

    /**
     * Duplicate a card
     */
    duplicateCard(cardId) {
        
        const card = this.findCardInFlow(cardId);
        if (!card) {
            return;
        }
        
        // Create a deep copy with new ID
        const newCard = JSON.parse(JSON.stringify(card));
        newCard.id = this.generateId();
        
        // Find position of original card
        const position = this.findCardPosition(cardId);
        
        if (position.nested) {
            // Insert after original in nested structure
            this.insertCardAfterInNested(cardId, newCard);
        } else {
            // Insert after original in main flow
            const flow = taflFlowStore.getFlow();
            const newFlow = [...flow.flow];
            newFlow.splice(position.index + 1, 0, newCard);
            taflFlowStore.updateFlow({ ...flow, flow: newFlow });
        }
        
        // Highlight the new card
        setTimeout(() => {
            const newCardElement = document.querySelector(`[data-card-id="${newCard.id}"]`);
            if (newCardElement) {
                newCardElement.classList.add('just-added');
                setTimeout(() => {
                    newCardElement.classList.remove('just-added');
                    newCardElement.classList.add('pulse-once');
                }, 100);
                setTimeout(() => newCardElement.classList.remove('pulse-once'), 600);
                
                // Add a small delay to ensure setState from addCard has completed
                setTimeout(() => {
                    this.selectCard(newCardElement);
                }, 150);
            }
        }, 100);
        
        this.editor.markDirty();
        taflNotifications.success('Card duplicated');
    }

    /**
     * Select a card (delegates to Store)
     */
    selectCard(cardElement) {
        // Prevent selection during card addition to avoid infinite loops
        if (this._isAddingCard) {
            return;
        }
        
        
        if (!cardElement) {
            taflFlowStore.deselectCard();
            return;
        }
        
        const cardId = cardElement.dataset.cardId;
        
        if (!cardId) {
            return;
        }
        
        // Check if card exists in store
        const cardInStore = taflFlowStore.findCardById(cardId);
        if (cardInStore) {
        }
        
        // Only update Store, UI will update via events
        taflFlowStore.selectCard(cardId);
    }

    /**
     * Clear card selection (delegates to Store)
     */
    clearSelection() {
        taflFlowStore.deselectCard();
    }
    
    /**
     * Scroll card into view
     */
    scrollCardIntoView(cardElement) {
        const canvas = document.getElementById('flow-canvas');
        const canvasRect = canvas.getBoundingClientRect();
        const cardRect = cardElement.getBoundingClientRect();
        
        if (cardRect.top < canvasRect.top || cardRect.bottom > canvasRect.bottom) {
            cardElement.scrollIntoView({ behavior: 'smooth', block: 'center' });
        }
    }

    /**
     * Update card parameters
     */
    updateCardParameters(cardId, params) {
        const card = this.findCardInFlow(cardId);
        if (!card) {
            return;
        }
        
        // Get verb from card
        const verb = this.getCardVerb(card);
        if (!verb) return;
        
        // Update card parameters
        card[verb] = { ...card[verb], ...params };
        
        // Trigger store update
        const flow = taflFlowStore.getFlow();
        taflFlowStore.updateFlow(flow);
        
        this.editor.markDirty();
        
        // Update the card element if it exists
        const cardElement = document.querySelector(`[data-card-id="${cardId}"]`);
        if (cardElement) {
            const titleElement = cardElement.querySelector('.card-title');
            if (titleElement) {
                titleElement.textContent = this.getCardTitle(card);
            }
            const bodyElement = cardElement.querySelector('.tafl-card-body');
            if (bodyElement) {
                bodyElement.innerHTML = this.renderCardParams(card);
            }
        }
    }

    /**
     * Get card title
     */
    getCardTitle(cardData) {
        const verb = this.getCardVerb(cardData);
        const params = cardData[verb];
        
        if (!params) return verb;
        
        // Generate meaningful title based on verb and params
        switch (verb) {
            case 'query':
                // TAFL v1.1.1: target + where + as
                return params.target ? `Query ${params.target}` : 'Query';
            case 'check':
                // TAFL v1.1.1: target + condition + as
                return params.condition ? `Check: ${this.truncate(params.condition, 30)}` : 'Check';
            case 'create':
                // TAFL v1.1.1: target + with + as
                return params.target ? `Create ${params.target}` : 'Create';
            case 'update':
                // TAFL v1.1.1: target + where + set
                return params.target ? `Update ${params.target}` : 'Update';
            case 'delete':
                return params.target ? `Delete ${params.target}` : 'Delete';
            case 'if':
                return params.condition ? `If: ${this.truncate(params.condition, 30)}` : 'If';
            case 'for':
                // TAFL v1.1.1: in + as + do (not 'each')
                if (params.in) {
                    return params.as ? `For ${params.as} in ${this.truncate(params.in, 20)}` : `For in ${this.truncate(params.in, 30)}`;
                }
                // Fallback for legacy format
                return params.each ? `For each ${params.each}` : 'For';
            case 'switch':
                // TAFL v1.1.1: expression (not 'on')
                if (params.expression) {
                    return `Switch: ${this.truncate(params.expression, 30)}`;
                }
                // Fallback for legacy format
                return params.on ? `Switch on ${params.on}` : 'Switch';
            case 'set':
                // Always use object format for SET cards
                if (typeof params === 'object' && params !== null) {
                    const keys = Object.keys(params).filter(k => k !== 'id');
                    if (keys.length === 0) {
                        return 'Set: (empty)';
                    } else if (keys.length === 1) {
                        return `Set: ${keys[0]}`;
                    } else {
                        return `Set: ${keys.length} variables`;
                    }
                }
                // Fallback
                return 'Set: variables';
            case 'log':
                return params.message ? `Log: ${this.truncate(params.message, 30)}` : 'Log';
            case 'notify':
                // TAFL v1.1.1: level is primary field
                if (params.level) {
                    return `Notify [${params.level}]: ${this.truncate(params.message || '', 20)}`;
                }
                return params.message ? `Notify: ${this.truncate(params.message, 30)}` : 'Notify';
            case 'wait':
                return params.duration ? `Wait ${params.duration}${params.unit || 's'}` : 'Wait';
            case 'http':
                return params.url ? `HTTP ${params.method || 'GET'} ${this.truncate(params.url, 30)}` : 'HTTP';
            default:
                return verb;
        }
    }

    /**
     * Render card parameters
     */
    renderCardParams(cardData) {
        const verb = this.getCardVerb(cardData);
        const params = cardData[verb];
        
        // Special handling for 'set' verb (TAFL v1.1.1)
        if (verb === 'set') {
            // Handle string format (single variable assignment)
            if (typeof params === 'string') {
                return `<div class="card-params">
                    <div class="param-item">
                        <span class="param-value">${this.formatParamValue(params)}</span>
                    </div>
                </div>`;
            }
            // Handle dictionary format (multiple variable assignments)
            else if (typeof params === 'object' && params !== null) {
                let html = '<div class="card-params">';
                const entries = Object.entries(params).slice(0, 5); // Limit to 5 for display
                
                if (entries.length === 0) {
                    html += '<div class="no-params">No variables set</div>';
                } else {
                    entries.forEach(([key, value]) => {
                        const displayValue = this.formatParamValue(value);
                        html += `
                            <div class="param-item">
                                <span class="param-key">${key}:</span>
                                <span class="param-value">${displayValue}</span>
                            </div>
                        `;
                    });
                    
                    if (Object.keys(params).length > 5) {
                        html += `<div class="param-item more-params">... and ${Object.keys(params).length - 5} more</div>`;
                    }
                }
                
                html += '</div>';
                return html;
            }
        }
        
        if (!params || typeof params !== 'object') {
            return '<div class="no-params">No parameters</div>';
        }
        
        let html = '<div class="card-params">';
        
        // Custom display based on verb (TAFL v1.1.1 compliance)
        let displayParams = [];
        
        switch(verb) {
            case 'query':
                // Show: target, where, as
                displayParams = Object.entries(params)
                    .filter(([key]) => ['target', 'where', 'as'].includes(key));
                break;
            case 'check':
                // Show: target, condition, as
                displayParams = Object.entries(params)
                    .filter(([key]) => ['target', 'condition', 'as'].includes(key));
                break;
            case 'create':
                // Show: target, with (not parameters), as
                displayParams = Object.entries(params)
                    .filter(([key]) => ['target', 'with', 'as'].includes(key) || (key === 'parameters' && !params.with));
                break;
            case 'update':
                // Show: target, where, set (not id_expr/changes)
                displayParams = Object.entries(params)
                    .filter(([key]) => ['target', 'where', 'set'].includes(key));
                break;
            case 'for':
                // Show: in, as, do, filter (not each/variable)
                displayParams = Object.entries(params)
                    .filter(([key]) => ['in', 'as', 'filter'].includes(key) || (key === 'each' && !params.in));
                break;
            case 'switch':
                // Show: expression (not on)
                displayParams = Object.entries(params)
                    .filter(([key]) => ['expression'].includes(key) || (key === 'on' && !params.expression));
                break;
            case 'notify':
                // Show: level (primary), message, recipients
                displayParams = Object.entries(params)
                    .filter(([key]) => ['level', 'message', 'recipients'].includes(key));
                break;
            default:
                // Default: filter out nested structures and id
                displayParams = Object.entries(params)
                    .filter(([key]) => !['then', 'else', 'do', 'cases', 'default', 'id'].includes(key));
        }
        
        // Limit to 5 parameters for display
        displayParams = displayParams.slice(0, 5);
        
        displayParams.forEach(([key, value]) => {
            const displayValue = this.formatParamValue(value);
            html += `
                <div class="param-item">
                    <span class="param-key">${key}:</span>
                    <span class="param-value">${displayValue}</span>
                </div>
            `;
        });
        
        if (displayParams.length === 0) {
            html += '<div class="no-params">Configure parameters</div>';
        }
        
        html += '</div>';
        return html;
    }


    /**
     * Helper: Get verb from card data
     */
    getCardVerb(cardData) {
        if (!cardData) return null;
        return Object.keys(cardData).find(key => 
            key !== 'id' && this.verbDefinitions && this.verbDefinitions[key]
        );
    }

    /**
     * Helper: Find card in flow
     */
    findCardInFlow(cardId, flowArray = null) {
        if (!flowArray) {
            flowArray = taflFlowStore.getFlow().flow;
        }
        
        for (const item of flowArray) {
            if (item.id === cardId) {
                return item;
            }
            
            // Search in nested structures
            const verb = this.getCardVerb(item);
            const params = item[verb];
            
            if (params && typeof params === 'object') {
                // Check nested arrays
                const nestedArrays = ['then', 'else', 'do', 'cases', 'default'];
                for (const key of nestedArrays) {
                    if (Array.isArray(params[key])) {
                        const found = this.findCardInFlow(cardId, params[key]);
                        if (found) return found;
                    } else if (key === 'cases' && Array.isArray(params.cases)) {
                        for (const caseItem of params.cases) {
                            if (caseItem.do && Array.isArray(caseItem.do)) {
                                const found = this.findCardInFlow(cardId, caseItem.do);
                                if (found) return found;
                            }
                        }
                    }
                }
            }
        }
        
        return null;
    }

    /**
     * Helper: Find card position
     */
    findCardPosition(cardId, flowArray = null, parentInfo = null) {
        if (!flowArray) {
            flowArray = taflFlowStore.getFlow().flow;
        }
        
        for (let i = 0; i < flowArray.length; i++) {
            const item = flowArray[i];
            
            if (item.id === cardId) {
                return parentInfo ? { nested: parentInfo, index: i } : { index: i };
            }
            
            // Search in nested structures
            const verb = this.getCardVerb(item);
            const params = item[verb];
            
            if (params && typeof params === 'object') {
                const nestedArrays = ['then', 'else', 'do', 'default'];
                for (const key of nestedArrays) {
                    if (Array.isArray(params[key])) {
                        const found = this.findCardPosition(
                            cardId, 
                            params[key], 
                            { parentCardId: item.id, branchType: key }
                        );
                        if (found) return found;
                    }
                }
                
                // Handle switch cases
                if (verb === 'switch' && Array.isArray(params.cases)) {
                    for (let j = 0; j < params.cases.length; j++) {
                        const caseItem = params.cases[j];
                        if (caseItem.do && Array.isArray(caseItem.do)) {
                            const found = this.findCardPosition(
                                cardId,
                                caseItem.do,
                                { parentCardId: item.id, branchType: `case-${j}` }
                            );
                            if (found) return found;
                        }
                    }
                }
            }
        }
        
        return null;
    }

    /**
     * Remove card from flow (wrapper for store's deleteCard)
     * This is called by dragdrop module with just cardId
     */
    removeCardFromFlow(cardId) {
        // Delegate to store's deleteCard which handles the removal
        taflFlowStore.deleteCard(cardId);
    }

    /**
     * Helper: Remove card from flow array (internal use)
     */
    removeCardFromFlowArray(flowArray, cardId) {
        const newFlow = [];
        
        for (const item of flowArray) {
            if (item.id === cardId) {
                continue; // Skip this card
            }
            
            // Process nested structures
            const verb = this.getCardVerb(item);
            const params = item[verb];
            
            if (params && typeof params === 'object') {
                const newItem = { ...item, [verb]: { ...params } };
                
                // Process nested arrays
                const nestedArrays = ['then', 'else', 'do', 'default'];
                for (const key of nestedArrays) {
                    if (Array.isArray(params[key])) {
                        newItem[verb][key] = this.removeCardFromFlowArray(params[key], cardId);
                    }
                }
                
                // Handle switch cases
                if (verb === 'switch' && Array.isArray(params.cases)) {
                    newItem[verb].cases = params.cases.map(caseItem => {
                        if (caseItem.do && Array.isArray(caseItem.do)) {
                            return {
                                ...caseItem,
                                do: this.removeCardFromFlowArray(caseItem.do, cardId)
                            };
                        }
                        return caseItem;
                    });
                }
                
                newFlow.push(newItem);
            } else {
                newFlow.push(item);
            }
        }
        
        return newFlow;
    }

    /**
     * Helper: Add to nested structure
     */
    addToNestedStructure(parentCardId, branchType, cardData, index = 0) {
        
        const flow = taflFlowStore.getFlow();
        const updatedFlow = this.insertCardInNested(flow.flow, cardData, {
            nested: { parentCardId, branchType },
            index: index  // Pass the index for proper positioning
        });
        
        taflFlowStore.updateFlow({ ...flow, flow: updatedFlow });
    }

    /**
     * Helper: Insert card in nested structure
     */
    insertCardInNested(flowArray, cardData, position) {
        
        return flowArray.map(item => {
            if (item.id === position.nested.parentCardId) {
                const verb = this.getCardVerb(item);
                const params = item[verb];
                const newItem = { ...item, [verb]: { ...params } };
                
                if (position.nested.branchType.startsWith('case-')) {
                    // Handle switch case
                    const caseIndex = parseInt(position.nested.branchType.split('-')[1]);
                    if (newItem[verb].cases && newItem[verb].cases[caseIndex]) {
                        if (!newItem[verb].cases[caseIndex].do) {
                            newItem[verb].cases[caseIndex].do = [];
                        }
                        // Use splice to insert at correct position instead of push
                        const insertIndex = typeof position.index === 'number' ? position.index : 0;
                        newItem[verb].cases[caseIndex].do.splice(insertIndex, 0, cardData);
                    }
                } else {
                    // Handle regular nested branch
                    if (!Array.isArray(newItem[verb][position.nested.branchType])) {
                        newItem[verb][position.nested.branchType] = [];
                    }
                    // Use splice to insert at correct position instead of push
                    const insertIndex = typeof position.index === 'number' ? position.index : 0;
                    newItem[verb][position.nested.branchType].splice(insertIndex, 0, cardData);
                }
                
                return newItem;
            }
            
            // Continue searching in nested structures
            const verb = this.getCardVerb(item);
            const params = item[verb];
            
            if (params && typeof params === 'object') {
                const newItem = { ...item, [verb]: { ...params } };
                
                // Process nested arrays
                const nestedArrays = ['then', 'else', 'do', 'default'];
                for (const key of nestedArrays) {
                    if (Array.isArray(params[key])) {
                        newItem[verb][key] = this.insertCardInNested(params[key], cardData, position);
                    }
                }
                
                // Handle switch cases
                if (verb === 'switch' && Array.isArray(params.cases)) {
                    newItem[verb].cases = params.cases.map((caseItem, index) => {
                        if (caseItem.do && Array.isArray(caseItem.do)) {
                            return {
                                ...caseItem,
                                do: this.insertCardInNested(caseItem.do, cardData, position)
                            };
                        }
                        return caseItem;
                    });
                }
                
                return newItem;
            }
            
            return item;
        });
    }

    /**
     * Helper: Insert card after another in nested structure
     */
    insertCardAfterInNested(afterCardId, newCard) {
        const position = this.findCardPosition(afterCardId);
        
        if (position.nested) {
            // Find parent and insert after the original
            const parent = this.findCardInFlow(position.nested.parentCardId);
            if (parent) {
                const verb = this.getCardVerb(parent);
                const branchArray = parent[verb][position.nested.branchType];
                if (Array.isArray(branchArray)) {
                    branchArray.splice(position.index + 1, 0, newCard);
                }
            }
        } else {
            // Insert in main flow
            const flow = taflFlowStore.getFlow();
            const newFlow = [...flow.flow];
            newFlow.splice(position.index + 1, 0, newCard);
            taflFlowStore.updateFlow({ ...flow, flow: newFlow });
        }
    }

    /**
     * Helper: Format parameter value for display
     */
    formatParamValue(value) {
        if (value === null || value === undefined) {
            return '<span class="null-value">null</span>';
        }
        if (typeof value === 'boolean') {
            return `<span class="bool-value">${value}</span>`;
        }
        if (typeof value === 'number') {
            return `<span class="number-value">${value}</span>`;
        }
        if (typeof value === 'string') {
            return this.escapeHtml(this.truncate(value, 50));
        }
        if (Array.isArray(value)) {
            return `<span class="array-value">[${value.length} items]</span>`;
        }
        if (typeof value === 'object') {
            const keys = Object.keys(value);
            return `<span class="object-value">{${keys.length} props}</span>`;
        }
        return String(value);
    }

    /**
     * Helper: Truncate text
     */
    truncate(text, maxLength) {
        if (!text || text.length <= maxLength) return text;
        return text.substring(0, maxLength) + '...';
    }

    /**
     * Helper: Escape HTML
     */
    escapeHtml(text) {
        const div = document.createElement('div');
        div.textContent = text;
        return div.innerHTML;
    }

    /**
     * Get selected card ID (delegates to Store)
     */
    getSelectedCardId() {
        return taflFlowStore.getSelectedCardId();
    }

    /**
     * Get selected card data (delegates to Store)
     */
    getSelectedCardData() {
        const selectedId = taflFlowStore.getSelectedCardId();
        if (!selectedId) return null;
        return this.findCardInFlow(selectedId);
    }
}

// Create singleton instance
const taflEditorCards = new TAFLEditorCards();

export default taflEditorCards;
export { TAFLEditorCards };