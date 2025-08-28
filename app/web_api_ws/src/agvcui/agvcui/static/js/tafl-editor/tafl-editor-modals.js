/**
 * TAFL Editor Modals Module
 * Centralized modal and dialog management for TAFL Editor
 * Provides unified interface for all modal interactions
 */

import { taflFlowStore } from './tafl-editor-store.js';
import taflAPI from './tafl-editor-api.js';
import taflNotifications from './tafl-notifications.js';
import { taflPanels } from './tafl-panels.js';

class TAFLEditorModals {
    constructor() {
        this.activeModals = new Set();
        this.confirmCallbacks = new Map();
    }

    // ============================================
    // Flow Management Modals
    // ============================================

    /**
     * Show new flow modal/action
     */
    newFlow() {
        // Reset flow to default state
        const defaultFlow = {
            metadata: {
                id: '',
                name: 'Untitled Flow',
                version: '1.1',
                description: ''
            },
            settings: {
                timeout: 3600,
                max_retries: 3,
                retry_on_failure: false
            },
            preload: {},
            rules: {},
            variables: {},
            flow: []
        };
        
        taflFlowStore.loadFlow(defaultFlow);
        taflFlowStore.setDirty(false);
        
        // Clear localStorage
        localStorage.removeItem('lastEditedFlowId');
        
        // Notify user
        taflNotifications.info('New flow created');
        
        // Trigger UI refresh through store events
        return true;
    }

    /**
     * Show save flow modal
     */
    showSaveFlowModal() {
        const modal = document.getElementById('save-flow-modal');
        if (!modal) {
            console.error('Save flow modal not found');
            return;
        }
        
        // Pre-fill form with current metadata
        const flow = taflFlowStore.getFlow();
        const idInput = document.getElementById('save-flow-id');
        const nameInput = document.getElementById('save-flow-name');
        const descInput = document.getElementById('save-flow-description');
        
        if (idInput) idInput.value = flow.metadata.id || '';
        if (nameInput) nameInput.value = flow.metadata.name || '';
        if (descInput) descInput.value = flow.metadata.description || '';
        
        this.showModal('save-flow-modal');
    }

    /**
     * Save flow with modal data
     */
    async saveFlow() {
        const flowId = document.getElementById('save-flow-id')?.value || 'untitled';
        const flowName = document.getElementById('save-flow-name')?.value || 'Untitled Flow';
        const flowDescription = document.getElementById('save-flow-description')?.value || '';
        
        if (!flowId) {
            taflNotifications.warning('Flow ID is required');
            return;
        }
        
        // Update current flow metadata
        const flow = taflFlowStore.getFlow();
        flow.metadata.id = flowId;
        flow.metadata.name = flowName;
        flow.metadata.description = flowDescription;
        
        try {
            await taflAPI.saveFlow(flow);
            
            // Save as last edited flow
            localStorage.setItem('lastEditedFlowId', flowId);
            taflNotifications.success('Flow saved successfully');
            taflFlowStore.setDirty(false);
            
            // Close modal
            this.closeModal('save-flow-modal');
            
            // Update flows dropdown
            if (window.loadFlowsDropdown) {
                window.loadFlowsDropdown();
            }
            
            return true;
        } catch (error) {
            taflNotifications.error(error.message);
            return false;
        }
    }

    // ============================================
    // Generic Dialog Methods
    // ============================================

    /**
     * Show confirmation dialog
     * @param {string} message - Confirmation message
     * @param {string} title - Dialog title (optional)
     * @returns {Promise<boolean>} - User's choice
     */
    async confirm(message, title = 'Confirm') {
        // For now, use native confirm
        // TODO: Replace with custom modal in future
        return window.confirm(message);
    }

    /**
     * Show alert dialog
     * @param {string} message - Alert message
     * @param {string} type - Alert type (info, warning, error, success)
     */
    alert(message, type = 'info') {
        // Use notification system for alerts
        switch (type) {
            case 'error':
                taflNotifications.error(message);
                break;
            case 'warning':
                taflNotifications.warning(message);
                break;
            case 'success':
                taflNotifications.success(message);
                break;
            default:
                taflNotifications.info(message);
        }
    }

    /**
     * Show prompt dialog
     * @param {string} message - Prompt message
     * @param {string} defaultValue - Default input value
     * @returns {Promise<string|null>} - User input or null
     */
    async prompt(message, defaultValue = '') {
        // For now, use native prompt
        // TODO: Replace with custom modal in future
        return window.prompt(message, defaultValue);
    }

    // ============================================
    // Variable/Rule/Preload Modals
    // ============================================

    /**
     * Show add variable modal
     */
    showAddVariableModal() {
        // This functionality is now in TAFLPanels
        // Just trigger the event or call the method
        if (taflPanels && taflPanels.showAddVariableModal) {
            taflPanels.showAddVariableModal();
        } else {
            // Fallback: simple prompt
            this.addVariableWithPrompt();
        }
    }

    /**
     * Add variable with simple prompt (fallback)
     */
    async addVariableWithPrompt() {
        const name = await this.prompt('Variable name:');
        if (!name) return;
        
        const value = await this.prompt('Variable value:', '');
        if (value === null) return;
        
        const flow = taflFlowStore.getFlow();
        if (!flow.variables) flow.variables = {};
        
        if (flow.variables[name] !== undefined) {
            const overwrite = await this.confirm(`Variable "${name}" already exists. Replace it?`);
            if (!overwrite) return;
        }
        
        flow.variables[name] = value;
        taflFlowStore.updateFlow(flow);
        taflNotifications.success(`Variable "${name}" added`);
    }

    /**
     * Show add rule modal
     */
    showAddRuleModal() {
        // This functionality is now in TAFLPanels
        if (taflPanels && taflPanels.showAddRuleModal) {
            taflPanels.showAddRuleModal();
        } else {
            this.addRuleWithPrompt();
        }
    }

    /**
     * Add rule with simple prompt (fallback)
     */
    async addRuleWithPrompt() {
        const name = await this.prompt('Rule name:');
        if (!name) return;
        
        const value = await this.prompt('Rule value:', 'true');
        if (value === null) return;
        
        const flow = taflFlowStore.getFlow();
        if (!flow.rules) flow.rules = {};
        
        if (flow.rules[name] !== undefined) {
            const overwrite = await this.confirm(`Rule "${name}" already exists. Replace it?`);
            if (!overwrite) return;
        }
        
        flow.rules[name] = value === 'true' || value === true;
        taflFlowStore.updateFlow(flow);
        taflNotifications.success(`Rule "${name}" added`);
    }

    /**
     * Show add preload modal
     */
    showAddPreloadModal() {
        // This functionality is now in TAFLPanels
        if (taflPanels && taflPanels.showAddPreloadModal) {
            taflPanels.showAddPreloadModal();
        } else {
            taflNotifications.info('Add preload through the Preload panel');
        }
    }

    // ============================================
    // Modal Control Methods
    // ============================================

    /**
     * Show a modal by ID
     * @param {string} modalId - Modal element ID
     */
    showModal(modalId) {
        const modal = document.getElementById(modalId);
        if (modal) {
            modal.classList.add('is-active');
            this.activeModals.add(modalId);
            
            // Auto-focus first input
            const firstInput = modal.querySelector('input:not([type="hidden"]), textarea');
            if (firstInput) {
                setTimeout(() => firstInput.focus(), 100);
            }
        }
    }

    /**
     * Close a modal by ID
     * @param {string} modalId - Modal element ID
     */
    closeModal(modalId) {
        const modal = document.getElementById(modalId);
        if (modal) {
            modal.classList.remove('is-active');
            this.activeModals.delete(modalId);
        }
    }

    /**
     * Close all open modals
     */
    closeAllModals() {
        this.activeModals.forEach(modalId => {
            this.closeModal(modalId);
        });
    }

    /**
     * Setup modal controls and event listeners
     */
    setupModalControls() {
        // Close buttons
        document.querySelectorAll('.modal .delete, .modal .cancel-btn').forEach(btn => {
            btn.addEventListener('click', (e) => {
                const modal = e.target.closest('.modal');
                if (modal) {
                    this.closeModal(modal.id);
                }
            });
        });
        
        // Background click to close
        document.querySelectorAll('.modal-background').forEach(bg => {
            bg.addEventListener('click', (e) => {
                const modal = e.target.closest('.modal');
                if (modal) {
                    this.closeModal(modal.id);
                }
            });
        });
        
        // Save flow button
        const confirmSaveBtn = document.getElementById('confirm-save-btn');
        if (confirmSaveBtn) {
            confirmSaveBtn.addEventListener('click', () => this.saveFlow());
        }
        
        // ESC key to close topmost modal
        document.addEventListener('keydown', (e) => {
            if (e.key === 'Escape' && this.activeModals.size > 0) {
                const modalIds = Array.from(this.activeModals);
                const lastModal = modalIds[modalIds.length - 1];
                this.closeModal(lastModal);
            }
        });
    }

    // ============================================
    // Validation and Confirmation Helpers
    // ============================================

    /**
     * Confirm deletion with standard message
     * @param {string} itemType - Type of item being deleted
     * @param {string} itemName - Name of item (optional)
     */
    async confirmDelete(itemType, itemName = '') {
        const message = itemName 
            ? `Are you sure you want to delete ${itemType} "${itemName}"?`
            : `Are you sure you want to delete this ${itemType}?`;
        return this.confirm(message, 'Confirm Delete');
    }

    /**
     * Confirm overwrite with standard message
     * @param {string} itemType - Type of item
     * @param {string} itemName - Name of item
     */
    async confirmOverwrite(itemType, itemName) {
        return this.confirm(
            `${itemType} "${itemName}" already exists. Replace it?`,
            'Confirm Overwrite'
        );
    }

    /**
     * Show validation error
     * @param {string} message - Error message
     */
    showValidationError(message) {
        this.alert(message, 'error');
    }

    /**
     * Show required fields error
     * @param {Array<string>} fields - Required field names
     */
    showRequiredFieldsError(fields) {
        const fieldList = fields.join(', ');
        this.alert(`Required fields: ${fieldList}`, 'warning');
    }
}

// Export singleton instance
const taflModals = new TAFLEditorModals();
export default taflModals;

// Also export class for testing
export { TAFLEditorModals };