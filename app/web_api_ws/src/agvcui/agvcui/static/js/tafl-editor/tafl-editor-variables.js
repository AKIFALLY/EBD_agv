/**
 * TAFL Editor Variables Management Module
 * Handles variable CRUD operations and management
 */

import { taflFlowStore } from './tafl-editor-store.js';

class TAFLEditorVariables {
    constructor(editor) {
        this.editor = editor;  // Reference to main TAFLEditor instance
    }

    /**
     * Initialize variables for a new flow
     */
    initializeVariables() {
        // Initialize with empty variables object
        // Users can add variables as needed
        const store = window.taflFlowStore || taflFlowStore;
        const flow = store.getFlow();
        if (flow) {
            flow.variables = flow.variables || {};
        }
        // SimpleTAFLPanels handles UI updates via store events
    }

    /**
     * Add a new variable
     */
    addVariable() {
        const name = prompt('Variable name:');
        if (name) {
            const store = window.taflFlowStore || taflFlowStore;
            const flow = store.getFlow();
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
                store.emit('variables:changed');
                this.editor.markDirty();
                this.editor.showNotification('Variable added successfully', 'success');
                
                // Switch to Variables tab to show the newly added variable
                this.editor.switchPropertiesTab('variables');
            } else {
                this.editor.showNotification('Variable already exists', 'warning');
            }
        }
    }

    /**
     * Remove a variable by name
     * @param {string} name - Variable name to remove
     */
    removeVariable(name) {
        const store = window.taflFlowStore || taflFlowStore;
        const flow = store.getFlow();
        if (flow.variables && flow.variables[name] !== undefined) {
            delete flow.variables[name];
            // Use the store's updateVariables method which will emit the event
            store.updateVariables(flow.variables);
            this.editor.markDirty();
            this.editor.showNotification('Variable removed successfully', 'success');
        }
    }

    /**
     * Update a variable's value
     * @param {string} name - Variable name
     * @param {any} value - New value
     */
    updateVariable(name, value) {
        const store = window.taflFlowStore || taflFlowStore;
        const flow = store.getFlow();
        if (!flow.variables) {
            flow.variables = {};
        }
        
        flow.variables[name] = value;
        store.updateVariables(flow.variables);
        this.editor.markDirty();
        this.editor.showNotification(`Variable ${name} updated`, 'success');
    }

    /**
     * Get a variable's value
     * @param {string} name - Variable name
     * @returns {any} Variable value or undefined
     */
    getVariable(name) {
        const store = window.taflFlowStore || taflFlowStore;
        const flow = store.getFlow();
        return flow.variables ? flow.variables[name] : undefined;
    }

    /**
     * Get all variables
     * @returns {object} All variables
     */
    getAllVariables() {
        const store = window.taflFlowStore || taflFlowStore;
        const flow = store.getFlow();
        return flow.variables || {};
    }

    /**
     * Check if a variable exists
     * @param {string} name - Variable name
     * @returns {boolean} True if variable exists
     */
    hasVariable(name) {
        const flow = window.taflFlowStore.getFlow();
        return flow.variables && flow.variables.hasOwnProperty(name);
    }

    /**
     * Clear all variables
     */
    clearVariables() {
        const flow = window.taflFlowStore.getFlow();
        flow.variables = {};
        window.taflFlowStore.updateVariables(flow.variables);
        this.editor.markDirty();
        this.editor.showNotification('All variables cleared', 'info');
    }

    /**
     * Import variables from object
     * @param {object} variables - Variables object to import
     */
    importVariables(variables) {
        if (typeof variables === 'object' && variables !== null) {
            const flow = window.taflFlowStore.getFlow();
            flow.variables = { ...variables };
            window.taflFlowStore.updateVariables(flow.variables);
            this.editor.markDirty();
            this.editor.showNotification('Variables imported successfully', 'success');
        } else {
            this.editor.showNotification('Invalid variables object', 'error');
        }
    }

    /**
     * Export variables as JSON
     * @returns {string} JSON string of variables
     */
    exportVariables() {
        const flow = window.taflFlowStore.getFlow();
        return JSON.stringify(flow.variables || {}, null, 2);
    }

    /**
     * Render variables (delegates to panels)
     */
    renderVariables() {
        // SimpleTAFLPanels handles UI updates via store events
        // This is kept for backward compatibility
        if (window.taflPanels && window.taflPanels.updateVariablesPanel) {
            window.taflPanels.updateVariablesPanel();
        }
    }

    /**
     * Validate variable name
     * @param {string} name - Variable name to validate
     * @returns {boolean} True if valid
     */
    isValidVariableName(name) {
        // Variable names should start with letter or underscore
        // and contain only letters, numbers, and underscores
        return /^[a-zA-Z_][a-zA-Z0-9_]*$/.test(name);
    }

    /**
     * Get variables used in flow
     * @returns {Set} Set of variable names used in the flow
     */
    getUsedVariables() {
        const usedVars = new Set();
        const flow = window.taflFlowStore.getFlow();
        
        // Helper function to extract variables from expressions
        const extractVars = (str) => {
            if (typeof str === 'string') {
                // Match $variable or ${variable} patterns
                const matches = str.match(/\$\{?([a-zA-Z_][a-zA-Z0-9_]*)\}?/g);
                if (matches) {
                    matches.forEach(match => {
                        const varName = match.replace(/\$\{?|\}?/g, '');
                        usedVars.add(varName);
                    });
                }
            }
        };
        
        // Recursively search through flow for variable references
        const searchFlow = (items) => {
            if (Array.isArray(items)) {
                items.forEach(item => searchInItem(item));
            }
        };
        
        const searchInItem = (item) => {
            if (!item) return;
            
            // Search in all properties
            Object.values(item).forEach(value => {
                if (typeof value === 'string') {
                    extractVars(value);
                } else if (Array.isArray(value)) {
                    searchFlow(value);
                } else if (typeof value === 'object' && value !== null) {
                    searchInItem(value);
                }
            });
        };
        
        if (flow.flow) {
            searchFlow(flow.flow);
        }
        
        return usedVars;
    }

    /**
     * Get unused variables
     * @returns {Array} Array of unused variable names
     */
    getUnusedVariables() {
        const flow = window.taflFlowStore.getFlow();
        if (!flow.variables) return [];
        
        const usedVars = this.getUsedVariables();
        const allVars = Object.keys(flow.variables);
        
        return allVars.filter(varName => !usedVars.has(varName));
    }
}

// Export singleton instance
const taflEditorVariables = new TAFLEditorVariables(null);

export default taflEditorVariables;
export { TAFLEditorVariables };