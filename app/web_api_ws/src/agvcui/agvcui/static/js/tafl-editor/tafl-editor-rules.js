/**
 * TAFL Editor Rules Module
 * Handles business rules management functionality
 */

import { taflFlowStore } from './tafl-editor-store.js';
import taflModals from './tafl-editor-modals.js';

class TAFLEditorRules {
    constructor(editor) {
        this.editor = editor;  // Reference to main TAFLEditor instance
    }

    /**
     * Add a new rule using prompts
     */
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
        this.editor.markDirty();
        this.editor.yamlModule.generateYaml();
    }
    
    /**
     * Save a new rule from modal inputs
     */
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
        this.editor.markDirty();
        this.editor.yamlModule.generateYaml();
    }
    
    /**
     * Edit an existing business rule
     * @param {string} name - Rule name to edit
     */
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
                    <button class="button is-success" onclick="window.taflEditor.rulesModule.saveRuleEdit('${name}')">Save Changes</button>
                    <button class="button" onclick="this.closest('.modal').remove()">Cancel</button>
                </footer>
            </div>
        `;
        document.body.appendChild(modal);
    }
    
    /**
     * Save edited rule
     * @param {string} oldName - Original rule name
     */
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
        this.editor.markDirty();
        this.editor.yamlModule.generateYaml();
    }
    
    /**
     * Remove a rule
     * @param {string} name - Rule name to remove
     */
    removeRule(name) {
        const flow = taflFlowStore.getFlow();
        if (flow.rules && flow.rules[name]) {
            delete flow.rules[name];
            // Panel update handled by SimpleTAFLPanels via store events
            taflFlowStore.emit('flow:changed');
            this.editor.markDirty();
            this.editor.showNotification('Rule removed successfully', 'success');
        }
    }
    
    /**
     * Parse expression with variable/rule/preload references
     * Enhanced expression parser for TAFL v1.1
     * @param {string} expr - Expression to parse
     * @returns {*} Parsed expression
     */
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
}

// Export singleton instance
const taflEditorRules = new TAFLEditorRules(null);

export default taflEditorRules;
export { TAFLEditorRules };