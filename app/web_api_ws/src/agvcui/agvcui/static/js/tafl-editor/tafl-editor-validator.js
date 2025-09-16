/**
 * TAFL Editor Validator Module
 * Handles all validation functionality for TAFL flows
 */

import { taflFlowStore } from './tafl-editor-store.js';
import taflNotifications from './tafl-notifications.js';
import taflAPI from './tafl-editor-api.js';

class TAFLEditorValidator {
    constructor() {
        this.validationResults = null;
        this.validationInProgress = false;
    }

    /**
     * Initialize validator
     */
    init(editor) {
        this.editor = editor;
        this.setupValidationUI();
    }

    /**
     * Setup validation UI elements
     */
    setupValidationUI() {
        // Ensure validation results container exists
        const resultsContainer = document.getElementById('validation-results');
        if (!resultsContainer) {
            console.warn('Validation results container not found');
        }
    }

    /**
     * Validate the entire flow
     */
    async validateFlow() {
        if (this.validationInProgress) {
            taflNotifications.warning('Validation already in progress');
            return;
        }

        this.validationInProgress = true;
        
        try {
            // Get current flow data
            const flowData = {
                metadata: taflFlowStore.getFlow().metadata,
                variables: taflFlowStore.getFlow().variables,
                flow: taflFlowStore.getFlow().flow,
                settings: taflFlowStore.getFlow().settings,
                rules: taflFlowStore.getFlow().rules,
                preload: taflFlowStore.getFlow().preload
            };

            // Call validation API
            const result = await taflAPI.validateFlow(flowData);
            
            this.validationResults = result;
            this.displayValidationResults(result);
            
            // Switch to results tab to show validation results
            if (this.editor && this.editor.switchPropertiesTab) {
                this.editor.switchPropertiesTab('results');
            }

            // Show notification based on validation result
            const errors = result.errors || [];
            const warnings = result.warnings || [];
            
            if (result.valid && errors.length === 0 && warnings.length === 0) {
                taflNotifications.success('Flow validation passed!');
            } else if (errors.length > 0) {
                taflNotifications.error(`Validation failed with ${errors.length} error(s)`);
            } else if (warnings.length > 0) {
                taflNotifications.warning(`Validation passed with ${warnings.length} warning(s)`);
            }

            return result;
        } catch (error) {
            console.error('Validation error:', error);
            taflNotifications.error('Validation failed: ' + error.message);
            throw error;
        } finally {
            this.validationInProgress = false;
        }
    }

    /**
     * Display validation results in the UI
     */
    displayValidationResults(result) {
        // Update mode indicator for validation
        const modeElement = document.getElementById('result-mode');
        if (modeElement) {
            const tagClass = result.valid ? 
                (result.warnings.length > 0 ? 'tag is-warning' : 'tag is-success') : 
                'tag is-danger';
            const text = result.valid ? 
                (result.warnings.length > 0 ? '⚠ Validation Passed with Warnings' : '✓ Validation Passed') : 
                '✗ Validation Failed';
            modeElement.innerHTML = `<span class="${tagClass}">${text}</span>`;
        }
        
        // Hide execution-related sections
        const execLog = document.getElementById('execution-log');
        if (execLog) execLog.style.display = 'none';
        
        const varChanges = document.getElementById('variable-changes');
        if (varChanges) varChanges.style.display = 'none';
        
        // Show validation results section
        const validationDiv = document.getElementById('validation-results');
        if (validationDiv) {
            validationDiv.style.display = 'block';
        }
        
        // Get the validation content container
        const resultsContainer = document.querySelector('.validation-content');
        if (!resultsContainer) {
            console.warn('Validation content container not found');
            return;
        }
        
        // 確保 errors 和 warnings 陣列存在
        const errors = result.errors || [];
        const warnings = result.warnings || [];
        
        if (result.valid && errors.length === 0 && warnings.length === 0) {
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
            
            // Display errors
            errors.forEach(error => {
                html += `
                    <div class="validation-item error">
                        <span class="icon validation-icon has-text-danger">
                            <i class="mdi mdi-alert-circle"></i>
                        </span>
                        <div class="validation-message">
                            <strong>Error:</strong> ${this.escapeHtml(error)}
                        </div>
                    </div>
                `;
            });
            
            // Display warnings
            warnings.forEach(warning => {
                html += `
                    <div class="validation-item warning">
                        <span class="icon validation-icon has-text-warning">
                            <i class="mdi mdi-alert"></i>
                        </span>
                        <div class="validation-message">
                            <strong>Warning:</strong> ${this.escapeHtml(warning)}
                        </div>
                    </div>
                `;
            });
            
            // Display success message if valid despite warnings
            if (result.valid && warnings.length > 0) {
                html = `
                    <div class="validation-item info">
                        <span class="icon validation-icon has-text-info">
                            <i class="mdi mdi-information"></i>
                        </span>
                        <div class="validation-message">
                            <strong>Flow is valid with warnings</strong>
                        </div>
                    </div>
                ` + html;
            }
            
            resultsContainer.innerHTML = html;
        }
        
        // Clear the summary area since we have specific results
        const summaryElement = document.getElementById('results-summary');
        if (summaryElement) {
            summaryElement.innerHTML = '';
        }
    }

    /**
     * Clear validation results
     */
    clearValidationResults() {
        const resultsContainer = document.getElementById('validation-results');
        if (resultsContainer) {
            resultsContainer.innerHTML = `
                <div class="validation-item info">
                    <span class="icon validation-icon has-text-grey">
                        <i class="mdi mdi-information-outline"></i>
                    </span>
                    <div class="validation-message">
                        Click "Validate" to check your TAFL flow for errors and warnings.
                    </div>
                </div>
            `;
        }
        this.validationResults = null;
    }

    /**
     * Validate individual card parameters
     */
    validateCardParameters(verb, params) {
        const errors = [];
        const warnings = [];

        // Get verb definition
        const verbDef = this.getVerbDefinition(verb);
        if (!verbDef) {
            warnings.push(`Unknown verb: ${verb}`);
            return { valid: true, errors, warnings };
        }

        // Check required parameters
        if (verbDef.required) {
            verbDef.required.forEach(param => {
                if (!params[param] || params[param] === '') {
                    errors.push(`Missing required parameter: ${param}`);
                }
            });
        }

        // Validate parameter types (basic validation)
        if (verbDef.parameters) {
            Object.entries(params).forEach(([key, value]) => {
                const paramDef = verbDef.parameters[key];
                if (paramDef) {
                    // Type validation
                    if (paramDef.type && value !== null && value !== undefined && value !== '') {
                        const valid = this.validateParameterType(value, paramDef.type);
                        if (!valid) {
                            warnings.push(`Parameter "${key}" might have incorrect type. Expected: ${paramDef.type}`);
                        }
                    }
                }
            });
        }

        return {
            valid: errors.length === 0,
            errors,
            warnings
        };
    }

    /**
     * Validate parameter type
     */
    validateParameterType(value, type) {
        switch (type) {
            case 'string':
                return typeof value === 'string';
            case 'number':
                return typeof value === 'number' || !isNaN(Number(value));
            case 'boolean':
                return typeof value === 'boolean' || value === 'true' || value === 'false';
            case 'array':
                return Array.isArray(value);
            case 'object':
                return typeof value === 'object' && !Array.isArray(value);
            default:
                return true;
        }
    }

    /**
     * Get verb definition (placeholder - should sync with actual verb definitions)
     */
    getVerbDefinition(verb) {
        // This should ideally come from the same source as the verb toolbox
        // For now, return basic definitions
        const definitions = {
            query: {
                required: ['target'],
                parameters: {
                    target: { type: 'string' },
                    where: { type: 'object' },
                    limit: { type: 'number' },
                    order: { type: 'string' },
                    as: { type: 'string' }
                }
            },
            check: {
                required: ['condition'],
                parameters: {
                    condition: { type: 'string' },
                    where: { type: 'object' },
                    as: { type: 'string' }
                }
            },
            create: {
                required: ['target'],
                parameters: {
                    target: { type: 'string' },
                    data: { type: 'object' },
                    as: { type: 'string' }
                }
            },
            update: {
                required: ['target', 'id'],
                parameters: {
                    target: { type: 'string' },
                    id: { type: 'string' },
                    data: { type: 'object' },
                    where: { type: 'object' }
                }
            },
            delete: {
                required: ['target'],
                parameters: {
                    target: { type: 'string' },
                    id: { type: 'string' },
                    where: { type: 'object' }
                }
            },
            'if': {
                required: ['condition'],
                parameters: {
                    condition: { type: 'string' },
                    then: { type: 'array' },
                    else: { type: 'array' }
                }
            },
            'for': {
                required: ['each', 'in', 'do'],
                parameters: {
                    each: { type: 'string' },
                    in: { type: 'string' },
                    do: { type: 'array' }
                }
            },
            'switch': {
                required: ['on'],
                parameters: {
                    on: { type: 'string' },
                    cases: { type: 'object' },
                    default: { type: 'array' }
                }
            },
            set: {
                parameters: {
                    // Dynamic parameters
                }
            },
            log: {
                parameters: {
                    message: { type: 'string' },
                    level: { type: 'string' }
                }
            },
            notify: {
                required: ['message'],
                parameters: {
                    message: { type: 'string' },
                    type: { type: 'string' },
                    channel: { type: 'string' }
                }
            },
            wait: {
                required: ['duration'],
                parameters: {
                    duration: { type: 'number' },
                    unit: { type: 'string' }
                }
            },
            http: {
                required: ['url'],
                parameters: {
                    url: { type: 'string' },
                    method: { type: 'string' },
                    headers: { type: 'object' },
                    data: { type: 'object' },
                    as: { type: 'string' }
                }
            }
        };

        return definitions[verb];
    }

    /**
     * Validate flow structure
     */
    validateFlowStructure() {
        const errors = [];
        const warnings = [];
        const flow = taflFlowStore.getFlow();

        // Check metadata
        if (!flow.metadata || !flow.metadata.id) {
            errors.push('Flow metadata is missing ID');
        }
        if (!flow.metadata || !flow.metadata.name) {
            warnings.push('Flow metadata is missing name');
        }

        // Check flow array
        if (!flow.flow || !Array.isArray(flow.flow)) {
            errors.push('Flow must contain a "flow" array');
        } else if (flow.flow.length === 0) {
            warnings.push('Flow is empty');
        }

        // Check for circular variable references
        if (flow.variables) {
            this.checkCircularVariables(flow.variables, warnings);
        }

        return {
            valid: errors.length === 0,
            errors,
            warnings
        };
    }

    /**
     * Check for circular variable references
     */
    checkCircularVariables(variables, warnings) {
        // Simple check for now - could be enhanced
        Object.entries(variables).forEach(([key, value]) => {
            if (typeof value === 'string' && value.includes(`$${key}`)) {
                warnings.push(`Potential circular reference in variable: ${key}`);
            }
        });
    }

    /**
     * Get last validation results
     */
    getLastValidationResults() {
        return this.validationResults;
    }

    /**
     * Check if validation is in progress
     */
    isValidating() {
        return this.validationInProgress;
    }

    /**
     * HTML escape utility
     */
    escapeHtml(text) {
        const div = document.createElement('div');
        div.textContent = text;
        return div.innerHTML;
    }
}

// Create singleton instance
const taflEditorValidator = new TAFLEditorValidator();

export default taflEditorValidator;
export { TAFLEditorValidator };