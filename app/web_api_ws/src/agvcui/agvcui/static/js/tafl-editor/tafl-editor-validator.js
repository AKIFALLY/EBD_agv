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