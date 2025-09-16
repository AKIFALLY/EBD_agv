/**
 * TAFL Editor Execution Panel Module
 * Handles execution results display for both test run and real execution
 */

class TAFLEditorExecutionPanel {
    constructor(editor) {
        this.editor = editor;
        this.executionMode = null;
        this.lastResult = null;
    }

    /**
     * Show execution results in the panel
     * @param {Object} result - Execution result from API
     * @param {string} mode - 'simulation' or 'real'
     */
    showResults(result, mode) {
        this.executionMode = mode;
        this.lastResult = result;
        
        // Switch to results tab
        this.switchToResultsTab();
        
        // Hide validation results
        const validationDiv = document.getElementById('validation-results');
        if (validationDiv) {
            validationDiv.style.display = 'none';
        }
        
        // Determine success status - check both 'success' field and 'status' field
        const isSuccess = result.success || result.status === 'completed';
        
        // Update mode indicator
        this.updateModeIndicator(mode, isSuccess);
        
        // Display execution log
        this.displayExecutionLog(result.execution_log || []);
        
        // Display variable changes
        if (result.final_variables) {
            this.displayVariableChanges(result.final_variables);
        }
        
        // Show summary
        this.showSummary(result);
    }

    /**
     * Switch to results tab
     */
    switchToResultsTab() {
        // Update tab active state
        document.querySelectorAll('.tabs li').forEach(tab => {
            tab.classList.toggle('is-active', tab.dataset.tab === 'results');
        });
        
        // Hide all panels
        document.querySelectorAll('.properties-content').forEach(panel => {
            panel.style.display = 'none';
        });
        
        // Show results panel
        const resultsPanel = document.getElementById('results-panel');
        if (resultsPanel) {
            resultsPanel.style.display = 'block';
        }
    }

    /**
     * Update mode indicator
     */
    updateModeIndicator(mode, success) {
        const modeElement = document.getElementById('result-mode');
        if (!modeElement) return;
        
        let tagClass = '';
        let text = '';
        
        if (mode === 'simulation') {
            tagClass = success ? 'tag is-info' : 'tag is-warning';
            text = success ? '✓ Test Run Completed' : '⚠ Test Run Failed';
        } else {
            tagClass = success ? 'tag is-warning' : 'tag is-danger';
            text = success ? '✓ Real Execution Completed' : '✗ Real Execution Failed';
        }
        
        modeElement.innerHTML = `<span class="${tagClass}">${text}</span>`;
    }

    /**
     * Display execution log
     */
    displayExecutionLog(executionLog) {
        const logContainer = document.getElementById('execution-log');
        const logContent = document.querySelector('.execution-log-content');
        
        if (!logContainer || !logContent) return;
        
        if (executionLog.length === 0) {
            logContainer.style.display = 'none';
            return;
        }
        
        logContainer.style.display = 'block';
        logContent.innerHTML = '';
        
        executionLog.forEach((entry, index) => {
            const logEntry = document.createElement('div');
            logEntry.className = 'box is-small mb-2';
            
            const statusIcon = this.getStatusIcon(entry.status);
            const statusClass = this.getStatusClass(entry.status);
            
            // Format the result properly
            let formattedResult = entry.result || 'N/A';
            if (typeof formattedResult === 'object' && formattedResult !== null) {
                // Convert object to formatted JSON string
                formattedResult = JSON.stringify(formattedResult, null, 2);
            }
            
            // Format the action properly
            let formattedAction = entry.action || 'N/A';
            if (typeof formattedAction === 'object' && formattedAction !== null) {
                formattedAction = JSON.stringify(formattedAction, null, 2);
            }
            
            logEntry.innerHTML = `
                <div class="level is-mobile">
                    <div class="level-left">
                        <div class="level-item">
                            <span class="tag ${statusClass} is-light">Step ${entry.step}</span>
                        </div>
                        <div class="level-item">
                            <strong>${entry.verb}</strong>
                        </div>
                    </div>
                    <div class="level-right">
                        <span class="icon ${statusClass}">
                            <i class="${statusIcon}"></i>
                        </span>
                    </div>
                </div>
                <div class="is-size-7 mt-2">
                    ${entry.action && entry.action !== 'N/A' ? `
                        <div class="mb-2">
                            <strong>Action:</strong>
                            ${typeof entry.action === 'object' ? 
                                `<pre class="has-background-light p-2">${formattedAction}</pre>` : 
                                formattedAction
                            }
                        </div>
                    ` : ''}
                    <div>
                        <strong>Result:</strong>
                        ${typeof entry.result === 'object' ? 
                            `<pre class="has-background-light p-2">${formattedResult}</pre>` : 
                            formattedResult
                        }
                    </div>
                    ${entry.message ? `
                        <div class="mt-2">
                            <strong>Message:</strong> ${entry.message}
                        </div>
                    ` : ''}
                </div>
            `;
            
            logContent.appendChild(logEntry);
        });
    }

    /**
     * Display variable changes
     */
    displayVariableChanges(variables) {
        const changesContainer = document.getElementById('variable-changes');
        const changesContent = document.querySelector('.variable-changes-content');
        
        if (!changesContainer || !changesContent) return;
        
        const varKeys = Object.keys(variables);
        if (varKeys.length === 0) {
            changesContainer.style.display = 'none';
            return;
        }
        
        changesContainer.style.display = 'block';
        changesContent.innerHTML = '';
        
        const table = document.createElement('table');
        table.className = 'table is-fullwidth is-striped';
        table.innerHTML = `
            <thead>
                <tr>
                    <th>Variable</th>
                    <th>Final Value</th>
                </tr>
            </thead>
            <tbody>
                ${varKeys.map(key => `
                    <tr>
                        <td><code>${key}</code></td>
                        <td><code>${JSON.stringify(variables[key])}</code></td>
                    </tr>
                `).join('')}
            </tbody>
        `;
        
        changesContent.appendChild(table);
    }

    /**
     * Show execution summary
     */
    showSummary(result) {
        const resultsElement = document.getElementById('results-summary');
        if (!resultsElement) return;
        
        // Check for success - handle both 'success' field and 'status' field
        const isSuccess = result.success === true || result.status === 'completed';
        const isFailed = result.success === false || result.status === 'failed';
        
        const summaryHtml = `
            <div class="notification ${isSuccess ? 'is-light' : 'is-danger is-light'}">
                <h5 class="title is-6">${result.flow_name || 'Flow'} - ${this.executionMode === 'simulation' ? 'Test Run' : 'Execution'}</h5>
                <p class="is-size-7">
                    <strong>Status:</strong> ${isSuccess ? 'Success' : (isFailed ? 'Failed' : result.status || 'Unknown')}<br>
                    <strong>Mode:</strong> ${result.mode || this.executionMode}<br>
                    <strong>Steps:</strong> ${result.total_steps || result.execution_log?.length || 0}<br>
                    <strong>Message:</strong> ${result.message || (isSuccess ? 'Completed successfully' : 'Execution failed')}
                </p>
                ${result.error ? `<p class="has-text-danger is-size-7 mt-2"><strong>Error:</strong> ${result.error}</p>` : ''}
            </div>
        `;
        
        resultsElement.innerHTML = summaryHtml;
    }

    /**
     * Get status icon based on status
     */
    getStatusIcon(status) {
        switch (status) {
            case 'success':
                return 'fas fa-check-circle';
            case 'warning':
                return 'fas fa-exclamation-triangle';
            case 'error':
            case 'failed':
                return 'fas fa-times-circle';
            case 'completed':
                return 'fas fa-flag-checkered';
            default:
                return 'fas fa-circle';
        }
    }

    /**
     * Get status class based on status
     */
    getStatusClass(status) {
        switch (status) {
            case 'success':
                return 'has-text-success';
            case 'warning':
                return 'has-text-warning';
            case 'error':
            case 'failed':
                return 'has-text-danger';
            case 'completed':
                return 'has-text-info';
            default:
                return 'has-text-grey';
        }
    }

    /**
     * Clear execution results
     */
    clearResults() {
        const modeElement = document.getElementById('result-mode');
        if (modeElement) {
            modeElement.innerHTML = '<span class="tag is-light">No results yet</span>';
        }
        
        const resultsElement = document.getElementById('results-summary');
        if (resultsElement) {
            resultsElement.innerHTML = `
                <div class="has-text-centered">
                    <span class="icon is-large has-text-grey-light">
                        <i class="fas fa-clipboard-check fa-3x"></i>
                    </span>
                    <p class="has-text-grey">Results will appear here</p>
                    <p class="has-text-grey is-size-7 mt-2">
                        <strong>Validate:</strong> Check TAFL syntax and structure<br>
                        <strong>Test Run:</strong> Simulate execution without affecting real systems<br>
                        <strong>Execute:</strong> Run on real system and affect actual resources
                    </p>
                </div>
            `;
        }
        
        const validationDiv = document.getElementById('validation-results');
        if (validationDiv) {
            validationDiv.style.display = 'none';
        }
        
        const logContainer = document.getElementById('execution-log');
        if (logContainer) {
            logContainer.style.display = 'none';
        }
        
        const changesContainer = document.getElementById('variable-changes');
        if (changesContainer) {
            changesContainer.style.display = 'none';
        }
    }
}

// Export as singleton
const taflExecutionPanel = new TAFLEditorExecutionPanel(null);

// Make it available globally
window.taflExecutionPanel = taflExecutionPanel;

export default taflExecutionPanel;
export { TAFLEditorExecutionPanel };