/**
 * TAFL Editor API Module
 * Centralized API communication layer for TAFL Editor
 * Handles all backend interactions with consistent error handling
 */

class TAFLEditorAPI {
    constructor() {
        this.baseUrl = '/tafl';
        this.defaultHeaders = {
            'Content-Type': 'application/json'
        };
    }

    /**
     * Load verb definitions from backend
     * @returns {Promise<Object>} Verb definitions object
     */
    async loadVerbs() {
        try {
            const response = await this._fetch(`${this.baseUrl}/verbs`);
            const data = await response.json();
            return data.verbs || {};
        } catch (error) {
            console.error('Error loading verb definitions:', error);
            throw new Error('Failed to load TAFL verbs');
        }
    }

    /**
     * Get list of all flows
     * @returns {Promise<Array>} Array of flow summaries
     */
    async getFlows() {
        try {
            const response = await this._fetch(`${this.baseUrl}/flows`);
            const data = await response.json();
            return data.flows || [];
        } catch (error) {
            console.error('Error loading flows:', error);
            throw new Error('Failed to load flows');
        }
    }

    /**
     * Get a specific flow by ID
     * @param {string} flowId - Flow identifier
     * @returns {Promise<Object>} Flow data
     */
    async getFlow(flowId) {
        try {
            const response = await this._fetch(`${this.baseUrl}/flows/${flowId}`);
            const data = await response.json();
            
            // Ensure flow has all required sections
            const normalizedFlow = this._normalizeFlow(data.flow, flowId);
            return { flow: normalizedFlow };
        } catch (error) {
            console.error(`Error loading flow ${flowId}:`, error);
            throw new Error(`Failed to load flow: ${flowId}`);
        }
    }

    /**
     * Save a flow to backend
     * @param {Object} flow - Flow data to save
     * @returns {Promise<Object>} Save result
     */
    async saveFlow(flow) {
        try {
            const response = await this._fetch(`${this.baseUrl}/flows`, {
                method: 'POST',
                headers: this.defaultHeaders,
                body: JSON.stringify(flow)
            });
            
            const result = await response.json();
            
            if (!response.ok) {
                throw new Error(result.detail || 'Save failed');
            }
            
            return result;
        } catch (error) {
            console.error('Error saving flow:', error);
            throw new Error(`Failed to save flow: ${error.message}`);
        }
    }

    /**
     * Validate a flow
     * @param {Object} flowData - Flow data to validate (metadata, variables, flow)
     * @returns {Promise<Object>} Validation result with valid, errors, warnings
     */
    async validateFlow(flowData) {
        try {
            const response = await this._fetch(`${this.baseUrl}/validate`, {
                method: 'POST',
                headers: this.defaultHeaders,
                body: JSON.stringify(flowData)
            });
            
            const result = await response.json();
            
            // 如果回應不成功，確保回傳包含必要的欄位
            if (!response.ok) {
                return {
                    valid: false,
                    errors: [result.detail || 'Validation request failed'],
                    warnings: []
                };
            }
            
            // 確保結果包含必要的欄位
            return {
                valid: result.valid !== undefined ? result.valid : false,
                errors: result.errors || [],
                warnings: result.warnings || []
            };
        } catch (error) {
            console.error('Validation error:', error);
            // 回傳標準格式的錯誤回應
            return {
                valid: false,
                errors: [`Validation failed: ${error.message}`],
                warnings: []
            };
        }
    }

    /**
     * Test run a flow in simulation mode (uses tafl_editor.py /testrun)
     * @param {Object} flowData - Flow data to test
     * @returns {Promise<Object>} Simulation result
     */
    async testRun(flowData) {
        try {
            const response = await this._fetch(`${this.baseUrl}/testrun`, {
                method: 'POST',
                headers: this.defaultHeaders,
                body: JSON.stringify(flowData)
            });
            
            const result = await response.json();
            
            if (!response.ok) {
                throw new Error(result.detail || 'Test run failed');
            }
            
            return result;
        } catch (error) {
            console.error('Test run error:', error);
            throw new Error(`Test run failed: ${error.message}`);
        }
    }

    /**
     * Execute a flow on real system (uses tafl_editor_direct.py /execute)
     * @param {Object} flowData - Flow data to execute
     * @param {string} mode - Execution mode ('real' or 'simulation')
     * @returns {Promise<Object>} Execution result
     */
    async executeFlow(flowData, mode = 'real') {
        try {
            console.log('[TAFL API] Executing flow with mode:', mode);
            console.log('[TAFL API] Flow data:', flowData);
            
            const response = await this._fetch(`${this.baseUrl}/execute`, {
                method: 'POST',
                headers: this.defaultHeaders,
                body: JSON.stringify({ ...flowData, mode })
            });
            
            // Check if response is JSON
            const contentType = response.headers.get('content-type');
            if (!contentType || !contentType.includes('application/json')) {
                console.error('[TAFL API] Non-JSON response:', contentType);
                const text = await response.text();
                console.error('[TAFL API] Response text:', text);
                throw new Error(`Server returned non-JSON response: ${text.substring(0, 100)}`);
            }
            
            let result;
            try {
                result = await response.json();
            } catch (parseError) {
                console.error('[TAFL API] JSON parse error:', parseError);
                throw new Error('Failed to parse server response as JSON');
            }
            
            if (!response.ok) {
                console.error('[TAFL API] Server error response:', result);
                throw new Error(result.detail || result.message || `Server error: ${response.status}`);
            }
            
            console.log('[TAFL API] Execution successful:', result);
            return result;
        } catch (error) {
            console.error('[TAFL API] Execution error details:', {
                name: error.name,
                message: error.message,
                stack: error.stack,
                error: error
            });
            
            // Extract meaningful error message
            let errorMessage = 'Execution failed';
            if (error.message) {
                errorMessage = error.message;
            } else if (typeof error === 'string') {
                errorMessage = error;
            } else if (error.toString && error.toString() !== '[object Object]') {
                errorMessage = error.toString();
            }
            
            throw new Error(errorMessage);
        }
    }

    /**
     * Delete a flow (for future implementation)
     * @param {string} flowId - Flow identifier to delete
     * @returns {Promise<Object>} Deletion result
     */
    async deleteFlow(flowId) {
        try {
            const response = await this._fetch(`${this.baseUrl}/flows/${flowId}`, {
                method: 'DELETE',
                headers: this.defaultHeaders
            });
            
            if (!response.ok) {
                const result = await response.json();
                throw new Error(result.detail || 'Delete failed');
            }
            
            return { success: true };
        } catch (error) {
            console.error(`Error deleting flow ${flowId}:`, error);
            throw new Error(`Failed to delete flow: ${error.message}`);
        }
    }

    /**
     * Check if a flow exists
     * @param {string} flowId - Flow identifier to check
     * @returns {Promise<boolean>} True if flow exists
     */
    async flowExists(flowId) {
        try {
            const flows = await this.getFlows();
            return flows.some(flow => flow.id === flowId);
        } catch (error) {
            console.error(`Error checking flow existence:`, error);
            return false;
        }
    }

    // ============================================
    // Private helper methods
    // ============================================

    /**
     * Wrapper around fetch with error handling
     * @private
     */
    async _fetch(url, options = {}) {
        try {
            console.log('[TAFL API] Fetching:', url, options);
            const response = await fetch(url, options);
            console.log('[TAFL API] Response status:', response.status, response.statusText);
            return response;
        } catch (error) {
            // Network error or other fetch failures
            console.error('[TAFL API] Fetch error:', {
                url: url,
                error: error,
                message: error.message,
                name: error.name
            });
            
            // Provide more descriptive error messages
            if (error.name === 'TypeError' && error.message.includes('Failed to fetch')) {
                throw new Error('Network error: Unable to connect to server. Please check if the service is running.');
            } else if (error.name === 'AbortError') {
                throw new Error('Request timeout: The server took too long to respond.');
            } else {
                throw new Error(`Network error: ${error.message || 'Unknown network error'}`);
            }
        }
    }

    /**
     * Normalize flow structure to ensure all required sections exist
     * @private
     */
    _normalizeFlow(flow, flowId) {
        return {
            metadata: flow.metadata || { 
                id: flowId, 
                name: 'Untitled Flow', 
                version: '1.1', 
                description: '' 
            },
            settings: flow.settings || {
                execution_interval: 5
            },
            preload: flow.preload || {},
            rules: flow.rules || {},
            variables: flow.variables || {},
            flow: flow.flow || []
        };
    }

    /**
     * Handle API errors consistently
     * @private
     */
    _handleError(error, context = '') {
        const message = error.message || 'Unknown error occurred';
        const fullMessage = context ? `${context}: ${message}` : message;
        
        console.error(fullMessage, error);
        
        // Return a structured error object
        return {
            success: false,
            error: message,
            context: context,
            timestamp: new Date().toISOString()
        };
    }
}

// Export singleton instance
const taflAPI = new TAFLEditorAPI();
export default taflAPI;

// Also export the class for testing purposes
export { TAFLEditorAPI };