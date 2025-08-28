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
            return result;
        } catch (error) {
            console.error('Validation error:', error);
            throw new Error(`Validation failed: ${error.message}`);
        }
    }

    /**
     * Execute a flow
     * @param {Object} flowData - Flow data to execute
     * @returns {Promise<Object>} Execution result
     */
    async executeFlow(flowData) {
        try {
            const response = await this._fetch(`${this.baseUrl}/execute`, {
                method: 'POST',
                headers: this.defaultHeaders,
                body: JSON.stringify(flowData)
            });
            
            const result = await response.json();
            
            if (!response.ok) {
                throw new Error(result.detail || 'Execution failed');
            }
            
            return result;
        } catch (error) {
            console.error('Execution error:', error);
            throw new Error(`Execution failed: ${error.message}`);
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
            const response = await fetch(url, options);
            return response;
        } catch (error) {
            // Network error or other fetch failures
            console.error('Fetch error:', error);
            throw new Error(`Network error: ${error.message}`);
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
                timeout: 3600, 
                max_retries: 3, 
                retry_on_failure: false 
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