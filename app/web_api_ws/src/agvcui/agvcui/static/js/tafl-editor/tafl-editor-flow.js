/**
 * TAFL Editor Flow Management Module
 * Handles flow loading, saving, execution, and management
 */

class TAFLEditorFlow {
    constructor(editor) {
        this.editor = editor;  // Reference to main TAFLEditor instance
    }

    /**
     * Load a flow from data or ID
     * @param {string|object} flowDataOrId - Flow ID string or flow data object
     */
    async loadFlow(flowDataOrId) {
        // Handle both flow data object and flow ID string
        if (typeof flowDataOrId === 'string') {
            // It's a flow ID, fetch the flow
            await this.openFlow(flowDataOrId);
        } else if (typeof flowDataOrId === 'object' && flowDataOrId !== null) {
            // It's already flow data
            let flowToLoad;
            if (flowDataOrId.flow) {
                // The response has a wrapper
                flowToLoad = flowDataOrId.flow;
            } else {
                // Direct flow data
                flowToLoad = flowDataOrId;
            }
            
            // Ensure flow has all required sections
            const normalizedFlow = {
                metadata: flowToLoad.metadata || { id: null, name: 'Untitled Flow', version: '1.1', description: '' },
                settings: flowToLoad.settings || { timeout: 3600, max_retries: 3, retry_on_failure: false },
                preload: flowToLoad.preload || {},
                rules: flowToLoad.rules || {},
                variables: flowToLoad.variables || {},
                flow: flowToLoad.flow || []
            };
            
            // First ensure all cards have IDs BEFORE loading to store
            // Force regenerate IDs for backend flows to ensure uniqueness
            if (normalizedFlow.flow && Array.isArray(normalizedFlow.flow)) {
                console.log('🔧 [loadFlow] Before ensureAllCardsHaveIds:', JSON.stringify(normalizedFlow.flow[0]?.id));
                // Pass true to force regenerate all IDs since backend returns duplicate IDs
                normalizedFlow.flow = this.ensureAllCardsHaveIds(normalizedFlow.flow, true);
                console.log('🔧 [loadFlow] After ensureAllCardsHaveIds:', JSON.stringify(normalizedFlow.flow[0]?.id));
                console.log('🔧 [loadFlow] First card full data:', normalizedFlow.flow[0]);
                console.log('🔧 [loadFlow] All regenerated card IDs:', normalizedFlow.flow.map(c => c.id));
            }
            
            // Now load the flow with IDs already in place
            window.taflFlowStore.loadFlow(normalizedFlow);
            
            // Manual refresh after open flow
            this.editor.updateCanvas({
                fullRefresh: true,
                preserveScroll: false,
                source: 'flow:open'
            });
            
            // Update UI
            this.editor.updateUI();
            window.taflFlowStore.setDirty(false);
            
            // Ensure properties panel is properly initialized after loading flow
            if (window.taflPanelsProperties) {
                console.log('🔧 Reinitializing properties panel after flow load');
                window.taflPanelsProperties.reinitialize();
            }
            
            const flowName = window.taflFlowStore.getFlow().metadata?.name || window.taflFlowStore.getFlow().name || 'Flow';
            this.editor.showNotification(`Loaded flow: ${flowName}`, 'is-success');
        } else {
            console.error('Invalid flow data or ID provided');
        }
    }

    /**
     * Open a flow by ID from the API
     * @param {string} flowId - Flow ID to load
     */
    async openFlow(flowId) {
        try {
            const data = await window.taflAPI.getFlow(flowId);
            let normalizedFlow = data.flow;
            
            // Ensure all cards have IDs before loading
            // Force regenerate IDs for backend flows to ensure uniqueness
            if (normalizedFlow.flow && Array.isArray(normalizedFlow.flow)) {
                console.log('🔧 [openFlow] Before ensureAllCardsHaveIds:', JSON.stringify(normalizedFlow.flow[0]?.id));
                // Pass true to force regenerate all IDs since backend returns duplicate IDs
                normalizedFlow.flow = this.ensureAllCardsHaveIds(normalizedFlow.flow, true);
                console.log('🔧 [openFlow] After ensureAllCardsHaveIds:', JSON.stringify(normalizedFlow.flow[0]?.id));
                console.log('🔧 [openFlow] First card full data:', normalizedFlow.flow[0]);
                console.log('🔧 [openFlow] All regenerated card IDs:', normalizedFlow.flow.map(c => c.id));
            }
            
            window.taflFlowStore.loadFlow(normalizedFlow);
            // flow:loaded event will trigger refresh
            // UI updates handled by SimpleTAFLPanels via store events
            this.editor.updateUI();
            window.taflFlowStore.setDirty(false);
            
            // Ensure properties panel is properly initialized after opening flow
            if (window.taflPanelsProperties) {
                console.log('🔧 Reinitializing properties panel after flow open');
                window.taflPanelsProperties.reinitialize();
            }
            
            this.editor.showNotification(`Opened flow: ${normalizedFlow.metadata.name}`, 'is-success');
        } catch (error) {
            this.editor.showNotification(error.message, 'is-danger');
        }
    }

    /**
     * Execute the current flow
     */
    async testRun() {
        try {
            const result = await window.taflAPI.testRun({
                metadata: window.taflFlowStore.getFlow().metadata,
                variables: window.taflFlowStore.getFlow().variables,
                flow: window.taflFlowStore.getFlow().flow
            });
            
            if (result.success) {
                this.editor.showNotification(`Test run completed: ${result.total_steps} steps simulated`, 'is-info');
                console.log('Simulation log:', result.execution_log);
                
                // Show execution results if panel exists
                if (window.taflExecutionPanel) {
                    window.taflExecutionPanel.showResults(result, 'simulation');
                }
            } else {
                this.editor.showNotification(`Test run failed: ${result.message}`, 'is-warning');
            }
        } catch (error) {
            this.editor.showNotification(error.message, 'is-danger');
        }
    }

    async executeFlow() {
        try {
            console.log('[TAFL Flow] Starting execution...');
            
            // Get complete flow data including all sections for 4-phase execution
            const flowData = window.taflFlowStore.getFlow();
            console.log('[TAFL Flow] Flow data to execute:', flowData);
            
            // Validate flow has content
            if (!flowData.flow || flowData.flow.length === 0) {
                throw new Error('Flow is empty. Please add at least one statement.');
            }
            
            const result = await window.taflAPI.executeFlow({
                metadata: flowData.metadata,
                settings: flowData.settings || {},
                preload: flowData.preload || {},
                rules: flowData.rules || {},
                variables: flowData.variables || {},
                flow: flowData.flow
            }, 'real');
            
            console.log('[TAFL Flow] Execution result:', result);
            
            // Check for success - API returns status: 'completed' for success
            if (result.success || result.status === 'completed') {
                this.editor.showNotification('TAFL flow executed successfully on real system', 'is-success');
                console.log('[TAFL Flow] Execution log:', result.execution_log || result.result);
                
                // Show execution results if panel exists
                if (window.taflExecutionPanel) {
                    window.taflExecutionPanel.showResults(result, 'real');
                }
            } else if (result.status === 'failed' || result.error) {
                const errorMsg = result.message || result.error || 'Execution failed';
                console.error('[TAFL Flow] Execution failed:', errorMsg, result);
                this.editor.showNotification(`Execution failed: ${errorMsg}`, 'is-danger');
            } else {
                // Unknown status
                console.warn('[TAFL Flow] Unknown execution status:', result);
                this.editor.showNotification('Execution completed with unknown status', 'is-warning');
            }
        } catch (error) {
            console.error('[TAFL Flow] Execution error caught:', {
                message: error.message,
                name: error.name,
                stack: error.stack,
                error: error
            });
            
            // Display user-friendly error message
            const errorMessage = error.message || 'Unknown error occurred during execution';
            this.editor.showNotification(errorMessage, 'is-danger');
        }
    }

    /**
     * Recursively ensure all cards have IDs (including nested cards)
     * @param {Array} cards - Array of card objects
     * @param {boolean} forceRegenerate - Force regeneration of all IDs (for backend flows)
     * @returns {Array} Cards with IDs
     */
    ensureAllCardsHaveIds(cards, forceRegenerate = false) {
        if (!Array.isArray(cards)) return cards;
        
        return cards.map(card => {
            // Create a new card object to avoid mutation
            let newCard = { ...card };
            
            // For backend flows, always regenerate IDs to ensure uniqueness
            // Backend returns identical IDs for all nested cards
            if (forceRegenerate) {
                const oldId = typeof newCard.id === 'object' ? 
                    (newCard.id.id || newCard.id.value || JSON.stringify(newCard.id)) : 
                    newCard.id;
                // Generate new ID - editor.generateId() always returns a string now
                newCard.id = this.editor.generateId();
                console.log('🔧 [ensureAllCardsHaveIds] Regenerated ID:', oldId, '->', newCard.id);
            } else {
                // Normal ID handling for locally created cards
                if (!newCard.id) {
                    newCard.id = this.editor.generateId();
                } else if (typeof newCard.id === 'object') {
                    // Fix: Extract string ID from object ID structure
                    const stringId = newCard.id.id || newCard.id.value || JSON.stringify(newCard.id);
                    console.log('🔧 [ensureAllCardsHaveIds] Converting object ID to string:', newCard.id, '->', stringId);
                    newCard.id = stringId;
                }
            }
            
            // Get the verb to find nested structures
            const verb = this.getCardVerb(newCard);
            if (verb && newCard[verb]) {
                const params = newCard[verb];
                let updatedParams = { ...params };
                
                // Process if/then/else structures
                if (verb === 'if') {
                    if (params.then && Array.isArray(params.then)) {
                        updatedParams.then = this.ensureAllCardsHaveIds(params.then, forceRegenerate);
                    }
                    if (params.else && Array.isArray(params.else)) {
                        updatedParams.else = this.ensureAllCardsHaveIds(params.else, forceRegenerate);
                    }
                    newCard[verb] = updatedParams;
                }
                // Process for/do structures
                else if (verb === 'for' && params.do && Array.isArray(params.do)) {
                    updatedParams.do = this.ensureAllCardsHaveIds(params.do, forceRegenerate);
                    newCard[verb] = updatedParams;
                }
                // Process while/do structures
                else if (verb === 'while' && params.do && Array.isArray(params.do)) {
                    updatedParams.do = this.ensureAllCardsHaveIds(params.do, forceRegenerate);
                    newCard[verb] = updatedParams;
                }
                // Process switch/cases structures
                else if (verb === 'switch') {
                    // Handle cases as object (YAML format) or array
                    if (params.cases) {
                        if (Array.isArray(params.cases)) {
                            // Cases is already an array
                            updatedParams.cases = params.cases.map(caseItem => {
                                if (caseItem.do && Array.isArray(caseItem.do)) {
                                    return {
                                        ...caseItem,
                                        do: this.ensureAllCardsHaveIds(caseItem.do, forceRegenerate)
                                    };
                                }
                                return caseItem;
                            });
                        } else if (typeof params.cases === 'object') {
                            // Cases is an object (YAML format), keep it as is
                            // Process each case value if it contains nested flow
                            const processedCases = {};
                            for (const [key, value] of Object.entries(params.cases)) {
                                if (Array.isArray(value)) {
                                    processedCases[key] = this.ensureAllCardsHaveIds(value, forceRegenerate);
                                } else {
                                    processedCases[key] = value;
                                }
                            }
                            updatedParams.cases = processedCases;
                        }
                    }
                    
                    // Process default case
                    if (params.default) {
                        if (Array.isArray(params.default)) {
                            // Default is an array of actions
                            updatedParams.default = this.ensureAllCardsHaveIds(params.default, forceRegenerate);
                        } else if (typeof params.default === 'object' && params.default.do) {
                            // Old format: default: { do: [...] }
                            // Only process if cases is an array format
                            if (Array.isArray(updatedParams.cases)) {
                                if (!updatedParams.cases) {
                                    updatedParams.cases = [];
                                }
                                // Check if default case already exists
                                const hasDefaultCase = updatedParams.cases.some(c => c.when === "default");
                                if (!hasDefaultCase) {
                                    updatedParams.cases.push({
                                        when: "default",
                                        do: params.default.do ? this.ensureAllCardsHaveIds(params.default.do, forceRegenerate) : []
                                    });
                                }
                                // Remove old format default
                                delete updatedParams.default;
                            }
                        }
                    }
                    
                    newCard[verb] = updatedParams;
                }
                // Process retry/do structures
                else if (verb === 'retry' && params.do && Array.isArray(params.do)) {
                    updatedParams.do = this.ensureAllCardsHaveIds(params.do, forceRegenerate);
                    newCard[verb] = updatedParams;
                }
            }
            
            return newCard;
        });
    }

    /**
     * Get the verb of a card (helper method)
     * @param {object} card - Card object
     * @returns {string|null} The verb key
     */
    getCardVerb(card) {
        if (!card) return null;
        // TAFL v1.1: verb is a direct key, not a property
        const keys = Object.keys(card).filter(key => key !== 'id' && key !== 'comment');
        return keys.length > 0 ? keys[0] : null;
    }

    /**
     * Create an empty flow structure
     * @returns {object} Empty flow structure
     */
    createEmptyFlow() {
        return {
            metadata: {
                id: null,
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
    }

    /**
     * Load an empty/new flow
     */
    loadEmptyFlow() {
        const emptyFlow = this.createEmptyFlow();
        window.taflFlowStore.loadFlow(emptyFlow);
        
        // Clear canvas and update UI
        this.editor.updateCanvas({
            fullRefresh: true,
            preserveScroll: false,
            source: 'flow:new'
        });
        
        this.editor.updateUI();
        window.taflFlowStore.setDirty(false);
        
        this.editor.showNotification('Created new flow', 'is-success');
    }

    /**
     * Reset the flow to initial state
     */
    resetFlow() {
        if (confirm('Are you sure you want to reset the flow? All unsaved changes will be lost.')) {
            this.loadEmptyFlow();
        }
    }

    /**
     * Check if current flow has unsaved changes
     * @returns {boolean} True if there are unsaved changes
     */
    hasUnsavedChanges() {
        return window.taflFlowStore.isDirty();
    }

    /**
     * Get the current flow data
     * @returns {object} Current flow data
     */
    getCurrentFlow() {
        return window.taflFlowStore.getFlow();
    }

    /**
     * Update flow metadata
     * @param {object} metadata - Metadata to update
     */
    updateMetadata(metadata) {
        const flow = window.taflFlowStore.getFlow();
        window.taflFlowStore.updateFlow({
            metadata: {
                ...flow.metadata,
                ...metadata
            }
        });
        this.editor.markDirty();
    }

    /**
     * Load flows into dropdown
     */
    async loadFlowsDropdown() {
        const dropdownContent = document.getElementById('flows-dropdown-content');
        if (!dropdownContent) return;
        
        try {
            const flows = await window.taflAPI.getFlows();
            
            if (flows.length === 0) {
                dropdownContent.innerHTML = `
                    <div class="dropdown-item has-text-grey">
                        No flows available
                    </div>
                `;
                return;
            }
            
            dropdownContent.innerHTML = flows.map(flow => `
                <a class="dropdown-item flow-dropdown-item" data-flow-id="${flow.id}">
                    <span class="icon is-small">
                        <i class="fas fa-file"></i>
                    </span>
                    <div class="flow-info">
                        <div class="flow-name">
                            <strong>${flow.name || flow.id}</strong>
                            <div class="flow-item-controls">
                                <span class="tag is-small ml-2">${flow.version || '1.0'}</span>
                                <button class="button is-small flow-delete-btn ml-2" 
                                        data-flow-id="${flow.id}" 
                                        data-flow-name="${flow.name || flow.id}"
                                        title="Delete flow">
                                    <span class="icon is-small">
                                        <i class="fas fa-trash"></i>
                                    </span>
                                </button>
                            </div>
                        </div>
                        ${flow.description ? `<div class="flow-desc has-text-grey">${flow.description}</div>` : ''}
                    </div>
                </a>
            `).join('');
        } catch (error) {
            console.error('Failed to load flows:', error);
            dropdownContent.innerHTML = `
                <div class="dropdown-item has-text-danger">
                    Failed to load flows
                </div>
            `;
        }
    }

    /**
     * Load first available flow from the list
     */
    async loadFirstAvailableFlow() {
        try {
            const flows = await window.taflAPI.getFlows();
            
            if (flows.length > 0) {
                // Load the first flow
                const firstFlow = flows[0];
                await this.loadFlow(firstFlow.id);
                localStorage.setItem('lastEditedFlowId', firstFlow.id);
            }
        } catch (error) {
            console.error('Failed to load first available flow:', error);
        }
    }
}

// Export singleton instance
const taflEditorFlow = new TAFLEditorFlow(null);

export default taflEditorFlow;
export { TAFLEditorFlow };