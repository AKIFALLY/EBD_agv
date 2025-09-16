/**
 * TAFL Editor Events Module
 * Handles all event bindings and interactions
 */

class TAFLEditorEvents {
    constructor(editor) {
        this.editor = editor;  // Reference to main TAFLEditor instance
        this.shortcuts = {
            'Ctrl+S': () => window.taflModals?.showSaveFlowModal(),
            'Ctrl+Z': () => this.editor.undo(),
            'Ctrl+Y': () => this.editor.redo(),
            'Delete': () => this.handleDeleteKey(),
            'Ctrl+A': () => this.selectAllCards(),
            'Escape': () => this.handleEscapeKey(),
            'F2': () => this.editFlowId(),
            '?': () => this.showShortcutsHelp()
        };
    }

    /**
     * Initialize all event listeners
     */
    initEventListeners() {
        // Toolbar buttons
        this.initToolbarEvents();
        
        // Metadata inputs
        this.initMetadataEvents();
        
        // Settings inputs
        this.initSettingsEvents();
        
        // Canvas events
        this.initCanvasEvents();
        
        // View mode tabs
        this.initViewModeEvents();
        
        // Properties tabs
        this.initPropertiesTabs();
        
        // Keyboard shortcuts
        this.initKeyboardShortcuts();
        
        // Verb category filters
        this.initVerbFilters();
        
        // YAML editor events
        this.initYamlEditorEvents();
        
        // Modal controls
        if (window.taflModals && window.taflModals.setupModalControls) {
            window.taflModals.setupModalControls();
        }
        
        // Window resize
        window.addEventListener('resize', () => {
            if (this.editor.debounce) {
                this.editor.debounce(() => {
                    this.editor.updateCanvas();
                }, 200)();
            }
        });
    }

    /**
     * Initialize toolbar button events
     */
    initToolbarEvents() {
        const newFlowBtn = document.getElementById('new-flow-btn');
        if (newFlowBtn) {
            newFlowBtn.addEventListener('click', () => window.taflModals?.newFlow());
        }
        
        const saveFlowBtn = document.getElementById('save-flow-btn');
        if (saveFlowBtn) {
            saveFlowBtn.addEventListener('click', () => window.taflModals?.showSaveFlowModal());
        }
        
        const validateBtn = document.getElementById('validate-btn');
        if (validateBtn) {
            validateBtn.addEventListener('click', () => this.editor.validatorModule.validateFlow());
        }
        
        // Test Run button (simulation)
        const testRunBtn = document.getElementById('test-run-btn');
        if (testRunBtn) {
            testRunBtn.addEventListener('click', async () => {
                // Get flow name for modal display
                const flowData = window.taflFlowStore.getFlow();
                const flowName = flowData.metadata?.name || flowData.metadata?.id || 'Unnamed Flow';
                
                // Show test run confirmation modal
                window.taflPanels.showExecuteFlowModal('simulation', flowName, async () => {
                    if (this.editor && this.editor.flowModule) {
                        try {
                            await this.editor.testRun();
                        } catch (error) {
                            console.error('Test run error:', error);
                            if (this.editor.showNotification) {
                                this.editor.showNotification(`Test run failed: ${error.message}`, 'is-warning');
                            }
                        }
                    } else {
                        console.error('Editor not available for test run');
                    }
                });
            });
        }
        
        // Execute button (real execution)
        const executeBtn = document.getElementById('execute-btn');
        if (executeBtn) {
            executeBtn.addEventListener('click', async () => {
                // Get flow name for modal display
                const flowData = window.taflFlowStore.getFlow();
                const flowName = flowData.metadata?.name || flowData.metadata?.id || 'Unnamed Flow';
                
                // Show execution confirmation modal
                window.taflPanels.showExecuteFlowModal('real', flowName, async () => {
                    if (this.editor && this.editor.flowModule) {
                        try {
                            await this.editor.executeFlow();
                        } catch (error) {
                            console.error('Execution error:', error);
                            if (this.editor.showNotification) {
                                this.editor.showNotification(`Execution failed: ${error.message}`, 'is-danger');
                            }
                        }
                    } else {
                        console.error('Editor or flowModule not available', {
                            editor: this.editor,
                            flowModule: this.editor?.flowModule
                        });
                        // Fallback to global window.taflEditor
                        if (window.taflEditor && typeof window.taflEditor.executeFlow === 'function') {
                            await window.taflEditor.executeFlow();
                        } else {
                            console.error('No valid editor instance found for execution');
                            if (this.editor?.showNotification) {
                                this.editor.showNotification('Flow execution is not available. Please refresh the page.', 'is-danger');
                            }
                        }
                    }
                });
            });
        }
    }

    /**
     * Initialize metadata input events
     */
    initMetadataEvents() {
        const flowId = document.getElementById('flow-id');
        if (flowId) {
            const handleFlowIdChange = (e) => {
                window.taflFlowStore.updateMetadata({ id: e.target.value });
                this.editor.markDirty();
            };
            // Listen to both input and change events for reliability
            flowId.addEventListener('input', handleFlowIdChange);
            flowId.addEventListener('change', handleFlowIdChange);
        }
        
        const flowName = document.getElementById('flow-name');
        if (flowName) {
            const handleFlowNameChange = (e) => {
                window.taflFlowStore.updateMetadata({ name: e.target.value });
                this.editor.markDirty();
            };
            // Listen to both input and change events for reliability
            flowName.addEventListener('input', handleFlowNameChange);
            flowName.addEventListener('change', handleFlowNameChange);
        }
        
        const flowVersion = document.getElementById('flow-version');
        if (flowVersion) {
            const handleFlowVersionChange = (e) => {
                window.taflFlowStore.updateMetadata({ version: e.target.value });
                this.editor.markDirty();
            };
            // Listen to both input and change events for reliability
            flowVersion.addEventListener('input', handleFlowVersionChange);
            flowVersion.addEventListener('change', handleFlowVersionChange);
        }
        
        const flowDescription = document.getElementById('flow-description');
        if (flowDescription) {
            const handleFlowDescriptionChange = (e) => {
                window.taflFlowStore.updateMetadata({ description: e.target.value });
                this.editor.markDirty();
            };
            // Listen to both input and change events for reliability
            flowDescription.addEventListener('input', handleFlowDescriptionChange);
            flowDescription.addEventListener('change', handleFlowDescriptionChange);
        }
        
        const flowEnabled = document.getElementById('flow-enabled');
        if (flowEnabled) {
            const handleFlowEnabledChange = (e) => {
                window.taflFlowStore.updateMetadata({ enabled: e.target.checked });
                this.editor.markDirty();
            };
            // Listen to change event for checkbox
            flowEnabled.addEventListener('change', handleFlowEnabledChange);
        }
    }

    /**
     * Initialize settings input events
     */
    initSettingsEvents() {
        const settingsTimeout = document.getElementById('settings-timeout');
        if (settingsTimeout) {
            const handleTimeoutChange = (e) => {
                const flow = window.taflFlowStore.getFlow();
                window.taflFlowStore.updateFlow({ 
                    ...flow, 
                    settings: { 
                        ...flow.settings, 
                        timeout: parseInt(e.target.value, 10) 
                    } 
                });
                this.editor.markDirty();
            };
            // Listen to both input and change events for reliability
            settingsTimeout.addEventListener('input', handleTimeoutChange);
            settingsTimeout.addEventListener('change', handleTimeoutChange);
        }
        
        const settingsMaxRetries = document.getElementById('settings-max-retries');
        if (settingsMaxRetries) {
            const handleMaxRetriesChange = (e) => {
                const flow = window.taflFlowStore.getFlow();
                window.taflFlowStore.updateFlow({ 
                    ...flow, 
                    settings: { 
                        ...flow.settings, 
                        max_retries: parseInt(e.target.value, 10) 
                    } 
                });
                this.editor.markDirty();
            };
            // Listen to both input and change events for reliability
            settingsMaxRetries.addEventListener('input', handleMaxRetriesChange);
            settingsMaxRetries.addEventListener('change', handleMaxRetriesChange);
        }
        
        const settingsRetryOnFailure = document.getElementById('settings-retry-on-failure');
        if (settingsRetryOnFailure) {
            settingsRetryOnFailure.addEventListener('change', (e) => {
                const flow = window.taflFlowStore.getFlow();
                window.taflFlowStore.updateFlow({ 
                    ...flow, 
                    settings: { 
                        ...flow.settings, 
                        retry_on_failure: e.target.checked 
                    } 
                });
                this.editor.markDirty();
            });
        }
    }

    /**
     * Initialize canvas click events
     */
    initCanvasEvents() {
        const flowCanvas = document.getElementById('flow-canvas');
        if (flowCanvas) {
            flowCanvas.addEventListener('click', (e) => {
                // Click on empty space to deselect
                if (e.target === flowCanvas || 
                    e.target.classList.contains('cards-container') ||
                    e.target.classList.contains('canvas-drop-zone') ||
                    e.target.classList.contains('tafl-canvas')) {
                    // Use deselectCard if available, otherwise use cardsModule.clearSelection
                    if (this.editor.deselectCard) {
                        this.editor.deselectCard();
                    } else if (this.editor.cardsModule && this.editor.cardsModule.clearSelection) {
                        this.editor.cardsModule.clearSelection();
                    }
                }
            });
        }
    }

    /**
     * Initialize view mode tab events
     */
    initViewModeEvents() {
        const viewModeTabs = document.querySelectorAll('#view-mode-tabs button');
        viewModeTabs.forEach(tab => {
            tab.addEventListener('click', (e) => {
                this.editor.setViewMode(e.target.dataset.mode);
            });
        });
    }

    /**
     * Initialize properties panel tabs
     */
    initPropertiesTabs() {
        // Add a small delay to ensure DOM is ready
        setTimeout(() => {
            const propTabs = document.querySelectorAll('.tabs li[data-tab]');
            console.log('Initializing properties tabs:', propTabs.length, 'tabs found');
            
            propTabs.forEach(tab => {
                tab.addEventListener('click', (e) => {
                    e.preventDefault();
                    e.stopPropagation();
                    const tabName = tab.dataset.tab;
                    console.log('Tab clicked:', tabName);
                    if (tabName && this.editor && this.editor.switchPropertiesTab) {
                        this.editor.switchPropertiesTab(tabName);
                    }
                });
            });
        }, 100);
    }

    /**
     * Initialize verb category filters
     */
    initVerbFilters() {
        // Panel tabs for verb categories
        document.querySelectorAll('.panel-tabs a').forEach(tab => {
            tab.addEventListener('click', (e) => {
                e.preventDefault();
                this.editor.switchVerbCategory(tab.dataset.tab);
            });
        });
        
        // Verb items (toolbox) - click to add
        document.querySelectorAll('.verb-item').forEach(item => {
            item.addEventListener('click', () => {
                const verb = item.dataset.verb;
                console.log('🔍 DEBUG-1: Verb clicked:', verb);
                console.log('🔍 DEBUG-2: Editor instance:', this.editor);
                console.log('🔍 DEBUG-2a: Editor addCard method:', this.editor?.addCard);
                this.editor.addCard(verb);
            });
        });
        
        // Legacy verb filter links if they exist
        const filterLinks = document.querySelectorAll('.verb-filter');
        filterLinks.forEach(item => {
            item.addEventListener('click', () => {
                const category = item.dataset.category;
                this.editor.filterVerbsByCategory(category);
            });
        });
    }

    /**
     * Initialize YAML editor events
     */
    initYamlEditorEvents() {
        const applyYamlBtn = document.getElementById('apply-yaml-btn');
        if (applyYamlBtn) {
            applyYamlBtn.addEventListener('click', () => {
                this.editor.yamlModule.applyYamlChanges();
            });
        }
        
        const refreshYamlBtn = document.getElementById('refresh-yaml-btn');
        if (refreshYamlBtn) {
            refreshYamlBtn.addEventListener('click', () => {
                this.editor.yamlModule.refreshYaml();
            });
        }
    }

    /**
     * Initialize keyboard shortcuts
     */
    initKeyboardShortcuts() {
        document.addEventListener('keydown', (e) => {
            // Skip if typing in input field
            if (e.target.tagName === 'INPUT' || 
                e.target.tagName === 'TEXTAREA' || 
                e.target.contentEditable === 'true') {
                return;
            }
            
            // Build key combination string
            let key = '';
            if (e.ctrlKey || e.metaKey) key += 'Ctrl+';
            if (e.altKey) key += 'Alt+';
            if (e.shiftKey) key += 'Shift+';
            
            // Add the actual key
            if (e.key === ' ') {
                key += 'Space';
            } else if (e.key.length === 1) {
                key += e.key.toUpperCase();
            } else {
                key += e.key;
            }
            
            // Check if we have a handler for this shortcut
            if (this.shortcuts[key]) {
                this.shortcuts[key](e);
            }
        });
    }

    /**
     * Handle delete key press
     */
    handleDeleteKey() {
        const selectedCardId = this.editor.cardsModule.getSelectedCardId();
        if (selectedCardId) {
            if (confirm('Delete selected card?')) {
                this.editor.cardsModule.deleteCard(selectedCardId);
            }
        }
    }

    /**
     * Handle escape key press
     */
    handleEscapeKey() {
        // Clear selection
        this.editor.cardsModule.clearSelection();
        
        // Close any open modals
        const modals = document.querySelectorAll('.modal.is-active');
        modals.forEach(modal => {
            modal.classList.remove('is-active');
        });
    }


    /**
     * Select all cards
     */
    selectAllCards() {
        const cards = document.querySelectorAll('.tafl-card');
        cards.forEach(card => {
            card.classList.add('selected');
        });
        
        // Update the store if needed
        if (cards.length > 0) {
            const lastCard = cards[cards.length - 1];
            const cardId = lastCard.dataset.cardId;
            if (cardId) {
                window.taflFlowStore?.setSelectedCardId(cardId);
            }
        }
    }
    
    /**
     * Edit flow ID - focus on the flow ID input
     */
    editFlowId() {
        const flowIdInput = document.getElementById('flow-id');
        if (flowIdInput) {
            flowIdInput.focus();
            flowIdInput.select();
        }
    }
    
    /**
     * Show keyboard shortcuts help
     */
    showShortcutsHelp() {
        const shortcuts = [
            { key: 'Ctrl+S', action: 'Save Flow' },
            { key: 'Ctrl+Z', action: 'Undo' },
            { key: 'Ctrl+Y', action: 'Redo' },
            { key: 'Delete', action: 'Delete Selected Card' },
            { key: 'Ctrl+A', action: 'Select All Cards' },
            { key: 'Escape', action: 'Deselect' },
            { key: 'F2', action: 'Edit Flow ID' },
            { key: '?', action: 'Show Keyboard Shortcuts' }
        ];
        
        let helpHtml = '<div class="shortcuts-help">';
        helpHtml += '<h3>Keyboard Shortcuts</h3>';
        helpHtml += '<table class="table is-striped">';
        helpHtml += '<thead><tr><th>Key</th><th>Action</th></tr></thead>';
        helpHtml += '<tbody>';
        
        shortcuts.forEach(shortcut => {
            helpHtml += `<tr>
                <td><kbd>${shortcut.key}</kbd></td>
                <td>${shortcut.action}</td>
            </tr>`;
        });
        
        helpHtml += '</tbody></table></div>';
        
        // Show in a modal if taflModals is available
        if (window.taflModals && window.taflModals.showCustomModal) {
            window.taflModals.showCustomModal('Keyboard Shortcuts', helpHtml);
        } else {
            // Fallback to alert
            console.log('Keyboard Shortcuts:', shortcuts);
            alert('Keyboard shortcuts:\n' + shortcuts.map(s => `${s.key}: ${s.action}`).join('\n'));
        }
    }

    /**
     * Add custom event listener
     */
    addEventListener(eventType, handler) {
        document.addEventListener(eventType, handler);
    }

    /**
     * Remove event listener
     */
    removeEventListener(eventType, handler) {
        document.removeEventListener(eventType, handler);
    }

    /**
     * Trigger custom event
     */
    triggerEvent(eventType, detail = {}) {
        const event = new CustomEvent(eventType, { detail });
        document.dispatchEvent(event);
    }

    /**
     * Clean up event listeners
     */
    cleanup() {
        // Remove window resize listener
        window.removeEventListener('resize', this.windowResizeHandler);
        
        // Clean up any other global listeners if needed
    }
}

// Export singleton instance
const taflEditorEvents = new TAFLEditorEvents(null);

export default taflEditorEvents;
export { TAFLEditorEvents };