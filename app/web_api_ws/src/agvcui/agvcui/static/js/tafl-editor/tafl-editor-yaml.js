/**
 * TAFL Editor YAML Module
 * Handles all YAML generation, parsing, and editing functionality
 */

import { taflFlowStore } from './tafl-editor-store.js';
import taflNotifications from './tafl-notifications.js';

class TAFLEditorYaml {
    constructor(editor) {
        this.editor = editor;  // Reference to main TAFLEditor instance
        this.yamlEditor = null; // CodeMirror instance
    }

    /**
     * Define custom TAFL overlay mode for variable highlighting
     */
    defineTAFLMode() {
        if (typeof CodeMirror === 'undefined') return;
        
        // Check if mode already exists to avoid redefinition
        if (CodeMirror.modes["yaml-tafl"]) {
            return;
        }
        
        // Define TAFL overlay mode
        CodeMirror.defineMode("yaml-tafl", function(config, parserConfig) {
            const yamlMode = CodeMirror.getMode(config, "yaml");
            const taflOverlay = {
                token: function(stream, state) {
                    // Match TAFL variable patterns
                    // $variable, $variable.property, $array[index], ${expression}
                    
                    // Match ${...} expression syntax
                    if (stream.match(/\$\{[^}]+\}/)) {
                        return "tafl-expression";
                    }
                    
                    // Match $variable with optional property/array access
                    // Examples: $locations, $locations[0], $locations[0].id, $tasks.count
                    if (stream.match(/\$[a-zA-Z_]\w*(\[[^\]]+\])?(\.[a-zA-Z_]\w*)*/)) {
                        return "tafl-variable";
                    }
                    
                    // Skip to next potential match
                    while (stream.next() != null && !stream.match(/\$/, false)) {}
                    return null;
                }
            };
            
            return CodeMirror.overlayMode(yamlMode, taflOverlay);
        });
    }

    /**
     * Initialize YAML CodeMirror editor
     */
    initYamlEditor(textareaId) {
        const yamlTextarea = document.getElementById(textareaId);
        if (!yamlTextarea) {
            console.warn('YAML textarea not found:', textareaId);
            return;
        }

        // Check if CodeMirror is available
        if (typeof CodeMirror === 'undefined') {
            console.error('CodeMirror is not loaded');
            return;
        }

        // Define custom TAFL mode if not already defined
        this.defineTAFLMode();
        
        // Try to use custom mode, fallback to standard yaml if needed
        let mode = 'yaml';
        if (CodeMirror.modes["yaml-tafl"]) {
            mode = 'yaml-tafl';
        }

        this.yamlEditor = window.CodeMirror.fromTextArea(yamlTextarea, {
            mode: mode,  // Use custom TAFL overlay mode if available
            theme: 'default',  // Default CodeMirror theme
            lineNumbers: true,
            lineWrapping: true,
            foldGutter: true,
            gutters: ['CodeMirror-linenumbers', 'CodeMirror-foldgutter'],
            indentUnit: 2,
            tabSize: 2,
            autoCloseBrackets: true,
            matchBrackets: true,
            height: '100%'
        });

        // Flag to prevent marking dirty during programmatic updates
        this.isProgrammaticUpdate = false;
        
        // Mark dirty on YAML changes (only for user edits)
        this.yamlEditor.on('change', () => {
            // Only mark dirty if this is a user change, not a programmatic update
            if (!this.isProgrammaticUpdate) {
                this.editor.markDirty();
            }
        });

        // Set size to fill container
        this.yamlEditor.setSize(null, '100%');
    }

    /**
     * Refresh YAML editor content
     */
    refreshYaml() {
        if (!this.yamlEditor) return;
        
        // Set flag to prevent marking dirty during programmatic update
        this.isProgrammaticUpdate = true;
        this.yamlEditor.setValue(this.generateYaml());
        // Reset flag after update
        this.isProgrammaticUpdate = false;
    }

    /**
     * Clean card IDs from flow data recursively
     * Removes all 'id' fields that start with 'card_'
     * Preserves TAFL syntax-required IDs
     */
    cleanCardIds(data) {
        if (Array.isArray(data)) {
            // Process array
            return data.map(item => this.cleanCardIds(item));
        } else if (data && typeof data === 'object') {
            // Process object
            const cleaned = {};
            for (const [key, value] of Object.entries(data)) {
                // Skip card IDs at the statement level
                if (key === 'id' && typeof value === 'string' && value.startsWith('card_')) {
                    continue; // Skip this card ID
                }
                // Recursively clean all other values
                cleaned[key] = this.cleanCardIds(value);
            }
            return cleaned;
        } else {
            // Return other data types as-is
            return data;
        }
    }

    /**
     * Generate YAML from current flow
     */
    generateYaml() {
        // Generate TAFL v1.1 compliant YAML
        const flow = taflFlowStore.getFlow();
        console.log('📄 generateYaml - flow from store:', flow);
        console.log('📄 generateYaml - flow.flow array:', flow.flow);
        console.log('📄 generateYaml - flow.flow length:', flow.flow ? flow.flow.length : 'undefined');
        
        // Create clean metadata - include all non-null values
        const metadata = {};
        
        // Include enabled field (always include, as it's important for TAFL WCS)
        metadata.enabled = flow.metadata.enabled !== false;  // Default to true
        
        // Include id if it exists
        if (flow.metadata.id) {
            metadata.id = flow.metadata.id;
        }
        // Only include non-null, non-empty values
        if (flow.metadata.name && flow.metadata.name !== 'Untitled Flow') {
            metadata.name = flow.metadata.name;
        }
        if (flow.metadata.version) {
            metadata.version = flow.metadata.version;
        }
        if (flow.metadata.description && flow.metadata.description.trim()) {
            metadata.description = flow.metadata.description;
        }
        
        const flowData = {
            metadata: metadata
        };
        
        // Add optional sections if they have content
        if (flow.rules && Object.keys(flow.rules).length > 0) {
            flowData.rules = flow.rules;
        }
        
        if (flow.preload && Object.keys(flow.preload).length > 0) {
            flowData.preload = flow.preload;
        }
        
        if (flow.settings) {
            // Only include non-default settings
            const settings = {};
            if (flow.settings.execution_interval && flow.settings.execution_interval !== 5) {
                settings.execution_interval = flow.settings.execution_interval;
            }
            if (Object.keys(settings).length > 0) {
                flowData.settings = settings;
            }
        }
        
        if (flow.variables && Object.keys(flow.variables).length > 0) {
            flowData.variables = flow.variables;
        }
        
        // Add flow section - clean card IDs recursively
        console.log('📄 Before cleanCardIds - flow.flow:', JSON.stringify(flow.flow, null, 2));
        flowData.flow = this.cleanCardIds(flow.flow);
        console.log('📄 After cleanCardIds - flowData.flow:', JSON.stringify(flowData.flow, null, 2));
        
        // Use js-yaml to convert to YAML
        try {
            const yamlOutput = jsyaml.dump(flowData, {
                indent: 2,
                lineWidth: -1,
                noRefs: true,
                sortKeys: false,
                skipInvalid: true,  // Skip undefined, null values
                quotingType: '"',   // Force double quotes for consistency
                forceQuotes: true   // Always quote string values
            });
            console.log('Generated YAML:', yamlOutput.substring(0, 200));
            return yamlOutput;
        } catch (error) {
            console.error('Error generating YAML:', error);
            return '# Error generating YAML\n# ' + error.message;
        }
    }

    /**
     * Parse YAML string to flow object
     */
    parseYaml(yamlString) {
        try {
            const parsed = jsyaml.load(yamlString);
            
            // Extract metadata from either top level or metadata object
            const metadata = parsed.metadata || {
                id: parsed.id || 'untitled',
                name: parsed.name || 'Untitled Flow',
                version: parsed.version || '1.0',
                description: parsed.description || ''
            };
            
            // Convert parsed YAML back to flow format
            const flow = {
                metadata: metadata,
                flow: [],  // Use 'flow' not 'steps' - this is TAFL standard!
                settings: parsed.settings || {},
                preload: parsed.preload || {},
                rules: parsed.rules || {},
                variables: parsed.variables || {}
            };
            
            // Convert flow array (not steps!)
            if (parsed.flow && Array.isArray(parsed.flow)) {
                flow.flow = parsed.flow;  // Keep the original format
            }
            
            return flow;
        } catch (error) {
            console.error('Error parsing YAML:', error);
            throw error;
        }
    }

    /**
     * Apply YAML changes to flow
     */
    applyYamlChanges() {
        // Set a flag for testing
        window._applyYAMLCalled = true;
        
        if (!this.yamlEditor) {
            console.warn('No YAML editor initialized');
            window._applyYAMLError = 'No YAML editor';
            return;
        }
        
        const yamlContent = this.yamlEditor.getValue();
        console.log('Applying YAML content:', yamlContent.substring(0, 200));
        window._yamlContent = yamlContent;
        
        try {
            // Parse YAML and update flow
            const flow = this.parseYaml(yamlContent);
            console.log('Parsed flow:', flow);
            
            // Update store with proper structure (flow.flow is already in correct format)
            const updateData = {
                metadata: flow.metadata,
                flow: flow.flow,  // Use 'flow' not 'steps'
                settings: flow.settings,
                preload: flow.preload,
                rules: flow.rules,
                variables: flow.variables
            };
            console.log('Updating store with:', updateData);
            taflFlowStore.updateFlow(updateData);
            
            // Trigger canvas update through editor
            this.editor.updateCanvas({ 
                fullRefresh: true, 
                preserveScroll: false,  // YAML changes might be significant
                source: 'yaml:apply' 
            });
            
            // Re-initialize drag handlers after rebuilding
            this.editor.refreshCardDragHandlers();
            
            // Show success message
            window._applyYAMLSuccess = true;
            taflNotifications.success('YAML changes applied successfully');
        } catch (error) {
            console.error('Error applying YAML changes:', error);
            window._applyYAMLError = error.message;
            taflNotifications.error('Error parsing YAML: ' + error.message);
        }
    }

    /**
     * Handle CodeMirror refresh when switching to YAML view
     */
    refreshEditor() {
        if (this.yamlEditor) {
            setTimeout(() => this.yamlEditor.refresh(), 100);
        }
    }
}

// Export class only - instance will be created by TAFLEditor
export default TAFLEditorYaml;
export { TAFLEditorYaml };