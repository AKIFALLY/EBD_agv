/**
 * TAFL Editor Preload Management Module
 * Handles TAFL v1.1 preload data functionality
 */

class TAFLEditorPreload {
    constructor(editor) {
        this.editor = editor;  // Reference to main TAFLEditor instance
    }

    /**
     * Parse where condition string into object
     * @param {string} whereStr - Comma-separated conditions like "status: active, room_id: 1"
     * @returns {object} Parsed conditions object
     */
    parseWhereConditions(whereStr) {
        if (!whereStr) return {};
        
        const conditions = {};
        const parts = whereStr.split(',');
        parts.forEach(part => {
            const [key, value] = part.split(':').map(s => s.trim());
            if (key && value) {
                // Try to parse value
                let parsedValue = value;
                // Remove quotes if present
                if ((value.startsWith('"') && value.endsWith('"')) || 
                    (value.startsWith("'") && value.endsWith("'"))) {
                    parsedValue = value.slice(1, -1);
                } else if (!isNaN(value)) {
                    parsedValue = Number(value);
                } else if (value === 'true' || value === 'false') {
                    parsedValue = value === 'true';
                }
                conditions[key] = parsedValue;
            }
        });
        return conditions;
    }

    /**
     * Add a new preload data item
     */
    async addPreload() {
        // Use simple prompts for consistency
        const key = prompt('Variable name to store the preloaded data:');
        if (!key) return;
        
        const target = prompt('Query target (locations/racks/tasks/agvs/products):');
        if (!target) return;
        
        const whereStr = prompt('Where conditions (optional, e.g. "status: active, room_id: 1"):', '');
        
        // Parse where conditions
        const where = whereStr ? this.parseWhereConditions(whereStr) : {};
        
        // Create preload config
        const preloadConfig = {
            query: {
                target: target,
                ...(Object.keys(where).length > 0 && { where: where })
            }
        };
        
        // Add to flow
        const flow = window.taflFlowStore.getFlow();
        if (!flow.preload) {
            flow.preload = {};
        }
        
        // Check if key already exists
        if (flow.preload[key]) {
            if (!await window.taflModals?.confirmOverwrite('Preload', key)) {
                return;
            }
        }
        
        // Add preload query
        flow.preload[key] = preloadConfig;
        
        // Trigger update
        window.taflFlowStore.updateFlow(flow);
        this.editor.markDirty();
        if (this.editor.yamlModule) {
            this.editor.yamlModule.generateYaml();
        }
    }

    /**
     * Save a new preload from modal form
     */
    async saveNewPreload() {
        const key = document.getElementById('new-preload-key')?.value.trim();
        const target = document.getElementById('new-preload-target')?.value;
        const whereStr = document.getElementById('new-preload-where')?.value.trim();
        const comment = document.getElementById('new-preload-comment')?.value.trim();
        
        if (!key || !target) {
            if (window.taflModals) {
                window.taflModals.showRequiredFieldsError(['Store As', 'Target']);
            } else {
                alert('Store As and Target are required fields');
            }
            return;
        }
        
        // Check if key already exists
        const flow = window.taflFlowStore.getFlow();
        if (flow.preload && flow.preload[key]) {
            if (window.taflModals) {
                window.taflModals.alert(`A preload with the key "${key}" already exists`, 'warning');
            } else {
                alert(`A preload with the key "${key}" already exists`);
            }
            return;
        }
        
        // Parse where conditions
        const where = this.parseWhereConditions(whereStr);
        
        // Create preload config
        const preloadConfig = {
            query: {
                target: target,
                ...(Object.keys(where).length > 0 && { where: where }),
                ...(comment && { comment: comment })
            }
        };
        
        // Add to flow
        if (!flow.preload) {
            flow.preload = {};
        }
        
        // Add preload query
        flow.preload[key] = preloadConfig;
        
        // Update flow
        window.taflFlowStore.updateFlow(flow);
        
        // Close modal and trigger updates
        const modal = document.querySelector('.modal.is-active');
        if (modal) modal.remove();
        
        window.taflFlowStore.emit('flow:changed');
        this.editor.markDirty();
        if (this.editor.yamlModule) {
            this.editor.yamlModule.generateYaml();
        }
    }

    /**
     * Edit an existing preload item
     * @param {string} key - The preload key to edit
     */
    editPreload(key) {
        const flow = window.taflFlowStore.getFlow();
        const preload = flow.preload?.[key];
        if (!preload) return;
        
        // Show edit dialog with current values
        const modal = document.createElement('div');
        modal.className = 'modal is-active';
        modal.innerHTML = `
            <div class="modal-background" onclick="this.parentElement.remove()"></div>
            <div class="modal-card">
                <header class="modal-card-head">
                    <p class="modal-card-title">Edit Preload Query</p>
                    <button class="delete" aria-label="close" onclick="this.closest('.modal').remove()"></button>
                </header>
                <section class="modal-card-body">
                    <div class="field">
                        <label class="label">Store As (Variable Name)</label>
                        <div class="control">
                            <input class="input" type="text" id="edit-preload-key" value="${key}" placeholder="e.g., available_locations">
                        </div>
                    </div>
                    <div class="field">
                        <label class="label">Target</label>
                        <div class="control">
                            <div class="select is-fullwidth">
                                <select id="edit-preload-target">
                                    <option value="locations" ${preload.query?.target === 'locations' ? 'selected' : ''}>locations</option>
                                    <option value="racks" ${preload.query?.target === 'racks' ? 'selected' : ''}>racks</option>
                                    <option value="tasks" ${preload.query?.target === 'tasks' ? 'selected' : ''}>tasks</option>
                                    <option value="agvs" ${preload.query?.target === 'agvs' ? 'selected' : ''}>agvs</option>
                                    <option value="products" ${preload.query?.target === 'products' ? 'selected' : ''}>products</option>
                                </select>
                            </div>
                        </div>
                    </div>
                    <div class="field">
                        <label class="label">Where Conditions</label>
                        <div class="control">
                            <textarea class="textarea" id="edit-preload-where" rows="3" placeholder="e.g., room_id: 1, status: available">${
                                typeof preload.query?.where === 'string' 
                                    ? preload.query.where 
                                    : typeof preload.query?.where === 'object'
                                        ? Object.entries(preload.query.where).map(([k, v]) => `${k}: ${v}`).join(', ')
                                        : ''
                            }</textarea>
                        </div>
                        <p class="help">Format: key: value, key2: value2 (comma-separated)</p>
                    </div>
                    <div class="field">
                        <label class="label">Comment (Optional)</label>
                        <div class="control">
                            <input class="input" type="text" id="edit-preload-comment" value="${preload.query?.comment || ''}" placeholder="Description of this preload query">
                        </div>
                    </div>
                </section>
                <footer class="modal-card-foot">
                    <button class="button is-success" onclick="window.taflEditor.preloadModule.savePreloadEdit('${key}')">Save changes</button>
                    <button class="button" onclick="this.closest('.modal').remove()">Cancel</button>
                </footer>
            </div>
        `;
        document.body.appendChild(modal);
    }

    /**
     * Save preload edit from modal
     * @param {string} oldKey - The original preload key
     */
    savePreloadEdit(oldKey) {
        const newKey = document.getElementById('edit-preload-key')?.value.trim();
        const target = document.getElementById('edit-preload-target')?.value;
        const whereStr = document.getElementById('edit-preload-where')?.value.trim();
        const comment = document.getElementById('edit-preload-comment')?.value.trim();
        
        if (!newKey || !target) {
            if (window.taflModals) {
                window.taflModals.showRequiredFieldsError(['Store As', 'Target']);
            } else {
                alert('Store As and Target are required fields');
            }
            return;
        }
        
        // Parse where conditions
        const where = this.parseWhereConditions(whereStr);
        
        // Create preload config
        const preloadConfig = {
            query: {
                target: target,
                ...(Object.keys(where).length > 0 && { where: where }),
                ...(comment && { comment: comment })
            }
        };
        
        const flow = window.taflFlowStore.getFlow();
        
        // If key changed, delete old one
        if (oldKey !== newKey && flow.preload?.[oldKey]) {
            delete flow.preload[oldKey];
        }
        
        // Set new/updated preload
        if (!flow.preload) {
            flow.preload = {};
        }
        flow.preload[newKey] = preloadConfig;
        
        // Update flow
        window.taflFlowStore.updateFlow(flow);
        
        // Close modal and trigger updates
        const modal = document.querySelector('.modal.is-active');
        if (modal) modal.remove();
        
        window.taflFlowStore.emit('flow:changed');
        this.editor.markDirty();
        if (this.editor.yamlModule) {
            this.editor.yamlModule.generateYaml();
        }
    }

    /**
     * Remove a preload item
     * @param {string} key - The preload key to remove
     */
    async removePreload(key) {
        const confirmDelete = window.taflModals?.confirmDelete || 
                             ((type, name) => confirm(`Delete ${type} "${name}"?`));
        
        if (await confirmDelete('preload query', key)) {
            const flow = window.taflFlowStore.getFlow();
            if (flow.preload && flow.preload[key]) {
                delete flow.preload[key];
                
                // Update flow
                window.taflFlowStore.updateFlow(flow);
                
                // Trigger updates
                window.taflFlowStore.emit('flow:changed');
                this.editor.markDirty();
                if (this.editor.yamlModule) {
                    this.editor.yamlModule.generateYaml();
                }
                
                if (this.editor.showNotification) {
                    this.editor.showNotification('Preload removed successfully', 'success');
                }
            }
        }
    }

    /**
     * Get all preload items
     * @returns {object} All preload configurations
     */
    getAllPreloads() {
        const flow = window.taflFlowStore.getFlow();
        return flow.preload || {};
    }

    /**
     * Check if a preload key exists
     * @param {string} key - The preload key to check
     * @returns {boolean} True if the preload exists
     */
    hasPreload(key) {
        const flow = window.taflFlowStore.getFlow();
        return flow.preload && flow.preload.hasOwnProperty(key);
    }

    /**
     * Get a specific preload configuration
     * @param {string} key - The preload key
     * @returns {object|undefined} The preload configuration
     */
    getPreload(key) {
        const flow = window.taflFlowStore.getFlow();
        return flow.preload?.[key];
    }

    /**
     * Clear all preload items
     */
    clearAllPreloads() {
        const flow = window.taflFlowStore.getFlow();
        if (flow.preload) {
            flow.preload = {};
            
            // Update flow
            window.taflFlowStore.updateFlow(flow);
            
            // Trigger updates
            window.taflFlowStore.emit('flow:changed');
            this.editor.markDirty();
            if (this.editor.yamlModule) {
                this.editor.yamlModule.generateYaml();
            }
            
            if (this.editor.showNotification) {
                this.editor.showNotification('All preloads cleared', 'info');
            }
        }
    }
}

// Export singleton instance
const taflEditorPreload = new TAFLEditorPreload(null);

export default taflEditorPreload;
export { TAFLEditorPreload };