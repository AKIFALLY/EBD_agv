/**
 * TAFL Editor Notifications Module
 * Handles all notification display and UI state updates
 */

class TAFLEditorNotifications {
    constructor(editor) {
        this.editor = editor;  // Reference to main TAFLEditor instance
        this.isDirty = false;
    }

    /**
     * Mark the editor as having unsaved changes
     */
    markDirty() {
        this.isDirty = true;
        this.updateSaveButton(true);
    }

    /**
     * Clear the dirty state
     */
    clearDirty() {
        this.isDirty = false;
        this.updateSaveButton(false);
    }

    /**
     * Get current dirty state
     */
    getDirtyState() {
        return this.isDirty;
    }

    /**
     * Update save button state based on dirty flag
     */
    updateSaveButton(isDirty) {
        // Try both possible button IDs
        const saveBtn = document.getElementById('save-flow-btn') || document.getElementById('saveBtn');
        if (saveBtn) {
            if (isDirty) {
                saveBtn.removeAttribute('disabled');  // Enable the button when there are changes
                saveBtn.classList.add('is-warning');
                const icon = saveBtn.querySelector('.icon i') || saveBtn.querySelector('i');
                if (icon) {
                    icon.classList.remove('fa-save');
                    icon.classList.add('fa-exclamation-triangle');
                }
                // Update text if button has text content
                // Get the text span (not the icon span)
                const textSpans = saveBtn.querySelectorAll('span');
                const textNode = textSpans.length > 1 ? textSpans[1] : saveBtn;
                if (textNode && textNode.textContent) {
                    // Only update if not already showing "Save Changes"
                    if (!textNode.textContent.includes('Changes')) {
                        textNode.textContent = 'Save Changes';
                    }
                }
            } else {
                saveBtn.setAttribute('disabled', 'disabled');  // Disable the button when no changes
                saveBtn.classList.remove('is-warning');
                const icon = saveBtn.querySelector('.icon i') || saveBtn.querySelector('i');
                if (icon) {
                    icon.classList.remove('fa-exclamation-triangle');
                    icon.classList.add('fa-save');
                }
                // Update text if button has text content
                // Get the text span (not the icon span)
                const textSpans = saveBtn.querySelectorAll('span');
                const textNode = textSpans.length > 1 ? textSpans[1] : saveBtn;
                if (textNode && textNode.textContent) {
                    // Reset to "Save" regardless of how many "Changes" were added
                    textNode.textContent = 'Save';
                }
            }
        }
    }

    /**
     * Update UI elements
     */
    updateUI() {
        // Update flow name display
        const flowNameElement = document.getElementById('flowName');
        if (flowNameElement && this.editor) {
            const flow = this.editor.taflFlowStore?.getFlow();
            if (flow) {
                const flowName = flow.metadata?.name || 'Untitled Flow';
                const flowId = flow.metadata?.id || 'untitled';
                flowNameElement.textContent = `${flowName} (${flowId})`;
            }
        }

        // Update save button based on dirty state
        this.updateSaveButton(this.isDirty);

        // Update any other UI elements as needed
        // This can be extended to update more UI components
    }

    /**
     * Show notification message
     * @param {string} message - Message to display
     * @param {string} type - Bulma notification type (is-info, is-success, is-warning, is-danger)
     */
    showNotification(message, type = 'is-info') {
        // Always use the original tafl-notifications for consistent styling
        if (window.taflNotifications && window.taflNotifications.show) {
            // Convert Bulma type to tafl-notifications type
            const typeMap = {
                'is-success': 'success',
                'is-warning': 'warning', 
                'is-danger': 'error',
                'is-error': 'error',
                'is-info': 'info'
            };
            const notificationType = typeMap[type] || type.replace('is-', '') || 'info';
            window.taflNotifications.show(message, notificationType);
        } else {
            // Simple console fallback if notification system not available
            console.log(`[${type}] ${message}`);
        }
    }

    /**
     * Convenience methods for different notification types
     */
    success(message) {
        // Prefer original tafl-notifications for consistent styling
        if (window.taflNotifications && window.taflNotifications.success) {
            window.taflNotifications.success(message);
        } else {
            this.showNotification(message, 'is-success');
        }
    }

    error(message) {
        // Prefer original tafl-notifications for consistent styling
        if (window.taflNotifications && window.taflNotifications.error) {
            window.taflNotifications.error(message);
        } else {
            this.showNotification(message, 'is-danger');
        }
    }

    warning(message) {
        // Prefer original tafl-notifications for consistent styling
        if (window.taflNotifications && window.taflNotifications.warning) {
            window.taflNotifications.warning(message);
        } else {
            this.showNotification(message, 'is-warning');
        }
    }

    info(message) {
        // Prefer original tafl-notifications for consistent styling
        if (window.taflNotifications && window.taflNotifications.info) {
            window.taflNotifications.info(message);
        } else {
            this.showNotification(message, 'is-info');
        }
    }

    /**
     * Clear all notifications
     */
    clearAll() {
        // Use original tafl-notifications clear if available
        if (window.taflNotifications && window.taflNotifications.clear) {
            window.taflNotifications.clear();
        }
    }
}

// Export singleton instance
const taflEditorNotifications = new TAFLEditorNotifications(null);

export default taflEditorNotifications;
export { TAFLEditorNotifications };