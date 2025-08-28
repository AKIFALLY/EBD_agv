/**
 * TAFL Editor Enhancements
 * 鍵盤快捷鍵和增強功能
 */

export class TAFLEditorEnhancements {
    constructor(taflEditor = null) {
        this.taflEditor = taflEditor || window.taflEditor;
        this.shortcuts = {
            'Ctrl+S': { action: 'save', description: 'Save Flow' },
            'Ctrl+N': { action: 'new', description: 'New Flow' },
            'Ctrl+D': { action: 'duplicate', description: 'Duplicate Card' },
            'Delete': { action: 'delete', description: 'Delete Selected Card' },
            // 'Ctrl+Z': { action: 'undo', description: 'Undo' },  // TODO: Implement undo functionality
            // 'Ctrl+Y': { action: 'redo', description: 'Redo' },  // TODO: Implement redo functionality
            'Ctrl+A': { action: 'selectAll', description: 'Select All Cards' },
            'Escape': { action: 'deselect', description: 'Deselect' },
            'F2': { action: 'rename', description: 'Rename Flow' },
            '?': { action: 'showHelp', description: 'Show Keyboard Shortcuts' }
        };
        
        this.isHelpVisible = false;
        this.init();
    }
    
    init() {
        this.bindKeyboardEvents();
        this.createHelpPanel();
        this.createShortcutIndicators();
        this.createShortcutHintBox();
    }
    
    bindKeyboardEvents() {
        document.addEventListener('keydown', (e) => {
            // Skip if user is typing in input/textarea
            if (e.target.tagName === 'INPUT' || 
                e.target.tagName === 'TEXTAREA' || 
                e.target.contentEditable === 'true') {
                return;
            }
            
            const key = this.getKeyCombo(e);
            const shortcut = this.shortcuts[key];
            
            if (shortcut) {
                e.preventDefault();
                this.handleShortcut(shortcut.action);
            }
        });
    }
    
    getKeyCombo(e) {
        const parts = [];
        if (e.ctrlKey) parts.push('Ctrl');
        if (e.altKey) parts.push('Alt');
        if (e.shiftKey) parts.push('Shift');
        
        // Special keys
        if (e.key === 'Delete') {
            parts.push('Delete');
        } else if (e.key === 'Escape') {
            parts.push('Escape');
        } else if (e.key === 'F2') {
            parts.push('F2');
        } else if (e.key === '?') {
            return '?';
        } else if (e.key.length === 1) {
            parts.push(e.key.toUpperCase());
        }
        
        return parts.join('+');
    }
    
    handleShortcut(action) {
        console.log('Shortcut action:', action);
        
        switch(action) {
            case 'save':
                // Trigger save button click
                const saveBtn = document.querySelector('.tafl-toolbar .button:has(.fa-save)');
                if (saveBtn) saveBtn.click();
                break;
                
            case 'new':
                // Trigger new button click
                const newBtn = document.querySelector('.tafl-toolbar .button:has(.fa-plus)');
                if (newBtn) newBtn.click();
                break;
                
            case 'delete':
                // Delete selected card
                if (this.taflEditor && this.taflEditor.selectedCard) {
                    this.taflEditor.deleteCard(this.taflEditor.selectedCard);
                }
                break;
                
            case 'duplicate':
                // Duplicate selected card
                if (this.taflEditor && this.taflEditor.selectedCard) {
                    this.duplicateCard();
                }
                break;
                
            case 'selectAll':
                // Select all cards
                if (this.taflEditor) {
                    const cards = document.querySelectorAll('.tafl-card');
                    cards.forEach(card => card.classList.add('selected'));
                }
                break;
                
            case 'deselect':
                // Deselect all
                if (this.taflEditor) {
                    this.taflEditor.selectedCard = null;
                    document.querySelectorAll('.tafl-card.selected')
                        .forEach(card => card.classList.remove('selected'));
                }
                break;
                
            case 'rename':
                // Focus on flow name input
                const nameInput = document.querySelector('input[placeholder="Flow Name"]');
                if (nameInput) {
                    nameInput.focus();
                    nameInput.select();
                }
                break;
                
            case 'showHelp':
                this.toggleHelp();
                break;
                
            // Undo/Redo cases removed until implementation is ready
            // case 'undo':
            //     // TODO: Implement undo functionality
            //     console.log('Undo not yet implemented');
            //     this.showToast('Undo functionality coming soon');
            //     break;
            //     
            // case 'redo':
            //     // TODO: Implement redo functionality
            //     console.log('Redo not yet implemented');
            //     this.showToast('Redo functionality coming soon');
            //     break;
        }
    }
    
    duplicateCard() {
        if (!this.taflEditor || !this.taflEditor.selectedCard) return;
        
        const selectedCard = this.taflEditor.selectedCard;
        const cardData = this.taflEditor.getCardData(selectedCard);
        
        if (cardData) {
            // Create duplicate with new ID
            const duplicateData = JSON.parse(JSON.stringify(cardData));
            duplicateData.id = this.taflEditor.generateId();
            
            // Add to flow
            this.taflEditor.addCardFromData(duplicateData);
            this.showToast('Card duplicated');
        }
    }
    
    createHelpPanel() {
        const helpPanel = document.createElement('div');
        helpPanel.id = 'keyboard-shortcuts-help';
        helpPanel.className = 'modal';
        helpPanel.innerHTML = `
            <div class="modal-background"></div>
            <div class="modal-card">
                <header class="modal-card-head">
                    <p class="modal-card-title">
                        <span class="icon">
                            <i class="fas fa-keyboard"></i>
                        </span>
                        <span>Keyboard Shortcuts</span>
                    </p>
                    <button class="delete" aria-label="close"></button>
                </header>
                <section class="modal-card-body">
                    <table class="table is-fullwidth">
                        <thead>
                            <tr>
                                <th>Shortcut</th>
                                <th>Action</th>
                            </tr>
                        </thead>
                        <tbody>
                            ${Object.entries(this.shortcuts).map(([key, info]) => `
                                <tr>
                                    <td><kbd>${key}</kbd></td>
                                    <td>${info.description}</td>
                                </tr>
                            `).join('')}
                        </tbody>
                    </table>
                </section>
                <footer class="modal-card-foot">
                    <button class="button" id="help-close-btn">Close</button>
                </footer>
            </div>
        `;
        
        document.body.appendChild(helpPanel);
        
        // Bind event listeners properly
        helpPanel.querySelector('.modal-background').addEventListener('click', () => this.toggleHelp());
        helpPanel.querySelector('.delete').addEventListener('click', () => this.toggleHelp());
        helpPanel.querySelector('#help-close-btn').addEventListener('click', () => this.toggleHelp());
    }
    
    toggleHelp() {
        const modal = document.getElementById('keyboard-shortcuts-help');
        if (modal) {
            this.isHelpVisible = !this.isHelpVisible;
            modal.classList.toggle('is-active', this.isHelpVisible);
        }
    }
    
    createShortcutIndicators() {
        // Add tooltips to toolbar buttons showing shortcuts
        const buttons = [
            { selector: '.button:has(.fa-plus)', shortcut: 'Ctrl+N' },
            { selector: '.button:has(.fa-save)', shortcut: 'Ctrl+S' }
        ];
        
        buttons.forEach(btn => {
            const element = document.querySelector(`.tafl-toolbar ${btn.selector}`);
            if (element) {
                element.title = `${element.title || ''} (${btn.shortcut})`;
            }
        });
        
        // Add help button to toolbar
        const toolbar = document.querySelector('.tafl-toolbar .level-right');
        if (toolbar && !document.querySelector('#help-button')) {
            const helpButton = document.createElement('div');
            helpButton.className = 'level-item';
            helpButton.innerHTML = `
                <button class="button is-light" id="help-button">
                    <span class="icon">
                        <i class="fas fa-question-circle"></i>
                    </span>
                    <span>Shortcuts</span>
                </button>
            `;
            toolbar.insertBefore(helpButton, toolbar.firstChild);
            
            // Bind event listener properly
            document.getElementById('help-button').addEventListener('click', () => this.toggleHelp());
        }
    }
    
    createShortcutHintBox() {
        // Create fixed position hint box in bottom-left corner
        const hintBox = document.createElement('div');
        hintBox.id = 'keyboard-hint-box';
        hintBox.className = 'keyboard-hint-box';
        hintBox.innerHTML = `
            <div class="hint-content">
                <span class="icon">
                    <i class="fas fa-keyboard"></i>
                </span>
                <span class="hint-text">Press <kbd>?</kbd> for shortcuts</span>
            </div>
        `;
        
        document.body.appendChild(hintBox);
        
        // Make the hint box clickable to open help
        hintBox.addEventListener('click', () => this.toggleHelp());
    }
    
    showToast(message, type = 'info') {
        // Remove existing toast
        const existingToast = document.querySelector('.toast-notification');
        if (existingToast) {
            existingToast.remove();
        }
        
        // Create new toast
        const toast = document.createElement('div');
        toast.className = `toast-notification notification is-${type}`;
        toast.style.cssText = `
            position: fixed;
            top: 80px;
            right: 20px;
            z-index: 9999;
            min-width: 250px;
            animation: slideIn 0.3s ease-out;
        `;
        toast.innerHTML = message;
        
        document.body.appendChild(toast);
        
        // Auto remove after 3 seconds
        setTimeout(() => {
            toast.style.animation = 'slideOut 0.3s ease-out';
            setTimeout(() => toast.remove(), 300);
        }, 3000);
    }
}

// Add animations
const style = document.createElement('style');
style.textContent = `
    @keyframes slideIn {
        from {
            transform: translateX(100%);
            opacity: 0;
        }
        to {
            transform: translateX(0);
            opacity: 1;
        }
    }
    
    @keyframes slideOut {
        from {
            transform: translateX(0);
            opacity: 1;
        }
        to {
            transform: translateX(100%);
            opacity: 0;
        }
    }
    
    kbd {
        background: #f4f4f4;
        border: 1px solid #ccc;
        border-radius: 3px;
        padding: 2px 6px;
        font-family: monospace;
        font-size: 0.9em;
        box-shadow: 0 1px 0 rgba(0, 0, 0, 0.1);
    }
    
    #keyboard-shortcuts-help .modal-card {
        width: 500px;
    }
    
    .keyboard-hint-box {
        position: fixed;
        bottom: 20px;
        left: 20px;
        background: rgba(32, 32, 32, 0.9);
        color: #fff;
        padding: 10px 15px;
        border-radius: 8px;
        font-size: 0.9rem;
        z-index: 1000;
        cursor: pointer;
        transition: all 0.3s ease;
        box-shadow: 0 2px 10px rgba(0, 0, 0, 0.3);
    }
    
    .keyboard-hint-box:hover {
        background: rgba(48, 48, 48, 0.95);
        transform: translateY(-2px);
        box-shadow: 0 4px 15px rgba(0, 0, 0, 0.4);
    }
    
    .keyboard-hint-box .hint-content {
        display: flex;
        align-items: center;
        gap: 8px;
    }
    
    .keyboard-hint-box .icon {
        color: #3273dc;
    }
    
    .keyboard-hint-box kbd {
        margin: 0 3px;
    }
`;

// Export style injection function
export function injectEnhancementsStyles() {
    if (!document.getElementById('tafl-enhancements-styles')) {
        style.id = 'tafl-enhancements-styles';
        document.head.appendChild(style);
    }
}

// Export default
export default TAFLEditorEnhancements;