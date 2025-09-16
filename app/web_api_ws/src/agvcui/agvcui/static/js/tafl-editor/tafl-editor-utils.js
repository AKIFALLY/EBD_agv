/**
 * TAFL Editor Utility Functions Module
 * Pure utility functions with no side effects
 */

class TAFLEditorUtils {
    constructor() {
        // Verb definitions cache (will be set by editor)
        this.verbDefinitions = {};
    }

    /**
     * Set verb definitions for utility functions
     */
    setVerbDefinitions(definitions) {
        this.verbDefinitions = definitions;
    }

    /**
     * Generate unique ID for cards
     * Delegates to store for consistent counter management
     */
    generateId() {
        // Use the store's generateId if available
        if (window.taflFlowStore && window.taflFlowStore.generateId) {
            const result = window.taflFlowStore.generateId();
            // Store's generateId returns {id, newCounter}, extract the id
            if (typeof result === 'object' && result.id) {
                // The store already updated the counter, just return the id
                return result.id;
            }
            // Fallback if store returns unexpected format
            return result;
        }
        // Fallback to simple generation with better uniqueness
        // Using performance.now() for microsecond precision + random string
        return `card_${Math.floor(performance.now() * 1000)}_${Math.random().toString(36).substr(2, 9)}`;
    }

    /**
     * Escape HTML to prevent XSS
     */
    escapeHtml(text) {
        if (!text) return '';
        const div = document.createElement('div');
        div.textContent = text;
        return div.innerHTML;
    }

    /**
     * Format parameter name for display
     */
    formatParamName(name) {
        if (!name) return '';
        // Convert snake_case or camelCase to Title Case
        return name
            .replace(/_/g, ' ')
            .replace(/([A-Z])/g, ' $1')
            .trim()
            .replace(/\b\w/g, l => l.toUpperCase());
    }

    /**
     * Format parameter value for display
     */
    formatParamValue(value) {
        if (value === null || value === undefined) return '';
        if (typeof value === 'object') return JSON.stringify(value, null, 2);
        return String(value);
    }

    /**
     * Get help text for a parameter
     */
    getParamHelp(param) {
        // Parameter help mapping
        const helpTexts = {
            'expression': 'Enter a TAFL expression (e.g., $variable > 10)',
            'condition': 'Boolean expression that evaluates to true/false',
            'target': 'Target entity or resource identifier',
            'filter': 'Filter criteria in JSON format',
            'timeout': 'Timeout value in seconds',
            'max_retries': 'Maximum number of retry attempts',
            'delay': 'Delay in milliseconds',
            'message': 'Message to display or send',
            'level': 'Log level (info, warning, error)',
            'id': 'Unique identifier',
            'name': 'Display name or identifier',
            'value': 'Value to set or compare'
        };
        
        return helpTexts[param] ? `<p class="help">${helpTexts[param]}</p>` : '';
    }

    /**
     * Get card title based on verb and parameters
     */
    getCardTitle(cardData) {
        if (!cardData) return 'Unknown';
        
        const verb = cardData.verb || Object.keys(cardData).find(key => key !== 'id' && key !== 'comment');
        if (!verb) return 'Unknown';
        
        const params = cardData[verb];
        
        // Special formatting for different verbs
        switch (verb) {
            case 'query':
                // TAFL v1.1.1: target + where + as
                return `Query: ${params?.target || 'undefined'}`;
            case 'create':
                // TAFL v1.1.1: target + with + as
                return `Create: ${params?.target || 'undefined'}`;
            case 'update':
                // TAFL v1.1.1: target + where + set
                return `Update: ${params?.target || 'undefined'}`;
            case 'delete':
                return `Delete: ${params?.target || 'undefined'}`;
            case 'check':
                // TAFL v1.1.1: target + condition + as
                return `Check: ${this.truncateExpression(params?.condition)}`;
            case 'if':
                return `If: ${this.truncateExpression(params?.condition)}`;
            case 'switch':
                // TAFL v1.1.1: expression (not 'on')
                return `Switch: ${this.truncateExpression(params?.expression || params?.on)}`;
            case 'for':
                // TAFL v1.1.1: in + as + do (not 'each')
                if (params?.in && params?.as) {
                    return `For: ${params.as} in ${this.truncateExpression(params.in)}`;
                }
                // Legacy format fallback
                return `For: ${params?.each || 'item'} in ${params?.in || '[]'}`;
            case 'while':
                return `While: ${this.truncateExpression(params?.condition)}`;
            case 'set':
                // Always use object format for SET cards
                if (typeof params === 'object' && params !== null) {
                    return `Set: ${Object.keys(params).join(', ')}`;
                }
                // Fallback for any legacy string format
                return `Set: variables`;
            case 'notify':
                // TAFL v1.1.1: level is primary field
                return `Notify [${params?.level || 'info'}]`;
            case 'log':
                return `Log: ${params?.level || 'info'}`;
            case 'wait':
                return `Wait: ${params?.seconds || params?.delay || '0'}s`;
            case 'retry':
                return `Retry: ${params?.max_attempts || '3'} times`;
            default:
                // Title case the verb
                return verb.charAt(0).toUpperCase() + verb.slice(1);
        }
    }

    /**
     * Truncate expression for display
     */
    truncateExpression(expr, maxLength = 25) {
        if (!expr) return 'undefined';
        const str = String(expr);
        if (str.length <= maxLength) return str;
        return str.substring(0, maxLength) + '...';
    }

    /**
     * Get default parameters for a verb
     */
    getDefaultParams(verb) {
        if (!verb) return {};
        
        // Define fallback defaults for all verbs (TAFL v1.1.1 compliant)
        const fallbackDefaults = {
            'query': { 
                target: 'tasks',
                where: {},
                as: 'query_result'  // TAFL v1.1.1: use 'as' not 'store_as'
            },
            'create': { 
                target: 'task',
                with: {},  // TAFL v1.1.1: use 'with' not 'params'
                as: 'created_item'
            },
            'update': { 
                target: 'task',
                where: { id: '${task_id}' },  // TAFL v1.1.1: where is for conditions
                set: { status: 'updated' }     // TAFL v1.1.1: set is for changes
            },
            'check': { 
                target: 'system',
                condition: '${system.ready == true}',
                as: 'check_result'  // TAFL v1.1.1: use 'as' not 'store_as'
            },
            'if': { 
                condition: 'true',
                then: [],
                else: []
            },
            'switch': { 
                expression: '${variable}',
                cases: []  // TAFL v1.1.1: array of {when, do}, default is a special case with when: 'default'
            },
            'for': { 
                in: '${items}',      // TAFL v1.1.1: use 'in' not 'each'
                as: 'item',          // TAFL v1.1.1: use 'as' for loop variable
                do: [],              // TAFL v1.1.1: use 'do' for body
                filter: null         // TAFL v1.1.1: optional filter
            },
            'set': { variable1: 'value1' },  // Always use object format for consistency
            'notify': { 
                level: 'info',       // TAFL v1.1.1: level is primary field
                message: 'Notification message',
                recipients: [],
                details: {}
            },
            'stop': { 
                reason: 'Flow stopped',
                condition: null      // TAFL v1.1.1: optional condition
            }
        };
        
        // Start with fallback defaults
        let params = fallbackDefaults[verb] || {};
        
        // If verb definitions exist and have params, merge/override
        if (this.verbDefinitions[verb]) {
            const definition = this.verbDefinitions[verb];
            
            // Only override if there are params defined
            // Check both 'params' and 'required_params' for compatibility
            const paramsList = definition.params || definition.required_params;
            if (paramsList && paramsList.length > 0) {
                const definitionParams = {};
                paramsList.forEach(param => {
                    const value = this.getDefaultValue(param, definition);
                    // Only set if we have a non-empty value, otherwise keep fallback
                    if (value !== '' && value !== undefined && value !== null) {
                        definitionParams[param] = value;
                    } else if (params[param] === undefined) {
                        // If fallback doesn't have it either, set the empty value
                        definitionParams[param] = value;
                    }
                });
                
                // Merge with fallback defaults (definition takes precedence for non-empty values)
                params = { ...params, ...definitionParams };
            }
        }
        
        return params;
    }

    /**
     * Get default value for a parameter
     */
    getDefaultValue(param, definition) {
        // Check for specific default in definition
        if (definition.params && definition.params[param] && 
            definition.params[param].default !== undefined) {
            return definition.params[param].default;
        }
        
        // Common parameter defaults
        const paramDefaults = {
            'target': 'locations',
            'condition': 'true',
            'expression': '$variable',
            'message': '',
            'level': 'info',
            'seconds': 1,
            'delay': 1000,
            'timeout': 30,
            'max_retries': 3,
            'max_attempts': 3,
            'limit': 10,
            'each': 'item',
            'in': '[]',
            'do': [],
            'then': [],
            'else': [],
            'cases': [],
            'data': {}
        };
        
        return paramDefaults[param] || '';
    }

    /**
     * Check if a value is empty
     */
    isEmpty(value) {
        if (value === null || value === undefined) return true;
        if (typeof value === 'string') return value.trim() === '';
        if (Array.isArray(value)) return value.length === 0;
        if (typeof value === 'object') return Object.keys(value).length === 0;
        return false;
    }

    /**
     * Deep clone an object
     */
    deepClone(obj) {
        if (obj === null || typeof obj !== 'object') return obj;
        if (obj instanceof Date) return new Date(obj);
        if (Array.isArray(obj)) return obj.map(item => this.deepClone(item));
        
        const cloned = {};
        for (const key in obj) {
            if (obj.hasOwnProperty(key)) {
                cloned[key] = this.deepClone(obj[key]);
            }
        }
        return cloned;
    }

    /**
     * Get nested value from object using dot notation
     */
    getNestedValue(obj, path, defaultValue = undefined) {
        if (!path) return obj;
        
        const keys = path.split('.');
        let result = obj;
        
        for (const key of keys) {
            if (result === null || result === undefined) {
                return defaultValue;
            }
            result = result[key];
        }
        
        return result !== undefined ? result : defaultValue;
    }

    /**
     * Set nested value in object using dot notation
     */
    setNestedValue(obj, path, value) {
        if (!path) return;
        
        const keys = path.split('.');
        const lastKey = keys.pop();
        
        let target = obj;
        for (const key of keys) {
            if (!(key in target) || typeof target[key] !== 'object') {
                target[key] = {};
            }
            target = target[key];
        }
        
        target[lastKey] = value;
    }
}

// Export singleton instance
const taflEditorUtils = new TAFLEditorUtils();

export default taflEditorUtils;
export { TAFLEditorUtils };