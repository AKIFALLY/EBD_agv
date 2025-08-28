/**
 * Expression Parser for Linear Flow Designer
 * 智能表達式解析器 - 正確識別變數、運算符、常量
 */

class ExpressionParser {
    constructor() {
        // 邏輯運算符
        this.logicalOperators = ['||', '&&', '!'];
        
        // 比較運算符
        this.comparisonOperators = ['<=', '>=', '==', '!=', '<', '>', '===', '!=='];
        
        // 數學運算符
        this.mathOperators = ['+', '-', '*', '/', '%', '**'];
        
        // 括號和其他符號
        this.brackets = ['(', ')', '[', ']', '{', '}'];
        
        // 關鍵字（不應被視為變數）
        this.keywords = [
            'true', 'false', 'null', 'undefined',
            'if', 'else', 'return', 'typeof',
            'new', 'this', 'var', 'let', 'const'
        ];
        
        // JavaScript 內建全域物件（不應被視為變數）
        this.globalObjects = [
            'Math', 'Date', 'Array', 'Object', 'String', 'Number', 'Boolean',
            'JSON', 'RegExp', 'Error', 'Promise', 'Map', 'Set', 'WeakMap', 'WeakSet',
            'Symbol', 'Proxy', 'Reflect', 'Intl', 'console', 'window', 'document',
            'navigator', 'location', 'history', 'localStorage', 'sessionStorage',
            'setTimeout', 'clearTimeout', 'setInterval', 'clearInterval',
            'parseInt', 'parseFloat', 'isNaN', 'isFinite', 'encodeURI', 'decodeURI',
            'encodeURIComponent', 'decodeURIComponent', 'eval', 'alert', 'confirm', 'prompt'
        ];
    }
    
    /**
     * 從表達式中提取變數引用
     * @param {string} expression - 要解析的表達式
     * @returns {Set<string>} - 變數名稱集合
     */
    extractVariables(expression) {
        const variables = new Set();
        
        if (!expression || typeof expression !== 'string') {
            return variables;
        }
        
        // 先處理 ${...} 格式的變數引用
        const templateMatches = expression.match(/\$\{([^}]+)\}/g);
        if (templateMatches) {
            templateMatches.forEach(match => {
                // 提取 ${} 內的內容
                const innerExpr = match.slice(2, -1);
                // 遞迴解析內部表達式
                const innerVars = this.parseExpression(innerExpr);
                innerVars.forEach(v => variables.add(v));
            });
        } else {
            // 如果沒有 ${} 包裹，直接解析表達式
            const vars = this.parseExpression(expression);
            vars.forEach(v => variables.add(v));
        }
        
        return variables;
    }
    
    /**
     * 解析表達式並提取變數
     * @param {string} expr - 表達式
     * @returns {Set<string>} - 變數名稱集合
     */
    parseExpression(expr) {
        const variables = new Set();
        
        if (!expr || typeof expr !== 'string') {
            return variables;
        }
        
        // 移除字串常量（避免將字串內容誤認為變數）
        expr = this.removeStringLiterals(expr);
        
        // 分詞
        const tokens = this.tokenize(expr);
        
        // 分析每個 token
        tokens.forEach(token => {
            if (this.isVariable(token)) {
                // 提取基礎變數名（去除陣列索引和屬性存取）
                const baseVar = this.extractBaseVariableName(token);
                if (baseVar) {
                    variables.add(baseVar);
                }
            }
        });
        
        return variables;
    }
    
    /**
     * 移除字串常量
     * @param {string} expr - 表達式
     * @returns {string} - 移除字串後的表達式
     */
    removeStringLiterals(expr) {
        // 移除雙引號字串
        expr = expr.replace(/"[^"]*"/g, '""');
        // 移除單引號字串
        expr = expr.replace(/'[^']*'/g, "''");
        // 移除模板字串（如果有）
        expr = expr.replace(/`[^`]*`/g, '``');
        return expr;
    }
    
    /**
     * 分詞
     * @param {string} expr - 表達式
     * @returns {Array<string>} - token 陣列
     */
    tokenize(expr) {
        const tokens = [];
        
        // 建立運算符的正則表達式
        // 注意：需要按長度排序，長的運算符優先匹配
        const allOperators = [
            ...this.logicalOperators,
            ...this.comparisonOperators,
            ...this.mathOperators,
            ...this.brackets
        ].sort((a, b) => b.length - a.length);
        
        // 建立分割模式
        const operatorPattern = allOperators.map(op => 
            op.replace(/[.*+?^${}()|[\]\\]/g, '\\$&')  // 轉義特殊字符
        ).join('|');
        
        // 使用正則表達式分割，但保留分隔符
        const pattern = new RegExp(`(${operatorPattern}|\\s+)`);
        const parts = expr.split(pattern);
        
        // 過濾並處理 tokens
        parts.forEach(part => {
            if (part && part.trim()) {
                const trimmed = part.trim();
                if (trimmed) {
                    tokens.push(trimmed);
                }
            }
        });
        
        return tokens;
    }
    
    /**
     * 判斷 token 是否為變數
     * @param {string} token - 要檢查的 token
     * @returns {boolean}
     */
    isVariable(token) {
        // 空 token
        if (!token || token.length === 0) {
            return false;
        }
        
        // 運算符
        if (this.isOperator(token)) {
            return false;
        }
        
        // 括號
        if (this.brackets.includes(token)) {
            return false;
        }
        
        // 數字常量
        if (this.isNumber(token)) {
            return false;
        }
        
        // 字串常量（已經被移除，但再檢查一次）
        if (token.startsWith('"') || token.startsWith("'") || token.startsWith('`')) {
            return false;
        }
        
        // 布林值和 null
        if (this.keywords.includes(token)) {
            return false;
        }
        
        // JavaScript 內建全域物件
        if (this.globalObjects.includes(token)) {
            return false;
        }
        
        // 空字串
        if (token === '""' || token === "''" || token === '``') {
            return false;
        }
        
        // 其餘的視為變數（包含屬性存取和陣列索引）
        return true;
    }
    
    /**
     * 判斷是否為運算符
     * @param {string} token
     * @returns {boolean}
     */
    isOperator(token) {
        return this.logicalOperators.includes(token) ||
               this.comparisonOperators.includes(token) ||
               this.mathOperators.includes(token);
    }
    
    /**
     * 判斷是否為數字
     * @param {string} token
     * @returns {boolean}
     */
    isNumber(token) {
        // 檢查純數字（整數或浮點數）
        return /^-?\d+(\.\d+)?$/.test(token);
    }
    
    /**
     * 提取基礎變數名稱
     * @param {string} varRef - 變數引用（可能包含屬性存取或陣列索引）
     * @returns {string} - 基礎變數名稱
     */
    extractBaseVariableName(varRef) {
        // 移除陣列索引：arr[0] -> arr
        varRef = varRef.split('[')[0];
        
        // 移除屬性存取：obj.prop -> obj
        const baseName = varRef.split('.')[0];
        
        // 如果基礎名稱是全域物件，則整個表達式是全域物件的方法調用，不是變數
        if (this.globalObjects.includes(baseName)) {
            return null;  // 不是變數，是全域物件的方法
        }
        
        // 移除函數調用：func() -> func
        varRef = baseName.split('(')[0];
        
        // 檢查是否為有效的變數名稱
        if (/^[a-zA-Z_$][a-zA-Z0-9_$]*$/.test(varRef)) {
            return varRef;
        }
        
        return null;
    }
    
    /**
     * 解析條件表達式（用於 skip_if, skip_if_not）
     * @param {string} condition - 條件表達式
     * @returns {Set<string>} - 變數名稱集合
     */
    parseCondition(condition) {
        // 條件表達式和普通表達式的解析方式相同
        return this.extractVariables(condition);
    }
    
    /**
     * 測試函數
     */
    static test() {
        const parser = new ExpressionParser();
        
        const testCases = [
            {
                expr: "${no_racks} || !${rack_is_processed}",
                expected: ["no_racks", "rack_is_processed"]
            },
            {
                expr: "${racks_at_location[0].next_destination || 'default'}",
                expected: ["racks_at_location"]
            },
            {
                expr: "${processed_count * 100 / (total_manual_racks || 1)}",
                expected: ["processed_count", "total_manual_racks"]
            },
            {
                expr: "${processing_rate < 30}",
                expected: ["processing_rate"]
            },
            {
                expr: "${empty_count}",
                expected: ["empty_count"]
            },
            {
                expr: "!${need_optimization}",
                expected: ["need_optimization"]
            }
        ];
        
        console.log("Testing Expression Parser:");
        testCases.forEach((test, index) => {
            const result = parser.extractVariables(test.expr);
            const resultArray = Array.from(result).sort();
            const expectedArray = test.expected.sort();
            const passed = JSON.stringify(resultArray) === JSON.stringify(expectedArray);
            
            console.log(`Test ${index + 1}: ${passed ? '✅ PASS' : '❌ FAIL'}`);
            console.log(`  Expression: ${test.expr}`);
            console.log(`  Expected: ${expectedArray}`);
            console.log(`  Got: ${resultArray}`);
            if (!passed) {
                console.log(`  ERROR: Mismatch!`);
            }
        });
    }
}

// 導出
export { ExpressionParser };