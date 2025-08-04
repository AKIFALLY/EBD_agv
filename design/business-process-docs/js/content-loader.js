/**
 * RosAGV 文檔內容載入器
 * 支援 Nginx 伺服器和 file:// 本地雙模式
 */
class ContentLoader {
    constructor() {
        this.baseUrl = this.detectBaseUrl();
        this.cache = new Map();
        this.isLocalFile = window.location.protocol === 'file:';
        
        // 確保 marked 已載入並配置渲染器
        if (typeof marked === 'undefined') {
            console.error('Marked.js 未載入');
        } else {
            this.configureRenderer();
        }
    }

    /**
     * 配置 Marked.js - 適配新版本 API
     */
    configureRenderer() {
        // 檢查 marked 是否載入
        if (typeof marked === 'undefined') {
            console.error('Marked.js 未載入');
            return;
        }
        
        console.log('配置 Marked.js...');
        
        try {
            // 創建自定義渲染器
            const renderer = new marked.Renderer();
            
            // 自定義代碼塊渲染器，智能處理縮排
            renderer.code = function(code, language) {
                // 分析代碼的最小縮排級別
                const lines = code.split('\n');
                const nonEmptyLines = lines.filter(line => line.trim().length > 0);
                
                if (nonEmptyLines.length === 0) {
                    // 空代碼塊
                    const validLanguage = language && language.trim() ? language.trim().toLowerCase() : 'text';
                    return `<pre class="language-${validLanguage}"><code class="language-${validLanguage}"></code></pre>`;
                }
                
                // 計算最小縮排 (排除空行)
                let minIndent = Infinity;
                for (const line of nonEmptyLines) {
                    const match = line.match(/^[ \t]*/);
                    if (match && line.trim().length > 0) {
                        minIndent = Math.min(minIndent, match[0].length);
                    }
                }
                
                // 如果所有行都有共同的縮排，移除它
                let cleanCode = code;
                if (minIndent > 0 && minIndent !== Infinity) {
                    const indentPattern = new RegExp(`^[ \\t]{${minIndent}}`, 'gm');
                    cleanCode = code.replace(indentPattern, '');
                }
                
                // 移除開頭和結尾的空行
                cleanCode = cleanCode.replace(/^\n+/, '').replace(/\n+$/, '');
                
                // 確定語言類別
                const validLanguage = language && language.trim() ? language.trim().toLowerCase() : 'text';
                
                // 生成 Prism.js 相容的 HTML
                return `<pre class="language-${validLanguage}"><code class="language-${validLanguage}">${this.escapeHtml(cleanCode)}</code></pre>`;
            };
            
            // 自定義行內代碼渲染器
            renderer.codespan = function(code) {
                return `<code class="inline-code">${this.escapeHtml(code)}</code>`;
            };
            
            // HTML 轉義函數
            renderer.escapeHtml = function(html) {
                const div = document.createElement('div');
                div.textContent = html;
                return div.innerHTML;
            };
            
            // 檢查 marked 版本並使用適當的 API
            if (marked.setOptions) {
                // 較舊版本的 API
                marked.setOptions({
                    renderer: renderer,
                    gfm: true,
                    breaks: false,
                    pedantic: false,
                    sanitize: false,
                    smartypants: true,
                    headerIds: true,
                    mangle: false
                });
            } else if (marked.use) {
                // 新版本的 API
                marked.use({
                    renderer: renderer,
                    gfm: true,
                    breaks: false,
                    pedantic: false,
                    smartypants: true,
                    headerIds: true,
                    mangle: false
                });
            }
            
            console.log('Marked.js 配置完成（包含自定義代碼渲染器）');
            
        } catch (error) {
            console.error('配置 Marked.js 失敗:', error);
        }
    }
    
    /**
     * 檢測提示框類型
     */
    detectAlertType(text) {
        // 確保 text 是字符串
        const textStr = String(text || '');
        
        if (textStr.includes('💡')) return 'info';
        if (textStr.includes('⚠️')) return 'warning';
        if (textStr.includes('✅')) return 'success';
        if (textStr.includes('❌')) return 'error';
        if (textStr.includes('🔧')) return 'note';
        return 'info';
    }
    
    /**
     * HTML 轉義
     */
    escapeHtml(html) {
        const div = document.createElement('div');
        div.textContent = html;
        return div.innerHTML;
    }

    /**
     * 檢測基礎 URL
     */
    detectBaseUrl() {
        if (window.location.protocol === 'file:') {
            // file:// 模式 - 相對路徑
            return './';
        } else {
            // HTTP 模式 - 相對路徑，因為我們在同一個目錄結構中
            return './';
        }
    }

    /**
     * 載入 Markdown 文件
     * @param {string} filePath - 文件路徑 (例如: 'getting-started/overview.md')
     * @returns {Promise<string>} 解析後的 HTML
     */
    async loadMarkdown(filePath) {
        try {
            // 檢查快取
            if (this.cache.has(filePath)) {
                return this.cache.get(filePath);
            }

            const url = `${this.baseUrl}content/${filePath}`;
            console.log(`載入 Markdown: ${url}`);

            const response = await fetch(url);
            if (!response.ok) {
                throw new Error(`HTTP ${response.status}: ${response.statusText}`);
            }

            const markdown = await response.text();
            
            // 使用 marked 解析 Markdown
            let html;
            if (typeof marked.parse === 'function') {
                // 新版本 API
                html = marked.parse(markdown);
            } else if (typeof marked === 'function') {
                // 舊版本 API
                html = marked(markdown);
            } else {
                throw new Error('無法識別 Marked.js API');
            }
            
            // 快取結果
            this.cache.set(filePath, html);
            
            return html;
        } catch (error) {
            console.error(`載入 Markdown 失敗: ${filePath}`, error);
            return `<div class="error">
                <h3>📄 內容載入失敗</h3>
                <p>無法載入文件: <code>${filePath}</code></p>
                <p>錯誤: ${error.message}</p>
                ${this.isLocalFile ? 
                    '<p><strong>本地模式提示:</strong> 請確保文件存在於 content/ 目錄中</p>' :
                    '<p><strong>伺服器模式提示:</strong> 請檢查 Nginx 配置和文件路徑</p>'
                }
            </div>`;
        }
    }

    /**
     * 載入 JSON 配置文件
     * @param {string} filePath - JSON 文件路徑
     * @returns {Promise<Object>} JSON 對象
     */
    async loadJson(filePath) {
        try {
            if (this.cache.has(filePath)) {
                return this.cache.get(filePath);
            }

            const url = `${this.baseUrl}${filePath}`;
            const response = await fetch(url);
            
            if (!response.ok) {
                throw new Error(`HTTP ${response.status}: ${response.statusText}`);
            }

            const json = await response.json();
            this.cache.set(filePath, json);
            
            return json;
        } catch (error) {
            console.error(`載入 JSON 失敗: ${filePath}`, error);
            return null;
        }
    }

    /**
     * 清除快取
     */
    clearCache() {
        this.cache.clear();
    }

    /**
     * 預載入常用文件
     */
    async preloadCommonFiles() {
        const commonFiles = [
            'getting-started/what-is-rosagv.md',
            'getting-started/system-overview.md'
        ];

        for (const file of commonFiles) {
            try {
                await this.loadMarkdown(file);
                console.log(`預載入成功: ${file}`);
            } catch (error) {
                console.warn(`預載入失敗: ${file}`, error);
            }
        }
    }
}

// 全域實例
window.contentLoader = new ContentLoader();