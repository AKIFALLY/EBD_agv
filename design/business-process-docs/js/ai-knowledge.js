/**
 * AI 知識庫導航器
 * 展示 docs-ai 文檔結構和 AI Agent 引用統計
 */
class AIKnowledgeNavigator {
    constructor() {
        this.indexData = null;
        this.currentFilter = 'all';
        this.currentSort = 'references';
        this.currentCategory = 'all';
        this.initialized = false;

        this.init();
    }

    async init() {
        try {
            // 載入索引資料
            const response = await fetch('./js/docs-ai-index.json');
            this.indexData = await response.json();
            this.initialized = true;

            console.log('📚 AI 知識庫索引載入成功:', this.indexData.stats);

            // 初始化事件監聽器
            this.bindEvents();

        } catch (error) {
            console.error('❌ 載入 AI 知識庫索引失敗:', error);
            this.showError('無法載入 AI 知識庫索引');
        }
    }

    bindEvents() {
        // 監聽過濾器變更
        document.addEventListener('change', (e) => {
            if (e.target.matches('[data-ai-filter]')) {
                this.currentFilter = e.target.value;
                this.render();
            }
            if (e.target.matches('[data-ai-sort]')) {
                this.currentSort = e.target.value;
                this.render();
            }
        });

        // 監聽文檔點擊
        document.addEventListener('click', (e) => {
            const docLink = e.target.closest('[data-ai-doc]');
            if (docLink) {
                e.preventDefault();
                const docPath = docLink.dataset.aiDoc;
                this.loadDocument(docPath);
            }
        });
    }

    render() {
        const container = document.getElementById('ai-knowledge-content');
        if (!container || !this.indexData) return;

        // 生成統計卡片
        const statsHtml = this.renderStats();

        // 生成分類視圖
        const categoriesHtml = this.renderCategories();

        // 生成文檔列表
        const documentsHtml = this.renderDocuments();

        container.innerHTML = `
            <div class="ai-knowledge-container">
                ${statsHtml}

                ${this.renderLegend()}

                ${this.currentCategory !== 'all' ? this.renderCategoryBreadcrumb() : ''}

                <div class="mb-6">
                    <div class="flex flex-wrap gap-4">
                        <div>
                            <select data-ai-filter class="px-4 py-2 border border-gray-300 rounded-lg bg-white focus:outline-none focus:ring-2 focus:ring-blue-500">
                                    <option value="all" ${this.currentFilter === 'all' ? 'selected' : ''}>所有文檔</option>
                                    <option value="critical" ${this.currentFilter === 'critical' ? 'selected' : ''}>🔥 關鍵文檔 (≥10次引用)</option>
                                    <option value="important" ${this.currentFilter === 'important' ? 'selected' : ''}>📚 重要文檔 (≥5次引用)</option>
                                    <option value="strong-only" ${this.currentFilter === 'strong-only' ? 'selected' : ''}>🔗 僅強引用文檔</option>
                                    <option value="weak-only" ${this.currentFilter === 'weak-only' ? 'selected' : ''}>🔖 僅弱引用文檔</option>
                                    <option value="mixed" ${this.currentFilter === 'mixed' ? 'selected' : ''}>🔀 混合引用文檔</option>
                                    <option value="referenced" ${this.currentFilter === 'referenced' ? 'selected' : ''}>📖 已引用文檔</option>
                                    <option value="unreferenced" ${this.currentFilter === 'unreferenced' ? 'selected' : ''}>⚠️ 未引用文檔</option>
                                </select>
                        </div>
                        <div>
                            <select data-ai-sort class="px-4 py-2 border border-gray-300 rounded-lg bg-white focus:outline-none focus:ring-2 focus:ring-blue-500">
                                    <option value="references" ${this.currentSort === 'references' ? 'selected' : ''}>按引用次數排序</option>
                                    <option value="category" ${this.currentSort === 'category' ? 'selected' : ''}>按分類排序</option>
                                    <option value="name" ${this.currentSort === 'name' ? 'selected' : ''}>按名稱排序</option>
                                </select>
                        </div>
                    </div>
                </div>

                ${categoriesHtml}
                ${documentsHtml}
            </div>
        `;

        // 綁定分類卡片點擊事件
        this.attachCategoryListeners();
    }

    renderStats() {
        const stats = this.indexData.stats;

        // 層級分佈統計（如果存在）
        const layerStatsHtml = stats.layer_distribution ? `
            <div class="bg-gradient-to-r from-gray-50 to-gray-100 rounded-lg p-4 mb-6 border border-gray-200">
                <h3 class="text-lg font-semibold text-gray-800 mb-3">📚 三層架構分佈</h3>
                <div class="grid grid-cols-1 md:grid-cols-3 gap-4">
                    ${Object.entries(stats.layer_distribution).map(([key, layer]) => {
                        const icon = key === 'layer1' ? '🌐' : key === 'layer2' ? '🔧' : '🔬';
                        // Remove icon from layer.name if it already contains it
                        const layerName = layer.name.replace(icon, '').trim();
                        return `
                        <div class="bg-white rounded-lg p-3 border border-gray-300">
                            <div class="flex items-center gap-2 mb-2">
                                <span class="text-xl">${icon}</span>
                                <span class="font-semibold text-gray-700">${layerName}</span>
                            </div>
                            <div class="text-2xl font-bold ${key === 'layer1' ? 'text-red-600' : key === 'layer2' ? 'text-blue-600' : 'text-green-600'}">${layer.count}</div>
                            <div class="text-xs text-gray-500 mt-1">${layer.description}</div>
                        </div>
                        `;
                    }).join('')}
                </div>
            </div>
        ` : '';

        return `
            ${layerStatsHtml}
            <div class="grid grid-cols-2 md:grid-cols-4 lg:grid-cols-7 gap-4 mb-6">
                <div class="bg-white rounded-lg shadow p-4 text-center border border-gray-200">
                    <div class="text-2xl font-bold text-gray-800">${stats.total_docs}</div>
                    <div class="text-sm text-gray-600 mt-1">總文檔數</div>
                </div>
                <div class="bg-red-50 rounded-lg shadow p-4 text-center border-l-4 border-red-500">
                    <div class="text-2xl font-bold text-red-700">${stats.critical_docs}</div>
                    <div class="text-sm text-red-600 mt-1">🔥 關鍵文檔</div>
                </div>
                <div class="bg-amber-50 rounded-lg shadow p-4 text-center border-l-4 border-amber-500">
                    <div class="text-2xl font-bold text-amber-700">${stats.important_docs}</div>
                    <div class="text-sm text-amber-600 mt-1">📚 重要文檔</div>
                </div>
                <div class="bg-blue-50 rounded-lg shadow p-4 text-center border-l-4 border-blue-500">
                    <div class="text-2xl font-bold text-blue-700">${stats.total_strong_refs}</div>
                    <div class="text-sm text-blue-600 mt-1">🔗 強引用總數</div>
                </div>
                <div class="bg-gray-50 rounded-lg shadow p-4 text-center border-l-4 border-gray-400 border-dashed">
                    <div class="text-2xl font-bold text-gray-700">${stats.total_weak_refs}</div>
                    <div class="text-sm text-gray-600 mt-1">🔖 弱引用總數</div>
                </div>
                <div class="bg-purple-50 rounded-lg shadow p-4 text-center border border-purple-200">
                    <div class="text-2xl font-bold text-purple-700">${stats.total_references}</div>
                    <div class="text-sm text-purple-600 mt-1">📊 總引用次數</div>
                </div>
                <div class="bg-yellow-50 rounded-lg shadow p-4 text-center border border-yellow-300">
                    <div class="text-2xl font-bold text-yellow-700">${stats.unreferenced_docs}</div>
                    <div class="text-sm text-yellow-600 mt-1">⚠️ 未引用文檔</div>
                </div>
            </div>
        `;
    }

    renderCategories() {
        const categories = this.indexData.stats.categories;

        // 添加「全部」按鈕
        const allButton = `
            <div class="category-card bg-white p-4 rounded-lg border-2 hover:shadow-md transition-all cursor-pointer ${this.currentCategory === 'all' ? 'border-indigo-500 bg-indigo-50' : 'border-gray-200'}" data-category="all">
                <h4 class="font-semibold text-gray-800">📚 全部文檔</h4>
                <p class="text-sm text-gray-600 mt-1">顯示所有文檔</p>
                <span class="inline-block mt-2 px-2 py-1 bg-blue-100 text-blue-700 text-xs rounded">${Object.keys(this.indexData.documents || {}).length} 個文檔</span>
            </div>
        `;

        const categoryCards = Object.entries(categories).map(([key, cat]) => `
            <div class="category-card bg-white p-4 rounded-lg border-2 hover:shadow-md transition-all cursor-pointer ${this.currentCategory === key ? 'border-indigo-500 bg-indigo-50' : 'border-gray-200'}" data-category="${key}">
                <h4 class="font-semibold text-gray-800">${cat.name}</h4>
                <p class="text-sm text-gray-600 mt-1">${cat.description}</p>
                <span class="inline-block mt-2 px-2 py-1 bg-blue-100 text-blue-700 text-xs rounded">${cat.count} 個文檔</span>
            </div>
        `).join('');

        return `
            <div class="mb-6">
                <h3 class="text-xl font-bold text-gray-800 mb-4">📁 文檔分類</h3>
                <div class="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-4">
                    ${allButton}
                    ${categoryCards}
                </div>
            </div>
        `;
    }

    renderDocuments() {
        let documents = Object.values(this.indexData.documents);

        // 分類過濾
        if (this.currentCategory !== 'all') {
            documents = documents.filter(doc => doc.category === this.currentCategory);
        }

        // 過濾
        if (this.currentFilter !== 'all') {
            documents = documents.filter(doc => {
                switch (this.currentFilter) {
                    case 'critical':
                        return doc.importance === 'critical';
                    case 'important':
                        return doc.importance === 'important';
                    case 'strong-only':
                        return doc.strong_references > 0 && doc.weak_references === 0;
                    case 'weak-only':
                        return doc.weak_references > 0 && doc.strong_references === 0;
                    case 'mixed':
                        return doc.strong_references > 0 && doc.weak_references > 0;
                    case 'referenced':
                        return doc.references > 0;
                    case 'unreferenced':
                        return doc.references === 0;
                    default:
                        return true;
                }
            });
        }

        // 排序
        documents.sort((a, b) => {
            switch (this.currentSort) {
                case 'references':
                    return b.references - a.references;
                case 'category':
                    return a.category.localeCompare(b.category);
                case 'name':
                    return a.title.localeCompare(b.title);
                default:
                    return 0;
            }
        });

        // 生成文檔卡片
        const docCards = documents.map(doc => this.renderDocumentCard(doc)).join('');

        return `
            <div>
                <h3 class="text-xl font-bold text-gray-800 mb-4">
                    📄 文檔列表
                    <span class="ml-2 px-2 py-1 bg-gray-100 text-gray-700 text-sm rounded">${documents.length} 個文檔</span>
                </h3>
                <div class="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-4">
                    ${docCards}
                </div>
            </div>
        `;
    }

    renderDocumentCard(doc) {
        const importanceIcon = this.getImportanceIcon(doc.importance);
        const categoryInfo = this.indexData.categories[doc.category] || {};

        // 生成引用徽章 - 分別顯示強引用和弱引用
        const referenceBadges = [];
        if (doc.strong_references > 0) {
            referenceBadges.push(`<span class="inline-block px-2 py-1 bg-blue-500 text-white text-xs rounded-full font-semibold" title="強引用 (@docs-ai/)">🔗 ${doc.strong_references}</span>`);
        }
        if (doc.weak_references > 0) {
            referenceBadges.push(`<span class="inline-block px-2 py-1 bg-gray-200 text-gray-600 text-xs rounded-full italic border border-dashed border-gray-400" title="弱引用 (#)">🔖 ${doc.weak_references}</span>`);
        }
        if (doc.total_weighted_refs) {
            referenceBadges.push(`<span class="inline-block px-2 py-1 bg-purple-500 text-white text-xs rounded-full" title="加權總分">⚖️ ${doc.total_weighted_refs.toFixed(1)}</span>`);
        }
        if (referenceBadges.length === 0) {
            referenceBadges.push('<span class="inline-block px-2 py-1 bg-gray-100 text-gray-500 text-xs rounded-full">未引用</span>');
        }

        // 生成層級徽章
        const layerBadge = this.getLayerBadge(doc.layer);

        // 生成引用來源列表（區分強弱引用）
        let referencedByList = '';
        if ((doc.strong_referenced_by && doc.strong_referenced_by.length > 0) ||
            (doc.weak_referenced_by && doc.weak_referenced_by.length > 0)) {
            const strongRefs = doc.strong_referenced_by || [];
            const weakRefs = doc.weak_referenced_by || [];

            let refDetails = [];
            if (strongRefs.length > 0) {
                refDetails.push(`<div class="text-xs text-gray-600">🔗 強引用來源：${this.formatRefList(strongRefs)}</div>`);
            }
            if (weakRefs.length > 0) {
                refDetails.push(`<div class="text-xs text-gray-500 italic">🔖 弱引用來源：${this.formatRefList(weakRefs)}</div>`);
            }

            referencedByList = `<div class="mt-2 pt-2 border-t border-gray-100">${refDetails.join('')}</div>`;
        }

        // 判斷卡片的引用類型樣式
        let borderStyle = 'border-l-4 border-gray-300';
        let bgStyle = 'bg-white';
        let mixedIndicator = '';

        if (doc.strong_references > 0 && doc.weak_references > 0) {
            // 混合引用：使用藍色實線邊框 + 特殊標記
            borderStyle = 'border-l-4 border-indigo-500';
            bgStyle = 'bg-white';
            mixedIndicator = '<div class="absolute top-2 left-0 flex flex-col gap-1"><span class="block w-1 h-3 bg-blue-500"></span><span class="block w-1 h-3 bg-gray-400"></span></div>';
        } else if (doc.strong_references > 0) {
            borderStyle = 'border-l-4 border-blue-500';
            bgStyle = 'bg-white';
        } else if (doc.weak_references > 0) {
            borderStyle = 'border-l-4 border-dashed border-gray-400';
            bgStyle = 'bg-white';
        }

        return `
            <div class="${bgStyle} rounded-lg shadow hover:shadow-md transition-shadow p-4 ${borderStyle} relative" data-category="${doc.category}">
                ${mixedIndicator}
                <div class="flex justify-between items-start mb-2">
                    <span class="text-2xl">${importanceIcon}</span>
                    <div class="flex flex-wrap gap-1">
                        ${referenceBadges.join('')}
                    </div>
                </div>
                <h4 class="font-semibold text-gray-800 mb-2">
                    <a href="#" data-ai-doc="${doc.path}" title="${doc.path}" class="hover:text-blue-600 transition-colors">
                        ${doc.title}
                    </a>
                </h4>
                ${doc.description ? `<p class="text-sm text-gray-600 mb-2">${doc.description}</p>` : ''}
                <div class="flex flex-wrap gap-2 text-xs">
                    <span class="px-2 py-1 bg-gray-100 text-gray-600 rounded">${categoryInfo.name || doc.category}</span>
                    ${layerBadge ? layerBadge : ''}
                    <span class="text-gray-400" title="${doc.path}">
                        📁 ${this.truncatePath(doc.path)}
                    </span>
                </div>
                ${referencedByList}
            </div>
        `;
    }

    formatRefList(refs) {
        const uniqueRefs = [...new Set(refs)]; // 去重
        const display = uniqueRefs.slice(0, 2).map(ref => {
            // 簡化顯示路徑
            return ref.replace('docs-ai/', '').replace('.md', '');
        }).join(', ');

        if (uniqueRefs.length > 2) {
            return `${display} ... (+${uniqueRefs.length - 2})`;
        }
        return display;
    }

    getImportanceIcon(importance) {
        switch (importance) {
            case 'critical':
                return '🔥';
            case 'important':
                return '📚';
            case 'common':
                return '📖';
            case 'referenced':
                return '📄';
            case 'unreferenced':
                return '⚠️';
            default:
                return '📄';
        }
    }

    getLayerBadge(layer) {
        const layerInfo = this.indexData?.layer_definitions?.[layer];
        if (!layerInfo) {
            return '';
        }

        const colors = {
            'layer1': 'bg-red-500 text-white',
            'layer2': 'bg-blue-500 text-white',
            'layer3': 'bg-green-500 text-white'
        };

        const layerIcons = {
            'layer1': '🌐',
            'layer2': '🔧',
            'layer3': '🔬'
        };

        // Remove icon from layerInfo.name if it already contains it
        const icon = layerIcons[layer] || '';
        const layerName = layerInfo.name.replace(icon, '').trim();
        return `<span class="inline-block px-2 py-1 ${colors[layer] || 'bg-gray-400 text-white'} text-xs rounded-full font-semibold" title="${layerInfo.description}">${icon} ${layerName}</span>`;
    }

    truncatePath(path) {
        if (path.length > 40) {
            const parts = path.split('/');
            if (parts.length > 2) {
                return `.../${parts.slice(-2).join('/')}`;
            }
        }
        return path;
    }

    renderLegend() {
        return `
            <div class="bg-gray-50 border border-gray-200 rounded-lg p-4 mb-6">
                <h4 class="font-semibold text-gray-800 mb-3">📊 引用類型說明</h4>
                <div class="grid grid-cols-1 sm:grid-cols-2 lg:grid-cols-3 gap-3">
                    <div class="flex items-center gap-2">
                        <span class="inline-block px-2 py-1 bg-blue-500 text-white text-xs rounded-full font-semibold">🔗 強引用</span>
                        <span class="text-sm text-gray-600">使用 @docs-ai/ 引用 (權重 1.0)</span>
                    </div>
                    <div class="flex items-center gap-2">
                        <span class="inline-block px-2 py-1 bg-gray-200 text-gray-600 text-xs rounded-full italic border border-dashed border-gray-400">🔖 弱引用</span>
                        <span class="text-sm text-gray-600">使用 # 引用 (權重 0.3)</span>
                    </div>
                    <div class="flex items-center gap-2">
                        <span class="inline-block px-2 py-1 bg-purple-500 text-white text-xs rounded-full">⚖️ 加權分</span>
                        <span class="text-sm text-gray-600">綜合權重得分</span>
                    </div>
                    <div class="flex items-center gap-2">
                        <div class="px-2 py-1 text-xs border-l-4 border-blue-500 bg-white rounded">強引用文檔</div>
                        <span class="text-sm text-gray-600">僅有強引用</span>
                    </div>
                    <div class="flex items-center gap-2">
                        <div class="px-2 py-1 text-xs border-l-4 border-dashed border-gray-400 bg-white rounded">弱引用文檔</div>
                        <span class="text-sm text-gray-600">僅有弱引用</span>
                    </div>
                    <div class="flex items-center gap-2">
                        <div class="px-3 py-1 text-xs bg-white rounded flex items-center gap-1">
                            <span class="inline-block w-1 h-4 bg-blue-500"></span>
                            <span class="inline-block w-1 h-4 bg-gray-400"></span>
                            <span>混合引用</span>
                        </div>
                        <span class="text-sm text-gray-600">同時有強弱引用</span>
                    </div>
                </div>
            </div>
        `;
    }

    async loadDocument(docPath) {
        const contentDiv = document.getElementById('main-content');
        const aiKnowledgeDiv = document.getElementById('ai-knowledge-content');
        const sidebar = document.getElementById('sidebar');
        if (!contentDiv) return;

        // 切換到主內容視圖
        if (aiKnowledgeDiv) {
            aiKnowledgeDiv.style.display = 'none';
        }
        if (sidebar) {
            sidebar.style.display = 'none';
        }
        contentDiv.style.display = 'block';

        try {
            // 顯示載入中
            contentDiv.innerHTML = `
                <div class="page-loading-container">
                    <div class="page-loading-spinner"></div>
                    <div class="page-loading-text">載入 AI 知識文檔中...</div>
                    <div class="page-loading-subtext">${docPath}</div>
                </div>
            `;

            // 載入 Markdown 內容
            const response = await fetch(`./docs-ai/${docPath}`);
            if (!response.ok) {
                throw new Error(`HTTP ${response.status}`);
            }

            const markdown = await response.text();

            // 使用 marked 解析 Markdown
            let html;
            if (typeof marked.parse === 'function') {
                html = marked.parse(markdown);
            } else if (typeof marked === 'function') {
                html = marked(markdown);
            } else {
                throw new Error('Marked.js 未載入');
            }

            // 獲取文檔資訊
            const docInfo = this.indexData.documents[docPath] || {};

            // 添加文檔元資訊頭部
            const metaHeader = `
                <div class="ai-doc-header mb-6">
                    <button onclick="window.navigationManager.switchTab('ai-knowledge'); return false;" class="inline-flex items-center px-4 py-2 mb-4 text-sm font-medium text-indigo-700 bg-indigo-100 rounded-lg hover:bg-indigo-200 transition-colors">
                        <svg class="w-4 h-4 mr-2" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                            <path stroke-linecap="round" stroke-linejoin="round" stroke-width="2" d="M10 19l-7-7m0 0l7-7m-7 7h18"></path>
                        </svg>
                        返回 AI 知識庫
                    </button>
                    <div class="breadcrumb text-sm text-gray-600 mb-2">
                        <a href="#ai-knowledge" class="text-indigo-600 hover:text-indigo-800">AI 知識庫</a>
                        <span class="mx-2">/</span>
                        <span>${docInfo.category}</span>
                        <span class="mx-2">/</span>
                        <span class="font-semibold">${docInfo.title || docPath}</span>
                    </div>
                    ${docInfo.references > 0 ? `
                        <div class="doc-stats flex gap-2 mt-3">
                            <span class="inline-block px-3 py-1 text-sm font-medium text-blue-700 bg-blue-100 rounded-full">
                                被引用 ${docInfo.references} 次
                            </span>
                            <span class="inline-block px-3 py-1 text-sm font-medium ${
                                docInfo.importance === 'critical' ? 'text-red-700 bg-red-100' :
                                docInfo.importance === 'important' ? 'text-yellow-700 bg-yellow-100' :
                                'text-gray-700 bg-gray-100'
                            } rounded-full">
                                ${this.getImportanceIcon(docInfo.importance)} ${docInfo.importance}
                            </span>
                            ${docInfo.layer ? this.getLayerBadge(docInfo.layer) : ''}
                        </div>
                    ` : ''}
                </div>
            `;

            // 顯示內容
            contentDiv.innerHTML = `
                ${metaHeader}
                <div class="prose max-w-none">
                    ${html}
                </div>
            `;

            // 觸發語法高亮
            if (typeof Prism !== 'undefined') {
                setTimeout(() => {
                    Prism.highlightAllUnder(contentDiv);
                }, 100);
            }

        } catch (error) {
            console.error('載入 AI 文檔失敗:', error);
            contentDiv.innerHTML = `
                <div class="max-w-2xl mx-auto">
                    <button onclick="window.navigationManager.switchTab('ai-knowledge'); return false;" class="inline-flex items-center px-4 py-2 mb-6 text-sm font-medium text-indigo-700 bg-indigo-100 rounded-lg hover:bg-indigo-200 transition-colors">
                        <svg class="w-4 h-4 mr-2" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                            <path stroke-linecap="round" stroke-linejoin="round" stroke-width="2" d="M10 19l-7-7m0 0l7-7m-7 7h18"></path>
                        </svg>
                        返回 AI 知識庫
                    </button>
                    <div class="bg-red-50 border border-red-200 rounded-lg p-6">
                        <h3 class="text-lg font-semibold text-red-800 mb-2">❌ 載入失敗</h3>
                        <p class="text-red-700">無法載入文檔: ${docPath}</p>
                        <p class="text-red-600 text-sm mt-2">錯誤: ${error.message}</p>
                    </div>
                </div>
            `;
        }
    }

    showError(message) {
        const container = document.getElementById('ai-knowledge-content');
        if (container) {
            container.innerHTML = `
                <div class="notification is-danger">
                    <p>${message}</p>
                </div>
            `;
        }
    }

    renderCategoryBreadcrumb() {
        const category = this.indexData.stats.categories[this.currentCategory];
        if (!category) return '';

        return `
            <div class="mb-4 p-3 bg-indigo-50 border border-indigo-200 rounded-lg flex items-center justify-between">
                <div class="flex items-center gap-2">
                    <span class="text-sm text-gray-600">當前分類：</span>
                    <span class="font-semibold text-indigo-700">${category.name}</span>
                    <span class="text-sm text-gray-600">（${category.count} 個文檔）</span>
                </div>
                <button onclick="window.aiKnowledgeNavigator.clearCategoryFilter()" class="text-sm text-indigo-600 hover:text-indigo-800 underline">
                    清除過濾
                </button>
            </div>
        `;
    }

    attachCategoryListeners() {
        // 綁定分類卡片點擊事件
        const categoryCards = document.querySelectorAll('.category-card');
        categoryCards.forEach(card => {
            card.addEventListener('click', (e) => {
                const category = e.currentTarget.dataset.category;
                this.filterByCategory(category);
            });
        });
    }

    filterByCategory(category) {
        this.currentCategory = category;
        this.render();
    }

    clearCategoryFilter() {
        this.currentCategory = 'all';
        this.render();
    }
}

// 全域實例
window.aiKnowledgeNavigator = null;

// 初始化函數
function initAIKnowledge() {
    if (!window.aiKnowledgeNavigator) {
        window.aiKnowledgeNavigator = new AIKnowledgeNavigator();
    }
    window.aiKnowledgeNavigator.render();
}

// 當文檔準備好時自動初始化
document.addEventListener('DOMContentLoaded', () => {
    // 檢查是否在 AI 知識庫頁籤
    if (window.location.hash === '#ai-knowledge' || window.location.hash === '') {
        // 稍後初始化，確保 DOM 已準備好
        setTimeout(initAIKnowledge, 100);
    }
});