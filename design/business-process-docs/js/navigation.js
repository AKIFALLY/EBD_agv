/**
 * RosAGV 文檔導航管理器
 * 提供簡潔的文檔瀏覽和頁面路由功能
 */
class NavigationManager {
    constructor() {
        this.currentPath = '';
        this.navigationConfig = this.getDefaultConfig();
        
        this.initRouter();
        this.bindEvents();
    }

    /**
     * 預設導航配置
     */
    getDefaultConfig() {
        return {
            sections: [
                {
                    id: 'getting-started',
                    title: '📖 快速開始',
                    icon: '📖',
                    description: '新工程師 15 分鐘快速理解 RosAGV',
                    items: [
                        { 
                            id: 'what-is-rosagv', 
                            title: '什麼是 RosAGV？', 
                            file: 'getting-started/what-is-rosagv.md',
                            description: '30 秒電梯簡報'
                        },
                        { 
                            id: 'system-overview', 
                            title: '系統整體概覽', 
                            file: 'getting-started/system-overview.md',
                            description: '5 分鐘了解系統架構'
                        },
                        { 
                            id: 'key-concepts', 
                            title: '核心概念解釋', 
                            file: 'getting-started/key-concepts.md',
                            description: '重要概念和術語'
                        },
                        { 
                            id: 'quick-start-guide', 
                            title: '快速上手指導', 
                            file: 'getting-started/quick-start-guide.md',
                            description: '立即開始使用'
                        }
                    ]
                },
                {
                    id: 'business-processes',
                    title: '🏭 業務流程',
                    icon: '🏭',
                    description: '核心業務流程和實際應用場景',
                    items: [
                        { 
                            id: 'eyewear-production', 
                            title: '眼鏡生產流程', 
                            file: 'business-processes/eyewear-production.md',
                            description: '射出機 → OPUI → KUKA AGV 完整流程'
                        },
                        { 
                            id: 'indoor-process', 
                            title: '室內物料搬運', 
                            file: 'business-processes/indoor-process.md',
                            description: 'Cargo/Loader/Unloader 工作流程'
                        }
                    ]
                },
                {
                    id: 'agv-vehicles',
                    title: '🚗 AGV 車型',
                    icon: '🚗',
                    description: 'AGV 車型特性和應用場景',
                    items: [
                        { 
                            id: 'vehicle-types', 
                            title: '車型介紹', 
                            file: 'agv-vehicles/vehicle-types.md',
                            description: 'Cargo/Loader/Unloader 三種車型'
                        },
                        { 
                            id: 'cargo-mover', 
                            title: 'Cargo Mover AGV', 
                            file: 'agv-vehicles/cargo-mover.md',
                            description: '貨物搬運車應用'
                        },
                        { 
                            id: 'loader', 
                            title: 'Loader AGV', 
                            file: 'agv-vehicles/loader.md',
                            description: '裝載車應用'
                        },
                        { 
                            id: 'unloader', 
                            title: 'Unloader AGV', 
                            file: 'agv-vehicles/unloader.md',
                            description: '卸載車應用'
                        }
                    ]
                },
                {
                    id: 'system-architecture',
                    title: '🏛️ 系統架構',
                    icon: '🏛️',
                    description: '技術架構設計和系統組成',
                    items: [
                        { 
                            id: 'dual-environment', 
                            title: '雙環境架構', 
                            file: 'system-architecture/dual-environment.md',
                            description: 'AGV 車載 vs AGVC 管理'
                        },
                        { 
                            id: 'technology-stack', 
                            title: '技術棧架構', 
                            file: 'system-architecture/technology-stack.md',
                            description: '完整技術棧和架構設計'
                        }
                    ]
                },
                {
                    id: 'operations',
                    title: '⚙️ 運維操作',
                    icon: '⚙️',
                    description: '系統部署、開發和維護操作',
                    items: [
                        { 
                            id: 'deployment', 
                            title: '部署指南', 
                            file: 'operations/deployment.md',
                            description: '系統部署和環境配置'
                        },
                        { 
                            id: 'development', 
                            title: '開發環境', 
                            file: 'operations/development.md',
                            description: '開發環境設定和工作流程'
                        },
                        { 
                            id: 'maintenance', 
                            title: '維護操作', 
                            file: 'operations/maintenance.md',
                            description: '日常維護和系統監控'
                        },
                        { 
                            id: 'troubleshooting', 
                            title: '故障排除', 
                            file: 'operations/troubleshooting.md',
                            description: '問題診斷和解決方案'
                        },
                        {
                            id: 'system-diagnostics',
                            title: '系統診斷工具',
                            file: 'operations/system-diagnostics.md',
                            description: '完整診斷工具和監控'
                        },
                        {
                            id: 'unified-tools',
                            title: '統一工具系統',
                            file: 'operations/unified-tools.md',
                            description: 'r 命令完整使用指南'
                        },
                        {
                            id: 'service-management',
                            title: '服務管理工具',
                            file: 'operations/service-management.md',
                            description: '統一服務管理 API'
                        }
                    ]
                },
                {
                    id: 'technical-details',
                    title: '🔧 技術實作',
                    icon: '🔧',
                    description: '技術實作細節和開發指導',
                    items: [
                        { 
                            id: 'ros2-integration', 
                            title: 'ROS 2 整合', 
                            file: 'technical-details/ros2-integration.md',
                            description: 'ROS 2 節點和服務開發'
                        },
                        { 
                            id: 'zenoh-communication', 
                            title: 'Zenoh 通訊機制', 
                            file: 'technical-details/zenoh-communication.md',
                            description: '高效能跨容器通訊'
                        },
                        { 
                            id: 'plc-integration', 
                            title: 'PLC 系統整合', 
                            file: 'technical-details/plc-integration.md',
                            description: 'Keyence PLC 通訊實作'
                        },
                        { 
                            id: 'kuka-integration', 
                            title: 'KUKA Fleet 整合', 
                            file: 'technical-details/kuka-integration.md',
                            description: '外部機器人系統協作'
                        },
                        {
                            id: 'tafl-editor',
                            title: 'TAFL 編輯器指南',
                            file: 'technical-details/tafl-editor.md',
                            description: '視覺化流程編輯器使用'
                        },
                        {
                            id: 'tafl-wcs-integration',
                            title: 'TAFL WCS 整合',
                            file: 'technical-details/tafl-wcs-integration.md',
                            description: 'TAFL 倉儲控制系統整合'
                        },
                        { 
                            id: 'performance-optimization', 
                            title: '效能調優', 
                            file: 'technical-details/performance-optimization.md',
                            description: '系統效能最佳化'
                        },
                        {
                            id: 'monitoring-setup',
                            title: '監控系統配置',
                            file: 'technical-details/monitoring-setup.md',
                            description: '完整監控和告警設定'
                        },
                        {
                            id: 'bash-command-reference',
                            title: 'Bash 命令使用手冊',
                            file: 'technical-details/bash-command-reference.md',
                            description: '檔案描述符、重定向、管道與 RosAGV 命令參考'
                        }
                    ]
                }
            ]
        };
    }

    /**
     * 初始化路由
     */
    initRouter() {
        // 監聽 hash 變化
        window.addEventListener('hashchange', () => this.handleRouteChange());

        // 初始載入
        this.handleRouteChange();
    }

    /**
     * 綁定事件
     */
    bindEvents() {
        // 導航點擊事件委託
        document.addEventListener('click', (e) => {
            const link = e.target.closest('[data-navigate]');
            if (link) {
                e.preventDefault();
                const path = link.dataset.navigate;
                this.navigateTo(path);
            }

            // 頁籤切換事件
            const tabButton = e.target.closest('[data-tab]');
            if (tabButton) {
                e.preventDefault();
                this.switchTab(tabButton.dataset.tab);
            }
        });
    }

    /**
     * 切換頁籤
     */
    switchTab(tabName) {
        // 更新頁籤按鈕狀態
        document.querySelectorAll('.tab-button').forEach(btn => {
            btn.classList.remove('active', 'border-blue-600', 'text-blue-600');
            btn.classList.add('border-transparent', 'text-gray-500', 'hover:text-gray-700');
        });

        const activeTab = document.querySelector(`[data-tab="${tabName}"]`);
        if (activeTab) {
            activeTab.classList.add('active', 'border-blue-600', 'text-blue-600');
            activeTab.classList.remove('border-transparent', 'text-gray-500', 'hover:text-gray-700');
        }

        // 切換內容顯示
        const mainContent = document.getElementById('main-content');
        const aiContent = document.getElementById('ai-knowledge-content');
        const sidebar = document.getElementById('sidebar');

        if (tabName === 'ai-knowledge') {
            // 顯示 AI 知識庫
            mainContent.style.display = 'none';
            aiContent.style.display = 'block';
            sidebar.style.display = 'none';

            // 初始化 AI 知識庫
            if (window.initAIKnowledge) {
                window.initAIKnowledge();
            }

            // 更新 URL
            window.location.hash = 'ai-knowledge';
        } else {
            // 顯示業務或技術文檔
            mainContent.style.display = 'block';
            aiContent.style.display = 'none';
            sidebar.style.display = '';

            // 載入對應的內容
            if (tabName === 'business') {
                // 載入業務文檔首頁
                this.navigateTo('index.md');
            } else if (tabName === 'technical') {
                // 載入技術文檔首頁
                this.navigateTo('technical-details/tafl-system.md');
            }
        }
    }

    /**
     * 檢查是否為文檔內錨點
     */
    isDocumentAnchor(hash) {
        // 錨點識別正則表達式
        // 匹配以下格式：
        // - 數字開頭: "1-檔案描述符與重定向"
        // - 中文字符開頭: "檔案描述符與重定向"
        // - 包含中文的複合錨點: "rosagv-核心命令集"
        // - 常見錨點模式: 不包含 "/" 路徑分隔符
        const anchorPattern = /^(\d+[-_]|.*[\u4e00-\u9fff].*|[^\/]*[-_][^\/]*[^\/]*|快速參考|常見問題|進階技巧)$/;

        // 檔案路徑會包含斜線或 .md 後綴，錨點通常不會

        // 如果不包含 "/" 且匹配錨點模式，很可能是文檔內錨點
        if (!hash.includes('/') && anchorPattern.test(hash)) {
            return true;
        }

        // 特別處理：明確的錨點格式
        if (hash.match(/^\d+[-_][\u4e00-\u9fff]/)) {
            return true; // 數字-中文 格式，如 "1-檔案描述符"
        }

        return false;
    }

    /**
     * 處理路由變化
     */
    async handleRouteChange() {
        const hash = window.location.hash.slice(1) || 'home';
        this.currentPath = hash;

        console.log(`導航到: ${hash}`);

        // 處理 AI 知識庫路由
        if (hash === 'ai-knowledge') {
            this.switchTab('ai-knowledge');
            return;
        }

        // 檢查是否為文檔內錨點
        if (this.isDocumentAnchor(hash)) {
            console.log(`檢測到文檔內錨點: ${hash}，跳過路由處理`);
            // 讓瀏覽器自然處理錨點跳轉
            this.scrollToAnchor(hash);
            return;
        }

        // 確保顯示正確的頁籤
        const mainContent = document.getElementById('main-content');
        const aiContent = document.getElementById('ai-knowledge-content');
        if (mainContent) mainContent.style.display = 'block';
        if (aiContent) aiContent.style.display = 'none';

        if (hash === 'home' || hash === '') {
            await this.loadContent('index.md');
        } else {
            // 直接傳遞 hash，讓 loadContent 處理檔案查找邏輯
            await this.loadContent(hash);
        }

        this.updateActiveNavigation();
    }

    /**
     * 滾動到指定錨點（帶平滑動畫）
     */
    scrollToAnchor(hash) {
        try {
            // 解碼 URL 編碼的錨點
            const decodedHash = decodeURIComponent(hash);

            // 嘗試直接找到元素
            let target = document.getElementById(decodedHash);

            // 如果找不到，嘗試其他可能的 ID 格式
            if (!target) {
                // 嘗試原始 hash
                target = document.getElementById(hash);
            }

            // 如果還是找不到，嘗試通過標題文本和多種 ID 格式查找
            if (!target) {
                const headings = document.querySelectorAll('h1, h2, h3, h4, h5, h6');
                for (const heading of headings) {
                    // 提取標題文本進行比對
                    const headingText = heading.textContent.trim();

                    // 多種匹配策略
                    const matches = [
                        // 完全匹配
                        heading.id === decodedHash,
                        heading.id === hash,
                        // 文本內容匹配（移除數字前綴）
                        headingText.includes(decodedHash.replace(/^\d+[-_]/, '')),
                        // 特殊處理 RosAGV 核心命令集
                        decodedHash.includes('rosagv-核心命令集') && headingText.includes('RosAGV 核心命令集'),
                        // 處理中文標點符號差異
                        headingText.replace(/[。，；：！？]/g, '').includes(decodedHash.replace(/^\d+[-_]/, '').replace(/[-_]/g, ' ')),
                        // 數字章節匹配
                        decodedHash.match(/^(\d+)/) && headingText.startsWith(decodedHash.match(/^(\d+)/)[1] + '.')
                    ];

                    if (matches.some(match => match)) {
                        target = heading;
                        console.log(`通過匹配策略找到目標: ${headingText}`);
                        break;
                    }
                }
            }

            if (target) {
                // 平滑滾動到目標元素
                target.scrollIntoView({
                    behavior: 'smooth',
                    block: 'start',
                    inline: 'nearest'
                });

                // 添加高亮效果
                target.classList.add('highlight-anchor');
                setTimeout(() => {
                    target.classList.remove('highlight-anchor');
                }, 2000);

                console.log(`成功滾動到錨點: ${hash}`);
            } else {
                console.warn(`找不到錨點元素: ${hash}`);
                // 如果找不到錨點，讓瀏覽器自然處理
                window.location.hash = hash;
            }
        } catch (error) {
            console.error('錨點滾動失敗:', error);
            // 降級到瀏覽器原生行為
            window.location.hash = hash;
        }
    }

    /**
     * 導航到指定路徑
     */
    navigateTo(path) {
        // 移除 .md 後綴以保持乾淨的 URL
        const cleanPath = path.replace(/\.md$/, '');
        window.location.hash = cleanPath;
    }

    /**
     * 創建頁面切換載入動畫
     */
    createPageLoadingAnimation() {
        return `
            <div class="page-loading-container">
                <div class="page-loading-spinner"></div>
                <div class="page-loading-text">載入中...</div>
                <div class="page-loading-subtext">正在載入文檔內容</div>
            </div>
        `;
    }

    /**
     * 載入內容
     */
    async loadContent(path) {
        const contentDiv = document.getElementById('main-content');
        if (!contentDiv) {
            console.error('找不到 #main-content 元素');
            return;
        }

        try {
            // 添加載入中的優雅動畫
            contentDiv.innerHTML = this.createPageLoadingAnimation();
            
            // 添加淡出效果
            contentDiv.style.opacity = '0.7';
            contentDiv.style.transform = 'translateY(10px)';
            contentDiv.style.transition = 'all 0.3s ease-out';
            
            let filePath = path;
            
            // 如果不是 .md 檔案，查找配置中的對應檔案
            if (!path.endsWith('.md')) {
                const fileInfo = this.findFileByPath(path);
                if (fileInfo) {
                    filePath = fileInfo.file;
                } else {
                    // 如果找不到配置，嘗試添加 .md 後綴
                    filePath = path + '.md';
                }
            }
            
            // 載入 Markdown 內容
            const html = await window.contentLoader.loadMarkdown(filePath);
            
            // 準備新內容
            const newContent = `<div class="prose max-w-none">${html}</div>`;
            
            // 等待最小載入時間（給用戶載入感）
            await new Promise(resolve => setTimeout(resolve, 200));
            
            // 載入完成，淡入新內容
            contentDiv.innerHTML = newContent;
            
            // 重置並添加淡入動畫
            contentDiv.style.opacity = '0';
            contentDiv.style.transform = 'translateY(20px)';
            
            // 使用 requestAnimationFrame 確保 DOM 更新後再執行動畫
            requestAnimationFrame(() => {
                contentDiv.style.opacity = '1';
                contentDiv.style.transform = 'translateY(0)';
            });
            
            // 處理內部連結
            this.processInternalLinks(contentDiv);
            
            // 觸發 Prism.js 語法高亮
            console.log('準備觸發語法高亮...');
            if (typeof Prism !== 'undefined') {
                console.log('Prism.js 可用，開始高亮處理');
                
                // 確保 DOM 更新完成後觸發語法高亮
                setTimeout(() => {
                    try {
                        // 查找所有代碼塊
                        const codeBlocks = contentDiv.querySelectorAll('pre code');
                        console.log(`找到 ${codeBlocks.length} 個代碼塊`);
                        
                        // 對每個代碼塊添加語言類別（如果還沒有的話）
                        codeBlocks.forEach((block, index) => {
                            const pre = block.parentElement;
                            
                            // 檢查是否已經有語言類別
                            if (!pre.className.includes('language-') && !block.className.includes('language-')) {
                                console.log(`代碼塊 ${index}: 沒有語言類別`);
                            } else {
                                console.log(`代碼塊 ${index}: ${pre.className} ${block.className}`);
                            }
                        });
                        
                        // 觸發語法高亮
                        Prism.highlightAllUnder(contentDiv);
                        console.log('語法高亮處理完成');
                    } catch (error) {
                        console.error('語法高亮處理失敗:', error);
                    }
                }, 100);
            } else {
                console.warn('Prism.js 未載入');
            }
            
        } catch (error) {
            console.error('載入內容失敗:', error);
            contentDiv.innerHTML = `
                <div class="error">
                    <h3>❌ 載入失敗</h3>
                    <p>無法載入內容: ${path}</p>
                    <p>錯誤: ${error.message}</p>
                </div>
            `;
        }
    }

    /**
     * 根據路徑查找檔案資訊
     */
    findFileByPath(path) {
        for (const section of this.navigationConfig.sections) {
            for (const item of section.items) {
                // 直接匹配 ID 或檔案路徑
                if (item.id === path || item.file === path) {
                    return item;
                }
                
                // 匹配檔案路徑去掉 .md 後綴
                if (item.file === path + '.md') {
                    return item;
                }
                
                // 匹配完整路徑的檔案名部分和 ID
                if (path.includes('/')) {
                    const pathFileName = path.split('/').pop();
                    const itemFileDir = item.file.includes('/') ? 
                        item.file.substring(0, item.file.lastIndexOf('/')) : '';
                    const pathDir = path.substring(0, path.lastIndexOf('/'));
                    
                    if (item.id === pathFileName && itemFileDir === pathDir) {
                        return item;
                    }
                }
            }
        }
        return null;
    }



    /**
     * 生成章節 HTML
     */
    generateSectionsHtml() {
        return this.navigationConfig.sections.map(section => `
            <div class="section-card" data-section="${section.id}">
                <div class="section-header">
                    <span class="section-icon">${section.icon}</span>
                    <h3>${section.title}</h3>
                </div>
                <p class="section-description">${section.description}</p>
                <ul class="section-items">
                    ${section.items.map(item => `
                        <li>
                            <a href="#${item.file}" data-navigate="${item.file}">
                                ${item.title}
                            </a>
                            <span class="item-description">${item.description}</span>
                        </li>
                    `).join('')}
                </ul>
            </div>
        `).join('');
    }


    /**
     * 更新導航狀態
     */
    updateActiveNavigation() {
        // 移除所有 active 狀態
        document.querySelectorAll('.nav-active').forEach(el => 
            el.classList.remove('nav-active'));
        
        // 添加當前路徑的 active 狀態
        const currentLink = document.querySelector(`[data-navigate="${this.currentPath}"]`);
        if (currentLink) {
            currentLink.classList.add('nav-active');
        }
    }

    /**
     * 解析相對路徑為絕對路徑
     */
    resolveRelativePath(currentPath, relativePath) {
        // 確保 currentPath 以 .md 結尾
        let fullCurrentPath = currentPath;
        if (!fullCurrentPath.endsWith('.md')) {
            fullCurrentPath += '.md';
        }
        
        // 獲取當前檔案的目錄路徑
        const currentDir = fullCurrentPath.includes('/') ? 
            fullCurrentPath.substring(0, fullCurrentPath.lastIndexOf('/')) : '';
        
        // 處理相對路徑
        let targetPath = relativePath;
        let baseDir = currentDir;
        
        // 處理 ../ 向上級目錄的導航
        while (targetPath.startsWith('../')) {
            targetPath = targetPath.substring(3); // 移除 '../'
            if (baseDir.includes('/')) {
                baseDir = baseDir.substring(0, baseDir.lastIndexOf('/'));
            } else {
                baseDir = ''; // 已經到根目錄
            }
        }
        
        // 處理 ./ 當前目錄的導航
        if (targetPath.startsWith('./')) {
            targetPath = targetPath.substring(2); // 移除 './'
        }
        
        // 組合最終路徑並移除 .md 後綴（用於導航）
        const finalPath = baseDir ? `${baseDir}/${targetPath}` : targetPath;
        const navigationPath = finalPath.replace('.md', '');
        
        console.log(`路徑解析: ${currentPath} + ${relativePath} = ${navigationPath}`);
        return navigationPath;
    }

    /**
     * 處理內部連結
     */
    processInternalLinks(container) {
        // 處理 hash 連結
        const hashLinks = container.querySelectorAll('a[href^="#"]');
        hashLinks.forEach(link => {
            const href = link.getAttribute('href').slice(1);
            link.setAttribute('data-navigate', href);
            link.removeAttribute('href');
        });
        
        // 處理 .md 相對路徑連結
        const mdLinks = container.querySelectorAll('a[href$=".md"]');
        mdLinks.forEach(link => {
            const href = link.getAttribute('href');
            // 檢查是否為相對路徑（不是完整 URL）
            if (!href.startsWith('http://') && !href.startsWith('https://') && !href.startsWith('/')) {
                // 如果包含相對路徑標記，需要解析
                if (href.includes('../') || href.includes('./')) {
                    const resolvedPath = this.resolveRelativePath(this.currentPath, href);
                    link.setAttribute('data-navigate', resolvedPath);
                } else {
                    // 直接相對路徑，不需要解析
                    link.setAttribute('data-navigate', href);
                }
                link.removeAttribute('href');
            }
        });
    }
}

// 全域實例
document.addEventListener('DOMContentLoaded', () => {
    window.navigationManager = new NavigationManager();
});