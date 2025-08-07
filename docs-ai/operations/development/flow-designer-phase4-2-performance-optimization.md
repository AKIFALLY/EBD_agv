# Flow Designer Phase 4.2 效能最佳化

## 🎯 最佳化目標
基於 Phase 4.1 系統完整性測試結果，針對大型流程圖處理、記憶體使用、渲染效能和載入速度進行深度最佳化。

## 📊 效能基準線 (Phase 4.1 測試結果)
- **渲染時間**: 目標 < 2秒 (15節點流程圖)
- **記憶體增長**: 目標 < 50MB
- **載入時間**: 目標 < 1秒
- **轉換時間**: 目標 < 3秒 (雙向轉換)

## 🚀 效能最佳化策略

### 1. 大型流程圖處理最佳化

#### 1.1 節點批量渲染機制
**問題**: 大量節點逐一渲染造成頁面卡頓
**解決方案**: 實施批量渲染和虛擬化滾動

```javascript
// Phase 4.2: 批量節點渲染最佳化
class BatchNodeRenderer {
    constructor(batchSize = 10, renderDelay = 16) {
        this.batchSize = batchSize;
        this.renderDelay = renderDelay; // 60fps = 16ms
        this.renderQueue = [];
        this.isRendering = false;
    }

    async batchRenderNodes(nodes) {
        console.log(`🔄 批量渲染 ${nodes.length} 個節點...`);
        
        // 分批處理節點
        for (let i = 0; i < nodes.length; i += this.batchSize) {
            const batch = nodes.slice(i, i + this.batchSize);
            this.renderQueue.push(batch);
        }

        // 開始批量渲染
        if (!this.isRendering) {
            this.processRenderQueue();
        }
    }

    async processRenderQueue() {
        this.isRendering = true;
        
        while (this.renderQueue.length > 0) {
            const batch = this.renderQueue.shift();
            
            // 渲染當前批次
            const renderPromises = batch.map(node => this.renderSingleNode(node));
            await Promise.all(renderPromises);
            
            // 讓出控制權給瀏覽器，避免卡頓
            await this.nextTick();
        }
        
        this.isRendering = false;
        console.log('✅ 批量渲染完成');
    }

    nextTick() {
        return new Promise(resolve => {
            requestAnimationFrame(() => {
                setTimeout(resolve, this.renderDelay);
            });
        });
    }

    async renderSingleNode(node) {
        // 單個節點渲染邏輯
        return new Promise(resolve => {
            const nodeElement = this.createNodeElement(node);
            document.getElementById('rete-editor').appendChild(nodeElement);
            resolve(nodeElement);
        });
    }
}
```

#### 1.2 視窗內節點可見性最佳化
**問題**: 畫面外的節點仍在渲染浪費資源
**解決方案**: 實施視窗裁剪和延遲載入

```javascript
// Phase 4.2: 視窗裁剪最佳化
class ViewportCulling {
    constructor(editor) {
        this.editor = editor;
        this.viewportBounds = null;
        this.visibleNodes = new Set();
        this.observer = null;
    }

    initializeViewportTracking() {
        const editorContainer = document.getElementById('rete-editor');
        
        // 使用 Intersection Observer 追蹤節點可見性
        this.observer = new IntersectionObserver((entries) => {
            entries.forEach(entry => {
                const nodeId = entry.target.dataset.nodeId;
                
                if (entry.isIntersecting) {
                    this.visibleNodes.add(nodeId);
                    this.enableNodeInteraction(entry.target);
                } else {
                    this.visibleNodes.delete(nodeId);
                    this.disableNodeInteraction(entry.target);
                }
            });
        }, {
            root: editorContainer,
            rootMargin: '100px', // 預載入邊距
            threshold: 0.1
        });
    }

    enableNodeInteraction(nodeElement) {
        // 啟用節點互動功能
        nodeElement.style.pointerEvents = 'auto';
        nodeElement.classList.remove('viewport-culled');
    }

    disableNodeInteraction(nodeElement) {
        // 停用節點互動功能以節省資源
        nodeElement.style.pointerEvents = 'none';
        nodeElement.classList.add('viewport-culled');
    }

    observeNode(nodeElement) {
        this.observer.observe(nodeElement);
    }

    getVisibleNodeCount() {
        return this.visibleNodes.size;
    }
}
```

### 2. 記憶體使用最佳化

#### 2.1 智能記憶體管理
**問題**: 大量節點和連接造成記憶體洩漏
**解決方案**: 實施物件池和智能垃圾回收

```javascript
// Phase 4.2: 記憶體管理最佳化
class MemoryManager {
    constructor() {
        this.nodePool = [];
        this.connectionPool = [];
        this.maxPoolSize = 100;
        this.gcInterval = 30000; // 30秒
        this.memoryThreshold = 100 * 1024 * 1024; // 100MB
        this.startGarbageCollection();
    }

    // 節點物件池
    acquireNode() {
        if (this.nodePool.length > 0) {
            return this.nodePool.pop();
        }
        return this.createNewNode();
    }

    releaseNode(node) {
        this.cleanupNode(node);
        if (this.nodePool.length < this.maxPoolSize) {
            this.nodePool.push(node);
        }
    }

    // 連接物件池
    acquireConnection() {
        if (this.connectionPool.length > 0) {
            return this.connectionPool.pop();
        }
        return this.createNewConnection();
    }

    releaseConnection(connection) {
        this.cleanupConnection(connection);
        if (this.connectionPool.length < this.maxPoolSize) {
            this.connectionPool.push(connection);
        }
    }

    // 智能垃圾回收
    startGarbageCollection() {
        setInterval(() => {
            this.performGarbageCollection();
        }, this.gcInterval);
    }

    performGarbageCollection() {
        const memoryInfo = this.getMemoryInfo();
        
        if (memoryInfo.usedJSHeapSize > this.memoryThreshold) {
            console.log('🗑️ 執行記憶體清理...');
            
            // 清理不再使用的DOM元素
            this.cleanupOrphanedElements();
            
            // 清理事件監聽器
            this.cleanupEventListeners();
            
            // 強制垃圾回收 (如果可用)
            if (window.gc) {
                window.gc();
            }
            
            console.log('✅ 記憶體清理完成');
        }
    }

    getMemoryInfo() {
        if (performance.memory) {
            return {
                usedJSHeapSize: performance.memory.usedJSHeapSize,
                totalJSHeapSize: performance.memory.totalJSHeapSize,
                jsHeapSizeLimit: performance.memory.jsHeapSizeLimit
            };
        }
        return { usedJSHeapSize: 0 };
    }

    cleanupOrphanedElements() {
        // 清理不再需要的 DOM 元素
        const orphanedNodes = document.querySelectorAll('.flow-node:not([data-active])');
        orphanedNodes.forEach(node => node.remove());
        
        const orphanedConnections = document.querySelectorAll('#connection-svg path:not([data-active])');
        orphanedConnections.forEach(path => path.remove());
    }

    cleanupEventListeners() {
        // 移除不再需要的事件監聽器
        // 這需要配合事件監聽器註冊機制
    }
}
```

#### 2.2 延遲載入機制
**問題**: 同時載入所有節點配置浪費記憶體
**解決方案**: 按需載入節點配置和資源

```javascript
// Phase 4.2: 延遲載入最佳化
class LazyLoader {
    constructor() {
        this.loadedConfigs = new Map();
        this.loadingPromises = new Map();
        this.cacheTimeout = 300000; // 5分鐘快取
    }

    async loadNodeTypeConfig(nodeType) {
        // 檢查快取
        if (this.loadedConfigs.has(nodeType)) {
            const cached = this.loadedConfigs.get(nodeType);
            if (Date.now() - cached.timestamp < this.cacheTimeout) {
                return cached.config;
            }
        }

        // 檢查是否正在載入
        if (this.loadingPromises.has(nodeType)) {
            return await this.loadingPromises.get(nodeType);
        }

        // 開始載入
        const loadingPromise = this.fetchNodeTypeConfig(nodeType);
        this.loadingPromises.set(nodeType, loadingPromise);

        try {
            const config = await loadingPromise;
            
            // 存入快取
            this.loadedConfigs.set(nodeType, {
                config,
                timestamp: Date.now()
            });
            
            return config;
        } finally {
            this.loadingPromises.delete(nodeType);
        }
    }

    async fetchNodeTypeConfig(nodeType) {
        console.log(`📥 延遲載入節點配置: ${nodeType}`);
        
        try {
            const response = await fetch(`/static/js/flow-designer/configs/${nodeType}.json`);
            if (!response.ok) {
                throw new Error(`載入失敗: ${response.status}`);
            }
            
            const config = await response.json();
            console.log(`✅ 節點配置載入完成: ${nodeType}`);
            
            return config;
        } catch (error) {
            console.error(`❌ 節點配置載入失敗 ${nodeType}:`, error);
            return this.getDefaultConfig(nodeType);
        }
    }

    getDefaultConfig(nodeType) {
        // 提供預設配置避免載入失敗
        return {
            id: nodeType,
            name: nodeType,
            description: `預設 ${nodeType} 節點`,
            category: 'default',
            inputs: ['input'],
            outputs: ['output'],
            parameters: {}
        };
    }

    clearCache() {
        this.loadedConfigs.clear();
        this.loadingPromises.clear();
        console.log('🗑️ 快取已清理');
    }
}
```

### 3. 渲染效能提升

#### 3.1 Canvas/WebGL 渲染引擎
**問題**: DOM 渲染在大量節點時效能瓶頸
**解決方案**: 使用 Canvas 或 WebGL 進行高效能渲染

```javascript
// Phase 4.2: Canvas 渲染引擎
class CanvasRenderer {
    constructor(canvas) {
        this.canvas = canvas;
        this.ctx = canvas.getContext('2d');
        this.nodes = [];
        this.connections = [];
        this.viewport = { x: 0, y: 0, scale: 1 };
        this.isDirty = true;
        this.animationFrame = null;
    }

    addNode(node) {
        this.nodes.push({
            id: node.id,
            x: node.x,
            y: node.y,
            width: 150,
            height: 80,
            type: node.type,
            name: node.name,
            selected: false
        });
        this.markDirty();
    }

    addConnection(connection) {
        this.connections.push({
            id: connection.id,
            sourceX: connection.sourceX,
            sourceY: connection.sourceY,
            targetX: connection.targetX,
            targetY: connection.targetY
        });
        this.markDirty();
    }

    markDirty() {
        this.isDirty = true;
        if (!this.animationFrame) {
            this.animationFrame = requestAnimationFrame(() => {
                this.render();
                this.animationFrame = null;
            });
        }
    }

    render() {
        if (!this.isDirty) return;

        const { ctx, canvas, viewport } = this;
        
        // 清空畫布
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        
        // 設定變換矩陣
        ctx.save();
        ctx.translate(viewport.x, viewport.y);
        ctx.scale(viewport.scale, viewport.scale);

        // 渲染連接線
        this.renderConnections();
        
        // 渲染節點
        this.renderNodes();
        
        ctx.restore();
        this.isDirty = false;
    }

    renderNodes() {
        const { ctx } = this;
        
        this.nodes.forEach(node => {
            // 節點背景
            ctx.fillStyle = this.getNodeColor(node.type);
            ctx.fillRect(node.x, node.y, node.width, node.height);
            
            // 節點邊框
            ctx.strokeStyle = node.selected ? '#48cae4' : '#ccc';
            ctx.lineWidth = node.selected ? 3 : 1;
            ctx.strokeRect(node.x, node.y, node.width, node.height);
            
            // 節點文字
            ctx.fillStyle = '#333';
            ctx.font = '14px Arial';
            ctx.textAlign = 'center';
            ctx.fillText(
                node.name, 
                node.x + node.width / 2, 
                node.y + node.height / 2 + 5
            );
        });
    }

    renderConnections() {
        const { ctx } = this;
        
        ctx.strokeStyle = '#2563eb';
        ctx.lineWidth = 2;
        
        this.connections.forEach(conn => {
            ctx.beginPath();
            
            // 貝塞爾曲線連接
            const controlX = (conn.sourceX + conn.targetX) / 2;
            ctx.moveTo(conn.sourceX, conn.sourceY);
            ctx.bezierCurveTo(
                controlX, conn.sourceY,
                controlX, conn.targetY,
                conn.targetX, conn.targetY
            );
            
            ctx.stroke();
            
            // 繪製箭頭
            this.drawArrow(conn.targetX, conn.targetY, 
                          Math.atan2(conn.targetY - conn.sourceY, 
                                   conn.targetX - conn.sourceX));
        });
    }

    drawArrow(x, y, angle) {
        const { ctx } = this;
        const arrowLength = 10;
        const arrowAngle = Math.PI / 6;
        
        ctx.save();
        ctx.translate(x, y);
        ctx.rotate(angle);
        
        ctx.beginPath();
        ctx.moveTo(0, 0);
        ctx.lineTo(-arrowLength * Math.cos(arrowAngle), -arrowLength * Math.sin(arrowAngle));
        ctx.moveTo(0, 0);
        ctx.lineTo(-arrowLength * Math.cos(arrowAngle), arrowLength * Math.sin(arrowAngle));
        ctx.stroke();
        
        ctx.restore();
    }

    getNodeColor(nodeType) {
        const colors = {
            'condition': '#dbeafe',
            'logic': '#fef3c7',
            'action': '#d1fae5',
            'script': '#ede9fe'
        };
        return colors[nodeType] || '#f3f4f6';
    }

    // 視窗控制
    setViewport(x, y, scale) {
        this.viewport = { x, y, scale };
        this.markDirty();
    }

    // 節點選擇
    selectNode(nodeId) {
        this.nodes.forEach(node => {
            node.selected = (node.id === nodeId);
        });
        this.markDirty();
    }
}
```

#### 3.2 連接線最佳化渲染
**問題**: SVG 連接線在大量連接時渲染緩慢
**解決方案**: 使用路徑合併和視窗裁剪

```javascript
// Phase 4.2: 連接線最佳化
class OptimizedConnectionRenderer {
    constructor(svg) {
        this.svg = svg;
        this.connectionPaths = new Map();
        this.visibleConnections = new Set();
        this.batchUpdateTimer = null;
    }

    batchUpdateConnections(connections) {
        // 批量更新連接線，避免頻繁重繪
        if (this.batchUpdateTimer) {
            clearTimeout(this.batchUpdateTimer);
        }

        this.batchUpdateTimer = setTimeout(() => {
            this.performBatchUpdate(connections);
        }, 16); // 60fps
    }

    performBatchUpdate(connections) {
        console.log(`🔄 批量更新 ${connections.length} 條連接線...`);
        
        // 使用 DocumentFragment 減少 DOM 操作
        const fragment = document.createDocumentFragment();
        const paths = [];

        connections.forEach(conn => {
            if (this.isConnectionVisible(conn)) {
                const path = this.createOptimizedPath(conn);
                paths.push(path);
                fragment.appendChild(path);
            }
        });

        // 一次性更新 DOM
        this.clearSvg();
        this.svg.appendChild(fragment);
        
        console.log(`✅ 連接線更新完成，顯示 ${paths.length} 條`);
    }

    createOptimizedPath(connection) {
        const path = document.createElementNS('http://www.w3.org/2000/svg', 'path');
        
        // 使用更高效的路徑計算
        const pathData = this.calculateOptimizedPath(connection);
        
        path.setAttribute('d', pathData);
        path.setAttribute('stroke', '#2563eb');
        path.setAttribute('stroke-width', '2');
        path.setAttribute('fill', 'none');
        path.setAttribute('data-connection-id', connection.id);
        
        // 使用 CSS 類別而非內聯樣式以提升效能
        path.classList.add('flow-connection');
        
        return path;
    }

    calculateOptimizedPath(connection) {
        // 使用簡化的貝塞爾曲線計算
        const { sourceX, sourceY, targetX, targetY } = connection;
        const controlOffset = Math.min(Math.abs(targetX - sourceX) * 0.5, 100);
        
        return `M${sourceX},${sourceY} C${sourceX + controlOffset},${sourceY} ${targetX - controlOffset},${targetY} ${targetX},${targetY}`;
    }

    isConnectionVisible(connection) {
        // 檢查連接線是否在視窗內
        const editorRect = this.svg.parentElement.getBoundingClientRect();
        const margin = 50; // 預載入邊距
        
        return (
            connection.sourceX >= -margin && connection.sourceX <= editorRect.width + margin &&
            connection.sourceY >= -margin && connection.sourceY <= editorRect.height + margin &&
            connection.targetX >= -margin && connection.targetX <= editorRect.width + margin &&
            connection.targetY >= -margin && connection.targetY <= editorRect.height + margin
        );
    }

    clearSvg() {
        // 高效清理 SVG 內容
        while (this.svg.firstChild) {
            this.svg.removeChild(this.svg.firstChild);
        }
    }
}
```

### 4. 載入速度優化

#### 4.1 資源預載入和快取策略
**問題**: 初始載入時間過長
**解決方案**: 實施智能預載入和多層快取

```javascript
// Phase 4.2: 資源預載入最佳化
class ResourcePreloader {
    constructor() {
        this.cache = new Map();
        this.preloadQueue = [];
        this.isPreloading = false;
        this.maxCacheSize = 50;
        this.preloadBatchSize = 5;
    }

    async preloadCriticalResources() {
        console.log('🚀 開始預載入關鍵資源...');
        
        const criticalResources = [
            // 核心 JavaScript 模組
            '/static/js/flow-designer/node-types.js',
            '/static/js/flow-designer/connection-manager.js',
            
            // 常用節點配置
            '/static/js/flow-designer/configs/condition_nodes.json',
            '/static/js/flow-designer/configs/logic_nodes.json',
            '/static/js/flow-designer/configs/action_nodes.json',
            
            // 圖示和樣式
            '/static/css/flowDesignerPage.css',
            '/static/images/node-icons.sprite.svg'
        ];

        // 並行預載入
        const preloadPromises = criticalResources.map(url => 
            this.preloadResource(url, 'critical')
        );

        try {
            await Promise.all(preloadPromises);
            console.log('✅ 關鍵資源預載入完成');
        } catch (error) {
            console.error('❌ 關鍵資源預載入失敗:', error);
        }

        // 開始預載入非關鍵資源
        this.startBackgroundPreloading();
    }

    async preloadResource(url, priority = 'normal') {
        if (this.cache.has(url)) {
            return this.cache.get(url);
        }

        console.log(`📥 預載入資源: ${url} (${priority})`);
        
        try {
            const response = await fetch(url, {
                cache: 'force-cache',
                priority: priority === 'critical' ? 'high' : 'low'
            });

            if (!response.ok) {
                throw new Error(`HTTP ${response.status}`);
            }

            const content = await this.parseResponseContent(response, url);
            
            // 存入快取
            this.addToCache(url, content);
            
            return content;
        } catch (error) {
            console.error(`❌ 資源預載入失敗 ${url}:`, error);
            throw error;
        }
    }

    async parseResponseContent(response, url) {
        const contentType = response.headers.get('content-type');
        
        if (contentType?.includes('application/json')) {
            return await response.json();
        } else if (contentType?.includes('text/')) {
            return await response.text();
        } else {
            return await response.blob();
        }
    }

    addToCache(url, content) {
        // LRU 快取管理
        if (this.cache.size >= this.maxCacheSize) {
            const firstKey = this.cache.keys().next().value;
            this.cache.delete(firstKey);
        }

        this.cache.set(url, {
            content,
            timestamp: Date.now(),
            accessCount: 1
        });
    }

    startBackgroundPreloading() {
        // 背景預載入非關鍵資源
        const nonCriticalResources = [
            '/static/js/flow-designer/configs/script_nodes.json',
            '/static/js/flow-designer/templates/',
            '/static/images/backgrounds/'
        ];

        nonCriticalResources.forEach(url => {
            this.preloadQueue.push({ url, priority: 'low' });
        });

        this.processPreloadQueue();
    }

    async processPreloadQueue() {
        if (this.isPreloading || this.preloadQueue.length === 0) {
            return;
        }

        this.isPreloading = true;

        while (this.preloadQueue.length > 0) {
            const batch = this.preloadQueue.splice(0, this.preloadBatchSize);
            
            const batchPromises = batch.map(item => 
                this.preloadResource(item.url, item.priority)
                    .catch(error => console.warn(`背景預載入失敗: ${item.url}`, error))
            );

            await Promise.allSettled(batchPromises);
            
            // 讓出控制權，避免阻塞主線程
            await new Promise(resolve => setTimeout(resolve, 100));
        }

        this.isPreloading = false;
    }

    getCachedResource(url) {
        const cached = this.cache.get(url);
        if (cached) {
            cached.accessCount++;
            return cached.content;
        }
        return null;
    }

    clearCache() {
        this.cache.clear();
        console.log('🗑️ 資源快取已清理');
    }
}
```

#### 4.2 漸進式載入機制
**問題**: 大型流程圖載入時造成頁面無回應
**解決方案**: 分階段漸進式載入和顯示

```javascript
// Phase 4.2: 漸進式載入最佳化
class ProgressiveLoader {
    constructor(flowDesigner) {
        this.flowDesigner = flowDesigner;
        this.loadingStages = [
            { name: '初始化編輯器', weight: 10 },
            { name: '載入節點配置', weight: 20 },
            { name: '渲染節點', weight: 40 },
            { name: '建立連接', weight: 20 },
            { name: '完成載入', weight: 10 }
        ];
        this.currentStage = 0;
        this.onProgress = null;
    }

    async loadFlowProgressively(flowData, onProgress = null) {
        this.onProgress = onProgress;
        this.currentStage = 0;
        
        console.log('🔄 開始漸進式載入流程...');
        this.reportProgress('開始載入', 0);

        try {
            // 階段 1: 初始化編輯器
            await this.executeStage('初始化編輯器', async () => {
                await this.flowDesigner.setupEditor();
                this.flowDesigner.clearEditor();
            });

            // 階段 2: 載入節點配置
            await this.executeStage('載入節點配置', async () => {
                const nodeTypes = this.extractNodeTypes(flowData);
                await this.preloadNodeConfigs(nodeTypes);
            });

            // 階段 3: 漸進式渲染節點
            await this.executeStage('渲染節點', async () => {
                await this.renderNodesProgressively(flowData.nodes || []);
            });

            // 階段 4: 建立連接
            await this.executeStage('建立連接', async () => {
                await this.createConnectionsProgressively(flowData.connections || []);
            });

            // 階段 5: 完成載入
            await this.executeStage('完成載入', async () => {
                this.flowDesigner.updateStatusBar();
                this.flowDesigner.updateFlowNameDisplay();
            });

            this.reportProgress('載入完成', 100);
            console.log('✅ 漸進式載入完成');

        } catch (error) {
            console.error('❌ 漸進式載入失敗:', error);
            this.reportProgress('載入失敗', 0, error.message);
            throw error;
        }
    }

    async executeStage(stageName, stageFunction) {
        console.log(`📍 執行階段: ${stageName}`);
        
        const stageIndex = this.loadingStages.findIndex(s => s.name === stageName);
        if (stageIndex === -1) return;

        const stage = this.loadingStages[stageIndex];
        const progressBefore = this.calculateProgressBefore(stageIndex);
        
        this.reportProgress(stageName, progressBefore);

        const startTime = performance.now();
        await stageFunction();
        const duration = performance.now() - startTime;

        const progressAfter = progressBefore + stage.weight;
        this.reportProgress(`${stageName} 完成`, progressAfter);
        
        console.log(`✅ ${stageName} 完成 (${duration.toFixed(0)}ms)`);
    }

    async renderNodesProgressively(nodes) {
        const batchSize = 5; // 每批渲染5個節點
        const renderDelay = 16; // 16ms = 60fps
        
        for (let i = 0; i < nodes.length; i += batchSize) {
            const batch = nodes.slice(i, i + batchSize);
            
            // 渲染當前批次
            const renderPromises = batch.map(node => this.renderNode(node));
            await Promise.all(renderPromises);
            
            // 更新進度
            const progress = (i + batch.length) / nodes.length * 100;
            this.reportProgress(`渲染節點 (${i + batch.length}/${nodes.length})`, 
                              30 + progress * 0.4); // 階段3佔40%權重
            
            // 讓出控制權
            if (i + batchSize < nodes.length) {
                await new Promise(resolve => setTimeout(resolve, renderDelay));
            }
        }
    }

    async createConnectionsProgressively(connections) {
        const batchSize = 10; // 每批處理10個連接
        
        for (let i = 0; i < connections.length; i += batchSize) {
            const batch = connections.slice(i, i + batchSize);
            
            // 處理當前批次
            batch.forEach(conn => {
                try {
                    this.flowDesigner.createConnection(conn.source, conn.target);
                } catch (error) {
                    console.warn(`連接建立失敗: ${conn.source} -> ${conn.target}`, error);
                }
            });
            
            // 更新進度
            const progress = (i + batch.length) / connections.length * 100;
            this.reportProgress(`建立連接 (${i + batch.length}/${connections.length})`, 
                              70 + progress * 0.2); // 階段4佔20%權重
            
            // 讓出控制權
            if (i + batchSize < connections.length) {
                await new Promise(resolve => requestAnimationFrame(resolve));
            }
        }
    }

    calculateProgressBefore(stageIndex) {
        return this.loadingStages
            .slice(0, stageIndex)
            .reduce((sum, stage) => sum + stage.weight, 0);
    }

    reportProgress(message, percentage, error = null) {
        const progressInfo = {
            message,
            percentage: Math.min(Math.max(percentage, 0), 100),
            error,
            timestamp: Date.now()
        };

        if (this.onProgress) {
            this.onProgress(progressInfo);
        }

        // 更新UI進度條
        this.updateProgressUI(progressInfo);
    }

    updateProgressUI(progressInfo) {
        // 更新進度條和狀態訊息
        const progressBar = document.getElementById('loading-progress');
        const statusMessage = document.getElementById('loading-status');
        
        if (progressBar) {
            progressBar.style.width = `${progressInfo.percentage}%`;
            progressBar.setAttribute('aria-valuenow', progressInfo.percentage);
        }
        
        if (statusMessage) {
            statusMessage.textContent = progressInfo.message;
            if (progressInfo.error) {
                statusMessage.classList.add('has-text-danger');
            } else {
                statusMessage.classList.remove('has-text-danger');
            }
        }
    }

    extractNodeTypes(flowData) {
        const nodeTypes = new Set();
        
        if (flowData.nodes) {
            flowData.nodes.forEach(node => {
                if (node.type) {
                    nodeTypes.add(node.type);
                }
            });
        }
        
        return Array.from(nodeTypes);
    }

    async preloadNodeConfigs(nodeTypes) {
        const preloadPromises = nodeTypes.map(async (nodeType) => {
            try {
                return await this.flowDesigner.lazyLoader.loadNodeTypeConfig(nodeType);
            } catch (error) {
                console.warn(`節點配置預載入失敗: ${nodeType}`, error);
                return null;
            }
        });

        await Promise.allSettled(preloadPromises);
    }

    async renderNode(nodeData) {
        try {
            await this.flowDesigner.createNodeFromStep(nodeData);
        } catch (error) {
            console.error(`節點渲染失敗: ${nodeData.id}`, error);
            throw error;
        }
    }
}
```

## 📊 效能監控和測量

### 效能指標追蹤
```javascript
// Phase 4.2: 效能監控系統
class PerformanceMonitor {
    constructor() {
        this.metrics = new Map();
        this.observers = [];
        this.isMonitoring = false;
    }

    startMonitoring() {
        if (this.isMonitoring) return;
        
        this.isMonitoring = true;
        console.log('📊 開始效能監控...');

        // 監控 FPS
        this.monitorFPS();
        
        // 監控記憶體使用
        this.monitorMemoryUsage();
        
        // 監控DOM操作
        this.monitorDOMOperations();
        
        // 監控網路請求
        this.monitorNetworkRequests();
    }

    monitorFPS() {
        let lastTime = performance.now();
        let frameCount = 0;

        const measureFPS = (currentTime) => {
            frameCount++;
            
            if (currentTime - lastTime >= 1000) {
                const fps = Math.round((frameCount * 1000) / (currentTime - lastTime));
                this.recordMetric('fps', fps);
                
                frameCount = 0;
                lastTime = currentTime;
            }
            
            if (this.isMonitoring) {
                requestAnimationFrame(measureFPS);
            }
        };

        requestAnimationFrame(measureFPS);
    }

    monitorMemoryUsage() {
        setInterval(() => {
            if (performance.memory) {
                const memoryInfo = {
                    used: Math.round(performance.memory.usedJSHeapSize / 1024 / 1024),
                    total: Math.round(performance.memory.totalJSHeapSize / 1024 / 1024),
                    limit: Math.round(performance.memory.jsHeapSizeLimit / 1024 / 1024)
                };
                
                this.recordMetric('memory', memoryInfo);
            }
        }, 5000);
    }

    recordMetric(name, value) {
        if (!this.metrics.has(name)) {
            this.metrics.set(name, []);
        }
        
        const history = this.metrics.get(name);
        history.push({
            value,
            timestamp: Date.now()
        });
        
        // 保持最近100個數據點
        if (history.length > 100) {
            history.shift();
        }
    }

    getPerformanceReport() {
        const report = {};
        
        this.metrics.forEach((history, metricName) => {
            if (history.length === 0) return;
            
            const values = history.map(h => h.value);
            
            if (typeof values[0] === 'number') {
                report[metricName] = {
                    current: values[values.length - 1],
                    average: values.reduce((a, b) => a + b, 0) / values.length,
                    min: Math.min(...values),
                    max: Math.max(...values),
                    samples: values.length
                };
            } else {
                report[metricName] = {
                    current: values[values.length - 1],
                    samples: values.length
                };
            }
        });
        
        return report;
    }

    stopMonitoring() {
        this.isMonitoring = false;
        console.log('⏹️ 效能監控已停止');
    }
}
```

## 🎯 優化實施計劃

### Phase 4.2.1: 核心效能優化 (Week 1)
- [x] 實施批量節點渲染機制
- [x] 添加視窗裁剪優化
- [x] 實施記憶體管理系統
- [x] 創建效能監控工具

### Phase 4.2.2: 進階渲染優化 (Week 2) 
- [ ] 實施 Canvas 渲染引擎
- [ ] 優化連接線渲染系統
- [ ] 添加延遲載入機制
- [ ] 實施資源預載入系統

### Phase 4.2.3: 載入速度優化 (Week 3)
- [ ] 實施漸進式載入機制
- [ ] 添加智能快取策略  
- [ ] 優化初始化流程
- [ ] 實施背景預載入

### Phase 4.2.4: 整合測試和驗證 (Week 4)
- [ ] 大型流程圖效能測試
- [ ] 記憶體洩漏檢測
- [ ] 跨瀏覽器相容性測試
- [ ] 效能基準測試和報告

## 📈 預期效能提升

### 目標指標
- **渲染時間**: 50% 改善 (從 2秒 到 1秒)
- **記憶體使用**: 40% 減少 (從 50MB 到 30MB)
- **載入時間**: 60% 改善 (從 1秒 到 0.4秒)
- **FPS**: 維持 ≥ 30fps (大型流程圖)
- **大型流程圖支援**: 100+ 節點流暢運行

### 測試場景
1. **小型流程圖** (≤ 10節點): < 0.2秒載入
2. **中型流程圖** (11-30節點): < 0.6秒載入  
3. **大型流程圖** (31-60節點): < 1.2秒載入
4. **超大型流程圖** (61-100節點): < 2.5秒載入

## 🔧 配置和部署

### 效能優化配置
```javascript
// Phase 4.2: 效能優化配置
const PERFORMANCE_CONFIG = {
    rendering: {
        batchSize: 10,
        renderDelay: 16,
        enableViewportCulling: true,
        useCanvasRenderer: false // 實驗性功能
    },
    memory: {
        maxPoolSize: 100,
        gcInterval: 30000,
        memoryThreshold: 100 * 1024 * 1024
    },
    loading: {
        preloadCriticalResources: true,
        enableProgressiveLoading: true,
        maxCacheSize: 50,
        preloadBatchSize: 5
    },
    monitoring: {
        enablePerformanceMonitoring: true,
        metricsRetentionCount: 100,
        reportingInterval: 5000
    }
};
```

Phase 4.2 效能最佳化將大幅提升 Flow Designer 在處理大型流程圖時的性能表現，為用戶提供更流暢的視覺化編程體驗。