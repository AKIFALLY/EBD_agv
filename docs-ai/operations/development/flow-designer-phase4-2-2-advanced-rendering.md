# Flow Designer Phase 4.2.2: Advanced Rendering Optimization

## 🎯 優化目標
實現高效能的 Canvas/WebGL 渲染引擎，專門針對大規模流程圖渲染進行優化，實現：
- Canvas 2D 渲染引擎替代 DOM 操作
- WebGL 硬體加速渲染 (備選方案)
- 優化連接線渲染演算法
- 視窗裁剪 (Viewport Culling) 實現

## 📋 Performance Targets (Phase 4.2.2)
- **渲染效能提升**: 300% (相比 DOM 渲染)
- **記憶體使用減少**: 60% (減少 DOM 節點)
- **大圖處理能力**: 支援 1000+ 節點流暢渲染
- **畫面更新率**: 60 FPS (在 500 節點規模下)

## 🚀 Phase 4.2.2 Implementation Plan

### 4.2.2.1 Canvas 2D 渲染引擎
基於 HTML5 Canvas 2D API 的高效能渲染系統

#### Core Canvas Renderer
```javascript
class CanvasRenderer {
    constructor(canvasElement, options = {}) {
        this.canvas = canvasElement;
        this.ctx = canvasElement.getContext('2d');
        this.options = {
            enableHiDPI: true,
            enableAntialiasing: true,
            enableCaching: true,
            ...options
        };
        
        this.viewportBounds = { x: 0, y: 0, width: 0, height: 0 };
        this.transform = { scale: 1, offsetX: 0, offsetY: 0 };
        this.renderCache = new Map();
        
        this.setupHiDPI();
        this.setupEventHandlers();
    }
    
    setupHiDPI() {
        const dpr = window.devicePixelRatio || 1;
        const rect = this.canvas.getBoundingClientRect();
        
        this.canvas.width = rect.width * dpr;
        this.canvas.height = rect.height * dpr;
        this.canvas.style.width = rect.width + 'px';
        this.canvas.style.height = rect.height + 'px';
        
        this.ctx.scale(dpr, dpr);
    }
    
    render(nodes, connections) {
        // 清除畫布
        this.clearCanvas();
        
        // 設置變換矩陣
        this.applyTransform();
        
        // 視窗裁剪 - 只渲染可見區域內的元素
        const visibleNodes = this.cullNodes(nodes);
        const visibleConnections = this.cullConnections(connections);
        
        // 分批渲染
        this.renderConnections(visibleConnections);
        this.renderNodes(visibleNodes);
        
        // 渲染 UI 覆蓋層
        this.renderOverlays();
    }
    
    cullNodes(nodes) {
        return nodes.filter(node => this.isNodeVisible(node));
    }
    
    cullConnections(connections) {
        return connections.filter(conn => this.isConnectionVisible(conn));
    }
    
    isNodeVisible(node) {
        const { x, y, width = 200, height = 100 } = node;
        return this.intersectsViewport(x, y, width, height);
    }
    
    intersectsViewport(x, y, width, height) {
        const vp = this.viewportBounds;
        return !(x + width < vp.x || 
                x > vp.x + vp.width || 
                y + height < vp.y || 
                y > vp.y + vp.height);
    }
}
```

#### Node Rendering System
```javascript
class NodeCanvasRenderer {
    constructor(canvasRenderer) {
        this.canvas = canvasRenderer;
        this.nodeCache = new Map();
        this.nodeStyles = this.loadNodeStyles();
    }
    
    renderNode(node) {
        const cacheKey = this.generateNodeCacheKey(node);
        
        // 檢查快取
        if (this.nodeCache.has(cacheKey) && !node.isDirty) {
            const cached = this.nodeCache.get(cacheKey);
            this.canvas.ctx.drawImage(cached, node.x, node.y);
            return;
        }
        
        // 渲染節點到離屏 Canvas
        const offscreen = this.createOffscreenCanvas(node);
        this.renderNodeToCanvas(offscreen, node);
        
        // 快取結果
        this.nodeCache.set(cacheKey, offscreen.canvas);
        
        // 繪製到主 Canvas
        this.canvas.ctx.drawImage(offscreen.canvas, node.x, node.y);
    }
    
    renderNodeToCanvas(offscreen, node) {
        const ctx = offscreen.ctx;
        const style = this.getNodeStyle(node);
        
        // 繪製節點背景
        this.drawNodeBackground(ctx, node, style);
        
        // 繪製節點內容
        this.drawNodeContent(ctx, node, style);
        
        // 繪製節點 Socket
        this.drawNodeSockets(ctx, node, style);
        
        // 繪製節點邊框
        this.drawNodeBorder(ctx, node, style);
    }
    
    drawNodeBackground(ctx, node, style) {
        const { width = 200, height = 100 } = node;
        
        // 漸變背景
        const gradient = ctx.createLinearGradient(0, 0, 0, height);
        gradient.addColorStop(0, style.backgroundColor);
        gradient.addColorStop(1, style.backgroundColorEnd || style.backgroundColor);
        
        ctx.fillStyle = gradient;
        ctx.fillRect(0, 0, width, height);
    }
    
    drawNodeContent(ctx, node, style) {
        const { width = 200, height = 100 } = node;
        
        // 節點標題
        ctx.fillStyle = style.titleColor;
        ctx.font = style.titleFont;
        ctx.textAlign = 'center';
        ctx.fillText(node.name || node.id, width / 2, 25);
        
        // 節點描述
        if (node.description) {
            ctx.fillStyle = style.descriptionColor;
            ctx.font = style.descriptionFont;
            this.drawWrappedText(ctx, node.description, width / 2, 45, width - 20, 16);
        }
    }
    
    drawWrappedText(ctx, text, x, y, maxWidth, lineHeight) {
        const words = text.split(' ');
        let line = '';
        let currentY = y;
        
        for (let n = 0; n < words.length; n++) {
            const testLine = line + words[n] + ' ';
            const metrics = ctx.measureText(testLine);
            const testWidth = metrics.width;
            
            if (testWidth > maxWidth && n > 0) {
                ctx.fillText(line, x, currentY);
                line = words[n] + ' ';
                currentY += lineHeight;
            } else {
                line = testLine;
            }
        }
        ctx.fillText(line, x, currentY);
    }
}
```

#### Connection Rendering System
```javascript
class ConnectionCanvasRenderer {
    constructor(canvasRenderer) {
        this.canvas = canvasRenderer;
        this.connectionCache = new Map();
    }
    
    renderConnection(connection) {
        const ctx = this.canvas.ctx;
        const { source, target, style = {} } = connection;
        
        // 計算連接點
        const sourcePoint = this.getSocketPosition(source);
        const targetPoint = this.getSocketPosition(target);
        
        // 繪製連接線
        this.drawConnectionPath(ctx, sourcePoint, targetPoint, style);
        
        // 繪製箭頭
        if (style.showArrow !== false) {
            this.drawArrowhead(ctx, sourcePoint, targetPoint, style);
        }
    }
    
    drawConnectionPath(ctx, source, target, style) {
        const controlOffset = 100;
        
        ctx.beginPath();
        ctx.moveTo(source.x, source.y);
        
        // 使用貝茲曲線繪製平滑連接線
        ctx.bezierCurveTo(
            source.x + controlOffset, source.y,
            target.x - controlOffset, target.y,
            target.x, target.y
        );
        
        ctx.strokeStyle = style.color || '#666';
        ctx.lineWidth = style.width || 2;
        ctx.stroke();
    }
    
    drawArrowhead(ctx, source, target, style) {
        const angle = Math.atan2(target.y - source.y, target.x - source.x);
        const arrowLength = 12;
        const arrowWidth = 8;
        
        ctx.save();
        ctx.translate(target.x, target.y);
        ctx.rotate(angle);
        
        ctx.beginPath();
        ctx.moveTo(0, 0);
        ctx.lineTo(-arrowLength, -arrowWidth / 2);
        ctx.lineTo(-arrowLength, arrowWidth / 2);
        ctx.closePath();
        
        ctx.fillStyle = style.color || '#666';
        ctx.fill();
        
        ctx.restore();
    }
}
```

### 4.2.2.2 WebGL 渲染引擎 (備選方案)
針對極大規模流程圖 (1000+ 節點) 的硬體加速渲染

#### WebGL Renderer Core
```javascript
class WebGLRenderer {
    constructor(canvasElement) {
        this.canvas = canvasElement;
        this.gl = canvasElement.getContext('webgl2') || canvasElement.getContext('webgl');
        
        if (!this.gl) {
            throw new Error('WebGL not supported');
        }
        
        this.programs = {};
        this.buffers = {};
        this.textures = {};
        
        this.initializeShaders();
        this.setupBuffers();
    }
    
    initializeShaders() {
        // 節點渲染著色器
        this.programs.node = this.createShaderProgram(
            this.nodeVertexShader(),
            this.nodeFragmentShader()
        );
        
        // 連接線渲染著色器
        this.programs.connection = this.createShaderProgram(
            this.connectionVertexShader(),
            this.connectionFragmentShader()
        );
    }
    
    nodeVertexShader() {
        return `
            attribute vec2 a_position;
            attribute vec2 a_texCoord;
            attribute vec4 a_color;
            
            uniform mat3 u_transform;
            uniform vec2 u_resolution;
            
            varying vec2 v_texCoord;
            varying vec4 v_color;
            
            void main() {
                vec3 position = u_transform * vec3(a_position, 1.0);
                vec2 clipSpace = ((position.xy / u_resolution) * 2.0 - 1.0) * vec2(1, -1);
                
                gl_Position = vec4(clipSpace, 0, 1);
                v_texCoord = a_texCoord;
                v_color = a_color;
            }
        `;
    }
    
    nodeFragmentShader() {
        return `
            precision mediump float;
            
            varying vec2 v_texCoord;
            varying vec4 v_color;
            
            uniform sampler2D u_texture;
            
            void main() {
                vec4 texColor = texture2D(u_texture, v_texCoord);
                gl_FragColor = texColor * v_color;
            }
        `;
    }
    
    render(nodes, connections) {
        // 清除畫布
        this.gl.clear(this.gl.COLOR_BUFFER_BIT);
        
        // 渲染連接線
        this.renderConnections(connections);
        
        // 渲染節點 (使用實例化渲染)
        this.renderNodesInstanced(nodes);
    }
    
    renderNodesInstanced(nodes) {
        const program = this.programs.node;
        this.gl.useProgram(program);
        
        // 設置統一變數
        this.setUniforms(program);
        
        // 批量上傳節點數據
        this.uploadNodeData(nodes);
        
        // 實例化渲染
        this.gl.drawArraysInstanced(this.gl.TRIANGLES, 0, 6, nodes.length);
    }
}
```

### 4.2.2.3 視窗裁剪系統
```javascript
class ViewportCuller {
    constructor(viewport) {
        this.viewport = viewport;
        this.spatialIndex = new QuadTree(viewport);
    }
    
    updateSpatialIndex(nodes, connections) {
        this.spatialIndex.clear();
        
        // 索引節點
        nodes.forEach(node => {
            this.spatialIndex.insert({
                x: node.x,
                y: node.y,
                width: node.width || 200,
                height: node.height || 100,
                data: node
            });
        });
    }
    
    cullObjects(viewportBounds) {
        return this.spatialIndex.query(viewportBounds);
    }
    
    isVisible(bounds) {
        const vp = this.viewport;
        return !(bounds.x + bounds.width < vp.x || 
                bounds.x > vp.x + vp.width || 
                bounds.y + bounds.height < vp.y || 
                bounds.y > vp.y + vp.height);
    }
}

class QuadTree {
    constructor(bounds, maxObjects = 10, maxLevels = 5, level = 0) {
        this.bounds = bounds;
        this.maxObjects = maxObjects;
        this.maxLevels = maxLevels;
        this.level = level;
        this.objects = [];
        this.nodes = [];
    }
    
    split() {
        const subWidth = this.bounds.width / 2;
        const subHeight = this.bounds.height / 2;
        const x = this.bounds.x;
        const y = this.bounds.y;
        
        this.nodes[0] = new QuadTree({x: x + subWidth, y: y, width: subWidth, height: subHeight}, this.maxObjects, this.maxLevels, this.level + 1);
        this.nodes[1] = new QuadTree({x: x, y: y, width: subWidth, height: subHeight}, this.maxObjects, this.maxLevels, this.level + 1);
        this.nodes[2] = new QuadTree({x: x, y: y + subHeight, width: subWidth, height: subHeight}, this.maxObjects, this.maxLevels, this.level + 1);
        this.nodes[3] = new QuadTree({x: x + subWidth, y: y + subHeight, width: subWidth, height: subHeight}, this.maxObjects, this.maxLevels, this.level + 1);
    }
    
    insert(object) {
        if (this.nodes.length > 0) {
            const index = this.getIndex(object);
            if (index !== -1) {
                this.nodes[index].insert(object);
                return;
            }
        }
        
        this.objects.push(object);
        
        if (this.objects.length > this.maxObjects && this.level < this.maxLevels) {
            if (this.nodes.length === 0) {
                this.split();
            }
            
            let i = 0;
            while (i < this.objects.length) {
                const index = this.getIndex(this.objects[i]);
                if (index !== -1) {
                    this.nodes[index].insert(this.objects.splice(i, 1)[0]);
                } else {
                    i++;
                }
            }
        }
    }
    
    query(bounds) {
        let objects = [];
        const index = this.getIndex(bounds);
        
        if (this.nodes.length > 0) {
            if (index !== -1) {
                objects = objects.concat(this.nodes[index].query(bounds));
            } else {
                for (let i = 0; i < this.nodes.length; i++) {
                    objects = objects.concat(this.nodes[i].query(bounds));
                }
            }
        }
        
        objects = objects.concat(this.objects.filter(obj => this.intersects(obj, bounds)));
        
        return objects;
    }
}
```

### 4.2.2.4 渲染管理器
```javascript
class AdvancedRenderingManager {
    constructor(options = {}) {
        this.options = {
            preferWebGL: true,
            fallbackToCanvas: true,
            enableViewportCulling: true,
            maxNodesForDOM: 100,
            ...options
        };
        
        this.renderer = null;
        this.viewportCuller = null;
        this.renderStats = {
            fps: 0,
            nodeCount: 0,
            visibleNodeCount: 0,
            renderTime: 0
        };
        
        this.initializeRenderer();
    }
    
    initializeRenderer() {
        const canvas = document.getElementById('flow-canvas');
        
        // 嘗試初始化 WebGL 渲染器
        if (this.options.preferWebGL) {
            try {
                this.renderer = new WebGLRenderer(canvas);
                console.log('✅ WebGL 渲染器已啟用');
                return;
            } catch (error) {
                console.warn('⚠️ WebGL 不可用，回退到 Canvas 2D:', error.message);
            }
        }
        
        // 回退到 Canvas 2D 渲染器
        if (this.options.fallbackToCanvas) {
            this.renderer = new CanvasRenderer(canvas);
            console.log('✅ Canvas 2D 渲染器已啟用');
        }
        
        // 設置視窗裁剪
        if (this.options.enableViewportCulling) {
            this.viewportCuller = new ViewportCuller(this.getViewportBounds());
        }
    }
    
    render(nodes, connections) {
        const startTime = performance.now();
        
        // 更新統計
        this.renderStats.nodeCount = nodes.length;
        
        // 視窗裁剪
        let visibleNodes = nodes;
        let visibleConnections = connections;
        
        if (this.viewportCuller) {
            this.viewportCuller.updateSpatialIndex(nodes, connections);
            const culledObjects = this.viewportCuller.cullObjects(this.getViewportBounds());
            visibleNodes = culledObjects.map(obj => obj.data);
            this.renderStats.visibleNodeCount = visibleNodes.length;
        }
        
        // 根據規模選擇渲染策略
        if (nodes.length > this.options.maxNodesForDOM) {
            // 大規模：使用 Canvas/WebGL 渲染
            this.renderer.render(visibleNodes, visibleConnections);
        } else {
            // 小規模：回退到 DOM 渲染
            this.renderWithDOM(visibleNodes, visibleConnections);
        }
        
        // 更新效能統計
        const endTime = performance.now();
        this.renderStats.renderTime = endTime - startTime;
        this.updateFPS();
    }
    
    updateFPS() {
        // FPS 計算邏輯
        this.renderStats.fps = Math.round(1000 / this.renderStats.renderTime);
    }
    
    getRenderStats() {
        return { ...this.renderStats };
    }
    
    switchRenderer(type) {
        if (type === 'webgl' && !this.renderer instanceof WebGLRenderer) {
            this.initializeWebGLRenderer();
        } else if (type === 'canvas' && !this.renderer instanceof CanvasRenderer) {
            this.initializeCanvasRenderer();
        }
    }
}
```

## 📊 Phase 4.2.2 Integration

### 整合到現有系統
```javascript
// 在 WcsFlowDesigner 中整合 Advanced Rendering
class WcsFlowDesigner {
    constructor() {
        // ... 現有初始化邏輯
        
        // 初始化高級渲染管理器
        this.advancedRenderer = new AdvancedRenderingManager({
            preferWebGL: true,
            enableViewportCulling: true,
            maxNodesForDOM: 50
        });
        
        // 設置渲染循環
        this.setupRenderLoop();
    }
    
    setupRenderLoop() {
        let lastTime = 0;
        const targetFPS = 60;
        const frameTime = 1000 / targetFPS;
        
        const renderLoop = (currentTime) => {
            if (currentTime - lastTime >= frameTime) {
                this.renderFrame();
                lastTime = currentTime;
            }
            requestAnimationFrame(renderLoop);
        };
        
        requestAnimationFrame(renderLoop);
    }
    
    renderFrame() {
        if (this.needsRender) {
            const nodes = this.getAllNodes();
            const connections = this.getAllConnections();
            
            this.advancedRenderer.render(nodes, connections);
            
            this.needsRender = false;
            this.updateRenderStats();
        }
    }
    
    updateRenderStats() {
        const stats = this.advancedRenderer.getRenderStats();
        
        // 更新 UI 中的效能指標
        if (window.PerformanceOptimizer) {
            window.PerformanceOptimizer.updateRenderStats(stats);
        }
    }
}
```

## 🧪 Testing Strategy

### 效能測試場景
1. **小規模測試** (10-50 節點): DOM vs Canvas 效能對比
2. **中規模測試** (100-500 節點): Canvas 2D vs WebGL 效能對比
3. **大規模測試** (1000+ 節點): 視窗裁剪效果測試
4. **互動測試**: 拖拽、縮放、平移操作流暢度

### 測試腳本範例
```javascript
// 大規模渲染效能測試
async function testLargeScaleRendering() {
    console.log('🧪 開始大規模渲染效能測試...');
    
    // 生成測試數據
    const testSizes = [100, 500, 1000, 2000];
    const results = {};
    
    for (const size of testSizes) {
        console.log(`測試 ${size} 節點渲染效能...`);
        
        const nodes = generateTestNodes(size);
        const connections = generateTestConnections(nodes);
        
        // 測試 DOM 渲染 (如果規模允許)
        if (size <= 100) {
            results[`dom_${size}`] = await benchmarkDOMRendering(nodes, connections);
        }
        
        // 測試 Canvas 渲染
        results[`canvas_${size}`] = await benchmarkCanvasRendering(nodes, connections);
        
        // 測試 WebGL 渲染 (如果支援)
        if (window.WebGLRenderer) {
            results[`webgl_${size}`] = await benchmarkWebGLRendering(nodes, connections);
        }
    }
    
    console.log('📊 測試結果:', results);
    return results;
}
```

## 📈 預期效果

### Phase 4.2.2 Performance Metrics
- **Canvas 2D 渲染**: 300% 效能提升 (vs DOM)
- **WebGL 渲染**: 500% 效能提升 (vs DOM)
- **視窗裁剪**: 80% 記憶體節省 (大規模場景)
- **流暢度**: 60 FPS @ 500 節點

### 使用者體驗改善
- **即時回應**: 大流程圖拖拽無延遲
- **平滑縮放**: 任意縮放級別保持流暢
- **快速載入**: 複雜流程圖秒級載入完成
- **穩定執行**: 長時間操作無效能衰減

## 🔗 相關文檔
- Phase 4.2 Performance Optimization: @docs-ai/operations/development/flow-designer-phase4-2-performance-optimization.md
- WebGL Rendering Guide: MDN WebGL Tutorial
- Canvas Performance Best Practices: HTML5 Canvas Handbook