/**
 * 地圖效能監控器
 * 監控地圖功能的效能和穩定性，提供優化建議
 */

export const mapPerformanceMonitor = (() => {
    let performanceData = {
        loadTimes: {},
        apiCalls: {},
        userInteractions: {},
        errors: [],
        memoryUsage: [],
        renderTimes: []
    };
    
    let monitoringEnabled = true;
    let startTime = Date.now();

    // 初始化
    function init() {
        setupPerformanceObserver();
        setupErrorHandling();
        setupMemoryMonitoring();
        setupInteractionTracking();
        
        console.log('Performance monitor initialized');
    }

    // 設置效能觀察器
    function setupPerformanceObserver() {
        if ('PerformanceObserver' in window) {
            // 監控導航時間
            const navObserver = new PerformanceObserver((list) => {
                for (const entry of list.getEntries()) {
                    recordLoadTime(entry.name, entry.duration);
                }
            });
            navObserver.observe({ entryTypes: ['navigation'] });

            // 監控資源載入時間
            const resourceObserver = new PerformanceObserver((list) => {
                for (const entry of list.getEntries()) {
                    if (entry.name.includes('/static/js/') || entry.name.includes('/api/')) {
                        recordLoadTime(entry.name, entry.duration);
                    }
                }
            });
            resourceObserver.observe({ entryTypes: ['resource'] });

            // 監控測量時間
            const measureObserver = new PerformanceObserver((list) => {
                for (const entry of list.getEntries()) {
                    recordRenderTime(entry.name, entry.duration);
                }
            });
            measureObserver.observe({ entryTypes: ['measure'] });
        }
    }

    // 設置錯誤處理
    function setupErrorHandling() {
        window.addEventListener('error', (event) => {
            recordError({
                type: 'javascript',
                message: event.message,
                filename: event.filename,
                lineno: event.lineno,
                colno: event.colno,
                stack: event.error?.stack,
                timestamp: new Date().toISOString()
            });
        });

        window.addEventListener('unhandledrejection', (event) => {
            recordError({
                type: 'promise',
                message: event.reason?.message || 'Unhandled promise rejection',
                stack: event.reason?.stack,
                timestamp: new Date().toISOString()
            });
        });
    }

    // 設置記憶體監控
    function setupMemoryMonitoring() {
        if ('memory' in performance) {
            setInterval(() => {
                recordMemoryUsage();
            }, 30000); // 每30秒記錄一次
        }
    }

    // 設置互動追蹤
    function setupInteractionTracking() {
        // 追蹤點擊事件
        document.addEventListener('click', (event) => {
            const target = event.target.closest('[id], [class*="button"], [class*="map-"]');
            if (target) {
                recordInteraction('click', getElementIdentifier(target));
            }
        });

        // 追蹤鍵盤事件
        document.addEventListener('keydown', (event) => {
            if (event.key === 'Escape' || event.ctrlKey || event.altKey) {
                recordInteraction('keyboard', event.key);
            }
        });
    }

    // 記錄載入時間
    function recordLoadTime(resource, duration) {
        if (!monitoringEnabled) return;
        
        const category = categorizeResource(resource);
        if (!performanceData.loadTimes[category]) {
            performanceData.loadTimes[category] = [];
        }
        
        performanceData.loadTimes[category].push({
            resource: resource,
            duration: duration,
            timestamp: Date.now()
        });
        
        // 限制記錄數量
        if (performanceData.loadTimes[category].length > 100) {
            performanceData.loadTimes[category] = performanceData.loadTimes[category].slice(-50);
        }
    }

    // 記錄 API 調用
    function recordApiCall(url, method, duration, status) {
        if (!monitoringEnabled) return;
        
        const endpoint = extractEndpoint(url);
        if (!performanceData.apiCalls[endpoint]) {
            performanceData.apiCalls[endpoint] = [];
        }
        
        performanceData.apiCalls[endpoint].push({
            url: url,
            method: method,
            duration: duration,
            status: status,
            timestamp: Date.now()
        });
        
        // 限制記錄數量
        if (performanceData.apiCalls[endpoint].length > 50) {
            performanceData.apiCalls[endpoint] = performanceData.apiCalls[endpoint].slice(-25);
        }
    }

    // 記錄渲染時間
    function recordRenderTime(operation, duration) {
        if (!monitoringEnabled) return;
        
        performanceData.renderTimes.push({
            operation: operation,
            duration: duration,
            timestamp: Date.now()
        });
        
        // 限制記錄數量
        if (performanceData.renderTimes.length > 100) {
            performanceData.renderTimes = performanceData.renderTimes.slice(-50);
        }
    }

    // 記錄錯誤
    function recordError(error) {
        if (!monitoringEnabled) return;
        
        performanceData.errors.push(error);
        
        // 限制錯誤記錄數量
        if (performanceData.errors.length > 50) {
            performanceData.errors = performanceData.errors.slice(-25);
        }
        
        console.error('Performance monitor recorded error:', error);
    }

    // 記錄記憶體使用
    function recordMemoryUsage() {
        if (!monitoringEnabled || !('memory' in performance)) return;
        
        const memory = performance.memory;
        performanceData.memoryUsage.push({
            used: memory.usedJSHeapSize,
            total: memory.totalJSHeapSize,
            limit: memory.jsHeapSizeLimit,
            timestamp: Date.now()
        });
        
        // 限制記錄數量
        if (performanceData.memoryUsage.length > 100) {
            performanceData.memoryUsage = performanceData.memoryUsage.slice(-50);
        }
    }

    // 記錄用戶互動
    function recordInteraction(type, target) {
        if (!monitoringEnabled) return;
        
        const key = `${type}_${target}`;
        if (!performanceData.userInteractions[key]) {
            performanceData.userInteractions[key] = 0;
        }
        performanceData.userInteractions[key]++;
    }

    // 輔助函數
    function categorizeResource(resource) {
        if (resource.includes('/api/')) return 'api';
        if (resource.includes('.js')) return 'javascript';
        if (resource.includes('.css')) return 'stylesheet';
        if (resource.includes('.png') || resource.includes('.jpg') || resource.includes('.svg')) return 'image';
        return 'other';
    }

    function extractEndpoint(url) {
        try {
            const urlObj = new URL(url, window.location.origin);
            return urlObj.pathname.replace(/\/\d+/g, '/:id'); // 替換數字 ID
        } catch {
            return url;
        }
    }

    function getElementIdentifier(element) {
        if (element.id) return `#${element.id}`;
        if (element.className) {
            const classes = element.className.split(' ').filter(c => c.startsWith('map-') || c.includes('button'));
            if (classes.length > 0) return `.${classes[0]}`;
        }
        return element.tagName.toLowerCase();
    }

    // 獲取效能報告
    function getPerformanceReport() {
        const now = Date.now();
        const uptime = now - startTime;
        
        return {
            uptime: uptime,
            summary: {
                totalErrors: performanceData.errors.length,
                avgLoadTime: calculateAverageLoadTime(),
                avgApiResponseTime: calculateAverageApiTime(),
                avgRenderTime: calculateAverageRenderTime(),
                memoryTrend: getMemoryTrend(),
                mostUsedFeatures: getMostUsedFeatures()
            },
            details: {
                loadTimes: performanceData.loadTimes,
                apiCalls: performanceData.apiCalls,
                renderTimes: performanceData.renderTimes.slice(-20),
                errors: performanceData.errors.slice(-10),
                memoryUsage: performanceData.memoryUsage.slice(-10),
                userInteractions: performanceData.userInteractions
            },
            recommendations: generateRecommendations()
        };
    }

    // 計算平均載入時間
    function calculateAverageLoadTime() {
        const allTimes = Object.values(performanceData.loadTimes).flat();
        if (allTimes.length === 0) return 0;
        return allTimes.reduce((sum, item) => sum + item.duration, 0) / allTimes.length;
    }

    // 計算平均 API 響應時間
    function calculateAverageApiTime() {
        const allCalls = Object.values(performanceData.apiCalls).flat();
        if (allCalls.length === 0) return 0;
        return allCalls.reduce((sum, item) => sum + item.duration, 0) / allCalls.length;
    }

    // 計算平均渲染時間
    function calculateAverageRenderTime() {
        if (performanceData.renderTimes.length === 0) return 0;
        return performanceData.renderTimes.reduce((sum, item) => sum + item.duration, 0) / performanceData.renderTimes.length;
    }

    // 獲取記憶體趨勢
    function getMemoryTrend() {
        if (performanceData.memoryUsage.length < 2) return 'stable';
        
        const recent = performanceData.memoryUsage.slice(-5);
        const trend = recent[recent.length - 1].used - recent[0].used;
        
        if (trend > 1024 * 1024) return 'increasing'; // 增加超過 1MB
        if (trend < -1024 * 1024) return 'decreasing'; // 減少超過 1MB
        return 'stable';
    }

    // 獲取最常用功能
    function getMostUsedFeatures() {
        const sorted = Object.entries(performanceData.userInteractions)
            .sort(([,a], [,b]) => b - a)
            .slice(0, 5);
        return sorted.map(([feature, count]) => ({ feature, count }));
    }

    // 生成優化建議
    function generateRecommendations() {
        const recommendations = [];
        
        // 檢查載入時間
        const avgLoadTime = calculateAverageLoadTime();
        if (avgLoadTime > 1000) {
            recommendations.push({
                type: 'performance',
                priority: 'high',
                message: '平均載入時間過長，建議優化資源載入'
            });
        }
        
        // 檢查錯誤率
        const errorRate = performanceData.errors.length / (Date.now() - startTime) * 60000; // 每分鐘錯誤數
        if (errorRate > 0.1) {
            recommendations.push({
                type: 'stability',
                priority: 'high',
                message: '錯誤率過高，需要檢查程式碼穩定性'
            });
        }
        
        // 檢查記憶體使用
        const memoryTrend = getMemoryTrend();
        if (memoryTrend === 'increasing') {
            recommendations.push({
                type: 'memory',
                priority: 'medium',
                message: '記憶體使用量持續增加，可能存在記憶體洩漏'
            });
        }
        
        // 檢查 API 響應時間
        const avgApiTime = calculateAverageApiTime();
        if (avgApiTime > 2000) {
            recommendations.push({
                type: 'api',
                priority: 'medium',
                message: 'API 響應時間過長，建議優化後端效能'
            });
        }
        
        return recommendations;
    }

    // 開始效能測量
    function startMeasure(name) {
        if ('performance' in window && 'mark' in performance) {
            performance.mark(`${name}-start`);
        }
    }

    // 結束效能測量
    function endMeasure(name) {
        if ('performance' in window && 'mark' in performance && 'measure' in performance) {
            performance.mark(`${name}-end`);
            performance.measure(name, `${name}-start`, `${name}-end`);
        }
    }

    // 清除效能資料
    function clearData() {
        performanceData = {
            loadTimes: {},
            apiCalls: {},
            userInteractions: {},
            errors: [],
            memoryUsage: [],
            renderTimes: []
        };
        startTime = Date.now();
    }

    // 啟用/禁用監控
    function setMonitoringEnabled(enabled) {
        monitoringEnabled = enabled;
        console.log('Performance monitoring', enabled ? 'enabled' : 'disabled');
    }

    // 公開方法
    return {
        init,
        recordApiCall,
        recordError,
        startMeasure,
        endMeasure,
        getPerformanceReport,
        clearData,
        setMonitoringEnabled
    };
})();

// 全域暴露
window.mapPerformanceMonitor = mapPerformanceMonitor;
