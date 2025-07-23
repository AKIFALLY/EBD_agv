/**
 * 階層視圖增量更新效能測試腳本
 * 
 * 使用方法：
 * 1. 在瀏覽器中打開任務管理頁面
 * 2. 切換到階層視圖
 * 3. 在控制台中執行此腳本
 */

class HierarchyPerformanceTester {
    constructor() {
        this.testResults = [];
        this.mockTasks = [];
        this.originalGenerateHierarchy = null;
    }

    /**
     * 初始化測試環境
     */
    init() {
        console.log('🧪 初始化階層視圖效能測試環境');
        
        // 檢查必要的函數是否可用
        if (!window.generateHierarchy) {
            throw new Error('generateHierarchy 函數不可用');
        }
        
        if (!window.generateHierarchyDebounced) {
            throw new Error('generateHierarchyDebounced 函數不可用');
        }
        
        if (!window.hierarchyPerformanceMetrics) {
            throw new Error('hierarchyPerformanceMetrics 不可用');
        }
        
        // 生成測試資料
        this.generateMockTasks();
        
        console.log('✅ 測試環境初始化完成');
    }

    /**
     * 生成模擬任務資料
     */
    generateMockTasks() {
        this.mockTasks = [];
        
        // 生成 50 個根任務
        for (let i = 1; i <= 50; i++) {
            const rootTask = {
                id: i,
                name: `根任務 ${i}`,
                status_id: Math.floor(Math.random() * 7),
                agv: Math.random() > 0.5 ? `AGV ${Math.floor(Math.random() * 10) + 1}` : '未分配',
                parent_task_id: null
            };
            this.mockTasks.push(rootTask);
            
            // 為每個根任務生成 2-5 個子任務
            const childCount = Math.floor(Math.random() * 4) + 2;
            for (let j = 1; j <= childCount; j++) {
                const childTask = {
                    id: i * 100 + j,
                    name: `子任務 ${i}-${j}`,
                    status_id: Math.floor(Math.random() * 7),
                    agv: Math.random() > 0.7 ? `AGV ${Math.floor(Math.random() * 10) + 1}` : '未分配',
                    parent_task_id: i
                };
                this.mockTasks.push(childTask);
            }
        }
        
        console.log(`生成了 ${this.mockTasks.length} 個模擬任務`);
    }

    /**
     * 測試完全重建效能
     */
    async testFullRebuildPerformance() {
        console.log('🔄 測試完全重建效能');
        
        const iterations = 5;
        const times = [];
        
        for (let i = 0; i < iterations; i++) {
            const startTime = performance.now();
            
            // 模擬完全重建
            if (window.rebuildHierarchyComplete) {
                window.rebuildHierarchyComplete();
            } else {
                window.generateHierarchy();
            }
            
            // 等待 DOM 更新完成
            await new Promise(resolve => setTimeout(resolve, 100));
            
            const endTime = performance.now();
            const duration = endTime - startTime;
            times.push(duration);
            
            console.log(`完全重建 ${i + 1}: ${duration.toFixed(2)}ms`);
        }
        
        const avgTime = times.reduce((a, b) => a + b, 0) / times.length;
        const result = {
            type: 'fullRebuild',
            iterations,
            times,
            averageTime: avgTime,
            minTime: Math.min(...times),
            maxTime: Math.max(...times)
        };
        
        this.testResults.push(result);
        console.log(`完全重建平均耗時: ${avgTime.toFixed(2)}ms`);
        
        return result;
    }

    /**
     * 測試增量更新效能
     */
    async testIncrementalUpdatePerformance() {
        console.log('⚡ 測試增量更新效能');
        
        const iterations = 10;
        const times = [];
        
        for (let i = 0; i < iterations; i++) {
            // 模擬小幅度資料變更
            const changedTasks = this.mockTasks.slice(0, 5).map(task => ({
                ...task,
                status_id: Math.floor(Math.random() * 7),
                name: `${task.name} (更新 ${i})`
            }));
            
            const startTime = performance.now();
            
            // 觸發增量更新
            window.generateHierarchy();
            
            // 等待更新完成
            await new Promise(resolve => setTimeout(resolve, 50));
            
            const endTime = performance.now();
            const duration = endTime - startTime;
            times.push(duration);
            
            console.log(`增量更新 ${i + 1}: ${duration.toFixed(2)}ms`);
        }
        
        const avgTime = times.reduce((a, b) => a + b, 0) / times.length;
        const result = {
            type: 'incrementalUpdate',
            iterations,
            times,
            averageTime: avgTime,
            minTime: Math.min(...times),
            maxTime: Math.max(...times)
        };
        
        this.testResults.push(result);
        console.log(`增量更新平均耗時: ${avgTime.toFixed(2)}ms`);
        
        return result;
    }

    /**
     * 測試防抖機制效能
     */
    async testDebouncePerformance() {
        console.log('🔀 測試防抖機制效能');
        
        const startTime = performance.now();
        const triggerCount = 20;
        
        // 快速觸發多次更新
        for (let i = 0; i < triggerCount; i++) {
            window.generateHierarchyDebounced(50);
        }
        
        // 等待防抖完成
        await new Promise(resolve => setTimeout(resolve, 200));
        
        const endTime = performance.now();
        const totalTime = endTime - startTime;
        
        const metrics = window.hierarchyPerformanceMetrics.getMetrics();
        
        const result = {
            type: 'debounce',
            triggerCount,
            totalTime,
            actualUpdates: metrics.updateCount,
            efficiency: (triggerCount - metrics.updateCount) / triggerCount * 100
        };
        
        this.testResults.push(result);
        console.log(`防抖測試: 觸發 ${triggerCount} 次，實際執行 ${metrics.updateCount} 次，效率提升 ${result.efficiency.toFixed(1)}%`);
        
        return result;
    }

    /**
     * 測試記憶體使用情況
     */
    testMemoryUsage() {
        console.log('💾 測試記憶體使用情況');
        
        if (!performance.memory) {
            console.warn('瀏覽器不支援記憶體監控');
            return null;
        }
        
        const beforeMemory = {
            used: performance.memory.usedJSHeapSize,
            total: performance.memory.totalJSHeapSize,
            limit: performance.memory.jsHeapSizeLimit
        };
        
        // 執行多次更新
        for (let i = 0; i < 10; i++) {
            window.generateHierarchy();
        }
        
        // 強制垃圾回收（如果可用）
        if (window.gc) {
            window.gc();
        }
        
        const afterMemory = {
            used: performance.memory.usedJSHeapSize,
            total: performance.memory.totalJSHeapSize,
            limit: performance.memory.jsHeapSizeLimit
        };
        
        const memoryDiff = afterMemory.used - beforeMemory.used;
        
        const result = {
            type: 'memory',
            beforeMemory,
            afterMemory,
            memoryDiff,
            memoryDiffMB: memoryDiff / 1024 / 1024
        };
        
        this.testResults.push(result);
        console.log(`記憶體變化: ${result.memoryDiffMB.toFixed(2)}MB`);
        
        return result;
    }

    /**
     * 執行完整的效能測試套件
     */
    async runFullTestSuite() {
        console.log('🚀 開始執行完整的階層視圖效能測試套件');
        console.log('=' * 60);
        
        try {
            this.init();
            
            // 執行各項測試
            await this.testFullRebuildPerformance();
            await this.testIncrementalUpdatePerformance();
            await this.testDebouncePerformance();
            this.testMemoryUsage();
            
            // 生成測試報告
            this.generateReport();
            
        } catch (error) {
            console.error('測試執行失敗:', error);
        }
    }

    /**
     * 生成測試報告
     */
    generateReport() {
        console.log('\n📊 階層視圖效能測試報告');
        console.log('=' * 60);
        
        this.testResults.forEach(result => {
            switch (result.type) {
                case 'fullRebuild':
                    console.log(`🔄 完全重建效能:`);
                    console.log(`   平均耗時: ${result.averageTime.toFixed(2)}ms`);
                    console.log(`   最快: ${result.minTime.toFixed(2)}ms`);
                    console.log(`   最慢: ${result.maxTime.toFixed(2)}ms`);
                    break;
                    
                case 'incrementalUpdate':
                    console.log(`⚡ 增量更新效能:`);
                    console.log(`   平均耗時: ${result.averageTime.toFixed(2)}ms`);
                    console.log(`   最快: ${result.minTime.toFixed(2)}ms`);
                    console.log(`   最慢: ${result.maxTime.toFixed(2)}ms`);
                    break;
                    
                case 'debounce':
                    console.log(`🔀 防抖機制效能:`);
                    console.log(`   觸發次數: ${result.triggerCount}`);
                    console.log(`   實際執行: ${result.actualUpdates}`);
                    console.log(`   效率提升: ${result.efficiency.toFixed(1)}%`);
                    break;
                    
                case 'memory':
                    console.log(`💾 記憶體使用:`);
                    console.log(`   記憶體變化: ${result.memoryDiffMB.toFixed(2)}MB`);
                    break;
            }
            console.log('');
        });
        
        // 效能評估
        const incrementalResult = this.testResults.find(r => r.type === 'incrementalUpdate');
        const fullRebuildResult = this.testResults.find(r => r.type === 'fullRebuild');
        
        if (incrementalResult && fullRebuildResult) {
            const improvement = ((fullRebuildResult.averageTime - incrementalResult.averageTime) / fullRebuildResult.averageTime) * 100;
            console.log(`🎯 效能改善: 增量更新比完全重建快 ${improvement.toFixed(1)}%`);
        }
        
        console.log('✅ 測試報告生成完成');
    }
}

// 暴露測試器到全域範圍
window.HierarchyPerformanceTester = HierarchyPerformanceTester;

// 提供快速測試函數
window.runHierarchyPerformanceTest = async function() {
    const tester = new HierarchyPerformanceTester();
    await tester.runFullTestSuite();
};

console.log('階層視圖效能測試腳本已載入');
console.log('執行 runHierarchyPerformanceTest() 開始測試');
