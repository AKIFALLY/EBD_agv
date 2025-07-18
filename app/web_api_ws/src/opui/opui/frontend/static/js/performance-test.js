/**
 * OPUI 首頁性能測試工具
 * 用於測試和驗證優化後的載入速度和響應性能
 */

class PerformanceTest {
    constructor() {
        this.metrics = {
            domContentLoaded: 0,
            appInitStart: 0,
            appInitEnd: 0,
            firstUIRender: 0,
            backgroundSyncComplete: 0,
            totalLoadTime: 0,
            userInteractionReady: 0
        };
        
        this.testResults = [];
        this.isTestMode = false;
    }

    /**
     * 開始性能測試
     */
    startTest() {
        this.isTestMode = true;
        this.metrics.domContentLoaded = performance.now();
        
        console.log('🧪 開始 OPUI 首頁性能測試');
        console.log('📊 DOM 載入完成時間:', this.metrics.domContentLoaded.toFixed(2), 'ms');
        
        // 監聽應用程式初始化事件
        this.setupPerformanceMonitoring();
        
        return this;
    }

    /**
     * 設定性能監控
     */
    setupPerformanceMonitoring() {
        // 監聽應用程式初始化開始
        const originalInit = window.opuiApp?.init;
        if (originalInit) {
            window.opuiApp.init = async function(...args) {
                window.performanceTest.metrics.appInitStart = performance.now();
                console.log('🚀 應用程式初始化開始:', window.performanceTest.metrics.appInitStart.toFixed(2), 'ms');
                
                const result = await originalInit.apply(this, args);
                
                window.performanceTest.metrics.appInitEnd = performance.now();
                window.performanceTest.metrics.userInteractionReady = window.performanceTest.metrics.appInitEnd;
                console.log('✅ 應用程式初始化完成:', window.performanceTest.metrics.appInitEnd.toFixed(2), 'ms');
                console.log('⚡ 用戶可開始互動時間:', window.performanceTest.metrics.userInteractionReady.toFixed(2), 'ms');
                
                return result;
            };
        }

        // 監聽 UI 更新
        const originalUpdateUI = window.opuiApp?.uiManager?.updateUI;
        if (originalUpdateUI) {
            let firstRender = true;
            window.opuiApp.uiManager.updateUI = function(...args) {
                if (firstRender && window.performanceTest.isTestMode) {
                    window.performanceTest.metrics.firstUIRender = performance.now();
                    console.log('🎨 首次 UI 渲染完成:', window.performanceTest.metrics.firstUIRender.toFixed(2), 'ms');
                    firstRender = false;
                }
                
                return originalUpdateUI.apply(this, args);
            };
        }

        // 監聽背景同步完成
        const originalPerformAsyncSync = window.opuiApp?.stateManager?.performAsyncSync;
        if (originalPerformAsyncSync) {
            window.opuiApp.stateManager.performAsyncSync = function(...args) {
                const result = originalPerformAsyncSync.apply(this, args);
                
                setTimeout(() => {
                    if (window.performanceTest.isTestMode) {
                        window.performanceTest.metrics.backgroundSyncComplete = performance.now();
                        console.log('🔄 背景同步完成:', window.performanceTest.metrics.backgroundSyncComplete.toFixed(2), 'ms');
                        window.performanceTest.calculateResults();
                    }
                }, 1000);
                
                return result;
            };
        }
    }

    /**
     * 計算測試結果
     */
    calculateResults() {
        const metrics = this.metrics;
        
        const results = {
            // 關鍵性能指標
            timeToInteractive: metrics.userInteractionReady - metrics.domContentLoaded,
            firstUIRenderTime: metrics.firstUIRender - metrics.domContentLoaded,
            appInitTime: metrics.appInitEnd - metrics.appInitStart,
            backgroundSyncTime: metrics.backgroundSyncComplete - metrics.appInitEnd,
            totalLoadTime: metrics.backgroundSyncComplete - metrics.domContentLoaded,
            
            // 性能評級
            grade: this.calculateGrade(metrics.userInteractionReady - metrics.domContentLoaded),
            
            // 詳細時間點
            timeline: {
                domReady: 0,
                appInitStart: metrics.appInitStart - metrics.domContentLoaded,
                userCanInteract: metrics.userInteractionReady - metrics.domContentLoaded,
                firstUIRender: metrics.firstUIRender - metrics.domContentLoaded,
                backgroundSyncComplete: metrics.backgroundSyncComplete - metrics.domContentLoaded
            }
        };
        
        this.testResults.push(results);
        this.displayResults(results);
        
        return results;
    }

    /**
     * 計算性能評級
     */
    calculateGrade(timeToInteractive) {
        if (timeToInteractive < 500) return 'A+';
        if (timeToInteractive < 1000) return 'A';
        if (timeToInteractive < 1500) return 'B';
        if (timeToInteractive < 2000) return 'C';
        return 'D';
    }

    /**
     * 顯示測試結果
     */
    displayResults(results) {
        console.log('\n📊 OPUI 首頁性能測試結果');
        console.log('=' .repeat(50));
        console.log(`🏆 性能評級: ${results.grade}`);
        console.log(`⚡ 用戶可互動時間: ${results.timeToInteractive.toFixed(2)} ms`);
        console.log(`🎨 首次 UI 渲染時間: ${results.firstUIRenderTime.toFixed(2)} ms`);
        console.log(`🚀 應用程式初始化時間: ${results.appInitTime.toFixed(2)} ms`);
        console.log(`🔄 背景同步時間: ${results.backgroundSyncTime.toFixed(2)} ms`);
        console.log(`📈 總載入時間: ${results.totalLoadTime.toFixed(2)} ms`);
        console.log('=' .repeat(50));
        
        // 性能建議
        this.showPerformanceRecommendations(results);
    }

    /**
     * 顯示性能建議
     */
    showPerformanceRecommendations(results) {
        console.log('\n💡 性能建議:');
        
        if (results.timeToInteractive > 1000) {
            console.log('⚠️  用戶可互動時間較長，建議進一步優化初始化流程');
        } else {
            console.log('✅ 用戶可互動時間良好');
        }
        
        if (results.firstUIRenderTime > 500) {
            console.log('⚠️  首次 UI 渲染時間較長，建議優化 DOM 操作');
        } else {
            console.log('✅ 首次 UI 渲染時間良好');
        }
        
        if (results.appInitTime > 300) {
            console.log('⚠️  應用程式初始化時間較長，建議進一步簡化初始化邏輯');
        } else {
            console.log('✅ 應用程式初始化時間良好');
        }
        
        console.log('\n🎯 優化目標:');
        console.log('• 用戶可互動時間 < 500ms (目標: A+ 級)');
        console.log('• 首次 UI 渲染時間 < 300ms');
        console.log('• 應用程式初始化時間 < 200ms');
    }

    /**
     * 獲取測試歷史
     */
    getTestHistory() {
        return this.testResults;
    }

    /**
     * 比較測試結果
     */
    compareResults(previousResults) {
        if (!previousResults || this.testResults.length === 0) {
            console.log('⚠️  沒有可比較的測試結果');
            return;
        }
        
        const current = this.testResults[this.testResults.length - 1];
        const improvement = {
            timeToInteractive: previousResults.timeToInteractive - current.timeToInteractive,
            firstUIRenderTime: previousResults.firstUIRenderTime - current.firstUIRenderTime,
            appInitTime: previousResults.appInitTime - current.appInitTime,
            totalLoadTime: previousResults.totalLoadTime - current.totalLoadTime
        };
        
        console.log('\n📈 性能改善對比:');
        console.log('=' .repeat(50));
        
        Object.entries(improvement).forEach(([key, value]) => {
            const icon = value > 0 ? '⬆️ ' : value < 0 ? '⬇️ ' : '➡️ ';
            const change = value > 0 ? `改善 ${value.toFixed(2)}ms` : 
                          value < 0 ? `退步 ${Math.abs(value).toFixed(2)}ms` : '無變化';
            console.log(`${icon} ${key}: ${change}`);
        });
    }

    /**
     * 重置測試
     */
    reset() {
        this.metrics = {
            domContentLoaded: 0,
            appInitStart: 0,
            appInitEnd: 0,
            firstUIRender: 0,
            backgroundSyncComplete: 0,
            totalLoadTime: 0,
            userInteractionReady: 0
        };
        this.isTestMode = false;
    }
}

// 建立全域性能測試實例
window.performanceTest = new PerformanceTest();

// 自動開始測試（如果在開發模式）
if (window.location.hostname === 'localhost' || window.location.hostname.includes('dev')) {
    document.addEventListener('DOMContentLoaded', () => {
        window.performanceTest.startTest();
    });
}

export { PerformanceTest };
