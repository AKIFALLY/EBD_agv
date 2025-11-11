/**
 * 地圖整合功能測試腳本
 * 驗證所有地圖整合功能的正常運作
 */

export const mapIntegrationTest = (() => {
    let testResults = [];
    let isRunning = false;

    // 測試配置
    const TEST_CONFIG = {
        timeout: 5000, // 5秒超時
        retries: 3,
        verbose: true
    };

    // 初始化測試
    function init() {
        console.log('Map integration test initialized');
    }

    // 運行所有測試
    async function runAllTests() {
        if (isRunning) {
            console.warn('Tests are already running');
            return;
        }

        isRunning = true;
        testResults = [];
        
        console.log('🚀 Starting map integration tests...');
        
        try {
            // 基礎功能測試
            await runBasicTests();
            
            // 權限系統測試
            await runPermissionTests();
            
            // 互動功能測試
            await runInteractionTests();
            
            // 資料同步測試
            await runDataSyncTests();
            
            // 效能測試
            await runPerformanceTests();
            
            // 審計記錄測試
            await runAuditTests();
            
            // 生成測試報告
            generateTestReport();
            
        } catch (error) {
            console.error('Test suite failed:', error);
        } finally {
            isRunning = false;
        }
    }

    // 基礎功能測試
    async function runBasicTests() {
        console.log('📋 Running basic functionality tests...');
        
        // 測試地圖初始化
        await runTest('Map Initialization', () => {
            return window.mapPage && typeof window.mapPage.setup === 'function';
        });
        
        // 測試工具列存在
        await runTest('Toolbar Exists', () => {
            return document.querySelector('.map-toolbar') !== null;
        });
        
        // 測試側邊面板存在
        await runTest('Sidebar Exists', () => {
            return document.getElementById('map-sidebar') !== null;
        });
        
        // 測試彈出視窗存在
        await runTest('Popup Exists', () => {
            return document.getElementById('map-popup') !== null;
        });
        
        // 測試圖例存在
        await runTest('Legend Exists', () => {
            return document.getElementById('map-legend') !== null;
        });
    }

    // 權限系統測試
    async function runPermissionTests() {
        console.log('🔐 Running permission system tests...');
        
        // 測試權限管理器初始化
        await runTest('Permission Manager Initialized', () => {
            return window.mapPermissions && typeof window.mapPermissions.hasPermission === 'function';
        });
        
        // 測試權限檢查
        await runTest('Permission Check Works', () => {
            return typeof window.mapPermissions.hasPermission('view_tasks') === 'boolean';
        });
        
        // 測試用戶資訊獲取
        await runTest('User Info Available', () => {
            const userInfo = window.mapPermissions.getUserInfo();
            return userInfo && typeof userInfo.role === 'string';
        });
    }

    // 互動功能測試
    async function runInteractionTests() {
        console.log('🖱️ Running interaction tests...');
        
        // 測試地圖互動管理器
        await runTest('Map Interaction Manager', () => {
            return window.mapInteraction && typeof window.mapInteraction.showPopup === 'function';
        });
        
        // 測試物件管理器
        await runTest('Object Manager', () => {
            return window.mapObjectManager && typeof window.mapObjectManager.init === 'function';
        });
        
        // 測試工具列按鈕點擊
        await runTest('Toolbar Button Click', () => {
            const button = document.getElementById('map-tool-tasks');
            if (button) {
                button.click();
                return true;
            }
            return false;
        });
    }

    // 資料同步測試
    async function runDataSyncTests() {
        console.log('🔄 Running data sync tests...');
        
        // 測試資料同步管理器
        await runTest('Data Sync Manager', () => {
            return window.mapDataSync && typeof window.mapDataSync.syncAllData === 'function';
        });
        
        // 測試貨架管理器
        await runTest('Rack Manager', () => {
            return window.mapRackManager && typeof window.mapRackManager.loadRackData === 'function';
        });
    }

    // 效能測試
    async function runPerformanceTests() {
        console.log('⚡ Running performance tests...');
        
        // 測試效能監控器
        await runTest('Performance Monitor', () => {
            return window.mapPerformanceMonitor && typeof window.mapPerformanceMonitor.getPerformanceReport === 'function';
        });
        
        // 測試記憶體使用
        await runTest('Memory Usage Check', () => {
            if ('memory' in performance) {
                const memory = performance.memory;
                return memory.usedJSHeapSize < memory.jsHeapSizeLimit * 0.8; // 使用量不超過80%
            }
            return true; // 如果不支援記憶體 API，視為通過
        });
        
        // 測試載入時間
        await runTest('Load Time Check', () => {
            const loadTime = performance.now();
            return loadTime < 10000; // 載入時間不超過10秒
        });
    }

    // 審計記錄測試
    async function runAuditTests() {
        console.log('📝 Running audit tests...');
        
        // 測試審計記錄器
        await runTest('Audit Logger', () => {
            return window.mapAuditLogger && typeof window.mapAuditLogger.logView === 'function';
        });
        
        // 測試記錄功能
        await runTest('Audit Logging Works', () => {
            if (window.mapAuditLogger) {
                window.mapAuditLogger.logView('test', 'test_id', { test: true });
                const history = window.mapAuditLogger.getHistory();
                return history.length > 0;
            }
            return false;
        });
    }

    // 運行單個測試
    async function runTest(testName, testFunction, retries = TEST_CONFIG.retries) {
        const startTime = performance.now();
        
        try {
            const result = await Promise.race([
                Promise.resolve(testFunction()),
                new Promise((_, reject) => 
                    setTimeout(() => reject(new Error('Test timeout')), TEST_CONFIG.timeout)
                )
            ]);
            
            const duration = performance.now() - startTime;
            const testResult = {
                name: testName,
                status: result ? 'PASS' : 'FAIL',
                duration: Math.round(duration),
                error: result ? null : 'Test returned false'
            };
            
            testResults.push(testResult);
            
            if (TEST_CONFIG.verbose) {
                console.log(`${result ? '✅' : '❌'} ${testName} (${testResult.duration}ms)`);
            }
            
            return result;
            
        } catch (error) {
            const duration = performance.now() - startTime;
            
            if (retries > 0) {
                console.warn(`⚠️ ${testName} failed, retrying... (${retries} retries left)`);
                await new Promise(resolve => setTimeout(resolve, 1000)); // 等待1秒後重試
                return runTest(testName, testFunction, retries - 1);
            }
            
            const testResult = {
                name: testName,
                status: 'ERROR',
                duration: Math.round(duration),
                error: error.message
            };
            
            testResults.push(testResult);
            
            if (TEST_CONFIG.verbose) {
                console.log(`❌ ${testName} (${testResult.duration}ms) - ${error.message}`);
            }
            
            return false;
        }
    }

    // 生成測試報告
    function generateTestReport() {
        const totalTests = testResults.length;
        const passedTests = testResults.filter(t => t.status === 'PASS').length;
        const failedTests = testResults.filter(t => t.status === 'FAIL').length;
        const errorTests = testResults.filter(t => t.status === 'ERROR').length;
        const totalDuration = testResults.reduce((sum, t) => sum + t.duration, 0);
        
        const report = {
            summary: {
                total: totalTests,
                passed: passedTests,
                failed: failedTests,
                errors: errorTests,
                successRate: Math.round((passedTests / totalTests) * 100),
                totalDuration: totalDuration
            },
            details: testResults,
            timestamp: new Date().toISOString()
        };
        
        console.log('\n📊 Test Report:');
        console.log(`Total Tests: ${totalTests}`);
        console.log(`Passed: ${passedTests} ✅`);
        console.log(`Failed: ${failedTests} ❌`);
        console.log(`Errors: ${errorTests} 💥`);
        console.log(`Success Rate: ${report.summary.successRate}%`);
        console.log(`Total Duration: ${totalDuration}ms`);
        
        if (failedTests > 0 || errorTests > 0) {
            console.log('\n❌ Failed/Error Tests:');
            testResults
                .filter(t => t.status !== 'PASS')
                .forEach(t => {
                    console.log(`  - ${t.name}: ${t.error}`);
                });
        }
        
        // 保存報告到 localStorage
        localStorage.setItem('map_test_report', JSON.stringify(report));
        
        return report;
    }

    // 獲取最新測試報告
    function getLastTestReport() {
        try {
            const stored = localStorage.getItem('map_test_report');
            return stored ? JSON.parse(stored) : null;
        } catch (error) {
            console.error('Failed to load test report:', error);
            return null;
        }
    }

    // 運行快速測試
    async function runQuickTest() {
        console.log('🏃‍♂️ Running quick test...');
        
        const quickTests = [
            () => window.mapPage !== undefined,
            () => document.querySelector('.map-toolbar') !== null,
            () => window.mapPermissions !== undefined,
            () => window.mapInteraction !== undefined
        ];
        
        let passed = 0;
        for (const test of quickTests) {
            if (test()) passed++;
        }
        
        const result = {
            passed: passed,
            total: quickTests.length,
            successRate: Math.round((passed / quickTests.length) * 100)
        };
        
        console.log(`Quick test result: ${passed}/${quickTests.length} (${result.successRate}%)`);
        return result;
    }

    // 公開方法
    return {
        init,
        runAllTests,
        runQuickTest,
        getLastTestReport,
        generateTestReport
    };
})();

// 全域暴露
window.mapIntegrationTest = mapIntegrationTest;

// 開發模式下自動運行快速測試
if (window.location.hostname === 'localhost' || window.location.hostname === '127.0.0.1') {
    document.addEventListener('DOMContentLoaded', () => {
        setTimeout(() => {
            mapIntegrationTest.init();
            mapIntegrationTest.runQuickTest();
        }, 2000);
    });
}
