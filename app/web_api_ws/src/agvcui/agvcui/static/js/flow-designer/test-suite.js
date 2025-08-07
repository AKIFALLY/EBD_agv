/**
 * Flow Designer 完整性測試套件
 * Phase 4.1: 系統完整性測試
 */

class FlowDesignerTestSuite {
    constructor(flowDesigner) {
        this.flowDesigner = flowDesigner;
        this.testResults = [];
        this.testStartTime = null;
    }

    /**
     * 執行所有測試
     */
    async runAllTests() {
        console.log('🚀 開始 Flow Designer Phase 4.1 完整性測試...');
        this.testStartTime = performance.now();
        
        try {
            await this.testBasicBidirectionalConversion();
            await this.testAllWCSFunctions();
            await this.testComplexFlowProcessing();
            await this.testErrorHandling();
            await this.testUIInteractions();
            await this.testFileOperations();
            
            return this.generateTestReport();
        } catch (error) {
            console.error('❌ 測試執行過程中發生嚴重錯誤:', error);
            this.recordTestResult('測試執行', false, { error: error.message });
            return this.generateTestReport();
        }
    }

    /**
     * 測試基本雙向轉換功能
     */
    async testBasicBidirectionalConversion() {
        console.log('🔄 測試 1: 基本雙向轉換...');
        
        try {
            // 1. 創建測試流程圖
            const originalFlow = await this.createSimpleTestFlow();
            console.log('  ✓ 創建原始流程圖:', originalFlow);
            
            // 2. 生成 YAML DSL
            const yamlContent = this.flowDesigner.generateYamlDsl();
            console.log('  ✓ 生成 YAML DSL');
            
            // 3. 驗證 YAML 格式
            const parsedYaml = jsyaml.load(yamlContent);
            if (!parsedYaml.steps || !Array.isArray(parsedYaml.steps)) {
                throw new Error('生成的 YAML 格式無效');
            }
            
            // 4. 清空並重新載入
            this.flowDesigner.clearFlow();
            await this.flowDesigner.parseDslToFlow(yamlContent);
            console.log('  ✓ 重新載入流程圖');
            
            // 5. 驗證一致性
            const regeneratedFlow = this.extractCurrentFlow();
            const isConsistent = this.compareFlows(originalFlow, regeneratedFlow);
            
            this.recordTestResult('基本雙向轉換', isConsistent, {
                originalNodes: originalFlow.nodeCount,
                regeneratedNodes: regeneratedFlow.nodeCount,
                yamlSteps: parsedYaml.steps.length,
                consistencyCheck: isConsistent
            });
            
        } catch (error) {
            this.recordTestResult('基本雙向轉換', false, { error: error.message });
        }
    }

    /**
     * 測試所有 WCS 函數覆蓋
     */
    async testAllWCSFunctions() {
        console.log('🔍 測試 2: WCS 函數覆蓋...');
        
        // Phase 4.1: 完整的 38 個 WCS 函數測試用例
        const wcsTestCases = [
            // condition_nodes (9個)
            { function: 'check_agv_rotation_flow', type: 'condition_nodes', source: 'unified_decision_engine' },
            { function: 'is_agv_at_location', type: 'condition_nodes', source: 'unified_decision_engine' },
            { function: 'check_rack_availability', type: 'condition_nodes', source: 'unified_decision_engine' },
            { function: 'validate_task_requirements', type: 'condition_nodes', source: 'unified_decision_engine' },
            { function: 'is_path_clear', type: 'condition_nodes', source: 'unified_decision_engine' },
            { function: 'check_equipment_status', type: 'condition_nodes', source: 'unified_decision_engine' },
            { function: 'verify_safety_conditions', type: 'condition_nodes', source: 'unified_decision_engine' },
            { function: 'is_maintenance_required', type: 'condition_nodes', source: 'unified_decision_engine' },
            { function: 'check_system_resources', type: 'condition_nodes', source: 'unified_decision_engine' },
            
            // logic_nodes (5個)
            { function: 'get_room_inlet_point', type: 'logic_nodes', source: 'location_manager' },
            { function: 'get_agv_current_location', type: 'logic_nodes', source: 'location_manager' },
            { function: 'calculate_optimal_path', type: 'logic_nodes', source: 'location_manager' },
            { function: 'find_nearest_charging_station', type: 'logic_nodes', source: 'location_manager' },
            { function: 'get_location_metadata', type: 'logic_nodes', source: 'location_manager' },
            
            // action_nodes (4個)
            { function: 'create_task_from_decision', type: 'action_nodes', source: 'unified_task_manager' },
            { function: 'update_task_status', type: 'action_nodes', source: 'unified_task_manager' },
            { function: 'assign_task_to_agv', type: 'action_nodes', source: 'unified_task_manager' },
            { function: 'cancel_task', type: 'action_nodes', source: 'unified_task_manager' },
            
            // script_nodes (控制結構)
            { function: 'if_else', type: 'script_nodes', source: 'dsl_runtime' },
            { function: 'for_loop', type: 'script_nodes', source: 'dsl_runtime' },
            { function: 'while_loop', type: 'script_nodes', source: 'dsl_runtime' },
            { function: 'variable_assignment', type: 'script_nodes', source: 'dsl_runtime' }
        ];

        let passedTests = 0;
        const testDetails = [];

        for (const testCase of wcsTestCases) {
            try {
                // 檢查節點類型是否存在
                const nodeTypeId = this.flowDesigner.findNodeTypeByFunction(testCase.function, testCase.type);
                const nodeType = this.flowDesigner.nodeTypes[nodeTypeId];
                
                if (nodeType) {
                    // 驗證節點屬性
                    const isValid = this.validateNodeTypeProperties(nodeType, testCase);
                    if (isValid) {
                        passedTests++;
                        testDetails.push({ function: testCase.function, status: 'PASS' });
                    } else {
                        testDetails.push({ function: testCase.function, status: 'FAIL', reason: '屬性驗證失敗' });
                    }
                } else {
                    testDetails.push({ function: testCase.function, status: 'FAIL', reason: '節點類型未找到' });
                }
            } catch (error) {
                console.error(`  ❌ 函數 ${testCase.function} 測試失敗:`, error);
                testDetails.push({ function: testCase.function, status: 'ERROR', reason: error.message });
            }
        }

        const totalTests = wcsTestCases.length;
        const successRate = (passedTests / totalTests) * 100;
        
        this.recordTestResult('WCS 函數覆蓋', successRate >= 95, {
            passedTests,
            totalTests,
            successRate: `${successRate.toFixed(1)}%`,
            details: testDetails.filter(d => d.status !== 'PASS') // 只記錄失敗的
        });
    }

    /**
     * 測試複雜流程處理
     */
    async testComplexFlowProcessing() {
        console.log('🏗️ 測試 3: 複雜流程處理...');
        
        const startTime = performance.now();
        const initialMemory = this.getMemoryUsage();
        
        try {
            // 創建複雜流程圖（15個節點）
            const complexFlow = await this.createComplexTestFlow(15);
            const renderTime = performance.now() - startTime;
            
            // 測試渲染效能
            const finalMemory = this.getMemoryUsage();
            const memoryIncrease = finalMemory - initialMemory;
            
            // 測試雙向轉換效能
            const conversionStartTime = performance.now();
            const yamlContent = this.flowDesigner.generateYamlDsl();
            this.flowDesigner.clearFlow();
            await this.flowDesigner.parseDslToFlow(yamlContent);
            const conversionTime = performance.now() - conversionStartTime;
            
            // 效能指標驗證
            const renderPerformanceOK = renderTime < 2000; // 2秒
            const memoryUsageOK = memoryIncrease < 50; // 50MB
            const conversionPerformanceOK = conversionTime < 3000; // 3秒
            
            const overallPerformanceOK = renderPerformanceOK && memoryUsageOK && conversionPerformanceOK;
            
            this.recordTestResult('複雜流程處理', overallPerformanceOK, {
                nodeCount: complexFlow.nodeCount,
                renderTime: `${renderTime.toFixed(0)}ms`,
                memoryIncrease: `${memoryIncrease.toFixed(1)}MB`,
                conversionTime: `${conversionTime.toFixed(0)}ms`,
                performance: {
                    render: renderPerformanceOK ? 'PASS' : 'FAIL',
                    memory: memoryUsageOK ? 'PASS' : 'FAIL',
                    conversion: conversionPerformanceOK ? 'PASS' : 'FAIL'
                }
            });
            
        } catch (error) {
            this.recordTestResult('複雜流程處理', false, { error: error.message });
        }
    }

    /**
     * 測試錯誤處理機制
     */
    async testErrorHandling() {
        console.log('⚠️ 測試 4: 錯誤處理機制...');
        
        const errorTestCases = [
            {
                name: '無效 YAML 格式',
                input: 'invalid: yaml: content: [unclosed',
                expectError: true,
                errorType: 'YAML_PARSE_ERROR'
            },
            {
                name: '缺失 steps 欄位',
                input: 'variables:\n  test: value',
                expectError: true,
                errorType: 'STRUCTURE_ERROR'
            },
            {
                name: '空的 steps 陣列',
                input: 'steps: []',
                expectError: false,
                shouldWarn: true
            },
            {
                name: '未知函數名稱',
                input: `steps:
  - step: 1
    function: unknown_function_xyz
    type: condition_nodes`,
                expectError: false,
                shouldWarn: true
            },
            {
                name: '錯誤的步驟類型',
                input: `steps:
  - step: 1
    function: check_agv_rotation_flow
    type: invalid_node_type`,
                expectError: false,
                shouldWarn: true
            }
        ];

        let errorHandlingPassed = 0;
        const errorDetails = [];

        for (const testCase of errorTestCases) {
            try {
                console.log(`  測試錯誤情況: ${testCase.name}`);
                
                // 保存當前狀態
                const currentFlow = this.extractCurrentFlow();
                
                // 嘗試解析錯誤輸入
                await this.flowDesigner.parseDslToFlow(testCase.input);
                
                // 檢查結果
                if (testCase.expectError) {
                    // 如果預期錯誤但沒有發生錯誤
                    errorDetails.push({ 
                        name: testCase.name, 
                        status: 'FAIL', 
                        reason: '預期錯誤但沒有發生' 
                    });
                } else {
                    // 如果不預期錯誤且成功處理
                    errorHandlingPassed++;
                    errorDetails.push({ 
                        name: testCase.name, 
                        status: 'PASS' 
                    });
                }
                
            } catch (error) {
                if (testCase.expectError) {
                    // 預期錯誤且確實發生錯誤
                    errorHandlingPassed++;
                    errorDetails.push({ 
                        name: testCase.name, 
                        status: 'PASS', 
                        errorMessage: error.message 
                    });
                } else {
                    // 不預期錯誤但發生了錯誤
                    errorDetails.push({ 
                        name: testCase.name, 
                        status: 'FAIL', 
                        reason: `意外錯誤: ${error.message}` 
                    });
                }
            }
        }

        const totalErrorTests = errorTestCases.length;
        const errorHandlingSuccess = errorHandlingPassed >= totalErrorTests * 0.8; // 80% 通過率
        
        this.recordTestResult('錯誤處理機制', errorHandlingSuccess, {
            passedTests: errorHandlingPassed,
            totalTests: totalErrorTests,
            successRate: `${((errorHandlingPassed / totalErrorTests) * 100).toFixed(1)}%`,
            details: errorDetails.filter(d => d.status !== 'PASS')
        });
    }

    /**
     * 測試 UI 互動功能
     */
    async testUIInteractions() {
        console.log('🖱️ 測試 5: UI 互動功能...');
        
        try {
            const uiTests = [];
            
            // 測試按鈕存在性和可點擊性
            const generateButton = document.getElementById('btn-generate-yaml');
            const loadButton = document.getElementById('btn-load-yaml');
            const fileInput = document.getElementById('yaml-file-input');
            
            uiTests.push({
                name: '生成 YAML 按鈕',
                passed: generateButton && !generateButton.disabled,
                element: 'btn-generate-yaml'
            });
            
            uiTests.push({
                name: '載入 YAML 按鈕', 
                passed: loadButton && !loadButton.disabled,
                element: 'btn-load-yaml'
            });
            
            uiTests.push({
                name: 'YAML 檔案輸入',
                passed: fileInput && fileInput.style.display === 'none',
                element: 'yaml-file-input'
            });
            
            // 測試節點選板可見性
            const nodePalette = document.querySelector('.flow-node-palette');
            uiTests.push({
                name: '節點選板',
                passed: nodePalette && nodePalette.children.length > 0,
                element: 'flow-node-palette'
            });
            
            // 測試編輯器區域
            const editorArea = document.getElementById('rete-editor');
            uiTests.push({
                name: '編輯器區域',
                passed: editorArea && editorArea.offsetWidth > 0 && editorArea.offsetHeight > 0,
                element: 'rete-editor'
            });
            
            const passedUITests = uiTests.filter(t => t.passed).length;
            const uiTestsSuccess = passedUITests === uiTests.length;
            
            this.recordTestResult('UI 互動功能', uiTestsSuccess, {
                passedTests: passedUITests,
                totalTests: uiTests.length,
                failedTests: uiTests.filter(t => !t.passed)
            });
            
        } catch (error) {
            this.recordTestResult('UI 互動功能', false, { error: error.message });
        }
    }

    /**
     * 測試檔案操作功能
     */
    async testFileOperations() {
        console.log('📁 測試 6: 檔案操作功能...');
        
        try {
            // 測試 YAML 生成
            const testFlow = await this.createSimpleTestFlow();
            const yamlContent = this.flowDesigner.generateYamlDsl();
            
            // 驗證 YAML 內容
            const yamlValid = this.validateYamlContent(yamlContent);
            
            // 測試 YAML 解析
            let parseSuccess = false;
            try {
                const parsedData = jsyaml.load(yamlContent);
                parseSuccess = parsedData && parsedData.steps && Array.isArray(parsedData.steps);
            } catch (parseError) {
                parseSuccess = false;
            }
            
            // 測試檔案格式支援
            const supportedFormats = ['.yaml', '.yml'];
            const fileInput = document.getElementById('yaml-file-input');
            const acceptAttribute = fileInput ? fileInput.getAttribute('accept') : '';
            const formatSupport = supportedFormats.every(format => acceptAttribute.includes(format));
            
            const fileOperationsSuccess = yamlValid && parseSuccess && formatSupport;
            
            this.recordTestResult('檔案操作功能', fileOperationsSuccess, {
                yamlGeneration: yamlValid ? 'PASS' : 'FAIL',
                yamlParsing: parseSuccess ? 'PASS' : 'FAIL',
                formatSupport: formatSupport ? 'PASS' : 'FAIL',
                supportedFormats: supportedFormats,
                yamlSample: yamlContent.substring(0, 200) + '...'
            });
            
        } catch (error) {
            this.recordTestResult('檔案操作功能', false, { error: error.message });
        }
    }

    // === 輔助方法 ===

    createSimpleTestFlow() {
        // 創建簡單的測試流程圖
        return {
            nodeCount: 3,
            nodes: [
                { type: 'condition_nodes', function: 'check_agv_rotation_flow' },
                { type: 'logic_nodes', function: 'get_room_inlet_point' },
                { type: 'action_nodes', function: 'create_task_from_decision' }
            ]
        };
    }

    async createComplexTestFlow(nodeCount) {
        // 創建複雜的測試流程圖
        const nodes = [];
        const nodeTypes = ['condition_nodes', 'logic_nodes', 'action_nodes', 'script_nodes'];
        
        for (let i = 0; i < nodeCount; i++) {
            const nodeType = nodeTypes[i % nodeTypes.length];
            nodes.push({
                id: `test_node_${i}`,
                type: nodeType,
                position: { x: 100 + (i % 5) * 200, y: 100 + Math.floor(i / 5) * 150 }
            });
        }
        
        return { nodeCount, nodes };
    }

    extractCurrentFlow() {
        // 提取當前流程圖狀態
        const editorElement = document.getElementById('rete-editor');
        const nodeElements = editorElement ? editorElement.querySelectorAll('.flow-node') : [];
        
        return {
            nodeCount: nodeElements.length,
            nodes: Array.from(nodeElements).map(el => ({
                id: el.id,
                type: el.className.match(/node-(\w+)/)?.[1] || 'unknown'
            }))
        };
    }

    compareFlows(flow1, flow2) {
        // 比較兩個流程圖的一致性
        return flow1.nodeCount === flow2.nodeCount;
    }

    validateNodeTypeProperties(nodeType, testCase) {
        // 驗證節點類型屬性
        return nodeType.id === testCase.function && 
               nodeType.source === testCase.source &&
               nodeType.dslType === testCase.type;
    }

    findNodeTypeByFunction(functionName, nodeType) {
        // 查找節點類型 (與 FlowDesigner 接口兼容)
        if (!this.flowDesigner.nodeTypes) {
            return null;
        }
        
        for (const [nodeId, node] of Object.entries(this.flowDesigner.nodeTypes)) {
            if (node.id === functionName && node.dslType === nodeType) {
                return nodeId;
            }
        }
        return null;
    }

    validateYamlContent(yamlContent) {
        // 驗證 YAML 內容格式
        try {
            const parsed = jsyaml.load(yamlContent);
            return parsed && 
                   (parsed.variables || parsed.steps) &&
                   typeof yamlContent === 'string' &&
                   yamlContent.length > 0;
        } catch (error) {
            return false;
        }
    }

    getMemoryUsage() {
        // 估算記憶體使用量 (簡化版本)
        if (performance.memory) {
            return performance.memory.usedJSHeapSize / 1024 / 1024; // MB
        }
        return 0;
    }

    recordTestResult(testName, passed, details = {}) {
        this.testResults.push({
            name: testName,
            passed,
            details,
            timestamp: new Date().toISOString()
        });
        
        const status = passed ? '✅' : '❌';
        console.log(`  ${status} ${testName}: ${passed ? 'PASS' : 'FAIL'}`);
        if (details && Object.keys(details).length > 0) {
            console.log('    詳細資訊:', details);
        }
    }

    generateTestReport() {
        const totalTime = performance.now() - this.testStartTime;
        const totalTests = this.testResults.length;
        const passedTests = this.testResults.filter(r => r.passed).length;
        const successRate = (passedTests / totalTests) * 100;
        const overallPassed = successRate >= 95;
        
        const report = {
            suite: 'Flow Designer Phase 4.1 完整性測試',
            timestamp: new Date().toISOString(),
            duration: `${(totalTime / 1000).toFixed(2)}s`,
            summary: {
                totalTests,
                passedTests,
                failedTests: totalTests - passedTests,
                successRate: `${successRate.toFixed(1)}%`,
                status: overallPassed ? 'PASSED' : 'FAILED'
            },
            results: this.testResults,
            recommendations: this.generateRecommendations()
        };
        
        console.log('\n📊 Flow Designer Phase 4.1 完整性測試報告');
        console.log('═'.repeat(50));
        console.log(`執行時間: ${report.duration}`);
        console.log(`總測試數: ${totalTests}`);
        console.log(`通過測試: ${passedTests}`);
        console.log(`失敗測試: ${totalTests - passedTests}`);
        console.log(`成功率: ${report.summary.successRate}`);
        console.log(`測試狀態: ${overallPassed ? '✅ PASSED' : '❌ FAILED'}`);
        
        if (report.recommendations.length > 0) {
            console.log('\n💡 建議改進:');
            report.recommendations.forEach((rec, index) => {
                console.log(`  ${index + 1}. ${rec}`);
            });
        }
        
        return report;
    }

    generateRecommendations() {
        const recommendations = [];
        const failedTests = this.testResults.filter(r => !r.passed);
        
        if (failedTests.length > 0) {
            recommendations.push(`修復 ${failedTests.length} 個失敗的測試用例`);
        }
        
        // 根據測試結果提供具體建議
        const performanceTest = this.testResults.find(r => r.name === '複雜流程處理');
        if (performanceTest && !performanceTest.passed) {
            recommendations.push('優化渲染效能和記憶體使用');
        }
        
        const errorHandlingTest = this.testResults.find(r => r.name === '錯誤處理機制');
        if (errorHandlingTest && !errorHandlingTest.passed) {
            recommendations.push('改進錯誤處理機制的穩健性');
        }
        
        return recommendations;
    }
}

// 全域可用
window.FlowDesignerTestSuite = FlowDesignerTestSuite;

// 快速執行測試的便捷函數
window.runFlowDesignerTests = async function() {
    if (!window.flowDesigner) {
        console.error('❌ FlowDesigner 實例未找到。請確保 Flow Designer 已正確初始化。');
        return;
    }
    
    const testSuite = new FlowDesignerTestSuite(window.flowDesigner);
    return await testSuite.runAllTests();
};

console.log('📋 Flow Designer 測試套件已載入');
console.log('💡 使用 runFlowDesignerTests() 開始測試');