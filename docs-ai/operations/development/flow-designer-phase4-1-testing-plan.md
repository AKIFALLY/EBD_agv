# Flow Designer Phase 4.1 系統完整性測試計劃

## 🎯 測試目標
驗證 Flow Designer 完整功能的穩定性、一致性和可靠性，確保生產環境部署準備就緒。

## 📋 測試範疇

### 1. 端到端功能測試
- 完整的雙向轉換流程
- 所有節點類型的創建和配置
- 視覺化流程圖的操作和編輯
- YAML DSL 的生成和載入

### 2. 雙向轉換一致性驗證
- Visual → YAML → Visual 往返測試
- 38個 WCS 函數的完整覆蓋
- 參數和元資料的保真度測試
- 複雜流程圖的轉換準確性

### 3. 錯誤處理機制驗證
- 異常檔案格式處理
- 網路中斷情況處理
- 瀏覽器相容性測試
- 邊界條件和極端情況

## 🧪 測試用例設計

### 測試用例 1：基本雙向轉換
```yaml
目標: 驗證基本的雙向轉換功能
步驟:
1. 創建包含3個節點的簡單流程圖
2. 生成 YAML DSL
3. 清空編輯器
4. 載入生成的 YAML DSL
5. 驗證流程圖完全一致

預期結果:
- 節點數量一致
- 節點類型正確
- 連接關係保持
- 參數值不變
```

### 測試用例 2：38個 WCS 函數覆蓋
```yaml
目標: 驗證所有 WCS 函數的節點映射
測試數據:
- condition_nodes: 9個函數
- logic_nodes: 5個函數  
- action_nodes: 4個函數
- script_nodes: 控制結構

驗證內容:
- 函數名正確映射到節點類型
- 參數定義完整
- 輸入輸出接口正確
- 視覺樣式應用正確
```

### 測試用例 3：複雜流程圖測試
```yaml
目標: 測試複雜業務流程的處理能力
流程複雜度:
- 10+ 節點
- 多分支結構
- 混合節點類型
- 複雜參數配置

驗證指標:
- 渲染效能 < 2秒
- 記憶體使用穩定
- 視覺布局合理
- 互動響應流暢
```

### 測試用例 4：錯誤處理測試
```yaml
異常情況:
- 無效的 YAML 格式
- 缺失必要欄位
- 不支援的函數名
- 檔案讀取失敗

預期行為:
- 友好的錯誤提示
- 不影響現有流程圖
- 提供解決建議
- 日誌記錄完整
```

## 🔧 自動化測試腳本

### JavaScript 測試框架
```javascript
// flow-designer-test-suite.js
class FlowDesignerTestSuite {
    constructor() {
        this.testResults = [];
        this.flowDesigner = null;
    }

    async runAllTests() {
        console.log('🚀 開始 Flow Designer 完整性測試...');
        
        await this.testBasicBidirectionalConversion();
        await this.testAllWCSFunctions();
        await this.testComplexFlowProcessing();
        await this.testErrorHandling();
        
        this.generateTestReport();
    }

    async testBasicBidirectionalConversion() {
        console.log('🔄 測試基本雙向轉換...');
        
        try {
            // 1. 創建測試流程圖
            const originalFlow = await this.createTestFlow();
            
            // 2. 生成 YAML DSL
            const yamlContent = await this.flowDesigner.generateYamlDsl();
            
            // 3. 清空並重新載入
            await this.flowDesigner.clearFlow();
            await this.flowDesigner.parseDslToFlow(yamlContent);
            
            // 4. 驗證一致性
            const regeneratedFlow = await this.extractCurrentFlow();
            const isConsistent = this.compareFlows(originalFlow, regeneratedFlow);
            
            this.recordTestResult('基本雙向轉換', isConsistent, {
                originalNodes: originalFlow.nodes.length,
                regeneratedNodes: regeneratedFlow.nodes.length,
                parametersMatch: this.compareParameters(originalFlow, regeneratedFlow)
            });
            
        } catch (error) {
            this.recordTestResult('基本雙向轉換', false, { error: error.message });
        }
    }

    async testAllWCSFunctions() {
        console.log('🔍 測試所有 WCS 函數覆蓋...');
        
        const wcsTestCases = [
            // condition_nodes
            { function: 'check_agv_rotation_flow', type: 'condition_nodes' },
            { function: 'is_agv_at_location', type: 'condition_nodes' },
            { function: 'check_rack_availability', type: 'condition_nodes' },
            
            // logic_nodes
            { function: 'get_room_inlet_point', type: 'logic_nodes' },
            { function: 'get_agv_current_location', type: 'logic_nodes' },
            
            // action_nodes
            { function: 'create_task_from_decision', type: 'action_nodes' },
            { function: 'update_task_status', type: 'action_nodes' },
            
            // script_nodes
            { function: 'if_else', type: 'script_nodes' },
            { function: 'for_loop', type: 'script_nodes' }
        ];

        let passedTests = 0;
        const totalTests = wcsTestCases.length;

        for (const testCase of wcsTestCases) {
            try {
                const nodeId = await this.createNodeFromFunction(testCase.function, testCase.type);
                const nodeElement = document.getElementById(nodeId);
                
                if (nodeElement && this.validateNodeProperties(nodeElement, testCase)) {
                    passedTests++;
                }
            } catch (error) {
                console.error(`❌ 函數 ${testCase.function} 測試失敗:`, error);
            }
        }

        const successRate = (passedTests / totalTests) * 100;
        this.recordTestResult('WCS 函數覆蓋', successRate >= 95, {
            passedTests,
            totalTests,
            successRate: `${successRate.toFixed(1)}%`
        });
    }

    async testComplexFlowProcessing() {
        console.log('🏗️ 測試複雜流程處理...');
        
        const startTime = performance.now();
        const initialMemory = this.getMemoryUsage();
        
        try {
            // 創建複雜流程圖（15個節點）
            const complexFlow = await this.createComplexTestFlow(15);
            
            // 測試渲染效能
            const renderTime = performance.now() - startTime;
            const finalMemory = this.getMemoryUsage();
            const memoryIncrease = finalMemory - initialMemory;
            
            // 測試互動響應
            const interactionTestResults = await this.testInteractionPerformance();
            
            const performanceAcceptable = renderTime < 2000 && memoryIncrease < 50; // 2秒渲染，50MB記憶體增長
            
            this.recordTestResult('複雜流程處理', performanceAcceptable, {
                renderTime: `${renderTime.toFixed(0)}ms`,
                memoryIncrease: `${memoryIncrease.toFixed(1)}MB`,
                nodeCount: complexFlow.nodes.length,
                interactionResults: interactionTestResults
            });
            
        } catch (error) {
            this.recordTestResult('複雜流程處理', false, { error: error.message });
        }
    }

    async testErrorHandling() {
        console.log('⚠️ 測試錯誤處理機制...');
        
        const errorTestCases = [
            {
                name: '無效 YAML 格式',
                input: 'invalid: yaml: content: [unclosed',
                expectError: true
            },
            {
                name: '缺失 steps 欄位',
                input: 'variables:\n  test: value',
                expectError: true
            },
            {
                name: '未知函數名稱',
                input: 'steps:\n  - function: unknown_function\n    type: condition_nodes',
                expectError: false // 應該警告但不中斷
            }
        ];

        let errorHandlingPassed = 0;
        
        for (const testCase of errorTestCases) {
            try {
                await this.flowDesigner.parseDslToFlow(testCase.input);
                
                if (!testCase.expectError) {
                    errorHandlingPassed++;
                }
            } catch (error) {
                if (testCase.expectError) {
                    errorHandlingPassed++;
                }
            }
        }

        const errorHandlingSuccess = errorHandlingPassed === errorTestCases.length;
        this.recordTestResult('錯誤處理機制', errorHandlingSuccess, {
            passedTests: errorHandlingPassed,
            totalTests: errorTestCases.length
        });
    }

    // 輔助方法
    recordTestResult(testName, passed, details = {}) {
        this.testResults.push({
            name: testName,
            passed,
            details,
            timestamp: new Date().toISOString()
        });
        
        const status = passed ? '✅' : '❌';
        console.log(`${status} ${testName}: ${passed ? '通過' : '失敗'}`, details);
    }

    generateTestReport() {
        const totalTests = this.testResults.length;
        const passedTests = this.testResults.filter(r => r.passed).length;
        const successRate = (passedTests / totalTests) * 100;
        
        console.log('\n📊 Flow Designer 完整性測試報告');
        console.log('═══════════════════════════════════');
        console.log(`總測試數: ${totalTests}`);
        console.log(`通過測試: ${passedTests}`);
        console.log(`成功率: ${successRate.toFixed(1)}%`);
        console.log(`測試狀態: ${successRate >= 95 ? '✅ 通過' : '❌ 需要修復'}`);
        
        // 詳細結果
        this.testResults.forEach(result => {
            console.log(`\n${result.passed ? '✅' : '❌'} ${result.name}`);
            if (result.details && Object.keys(result.details).length > 0) {
                console.log('  詳細資訊:', result.details);
            }
        });
        
        return {
            totalTests,
            passedTests,
            successRate,
            passed: successRate >= 95,
            details: this.testResults
        };
    }
}

// 測試執行器
window.FlowDesignerTestSuite = FlowDesignerTestSuite;
```

## 🏃‍♂️ 測試執行流程

### 手動測試步驟
1. **開啟 Flow Designer**
   ```bash
   # 啟動 AGVC 系統
   agvc_start
   
   # 開啟瀏覽器
   http://localhost:8001/flows
   ```

2. **載入測試腳本**
   ```javascript
   // 在瀏覽器開發工具控制台執行
   const testSuite = new FlowDesignerTestSuite();
   testSuite.flowDesigner = window.flowDesigner; // 假設全局可用
   await testSuite.runAllTests();
   ```

3. **驗證測試結果**
   - 檢查控制台輸出
   - 驗證成功率 ≥ 95%
   - 記錄失敗的測試用例

### 自動化測試腳本
```bash
#!/bin/bash
# flow-designer-test-runner.sh

echo "🚀 啟動 Flow Designer 完整性測試"

# 1. 確保 AGVC 系統運行
echo "📋 檢查 AGVC 系統狀態..."
r agvc-check

# 2. 啟動測試瀏覽器會話
echo "🌐 啟動測試瀏覽器..."
# 使用 Playwright 或 Selenium 進行自動化測試

# 3. 執行測試套件
echo "🧪 執行測試套件..."
# 注入測試腳本並執行

# 4. 生成測試報告
echo "📊 生成測試報告..."
# 收集測試結果並生成報告

echo "✅ Flow Designer 完整性測試完成"
```

## 📊 測試指標和成功標準

### 成功標準
- **功能測試**: 100% 核心功能正常
- **轉換一致性**: ≥ 99.5% 數據保真度
- **WCS 函數覆蓋**: ≥ 95% 函數正確映射
- **錯誤處理**: 100% 異常情況優雅處理
- **效能指標**: 
  - 渲染時間 < 2秒 (10節點流程)
  - 記憶體增長 < 50MB
  - 載入時間 < 1秒

### 測試報告格式
```json
{
  "testSuite": "Flow Designer Phase 4.1",
  "timestamp": "2024-01-15T10:30:00Z",
  "summary": {
    "totalTests": 25,
    "passedTests": 24,
    "successRate": 96.0,
    "status": "PASSED"
  },
  "categories": {
    "bidirectionalConversion": { "passed": true, "details": "..." },
    "wcsFunctionCoverage": { "passed": true, "successRate": "97.4%" },
    "complexFlowProcessing": { "passed": true, "renderTime": "1.2s" },
    "errorHandling": { "passed": true, "coverage": "100%" }
  },
  "recommendations": [
    "建議優化記憶體使用",
    "增加更多邊界條件測試"
  ]
}
```

## 🔧 問題修復流程

### 發現問題時的處理步驟
1. **記錄問題詳情**
2. **分析根本原因**
3. **制定修復方案**
4. **實施修復**
5. **重新測試驗證**
6. **更新測試用例**

### 常見問題和解決方案
- **轉換不一致**: 檢查節點映射邏輯
- **效能問題**: 分析 DOM 操作和記憶體洩漏
- **錯誤處理不當**: 增強異常捕獲機制
- **瀏覽器相容性**: 測試不同瀏覽器版本

Phase 4.1 的完整性測試將確保 Flow Designer 達到生產環境的品質標準。