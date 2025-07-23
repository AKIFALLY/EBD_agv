# AI Agent 錯誤處理協定

## 📋 概述

本文檔定義 AI Agent 在 RosAGV 專案開發過程中的錯誤處理協定，基於現有測試檔案中的錯誤處理模式，建立完整的錯誤檢測、分類、處理和恢復機制。

## 🎯 錯誤處理目標

### 核心原則
- **快速檢測**: 及早發現和識別錯誤
- **智能分類**: 根據錯誤類型採用不同處理策略
- **自動恢復**: 優先嘗試自動修正和恢復
- **優雅降級**: 無法自動處理時提供有意義的錯誤資訊

### 處理範圍
- **需求解讀錯誤**: 需求描述不清、衝突、缺失
- **程式碼生成錯誤**: 語法錯誤、邏輯錯誤、架構不符
- **測試執行錯誤**: 測試失敗、覆蓋率不足、效能回歸
- **系統整合錯誤**: 依賴衝突、環境問題、資源不足

## 🚨 錯誤分類和檢測

### 錯誤嚴重性分級
```yaml
CRITICAL (嚴重):
  - 系統無法啟動
  - 資料庫連線失敗
  - 核心服務崩潰
  - 安全性漏洞

HIGH (高):
  - 功能完全失效
  - API 端點無回應
  - 測試覆蓋率 < 50%
  - 效能回歸 > 50%

MEDIUM (中):
  - 部分功能異常
  - 測試失敗率 > 20%
  - 程式碼風格不符
  - 文檔同步失敗

LOW (低):
  - 警告訊息
  - 程式碼建議
  - 效能優化建議
  - 文檔格式問題
```

### 錯誤檢測機制
```python
# AI Agent 錯誤檢測範例
class ErrorDetector:
    """AI Agent 錯誤檢測器"""
    
    def __init__(self):
        self.error_patterns = self._load_error_patterns()
        self.severity_rules = self._load_severity_rules()
    
    def detect_errors(self, context: ProcessingContext) -> List[DetectedError]:
        """檢測處理過程中的錯誤"""
        errors = []
        
        # 需求解讀錯誤檢測
        if context.stage == "requirement_analysis":
            errors.extend(self._detect_requirement_errors(context))
        
        # 程式碼生成錯誤檢測
        elif context.stage == "code_generation":
            errors.extend(self._detect_code_generation_errors(context))
        
        # 測試執行錯誤檢測
        elif context.stage == "test_execution":
            errors.extend(self._detect_test_errors(context))
        
        # 系統整合錯誤檢測
        elif context.stage == "system_integration":
            errors.extend(self._detect_integration_errors(context))
        
        return self._classify_and_prioritize(errors)
    
    def _detect_requirement_errors(self, context: ProcessingContext) -> List[DetectedError]:
        """檢測需求解讀錯誤"""
        errors = []
        
        # 需求描述完整性檢查
        if not context.requirement.description:
            errors.append(DetectedError(
                type="MISSING_REQUIREMENT_DESCRIPTION",
                severity="HIGH",
                message="需求描述缺失",
                context=context
            ))
        
        # 驗收標準檢查
        if not context.requirement.acceptance_criteria:
            errors.append(DetectedError(
                type="MISSING_ACCEPTANCE_CRITERIA",
                severity="MEDIUM",
                message="驗收標準缺失",
                context=context
            ))
        
        # 需求衝突檢查
        conflicts = self._check_requirement_conflicts(context.requirement)
        for conflict in conflicts:
            errors.append(DetectedError(
                type="REQUIREMENT_CONFLICT",
                severity="HIGH",
                message=f"需求衝突: {conflict}",
                context=context
            ))
        
        return errors
    
    def _detect_code_generation_errors(self, context: ProcessingContext) -> List[DetectedError]:
        """檢測程式碼生成錯誤"""
        errors = []
        
        # 語法錯誤檢查
        syntax_errors = self._check_syntax_errors(context.generated_code)
        for error in syntax_errors:
            errors.append(DetectedError(
                type="SYNTAX_ERROR",
                severity="HIGH",
                message=f"語法錯誤: {error}",
                context=context
            ))
        
        # 程式碼風格檢查
        style_violations = self._check_coding_standards(context.generated_code)
        for violation in style_violations:
            errors.append(DetectedError(
                type="STYLE_VIOLATION",
                severity="MEDIUM",
                message=f"程式碼風格違規: {violation}",
                context=context
            ))
        
        # 架構一致性檢查
        architecture_issues = self._check_architecture_compliance(context.generated_code)
        for issue in architecture_issues:
            errors.append(DetectedError(
                type="ARCHITECTURE_VIOLATION",
                severity="HIGH",
                message=f"架構違規: {issue}",
                context=context
            ))
        
        return errors
```

## 🔧 自動修正機制

### 需求解讀錯誤修正
```python
class RequirementErrorHandler:
    """需求解讀錯誤處理器"""
    
    def handle_missing_description(self, error: DetectedError) -> FixResult:
        """處理缺失需求描述"""
        try:
            # 嘗試從相關文檔推斷需求描述
            inferred_description = self._infer_description_from_context(error.context)
            
            if inferred_description:
                return FixResult(
                    success=True,
                    action="INFERRED_DESCRIPTION",
                    result=inferred_description,
                    confidence=0.7
                )
            else:
                return FixResult(
                    success=False,
                    action="REQUEST_HUMAN_INPUT",
                    message="無法推斷需求描述，需要人工補充"
                )
        except Exception as e:
            return FixResult(
                success=False,
                action="ERROR",
                message=f"修正失敗: {str(e)}"
            )
    
    def handle_requirement_conflict(self, error: DetectedError) -> FixResult:
        """處理需求衝突"""
        try:
            conflict_analysis = self._analyze_conflict(error.context)
            
            if conflict_analysis.resolvable:
                # 嘗試自動解決衝突
                resolution = self._resolve_conflict(conflict_analysis)
                return FixResult(
                    success=True,
                    action="CONFLICT_RESOLVED",
                    result=resolution,
                    confidence=0.8
                )
            else:
                # 標記需要人工介入
                return FixResult(
                    success=False,
                    action="ESCALATE_TO_HUMAN",
                    message=f"需求衝突需要人工解決: {conflict_analysis.description}"
                )
        except Exception as e:
            return FixResult(
                success=False,
                action="ERROR",
                message=f"衝突分析失敗: {str(e)}"
            )
```

### 程式碼生成錯誤修正
```python
class CodeGenerationErrorHandler:
    """程式碼生成錯誤處理器"""
    
    def handle_syntax_error(self, error: DetectedError) -> FixResult:
        """處理語法錯誤"""
        try:
            # 基於錯誤模式進行自動修正
            fixed_code = self._apply_syntax_fixes(
                error.context.generated_code,
                error.details
            )
            
            # 驗證修正結果
            if self._validate_syntax(fixed_code):
                return FixResult(
                    success=True,
                    action="SYNTAX_FIXED",
                    result=fixed_code,
                    confidence=0.9
                )
            else:
                return FixResult(
                    success=False,
                    action="REGENERATE_CODE",
                    message="語法修正失敗，需要重新生成程式碼"
                )
        except Exception as e:
            return FixResult(
                success=False,
                action="ERROR",
                message=f"語法修正失敗: {str(e)}"
            )
    
    def handle_style_violation(self, error: DetectedError) -> FixResult:
        """處理程式碼風格違規"""
        try:
            # 應用自動格式化工具
            formatted_code = self._apply_code_formatting(
                error.context.generated_code
            )
            
            # 檢查是否符合專案標準
            if self._check_style_compliance(formatted_code):
                return FixResult(
                    success=True,
                    action="STYLE_FIXED",
                    result=formatted_code,
                    confidence=0.95
                )
            else:
                # 嘗試手動修正
                manually_fixed = self._manual_style_fixes(formatted_code, error.details)
                return FixResult(
                    success=True,
                    action="MANUAL_STYLE_FIX",
                    result=manually_fixed,
                    confidence=0.8
                )
        except Exception as e:
            return FixResult(
                success=False,
                action="ERROR",
                message=f"風格修正失敗: {str(e)}"
            )
```

### 測試執行錯誤修正
```python
class TestExecutionErrorHandler:
    """測試執行錯誤處理器"""
    
    def handle_test_failure(self, error: DetectedError) -> FixResult:
        """處理測試失敗"""
        try:
            test_failure = error.context.test_failure
            
            # 分析失敗原因
            failure_analysis = self._analyze_test_failure(test_failure)
            
            if failure_analysis.type == "ASSERTION_ERROR":
                # 嘗試修正程式碼邏輯
                fixed_code = self._fix_logic_error(
                    error.context.generated_code,
                    failure_analysis
                )
                return FixResult(
                    success=True,
                    action="LOGIC_FIXED",
                    result=fixed_code,
                    confidence=0.7
                )
            
            elif failure_analysis.type == "DEPENDENCY_ERROR":
                # 修正依賴問題
                fixed_dependencies = self._fix_dependencies(failure_analysis)
                return FixResult(
                    success=True,
                    action="DEPENDENCIES_FIXED",
                    result=fixed_dependencies,
                    confidence=0.8
                )
            
            else:
                return FixResult(
                    success=False,
                    action="MANUAL_INVESTIGATION_REQUIRED",
                    message=f"測試失敗需要人工調查: {failure_analysis.description}"
                )
        
        except Exception as e:
            return FixResult(
                success=False,
                action="ERROR",
                message=f"測試錯誤分析失敗: {str(e)}"
            )
    
    def handle_coverage_insufficient(self, error: DetectedError) -> FixResult:
        """處理測試覆蓋率不足"""
        try:
            coverage_report = error.context.coverage_report
            
            # 識別未覆蓋的程式碼區域
            uncovered_areas = self._identify_uncovered_areas(coverage_report)
            
            # 生成額外的測試用例
            additional_tests = self._generate_additional_tests(uncovered_areas)
            
            return FixResult(
                success=True,
                action="TESTS_GENERATED",
                result=additional_tests,
                confidence=0.75
            )
        
        except Exception as e:
            return FixResult(
                success=False,
                action="ERROR",
                message=f"測試覆蓋率修正失敗: {str(e)}"
            )
```

## 🔄 錯誤恢復策略

### 重試機制
```python
class RetryStrategy:
    """重試策略管理器"""
    
    def __init__(self):
        self.max_retries = {
            "code_generation": 3,
            "test_execution": 2,
            "requirement_analysis": 1,
            "system_integration": 2
        }
        self.retry_delays = {
            "code_generation": [1, 2, 5],  # 秒
            "test_execution": [2, 5],
            "requirement_analysis": [1],
            "system_integration": [3, 10]
        }
    
    def should_retry(self, error: DetectedError, attempt: int) -> bool:
        """判斷是否應該重試"""
        max_attempts = self.max_retries.get(error.context.stage, 1)
        
        # 嚴重錯誤不重試
        if error.severity == "CRITICAL":
            return False
        
        # 超過最大重試次數
        if attempt >= max_attempts:
            return False
        
        # 特定錯誤類型不重試
        no_retry_types = [
            "REQUIREMENT_CONFLICT",
            "ARCHITECTURE_VIOLATION",
            "SECURITY_ISSUE"
        ]
        if error.type in no_retry_types:
            return False
        
        return True
    
    def get_retry_delay(self, error: DetectedError, attempt: int) -> int:
        """取得重試延遲時間"""
        delays = self.retry_delays.get(error.context.stage, [1])
        return delays[min(attempt, len(delays) - 1)]
```

### 降級策略
```python
class DegradationStrategy:
    """降級策略管理器"""
    
    def apply_degradation(self, error: DetectedError) -> DegradationResult:
        """應用降級策略"""
        
        if error.context.stage == "code_generation":
            return self._degrade_code_generation(error)
        elif error.context.stage == "test_execution":
            return self._degrade_test_execution(error)
        elif error.context.stage == "requirement_analysis":
            return self._degrade_requirement_analysis(error)
        else:
            return DegradationResult(
                success=False,
                message="無可用的降級策略"
            )
    
    def _degrade_code_generation(self, error: DetectedError) -> DegradationResult:
        """程式碼生成降級策略"""
        
        # 策略 1: 使用簡化模板
        if error.type in ["COMPLEXITY_ERROR", "ARCHITECTURE_VIOLATION"]:
            simplified_code = self._generate_simplified_template(error.context)
            return DegradationResult(
                success=True,
                action="SIMPLIFIED_TEMPLATE",
                result=simplified_code,
                message="使用簡化模板生成程式碼"
            )
        
        # 策略 2: 生成基礎骨架
        elif error.type in ["LOGIC_ERROR", "DEPENDENCY_ERROR"]:
            skeleton_code = self._generate_code_skeleton(error.context)
            return DegradationResult(
                success=True,
                action="CODE_SKELETON",
                result=skeleton_code,
                message="生成程式碼骨架，需要手動完成實作"
            )
        
        # 策略 3: 提供實作指導
        else:
            implementation_guide = self._generate_implementation_guide(error.context)
            return DegradationResult(
                success=True,
                action="IMPLEMENTATION_GUIDE",
                result=implementation_guide,
                message="提供實作指導文檔"
            )
```

## 🚨 人工介入觸發條件

### 自動觸發條件
```yaml
立即觸發人工介入:
  - CRITICAL 級別錯誤
  - 連續 3 次自動修正失敗
  - 安全性相關錯誤
  - 資料完整性威脅
  - 系統架構重大變更需求

延遲觸發人工介入:
  - HIGH 級別錯誤超過 30 分鐘未解決
  - 測試覆蓋率持續低於 50%
  - 效能回歸超過 24 小時未修復
  - 需求衝突無法自動解決

建議觸發人工介入:
  - MEDIUM 級別錯誤累積超過 10 個
  - 程式碼品質指標持續下降
  - 文檔同步失敗超過 48 小時
  - 使用者回饋負面趨勢
```

### 人工介入通知機制
```python
class HumanInterventionNotifier:
    """人工介入通知器"""
    
    def notify_intervention_required(
        self, 
        error: DetectedError, 
        trigger_reason: str
    ) -> None:
        """通知需要人工介入"""
        
        notification = {
            "timestamp": datetime.now().isoformat(),
            "error_id": error.id,
            "error_type": error.type,
            "severity": error.severity,
            "trigger_reason": trigger_reason,
            "context": {
                "stage": error.context.stage,
                "requirement_id": error.context.requirement_id,
                "affected_files": error.context.affected_files
            },
            "suggested_actions": self._generate_suggested_actions(error),
            "escalation_level": self._determine_escalation_level(error)
        }
        
        # 發送通知
        self._send_notification(notification)
        
        # 記錄到日誌
        self._log_intervention_request(notification)
    
    def _generate_suggested_actions(self, error: DetectedError) -> List[str]:
        """生成建議的人工處理動作"""
        actions = []
        
        if error.type == "REQUIREMENT_CONFLICT":
            actions.extend([
                "檢查需求文檔的一致性",
                "與需求提供者確認優先級",
                "考慮需求分階段實作"
            ])
        
        elif error.type == "ARCHITECTURE_VIOLATION":
            actions.extend([
                "檢查系統架構設計",
                "評估架構變更的必要性",
                "更新架構文檔"
            ])
        
        elif error.type == "SECURITY_ISSUE":
            actions.extend([
                "立即進行安全性評估",
                "檢查相關程式碼的安全性",
                "更新安全性指導原則"
            ])
        
        return actions
```

## 📊 錯誤處理效能指標

### 處理效能統計
```yaml
自動修正成功率:
  - 語法錯誤: 95%
  - 程式碼風格: 98%
  - 簡單邏輯錯誤: 70%
  - 依賴問題: 85%
  - 測試覆蓋率: 80%

平均處理時間:
  - 語法錯誤: 30 秒
  - 程式碼風格: 15 秒
  - 邏輯錯誤: 2 分鐘
  - 測試失敗: 5 分鐘
  - 架構問題: 10 分鐘

人工介入率:
  - 需求衝突: 80%
  - 架構違規: 60%
  - 安全性問題: 100%
  - 複雜邏輯錯誤: 40%
  - 效能問題: 30%
```

### 持續改進機制
```python
class ErrorHandlingImprovement:
    """錯誤處理持續改進機制"""
    
    def analyze_error_patterns(self) -> ErrorPatternAnalysis:
        """分析錯誤模式"""
        
        # 收集錯誤統計資料
        error_stats = self._collect_error_statistics()
        
        # 識別高頻錯誤類型
        frequent_errors = self._identify_frequent_errors(error_stats)
        
        # 分析修正成功率
        fix_success_rates = self._analyze_fix_success_rates(error_stats)
        
        # 識別改進機會
        improvement_opportunities = self._identify_improvements(
            frequent_errors, fix_success_rates
        )
        
        return ErrorPatternAnalysis(
            frequent_errors=frequent_errors,
            success_rates=fix_success_rates,
            improvements=improvement_opportunities
        )
    
    def update_error_handling_rules(self, analysis: ErrorPatternAnalysis) -> None:
        """更新錯誤處理規則"""
        
        for improvement in analysis.improvements:
            if improvement.type == "NEW_PATTERN":
                self._add_error_pattern(improvement.pattern)
            elif improvement.type == "IMPROVED_FIX":
                self._update_fix_strategy(improvement.error_type, improvement.strategy)
            elif improvement.type == "BETTER_DETECTION":
                self._enhance_detection_rule(improvement.detection_rule)
```

---

**最後更新**: 2025-01-23  
**維護責任**: AI Agent 開發團隊、品質保證工程師  
**版本**: v1.0.0 (基於實際錯誤處理模式分析)
