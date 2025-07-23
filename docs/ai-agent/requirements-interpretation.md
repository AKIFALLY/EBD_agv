# AI Agent 需求解讀指導

## 📋 概述

本文檔定義 AI Agent 如何解析和解讀 RosAGV 專案的需求文檔，基於現有的 `docs/requirements/functional-requirements.md` 和 `docs/traceability/requirements-traceability.md`，建立完整的需求分析、分類和影響評估機制。

## 🎯 需求解讀目標

### 核心原則
- **結構化解析**: 系統性解析需求文檔的結構和內容
- **智能分類**: 自動識別需求類型、優先級和影響範圍
- **變更檢測**: 及時發現需求變更並評估影響
- **衝突識別**: 自動檢測需求間的衝突和不一致

### 解讀範圍
- **功能需求**: 系統功能和業務邏輯需求
- **非功能需求**: 效能、安全性、可靠性需求
- **介面需求**: 使用者介面和系統介面需求
- **整合需求**: 外部系統整合需求

## 📖 需求文檔結構解析

### RosAGV 需求文檔結構分析
```python
# AI Agent 需求文檔結構解析器
class RequirementDocumentParser:
    """需求文檔結構解析器"""
    
    def __init__(self):
        self.document_structure = {
            "functional-requirements.md": {
                "sections": [
                    "核心系統需求",
                    "Web API 服務需求", 
                    "ROS 2 工作空間需求",
                    "車隊管理需求",
                    "整合需求",
                    "品質需求"
                ],
                "requirement_pattern": r"### (FR-[A-Z]+-\d+): (.+)",
                "fields": ["需求描述", "實作狀態", "驗收標準", "相關工作空間"]
            },
            "user-interface-requirements.md": {
                "sections": [
                    "AGVCUI 車隊管理介面需求",
                    "OPUI 機台操作介面需求"
                ],
                "requirement_pattern": r"### (UI-[A-Z]+-\d+): (.+)",
                "fields": ["介面描述", "功能需求", "使用者體驗", "技術實作"]
            }
        }
    
    def parse_document(self, document_path: str) -> DocumentStructure:
        """解析需求文檔結構"""
        
        with open(document_path, 'r', encoding='utf-8') as file:
            content = file.read()
        
        # 識別文檔類型
        doc_type = self._identify_document_type(document_path)
        structure_config = self.document_structure[doc_type]
        
        # 解析章節結構
        sections = self._parse_sections(content, structure_config)
        
        # 提取需求項目
        requirements = self._extract_requirements(content, structure_config)
        
        # 分析需求關聯
        relationships = self._analyze_relationships(requirements)
        
        return DocumentStructure(
            document_type=doc_type,
            sections=sections,
            requirements=requirements,
            relationships=relationships
        )
    
    def _extract_requirements(self, content: str, config: dict) -> List[Requirement]:
        """提取需求項目"""
        requirements = []
        
        # 使用正則表達式匹配需求
        import re
        pattern = config["requirement_pattern"]
        matches = re.finditer(pattern, content, re.MULTILINE)
        
        for match in matches:
            req_id = match.group(1)
            req_title = match.group(2)
            
            # 提取需求詳細內容
            req_content = self._extract_requirement_content(content, match.end())
            
            # 解析需求欄位
            req_fields = self._parse_requirement_fields(req_content, config["fields"])
            
            requirement = Requirement(
                id=req_id,
                title=req_title,
                content=req_content,
                fields=req_fields,
                source_document=config
            )
            
            requirements.append(requirement)
        
        return requirements
```

### 需求內容解析模式
```python
class RequirementContentAnalyzer:
    """需求內容分析器"""
    
    def analyze_requirement_content(self, requirement: Requirement) -> RequirementAnalysis:
        """分析需求內容"""
        
        analysis = RequirementAnalysis(requirement_id=requirement.id)
        
        # 1. 需求類型識別
        analysis.type = self._identify_requirement_type(requirement)
        
        # 2. 優先級判斷
        analysis.priority = self._determine_priority(requirement)
        
        # 3. 複雜度評估
        analysis.complexity = self._assess_complexity(requirement)
        
        # 4. 依賴關係分析
        analysis.dependencies = self._analyze_dependencies(requirement)
        
        # 5. 影響範圍評估
        analysis.impact_scope = self._assess_impact_scope(requirement)
        
        return analysis
    
    def _identify_requirement_type(self, requirement: Requirement) -> str:
        """識別需求類型"""
        
        # 基於需求 ID 前綴識別
        if requirement.id.startswith("FR-CORE"):
            return "CORE_SYSTEM"
        elif requirement.id.startswith("FR-WEB"):
            return "WEB_API"
        elif requirement.id.startswith("FR-ROS"):
            return "ROS2_WORKSPACE"
        elif requirement.id.startswith("FR-FLEET"):
            return "FLEET_MANAGEMENT"
        elif requirement.id.startswith("UI-AGVC"):
            return "AGVCUI_INTERFACE"
        elif requirement.id.startswith("UI-OPUI"):
            return "OPUI_INTERFACE"
        
        # 基於內容關鍵字識別
        content_lower = requirement.content.lower()
        
        if any(keyword in content_lower for keyword in ["api", "endpoint", "http", "rest"]):
            return "API_REQUIREMENT"
        elif any(keyword in content_lower for keyword in ["database", "crud", "model", "table"]):
            return "DATABASE_REQUIREMENT"
        elif any(keyword in content_lower for keyword in ["ros2", "node", "topic", "service"]):
            return "ROS2_REQUIREMENT"
        elif any(keyword in content_lower for keyword in ["ui", "interface", "frontend", "page"]):
            return "UI_REQUIREMENT"
        elif any(keyword in content_lower for keyword in ["performance", "security", "reliability"]):
            return "NON_FUNCTIONAL_REQUIREMENT"
        
        return "GENERAL_REQUIREMENT"
    
    def _determine_priority(self, requirement: Requirement) -> str:
        """判斷需求優先級"""
        
        # 基於實作狀態判斷
        if "✅ 完成" in requirement.content:
            return "IMPLEMENTED"
        elif "🚧 進行中" in requirement.content:
            return "IN_PROGRESS"
        elif "⚠️ 待實作" in requirement.content:
            return "PENDING"
        
        # 基於關鍵字判斷優先級
        content_lower = requirement.content.lower()
        
        if any(keyword in content_lower for keyword in ["critical", "essential", "must", "核心"]):
            return "HIGH"
        elif any(keyword in content_lower for keyword in ["important", "should", "重要"]):
            return "MEDIUM"
        elif any(keyword in content_lower for keyword in ["nice to have", "could", "可選"]):
            return "LOW"
        
        # 基於需求類型預設優先級
        if requirement.id.startswith("FR-CORE"):
            return "HIGH"
        elif requirement.id.startswith("FR-WEB"):
            return "MEDIUM"
        else:
            return "MEDIUM"
    
    def _assess_impact_scope(self, requirement: Requirement) -> ImpactScope:
        """評估影響範圍"""
        
        impact = ImpactScope()
        
        # 基於追溯矩陣分析影響範圍
        traceability = self._get_traceability_info(requirement.id)
        
        if traceability:
            impact.affected_specifications = traceability.specifications
            impact.affected_code_modules = traceability.code_modules
            impact.affected_test_cases = traceability.test_cases
        
        # 基於內容分析影響範圍
        content_lower = requirement.content.lower()
        
        # 工作空間影響分析
        if "agv_ws" in content_lower:
            impact.affected_workspaces.append("agv_ws")
        if "web_api_ws" in content_lower:
            impact.affected_workspaces.append("web_api_ws")
        if "db_proxy_ws" in content_lower:
            impact.affected_workspaces.append("db_proxy_ws")
        
        # 服務影響分析
        if any(keyword in content_lower for keyword in ["api", "endpoint"]):
            impact.affected_services.extend(["web_api", "agvcui", "opui"])
        if any(keyword in content_lower for keyword in ["database", "model"]):
            impact.affected_services.append("db_proxy")
        if any(keyword in content_lower for keyword in ["plc", "hardware"]):
            impact.affected_services.extend(["plc_proxy", "keyence_plc"])
        
        return impact
```

## 🔍 需求變更檢測機制

### 自動變更檢測
```python
class RequirementChangeDetector:
    """需求變更檢測器"""
    
    def __init__(self):
        self.baseline_requirements = {}
        self.change_patterns = {
            "NEW_REQUIREMENT": r"### (FR-[A-Z]+-\d+): (.+)",
            "MODIFIED_CONTENT": r"(?<=### FR-[A-Z]+-\d+:).*?(?=###|$)",
            "STATUS_CHANGE": r"實作狀態[：:]\s*(✅|🚧|⚠️)",
            "ACCEPTANCE_CRITERIA_CHANGE": r"驗收標準[：:].*?(?=\n\n|\n###|$)"
        }
    
    def detect_changes(self, current_document: str, baseline_document: str) -> List[RequirementChange]:
        """檢測需求變更"""
        
        changes = []
        
        # 解析當前和基準文檔
        current_reqs = self._parse_requirements(current_document)
        baseline_reqs = self._parse_requirements(baseline_document)
        
        # 檢測新增需求
        new_requirements = self._detect_new_requirements(current_reqs, baseline_reqs)
        changes.extend(new_requirements)
        
        # 檢測修改需求
        modified_requirements = self._detect_modified_requirements(current_reqs, baseline_reqs)
        changes.extend(modified_requirements)
        
        # 檢測刪除需求
        deleted_requirements = self._detect_deleted_requirements(current_reqs, baseline_reqs)
        changes.extend(deleted_requirements)
        
        return changes
    
    def _detect_modified_requirements(
        self, 
        current_reqs: Dict[str, Requirement], 
        baseline_reqs: Dict[str, Requirement]
    ) -> List[RequirementChange]:
        """檢測修改的需求"""
        
        changes = []
        
        for req_id, current_req in current_reqs.items():
            if req_id in baseline_reqs:
                baseline_req = baseline_reqs[req_id]
                
                # 檢測內容變更
                if current_req.content != baseline_req.content:
                    change_details = self._analyze_content_changes(current_req, baseline_req)
                    
                    change = RequirementChange(
                        type="MODIFIED",
                        requirement_id=req_id,
                        old_value=baseline_req.content,
                        new_value=current_req.content,
                        change_details=change_details,
                        impact_assessment=self._assess_change_impact(change_details)
                    )
                    
                    changes.append(change)
        
        return changes
    
    def _analyze_content_changes(
        self, 
        current_req: Requirement, 
        baseline_req: Requirement
    ) -> ChangeDetails:
        """分析內容變更詳情"""
        
        details = ChangeDetails()
        
        # 檢測狀態變更
        current_status = self._extract_status(current_req.content)
        baseline_status = self._extract_status(baseline_req.content)
        
        if current_status != baseline_status:
            details.status_change = {
                "from": baseline_status,
                "to": current_status
            }
        
        # 檢測驗收標準變更
        current_criteria = self._extract_acceptance_criteria(current_req.content)
        baseline_criteria = self._extract_acceptance_criteria(baseline_req.content)
        
        if current_criteria != baseline_criteria:
            details.acceptance_criteria_change = {
                "from": baseline_criteria,
                "to": current_criteria
            }
        
        # 檢測描述變更
        current_desc = self._extract_description(current_req.content)
        baseline_desc = self._extract_description(baseline_req.content)
        
        if current_desc != baseline_desc:
            details.description_change = {
                "from": baseline_desc,
                "to": current_desc
            }
        
        return details
```

## ⚔️ 需求衝突識別機制

### 衝突檢測算法
```python
class RequirementConflictDetector:
    """需求衝突檢測器"""
    
    def __init__(self):
        self.conflict_rules = [
            self._check_resource_conflicts,
            self._check_functional_conflicts,
            self._check_constraint_conflicts,
            self._check_priority_conflicts
        ]
    
    def detect_conflicts(self, requirements: List[Requirement]) -> List[RequirementConflict]:
        """檢測需求衝突"""
        
        conflicts = []
        
        # 兩兩比較需求
        for i, req1 in enumerate(requirements):
            for j, req2 in enumerate(requirements[i+1:], i+1):
                
                # 應用所有衝突檢測規則
                for rule in self.conflict_rules:
                    conflict = rule(req1, req2)
                    if conflict:
                        conflicts.append(conflict)
        
        return conflicts
    
    def _check_resource_conflicts(self, req1: Requirement, req2: Requirement) -> Optional[RequirementConflict]:
        """檢查資源衝突"""
        
        # 檢查端口衝突
        ports1 = self._extract_ports(req1.content)
        ports2 = self._extract_ports(req2.content)
        
        common_ports = set(ports1) & set(ports2)
        if common_ports:
            return RequirementConflict(
                type="RESOURCE_CONFLICT",
                requirement1=req1.id,
                requirement2=req2.id,
                conflict_description=f"端口衝突: {common_ports}",
                severity="HIGH",
                resolution_suggestions=[
                    "使用不同的端口號",
                    "實作端口共享機制",
                    "重新設計服務架構"
                ]
            )
        
        # 檢查資料庫表衝突
        tables1 = self._extract_database_tables(req1.content)
        tables2 = self._extract_database_tables(req2.content)
        
        conflicting_tables = self._check_table_conflicts(tables1, tables2)
        if conflicting_tables:
            return RequirementConflict(
                type="DATABASE_CONFLICT",
                requirement1=req1.id,
                requirement2=req2.id,
                conflict_description=f"資料庫表衝突: {conflicting_tables}",
                severity="MEDIUM",
                resolution_suggestions=[
                    "統一資料模型設計",
                    "使用不同的表名",
                    "建立資料庫視圖"
                ]
            )
        
        return None
    
    def _check_functional_conflicts(self, req1: Requirement, req2: Requirement) -> Optional[RequirementConflict]:
        """檢查功能衝突"""
        
        # 檢查 API 端點衝突
        endpoints1 = self._extract_api_endpoints(req1.content)
        endpoints2 = self._extract_api_endpoints(req2.content)
        
        common_endpoints = set(endpoints1) & set(endpoints2)
        if common_endpoints:
            # 檢查是否為相同功能的不同實作
            if self._are_conflicting_implementations(req1, req2, common_endpoints):
                return RequirementConflict(
                    type="FUNCTIONAL_CONFLICT",
                    requirement1=req1.id,
                    requirement2=req2.id,
                    conflict_description=f"API 端點功能衝突: {common_endpoints}",
                    severity="HIGH",
                    resolution_suggestions=[
                        "統一 API 設計",
                        "使用不同的端點路徑",
                        "合併相似功能"
                    ]
                )
        
        # 檢查業務邏輯衝突
        business_logic1 = self._extract_business_logic(req1.content)
        business_logic2 = self._extract_business_logic(req2.content)
        
        if self._are_conflicting_business_logic(business_logic1, business_logic2):
            return RequirementConflict(
                type="BUSINESS_LOGIC_CONFLICT",
                requirement1=req1.id,
                requirement2=req2.id,
                conflict_description="業務邏輯衝突",
                severity="HIGH",
                resolution_suggestions=[
                    "澄清業務需求",
                    "建立業務規則優先級",
                    "設計條件式邏輯"
                ]
            )
        
        return None
```

## 📊 需求分析報告生成

### 自動報告生成
```python
class RequirementAnalysisReporter:
    """需求分析報告生成器"""
    
    def generate_analysis_report(
        self, 
        requirements: List[Requirement],
        changes: List[RequirementChange],
        conflicts: List[RequirementConflict]
    ) -> AnalysisReport:
        """生成需求分析報告"""
        
        report = AnalysisReport()
        
        # 需求統計分析
        report.statistics = self._generate_statistics(requirements)
        
        # 變更影響分析
        report.change_impact = self._analyze_change_impact(changes)
        
        # 衝突分析
        report.conflict_analysis = self._analyze_conflicts(conflicts)
        
        # 實作建議
        report.implementation_recommendations = self._generate_recommendations(
            requirements, changes, conflicts
        )
        
        # 風險評估
        report.risk_assessment = self._assess_risks(requirements, changes, conflicts)
        
        return report
    
    def _generate_statistics(self, requirements: List[Requirement]) -> RequirementStatistics:
        """生成需求統計"""
        
        stats = RequirementStatistics()
        
        # 按類型統計
        type_counts = {}
        for req in requirements:
            req_type = self._get_requirement_type(req)
            type_counts[req_type] = type_counts.get(req_type, 0) + 1
        
        stats.by_type = type_counts
        
        # 按優先級統計
        priority_counts = {}
        for req in requirements:
            priority = self._get_requirement_priority(req)
            priority_counts[priority] = priority_counts.get(priority, 0) + 1
        
        stats.by_priority = priority_counts
        
        # 按狀態統計
        status_counts = {}
        for req in requirements:
            status = self._get_requirement_status(req)
            status_counts[status] = status_counts.get(status, 0) + 1
        
        stats.by_status = status_counts
        
        # 複雜度分析
        complexity_distribution = self._analyze_complexity_distribution(requirements)
        stats.complexity_distribution = complexity_distribution
        
        return stats
    
    def _analyze_change_impact(self, changes: List[RequirementChange]) -> ChangeImpactAnalysis:
        """分析變更影響"""
        
        impact = ChangeImpactAnalysis()
        
        # 按變更類型分組
        changes_by_type = {}
        for change in changes:
            change_type = change.type
            if change_type not in changes_by_type:
                changes_by_type[change_type] = []
            changes_by_type[change_type].append(change)
        
        impact.changes_by_type = changes_by_type
        
        # 影響範圍分析
        affected_modules = set()
        affected_services = set()
        
        for change in changes:
            if change.impact_assessment:
                affected_modules.update(change.impact_assessment.affected_modules)
                affected_services.update(change.impact_assessment.affected_services)
        
        impact.affected_modules = list(affected_modules)
        impact.affected_services = list(affected_services)
        
        # 風險評估
        high_risk_changes = [c for c in changes if c.impact_assessment and c.impact_assessment.risk_level == "HIGH"]
        impact.high_risk_changes = high_risk_changes
        
        return impact
```

## 🎯 需求解讀品質指標

### 解讀準確性指標
```yaml
需求識別準確率:
  - 需求類型識別: ≥ 95%
  - 優先級判斷: ≥ 90%
  - 影響範圍評估: ≥ 85%
  - 依賴關係識別: ≥ 80%

變更檢測效能:
  - 變更檢測延遲: ≤ 5 分鐘
  - 誤報率: ≤ 5%
  - 漏檢率: ≤ 2%
  - 變更分類準確率: ≥ 90%

衝突檢測效能:
  - 衝突檢測覆蓋率: ≥ 95%
  - 誤報率: ≤ 10%
  - 解決建議有效性: ≥ 80%
  - 檢測時間: ≤ 30 秒
```

### 持續改進機制
```python
class RequirementInterpretationImprovement:
    """需求解讀持續改進機制"""
    
    def analyze_interpretation_accuracy(self) -> AccuracyAnalysis:
        """分析解讀準確性"""
        
        # 收集解讀結果和人工驗證資料
        interpretation_results = self._collect_interpretation_results()
        human_validations = self._collect_human_validations()
        
        # 計算準確性指標
        accuracy_metrics = self._calculate_accuracy_metrics(
            interpretation_results, human_validations
        )
        
        # 識別改進機會
        improvement_areas = self._identify_improvement_areas(accuracy_metrics)
        
        return AccuracyAnalysis(
            metrics=accuracy_metrics,
            improvement_areas=improvement_areas
        )
    
    def update_interpretation_rules(self, accuracy_analysis: AccuracyAnalysis) -> None:
        """更新解讀規則"""
        
        for area in accuracy_analysis.improvement_areas:
            if area.type == "TYPE_CLASSIFICATION":
                self._improve_type_classification_rules(area)
            elif area.type == "PRIORITY_DETERMINATION":
                self._improve_priority_rules(area)
            elif area.type == "CONFLICT_DETECTION":
                self._improve_conflict_detection_rules(area)
```

## 📋 相關文檔

- [AI Agent 開發工作流程](./development-workflow.md)
- [程式碼生成指導](./code-generation-guidelines.md)
- [錯誤處理協定](./error-handling-protocols.md)
- [功能需求規格](../requirements/functional-requirements.md)
- [需求追溯矩陣](../traceability/requirements-traceability.md)

---

**最後更新**: 2025-01-23  
**維護責任**: AI Agent 開發團隊、需求分析師  
**版本**: v1.0.0 (基於實際需求文檔結構分析)
