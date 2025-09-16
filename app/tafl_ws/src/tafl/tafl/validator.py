"""
TAFL Validator Implementation
驗證 TAFL 語法和語義

This module implements validation for TAFL programs, checking syntax
correctness and semantic consistency.
"""

from typing import Any, Dict, List, Optional, Set
from .ast_nodes import *


class TAFLValidationError(Exception):
    """TAFL validation error / TAFL 驗證錯誤"""
    def __init__(self, message: str, node: Optional[ASTNode] = None):
        self.node = node
        if node and hasattr(node, 'line_number'):
            super().__init__(f"Validation error at line {node.line_number}: {message}")
        else:
            super().__init__(f"Validation error: {message}")


class TAFLValidator:
    """
    TAFL Program Validator
    TAFL 程序驗證器
    
    Performs syntax and semantic validation on TAFL AST.
    """
    
    # Core verb requirements (TAFL v1.1.2 - Strict specification compliance)
    VERB_REQUIREMENTS = {
        # query: 根據規格書，使用 where 而非 filter，使用 as 而非 store_as
        'query': {'required': ['target'], 'optional': ['where', 'as', 'order', 'limit', 'description']},
        # check: 使用 as 而非 store_as
        'check': {'required': ['condition'], 'optional': ['where', 'as', 'description']},
        # create: 使用 with 而非 params，使用 as 而非 store_as
        'create': {'required': ['target'], 'optional': ['with', 'as', 'description']},
        # update: where 和 set 是必需的
        'update': {'required': ['target', 'where', 'set'], 'optional': ['description']},
        # if: 標準條件結構
        'if': {'required': ['condition'], 'optional': ['then', 'else', 'description']},
        # for: 使用 in/as/do 而非 each/items，v1.1 增加 filter
        'for': {'required': ['in', 'as', 'do'], 'optional': ['filter', 'description']},
        # switch: v1.1.2 使用 expression 和 cases，default 在 cases 內
        'switch': {'required': ['expression', 'cases'], 'optional': ['description']},
        # set: v1.1.2 只接受物件格式，不再支援字串格式
        'set': {'required': [], 'optional': ['description']},  # Validated separately
        # stop: 停止流程
        'stop': {'required': [], 'optional': ['reason', 'message', 'description']},
        # notify: 使用 level 而非 channel
        'notify': {'required': ['message'], 'optional': ['level', 'recipients', 'details', 'description']}
    }
    
    def __init__(self):
        self.errors = []
        self.warnings = []
        # TAFL v1.1: Enhanced variable scoping
        self.rules_scope = set()      # Variables from rules section
        self.preload_scope = set()    # Variables from preload section 
        self.defined_variables = set()  # Variables from variables section
        self.used_variables = set()
        self.defined_rules = set()    # Rules defined in rules section
    
    def validate(self, program: TAFLProgram) -> bool:
        """
        Validate a TAFL program
        驗證 TAFL 程序
        
        Returns:
            True if valid, False otherwise
        """
        self.errors = []
        self.warnings = []
        self.defined_variables = set(program.variables.keys())
        self.used_variables = set()
        
        try:
            # Validate metadata
            self._validate_metadata(program.metadata)
            
            # Validate settings
            self._validate_settings(program.settings)
            
            # TAFL v1.1: Validate preload statements
            if program.preload:
                for preload_stmt in program.preload:
                    self._validate_preload(preload_stmt)
            
            # TAFL v1.1: Validate rules definitions
            if program.rules:
                for rule_name, rule_def in program.rules.items():
                    self._validate_rule(rule_name, rule_def)
            
            # Validate variables have initial values
            for var_name, var_value in program.variables.items():
                if var_value is None:
                    self.warnings.append(f"Variable '{var_name}' initialized with None")
            
            # Validate flow statements
            if not program.flow:
                self.warnings.append("Flow is empty")
            else:
                for stmt in program.flow:
                    self._validate_statement(stmt)
            
            # TAFL v1.1: Enhanced variable checking with multi-scope support
            all_defined = self.defined_variables | self.preload_scope | self.rules_scope
            unused = all_defined - self.used_variables
            if unused:
                self.warnings.append(f"Unused variables: {', '.join(unused)}")
            
            # Check for undefined variables (excluding loop variables and rules references)
            undefined = self.used_variables - all_defined
            # Filter out common loop variables and special variables
            undefined = {v for v in undefined if not v in ['item', 'i', 'index', 'rules']}
            if undefined:
                self.warnings.append(f"Potentially undefined variables: {', '.join(undefined)}")
                
            # Check for unused rules
            if self.defined_rules:
                unused_rules = self.defined_rules - {r.replace('rules.', '') for r in self.used_variables if r.startswith('rules.')}
                if unused_rules:
                    self.warnings.append(f"Unused rules: {', '.join(unused_rules)}")
            
        except TAFLValidationError as e:
            self.errors.append(str(e))
        except Exception as e:
            self.errors.append(f"Validation error: {e}")
        
        return len(self.errors) == 0
    
    def get_errors(self) -> List[str]:
        """Get validation errors / 獲取驗證錯誤"""
        return self.errors
    
    def get_warnings(self) -> List[str]:
        """Get validation warnings / 獲取驗證警告"""
        return self.warnings
    
    def _validate_metadata(self, metadata: FlowMetadata):
        """Validate flow metadata / 驗證流程元數據"""
        if not metadata.id:
            raise TAFLValidationError("Flow ID is required")
        
        if not metadata.name:
            raise TAFLValidationError("Flow name is required")
        
        # Validate ID format (alphanumeric and underscore)
        if not metadata.id.replace('_', '').replace('-', '').isalnum():
            raise TAFLValidationError(f"Invalid flow ID format: {metadata.id}")
        
        # v1.1.2: Validate enabled field (optional, but must be boolean if present)
        if hasattr(metadata, 'enabled') and not isinstance(metadata.enabled, bool):
            raise TAFLValidationError(f"Flow enabled must be boolean, got: {type(metadata.enabled).__name__}")
    
    def _validate_settings(self, settings: FlowSettings):
        """Validate flow settings / 驗證流程設置"""
        if settings.timeout <= 0:
            raise TAFLValidationError(f"Invalid timeout: {settings.timeout}")
        
        if settings.max_retries < 0:
            raise TAFLValidationError(f"Invalid max_retries: {settings.max_retries}")
        
        valid_log_levels = ['DEBUG', 'INFO', 'WARNING', 'ERROR', 'CRITICAL', 'debug', 'info', 'warning', 'error', 'critical']
        if settings.log_level not in valid_log_levels:
            raise TAFLValidationError(f"Invalid log_level: {settings.log_level}")
    
    def _validate_preload(self, stmt: PreloadStatement):
        """Validate preload statement / 驗證預載入語句"""
        if not stmt.target:
            raise TAFLValidationError("Preload query target is required", stmt)
        
        # Validate known preload targets
        valid_targets = ['racks', 'locations', 'tasks', 'works', 'agvs', 'alarms']
        if stmt.target not in valid_targets:
            self.warnings.append(f"Unknown preload target: {stmt.target}")
        
        # Validate filters
        if stmt.filters:
            for key, expr in stmt.filters.items():
                self._validate_expression(expr)
        
        # Validate limit
        if stmt.limit is not None:
            if not isinstance(stmt.limit, int) or stmt.limit <= 0:
                raise TAFLValidationError(f"Invalid preload limit: {stmt.limit}", stmt)
        
        # Register preloaded variable
        if stmt.as_:
            self.preload_scope.add(stmt.as_)
        else:
            raise TAFLValidationError("Preload statement must specify 'as'", stmt)
    
    def _validate_rule(self, rule_name: str, rule_def: RuleDefinition):
        """Validate rule definition / 驗證規則定義"""
        if not rule_name:
            raise TAFLValidationError("Rule name cannot be empty")
        
        # Validate rule name format
        if not rule_name.replace('_', '').isalnum():
            self.warnings.append(f"Rule name '{rule_name}' should be alphanumeric")
        
        self.defined_rules.add(rule_name)
        
        # If rule has a condition, validate it
        if rule_def.condition is not None:
            self._validate_expression(rule_def.condition)
        
        # If rule has a value, it's available as a variable
        if rule_def.value is not None:
            self.rules_scope.add(rule_name)
    
    def _validate_statement(self, stmt: Statement):
        """Validate a statement / 驗證語句"""
        if isinstance(stmt, QueryStatement):
            self._validate_query(stmt)
        elif isinstance(stmt, CheckStatement):
            self._validate_check(stmt)
        elif isinstance(stmt, CreateStatement):
            self._validate_create(stmt)
        elif isinstance(stmt, UpdateStatement):
            self._validate_update(stmt)
        elif isinstance(stmt, IfStatement):
            self._validate_if(stmt)
        elif isinstance(stmt, ForStatement):
            self._validate_for(stmt)
        elif isinstance(stmt, SwitchStatement):
            self._validate_switch(stmt)
        elif isinstance(stmt, SetStatement):
            self._validate_set(stmt)
        elif isinstance(stmt, StopStatement):
            self._validate_stop(stmt)
        elif isinstance(stmt, NotifyStatement):
            self._validate_notify(stmt)
        else:
            raise TAFLValidationError(f"Unknown statement type: {type(stmt)}", stmt)
    
    def _validate_query(self, stmt: QueryStatement):
        """Validate query statement / 驗證查詢語句"""
        if not stmt.target:
            raise TAFLValidationError("Query target is required", stmt)
        
        # Validate known query targets
        valid_targets = ['racks', 'locations', 'tasks', 'works', 'agvs', 'alarms']
        if stmt.target not in valid_targets:
            self.warnings.append(f"Unknown query target: {stmt.target}")
        
        # Validate filters
        if stmt.filters:
            for key, expr in stmt.filters.items():
                self._validate_expression(expr)
        
        # Register variable if storing result
        if stmt.as_:
            self.defined_variables.add(stmt.as_)
    
    def _validate_check(self, stmt: CheckStatement):
        """Validate check statement / 驗證檢查語句
        
        v1.1.3: 'as' parameter is now required for all check statements
        """
        self._validate_expression(stmt.condition)
        
        # v1.1.3: 'as' is now required
        if not stmt.as_:
            raise TAFLValidationError(
                f"Check statement must include 'as' parameter to store result. "
                f"Example: check: {{condition: '...', as: 'result_variable'}}",
                stmt
            )
        
        self.defined_variables.add(stmt.as_)
    
    def _validate_create(self, stmt: CreateStatement):
        """Validate create statement / 驗證創建語句"""
        if not stmt.target:
            raise TAFLValidationError("Create target is required", stmt)
        
        valid_targets = ['task', 'alarm', 'notification', 'work']
        if stmt.target not in valid_targets:
            self.warnings.append(f"Unknown create target: {stmt.target}")
        
        # Validate with parameters (符合規格書: with)
        for key, expr in stmt.with_.items():
            self._validate_expression(expr)
        
        # Check required parameters for tasks
        if stmt.target == 'task':
            required = ['type', 'work_id']
            for req in required:
                if req not in stmt.with_:
                    self.warnings.append(f"Task creation missing recommended parameter: {req}")
        
        if stmt.as_:
            self.defined_variables.add(stmt.as_)
    
    def _validate_update(self, stmt: UpdateStatement):
        """Validate update statement / 驗證更新語句"""
        if not stmt.target:
            raise TAFLValidationError("Update target is required", stmt)
        
        # Validate where conditions (符合規格書: where)
        if not stmt.where:
            raise TAFLValidationError("Update requires where conditions", stmt)
        
        for key, expr in stmt.where.items():
            self._validate_expression(expr)
        
        # Validate set changes (符合規格書: set)
        if not stmt.set:
            self.warnings.append("Update statement has no set changes specified")
        
        for key, expr in stmt.set.items():
            self._validate_expression(expr)
    
    def _validate_if(self, stmt: IfStatement):
        """Validate if statement / 驗證條件語句"""
        self._validate_expression(stmt.condition)
        
        if not stmt.then_branch:
            self.warnings.append("If statement has empty then branch")
        
        for s in stmt.then_branch:
            self._validate_statement(s)
        
        if stmt.else_branch:
            for s in stmt.else_branch:
                self._validate_statement(s)
    
    def _validate_for(self, stmt: ForStatement):
        """Validate for loop / 驗證循環語句"""
        if not stmt.as_:
            raise TAFLValidationError("For loop 'as' variable is required", stmt)
        
        # Validate 'in' expression (符合規格書: in)
        self._validate_expression(stmt.in_)
        
        # TAFL v1.1: Validate filter if present
        if stmt.filter is not None:
            self._validate_expression(stmt.filter)
        
        # Validate 'do' body (符合規格書: do)
        if not stmt.do:
            self.warnings.append("For loop has empty do body")
        
        # Add loop variable to scope temporarily
        was_defined = stmt.as_ in self.defined_variables
        self.defined_variables.add(stmt.as_)
        
        for s in stmt.do:
            self._validate_statement(s)
        
        # Remove loop variable from scope if it wasn't defined before
        if not was_defined:
            self.defined_variables.discard(stmt.as_)
    
    def _validate_switch(self, stmt: SwitchStatement):
        """Validate switch statement / 驗證分支語句"""
        self._validate_expression(stmt.expression)
        
        # Allow empty switch (may be a placeholder or simplified format)
        if not stmt.cases:
            self.warnings.append("Switch statement has no cases - may be a placeholder")
            return
        
        # Check for duplicate case values
        case_values = []
        for case in stmt.cases:
            # Validate 'when' expression (符合規格書: when)
            self._validate_expression(case.when)
            
            # Check for duplicates (simplified - doesn't evaluate expressions)
            if isinstance(case.when, Literal):
                if case.when.value in case_values:
                    self.warnings.append(f"Duplicate case value: {case.when.value}")
                case_values.append(case.when.value)
            
            # Validate 'do' body (符合規格書: do)
            for s in case.do:
                self._validate_statement(s)
        
        # TAFL v1.1.2: default is handled as a regular case with when:"default"
        # No separate default attribute to validate
    
    def _validate_set(self, stmt: SetStatement):
        """Validate set statement / 驗證設置語句
        
        TAFL v1.1.2: Accepts only object format, no string format
        """
        if not stmt.variable:
            raise TAFLValidationError("Set statement requires a variable name", stmt)
        
        self._validate_expression(stmt.value)
        self.defined_variables.add(stmt.variable)
    
    def _validate_stop(self, stmt: StopStatement):
        """Validate stop statement / 驗證停止語句"""
        if stmt.condition:
            self._validate_expression(stmt.condition)
    
    def _validate_notify(self, stmt: NotifyStatement):
        """Validate notify statement / 驗證通知語句"""
        # Level is the primary field per spec (符合規格書: level 為主要欄位)
        if not stmt.level:
            raise TAFLValidationError("Notify level is required", stmt)
        
        valid_levels = ['info', 'warning', 'error', 'critical', 'alarm', 'INFO', 'WARNING', 'ERROR', 'CRITICAL', 'ALARM']
        if stmt.level not in valid_levels:
            self.warnings.append(f"Unknown notify level: {stmt.level}")
        
        self._validate_expression(stmt.message)
        
        # Validate optional recipients
        if stmt.recipients:
            if not isinstance(stmt.recipients, list):
                self.warnings.append("Notify recipients should be a list")
        
        # Validate optional details (符合規格書: details)
        if stmt.details:
            for key, expr in stmt.details.items():
                self._validate_expression(expr)
    
    def _validate_expression(self, expr: Expression):
        """Validate an expression / 驗證表達式"""
        if isinstance(expr, Variable):
            # TAFL v1.1: Handle rules references specially
            if expr.name == 'rules' and expr.path:
                rule_name = expr.path[0]
                if rule_name not in self.defined_rules:
                    self.warnings.append(f"Undefined rule: {rule_name}")
                self.used_variables.add(f"rules.{rule_name}")
            else:
                self.used_variables.add(expr.name)
                
                # Check if variable might be defined in any scope
                all_defined = self.defined_variables | self.preload_scope | self.rules_scope
                if expr.name not in all_defined:
                    # Could be a loop variable or external variable
                    if not expr.name.startswith('_'):  # Convention: _ prefix for external
                        # Don't warn about common loop variables and special variables
                        if expr.name not in ['item', 'i', 'index', 'element', 'rules']:
                            self.warnings.append(f"Possible undefined variable: {expr.name}")
        
        elif isinstance(expr, BinaryOp):
            self._validate_expression(expr.left)
            self._validate_expression(expr.right)
            
            # Validate operator
            valid_operators = ['+', '-', '*', '/', '==', '!=', '<', '>', '<=', '>=', 'and', 'or']
            if expr.operator not in valid_operators:
                self.errors.append(f"Invalid operator: {expr.operator}")
        
        elif isinstance(expr, UnaryOp):
            self._validate_expression(expr.operand)
            
            # Validate unary operator
            if expr.operator not in ['not', '-']:
                self.errors.append(f"Invalid unary operator: {expr.operator}")
        
        elif isinstance(expr, FunctionCall):
            # Validate function exists (would need function registry)
            known_functions = [
                'min', 'max', 'sum', 'len', 'abs', 'round',
                'int', 'float', 'str', 'bool',
                'upper', 'lower', 'strip',
                'now', 'today'
            ]
            if expr.name not in known_functions:
                self.warnings.append(f"Unknown function: {expr.name}")
            
            for arg in expr.arguments:
                self._validate_expression(arg)
        
        elif isinstance(expr, ArrayExpression):
            if not expr.elements:
                self.warnings.append("Empty array expression")
            for elem in expr.elements:
                self._validate_expression(elem)
        
        elif isinstance(expr, DictExpression):
            if not expr.pairs:
                self.warnings.append("Empty dictionary expression")
            for key, value_expr in expr.pairs:
                if not key:
                    self.errors.append("Dictionary key cannot be empty")
                self._validate_expression(value_expr)
        
        elif isinstance(expr, Literal):
            # Validate literal values
            if expr.type not in ['string', 'number', 'boolean', 'null']:
                self.errors.append(f"Invalid literal type: {expr.type}")
        
        # Handle other expression types that may be added in v1.1
        elif hasattr(expr, '__class__'):
            # For future extensibility
            pass
        else:
            self.warnings.append(f"Unknown expression type: {type(expr)}")
    
    def _validate_rules_reference(self, rule_name: str) -> bool:
        """Validate a rules reference / 驗證規則引用"""
        if rule_name not in self.defined_rules:
            self.warnings.append(f"Undefined rule referenced: {rule_name}")
            return False
        return True