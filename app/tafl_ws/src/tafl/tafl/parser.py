"""
TAFL Parser Implementation
將 TAFL YAML 解析為抽象語法樹 (AST)

This module implements the parser that converts TAFL YAML files into
Abstract Syntax Tree (AST) representations for execution.
"""

import yaml
import re
from typing import Any, Dict, List, Optional, Union
from pathlib import Path
from .ast_nodes import *


class TAFLParseError(Exception):
    """TAFL parsing error / TAFL 解析錯誤"""
    def __init__(self, message: str, line: int = 0, column: int = 0):
        self.line = line
        self.column = column
        super().__init__(f"Parse error at line {line}: {message}")


class TAFLParser:
    """
    TAFL YAML Parser
    將 TAFL YAML 文件解析為 AST
    """
    
    # Regular expressions for parsing expressions
    VARIABLE_PATTERN = re.compile(r'\$\{([^}]+)\}')
    # 支援 &&, ||, and, or 運算符
    OPERATOR_PATTERN = re.compile(r'(==|!=|<=|>=|<|>|\+|-|\*|/|&&|\|\||and|or|not)')
    
    def __init__(self):
        self.current_file = ""
        self.current_line = 0
        
    def parse_file(self, file_path: str) -> TAFLProgram:
        """
        Parse a TAFL YAML file
        解析 TAFL YAML 文件
        """
        self.current_file = file_path
        
        with open(file_path, 'r', encoding='utf-8') as f:
            try:
                data = yaml.safe_load(f)
            except yaml.YAMLError as e:
                raise TAFLParseError(f"YAML syntax error: {e}")
        
        return self.parse_program(data)
    
    def parse_string(self, yaml_content: str) -> TAFLProgram:
        """
        Parse TAFL YAML from string
        從字符串解析 TAFL YAML
        """
        try:
            data = yaml.safe_load(yaml_content)
        except yaml.YAMLError as e:
            raise TAFLParseError(f"YAML syntax error: {e}")
        
        return self.parse_program(data)
    
    def parse_program(self, data: Dict[str, Any]) -> TAFLProgram:
        """
        Parse the complete TAFL program
        解析完整的 TAFL 程序
        """
        # Parse metadata
        metadata = self._parse_metadata(data.get('metadata', {}))
        
        # Parse settings
        settings = self._parse_settings(data.get('settings', {}))
        
        # Parse preload statements (new in v1.1)
        preload_data = data.get('preload', [])
        preload = self._parse_preload_statements(preload_data)
        
        # Parse rules (new in v1.1)
        rules_data = data.get('rules', {})
        rules = self._parse_rules(rules_data)
        
        # Parse initial variables
        variables = data.get('variables', {})
        
        # Parse flow statements
        flow_data = data.get('flow', [])
        flow = self._parse_statements(flow_data)
        
        return TAFLProgram(
            metadata=metadata,
            settings=settings,
            preload=preload,
            rules=rules,
            variables=variables,
            flow=flow
        )
    
    def _parse_metadata(self, data: Dict[str, Any]) -> FlowMetadata:
        """Parse flow metadata / 解析流程元數據"""
        if not data:
            data = {}
        return FlowMetadata(
            id=data.get('id', 'unnamed_flow'),
            name=data.get('name', 'Unnamed Flow'),
            version=data.get('version', '1.1.2'),
            author=data.get('author'),
            enabled=data.get('enabled', True),  # v1.1.2: Default to True if not specified
            description=data.get('description'),
            tags=data.get('tags', []),
            created_at=data.get('created_at'),
            updated_at=data.get('updated_at')
        )
    
    def _parse_settings(self, data: Dict[str, Any]) -> FlowSettings:
        """Parse flow settings / 解析流程設置"""
        if not data:
            data = {}
        return FlowSettings(
            timeout=data.get('timeout', 300),
            retry_on_failure=data.get('retry_on_failure', False),
            max_retries=data.get('max_retries', 3),
            parallel_execution=data.get('parallel_execution', False),
            log_level=data.get('log_level', 'INFO'),
            checkpoint=data.get('checkpoint', True)
        )
    
    def _parse_statements(self, statements: List[Any]) -> List[Statement]:
        """Parse a list of statements / 解析語句列表"""
        result = []
        for stmt_data in statements:
            if isinstance(stmt_data, dict):
                stmt = self._parse_statement(stmt_data)
                if stmt:
                    # Handle both single statement and list of statements (for multi-variable set)
                    if isinstance(stmt, list):
                        result.extend(stmt)
                    else:
                        result.append(stmt)
        return result
    
    def _parse_statement(self, data: Dict[str, Any]) -> Optional[Statement]:
        """Parse a single statement / 解析單個語句"""
        # Get the verb (first key that matches a known verb)
        verb = None
        for key in data.keys():
            if key in ['query', 'check', 'create', 'update', 'if', 'for', 
                      'switch', 'set', 'stop', 'notify', 'log']:
                verb = key
                break
        
        if not verb:
            return None
        
        # Parse based on verb type
        stmt_data = data[verb]
        comment = data.get('_comment') or data.get('comment')
        
        if verb == 'query':
            return self._parse_query(stmt_data, comment)
        elif verb == 'check':
            return self._parse_check(stmt_data, comment)
        elif verb == 'create':
            return self._parse_create(stmt_data, comment)
        elif verb == 'update':
            return self._parse_update(stmt_data, comment)
        elif verb == 'if':
            return self._parse_if(stmt_data, comment)
        elif verb == 'for':
            return self._parse_for(stmt_data, comment)
        elif verb == 'switch':
            return self._parse_switch(stmt_data, comment)
        elif verb == 'set':
            # Special handling for set - might return multiple statements
            set_statements = self._parse_set(stmt_data, comment)
            return set_statements  # Return list or single statement
        elif verb == 'stop':
            return self._parse_stop(stmt_data, comment)
        elif verb == 'notify':
            return self._parse_notify(stmt_data, comment)
        
        return None
    
    def _parse_query(self, data: Any, comment: Optional[str]) -> QueryStatement:
        """Parse query statement / 解析查詢語句 - TAFL v1.1.2"""
        if isinstance(data, str):
            # Simple query: "query: locations"
            return QueryStatement(target=data, comment=comment)
        
        # Complex query with filters - TAFL v1.1.2
        target = data.get('target')
        if not target:
            raise TAFLParseError("Query statement requires 'target' field (TAFL v1.1.2)")
        
        filters = {}
        # TAFL v1.1.2: 'as' field for storing results
        as_ = data.get('as')
        
        # Parse where clause - TAFL v1.1.2
        where = data.get('where', {})
        if where:
            if isinstance(where, dict):
                # Dictionary format: {key: value, ...}
                for key, value in where.items():
                    filters[key] = self._parse_expression(value)
            elif isinstance(where, str):
                # String format: "status = 'pending'"
                # For now, treat as a single filter expression
                filters['_expression'] = self._parse_expression(where)
        
        # Parse limit parameter
        limit = None
        if 'limit' in data:
            limit = self._parse_expression(data['limit'])
        
        return QueryStatement(
            target=target,
            filters=filters if filters else None,
            limit=limit,
            as_=as_,
            comment=comment
        )
    
    def _parse_check(self, data: Any, comment: Optional[str]) -> CheckStatement:
        """Parse check statement / 解析檢查語句
        
        v1.1.3: 'as' parameter is now required for all check statements
        """
        if isinstance(data, str):
            # Simple format not allowed for check - must have 'as' parameter
            raise TAFLParseError(
                f"Check statement must include 'as' parameter to store result. "
                f"Use structured format: check: {{condition: '{data}', as: 'variable_name'}}"
            )
        
        # TAFL v1.1.3: condition field is required
        condition_expr = data.get('condition')
        if condition_expr is None:
            raise TAFLParseError("Check statement requires 'condition' field (TAFL v1.1.3)")
        
        target = data.get('target', 'condition')
        condition = self._parse_expression(condition_expr)
        # TAFL v1.1.3: 'as' parameter is required
        as_ = data.get('as')
        
        if not as_:
            raise TAFLParseError(
                f"Check statement must include 'as' parameter to store result. "
                f"Example: check: {{condition: '{condition_expr}', as: 'result_variable'}}"
            )
        
        return CheckStatement(
            target=target,
            condition=condition,
            as_=as_,
            comment=comment
        )
    
    def _parse_create(self, data: Any, comment: Optional[str]) -> CreateStatement:
        """Parse create statement / 解析創建語句 - TAFL v1.1.2"""
        if isinstance(data, str):
            # Simple create: "create: task"
            return CreateStatement(target=data, comment=comment)
        
        target = data.get('target')
        if not target:
            raise TAFLParseError("Create statement requires 'target' field (TAFL v1.1.2)")
        
        # TAFL v1.1.2: 'as' field for storing results
        as_ = data.get('as')
        
        # TAFL v1.1.2: 'with' field for parameters
        with_ = {}
        
        if 'with' in data:
            with_data = data['with']
            if isinstance(with_data, dict):
                for key, value in with_data.items():
                    with_[key] = self._parse_expression(value)
        
        return CreateStatement(
            target=target,
            with_=with_,
            as_=as_,
            comment=comment
        )
    
    def _parse_update(self, data: Any, comment: Optional[str]) -> UpdateStatement:
        """Parse update statement / 解析更新語句 - TAFL v1.1.2"""
        target = data.get('target')
        if not target:
            raise TAFLParseError("Update statement requires 'target' field (TAFL v1.1.2)")
        
        # Parse where conditions - TAFL v1.1.2
        where = {}
        where_data = data.get('where', {})
        
        # Parse where conditions
        if isinstance(where_data, dict):
            for key, value in where_data.items():
                where[key] = self._parse_expression(value)
        
        # Parse set changes - TAFL v1.1.2
        set_ = {}
        set_data = data.get('set', {})
        if isinstance(set_data, dict):
            for key, value in set_data.items():
                set_[key] = self._parse_expression(value)
        
        return UpdateStatement(
            target=target,
            where=where,
            set=set_,
            comment=comment
        )
    
    def _parse_if(self, data: Any, comment: Optional[str]) -> IfStatement:
        """Parse if statement / 解析條件語句 - TAFL v1.1.2"""
        condition = data.get('condition')
        if not condition:
            raise TAFLParseError("If statement requires 'condition' field (TAFL v1.1.2)")
        condition = self._parse_expression(condition)
        
        then_data = data.get('then', [])
        then_branch = self._parse_statements(then_data if isinstance(then_data, list) else [then_data])
        
        else_data = data.get('else')
        else_branch = None
        if else_data:
            else_branch = self._parse_statements(else_data if isinstance(else_data, list) else [else_data])
        
        return IfStatement(
            condition=condition,
            then_branch=then_branch,
            else_branch=else_branch,
            comment=comment
        )
    
    def _parse_for(self, data: Any, comment: Optional[str]) -> ForStatement:
        """Parse for loop statement / 解析循環語句 - TAFL v1.1.2"""
        # TAFL v1.1.2: 'as' field is required
        as_ = data.get('as')
        if not as_:
            raise TAFLParseError("For statement requires 'as' field (TAFL v1.1.2)")
        
        # Parse iteration target - TAFL v1.1.2
        in_expr = data.get('in')
        if not in_expr:
            raise TAFLParseError("For statement requires 'in' field (TAFL v1.1.2)")
        
        # Parse filter condition (v1.1)
        filter_expr = data.get('filter')
        filter_condition = None
        if filter_expr:
            filter_condition = self._parse_expression(filter_expr)
        
        # Parse loop body - TAFL v1.1.2
        do_data = data.get('do', [])
        do = self._parse_statements(do_data if isinstance(do_data, list) else [do_data])
        
        return ForStatement(
            in_=self._parse_expression(in_expr),
            as_=as_,
            do=do,
            filter=filter_condition,
            comment=comment
        )
    
    def _parse_switch(self, data: Any, comment: Optional[str]) -> SwitchStatement:
        """Parse switch statement / 解析分支語句
        
        TAFL v1.1.2 format: expression + cases array with when/do
        Default case uses when:"default" in cases array
        """
        # Handle simplified format: switch: expression
        if isinstance(data, str):
            expression = self._parse_expression(data)
            cases = []
        else:
            # TAFL v1.1.2 format requires expression and cases array
            if 'expression' not in data or not isinstance(data.get('cases'), list):
                raise TAFLParseError(
                    f"Switch statement must have 'expression' and 'cases' array (TAFL v1.1.2 format). "
                    f"Found: {list(data.keys()) if isinstance(data, dict) else type(data)}"
                )
            
            expression = self._parse_expression(data['expression'])
            cases = []
            has_default = False
            
            for case_item in data['cases']:
                when_value = case_item.get('when')
                do_body = case_item.get('do', [])
                
                if not when_value:
                    raise TAFLParseError(f"Switch case must have 'when' field")
                
                # Check if this is a default case - keep it as a regular case
                if when_value == 'default':
                    if has_default:
                        raise TAFLParseError(f"Switch can only have one default case")
                    has_default = True
                    # Store "default" as a special literal
                    case_value = Literal(value='default', type='string')
                else:
                    # Parse the when condition
                    if isinstance(when_value, str):
                        # Parse conditions like "> 8", "5..8", etc.
                        case_value = self._parse_case_condition(when_value)
                    else:
                        case_value = self._parse_expression(when_value)
                
                case = SwitchCase(
                    when=case_value,
                    do=self._parse_statements(do_body if isinstance(do_body, list) else [do_body])
                )
                cases.append(case)
        
        return SwitchStatement(
            expression=expression,
            cases=cases,
            comment=comment
        )
    
    def _parse_case_condition(self, condition: str) -> Expression:
        """Parse case condition strings like '> 8', '5..8', '< 5'
        
        For now, returns as string literal to maintain compatibility.
        Future enhancement: parse into proper comparison expressions.
        """
        # For now, keep it simple and return as string literal
        # The executor will need to handle these special conditions
        return Literal(value=condition, type='string')
    
    def _parse_set(self, data: Any, comment: Optional[str]):
        """Parse set statement / 解析設置語句
        
        TAFL v1.1.2: Only accepts object/dictionary format
        不再支援單行字串格式 "variable = value"
        """
        if isinstance(data, dict):
            # Dictionary format: set: {var1: value1, var2: value2, ...}
            statements = []
            for var, value in data.items():
                statements.append(SetStatement(
                    variable=var,
                    value=self._parse_expression(value),
                    comment=comment if len(statements) == 0 else None  # Only first gets comment
                ))
            
            # Return single statement if only one, list if multiple
            return statements[0] if len(statements) == 1 else statements
        
        # TAFL v1.1.2: String format no longer supported
        if isinstance(data, str):
            if '=' in data:
                raise TAFLParseError(
                    f"TAFL v1.1.2: String format 'variable = value' is no longer supported. "
                    f"Use object format instead: set: {{variable: value}} or multi-line YAML format. "
                    f"Found: {data}"
                )
            else:
                raise TAFLParseError(f"Invalid set statement format: {data}")
        
        raise TAFLParseError(f"Invalid set statement format: {data}")
    
    def _parse_stop(self, data: Any, comment: Optional[str]) -> StopStatement:
        """Parse stop statement / 解析停止語句"""
        if isinstance(data, str):
            # Simple stop with reason
            return StopStatement(reason=data, comment=comment)
        
        if isinstance(data, dict):
            reason = data.get('reason', data.get('message'))
            condition = data.get('if', data.get('when'))
            return StopStatement(
                reason=reason,
                condition=self._parse_expression(condition) if condition else None,
                comment=comment
            )
        
        return StopStatement(comment=comment)
    
    def _parse_notify(self, data: Any, comment: Optional[str]) -> NotifyStatement:
        """Parse notify statement / 解析通知語句 - 符合 TAFL v1.1.1 規格"""
        if isinstance(data, str):
            # Simple notification - interpret as level
            return NotifyStatement(
                level=data if data in ['info', 'warning', 'error', 'critical', 'alarm'] else 'info',
                message=Literal(value=data if data not in ['info', 'warning', 'error', 'critical', 'alarm'] else '', type='string'),
                comment=comment
            )
        
        if not isinstance(data, dict):
            # Handle None or other invalid types
            return NotifyStatement(
                level='info',
                message=Literal(value='', type='string'),
                comment=comment
            )
        
        # 使用標準 'level' 欄位（TAFL v1.1 規格）
        level = data.get('level', 'info')
        message = self._parse_expression(data.get('message', ''))
        
        # Parse recipients
        recipients = data.get('recipients')
        if recipients and not isinstance(recipients, list):
            recipients = [recipients]
        
        # Parse details (replacing metadata)
        details = {}
        for key, value in data.items():
            if key not in ['level', 'message', 'recipients', 'details']:
                details[key] = self._parse_expression(value)
        
        # Also include explicit details field if present
        if 'details' in data and isinstance(data['details'], dict):
            for key, value in data['details'].items():
                details[key] = self._parse_expression(value)
        
        return NotifyStatement(
            level=level,
            message=message,
            recipients=recipients,
            details=details if details else None,
            comment=comment
        )
    
    def _parse_expression(self, expr: Any) -> Expression:
        """
        Parse an expression
        解析表達式
        """
        if expr is None:
            return Literal(value=None, type='null')
        
        if isinstance(expr, bool):
            return Literal(value=expr, type='boolean')
        
        if isinstance(expr, (int, float)):
            return Literal(value=expr, type='number')
        
        if isinstance(expr, list):
            elements = [self._parse_expression(e) for e in expr]
            return ArrayExpression(elements=elements)
        
        if isinstance(expr, dict):
            pairs = [(k, self._parse_expression(v)) for k, v in expr.items()]
            return DictExpression(pairs=pairs)
        
        if isinstance(expr, str):
            # Check for pure variable reference (entire string is ${...})
            var_match = self.VARIABLE_PATTERN.fullmatch(expr)
            if var_match:
                inner_content = var_match.group(1)
                
                # Check for array/dict indexing BEFORE checking for operators
                if '[' in inner_content and ']' in inner_content:
                    return self._parse_index_access(inner_content)
                
                # Check if the content contains operators - if so, parse as complex expression
                # Check for any operator presence (包含 &&, || 運算符)
                has_operator = any(op in inner_content for op in
                                 ['==', '!=', '<=', '>=', '<', '>', ' and ', ' or ', ' && ', ' || ',
                                  '+', '-', '*', '/', 'not '])
                
                if has_operator or '(' in inner_content:  # Also check for parentheses
                    # Parse as complex expression
                    return self._parse_complex_expression(expr)
                else:
                    # Parse as variable reference
                    var_path = inner_content.split('.')
                    return Variable(name=var_path[0], path=var_path[1:])
            
            # Check for function calls (including those wrapped in ${})
            # First check if entire expression is ${func(...)}
            if expr.startswith('${') and expr.endswith('}'):
                inner = expr[2:-1].strip()
                if '(' in inner and ')' in inner:
                    # Try to parse as function call
                    func_match = re.match(r'\w+\(.*\)', inner)
                    if func_match:
                        return self._parse_function_call(inner)
            # Then check for plain function calls
            elif '(' in expr and ')' in expr:
                return self._parse_function_call(expr)
            
            # Check if it's a pure number (no ${} interpolation)
            if '${' not in expr:
                try:
                    if '.' in expr:
                        return Literal(value=float(expr), type='number')
                    else:
                        return Literal(value=int(expr), type='number')
                except ValueError:
                    pass
            
            # Strip quotes from string literals
            # Check if the string is quoted (starts and ends with matching quotes)
            if ((expr.startswith('"') and expr.endswith('"')) or 
                (expr.startswith("'") and expr.endswith("'"))) and len(expr) >= 2:
                # Strip the quotes from the value
                expr = expr[1:-1]
            
            # IMPORTANT: Expressions with ${...} interpolation should be treated as
            # string literals that will be evaluated at runtime by the executor.
            # This includes mathematical expressions like "${a} + ${b}"
            # The executor's _interpolate_string and _evaluate_expression will handle them.
            return Literal(value=expr, type='string')
        
        # Default to literal
        return Literal(value=expr, type='string')
    
    def _parse_complex_expression(self, expr: str) -> Expression:
        """
        Parse complex expression with operators
        解析包含運算符的複雜表達式
        """
        expr = expr.strip()
        
        # Handle ${...} wrapped expressions
        if expr.startswith('${') and expr.endswith('}'):
            # Extract content inside ${...}
            inner_expr = expr[2:-1].strip()
            
            # Handle 'not' operator
            if inner_expr.startswith('not '):
                operand_expr = inner_expr[4:].strip()
                # Check if operand is a variable
                if operand_expr in self.VARIABLE_PATTERN.pattern:
                    operand = Variable(name=operand_expr, path=[])
                else:
                    operand = self._parse_expression(operand_expr)
                return UnaryOp(operator='not', operand=operand)
            
            # Parse the expression respecting parentheses
            return self._parse_binary_expression(inner_expr)
        
        # Handle expressions without ${...} wrapper
        # Handle 'not' operator
        if expr.startswith('not '):
            return UnaryOp(operator='not', operand=self._parse_expression(expr[4:]))
        
        # Parse binary expressions
        return self._parse_binary_expression(expr)
    
    def _parse_binary_expression(self, expr: str) -> Expression:
        """
        Parse binary expression respecting parentheses and operator precedence
        """
        expr = expr.strip()
        
        # Remove outer parentheses if they wrap the entire expression
        if expr.startswith('(') and expr.endswith(')'):
            # Check if these parentheses are balanced and wrap the whole expression
            if self._find_matching_paren(expr, 0) == len(expr) - 1:
                # Recursively parse the content inside parentheses
                return self._parse_binary_expression(expr[1:-1])
        
        # Process operators in order of precedence (lower precedence first)
        # This way higher precedence operators are nested deeper in the tree
        # Comparison operators have lower precedence than arithmetic
        operator_groups = [
            [' or ', ' || '],  # 支援 or 和 ||
            [' and ', ' && '],  # 支援 and 和 &&
            ['==', '!=', '<=', '>=', '<', '>'],
            ['+', '-'],
            ['*', '/']
        ]
        
        for operators in operator_groups:
            for op in operators:
                # Find the operator at the top level (not inside parentheses)
                pos = self._find_operator_at_level(expr, op)
                if pos != -1:
                    left_part = expr[:pos].strip()
                    right_part = expr[pos + len(op):].strip()
                    
                    # Helper function to parse operand
                    def parse_operand(part):
                        # Check if it needs recursive parsing
                        if '(' in part or any(o in part for o in ['*', '/', '+', '-', '<', '>', '==', '!=']):
                            return self._parse_binary_expression(part)
                        
                        # Check if it's a simple variable
                        if self._is_simple_variable(part):
                            return Variable(name=part, path=[])
                        
                        # Check if it's a dotted variable (e.g., rules.priority_multiplier)
                        if '.' in part and all(self._is_simple_variable(p) for p in part.split('.')):
                            parts = part.split('.')
                            return Variable(name=parts[0], path=parts[1:])
                        
                        # Otherwise parse as expression
                        return self._parse_expression(part)
                    
                    # Recursively parse operands
                    left = parse_operand(left_part)
                    right = parse_operand(right_part)

                    # 規範化運算符：|| -> or, && -> and
                    normalized_op = op.strip()
                    if normalized_op == '||':
                        normalized_op = 'or'
                    elif normalized_op == '&&':
                        normalized_op = 'and'

                    return BinaryOp(
                        operator=normalized_op,
                        left=left,
                        right=right
                    )
        
        # If no operators found, parse as simple expression
        # Try to parse as number first
        try:
            if '.' in expr:
                return Literal(value=float(expr), type='number')
            else:
                return Literal(value=int(expr), type='number')
        except ValueError:
            pass
        
        # Check if it's a variable
        if self._is_simple_variable(expr):
            return Variable(name=expr, path=[])
        else:
            return self._parse_expression(expr)
    
    def _parse_index_access(self, expr: str) -> Expression:
        """
        Parse array/dict index access expressions like variable[0] or array[-1]
        解析陣列/字典索引存取表達式
        """
        # Find the first '[' to split object and index
        bracket_pos = expr.find('[')
        if bracket_pos == -1:
            # No brackets, shouldn't happen but handle gracefully
            var_path = expr.split('.')
            return Variable(name=var_path[0], path=var_path[1:])
        
        # Get the object part (before '[')
        object_part = expr[:bracket_pos].strip()
        
        # Parse remaining part for indices (may have nested indexing like [0][1])
        remaining = expr[bracket_pos:]
        indices = []
        
        while remaining and remaining.startswith('['):
            # Find matching ']'
            bracket_end = self._find_matching_bracket(remaining, 0)
            if bracket_end == -1:
                raise TAFLSyntaxError(f"Unmatched '[' in expression: {expr}")
            
            # Extract index expression (without brackets)
            index_expr = remaining[1:bracket_end]
            indices.append(index_expr)
            
            # Move to next potential index
            remaining = remaining[bracket_end + 1:]
        
        # Parse the object part
        if '.' in object_part:
            var_path = object_part.split('.')
            obj_expr = Variable(name=var_path[0], path=var_path[1:])
        else:
            obj_expr = Variable(name=object_part, path=[])
        
        # Build nested IndexAccess for multiple indices
        result = obj_expr
        for index_str in indices:
            # Parse the index expression
            # Check if it's a negative number (common case)
            if index_str.startswith('-') and index_str[1:].isdigit():
                index_expr = Literal(value=int(index_str), type='number')
            # Check if it's a positive number
            elif index_str.isdigit():
                index_expr = Literal(value=int(index_str), type='number')
            else:
                # Parse as expression (could be variable or complex expression)
                index_expr = self._parse_expression(index_str)
            
            result = IndexAccess(object=result, index=index_expr)
        
        return result
    
    def _find_matching_bracket(self, expr: str, start_pos: int) -> int:
        """
        Find the position of matching closing bracket ']'
        """
        if expr[start_pos] != '[':
            return -1
        level = 1
        i = start_pos + 1
        while i < len(expr):
            if expr[i] == '[':
                level += 1
            elif expr[i] == ']':
                level -= 1
                if level == 0:
                    return i
            i += 1
        return -1
    
    def _find_operator_at_level(self, expr: str, op: str) -> int:
        """
        Find operator position at top level (not inside parentheses)
        Returns -1 if not found at top level
        """
        level = 0
        i = 0
        while i < len(expr):
            if expr[i] == '(':
                level += 1
            elif expr[i] == ')':
                level -= 1
            elif level == 0 and expr[i:i+len(op)] == op:
                return i
            i += 1
        return -1
    
    def _find_matching_paren(self, expr: str, start_pos: int) -> int:
        """
        Find the position of matching closing parenthesis
        """
        if expr[start_pos] != '(':
            return -1
        level = 1
        i = start_pos + 1
        while i < len(expr) and level > 0:
            if expr[i] == '(':
                level += 1
            elif expr[i] == ')':
                level -= 1
            i += 1
        return i - 1 if level == 0 else -1
        
        # If no operators found, treat as string
        return Literal(value=expr, type='string')
    
    def _is_simple_variable(self, name: str) -> bool:
        """Check if a string is a simple variable name"""
        import re
        return bool(re.match(r'^[a-zA-Z_][a-zA-Z0-9_]*$', name))
    
    def _parse_function_call(self, expr: str) -> Expression:
        """
        Parse function call expression with improved argument parsing
        改進的函數調用表達式解析，支援 ${var} 變數引用

        Supports:
        - ${var} for variable references
        - 'string' or "string" for string literals
        - numbers for numeric literals
        - backward compatibility with simple identifiers
        """
        match = re.match(r'(\w+)\((.*)\)', expr.strip())
        if match:
            func_name = match.group(1)
            args_str = match.group(2)

            # Parse arguments with improved splitting
            args = []
            if args_str:
                # Use intelligent splitting that respects nesting and quotes
                arg_list = self._split_function_args(args_str)
                for arg in arg_list:
                    args.append(self._parse_function_arg(arg.strip()))

            return FunctionCall(name=func_name, arguments=args)

        return Literal(value=expr, type='string')

    def _split_function_args(self, args_str: str) -> list:
        """
        Intelligently split function arguments, handling nested parentheses and quotes
        智能分割函數參數，處理嵌套括號和引號
        """
        args = []
        current_arg = ""
        depth = 0
        in_quotes = False
        quote_char = None

        for i, char in enumerate(args_str):
            # Handle quotes
            if char in ('"', "'") and not in_quotes:
                in_quotes = True
                quote_char = char
                current_arg += char
            elif char == quote_char and in_quotes:
                # Check for escaped quotes
                if i > 0 and args_str[i-1] != '\\':
                    in_quotes = False
                    quote_char = None
                current_arg += char
            elif char == ',' and depth == 0 and not in_quotes:
                # Found argument separator
                if current_arg.strip():
                    args.append(current_arg.strip())
                current_arg = ""
            else:
                # Track parentheses depth
                if char == '(' and not in_quotes:
                    depth += 1
                elif char == ')' and not in_quotes:
                    depth -= 1
                current_arg += char

        # Add the last argument
        if current_arg.strip():
            args.append(current_arg.strip())

        return args

    def _parse_function_arg(self, arg: str) -> Expression:
        """
        Parse a single function argument
        解析單個函數參數

        Priority:
        1. ${var} or ${var.path} or ${var[index].path} -> Variable or complex expression
        2. 'string' or "string" -> Literal string
        3. numbers -> Literal number
        4. true/false -> Literal boolean
        5. other identifiers -> Literal string (for backward compatibility)
        """
        arg = arg.strip()

        # Check for ${var} format - parse as expression
        if arg.startswith('${') and arg.endswith('}'):
            var_content = arg[2:-1].strip()

            # For complex expressions with array access, return as string literal for runtime interpolation
            if '[' in var_content and ']' in var_content:
                # This is an array access expression like available_locations[0].id
                # Return as string literal so it gets interpolated at runtime
                return Literal(value=arg, type='string')

            # Simple variable paths (e.g., ${object.property}) - handle normally
            var_parts = var_content.split('.')
            return Variable(name=var_parts[0], path=var_parts[1:] if len(var_parts) > 1 else [])

        # Check for quoted strings
        if (arg.startswith('"') and arg.endswith('"')) or \
           (arg.startswith("'") and arg.endswith("'")):
            # Remove quotes and return as string literal
            return Literal(value=arg[1:-1], type='string')

        # Check for numbers
        try:
            # Try integer first
            if '.' not in arg and 'e' not in arg.lower():
                return Literal(value=int(arg), type='number')
            else:
                # Try float
                return Literal(value=float(arg), type='number')
        except ValueError:
            pass

        # Check for boolean literals
        if arg.lower() == 'true':
            return Literal(value=True, type='boolean')
        elif arg.lower() == 'false':
            return Literal(value=False, type='boolean')
        elif arg.lower() == 'null' or arg.lower() == 'none':
            return Literal(value=None, type='null')

        # NEW: Check if it's a valid variable identifier (fixes math function bug)
        # A valid identifier starts with letter or underscore, followed by alphanumeric or underscore
        if re.match(r'^[a-zA-Z_]\w*$', arg):
            # This is a variable reference (e.g., sum(items) where items is a variable)
            # Parse it as Variable instead of string literal
            return Variable(name=arg, path=[])

        # For backward compatibility: treat unquoted identifiers as string literals
        # This allows existing code like func(test_id) to work as before
        return Literal(value=arg, type='string')
    
    def _parse_preload_statements(self, preload_data: Any) -> List[PreloadStatement]:
        """Parse preload statements / 解析預載入語句
        
        Supports both dict format (v1.1) and list format:
        - Dict: {name: {query: {...}}, ...}
        - List: [{query: {...}}, ...]
        """
        result = []
        
        # Handle dict format (v1.1)
        if isinstance(preload_data, dict):
            for name, stmt_data in preload_data.items():
                if isinstance(stmt_data, dict) and 'query' in stmt_data:
                    query_data = stmt_data['query']
                    comment = stmt_data.get('comment')
                    
                    # Parse query fields
                    target = query_data.get('target', '')
                    # 統一使用 'as' （規格書定義）
                    as_ = name  # Use the key as as_ name
                    
                    # Parse where clause
                    filters = None
                    where = query_data.get('where')
                    if where:
                        if isinstance(where, dict):
                            filters = {}
                            for key, value in where.items():
                                filters[key] = self._parse_expression(value)
                        elif isinstance(where, str):
                            # Handle string where clause like "active = true"
                            # Wrap in dict to maintain consistent structure
                            filters = {'_expression': self._parse_expression(where)}
                    
                    # Parse limit
                    limit = query_data.get('limit')
                    
                    preload_stmt = PreloadStatement(
                        target=target,
                        filters=filters,
                        limit=limit,
                        as_=as_,
                        comment=comment
                    )
                    result.append(preload_stmt)
        
        # Handle list format (backward compatibility)
        elif isinstance(preload_data, list):
            for stmt_data in preload_data:
                if isinstance(stmt_data, dict) and 'query' in stmt_data:
                    query_data = stmt_data['query']
                    comment = stmt_data.get('comment')
                    
                    # Parse query fields - TAFL v1.1.2
                    target = query_data.get('target')
                    if not target:
                        raise TAFLParseError("Preload query requires 'target' field (TAFL v1.1.2)")
                    # TAFL v1.1.2: 'as' field for storing results
                    as_ = query_data.get('as')
                    
                    # Parse where clause
                    filters = None
                    where = query_data.get('where', {})
                    if where:
                        filters = {}
                        for key, value in where.items():
                            filters[key] = self._parse_expression(value)
                    
                    # Parse limit
                    limit = query_data.get('limit')
                    
                    preload_stmt = PreloadStatement(
                        target=target,
                        filters=filters,
                        limit=limit,
                        as_=as_,
                        comment=comment
                    )
                    result.append(preload_stmt)
        
        return result
    
    def _parse_rules(self, rules_data: Dict[str, Any]) -> Dict[str, RuleDefinition]:
        """Parse rules section / 解析規則區塊"""
        result = {}
        
        for rule_name, rule_data in rules_data.items():
            if isinstance(rule_data, dict):
                # Rule with condition and description
                if 'condition' in rule_data:
                    condition = self._parse_expression(rule_data['condition'])
                    description = rule_data.get('description')
                    rule = RuleDefinition(
                        condition=condition,
                        description=description
                    )
                elif 'value' in rule_data:
                    # Configuration rule with value and optional description
                    value = rule_data.get('value')
                    description = rule_data.get('description')
                    rule = RuleDefinition(
                        value=value,
                        description=description
                    )
                else:
                    # Dict without 'condition' or 'value' keys - treat as value
                    rule = RuleDefinition(value=rule_data)
            else:
                # Simple configuration value
                rule = RuleDefinition(value=rule_data)
            
            result[rule_name] = rule
        
        return result