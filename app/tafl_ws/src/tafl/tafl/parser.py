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
    OPERATOR_PATTERN = re.compile(r'(==|!=|<=|>=|<|>|\+|-|\*|/|and|or|not)')
    
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
            version=data.get('version', '1.0'),
            author=data.get('author'),
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
                      'switch', 'set', 'stop', 'notify']:
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
        """Parse query statement / 解析查詢語句"""
        if isinstance(data, str):
            # Simple query: "query: racks"
            return QueryStatement(target=data, comment=comment)
        
        # Complex query with filters
        # Support 'from', 'target', or 'what' as target field
        target = data.get('from', data.get('target', data.get('what', '')))
        filters = {}
        store_as = data.get('store_as', data.get('store', data.get('as')))
        
        # Parse where clause or filters
        where = data.get('where', data.get('filter', {}))
        if where:
            for key, value in where.items():
                filters[key] = self._parse_expression(value)
        
        return QueryStatement(
            target=target,
            filters=filters if filters else None,
            store_as=store_as,
            comment=comment
        )
    
    def _parse_check(self, data: Any, comment: Optional[str]) -> CheckStatement:
        """Parse check statement / 解析檢查語句"""
        if isinstance(data, str):
            # Simple check parsed as expression
            return CheckStatement(
                target="condition",
                condition=self._parse_expression(data),
                comment=comment
            )
        
        target = data.get('target', data.get('what', 'condition'))
        condition = self._parse_expression(data.get('condition', data.get('if', True)))
        store_as = data.get('store', data.get('as'))
        
        return CheckStatement(
            target=target,
            condition=condition,
            store_as=store_as,
            comment=comment
        )
    
    def _parse_create(self, data: Any, comment: Optional[str]) -> CreateStatement:
        """Parse create statement / 解析創建語句"""
        if isinstance(data, str):
            # Simple create: "create: task"
            return CreateStatement(target=data, comment=comment)
        
        target = data.get('target', data.get('what', 'task'))
        store_as = data.get('store', data.get('as'))
        
        # Parse parameters
        parameters = {}
        for key, value in data.items():
            if key not in ['target', 'what', 'store', 'as']:
                parameters[key] = self._parse_expression(value)
        
        return CreateStatement(
            target=target,
            parameters=parameters,
            store_as=store_as,
            comment=comment
        )
    
    def _parse_update(self, data: Any, comment: Optional[str]) -> UpdateStatement:
        """Parse update statement / 解析更新語句"""
        target = data.get('target', data.get('what', ''))
        id_expr = self._parse_expression(data.get('id', data.get('which', 0)))
        
        # Parse changes
        changes = {}
        set_data = data.get('set', data.get('with', {}))
        for key, value in set_data.items():
            changes[key] = self._parse_expression(value)
        
        return UpdateStatement(
            target=target,
            id_expr=id_expr,
            changes=changes,
            comment=comment
        )
    
    def _parse_if(self, data: Any, comment: Optional[str]) -> IfStatement:
        """Parse if statement / 解析條件語句"""
        condition = self._parse_expression(data.get('condition', data.get('when', True)))
        
        then_data = data.get('then', data.get('do', []))
        then_branch = self._parse_statements(then_data if isinstance(then_data, list) else [then_data])
        
        else_data = data.get('else', data.get('otherwise'))
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
        """Parse for loop statement / 解析循環語句"""
        # Parse loop variable and iterable
        each = data.get('each', data.get('item', 'item'))
        in_expr = data.get('in', data.get('from', []))
        
        # Parse filter condition (new in v1.1)
        filter_expr = data.get('filter')
        filter_condition = None
        if filter_expr:
            filter_condition = self._parse_expression(filter_expr)
        
        # Parse loop body
        do_data = data.get('do', data.get('body', []))
        body = self._parse_statements(do_data if isinstance(do_data, list) else [do_data])
        
        return ForStatement(
            variable=each,
            iterable=self._parse_expression(in_expr),
            filter=filter_condition,
            body=body,
            comment=comment
        )
    
    def _parse_switch(self, data: Any, comment: Optional[str]) -> SwitchStatement:
        """Parse switch statement / 解析分支語句"""
        # Handle simplified format: switch: expression
        if isinstance(data, str):
            on = self._parse_expression(data)
            cases = []
            default = []
        else:
            # Handle YAML quirk where 'on' becomes boolean True
            on_value = data.get('on') or data.get(True) or data.get('value', '')
            on = self._parse_expression(on_value) if on_value else Literal(value=None, type='null')
            
            cases = []
            cases_data = data.get('cases', {})
            # Handle empty cases list
            if isinstance(cases_data, list) and len(cases_data) == 0:
                cases_data = {}
            for value, body in cases_data.items():
                # Parse the case value based on its type
                if isinstance(value, (int, float)):
                    case_value = Literal(value=value, type='number')
                elif isinstance(value, bool):
                    case_value = Literal(value=value, type='boolean')
                elif isinstance(value, str):
                    # Check if it's a number string
                    try:
                        if '.' in value:
                            case_value = Literal(value=float(value), type='number')
                        else:
                            case_value = Literal(value=int(value), type='number')
                    except ValueError:
                        # It's a string literal
                        case_value = Literal(value=value, type='string')
                else:
                    # Default to string representation
                    case_value = Literal(value=str(value), type='string')
                
                case = SwitchCase(
                    value=case_value,
                    body=self._parse_statements(body if isinstance(body, list) else [body])
                )
                cases.append(case)
            
            default_data = data.get('default')
            default = None
            if default_data:
                default = self._parse_statements(default_data if isinstance(default_data, list) else [default_data])
        
        return SwitchStatement(
            expression=on,
            cases=cases,
            default=default,
            comment=comment
        )
    
    def _parse_set(self, data: Any, comment: Optional[str]):
        """Parse set statement / 解析設置語句"""
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
        
        # String format parsed as assignment
        if isinstance(data, str) and '=' in data:
            var, value = data.split('=', 1)
            return SetStatement(
                variable=var.strip(),
                value=self._parse_expression(value.strip()),
                comment=comment
            )
        
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
        """Parse notify statement / 解析通知語句"""
        if isinstance(data, str):
            # Simple notification
            return NotifyStatement(
                channel='log',
                message=Literal(value=data, type='string'),
                comment=comment
            )
        
        if not isinstance(data, dict):
            # Handle None or other invalid types
            return NotifyStatement(
                channel='log',
                message=Literal(value='', type='string'),
                comment=comment
            )
        
        channel = data.get('channel', data.get('to', 'log'))
        message = self._parse_expression(data.get('message', data.get('text', '')))
        level = data.get('level', data.get('severity'))
        
        # Parse metadata
        metadata = {}
        for key, value in data.items():
            if key not in ['channel', 'to', 'message', 'text', 'level', 'severity']:
                metadata[key] = self._parse_expression(value)
        
        return NotifyStatement(
            channel=channel,
            message=message,
            level=level,
            metadata=metadata if metadata else None,
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
            # TAFL v1.1 Fix: Check for expressions with operators FIRST
            # This fixes ${variable + 1} being parsed as Variable instead of expression
            if any(op in expr for op in ['==', '!=', '<=', '>=', '<', '>', '+', '-', '*', '/', 
                                         ' and ', ' or ', 'not ']):
                return self._parse_complex_expression(expr)
            
            # Check for pure variable reference (entire string is ${...})
            # Only after checking for operators
            var_match = self.VARIABLE_PATTERN.fullmatch(expr)
            if var_match:
                var_path = var_match.group(1).split('.')
                return Variable(name=var_path[0], path=var_path[1:])
            
            # Check for function calls
            if '(' in expr and ')' in expr:
                return self._parse_function_call(expr)
            
            # Check if it's a number
            try:
                if '.' in expr:
                    return Literal(value=float(expr), type='number')
                else:
                    return Literal(value=int(expr), type='number')
            except ValueError:
                pass
            
            # Otherwise, it's a string literal
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
            
            # Handle binary operators (order matters: check longer operators first)
            for op in ['==', '!=', '<=', '>=', ' and ', ' or ', '<', '>', '+', '-', '*', '/']:
                if op in inner_expr:
                    parts = inner_expr.split(op, 1)
                    if len(parts) == 2:
                        left_part = parts[0].strip()
                        right_part = parts[1].strip()
                        
                        # Parse left operand
                        if self._is_simple_variable(left_part):
                            left = Variable(name=left_part, path=[])
                        else:
                            left = self._parse_expression(left_part)
                        
                        # Parse right operand  
                        if self._is_simple_variable(right_part):
                            right = Variable(name=right_part, path=[])
                        else:
                            right = self._parse_expression(right_part)
                        
                        return BinaryOp(
                            operator=op.strip(),
                            left=left,
                            right=right
                        )
            
            # If no operators found in ${...}, treat as variable
            if self._is_simple_variable(inner_expr):
                return Variable(name=inner_expr, path=[])
            else:
                return self._parse_expression(inner_expr)
        
        # Handle expressions without ${...} wrapper
        # Handle 'not' operator
        if expr.startswith('not '):
            return UnaryOp(operator='not', operand=self._parse_expression(expr[4:]))
        
        # Handle binary operators (simplified)
        for op in ['==', '!=', '<=', '>=', '<', '>', ' and ', ' or ', '+', '-', '*', '/']:
            if op in expr:
                parts = expr.split(op, 1)
                if len(parts) == 2:
                    return BinaryOp(
                        operator=op.strip(),
                        left=self._parse_expression(parts[0].strip()),
                        right=self._parse_expression(parts[1].strip())
                    )
        
        # If no operators found, treat as string
        return Literal(value=expr, type='string')
    
    def _is_simple_variable(self, name: str) -> bool:
        """Check if a string is a simple variable name"""
        import re
        return bool(re.match(r'^[a-zA-Z_][a-zA-Z0-9_]*$', name))
    
    def _parse_function_call(self, expr: str) -> Expression:
        """
        Parse function call expression
        解析函數調用表達式
        """
        match = re.match(r'(\w+)\((.*)\)', expr.strip())
        if match:
            func_name = match.group(1)
            args_str = match.group(2)
            
            # Parse arguments (simplified)
            args = []
            if args_str:
                # Simple split by comma (doesn't handle nested commas)
                for arg in args_str.split(','):
                    args.append(self._parse_expression(arg.strip()))
            
            return FunctionCall(name=func_name, arguments=args)
        
        return Literal(value=expr, type='string')
    
    def _parse_preload_statements(self, preload_data: List[Any]) -> List[PreloadStatement]:
        """Parse preload statements / 解析預載入語句"""
        result = []
        for stmt_data in preload_data:
            if isinstance(stmt_data, dict) and 'query' in stmt_data:
                query_data = stmt_data['query']
                comment = stmt_data.get('comment')
                
                # Parse query fields
                target = query_data.get('target', '')
                store_as = query_data.get('store_as', '')
                
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
                    store_as=store_as,
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
                else:
                    # Configuration rule with static value
                    rule = RuleDefinition(value=rule_data)
            else:
                # Simple configuration value
                rule = RuleDefinition(value=rule_data)
            
            result[rule_name] = rule
        
        return result