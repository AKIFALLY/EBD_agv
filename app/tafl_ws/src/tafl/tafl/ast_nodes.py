"""
TAFL Abstract Syntax Tree Node Definitions
定義 TAFL 語言的抽象語法樹節點

This module defines the AST node classes used to represent TAFL programs
in memory after parsing.
"""

from typing import Any, Dict, List, Optional, Union, Tuple
from dataclasses import dataclass, field
from enum import Enum


class VerbType(Enum):
    """TAFL core verb types / TAFL 核心動詞類型"""
    QUERY = "query"
    CHECK = "check"
    CREATE = "create"
    UPDATE = "update"
    IF = "if"
    FOR = "for"
    SWITCH = "switch"
    SET = "set"
    STOP = "stop"
    NOTIFY = "notify"
    LOG = "log"  # Added for debugging and execution tracking


@dataclass
class ASTNode:
    """Base class for all AST nodes / 所有 AST 節點的基類"""
    line_number: int = 0
    column_number: int = 0
    source_file: str = ""


@dataclass
class Expression(ASTNode):
    """Base class for expressions / 表達式基類"""
    pass


@dataclass
class Literal(Expression):
    """Literal value expression / 字面值表達式"""
    value: Any = None
    type: str = ''  # 'string', 'number', 'boolean', 'null'


@dataclass
class Variable(Expression):
    """Variable reference expression / 變數引用表達式"""
    name: str = ''
    path: List[str] = field(default_factory=list)  # For nested access like rack.status


@dataclass
class BinaryOp(Expression):
    """Binary operation expression / 二元運算表達式"""
    operator: str = ''  # '+', '-', '*', '/', '==', '!=', '<', '>', '<=', '>=', 'and', 'or'
    left: Expression = None
    right: Expression = None


@dataclass
class UnaryOp(Expression):
    """Unary operation expression / 一元運算表達式"""
    operator: str = ''  # 'not', '-'
    operand: Expression = None


@dataclass
class FunctionCall(Expression):
    """Function call expression / 函數調用表達式"""
    name: str = ''
    arguments: List[Expression] = field(default_factory=list)


@dataclass
class ArrayExpression(Expression):
    """Array literal expression / 陣列字面值表達式"""
    elements: List[Expression] = field(default_factory=list)


@dataclass
class DictExpression(Expression):
    """Dictionary literal expression / 字典字面值表達式"""
    pairs: List[Tuple[str, Expression]] = field(default_factory=list)


@dataclass
class IndexAccess(Expression):
    """Array/Dictionary index access expression / 陣列/字典索引存取表達式"""
    object: Expression = None  # The object being indexed
    index: Expression = None   # The index expression


@dataclass
class Statement(ASTNode):
    """Base class for statements / 語句基類"""
    comment: Optional[str] = None  # Optional comment for the statement


@dataclass
class QueryStatement(Statement):
    """Query statement node / 查詢語句節點"""
    target: str = ''  # What to query (racks, locations, tasks, etc.)
    filters: Optional[Dict[str, Expression]] = None
    limit: Optional[Expression] = None  # Limit number of results
    as_: Optional[str] = None  # Variable to store result (使用 as_ 避免 Python 關鍵字)


@dataclass
class CheckStatement(Statement):
    """Check statement node / 檢查語句節點"""
    target: str = ''  # What to check
    condition: Expression = None
    as_: Optional[str] = None  # Variable to store result (使用 as_ 避免 Python 關鍵字)


@dataclass 
class CreateStatement(Statement):
    """Create statement node / 創建語句節點 - 符合 TAFL v1.1.1 規格"""
    target: str = ''  # What to create (task, alarm, etc.)
    with_: Dict[str, Expression] = field(default_factory=dict)  # Parameters (使用 with_ 避免 Python 關鍵字)
    as_: Optional[str] = None  # Variable to store result (使用 as_ 避免 Python 關鍵字)


@dataclass
class UpdateStatement(Statement):
    """Update statement node / 更新語句節點 - 符合 TAFL v1.1.1 規格"""
    target: str = ''  # What to update
    where: Dict[str, Expression] = field(default_factory=dict)  # Where conditions (符合規格書)
    set: Dict[str, Expression] = field(default_factory=dict)  # Set changes (符合規格書)


@dataclass
class IfStatement(Statement):
    """If statement node / 條件語句節點"""
    condition: Expression = None
    then_branch: List[Statement] = field(default_factory=list)
    else_branch: Optional[List[Statement]] = None


@dataclass
class ForStatement(Statement):
    """For loop statement node / 循環語句節點 - 符合 TAFL v1.1.1 規格"""
    in_: Expression = None  # What to iterate over (使用 in_ 避免 Python 關鍵字)
    as_: str = ''  # Loop variable name (使用 as_ 避免 Python 關鍵字)
    do: List[Statement] = field(default_factory=list)  # Loop body
    filter: Optional[Expression] = None  # Optional filter condition (v1.1)


@dataclass
class SwitchCase:
    """Single case in a switch statement / switch 語句中的單個 case - 符合 TAFL v1.1.1 規格"""
    when: Expression = None  # Case condition (符合規格書)
    do: List[Statement] = field(default_factory=list)  # Case body (符合規格書)


@dataclass
class SwitchStatement(Statement):
    """Switch statement node / 分支語句節點 - TAFL v1.1.2 format"""
    expression: Expression = None
    cases: List[SwitchCase] = field(default_factory=list)
    # v1.1.2: No separate default property - use when:"default" in cases


@dataclass
class SetStatement(Statement):
    """Set variable statement node / 設置變數語句節點"""
    variable: str = ''
    value: Expression = None


@dataclass
class StopStatement(Statement):
    """Stop flow statement node / 停止流程語句節點"""
    reason: Optional[str] = None
    condition: Optional[Expression] = None  # Optional condition for stop


@dataclass
class NotifyStatement(Statement):
    """Notify statement node / 通知語句節點 - 符合 TAFL v1.1.1 規格"""
    level: str = 'info'  # Notification level (info, warning, error, critical, alarm) - 主要欄位
    message: Expression = None
    recipients: Optional[List[str]] = None  # Optional recipients list
    details: Optional[Dict[str, Expression]] = None  # Optional additional details


@dataclass
class FlowMetadata:
    """Flow metadata information / 流程元數據資訊"""
    id: str = ''
    name: str = ''
    version: str = "1.1.2"
    author: Optional[str] = None
    enabled: bool = True  # v1.1.2: Controls whether flow is auto-executed
    description: Optional[str] = None
    tags: List[str] = field(default_factory=list)
    created_at: Optional[str] = None
    updated_at: Optional[str] = None


@dataclass
class FlowSettings:
    """Flow execution settings / 流程執行設置"""
    timeout: int = 300  # seconds
    retry_on_failure: bool = False
    max_retries: int = 3
    parallel_execution: bool = False
    log_level: str = "INFO"
    checkpoint: bool = True


@dataclass
class RuleDefinition(ASTNode):
    """Rule definition node / 規則定義節點"""
    condition: Optional[Expression] = None  # Boolean condition expression
    value: Any = None  # Static value (for configuration rules)
    description: Optional[str] = None  # Optional rule description


@dataclass
class PreloadStatement(Statement):
    """Preload statement node / 預載語句節點"""
    target: str = ''  # Target to query
    filters: Optional[Dict[str, Expression]] = None  # Query conditions (where clause)
    limit: Optional[int] = None  # Result limit
    as_: str = ''  # Required: variable name to store results (使用 as_ 避免 Python 關鍵字)


@dataclass
class TAFLProgram(ASTNode):
    """Root node representing a complete TAFL program / 代表完整 TAFL 程序的根節點"""
    metadata: FlowMetadata = None
    settings: FlowSettings = None
    preload: List[PreloadStatement] = field(default_factory=list)  # Data preloading statements
    rules: Dict[str, RuleDefinition] = field(default_factory=dict)  # Business rules
    variables: Dict[str, Any] = field(default_factory=dict)  # Initial variables
    flow: List[Statement] = field(default_factory=list)  # Main flow statements
    
    def __repr__(self):
        return f"TAFLProgram(id='{self.metadata.id if self.metadata else 'unknown'}', preload={len(self.preload)}, rules={len(self.rules)}, statements={len(self.flow)})"