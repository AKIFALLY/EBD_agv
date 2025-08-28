"""
TAFL (Task Automation Flow Language) Module
簡化的任務自動化流程語言實現

This module provides the core implementation of TAFL, a simplified YAML-based
domain-specific language for WCS/AGV task automation.

Core Components:
- parser: TAFL YAML parser and AST builder
- executor: TAFL runtime execution engine
- validator: TAFL syntax and semantic validator
"""

from .parser import TAFLParser
from .executor import TAFLExecutor
from .validator import TAFLValidator

__all__ = [
    'TAFLParser',
    'TAFLExecutor', 
    'TAFLValidator'
]

__version__ = '1.0.0'