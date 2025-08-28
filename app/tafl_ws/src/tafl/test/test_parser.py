#!/usr/bin/env python3
"""
Test TAFL Parser
測試 TAFL 解析器
"""

import unittest
import sys
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from tafl.parser import TAFLParser, TAFLParseError
from tafl.ast_nodes import *


class TestTAFLParser(unittest.TestCase):
    """Test TAFL Parser functionality"""
    
    def setUp(self):
        self.parser = TAFLParser()
    
    def test_parse_metadata(self):
        """Test metadata parsing"""
        yaml_content = """
metadata:
  id: test_flow
  name: Test Flow
  version: "1.0"
  author: Test Author
  description: Test Description
        """
        program = self.parser.parse_string(yaml_content)
        
        self.assertEqual(program.metadata.id, 'test_flow')
        self.assertEqual(program.metadata.name, 'Test Flow')
        self.assertEqual(program.metadata.version, '1.0')
        self.assertEqual(program.metadata.author, 'Test Author')
        self.assertEqual(program.metadata.description, 'Test Description')
    
    def test_parse_variables(self):
        """Test variable parsing"""
        yaml_content = """
variables:
  counter: 10
  name: "test"
  flag: true
  items: [1, 2, 3]
        """
        program = self.parser.parse_string(yaml_content)
        
        self.assertEqual(program.variables['counter'], 10)
        self.assertEqual(program.variables['name'], 'test')
        self.assertEqual(program.variables['flag'], True)
        self.assertEqual(program.variables['items'], [1, 2, 3])
    
    def test_parse_set_statement(self):
        """Test set statement parsing"""
        yaml_content = """
flow:
  - set:
      counter: 10
  - set:
      message: "Hello World"
        """
        program = self.parser.parse_string(yaml_content)
        
        self.assertEqual(len(program.flow), 2)
        self.assertIsInstance(program.flow[0], SetStatement)
        self.assertEqual(program.flow[0].variable, 'counter')
        self.assertIsInstance(program.flow[0].value, Literal)
        self.assertEqual(program.flow[0].value.value, 10)
    
    def test_parse_if_statement(self):
        """Test if statement parsing"""
        yaml_content = """
flow:
  - if:
      condition: "${counter} > 5"
      then:
        - set:
            result: "large"
      else:
        - set:
            result: "small"
        """
        program = self.parser.parse_string(yaml_content)
        
        self.assertEqual(len(program.flow), 1)
        self.assertIsInstance(program.flow[0], IfStatement)
        self.assertIsInstance(program.flow[0].condition, BinaryOp)
        self.assertEqual(len(program.flow[0].then_branch), 1)
        self.assertEqual(len(program.flow[0].else_branch), 1)
    
    def test_parse_for_statement(self):
        """Test for statement parsing"""
        yaml_content = """
flow:
  - for:
      each: item
      in: "${items}"
      do:
        - set:
            current: "${item}"
        """
        program = self.parser.parse_string(yaml_content)
        
        self.assertEqual(len(program.flow), 1)
        self.assertIsInstance(program.flow[0], ForStatement)
        self.assertEqual(program.flow[0].variable, 'item')
        self.assertIsInstance(program.flow[0].iterable, Variable)
        self.assertEqual(len(program.flow[0].body), 1)
    
    def test_parse_notify_statement(self):
        """Test notify statement parsing"""
        yaml_content = """
flow:
  - notify:
      channel: log
      message: "Test message"
      level: info
        """
        program = self.parser.parse_string(yaml_content)
        
        self.assertEqual(len(program.flow), 1)
        self.assertIsInstance(program.flow[0], NotifyStatement)
        self.assertEqual(program.flow[0].channel, 'log')
        self.assertIsInstance(program.flow[0].message, Literal)
        self.assertEqual(program.flow[0].level, 'info')
    
    def test_parse_expression_with_interpolation(self):
        """Test string with variable interpolation"""
        yaml_content = """
flow:
  - notify:
      channel: log
      message: "Counter is ${counter}"
        """
        program = self.parser.parse_string(yaml_content)
        
        stmt = program.flow[0]
        self.assertIsInstance(stmt.message, Literal)
        self.assertEqual(stmt.message.type, 'string')
        self.assertIn('${counter}', stmt.message.value)
    
    def test_parse_binary_operation(self):
        """Test binary operation parsing"""
        yaml_content = """
flow:
  - if:
      condition: "${a} + ${b} > 10"
      then:
        - set:
            result: true
        """
        program = self.parser.parse_string(yaml_content)
        
        condition = program.flow[0].condition
        self.assertIsInstance(condition, BinaryOp)
        self.assertEqual(condition.operator, '>')
        # Left side should be addition
        self.assertIsInstance(condition.left, BinaryOp)
        self.assertEqual(condition.left.operator, '+')
    
    def test_parse_switch_statement(self):
        """Test switch statement parsing"""
        yaml_content = """
flow:
  - switch:
      on: "${value}"
      cases:
        1:
          - set:
              result: "one"
        2:
          - set:
              result: "two"
      default:
        - set:
            result: "other"
        """
        program = self.parser.parse_string(yaml_content)
        
        self.assertEqual(len(program.flow), 1)
        self.assertIsInstance(program.flow[0], SwitchStatement)
        self.assertEqual(len(program.flow[0].cases), 2)
        self.assertIsNotNone(program.flow[0].default)
    
    def test_parse_query_statement(self):
        """Test query statement parsing"""
        yaml_content = """
flow:
  - query:
      from: racks
      where:
        status: "ready"
      store_as: available_racks
        """
        program = self.parser.parse_string(yaml_content)
        
        self.assertEqual(len(program.flow), 1)
        self.assertIsInstance(program.flow[0], QueryStatement)
        self.assertEqual(program.flow[0].target, 'racks')
        self.assertEqual(program.flow[0].store_as, 'available_racks')
        self.assertIsNotNone(program.flow[0].filters)
    
    def test_invalid_yaml(self):
        """Test invalid YAML raises error"""
        yaml_content = """
flow:
  - set: {invalid yaml
        """
        with self.assertRaises(TAFLParseError):
            self.parser.parse_string(yaml_content)


if __name__ == '__main__':
    unittest.main()