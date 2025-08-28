#!/usr/bin/env python3
"""
Test TAFL Validator
測試 TAFL 驗證器
"""

import unittest
import sys
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from tafl.parser import TAFLParser
from tafl.validator import TAFLValidator
from tafl.ast_nodes import *


class TestTAFLValidator(unittest.TestCase):
    """Test TAFL Validator functionality"""
    
    def setUp(self):
        self.parser = TAFLParser()
        self.validator = TAFLValidator()
    
    def test_valid_program(self):
        """Test validation of a valid program"""
        yaml_content = """
metadata:
  id: test_flow
  name: Test Flow
variables:
  counter: 0
flow:
  - set:
      counter: 10
  - if:
      condition: "${counter} > 5"
      then:
        - set:
            result: "large"
        """
        program = self.parser.parse_string(yaml_content)
        is_valid = self.validator.validate(program)
        
        self.assertTrue(is_valid)
        self.assertEqual(len(self.validator.get_errors()), 0)
    
    def test_undefined_variable_warning(self):
        """Test warning for undefined variables"""
        yaml_content = """
flow:
  - set:
      result: "${undefined_var}"
        """
        program = self.parser.parse_string(yaml_content)
        is_valid = self.validator.validate(program)
        
        # Should still be valid but with warnings
        self.assertTrue(is_valid)
        warnings = self.validator.get_warnings()
        self.assertTrue(any('undefined_var' in w for w in warnings))
    
    def test_unused_variable_warning(self):
        """Test warning for unused variables"""
        yaml_content = """
variables:
  unused: "value"
  used: "value"
flow:
  - set:
      result: "${used}"
        """
        program = self.parser.parse_string(yaml_content)
        is_valid = self.validator.validate(program)
        
        self.assertTrue(is_valid)
        warnings = self.validator.get_warnings()
        self.assertTrue(any('unused' in w for w in warnings))
    
    def test_for_loop_variable_scope(self):
        """Test for loop variable is properly scoped"""
        yaml_content = """
variables:
  items: [1, 2, 3]
flow:
  - for:
      each: loop_var
      in: "${items}"
      do:
        - set:
            doubled: "${loop_var} * 2"
  - set:
      result: "${loop_var}"  # This should warn about undefined variable
        """
        program = self.parser.parse_string(yaml_content)
        is_valid = self.validator.validate(program)
        
        self.assertTrue(is_valid)
        warnings = self.validator.get_warnings()
        # Should warn that 'loop_var' is undefined outside the loop
        self.assertTrue(any('loop_var' in w for w in warnings))
    
    def test_nested_for_loops(self):
        """Test nested for loops don't cause conflicts"""
        yaml_content = """
variables:
  matrix:
    - [1, 2]
    - [3, 4]
flow:
  - for:
      each: row
      in: "${matrix}"
      do:
        - for:
            each: item
            in: "${row}"
            do:
              - set:
                  value: "${item}"
        """
        program = self.parser.parse_string(yaml_content)
        is_valid = self.validator.validate(program)
        
        self.assertTrue(is_valid)
        self.assertEqual(len(self.validator.get_errors()), 0)
    
    def test_switch_case_validation(self):
        """Test switch statement validation"""
        yaml_content = """
variables:
  value: 1
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
        is_valid = self.validator.validate(program)
        
        self.assertTrue(is_valid)
        self.assertEqual(len(self.validator.get_errors()), 0)
    
    def test_empty_flow_warning(self):
        """Test warning for empty flow"""
        yaml_content = """
metadata:
  id: empty_flow
  name: Empty Flow
flow: []
        """
        program = self.parser.parse_string(yaml_content)
        is_valid = self.validator.validate(program)
        
        self.assertTrue(is_valid)
        warnings = self.validator.get_warnings()
        self.assertTrue(any('empty' in w.lower() for w in warnings))
    
    def test_circular_dependency_detection(self):
        """Test detection of potential circular dependencies"""
        yaml_content = """
variables:
  a: "${b}"
  b: "${a}"
flow:
  - set:
      result: "${a}"
        """
        program = self.parser.parse_string(yaml_content)
        is_valid = self.validator.validate(program)
        
        # Should warn about potential circular reference
        warnings = self.validator.get_warnings()
        self.assertTrue(len(warnings) > 0)
    
    def test_type_checking_in_operations(self):
        """Test type checking in binary operations"""
        yaml_content = """
variables:
  number: 10
  text: "hello"
flow:
  - set:
      result: "${number} + ${text}"  # Should warn about type mismatch
        """
        program = self.parser.parse_string(yaml_content)
        is_valid = self.validator.validate(program)
        
        # Should still be valid but with warnings
        self.assertTrue(is_valid)
        # Type checking warnings would be good to have
    
    def test_query_statement_validation(self):
        """Test query statement validation"""
        yaml_content = """
flow:
  - query:
      from: racks
      where:
        status: "ready"
      store_as: available_racks
  - set:
      first_rack: "${available_racks[0]}"
        """
        program = self.parser.parse_string(yaml_content)
        is_valid = self.validator.validate(program)
        
        self.assertTrue(is_valid)
    
    def test_create_statement_validation(self):
        """Test create statement validation"""
        yaml_content = """
flow:
  - create:
      target: task
      params:
        name: "Test Task"
        priority: 1
      store_as: new_task
  - set:
      task_id: "${new_task.id}"
        """
        program = self.parser.parse_string(yaml_content)
        is_valid = self.validator.validate(program)
        
        self.assertTrue(is_valid)
    
    def test_notify_statement_validation(self):
        """Test notify statement validation"""
        yaml_content = """
variables:
  message: "Test notification"
flow:
  - notify:
      channel: log
      message: "${message}"
      level: info
        """
        program = self.parser.parse_string(yaml_content)
        is_valid = self.validator.validate(program)
        
        self.assertTrue(is_valid)
        self.assertEqual(len(self.validator.get_errors()), 0)


if __name__ == '__main__':
    unittest.main()