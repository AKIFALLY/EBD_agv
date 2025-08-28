#!/usr/bin/env python3
"""
Test TAFL Executor
測試 TAFL 執行器
"""

import unittest
import asyncio
import sys
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from tafl.parser import TAFLParser
from tafl.executor import TAFLExecutor, TAFLExecutionContext, TAFLRuntimeError
from tafl.ast_nodes import *


class TestTAFLExecutor(unittest.TestCase):
    """Test TAFL Executor functionality"""
    
    def setUp(self):
        self.parser = TAFLParser()
        self.executor = TAFLExecutor()
    
    def test_variable_assignment(self):
        """Test variable assignment and retrieval"""
        yaml_content = """
variables:
  initial: 5
flow:
  - set:
      counter: 10
  - set:
      message: "Hello"
        """
        program = self.parser.parse_string(yaml_content)
        result = asyncio.run(self.executor.execute(program))
        
        self.assertEqual(result['initial'], 5)
        self.assertEqual(result['counter'], 10)
        self.assertEqual(result['message'], 'Hello')
    
    def test_string_interpolation(self):
        """Test variable interpolation in strings"""
        yaml_content = """
variables:
  name: "World"
  count: 42
flow:
  - set:
      greeting: "Hello ${name}"
  - set:
      message: "Count is ${count}"
        """
        program = self.parser.parse_string(yaml_content)
        result = asyncio.run(self.executor.execute(program))
        
        self.assertEqual(result['greeting'], 'Hello World')
        self.assertEqual(result['message'], 'Count is 42')
    
    def test_if_statement_execution(self):
        """Test if statement conditional execution"""
        yaml_content = """
variables:
  value: 10
flow:
  - if:
      condition: "${value} > 5"
      then:
        - set:
            result: "large"
      else:
        - set:
            result: "small"
        """
        program = self.parser.parse_string(yaml_content)
        result = asyncio.run(self.executor.execute(program))
        
        self.assertEqual(result['result'], 'large')
        
        # Test with false condition
        yaml_content = """
variables:
  value: 3
flow:
  - if:
      condition: "${value} > 5"
      then:
        - set:
            result: "large"
      else:
        - set:
            result: "small"
        """
        program = self.parser.parse_string(yaml_content)
        result = asyncio.run(self.executor.execute(program))
        
        self.assertEqual(result['result'], 'small')
    
    def test_for_loop_execution(self):
        """Test for loop execution"""
        yaml_content = """
variables:
  items: [1, 2, 3]
  sum: 0
flow:
  - for:
      each: item
      in: "${items}"
      do:
        - set:
            sum: "${sum} + ${item}"
        """
        program = self.parser.parse_string(yaml_content)
        result = asyncio.run(self.executor.execute(program))
        
        self.assertEqual(result['sum'], 6)
    
    def test_for_loop_variable_scoping(self):
        """Test that loop variables don't leak to outer scope"""
        yaml_content = """
variables:
  items: [1, 2, 3]
  outer: "unchanged"
flow:
  - for:
      each: item
      in: "${items}"
      do:
        - set:
            inner: "${item}"
  - set:
      after_loop: "done"
        """
        program = self.parser.parse_string(yaml_content)
        result = asyncio.run(self.executor.execute(program))
        
        # Loop variable itself should not exist in outer scope
        self.assertNotIn('item', result)
        # Variables set in loop body persist (this is expected behavior)
        self.assertEqual(result['inner'], 3)  # Last value from loop
        self.assertEqual(result['outer'], 'unchanged')
        self.assertEqual(result['after_loop'], 'done')
    
    def test_nested_for_loops(self):
        """Test nested for loops with proper scoping"""
        yaml_content = """
variables:
  matrix:
    - row: [1, 2]
    - row: [3, 4]
  results: []
flow:
  - for:
      each: row_data
      in: "${matrix}"
      do:
        - for:
            each: value
            in: "${row_data.row}"
            do:
              - set:
                  last_value: "${value}"
        """
        program = self.parser.parse_string(yaml_content)
        result = asyncio.run(self.executor.execute(program))
        
        # Neither loop variable should exist in outer scope
        self.assertNotIn('row_data', result)
        self.assertNotIn('value', result)
    
    def test_switch_statement(self):
        """Test switch statement execution"""
        yaml_content = """
variables:
  value: 2
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
        3:
          - set:
              result: "three"
      default:
        - set:
            result: "other"
        """
        program = self.parser.parse_string(yaml_content)
        result = asyncio.run(self.executor.execute(program))
        
        self.assertEqual(result['result'], 'two')
        
        # Test default case
        yaml_content = """
variables:
  value: 99
flow:
  - switch:
      on: "${value}"
      cases:
        1:
          - set:
              result: "one"
      default:
        - set:
            result: "other"
        """
        program = self.parser.parse_string(yaml_content)
        result = asyncio.run(self.executor.execute(program))
        
        self.assertEqual(result['result'], 'other')
    
    def test_binary_operations(self):
        """Test binary operations in expressions"""
        yaml_content = """
variables:
  a: 10
  b: 5
flow:
  - set:
      sum: "${a} + ${b}"
  - set:
      diff: "${a} - ${b}"
  - set:
      product: "${a} * ${b}"
  - set:
      quotient: "${a} / ${b}"
  - set:
      is_greater: "${a} > ${b}"
  - set:
      is_equal: "${a} == ${b}"
        """
        program = self.parser.parse_string(yaml_content)
        result = asyncio.run(self.executor.execute(program))
        
        self.assertEqual(result['sum'], 15)
        self.assertEqual(result['diff'], 5)
        self.assertEqual(result['product'], 50)
        self.assertEqual(result['quotient'], 2.0)
        self.assertEqual(result['is_greater'], True)
        self.assertEqual(result['is_equal'], False)
    
    def test_logical_operations(self):
        """Test logical operations"""
        yaml_content = """
variables:
  a: true
  b: false
flow:
  - set:
      and_result: "${a} and ${b}"
  - set:
      or_result: "${a} or ${b}"
  - set:
      not_result: "not ${b}"
        """
        program = self.parser.parse_string(yaml_content)
        result = asyncio.run(self.executor.execute(program))
        
        self.assertEqual(result['and_result'], False)
        self.assertEqual(result['or_result'], True)
        self.assertEqual(result['not_result'], True)
    
    def test_stop_statement(self):
        """Test stop statement halts execution"""
        yaml_content = """
flow:
  - set:
      before: "executed"
  - stop:
      reason: "Test stop"
  - set:
      after: "should not execute"
        """
        program = self.parser.parse_string(yaml_content)
        result = asyncio.run(self.executor.execute(program))
        
        self.assertEqual(result['before'], 'executed')
        self.assertNotIn('after', result)
    
    def test_nested_property_access(self):
        """Test accessing nested properties"""
        yaml_content = """
variables:
  data:
    user:
      name: "John"
      age: 30
flow:
  - set:
      user_name: "${data.user.name}"
  - set:
      user_age: "${data.user.age}"
        """
        program = self.parser.parse_string(yaml_content)
        result = asyncio.run(self.executor.execute(program))
        
        self.assertEqual(result['user_name'], 'John')
        self.assertEqual(result['user_age'], 30)
    
    def test_array_iteration(self):
        """Test iterating over arrays"""
        yaml_content = """
variables:
  numbers: [1, 2, 3, 4, 5]
  doubled: []
flow:
  - for:
      each: num
      in: "${numbers}"
      do:
        - set:
            temp: "${num} * 2"
        """
        program = self.parser.parse_string(yaml_content)
        result = asyncio.run(self.executor.execute(program))
        
        # Verify loop variable doesn't leak, but set variables persist
        self.assertNotIn('num', result)
        # Variables set in loop persist with last value
        self.assertEqual(result['temp'], 10)  # Last iteration: 5 * 2
    
    @unittest.skip("External function integration test - requires flow_wcs integration")
    def test_external_functions(self):
        """Test calling external functions"""
        # Mock external function
        called_params = {}
        def mock_function(**kwargs):
            called_params.update(kwargs)
            return "success"
        
        executor = TAFLExecutor(function_registry={
            'test_func': mock_function
        })
        
        yaml_content = """
flow:
  - create:
      target: test_func
      params:
        param1: "value1"
        param2: 123
      store_as: result
        """
        program = self.parser.parse_string(yaml_content)
        result = asyncio.run(executor.execute(program))
        
        self.assertEqual(result['result'], 'success')
        self.assertEqual(called_params['param1'], 'value1')
        self.assertEqual(called_params['param2'], 123)


if __name__ == '__main__':
    unittest.main()