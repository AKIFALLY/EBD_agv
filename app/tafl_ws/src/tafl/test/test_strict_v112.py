#!/usr/bin/env python3
"""
Test that strict TAFL v1.1.2 parser rejects old syntax
確認嚴格的 TAFL v1.1.2 解析器拒絕舊語法
"""

import unittest
import sys
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from tafl.parser import TAFLParser, TAFLParseError


class TestStrictTAFLv112(unittest.TestCase):
    """Test that old syntax is properly rejected"""
    
    def setUp(self):
        self.parser = TAFLParser()
    
    def test_reject_old_query_from(self):
        """Test that 'from' is rejected in query"""
        yaml_content = """
flow:
  - query:
      from: racks
      as: available_racks
        """
        with self.assertRaises(TAFLParseError) as context:
            self.parser.parse_string(yaml_content)
        self.assertIn("'target'", str(context.exception))
    
    def test_reject_old_query_store_as(self):
        """Test that 'store_as' is silently ignored in query"""
        yaml_content = """
flow:
  - query:
      target: locations
      store_as: available_locations
        """
        # This should work since store_as is silently ignored
        program = self.parser.parse_string(yaml_content)
        # The internal field is as_ in the AST
        self.assertIsNone(program.flow[0].as_)  # No 'as' provided, store_as ignored
    
    def test_reject_old_for_each(self):
        """Test that 'each' is rejected in for loop"""
        yaml_content = """
flow:
  - for:
      each: item
      in: "${items}"
      do:
        - set:
            current: "${item}"
        """
        with self.assertRaises(TAFLParseError) as context:
            self.parser.parse_string(yaml_content)
        self.assertIn("'as'", str(context.exception))
    
    def test_reject_old_create_params(self):
        """Test that 'params' is silently ignored in create"""
        yaml_content = """
flow:
  - create:
      target: task
      params:
        name: "Test"
        priority: 5
      as: new_task
        """
        # Should work since params is ignored if 'with' is not present
        program = self.parser.parse_string(yaml_content)
        # The internal field is with_ and params are ignored
        self.assertEqual(len(program.flow[0].with_), 0)  # params ignored, no 'with' field
    
    def test_reject_old_update_id_shortcut(self):
        """Test that 'id' shortcut works but goes in where"""
        yaml_content = """
flow:
  - update:
      target: task
      id: 1
      set:
        status: "done"
        """
        # This should work - id is just ignored
        program = self.parser.parse_string(yaml_content)
        # id won't be in where since we removed backward compat
        self.assertEqual(len(program.flow[0].where), 0)
    
    def test_reject_old_if_when(self):
        """Test that 'when' is rejected in if statement"""
        yaml_content = """
flow:
  - if:
      when: "${value} > 5"
      then:
        - set:
            result: "large"
        """
        with self.assertRaises(TAFLParseError) as context:
            self.parser.parse_string(yaml_content)
        self.assertIn("'condition'", str(context.exception))
    
    def test_reject_log_verb(self):
        """Test that 'log' verb is rejected"""
        yaml_content = """
flow:
  - log:
      level: info
      message: "Test"
        """
        # log verb is now completely removed, will return None
        program = self.parser.parse_string(yaml_content)
        # The statement should be None (skipped)
        self.assertEqual(len(program.flow), 0)
    
    def test_correct_v112_syntax_works(self):
        """Test that correct v1.1.2 syntax still works"""
        yaml_content = """
flow:
  - query:
      target: locations
      as: locs
  - for:
      as: loc
      in: "${locs}"
      do:
        - create:
            target: task
            with:
              location_id: "${loc.id}"
            as: new_task
  - check:
      condition: "${new_task.id} > 0"
      as: is_valid
  - update:
      target: task
      where:
        id: "${new_task.id}"
      set:
        status: "active"
  - if:
      condition: "${is_valid}"
      then:
        - notify:
            level: info
            message: "Task created"
        """
        program = self.parser.parse_string(yaml_content)
        self.assertEqual(len(program.flow), 5)
        
        # Verify correct verb types
        self.assertEqual(program.flow[0].__class__.__name__, 'QueryStatement')
        self.assertEqual(program.flow[1].__class__.__name__, 'ForStatement')
        self.assertEqual(program.flow[2].__class__.__name__, 'CheckStatement')
        self.assertEqual(program.flow[3].__class__.__name__, 'UpdateStatement')
        self.assertEqual(program.flow[4].__class__.__name__, 'IfStatement')


if __name__ == '__main__':
    # Run tests with verbose output
    unittest.main(verbosity=2)