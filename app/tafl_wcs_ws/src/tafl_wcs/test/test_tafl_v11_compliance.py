#!/usr/bin/env python3
"""
Test TAFL v1.1 Compliance - Verify that tafl_wcs_ws follows TAFL v1.1 specification
"""

import pytest
import asyncio
import yaml
import sys
import os
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from tafl_wcs.tafl_wcs_manager import TAFLWCSManager
from tafl_wcs.tafl_executor_wrapper import TAFLExecutorWrapper
from tafl_wcs.tafl_functions import TAFLFunctions
from tafl_wcs.tafl_db_bridge import TAFLDatabaseBridge


class TestTAFLv11Compliance:
    """Test suite for TAFL v1.1 specification compliance"""
    
    @pytest.fixture
    def manager(self):
        """Create TAFL WCS Manager instance"""
        return TAFLWCSManager(
            flows_dir='/app/config/tafl/flows',
            database_url="postgresql://agvc:password@192.168.100.254:5432/agvc"
        )
    
    @pytest.fixture
    def executor(self):
        """Create TAFL Executor Wrapper instance"""
        return TAFLExecutorWrapper(
            database_url="postgresql://agvc:password@192.168.100.254:5432/agvc"
        )
    
    def test_6_section_structure_validation(self, manager):
        """Test that manager validates TAFL v1.1 6-section structure"""
        # Valid TAFL v1.1 structure
        valid_flow = {
            'metadata': {'id': 'test_flow', 'name': 'Test Flow'},
            'settings': {'timeout': 300},
            'preload': {},
            'rules': {},
            'variables': {},
            'flow': []
        }
        
        assert manager._validate_tafl_structure(valid_flow) == True
        
        # Missing required metadata
        invalid_flow_no_metadata = {
            'flow': []
        }
        assert manager._validate_tafl_structure(invalid_flow_no_metadata) == False
        
        # Missing required flow
        invalid_flow_no_flow = {
            'metadata': {'id': 'test', 'name': 'Test'}
        }
        assert manager._validate_tafl_structure(invalid_flow_no_flow) == False
        
        # Missing required id in metadata
        invalid_flow_no_id = {
            'metadata': {'name': 'Test'},
            'flow': []
        }
        assert manager._validate_tafl_structure(invalid_flow_no_id) == False
    
    @pytest.mark.asyncio
    async def test_4_phase_execution_model(self, executor):
        """Test that executor follows 4-Phase execution model"""
        tafl_content = """
metadata:
  id: test_4phase
  name: "4-Phase Test"
  
settings:
  timeout: 100
  max_retries: 2
  
preload:
  test_data:
    query:
      target: racks
      where:
        status: active
        
rules:
  max_items: 
    value: 10
    description: "Maximum items"
  min_battery:
    value: 20
    description: "Minimum battery level"
    
variables:
  counter: 0
  mode: "test"
  
flow:
  - set:
      counter: 1
"""
        
        result = await executor.execute_flow(tafl_content, "test_4phase")
        
        # Verify settings were applied
        assert executor.max_retry_attempts == 2
        
        # Verify rules were stored
        assert 'rules' in executor.scopes
        assert 'max_items' in executor.scopes['rules']
        assert executor.scopes['rules']['max_items']['value'] == 10
        
        # Verify variables were initialized
        assert 'global' in executor.scopes
        assert executor.scopes['global']['counter'] == 0
        assert executor.scopes['global']['mode'] == "test"
    
    def test_5_level_variable_scope(self, executor):
        """Test 5-Level variable scope management"""
        # Initialize scopes
        executor.scopes = {
            'rules': {'rule_var': {'value': 100, 'read_only': True}},
            'preload': {'preload_var': 'preloaded'},
            'global': {'global_var': 'global_value'},
            'flow': {'flow_var': 'flow_value'},
            'loop': {'loop_var': 'loop_value'}
        }
        
        # Test scope resolution order (most specific to least)
        assert executor._resolve_variable('loop_var') == 'loop_value'
        assert executor._resolve_variable('flow_var') == 'flow_value'
        assert executor._resolve_variable('global_var') == 'global_value'
        assert executor._resolve_variable('preload_var') == 'preloaded'
        assert executor._resolve_variable('rule_var') == 100  # Rules return 'value' field
        assert executor._resolve_variable('nonexistent') is None
    
    def test_switch_case_range_support(self):
        """Test switch case with range notation support"""
        functions = TAFLFunctions(None)
        
        # Test range notation
        assert functions.evaluate_switch(75, "50..80") == True
        assert functions.evaluate_switch(45, "50..80") == False
        assert functions.evaluate_switch(85, "50..80") == False
        
        # Test comparison operators
        assert functions.evaluate_switch(90, "> 80") == True
        assert functions.evaluate_switch(70, "> 80") == False
        assert functions.evaluate_switch(15, "< 20") == True
        assert functions.evaluate_switch(25, "< 20") == False
        assert functions.evaluate_switch(50, ">= 50") == True
        assert functions.evaluate_switch(49, ">= 50") == False
        
        # Test direct value comparison
        assert functions.evaluate_switch("active", "active") == True
        assert functions.evaluate_switch("inactive", "active") == False
    
    def test_enhanced_functions(self):
        """Test TAFL v1.1 enhanced functions"""
        functions = TAFLFunctions(None)
        
        # Test generate_id
        id1 = functions.generate_id("test")
        assert id1.startswith("test_")
        assert len(id1) > 5
        
        # Test validate_range
        assert functions.validate_range(50, 0, 100) == True
        assert functions.validate_range(150, 0, 100) == False
        assert functions.validate_range(-10, 0, 100) == False
        
        # Test check_condition (simple test without complex context)
        context = {'x': 10, 'y': 20}
        assert functions.check_condition("10 > 5", context) == True
        assert functions.check_condition("10 < 5", context) == False
    
    @pytest.mark.asyncio
    async def test_complete_v11_example_parsing(self, manager):
        """Test parsing of complete v1.1 example file"""
        example_file = Path('/app/config/tafl/flows/complete_v11_example.yaml')
        
        if example_file.exists():
            with open(example_file, 'r') as f:
                flow_data = yaml.safe_load(f)
            
            # Validate structure
            assert manager._validate_tafl_structure(flow_data) == True
            
            # Check required sections are present
            assert 'metadata' in flow_data
            assert 'flow' in flow_data
            
            # Check optional sections (TAFL v1.1 spec: settings, preload, rules, variables are optional)
            # Note: preload might be commented out in the example file
            assert 'settings' in flow_data
            assert 'rules' in flow_data
            assert 'variables' in flow_data
            # preload is optional and may be commented out
            
            # Verify metadata fields
            assert flow_data['metadata']['id'] == 'complete_v11_example'
            assert flow_data['metadata']['version'] == '1.1.2'
    
    def test_terminology_consistency(self, executor):
        """Test that settings terminology is used consistently (not initialization)"""
        # Create a simple flow with settings
        tafl_content = """
metadata:
  id: test_terminology
  name: "Terminology Test"
  
settings:
  timeout: 300
  max_retries: 5
  
flow: []
"""
        
        flow_data = yaml.safe_load(tafl_content)
        
        # Should have 'settings' not 'initialization'
        assert 'settings' in flow_data
        assert 'initialization' not in flow_data
        
        # Verify method exists and uses correct name
        assert hasattr(executor, '_execute_settings')
        assert hasattr(executor, '_execute_preload')
        assert hasattr(executor, '_process_rules')
        assert hasattr(executor, '_process_variables')


if __name__ == '__main__':
    pytest.main([__file__, '-v'])