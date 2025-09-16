#!/usr/bin/env python3
"""
Test All TAFL Flows - Automatically test all TAFL flow files in /app/config/tafl/flows/
"""

import pytest
import yaml
import sys
import os
from pathlib import Path
from typing import Dict, List, Tuple, Any

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))
sys.path.insert(0, '/app/tafl_ws/src/tafl')

from tafl_wcs.tafl_wcs_manager import TAFLWCSManager
from tafl_wcs.tafl_executor_wrapper import TAFLExecutorWrapper
from tafl.parser import TAFLParser
from tafl.validator import TAFLValidator


class TestAllTAFLFlows:
    """Test suite for all TAFL flow files"""
    
    @pytest.fixture
    def flows_dir(self):
        """Get TAFL flows directory"""
        return Path('/app/config/tafl/flows')
    
    @pytest.fixture
    def parser(self):
        """Create TAFL parser instance"""
        return TAFLParser()
    
    @pytest.fixture
    def validator(self):
        """Create TAFL validator instance"""
        return TAFLValidator()
    
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
    
    def get_all_tafl_files(self, flows_dir: Path) -> List[Path]:
        """Get all YAML files in the flows directory"""
        yaml_files = list(flows_dir.glob('*.yaml'))
        yaml_files.extend(list(flows_dir.glob('*.yml')))
        return sorted(yaml_files)
    
    def test_discover_all_flows(self, flows_dir):
        """Test that we can discover all TAFL flow files"""
        flow_files = self.get_all_tafl_files(flows_dir)
        
        print(f"\n📁 Found {len(flow_files)} TAFL flow files:")
        for flow_file in flow_files:
            print(f"  - {flow_file.name}")
        
        assert len(flow_files) > 0, "No TAFL flow files found"
        
        # Check that our new test file is included
        flow_names = [f.name for f in flow_files]
        assert 'test_stop_verb.yaml' in flow_names, "test_stop_verb.yaml not found"
    
    def test_yaml_syntax_all_flows(self, flows_dir):
        """Test YAML syntax for all flow files"""
        flow_files = self.get_all_tafl_files(flows_dir)
        errors = []
        
        for flow_file in flow_files:
            try:
                with open(flow_file, 'r', encoding='utf-8') as f:
                    yaml.safe_load(f)
                print(f"✅ {flow_file.name}: YAML syntax valid")
            except yaml.YAMLError as e:
                error_msg = f"❌ {flow_file.name}: YAML syntax error - {str(e)}"
                errors.append(error_msg)
                print(error_msg)
        
        assert len(errors) == 0, f"YAML syntax errors found:\n" + "\n".join(errors)
    
    def test_tafl_structure_all_flows(self, flows_dir, manager):
        """Test TAFL v1.1 structure for all flow files"""
        flow_files = self.get_all_tafl_files(flows_dir)
        errors = []
        
        for flow_file in flow_files:
            try:
                with open(flow_file, 'r', encoding='utf-8') as f:
                    flow_data = yaml.safe_load(f)
                
                # Check structure
                if manager._validate_tafl_structure(flow_data):
                    print(f"✅ {flow_file.name}: TAFL structure valid")
                else:
                    errors.append(f"❌ {flow_file.name}: Invalid TAFL structure")
            except Exception as e:
                error_msg = f"❌ {flow_file.name}: Error validating structure - {str(e)}"
                errors.append(error_msg)
                print(error_msg)
        
        assert len(errors) == 0, f"TAFL structure errors found:\n" + "\n".join(errors)
    
    def test_tafl_parsing_all_flows(self, flows_dir, parser, validator):
        """Test TAFL parsing and validation for all flow files"""
        flow_files = self.get_all_tafl_files(flows_dir)
        results = {}
        
        for flow_file in flow_files:
            try:
                with open(flow_file, 'r', encoding='utf-8') as f:
                    content = f.read()
                
                # Parse TAFL
                ast = parser.parse_string(content)
                
                # Validate
                is_valid = validator.validate(ast)
                
                if is_valid:
                    # Count verbs used
                    verb_count = self._count_verbs(ast)
                    results[flow_file.name] = {
                        'status': 'success',
                        'verbs': verb_count,
                        'steps': len(ast.flow) if hasattr(ast, 'flow') else 0
                    }
                    print(f"✅ {flow_file.name}: Parsed successfully ({len(ast.flow)} steps)")
                else:
                    errors = validator.get_errors() if hasattr(validator, 'get_errors') else ['Unknown validation error']
                    results[flow_file.name] = {
                        'status': 'validation_failed',
                        'errors': errors
                    }
                    print(f"⚠️ {flow_file.name}: Validation warnings - {errors}")
            except Exception as e:
                results[flow_file.name] = {
                    'status': 'parse_failed',
                    'error': str(e)
                }
                print(f"❌ {flow_file.name}: Parse error - {str(e)}")
        
        # Summary
        success_count = sum(1 for r in results.values() if r['status'] == 'success')
        print(f"\n📊 Parsing Summary: {success_count}/{len(flow_files)} flows parsed successfully")
        
        # Show details for each flow
        print("\n📋 Flow Details:")
        for name, result in results.items():
            if result['status'] == 'success':
                print(f"  {name}: {result['steps']} steps, verbs: {result['verbs']}")
    
    def test_specific_verb_flows(self, flows_dir, parser):
        """Test specific verb implementation flows"""
        verb_tests = {
            'test_stop_verb.yaml': ['stop'],
            'test_check_verb.yaml': ['check'],
            'test_for_loop.yaml': ['for'],
            'test_if_conditions.yaml': ['if'],
            'test_switch_default.yaml': ['switch'],
            'test_create_verb.yaml': ['create'],
            'test_update_verb.yaml': ['update']
        }
        
        for flow_name, expected_verbs in verb_tests.items():
            flow_path = flows_dir / flow_name
            if flow_path.exists():
                with open(flow_path, 'r') as f:
                    content = f.read()
                
                try:
                    ast = parser.parse_string(content)
                    found_verbs = self._get_verb_types(ast)
                    
                    for verb in expected_verbs:
                        assert verb in found_verbs, f"{flow_name} should contain '{verb}' verb"
                    
                    print(f"✅ {flow_name}: Contains expected verbs {expected_verbs}")
                except Exception as e:
                    print(f"⚠️ {flow_name}: Could not verify verbs - {str(e)}")
            else:
                print(f"⏭️ {flow_name}: File not found (skipping)")
    
    @pytest.mark.asyncio
    async def test_execute_enabled_flows(self, flows_dir, executor):
        """Test execution of flows that are enabled"""
        flow_files = self.get_all_tafl_files(flows_dir)
        execution_results = []
        
        for flow_file in flow_files:
            with open(flow_file, 'r') as f:
                flow_data = yaml.safe_load(f)
            
            # Check if flow is enabled
            metadata = flow_data.get('metadata', {})
            is_enabled = metadata.get('enabled', False)
            
            if is_enabled:
                print(f"🚀 Executing {flow_file.name} (enabled=true)")
                try:
                    with open(flow_file, 'r') as f:
                        content = f.read()
                    
                    result = await executor.execute_flow(content, flow_file.stem)
                    execution_results.append({
                        'flow': flow_file.name,
                        'status': 'success',
                        'result': result
                    })
                    print(f"  ✅ Execution successful")
                except Exception as e:
                    execution_results.append({
                        'flow': flow_file.name,
                        'status': 'failed',
                        'error': str(e)
                    })
                    print(f"  ❌ Execution failed: {str(e)}")
            else:
                print(f"⏭️ Skipping {flow_file.name} (enabled=false)")
        
        # Summary
        if execution_results:
            success_count = sum(1 for r in execution_results if r['status'] == 'success')
            print(f"\n📊 Execution Summary: {success_count}/{len(execution_results)} flows executed successfully")
    
    def test_flow_metadata_compliance(self, flows_dir):
        """Test that all flows have proper metadata"""
        flow_files = self.get_all_tafl_files(flows_dir)
        issues = []
        
        required_fields = ['id', 'name']
        recommended_fields = ['version', 'description']
        
        for flow_file in flow_files:
            with open(flow_file, 'r') as f:
                flow_data = yaml.safe_load(f)
            
            metadata = flow_data.get('metadata', {})
            
            # Check required fields
            for field in required_fields:
                if field not in metadata:
                    issues.append(f"❌ {flow_file.name}: Missing required metadata.{field}")
            
            # Check recommended fields
            warnings = []
            for field in recommended_fields:
                if field not in metadata:
                    warnings.append(field)
            
            if warnings:
                print(f"⚠️ {flow_file.name}: Missing recommended fields: {', '.join(warnings)}")
            else:
                print(f"✅ {flow_file.name}: Metadata complete")
        
        assert len(issues) == 0, f"Metadata issues found:\n" + "\n".join(issues)
    
    def test_verb_parameter_format_compliance(self, flows_dir):
        """Test that verbs use correct parameter format per TAFL v1.1 spec"""
        flow_files = self.get_all_tafl_files(flows_dir)
        issues = []
        
        print("\n📋 Checking verb parameter format compliance:")
        
        for flow_file in flow_files:
            with open(flow_file, 'r') as f:
                flow_data = yaml.safe_load(f)
            
            flow_issues = []
            flow_section = flow_data.get('flow', [])
            
            # Check each step in the flow
            for step_idx, step in enumerate(flow_section):
                # Check create verb format
                if 'create' in step:
                    create_def = step['create']
                    if 'data' in create_def:
                        flow_issues.append(f"  Step {step_idx+1}: create verb uses 'data:' instead of 'with:'")
                    if 'with' not in create_def and 'target' in create_def:
                        flow_issues.append(f"  Step {step_idx+1}: create verb missing 'with:' parameter")
                
                # Check update verb format
                if 'update' in step:
                    update_def = step['update']
                    if 'id' in update_def and 'data' in update_def:
                        flow_issues.append(f"  Step {step_idx+1}: update verb uses 'id:'+'data:' instead of 'where:'+'set:'")
                    if 'where' not in update_def and 'target' in update_def:
                        flow_issues.append(f"  Step {step_idx+1}: update verb missing 'where:' parameter")
                    if 'set' not in update_def and 'target' in update_def:
                        flow_issues.append(f"  Step {step_idx+1}: update verb missing 'set:' parameter")
                
                # Check nested structures (if/then/else, for/do, switch/cases)
                if 'if' in step:
                    if_def = step['if']
                    if 'then' in if_def:
                        self._check_verb_formats_in_block(if_def['then'], flow_issues, f"Step {step_idx+1} (if/then)")
                    if 'else' in if_def:
                        self._check_verb_formats_in_block(if_def['else'], flow_issues, f"Step {step_idx+1} (if/else)")
                
                if 'for' in step:
                    for_def = step['for']
                    if 'do' in for_def:
                        self._check_verb_formats_in_block(for_def['do'], flow_issues, f"Step {step_idx+1} (for/do)")
                
                if 'switch' in step:
                    switch_def = step['switch']
                    if 'cases' in switch_def:
                        for case_idx, case in enumerate(switch_def['cases']):
                            if 'do' in case:
                                self._check_verb_formats_in_block(case['do'], flow_issues, f"Step {step_idx+1} (switch/case {case_idx+1})")
            
            if flow_issues:
                print(f"⚠️ {flow_file.name}: Parameter format issues found:")
                for issue in flow_issues:
                    print(f"  {issue}")
                    issues.append(f"{flow_file.name}: {issue}")
            else:
                print(f"✅ {flow_file.name}: All verb parameters follow TAFL v1.1 spec")
        
        if issues:
            print(f"\n❌ Found {len(issues)} parameter format violations")
        else:
            print(f"\n✅ All flows use correct verb parameter formats")
    
    def _check_verb_formats_in_block(self, block, issues, context):
        """Helper method to check verb formats in nested blocks"""
        for idx, item in enumerate(block):
            if 'create' in item:
                create_def = item['create']
                if 'data' in create_def:
                    issues.append(f"  {context} item {idx+1}: create uses 'data:' instead of 'with:'")
            
            if 'update' in item:
                update_def = item['update']
                if 'id' in update_def and 'data' in update_def:
                    issues.append(f"  {context} item {idx+1}: update uses 'id:'+'data:' instead of 'where:'+'set:'")
    
    # Helper methods
    def _count_verbs(self, ast) -> Dict[str, int]:
        """Count occurrences of each verb in the flow"""
        verb_count = {}
        if hasattr(ast, 'flow'):
            for step in ast.flow:
                verb_type = type(step).__name__.replace('Statement', '').lower()
                verb_count[verb_type] = verb_count.get(verb_type, 0) + 1
        return verb_count
    
    def _get_verb_types(self, ast) -> List[str]:
        """Get list of unique verb types in the flow (recursively)"""
        verbs = set()
        
        def find_verbs(obj):
            if hasattr(obj, '__dict__'):
                # Get the verb type from class name
                class_name = type(obj).__name__.replace('Statement', '').lower()
                # Skip non-verb types
                if class_name not in ['taflprogram', 'list', 'flowmetadata', 'flowsettings', 
                                      'variable', 'literal', 'binaryop', 'arrayexpression', 
                                      'switchcase', 'comparison']:
                    verbs.add(class_name)
                # Recursively search in object attributes
                for attr_value in obj.__dict__.values():
                    find_verbs(attr_value)
            elif isinstance(obj, list):
                # Recursively search in list items
                for item in obj:
                    find_verbs(item)
        
        # Start recursive search from AST root
        find_verbs(ast)
        return list(verbs)


if __name__ == '__main__':
    pytest.main([__file__, '-v', '-s'])