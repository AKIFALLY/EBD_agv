#!/usr/bin/env python3
"""
Test cases for Flow Executor
"""

import pytest
import asyncio
import yaml
from pathlib import Path
import sys
import os

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from flow_wcs.flow_executor import FlowExecutor


class TestFlowExecutor:
    """Test Flow Executor functionality"""
    
    @pytest.fixture
    def sample_flow(self):
        """Create a sample flow for testing"""
        return {
            'meta': {
                'system': 'linear_flow_v2',
                'version': '2.0.0'
            },
            'flow': {
                'id': 'test_flow',
                'name': 'Test Flow',
                'work_id': '999999',
                'enabled': True
            },
            'workflow': [
                {
                    'section': 'Initialize',
                    'steps': [
                        {
                            'id': 'set_variable',
                            'exec': 'action.log',
                            'params': {
                                'message': 'Starting test flow',
                                'level': 'info'
                            }
                        }
                    ]
                },
                {
                    'section': 'Query Data',
                    'steps': [
                        {
                            'id': 'query_locations',
                            'exec': 'query.locations',
                            'params': {
                                'type': 'room_inlet',
                                'has_rack': True
                            },
                            'store': 'locations'
                        }
                    ]
                }
            ],
            'variables': {}
        }
    
    @pytest.mark.asyncio
    async def test_executor_initialization(self, sample_flow):
        """Test executor initialization"""
        executor = FlowExecutor(sample_flow)
        
        assert executor.flow_data == sample_flow
        assert executor.context['flow_id'] == 'test_flow'
        assert executor.context['work_id'] == '999999'
        assert executor.context['status'] == 'initialized'
    
    @pytest.mark.asyncio
    async def test_execute_simple_flow(self, sample_flow):
        """Test executing a simple flow"""
        executor = FlowExecutor(sample_flow)
        context = await executor.execute()
        
        assert context['status'] == 'completed'
        assert len(context['logs']) > 0
        assert context['logs'][0]['message'] == 'Executing section: Initialize'
    
    @pytest.mark.asyncio
    async def test_variable_resolution(self):
        """Test variable resolution in flow"""
        flow = {
            'meta': {'system': 'linear_flow_v2', 'version': '2.0.0'},
            'flow': {'id': 'var_test', 'name': 'Variable Test'},
            'workflow': [
                {
                    'section': 'Test Variables',
                    'steps': [
                        {
                            'id': 'set_var',
                            'exec': 'query.locations',
                            'params': {'type': 'test'},
                            'store': 'test_locations'
                        },
                        {
                            'id': 'use_var',
                            'exec': 'action.log',
                            'params': {
                                'message': 'Found ${test_locations} locations'
                            }
                        }
                    ]
                }
            ]
        }
        
        executor = FlowExecutor(flow)
        context = await executor.execute()
        
        assert 'test_locations' in context['variables']
        assert context['status'] == 'completed'
    
    @pytest.mark.asyncio
    async def test_conditional_execution(self):
        """Test conditional step execution"""
        flow = {
            'meta': {'system': 'linear_flow_v2', 'version': '2.0.0'},
            'flow': {'id': 'cond_test', 'name': 'Conditional Test'},
            'workflow': [
                {
                    'section': 'Conditional Steps',
                    'steps': [
                        {
                            'id': 'always_run',
                            'exec': 'action.log',
                            'params': {'message': 'This always runs'}
                        },
                        {
                            'id': 'skip_this',
                            'exec': 'action.log',
                            'params': {'message': 'This should be skipped'},
                            'skip_if': 'true'
                        },
                        {
                            'id': 'run_this',
                            'exec': 'action.log',
                            'params': {'message': 'This should run'},
                            'skip_if': 'false'
                        }
                    ]
                }
            ]
        }
        
        executor = FlowExecutor(flow)
        context = await executor.execute()
        
        # Check that the right messages were logged
        messages = [log['message'] for log in context['logs'] if 'This' in log['message']]
        
        assert 'This always runs' in str(messages)
        assert 'This should be skipped' not in str(messages)
        assert 'This should run' in str(messages)
    
    @pytest.mark.asyncio
    async def test_foreach_loop(self):
        """Test foreach loop execution"""
        flow = {
            'meta': {'system': 'linear_flow_v2', 'version': '2.0.0'},
            'flow': {'id': 'foreach_test', 'name': 'ForEach Test'},
            'workflow': [
                {
                    'section': 'Loop Test',
                    'steps': [
                        {
                            'id': 'get_items',
                            'exec': 'query.locations',
                            'params': {'type': 'room_inlet'},
                            'store': 'items'
                        },
                        {
                            'id': 'process_items',
                            'exec': 'foreach',
                            'items': '${items}',
                            'var': 'item',
                            'steps': [
                                {
                                    'id': 'log_item',
                                    'exec': 'action.log',
                                    'params': {
                                        'message': 'Processing item: ${_item}'
                                    }
                                }
                            ]
                        }
                    ]
                }
            ]
        }
        
        executor = FlowExecutor(flow)
        context = await executor.execute()
        
        assert context['status'] == 'completed'
        # Check that foreach was executed
        assert any('Processing item' in log.get('message', '') for log in context['logs'])
    
    @pytest.mark.asyncio
    async def test_error_handling(self):
        """Test error handling in flow"""
        flow = {
            'meta': {'system': 'linear_flow_v2', 'version': '2.0.0'},
            'flow': {'id': 'error_test', 'name': 'Error Test'},
            'workflow': [
                {
                    'section': 'Error Section',
                    'steps': [
                        {
                            'id': 'invalid_step',
                            'exec': 'invalid.function',
                            'params': {}
                        }
                    ]
                }
            ]
        }
        
        executor = FlowExecutor(flow)
        context = await executor.execute()
        
        # Should complete even with unknown function
        assert context['status'] == 'completed'
        assert any('Unknown function' in log.get('message', '') for log in context['logs'])
    
    @pytest.mark.asyncio
    async def test_parallel_execution(self):
        """Test parallel branch execution"""
        flow = {
            'meta': {'system': 'linear_flow_v2', 'version': '2.0.0'},
            'flow': {'id': 'parallel_test', 'name': 'Parallel Test'},
            'workflow': [
                {
                    'section': 'Parallel Section',
                    'steps': [
                        {
                            'id': 'parallel_tasks',
                            'exec': 'parallel',
                            'branches': [
                                {
                                    'name': 'branch1',
                                    'steps': [
                                        {
                                            'id': 'branch1_step',
                                            'exec': 'action.log',
                                            'params': {'message': 'Branch 1 executed'}
                                        }
                                    ]
                                },
                                {
                                    'name': 'branch2',
                                    'steps': [
                                        {
                                            'id': 'branch2_step',
                                            'exec': 'action.log',
                                            'params': {'message': 'Branch 2 executed'}
                                        }
                                    ]
                                }
                            ]
                        }
                    ]
                }
            ]
        }
        
        executor = FlowExecutor(flow)
        context = await executor.execute()
        
        assert context['status'] == 'completed'
        # Check both branches executed
        messages = [log['message'] for log in context['logs']]
        assert any('Branch 1 executed' in msg for msg in messages)
        assert any('Branch 2 executed' in msg for msg in messages)


def test_flow_validation():
    """Test flow structure validation"""
    from flow_wcs.flow_wcs_node import FlowWCSNode
    
    # Valid flow
    valid_flow = {
        'meta': {'system': 'linear_flow_v2'},
        'flow': {'id': 'test'},
        'workflow': []
    }
    
    node = FlowWCSNode.__new__(FlowWCSNode)
    assert node.validate_flow(valid_flow) == True
    
    # Invalid flow - missing meta
    invalid_flow1 = {
        'flow': {'id': 'test'},
        'workflow': []
    }
    assert node.validate_flow(invalid_flow1) == False
    
    # Invalid flow - wrong system
    invalid_flow2 = {
        'meta': {'system': 'wrong_system'},
        'flow': {'id': 'test'},
        'workflow': []
    }
    assert node.validate_flow(invalid_flow2) == False
    
    # Invalid flow - missing flow id
    invalid_flow3 = {
        'meta': {'system': 'linear_flow_v2'},
        'flow': {},
        'workflow': []
    }
    assert node.validate_flow(invalid_flow3) == False


if __name__ == '__main__':
    pytest.main([__file__, '-v'])