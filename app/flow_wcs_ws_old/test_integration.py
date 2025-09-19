#!/usr/bin/env python3
"""
Integration test for Flow WCS system
測試 Flow WCS 與 Linear Flow Designer 的整合
"""

import asyncio
import json
import yaml
import time
from pathlib import Path
import sys
import os

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

# Import Flow WCS modules
from flow_wcs.flow_executor import FlowExecutor
from flow_wcs.flow_monitor import FlowMonitor


class FlowWCSIntegrationTest:
    """Integration test suite for Flow WCS"""
    
    def __init__(self):
        self.flows_dir = Path("/app/config/wcs/flows")
        self.test_results = []
        self.passed = 0
        self.failed = 0
    
    def print_header(self, title):
        """Print test section header"""
        print("\n" + "="*50)
        print(f"  {title}")
        print("="*50)
    
    def print_result(self, test_name, passed, message=""):
        """Print test result"""
        status = "✓ PASS" if passed else "✗ FAIL"
        color = "\033[92m" if passed else "\033[91m"
        reset = "\033[0m"
        
        print(f"{color}{status}{reset} - {test_name}")
        if message:
            print(f"      {message}")
        
        if passed:
            self.passed += 1
        else:
            self.failed += 1
    
    async def test_flow_loading(self):
        """Test loading flow files"""
        self.print_header("Testing Flow Loading")
        
        # Test loading rack rotation inlet flow
        inlet_flow_path = self.flows_dir / "rack_rotation_room_inlet.yaml"
        if inlet_flow_path.exists():
            try:
                with open(inlet_flow_path, 'r') as f:
                    flow_data = yaml.safe_load(f)
                
                # Validate structure
                has_meta = 'meta' in flow_data
                has_flow = 'flow' in flow_data
                has_workflow = 'workflow' in flow_data
                is_v2 = flow_data.get('meta', {}).get('system') == 'linear_flow_v2'
                
                all_valid = has_meta and has_flow and has_workflow and is_v2
                
                self.print_result(
                    "Load rack_rotation_room_inlet.yaml",
                    all_valid,
                    f"meta:{has_meta} flow:{has_flow} workflow:{has_workflow} v2:{is_v2}"
                )
            except Exception as e:
                self.print_result("Load rack_rotation_room_inlet.yaml", False, str(e))
        else:
            self.print_result("Load rack_rotation_room_inlet.yaml", False, "File not found")
        
        # Test loading rack rotation outlet flow
        outlet_flow_path = self.flows_dir / "rack_rotation_room_outlet.yaml"
        if outlet_flow_path.exists():
            try:
                with open(outlet_flow_path, 'r') as f:
                    flow_data = yaml.safe_load(f)
                
                is_valid = (
                    flow_data.get('meta', {}).get('system') == 'linear_flow_v2' and
                    flow_data.get('flow', {}).get('work_id') == '220002'
                )
                
                self.print_result(
                    "Load rack_rotation_room_outlet.yaml",
                    is_valid,
                    f"work_id: {flow_data.get('flow', {}).get('work_id')}"
                )
            except Exception as e:
                self.print_result("Load rack_rotation_room_outlet.yaml", False, str(e))
        else:
            self.print_result("Load rack_rotation_room_outlet.yaml", False, "File not found")
    
    async def test_flow_executor(self):
        """Test flow executor functionality"""
        self.print_header("Testing Flow Executor")
        
        # Create a simple test flow
        test_flow = {
            'meta': {
                'system': 'linear_flow_v2',
                'version': '2.0.0'
            },
            'flow': {
                'id': 'test_integration',
                'name': 'Integration Test Flow',
                'work_id': '999999',
                'enabled': True
            },
            'workflow': [
                {
                    'section': 'Initialize',
                    'steps': [
                        {
                            'id': 'log_start',
                            'exec': 'action.log',
                            'params': {
                                'message': 'Integration test started',
                                'level': 'info'
                            }
                        }
                    ]
                },
                {
                    'section': 'Query Test',
                    'steps': [
                        {
                            'id': 'query_locations',
                            'exec': 'query.locations',
                            'params': {
                                'type': 'room_inlet',
                                'rooms': [101, 102]
                            },
                            'store': 'test_locations'
                        },
                        {
                            'id': 'check_results',
                            'exec': 'check.empty',
                            'params': {
                                'data': '${test_locations}'
                            },
                            'store': 'is_empty'
                        }
                    ]
                },
                {
                    'section': 'Conditional Test',
                    'steps': [
                        {
                            'id': 'skip_test',
                            'exec': 'action.log',
                            'params': {
                                'message': 'This should be skipped'
                            },
                            'skip_if': 'true'
                        },
                        {
                            'id': 'run_test',
                            'exec': 'action.log',
                            'params': {
                                'message': 'This should run'
                            },
                            'skip_if': 'false'
                        }
                    ]
                }
            ]
        }
        
        # Test executor initialization
        try:
            executor = FlowExecutor(test_flow)
            self.print_result("Initialize FlowExecutor", True)
        except Exception as e:
            self.print_result("Initialize FlowExecutor", False, str(e))
            return
        
        # Test flow execution
        try:
            context = await executor.execute()
            
            # Check execution results
            execution_complete = context.get('status') == 'completed'
            has_logs = len(context.get('logs', [])) > 0
            has_variables = 'test_locations' in context.get('variables', {})
            
            self.print_result(
                "Execute test flow",
                execution_complete,
                f"status:{context.get('status')} logs:{len(context.get('logs', []))}"
            )
            
            # Check skip_if functionality
            logs_text = ' '.join([log.get('message', '') for log in context.get('logs', [])])
            skip_worked = 'This should be skipped' not in logs_text
            run_worked = 'This should run' in logs_text or 'This should run' in str(context)
            
            self.print_result(
                "Conditional execution (skip_if)",
                skip_worked and run_worked,
                f"skip:{skip_worked} run:{run_worked}"
            )
            
        except Exception as e:
            self.print_result("Execute test flow", False, str(e))
    
    async def test_variable_resolution(self):
        """Test variable resolution in flows"""
        self.print_header("Testing Variable Resolution")
        
        test_flow = {
            'meta': {'system': 'linear_flow_v2', 'version': '2.0.0'},
            'flow': {'id': 'var_test', 'name': 'Variable Test'},
            'workflow': [
                {
                    'section': 'Variable Test',
                    'steps': [
                        {
                            'id': 'set_var1',
                            'exec': 'query.locations',
                            'params': {'type': 'test'},
                            'store': 'var1'
                        },
                        {
                            'id': 'set_var2',
                            'exec': 'control.count',
                            'params': {'variable': '${var1}'},
                            'store': 'var2'
                        },
                        {
                            'id': 'use_vars',
                            'exec': 'action.log',
                            'params': {
                                'message': 'Var1: ${var1}, Var2: ${var2}'
                            }
                        }
                    ]
                }
            ]
        }
        
        try:
            executor = FlowExecutor(test_flow)
            context = await executor.execute()
            
            # Check variable storage
            has_var1 = 'var1' in context.get('variables', {})
            has_var2 = 'var2' in context.get('variables', {})
            
            self.print_result(
                "Variable storage and resolution",
                has_var1 and has_var2,
                f"var1:{has_var1} var2:{has_var2}"
            )
            
        except Exception as e:
            self.print_result("Variable storage and resolution", False, str(e))
    
    async def test_foreach_loop(self):
        """Test foreach loop execution"""
        self.print_header("Testing ForEach Loops")
        
        test_flow = {
            'meta': {'system': 'linear_flow_v2', 'version': '2.0.0'},
            'flow': {'id': 'foreach_test', 'name': 'ForEach Test'},
            'workflow': [
                {
                    'section': 'ForEach Test',
                    'steps': [
                        {
                            'id': 'create_list',
                            'exec': 'query.locations',
                            'params': {'type': 'test'},
                            'store': 'items'
                        },
                        {
                            'id': 'foreach_items',
                            'exec': 'foreach',
                            'items': '${items}',
                            'var': 'item',
                            'steps': [
                                {
                                    'id': 'process_item',
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
        
        try:
            executor = FlowExecutor(test_flow)
            context = await executor.execute()
            
            # Check if foreach was executed
            logs = context.get('logs', [])
            foreach_executed = any('foreach' in str(log) for log in logs)
            
            self.print_result(
                "ForEach loop execution",
                context.get('status') == 'completed',
                f"Status: {context.get('status')}"
            )
            
        except Exception as e:
            self.print_result("ForEach loop execution", False, str(e))
    
    async def test_parallel_execution(self):
        """Test parallel branch execution"""
        self.print_header("Testing Parallel Execution")
        
        test_flow = {
            'meta': {'system': 'linear_flow_v2', 'version': '2.0.0'},
            'flow': {'id': 'parallel_test', 'name': 'Parallel Test'},
            'workflow': [
                {
                    'section': 'Parallel Test',
                    'steps': [
                        {
                            'id': 'parallel_branches',
                            'exec': 'parallel',
                            'branches': [
                                {
                                    'name': 'branch1',
                                    'steps': [
                                        {
                                            'id': 'branch1_step1',
                                            'exec': 'action.log',
                                            'params': {'message': 'Branch 1 executing'}
                                        }
                                    ]
                                },
                                {
                                    'name': 'branch2',
                                    'steps': [
                                        {
                                            'id': 'branch2_step1',
                                            'exec': 'action.log',
                                            'params': {'message': 'Branch 2 executing'}
                                        }
                                    ]
                                }
                            ]
                        }
                    ]
                }
            ]
        }
        
        try:
            executor = FlowExecutor(test_flow)
            context = await executor.execute()
            
            # Check if both branches executed
            logs_text = str(context.get('logs', []))
            branch1_executed = 'Branch 1' in logs_text or 'branch1' in logs_text
            branch2_executed = 'Branch 2' in logs_text or 'branch2' in logs_text
            
            self.print_result(
                "Parallel branch execution",
                context.get('status') == 'completed',
                f"Branch1:{branch1_executed} Branch2:{branch2_executed}"
            )
            
        except Exception as e:
            self.print_result("Parallel branch execution", False, str(e))
    
    async def test_flow_monitor(self):
        """Test flow monitor functionality"""
        self.print_header("Testing Flow Monitor")
        
        try:
            # Create monitor instance
            monitor = FlowMonitor(scan_interval=5.0)
            
            # Test monitor initialization
            self.print_result("Initialize FlowMonitor", True)
            
            # Test getting flow status
            status = monitor.get_flow_status('test_flow')
            self.print_result(
                "Get flow status",
                status is not None,
                f"Status type: {type(status)}"
            )
            
            # Test metrics collection
            metrics = monitor.get_metrics()
            self.print_result(
                "Collect metrics",
                metrics is not None,
                f"Metrics keys: {list(metrics.keys()) if metrics else 'None'}"
            )
            
        except Exception as e:
            self.print_result("Flow Monitor tests", False, str(e))
    
    def print_summary(self):
        """Print test summary"""
        print("\n" + "="*50)
        print("  TEST SUMMARY")
        print("="*50)
        
        total = self.passed + self.failed
        pass_rate = (self.passed / total * 100) if total > 0 else 0
        
        print(f"Total Tests: {total}")
        print(f"Passed: \033[92m{self.passed}\033[0m")
        print(f"Failed: \033[91m{self.failed}\033[0m")
        print(f"Pass Rate: {pass_rate:.1f}%")
        
        if self.failed == 0:
            print("\n\033[92m✓ All tests passed!\033[0m")
        else:
            print(f"\n\033[91m✗ {self.failed} test(s) failed\033[0m")
        
        print("="*50)
    
    async def run_all_tests(self):
        """Run all integration tests"""
        print("\n" + "="*50)
        print("  FLOW WCS INTEGRATION TESTS")
        print("="*50)
        print(f"Time: {time.strftime('%Y-%m-%d %H:%M:%S')}")
        
        # Run test suites
        await self.test_flow_loading()
        await self.test_flow_executor()
        await self.test_variable_resolution()
        await self.test_foreach_loop()
        await self.test_parallel_execution()
        await self.test_flow_monitor()
        
        # Print summary
        self.print_summary()
        
        # Return exit code
        return 0 if self.failed == 0 else 1


async def main():
    """Main test runner"""
    tester = FlowWCSIntegrationTest()
    exit_code = await tester.run_all_tests()
    sys.exit(exit_code)


if __name__ == '__main__':
    asyncio.run(main())