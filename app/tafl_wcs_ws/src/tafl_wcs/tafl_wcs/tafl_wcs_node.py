#!/usr/bin/env python3
"""
Enhanced TAFL WCS Node with Progress Reporting and Active Execution
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import asyncio
from datetime import datetime
from typing import Dict, Any, Optional
import threading
from .tafl_wcs_manager import TAFLWCSManager

class ProgressReporter:
    """Progress reporting component for TAFL execution"""
    
    def __init__(self, node: Node):
        self.node = node
        self.current_progress = {}
        self.progress_publisher = node.create_publisher(String, '/tafl/execution_progress', 10)
        self.history_storage = []
        self.max_history = 1000
        
    def report_progress(self, flow_id: str, step: int, total_steps: int, 
                       status: str, message: str = None):
        """Report execution progress"""
        progress_data = {
            'flow_id': flow_id,
            'current_step': step,
            'total_steps': total_steps,
            'percentage': (step / total_steps * 100) if total_steps > 0 else 0,
            'status': status,
            'message': message,
            'timestamp': datetime.now().isoformat()
        }
        
        # Update current progress
        self.current_progress[flow_id] = progress_data
        
        # Publish progress
        msg = String()
        msg.data = json.dumps(progress_data)
        self.progress_publisher.publish(msg)
        
        # Log progress
        self.node.get_logger().info(f"Progress: {flow_id} - Step {step}/{total_steps} ({progress_data['percentage']:.1f}%) - {status}")
        
        return progress_data
    
    def get_progress(self, flow_id: str = None) -> Dict:
        """Get current progress"""
        if flow_id:
            return self.current_progress.get(flow_id, {})
        return self.current_progress
    
    def save_to_history(self, flow_id: str, execution_data: Dict):
        """Save execution to history"""
        history_entry = {
            'flow_id': flow_id,
            'timestamp': datetime.now().isoformat(),
            'execution_data': execution_data,
            'progress': self.current_progress.get(flow_id, {})
        }
        
        self.history_storage.append(history_entry)
        
        # Limit history size
        if len(self.history_storage) > self.max_history:
            self.history_storage = self.history_storage[-self.max_history:]
        
        # Could also persist to file or database here
        self._persist_history()
        
        return history_entry
    
    def _persist_history(self):
        """Persist history to file"""
        try:
            import os
            history_file = '/tmp/tafl_execution_history.json'
            
            # Save last 100 entries to file
            recent_history = self.history_storage[-100:]
            
            with open(history_file, 'w') as f:
                json.dump(recent_history, f, indent=2)
                
            self.node.get_logger().debug(f"History persisted to {history_file}")
            
        except Exception as e:
            self.node.get_logger().error(f"Failed to persist history: {e}")
    
    def load_history(self) -> list:
        """Load history from file"""
        try:
            import os
            history_file = '/tmp/tafl_execution_history.json'
            
            if os.path.exists(history_file):
                with open(history_file, 'r') as f:
                    loaded_history = json.load(f)
                    self.history_storage = loaded_history
                    self.node.get_logger().info(f"Loaded {len(loaded_history)} history entries")
                    return loaded_history
                    
        except Exception as e:
            self.node.get_logger().error(f"Failed to load history: {e}")
        
        return []


class EnhancedTAFLWCSNode(Node):
    """Enhanced TAFL WCS Node with progress reporting and monitoring"""
    
    def __init__(self):
        super().__init__('enhanced_tafl_wcs_node')
        
        # Declare parameters for active execution
        self.declare_parameter('flows_dir', '/app/config/tafl/flows')
        self.declare_parameter('scan_interval', 3.0)
        self.declare_parameter('execution_interval', 5.0)
        self.declare_parameter('auto_execute', True)
        self.declare_parameter('database_url', 
            'postgresql://agvc:password@192.168.100.254:5432/agvc')
        
        # Get parameter values
        flows_dir = self.get_parameter('flows_dir').value
        scan_interval = self.get_parameter('scan_interval').value
        execution_interval = self.get_parameter('execution_interval').value
        auto_execute = self.get_parameter('auto_execute').value
        database_url = self.get_parameter('database_url').value
        
        # Initialize TAFLWCSManager
        self.manager = TAFLWCSManager(
            flows_dir=flows_dir,
            database_url=database_url,
            logger=self.get_logger()
        )
        
        # Initialize progress reporter
        self.progress_reporter = ProgressReporter(self)
        
        # Load history on startup
        self.progress_reporter.load_history()
        
        # Track active executions
        self.active_executions = {}
        
        # Create subscribers and publishers
        self.flow_subscriber = self.create_subscription(
            String,
            '/tafl/execute_flow',
            self.execute_flow_callback,
            10
        )
        
        self.result_publisher = self.create_publisher(
            String,
            '/tafl/execution_result',
            10
        )
        
        # Create service for progress queries
        from std_srvs.srv import Trigger
        self.progress_service = self.create_service(
            Trigger,
            '/tafl/get_progress',
            self.get_progress_callback
        )
        
        # Performance metrics
        self.metrics = {
            'total_flows': 0,
            'successful_flows': 0,
            'failed_flows': 0,
            'average_execution_time': 0,
            'total_execution_time': 0
        }
        
        # Execution thread pool
        self.executor_thread = None
        
        # Create timers for active execution
        if auto_execute:
            # Scan timer
            self.scan_timer = self.create_timer(
                scan_interval,
                self.scan_flows_callback
            )
            self.get_logger().info(f'Created scan timer: every {scan_interval} seconds')
            
            # Execution timer
            self.execute_timer = self.create_timer(
                execution_interval,
                self.execute_flows_callback
            )
            self.get_logger().info(f'Created execution timer: every {execution_interval} seconds')
            
            # Initial scan
            self.scan_flows_callback()
        
        self.get_logger().info('Enhanced TAFL WCS Node started with active execution mode')
    
    def scan_flows_callback(self):
        """Periodically scan flow files"""
        try:
            self.get_logger().debug('Starting flow directory scan...')
            loaded_flows = self.manager.scan_flows()
            
            if loaded_flows:
                self.get_logger().info(
                    f'Scan complete: loaded/updated {len(loaded_flows)} flows'
                )
                for flow_id in loaded_flows:
                    flow = self.manager.loaded_flows.get(flow_id, {})
                    self.get_logger().debug(
                        f'  - {flow_id}: {flow.get("name", "unnamed")} '
                        f'(enabled: {flow.get("enabled", True)})'
                    )
            else:
                self.get_logger().debug('Scan complete: no new or updated flows')
                
        except Exception as e:
            self.get_logger().error(f'Flow scan failed: {e}')
    
    def execute_flows_callback(self):
        """Periodically execute all enabled flows synchronously (like RCS dispatch)"""
        try:
            # Scan for latest flows (only load enabled ones)
            self.manager.scan_flows(only_enabled=True)
            
            # Get all enabled flows
            enabled_flows = []
            for flow_id, flow in self.manager.loaded_flows.items():
                # Double check enabled status
                if flow.get('enabled', True):
                    enabled_flows.append(flow_id)
                else:
                    self.get_logger().debug(f'Skipping disabled flow: {flow_id}')
            
            if not enabled_flows:
                self.get_logger().debug('No enabled flows to execute')
                return
            
            self.get_logger().info(f'Executing {len(enabled_flows)} enabled flows sequentially')
            
            # Execute each flow synchronously (like RCS dispatch)
            for flow_id in enabled_flows:
                try:
                    self.get_logger().info(f'Executing flow: {flow_id}')
                    
                    # Report start
                    self.progress_reporter.report_progress(
                        flow_id, 0, 100, 'EXECUTING', 
                        f'Starting synchronous execution'
                    )
                    
                    # Execute flow synchronously using manager
                    start_time = datetime.now()
                    result = self.manager.execute_flow_sync(flow_id)
                    execution_time = (datetime.now() - start_time).total_seconds()
                    
                    # Report completion
                    if result.get('status') == 'completed':
                        self.progress_reporter.report_progress(
                            flow_id, 100, 100, 'COMPLETED', 
                            f'Flow completed in {execution_time:.2f}s'
                        )
                        self.get_logger().info(f'Flow {flow_id} completed successfully in {execution_time:.2f}s')
                    elif result.get('status') == 'skipped':
                        self.progress_reporter.report_progress(
                            flow_id, -1, 100, 'SKIPPED', 
                            result.get('reason', 'Flow was skipped')
                        )
                        self.get_logger().info(f'Flow {flow_id} skipped: {result.get("reason")}')
                    else:
                        self.progress_reporter.report_progress(
                            flow_id, -1, 100, 'FAILED', 
                            f'Flow failed: {result.get("error", "Unknown error")}'
                        )
                        self.get_logger().error(f'Flow {flow_id} failed: {result.get("error")}')
                    
                    # Update metrics
                    if result.get('status') == 'completed':
                        self._update_metrics(True, execution_time)
                    else:
                        self._update_metrics(False, execution_time)
                        
                except Exception as e:
                    self.get_logger().error(f'Failed to execute flow {flow_id}: {e}')
                    self.progress_reporter.report_progress(
                        flow_id, -1, 100, 'FAILED', 
                        f'Execution failed: {str(e)}'
                    )
                    self._update_metrics(False, 0)
                        
        except Exception as e:
            self.get_logger().error(f'Execute flows callback failed: {e}')
    
    def execute_flow_callback(self, msg: String):
        """Handle flow execution request"""
        try:
            flow_data = json.loads(msg.data)
            flow_id = flow_data.get('flow_id', f'flow_{datetime.now().strftime("%Y%m%d_%H%M%S")}')
            flow_content = flow_data.get('content', '')
            
            self.get_logger().info(f'Received flow execution request: {flow_id}')
            
            # Start execution in background thread
            if self.executor_thread and self.executor_thread.is_alive():
                self.get_logger().warning('Previous execution still running')
                return
            
            self.executor_thread = threading.Thread(
                target=self._execute_flow_async,
                args=(flow_id, flow_content)
            )
            self.executor_thread.start()
            
        except Exception as e:
            self.get_logger().error(f'Failed to process flow request: {e}')
    
    def _execute_flow_async(self, flow_id: str, flow_content: str):
        """Execute flow asynchronously with progress reporting"""
        start_time = datetime.now()
        
        try:
            # Report start
            self.progress_reporter.report_progress(
                flow_id, 0, 100, 'STARTED', 'Flow execution started'
            )
            
            # Simulate flow parsing (10%)
            self.progress_reporter.report_progress(
                flow_id, 10, 100, 'PARSING', 'Parsing TAFL content'
            )
            
            # Simulate validation (20%)
            self.progress_reporter.report_progress(
                flow_id, 20, 100, 'VALIDATING', 'Validating flow structure'
            )
            
            # Simulate execution steps
            steps = 10  # Simulate 10 execution steps
            for i in range(steps):
                step_progress = 20 + (60 * (i + 1) / steps)  # 20% to 80%
                self.progress_reporter.report_progress(
                    flow_id, int(step_progress), 100, 'EXECUTING', 
                    f'Executing step {i+1}/{steps}'
                )
                # Simulate work
                import time
                time.sleep(0.5)
            
            # Report completion
            self.progress_reporter.report_progress(
                flow_id, 100, 100, 'COMPLETED', 'Flow execution completed successfully'
            )
            
            # Calculate execution time
            execution_time = (datetime.now() - start_time).total_seconds()
            
            # Update metrics
            self._update_metrics(True, execution_time)
            
            # Save to history
            execution_data = {
                'status': 'completed',
                'execution_time': execution_time,
                'steps_executed': steps
            }
            self.progress_reporter.save_to_history(flow_id, execution_data)
            
            # Publish result
            result = {
                'flow_id': flow_id,
                'status': 'completed',
                'execution_time': execution_time,
                'metrics': self.get_metrics()
            }
            
            result_msg = String()
            result_msg.data = json.dumps(result)
            self.result_publisher.publish(result_msg)
            
            self.get_logger().info(f'Flow {flow_id} completed in {execution_time:.2f}s')
            
        except Exception as e:
            # Report failure
            self.progress_reporter.report_progress(
                flow_id, -1, 100, 'FAILED', f'Execution failed: {str(e)}'
            )
            
            # Update metrics
            execution_time = (datetime.now() - start_time).total_seconds()
            self._update_metrics(False, execution_time)
            
            # Save to history
            execution_data = {
                'status': 'failed',
                'error': str(e),
                'execution_time': execution_time
            }
            self.progress_reporter.save_to_history(flow_id, execution_data)
            
            self.get_logger().error(f'Flow {flow_id} failed: {e}')
    
    def _execute_flow_with_manager(self, flow_id: str, flow_content: str):
        """Execute flow using TAFLWCSManager"""
        start_time = datetime.now()
        
        try:
            # Report execution start
            self.progress_reporter.report_progress(
                flow_id, 10, 100, 'EXECUTING', 'Starting TAFL flow execution'
            )
            
            # Use asyncio to execute async method
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            
            # Execute flow
            result = loop.run_until_complete(
                self.manager.execute_flow(flow_id)
            )
            
            # Calculate execution time
            execution_time = (datetime.now() - start_time).total_seconds()
            
            # Report completion
            if result.get('status') == 'success':
                self.progress_reporter.report_progress(
                    flow_id, 100, 100, 'COMPLETED', 
                    f'Flow executed successfully (time: {execution_time:.2f}s)'
                )
                
                # Update metrics
                self._update_metrics(True, execution_time)
            else:
                error_msg = result.get('error', 'Unknown error')
                self.progress_reporter.report_progress(
                    flow_id, -1, 100, 'FAILED', f'Execution failed: {error_msg}'
                )
                
                # Update metrics
                self._update_metrics(False, execution_time)
            
            # Save to history
            execution_data = {
                'status': result.get('status', 'unknown'),
                'execution_time': execution_time,
                'result': result
            }
            self.progress_reporter.save_to_history(flow_id, execution_data)
            
            # Publish result
            result_msg = String()
            result_msg.data = json.dumps({
                'flow_id': flow_id,
                'status': result.get('status'),
                'execution_time': execution_time,
                'auto_triggered': True,
                'timestamp': datetime.now().isoformat()
            })
            self.result_publisher.publish(result_msg)
            
            self.get_logger().info(
                f'Flow {flow_id} completed: {result.get("status")} '
                f'(time: {execution_time:.2f}s)'
            )
            
        except Exception as e:
            self.get_logger().error(f'Error executing flow {flow_id}: {e}')
            
            # Report failure
            self.progress_reporter.report_progress(
                flow_id, -1, 100, 'FAILED', f'Execution exception: {str(e)}'
            )
            
            # Update metrics
            execution_time = (datetime.now() - start_time).total_seconds()
            self._update_metrics(False, execution_time)
            
        finally:
            # Clear execution marker
            if flow_id in self.active_executions:
                del self.active_executions[flow_id]
            
            # Close event loop
            loop.close()
    
    def get_progress_callback(self, request, response):
        """Service callback to get progress"""
        try:
            # Trigger service doesn't have input data
            progress_data = {
                'active_flows': list(self.active_flows),
                'progress': self.progress_reporter.progress_data,
                'history': self.progress_reporter.history[-10:] if hasattr(self.progress_reporter, 'history') else [],
                'metrics': self.get_metrics()
            }
            
            response.success = True
            response.message = json.dumps(progress_data)
            
        except Exception as e:
            self.get_logger().error(f'Failed to get progress: {e}')
            response.success = False
            response.message = str(e)
        
        return response
    
    def _update_metrics(self, success: bool, execution_time: float):
        """Update performance metrics"""
        self.metrics['total_flows'] += 1
        self.metrics['total_execution_time'] += execution_time
        
        if success:
            self.metrics['successful_flows'] += 1
        else:
            self.metrics['failed_flows'] += 1
        
        # Update average
        self.metrics['average_execution_time'] = (
            self.metrics['total_execution_time'] / self.metrics['total_flows']
        )
    
    def get_metrics(self) -> Dict:
        """Get performance metrics"""
        return {
            **self.metrics,
            'success_rate': (self.metrics['successful_flows'] / 
                           self.metrics['total_flows'] * 100) 
                          if self.metrics['total_flows'] > 0 else 0,
            'failure_rate': (self.metrics['failed_flows'] / 
                           self.metrics['total_flows'] * 100) 
                          if self.metrics['total_flows'] > 0 else 0
        }


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    node = EnhancedTAFLWCSNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()