#!/usr/bin/env python3
"""
Simplified Flow WCS Node - Main ROS 2 node for executing linear workflows
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import yaml
import json
import os
from pathlib import Path
from typing import Dict, Any, List, Optional
from datetime import datetime
import asyncio
from concurrent.futures import ThreadPoolExecutor
import signal
import sys
import threading

from .flow_executor import FlowExecutor
from .flow_monitor import FlowMonitor


class FlowWCSNode(Node):
    """Simplified Flow WCS ROS 2 Node"""
    
    def __init__(self):
        super().__init__('flow_wcs_node')
        
        # Shutdown flag
        self.is_shutting_down = False
        self.shutdown_event = threading.Event()
        
        # Node parameters
        self.declare_parameter('flows_dir', '/app/config/wcs/flows')
        self.declare_parameter('scan_interval', 10.0)
        self.declare_parameter('max_parallel_flows', 5)
        
        # Get parameters
        self.flows_dir = Path(self.get_parameter('flows_dir').value)
        self.scan_interval = self.get_parameter('scan_interval').value
        self.max_parallel_flows = self.get_parameter('max_parallel_flows').value
        
        # Flow management
        self.loaded_flows = {}
        self.active_executions = {}
        self.thread_executor = ThreadPoolExecutor(max_workers=self.max_parallel_flows)
        self.monitor = FlowMonitor()
        
        # Publishers
        self.status_pub = self.create_publisher(String, '/flow_wcs/status', 10)
        self.event_pub = self.create_publisher(String, '/flow_wcs/events', 10)
        
        # Subscribers
        self.trigger_sub = self.create_subscription(
            String, '/flow_wcs/trigger', self.handle_trigger, 10)
        
        # Start flow scanning
        self.scan_timer = self.create_timer(self.scan_interval, self.scan_flows)
        
        # Initial scan
        self.scan_flows()
        
        self.get_logger().info(f"Flow WCS Node started - monitoring {self.flows_dir}")
    
    def scan_flows(self):
        """Scan flows directory and load YAML files"""
        # Skip if shutting down
        if self.is_shutting_down:
            return
            
        if not self.flows_dir.exists():
            self.get_logger().warning(f"Flows directory not found: {self.flows_dir}")
            self.flows_dir.mkdir(parents=True, exist_ok=True)
            self.get_logger().info(f"Created flows directory: {self.flows_dir}")
            return
        
        # Load all YAML files
        yaml_files = list(self.flows_dir.glob("*.yaml"))
        self.get_logger().info(f"Found {len(yaml_files)} flow files")
        
        for flow_file in yaml_files:
            try:
                with open(flow_file, 'r', encoding='utf-8') as f:
                    flow_data = yaml.safe_load(f)
                
                if self.validate_flow(flow_data):
                    flow_id = flow_data['flow']['id']
                    self.loaded_flows[flow_id] = flow_data
                    self.get_logger().info(f"Loaded flow: {flow_id}")
                    
                    # Auto-execute if enabled
                    if flow_data['flow'].get('enabled', True):
                        if flow_id not in self.active_executions:
                            self.execute_flow_async(flow_id, flow_data)
                else:
                    self.get_logger().warning(f"Invalid flow structure: {flow_file}")
                    
            except Exception as e:
                self.get_logger().error(f"Failed to load flow {flow_file}: {e}")
    
    def validate_flow(self, flow_data: Dict) -> bool:
        """Validate flow structure"""
        try:
            # Check required keys
            if not all(key in flow_data for key in ['meta', 'flow', 'workflow']):
                return False
            
            # Check meta
            if flow_data['meta'].get('system') != 'linear_flow_v2':
                return False
            
            # Check flow
            if 'id' not in flow_data['flow']:
                return False
            
            # Check workflow
            if not isinstance(flow_data['workflow'], list):
                return False
            
            return True
        except:
            return False
    
    def execute_flow_async(self, flow_id: str, flow_data: Dict):
        """Execute a flow asynchronously"""
        # Skip if shutting down
        if self.is_shutting_down:
            self.get_logger().info(f"Skipping flow {flow_id} - node is shutting down")
            return
            
        if flow_id in self.active_executions:
            self.get_logger().info(f"Flow {flow_id} is already executing")
            return
        
        # Mark as active
        self.active_executions[flow_id] = True
        
        # Submit to executor
        future = self.thread_executor.submit(self.execute_flow, flow_id, flow_data)
        future.add_done_callback(lambda f: self.flow_complete(flow_id, f))
        
        # Publish event
        event = {
            'type': 'flow_started',
            'flow_id': flow_id,
            'timestamp': datetime.now().isoformat()
        }
        self.event_pub.publish(String(data=json.dumps(event)))
        
        self.get_logger().info(f"Started execution of flow: {flow_id}")
    
    def execute_flow(self, flow_id: str, flow_data: Dict) -> Dict:
        """Execute a flow (runs in thread pool)"""
        try:
            # Create executor with ROS node for logging
            executor = FlowExecutor(flow_data, ros_node=self)
            
            # Register with monitor
            self.monitor.register_flow_start(
                flow_id, 
                flow_data['flow'].get('name', flow_id),
                flow_data['flow'].get('work_id', '')
            )
            
            # Run flow
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            result = loop.run_until_complete(executor.execute())
            loop.close()
            
            # Update monitor
            self.monitor.register_flow_complete(flow_id, result.get('status', 'unknown'))
            
            return result
            
        except Exception as e:
            self.get_logger().error(f"Flow execution failed: {e}")
            self.monitor.register_flow_complete(flow_id, 'failed')
            return {'status': 'failed', 'error': str(e)}
    
    def flow_complete(self, flow_id: str, future):
        """Handle flow completion"""
        try:
            result = future.result()
            status = result.get('status', 'unknown')
            
            # Remove from active
            if flow_id in self.active_executions:
                del self.active_executions[flow_id]
            
            # Publish event
            event = {
                'type': 'flow_completed',
                'flow_id': flow_id,
                'status': status,
                'timestamp': datetime.now().isoformat()
            }
            self.event_pub.publish(String(data=json.dumps(event)))
            
            self.get_logger().info(f"Flow {flow_id} completed with status: {status}")
            
        except Exception as e:
            self.get_logger().error(f"Error handling flow completion: {e}")
    
    def handle_trigger(self, msg: String):
        """Handle manual flow trigger"""
        try:
            data = json.loads(msg.data)
            flow_id = data.get('flow_id')
            
            if flow_id in self.loaded_flows:
                self.execute_flow_async(flow_id, self.loaded_flows[flow_id])
                self.get_logger().info(f"Manually triggered flow: {flow_id}")
            else:
                self.get_logger().warning(f"Flow not found: {flow_id}")
                
        except Exception as e:
            self.get_logger().error(f"Error handling trigger: {e}")
    
    def get_status(self) -> Dict:
        """Get current status"""
        return {
            'loaded_flows': list(self.loaded_flows.keys()),
            'active_executions': list(self.active_executions.keys()),
            'metrics': self.monitor.get_metrics()
        }
    
    def cleanup(self):
        """Clean up resources gracefully"""
        self.get_logger().info("Starting graceful shutdown...")
        
        # Set shutdown flag
        self.is_shutting_down = True
        self.shutdown_event.set()
        
        # Cancel timer
        if hasattr(self, 'scan_timer'):
            self.scan_timer.cancel()
            self.destroy_timer(self.scan_timer)
        
        # Wait for active executions to complete (with timeout)
        if self.active_executions:
            self.get_logger().info(f"Waiting for {len(self.active_executions)} active flows to complete...")
            timeout = 10.0  # 10 seconds timeout
            start_time = datetime.now()
            
            while self.active_executions and (datetime.now() - start_time).total_seconds() < timeout:
                import time
                time.sleep(0.1)
            
            if self.active_executions:
                self.get_logger().warning(f"Forcefully stopping {len(self.active_executions)} flows")
        
        # Shutdown thread executor
        if hasattr(self, 'thread_executor'):
            self.get_logger().info("Shutting down thread executor...")
            self.thread_executor.shutdown(wait=False)
        
        # Destroy publishers and subscriptions
        if hasattr(self, 'status_pub'):
            self.destroy_publisher(self.status_pub)
        if hasattr(self, 'event_pub'):
            self.destroy_publisher(self.event_pub)
        if hasattr(self, 'trigger_sub'):
            self.destroy_subscription(self.trigger_sub)
        
        self.get_logger().info("Graceful shutdown completed")


def main(args=None):
    """Main entry point"""
    # Initialize ROS 2
    rclpy.init(args=args)
    
    node = None
    executor = None
    
    def signal_handler(signum, frame):
        """Handle shutdown signals gracefully"""
        nonlocal node, executor
        
        print("\n[INFO] Received shutdown signal, cleaning up...")
        
        if node:
            node.cleanup()
        
        if executor:
            executor.shutdown(timeout_sec=0.1)
        
        # Trigger shutdown
        if rclpy.ok():
            rclpy.shutdown()
        
        sys.exit(0)
    
    # Register signal handlers
    signal.signal(signal.SIGINT, signal_handler)   # Ctrl+C
    signal.signal(signal.SIGTERM, signal_handler)  # Termination signal
    
    try:
        # Create node
        node = FlowWCSNode()
        
        # Create single-threaded executor for graceful shutdown
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(node)
        
        # Spin the node
        try:
            executor.spin()
        except KeyboardInterrupt:
            print("\n[INFO] Keyboard interrupt received")
        except Exception as e:
            if node:
                node.get_logger().error(f"Unexpected error: {e}")
            else:
                print(f"[ERROR] Unexpected error: {e}")
        
    except Exception as e:
        print(f"[ERROR] Failed to initialize node: {e}")
        
    finally:
        # Clean up
        if node:
            try:
                node.cleanup()
                node.destroy_node()
            except Exception as e:
                print(f"[WARNING] Error during cleanup: {e}")
        
        if executor:
            try:
                executor.shutdown(timeout_sec=0.1)
            except Exception as e:
                print(f"[WARNING] Error shutting down executor: {e}")
        
        # Shutdown ROS 2
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception as e:
                print(f"[WARNING] Error during ROS 2 shutdown: {e}")


if __name__ == '__main__':
    main()