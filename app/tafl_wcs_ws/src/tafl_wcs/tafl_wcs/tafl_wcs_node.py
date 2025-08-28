#!/usr/bin/env python3
"""
TAFL WCS Node - Main ROS 2 node for TAFL-based WCS
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import yaml
import json
import asyncio
from pathlib import Path
from typing import Dict, Any, List
from datetime import datetime
import threading
import signal
import sys

from .tafl_wcs_manager import TAFLWCSManager


class TAFLWCSNode(Node):
    """TAFL WCS ROS 2 Node"""
    
    def __init__(self):
        super().__init__('tafl_wcs_node')
        
        # Shutdown flag
        self.is_shutting_down = False
        self.shutdown_event = threading.Event()
        
        # Node parameters
        self.declare_parameter('flows_dir', '/app/config/tafl/flows')
        self.declare_parameter('scan_interval', 10.0)
        self.declare_parameter('database_url', 'postgresql://agvc:password@192.168.100.254:5432/agvc')
        self.declare_parameter('auto_execute', True)
        self.declare_parameter('execution_interval', 60.0)
        
        # Get parameters
        self.flows_dir = self.get_parameter('flows_dir').value
        self.scan_interval = self.get_parameter('scan_interval').value
        self.database_url = self.get_parameter('database_url').value
        self.auto_execute = self.get_parameter('auto_execute').value
        self.execution_interval = self.get_parameter('execution_interval').value
        
        # Initialize TAFL WCS Manager
        self.manager = TAFLWCSManager(
            flows_dir=self.flows_dir,
            database_url=self.database_url,
            logger=self.get_logger()
        )
        
        # Publishers
        self.status_pub = self.create_publisher(String, '/tafl_wcs/status', 10)
        self.event_pub = self.create_publisher(String, '/tafl_wcs/events', 10)
        
        # Subscribers
        self.trigger_sub = self.create_subscription(
            String, '/tafl_wcs/trigger', self.handle_trigger, 10)
        self.command_sub = self.create_subscription(
            String, '/tafl_wcs/command', self.handle_command, 10)
        
        # Timers
        self.scan_timer = self.create_timer(self.scan_interval, self.scan_flows)
        
        if self.auto_execute:
            self.execute_timer = self.create_timer(
                self.execution_interval, self.auto_execute_flows)
        
        # Asyncio event loop for async operations
        self.loop = asyncio.new_event_loop()
        self.async_thread = threading.Thread(target=self._run_async_loop, daemon=True)
        self.async_thread.start()
        
        # Initial scan
        self.scan_flows()
        
        self.get_logger().info(f"TAFL WCS Node started - monitoring {self.flows_dir}")
        self.publish_event('node_started', {'flows_dir': self.flows_dir})
    
    def _run_async_loop(self):
        """Run async event loop in separate thread"""
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()
    
    def scan_flows(self):
        """Scan and load TAFL flows"""
        if self.is_shutting_down:
            return
        
        try:
            loaded_flows = self.manager.scan_flows()
            
            if loaded_flows:
                self.get_logger().info(f"Loaded/updated {len(loaded_flows)} TAFL flows")
                self.publish_event('flows_loaded', {
                    'count': len(loaded_flows),
                    'flow_ids': loaded_flows
                })
            
            # Publish status
            self.publish_status()
            
        except Exception as e:
            self.get_logger().error(f"Error scanning flows: {e}")
            self.publish_event('scan_error', {'error': str(e)})
    
    def auto_execute_flows(self):
        """Auto-execute enabled flows"""
        if self.is_shutting_down:
            return
        
        self.get_logger().info("Starting auto-execution of enabled flows")
        
        # Submit async execution to event loop
        future = asyncio.run_coroutine_threadsafe(
            self.manager.execute_all_enabled_flows(),
            self.loop
        )
        
        # Add callback for completion
        future.add_done_callback(self._handle_auto_execution_complete)
    
    def _handle_auto_execution_complete(self, future):
        """Handle auto-execution completion
        
        Args:
            future: Completed future with results
        """
        try:
            results = future.result()
            successful = sum(1 for r in results if r.get('status') == 'completed')
            failed = sum(1 for r in results if r.get('status') == 'failed')
            
            self.get_logger().info(
                f"Auto-execution complete: {successful} successful, {failed} failed")
            
            self.publish_event('auto_execution_complete', {
                'total': len(results),
                'successful': successful,
                'failed': failed,
                'results': results
            })
            
        except Exception as e:
            self.get_logger().error(f"Auto-execution error: {e}")
            self.publish_event('auto_execution_error', {'error': str(e)})
    
    def handle_trigger(self, msg: String):
        """Handle manual flow trigger
        
        Args:
            msg: Trigger message with flow_id
        """
        try:
            data = json.loads(msg.data)
            flow_id = data.get('flow_id')
            
            if not flow_id:
                self.get_logger().warning("Trigger message missing flow_id")
                return
            
            self.get_logger().info(f"Manual trigger for flow: {flow_id}")
            
            # Submit async execution to event loop
            future = asyncio.run_coroutine_threadsafe(
                self.manager.execute_flow(flow_id),
                self.loop
            )
            
            # Add callback for completion
            future.add_done_callback(
                lambda f: self._handle_execution_complete(flow_id, f))
            
        except Exception as e:
            self.get_logger().error(f"Error handling trigger: {e}")
    
    def _handle_execution_complete(self, flow_id: str, future):
        """Handle flow execution completion
        
        Args:
            flow_id: Flow identifier
            future: Completed future with result
        """
        try:
            result = future.result()
            status = result.get('status', 'unknown')
            
            self.get_logger().info(f"Flow {flow_id} completed with status: {status}")
            
            self.publish_event('flow_completed', {
                'flow_id': flow_id,
                'status': status,
                'result': result
            })
            
        except Exception as e:
            self.get_logger().error(f"Flow {flow_id} execution error: {e}")
            self.publish_event('flow_error', {
                'flow_id': flow_id,
                'error': str(e)
            })
    
    def handle_command(self, msg: String):
        """Handle control commands
        
        Args:
            msg: Command message
        """
        try:
            data = json.loads(msg.data)
            command = data.get('command')
            
            if command == 'reload':
                # Reload flows
                self.scan_flows()
                
            elif command == 'enable':
                # Enable a flow
                flow_id = data.get('flow_id')
                if flow_id:
                    success = self.manager.enable_flow(flow_id)
                    self.publish_event('flow_enabled', {
                        'flow_id': flow_id,
                        'success': success
                    })
                    
            elif command == 'disable':
                # Disable a flow
                flow_id = data.get('flow_id')
                if flow_id:
                    success = self.manager.disable_flow(flow_id)
                    self.publish_event('flow_disabled', {
                        'flow_id': flow_id,
                        'success': success
                    })
                    
            elif command == 'status':
                # Publish current status
                self.publish_status()
                
            elif command == 'stats':
                # Publish statistics
                stats = self.manager.get_stats()
                self.publish_event('statistics', stats)
                
            else:
                self.get_logger().warning(f"Unknown command: {command}")
                
        except Exception as e:
            self.get_logger().error(f"Error handling command: {e}")
    
    def publish_status(self):
        """Publish current system status"""
        try:
            flows = self.manager.get_all_flows()
            stats = self.manager.get_stats()
            
            status = {
                'timestamp': datetime.now().isoformat(),
                'flows': flows,
                'stats': stats
            }
            
            msg = String()
            msg.data = json.dumps(status)
            self.status_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"Error publishing status: {e}")
    
    def publish_event(self, event_type: str, data: Dict[str, Any]):
        """Publish an event
        
        Args:
            event_type: Type of event
            data: Event data
        """
        try:
            event = {
                'type': event_type,
                'timestamp': datetime.now().isoformat(),
                'data': data
            }
            
            msg = String()
            msg.data = json.dumps(event)
            self.event_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"Error publishing event: {e}")
    
    def cleanup(self):
        """Clean up resources gracefully"""
        self.get_logger().info("Starting graceful shutdown...")
        
        # Set shutdown flag
        self.is_shutting_down = True
        self.shutdown_event.set()
        
        # Cancel timers
        if hasattr(self, 'scan_timer'):
            self.scan_timer.cancel()
            self.destroy_timer(self.scan_timer)
        
        if hasattr(self, 'execute_timer'):
            self.execute_timer.cancel()
            self.destroy_timer(self.execute_timer)
        
        # Stop async loop
        if hasattr(self, 'loop') and self.loop.is_running():
            self.loop.call_soon_threadsafe(self.loop.stop)
            if hasattr(self, 'async_thread'):
                self.async_thread.join(timeout=2)
        
        # Shutdown manager
        if hasattr(self, 'manager'):
            self.manager.shutdown()
        
        # Destroy publishers and subscriptions
        if hasattr(self, 'status_pub'):
            self.destroy_publisher(self.status_pub)
        if hasattr(self, 'event_pub'):
            self.destroy_publisher(self.event_pub)
        if hasattr(self, 'trigger_sub'):
            self.destroy_subscription(self.trigger_sub)
        if hasattr(self, 'command_sub'):
            self.destroy_subscription(self.command_sub)
        
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
        node = TAFLWCSNode()
        
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
