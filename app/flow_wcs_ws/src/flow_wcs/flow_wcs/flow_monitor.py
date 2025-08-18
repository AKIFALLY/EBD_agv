#!/usr/bin/env python3
"""
Flow Monitor - Monitoring service for flow execution
監控流程執行狀態和效能指標
"""

import asyncio
import json
import time
import threading
from typing import Dict, List, Any, Optional
from datetime import datetime, timedelta
from pathlib import Path
import psutil


class FlowMonitor:
    """Flow execution monitor"""
    
    def __init__(self, scan_interval: float = 5.0):
        self.scan_interval = scan_interval
        self.active_flows = {}
        self.execution_history = []
        self.metrics = {
            'total_executed': 0,
            'total_succeeded': 0,
            'total_failed': 0,
            'average_duration': 0.0,
            'current_active': 0
        }
        self.running = False
        self.monitor_thread = None
        
    def start_monitoring(self):
        """Start monitoring loop"""
        self.running = True
        print(f"Flow Monitor started (scan interval: {self.scan_interval}s)")
        
        while self.running:
            try:
                self.update_metrics()
                self.check_stuck_flows()
                self.cleanup_history()
                time.sleep(self.scan_interval)
            except Exception as e:
                print(f"Monitor error: {e}")
                time.sleep(1)
    
    def stop_monitoring(self):
        """Stop monitoring loop"""
        self.running = False
        if self.monitor_thread and self.monitor_thread.is_alive():
            self.monitor_thread.join(timeout=5)
        print("Flow Monitor stopped")
    
    def register_flow_start(self, flow_id: str, flow_name: str, work_id: str):
        """Register a flow execution start"""
        self.active_flows[flow_id] = {
            'flow_id': flow_id,
            'flow_name': flow_name,
            'work_id': work_id,
            'start_time': datetime.now(),
            'status': 'running',
            'current_section': None,
            'current_step': None,
            'steps_completed': 0,
            'errors': []
        }
        self.metrics['current_active'] = len(self.active_flows)
        print(f"Flow started: {flow_id} ({flow_name})")
    
    def register_flow_complete(self, flow_id: str, status: str = 'completed'):
        """Register a flow execution completion"""
        if flow_id in self.active_flows:
            flow_info = self.active_flows[flow_id]
            flow_info['end_time'] = datetime.now()
            flow_info['status'] = status
            flow_info['duration'] = (flow_info['end_time'] - flow_info['start_time']).total_seconds()
            
            # Update metrics
            self.metrics['total_executed'] += 1
            if status == 'completed':
                self.metrics['total_succeeded'] += 1
            else:
                self.metrics['total_failed'] += 1
            
            # Move to history
            self.execution_history.append(flow_info)
            del self.active_flows[flow_id]
            self.metrics['current_active'] = len(self.active_flows)
            
            # Update average duration
            self.update_average_duration()
            
            print(f"Flow {status}: {flow_id} (duration: {flow_info['duration']:.2f}s)")
    
    def update_flow_progress(self, flow_id: str, section: str = None, step: str = None):
        """Update flow execution progress"""
        if flow_id in self.active_flows:
            flow_info = self.active_flows[flow_id]
            if section:
                flow_info['current_section'] = section
            if step:
                flow_info['current_step'] = step
                flow_info['steps_completed'] += 1
            flow_info['last_update'] = datetime.now()
    
    def add_flow_error(self, flow_id: str, error: str):
        """Add error to flow execution"""
        if flow_id in self.active_flows:
            self.active_flows[flow_id]['errors'].append({
                'error': error,
                'timestamp': datetime.now().isoformat()
            })
    
    def update_metrics(self):
        """Update monitoring metrics"""
        # Check system resources
        cpu_percent = psutil.cpu_percent(interval=1)
        memory = psutil.virtual_memory()
        
        self.metrics.update({
            'cpu_usage': cpu_percent,
            'memory_usage': memory.percent,
            'memory_available': memory.available / (1024 * 1024 * 1024),  # GB
            'timestamp': datetime.now().isoformat()
        })
    
    def check_stuck_flows(self, timeout_minutes: int = 10):
        """Check for stuck or timed-out flows"""
        current_time = datetime.now()
        timeout_delta = timedelta(minutes=timeout_minutes)
        
        for flow_id, flow_info in list(self.active_flows.items()):
            if current_time - flow_info['start_time'] > timeout_delta:
                print(f"Flow timeout detected: {flow_id}")
                self.register_flow_complete(flow_id, 'timeout')
    
    def cleanup_history(self, max_history: int = 100):
        """Cleanup old execution history"""
        if len(self.execution_history) > max_history:
            # Keep only the most recent entries
            self.execution_history = self.execution_history[-max_history:]
    
    def update_average_duration(self):
        """Update average flow duration metric"""
        if self.execution_history:
            durations = [f.get('duration', 0) for f in self.execution_history[-20:]]  # Last 20 flows
            self.metrics['average_duration'] = sum(durations) / len(durations) if durations else 0
    
    def get_flow_status(self, flow_id: str) -> Optional[Dict]:
        """Get status of a specific flow"""
        if flow_id in self.active_flows:
            return self.active_flows[flow_id]
        
        # Check history
        for flow in reversed(self.execution_history):
            if flow['flow_id'] == flow_id:
                return flow
        
        return None
    
    def get_active_flows(self) -> List[Dict]:
        """Get list of currently active flows"""
        return list(self.active_flows.values())
    
    def get_metrics(self) -> Dict:
        """Get current monitoring metrics"""
        return self.metrics.copy()
    
    def get_execution_summary(self, hours: int = 1) -> Dict:
        """Get execution summary for the last N hours"""
        cutoff_time = datetime.now() - timedelta(hours=hours)
        recent_flows = [
            f for f in self.execution_history
            if f.get('start_time', datetime.min) > cutoff_time
        ]
        
        summary = {
            'period_hours': hours,
            'total_flows': len(recent_flows),
            'succeeded': sum(1 for f in recent_flows if f.get('status') == 'completed'),
            'failed': sum(1 for f in recent_flows if f.get('status') == 'failed'),
            'timeout': sum(1 for f in recent_flows if f.get('status') == 'timeout'),
            'average_duration': 0.0,
            'flows': recent_flows
        }
        
        if recent_flows:
            durations = [f.get('duration', 0) for f in recent_flows if 'duration' in f]
            if durations:
                summary['average_duration'] = sum(durations) / len(durations)
        
        return summary
    
    def export_metrics(self, filepath: str = '/tmp/flow_metrics.json'):
        """Export metrics to JSON file"""
        data = {
            'timestamp': datetime.now().isoformat(),
            'metrics': self.metrics,
            'active_flows': list(self.active_flows.values()),
            'recent_history': self.execution_history[-50:]  # Last 50 executions
        }
        
        with open(filepath, 'w') as f:
            json.dump(data, f, indent=2, default=str)
        
        print(f"Metrics exported to {filepath}")
        return filepath


def main(args=None):
    """Main entry point for flow monitor"""
    import sys
    
    # Parse arguments if needed
    scan_interval = 5.0
    if args and len(args) > 1:
        try:
            scan_interval = float(args[1])
        except ValueError:
            pass
    
    try:
        monitor = FlowMonitor(scan_interval=scan_interval)
        
        # Start monitoring in a separate thread
        monitor_thread = threading.Thread(target=monitor.start_monitoring)
        monitor_thread.daemon = True
        monitor_thread.start()
        
        print("Flow Monitor is running. Press Ctrl+C to stop.")
        
        # Keep running
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\nShutting down Flow Monitor...")
        monitor.stop_monitoring()
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)


if __name__ == '__main__':
    import sys
    main(sys.argv)