#!/usr/bin/env python3
"""
TAFL WCS Manager - Manages TAFL flow loading, caching, and execution
"""

import os
import yaml
import asyncio
from pathlib import Path
from typing import Dict, List, Any, Optional
from datetime import datetime
import hashlib
from concurrent.futures import ThreadPoolExecutor

from .tafl_executor_wrapper import TAFLExecutorWrapper


class TAFLWCSManager:
    """Manager for TAFL WCS flows"""
    
    def __init__(self, flows_dir: str = None, database_url: str = None, logger=None):
        """Initialize TAFL WCS Manager
        
        Args:
            flows_dir: Directory containing TAFL flow files
            database_url: Database connection string
            logger: Logger instance
        """
        self.logger = logger
        self.flows_dir = Path(flows_dir or '/app/config/tafl/flows')
        self.database_url = database_url
        
        # Flow cache
        self.loaded_flows = {}
        self.flow_checksums = {}
        self.active_executions = {}

        # Track last execution time for each flow
        self.flow_last_execution = {}

        # Executor
        self.executor_wrapper = TAFLExecutorWrapper(database_url, logger)
        
        # Thread pool for parallel execution
        self.thread_pool = ThreadPoolExecutor(max_workers=5)
        
        # Ensure flows directory exists
        self.flows_dir.mkdir(parents=True, exist_ok=True)
        if self.logger:
            self.logger.info(f"TAFL WCS Manager initialized with flows_dir: {self.flows_dir}")
    
    def scan_flows(self, only_enabled: bool = True) -> List[str]:
        """Scan flows directory and load TAFL files
        
        Args:
            only_enabled: If True, only load flows with enabled=true
        
        Returns:
            List of loaded flow IDs
        """
        if not self.flows_dir.exists():
            if self.logger:
                self.logger.warning(f"Flows directory not found: {self.flows_dir}")
            return []
        
        loaded_flow_ids = []
        yaml_files = list(self.flows_dir.glob("*.yaml")) + list(self.flows_dir.glob("*.yml"))
        
        if self.logger:
            self.logger.info(f"Found {len(yaml_files)} YAML files in {self.flows_dir}")
        
        for flow_file in yaml_files:
            try:
                # Read file content
                content = flow_file.read_text(encoding='utf-8')
                
                # Calculate checksum
                checksum = hashlib.md5(content.encode()).hexdigest()
                
                # Parse YAML to get metadata
                flow_data = yaml.safe_load(content)
                
                if self._validate_tafl_structure(flow_data):
                    flow_id = flow_data.get('metadata', {}).get('id', flow_file.stem)
                    is_enabled = flow_data.get('metadata', {}).get('enabled', True)
                    
                    # Skip disabled flows if only_enabled is True
                    if only_enabled and not is_enabled:
                        if self.logger:
                            self.logger.debug(f"Skipping disabled flow: {flow_id}")
                        # Remove from loaded_flows if it was previously loaded
                        if flow_id in self.loaded_flows:
                            del self.loaded_flows[flow_id]
                            if self.logger:
                                self.logger.info(f"Removed disabled flow from cache: {flow_id}")
                        continue
                    
                    # Check if flow needs reloading
                    if flow_id not in self.flow_checksums or self.flow_checksums[flow_id] != checksum:
                        self.loaded_flows[flow_id] = {
                            'id': flow_id,
                            'name': flow_data.get('metadata', {}).get('name', flow_id),
                            'description': flow_data.get('metadata', {}).get('description', ''),
                            'enabled': is_enabled,
                            'content': content,
                            'data': flow_data,
                            'file': str(flow_file),
                            'checksum': checksum,
                            'loaded_at': datetime.now().isoformat()
                        }
                        self.flow_checksums[flow_id] = checksum
                        loaded_flow_ids.append(flow_id)
                        
                        status = "enabled" if is_enabled else "disabled"
                        if self.logger:
                            self.logger.info(f"Loaded {status} TAFL flow: {flow_id}")
                else:
                    if self.logger:
                        self.logger.warning(f"Invalid TAFL structure in {flow_file}")
                        
            except Exception as e:
                if self.logger:
                    self.logger.error(f"Failed to load flow {flow_file}: {e}")
        
        return loaded_flow_ids
    
    def _validate_tafl_structure(self, flow_data: Dict) -> bool:
        """Validate TAFL flow structure according to v1.1 specification
        
        Args:
            flow_data: Parsed YAML data
        
        Returns:
            True if valid TAFL v1.1 structure
        """
        # Check for TAFL v1.1 6-section structure
        if not isinstance(flow_data, dict):
            return False
        
        # Required sections
        if 'metadata' not in flow_data:
            if self.logger:
                self.logger.warning("Missing required 'metadata' section")
            return False
        
        if 'flow' not in flow_data:
            if self.logger:
                self.logger.warning("Missing required 'flow' section")
            return False
        
        # Metadata must have id and name
        metadata = flow_data.get('metadata', {})
        if not metadata.get('id'):
            if self.logger:
                self.logger.warning("Missing required 'id' in metadata")
            return False
        
        if not metadata.get('name'):
            if self.logger:
                self.logger.warning("Missing required 'name' in metadata")
            return False
        
        # Optional but validated sections (TAFL v1.1)
        valid_sections = {
            'metadata',    # Required: Program metadata
            'settings',    # Optional: Execution settings
            'preload',     # Optional: Data preloading (Phase 1)
            'rules',       # Optional: Rule definitions (Phase 2)
            'variables',   # Optional: Variable initialization (Phase 3)
            'flow'        # Required: Main flow execution (Phase 4)
        }
        
        # Check for unknown sections
        for section in flow_data.keys():
            if section not in valid_sections:
                if self.logger:
                    self.logger.warning(f"Unknown section '{section}' in TAFL flow. Valid sections: {valid_sections}")
        
        # Log detected TAFL version
        if 'preload' in flow_data or 'rules' in flow_data:
            if self.logger:
                self.logger.info("Detected TAFL v1.1+ flow with preload/rules sections")
        
        return True
    
    async def execute_flow(self, flow_id: str) -> Dict[str, Any]:
        """Execute a TAFL flow by ID
        
        Args:
            flow_id: Flow identifier
        
        Returns:
            Execution result
        """
        if flow_id not in self.loaded_flows:
            return {
                'status': 'failed',
                'error': f"Flow '{flow_id}' not found",
                'flow_id': flow_id
            }
        
        if flow_id in self.active_executions:
            return {
                'status': 'skipped',
                'reason': 'Flow is already executing',
                'flow_id': flow_id
            }
        
        flow = self.loaded_flows[flow_id]
        
        # Check if flow is enabled
        if not flow.get('enabled', True):
            return {
                'status': 'skipped',
                'reason': 'Flow is disabled',
                'flow_id': flow_id
            }
        
        # Mark as active
        self.active_executions[flow_id] = {
            'started_at': datetime.now().isoformat(),
            'status': 'running'
        }
        
        try:
            # Execute flow
            if self.logger:
                self.logger.info(f"Starting execution of flow: {flow_id}")
            
            result = await self.executor_wrapper.execute_flow(
                flow['content'],
                flow_id
            )
            
            # Update active executions
            self.active_executions[flow_id].update({
                'completed_at': datetime.now().isoformat(),
                'status': result.get('status', 'unknown'),
                'result': result
            })
            
            return result
            
        except Exception as e:
            if self.logger:
                self.logger.error(f"Flow execution failed for {flow_id}: {e}")
            
            self.active_executions[flow_id].update({
                'completed_at': datetime.now().isoformat(),
                'status': 'failed',
                'error': str(e)
            })
            
            return {
                'status': 'failed',
                'error': str(e),
                'flow_id': flow_id
            }
        finally:
            # Remove from active after a delay
            await asyncio.sleep(5)
            if flow_id in self.active_executions:
                del self.active_executions[flow_id]
    
    async def execute_all_enabled_flows(self) -> List[Dict[str, Any]]:
        """Execute all enabled flows
        
        Returns:
            List of execution results
        """
        results = []
        
        for flow_id, flow in self.loaded_flows.items():
            if flow.get('enabled', True):
                result = await self.execute_flow(flow_id)
                results.append(result)
        
        return results
    
    def execute_flow_sync(self, flow_id: str) -> Dict[str, Any]:
        """Execute a TAFL flow synchronously (like RCS dispatch)
        
        Args:
            flow_id: Flow identifier
        
        Returns:
            Execution result
        """
        if flow_id not in self.loaded_flows:
            return {
                'status': 'failed',
                'error': f"Flow '{flow_id}' not found",
                'flow_id': flow_id
            }
        
        flow = self.loaded_flows[flow_id]
        
        # Check if flow is enabled
        if not flow.get('enabled', True):
            return {
                'status': 'skipped',
                'reason': 'Flow is disabled',
                'flow_id': flow_id
            }
        
        # Check if already executing
        if flow_id in self.active_executions:
            return {
                'status': 'skipped',
                'reason': 'Flow is already executing',
                'flow_id': flow_id
            }
        
        # Mark as active
        self.active_executions[flow_id] = {
            'started_at': datetime.now().isoformat(),
            'status': 'running'
        }
        
        try:
            # Execute flow synchronously
            if self.logger:
                self.logger.info(f"Starting synchronous execution of flow: {flow_id}")
            
            # Use executor_wrapper's synchronous execution
            result = self.executor_wrapper.execute_flow_sync(
                flow['content'],
                flow_id
            )
            
            # Update active executions
            self.active_executions[flow_id].update({
                'completed_at': datetime.now().isoformat(),
                'status': result.get('status', 'unknown'),
                'result': result
            })
            
            return result
            
        except Exception as e:
            if self.logger:
                self.logger.error(f"Flow execution failed for {flow_id}: {e}")
            
            self.active_executions[flow_id].update({
                'completed_at': datetime.now().isoformat(),
                'status': 'failed',
                'error': str(e)
            })
            
            return {
                'status': 'failed',
                'error': str(e),
                'flow_id': flow_id
            }
        finally:
            # Clean up active execution after a short delay
            # Note: In sync mode, we just remove it immediately
            if flow_id in self.active_executions:
                del self.active_executions[flow_id]
    
    def get_flow_status(self, flow_id: str) -> Dict[str, Any]:
        """Get flow status
        
        Args:
            flow_id: Flow identifier
        
        Returns:
            Flow status information
        """
        if flow_id not in self.loaded_flows:
            return {
                'status': 'not_found',
                'flow_id': flow_id
            }
        
        flow = self.loaded_flows[flow_id]
        status = {
            'flow_id': flow_id,
            'name': flow['name'],
            'enabled': flow.get('enabled', True),
            'loaded_at': flow['loaded_at'],
            'file': flow['file']
        }
        
        # Add execution status if active
        if flow_id in self.active_executions:
            status['execution'] = self.active_executions[flow_id]
        else:
            status['execution'] = {'status': 'idle'}
        
        # Add execution stats if available
        stats = self.executor_wrapper.execution_stats.get(flow_id)
        if stats:
            status['last_execution'] = stats
        
        return status
    
    def get_all_flows(self) -> Dict[str, Dict[str, Any]]:
        """Get all loaded flows
        
        Returns:
            Dictionary of flow_id to flow info
        """
        flows = {}
        for flow_id, flow in self.loaded_flows.items():
            flows[flow_id] = {
                'id': flow_id,
                'name': flow['name'],
                'description': flow['description'],
                'enabled': flow.get('enabled', True),
                'file': flow['file'],
                'loaded_at': flow['loaded_at']
            }
        return flows
    
    def enable_flow(self, flow_id: str) -> bool:
        """Enable a flow
        
        Args:
            flow_id: Flow identifier
        
        Returns:
            Success status
        """
        if flow_id in self.loaded_flows:
            self.loaded_flows[flow_id]['enabled'] = True
            return True
        return False
    
    def disable_flow(self, flow_id: str) -> bool:
        """Disable a flow
        
        Args:
            flow_id: Flow identifier
        
        Returns:
            Success status
        """
        if flow_id in self.loaded_flows:
            self.loaded_flows[flow_id]['enabled'] = False
            return True
        return False
    
    def reload_flow(self, flow_id: str) -> bool:
        """Reload a specific flow from disk
        
        Args:
            flow_id: Flow identifier
        
        Returns:
            Success status
        """
        if flow_id in self.loaded_flows:
            flow_file = Path(self.loaded_flows[flow_id]['file'])
            if flow_file.exists():
                try:
                    content = flow_file.read_text(encoding='utf-8')
                    checksum = hashlib.md5(content.encode()).hexdigest()
                    flow_data = yaml.safe_load(content)
                    
                    self.loaded_flows[flow_id].update({
                        'content': content,
                        'data': flow_data,
                        'checksum': checksum,
                        'loaded_at': datetime.now().isoformat()
                    })
                    self.flow_checksums[flow_id] = checksum
                    
                    if self.logger:
                        self.logger.info(f"Reloaded flow: {flow_id}")
                    return True
                except Exception as e:
                    if self.logger:
                        self.logger.error(f"Failed to reload flow {flow_id}: {e}")
        return False
    
    def get_stats(self) -> Dict[str, Any]:
        """Get manager statistics
        
        Returns:
            Statistics dictionary
        """
        return {
            'total_flows': len(self.loaded_flows),
            'enabled_flows': sum(1 for f in self.loaded_flows.values() if f.get('enabled', True)),
            'active_executions': len(self.active_executions),
            'flows_directory': str(self.flows_dir),
            'executor_stats': self.executor_wrapper.get_stats()
        }
    
    def check_and_execute_flows(self, current_time: float, progress_reporter=None):
        """Check each flow and execute if its execution_interval has passed

        Args:
            current_time: Current timestamp
            progress_reporter: Optional progress reporter for execution updates
        """
        # Note: scan_flows is now handled by main_timer_callback every 3 seconds
        # No need to scan here on every execution check

        # Check each enabled flow
        for flow_id, flow in self.loaded_flows.items():
            # Skip disabled flows
            if not flow.get('enabled', True):
                continue

            # Get execution interval from settings (default 5 seconds)
            settings = flow.get('data', {}).get('settings', {})
            execution_interval = settings.get('execution_interval', 5.0)

            # Check if it's time to execute this flow
            last_execution = self.flow_last_execution.get(flow_id, 0)
            time_since_last = current_time - last_execution

            if time_since_last >= execution_interval:
                try:
                    if self.logger:
                        self.logger.info(
                            f'Executing flow {flow_id} (interval: {execution_interval}s, '
                            f'last: {time_since_last:.1f}s ago)'
                        )

                    # Report execution start if reporter available
                    if progress_reporter:
                        progress_reporter.report_progress(
                            flow_id, 0, 100, 'EXECUTING',
                            f'Starting execution (interval: {execution_interval}s)'
                        )

                    # Execute the flow synchronously
                    start_time = datetime.now()
                    result = self.execute_flow_sync(flow_id)
                    execution_time = (datetime.now() - start_time).total_seconds()

                    # Update last execution time
                    self.flow_last_execution[flow_id] = current_time

                    # Report execution result if reporter available
                    if progress_reporter:
                        if result.get('status') == 'completed':
                            progress_reporter.report_progress(
                                flow_id, 100, 100, 'COMPLETED',
                                f'Completed in {execution_time:.2f}s'
                            )
                        elif result.get('status') == 'skipped':
                            progress_reporter.report_progress(
                                flow_id, -1, 100, 'SKIPPED',
                                result.get('reason', 'Flow was skipped')
                            )
                        else:
                            progress_reporter.report_progress(
                                flow_id, -1, 100, 'FAILED',
                                f'Failed: {result.get("error", "Unknown error")}'
                            )

                    if self.logger:
                        self.logger.info(
                            f'Flow {flow_id} {result.get("status", "unknown")} '
                            f'in {execution_time:.2f}s'
                        )

                except Exception as e:
                    if self.logger:
                        self.logger.error(f'Failed to execute flow {flow_id}: {e}')
                    # Update last execution time even on failure to avoid rapid retries
                    self.flow_last_execution[flow_id] = current_time

    def shutdown(self):
        """Shutdown manager and clean up resources"""
        self.thread_pool.shutdown(wait=True)
        self.executor_wrapper.shutdown()
        print("TAFL WCS Manager shutdown complete")
