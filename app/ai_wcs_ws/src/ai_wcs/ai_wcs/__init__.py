"""
AI-driven Warehouse Control System for intelligent Rack management

This package implements a sophisticated WCS system based on ROS 2 Jazzy with:
- Four-tier priority decision engine
- Intelligent Rack state analysis 
- Process validation and compatibility checking
- Multi-Rack scheduling with conflict resolution
- Exception handling and deadlock prevention

Core modules:
- ai_wcs_node: Main WCS coordination node
- rack_analyzer: Rack state analysis and A/B side management
- decision_engine: Four-tier priority task scheduling
- task_manager: Task generation and lifecycle management
- database_client: Database integration and query operations
"""

__version__ = "0.0.0"
__author__ = "Ching Tech Team"
__description__ = "AI-driven Warehouse Control System for intelligent Rack management"