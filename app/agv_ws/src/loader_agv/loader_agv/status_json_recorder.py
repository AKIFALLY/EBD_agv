#!/usr/bin/env python3
"""
Loader AGV Status JSON Recorder

用於記錄 Loader AGV 的完整狀態到 JSON 檔案，包含所有 Context 資訊和車型特定狀態。
統一輸出到 /tmp/ 目錄，方便 agvui 監控系統讀取。
"""

import json
import os
from datetime import datetime
from typing import Dict, Any, Optional


class LoaderAgvStatusJsonRecorder:
    """Loader AGV 狀態 JSON 記錄器"""
    
    def __init__(self, output_dir: str = "/tmp"):
        """
        初始化記錄器
        
        Args:
            output_dir: 輸出目錄，預設為 /tmp
        """
        self.output_dir = output_dir
        
        # 確保輸出目錄存在
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
    
    def create_complete_status(self, agv_core_node) -> Dict[str, Any]:
        """
        創建完整的 Loader AGV 狀態資料
        
        Args:
            agv_core_node: LoaderAgvCoreNode 實例
            
        Returns:
            完整狀態字典
        """
        # 基本 metadata
        agv_id = agv_core_node.agv_id if hasattr(agv_core_node, 'agv_id') else "loader01"
        
        status = {
            "metadata": {
                "agv_id": agv_id,
                "agv_type": "loader",
                "timestamp": datetime.now().isoformat(),
                "version": "2.0"
            },
            "contexts": {},
            "agv_status": {},
            "type_specific": {}
        }
        
        # Base Context
        if hasattr(agv_core_node, 'base_context'):
            base_ctx = agv_core_node.base_context
            status["contexts"]["base_context"] = {
                "current_state": base_ctx.state.__class__.__name__ if base_ctx.state else "Unknown",
                "state_module": base_ctx.state.__class__.__module__ if base_ctx.state else "",
                "context_class": base_ctx.__class__.__name__
            }
        
        # Loader Context (AGV 層)
        if hasattr(agv_core_node, 'loader_context'):
            loader_ctx = agv_core_node.loader_context
            status["contexts"]["loader_context"] = {
                "current_state": loader_ctx.state.__class__.__name__ if loader_ctx.state else "Unknown",
                "state_module": loader_ctx.state.__class__.__module__ if loader_ctx.state else "",
                "context_class": loader_ctx.__class__.__name__
            }
        
        # Robot Context
        if hasattr(agv_core_node, 'robot_context'):
            robot_ctx = agv_core_node.robot_context
            status["contexts"]["robot_context"] = {
                "current_state": robot_ctx.state.__class__.__name__ if robot_ctx.state else "Unknown",
                "state_module": robot_ctx.state.__class__.__module__ if robot_ctx.state else "",
                "context_class": robot_ctx.__class__.__name__,
                "step": getattr(robot_ctx, 'step', 0),
                "pgno": getattr(robot_ctx, 'pgno', "")
            }
            
            # 如果有 Robot Parameter，加入端口和工位資訊
            if hasattr(robot_ctx, 'robot_parameter'):
                param = robot_ctx.robot_parameter
                status["contexts"]["robot_context"]["parameters"] = {
                    "loader_agv_port_front": getattr(param, 'loader_agv_port_front', None),
                    "loader_agv_port_back": getattr(param, 'loader_agv_port_back', None),
                    "boxin_port": getattr(param, 'boxin_port', None),
                    "cleaner_port": getattr(param, 'cleaner_port', None),
                    "soaker_port": getattr(param, 'soaker_port', None),
                    "pre_dryer_port": getattr(param, 'pre_dryer_port', None)
                }
        
        # 通用 AGV 狀態
        if hasattr(agv_core_node, 'agv'):
            agv = agv_core_node.agv
            status["agv_status"] = {
                "AGV_ID": agv_id,
                "POWER": getattr(agv, 'POWER', 0.0),
                "AGV_Auto": getattr(agv, 'AGV_Auto', False),
                "AGV_MANUAL": getattr(agv, 'AGV_MANUAL', False),
                "AGV_IDLE": getattr(agv, 'AGV_IDLE', False),
                "AGV_ALARM": getattr(agv, 'AGV_ALARM', False),
                "AGV_MOVING": getattr(agv, 'AGV_MOVING', False),
                "X_DIST": getattr(agv, 'X_DIST', 0.0),
                "Y_DIST": getattr(agv, 'Y_DIST', 0.0),
                "THETA": getattr(agv, 'THETA', 0.0)
            }
        
        # Loader 特有狀態
        loader_specific = {
            "agv_ports": {},
            "station_status": {},
            "work_id": None,
            "task_progress": {}
        }
        
        # AGV 端口狀態 (4個端口)
        if hasattr(agv_core_node, 'loader_context'):
            ctx = agv_core_node.loader_context
            loader_specific["agv_ports"] = {
                "port1": getattr(ctx, 'agv_port1', False),
                "port2": getattr(ctx, 'agv_port2', False),
                "port3": getattr(ctx, 'agv_port3', False),
                "port4": getattr(ctx, 'agv_port4', False)
            }
        
        # 工位狀態
        loader_specific["station_status"] = {
            "transfer": {
                "boxin_port1": False,
                "boxin_port2": False,
                "boxin_port3": False,
                "boxin_port4": False
            },
            "cleaner": {
                "cleaner_port1": False,
                "cleaner_port2": False
            },
            "soaker": {
                "soaker_port1": False,
                "soaker_port2": False,
                "soaker_port3": False,
                "soaker_port4": False,
                "soaker_port5": False,
                "soaker_port6": False
            },
            "pre_dryer": {
                "pre_dryer_port1": False,
                "pre_dryer_port2": False,
                "pre_dryer_port3": False,
                "pre_dryer_port4": False,
                "pre_dryer_port5": False,
                "pre_dryer_port6": False,
                "pre_dryer_port7": False,
                "pre_dryer_port8": False
            }
        }
        
        # Work ID 資訊
        if hasattr(agv_core_node, 'work_id'):
            loader_specific["work_id"] = agv_core_node.work_id
            # 解析 work_id 來判斷當前任務類型
            if agv_core_node.work_id:
                try:
                    # work_id 格式: room_id + equipment_type + station_number + action_type
                    work_str = str(agv_core_node.work_id)
                    if len(work_str) >= 6:
                        equipment_type = work_str[-4:-2]
                        action_type = work_str[-2:]
                        equipment_map = {
                            "01": "TRANSFER",
                            "03": "CLEANER", 
                            "04": "SOAKER",
                            "05": "PRE_DRYER"
                        }
                        action_map = {
                            "01": "TAKE",
                            "02": "PUT"
                        }
                        loader_specific["task_progress"]["equipment"] = equipment_map.get(equipment_type, "UNKNOWN")
                        loader_specific["task_progress"]["action"] = action_map.get(action_type, "UNKNOWN")
                except Exception:
                    pass
        
        status["type_specific"] = loader_specific
        
        return status
    
    def save_complete_status_to_file(self, agv_core_node, filename: str = None) -> str:
        """
        保存完整狀態到檔案
        
        Args:
            agv_core_node: LoaderAgvCoreNode 實例
            filename: 檔案名稱，如果為 None 則自動生成
            
        Returns:
            保存的檔案路徑
        """
        # 如果沒有指定檔案名，使用預設格式
        if filename is None:
            agv_id = agv_core_node.agv_id if hasattr(agv_core_node, 'agv_id') else "loader01"
            filename = f"agv_status_{agv_id}.json"
        
        complete_status = self.create_complete_status(agv_core_node)
        filepath = os.path.join(self.output_dir, filename)
        
        try:
            with open(filepath, 'w', encoding='utf-8') as f:
                json.dump(complete_status, f, indent=2, ensure_ascii=False)
            return filepath
        except Exception as e:
            raise Exception(f"Failed to save Loader AGV status to {filepath}: {str(e)}")