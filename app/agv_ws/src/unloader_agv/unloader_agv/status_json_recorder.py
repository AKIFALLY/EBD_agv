#!/usr/bin/env python3
"""
Unloader AGV Status JSON Recorder

用於記錄 Unloader AGV 的完整狀態到 JSON 檔案，包含所有 Context 資訊和車型特定狀態。
統一輸出到 /tmp/ 目錄，方便 agvui 監控系統讀取。
"""

import json
import os
from datetime import datetime
from typing import Dict, Any, Optional


class UnloaderAgvStatusJsonRecorder:
    """Unloader AGV 狀態 JSON 記錄器"""
    
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
        創建完整的 Unloader AGV 狀態資料
        
        Args:
            agv_core_node: UnloaderAgvCoreNode 實例
            
        Returns:
            完整狀態字典
        """
        # 基本 metadata - 從 ROS namespace 提取 AGV 名稱
        ns = agv_core_node.get_namespace() if hasattr(agv_core_node, 'get_namespace') else None
        agv_id = ns.strip("/") if ns else "unloader01"

        status = {
            "metadata": {
                "agv_id": agv_id,
                "agv_type": "unloader",
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
        
        # Unloader Context (AGV 層)
        if hasattr(agv_core_node, 'unloader_context'):
            unloader_ctx = agv_core_node.unloader_context
            status["contexts"]["unloader_context"] = {
                "current_state": unloader_ctx.state.__class__.__name__ if unloader_ctx.state else "Unknown",
                "state_module": unloader_ctx.state.__class__.__module__ if unloader_ctx.state else "",
                "context_class": unloader_ctx.__class__.__name__
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
                    "oven_upper_port": getattr(param, 'oven_upper_port', None),
                    "oven_lower_port": getattr(param, 'oven_lower_port', None),
                    "pre_dryer_port": getattr(param, 'pre_dryer_port', None),
                    "boxout_port": getattr(param, 'boxout_port', None),
                    "agv_carrier_port": getattr(param, 'agv_carrier_port', None)
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
        
        # Unloader 特有狀態
        unloader_specific = {
            "batch_processing": {},
            "station_status": {},
            "work_id": None,
            "task_progress": {}
        }
        
        # 批量處理狀態 (一次2格)
        if hasattr(agv_core_node, 'unloader_context'):
            ctx = agv_core_node.unloader_context
            unloader_specific["batch_processing"] = {
                "batch_size": 2,  # 一次處理2格
                "current_batch": getattr(ctx, 'current_batch', 0),
                "total_batches": getattr(ctx, 'total_batches', 0),
                "s_size_mode": getattr(ctx, 's_size_mode', True),  # S尺寸使用全部4格
                "l_size_mode": getattr(ctx, 'l_size_mode', False)  # L尺寸只使用上排2格
            }
        
        # 工位狀態
        unloader_specific["station_status"] = {
            "pre_dryer": {
                "port1": False,
                "port2": False,
                "port3": False,
                "port4": False,
                "port5": False,
                "port6": False,
                "port7": False,
                "port8": False
            },
            "oven": {
                "upper_port1": False,  # 上排進料位置
                "upper_port2": False,
                "upper_port3": False,
                "upper_port4": False,
                "lower_port1": False,  # 下排出料位置
                "lower_port2": False,
                "lower_port3": False,
                "lower_port4": False
            },
            "boxout_transfer": {
                "port1": False,
                "port2": False,
                "port3": False,
                "port4": False
            }
        }
        
        # AGV 車載料架狀態 (4格)
        unloader_specific["agv_carrier_status"] = {
            "upper_left": False,   # 上排左格
            "upper_right": False,  # 上排右格
            "lower_left": False,   # 下排左格 (L尺寸不使用)
            "lower_right": False   # 下排右格 (L尺寸不使用)
        }
        
        # Work ID 資訊
        if hasattr(agv_core_node, 'work_id'):
            unloader_specific["work_id"] = agv_core_node.work_id
            # 解析 work_id 來判斷當前任務類型
            if agv_core_node.work_id:
                try:
                    # work_id 格式: room_id + equipment_type + station_number + action_type
                    work_str = str(agv_core_node.work_id)
                    if len(work_str) >= 6:
                        equipment_type = work_str[-4:-2]
                        action_type = work_str[-2:]
                        equipment_map = {
                            "02": "BOX_OUT_TRANSFER",
                            "05": "PRE_DRYER",
                            "06": "OVEN"
                        }
                        action_map = {
                            "01": "TAKE",
                            "02": "PUT"
                        }
                        unloader_specific["task_progress"]["equipment"] = equipment_map.get(equipment_type, "UNKNOWN")
                        unloader_specific["task_progress"]["action"] = action_map.get(action_type, "UNKNOWN")
                        
                        # 特殊處理烘箱上下排資訊
                        if equipment_type == "06":  # OVEN
                            if action_type == "01":  # TAKE
                                unloader_specific["task_progress"]["oven_position"] = "LOWER"  # 從下排取料
                            elif action_type == "02":  # PUT
                                unloader_specific["task_progress"]["oven_position"] = "UPPER"  # 放到上排
                except Exception:
                    pass
        
        status["type_specific"] = unloader_specific
        
        return status
    
    def save_complete_status_to_file(self, agv_core_node, filename: str = None) -> str:
        """
        保存完整狀態到檔案
        
        Args:
            agv_core_node: UnloaderAgvCoreNode 實例
            filename: 檔案名稱，如果為 None 則自動生成
            
        Returns:
            保存的檔案路徑
        """
        # 如果沒有指定檔案名，使用預設格式
        if filename is None:
            # 從 ROS namespace 提取 AGV 名稱（字符串），而不是使用 agv_id（可能是整數）
            ns = agv_core_node.get_namespace() if hasattr(agv_core_node, 'get_namespace') else None
            agv_name = ns.strip("/") if ns else "unloader01"
            filename = f"agv_status_{agv_name}.json"
        
        complete_status = self.create_complete_status(agv_core_node)
        filepath = os.path.join(self.output_dir, filename)
        
        try:
            with open(filepath, 'w', encoding='utf-8') as f:
                json.dump(complete_status, f, indent=2, ensure_ascii=False)
            return filepath
        except Exception as e:
            raise Exception(f"Failed to save Unloader AGV status to {filepath}: {str(e)}")