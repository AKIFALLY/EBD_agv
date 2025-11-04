#!/usr/bin/env python3
"""
Cargo Mover AGV Status JSON Recorder
將 cargo_mover_agv 的所有狀態變數記錄到 JSON 文件中

功能:
- 記錄 AgvCoreNode 的所有 self 變數
- 記錄繼承的 AgvNodebase 變數
- 記錄 3層狀態機 (Base/Cargo/Robot) 的狀態和變數
- 記錄 AgvStatus 的所有 PLC 狀態細項
- 記錄 Robot 和 Hokuyo 設備狀態
- 提供時間戳記和狀態快照功能
"""

import json
import time
from datetime import datetime
from typing import Any, Dict, Optional
import os


class CargoAgvStatusJsonRecorder:
    """Cargo AGV 狀態 JSON 記錄器"""
    
    def __init__(self, output_dir: str = "/tmp/cargo_agv_status"):
        """
        初始化 JSON 記錄器
        
        Args:
            output_dir: JSON 文件輸出目錄
        """
        self.output_dir = output_dir
        self.ensure_output_directory()
        
    def ensure_output_directory(self):
        """確保輸出目錄存在"""
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir, exist_ok=True)
            
    def safe_serialize(self, obj: Any, visited: set = None) -> Any:
        """
        安全地序列化物件，處理無法直接 JSON 序列化的物件
        
        Args:
            obj: 要序列化的物件
            visited: 已訪問物件集合，用於檢測循環引用
            
        Returns:
            可以 JSON 序列化的物件
        """
        if visited is None:
            visited = set()
        
        # 檢測循環引用
        obj_id = id(obj)
        if obj_id in visited:
            return f"<circular_reference: {type(obj).__name__}>"
        
        # 將當前物件添加到已訪問集合（僅對可能產生循環引用的類型）
        if isinstance(obj, (dict, list, tuple)) or hasattr(obj, '__dict__'):
            visited.add(obj_id)
        if obj is None:
            return None
        elif isinstance(obj, (bool, int, float, str)):
            return obj
        elif hasattr(obj, '__iter__') and hasattr(obj, '__next__'):
            # 處理 generator 和 iterator 物件
            try:
                # 將 generator 轉換為列表，但限制數量以避免記憶體問題
                result_list = []
                count = 0
                for item in obj:
                    if count >= 100:  # 限制最多100個項目
                        result_list.append("<truncated: too many items>")
                        break
                    result_list.append(self.safe_serialize(item, visited.copy()))
                    count += 1
                return result_list
            except Exception as e:
                return f"<generator_error: {str(e)}>"
        elif isinstance(obj, (list, tuple)):
            try:
                result_list = []
                for i, item in enumerate(obj):
                    try:
                        result_list.append(self.safe_serialize(item, visited.copy()))
                    except Exception as e:
                        # 如果單個項目序列化失敗，記錄錯誤但不影響其他項目
                        result_list.append(f"<item_{i}_serialization_error: {str(e)}>")
                return result_list
            except Exception as e:
                return f"<list_serialization_error: {str(e)}>"
        elif isinstance(obj, dict):
            try:
                result_dict = {}
                for key, value in obj.items():
                    try:
                        # 確保鍵是字串類型
                        if not isinstance(key, str):
                            key = str(key)
                        # 遞迴序列化值
                        result_dict[key] = self.safe_serialize(value, visited.copy())
                    except Exception as e:
                        # 如果單個項目序列化失敗，記錄錯誤但不影響其他項目
                        result_dict[str(key)] = f"<serialization_error: {str(e)}>"
                return result_dict
            except Exception as e:
                return f"<dict_serialization_error: {str(e)}>"
        elif hasattr(obj, '__dict__'):
            # 處理有 __dict__ 的物件
            try:
                obj_dict = {
                    '_class': obj.__class__.__name__,
                    '_module': obj.__class__.__module__
                }
                
                # 安全地處理 __dict__ 中的每個項目
                for key, value in obj.__dict__.items():
                    try:
                        # 確保鍵是字串類型
                        if not isinstance(key, str):
                            key = str(key)
                        # 遞迴序列化值
                        obj_dict[key] = self.safe_serialize(value, visited.copy())
                    except Exception as e:
                        # 如果單個屬性序列化失敗，記錄錯誤但不影響其他屬性
                        obj_dict[key] = f"<serialization_error: {str(e)}>"
                
                return obj_dict
                
            except Exception as e:
                return f"<{obj.__class__.__name__} object at {hex(id(obj))}, error: {str(e)}>"
        else:
            # 其他類型轉為字串
            try:
                return str(obj)
            except Exception:
                return f"<{type(obj).__name__} object>"
    
    def record_agv_status_details(self, agv_status) -> Dict[str, Any]:
        """
        記錄 AgvStatus 物件的所有細項
        
        Args:
            agv_status: AgvStatus 物件
            
        Returns:
            包含所有 AgvStatus 變數的字典
        """
        if agv_status is None:
            return {}
            
        status_dict = {}
        
        # 獲取 AgvStatus 的所有屬性
        for attr_name in dir(agv_status):
            if not attr_name.startswith('_') and not callable(getattr(agv_status, attr_name)):
                try:
                    value = getattr(agv_status, attr_name)
                    status_dict[attr_name] = self.safe_serialize(value)
                except Exception as e:
                    status_dict[attr_name] = f"Error reading: {str(e)}"
                    
        return status_dict
    
    def record_context_state(self, context, context_name: str) -> Dict[str, Any]:
        """
        記錄狀態機 Context 的狀態和變數
        
        Args:
            context: 狀態機 Context 物件
            context_name: Context 名稱
            
        Returns:
            包含 Context 狀態的字典
        """
        if context is None:
            return {}
            
        context_dict = {
            "context_name": context_name,
            "current_state": {
                "class": context.state.__class__.__name__ if context.state else None,
                "module": context.state.__class__.__module__ if context.state else None,
                "state_info": self.safe_serialize(context.state.__dict__ if context.state else {})
            },
            "context_variables": {}
        }
        
        # 記錄 Context 的所有變數
        for attr_name in dir(context):
            if not attr_name.startswith('_') and not callable(getattr(context, attr_name)):
                if attr_name != 'state':  # state 已經單獨處理
                    try:
                        value = getattr(context, attr_name)
                        context_dict["context_variables"][attr_name] = self.safe_serialize(value)
                    except Exception as e:
                        context_dict["context_variables"][attr_name] = f"Error reading: {str(e)}"
                        
        return context_dict
    
    def record_robot_device_status(self, robot_device, device_name: str) -> Dict[str, Any]:
        """
        記錄 Robot 或 Hokuyo 等設備的狀態
        
        Args:
            robot_device: 設備物件
            device_name: 設備名稱
            
        Returns:
            包含設備狀態的字典
        """
        if robot_device is None:
            return {}
            
        device_dict = {
            "device_name": device_name,
            "device_class": robot_device.__class__.__name__,
            "device_module": robot_device.__class__.__module__,
            "device_variables": {}
        }
        
        # 記錄設備的所有變數
        for attr_name in dir(robot_device):
            if not attr_name.startswith('_') and not callable(getattr(robot_device, attr_name)):
                try:
                    value = getattr(robot_device, attr_name)
                    device_dict["device_variables"][attr_name] = self.safe_serialize(value)
                except Exception as e:
                    device_dict["device_variables"][attr_name] = f"Error reading: {str(e)}"
                    
        return device_dict
    
    def record_core_node_variables(self, agv_core_node) -> Dict[str, Any]:
        """
        記錄 AgvCoreNode 的所有 self 變數
        
        Args:
            agv_core_node: AgvCoreNode 實例
            
        Returns:
            包含所有核心節點變數的字典
        """
        if agv_core_node is None:
            return {}
            
        core_variables = {}
        
        # 記錄 AgvCoreNode 的所有變數
        for attr_name in dir(agv_core_node):
            if not attr_name.startswith('_') and not callable(getattr(agv_core_node, attr_name)):
                # 跳過一些特殊處理的屬性，這些會在其他地方專門處理
                if attr_name in ['agv_status', 'base_context', 'cargo_context', 'robot_context', 
                               'robot', 'hokuyo_dms_8bit_1', 'hokuyo_dms_8bit_2']:
                    continue
                    
                try:
                    value = getattr(agv_core_node, attr_name)
                    core_variables[attr_name] = self.safe_serialize(value)
                except Exception as e:
                    core_variables[attr_name] = f"Error reading: {str(e)}"
                    
        return core_variables
    
    def create_complete_status_snapshot(self, agv_core_node) -> Dict[str, Any]:
        """
        創建完整的狀態快照
        
        Args:
            agv_core_node: AgvCoreNode 實例
            
        Returns:
            完整的狀態快照字典
        """
        timestamp = datetime.now()
        
        snapshot = {
            "metadata": {
                "timestamp": timestamp.isoformat(),
                "timestamp_unix": time.time(),
                "recorder_version": "1.0.0",
                "agv_type": "cargo_mover_agv"
            },
            "agv_core_node": {
                "class_info": {
                    "class": agv_core_node.__class__.__name__,
                    "module": agv_core_node.__class__.__module__,
                    "base_classes": [base.__name__ for base in agv_core_node.__class__.__mro__[1:]]
                },
                "variables": self.record_core_node_variables(agv_core_node)
            },
            "agv_status": {
                "description": "PLC 狀態和所有細項",
                "variables": self.record_agv_status_details(getattr(agv_core_node, 'agv_status', None))
            },
            "state_machines": {
                "base_context": self.record_context_state(
                    getattr(agv_core_node, 'base_context', None), "BaseContext"
                ),
                "cargo_context": self.record_context_state(
                    getattr(agv_core_node, 'cargo_context', None), "CargoContext"
                ),
                "robot_context": self.record_context_state(
                    getattr(agv_core_node, 'robot_context', None), "RobotContext"
                )
            },
            "devices": {
                "robot": self.record_robot_device_status(
                    getattr(agv_core_node, 'robot', None), "Robot"
                ),
                "hokuyo_dms_8bit_1": self.record_robot_device_status(
                    getattr(agv_core_node, 'hokuyo_dms_8bit_1', None), "HokuyoDMS8Bit_1"
                ),
                "hokuyo_dms_8bit_2": self.record_robot_device_status(
                    getattr(agv_core_node, 'hokuyo_dms_8bit_2', None), "HokuyoDMS8Bit_2"
                )
            }
        }
        
        return snapshot
    
    def save_status_to_file(self, agv_core_node, filename: Optional[str] = None) -> str:
        """
        將狀態保存到 JSON 文件
        
        Args:
            agv_core_node: AgvCoreNode 實例
            filename: 可選的文件名，如果未提供則使用時間戳
            
        Returns:
            保存的文件路徑
        """
        snapshot = self.create_complete_status_snapshot(agv_core_node)
        
        if filename is None:
            # 從 ROS namespace 提取 AGV 名稱（字符串），而不是使用 agv_id（可能是整數）
            ns = agv_core_node.get_namespace() if hasattr(agv_core_node, 'get_namespace') else None
            agv_name = ns.strip("/") if ns else "cargo01"
            filename = f"agv_status_{agv_name}.json"
            
        filepath = os.path.join(self.output_dir, filename)
        
        try:
            with open(filepath, 'w', encoding='utf-8') as f:
                json.dump(snapshot, f, indent=2, ensure_ascii=False)
            return filepath
        except Exception as e:
            raise Exception(f"Failed to save status to {filepath}: {str(e)}")
    
    def save_status_continuously(self, agv_core_node, interval_seconds: float = 1.0, max_files: int = 100):
        """
        持續保存狀態 (可用於監控)
        
        Args:
            agv_core_node: AgvCoreNode 實例
            interval_seconds: 保存間隔 (秒)
            max_files: 最大保存文件數量
        """
        import threading
        import time
        
        def continuous_save():
            file_count = 0
            while file_count < max_files:
                try:
                    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]  # 包含毫秒
                    filename = f"cargo_agv_continuous_{timestamp}.json"
                    self.save_status_to_file(agv_core_node, filename)
                    file_count += 1
                    time.sleep(interval_seconds)
                except Exception as e:
                    print(f"Error in continuous save: {e}")
                    break
                    
        thread = threading.Thread(target=continuous_save, daemon=True)
        thread.start()
        return thread
    
    def get_status_summary(self, agv_core_node) -> Dict[str, Any]:
        """
        獲取狀態摘要 (較小的 JSON，適合頻繁記錄)
        
        Args:
            agv_core_node: AgvCoreNode 實例
            
        Returns:
            狀態摘要字典
        """
        timestamp = datetime.now()
        
        # 獲取當前狀態
        base_state = getattr(agv_core_node.base_context, 'state', None)
        cargo_state = getattr(agv_core_node.cargo_context, 'state', None)
        robot_state = getattr(agv_core_node.robot_context, 'state', None)
        
        summary = {
            "timestamp": timestamp.isoformat(),
            "current_states": {
                "base_state": base_state.__class__.__name__ if base_state else None,
                "cargo_state": cargo_state.__class__.__name__ if cargo_state else None,
                "robot_state": robot_state.__class__.__name__ if robot_state else None
            },
            "key_status": {
                "rack_rotation": getattr(agv_core_node.cargo_context, 'rack_rotation', None),
                "completed": getattr(agv_core_node.cargo_context, 'completed', None),
                "agv_auto": getattr(agv_core_node.agv_status, 'AGV_Auto', None),
                "agv_moving": getattr(agv_core_node.agv_status, 'AGV_MOVING', None),
                "agv_alarm": getattr(agv_core_node.agv_status, 'AGV_ALARM', None),
                "power": getattr(agv_core_node.agv_status, 'POWER', None)
            }
        }
        
        return summary

    def create_complete_frontend_status(self, agv_core_node) -> Dict[str, Any]:
        """
        創建前端狀態快照，包含 AGV 狀態資料和基礎節點變數
        
        Args:
            agv_core_node: AgvCoreNode 實例
            
        Returns:
            前端狀態字典，包含 agv_status.py 和 agv_node_base.py 資料
        """
        timestamp = datetime.now()
        
        try:
            # 獲取當前狀態
            base_state = getattr(agv_core_node.base_context, 'state', None)
            cargo_state = getattr(agv_core_node.cargo_context, 'state', None)
            robot_state = getattr(agv_core_node.robot_context, 'state', None)
            
            # 狀態資訊 - 包含 agv_status.py 和 agv_node_base.py 重要變數
            complete_status = {
                "metadata": {
                    "timestamp": timestamp.isoformat(),
                    "timestamp_unix": time.time(),
                    "agv_type": "cargo_mover_agv",
                    "agv_id": agv_core_node.get_namespace().lstrip('/') if hasattr(agv_core_node, 'get_namespace') else getattr(agv_core_node.agv_status, 'AGV_ID', 'unknown') if agv_core_node.agv_status else 'unknown'
                },
                
                # AGV 狀態 (agv_status.py 所有細項)
                "agv_status_complete": self._get_complete_agv_status(agv_core_node.agv_status) if agv_core_node.agv_status else {},
                
                # AGV 基礎節點變數 (agv_node_base.py 重要變數)
                "agv_base_variables": self._get_agv_base_variables(agv_core_node),
                
                # 基本 Context 狀態資訊（不包含變數）
                "contexts": {
                    "base_context": {
                        "current_state": base_state.__class__.__name__ if base_state else "Unknown",
                        "state_module": base_state.__class__.__module__ if base_state else "Unknown"
                    },
                    "cargo_context": {
                        "current_state": cargo_state.__class__.__name__ if cargo_state else "Unknown",
                        "state_module": cargo_state.__class__.__module__ if cargo_state else "Unknown"
                    },
                    "robot_context": {
                        "current_state": robot_state.__class__.__name__ if robot_state else "Unknown",
                        "state_module": robot_state.__class__.__module__ if robot_state else "Unknown"
                    }
                }
            }
            
            return complete_status
            
        except Exception as e:
            # 如果出現任何錯誤，返回基本資訊
            return {
                "metadata": {
                    "timestamp": timestamp.isoformat(),
                    "timestamp_unix": time.time(), 
                    "agv_type": "cargo_mover_agv",
                    "agv_id": agv_core_node.get_namespace().lstrip('/') if hasattr(agv_core_node, 'get_namespace') else 'unknown',
                    "error": f"Status collection error: {str(e)}"
                },
                "agv_status_complete": {},
                "agv_base_variables": {},
                "contexts": {
                    "base_context": {"current_state": "Error", "state_module": "Unknown"},
                    "cargo_context": {"current_state": "Error", "state_module": "Unknown"},
                    "robot_context": {"current_state": "Error", "state_module": "Unknown"}
                },
                "error": "Failed to collect status"
            }

    def _get_complete_agv_status(self, agv_status) -> Dict[str, Any]:
        """獲取 agv_status.py 中指定變數的狀態"""
        if not agv_status:
            return {}
        
        # 指定要收集的AGV狀態變數清單
        selected_vars = [
            'POWER', 'AGV_X_SPEED', 'AGV_Y_SPEED', 'AGV_THETA_SPEED',
            'AGV_FPGV', 'AGV_BPGV', 'AGV_START_POINT', 'AGV_END_POINT', 'AGV_ACTION',
            'AGV_ZONE', 'AGV_SLAM_X', 'AGV_SLAM_Y', 'AGV_SLAM_THETA', 'AGV_LAYER',
            'AGV_ID1', 'MAGIC', 'AGV_Auto', 'AGV_MOVING', 'AGV_ALARM',
            'AGV_PATH', 'AGV_PATH_REQ', 'AGV_IN_MISSION', 'AGV_LOCAL', 'AGV_LD_COMPLETE',
            'AGV_UD_COMPLETE', 'LOW_POWER', 'MISSION_CANCEL', 'TRAFFIC_STOP', 'TRAFFIC_ALLOW',
            'PS_RETRUN', 'AGV_MANUAL', 'AGV_2POSITION',
            'IN_1', 'IN_2', 'IN_3', 'IN_4', 'IN_5', 'BARCODE_READER_FINISH', 'TAG_REQ', 'Req_TAGNo',
            'DOOR_OPEN_1', 'DOOR_CLOSE_1', 'DOOR_OPEN_2', 'DOOR_CLOSE_2',
            'DOOR_OPEN_3', 'DOOR_CLOSE_3', 'DOOR_OPEN_4', 'DOOR_CLOSE_4',
            'DOOR_OPEN_5', 'DOOR_CLOSE_5', 'DOOR_OPEN_6', 'DOOR_CLOSE_6',
            'DOOR_OPEN_7', 'DOOR_CLOSE_7', 'DOOR_OPEN_8', 'DOOR_CLOSE_8'
        ]
        
        status_dict = {}
        for var_name in selected_vars:
            try:
                if hasattr(agv_status, var_name):
                    value = getattr(agv_status, var_name)
                    # 使用安全序列化處理所有值
                    status_dict[var_name] = self.safe_serialize(value)
                else:
                    status_dict[var_name] = None
            except Exception as e:
                status_dict[var_name] = f"Error: {str(e)}"
        
        return status_dict

    def _get_agv_base_variables(self, agv_core_node) -> Dict[str, Any]:
        """獲取 agv_node_base.py 中的重要變數"""
        if not agv_core_node:
            return {}
        
        base_variables = {}
        
        # 定義要收集的 agv_node_base.py 變數
        base_var_names = [
            'pathdata',        # 路徑資料
            'mission_id',      # 任務ID
            'node_id',         # 任務目標節點
            'agv_id',          # AGV ID (數據庫 agv 表主键)
            'task',            # 任務訊息
            'robot_finished'   # 機器人完成狀態
        ]
        
        for var_name in base_var_names:
            try:
                if hasattr(agv_core_node, var_name):
                    value = getattr(agv_core_node, var_name)
                    # 使用安全序列化處理值
                    base_variables[var_name] = self.safe_serialize(value)
                else:
                    base_variables[var_name] = None
            except Exception as e:
                base_variables[var_name] = f"Error: {str(e)}"
        
        return base_variables

    def _get_node_variables(self, node, exclude_contexts=False) -> Dict[str, Any]:
        """獲取節點的所有變數成員"""
        if not node:
            return {}
        
        variables = {}
        for attr_name in dir(node):
            if not attr_name.startswith('_') and not callable(getattr(node, attr_name)):
                # 排除 context 相關變數，避免重複
                if exclude_contexts and attr_name in ['base_context', 'cargo_context', 'robot_context', 'agv_status']:
                    continue
                try:
                    value = getattr(node, attr_name)
                    # 使用安全序列化處理所有值
                    variables[attr_name] = self.safe_serialize(value)
                except Exception as e:
                    variables[attr_name] = f"Error: {str(e)}"
        
        return variables

    def _get_base_node_variables(self, node) -> Dict[str, Any]:
        """獲取從 AgvNodebase 繼承的變數"""
        if not node:
            return {}
        
        base_variables = {}
        # 獲取基類的屬性
        for cls in node.__class__.__mro__:
            if cls.__name__ == 'AgvNodebase':
                for attr_name in cls.__dict__:
                    if not attr_name.startswith('_') and hasattr(node, attr_name):
                        try:
                            value = getattr(node, attr_name)
                            if not callable(value):
                                # 使用安全序列化處理所有值
                                base_variables[attr_name] = self.safe_serialize(value)
                        except Exception as e:
                            base_variables[attr_name] = f"Error: {str(e)}"
                break
        
        return base_variables

    def _get_context_variables(self, context) -> Dict[str, Any]:
        """獲取 context 的所有變數"""
        if not context:
            return {}
        
        context_vars = {}
        for attr_name in dir(context):
            if not attr_name.startswith('_') and not callable(getattr(context, attr_name)):
                if attr_name != 'state':  # state 已在上層處理
                    try:
                        value = getattr(context, attr_name)
                        # 使用安全序列化處理所有值
                        context_vars[attr_name] = self.safe_serialize(value)
                    except Exception as e:
                        context_vars[attr_name] = f"Error: {str(e)}"
        
        return context_vars

    def create_lightweight_status(self, agv_core_node) -> Dict[str, Any]:
        """
        創建輕量級狀態快照 (向後相容)
        
        Args:
            agv_core_node: AgvCoreNode 實例
            
        Returns:
            輕量級狀態字典
        """
        # 為了向後相容，保留此方法但使用完整前端狀態
        return self.create_complete_frontend_status(agv_core_node)

    def save_lightweight_status_to_file(self, agv_core_node, filename: str = "current_status.json") -> str:
        """
        保存輕量級狀態到文件
        
        Args:
            agv_core_node: AgvCoreNode 實例
            filename: 文件名
            
        Returns:
            保存的文件路徑
        """
        lightweight_status = self.create_lightweight_status(agv_core_node)
        
        filepath = os.path.join(self.output_dir, filename)
        
        try:
            with open(filepath, 'w', encoding='utf-8') as f:
                json.dump(lightweight_status, f, indent=2, ensure_ascii=False)
            return filepath
        except Exception as e:
            raise Exception(f"Failed to save lightweight status to {filepath}: {str(e)}")

    def save_complete_frontend_status_to_file(self, agv_core_node, filename: str = "current_status.json") -> str:
        """
        保存完整前端狀態到文件
        
        Args:
            agv_core_node: AgvCoreNode 實例
            filename: 文件名
            
        Returns:
            保存的文件路徑
        """
        complete_status = self.create_complete_frontend_status(agv_core_node)
        
        filepath = os.path.join(self.output_dir, filename)
        
        try:
            with open(filepath, 'w', encoding='utf-8') as f:
                json.dump(complete_status, f, indent=2, ensure_ascii=False)
            return filepath
        except Exception as e:
            raise Exception(f"Failed to save complete frontend status to {filepath}: {str(e)}")


def example_usage():
    """使用範例"""
    # 假設有一個 AgvCoreNode 實例
    # agv_node = AgvCoreNode()
    
    # 創建記錄器
    recorder = CargoAgvStatusJsonRecorder(output_dir="/tmp/cargo_agv_logs")
    
    # 保存完整狀態快照
    # filepath = recorder.save_status_to_file(agv_node)
    # print(f"Status saved to: {filepath}")
    
    # 獲取狀態摘要
    # summary = recorder.get_status_summary(agv_node)
    # print(json.dumps(summary, indent=2))
    
    pass


if __name__ == "__main__":
    example_usage()