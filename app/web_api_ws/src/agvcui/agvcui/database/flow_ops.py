"""
Flow Designer 文件系統操作模組
負責管理 config/wcs/ 資料夾中的流程定義文件 (YAML 格式)
不涉及資料庫操作，純文件系統管理
"""

import json
import yaml
import os
import shutil
import tempfile
from pathlib import Path
from typing import List, Dict, Any, Optional, Union
from datetime import datetime
import logging

logger = logging.getLogger(__name__)


class FlowFileManager:
    """
    流程文件管理器
    管理 config/wcs/ 資料夾中的 YAML 格式流程定義文件
    """
    
    def __init__(self):
        # 設置基礎路徑
        self.base_path = Path("/app/config/wcs")
        self.flows_path = self.base_path / "flows"
        self.templates_path = self.base_path / "templates"
        self.backup_path = self.base_path / "backups"
        
        # 確保目錄存在
        self._ensure_directories()
        
        # 函數配置文件路徑
        self.functions_config_path = Path("/app/wcs-flow-designer參考/functions-config.json")
    
    def _ensure_directories(self):
        """確保所需目錄存在"""
        try:
            self.flows_path.mkdir(parents=True, exist_ok=True)
            self.templates_path.mkdir(parents=True, exist_ok=True)
            self.backup_path.mkdir(parents=True, exist_ok=True)
            logger.info(f"確保目錄存在: {self.base_path}")
        except Exception as e:
            logger.error(f"創建目錄失敗: {e}")
            raise
    
    def _get_flow_file_path(self, flow_name: str) -> Path:
        """獲取流程文件的完整路徑"""
        # 清理文件名，移除不安全字符
        safe_name = "".join(c for c in flow_name if c.isalnum() or c in (' ', '-', '_')).rstrip()
        return self.flows_path / f"{safe_name}.yaml"
    
    def _backup_flow(self, flow_name: str) -> bool:
        """備份現有流程文件"""
        try:
            flow_file = self._get_flow_file_path(flow_name)
            if flow_file.exists():
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                backup_name = f"{flow_name}_{timestamp}.yaml"
                backup_file = self.backup_path / backup_name
                
                shutil.copy2(flow_file, backup_file)
                logger.info(f"備份流程文件: {flow_name} -> {backup_file}")
                return True
        except Exception as e:
            logger.error(f"備份流程文件失敗: {e}")
        return False
    
    def list_flows(self) -> List[Dict[str, Any]]:
        """
        列出所有流程文件
        
        Returns:
            List[Dict]: 流程摘要信息列表
        """
        flows = []
        try:
            if not self.flows_path.exists():
                return flows
            
            for file_path in self.flows_path.glob("*.yaml"):
                try:
                    with open(file_path, 'r', encoding='utf-8') as f:
                        flow_data = yaml.safe_load(f)
                    
                    # 獲取文件統計信息
                    stat = file_path.stat()
                    
                    # 從 flow_designer_data 或頂層獲取節點和連接
                    designer_data = flow_data.get("flow_designer_data", {})
                    nodes = designer_data.get("nodes", flow_data.get("nodes", []))
                    connections = designer_data.get("connections", flow_data.get("connections", []))
                    
                    flows.append({
                        "name": flow_data.get("name", file_path.stem),
                        "description": flow_data.get("description", ""),
                        "nodes": nodes,
                        "connections": connections,
                        "metadata": flow_data.get("metadata", {}),
                        "file_size": stat.st_size,
                        "created_at": flow_data.get("metadata", {}).get("created_at"),
                        "updated_at": flow_data.get("metadata", {}).get("updated_at"),
                        "file_modified": datetime.fromtimestamp(stat.st_mtime).isoformat()
                    })
                    
                except (yaml.YAMLError, KeyError) as e:
                    logger.warning(f"跳過無效的流程文件 {file_path}: {e}")
                    continue
                    
            # 按更新時間排序 (處理 None 值)
            flows.sort(key=lambda x: x.get("updated_at") or "", reverse=True)
            
        except Exception as e:
            logger.error(f"列出流程文件失敗: {e}")
            raise
        
        return flows
    
    def load_flow(self, flow_name: str) -> Optional[Dict[str, Any]]:
        """
        載入指定的流程文件
        
        Args:
            flow_name: 流程名稱
            
        Returns:
            Dict: 流程定義數據，如果文件不存在則返回 None
        """
        try:
            flow_file = self._get_flow_file_path(flow_name)
            
            if not flow_file.exists():
                logger.warning(f"流程文件不存在: {flow_name}")
                return None
            
            with open(flow_file, 'r', encoding='utf-8') as f:
                flow_data = yaml.safe_load(f)
            
            logger.info(f"載入流程文件: {flow_name}")
            return flow_data
            
        except yaml.YAMLError as e:
            logger.error(f"YAML 格式錯誤 {flow_name}: {e}")
            raise ValueError(f"流程文件格式錯誤: {e}")
        except Exception as e:
            logger.error(f"載入流程文件失敗 {flow_name}: {e}")
            raise
    
    def save_flow(self, flow_name: str, flow_data: Dict[str, Any]) -> bool:
        """
        保存流程文件
        
        Args:
            flow_name: 流程名稱
            flow_data: 流程定義數據
            
        Returns:
            bool: 保存是否成功
        """
        try:
            flow_file = self._get_flow_file_path(flow_name)
            
            # 如果文件已存在，先備份
            if flow_file.exists():
                self._backup_flow(flow_name)
            
            # 確保流程名稱與文件名一致
            flow_data["name"] = flow_name
            
            # 添加/更新時間戳
            now = datetime.now().isoformat()
            if "metadata" not in flow_data:
                flow_data["metadata"] = {}
            
            if not flow_file.exists():
                flow_data["metadata"]["created_at"] = now
            flow_data["metadata"]["updated_at"] = now
            
            # 寫入文件
            with open(flow_file, 'w', encoding='utf-8') as f:
                yaml.dump(flow_data, f, allow_unicode=True, default_flow_style=False, sort_keys=False)
            
            logger.info(f"保存流程文件: {flow_name}")
            return True
            
        except Exception as e:
            logger.error(f"保存流程文件失敗 {flow_name}: {e}")
            return False
    
    def delete_flow(self, flow_name: str) -> bool:
        """
        刪除流程文件
        
        Args:
            flow_name: 流程名稱
            
        Returns:
            bool: 刪除是否成功
        """
        try:
            flow_file = self._get_flow_file_path(flow_name)
            
            if not flow_file.exists():
                logger.warning(f"要刪除的流程文件不存在: {flow_name}")
                return False
            
            # 備份後刪除
            self._backup_flow(flow_name)
            flow_file.unlink()
            
            logger.info(f"刪除流程文件: {flow_name}")
            return True
            
        except Exception as e:
            logger.error(f"刪除流程文件失敗 {flow_name}: {e}")
            return False
    
    def duplicate_flow(self, source_name: str, target_name: str) -> bool:
        """
        複製流程文件
        
        Args:
            source_name: 來源流程名稱
            target_name: 目標流程名稱
            
        Returns:
            bool: 複製是否成功
        """
        try:
            source_data = self.load_flow(source_name)
            if not source_data:
                return False
            
            # 修改名稱和元數據
            source_data["name"] = target_name
            source_data["description"] = f"{source_data.get('description', '')} (複製自 {source_name})"
            
            if "metadata" in source_data:
                source_data["metadata"]["duplicated_from"] = source_name
                source_data["metadata"]["duplicated_at"] = datetime.now().isoformat()
            
            return self.save_flow(target_name, source_data)
            
        except Exception as e:
            logger.error(f"複製流程文件失敗 {source_name} -> {target_name}: {e}")
            return False
    
    def export_flow_to_file(self, flow_name: str) -> Optional[str]:
        """
        匯出流程到臨時文件
        
        Args:
            flow_name: 流程名稱
            
        Returns:
            str: 臨時文件路徑，如果失敗則返回 None
        """
        try:
            flow_data = self.load_flow(flow_name)
            if not flow_data:
                return None
            
            # 創建臨時文件
            temp_fd, temp_path = tempfile.mkstemp(suffix='.yaml', prefix=f'{flow_name}_')
            
            with os.fdopen(temp_fd, 'w', encoding='utf-8') as f:
                yaml.dump(flow_data, f, allow_unicode=True, default_flow_style=False, sort_keys=False)
            
            logger.info(f"匯出流程到臨時文件: {flow_name} -> {temp_path}")
            return temp_path
            
        except Exception as e:
            logger.error(f"匯出流程文件失敗 {flow_name}: {e}")
            return None
    
    def get_functions_config(self) -> Dict[str, Any]:
        """
        獲取函數配置
        
        Returns:
            Dict: 函數配置數據
        """
        try:
            if self.functions_config_path.exists():
                with open(self.functions_config_path, 'r', encoding='utf-8') as f:
                    return json.load(f)
            else:
                # 返回預設配置
                return self._get_default_functions_config()
                
        except Exception as e:
            logger.error(f"載入函數配置失敗: {e}")
            return self._get_default_functions_config()
    
    def _get_default_functions_config(self) -> Dict[str, Any]:
        """獲取預設函數配置"""
        return {
            "condition_functions": [
                {
                    "value": "check_rack_position",
                    "label": "檢查貨架位置",
                    "description": "檢查指定位置的貨架狀態",
                    "module": "wcs.conditions",
                    "parameters": [
                        {
                            "name": "location_id",
                            "type": "str",
                            "required": True,
                            "label": "位置編號",
                            "placeholder": "例如: transfer_exit"
                        },
                        {
                            "name": "check_type",
                            "type": "str",
                            "required": True,
                            "label": "檢查類型",
                            "options": ["has_rack", "is_empty", "is_full"]
                        }
                    ]
                },
                {
                    "value": "no_active_task",
                    "label": "無活動任務",
                    "description": "檢查指定來源地是否沒有進行中的任務",
                    "module": "wcs.conditions",
                    "parameters": [
                        {
                            "name": "source_location",
                            "type": "str",
                            "required": True,
                            "label": "來源位置",
                            "placeholder": "例如: transfer_exit"
                        },
                        {
                            "name": "work_id",
                            "type": "str",
                            "required": False,
                            "label": "工作ID",
                            "placeholder": "例如: 220001"
                        }
                    ]
                }
            ],
            "action_functions": [
                {
                    "value": "create_transport_task",
                    "label": "創建運輸任務",
                    "description": "創建新的 AGV 運輸任務",
                    "module": "wcs.tasks",
                    "parameters": [
                        {
                            "name": "robot_model",
                            "type": "str",
                            "required": True,
                            "label": "機器人型號",
                            "options": ["KUKA400i", "cargo", "loader", "unloader"]
                        },
                        {
                            "name": "source",
                            "type": "str",
                            "required": True,
                            "label": "來源位置",
                            "placeholder": "例如: transfer_exit"
                        },
                        {
                            "name": "destination",
                            "type": "str",
                            "required": True,
                            "label": "目標位置",
                            "placeholder": "例如: manual_collection_area"
                        },
                        {
                            "name": "task_type",
                            "type": "str",
                            "required": False,
                            "label": "任務類型",
                            "default": "RACK_MOVE",
                            "options": ["RACK_MOVE", "RACK_ROTATION", "CARRIER_TRANSPORT"]
                        }
                    ]
                }
            ]
        }
    
    def validate_flow(self, flow_data: Dict[str, Any]) -> Dict[str, Any]:
        """
        驗證流程定義的有效性
        
        Args:
            flow_data: 流程定義數據
            
        Returns:
            Dict: 驗證結果
        """
        try:
            validation_result = {
                "valid": True,
                "errors": [],
                "warnings": [],
                "suggestions": []
            }
            
            # 基本結構檢查
            required_fields = ["name", "nodes", "connections"]
            for field in required_fields:
                if field not in flow_data:
                    validation_result["errors"].append(f"缺少必要欄位: {field}")
                    validation_result["valid"] = False
            
            # 節點檢查
            nodes = flow_data.get("nodes", [])
            if not nodes:
                validation_result["warnings"].append("流程中沒有節點")
            
            # 連接檢查
            connections = flow_data.get("connections", [])
            node_ids = {node.get("id") for node in nodes}
            
            for conn in connections:
                source = conn.get("source")
                target = conn.get("target")
                
                if source not in node_ids:
                    validation_result["errors"].append(f"連接中的來源節點不存在: {source}")
                    validation_result["valid"] = False
                
                if target not in node_ids:
                    validation_result["errors"].append(f"連接中的目標節點不存在: {target}")
                    validation_result["valid"] = False
            
            # 邏輯檢查
            if len(nodes) > 0 and len(connections) == 0:
                validation_result["warnings"].append("節點之間沒有連接")
            
            # 建議
            if len(nodes) > 20:
                validation_result["suggestions"].append("流程複雜度較高，建議分解為多個子流程")
            
            logger.info(f"驗證流程: {flow_data.get('name', 'unknown')} - {'有效' if validation_result['valid'] else '無效'}")
            return validation_result
            
        except Exception as e:
            logger.error(f"驗證流程失敗: {e}")
            return {
                "valid": False,
                "errors": [f"驗證過程發生錯誤: {str(e)}"],
                "warnings": [],
                "suggestions": []
            }
    
    def get_templates(self) -> List[Dict[str, Any]]:
        """
        獲取流程模板列表
        
        Returns:
            List[Dict]: 模板信息列表
        """
        templates = []
        try:
            if not self.templates_path.exists():
                return templates
            
            for file_path in self.templates_path.glob("*.yaml"):
                try:
                    with open(file_path, 'r', encoding='utf-8') as f:
                        template_data = yaml.safe_load(f)
                    
                    templates.append({
                        "name": template_data.get("name", file_path.stem),
                        "description": template_data.get("description", ""),
                        "category": template_data.get("category", "general"),
                        "node_count": len(template_data.get("nodes", [])),
                        "connection_count": len(template_data.get("connections", [])),
                        "filename": file_path.name
                    })
                    
                except (yaml.YAMLError, KeyError) as e:
                    logger.warning(f"跳過無效的模板文件 {file_path}: {e}")
                    continue
            
        except Exception as e:
            logger.error(f"獲取模板列表失敗: {e}")
        
        return templates
    
    def get_statistics(self) -> Dict[str, Any]:
        """
        獲取流程統計信息
        
        Returns:
            Dict: 統計數據
        """
        try:
            flows = self.list_flows()
            templates = self.get_templates()
            
            total_nodes = sum(len(flow.get("nodes", [])) for flow in flows)
            total_connections = sum(len(flow.get("connections", [])) for flow in flows)
            
            # 計算目錄大小
            total_size = 0
            if self.flows_path.exists():
                for file_path in self.flows_path.glob("*.yaml"):
                    total_size += file_path.stat().st_size
            
            return {
                "total_flows": len(flows),
                "total_templates": len(templates),
                "total_nodes": total_nodes,
                "total_connections": total_connections,
                "total_size_bytes": total_size,
                "average_nodes_per_flow": total_nodes / len(flows) if flows else 0,
                "average_connections_per_flow": total_connections / len(flows) if flows else 0,
                "flows_directory": str(self.flows_path),
                "last_updated": datetime.now().isoformat()
            }
            
        except Exception as e:
            logger.error(f"獲取統計信息失敗: {e}")
            return {
                "error": str(e),
                "last_updated": datetime.now().isoformat()
            }