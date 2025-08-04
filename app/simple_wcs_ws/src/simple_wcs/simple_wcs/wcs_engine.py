#!/usr/bin/env python3
"""
Simple WCS Engine
極簡化配置驅動的 WCS 決策引擎
支持多種業務流程的配置驅動決策
"""

import os
import sys
import yaml
import logging
import threading
import time
from typing import List, Dict, Any, Optional
from dataclasses import dataclass
from pathlib import Path

# 添加 ROS 2 路徑
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# 本地模組
from simple_wcs.flow_parser import FlowParser, BusinessFlow
from simple_wcs.database_client import DatabaseClient


@dataclass
class TaskDecision:
    """任務決策結果"""
    name: str
    work_id: str
    priority: int
    room_id: int
    rack_id: int
    nodes: List[int]
    parameters: Dict[str, Any]
    reason: str


class LocationManager:
    """靜態位置配置管理器"""
    
    def __init__(self, config_file: str):
        self.config_file = config_file
        self.logger = logging.getLogger('simple_wcs.location_manager')
        self.config = self._load_config()
    
    def _load_config(self) -> Dict:
        """載入 YAML 位置配置"""
        try:
            with open(self.config_file, 'r', encoding='utf-8') as f:
                return yaml.safe_load(f)
        except FileNotFoundError:
            self.logger.error(f"位置配置檔案不存在: {self.config_file}")
            return {}
        except Exception as e:
            self.logger.error(f"載入位置配置失敗: {e}")
            return {}
    
    def get_room_inlet_point(self, room_id: int) -> int:
        """獲取房間入口停靠點"""
        return self.config.get('rooms', {}).get(str(room_id), {}).get('inlet', {}).get('stop_point', 0)
    
    def get_inlet_rotation_point(self, room_id: int) -> int:
        """獲取入口旋轉中間點"""
        return self.config.get('rooms', {}).get(str(room_id), {}).get('inlet', {}).get('rotation_point', 0)
    
    def get_room_exit_point(self, room_id: int) -> int:
        """獲取房間出口停靠點"""
        return self.config.get('rooms', {}).get(str(room_id), {}).get('exit', {}).get('stop_point', 0)
    
    def get_exit_rotation_point(self, room_id: int) -> int:
        """獲取出口旋轉中間點"""
        return self.config.get('rooms', {}).get(str(room_id), {}).get('exit', {}).get('rotation_point', 0)


class SimpleWCSEngine(Node):
    """Simple WCS 決策引擎 - ROS 2 節點"""
    
    def __init__(self):
        super().__init__('simple_wcs_engine')
        
        # 設定日誌
        self.logger = self.get_logger()
        self.logger.info("🚀 Simple WCS Engine 啟動中...")
        
        # 初始化組件
        self._init_components()
        
        # 設定決策循環定時器 (5秒一次)
        self.decision_timer = self.create_timer(5.0, self.decision_cycle_callback)
        
        # ROS 2 發布者
        self.task_publisher = self.create_publisher(String, '/simple_wcs/task_decisions', 10)
        self.status_publisher = self.create_publisher(String, '/simple_wcs/system_status', 10)
        
        self.logger.info("✅ Simple WCS Engine 啟動完成")
    
    def _init_components(self):
        """初始化系統組件"""
        try:
            # 取得配置檔案路徑 - 使用統一配置目錄
            config_dir = Path('/app/config/wcs')
            
            # 初始化資料庫客戶端
            self.db = DatabaseClient()
            self.logger.info("📊 資料庫客戶端初始化完成")
            
            # 初始化位置管理器
            locations_path = config_dir / 'locations.yaml'
            self.locations = LocationManager(str(locations_path))
            self.logger.info("📍 位置管理器初始化完成")
            
            # 初始化流程解析器 - 支援多檔案目錄
            flows_path = config_dir / 'flows'  # 改為目錄
            self.flow_parser = FlowParser(str(flows_path))
            self.business_flows = self.flow_parser.parse()
            self.logger.info(f"📋 載入 {len(self.business_flows)} 個業務流程")
            
            # 驗證配置
            validation = self.flow_parser.validate_flows(self.business_flows)
            if validation['errors']:
                self.logger.error(f"配置錯誤: {validation['errors']}")
            if validation['warnings']:
                self.logger.warning(f"配置警告: {validation['warnings']}")
            
        except Exception as e:
            self.logger.error(f"組件初始化失敗: {e}")
            raise
    
    def decision_cycle_callback(self):
        """決策循環回調函數 - 每5秒執行一次"""
        try:
            self.logger.info("🔄 開始決策循環...")
            
            # 發布系統狀態
            self._publish_system_status()
            
            # 執行業務流程檢查
            decisions = self._run_business_flows()
            
            # 處理決策結果
            if decisions:
                self.logger.info(f"📋 產生 {len(decisions)} 個任務決策")
                for decision in decisions:
                    self._execute_task_decision(decision)
            else:
                self.logger.debug("💤 本輪無任務需要執行")
            
        except Exception as e:
            self.logger.error(f"決策循環執行失敗: {e}")
    
    def _run_business_flows(self) -> List[TaskDecision]:
        """執行所有業務流程檢查"""
        all_decisions = []
        
        self.logger.info(f"🔍 開始檢查 {len(self.business_flows)} 個業務流程")
        
        # 按優先級排序執行
        sorted_flows = sorted(self.business_flows, key=lambda f: f.priority, reverse=True)
        
        for flow in sorted_flows:
            try:
                self.logger.info(f"📋 檢查流程: {flow.name} (優先級: {flow.priority})")
                decisions = self._check_single_flow(flow)
                
                if decisions:
                    self.logger.info(f"✅ 流程 '{flow.name}' 產生了 {len(decisions)} 個任務決策")
                    all_decisions.extend(decisions)
                else:
                    self.logger.info(f"💤 流程 '{flow.name}' 未產生任務")
                    
            except Exception as e:
                self.logger.error(f"業務流程 '{flow.name}' 檢查失敗: {e}")
                continue
        
        self.logger.info(f"📊 決策結果：共產生 {len(all_decisions)} 個任務決策")
        return all_decisions
    
    def _check_single_flow(self, flow: BusinessFlow) -> List[TaskDecision]:
        """檢查單一業務流程 - 通用 YAML 條件檢查"""
        decisions = []
        
        try:
            # 使用通用的條件檢查邏輯
            if flow.name == "Rack旋轉檢查-房間入口":
                decisions = self._check_rack_rotation_flow(flow)
            elif flow.name == "滿料架到人工收料區-傳送箱出口":
                decisions = self._check_transport_to_manual_flow(flow)
            elif flow.name == "Rack旋轉檢查-房間出口":
                decisions = self._check_rack_rotation_exit_flow(flow)
            else:
                self.logger.debug(f"業務流程 '{flow.name}' 暫未實現")
            
        except Exception as e:
            self.logger.error(f"檢查業務流程 '{flow.name}' 失敗: {e}")
        
        return decisions
    
    def _check_rack_rotation_flow(self, flow: BusinessFlow) -> List[TaskDecision]:
        """檢查 Rack 旋轉業務流程 - 基於 YAML 配置"""
        decisions = []
        
        try:
            # 根據 applicable_rooms 檢查對應房間
            rooms_to_check = flow.applicable_rooms if flow.applicable_rooms else range(1, 6)
            self.logger.info(f"🏠 檢查房間: {list(rooms_to_check)}")
            
            for room_id in rooms_to_check:
                inlet_point = self.locations.get_room_inlet_point(room_id)
                self.logger.info(f"🔍 房間 {room_id} 入口點: {inlet_point}")
                
                if inlet_point == 0:
                    self.logger.info(f"⏭️  跳過房間 {room_id} - 無效的入口點配置")
                    continue  # 跳過無效配置
                
                decision = self._check_single_room_rotation(flow, room_id, inlet_point)
                if decision:
                    self.logger.info(f"✅ 房間 {room_id} 產生旋轉任務")
                    decisions.append(decision)
                    # 一次只處理一個旋轉任務
                    break
                else:
                    self.logger.info(f"❌ 房間 {room_id} 不滿足旋轉條件")
            
            return decisions
            
        except Exception as e:
            self.logger.error(f"Rack旋轉流程檢查失敗: {e}")
            return []
    
    def _check_single_room_rotation(self, flow: BusinessFlow, room_id: int, inlet_point: int) -> Optional[TaskDecision]:
        """檢查單一房間入口的 Rack 旋轉需求"""
        try:
            self.logger.info(f"🔎 檢查房間 {room_id} 入口點 {inlet_point}")
            
            # 1. 檢查該位置是否有 Rack
            has_rack = self.db.rack_at_location_exists(inlet_point)
            self.logger.info(f"📍 位置 {inlet_point} 是否有 Rack: {has_rack}")
            
            if not has_rack:
                self.logger.info(f"❌ 位置 {inlet_point} 沒有 Rack，跳過")
                return None
            
            rack_info = self.db.get_rack_at_location(inlet_point)
            if not rack_info:
                self.logger.info(f"❌ 無法獲取位置 {inlet_point} 的 Rack 資訊")
                return None
            
            rack_id = rack_info['id']
            self.logger.info(f"🎯 找到 Rack ID: {rack_id}, 當前朝向: {rack_info.get('direction', 0)}°")
            
            # 2. 根據 YAML 配置檢查觸發條件
            self.logger.info(f"📋 開始檢查 {len(flow.trigger_conditions)} 個觸發條件:")
            conditions_met = []
            
            for i, trigger in enumerate(flow.trigger_conditions):
                condition_result = self._evaluate_trigger_condition(trigger, rack_id, room_id, inlet_point)
                conditions_met.append(condition_result)
                
                status = "✅" if condition_result else "❌"
                self.logger.info(f"  {i+1}. {trigger.condition}: {status} - {trigger.description}")
            
            all_conditions_met = all(conditions_met)
            self.logger.info(f"🎯 整體條件評估: {'✅ 所有條件滿足' if all_conditions_met else '❌ 部分條件不滿足'}")
            
            if not all_conditions_met:
                return None
            
            # 3. 根據 YAML 配置產生任務決策
            rotation_point = self.locations.get_inlet_rotation_point(room_id)
            nodes = [inlet_point, rotation_point, inlet_point]
            
            decision = TaskDecision(
                name=f"rack_rotation_inlet_room_{room_id}",
                work_id=flow.work_id,
                priority=flow.priority,
                room_id=room_id,
                rack_id=rack_id,
                nodes=nodes,
                parameters={
                    'function': flow.action.function,
                    'model': flow.action.model,
                    'api': flow.action.api,
                    'missionType': flow.action.mission_type,
                    'rack_id': rack_id,
                    'rotation_type': 'room_inlet',
                    'target_direction': 180,  # 入口：0度 → 180度
                    'task_category': flow.action.task_type
                },
                reason=f"{flow.description} - 房間{room_id}入口 (0°→180°)"
            )
            
            self.logger.info(f"🔄 產生旋轉任務: {decision.reason}")
            return decision
            
        except Exception as e:
            self.logger.error(f"檢查房間{room_id}入口旋轉需求失敗: {e}")
            return None
    
    def _check_rack_rotation_exit_flow(self, flow: BusinessFlow) -> List[TaskDecision]:
        """檢查 Rack 出口旋轉業務流程 - 基於 YAML 配置"""
        decisions = []
        
        try:
            # 根據 applicable_rooms 檢查對應房間
            rooms_to_check = flow.applicable_rooms if flow.applicable_rooms else range(1, 6)
            self.logger.info(f"🏠 檢查房間出口: {list(rooms_to_check)}")
            
            for room_id in rooms_to_check:
                exit_point = self.locations.get_room_exit_point(room_id)
                self.logger.info(f"🔍 房間 {room_id} 出口點: {exit_point}")
                
                if exit_point == 0:
                    self.logger.info(f"⏭️  跳過房間 {room_id} - 無效的出口點配置")
                    continue
                
                decision = self._check_single_room_exit_rotation(flow, room_id, exit_point)
                if decision:
                    self.logger.info(f"✅ 房間 {room_id} 產生出口旋轉任務")
                    decisions.append(decision)
                    # 一次只處理一個旋轉任務
                    break
                else:
                    self.logger.info(f"❌ 房間 {room_id} 不滿足出口旋轉條件")
            
            return decisions
            
        except Exception as e:
            self.logger.error(f"Rack出口旋轉流程檢查失敗: {e}")
            return []
    
    def _check_single_room_exit_rotation(self, flow: BusinessFlow, room_id: int, exit_point: int) -> Optional[TaskDecision]:
        """檢查單一房間出口的 Rack 旋轉需求"""
        try:
            self.logger.info(f"🔎 檢查房間 {room_id} 出口點 {exit_point}")
            
            # 1. 檢查該位置是否有 Rack
            has_rack = self.db.rack_at_location_exists(exit_point)
            self.logger.info(f"📍 位置 {exit_point} 是否有 Rack: {has_rack}")
            
            if not has_rack:
                self.logger.info(f"❌ 位置 {exit_point} 沒有 Rack，跳過")
                return None
            
            rack_info = self.db.get_rack_at_location(exit_point)
            if not rack_info:
                self.logger.info(f"❌ 無法獲取位置 {exit_point} 的 Rack 資訊")
                return None
            
            rack_id = rack_info['id']
            self.logger.info(f"🎯 找到 Rack ID: {rack_id}, 當前朝向: {rack_info.get('direction', 180)}°")
            
            # 2. 根據 YAML 配置檢查觸發條件
            self.logger.info(f"📋 開始檢查 {len(flow.trigger_conditions)} 個觸發條件:")
            conditions_met = []
            
            for i, trigger in enumerate(flow.trigger_conditions):
                condition_result = self._evaluate_trigger_condition(trigger, rack_id, room_id, exit_point)
                conditions_met.append(condition_result)
                
                status = "✅" if condition_result else "❌"
                self.logger.info(f"  {i+1}. {trigger.condition}: {status} - {trigger.description}")
            
            all_conditions_met = all(conditions_met)
            self.logger.info(f"🎯 整體條件評估: {'✅ 所有條件滿足' if all_conditions_met else '❌ 部分條件不滿足'}")
            
            if not all_conditions_met:
                return None
            
            # 3. 根據 YAML 配置產生任務決策
            exit_rotation_point = self.locations.get_exit_rotation_point(room_id)
            nodes = [exit_point, exit_rotation_point, exit_point]
            
            decision = TaskDecision(
                name=f"rack_rotation_exit_room_{room_id}",
                work_id=flow.work_id,
                priority=flow.priority,
                room_id=room_id,
                rack_id=rack_id,
                nodes=nodes,
                parameters={
                    'function': flow.action.function,
                    'model': flow.action.model,
                    'api': flow.action.api,
                    'missionType': flow.action.mission_type,
                    'rack_id': rack_id,
                    'rotation_type': 'room_exit',
                    'target_direction': 0,  # 出口：180度 → 0度
                    'task_category': flow.action.task_type
                },
                reason=f"{flow.description} - 房間{room_id}出口 (180°→0°)"
            )
            
            self.logger.info(f"🔄 產生出口旋轉任務: {decision.reason}")
            return decision
            
        except Exception as e:
            self.logger.error(f"檢查房間{room_id}出口旋轉需求失敗: {e}")
            return None
    
    def _check_transport_to_manual_flow(self, flow: BusinessFlow) -> List[TaskDecision]:
        """檢查運輸到人工收料區業務流程 - 基於 YAML 配置"""
        decisions = []
        
        try:
            # 根據 applicable_locations 檢查對應傳送箱出口
            locations_to_check = flow.applicable_locations if hasattr(flow, 'applicable_locations') and flow.applicable_locations else [20001]
            self.logger.info(f"🚚 檢查傳送箱出口: {list(locations_to_check)}")
            
            for location_id in locations_to_check:
                self.logger.info(f"🔍 檢查傳送箱出口 {location_id}...")
                
                decision = self._check_single_location_transport(flow, location_id)
                if decision:
                    self.logger.info(f"✅ 傳送箱出口 {location_id} 產生運輸任務")
                    decisions.append(decision)
                    # 一次只處理一個運輸任務
                    break
                else:
                    self.logger.info(f"❌ 傳送箱出口 {location_id} 不滿足運輸條件")
            
            return decisions
            
        except Exception as e:
            self.logger.error(f"運輸到人工收料區流程檢查失敗: {e}")
            return []
    
    def _check_single_location_transport(self, flow: BusinessFlow, location_id: int) -> Optional[TaskDecision]:
        """檢查單一傳送箱出口的運輸需求"""
        try:
            self.logger.info(f"🔎 檢查傳送箱出口 {location_id}")
            
            # 1. 檢查該位置是否有滿料架
            has_full_rack = self.db.transfer_exit_has_full_rack(location_id)
            self.logger.info(f"📦 位置 {location_id} 是否有滿料架: {has_full_rack}")
            
            if not has_full_rack:
                self.logger.info(f"❌ 位置 {location_id} 沒有滿料架，跳過")
                return None
            
            # 獲取該位置的 Rack 資訊
            rack_info = self.db.get_rack_at_location(location_id)
            if not rack_info:
                self.logger.info(f"❌ 無法獲取位置 {location_id} 的 Rack 資訊")
                return None
            
            rack_id = rack_info['id']
            self.logger.info(f"🎯 找到滿料架 Rack ID: {rack_id}")
            
            # 2. 根據 YAML 配置檢查觸發條件
            self.logger.info(f"📋 開始檢查 {len(flow.trigger_conditions)} 個觸發條件:")
            conditions_met = []
            
            for i, trigger in enumerate(flow.trigger_conditions):
                condition_result = self._evaluate_trigger_condition(trigger, rack_id, 0, location_id)
                conditions_met.append(condition_result)
                
                status = "✅" if condition_result else "❌"
                self.logger.info(f"  {i+1}. {trigger.condition}: {status} - {trigger.description}")
            
            all_conditions_met = all(conditions_met)
            self.logger.info(f"🎯 整體條件評估: {'✅ 所有條件滿足' if all_conditions_met else '❌ 部分條件不滿足'}")
            
            if not all_conditions_met:
                return None
            
            # 3. 根據 YAML 配置產生任務決策
            # 動態分配人工收料區位置
            target_location = self.db.find_available_manual_location()
            if target_location == 0:
                self.logger.info(f"❌ 沒有可用的人工收料區位置")
                return None
            
            self.logger.info(f"📍 分配目標位置: {target_location}")
            
            # 再次檢查該具體位置是否有衝突 (雙重確認)
            if not self.db.no_active_task_to_specific_location(target_location):
                self.logger.info(f"❌ 目標位置 {target_location} 已被其他任務佔用")
                return None
            
            nodes = [location_id, target_location]
            
            decision = TaskDecision(
                name=f"transport_from_transfer_exit_{location_id}_to_location_{target_location}",
                work_id=flow.work_id,
                priority=flow.priority,
                room_id=0,  # 運輸任務可能跨房間
                rack_id=rack_id,
                nodes=nodes,
                parameters={
                    'function': flow.action.function,
                    'model': flow.action.model,
                    'api': flow.action.api,
                    'missionType': flow.action.mission_type,
                    'rack_id': rack_id,
                    'source_location': location_id,
                    'destination_type': 'manual_collection_area',
                    'destination_location': target_location,  # 具體目的地位置
                    'task_category': flow.action.task_type
                },
                reason=f"{flow.description} - 傳送箱出口{location_id} → 人工收料區位置{target_location}"
            )
            
            self.logger.info(f"🚚 產生運輸任務: {decision.reason}")
            return decision
            
        except Exception as e:
            self.logger.error(f"檢查傳送箱出口{location_id}運輸需求失敗: {e}")
            return None
    
    def _evaluate_trigger_condition(self, trigger, rack_id: int, room_id: int, location: int) -> bool:
        """評估單一觸發條件"""
        try:
            condition = trigger.condition
            params = trigger.parameters
            
            if condition == "rack_at_location_exists":
                location_type = params.get('location_type', 'room_inlet')
                return self.db.rack_at_location_exists(location)
                
            elif condition == "rack_side_completed":
                side = params.get('side', 'A')
                return self.db.rack_side_completed(rack_id, side)
                
            elif condition == "rack_has_b_side_work":
                return self.db.rack_has_b_side_work(rack_id)
                
            elif condition == "rack_needs_rotation_for_b_side":
                location_type = params.get('location_type', 'room_inlet')
                return self.db.rack_needs_rotation_for_b_side(rack_id, location_type)
                
            elif condition == "no_active_task":
                work_id = params.get('work_id', '220001')
                return self.db.no_active_task(work_id, location)
                
            # 新增的運輸任務相關條件
            elif condition == "transfer_exit_has_full_rack":
                return self.db.transfer_exit_has_full_rack(location)
                
            elif condition == "rack_is_full":
                return self.db.rack_is_full(rack_id)
                
            elif condition == "manual_collection_area_available":
                return self.db.manual_collection_area_available()
                
            elif condition == "no_active_task_to_destination":
                destination_type = params.get('destination_type', 'manual_collection_area')
                work_id = params.get('work_id', '220001')
                return self.db.no_active_task_to_destination(destination_type, work_id)
                
            elif condition == "no_active_task_from_source":
                source_type = params.get('source_type', 'transfer_exit')
                source_location = params.get('source_location', location)
                work_id = params.get('work_id', '220001')
                return self.db.no_active_task_from_source(source_type, source_location, work_id)
                
            elif condition == "no_active_task_to_specific_location":
                target_location = params.get('target_location', 0)
                return self.db.no_active_task_to_specific_location(target_location)
                
            # 新增：出口料架旋轉相關條件
            elif condition == "rack_has_a_side_work":
                return self.db.rack_has_a_side_work(rack_id)
                
            elif condition == "rack_needs_rotation_for_a_side":
                location_type = params.get('location_type', 'room_exit')
                return self.db.rack_needs_rotation_for_a_side(rack_id, location_type)
                
            else:
                self.logger.warning(f"未知的觸發條件: {condition}")
                return False
                
        except Exception as e:
            self.logger.error(f"評估觸發條件 '{condition}' 失敗: {e}")
            return False
    
    def _execute_task_decision(self, decision: TaskDecision):
        """執行任務決策"""
        try:
            # 根據任務類型選擇對應的建立方法
            if 'rotation' in decision.name:
                # Rack 旋轉任務 - 根據任務名稱判斷是入口還是出口旋轉
                location_type = 'room_inlet' if 'inlet' in decision.name else 'room_exit'
                result = self.db.create_rack_rotation_task(
                    rack_id=decision.rack_id,
                    room_id=decision.room_id,
                    location_type=location_type,
                    nodes=decision.nodes
                )
            elif 'transport' in decision.name:
                # Rack 運輸任務
                source_location = decision.parameters.get('source_location', decision.nodes[0] if decision.nodes else 0)
                destination_type = decision.parameters.get('destination_type', 'manual_collection_area')
                target_location = decision.parameters.get('destination_location', None)
                
                result = self.db.create_rack_transport_task(
                    rack_id=decision.rack_id,
                    source_location=source_location,
                    destination_type=destination_type,
                    target_location=target_location,
                    nodes=decision.nodes
                )
            else:
                self.logger.error(f"❌ 未知的任務類型: {decision.name}")
                return
            
            if result.get('status') == 'created':
                self.logger.info(f"✅ 任務建立成功: {decision.name}")
                
                # 發布任務決策到 ROS topic
                self._publish_task_decision(decision)
            else:
                self.logger.error(f"❌ 任務建立失敗: {result.get('message', 'Unknown error')}")
                
        except Exception as e:
            self.logger.error(f"執行任務決策失敗: {e}")
    
    def _publish_task_decision(self, decision: TaskDecision):
        """發布任務決策到 ROS topic"""
        try:
            message = String()
            message.data = f"TaskDecision: {decision.name} | {decision.reason}"
            self.task_publisher.publish(message)
        except Exception as e:
            self.logger.error(f"發布任務決策失敗: {e}")
    
    def _publish_system_status(self):
        """發布系統狀態"""
        try:
            message = String()
            message.data = f"Simple WCS Engine Running | Flows: {len(self.business_flows)}"
            self.status_publisher.publish(message)
        except Exception as e:
            self.logger.error(f"發布系統狀態失敗: {e}")


def main(args=None):
    """主函數"""
    rclpy.init(args=args)
    
    try:
        engine = SimpleWCSEngine()
        rclpy.spin(engine)
    except KeyboardInterrupt:
        print("Simple WCS Engine 正在關閉...")
    except Exception as e:
        print(f"Simple WCS Engine 錯誤: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()