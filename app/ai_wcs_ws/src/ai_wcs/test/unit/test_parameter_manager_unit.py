#!/usr/bin/env python3
"""
WorkIDParameterManager 單元測試
驗證 Work ID 參數管理器的單一方法功能
"""

import unittest
import sys
from unittest.mock import Mock, patch

# 添加路徑
sys.path.append('/app/ai_wcs_ws/src')

from ai_wcs.unified_task_manager import WorkIDParameterManager
from ai_wcs.unified_decision_engine import TaskDecision, BusinessFlowPriority


class TestParameterManagerUnit(unittest.TestCase):
    """Work ID 參數管理器單元測試"""
    
    def setUp(self):
        """測試前設定"""
        self.manager = WorkIDParameterManager()
    
    def test_work_id_mappings_structure(self):
        """測試 Work ID 映射結構"""
        # 檢查映射是否存在
        self.assertIsInstance(self.manager.WORK_ID_MAPPINGS, dict)
        self.assertGreater(len(self.manager.WORK_ID_MAPPINGS), 0)
        
        # 檢查關鍵 Work ID
        expected_work_ids = ['100001', '100002', '220001', '230001', '2000102', '2000201']
        
        for work_id in expected_work_ids:
            with self.subTest(work_id=work_id):
                self.assertIn(work_id, self.manager.WORK_ID_MAPPINGS,
                             f"Work ID {work_id} 應該存在於映射中")
    
    def test_business_flow_work_ids_structure(self):
        """測試業務流程 Work ID 結構"""
        # 檢查業務流程映射是否存在
        self.assertIsInstance(self.manager.BUSINESS_FLOW_WORK_IDS, dict)
        
        # 檢查關鍵業務流程（基於實際實現）
        expected_flows = [
            'agv_rotation',
            'ng_rack_recycling',
            'full_rack_to_manual',
            'manual_area_transport',  # 實際是 manual_area_transport 而不是 manual_transport
            'system_to_room',
            'empty_rack_transfer',
            'manual_empty_recycling',
            'opui_call_empty',
            'opui_dispatch_full'
        ]
        
        for flow in expected_flows:
            with self.subTest(flow=flow):
                self.assertIn(flow, self.manager.BUSINESS_FLOW_WORK_IDS,
                             f"業務流程 {flow} 應該存在於映射中")
    
    def test_get_work_id_info_valid(self):
        """測試取得有效的 Work ID 資訊"""
        # 測試 OPUI 叫空車
        info = self.manager.get_work_id_info('100001')
        
        self.assertIsInstance(info, dict)
        self.assertEqual(info['name'], 'opui-call-empty')
        self.assertEqual(info['category'], 'opui_operations')
        self.assertEqual(info['function'], 'rack_move')
        self.assertEqual(info['api'], 'submit_mission')
        
        # 測試主要料架操作（基於實際實現）
        info = self.manager.get_work_id_info('220001')
        
        self.assertIsInstance(info, dict)
        self.assertEqual(info['name'], 'kuka-移動貨架')
        self.assertEqual(info['category'], 'rack_transport')  # 實際是 rack_transport
        self.assertEqual(info['function'], 'rack_move')
        self.assertEqual(info['api'], 'submit_mission')
    
    def test_get_work_id_info_invalid(self):
        """測試取得無效的 Work ID 資訊"""
        # 測試不存在的 Work ID（基於實際實現，返回空字典而不是 None）
        info = self.manager.get_work_id_info('999999')
        self.assertEqual(info, {})
        
        # 測試空字串
        info = self.manager.get_work_id_info('')
        self.assertEqual(info, {})
        
        # 測試 None
        info = self.manager.get_work_id_info(None)
        self.assertEqual(info, {})
    
    def test_work_id_mappings_content(self):
        """測試 Work ID 映射內容"""
        # 測試 220001 映射
        mapping_220001 = self.manager.WORK_ID_MAPPINGS.get('220001')
        self.assertIsNotNone(mapping_220001)
        self.assertEqual(mapping_220001['name'], 'kuka-移動貨架')
        self.assertEqual(mapping_220001['category'], 'rack_transport')
        
        # 測試 100001 映射
        mapping_100001 = self.manager.WORK_ID_MAPPINGS.get('100001')
        self.assertIsNotNone(mapping_100001)
        self.assertEqual(mapping_100001['name'], 'opui-call-empty')
        self.assertEqual(mapping_100001['category'], 'opui_operations')
    
    def test_build_kuka_rack_move_parameters(self):
        """測試建構 KUKA 料架移動參數"""
        decision = TaskDecision(
            work_id='220001',
            task_type='agv_rotation',
            priority=BusinessFlowPriority.AGV_ROTATION,
            source_location=10001,
            target_location=10001,
            nodes=[10001, 10011, 10001],
            agv_id=1,
            reason='測試 KUKA 參數'
        )
        
        params = self.manager.build_kuka_rack_move_parameters(decision)
        
        # 檢查 KUKA 特定參數
        self.assertEqual(params['work_id'], 220001)
        self.assertEqual(params['missionType'], 'RACK_MOVE')
        self.assertEqual(params['rotation_type'], '3_node_movement')
        self.assertEqual(params['agv_id'], 1)
        self.assertEqual(params['nodes'], [10001, 10011, 10001])
    
    def test_build_opui_call_empty_parameters(self):
        """測試建構 OPUI 叫空車參數"""
        decision = TaskDecision(
            work_id='100001',
            task_type='opui_call_empty',
            priority=BusinessFlowPriority.EMPTY_RACK_TRANSFER,  # 修正為實際存在的優先級
            source_location=91,
            target_location=95,
            parameters={
                'machine_id': 1,
                'space_num': 1,
                'client_id': 'test_client'
            },
            reason='測試 OPUI 叫空車'
        )
        
        params = self.manager.build_opui_call_empty_parameters(decision)
        
        # 檢查 OPUI 叫空車特定參數
        self.assertEqual(params['work_id'], 100001)
        self.assertEqual(params['task_type'], 'call_empty')
        self.assertEqual(params['machine_id'], 1)
        self.assertEqual(params['space_num'], 1)
        self.assertEqual(params['node_id'], 95)
        self.assertEqual(params['parking_space_status'], 1)
        self.assertEqual(params['client_id'], 'test_client')
    
    def test_build_opui_dispatch_full_parameters(self):
        """測試建構 OPUI 派滿車參數"""
        decision = TaskDecision(
            work_id='100002',
            task_type='opui_dispatch_full',
            priority=BusinessFlowPriority.EMPTY_RACK_TRANSFER,  # 修正為實際存在的優先級
            source_location=95,
            target_location=15,
            parameters={
                'machine_id': 1,
                'rack_id': 123,
                'room_id': 1,
                'side': 'left',
                'client_id': 'test_client'
            },
            reason='測試 OPUI 派滿車'
        )
        
        params = self.manager.build_opui_dispatch_full_parameters(decision)
        
        # 檢查 OPUI 派滿車特定參數（基於實際實現）
        self.assertEqual(params['work_id'], 100002)
        self.assertEqual(params['task_type'], 'dispatch_full')
        self.assertEqual(params['machine_id'], 1)
        self.assertEqual(params['rack_id'], 123)
        self.assertEqual(params['room_id'], 1)
        self.assertEqual(params['side'], 'left')
        self.assertEqual(params['client_id'], 'test_client')
        
        # 檢查其他實際參數
        self.assertEqual(params['function'], 'rack_move')
        self.assertEqual(params['api'], 'submit_mission')
        self.assertEqual(params['missionType'], 'RACK_MOVE')
        self.assertEqual(params['task_category'], 'opui_dispatch_full')
        self.assertIsInstance(params['nodes'], list)
        self.assertEqual(params['target_area'], 'system_prep_area')
    
    def test_kuka_rack_move_type_conversion(self):
        """測試 KUKA 料架移動參數類型轉換"""
        decision = TaskDecision(
            work_id='220001',  # 字串輸入
            task_type='agv_rotation',
            priority=BusinessFlowPriority.AGV_ROTATION,
            source_location=10,
            target_location=20,
            nodes=[10, 11, 10],
            agv_id=1
        )
        
        params = self.manager.build_kuka_rack_move_parameters(decision)
        
        # 檢查類型轉換
        self.assertIsInstance(params['work_id'], int)
        self.assertEqual(params['work_id'], 220001)
        
        # 檢查其他類型
        self.assertIsInstance(params['source_location'], int)
        self.assertIsInstance(params['target_location'], int)
        self.assertIsInstance(params['nodes'], list)
        self.assertIsInstance(params['agv_id'], int)
    
    def test_machine_parking_config(self):
        """測試機台停車格配置"""
        # 檢查 MACHINE_PARKING_CONFIG 是否存在
        self.assertIsInstance(self.manager.MACHINE_PARKING_CONFIG, dict)
        
        # 檢查是否有機台配置
        self.assertGreater(len(self.manager.MACHINE_PARKING_CONFIG), 0)
        
        # 檢查每個機台配置的結構
        for machine_id, config in self.manager.MACHINE_PARKING_CONFIG.items():
            with self.subTest(machine_id=machine_id):
                self.assertIsInstance(config, dict)
                # 檢查停車格配置
                if 'parking_spaces' in config:
                    self.assertIsInstance(config['parking_spaces'], dict)


if __name__ == '__main__':
    unittest.main()