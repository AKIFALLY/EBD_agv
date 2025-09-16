#!/usr/bin/env python3
"""
測試檔案: test_flow_wcs_node.py
用途: 執行 flow_wcs_node 並測試 rack_rotation_room_inlet 流程
創建日期: 2025-08-14
狀態: 臨時測試檔案，功能驗證後可刪除
"""

import rclpy
from rclpy.node import Node
from flow_wcs_interfaces.srv import ExecuteFlow
import time
import sys

class FlowWCSTestClient(Node):
    def __init__(self):
        super().__init__('flow_wcs_test_client')
        self.client = self.create_client(ExecuteFlow, '/flow_wcs/execute_flow')
        
        # 等待服務啟動
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 flow_wcs 服務啟動...')
    
    def execute_flow(self, flow_id: str, work_id: str = None):
        """執行流程"""
        request = ExecuteFlow.Request()
        request.flow_id = flow_id
        if work_id:
            request.work_id = work_id
        
        self.get_logger().info(f'🚀 執行流程: {flow_id}')
        future = self.client.call_async(request)
        
        # 等待結果
        rclpy.spin_until_future_complete(self, future)
        
        response = future.result()
        if response:
            self.get_logger().info(f'✅ 流程執行狀態: {response.status}')
            if response.message:
                self.get_logger().info(f'📝 訊息: {response.message}')
            if response.result:
                self.get_logger().info(f'📊 結果: {response.result}')
            return response.success
        else:
            self.get_logger().error('❌ 服務呼叫失敗')
            return False

def main():
    print("\n" + "="*60)
    print("🔧 Flow WCS Node 測試")
    print("="*60 + "\n")
    
    rclpy.init()
    
    try:
        # 創建測試客戶端
        client = FlowWCSTestClient()
        
        # 測試 rack_rotation_room_inlet 流程
        print("\n📋 測試流程: rack_rotation_room_inlet")
        print("-"*40)
        
        success = client.execute_flow('rack_rotation_room_inlet', '220001')
        
        if success:
            print("\n✅ 流程執行成功！")
            
            # 等待一下讓資料庫更新
            time.sleep(1)
            
            # 檢查任務是否創建
            print("\n🔍 驗證結果...")
            # 這裡可以加入資料庫查詢來驗證任務是否創建
            
        else:
            print("\n❌ 流程執行失敗")
            
    except Exception as e:
        print(f"\n❌ 測試錯誤: {e}")
        import traceback
        traceback.print_exc()
        
    finally:
        rclpy.shutdown()
    
    print("\n" + "="*60)
    print("測試結束")
    print("="*60)

if __name__ == '__main__':
    main()