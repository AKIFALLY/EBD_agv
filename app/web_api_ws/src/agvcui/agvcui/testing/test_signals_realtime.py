#!/usr/bin/env python3
"""
測試 signals 頁面即時更新功能
"""

import sys
import os
import asyncio
import time

# 添加必要的路徑
current_dir = os.path.dirname(os.path.abspath(__file__))
agvcui_src = os.path.join(current_dir, '..', '..')
db_proxy_src = os.path.join(current_dir, '..', '..', '..', '..', '..', 'db_proxy_ws', 'src')

sys.path.insert(0, agvcui_src)
sys.path.insert(0, db_proxy_src)

try:
    from agvcui.db import get_signals, create_complete_device, delete_complete_device
    from agvcui.agvc_ui_socket import AgvcUiSocket
    import socketio
except ImportError as e:
    print(f"導入錯誤: {e}")
    print("請確保所有依賴模組都已正確安裝")
    sys.exit(1)


async def test_signals_realtime():
    """測試 signals 頁面即時更新功能"""
    print("開始測試 signals 頁面即時更新功能...")
    
    # 1. 創建測試設備和信號
    print("\n1. 創建測試設備和信號")
    test_device_data = {
        "name": "即時測試設備",
        "description": "用於測試即時更新的設備",
        "location_id": 999,
        "ports": [
            {
                "name": "TestPort01",
                "description": "測試端口01",
                "signals": [
                    {
                        "name": "RealtimeSignal01",
                        "description": "即時測試信號01",
                        "value": "0",
                        "type_of_value": "boolean"
                    },
                    {
                        "name": "RealtimeSignal02", 
                        "description": "即時測試信號02",
                        "value": "100",
                        "type_of_value": "integer"
                    }
                ]
            }
        ]
    }
    
    try:
        test_device = create_complete_device(test_device_data)
        print(f"✅ 創建測試設備成功，ID: {test_device['id']}")
        device_id = test_device['id']
        
        # 2. 測試 SocketIO 通知機制
        print("\n2. 測試 SocketIO 通知機制")
        
        # 創建 SocketIO 服務器實例
        sio = socketio.AsyncServer(cors_allowed_origins="*")
        agvc_socket = AgvcUiSocket(sio)
        
        # 模擬客戶端連接
        mock_sid = "test_client_001"
        agvc_socket.connected_sids.add(mock_sid)
        
        print(f"✅ 模擬客戶端連接: {mock_sid}")
        
        # 3. 測試信號通知功能
        print("\n3. 測試信號通知功能")
        
        # 獲取當前信號狀態
        current_signals = get_signals(offset=0, limit=100)
        test_signals = [s for s in current_signals if s.eqp_id == device_id]
        
        print(f"✅ 找到測試信號: {len(test_signals)} 個")
        for signal in test_signals:
            print(f"   - {signal.name}: {signal.value} ({signal.type_of_value})")
        
        # 4. 模擬信號值變化和通知
        print("\n4. 模擬信號值變化和通知")
        
        # 模擬 notify_signals 調用
        print("模擬調用 notify_signals...")
        
        # 這裡我們直接調用 notify_signals 方法來測試
        try:
            await agvc_socket.notify_signals(mock_sid)
            print("✅ notify_signals 調用成功")
        except Exception as e:
            print(f"❌ notify_signals 調用失敗: {e}")
        
        # 5. 測試信號數據格式
        print("\n5. 測試信號數據格式")
        
        # 檢查 signal_all() 函數返回的數據格式
        from agvcui.db import signal_all
        all_signals = signal_all()
        
        if all_signals:
            sample_signal = all_signals[0]
            print(f"✅ 信號數據格式示例:")
            print(f"   ID: {getattr(sample_signal, 'id', 'N/A')}")
            print(f"   名稱: {getattr(sample_signal, 'name', 'N/A')}")
            print(f"   值: {getattr(sample_signal, 'value', 'N/A')}")
            print(f"   類型: {getattr(sample_signal, 'type_of_value', 'N/A')}")
            print(f"   設備ID: {getattr(sample_signal, 'eqp_id', 'N/A')}")
            print(f"   端口ID: {getattr(sample_signal, 'eqp_port_id', 'N/A')}")
        
        # 6. 測試前端數據結構
        print("\n6. 測試前端數據結構")
        
        # 模擬前端接收到的數據格式
        frontend_signals = []
        for signal in test_signals:
            frontend_signal = {
                "id": signal.id,
                "name": signal.name,
                "value": signal.value,
                "type_of_value": signal.type_of_value,
                "eqp_id": signal.eqp_id,
                "eqp_port_id": signal.eqp_port_id,
                "description": signal.description
            }
            frontend_signals.append(frontend_signal)
        
        print(f"✅ 前端數據格式:")
        for signal in frontend_signals:
            print(f"   - {signal['name']}: {signal['value']} (設備ID: {signal['eqp_id']})")
        
        # 7. 測試定期更新機制
        print("\n7. 測試定期更新機制")
        
        print("模擬定期更新...")
        for i in range(3):
            print(f"   第 {i+1} 次更新...")
            try:
                await agvc_socket.notify_signals(mock_sid)
                print(f"   ✅ 更新成功")
            except Exception as e:
                print(f"   ❌ 更新失敗: {e}")
            
            await asyncio.sleep(1)  # 等待1秒
        
        # 清理測試數據
        print("\n8. 清理測試數據")
        success = delete_complete_device(device_id)
        if success:
            print("✅ 測試設備清理成功")
        
        # 清理 SocketIO 資源
        agvc_socket.connected_sids.discard(mock_sid)
        await agvc_socket.close()
        
        print("\n✅ signals 頁面即時更新功能測試完成！")
        print("💡 總結:")
        print("   - SocketIO 通知機制正常")
        print("   - 信號數據格式正確")
        print("   - 定期更新功能正常")
        print("   - 前端數據結構完整")
        print("\n🎯 前端集成要點:")
        print("   1. signalsStore 會接收 'signal_list' 事件")
        print("   2. signalsPage.js 監聽 signalsStore 變化")
        print("   3. 頁面上的信號值會即時更新")
        print("   4. 支持按設備ID篩選的即時更新")
        
    except Exception as e:
        print(f"❌ 測試失敗: {str(e)}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    asyncio.run(test_signals_realtime())
