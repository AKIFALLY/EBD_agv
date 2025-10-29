"""
RCS Core - 機器人控制系統核心

⚠️ 系統級重構警告 (2025-07-29 簡化重構):
=====================================
此系統曾經歷大規模簡化重構，刪除了複雜的 WCS 適配器和優先度調度器
原先分散在多個模組的功能現在合併到 simple_ct_manager 和 simple_kuka_manager

🔴 絕對不可移除的功能：
1. KukaManager 的回調機制 (on_robot_update, on_container_update)
2. CtManager 的 AGV 狀態監控 (訂閱 /agv/state_change)
3. 1秒定時器主迴圈 (main_loop)

這些機制是前後端即時同步的基礎，失去它們會導致系統無法正常運作
"""
import rclpy
from rclpy.node import Node
from db_proxy.connection_pool_manager import ConnectionPoolManager
from rcs.simple_kuka_manager import KukaManager
from rcs.simple_ct_manager import CtManager
import signal
import sys


class RcsCore(Node):

    def __init__(self):
        super().__init__("rcs_core")
        self.get_logger().info("簡化版 RCS Core 節點啟動中...")
        
        # 標記節點是否正在關閉
        self.is_shutting_down = False

        # 初始化資料庫連線池
        try:
            self.db_pool = ConnectionPoolManager(
                'postgresql+psycopg2://agvc:password@192.168.100.254/agvc')
            self.get_logger().info("資料庫連線池已建立。")
        except Exception as e:
            self.get_logger().fatal(f"建立資料庫連線池失敗: {e}")
            # 如果資料庫連線失敗，可能需要決定是否要讓節點繼續執行
            self.db_pool = None

        # --- 初始化簡化的車隊管理器 ---
        # 簡化的 KUKA 車隊管理器
        # ⚠️ 重要：KukaManager 包含機器人位置和容器狀態更新功能
        # 這些功能曾在重構中被誤刪，已於 commit d77f8275 恢復
        self.kuka_manager = KukaManager(self)

        # 簡化的 CT 車隊管理器
        # ⚠️ 重要：CtManager 訂閱 AGV 狀態變更，同步到資料庫
        self.ct_manager = CtManager(self)

        # 1秒定時器
        self.timer_1s = self.create_timer(1.0, self.main_loop)
        
        self.get_logger().info("✅ 簡化版 RCS Core 節點啟動完成")

    def main_loop(self):
        """主迴圈：每秒執行一次的任務派發"""
        # 如果正在關閉，不執行任務
        if self.is_shutting_down:
            return
            
        self.get_logger().debug("1秒定時器觸發 (timer_1s)")

        # KUKA 車隊任務派發 (簡化版)
        self.kuka_manager.dispatch()

        # CT 車隊任務派發 (簡化版)
        self.ct_manager.dispatch()
    
    def shutdown(self):
        """優雅地關閉節點"""
        if self.is_shutting_down:
            return
        
        self.is_shutting_down = True
        self.get_logger().info("🛑 正在優雅地關閉 RCS Core 節點...")
        
        # 停止定時器
        if hasattr(self, 'timer_1s'):
            self.timer_1s.cancel()
            self.get_logger().info("定時器已停止")
        
        # 停止 KUKA 管理器
        if hasattr(self, 'kuka_manager') and self.kuka_manager:
            try:
                self.kuka_manager.stop_monitoring()
                self.get_logger().info("KUKA Manager 已停止")
            except Exception as e:
                self.get_logger().error(f"停止 KUKA 管理器時出錯: {e}")
        
        # 停止 CT 管理器
        if hasattr(self, 'ct_manager') and self.ct_manager:
            try:
                self.ct_manager.shutdown()
                self.get_logger().info("CT Manager 已停止")
            except Exception as e:
                self.get_logger().error(f"停止 CT 管理器時出錯: {e}")
        
        # 關閉資料庫連線池
        if hasattr(self, 'db_pool') and self.db_pool:
            try:
                self.db_pool.shutdown()
                self.get_logger().info("資料庫連線池已關閉")
            except Exception as e:
                self.get_logger().error(f"關閉資料庫連線池時出錯: {e}")
        
        self.get_logger().info("✅ RCS Core 節點已優雅關閉")


def main(args=None):
    rclpy.init(args=args)
    node = RcsCore()
    
    # 設置信號處理器
    def signal_handler(sig, frame):
        """處理 SIGTERM 和 SIGINT 信號"""
        signal_name = signal.Signals(sig).name
        node.get_logger().info(f"🛑 收到 {signal_name} 信號，正在優雅關閉...")
        # 不要在這裡呼叫 rclpy.shutdown() 和 sys.exit()
        # 讓 main 函數的 finally 區塊處理
        raise KeyboardInterrupt(f"收到 {signal_name} 信號")
    
    # 註冊信號處理器
    signal.signal(signal.SIGTERM, signal_handler)  # kill 命令的默認信號
    signal.signal(signal.SIGINT, signal_handler)   # Ctrl+C
    
    try:
        # 只有在資料庫連線成功時才 spin
        if node.db_pool:
            rclpy.spin(node)
        else:
            node.get_logger().fatal("因資料庫連線失敗，節點無法啟動。")
    except KeyboardInterrupt as e:
        node.get_logger().info(f"🛑 正在關閉 RCS Core 節點: {e}")
    except Exception as e:
        node.get_logger().error(f"❌ 發生異常: {e}")
    finally:
        # 確保節點正確關閉
        try:
            node.get_logger().info("🔄 執行清理程序...")
            node.shutdown()
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            node.get_logger().info("✅ RCS Core 節點已完全關閉")
        except Exception as e:
            print(f"清理時發生錯誤: {e}")
        
    # 確保程式完全退出
    sys.exit(0)


if __name__ == '__main__':
    main()
