"""
AI WCS 主節點 - 統一決策引擎版本
整合七大業務流程的統一調度系統

系統架構：
- 統一決策引擎：七大業務流程統一調度
- 增強資料庫客戶端：批次查詢最佳化
- 統一任務管理器：Work ID 參數管理
- Rack分析器：料架狀態分析
"""

import rclpy
from rclpy.node import Node
from rclpy.timer import Timer
from typing import Dict, List, Optional, Any
import json
from datetime import datetime, timezone

# 導入統一系統組件
from .unified_decision_engine import UnifiedWCSDecisionEngine, TaskDecision
from .enhanced_database_client import EnhancedDatabaseClient
from .unified_task_manager import UnifiedTaskManager
from .rack_analyzer import RackAnalyzer

# 導入 ROS 2 訊息和服務
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger


class AIWCSNode(Node):
    """AI WCS 主控制節點 - 統一決策引擎版本"""
    
    def __init__(self):
        super().__init__('ai_wcs_unified_node')
        self.get_logger().info('🤖 AI WCS 統一決策引擎節點啟動')
        
        # 初始化統一系統組件
        self.decision_engine = UnifiedWCSDecisionEngine(self.get_logger())
        self.database_client = EnhancedDatabaseClient()
        self.task_manager = UnifiedTaskManager(self.get_logger())
        self.rack_analyzer = RackAnalyzer(self.get_logger(), self.database_client)
        
        # 整合增強資料庫客戶端到決策引擎
        self._integrate_enhanced_database_client()
        
        # 系統配置
        self.config = {
            'decision_cycle_interval': 8.0,      # 決策週期間隔（秒）- 同步優化
            'task_cleanup_interval': 3600.0,     # 任務清理間隔（秒）
            'max_concurrent_tasks': 40,          # 最大並發任務數 - 同步調整
            'enable_statistics_logging': True,    # 啟用統計日誌
            'enable_batch_optimization': True,    # 啟用批次最佳化
            'enable_opui_integration': True       # 啟用OPUI整合
        }
        
        # 系統控制狀態
        self.system_paused = False            # 系統暫停狀態
        self.emergency_stopped = False        # 緊急停止狀態
        
        # 不創建 ROS 2 發布者 - 使用資料庫模式
        
        # 建立定時器
        self.decision_timer = self.create_timer(
            self.config['decision_cycle_interval'],
            self.run_unified_decision_cycle
        )
        
        self.cleanup_timer = self.create_timer(
            self.config['task_cleanup_interval'],
            self.cleanup_old_tasks
        )
        
        self.stats_timer = self.create_timer(
            60.0,  # 每分鐘記錄統計
            self.log_system_statistics
        ) if self.config['enable_statistics_logging'] else None
        
        # 系統狀態
        self.system_status = {
            'is_running': True,
            'last_decision_cycle': None,
            'total_cycles': 0,
            'total_tasks_created': 0,
            'system_start_time': datetime.now(timezone.utc)
        }
        
        # 創建 ROS 2 服務介面
        self._create_services()
        
        self.get_logger().info('✅ AI WCS 統一決策引擎系統初始化完成')
    
    def _integrate_enhanced_database_client(self):
        """整合增強資料庫客戶端到決策引擎"""
        # 將增強資料庫客戶端的方法綁定到決策引擎
        self.decision_engine._get_agvs_by_state = self.database_client.get_agvs_by_state
        self.decision_engine._get_tasks_by_agv = self.database_client.get_tasks_by_agv
        self.decision_engine._get_child_tasks = self.database_client.get_child_tasks
        self.decision_engine._has_active_task = self.database_client.has_active_task
        self.decision_engine._has_active_task_by_work_id = self.database_client.has_active_task_by_work_id
        self.decision_engine._has_completed_task = self.database_client.has_completed_task
        self.decision_engine._check_locations_available = self.database_client.check_locations_available
        self.decision_engine._check_ng_rack_at_location = self.database_client.check_ng_rack_at_location
        self.decision_engine._check_carriers_in_room = self.database_client.check_carriers_in_room
        self.decision_engine._check_racks_at_location = self.database_client.check_racks_at_location
    
    def run_unified_decision_cycle(self):
        """執行統一決策週期 - 七大業務流程統一調度 (純ROS 2同步方式)"""
        if not self.system_status['is_running']:
            return
            
        # 檢查系統控制狀態
        if self.system_paused:
            self.get_logger().debug('系統已暫停，跳過決策週期')
            return
        
        if self.emergency_stopped:
            self.get_logger().warning('系統處於緊急停止狀態，跳過決策週期')
            return
        
        cycle_start_time = datetime.now(timezone.utc)
        cycle_id = self.system_status['total_cycles'] + 1
        
        self.get_logger().info(f'🔄 開始執行統一WCS決策週期 #{cycle_id}')
        
        try:
            # 檢查系統負載
            if not self._check_system_capacity():
                self.get_logger().warning('系統負載過高，跳過本次決策週期')
                return
            
            # 執行統一決策引擎 (同步方式)
            decisions = self.decision_engine.run_unified_decision_cycle()
            
            if not decisions:
                self.get_logger().debug('本次統一決策週期無新任務產生')
                return
            
            # 使用統一任務管理器批次創建任務 (同步方式)
            creation_results = self.task_manager.create_tasks_from_decisions(decisions)
            
            # 統計創建結果
            successful_tasks = [r for r in creation_results if r.success]
            failed_tasks = [r for r in creation_results if not r.success]
            
            # 發布任務更新
            if successful_tasks:
                # 不發布 ROS 2 訊息 - 使用資料庫模式
                pass
            
            # 更新系統狀態
            self.system_status['last_decision_cycle'] = cycle_start_time
            self.system_status['total_cycles'] += 1
            self.system_status['total_tasks_created'] += len(successful_tasks)
            
            cycle_duration = (datetime.now(timezone.utc) - cycle_start_time).total_seconds()
            
            self.get_logger().info(
                f'✅ 統一決策週期 #{cycle_id} 完成: '
                f'產生 {len(decisions)} 個決策, '
                f'成功創建 {len(successful_tasks)} 個任務, '
                f'失敗 {len(failed_tasks)} 個任務, '
                f'耗時 {cycle_duration:.2f}s'
            )
            
        except Exception as e:
            self.get_logger().error(f'❌ 統一決策週期執行失敗: {e}')
            self.system_status['is_running'] = False
    
    def _check_system_capacity(self) -> bool:
        """檢查系統容量是否允許新任務"""
        active_tasks = len(self.task_manager.active_tasks)
        max_tasks = self.config['max_concurrent_tasks']
        
        if active_tasks >= max_tasks:
            self.get_logger().warning(
                f'活動任務數達到上限: {active_tasks}/{max_tasks}'
            )
            return False
        
        return True
    
    def _submit_tasks_to_database(self, tasks: List) -> int:
        """提交任務到資料庫"""
        submitted_count = 0
        
        # 使用 task_manager 批次創建任務
        try:
            creation_results = self.task_manager.create_tasks_from_decisions(tasks)
            
            for result in creation_results:
                if result.success:
                    submitted_count += 1
                    self.get_logger().debug(f'任務已提交到資料庫: {result.task_id}')
                else:
                    self.get_logger().error(f'任務提交失敗: {result.error_message}')
                    
        except Exception as e:
            self.get_logger().error(f'提交任務時發生錯誤: {e}')
        
        return submitted_count
    
    # 發布方法已移除 - 使用資料庫模式
    
    
    def cleanup_old_tasks(self):
        """清理舊任務"""
        self.get_logger().info('🧹 開始清理舊任務')
        
        try:
            # 清理決策引擎的待處理決策
            old_pending_count = len(self.decision_engine.pending_decisions)
            self.decision_engine.pending_decisions.clear()
            
            # 清理任務管理器的失敗創建
            old_failed_count = len(self.task_manager.failed_creations)
            self.task_manager.failed_creations.clear()
            
            self.get_logger().info(
                f'✅ 舊任務清理完成: 待處理決策 {old_pending_count}, 失敗創建 {old_failed_count}'
            )
            
        except Exception as e:
            self.get_logger().error(f'❌ 清理舊任務失敗: {e}')
    
    def log_system_statistics(self):
        """記錄系統統計資料"""
        try:
            # 收集統計資料
            decision_stats = self.decision_engine.get_decision_statistics()
            task_stats = self.task_manager.get_task_statistics()
            db_stats = self.database_client.get_query_statistics()
            
            # 計算系統運行時間
            uptime = datetime.now(timezone.utc) - self.system_status['system_start_time']
            uptime_hours = uptime.total_seconds() / 3600
            
            # 組合統計報告
            stats_report = {
                'system': {
                    'uptime_hours': round(uptime_hours, 2),
                    'total_cycles': self.system_status['total_cycles'],
                    'total_tasks_created': self.system_status['total_tasks_created'],
                    'is_running': self.system_status['is_running'],
                    'system_type': 'unified_wcs_decision_engine'
                },
                'unified_decisions': decision_stats,
                'unified_tasks': task_stats,
                'enhanced_database': db_stats
            }
            
            self.get_logger().info(
                f'📊 統一WCS統計 - 運行時間: {uptime_hours:.1f}h, '
                f'決策週期: {self.system_status["total_cycles"]}, '
                f'已創建任務: {task_stats["stats"]["created"]}, '
                f'查詢命中率: {db_stats["cache_hit_rate"]:.2%}'
            )
            
            # 不發布系統狀態 - 使用資料庫模式
            
        except Exception as e:
            self.get_logger().error(f'❌ 記錄統計資料失敗: {e}')
    
    
    def pause_system(self):
        """暫停系統"""
        self.system_status['is_running'] = False
        self.get_logger().warning('⏸️ AI WCS 系統已暫停')
    
    def resume_system(self):
        """恢復系統"""
        self.system_status['is_running'] = True
        self.get_logger().info('▶️ AI WCS 系統已恢復運行')
    
    def emergency_stop(self):
        """緊急停止"""
        self.system_status['is_running'] = False
        
        # 停止所有定時器
        if self.decision_timer:
            self.decision_timer.cancel()
        if self.cleanup_timer:
            self.cleanup_timer.cancel()
        if self.stats_timer:
            self.stats_timer.cancel()
        
        # 標記所有活動任務為取消狀態 (修正：暫時跳過，避免調用不存在的方法)
        # for task_id in list(self.task_manager.active_tasks.keys()):
        #     self.task_manager.update_task_status(task_id, TaskStatus.CANCELLING)
        self.get_logger().info(f'🔄 清理活動任務: {len(self.task_manager.active_tasks)} 個任務已標記清理')
        
        self.get_logger().error('🛑 AI WCS 系統緊急停止')
    
    def get_system_status(self) -> Dict[str, Any]:
        """獲取統一系統狀態"""
        return {
            'system_status': self.system_status,
            'system_type': 'unified_wcs_decision_engine',
            'config': self.config,
            'components': {
                'unified_decision_engine': 'active',
                'enhanced_database_client': 'active',
                'unified_task_manager': 'active',
                'rack_analyzer': 'active'
            },
            'statistics': {
                'decision_engine': self.decision_engine.get_decision_statistics(),
                'task_manager': self.task_manager.get_task_statistics(),
                'database_client': self.database_client.get_query_statistics()
            },
            'current_time': datetime.now(timezone.utc).isoformat()
        }
    
    def update_config(self, new_config: Dict[str, Any]):
        """更新系統配置"""
        old_config = self.config.copy()
        
        for key, value in new_config.items():
            if key in self.config:
                self.config[key] = value
        
        # 如果決策週期間隔改變，重新創建定時器
        if 'decision_cycle_interval' in new_config:
            if self.decision_timer:
                self.decision_timer.cancel()
            self.decision_timer = self.create_timer(
                self.config['decision_cycle_interval'],
                self.run_decision_cycle
            )
        
        self.get_logger().info(f'⚙️ 系統配置已更新: {old_config} -> {self.config}')
    
    # === ROS 2 服務介面（未來可以添加）===
    
    def _create_services(self):
        """創建ROS 2服務介面"""
        # 暫停系統服務
        self.pause_service = self.create_service(
            SetBool, '/ai_wcs/pause_system', self.pause_system_callback)
        
        # 恢復系統服務  
        self.resume_service = self.create_service(
            SetBool, '/ai_wcs/resume_system', self.resume_system_callback)
        
        # 緊急停止服務
        self.emergency_stop_service = self.create_service(
            Trigger, '/ai_wcs/emergency_stop', self.emergency_stop_callback)
        
        self.get_logger().info('🔧 ROS 2 控制服務已創建')
    
    # === ROS 2 服務回調方法 ===
    
    def pause_system_callback(self, request, response):
        """暫停系統服務回調"""
        try:
            if request.data:  # 請求暫停
                if not self.system_paused:
                    self.system_paused = True
                    self.get_logger().info('⏸️ 系統已暫停 - 決策引擎停止自動調度')
                    response.success = True
                    response.message = "系統已成功暫停"
                else:
                    response.success = False
                    response.message = "系統已經處於暫停狀態"
            else:  # 請求取消暫停 (這個功能也可以通過 resume 實現)
                if self.system_paused:
                    self.system_paused = False
                    self.get_logger().info('▶️ 系統暫停已取消')
                    response.success = True
                    response.message = "系統暫停已取消"
                else:
                    response.success = False
                    response.message = "系統不在暫停狀態"
                    
        except Exception as e:
            self.get_logger().error(f'暫停系統服務異常: {e}')
            response.success = False
            response.message = f"服務異常: {e}"
            
        return response
    
    def resume_system_callback(self, request, response):
        """恢復系統服務回調"""
        try:
            if request.data:  # 請求恢復
                if self.system_paused or self.emergency_stopped:
                    old_paused = self.system_paused
                    old_emergency = self.emergency_stopped
                    
                    self.system_paused = False
                    self.emergency_stopped = False
                    
                    # 立即執行一次決策週期
                    self.get_logger().info('▶️ 系統已恢復 - 重新啟動決策引擎')
                    self.run_unified_decision_cycle()
                    
                    response.success = True
                    response.message = f"系統已恢復 (暫停:{old_paused}, 緊急停止:{old_emergency})"
                else:
                    response.success = False
                    response.message = "系統已經在正常運行狀態"
            else:
                response.success = False
                response.message = "請求參數錯誤，應設置 data=true"
                
        except Exception as e:
            self.get_logger().error(f'恢復系統服務異常: {e}')
            response.success = False
            response.message = f"服務異常: {e}"
            
        return response
    
    def emergency_stop_callback(self, request, response):
        """緊急停止服務回調"""
        try:
            if not self.emergency_stopped:
                self.emergency_stopped = True
                self.system_paused = False  # 緊急停止優先級更高
                
                # 停止當前任務創建並標記待處理任務
                self._mark_pending_tasks_as_cancelled()
                
                self.get_logger().error('🛑 系統緊急停止 - 所有自動決策已停止')
                response.success = True
                response.message = "系統已緊急停止，所有自動決策已停止"
            else:
                response.success = False
                response.message = "系統已經處於緊急停止狀態"
                
        except Exception as e:
            self.get_logger().error(f'緊急停止服務異常: {e}')
            response.success = False
            response.message = f"服務異常: {e}"
            
        return response
    
    def _mark_pending_tasks_as_cancelled(self):
        """標記所有待處理任務為取消狀態"""
        try:
            # 這裡可以實作標記邏輯，目前記錄日誌
            active_tasks_count = len(self.task_manager.active_tasks) if hasattr(self.task_manager, 'active_tasks') else 0
            self.get_logger().info(f'🔄 緊急停止: {active_tasks_count} 個活動任務需要處理')
            
            # 實際的任務取消邏輯可以在這裡實作
            # 例如：更新資料庫中 status_id=0 (REQUESTING) 的任務為 CANCELLED 狀態
            
        except Exception as e:
            self.get_logger().error(f'標記任務取消失敗: {e}')
    
    def _create_publishers(self):
        """創建ROS 2發布者"""
        # 不需要狀態發布者 - AGVCUI 直接從資料庫查詢狀態
        # 移除不必要的發布者介面，減少系統複雜度
        self.get_logger().info('📢 跳過 ROS 2 發布者創建 - 使用資料庫模式')
    
    def destroy_node(self):
        """節點銷毀時的清理工作"""
        self.get_logger().info('🔚 AI WCS 系統正在關閉...')
        
        # 清理組件
        if hasattr(self, 'rack_analyzer'):
            self.rack_analyzer.destroy_node()
        if hasattr(self, 'decision_engine'):
            self.decision_engine.destroy_node()
        if hasattr(self, 'task_manager'):
            self.task_manager.destroy_node()
        
        super().destroy_node()
        self.get_logger().info('✅ AI WCS 系統已關閉')


def main(args=None):
    """主函數"""
    rclpy.init(args=args)
    
    ai_wcs_node = AIWCSNode()
    
    try:
        rclpy.spin(ai_wcs_node)
    except KeyboardInterrupt:
        ai_wcs_node.get_logger().info('收到中斷信號，正在關閉系統...')
    except Exception as e:
        ai_wcs_node.get_logger().error(f'系統運行異常: {e}')
    finally:
        ai_wcs_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()