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
import asyncio
from datetime import datetime, timezone

# 導入統一系統組件
from .unified_decision_engine import UnifiedWCSDecisionEngine, TaskDecision
from .enhanced_database_client import EnhancedDatabaseClient
from .unified_task_manager import UnifiedTaskManager
from .rack_analyzer import RackAnalyzer

# 導入 ROS 2 訊息
from std_msgs.msg import String


class AIWCSNode(Node):
    """AI WCS 主控制節點 - 統一決策引擎版本"""
    
    def __init__(self):
        super().__init__('ai_wcs_unified_node')
        self.get_logger().info('🤖 AI WCS 統一決策引擎節點啟動')
        
        # 初始化統一系統組件
        self.decision_engine = UnifiedWCSDecisionEngine(self.get_logger())
        self.database_client = EnhancedDatabaseClient()
        self.task_manager = UnifiedTaskManager(self.get_logger())
        self.rack_analyzer = RackAnalyzer(self.get_logger())
        
        # 整合增強資料庫客戶端到決策引擎
        self._integrate_enhanced_database_client()
        
        # 系統配置
        self.config = {
            'decision_cycle_interval': 10.0,     # 決策週期間隔（秒）
            'task_cleanup_interval': 3600.0,    # 任務清理間隔（秒）
            'max_concurrent_tasks': 50,         # 最大並發任務數
            'enable_statistics_logging': True,   # 啟用統計日誌
            'enable_batch_optimization': True,   # 啟用批次最佳化
            'enable_opui_integration': True      # 啟用OPUI整合
        }
        
        # 建立ROS 2發布者
        self.system_status_pub = self.create_publisher(
            String, '/ai_wcs/unified_system_status', 10
        )
        self.decision_metrics_pub = self.create_publisher(
            String, '/ai_wcs/unified_decision_metrics', 10
        )
        self.task_updates_pub = self.create_publisher(
            String, '/ai_wcs/unified_task_updates', 10
        )
        
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
        """執行統一決策週期 - 七大業務流程統一調度"""
        if not self.system_status['is_running']:
            return
        
        cycle_start_time = datetime.now(timezone.utc)
        cycle_id = self.system_status['total_cycles'] + 1
        
        self.get_logger().info(f'🔄 開始執行統一WCS決策週期 #{cycle_id}')
        
        try:
            # 檢查系統負載
            if not self._check_system_capacity():
                self.get_logger().warning('系統負載過高，跳過本次決策週期')
                return
            
            # 使用異步執行統一決策週期
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            
            try:
                # 執行統一決策引擎
                decisions = loop.run_until_complete(
                    self.decision_engine.run_unified_decision_cycle()
                )
                
                if not decisions:
                    self.get_logger().debug('本次統一決策週期無新任務產生')
                    return
                
                # 使用統一任務管理器批次創建任務
                creation_results = loop.run_until_complete(
                    self.task_manager.create_tasks_from_decisions(decisions)
                )
                
                # 統計創建結果
                successful_tasks = [r for r in creation_results if r.success]
                failed_tasks = [r for r in creation_results if not r.success]
                
                # 發布任務更新
                if successful_tasks:
                    self.publish_task_updates(successful_tasks, decisions)
                
                # 發布決策指標
                self.publish_decision_metrics()
                
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
                
            finally:
                loop.close()
            
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
        
        for task in tasks:
            try:
                # TODO: 實作資料庫提交邏輯
                # success = self._insert_task_to_db(task)
                success = True  # 暫時模擬成功
                
                if success:
                    submitted_count += 1
                    self.get_logger().debug(f'任務已提交到資料庫: {task.task_id}')
                else:
                    self.get_logger().error(f'任務提交失敗: {task.task_id}')
                    
            except Exception as e:
                self.get_logger().error(f'提交任務時發生錯誤: {task.task_id} - {e}')
        
        return submitted_count
    
    def publish_decision_metrics(self):
        """發布決策指標"""
        try:
            # 整合所有組件的統計資料
            decision_stats = self.decision_engine.get_decision_statistics()
            task_stats = self.task_manager.get_task_statistics()
            db_stats = self.database_client.get_query_statistics()
            
            metrics = {
                'decision_engine': decision_stats,
                'task_manager': task_stats,
                'database_client': db_stats,
                'system_metrics': {
                    'total_cycles': self.system_status['total_cycles'],
                    'task_success_rate': task_stats['stats']['created'] / max(task_stats['stats']['created'] + task_stats['stats']['failed'], 1)
                },
                'timestamp': datetime.now(timezone.utc).isoformat()
            }
            
            msg = String()
            msg.data = json.dumps(metrics, ensure_ascii=False)
            self.decision_metrics_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'❌ 發布決策指標失敗: {e}')
    
    def publish_task_updates(self, creation_results: List, decisions: List[TaskDecision]):
        """發布任務更新"""
        try:
            task_data = {
                'created_tasks': [
                    {
                        'task_id': result.task_id,
                        'work_id': decisions[i].work_id,
                        'task_type': decisions[i].task_type,
                        'priority': decisions[i].priority,
                        'created_at': result.created_at.isoformat()
                    }
                    for i, result in enumerate(creation_results) if result.success
                ],
                'system_type': 'unified_wcs',
                'cycle_number': self.system_status['total_cycles'],
                'timestamp': datetime.now(timezone.utc).isoformat()
            }
            
            msg = String()
            msg.data = json.dumps(task_data, ensure_ascii=False)
            self.task_updates_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'❌ 發布任務更新失敗: {e}')
    
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
            
            # 發布系統狀態
            self.publish_unified_system_status()
            
        except Exception as e:
            self.get_logger().error(f'❌ 記錄統計資料失敗: {e}')
    
    def publish_unified_system_status(self):
        """發布統一系統狀態"""
        try:
            status_data = {
                'system_status': self.system_status,
                'system_type': 'unified_wcs_decision_engine',
                'components': {
                    'unified_decision_engine': 'active',
                    'enhanced_database_client': 'active',
                    'unified_task_manager': 'active',
                    'rack_analyzer': 'active'
                },
                'config': self.config,
                'timestamp': datetime.now(timezone.utc).isoformat()
            }
            
            msg = String()
            msg.data = json.dumps(status_data, ensure_ascii=False)
            self.system_status_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'❌ 發布統一系統狀態失敗: {e}')
    
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
        
        # 標記所有活動任務為取消狀態
        for task_id in list(self.task_manager.active_tasks.keys()):
            self.task_manager.update_task_status(task_id, TaskStatus.CANCELLING)
        
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
        # TODO: 可以添加以下服務
        # - /ai_wcs/get_status (獲取系統狀態)
        # - /ai_wcs/pause_system (暫停系統)
        # - /ai_wcs/resume_system (恢復系統)
        # - /ai_wcs/emergency_stop (緊急停止)
        # - /ai_wcs/update_config (更新配置)
        pass
    
    def _create_publishers(self):
        """創建ROS 2發布者"""
        # TODO: 可以添加以下發布者
        # - /ai_wcs/system_status (系統狀態)
        # - /ai_wcs/task_updates (任務更新)
        # - /ai_wcs/decision_metrics (決策指標)
        pass
    
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