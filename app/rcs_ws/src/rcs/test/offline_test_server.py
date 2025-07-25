#!/usr/bin/env python3
"""
離線測試服務器
提供獨立的 KUKA Fleet API 模擬服務器，用於離線測試和開發
"""
import json
import asyncio
import logging
from datetime import datetime, timezone
from typing import Dict, List, Any, Optional
from pathlib import Path
import argparse

try:
    from aiohttp import web, WSMsgType
    from aiohttp.web import Request, Response, WebSocketResponse
    AIOHTTP_AVAILABLE = True
except ImportError:
    AIOHTTP_AVAILABLE = False
    print("警告: aiohttp 未安裝，Web 服務器功能不可用")
    print("安裝命令: pip install aiohttp")

from mock_environment import MockTestEnvironment, MockKukaFleetAPI


class OfflineTestServer:
    """離線測試服務器"""
    
    def __init__(self, host: str = "localhost", port: int = 8080):
        self.host = host
        self.port = port
        self.app = None
        self.mock_env = MockTestEnvironment()
        self.kuka_api = self.mock_env.kuka_api
        self.websockets = set()
        self.logger = self._setup_logger()
        
        if AIOHTTP_AVAILABLE:
            self.app = web.Application()
            self._setup_routes()
    
    def _setup_logger(self):
        """設置日誌記錄器"""
        logger = logging.getLogger('OfflineTestServer')
        logger.setLevel(logging.INFO)
        
        handler = logging.StreamHandler()
        formatter = logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )
        handler.setFormatter(formatter)
        logger.addHandler(handler)
        
        return logger
    
    def _setup_routes(self):
        """設置路由"""
        if not self.app:
            return
        
        # KUKA Fleet API 模擬端點
        self.app.router.add_get('/api/robots', self._get_robots)
        self.app.router.add_get('/api/containers', self._get_containers)
        self.app.router.add_post('/api/missions/move', self._create_move_mission)
        self.app.router.add_post('/api/missions/rack_move', self._create_rack_move_mission)
        self.app.router.add_post('/api/missions/workflow', self._create_workflow_mission)
        self.app.router.add_get('/api/missions/{mission_id}', self._get_mission)
        self.app.router.add_get('/api/status', self._get_api_status)
        
        # 測試環境管理端點
        self.app.router.add_get('/test/environment', self._get_environment_state)
        self.app.router.add_post('/test/environment/reset', self._reset_environment)
        self.app.router.add_post('/test/environment/scenario', self._load_scenario)
        self.app.router.add_post('/test/simulate/network_issues', self._simulate_network_issues)
        self.app.router.add_post('/test/simulate/high_latency', self._simulate_high_latency)
        
        # WebSocket 端點
        self.app.router.add_get('/ws', self._websocket_handler)
        
        # 靜態檔案服務
        static_dir = Path(__file__).parent / 'static'
        if static_dir.exists():
            self.app.router.add_static('/', path=str(static_dir), name='static')
        
        # 健康檢查
        self.app.router.add_get('/health', self._health_check)
    
    async def _get_robots(self, request: Request) -> Response:
        """取得機器人狀態"""
        try:
            robots = self.kuka_api.get_robots()
            return web.json_response({
                "success": True,
                "data": robots,
                "timestamp": datetime.now(timezone.utc).isoformat()
            })
        except Exception as e:
            self.logger.error(f"取得機器人狀態失敗: {e}")
            return web.json_response({
                "success": False,
                "error": str(e)
            }, status=500)
    
    async def _get_containers(self, request: Request) -> Response:
        """取得容器狀態"""
        try:
            containers = self.kuka_api.get_containers()
            return web.json_response({
                "success": True,
                "data": containers,
                "timestamp": datetime.now(timezone.utc).isoformat()
            })
        except Exception as e:
            self.logger.error(f"取得容器狀態失敗: {e}")
            return web.json_response({
                "success": False,
                "error": str(e)
            }, status=500)
    
    async def _create_move_mission(self, request: Request) -> Response:
        """創建移動任務"""
        try:
            data = await request.json()
            nodes = data.get('nodes', [])
            agv_id = data.get('agv_id')
            mission_code = data.get('mission_code')
            
            result = self.kuka_api.move(nodes, agv_id, mission_code)
            await self._broadcast_mission_update(result)
            
            return web.json_response(result)
        except Exception as e:
            self.logger.error(f"創建移動任務失敗: {e}")
            return web.json_response({
                "success": False,
                "error": str(e)
            }, status=500)
    
    async def _create_rack_move_mission(self, request: Request) -> Response:
        """創建貨架移動任務"""
        try:
            data = await request.json()
            nodes = data.get('nodes', [])
            agv_id = data.get('agv_id')
            mission_code = data.get('mission_code')
            
            result = self.kuka_api.rack_move(nodes, agv_id, mission_code)
            await self._broadcast_mission_update(result)
            
            return web.json_response(result)
        except Exception as e:
            self.logger.error(f"創建貨架移動任務失敗: {e}")
            return web.json_response({
                "success": False,
                "error": str(e)
            }, status=500)
    
    async def _create_workflow_mission(self, request: Request) -> Response:
        """創建工作流程任務"""
        try:
            data = await request.json()
            template_code = data.get('template_code')
            agv_id = data.get('agv_id')
            mission_code = data.get('mission_code')
            
            result = self.kuka_api.workflow(template_code, agv_id, mission_code)
            await self._broadcast_mission_update(result)
            
            return web.json_response(result)
        except Exception as e:
            self.logger.error(f"創建工作流程任務失敗: {e}")
            return web.json_response({
                "success": False,
                "error": str(e)
            }, status=500)
    
    async def _get_mission(self, request: Request) -> Response:
        """取得任務狀態"""
        try:
            mission_id = request.match_info['mission_id']
            mission = self.kuka_api.missions.get(mission_id)
            
            if mission:
                return web.json_response({
                    "success": True,
                    "data": mission
                })
            else:
                return web.json_response({
                    "success": False,
                    "error": "Mission not found"
                }, status=404)
        except Exception as e:
            self.logger.error(f"取得任務狀態失敗: {e}")
            return web.json_response({
                "success": False,
                "error": str(e)
            }, status=500)
    
    async def _get_api_status(self, request: Request) -> Response:
        """取得 API 狀態"""
        return web.json_response({
            "success": True,
            "data": {
                "online": self.kuka_api.is_online,
                "response_delay": self.kuka_api.response_delay,
                "mission_count": len(self.kuka_api.missions),
                "server_time": datetime.now(timezone.utc).isoformat()
            }
        })
    
    async def _get_environment_state(self, request: Request) -> Response:
        """取得測試環境狀態"""
        try:
            state = self.mock_env.get_system_state()
            return web.json_response({
                "success": True,
                "data": state
            })
        except Exception as e:
            self.logger.error(f"取得環境狀態失敗: {e}")
            return web.json_response({
                "success": False,
                "error": str(e)
            }, status=500)
    
    async def _reset_environment(self, request: Request) -> Response:
        """重置測試環境"""
        try:
            self.mock_env = MockTestEnvironment()
            self.kuka_api = self.mock_env.kuka_api
            
            await self._broadcast_environment_update("environment_reset")
            
            return web.json_response({
                "success": True,
                "message": "Environment reset successfully"
            })
        except Exception as e:
            self.logger.error(f"重置環境失敗: {e}")
            return web.json_response({
                "success": False,
                "error": str(e)
            }, status=500)
    
    async def _load_scenario(self, request: Request) -> Response:
        """載入測試場景"""
        try:
            data = await request.json()
            scenario_name = data.get('scenario')
            
            if not scenario_name:
                return web.json_response({
                    "success": False,
                    "error": "Missing scenario parameter"
                }, status=400)
            
            self.mock_env.load_scenario(scenario_name)
            self.kuka_api = self.mock_env.kuka_api
            
            await self._broadcast_environment_update(f"scenario_loaded_{scenario_name}")
            
            return web.json_response({
                "success": True,
                "message": f"Scenario '{scenario_name}' loaded successfully"
            })
        except Exception as e:
            self.logger.error(f"載入場景失敗: {e}")
            return web.json_response({
                "success": False,
                "error": str(e)
            }, status=500)
    
    async def _simulate_network_issues(self, request: Request) -> Response:
        """模擬網路問題"""
        try:
            data = await request.json()
            duration = data.get('duration', 5.0)
            
            self.mock_env.simulate_network_issues(duration)
            
            await self._broadcast_environment_update("network_issues_simulated")
            
            return web.json_response({
                "success": True,
                "message": f"Network issues simulated for {duration} seconds"
            })
        except Exception as e:
            self.logger.error(f"模擬網路問題失敗: {e}")
            return web.json_response({
                "success": False,
                "error": str(e)
            }, status=500)
    
    async def _simulate_high_latency(self, request: Request) -> Response:
        """模擬高延遲"""
        try:
            data = await request.json()
            delay = data.get('delay', 2.0)
            
            self.mock_env.simulate_high_latency(delay)
            
            await self._broadcast_environment_update("high_latency_simulated")
            
            return web.json_response({
                "success": True,
                "message": f"High latency simulated with {delay}s delay"
            })
        except Exception as e:
            self.logger.error(f"模擬高延遲失敗: {e}")
            return web.json_response({
                "success": False,
                "error": str(e)
            }, status=500)
    
    async def _websocket_handler(self, request: Request) -> WebSocketResponse:
        """WebSocket 處理器"""
        ws = web.WebSocketResponse()
        await ws.prepare(request)
        
        self.websockets.add(ws)
        self.logger.info(f"WebSocket 連接建立，總連接數: {len(self.websockets)}")
        
        try:
            async for msg in ws:
                if msg.type == WSMsgType.TEXT:
                    try:
                        data = json.loads(msg.data)
                        await self._handle_websocket_message(ws, data)
                    except json.JSONDecodeError:
                        await ws.send_str(json.dumps({
                            "type": "error",
                            "message": "Invalid JSON format"
                        }))
                elif msg.type == WSMsgType.ERROR:
                    self.logger.error(f'WebSocket 錯誤: {ws.exception()}')
        except Exception as e:
            self.logger.error(f"WebSocket 處理錯誤: {e}")
        finally:
            self.websockets.discard(ws)
            self.logger.info(f"WebSocket 連接關閉，剩餘連接數: {len(self.websockets)}")
        
        return ws
    
    async def _handle_websocket_message(self, ws: WebSocketResponse, data: Dict):
        """處理 WebSocket 訊息"""
        message_type = data.get('type')
        
        if message_type == 'ping':
            await ws.send_str(json.dumps({
                "type": "pong",
                "timestamp": datetime.now(timezone.utc).isoformat()
            }))
        elif message_type == 'subscribe':
            # 客戶端訂閱更新
            await ws.send_str(json.dumps({
                "type": "subscribed",
                "message": "Successfully subscribed to updates"
            }))
        else:
            await ws.send_str(json.dumps({
                "type": "error",
                "message": f"Unknown message type: {message_type}"
            }))
    
    async def _broadcast_mission_update(self, mission_data: Dict):
        """廣播任務更新"""
        if not self.websockets:
            return
        
        message = {
            "type": "mission_update",
            "data": mission_data,
            "timestamp": datetime.now(timezone.utc).isoformat()
        }
        
        await self._broadcast_message(message)
    
    async def _broadcast_environment_update(self, event_type: str):
        """廣播環境更新"""
        if not self.websockets:
            return
        
        message = {
            "type": "environment_update",
            "event": event_type,
            "timestamp": datetime.now(timezone.utc).isoformat()
        }
        
        await self._broadcast_message(message)
    
    async def _broadcast_message(self, message: Dict):
        """廣播訊息給所有 WebSocket 連接"""
        if not self.websockets:
            return
        
        message_str = json.dumps(message)
        disconnected = set()
        
        for ws in self.websockets:
            try:
                await ws.send_str(message_str)
            except Exception as e:
                self.logger.warning(f"發送 WebSocket 訊息失敗: {e}")
                disconnected.add(ws)
        
        # 清理斷開的連接
        self.websockets -= disconnected
    
    async def _health_check(self, request: Request) -> Response:
        """健康檢查"""
        return web.json_response({
            "status": "healthy",
            "server": "Offline KUKA Test Server",
            "version": "1.0.0",
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "connections": len(self.websockets)
        })
    
    async def start_server(self):
        """啟動服務器"""
        if not AIOHTTP_AVAILABLE:
            self.logger.error("無法啟動服務器：aiohttp 未安裝")
            return
        
        self.logger.info(f"啟動離線測試服務器於 http://{self.host}:{self.port}")
        self.logger.info("可用端點:")
        self.logger.info("  - GET  /api/robots - 取得機器人狀態")
        self.logger.info("  - GET  /api/containers - 取得容器狀態") 
        self.logger.info("  - POST /api/missions/move - 創建移動任務")
        self.logger.info("  - POST /api/missions/rack_move - 創建貨架移動任務")
        self.logger.info("  - POST /api/missions/workflow - 創建工作流程任務")
        self.logger.info("  - GET  /test/environment - 取得測試環境狀態")
        self.logger.info("  - POST /test/environment/reset - 重置測試環境")
        self.logger.info("  - GET  /ws - WebSocket 連接")
        self.logger.info("  - GET  /health - 健康檢查")
        
        runner = web.AppRunner(self.app)
        await runner.setup()
        
        site = web.TCPSite(runner, self.host, self.port)
        await site.start()
        
        self.logger.info("服務器已啟動，按 Ctrl+C 停止服務器")
        
        try:
            while True:
                await asyncio.sleep(1)
        except KeyboardInterrupt:
            self.logger.info("正在停止服務器...")
            await runner.cleanup()


def main():
    """主函數"""
    parser = argparse.ArgumentParser(description="KUKA 離線測試服務器")
    parser.add_argument("--host", default="localhost", help="服務器主機 (預設: localhost)")
    parser.add_argument("--port", type=int, default=8080, help="服務器端口 (預設: 8080)")
    parser.add_argument("--log-level", default="INFO", 
                       choices=["DEBUG", "INFO", "WARNING", "ERROR"],
                       help="日誌等級 (預設: INFO)")
    
    args = parser.parse_args()
    
    # 設置日誌等級
    logging.getLogger().setLevel(getattr(logging, args.log_level))
    
    if not AIOHTTP_AVAILABLE:
        print("錯誤: 需要安裝 aiohttp 才能運行服務器")
        print("安裝命令: pip install aiohttp")
        return 1
    
    try:
        server = OfflineTestServer(args.host, args.port)
        asyncio.run(server.start_server())
        return 0
    except Exception as e:
        print(f"服務器啟動失敗: {e}")
        return 1


if __name__ == "__main__":
    exit(main())