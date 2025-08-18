#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
統一節點管理 API
提供 ROS 2 節點的狀態查詢、啟動、停止和重啟功能
支援本地 AGVC 節點和遠端 AGV 節點管理
"""

from fastapi import APIRouter, HTTPException, BackgroundTasks
from fastapi.responses import JSONResponse
from typing import Dict, List, Optional, Any
import subprocess
import asyncio
import yaml
import os
import json
from datetime import datetime
import logging

# 設定日誌
logger = logging.getLogger(__name__)

# 建立路由器
router = APIRouter(
    prefix="/api/nodes",
    tags=["nodes"],
    responses={404: {"description": "Not found"}},
)

# 載入節點註冊表
REGISTRY_PATH = "/app/config/node_registry.yaml"
node_registry = {}

def load_registry():
    """載入節點註冊表配置"""
    global node_registry
    try:
        with open(REGISTRY_PATH, 'r') as f:
            node_registry = yaml.safe_load(f)
            logger.info(f"載入節點註冊表: {len(node_registry.get('nodes', {}))} 個節點")
    except Exception as e:
        logger.error(f"無法載入節點註冊表: {e}")
        node_registry = {"nodes": {}, "agv_nodes": {}, "remote_agvs": {}}

# 初始載入
load_registry()


class NodeManager:
    """節點管理器類別"""
    
    @staticmethod
    async def run_command(cmd: str, timeout: int = 5) -> Dict[str, Any]:
        """執行系統命令 - 減少預設超時時間"""
        try:
            process = await asyncio.create_subprocess_shell(
                cmd,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE
            )
            
            try:
                stdout, stderr = await asyncio.wait_for(
                    process.communicate(), timeout=timeout
                )
                
                return {
                    "success": process.returncode == 0,
                    "stdout": stdout.decode() if stdout else "",
                    "stderr": stderr.decode() if stderr else "",
                    "returncode": process.returncode
                }
            except asyncio.TimeoutError:
                process.kill()
                return {
                    "success": False,
                    "error": f"Command timeout after {timeout} seconds"
                }
                
        except Exception as e:
            return {
                "success": False,
                "error": str(e)
            }
    
    @staticmethod
    async def get_node_status(node_name: str) -> Dict[str, Any]:
        """獲取單個節點狀態"""
        node_info = node_registry.get("nodes", {}).get(node_name)
        if not node_info:
            return {
                "name": node_name,
                "status": "unknown",
                "error": "Node not found in registry"
            }
        
        status = {
            "name": node_name,
            "type": node_info.get("type"),
            "description": node_info.get("description"),
            "status": "unknown",
            "running": False,
            "details": {}
        }
        
        # 檢查節點狀態
        if node_info.get("type") == "launch":
            # Launch 群組檢查
            # 使用更簡潔的命令，避免環境載入的冗餘輸出
            cmd = f"bash -c 'source /app/setup.bash >/dev/null 2>&1 && manage_{node_name} status 2>/dev/null'"
            result = await NodeManager.run_command(cmd)
            
            if result["success"]:
                output = result["stdout"]
                # 檢查各種可能的運行狀態文字
                if "運行中" in output or "正在運行" in output or "running" in output.lower() or "✅ Web API Launch 正在運行" in output:
                    status["status"] = "running"
                    status["running"] = True
                elif "停止" in output or "stopped" in output.lower():
                    status["status"] = "stopped"
                else:
                    status["status"] = "partial"
                
                # 解析子節點狀態
                if "nodes" in node_info:
                    status["details"]["nodes"] = {}
                    for sub_node in node_info["nodes"]:
                        status["details"]["nodes"][sub_node] = sub_node in output
                        
        elif node_info.get("type") == "node":
            # 單一節點檢查
            if "status_check" in node_info:
                # 需要載入環境來執行 ros2 命令
                status_check_cmd = node_info["status_check"]
                cmd = f"bash -c 'source /app/setup.bash >/dev/null 2>&1 && agvc_source >/dev/null 2>&1 && {status_check_cmd}'"
                result = await NodeManager.run_command(cmd)
                status["running"] = result["success"] and bool(result["stdout"].strip())
                status["status"] = "running" if status["running"] else "stopped"
        
        # 檢查端口
        if "port_check" in node_info:
            ports = node_info["port_check"]
            if not isinstance(ports, list):
                ports = [ports]
            
            status["details"]["ports"] = {}
            for port in ports:
                cmd = f"ss -tulpn | grep :{port}"
                result = await NodeManager.run_command(cmd)
                status["details"]["ports"][port] = result["success"] and bool(result["stdout"])
        
        return status
    
    @staticmethod
    async def start_node(node_name: str) -> Dict[str, Any]:
        """啟動節點"""
        node_info = node_registry.get("nodes", {}).get(node_name)
        if not node_info:
            raise HTTPException(status_code=404, detail=f"Node {node_name} not found")
        
        # 根據節點類型使用不同的啟動命令
        if node_info.get("type") == "launch":
            # Launch 類型使用 manage_ 函數
            cmd = f"bash -c 'source /app/setup.bash >/dev/null 2>&1 && manage_{node_name} start'"
        else:
            # Node 類型使用 ros2 run
            package = node_info.get("package")
            executable = node_info.get("executable")
            namespace = node_info.get("namespace", "")
            if not package or not executable:
                return {
                    "success": False,
                    "message": f"Missing package or executable for node {node_name}",
                    "error": "Configuration incomplete"
                }
            
            ns_param = f"--ros-args -r __ns:=/{namespace}" if namespace else ""
            cmd = f"bash -c 'source /app/setup.bash >/dev/null 2>&1 && agvc_source >/dev/null 2>&1 && nohup ros2 run {package} {executable} {ns_param} > {node_info.get("log_file", "/tmp/" + node_name + ".log")} 2>&1 &'"
        
        result = await NodeManager.run_command(cmd, timeout=10)
        
        if result["success"]:
            # 等待啟動完成
            await asyncio.sleep(2)
            status = await NodeManager.get_node_status(node_name)
            return {
                "success": True,
                "message": f"Node {node_name} started",
                "status": status
            }
        else:
            return {
                "success": False,
                "message": f"Failed to start node {node_name}",
                "error": result.get("stderr", result.get("error"))
            }
    
    @staticmethod
    async def stop_node(node_name: str) -> Dict[str, Any]:
        """停止節點"""
        node_info = node_registry.get("nodes", {}).get(node_name)
        if not node_info:
            raise HTTPException(status_code=404, detail=f"Node {node_name} not found")
        
        # 根據節點類型使用不同的停止命令
        if node_info.get("type") == "launch":
            # Launch 類型使用 manage_ 函數
            cmd = f"bash -c 'source /app/setup.bash >/dev/null 2>&1 && manage_{node_name} stop'"
        else:
            # Node 類型使用 pkill 停止進程
            executable = node_info.get("executable")
            if not executable:
                return {
                    "success": False,
                    "message": f"Missing executable for node {node_name}",
                    "error": "Configuration incomplete"
                }
            cmd = f"bash -c 'pkill -f {executable}'"
        
        result = await NodeManager.run_command(cmd, timeout=10)
        
        if result["success"]:
            return {
                "success": True,
                "message": f"Node {node_name} stopped"
            }
        else:
            return {
                "success": False,
                "message": f"Failed to stop node {node_name}",
                "error": result.get("stderr", result.get("error"))
            }
    
    @staticmethod
    async def restart_node(node_name: str) -> Dict[str, Any]:
        """重啟節點"""
        # 先停止
        stop_result = await NodeManager.stop_node(node_name)
        if not stop_result["success"]:
            return stop_result
        
        # 等待停止完成
        await asyncio.sleep(2)
        
        # 再啟動
        return await NodeManager.start_node(node_name)
    
    @staticmethod
    async def get_agv_status(agv_name: str) -> Dict[str, Any]:
        """獲取遠端 AGV 狀態"""
        agv_info = node_registry.get("remote_agvs", {}).get(agv_name)
        if not agv_info:
            raise HTTPException(status_code=404, detail=f"AGV {agv_name} not found")
        
        cmd = f"source /app/setup.bash && agvc_source && manage_agv_launch {agv_name} status"
        result = await NodeManager.run_command(cmd, timeout=30)
        
        status = {
            "name": agv_name,
            "type": agv_info.get("type"),
            "ip": agv_info.get("ip"),
            "status": "unknown",
            "details": {}
        }
        
        if result["success"]:
            output = result["stdout"]
            if "運行中" in output or "running" in output.lower():
                status["status"] = "running"
            elif "停止" in output or "stopped" in output.lower():
                status["status"] = "stopped"
            else:
                status["status"] = "partial"
            
            status["details"]["output"] = output
        else:
            status["status"] = "error"
            status["details"]["error"] = result.get("error")
        
        return status


# API 端點

@router.get("/")
async def get_all_nodes():
    """獲取所有節點列表"""
    return {
        "nodes": list(node_registry.get("nodes", {}).keys()),
        "agv_nodes": list(node_registry.get("remote_agvs", {}).keys()),
        "groups": list(node_registry.get("node_groups", {}).keys())
    }


@router.get("/status")
async def get_all_status():
    """獲取所有節點狀態 - 優化版本，快速檢查實際狀態"""
    status_list = []
    
    # 快速獲取所有運行中的 ROS 2 節點
    running_nodes = set()
    try:
        # 使用更簡單的方式檢查節點 - 先嘗試直接執行
        cmd = "ros2 node list 2>/dev/null"
        result = await NodeManager.run_command(cmd, timeout=3)
        
        # 如果直接執行失敗，嘗試載入環境
        if not result["success"] or not result["stdout"]:
            cmd = "bash -c 'source /app/setup.bash >/dev/null 2>&1 && agvc_source >/dev/null 2>&1 && ros2 node list 2>/dev/null'"
            result = await NodeManager.run_command(cmd, timeout=8)
        
        if result["success"]:
            # 解析輸出的節點列表
            lines = result["stdout"].strip().split('\n')
            for line in lines:
                if line and not line.startswith("WARNING") and not line.startswith("✅") and not line.startswith("🖥️"):
                    # 提取節點名稱，去掉前綴 /
                    node = line.strip().lstrip('/')
                    if node:  # 確保節點名稱不為空
                        running_nodes.add(node)
            logger.info(f"Running nodes detected: {running_nodes}")
        else:
            logger.warning(f"Failed to get node list: {result.get('error', 'Unknown error')}")
    except Exception as e:
        logger.error(f"Failed to get running nodes: {e}")
    
    # 獲取本地節點狀態
    for node_name, node_info in node_registry.get("nodes", {}).items():
        status = {
            "name": node_name,
            "type": node_info.get("type"),
            "description": node_info.get("description"),
            "status": "unknown",
            "running": False,
            "details": {}
        }
        
        # 根據節點類型檢查狀態
        if node_info.get("type") == "launch":
            # Launch 類型節點特殊處理
            # 如果無法獲取 ROS 2 節點列表，使用 manage 函數的 status 命令
            if not running_nodes:
                # 沒有節點資訊，使用備用方法
                # 先嘗試簡單的 PID 檔案檢查（更快更可靠）
                # 為了確保可靠性，使用最簡單的檢查方式
                if node_name == "flow_wcs":
                    # Flow WCS 特殊處理
                    pid_check_cmd = "pgrep -f 'flow_wcs' > /dev/null 2>&1 && echo running || echo stopped"
                elif node_name == "ecs_core":
                    # ECS 特殊處理
                    pid_check_cmd = "pgrep -f 'ecs_core' > /dev/null 2>&1 && echo running || echo stopped"
                elif node_name == "db_proxy":
                    # DB Proxy 特殊處理
                    pid_check_cmd = "pgrep -f 'agvc_database_node' > /dev/null 2>&1 && echo running || echo stopped"
                elif node_name == "kuka_fleet":
                    # KUKA Fleet 特殊處理
                    pid_check_cmd = "pgrep -f 'kuka.*adapter' > /dev/null 2>&1 && echo running || echo stopped"
                elif node_name == "rcs_core":
                    # RCS Core 特殊處理
                    pid_check_cmd = "pgrep -f 'rcs_core' > /dev/null 2>&1 && echo running || echo stopped"
                elif node_name == "zenoh":
                    # Zenoh Router 特殊處理
                    pid_check_cmd = "pgrep -f 'rmw_zenohd' > /dev/null 2>&1 && echo running || echo stopped"
                elif node_name == "ssh":
                    # SSH 服務特殊處理
                    pid_check_cmd = "pgrep -f 'sshd.*2200' > /dev/null 2>&1 && echo running || echo stopped"
                elif node_name == "agvui":
                    # AGVUI 特殊處理 (通常不運行)
                    pid_check_cmd = "echo stopped"
                else:
                    # 預設使用 PID 檔案檢查
                    pid_file = f"/tmp/{node_name}.pid"
                    pid_check_cmd = f"test -f {pid_file} && head -1 {pid_file} | xargs ps -p > /dev/null 2>&1 && echo running || echo stopped"
                
                pid_result = await NodeManager.run_command(pid_check_cmd, timeout=2)
                
                if pid_result.get("success") and pid_result.get("stdout"):
                    pid_status = pid_result.get("stdout", "").strip()
                    if pid_status == "running":
                        status["status"] = "running"
                        status["running"] = True
                    elif pid_status == "stopped":
                        status["status"] = "stopped"
                        status["running"] = False
                    else:
                        # 如果輸出不是預期的 running/stopped，記錄並設為 unknown
                        logger.warning(f"Unexpected PID check output for {node_name}: '{pid_status}'")
                        status["status"] = "unknown"
                        status["running"] = False
                else:
                    # PID 檔案檢查失敗，嘗試使用 manage_ 函數
                    logger.debug(f"PID check failed for {node_name}, trying manage_ function")
                    # 使用簡化的命令，避免 bash -i 造成的問題
                    status_cmd = f"bash -c 'source /app/setup.bash >/dev/null 2>&1 && manage_{node_name} status 2>&1 | grep -q \"正在運行\\|運行中\\|running\" && echo running || echo stopped'"
                    status_result = await NodeManager.run_command(status_cmd, timeout=3)
                    
                    if status_result["success"] and status_result["stdout"]:
                        manage_status = status_result["stdout"].strip()
                        if manage_status == "running":
                            status["status"] = "running"
                            status["running"] = True
                        else:
                            status["status"] = "stopped"
                            status["running"] = False
                    else:
                        # 如果都失敗了，保持 unknown
                        logger.warning(f"Cannot determine status for {node_name}, both PID and manage_ checks failed")
            elif node_name == "web_api_launch":
                # web_api_launch 包含多個子節點
                if "agvc/web_api_server" in running_nodes or "agv_ui_server_node" in running_nodes:
                    status["status"] = "running"
                    status["running"] = True
                else:
                    status["status"] = "stopped"
            elif node_name == "ecs_core":
                # ecs_core 節點檢查
                if "ecs_core" in running_nodes:
                    status["status"] = "running"
                    status["running"] = True
                else:
                    status["status"] = "stopped"
            elif node_name == "flow_wcs":
                # flow_wcs 節點檢查
                if "flow_wcs_node" in running_nodes:
                    status["status"] = "running"
                    status["running"] = True
                else:
                    status["status"] = "stopped"
            elif node_name == "rcs_core":
                # rcs_core 節點檢查
                if "rcs_core" in running_nodes:
                    status["status"] = "running"
                    status["running"] = True
                else:
                    status["status"] = "stopped"
            elif node_name == "db_proxy":
                # db_proxy 節點檢查
                if "db_proxy_node" in running_nodes or "agvc_database_node" in running_nodes:
                    status["status"] = "running"
                    status["running"] = True
                else:
                    status["status"] = "stopped"
            elif node_name == "kuka_fleet":
                # kuka_fleet 節點檢查
                if "kuka_adapter_demo_node" in running_nodes or "kuka_fleet_adapter" in running_nodes:
                    status["status"] = "running"
                    status["running"] = True
                else:
                    status["status"] = "stopped"
            elif node_name == "agvui":
                # agvui 節點檢查 (通常不會真的運行)
                status["status"] = "stopped"
            elif node_name == "ssh":
                # SSH 服務檢查 (使用 pgrep 而非 ros2 node list)
                ssh_cmd = "pgrep -f '/usr/sbin/sshd -D -p 2200'"
                ssh_result = await NodeManager.run_command(ssh_cmd, timeout=2)
                if ssh_result["success"] and ssh_result["stdout"]:
                    status["status"] = "running"
                    status["running"] = True
                else:
                    status["status"] = "stopped"
            elif node_name == "zenoh":
                # Zenoh Router 檢查 (使用 pgrep 而非 ros2 node list)
                zenoh_cmd = "pgrep -f 'rmw_zenohd'"
                zenoh_result = await NodeManager.run_command(zenoh_cmd, timeout=2)
                if zenoh_result["success"] and zenoh_result["stdout"]:
                    status["status"] = "running"
                    status["running"] = True
                else:
                    status["status"] = "stopped"
            else:
                # 其他 launch 類型節點預設為 stopped
                status["status"] = "stopped"
        else:
            # 單一節點類型
            # 如果 running_nodes 為空，嘗試使用其他方法檢查
            if not running_nodes:
                # 特別處理 plc_service_agvc，因為它沒有 manage_ 函數
                if node_name == "plc_service_agvc":
                    # 檢查進程是否存在
                    check_cmd = "pgrep -f 'plc_service.*namespace.*agvc'"
                    check_result = await NodeManager.run_command(check_cmd, timeout=2)
                    if check_result["success"] and check_result["stdout"]:
                        status["status"] = "running"
                        status["running"] = True
                    else:
                        status["status"] = "stopped"
                        status["running"] = False
                else:
                    # 其他節點保持 stopped 狀態
                    status["status"] = "stopped"
                    status["running"] = False
            else:
                # 有 running_nodes 資訊時的處理
                node_key = node_info.get("node_name", node_name)
                
                # 特殊節點名稱映射
                node_mapping = {
                    "flow_wcs": "flow_wcs_node",
                    "ecs_core": "ecs_core",
                    "rcs_core": "rcs_core",
                    "db_proxy": "db_proxy_node",
                    "kuka_fleet": "kuka_adapter_demo_node",
                    "plc_service_agvc": "plc_service"
                }
                
                check_name = node_mapping.get(node_name, node_key)
                
                # 檢查節點是否在運行列表中
                if check_name in running_nodes:
                    status["status"] = "running"
                    status["running"] = True
                elif any(check_name in node for node in running_nodes):
                    # 部分匹配檢查
                    status["status"] = "running"
                    status["running"] = True
                else:
                    status["status"] = "stopped"
        
        status_list.append(status)
    
    # 獲取遠端 AGV 狀態 - 簡化版本
    agv_status_list = []
    for agv_name, agv_info in node_registry.get("remote_agvs", {}).items():
        agv_status_list.append({
            "name": agv_name,
            "type": agv_info.get("type"),
            "ip": agv_info.get("ip"),
            "status": "unknown",  # 預設為 unknown
            "details": {}
        })
    
    return {
        "timestamp": datetime.now().isoformat(),
        "nodes": status_list,
        "agvs": agv_status_list
    }


@router.get("/node/{node_name}")
async def get_node_status(node_name: str):
    """獲取特定節點狀態"""
    if node_name not in node_registry.get("nodes", {}):
        raise HTTPException(status_code=404, detail=f"Node {node_name} not found")
    
    return await NodeManager.get_node_status(node_name)


@router.post("/node/{node_name}/start")
async def start_node(node_name: str, background_tasks: BackgroundTasks):
    """啟動特定節點"""
    if node_name not in node_registry.get("nodes", {}):
        raise HTTPException(status_code=404, detail=f"Node {node_name} not found")
    
    return await NodeManager.start_node(node_name)


@router.post("/node/{node_name}/stop")
async def stop_node(node_name: str):
    """停止特定節點"""
    if node_name not in node_registry.get("nodes", {}):
        raise HTTPException(status_code=404, detail=f"Node {node_name} not found")
    
    return await NodeManager.stop_node(node_name)


@router.post("/node/{node_name}/restart")
async def restart_node(node_name: str, background_tasks: BackgroundTasks):
    """重啟特定節點"""
    if node_name not in node_registry.get("nodes", {}):
        raise HTTPException(status_code=404, detail=f"Node {node_name} not found")
    
    return await NodeManager.restart_node(node_name)


@router.get("/group/{group_name}")
async def get_group_status(group_name: str):
    """獲取節點群組狀態"""
    group_info = node_registry.get("node_groups", {}).get(group_name)
    if not group_info:
        raise HTTPException(status_code=404, detail=f"Group {group_name} not found")
    
    status_list = []
    for node_name in group_info.get("nodes", []):
        status = await NodeManager.get_node_status(node_name)
        status_list.append(status)
    
    return {
        "group": group_name,
        "description": group_info.get("description"),
        "nodes": status_list
    }


@router.post("/group/{group_name}/start")
async def start_group(group_name: str, background_tasks: BackgroundTasks):
    """啟動節點群組"""
    group_info = node_registry.get("node_groups", {}).get(group_name)
    if not group_info:
        raise HTTPException(status_code=404, detail=f"Group {group_name} not found")
    
    results = []
    for node_name in group_info.get("nodes", []):
        result = await NodeManager.start_node(node_name)
        results.append({
            "node": node_name,
            "result": result
        })
    
    return {
        "group": group_name,
        "results": results
    }


@router.post("/group/{group_name}/stop")
async def stop_group(group_name: str):
    """停止節點群組"""
    group_info = node_registry.get("node_groups", {}).get(group_name)
    if not group_info:
        raise HTTPException(status_code=404, detail=f"Group {group_name} not found")
    
    results = []
    for node_name in group_info.get("nodes", []):
        result = await NodeManager.stop_node(node_name)
        results.append({
            "node": node_name,
            "result": result
        })
    
    return {
        "group": group_name,
        "results": results
    }


@router.get("/agv/{agv_name}")
async def get_agv_status(agv_name: str):
    """獲取遠端 AGV 狀態"""
    return await NodeManager.get_agv_status(agv_name)


@router.post("/agv/{agv_name}/{action}")
async def control_agv(agv_name: str, action: str):
    """控制遠端 AGV 節點"""
    if action not in ["start", "stop", "restart"]:
        raise HTTPException(status_code=400, detail=f"Invalid action: {action}")
    
    agv_info = node_registry.get("remote_agvs", {}).get(agv_name)
    if not agv_info:
        raise HTTPException(status_code=404, detail=f"AGV {agv_name} not found")
    
    cmd = f"source /app/setup.bash && agvc_source && manage_agv_launch {agv_name} {action}"
    result = await NodeManager.run_command(cmd, timeout=60)
    
    if result["success"]:
        return {
            "success": True,
            "message": f"AGV {agv_name} {action} completed",
            "output": result["stdout"]
        }
    else:
        return {
            "success": False,
            "message": f"Failed to {action} AGV {agv_name}",
            "error": result.get("stderr", result.get("error"))
        }


@router.post("/reload-registry")
async def reload_registry():
    """重新載入節點註冊表"""
    load_registry()
    return {
        "success": True,
        "message": "Registry reloaded",
        "nodes_count": len(node_registry.get("nodes", {})),
        "agvs_count": len(node_registry.get("remote_agvs", {}))
    }


@router.get("/health")
async def health_check():
    """健康檢查端點"""
    return {
        "status": "healthy",
        "service": "node_management",
        "timestamp": datetime.now().isoformat()
    }