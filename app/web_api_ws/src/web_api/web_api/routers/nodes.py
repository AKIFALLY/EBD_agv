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
    async def run_command(cmd: str, timeout: int = 15) -> Dict[str, Any]:
        """執行系統命令 - 預設 15 秒超時（足夠應付節點驗證的 10 秒智能重試）"""
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
                elif "停止" in output or "stopped" in output.lower() or "未運行" in output or "未啟動" in output:
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

            # 構建命令（使用雙引號確保變數正確展開）
            log_file = node_info.get("log_file", f"/tmp/{node_name}.log")
            if namespace:
                ns_param = f"--ros-args -r __ns:=/{namespace}"
                cmd = f'bash -c "source /app/setup.bash >/dev/null 2>&1 && agvc_source >/dev/null 2>&1 && nohup ros2 run {package} {executable} {ns_param} > {log_file} 2>&1 &"'
            else:
                cmd = f'bash -c "source /app/setup.bash >/dev/null 2>&1 && agvc_source >/dev/null 2>&1 && nohup ros2 run {package} {executable} > {log_file} 2>&1 &"'

            # 記錄啟動命令（用於調試）
            logger.info(f"📝 啟動節點 {node_name}:")
            logger.info(f"   - package: {package}")
            logger.info(f"   - executable: {executable}")
            logger.info(f"   - namespace: {namespace if namespace else 'None'}")
            logger.info(f"   - 完整命令: {cmd}")
        
        result = await NodeManager.run_command(cmd, timeout=10)

        if result["success"]:
            # 不需要額外等待：manage 函數內的 verify_ros2_node_startup 已確認節點啟動
            # 直接查詢狀態即可（節點已經通過 ROS 2 網路註冊驗證）
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
        """停止節點 - 改進版本，驗證實際停止狀態而非僅依賴退出碼"""
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

        # 執行停止命令 (增加超時時間以適應 Launch 類型節點的清理時間)
        result = await NodeManager.run_command(cmd, timeout=15)

        # 等待節點完全停止和清理
        await asyncio.sleep(2)

        # 驗證實際停止狀態 (關鍵改進：不依賴 bash 退出碼)
        status = await NodeManager.get_node_status(node_name)
        actual_stopped = status["status"] in ["stopped", "unknown"]

        # 根據實際狀態返回結果
        if actual_stopped:
            return {
                "success": True,
                "message": f"Node {node_name} stopped",
                "verified": True,  # 標記已驗證實際狀態
                "status": status["status"]
            }
        else:
            # 節點仍在運行，真正的停止失敗
            return {
                "success": False,
                "message": f"Node {node_name} still running after stop command",
                "error": result.get("stderr", result.get("error")),
                "status": status["status"],
                "command_exit_code": result.get("returncode")
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


async def check_port(port: int) -> bool:
    """檢查端口是否在監聽"""
    try:
        cmd = f"ss -tuln | grep -q ':{port}\\s' && echo 'listening' || echo 'not_listening'"
        result = await NodeManager.run_command(cmd, timeout=2)
        if result["success"] and result["stdout"]:
            return result["stdout"].strip() == "listening"
        return False
    except Exception as e:
        logger.error(f"Port check failed for {port}: {e}")
        return False


async def check_multiple_ports(ports: List[int]) -> Dict[int, bool]:
    """檢查多個端口的狀態"""
    results = {}
    for port in ports:
        results[port] = await check_port(port)
    return results


async def check_pid_file_valid(pid_file: str) -> bool:
    """檢查 PID 文件是否有效（文件存在且進程仍在運行）"""
    try:
        cmd = f"test -f {pid_file} && head -1 {pid_file} | xargs -I{{}} sh -c 'ps -p {{}} > /dev/null 2>&1' && echo valid || echo invalid"
        result = await NodeManager.run_command(cmd, timeout=2)
        if result["success"] and result["stdout"]:
            return result["stdout"].strip() == "valid"
        return False
    except Exception as e:
        logger.error(f"PID file check failed for {pid_file}: {e}")
        return False


@router.get("/status")
async def get_all_status():
    """獲取所有節點狀態 - 改進版本，統一檢查邏輯"""
    status_list = []

    # 快速獲取所有運行中的 ROS 2 節點（備用方法）
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

        # ===== 多層檢查邏輯：端口 → status_check → PID → ROS 2 =====

        # 第1層：端口檢查（最可靠）
        port_check_config = node_info.get("port_check")
        if port_check_config:
            ports_to_check = port_check_config if isinstance(port_check_config, list) else [port_check_config]
            port_results = await check_multiple_ports(ports_to_check)
            all_ports_up = all(port_results.values())

            if all_ports_up:
                status["status"] = "running"
                status["running"] = True
                status["details"]["port_check"] = {
                    "method": "port",
                    "ports": port_results
                }
                status_list.append(status)
                logger.debug(f"{node_name}: Port check passed (all ports listening)")
                continue
            else:
                # 端口未全部監聽，但不直接判定為 stopped，繼續其他檢查
                status["details"]["port_check"] = {
                    "method": "port",
                    "ports": port_results,
                    "note": "Some ports not listening"
                }

        # 第2層：使用配置的 status_check 命令（優先於 ROS 2 節點列表）
        status_check_cmd = node_info.get("status_check")
        if status_check_cmd:
            try:
                result = await NodeManager.run_command(status_check_cmd, timeout=3)
                if result["success"] and result["stdout"]:
                    # status_check 命令成功返回輸出，視為運行中
                    status["status"] = "running"
                    status["running"] = True
                    status["details"]["check_method"] = "status_check"
                    status_list.append(status)
                    logger.debug(f"{node_name}: status_check passed")
                    continue
                else:
                    # status_check 失敗，繼續其他檢查
                    status["details"]["status_check"] = "failed"
            except Exception as e:
                logger.warning(f"{node_name}: status_check error: {e}")

        # 第3層：PID 文件驗證
        if node_info.get("type") == "launch":
            pid_file = f"/tmp/{node_name}.pid"
            is_valid = await check_pid_file_valid(pid_file)
            if is_valid:
                status["status"] = "running"
                status["running"] = True
                status["details"]["check_method"] = "pid_file"
                status_list.append(status)
                logger.debug(f"{node_name}: PID file check passed")
                continue

        # 第4層：ROS 2 節點列表檢查（最後備用）
        if running_nodes:
            # 節點名稱映射（從配置中的 nodes 列表或特殊映射）
            node_names_to_check = node_info.get("nodes", [])  # Launch 類型可能有多個子節點
            if not node_names_to_check:
                # 單節點，使用節點名稱
                node_names_to_check = [node_name]

            # 檢查任一子節點是否在運行列表中
            for check_name in node_names_to_check:
                if check_name in running_nodes or any(check_name in rn for rn in running_nodes):
                    status["status"] = "running"
                    status["running"] = True
                    status["details"]["check_method"] = "ros2_node_list"
                    break

        # 如果所有檢查都失敗，標記為 stopped
        if status["status"] == "unknown":
            status["status"] = "stopped"
            status["running"] = False

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
    
    cmd = f"bash -c 'source /app/setup.bash && agvc_source && manage_remote_agv_launch {agv_name} {action}'"
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