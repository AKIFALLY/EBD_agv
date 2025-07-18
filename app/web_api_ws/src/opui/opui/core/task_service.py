# 任務業務邏輯服務層
from typing import Dict, Any, Tuple, Optional
from opui.database.operations import (
    create_task, get_call_empty_work_id, get_dispatch_full_work_id,
    connection_pool, machine_crud,
    task_crud, rack_crud, location_crud, client_crud, delete_task_by_parking
)
from db_proxy.models import Rack, TaskStatus
from db_proxy.models.machine import Machine
from db_proxy.crud.machine_crud import machine_crud as db_machine_crud
from sqlmodel import select
from opui.services import OpuiTaskService


class TaskService:
    """任務相關業務邏輯服務 - 保持向後相容性的包裝層"""

    def __init__(self):
        # 使用新的 OPUI Service 層
        self.opui_task_service = OpuiTaskService(connection_pool)

    def check_parking_space_status(self, machine_id: int, node_id: int) -> Tuple[bool, str]:
        """檢查停車格狀態 - 委託給新的 Service 層"""
        with connection_pool.get_session() as session:
            # 先確定是哪個停車格
            machine = self.opui_task_service.machine_service.crud.get_by_id(session, machine_id)
            if not machine:
                return False, "找不到機台資訊"

            space_num = None
            if getattr(machine, 'parking_space_1', None) == node_id:
                space_num = 1
            elif getattr(machine, 'parking_space_2', None) == node_id:
                space_num = 2
            else:
                return False, f"停車位 [{node_id}] 不屬於機台 [{machine_id}]"

            # 使用新的 Service 層檢查狀態
            if self.opui_task_service.machine_service.is_parking_task_active(machine_id, space_num, session):
                return False, f"停車位 [{node_id}] 已有車輛，無法叫車"
            elif self.opui_task_service.machine_service.is_parking_task_completed(machine_id, space_num, session):
                return False, f"停車位 [{node_id}] 已送達，請先確認取貨"
            elif self.opui_task_service.machine_service.is_parking_available(machine_id, space_num, session):
                return True, "停車位可用"
            else:
                return False, f"停車位 [{node_id}] 狀態異常"

    def update_machine_parking_status(self, machine_id: int, node_id: int, status: int = Machine.PARKING_TASK_ACTIVE) -> bool:
        """更新機台停車格狀態 - 委託給新的 Service 層"""
        with connection_pool.get_session() as session:
            # 先確定是哪個停車格
            machine = self.opui_task_service.machine_service.crud.get_by_id(session, machine_id)
            if not machine:
                return False

            space_num = None
            if getattr(machine, 'parking_space_1', None) == node_id:
                space_num = 1
            elif getattr(machine, 'parking_space_2', None) == node_id:
                space_num = 2
            else:
                return False

            # 使用新的 Service 層更新狀態
            result = self.opui_task_service.machine_service.update_parking_status(
                machine_id, space_num, status, session
            )
            if result:
                print(f"🔄 更新停車格狀態: machine_id={machine_id}, node_id={node_id}, status={status}")
                return True
            return False

    def create_call_empty_task(self, client_id: str, machine_id: int, node_id: int) -> Dict[str, Any]:
        """創建叫空車任務"""
        try:
            # 檢查停車格狀態
            ok, msg = self.check_parking_space_status(machine_id, node_id)
            if not ok:
                return {"success": False, "message": msg}

            # 創建任務
            work_id = get_call_empty_work_id()
            status_id = TaskStatus.REQUESTING

            task_data = {
                "work_id": work_id,
                "status_id": status_id,
                "parameters": {
                    "client_id": client_id,
                    "machine_id": machine_id,
                    "node_id": node_id,
                    "task_type": "call_empty"
                }
            }

            task = create_task(task_data)
            if task:
                # 更新停車格狀態為任務進行中
                self.update_machine_parking_status(machine_id, node_id, Machine.PARKING_TASK_ACTIVE)
                return {
                    "success": True,
                    "message": f"叫空車成功，任務ID: {task.id}",
                    "task_id": task.id
                }
            else:
                return {"success": False, "message": "任務創建失敗"}

        except Exception as e:
            print(f"[create_call_empty_task] 錯誤: {str(e)}")
            return {"success": False, "message": f"叫車失敗: {str(e)}"}

    def create_dispatch_full_task(self, client_id: str, machine_id: int, node_id: int,
                                  product_name: str, count: int, rack_id: int, room: int) -> Dict[str, Any]:
        """創建派滿車任務"""
        try:
            # 檢查停車格狀態
            ok, msg = self.check_parking_space_status(machine_id, node_id)
            if not ok:
                return {"success": False, "message": msg}

            # 創建任務
            work_id = get_dispatch_full_work_id()
            status_id = TaskStatus.REQUESTING

            task_data = {
                "work_id": work_id,
                "status_id": status_id,
                "parameters": {
                    "client_id": client_id,
                    "machine_id": machine_id,
                    "node_id": node_id,
                    "task_type": "dispatch_full",
                    "product_name": product_name,
                    "count": count,
                    "rack_id": rack_id,
                    "room": room
                }
            }

            task = create_task(task_data)
            if task:
                # 更新停車格狀態為任務進行中
                self.update_machine_parking_status(machine_id, node_id, Machine.PARKING_TASK_ACTIVE)
                return {
                    "success": True,
                    "message": f"派滿車成功，任務ID: {task.id}",
                    "task_id": task.id
                }
            else:
                return {"success": False, "message": "任務創建失敗"}

        except Exception as e:
            print(f"[create_dispatch_full_task] 錯誤: {str(e)}")
            return {"success": False, "message": f"派車失敗: {str(e)}"}

    def cancel_task(self, machine_id: int, node_id: int) -> Dict[str, Any]:
        """取消任務"""
        try:
            # 刪除任務
            deleted = delete_task_by_parking(node_id)
            if not deleted:
                print(f"[cancel_task] delete_task_by_parking({node_id}) failed")

            # 重設機台停車格狀態為可用
            updated = self.update_machine_parking_status(
                machine_id, node_id, Machine.PARKING_AVAILABLE)

            return {
                "success": True,
                "message": f"已取消停車位 [{node_id}] 的任務",
                "updated": updated
            }

        except Exception as e:
            print(f"[cancel_task] 錯誤: {str(e)}")
            return {"success": False, "message": f"取消任務失敗: {str(e)}"}

    def get_parking_list_by_machine_id(self, machine_id: int) -> Dict[str, Any]:
        """根據機台ID獲取停車位列表"""
        parking_list = {"left": [], "right": []}

        with connection_pool.get_session() as session:
            machine = machine_crud.get_by_id(session, int(machine_id))

            if not machine:
                return parking_list

            # 獲取左側停車位的料架
            if machine.parking_space_1:
                left_racks = rack_crud.get_by_location_id(session, machine.parking_space_1)
                parking_list["left"] = [
                    {"id": r.id, "name": r.name} for r in left_racks]

            # 獲取右側停車位的料架
            if machine.parking_space_2:
                right_racks = rack_crud.get_by_location_id(session, machine.parking_space_2)
                parking_list["right"] = [
                    {"id": r.id, "name": r.name} for r in right_racks]

        return parking_list

    def add_rack(self, side: str, rack_name: str, machine_id: int) -> Dict[str, Any]:
        """新增料架"""
        try:
            if side is None or not rack_name:
                return {"success": False, "message": "缺少必要參數"}

            with connection_pool.get_session() as session:
                machine = machine_crud.get_by_id(session, int(machine_id))
                if not machine:
                    return {"success": False, "message": "找不到機台資訊"}

                # 根據 side 決定 location_id
                if side == "left":
                    location_id = machine.parking_space_1
                elif side == "right":
                    location_id = machine.parking_space_2
                else:
                    return {"success": False, "message": "無效的 side 參數"}

                if not location_id:
                    return {"success": False, "message": f"機台 {side} 側沒有配置停車位"}

                # 檢查料架名稱是否已存在
                existing_racks = rack_crud.get_by_location_id(session, location_id)
                if any(r.name == rack_name for r in existing_racks):
                    return {"success": False, "message": f"料架名稱 '{rack_name}' 已存在"}

                # 創建新料架
                new_rack = Rack(name=rack_name, location_id=location_id)
                rack_crud.create(session, new_rack)

                return {"success": True, "message": f"料架 '{rack_name}' 新增成功"}

        except Exception as e:
            return {"success": False, "message": f"料架新增失敗: {str(e)}"}

    def delete_rack(self, rack_id: int) -> Dict[str, Any]:
        """刪除料架"""
        try:
            if not rack_id:
                return {"success": False, "message": "缺少必要參數rackId"}

            with connection_pool.get_session() as session:
                rack = rack_crud.get_by_id(session, rack_id)
                if not rack:
                    return {"success": False, "message": f"找不到料架 ID {rack_id}"}

                rack_name = rack.name
                rack_crud.delete(session, rack_id)

                return {"success": True, "message": f"料架 '{rack_name}' 刪除成功"}

        except Exception as e:
            return {"success": False, "message": f"料架刪除失敗: {str(e)}"}

    def get_rack_name_by_id(self, rack_id: int) -> Optional[str]:
        """根據rack_id從資料庫獲取rack名稱"""
        try:
            with connection_pool.get_session() as session:
                statement = select(Rack).where(Rack.id == rack_id)
                result = session.exec(statement).first()
                return result.name if result else None
        except Exception as e:
            print(f"❌ 獲取料架名稱失敗: {e}")
            return None

    def get_current_parking_status(self, machine_id: int, node_id: int) -> int:
        """獲取當前停車格狀態"""
        try:
            with connection_pool.get_session() as session:
                machine = machine_crud.get_by_id(session, machine_id)
                if machine:
                    if getattr(machine, 'parking_space_1', None) == node_id:
                        return machine.parking_space_1_status
                    elif getattr(machine, 'parking_space_2', None) == node_id:
                        return machine.parking_space_2_status
            return Machine.PARKING_AVAILABLE
        except Exception as e:
            print(f"❌ 獲取停車格狀態失敗: {e}")
            return Machine.PARKING_AVAILABLE

    def client_uses_machine(self, client_id: str, machine_id: int) -> bool:
        """檢查客戶端是否使用指定的機台"""
        try:
            with connection_pool.get_session() as session:
                client = client_crud.get_by_id(session, client_id)
                return client and getattr(client, 'machine_id', None) == machine_id
        except Exception as e:
            print(f"❌ 檢查客戶端機台失敗: {e}")
            return False
