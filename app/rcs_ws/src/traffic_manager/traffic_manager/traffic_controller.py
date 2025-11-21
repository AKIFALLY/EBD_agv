import logging
from db_proxy.connection_pool_manager import ConnectionPoolManager
from db_proxy.models import TrafficZone, AGV
from sqlmodel import select


class TrafficController:
    def __init__(self, db_pool: ConnectionPoolManager):
        """
        Initialize the TrafficController with a reference to AGVCDatabaseNode.
        :param db_pool: Database connection pool (should provide get_session()).
        """
        self.db_pool = db_pool
        self.logger = logging.getLogger(__name__)

    def acquire_traffic_zone(self, traffic_id, agv_id):
        """
        Acquire control of a traffic zone.
        :param traffic_id: ID of the traffic zone.
        :param agv_id: ID of the AGV requesting control.
        :return: dict with 'success' (bool), 'reason' (str), and 'owner_agv_id' (int, optional)
        """
        try:
            with self.db_pool.get_session() as session:
                zone = session.exec(
                    select(TrafficZone).where(TrafficZone.id == traffic_id)
                ).first()

                # 檢查交管區是否存在
                if not zone:
                    return {
                        "success": False,
                        "reason": "zone_not_found",
                        "message": f"交管區 ID {traffic_id} 不存在"
                    }

                # 檢查交管區是否啟用
                if not zone.enable:
                    self.logger.warning(
                        f"[交管區禁用] AGV {agv_id} 請求交管區 {traffic_id} - "
                        f"交管區已禁用 (enable=False) - 拒絕請求"
                    )
                    return {
                        "success": False,
                        "reason": "zone_disabled",
                        "message": f"交管區 ID {traffic_id} 已被禁用"
                    }

                # 檢查交管區是否可用
                if zone.status != "free":
                    # 冪等性檢查：如果是同一個 AGV 請求，視為冪等操作
                    if zone.owner_agv_id == agv_id:
                        self.logger.info(
                            f"[交管區冪等] AGV {agv_id} 請求交管區 {traffic_id} - "
                            f"狀態: {zone.status}, 當前擁有者: AGV {zone.owner_agv_id} - "
                            f"冪等操作，允許重複獲取"
                        )
                        return {"success": True, "message": "交管區已被本車持有"}
                    else:
                        # KUKA400i 共享檢查：如果擁有者和請求者都是 KUKA400i，允許共享通行
                        owner_agv = session.get(AGV, zone.owner_agv_id)
                        requester_agv = session.get(AGV, agv_id)

                        if (owner_agv and requester_agv and
                            owner_agv.model == "KUKA400i" and
                            requester_agv.model == "KUKA400i"):
                            self.logger.info(
                                f"[KUKA400i共享] AGV {agv_id}(KUKA400i) 請求交管區 {traffic_id} - "
                                f"當前擁有者: AGV {zone.owner_agv_id}(KUKA400i) - "
                                f"允許KUKA400i共享通行，不改變擁有者"
                            )
                            return {
                                "success": True,
                                "message": "KUKA400i共享通行",
                                "owner_agv_id": zone.owner_agv_id
                            }

                        # 非 KUKA400i 或只有一方是 KUKA400i，拒絕請求
                        self.logger.warning(
                            f"[交管區拒絕] AGV {agv_id} 請求交管區 {traffic_id} - "
                            f"狀態: {zone.status}, 當前擁有者: AGV {zone.owner_agv_id} - "
                            f"拒絕請求"
                        )
                        return {
                            "success": False,
                            "reason": "zone_controlled",
                            "message": f"交管區已被 AGV {zone.owner_agv_id} 占用",
                            "owner_agv_id": zone.owner_agv_id
                        }

                # 獲取交管區使用權
                zone.status = "controlled"
                zone.owner_agv_id = agv_id
                session.add(zone)
                session.commit()
                session.refresh(zone)
                self.logger.info(
                    f"[交管區獲取] AGV {agv_id} 請求交管區 {traffic_id} - "
                    f"原狀態: free → 新狀態: controlled, 擁有者: AGV {agv_id} - "
                    f"成功獲取"
                )
                return {"success": True, "message": "成功獲取交管區使用權"}

        except Exception as e:
            self.logger.error(f"Error acquiring traffic zone: {e}")
            return {
                "success": False,
                "reason": "database_error",
                "message": f"資料庫錯誤: {str(e)}"
            }

    def release_traffic_zone(self, traffic_id, agv_id):
        """
        Release control of a traffic zone.
        :param traffic_id: ID of the traffic zone.
        :param agv_id: ID of the AGV releasing control.
        :return: True if release is successful, False otherwise.
        """
        try:
            with self.db_pool.get_session() as session:
                zone = session.exec(
                    select(TrafficZone).where(TrafficZone.id == traffic_id)
                ).first()

                # 檢查交管區是否存在
                if not zone:
                    return False

                # 情況 1: 擁有者釋放 → 真正釋放交管區
                if zone.owner_agv_id == agv_id:
                    if zone.status == "controlled":
                        zone.status = "free"
                        zone.owner_agv_id = None
                        session.add(zone)
                        session.commit()
                        session.refresh(zone)
                        self.logger.info(
                            f"[交管區釋放] AGV {agv_id} 釋放交管區 {traffic_id} - "
                            f"原狀態: controlled → 新狀態: free - 成功釋放"
                        )
                        return True
                    else:
                        # 交管區已經是 free 狀態，直接返回成功
                        return True

                # 情況 2: KUKA400i 共享者釋放 → 允許但不實際釋放
                if zone.status == "controlled" and zone.owner_agv_id is not None:
                    owner_agv = session.get(AGV, zone.owner_agv_id)
                    requester_agv = session.get(AGV, agv_id)

                    if (owner_agv and requester_agv and
                        owner_agv.model == "KUKA400i" and
                        requester_agv.model == "KUKA400i"):
                        self.logger.info(
                            f"[KUKA400i共享釋放] AGV {agv_id}(KUKA400i) 請求釋放交管區 {traffic_id} - "
                            f"當前擁有者: AGV {zone.owner_agv_id}(KUKA400i) - "
                            f"共享者釋放請求，保持擁有者不變"
                        )
                        return True

                # 情況 3: 其他情況 → 拒絕釋放
                self.logger.warning(
                    f"[交管區釋放拒絕] AGV {agv_id} 嘗試釋放交管區 {traffic_id} - "
                    f"當前擁有者: AGV {zone.owner_agv_id} - 拒絕請求"
                )
                return False

        except Exception as e:
            self.logger.error(f"Error releasing traffic zone: {e}")
            return False

    def acquire_traffic_zone_by_name(self, traffic_name, agv_name):
        """
        Acquire control of a traffic zone by its name.
        :param traffic_name: Name of the traffic zone.
        :param agv_name: Name of the AGV requesting control.
        :return: dict with 'success' (bool), 'reason' (str), 'message' (str), and 'owner_agv_id' (int, optional)
        """
        try:
            with self.db_pool.get_session() as session:
                zone = session.exec(
                    select(TrafficZone).where(TrafficZone.name == traffic_name)
                ).first()
                agv = session.exec(
                    select(AGV).where(AGV.name == agv_name)
                ).first()

                # 檢查交管區是否存在
                if not zone:
                    return {
                        "success": False,
                        "reason": "zone_not_found",
                        "message": f"交管區 '{traffic_name}' 不存在"
                    }

                # 檢查 AGV 是否存在
                if not agv:
                    return {
                        "success": False,
                        "reason": "agv_not_found",
                        "message": f"AGV '{agv_name}' 不存在"
                    }

                # 檢查交管區是否啟用
                if not zone.enable:
                    self.logger.warning(
                        f"[交管區禁用] AGV {agv_name}(ID:{agv.id}) 請求交管區 '{traffic_name}'(ID:{zone.id}) - "
                        f"交管區已禁用 (enable=False) - 拒絕請求"
                    )
                    return {
                        "success": False,
                        "reason": "zone_disabled",
                        "message": f"交管區 '{traffic_name}' 已被禁用"
                    }

                # 冪等性檢查：如果交管區已被該 AGV 佔用，返回成功
                if zone.status == "controlled" and zone.owner_agv_id == agv.id:
                    self.logger.info(
                        f"[交管區冪等] AGV {agv_name}(ID:{agv.id}) 請求交管區 '{traffic_name}'(ID:{zone.id}) - "
                        f"狀態: {zone.status}, 當前擁有者: AGV {agv_name}(ID:{zone.owner_agv_id}) - "
                        f"冪等操作，允許重複獲取"
                    )
                    return {"success": True, "message": "交管區已被本車持有"}

                # 交管區空閒，正常獲取
                if zone.status == "free":
                    zone.status = "controlled"
                    zone.owner_agv_id = agv.id
                    session.add(zone)
                    session.commit()
                    session.refresh(zone)
                    self.logger.info(
                        f"[交管區獲取] AGV {agv_name}(ID:{agv.id}) 請求交管區 '{traffic_name}'(ID:{zone.id}) - "
                        f"原狀態: free → 新狀態: controlled, 擁有者: AGV {agv_name}(ID:{agv.id}) - "
                        f"成功獲取"
                    )
                    return {"success": True, "message": "成功獲取交管區使用權"}

                # 交管區已被占用
                # KUKA400i 共享檢查：如果擁有者和請求者都是 KUKA400i，允許共享通行
                owner_agv = session.get(AGV, zone.owner_agv_id)

                if (owner_agv and
                    owner_agv.model == "KUKA400i" and
                    agv.model == "KUKA400i"):
                    self.logger.info(
                        f"[KUKA400i共享] AGV {agv_name}(ID:{agv.id}, KUKA400i) 請求交管區 '{traffic_name}'(ID:{zone.id}) - "
                        f"當前擁有者: AGV ID {zone.owner_agv_id}(KUKA400i) - "
                        f"允許KUKA400i共享通行，不改變擁有者"
                    )
                    return {
                        "success": True,
                        "message": "KUKA400i共享通行",
                        "owner_agv_id": zone.owner_agv_id
                    }

                # 非 KUKA400i 或只有一方是 KUKA400i，拒絕請求
                self.logger.warning(
                    f"[交管區拒絕] AGV {agv_name}(ID:{agv.id}) 請求交管區 '{traffic_name}'(ID:{zone.id}) - "
                    f"狀態: {zone.status}, 當前擁有者: AGV ID {zone.owner_agv_id} - "
                    f"拒絕請求"
                )
                return {
                    "success": False,
                    "reason": "zone_controlled",
                    "message": f"交管區已被 AGV ID {zone.owner_agv_id} 占用",
                    "owner_agv_id": zone.owner_agv_id
                }

        except Exception as e:
            self.logger.error(f"Error acquiring traffic zone by name: {e}")
            return {
                "success": False,
                "reason": "database_error",
                "message": f"資料庫錯誤: {str(e)}"
            }

    def release_traffic_zone_by_name(self, traffic_name, agv_name):
        """
        Release control of a traffic zone by its name.
        :param traffic_name: Name of the traffic zone.
        :param agv_name: Name of the AGV releasing control.
        :return: dict with 'success' (bool), 'reason' (str), and 'message' (str)
        """
        try:
            with self.db_pool.get_session() as session:
                zone = session.exec(
                    select(TrafficZone).where(TrafficZone.name == traffic_name)
                ).first()
                agv = session.exec(
                    select(AGV).where(AGV.name == agv_name)
                ).first()

                # 檢查交管區是否存在
                if not zone:
                    return {
                        "success": False,
                        "reason": "zone_not_found",
                        "message": f"交管區 '{traffic_name}' 不存在"
                    }

                # 檢查 AGV 是否存在
                if not agv:
                    return {
                        "success": False,
                        "reason": "agv_not_found",
                        "message": f"AGV '{agv_name}' 不存在"
                    }

                # 情況 1: 擁有者釋放 → 真正釋放交管區
                if zone.owner_agv_id == agv.id:
                    if zone.status == "controlled":
                        zone.status = "free"
                        zone.owner_agv_id = None
                        session.add(zone)
                        session.commit()
                        session.refresh(zone)
                        self.logger.info(
                            f"[交管區釋放] AGV {agv_name}(ID:{agv.id}) 釋放交管區 '{traffic_name}'(ID:{zone.id}) - "
                            f"原狀態: controlled → 新狀態: free - 成功釋放"
                        )
                        return {"success": True, "message": "成功釋放交管區"}
                    else:
                        # 交管區已經是 free 狀態，直接返回成功（冪等性）
                        return {"success": True, "message": "交管區已經是空閒狀態"}

                # 情況 2: KUKA400i 共享者釋放 → 允許但不實際釋放
                if zone.status == "controlled" and zone.owner_agv_id is not None:
                    owner_agv = session.get(AGV, zone.owner_agv_id)

                    if (owner_agv and
                        owner_agv.model == "KUKA400i" and
                        agv.model == "KUKA400i"):
                        self.logger.info(
                            f"[KUKA400i共享釋放] AGV {agv_name}(ID:{agv.id}, KUKA400i) 請求釋放交管區 '{traffic_name}'(ID:{zone.id}) - "
                            f"當前擁有者: AGV ID {zone.owner_agv_id}(KUKA400i) - "
                            f"共享者釋放請求，保持擁有者不變"
                        )
                        return {"success": True, "message": "KUKA400i共享釋放"}

                # 情況 3: 其他情況 → 拒絕釋放
                self.logger.warning(
                    f"[交管區釋放拒絕] AGV {agv_name}(ID:{agv.id}) 嘗試釋放交管區 '{traffic_name}'(ID:{zone.id}) - "
                    f"當前擁有者: AGV ID {zone.owner_agv_id} - 拒絕請求"
                )
                return {
                    "success": False,
                    "reason": "not_owner",
                    "message": f"無權釋放，當前擁有者為 AGV ID {zone.owner_agv_id}"
                }

        except Exception as e:
            self.logger.error(f"Error releasing traffic zone by name: {e}")
            return {
                "success": False,
                "reason": "database_error",
                "message": f"資料庫錯誤: {str(e)}"
            }
