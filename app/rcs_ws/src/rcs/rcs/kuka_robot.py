"""
KUKA 機器人管理模組
處理 KUKA 機器人狀態更新和相關操作
"""
import traceback
from db_proxy.crud.agv_crud import agv_crud
from db_proxy.models import AGV, ModifyLog


class KukaRobot:
    """KUKA 機器人管理類別"""

    def __init__(self, rcs_core_node):
        """
        初始化 KUKA 機器人管理器

        Args:
            rcs_core_node: RCS Core 節點實例，用於日誌記錄和資料庫存取
        """
        self.rcs_core = rcs_core_node
        self.logger = rcs_core_node.get_logger()

    def kuka_unit_2_px(self, y, x):
        """將 KUKA 單位轉換為像素座標"""
        # m to mm to px 0.0125 m => 1 px
        return y / 12.5, x / 12.5

    def kuka_angle_2_map_angle(self, angle):
        """將 KUKA 角度轉換為地圖角度"""
        # map angle to kuka angle
        angle = ((-1 * angle + 90) + 540 % 360) - 180
        return angle

    def on_robot_update(self, robots: list):
        """
        當 KukaFleetAdapter 查詢到機器人狀態時，更新資料庫。

        Args:
            robots: 機器人狀態列表
        """
        if not self._validate_database_connection():
            return

        # self.logger.info(
        #    f"--- [KUKA Robot] 接收到 {len(robots)} 台 KUKA 機器人狀態更新，準備寫入資料庫 ---")

        try:
            with self.rcs_core.db_pool.get_session() as session:
                for robot in robots:
                    if self._process_single_robot(session, robot):
                        self.logger.debug(f"已成功更新 AGV (ID: {robot.get('robotId')}) 的狀態。")

                # 標記 AGV 資料已更新
                ModifyLog.mark(session, "agv")

        except Exception as e:
            self.logger.error(f"更新 AGV 狀態時發生資料庫錯誤: {e}", exc_info=True)

    def _validate_database_connection(self):
        """驗證資料庫連線是否可用"""
        if not self.rcs_core.db_pool:
            self.logger.error("資料庫連線池不可用，無法更新機器人狀態。")
            return False
        return True

    def _process_single_robot(self, session, robot_data: dict):
        """
        處理單個機器人的資料更新

        Args:
            session: 資料庫 session
            robot_data: 機器人資料字典

        Returns:
            bool: 是否成功處理
        """
        robot_id = robot_data.get("robotId")
        if not robot_id:
            return False

        try:
            # 驗證機器人資料
            if not self.validate_robot_data(robot_data):
                return False

            # 取得機器人在資料庫中的記錄
            agv = self._get_agv_from_database(session, robot_id)
            if not agv:
                return False

            # 更新 AGV 資料
            self._update_agv_data(agv, robot_data)

            # 提交變更
            session.commit()
            return True

        except ValueError as e:
            # AGV 不存在的情況，不記錄錯誤
            return False
        except Exception as e:
            self.logger.error(f"更新 AGV 狀態時發生資料庫錯誤: {e}")
            tb_str = traceback.format_exc()
            self.logger.error(f"堆疊訊息:\n{tb_str}")
            return False

    def _get_agv_from_database(self, session, robot_id: str):
        """
        從資料庫取得 AGV 記錄

        Args:
            session: 資料庫 session
            robot_id: 機器人 ID

        Returns:
            AGV: AGV 物件，如果不存在則拋出 ValueError
        """
        agv: AGV = agv_crud.get_by_id(session, robot_id)
        if not agv:
            raise ValueError(f"AGV with ID {robot_id} does not exist.")
        return agv

    def _update_agv_data(self, agv: AGV, robot_data: dict):
        """
        更新 AGV 物件的資料

        Args:
            agv: AGV 物件
            robot_data: 機器人資料字典
        """
        # 取得位置和角度
        pos_px_x, pos_px_y = self.get_robot_position(robot_data)
        heading = self.get_robot_heading(robot_data)

        # 取得節點編號
        node_number = self._get_node_number(robot_data)

        # 更新 AGV 屬性
        agv.x = pos_px_x
        agv.y = pos_px_y
        agv.heading = heading
        agv.battery = robot_data.get("batteryLevel")
        agv.last_node_id = node_number
        agv.status_id = robot_data.get("status")

    def _get_node_number(self, robot_data: dict):
        """
        取得機器人的節點編號

        Args:
            robot_data: 機器人資料字典

        Returns:
            int: 節點編號，如果無效則返回 None
        """
        node_number_str = robot_data.get("nodeNumber")
        if node_number_str and node_number_str != '':
            try:
                return int(node_number_str)
            except (ValueError, TypeError):
                self.logger.warning(f"無效的節點編號: {node_number_str}")
                return None
        return None

    def get_robot_status(self, robot_id: str):
        """
        取得指定機器人的狀態

        Args:
            robot_id: 機器人 ID

        Returns:
            dict: 機器人狀態資訊，如果找不到則返回 None
        """
        if not self.rcs_core.db_pool:
            self.logger.error("資料庫連線池不可用，無法查詢機器人狀態。")
            return None

        try:
            with self.rcs_core.db_pool.get_session() as session:
                agv = agv_crud.get_by_id(session, robot_id)
                if agv:
                    return {
                        "robotId": agv.id,
                        "x": agv.x,
                        "y": agv.y,
                        "heading": agv.heading,
                        "battery": agv.battery,
                        "status": agv.status_id,
                        "last_node_id": agv.last_node_id
                    }
                return None
        except Exception as e:
            self.logger.error(f"查詢機器人狀態時發生錯誤: {e}")
            return None

    def validate_robot_data(self, robot_data: dict):
        """
        驗證機器人資料的完整性

        Args:
            robot_data: 機器人資料字典

        Returns:
            bool: 資料是否有效
        """
        if not robot_data:
            self.logger.warning("機器人資料為空")
            return False

        required_fields = ["robotId", "x", "y", "robotOrientation", "status"]

        for field in required_fields:
            if field not in robot_data or robot_data[field] is None:
                self.logger.warning(f"機器人資料缺少必要欄位: {field}")
                return False

        # 驗證數值型欄位
        try:
            float(robot_data["x"])
            float(robot_data["y"])
            float(robot_data["robotOrientation"])
        except (ValueError, TypeError) as e:
            self.logger.warning(f"機器人座標或角度資料格式錯誤: {e}")
            return False

        return True

    def get_robot_position(self, robot_data: dict):
        """
        取得機器人的像素座標位置

        Args:
            robot_data: 機器人資料字典

        Returns:
            tuple: (x, y) 像素座標
        """
        if not self.validate_robot_data(robot_data):
            return None, None

        pos_px_y, pos_px_x = self.kuka_unit_2_px(
            float(robot_data.get("y")),
            float(robot_data.get("x"))
        )

        return pos_px_x, pos_px_y

    def get_robot_heading(self, robot_data: dict):
        """
        取得機器人的地圖角度

        Args:
            robot_data: 機器人資料字典

        Returns:
            float: 地圖角度
        """
        if not self.validate_robot_data(robot_data):
            return None

        angle = self.kuka_angle_2_map_angle(
            float(robot_data.get("robotOrientation"))
        )

        return angle

    def get_robot_battery_status(self, robot_data: dict):
        """
        取得機器人電池狀態分析

        Args:
            robot_data: 機器人資料字典

        Returns:
            dict: 電池狀態分析
        """
        battery_level = robot_data.get("batteryLevel", 0)

        if battery_level >= 80:
            status = "excellent"
        elif battery_level >= 50:
            status = "good"
        elif battery_level >= 20:
            status = "low"
        else:
            status = "critical"

        return {
            "level": battery_level,
            "status": status,
            "needs_charging": battery_level < 20
        }

    def is_robot_available(self, robot_data: dict):
        """
        檢查機器人是否可用於任務

        Args:
            robot_data: 機器人資料字典

        Returns:
            bool: 機器人是否可用
        """
        if not self.validate_robot_data(robot_data):
            return False

        status = robot_data.get("status")
        battery_info = self.get_robot_battery_status(robot_data)

        # 狀態為 3 (空閒) 且電量充足
        return status == 3 and not battery_info["needs_charging"]

    def log_robot_status_change(self, robot_id: str, old_status: int, new_status: int):
        """
        記錄機器人狀態變更

        Args:
            robot_id: 機器人 ID
            old_status: 舊狀態
            new_status: 新狀態
        """
        status_names = {
            1: "離場",
            2: "離線",
            3: "空閒",
            4: "任務中",
            5: "充電中",
            6: "更新中",
            7: "異常"
        }

        old_name = status_names.get(old_status, f"未知({old_status})")
        new_name = status_names.get(new_status, f"未知({new_status})")

        self.logger.info(f"機器人 {robot_id} 狀態變更: {old_name} → {new_name}")
