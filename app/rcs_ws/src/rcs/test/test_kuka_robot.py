"""
KukaRobot 單元測試
測試 KUKA 機器人管理模組的所有功能
"""
import pytest
from unittest.mock import Mock, MagicMock, patch
from datetime import datetime, timezone

# Import the class under test
from rcs.kuka_robot import KukaRobot


class TestKukaRobot:
    """KukaRobot 測試類別"""

    def test_init(self, mock_rcs_core):
        """測試 KukaRobot 初始化"""
        robot = KukaRobot(mock_rcs_core)
        
        assert robot.rcs_core == mock_rcs_core
        assert robot.logger == mock_rcs_core.get_logger.return_value

    def test_kuka_unit_2_px(self, mock_rcs_core):
        """測試 KUKA 單位轉換為像素座標"""
        robot = KukaRobot(mock_rcs_core)
        
        # 測試基本轉換 (除以 12.5)
        y_px, x_px = robot.kuka_unit_2_px(125.0, 250.0)
        assert y_px == 10.0  # 125 / 12.5
        assert x_px == 20.0  # 250 / 12.5
        
        # 測試零值
        y_px, x_px = robot.kuka_unit_2_px(0, 0)
        assert y_px == 0.0
        assert x_px == 0.0

    def test_kuka_angle_2_map_angle(self, mock_rcs_core):
        """測試 KUKA 角度轉換為地圖角度"""
        robot = KukaRobot(mock_rcs_core)
        
        # 測試角度轉換
        # 公式: ((-1 * angle + 90) + 540 % 360) - 180
        map_angle = robot.kuka_angle_2_map_angle(0)
        expected = ((-1 * 0 + 90) + 540 % 360) - 180
        assert map_angle == expected
        
        map_angle = robot.kuka_angle_2_map_angle(90)
        expected = ((-1 * 90 + 90) + 540 % 360) - 180
        assert map_angle == expected

    def test_validate_robot_data_success(self, mock_rcs_core, sample_robot_data):
        """測試機器人資料驗證成功"""
        robot = KukaRobot(mock_rcs_core)
        
        assert robot.validate_robot_data(sample_robot_data) is True

    def test_validate_robot_data_missing_fields(self, mock_rcs_core, mock_logger):
        """測試機器人資料缺少必要欄位"""
        robot = KukaRobot(mock_rcs_core)
        
        # 測試空資料
        assert robot.validate_robot_data(None) is False
        mock_logger.warning.assert_called_with("機器人資料為空")
        
        # 測試缺少 robotId
        invalid_data = {"x": 10, "y": 20}
        assert robot.validate_robot_data(invalid_data) is False
        mock_logger.warning.assert_called_with("機器人資料缺少必要欄位: robotId")

    def test_validate_robot_data_invalid_types(self, mock_rcs_core, mock_logger):
        """測試機器人資料類型錯誤"""
        robot = KukaRobot(mock_rcs_core)
        
        # 測試無效的座標資料
        invalid_data = {
            "robotId": "101",
            "x": "invalid",  # 應該是數字
            "y": 20,
            "robotOrientation": 90,
            "status": 3
        }
        
        assert robot.validate_robot_data(invalid_data) is False
        mock_logger.warning.assert_called()

    def test_get_robot_position(self, mock_rcs_core, sample_robot_data):
        """測試取得機器人位置"""
        robot = KukaRobot(mock_rcs_core)
        
        pos_x, pos_y = robot.get_robot_position(sample_robot_data)
        
        # 驗證位置轉換: y/12.5, x/12.5 並且 x, y 交換
        expected_x = sample_robot_data["x"] / 12.5
        expected_y = sample_robot_data["y"] / 12.5
        
        assert pos_x == expected_x
        assert pos_y == expected_y

    def test_get_robot_position_invalid_data(self, mock_rcs_core):
        """測試取得機器人位置 - 無效資料"""
        robot = KukaRobot(mock_rcs_core)
        
        pos_x, pos_y = robot.get_robot_position(None)
        assert pos_x is None
        assert pos_y is None

    def test_get_robot_heading(self, mock_rcs_core, sample_robot_data):
        """測試取得機器人方位角"""
        robot = KukaRobot(mock_rcs_core)
        
        heading = robot.get_robot_heading(sample_robot_data)
        
        # 驗證角度轉換
        expected = robot.kuka_angle_2_map_angle(sample_robot_data["robotOrientation"])
        assert heading == expected

    def test_get_robot_battery_status(self, mock_rcs_core):
        """測試電池狀態分析"""
        robot = KukaRobot(mock_rcs_core)
        
        # 測試優秀電量 (>= 80)
        status = robot.get_robot_battery_status({"batteryLevel": 90})
        assert status["level"] == 90
        assert status["status"] == "excellent"
        assert status["needs_charging"] is False
        
        # 測試良好電量 (50-79)
        status = robot.get_robot_battery_status({"batteryLevel": 60})
        assert status["status"] == "good"
        assert status["needs_charging"] is False
        
        # 測試低電量 (20-49)
        status = robot.get_robot_battery_status({"batteryLevel": 30})
        assert status["status"] == "low"
        assert status["needs_charging"] is False
        
        # 測試危險電量 (< 20)
        status = robot.get_robot_battery_status({"batteryLevel": 10})
        assert status["status"] == "critical"
        assert status["needs_charging"] is True
        
        # 測試無電量資料
        status = robot.get_robot_battery_status({})
        assert status["level"] == 0
        assert status["status"] == "critical"

    def test_is_robot_available(self, mock_rcs_core, sample_robot_data):
        """測試機器人可用性檢查"""
        robot = KukaRobot(mock_rcs_core)
        
        # 測試可用機器人 (狀態 3 且電量充足)
        available_data = sample_robot_data.copy()
        available_data["status"] = 3
        available_data["batteryLevel"] = 50
        
        assert robot.is_robot_available(available_data) is True
        
        # 測試不可用機器人 (狀態非 3)
        unavailable_data = sample_robot_data.copy()
        unavailable_data["status"] = 4  # 任務中
        unavailable_data["batteryLevel"] = 50
        
        assert robot.is_robot_available(unavailable_data) is False
        
        # 測試電量不足
        low_battery_data = sample_robot_data.copy()
        low_battery_data["status"] = 3
        low_battery_data["batteryLevel"] = 10  # 需要充電
        
        assert robot.is_robot_available(low_battery_data) is False

    def test_log_robot_status_change(self, mock_rcs_core, mock_logger):
        """測試機器人狀態變更日誌"""
        robot = KukaRobot(mock_rcs_core)
        
        robot.log_robot_status_change("101", 3, 4)
        
        mock_logger.info.assert_called_with("機器人 101 狀態變更: 空閒 → 任務中")

    @patch('rcs.kuka_robot.agv_crud')
    def test_get_robot_status_success(self, mock_agv_crud, mock_rcs_core, mock_db_pool, sample_agv_data):
        """測試取得機器人狀態 - 成功"""
        robot = KukaRobot(mock_rcs_core)
        
        # 設置模擬 AGV 資料
        mock_agv = Mock()
        for key, value in sample_agv_data.items():
            setattr(mock_agv, key, value)
        
        mock_agv_crud.get_by_id.return_value = mock_agv
        
        status = robot.get_robot_status("101")
        
        assert status is not None
        assert status["robotId"] == sample_agv_data["id"]
        assert status["x"] == sample_agv_data["x"]
        assert status["y"] == sample_agv_data["y"]
        assert status["battery"] == sample_agv_data["battery"]

    def test_get_robot_status_no_db_pool(self, mock_rcs_core, mock_logger):
        """測試取得機器人狀態 - 無資料庫連線"""
        mock_rcs_core.db_pool = None
        robot = KukaRobot(mock_rcs_core)
        
        status = robot.get_robot_status("101")
        
        assert status is None
        mock_logger.error.assert_called_with("資料庫連線池不可用，無法查詢機器人狀態。")

    @patch('rcs.kuka_robot.agv_crud')
    def test_get_robot_status_not_found(self, mock_agv_crud, mock_rcs_core, mock_db_pool):
        """測試取得機器人狀態 - 找不到機器人"""
        robot = KukaRobot(mock_rcs_core)
        
        mock_agv_crud.get_by_id.return_value = None
        
        status = robot.get_robot_status("999")
        
        assert status is None

    def test_on_robot_update_no_db_pool(self, mock_rcs_core, mock_logger):
        """測試機器人狀態更新 - 無資料庫連線"""
        mock_rcs_core.db_pool = None
        robot = KukaRobot(mock_rcs_core)
        
        robot.on_robot_update([{"robotId": "101"}])
        
        mock_logger.error.assert_called_with("資料庫連線池不可用，無法更新機器人狀態。")

    @patch('rcs.kuka_robot.agv_crud')
    @patch('rcs.kuka_robot.ModifyLog')
    def test_on_robot_update_success(self, mock_modify_log, mock_agv_crud, 
                                   mock_rcs_core, mock_db_session, sample_robot_data, sample_agv_data):
        """測試機器人狀態更新 - 成功"""
        robot = KukaRobot(mock_rcs_core)
        
        # 設置模擬 AGV 資料
        mock_agv = Mock()
        for key, value in sample_agv_data.items():
            setattr(mock_agv, key, value)
        
        mock_agv_crud.get_by_id.return_value = mock_agv
        
        robots = [sample_robot_data]
        robot.on_robot_update(robots)
        
        # 驗證資料庫操作
        mock_db_session.commit.assert_called()
        mock_modify_log.mark.assert_called_with(mock_db_session, "agv")

    def test_get_node_number_valid(self, mock_rcs_core):
        """測試取得節點編號 - 有效"""
        robot = KukaRobot(mock_rcs_core)
        
        robot_data = {"nodeNumber": "123"}
        node_number = robot._get_node_number(robot_data)
        assert node_number == 123

    def test_get_node_number_invalid(self, mock_rcs_core, mock_logger):
        """測試取得節點編號 - 無效"""
        robot = KukaRobot(mock_rcs_core)
        
        # 測試無效格式
        robot_data = {"nodeNumber": "invalid"}
        node_number = robot._get_node_number(robot_data)
        assert node_number is None
        mock_logger.warning.assert_called_with("無效的節點編號: invalid")
        
        # 測試空值
        robot_data = {"nodeNumber": ""}
        node_number = robot._get_node_number(robot_data)
        assert node_number is None

    @patch('rcs.kuka_robot.agv_crud')
    def test_update_agv_data(self, mock_agv_crud, mock_rcs_core, sample_robot_data, sample_agv_data):
        """測試更新 AGV 資料"""
        robot = KukaRobot(mock_rcs_core)
        
        # 設置模擬 AGV
        mock_agv = Mock()
        for key, value in sample_agv_data.items():
            setattr(mock_agv, key, value)
        
        robot._update_agv_data(mock_agv, sample_robot_data)
        
        # 驗證位置更新 (轉換後的像素座標)
        expected_x = sample_robot_data["x"] / 12.5
        expected_y = sample_robot_data["y"] / 12.5
        
        assert mock_agv.x == expected_x
        assert mock_agv.y == expected_y
        assert mock_agv.battery == sample_robot_data["batteryLevel"]
        assert mock_agv.status_id == sample_robot_data["status"]
        assert mock_agv.last_node_id == int(sample_robot_data["nodeNumber"])

    def test_process_single_robot_invalid_data(self, mock_rcs_core, mock_db_session):
        """測試處理單個機器人 - 無效資料"""
        robot = KukaRobot(mock_rcs_core)
        
        # 測試缺少 robotId
        result = robot._process_single_robot(mock_db_session, {})
        assert result is False
        
        # 測試無效的機器人資料
        invalid_data = {"robotId": "101", "x": "invalid"}
        result = robot._process_single_robot(mock_db_session, invalid_data)
        assert result is False

    @patch('rcs.kuka_robot.agv_crud')
    def test_process_single_robot_agv_not_found(self, mock_agv_crud, mock_rcs_core, 
                                              mock_db_session, sample_robot_data):
        """測試處理單個機器人 - AGV 不存在"""
        robot = KukaRobot(mock_rcs_core)
        
        mock_agv_crud.get_by_id.return_value = None
        
        result = robot._process_single_robot(mock_db_session, sample_robot_data)
        assert result is False