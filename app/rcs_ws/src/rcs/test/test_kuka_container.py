"""
KukaContainer 單元測試
測試 KUKA 容器管理模組的所有功能
"""
import pytest
from unittest.mock import Mock, MagicMock, patch

# Import the class under test
from rcs.kuka_container import KukaContainer


class TestKukaContainer:
    """KukaContainer 測試類別"""

    def test_init(self, mock_rcs_core):
        """測試 KukaContainer 初始化"""
        container = KukaContainer(mock_rcs_core)
        
        assert container.rcs_core == mock_rcs_core
        assert container.logger == mock_rcs_core.get_logger.return_value

    def test_rack_to_dict(self, mock_rcs_core, sample_rack_data):
        """測試 Rack 物件轉換為字典"""
        container = KukaContainer(mock_rcs_core)
        
        # 創建模擬 Rack 物件
        mock_rack = Mock()
        for key, value in sample_rack_data.items():
            setattr(mock_rack, key, value)
        
        result = container._rack_to_dict(mock_rack)
        
        assert result["id"] == sample_rack_data["id"]
        assert result["name"] == sample_rack_data["name"]
        assert result["agv_id"] == sample_rack_data["agv_id"]
        assert result["location_id"] == sample_rack_data["location_id"]
        assert result["is_carry"] == sample_rack_data["is_carry"]
        assert result["is_in_map"] == sample_rack_data["is_in_map"]

    def test_validate_container_data_success(self, mock_rcs_core, sample_container_data):
        """測試容器資料驗證成功"""
        container = KukaContainer(mock_rcs_core)
        
        assert container._validate_container_data(sample_container_data) is True

    def test_validate_container_data_empty(self, mock_rcs_core, mock_logger):
        """測試容器資料驗證 - 空資料"""
        container = KukaContainer(mock_rcs_core)
        
        assert container._validate_container_data(None) is False
        mock_logger.warning.assert_called_with("容器資料為空")
        
        assert container._validate_container_data({}) is False

    def test_validate_container_data_missing_field(self, mock_rcs_core, mock_logger):
        """測試容器資料驗證 - 缺少必要欄位"""
        container = KukaContainer(mock_rcs_core)
        
        invalid_data = {"isCarry": True}  # 缺少 containerCode
        assert container._validate_container_data(invalid_data) is False
        mock_logger.warning.assert_called_with("容器資料缺少必要欄位: containerCode")

    def test_get_rack_status_no_db_pool(self, mock_rcs_core, mock_logger):
        """測試取得 Rack 狀態 - 無資料庫連線"""
        mock_rcs_core.db_pool = None
        container = KukaContainer(mock_rcs_core)
        
        result = container.get_rack_status("RACK001")
        
        assert result is None
        mock_logger.error.assert_called_with("資料庫連線池不可用，無法查詢 Rack 狀態。")

    def test_get_rack_status_success(self, mock_rcs_core, mock_db_pool, mock_db_session, sample_rack_data):
        """測試取得 Rack 狀態 - 成功"""
        container = KukaContainer(mock_rcs_core)
        
        # 設置模擬 Rack
        mock_rack = Mock()
        for key, value in sample_rack_data.items():
            setattr(mock_rack, key, value)
        
        mock_db_session.exec.return_value.first.return_value = mock_rack
        
        result = container.get_rack_status("RACK001")
        
        assert result is not None
        assert result["id"] == sample_rack_data["id"]
        assert result["name"] == sample_rack_data["name"]

    def test_get_rack_status_not_found(self, mock_rcs_core, mock_db_pool, mock_db_session):
        """測試取得 Rack 狀態 - 找不到"""
        container = KukaContainer(mock_rcs_core)
        
        mock_db_session.exec.return_value.first.return_value = None
        
        result = container.get_rack_status("NONEXISTENT")
        
        assert result is None

    def test_get_rack_by_id_success(self, mock_rcs_core, mock_db_pool, mock_db_session, sample_rack_data):
        """測試根據 ID 取得 Rack - 成功"""
        container = KukaContainer(mock_rcs_core)
        
        # 設置模擬 Rack
        mock_rack = Mock()
        for key, value in sample_rack_data.items():
            setattr(mock_rack, key, value)
        
        mock_db_session.exec.return_value.first.return_value = mock_rack
        
        result = container.get_rack_by_id(1)
        
        assert result is not None
        assert result["id"] == sample_rack_data["id"]

    def test_get_carrying_racks(self, mock_rcs_core, mock_db_pool, mock_db_session, sample_rack_data):
        """測試取得搬運中的 Rack"""
        container = KukaContainer(mock_rcs_core)
        
        # 設置搬運中的 Rack
        carrying_rack_data = sample_rack_data.copy()
        carrying_rack_data["is_carry"] = 1
        
        mock_rack = Mock()
        for key, value in carrying_rack_data.items():
            setattr(mock_rack, key, value)
        
        mock_db_session.exec.return_value.all.return_value = [mock_rack]
        
        result = container.get_carrying_racks()
        
        assert len(result) == 1
        assert result[0]["id"] == carrying_rack_data["id"]
        assert result[0]["is_carry"] == 1

    def test_get_in_map_racks(self, mock_rcs_core, mock_db_pool, mock_db_session, sample_rack_data):
        """測試取得地圖中的 Rack"""
        container = KukaContainer(mock_rcs_core)
        
        # 設置地圖中的 Rack
        in_map_rack_data = sample_rack_data.copy()
        in_map_rack_data["is_in_map"] = 1
        
        mock_rack = Mock()
        for key, value in in_map_rack_data.items():
            setattr(mock_rack, key, value)
        
        mock_db_session.exec.return_value.all.return_value = [mock_rack]
        
        result = container.get_in_map_racks()
        
        assert len(result) == 1
        assert result[0]["id"] == in_map_rack_data["id"]
        assert result[0]["is_in_map"] == 1

    def test_get_racks_by_ids_empty_list(self, mock_rcs_core):
        """測試批量取得 Rack - 空列表"""
        container = KukaContainer(mock_rcs_core)
        
        result = container.get_racks_by_ids([])
        
        assert result == []

    def test_get_racks_by_ids_success(self, mock_rcs_core, mock_db_pool, mock_db_session, sample_rack_data):
        """測試批量取得 Rack - 成功"""
        container = KukaContainer(mock_rcs_core)
        
        mock_rack = Mock()
        for key, value in sample_rack_data.items():
            setattr(mock_rack, key, value)
        
        mock_db_session.exec.return_value.all.return_value = [mock_rack]
        
        result = container.get_racks_by_ids([1, 2, 3])
        
        assert len(result) == 1
        assert result[0]["id"] == sample_rack_data["id"]

    def test_get_all_racks(self, mock_rcs_core, mock_db_pool, mock_db_session, sample_rack_data):
        """測試取得所有 Rack"""
        container = KukaContainer(mock_rcs_core)
        
        mock_rack = Mock()
        for key, value in sample_rack_data.items():
            setattr(mock_rack, key, value)
        
        mock_db_session.exec.return_value.all.return_value = [mock_rack]
        
        result = container.get_all_racks()
        
        assert len(result) == 1
        assert result[0]["id"] == sample_rack_data["id"]

    def test_on_container_update_empty_list(self, mock_rcs_core):
        """測試容器狀態更新 - 空列表"""
        container = KukaContainer(mock_rcs_core)
        
        # 不應該拋出異常
        container.on_container_update([])

    @patch('rcs.kuka_container.ModifyLog')
    def test_update_container_database_success(self, mock_modify_log, mock_rcs_core, 
                                             mock_db_pool, mock_db_session, sample_container_data, sample_rack_data):
        """測試更新容器資料庫 - 成功"""
        container = KukaContainer(mock_rcs_core)
        
        # 設置模擬 Rack
        mock_rack = Mock()
        for key, value in sample_rack_data.items():
            setattr(mock_rack, key, value)
        
        mock_db_session.exec.return_value.first.return_value = mock_rack
        
        containers = [sample_container_data]
        container.update_container_database(containers)
        
        # 驗證資料庫操作
        mock_db_session.commit.assert_called()
        mock_modify_log.mark.assert_called_with(mock_db_session, "rack")

    def test_update_container_database_no_db_pool(self, mock_rcs_core, mock_logger):
        """測試更新容器資料庫 - 無資料庫連線"""
        mock_rcs_core.db_pool = None
        container = KukaContainer(mock_rcs_core)
        
        container.update_container_database([{"containerCode": "RACK001"}])
        
        mock_logger.error.assert_called_with("資料庫連線池不可用，無法更新容器狀態。")

    def test_get_rack_by_name_success(self, mock_rcs_core, mock_db_session, sample_rack_data):
        """測試根據名稱查找 Rack - 成功"""
        container = KukaContainer(mock_rcs_core)
        
        mock_rack = Mock()
        for key, value in sample_rack_data.items():
            setattr(mock_rack, key, value)
        
        mock_db_session.exec.return_value.first.return_value = mock_rack
        
        result = container._get_rack_by_name(mock_db_session, "RACK001")
        
        assert result is not None

    def test_get_rack_by_name_exception(self, mock_rcs_core, mock_db_session, mock_logger):
        """測試根據名稱查找 Rack - 異常"""
        container = KukaContainer(mock_rcs_core)
        
        mock_db_session.exec.side_effect = Exception("Database error")
        
        result = container._get_rack_by_name(mock_db_session, "RACK001")
        
        assert result is None
        mock_logger.error.assert_called()

    def test_update_rack_data(self, mock_rcs_core, mock_logger, sample_rack_data, sample_container_data):
        """測試更新 Rack 資料"""
        container = KukaContainer(mock_rcs_core)
        
        # 設置模擬 Rack
        mock_rack = Mock()
        for key, value in sample_rack_data.items():
            setattr(mock_rack, key, value)
        
        container._update_rack_data(mock_rack, sample_container_data)
        
        # 驗證資料更新
        assert mock_rack.is_carry == 1  # isCarry: True -> 1
        assert mock_rack.is_in_map == 1  # inMapStatus: True -> 1
        
        mock_logger.debug.assert_called()

    def test_update_rack_data_false_values(self, mock_rcs_core, sample_rack_data):
        """測試更新 Rack 資料 - False 值"""
        container = KukaContainer(mock_rcs_core)
        
        mock_rack = Mock()
        for key, value in sample_rack_data.items():
            setattr(mock_rack, key, value)
        
        container_data = {
            "containerCode": "RACK001",
            "isCarry": False,
            "inMapStatus": False
        }
        
        container._update_rack_data(mock_rack, container_data)
        
        assert mock_rack.is_carry == 0
        assert mock_rack.is_in_map == 0

    def test_update_rack_data_none_values(self, mock_rcs_core, sample_rack_data):
        """測試更新 Rack 資料 - None 值"""
        container = KukaContainer(mock_rcs_core)
        
        mock_rack = Mock()
        for key, value in sample_rack_data.items():
            setattr(mock_rack, key, value)
        
        original_is_carry = mock_rack.is_carry
        original_is_in_map = mock_rack.is_in_map
        
        container_data = {
            "containerCode": "RACK001",
            "isCarry": None,
            "inMapStatus": None
        }
        
        container._update_rack_data(mock_rack, container_data)
        
        # None 值不應該更新資料
        assert mock_rack.is_carry == original_is_carry
        assert mock_rack.is_in_map == original_is_in_map

    def test_process_single_container_missing_container_code(self, mock_rcs_core, mock_db_session, mock_logger):
        """測試處理單個容器 - 缺少容器代碼"""
        container = KukaContainer(mock_rcs_core)
        
        result = container._process_single_container(mock_db_session, {})
        
        assert result is False
        mock_logger.warning.assert_called_with("收到的容器資料缺少 'containerCode'，跳過此筆更新")

    def test_process_single_container_rack_not_found(self, mock_rcs_core, mock_db_session, sample_container_data):
        """測試處理單個容器 - 找不到 Rack"""
        container = KukaContainer(mock_rcs_core)
        
        mock_db_session.exec.return_value.first.return_value = None
        
        result = container._process_single_container(mock_db_session, sample_container_data)
        
        assert result is False

    def test_process_single_container_success(self, mock_rcs_core, mock_db_session, 
                                            sample_container_data, sample_rack_data, mock_logger):
        """測試處理單個容器 - 成功"""
        container = KukaContainer(mock_rcs_core)
        
        # 設置模擬 Rack
        mock_rack = Mock()
        for key, value in sample_rack_data.items():
            setattr(mock_rack, key, value)
        
        mock_db_session.exec.return_value.first.return_value = mock_rack
        
        result = container._process_single_container(mock_db_session, sample_container_data)
        
        assert result is True
        mock_db_session.commit.assert_called()
        mock_logger.debug.assert_called_with("已成功更新 Rack RACK001 的狀態")

    def test_process_single_container_exception(self, mock_rcs_core, mock_db_session, 
                                              sample_container_data, mock_logger):
        """測試處理單個容器 - 異常"""
        container = KukaContainer(mock_rcs_core)
        
        mock_db_session.exec.side_effect = Exception("Database error")
        
        result = container._process_single_container(mock_db_session, sample_container_data)
        
        assert result is False
        mock_logger.error.assert_called()