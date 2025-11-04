import pytest
from unittest.mock import MagicMock
from traffic_manager.traffic_controller import TrafficController
from db_proxy.models import TrafficZone, AGV


@pytest.fixture
def mock_session():
    return MagicMock()


@pytest.fixture
def traffic_controller(mock_session):
    mock_pool = MagicMock()
    mock_pool.get_session.return_value.__enter__.return_value = mock_session
    return TrafficController(mock_pool)


def test_acquire_traffic_zone_success(traffic_controller, mock_session):
    mock_zone = MagicMock(spec=TrafficZone)
    mock_zone.status = "free"
    mock_session.exec.return_value.first.return_value = mock_zone

    result = traffic_controller.acquire_traffic_zone(1, 101)
    assert result
    assert mock_zone.status == "controlled"
    assert mock_zone.owner_agv_id == 101
    mock_session.add.assert_called_once_with(mock_zone)
    mock_session.commit.assert_called_once()
    mock_session.refresh.assert_called_once_with(mock_zone)


def test_acquire_traffic_zone_failure(traffic_controller, mock_session):
    """測試交管區被其他 AGV 佔用時拒絕請求"""
    mock_zone = MagicMock(spec=TrafficZone)
    mock_zone.status = "controlled"
    mock_zone.owner_agv_id = 102  # 被其他 AGV (102) 佔用
    mock_session.exec.return_value.first.return_value = mock_zone

    result = traffic_controller.acquire_traffic_zone(1, 101)  # AGV 101 請求
    assert result["success"] is False
    assert result["reason"] == "zone_controlled"
    assert result["owner_agv_id"] == 102
    mock_session.commit.assert_not_called()


def test_acquire_traffic_zone_idempotent(traffic_controller, mock_session):
    """測試同一 AGV 重複獲取交管區（冪等性）"""
    mock_zone = MagicMock(spec=TrafficZone)
    mock_zone.id = 1
    mock_zone.status = "controlled"
    mock_zone.owner_agv_id = 101  # 已被 AGV 101 佔用
    mock_session.exec.return_value.first.return_value = mock_zone

    # 同一 AGV (101) 重複請求
    result = traffic_controller.acquire_traffic_zone(1, 101)
    assert result["success"] is True
    assert result["message"] == "交管區已被本車持有"
    # 冪等操作不應該修改資料庫
    mock_session.commit.assert_not_called()


def test_acquire_traffic_zone_exception(traffic_controller, mock_session):
    """測試資料庫異常時的錯誤處理"""
    mock_session.exec.side_effect = Exception("Database error")
    result = traffic_controller.acquire_traffic_zone(1, 101)
    assert result["success"] is False
    assert result["reason"] == "database_error"


def test_release_traffic_zone_success(traffic_controller, mock_session):
    mock_zone = MagicMock(spec=TrafficZone)
    mock_zone.status = "controlled"
    mock_zone.owner_agv_id = 101
    mock_session.exec.return_value.first.return_value = mock_zone

    result = traffic_controller.release_traffic_zone(1, 101)
    assert result
    assert mock_zone.status == "free"
    assert mock_zone.owner_agv_id is None
    mock_session.add.assert_called_once_with(mock_zone)
    mock_session.commit.assert_called_once()
    mock_session.refresh.assert_called_once_with(mock_zone)


def test_release_traffic_zone_failure(traffic_controller, mock_session):
    mock_zone = MagicMock(spec=TrafficZone)
    mock_zone.status = "controlled"
    mock_zone.owner_agv_id = 102
    mock_session.exec.return_value.first.return_value = mock_zone

    result = traffic_controller.release_traffic_zone(1, 101)
    assert not result
    mock_session.commit.assert_not_called()


def test_release_traffic_zone_exception(traffic_controller, mock_session):
    mock_session.exec.side_effect = Exception("Database error")
    result = traffic_controller.release_traffic_zone(1, 101)
    assert not result


def test_acquire_traffic_zone_by_name_success(traffic_controller, mock_session):
    mock_zone = MagicMock(spec=TrafficZone)
    mock_zone.status = "free"
    mock_agv = MagicMock(spec=AGV)
    mock_agv.id = 101
    mock_session.exec.side_effect = [
        MagicMock(first=MagicMock(return_value=mock_zone)),
        MagicMock(first=MagicMock(return_value=mock_agv))
    ]

    result = traffic_controller.acquire_traffic_zone_by_name("Area1", "AGV1")
    assert result
    assert mock_zone.status == "controlled"
    assert mock_zone.owner_agv_id == 101
    mock_session.add.assert_called_once_with(mock_zone)
    mock_session.commit.assert_called_once()
    mock_session.refresh.assert_called_once_with(mock_zone)


def test_release_traffic_zone_by_name_success(traffic_controller, mock_session):
    mock_zone = MagicMock(spec=TrafficZone)
    mock_zone.status = "controlled"
    mock_zone.owner_agv_id = 101
    mock_agv = MagicMock(spec=AGV)
    mock_agv.id = 101
    mock_session.exec.side_effect = [
        MagicMock(first=MagicMock(return_value=mock_zone)),
        MagicMock(first=MagicMock(return_value=mock_agv))
    ]

    result = traffic_controller.release_traffic_zone_by_name("Area1", "AGV1")
    assert result
    assert mock_zone.status == "free"
    assert mock_zone.owner_agv_id is None
    mock_session.add.assert_called_once_with(mock_zone)
    mock_session.commit.assert_called_once()
    mock_session.refresh.assert_called_once_with(mock_zone)


def test_acquire_zone_disabled(traffic_controller, mock_session):
    """測試交管區 enable=False 時拒絕獲取"""
    mock_zone = MagicMock(spec=TrafficZone)
    mock_zone.id = 1
    mock_zone.enable = False  # 交管區已禁用
    mock_zone.status = "free"
    mock_session.exec.return_value.first.return_value = mock_zone

    result = traffic_controller.acquire_traffic_zone(1, 101)
    assert result["success"] is False
    assert result["reason"] == "zone_disabled"
    # 禁用的交管區不應該修改資料庫
    mock_session.commit.assert_not_called()


def test_kuka400i_shared_access(traffic_controller, mock_session):
    """測試兩個 KUKA400i AGV 可以共享交管區"""
    mock_zone = MagicMock(spec=TrafficZone)
    mock_zone.id = 1
    mock_zone.enable = True
    mock_zone.status = "controlled"
    mock_zone.owner_agv_id = 101  # 被 KUKA400i AGV 101 佔用

    # 模擬 owner AGV (KUKA400i)
    mock_owner_agv = MagicMock(spec=AGV)
    mock_owner_agv.id = 101
    mock_owner_agv.model = "KUKA400i"

    # 模擬 requester AGV (KUKA400i)
    mock_requester_agv = MagicMock(spec=AGV)
    mock_requester_agv.id = 102
    mock_requester_agv.model = "KUKA400i"

    # 設定 session.exec 返回 zone，session.get 返回對應的 AGV
    mock_session.exec.return_value.first.return_value = mock_zone
    mock_session.get.side_effect = lambda model, agv_id: (
        mock_owner_agv if agv_id == 101 else mock_requester_agv
    )

    # AGV 102 (KUKA400i) 請求交管區
    result = traffic_controller.acquire_traffic_zone(1, 102)
    assert result["success"] is True
    assert result["message"] == "KUKA400i共享通行"
    assert result["owner_agv_id"] == 101  # owner 不變
    # KUKA400i 共享不修改資料庫
    mock_session.commit.assert_not_called()


def test_kuka400i_mixed_access_denied(traffic_controller, mock_session):
    """測試 KUKA400i 和其他型號 AGV 不能共享交管區"""
    mock_zone = MagicMock(spec=TrafficZone)
    mock_zone.id = 1
    mock_zone.enable = True
    mock_zone.status = "controlled"
    mock_zone.owner_agv_id = 101  # 被 Cargo AGV 101 佔用

    # 模擬 owner AGV (Cargo)
    mock_owner_agv = MagicMock(spec=AGV)
    mock_owner_agv.id = 101
    mock_owner_agv.model = "Cargo"

    # 模擬 requester AGV (KUKA400i)
    mock_requester_agv = MagicMock(spec=AGV)
    mock_requester_agv.id = 102
    mock_requester_agv.model = "KUKA400i"

    # 設定 mock 返回值
    mock_session.exec.return_value.first.return_value = mock_zone
    mock_session.get.side_effect = lambda model, agv_id: (
        mock_owner_agv if agv_id == 101 else mock_requester_agv
    )

    # AGV 102 (KUKA400i) 請求交管區，應該被拒絕
    result = traffic_controller.acquire_traffic_zone(1, 102)
    assert result["success"] is False
    assert result["reason"] == "zone_controlled"
    assert result["owner_agv_id"] == 101
    mock_session.commit.assert_not_called()


def test_non_kuka_cannot_share(traffic_controller, mock_session):
    """測試非 KUKA400i 型號之間不能共享交管區"""
    mock_zone = MagicMock(spec=TrafficZone)
    mock_zone.id = 1
    mock_zone.enable = True
    mock_zone.status = "controlled"
    mock_zone.owner_agv_id = 101  # 被 Cargo AGV 101 佔用

    # 模擬 owner AGV (Cargo)
    mock_owner_agv = MagicMock(spec=AGV)
    mock_owner_agv.id = 101
    mock_owner_agv.model = "Cargo"

    # 模擬 requester AGV (Loader)
    mock_requester_agv = MagicMock(spec=AGV)
    mock_requester_agv.id = 102
    mock_requester_agv.model = "Loader"

    # 設定 mock 返回值
    mock_session.exec.return_value.first.return_value = mock_zone
    mock_session.get.side_effect = lambda model, agv_id: (
        mock_owner_agv if agv_id == 101 else mock_requester_agv
    )

    # AGV 102 (Loader) 請求交管區，應該被拒絕
    result = traffic_controller.acquire_traffic_zone(1, 102)
    assert result["success"] is False
    assert result["reason"] == "zone_controlled"
    assert result["owner_agv_id"] == 101
    mock_session.commit.assert_not_called()


def test_kuka400i_shared_release_allowed(traffic_controller, mock_session):
    """測試 KUKA400i 共享者可以釋放（返回成功但數據不變）"""
    # 創建交管區 mock（不使用 spec）
    mock_zone = MagicMock()
    mock_zone.id = 1
    mock_zone.enable = True
    mock_zone.status = "controlled"
    mock_zone.owner_agv_id = 101

    # 創建 AGV mocks（不使用 spec）
    mock_owner_agv = MagicMock()
    mock_owner_agv.id = 101
    mock_owner_agv.model = "KUKA400i"

    mock_requester_agv = MagicMock()
    mock_requester_agv.id = 102
    mock_requester_agv.model = "KUKA400i"

    # 設定 mock 返回值（不調用 reset_mock，避免清除其他測試的設置）
    mock_session.exec.return_value.first.return_value = mock_zone

    # 使用列表來追踪調用並返回對應的 mock
    get_calls = []
    def mock_get_func(model_class, obj_id):
        get_calls.append((model_class, obj_id))
        if obj_id == 101:
            return mock_owner_agv
        elif obj_id == 102:
            return mock_requester_agv
        return None

    mock_session.get = MagicMock(side_effect=mock_get_func)

    # AGV 102 (KUKA400i 共享者) 請求釋放交管區
    result = traffic_controller.release_traffic_zone(1, 102)

    # 打印調試信息
    print(f"\nDebug: get_calls = {get_calls}")
    print(f"Debug: result = {result}")
    print(f"Debug: mock_session.get.called = {mock_session.get.called}")

    assert result is True  # 返回成功
    # KUKA400i 共享者釋放不應該修改數據庫
    mock_session.commit.assert_not_called()
    # 交管區狀態應該保持不變
    assert mock_zone.status == "controlled"
    assert mock_zone.owner_agv_id == 101


def test_owner_release_actually_frees(traffic_controller, mock_session):
    """測試擁有者釋放真正清空交管區"""
    mock_zone = MagicMock(spec=TrafficZone)
    mock_zone.id = 1
    mock_zone.enable = True
    mock_zone.status = "controlled"
    mock_zone.owner_agv_id = 101  # 被 AGV 101 佔用

    # 設定 mock 返回值
    mock_session.exec.return_value.first.return_value = mock_zone

    # AGV 101 (擁有者) 請求釋放交管區
    result = traffic_controller.release_traffic_zone(1, 101)
    assert result is True  # 返回成功
    # 擁有者釋放應該真正修改數據庫
    assert mock_zone.status == "free"
    assert mock_zone.owner_agv_id is None
    mock_session.add.assert_called_once_with(mock_zone)
    mock_session.commit.assert_called_once()
    mock_session.refresh.assert_called_once_with(mock_zone)


def test_non_owner_non_kuka_cannot_release(traffic_controller, mock_session):
    """測試非擁有者非 KUKA400i 無法釋放交管區"""
    mock_zone = MagicMock(spec=TrafficZone)
    mock_zone.id = 1
    mock_zone.enable = True
    mock_zone.status = "controlled"
    mock_zone.owner_agv_id = 101  # 被 Cargo AGV 101 佔用

    # 模擬 owner AGV (Cargo)
    mock_owner_agv = MagicMock(spec=AGV)
    mock_owner_agv.id = 101
    mock_owner_agv.model = "Cargo"

    # 模擬 requester AGV (Loader)
    mock_requester_agv = MagicMock(spec=AGV)
    mock_requester_agv.id = 102
    mock_requester_agv.model = "Loader"

    # 設定 mock 返回值
    mock_session.exec.return_value.first.return_value = mock_zone
    # 使用 side_effect 來正確模擬 session.get 行為
    def mock_get_func(model_class, obj_id):
        if obj_id == 101:
            return mock_owner_agv
        elif obj_id == 102:
            return mock_requester_agv
        return None
    mock_session.get = MagicMock(side_effect=mock_get_func)

    # AGV 102 (Loader) 嘗試釋放交管區，應該被拒絕
    result = traffic_controller.release_traffic_zone(1, 102)
    assert result is False  # 返回失敗
    # 不應該修改數據庫
    mock_session.commit.assert_not_called()
    # 交管區狀態應該保持不變
    assert mock_zone.status == "controlled"
    assert mock_zone.owner_agv_id == 101
