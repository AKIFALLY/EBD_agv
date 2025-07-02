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
    mock_zone = MagicMock(spec=TrafficZone)
    mock_zone.status = "controlled"
    mock_session.exec.return_value.first.return_value = mock_zone

    result = traffic_controller.acquire_traffic_zone(1, 101)
    assert not result
    mock_session.commit.assert_not_called()


def test_acquire_traffic_zone_exception(traffic_controller, mock_session):
    mock_session.exec.side_effect = Exception("Database error")
    result = traffic_controller.acquire_traffic_zone(1, 101)
    assert not result


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
