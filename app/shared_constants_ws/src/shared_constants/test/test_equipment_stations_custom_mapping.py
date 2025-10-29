"""
Unit tests for EquipmentStations UnloaderAGV custom mapping

This test suite verifies the custom Station-Port mapping for UnloaderAGV,
which differs from the standard LoaderAGV mapping:

UnloaderAGV Custom Mapping:
- Pre-dryer (205): Station 1→[1,2,5,6], Station 3→[3,4,7,8] (批量4格)
- Oven (206): Station 1→[1,2,3,4], Station 5→[5,6,7,8] (批量4格)

LoaderAGV Standard Mapping:
- Pre-dryer (205): Station N→[N, N+1] (標準2格)
- Oven (206): Station N→[N, N+1] (標準2格)
"""

import pytest
from shared_constants.equipment_stations import EquipmentStations


class TestUnloaderPreDryerMapping:
    """測試 UnloaderAGV 預烘機自定義映射"""

    def test_unloader_pre_dryer_station_1(self):
        """UnloaderAGV 預烘機 Station 1 → Ports [1, 2, 5, 6]"""
        ports = EquipmentStations.station_to_ports(205, 1, agv_type="unloader")
        assert ports == [1, 2, 5, 6], "UnloaderAGV 預烘機 Station 1 應映射到 [1, 2, 5, 6]"

    def test_unloader_pre_dryer_station_3(self):
        """UnloaderAGV 預烘機 Station 3 → Ports [3, 4, 7, 8]"""
        ports = EquipmentStations.station_to_ports(205, 3, agv_type="unloader")
        assert ports == [3, 4, 7, 8], "UnloaderAGV 預烘機 Station 3 應映射到 [3, 4, 7, 8]"


class TestUnloaderBoxOutMapping:
    """測試 UnloaderAGV 出口傳送箱自定義映射"""

    def test_unloader_boxout_station_1(self):
        """UnloaderAGV 出口傳送箱 Station 1 → Ports [1, 2, 3, 4]"""
        ports = EquipmentStations.station_to_ports(202, 1, agv_type="unloader")
        assert ports == [1, 2, 3, 4], "UnloaderAGV 出口傳送箱 Station 1 應映射到 [1, 2, 3, 4]"


class TestUnloaderOvenMapping:
    """測試 UnloaderAGV 烤箱自定義映射"""

    def test_unloader_oven_station_1(self):
        """UnloaderAGV 烤箱 Station 1 → Ports [1, 2, 3, 4]"""
        ports = EquipmentStations.station_to_ports(206, 1, agv_type="unloader")
        assert ports == [1, 2, 3, 4], "UnloaderAGV 烤箱 Station 1 應映射到 [1, 2, 3, 4]"

    def test_unloader_oven_station_5(self):
        """UnloaderAGV 烤箱 Station 5 → Ports [5, 6, 7, 8]"""
        ports = EquipmentStations.station_to_ports(206, 5, agv_type="unloader")
        assert ports == [5, 6, 7, 8], "UnloaderAGV 烤箱 Station 5 應映射到 [5, 6, 7, 8]"


class TestLoaderStandardMapping:
    """測試 LoaderAGV 保持標準映射（不受 UnloaderAGV 自定義映射影響）"""

    def test_loader_pre_dryer_station_1(self):
        """LoaderAGV 預烘機 Station 1 → Ports [1, 2] (標準 1:2 映射)"""
        ports = EquipmentStations.station_to_ports(205, 1)  # 預設 agv_type="loader"
        assert ports == [1, 2], "LoaderAGV 預烘機 Station 1 應使用標準映射 [1, 2]"

    def test_loader_pre_dryer_station_3(self):
        """LoaderAGV 預烘機 Station 3 → Ports [3, 4] (標準 1:2 映射)"""
        ports = EquipmentStations.station_to_ports(205, 3, agv_type="loader")
        assert ports == [3, 4], "LoaderAGV 預烘機 Station 3 應使用標準映射 [3, 4]"

    def test_loader_oven_station_1(self):
        """LoaderAGV 烤箱 Station 1 → Ports [1, 2] (標準 1:2 映射)"""
        ports = EquipmentStations.station_to_ports(206, 1)  # 預設 agv_type="loader"
        assert ports == [1, 2], "LoaderAGV 烤箱 Station 1 應使用標準映射 [1, 2]"

    def test_loader_oven_station_5(self):
        """LoaderAGV 烤箱 Station 5 → Ports [5, 6] (標準 1:2 映射)"""
        ports = EquipmentStations.station_to_ports(206, 5, agv_type="loader")
        assert ports == [5, 6], "LoaderAGV 烤箱 Station 5 應使用標準映射 [5, 6]"


class TestWorkIdIntegration:
    """測試 Work ID 整合（從 Work ID 解析到 Ports）"""

    def test_unloader_pre_dryer_work_id_2050101(self):
        """Work ID 2050101 (UnloaderAGV取預烘Station01) → Ports [1, 2, 5, 6]"""
        room_id, eqp_id, station, action_type = \
            EquipmentStations.extract_station_from_work_id(2050101)

        assert room_id == 2, "room_id 應為 2"
        assert eqp_id == 205, "eqp_id 應為 205 (預烘機)"
        assert station == 1, "station 應為 1"
        assert action_type == 1, "action_type 應為 1 (TAKE)"

        # UnloaderAGV 應使用自定義映射
        ports = EquipmentStations.station_to_ports(eqp_id, station, agv_type="unloader")
        assert ports == [1, 2, 5, 6], "UnloaderAGV Work ID 2050101 應映射到 [1, 2, 5, 6]"

    def test_unloader_pre_dryer_work_id_2050301(self):
        """Work ID 2050301 (UnloaderAGV取預烘Station03) → Ports [3, 4, 7, 8]"""
        room_id, eqp_id, station, action_type = \
            EquipmentStations.extract_station_from_work_id(2050301)

        assert room_id == 2, "room_id 應為 2"
        assert eqp_id == 205, "eqp_id 應為 205 (預烘機)"
        assert station == 3, "station 應為 3"
        assert action_type == 1, "action_type 應為 1 (TAKE)"

        ports = EquipmentStations.station_to_ports(eqp_id, station, agv_type="unloader")
        assert ports == [3, 4, 7, 8], "UnloaderAGV Work ID 2050301 應映射到 [3, 4, 7, 8]"

    def test_unloader_oven_work_id_2060101(self):
        """Work ID 2060101 (UnloaderAGV取烤箱Station01) → Ports [1, 2, 3, 4]"""
        room_id, eqp_id, station, action_type = \
            EquipmentStations.extract_station_from_work_id(2060101)

        assert room_id == 2, "room_id 應為 2"
        assert eqp_id == 206, "eqp_id 應為 206 (烤箱)"
        assert station == 1, "station 應為 1"
        assert action_type == 1, "action_type 應為 1 (TAKE)"

        ports = EquipmentStations.station_to_ports(eqp_id, station, agv_type="unloader")
        assert ports == [1, 2, 3, 4], "UnloaderAGV Work ID 2060101 應映射到 [1, 2, 3, 4]"

    def test_unloader_oven_work_id_2060502(self):
        """Work ID 2060502 (UnloaderAGV放烤箱Station05) → Ports [5, 6, 7, 8]"""
        room_id, eqp_id, station, action_type = \
            EquipmentStations.extract_station_from_work_id(2060502)

        assert room_id == 2, "room_id 應為 2"
        assert eqp_id == 206, "eqp_id 應為 206 (烤箱)"
        assert station == 5, "station 應為 5"
        assert action_type == 2, "action_type 應為 2 (PUT)"

        ports = EquipmentStations.station_to_ports(eqp_id, station, agv_type="unloader")
        assert ports == [5, 6, 7, 8], "UnloaderAGV Work ID 2060502 應映射到 [5, 6, 7, 8]"

    def test_unloader_boxout_work_id_2020102(self):
        """Work ID 2020102 (UnloaderAGV放出口傳送箱Station01) → Ports [1, 2, 3, 4]"""
        room_id, eqp_id, station, action_type = \
            EquipmentStations.extract_station_from_work_id(2020102)

        assert room_id == 2, "room_id 應為 2"
        assert eqp_id == 202, "eqp_id 應為 202 (出口傳送箱)"
        assert station == 1, "station 應為 1"
        assert action_type == 2, "action_type 應為 2 (PUT)"

        ports = EquipmentStations.station_to_ports(eqp_id, station, agv_type="unloader")
        assert ports == [1, 2, 3, 4], "UnloaderAGV Work ID 2020102 應映射到 [1, 2, 3, 4]"


class TestFallbackBehavior:
    """測試回退邏輯（當 UnloaderAGV 的 station 不在自定義映射中時）"""

    def test_unloader_soaker_fallback_to_special(self):
        """UnloaderAGV 泡藥機 (204) 應回退到特殊設備邏輯 (1:1 映射)"""
        ports = EquipmentStations.station_to_ports(204, 3, agv_type="unloader")
        assert ports == [3], "UnloaderAGV 泡藥機應回退到特殊設備映射 [3]"

    def test_unloader_cleaner_fallback_to_standard(self):
        """UnloaderAGV 清洗機 (203) 應回退到標準設備邏輯 (1:2 映射)"""
        ports = EquipmentStations.station_to_ports(203, 1, agv_type="unloader")
        assert ports == [1, 2], "UnloaderAGV 清洗機應回退到標準設備映射 [1, 2]"


class TestErrorHandling:
    """測試錯誤處理"""

    def test_invalid_station_for_standard_equipment(self):
        """測試標準設備的無效 station (必須是奇數)"""
        with pytest.raises(ValueError, match="odd station number"):
            EquipmentStations.station_to_ports(206, 2, agv_type="loader")

    def test_invalid_station_not_in_config(self):
        """測試 station 不在設備配置中"""
        with pytest.raises(ValueError, match="Invalid station"):
            EquipmentStations.station_to_ports(206, 9, agv_type="loader")

    def test_unknown_equipment_id(self):
        """測試未知的設備 ID"""
        with pytest.raises(ValueError, match="Unknown equipment ID"):
            EquipmentStations.station_to_ports(999, 1, agv_type="unloader")


class TestEquipmentNames:
    """測試設備名稱查詢"""

    def test_get_equipment_name_chinese(self):
        """測試取得中文設備名稱"""
        assert EquipmentStations.get_equipment_name(205, 'zh') == "預烘機"
        assert EquipmentStations.get_equipment_name(206, 'zh') == "烤箱"

    def test_get_equipment_name_english(self):
        """測試取得英文設備名稱"""
        assert EquipmentStations.get_equipment_name(205, 'en') == "Pre-dryer"
        assert EquipmentStations.get_equipment_name(206, 'en') == "Oven"


class TestWorkIdToPortsConvenience:
    """測試 work_id_to_ports 便利方法的 agv_type 參數支援"""

    def test_loader_work_id_to_ports_pre_dryer(self):
        """LoaderAGV 使用 work_id_to_ports - 預烘機 Station 1"""
        room_id, eqp_id, ports, action_type = \
            EquipmentStations.work_id_to_ports(2050102)  # 預設 agv_type="loader"

        assert room_id == 2, "room_id 應為 2"
        assert eqp_id == 205, "eqp_id 應為 205 (預烘機)"
        assert ports == [1, 2], "LoaderAGV 預烘機 Station 1 應返回 [1, 2]"
        assert action_type == 2, "action_type 應為 2 (PUT)"

    def test_loader_work_id_to_ports_oven(self):
        """LoaderAGV 使用 work_id_to_ports - 烤箱 Station 5"""
        room_id, eqp_id, ports, action_type = \
            EquipmentStations.work_id_to_ports(2060502, agv_type="loader")

        assert room_id == 2, "room_id 應為 2"
        assert eqp_id == 206, "eqp_id 應為 206 (烤箱)"
        assert ports == [5, 6], "LoaderAGV 烤箱 Station 5 應返回 [5, 6]"
        assert action_type == 2, "action_type 應為 2 (PUT)"

    def test_unloader_work_id_to_ports_pre_dryer(self):
        """UnloaderAGV 使用 work_id_to_ports - 預烘機 Station 1"""
        room_id, eqp_id, ports, action_type = \
            EquipmentStations.work_id_to_ports(2050101, agv_type="unloader")

        assert room_id == 2, "room_id 應為 2"
        assert eqp_id == 205, "eqp_id 應為 205 (預烘機)"
        assert ports == [1, 2, 5, 6], "UnloaderAGV 預烘機 Station 1 應返回 [1, 2, 5, 6]"
        assert action_type == 1, "action_type 應為 1 (TAKE)"

    def test_unloader_work_id_to_ports_oven(self):
        """UnloaderAGV 使用 work_id_to_ports - 烤箱 Station 1"""
        room_id, eqp_id, ports, action_type = \
            EquipmentStations.work_id_to_ports(2060101, agv_type="unloader")

        assert room_id == 2, "room_id 應為 2"
        assert eqp_id == 206, "eqp_id 應為 206 (烤箱)"
        assert ports == [1, 2, 3, 4], "UnloaderAGV 烤箱 Station 1 應返回 [1, 2, 3, 4]"
        assert action_type == 1, "action_type 應為 1 (TAKE)"


if __name__ == "__main__":
    # 允許直接執行測試
    pytest.main([__file__, "-v", "--tb=short"])
