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
        :return: True if acquisition is successful, False otherwise.
        """
        try:
            with self.db_pool.get_session() as session:
                zone = session.exec(
                    select(TrafficZone).where(TrafficZone.id == traffic_id)
                ).first()
                if zone and zone.status == "free":
                    zone.status = "controlled"
                    zone.owner_agv_id = agv_id
                    session.add(zone)
                    session.commit()
                    session.refresh(zone)
                    return True
                return False
        except Exception as e:
            self.logger.error(f"Error acquiring traffic zone: {e}")
            return False

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
                if zone and zone.status == "controlled" and zone.owner_agv_id == agv_id:
                    zone.status = "free"
                    zone.owner_agv_id = None
                    session.add(zone)
                    session.commit()
                    session.refresh(zone)
                    return True
                return False
        except Exception as e:
            self.logger.error(f"Error releasing traffic zone: {e}")
            return False

    def acquire_traffic_zone_by_name(self, traffic_name, agv_name):
        """
        Acquire control of a traffic zone by its name.
        :param traffic_name: Name of the traffic zone.
        :param agv_name: Name of the AGV requesting control.
        :return: True if acquisition is successful, False otherwise.
        """
        try:
            with self.db_pool.get_session() as session:
                zone = session.exec(
                    select(TrafficZone).where(TrafficZone.name == traffic_name)
                ).first()
                agv = session.exec(
                    select(AGV).where(AGV.name == agv_name)
                ).first()
                if zone and agv and zone.status == "free":
                    zone.status = "controlled"
                    zone.owner_agv_id = agv.id
                    session.add(zone)
                    session.commit()
                    session.refresh(zone)
                    return True
                return False
        except Exception as e:
            self.logger.error(f"Error acquiring traffic zone by name: {e}")
            return False

    def release_traffic_zone_by_name(self, traffic_name, agv_name):
        """
        Release control of a traffic zone by its name.
        :param traffic_name: Name of the traffic zone.
        :param agv_name: Name of the AGV releasing control.
        :return: True if release is successful, False otherwise.
        """
        try:
            with self.db_pool.get_session() as session:
                zone = session.exec(
                    select(TrafficZone).where(TrafficZone.name == traffic_name)
                ).first()
                agv = session.exec(
                    select(AGV).where(AGV.name == agv_name)
                ).first()
                if zone and agv and zone.status == "controlled" and zone.owner_agv_id == agv.id:
                    zone.status = "free"
                    zone.owner_agv_id = None
                    session.add(zone)
                    session.commit()
                    session.refresh(zone)
                    return True
                return False
        except Exception as e:
            self.logger.error(f"Error releasing traffic zone by name: {e}")
            return False
