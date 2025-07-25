#!/usr/bin/env python3
"""
KUKA 統一配置管理系統
整合 KUKA 和 CT AGV 的配置管理功能，提供集中化的配置管理服務
"""

import json
import yaml
import os
from typing import Dict, List, Any, Optional, Union
from pathlib import Path
from datetime import datetime, timezone
from dataclasses import dataclass, asdict, field
from enum import Enum
import logging

from db_proxy.connection_pool_manager import ConnectionPoolManager


class AGVModel(Enum):
    """AGV 車型枚舉"""
    CARGO = "Cargo"
    LOADER = "Loader"
    UNLOADER = "Unloader"
    KUKA400I = "KUKA400i"


class AGVStatus(Enum):
    """AGV 狀態枚舉 (統一 KUKA 和 CT AGV)"""
    # KUKA 狀態 (基於 API 文檔)
    REMOVED = 1      # 離場
    OFFLINE = 2      # 離線
    IDLE = 3         # 空閒
    RUNNING = 4      # 任務中
    CHARGING = 5     # 充電中
    UPDATING = 6     # 更新中
    ERROR = 7        # 異常


@dataclass
class KukaAPIConfig:
    """KUKA API 配置"""
    base_url: str = "http://192.168.10.3:10870"
    username: str = "admin"
    password: str = "Admin"
    timeout: float = 30.0
    max_retries: int = 3
    retry_delay: float = 1.0
    
    # API 端點配置
    endpoints: Dict[str, str] = field(default_factory=lambda: {
        'login': '/api/login',
        'robot_query': '/api/amr/robotQuery',
        'container_query': '/api/amr/containerQuery',
        'container_query_all': '/api/amr/containerQueryAll',
        'submit_mission': '/api/amr/submitMission',
        'mission_cancel': '/api/amr/missionCancel',
        'job_query': '/api/amr/jobQuery'
    })


@dataclass
class AGVConfig:
    """AGV 配置資訊"""
    id: int
    name: str
    model: AGVModel
    description: str
    
    # 位置信息
    initial_x: float = 0.0
    initial_y: float = 0.0
    initial_heading: float = 0.0
    
    # 狀態配置
    default_status: AGVStatus = AGVStatus.IDLE
    enable: bool = True
    
    # KUKA 特定配置
    kuka_robot_id: Optional[str] = None
    kuka_robot_type: Optional[str] = None
    
    # CT AGV 特定配置
    ct_room_assignment: Optional[int] = None  # 房間指派
    ct_capabilities: List[str] = field(default_factory=list)  # 能力列表
    
    # 性能參數
    battery_threshold_low: float = 20.0
    battery_threshold_critical: float = 10.0
    max_payload: float = 50.0  # kg
    max_speed: float = 1.5     # m/s


@dataclass
class KukaFleetConfig:
    """KUKA 車隊配置"""
    map_layout_district: str = "test-test1"
    mission_types: Dict[str, str] = field(default_factory=lambda: {
        'MOVE': 'MOVE',
        'RACK_MOVE': 'RACK_MOVE'
    })
    
    # 座標轉換參數
    coordinate_transform: Dict[str, List[float]] = field(default_factory=lambda: {
        'translation': [0.0, 0.0, 0.0],  # x, y, z 偏移 (m)
        'rotation': [0.0, 0.0, 0.0],     # roll, pitch, yaw (rad)
        'scale': [1.0, 1.0, 1.0]         # x, y, z 縮放
    })
    
    # 狀態映射 (CT AGV 狀態 -> KUKA 狀態)
    status_mapping: Dict[str, int] = field(default_factory=lambda: {
        'IDLE': AGVStatus.IDLE.value,
        'MOVING': AGVStatus.RUNNING.value,
        'LOADING': AGVStatus.RUNNING.value,
        'UNLOADING': AGVStatus.RUNNING.value,
        'CHARGING': AGVStatus.CHARGING.value,
        'ERROR': AGVStatus.ERROR.value,
        'OFFLINE': AGVStatus.OFFLINE.value
    })


@dataclass
class CTFleetConfig:
    """CT 車隊配置"""
    supported_models: List[str] = field(default_factory=lambda: [
        AGVModel.CARGO.value,
        AGVModel.LOADER.value,
        AGVModel.UNLOADER.value
    ])
    
    # 房間分派規則
    room_dispatch_rules: Dict[str, Dict] = field(default_factory=lambda: {
        'cargo_rules': {
            'default_agv': 'Cargo02',
            'room_assignment': None  # 負責房外任務
        },
        'loader_rules': {
            'pattern': 'Loader{room:02d}',
            'room_based': True
        },
        'unloader_rules': {
            'pattern': 'Unloader{room:02d}',
            'room_based': True
        }
    })
    
    # 任務優先級配置
    task_priorities: Dict[str, int] = field(default_factory=lambda: {
        'emergency': 90,
        'high': 70,
        'normal': 50,
        'low': 30
    })


@dataclass
class UnifiedFleetConfig:
    """統一車隊配置"""
    kuka_api: KukaAPIConfig = field(default_factory=KukaAPIConfig)
    kuka_fleet: KukaFleetConfig = field(default_factory=KukaFleetConfig)
    ct_fleet: CTFleetConfig = field(default_factory=CTFleetConfig)
    
    # AGV 列表
    agvs: Dict[str, AGVConfig] = field(default_factory=dict)
    
    # 系統配置
    system: Dict[str, Any] = field(default_factory=lambda: {
        'update_interval': 5.0,         # 監控更新間隔 (秒)
        'heartbeat_timeout': 30.0,      # 心跳超時 (秒)
        'task_dispatch_interval': 1.0,  # 任務派發間隔 (秒)
        'log_level': 'INFO',
        'enable_monitoring': True,
        'enable_auto_recovery': True
    })


class KukaConfigManager:
    """KUKA 統一配置管理器"""
    
    def __init__(self, config_dir: str = "/app/config", rcs_core=None):
        """
        初始化配置管理器
        
        Args:
            config_dir: 配置文件目錄
            rcs_core: RCS 核心節點實例（可選）
        """
        self.config_dir = Path(config_dir)
        self.rcs_core = rcs_core
        
        # 設置日誌
        self.logger = rcs_core.get_logger() if rcs_core else self._setup_logger()
        
        # 初始化資料庫連線 (如果可用)
        self.db_pool = None
        if rcs_core and hasattr(rcs_core, 'db_pool'):
            self.db_pool = rcs_core.db_pool
        else:
            try:
                self.db_pool = ConnectionPoolManager(
                    'postgresql+psycopg2://agvc:password@192.168.100.254/agvc'
                )
            except Exception as e:
                self.logger.warning(f"無法初始化資料庫連線: {e}")
        
        # 配置文件路徑
        self.config_files = {
            'unified': self.config_dir / 'unified_fleet_config.yaml',
            'kuka_api': self.config_dir / 'kuka_api_config.yaml',
            'agv_mapping': self.config_dir / 'agv_config_mapping.yaml',
            'backup_dir': self.config_dir / 'backups'
        }
        
        # 確保目錄存在
        self.config_files['backup_dir'].mkdir(exist_ok=True)
        
        # 加載配置
        self.config = self._load_or_create_config()
        
        self.logger.info("KUKA 統一配置管理器已初始化")
    
    def _setup_logger(self) -> logging.Logger:
        """設置日誌器"""
        logger = logging.getLogger('KukaConfigManager')
        if not logger.handlers:
            handler = logging.StreamHandler()
            formatter = logging.Formatter(
                '[%(asctime)s] [%(name)s] [%(levelname)s] %(message)s'
            )
            handler.setFormatter(formatter)
            logger.addHandler(handler)
            logger.setLevel(logging.INFO)
        return logger
    
    def _load_or_create_config(self) -> UnifiedFleetConfig:
        """載入或創建配置"""
        config_file = self.config_files['unified']
        
        if config_file.exists():
            return self._load_config_from_file(config_file)
        else:
            # 創建預設配置
            config = self._create_default_config()
            self._save_config_to_file(config, config_file)
            return config
    
    def _create_default_config(self) -> UnifiedFleetConfig:
        """創建預設配置"""
        self.logger.info("創建預設配置...")
        
        # 創建基礎配置
        config = UnifiedFleetConfig()
        
        # 從資料庫載入 AGV 信息（如果可用）
        if self.db_pool:
            self._load_agvs_from_database(config)
        else:
            # 使用硬編碼的預設 AGV 配置
            self._create_default_agvs(config)
        
        return config
    
    def _load_agvs_from_database(self, config: UnifiedFleetConfig):
        """從資料庫載入 AGV 配置"""
        try:
            with self.db_pool.get_session() as session:
                from db_proxy.models import AGV
                from sqlmodel import select
                
                agvs = session.exec(select(AGV).where(AGV.enable == 1)).all()
                
                for agv in agvs:
                    agv_config = AGVConfig(
                        id=agv.id,
                        name=agv.name,
                        model=AGVModel(agv.model),
                        description=agv.description or "",
                        initial_x=float(agv.x),
                        initial_y=float(agv.y),
                        initial_heading=float(agv.heading),
                        default_status=AGVStatus(agv.status_id) if agv.status_id else AGVStatus.IDLE,
                        enable=bool(agv.enable)
                    )
                    
                    # 設置特定類型的配置
                    if agv.model == "KUKA400i":
                        agv_config.kuka_robot_id = str(agv.id)
                        agv_config.kuka_robot_type = "KMP 400i diffDrive"
                    else:
                        # CT AGV 配置
                        if agv.model == "Cargo":
                            agv_config.ct_room_assignment = None  # 房外任務
                            agv_config.ct_capabilities = ["transport", "corridor_navigation"]
                        elif agv.model in ["Loader", "Unloader"]:
                            # 從名稱提取房間號 (如 Loader02 -> 房間2)
                            try:
                                room_num = int(agv.name[-2:])
                                agv_config.ct_room_assignment = room_num
                            except:
                                agv_config.ct_room_assignment = 1
                            
                            agv_config.ct_capabilities = [
                                "material_handling",
                                f"{agv.model.lower()}_operations"
                            ]
                    
                    config.agvs[agv.name] = agv_config
                
                self.logger.info(f"從資料庫載入了 {len(config.agvs)} 台 AGV 配置")
                
        except Exception as e:
            self.logger.error(f"從資料庫載入 AGV 配置失敗: {e}")
            self._create_default_agvs(config)
    
    def _create_default_agvs(self, config: UnifiedFleetConfig):
        """創建預設 AGV 配置"""
        default_agvs = [
            # CT AGV 配置
            AGVConfig(
                id=1, name="Cargo02", model=AGVModel.CARGO,
                description="走廊AGV(暫時規劃僅負責房間2)",
                ct_room_assignment=None,
                ct_capabilities=["transport", "corridor_navigation"]
            ),
            AGVConfig(
                id=2, name="Loader02", model=AGVModel.LOADER,
                description="房間2Loader(取入口傳送箱、清洗、泡藥、放預烘)",
                ct_room_assignment=2,
                ct_capabilities=["material_handling", "loader_operations"]
            ),
            AGVConfig(
                id=3, name="Unloader02", model=AGVModel.UNLOADER,
                description="房間2Unloader(取預烘、烤箱、放出口傳送箱)",
                ct_room_assignment=2,
                ct_capabilities=["material_handling", "unloader_operations"]
            ),
            
            # KUKA AGV 配置
            AGVConfig(
                id=8506941, name="KUKA001", model=AGVModel.KUKA400I,
                description="在房間外負責料架搬運",
                initial_x=3116.0, initial_y=1852.0,
                kuka_robot_id="8506941",
                kuka_robot_type="KMP 400i diffDrive"
            ),
            AGVConfig(
                id=8506995, name="KUKA002", model=AGVModel.KUKA400I,
                description="在房間外負責料架搬運",
                initial_x=2860.0, initial_y=1680.0,
                kuka_robot_id="8506995",
                kuka_robot_type="KMP 400i diffDrive"
            ),
            AGVConfig(
                id=123, name="KUKA003", model=AGVModel.KUKA400I,
                description="(SimCar)在房間外負責料架搬運",
                kuka_robot_id="123",
                kuka_robot_type="KMP 400i diffDrive"
            )
        ]
        
        for agv in default_agvs:
            config.agvs[agv.name] = agv
        
        self.logger.info(f"創建了 {len(default_agvs)} 個預設 AGV 配置")
    
    def _load_config_from_file(self, config_file: Path) -> UnifiedFleetConfig:
        """從文件載入配置"""
        try:
            with open(config_file, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
            
            # 重建配置對象
            config = UnifiedFleetConfig()
            
            if 'kuka_api' in data:
                config.kuka_api = KukaAPIConfig(**data['kuka_api'])
            
            if 'kuka_fleet' in data:
                config.kuka_fleet = KukaFleetConfig(**data['kuka_fleet'])
            
            if 'ct_fleet' in data:
                config.ct_fleet = CTFleetConfig(**data['ct_fleet'])
            
            if 'system' in data:
                config.system.update(data['system'])
            
            if 'agvs' in data:
                for agv_name, agv_data in data['agvs'].items():
                    # 轉換 model 字串為 enum
                    if 'model' in agv_data:
                        agv_data['model'] = AGVModel(agv_data['model'])
                    if 'default_status' in agv_data:
                        agv_data['default_status'] = AGVStatus(agv_data['default_status'])
                    
                    config.agvs[agv_name] = AGVConfig(**agv_data)
            
            self.logger.info(f"從 {config_file} 載入配置成功")
            return config
            
        except Exception as e:
            self.logger.error(f"載入配置文件失敗 {config_file}: {e}")
            return self._create_default_config()
    
    def _save_config_to_file(self, config: UnifiedFleetConfig, config_file: Path):
        """儲存配置到文件"""
        try:
            # 創建備份
            if config_file.exists():
                self._create_backup(config_file)
            
            # 轉換為可序列化的字典
            config_dict = self._config_to_dict(config)
            
            with open(config_file, 'w', encoding='utf-8') as f:
                yaml.dump(config_dict, f, default_flow_style=False, 
                         allow_unicode=True, indent=2)
            
            self.logger.info(f"配置已儲存到 {config_file}")
            
        except Exception as e:
            self.logger.error(f"儲存配置文件失敗 {config_file}: {e}")
    
    def _config_to_dict(self, config: UnifiedFleetConfig) -> Dict[str, Any]:
        """將配置對象轉換為字典"""
        result = {
            'kuka_api': asdict(config.kuka_api),
            'kuka_fleet': asdict(config.kuka_fleet),
            'ct_fleet': asdict(config.ct_fleet),
            'system': config.system,
            'agvs': {}
        }
        
        # 轉換 AGV 配置
        for agv_name, agv_config in config.agvs.items():
            agv_dict = asdict(agv_config)
            # 轉換 enum 為字串
            agv_dict['model'] = agv_config.model.value
            agv_dict['default_status'] = agv_config.default_status.value
            result['agvs'][agv_name] = agv_dict
        
        return result
    
    def _create_backup(self, config_file: Path):
        """創建配置文件備份"""
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        backup_file = self.config_files['backup_dir'] / f"{config_file.stem}_{timestamp}.yaml"
        
        try:
            import shutil
            shutil.copy2(config_file, backup_file)
            self.logger.info(f"已創建配置備份: {backup_file}")
        except Exception as e:
            self.logger.warning(f"創建備份失敗: {e}")
    
    def get_agv_config(self, agv_name: str) -> Optional[AGVConfig]:
        """取得 AGV 配置"""
        return self.config.agvs.get(agv_name)
    
    def get_kuka_agvs(self) -> List[AGVConfig]:
        """取得所有 KUKA AGV 配置"""
        return [agv for agv in self.config.agvs.values() 
                if agv.model == AGVModel.KUKA400I]
    
    def get_ct_agvs(self) -> List[AGVConfig]:
        """取得所有 CT AGV 配置"""
        return [agv for agv in self.config.agvs.values() 
                if agv.model in [AGVModel.CARGO, AGVModel.LOADER, AGVModel.UNLOADER]]
    
    def get_agvs_by_room(self, room_id: int) -> List[AGVConfig]:
        """取得指定房間的 AGV 配置"""
        return [agv for agv in self.config.agvs.values() 
                if agv.ct_room_assignment == room_id]
    
    def update_agv_config(self, agv_name: str, updates: Dict[str, Any]) -> bool:
        """更新 AGV 配置"""
        try:
            if agv_name not in self.config.agvs:
                self.logger.error(f"AGV {agv_name} 不存在")
                return False
            
            agv_config = self.config.agvs[agv_name]
            
            # 更新配置
            for key, value in updates.items():
                if hasattr(agv_config, key):
                    setattr(agv_config, key, value)
                else:
                    self.logger.warning(f"未知的配置項: {key}")
            
            # 儲存配置
            self.save_config()
            
            self.logger.info(f"AGV {agv_name} 配置已更新")
            return True
            
        except Exception as e:
            self.logger.error(f"更新 AGV 配置失敗: {e}")
            return False
    
    def add_agv_config(self, agv_config: AGVConfig) -> bool:
        """新增 AGV 配置"""
        try:
            if agv_config.name in self.config.agvs:
                self.logger.warning(f"AGV {agv_config.name} 已存在，將覆蓋")
            
            self.config.agvs[agv_config.name] = agv_config
            self.save_config()
            
            self.logger.info(f"AGV {agv_config.name} 配置已新增")
            return True
            
        except Exception as e:
            self.logger.error(f"新增 AGV 配置失敗: {e}")
            return False
    
    def remove_agv_config(self, agv_name: str) -> bool:
        """移除 AGV 配置"""
        try:
            if agv_name not in self.config.agvs:
                self.logger.error(f"AGV {agv_name} 不存在")
                return False
            
            del self.config.agvs[agv_name]
            self.save_config()
            
            self.logger.info(f"AGV {agv_name} 配置已移除")
            return True
            
        except Exception as e:
            self.logger.error(f"移除 AGV 配置失敗: {e}")
            return False
    
    def update_kuka_api_config(self, updates: Dict[str, Any]) -> bool:
        """更新 KUKA API 配置"""
        try:
            for key, value in updates.items():
                if hasattr(self.config.kuka_api, key):
                    setattr(self.config.kuka_api, key, value)
                else:
                    self.logger.warning(f"未知的 KUKA API 配置項: {key}")
            
            self.save_config()
            self.logger.info("KUKA API 配置已更新")
            return True
            
        except Exception as e:
            self.logger.error(f"更新 KUKA API 配置失敗: {e}")
            return False
    
    def sync_with_database(self) -> bool:
        """與資料庫同步 AGV 配置"""
        if not self.db_pool:
            self.logger.warning("資料庫連線不可用，無法同步")
            return False
        
        try:
            with self.db_pool.get_session() as session:
                from db_proxy.models import AGV
                from sqlmodel import select
                
                # 查詢資料庫中的 AGV
                db_agvs = session.exec(select(AGV)).all()
                
                updated_count = 0
                for db_agv in db_agvs:
                    agv_name = db_agv.name
                    
                    if agv_name in self.config.agvs:
                        # 更新現有配置
                        agv_config = self.config.agvs[agv_name]
                        
                        # 更新基本信息
                        agv_config.id = db_agv.id
                        agv_config.description = db_agv.description or agv_config.description
                        agv_config.initial_x = float(db_agv.x)
                        agv_config.initial_y = float(db_agv.y)
                        agv_config.initial_heading = float(db_agv.heading)
                        agv_config.enable = bool(db_agv.enable)
                        
                        if db_agv.status_id:
                            agv_config.default_status = AGVStatus(db_agv.status_id)
                        
                        updated_count += 1
                    else:
                        # 新增 AGV 配置
                        new_config = AGVConfig(
                            id=db_agv.id,
                            name=db_agv.name,
                            model=AGVModel(db_agv.model),
                            description=db_agv.description or "",
                            initial_x=float(db_agv.x),
                            initial_y=float(db_agv.y),
                            initial_heading=float(db_agv.heading),
                            enable=bool(db_agv.enable)
                        )
                        
                        if db_agv.status_id:
                            new_config.default_status = AGVStatus(db_agv.status_id)
                        
                        # 設置特定類型配置
                        if db_agv.model == "KUKA400i":
                            new_config.kuka_robot_id = str(db_agv.id)
                            new_config.kuka_robot_type = "KMP 400i diffDrive"
                        
                        self.config.agvs[agv_name] = new_config
                        updated_count += 1
                
                # 儲存更新後的配置
                self.save_config()
                
                self.logger.info(f"已與資料庫同步，更新了 {updated_count} 個 AGV 配置")
                return True
                
        except Exception as e:
            self.logger.error(f"與資料庫同步失敗: {e}")
            return False
    
    def validate_config(self) -> Dict[str, List[str]]:
        """驗證配置"""
        errors = {'agvs': [], 'kuka_api': [], 'system': []}
        
        # 驗證 AGV 配置
        agv_ids = set()
        agv_names = set()
        
        for agv_name, agv_config in self.config.agvs.items():
            # 檢查重複 ID
            if agv_config.id in agv_ids:
                errors['agvs'].append(f"重複的 AGV ID: {agv_config.id}")
            agv_ids.add(agv_config.id)
            
            # 檢查重複名稱
            if agv_config.name in agv_names:
                errors['agvs'].append(f"重複的 AGV 名稱: {agv_config.name}")
            agv_names.add(agv_config.name)
            
            # 檢查 KUKA AGV 的必要配置
            if agv_config.model == AGVModel.KUKA400I:
                if not agv_config.kuka_robot_id:
                    errors['agvs'].append(f"KUKA AGV {agv_name} 缺少 robot_id")
                if not agv_config.kuka_robot_type:
                    errors['agvs'].append(f"KUKA AGV {agv_name} 缺少 robot_type")
        
        # 驗證 KUKA API 配置
        if not self.config.kuka_api.base_url:
            errors['kuka_api'].append("缺少 base_url")
        if not self.config.kuka_api.username:
            errors['kuka_api'].append("缺少 username")
        if not self.config.kuka_api.password:
            errors['kuka_api'].append("缺少 password")
        
        # 驗證系統配置
        if self.config.system.get('update_interval', 0) <= 0:
            errors['system'].append("update_interval 必須大於 0")
        if self.config.system.get('heartbeat_timeout', 0) <= 0:
            errors['system'].append("heartbeat_timeout 必須大於 0")
        
        return errors
    
    def save_config(self):
        """儲存配置"""
        self._save_config_to_file(self.config, self.config_files['unified'])
    
    def export_config(self, export_path: str, format: str = 'yaml') -> bool:
        """匯出配置"""
        try:
            export_file = Path(export_path)
            
            if format.lower() == 'json':
                config_dict = self._config_to_dict(self.config)
                with open(export_file, 'w', encoding='utf-8') as f:
                    json.dump(config_dict, f, indent=2, ensure_ascii=False)
            else:  # yaml
                self._save_config_to_file(self.config, export_file)
            
            self.logger.info(f"配置已匯出到 {export_file}")
            return True
            
        except Exception as e:
            self.logger.error(f"匯出配置失敗: {e}")
            return False
    
    def get_config_summary(self) -> Dict[str, Any]:
        """取得配置摘要"""
        kuka_agvs = self.get_kuka_agvs()
        ct_agvs = self.get_ct_agvs()
        
        return {
            'total_agvs': len(self.config.agvs),
            'kuka_agvs': len(kuka_agvs),
            'ct_agvs': len(ct_agvs),
            'agv_models': {
                'KUKA400i': len([a for a in kuka_agvs]),
                'Cargo': len([a for a in ct_agvs if a.model == AGVModel.CARGO]),
                'Loader': len([a for a in ct_agvs if a.model == AGVModel.LOADER]),
                'Unloader': len([a for a in ct_agvs if a.model == AGVModel.UNLOADER])
            },
            'kuka_api_endpoint': self.config.kuka_api.base_url,
            'system_config': {
                'update_interval': self.config.system.get('update_interval'),
                'log_level': self.config.system.get('log_level'),
                'monitoring_enabled': self.config.system.get('enable_monitoring')
            },
            'config_file': str(self.config_files['unified']),
            'last_modified': datetime.now().isoformat()
        }


# 全域配置管理實例
_config_manager_instance = None

def get_config_manager(rcs_core=None) -> KukaConfigManager:
    """獲取配置管理實例（單例模式）"""
    global _config_manager_instance
    if _config_manager_instance is None:
        _config_manager_instance = KukaConfigManager(rcs_core=rcs_core)
    return _config_manager_instance


if __name__ == "__main__":
    # 測試配置管理器
    config_manager = KukaConfigManager()
    
    print("=== 配置摘要 ===")
    summary = config_manager.get_config_summary()
    for key, value in summary.items():
        print(f"{key}: {value}")
    
    print("\n=== 配置驗證 ===")
    errors = config_manager.validate_config()
    if any(errors.values()):
        for category, error_list in errors.items():
            if error_list:
                print(f"{category} 錯誤:")
                for error in error_list:
                    print(f"  - {error}")
    else:
        print("配置驗證通過 ✅")
    
    print("\n=== KUKA AGV 列表 ===")
    kuka_agvs = config_manager.get_kuka_agvs()
    for agv in kuka_agvs:
        print(f"- {agv.name} (ID: {agv.kuka_robot_id}): {agv.description}")
    
    print("\n=== CT AGV 列表 ===")
    ct_agvs = config_manager.get_ct_agvs()
    for agv in ct_agvs:
        room = f"房間{agv.ct_room_assignment}" if agv.ct_room_assignment else "房外"
        print(f"- {agv.name} ({agv.model.value}, {room}): {agv.description}")