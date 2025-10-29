"""
載具狀態初始化資料
"""

from db_proxy.models import CarrierStatus
from db_proxy.sql.db_install import insert_data_if_not_exists_name_and_not_exists_id


def initialize_carrier_status(session):
    """初始化載具狀態資料"""
    print("📋 初始化載具狀態資料...")

    carrier_status_data = [
        {
            "id": 1,
            "name": "空閒",
            "description": "載具空閒，可以使用",
            "color": "is-success"
        },
        {
            "id": 2,
            "name": "使用中",
            "description": "載具正在使用中",
            "color": "is-warning"
        },
        {
            "id": 3,
            "name": "故障",
            "description": "載具發生故障",
            "color": "is-danger"
        },
        {
            "id": 4,
            "name": "待處理",
            "description": "載具等待處理",
            "color": "is-info"
        },
        {
            "id": 5,
            "name": "處理中",
            "description": "載具正在處理製程",
            "color": "is-primary"
        },
        {
            "id": 6,
            "name": "NG",
            "description": "載具處理結果不良",
            "color": "is-dark"
        },
        {
            "id": 7,
            "name": "維護中",
            "description": "載具正在維護",
            "color": "is-light"
        },
        {
            "id": 8,
            "name": "已完成",
            "description": "載具處理完成",
            "color": "is-link"
        },
        {
            "id": 101,
            "name": "準備進入入口傳送箱",
            "description": "載具準備進入入口傳送箱",
            "color": "is-primary"
        },
        {
            "id": 102,
            "name": "進入入口傳送箱完成",
            "description": "載具已進入入口傳送箱，等待 Loader AGV 取料",
            "color": "is-primary"
        },
        {
            "id": 301,
            "name": "準備進入清洗機處理中",
            "description": "載具正在處理製程",
            "color": "is-primary"
        },
        {
            "id": 302,
            "name": "進入清洗機處理中",
            "description": "載具正在處理製程",
            "color": "is-primary"
        },
        {
            "id": 303,
            "name": "清洗機處理完成",
            "description": "載具正在處理製程",
            "color": "is-primary"
        },
        {
            "id": 401,
            "name": "準備進入強化機處理中",
            "description": "載具正在處理製程",
            "color": "is-primary"
        },
        {
            "id": 402,
            "name": "進入強化機處理中",
            "description": "載具正在處理製程",
            "color": "is-primary"
        },
        {
            "id": 403,
            "name": "強化機處理完成",
            "description": "載具正在處理製程",
            "color": "is-primary"
        },
        {
            "id": 501,
            "name": "準備進入預烘機處理中",
            "description": "載具正在處理製程",
            "color": "is-primary"
        },
        {
            "id": 502,
            "name": "進入預烘機處理中",
            "description": "載具正在處理製程",
            "color": "is-primary"
        },
        {
            "id": 503,
            "name": "預烘機處理完成",
            "description": "載具正在處理製程",
            "color": "is-primary"
        },
        {
            "id": 601,
            "name": "準備進入烤箱處理中",
            "description": "載具正在處理製程",
            "color": "is-primary"
        },
        {
            "id": 602,
            "name": "進入烤箱處理中",
            "description": "載具正在處理製程",
            "color": "is-primary"
        },
        {
            "id": 603,
            "name": "烤箱處理完成",
            "description": "載具正在處理製程",
            "color": "is-primary"
        },
        {
            "id": 201,
            "name": "準備進入出口傳送箱",
            "description": "載具準備進入出口傳送箱",
            "color": "is-primary"
        },
        {
            "id": 202,
            "name": "進入出口傳送箱完成",
            "description": "載具已進入出口傳送箱，等待 Cargo AGV 裝載回 Rack",
            "color": "is-primary"
        },

    ]

    insert_data_if_not_exists_name_and_not_exists_id(
        session, carrier_status_data, CarrierStatus
    )

    print(f"   ✅ 載具狀態資料: {len(carrier_status_data)} 筆")
