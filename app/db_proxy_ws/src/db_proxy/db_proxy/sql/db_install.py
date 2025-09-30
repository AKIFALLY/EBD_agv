from db_proxy.connection_pool_manager import ConnectionPoolManager
# 只需匯入 SQLModel 與 model
from db_proxy.models import ProcessSettings, Product, Work, Task, TaskStatus, AGV, AGVContext, TrafficZone, Node, NodeType, Room, Machine, RackStatus, Rack, LocationStatus, Location, Carrier, CarrierStatus, Eqp, EqpPort, EqpSignal, RosoutLog, RuntimeLog, AuditLog, KukaNode, KukaEdge, License
from sqlmodel import select
from sqlalchemy import text
# from db_proxy_interfaces.srv import AcquireTrafficArea, ReleaseTrafficArea, AddTrafficArea  # Define custom services


def initialize_default_data(pool_agvc):
    """初始化預設資料，若已存在則不插入"""

    from .init_data.init_manager import initialize_all_data

    with pool_agvc.get_session() as session:
        # 使用模組化的初始化管理器
        initialize_all_data(session)

        # 重置所有序列到正確的最大值
        reset_all_sequences(session)

    print("✅ 預設資料初始化完成")


def insert_data_if_not_exists_name(session, datas, model):
    """檢查是否已存在預設資料，如果不存在則插入"""
    # Use no_autoflush to prevent foreign key constraint issues during initialization
    with session.no_autoflush:
        for data in datas:
            data_name = data.get("name")
            exists = session.exec(select(model).where(
                model.name == data_name)).first()
            if not exists:
                session.add(model(**data))

    session.commit()


def insert_data_if_not_exists_name_and_not_exists_id(session, datas, model):
    """檢查是否已存在預設資料，如果不存在則插入"""
    # Use no_autoflush to prevent foreign key constraint issues during initialization
    with session.no_autoflush:
        for data in datas:
            data_id = data.get("id")  # 安全地取得 id，如果不存在會是 None
            data_name = data.get("name")
            
            # 分別檢查 id 和 name 是否存在
            exists_by_id = False
            exists_by_name = False
            
            if data_id is not None:
                exists_by_id = session.exec(select(model).where(
                    model.id == data_id)).first() is not None
            
            if data_name is not None:
                exists_by_name = session.exec(select(model).where(
                    model.name == data_name)).first() is not None
            
            # 只有當 id 和 name 都不存在時才插入
            if not exists_by_id and not exists_by_name:
                session.add(model(**data))

    session.commit()


def reset_sequence_for_table(session, table_name: str):
    """重置單個表的序列到正確的最大值"""
    try:
        # 檢查表是否存在
        check_table_sql = text(f"""
            SELECT EXISTS (
                SELECT FROM information_schema.tables
                WHERE table_schema = 'public'
                AND table_name = '{table_name}'
            )
        """)
        table_exists = session.exec(check_table_sql).first()[0]

        if not table_exists:
            print(f"⚠️  表 {table_name} 不存在，跳過序列重置")
            return

        # 檢查序列是否存在
        sequence_name = f"{table_name}_id_seq"
        check_sequence_sql = text(f"""
            SELECT EXISTS (
                SELECT FROM information_schema.sequences
                WHERE sequence_schema = 'public'
                AND sequence_name = '{sequence_name}'
            )
        """)
        sequence_exists = session.exec(check_sequence_sql).first()[0]

        if not sequence_exists:
            print(f"⚠️  序列 {sequence_name} 不存在，跳過")
            return

        # 獲取表中的最大 ID
        max_id_sql = text(f"SELECT COALESCE(MAX(id), 0) FROM {table_name}")
        max_id = session.exec(max_id_sql).first()[0]

        # 獲取當前序列值
        current_seq_sql = text(f"SELECT last_value FROM {sequence_name}")
        current_seq = session.exec(current_seq_sql).first()[0]

        # 如果序列值小於最大 ID，則重置
        if current_seq < max_id:
            reset_sql = text(f"SELECT setval('{sequence_name}', {max_id})")
            new_value = session.exec(reset_sql).first()[0]
            print(f"✅ {table_name}: 序列從 {current_seq} 重置為 {new_value}")
        else:
            print(f"✓  {table_name}: 序列 {current_seq} 已經正確，無需重置")

    except Exception as e:
        print(f"❌ 重置 {table_name} 序列時發生錯誤: {e}")


def reset_all_sequences(session):
    """重置所有有 id 主鍵的表的序列"""
    print("\n🔄 開始重置所有資料表序列...")

    # 需要重置序列的表格列表（有 id 主鍵的表）
    tables_with_id_sequences = [
        'rack',
        'product',
        'agv',
        'location',
        'location_status',
        'rack_status',
        'node',
        'node_type',
        'machine',
        'room',
        'process_settings',
        'work',
        'task',
        'task_status',
        'carrier',
        'carrier_status',
        'eqp',
        'eqp_port',
        'eqp_signal',
        'traffic_zone',
        'agv_context',
        'audit_log',
        'kuka_node',
        'kuka_edge'
    ]

    for table_name in tables_with_id_sequences:
        reset_sequence_for_table(session, table_name)

    # 提交所有序列重置
    session.commit()
    print("✅ 序列重置完成！\n")


def main(args=None):
    """主函數"""
    print("🚀 開始資料庫初始化...")

    # 資料庫連接設定
    db_url_agvc = 'postgresql+psycopg2://agvc:password@192.168.100.254/agvc'

    try:
        # 建立連接池
        pool_agvc = ConnectionPoolManager(db_url_agvc, 1)

        # 初始化預設資料
        initialize_default_data(pool_agvc)

        # 關閉連接池
        pool_agvc.shutdown()

        print("🎉 資料庫初始化完成！")

    except Exception as e:
        print(f"❌ 資料庫初始化失敗: {e}")
        raise


if __name__ == "__main__":
    main()
