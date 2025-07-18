"""
初始化資料管理器
按照相依性順序載入所有初始化資料
"""

# 按照相依性順序導入所有初始化模組
# 使用 importlib 動態導入以避免數字開頭的模組名稱問題
import importlib


def _import_module_function(module_name, function_name):
    """動態導入模組中的函數"""
    module_path = f"db_proxy.sql.init_data.{module_name}"
    module = importlib.import_module(module_path)
    return getattr(module, function_name)


# 動態導入所有初始化函數
initialize_node_types = _import_module_function(
    "01_node_types", "initialize_node_types")
initialize_location_status = _import_module_function(
    "02_location_status", "initialize_location_status")
initialize_rack_status = _import_module_function(
    "03_rack_status", "initialize_rack_status")
initialize_process_settings = _import_module_function(
    "04_process_settings", "initialize_process_settings")
initialize_nodes = _import_module_function("05_nodes", "initialize_nodes")
initialize_machines = _import_module_function(
    "06_machines", "initialize_machines")
initialize_rooms = _import_module_function("07_rooms", "initialize_rooms")
initialize_locations = _import_module_function(
    "08_locations", "initialize_locations")
initialize_products = _import_module_function(
    "09_products", "initialize_products")
initialize_agvs = _import_module_function("10_agvs", "initialize_agvs")
initialize_racks = _import_module_function("11_racks", "initialize_racks")
initialize_equipment = _import_module_function(
    "12_equipment", "initialize_equipment")
initialize_equipment_ports = _import_module_function(
    "12_equipment", "initialize_equipment_ports")
initialize_equipment_signals = _import_module_function(
    "12_equipment", "initialize_equipment_signals")
initialize_task_status = _import_module_function(
    "13_works_tasks", "initialize_task_status")
initialize_works = _import_module_function(
    "13_works_tasks", "initialize_works")
initialize_tasks = _import_module_function(
    "13_works_tasks", "initialize_tasks")
initialize_carriers = _import_module_function(
    "14_carriers", "initialize_carriers")
initialize_carrier_status = _import_module_function(
    "15_carrier_status", "initialize_carrier_status")
initialize_traffic_zones = _import_module_function(
    "16_traffic_zone", "initialize_traffic_zones")
initialize_agv_status = _import_module_function(
    "17_agv_status", "init_agv_status")
initialize_kuka_map = _import_module_function(
    "18_kuka_map", "initialize_kuka_map")
initialize_ct_map = _import_module_function(
    "19_ct_map", "initialize_ct_map")
initialize_license = _import_module_function(
    "20_license", "initialize_license")


def initialize_all_data(session):
    """
    按照相依性順序初始化所有資料

    初始化順序：
    1. 基礎類型資料（無相依性）
    2. 節點資料（依賴節點類型）
    3. 機器和房間資料（依賴節點和製程設置）
    4. 位置資料（依賴位置狀態、房間、節點）
    5. 產品資料（依賴製程設置）
    6. AGV 資料（無相依性，但需要在貨架之前）
    7. 貨架和載具資料（依賴位置、AGV、產品、貨架狀態）
    8. 設備資料（依賴位置）
    9. 工作和任務資料（依賴多個表）
    """

    print("🚀 開始初始化所有資料...")
    print("=" * 50)

    try:
        # 第一階段：基礎類型資料
        print("📍 第一階段：基礎類型資料")
        initialize_node_types(session)
        initialize_location_status(session)
        initialize_rack_status(session)
        initialize_carrier_status(session)
        initialize_agv_status(session)
        initialize_process_settings(session)
        initialize_traffic_zones(session)
        initialize_license(session)
        print()

        # 第二階段：節點資料
        print("🗺️ 第二階段：節點資料")
        initialize_nodes(session)
        initialize_kuka_map(session)  # KUKA 地圖資料匯入
        initialize_ct_map(session)    # CT 地圖資料匯入
        print()

        # 第三階段：機器和房間資料
        print("🏭 第三階段：機器和房間資料")
        initialize_machines(session)
        initialize_rooms(session)
        print()

        # 第四階段：位置資料
        print("📍 第四階段：位置資料")
        initialize_locations(session)
        print()

        # 第五階段：產品和 AGV 資料
        print("📦 第五階段：產品和 AGV 資料")
        initialize_products(session)
        initialize_agvs(session)
        print()

        # 第六階段：貨架資料
        print("🗄️ 第六階段：貨架資料")
        initialize_racks(session)
        initialize_carriers(session)
        print()

        # 第七階段：設備資料
        print("🏭 第七階段：設備資料")
        initialize_equipment(session)
        initialize_equipment_ports(session)
        initialize_equipment_signals(session)
        print()

        # 第八階段：工作和任務資料
        print("⚙️ 第八階段：工作和任務資料")
        initialize_task_status(session)
        initialize_works(session)
        initialize_tasks(session)
        print()

        print("=" * 50)
        print("✅ 所有資料初始化完成！")

    except Exception as e:
        print(f"❌ 資料初始化失敗: {e}")
        raise
