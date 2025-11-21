import time
import uuid
import requests


class KukaApiClient:
    def __init__(self, base_url="http://192.168.10.3:10870", username=None, password=None):
        """
        初始化 Kuka API 客戶端。
        :param base_url: Kuka API 的基礎 URL。
        :param username: 用於自動登入的使用者名稱。
        :param password: 用於自動登入的密碼。
        """
        self.base_url = base_url
        self.token = None
        if username and password:
            self.login(username, password)

    def _get_headers(self, include_auth=True):
        # print(f"{self.token}")
        headers = {"Content-Type": "application/json"}
        if include_auth and self.token:
            headers["Authorization"] = f"{self.token}"
        return headers

    def _post_request(self, endpoint, data=None, params=None, include_auth=True):
        """通用 POST 請求"""
        try:
            url = f"{self.base_url}{endpoint}"
            response = requests.post(
                url,
                headers=self._get_headers(include_auth=include_auth),
                json=data,
                params=params
            )
            response.raise_for_status()
            return response.json()
        except requests.exceptions.RequestException as e:
            return {"success": False, "message": str(e)}

    def _get_request(self, endpoint, params=None, include_auth=True):
        """通用 GET 請求"""
        try:
            url = f"{self.base_url}{endpoint}"
            response = requests.get(
                url,
                params=params,
                headers=self._get_headers(include_auth=include_auth)
            )
            response.raise_for_status()
            return response.json()
        except requests.exceptions.RequestException as e:
            return {"success": False, "message": str(e)}

    def login(self, username: str, password: str):
        """呼叫 /api/login 並取得 token"""
        payload = {"username": username, "password": password}
        # Login request does not include auth token
        response_data = self._post_request(
            "/api/login", data=payload, include_auth=False)

        if response_data.get("success"):
            token = response_data.get("data", {}).get("token") or response_data.get(
                "data", {}).get("access_token")
            if token:
                self.token = f"{token}"
                print("Successfully logged in. Token set for the instance.")
            else:
                print("Login successful, but no token found in response.")
        else:
            print(f"Login failed: {response_data.get('message')}")

        return response_data

    # --- AMR 任務執行相關接口 ---

    def job_query(self, query_job_dto: dict):
        """查詢作業 - POST /api/amr/jobQuery"""
        return self._post_request("/api/amr/jobQuery", data=query_job_dto)

    def mission_cancel(self, mission_cancel_dto: dict):
        """AMR任務取消 - POST /api/amr/missionCancel"""
        return self._post_request("/api/amr/missionCancel", data=mission_cancel_dto)

    def operation_feedback(self, operation_feedback_dto: dict):
        """AMR任務節點操作完成反饋 - POST /api/amr/operationFeedback"""
        return self._post_request("/api/amr/operationFeedback", data=operation_feedback_dto)

    def pause_mission(self, mission_code: str):
        """暫停任務 - POST /api/amr/pauseMission"""
        return self._post_request("/api/amr/pauseMission", params={"missionCode": mission_code})

    def recover_mission(self, mission_code: str):
        """恢復任務 - POST /api/amr/recoverMission"""
        return self._post_request("/api/amr/recoverMission", params={"missionCode": mission_code})

    def redirect_mission(self, redirect_mission_dto: dict):
        """AMR任務重新選擇目標點 - POST /api/amr/redirectMission"""
        return self._post_request("/api/amr/redirectMission", data=redirect_mission_dto)

    def submit_mission(self, agv_task: dict):
        """AMR任務下發接口 - POST /api/amr/submitMission"""
        return self._post_request("/api/amr/submitMission", data=agv_task)

    # --- AMR地圖點位與區域相關接口 ---

    def area_nodes_query(self, area_nodes_query_dto: dict):
        """區域內點位信息查詢 - POST /api/amr/areaNodesQuery"""
        return self._post_request("/api/amr/areaNodesQuery", data=area_nodes_query_dto)

    def area_query(self):
        """查詢所有WCS區域信息 - GET /api/amr/areaQuery"""
        return self._get_request("/api/amr/areaQuery")

    def query_all_forbidden_areas(self):
        """查詢所有禁行區 - GET /api/amr/queryAllForbiddenAreas"""
        return self._get_request("/api/amr/queryAllForbiddenAreas")

    def query_function_node(self, query_dto: dict):
        """查詢功能點位 - POST /api/amr/queryFunctionNode"""
        return self._post_request("/api/amr/queryFunctionNode", data=query_dto)

    def query_one_forbidden_area(self, forbidden_area_code: str = None, forbidden_area_id: int = None):
        """查詢指定禁行區 - GET /api/amr/queryOneForbiddenArea"""
        params = {}
        if forbidden_area_code:
            params["forbiddenAreaCode"] = forbidden_area_code
        if forbidden_area_id:
            params["forbiddenAreaId"] = forbidden_area_id
        return self._get_request("/api/amr/queryOneForbiddenArea", params=params)

    def query_wcs_area_by_map_node(self, node_uuid: str):
        """查詢點位所屬區域 - GET /api/amr/queryWCSAreaByMapNode"""
        return self._get_request("/api/amr/queryWCSAreaByMapNode", params={"nodeUuid": node_uuid})

    def update_forbidden_area_status(self, update_dto: dict):
        """更新指定禁行區的狀態 - POST /api/amr/updateForbiddenAreaStatus"""
        return self._post_request("/api/amr/updateForbiddenAreaStatus", data=update_dto)

    # --- AMR容器相關接口 ---

    def container_in(self, container_move_dto: dict):
        """容器入場 - POST /api/amr/containerIn"""
        return self._post_request("/api/amr/containerIn", data=container_move_dto)

    def container_out(self, container_move_out_dto: dict):
        """容器出場 - POST /api/amr/containerOut"""
        return self._post_request("/api/amr/containerOut", data=container_move_out_dto)

    def container_query(self, query_dto: dict):
        """容器查詢(僅包括入場狀態的) - POST /api/amr/containerQuery"""
        return self._post_request("/api/amr/containerQuery", data=query_dto)

    def container_query_all(self, query_dto: dict):
        """容器查詢(包括入場和離場) - POST /api/amr/containerQueryAll"""
        return self._post_request("/api/amr/containerQueryAll", data=query_dto)

    def query_all_container_model_code(self):
        """查詢所有容器模型 - GET /api/amr/queryAllContainerModelCode"""
        return self._get_request("/api/amr/queryAllContainerModelCode")

    def query_area_code_for_container_model(self, container_model_code: str, no_container_first: bool = None):
        """查詢容器模型對應的存放區域 - GET /api/amr/queryAreaCodeForContainerModel"""
        params = {"containerModelCode": container_model_code}
        if no_container_first is not None:
            params["noContainerFirst"] = no_container_first
        return self._get_request("/api/amr/queryAreaCodeForContainerModel", params=params)

    def update_container(self, update_dto: dict):
        """容器信息更新 - POST /api/amr/updateContainer"""
        return self._post_request("/api/amr/updateContainer", data=update_dto)

    # --- Plugin Operation ---

    def add_tag(self, plugin_id: str, tag: str, version: str):
        """Tag plugin - POST /api/plugin/addTag/{pluginId}"""
        return self._post_request(f"/api/plugin/addTag/{plugin_id}", params={"tag": tag, "version": version})

    def delete_plugin(self, plugin_id: str, version: str = None):
        """Delete plugin - POST /api/plugin/delete/{pluginId}"""
        params = {}
        if version:
            params["version"] = version
        return self._post_request(f"/api/plugin/delete/{plugin_id}", params=params)

    def get_record(self, params: dict):
        """Query record - GET /api/plugin/getRecord"""
        return self._get_request("/api/plugin/getRecord", params=params)

    def get_record_http_message(self, record_id: int):
        """Get record body - GET /api/plugin/getRecordHttpMessage"""
        return self._get_request("/api/plugin/getRecordHttpMessage", params={"id": record_id})

    def list_plugins(self, plugin_id: str = None):
        """List all plugins - GET /api/plugin/listPlugins"""
        params = {}
        if plugin_id:
            params["pluginId"] = plugin_id
        return self._get_request("/api/plugin/listPlugins", params=params)

    def reload_plugin(self, plugin_id: str, version: str = None):
        """Reload plugin - POST /api/plugin/reload/{pluginId}"""
        params = {}
        if version:
            params["version"] = version
        return self._post_request(f"/api/plugin/reload/{plugin_id}", params=params)

    def resend_record(self, record_id: int, parameter_map: dict):
        """Resend - POST /api/plugin/resend/{id}"""
        return self._post_request(f"/api/plugin/resend/{record_id}", data=parameter_map)

    def start_plugin(self, plugin_id: str, version: str = None):
        """Enable plugin - POST /api/plugin/start/{pluginId}"""
        params = {}
        if version:
            params["version"] = version
        return self._post_request(f"/api/plugin/start/{plugin_id}", params=params)

    def stop_plugin(self, plugin_id: str):
        """Disable plugin - POST /api/plugin/stop/{pluginId}"""
        return self._post_request(f"/api/plugin/stop/{plugin_id}")

    def unload_plugin(self, plugin_id: str):
        """Unload plugin - POST /api/plugin/unload/{pluginId}"""
        return self._post_request(f"/api/plugin/unload/{plugin_id}")

    def upload_plugin(self, jar_file_path: str):
        """Upload Plugin - POST /api/plugin/upload"""
        url = f"{self.base_url}/api/plugin/upload"
        headers = self._get_headers()
        if 'Content-Type' in headers:
            # Let requests library set the multipart header
            del headers['Content-Type']
        try:
            with open(jar_file_path, 'rb') as f:
                files = {
                    'jarFile': (jar_file_path.split('/')[-1], f, 'application/java-archive')}
                response = requests.post(url, files=files, headers=headers)
            response.raise_for_status()
            return response.json()
        except FileNotFoundError:
            return {"success": False, "message": f"File not found: {jar_file_path}"}
        except requests.exceptions.RequestException as e:
            return {"success": False, "message": str(e)}

    # --- 機器人相關接口 ---

    def charge_robot(self, request_dto: dict):
        """機器人充電 - POST /api/amr/chargeRobot"""
        return self._post_request("/api/amr/chargeRobot", data=request_dto)

    def insert_robot(self, request_dto: dict):
        """入場機器人 - POST /api/amr/insertRobot"""
        return self._post_request("/api/amr/insertRobot", data=request_dto)

    def query_robot_by_node_uuid_or_foreign_code(self, node_code: str):
        """根據點位UUID或外部編碼查詢機器人 - POST /api/amr/queryRobByNodeUuidOrForeignCode"""
        return self._post_request("/api/amr/queryRobByNodeUuidOrForeignCode", params={"nodeCode": node_code})

    def remove_robot(self, request_dto: dict):
        """離場機器人 - POST /api/amr/removeRobot"""
        return self._post_request("/api/amr/removeRobot", data=request_dto)

    def robot_move_carry(self, request_dto: dict):
        """機器人移動搬運 - POST /api/amr/robotMoveCarry"""
        return self._post_request("/api/amr/robotMoveCarry", data=request_dto)

    def robot_query(self, query_dto: dict):
        """機器人狀態查詢接口 - POST /api/amr/robotQuery"""
        return self._post_request("/api/amr/robotQuery", data=query_dto)
    
    # --- 便利方法 ---
    
    def get_all_robots(self):
        """獲取所有機器人狀態 - 便利方法"""
        return self.robot_query({})
    
    def get_robot_by_id(self, robot_id: str):
        """根據機器人ID查詢狀態 - 便利方法"""
        return self.robot_query({"robotId": robot_id})
    
    def get_all_containers_in_map(self):
        """獲取所有在場容器 - 便利方法"""
        return self.container_query({})
    
    def get_container_by_code(self, container_code: str):
        """根據容器代碼查詢 - 便利方法"""
        return self.container_query_all({"containerCode": container_code})
    
    def get_jobs_by_status(self, status: int):
        """根據狀態查詢作業 - 便利方法
        Args:
            status: KUKA AMR 官方狀態碼
                   10=待執行, 20=執行中, 25=等待放行, 28=取消中,
                   30=已完成, 31=已取消, 35=手動完成, 50=告警, 60=流程啟動異常
        """
        return self.job_query({"status": status})
    
    def get_running_jobs(self):
        """獲取運行中的作業 - 便利方法"""
        return self.get_jobs_by_status(20)  # 20 = 執行中
    
    def get_pending_jobs(self):
        """獲取待執行的作業 - 便利方法"""
        return self.get_jobs_by_status(10)  # 10 = 待執行
    
    def is_token_valid(self):
        """檢查 token 是否有效 - 便利方法"""
        if not self.token:
            return False
        # 嘗試調用一個簡單的 API 來驗證 token
        try:
            result = self.area_query()
            return result.get("success", False)
        except:
            return False
    
    def force_relogin(self, username: str, password: str):
        """強制重新登入 - 便利方法"""
        self.token = None
        return self.login(username, password)

    # --- 地圖資料相關接口 ---

    def get_map_floor(self, map_code: str, floor_number: str):
        """從 KUKA Fleet 獲取地圖樓層資料 - GET /api/v1/data/open-monitor/map-floor

        Args:
            map_code: 地圖代碼（如 "AlanACT"）
            floor_number: 樓層編號（如 "AlanSec1"）

        Returns:
            完整的 KUKA API 回應，包含：
            - floorInfo: 樓層資訊
            - nodes: 節點陣列（id, nodeNumber, nodeUuid, xCoordinate, yCoordinate, types等）
            - edges: 邊陣列（beginNodeId, endNodeId, edgeType等）

        注意: 此 API 使用不同的端口 (7888) 而非標準 KUKA Fleet API 端口 (10870)
        """
        # KUKA Fleet 地圖 API 使用端口 7888
        map_api_base_url = self.base_url.replace(':10870', ':7888')

        try:
            url = f"{map_api_base_url}/api/v1/data/open-monitor/map-floor"
            params = {
                'mapCode': map_code,
                'floorNumber': floor_number
            }
            response = requests.get(
                url,
                params=params,
                headers=self._get_headers(include_auth=True)
            )
            response.raise_for_status()
            return response.json()
        except requests.exceptions.RequestException as e:
            return {"success": False, "message": str(e)}


if __name__ == "__main__":
    # --- 初始化客戶端 ---
    # 可以在初始化時直接登入
    client = KukaApiClient(username="admin", password="Admin")

    # 或者先初始化，再手動登入
    # client = KukaApiClient()
    # client.login("admin", "Admin")

    # 檢查 token 是否成功設定
    if client.token:
        print("\n--- 開始測試 API ---")

        # --- AMR 任務執行相關接口 ---
        # print("查詢作業:", client.job_query({"jobCode": "test_job"}))
        # print("取消任務:", client.mission_cancel({"missionCode": "test_mission"}))

        # --- AMR地圖點位與區域相關接口 ---
        # print("查詢所有WCS區域資訊:", client.area_query())
        # print("查詢所有禁行区:", client.query_all_forbidden_areas())

        # --- AMR容器相關接口 ---
        # container_in_data = {
        #     "requestId": str(uuid.uuid4()),
        #     "containerCode": "container-test-002",
        #     "position": "map0512-sectionNUM-33",
        #     "isNew": True
        # }
        # print("容器入場:", client.container_in(container_in_data))
        # print("容器查詢(所有):", client.container_query_all({}))

        # --- 機器人相關接口 ---
        print("機器人狀態查詢:", client.robot_query({}))

    else:
        print("\n--- 登入失敗，無法執行 API 測試 ---")
