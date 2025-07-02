import json
import os
import math
import networkx as nx
import yaml

class AStarAlgorithm:

    _site_map = None  # 類別層級共享的站點對應表

    def __init__(self, start_node=None, end_node=None):

        # Step 1: 讀取 JSON 檔
        self.source_data = self.load_path_test_json()

        # Step 2: 轉換為 graph-friendly 結構
        self.converted_data = self.convert_tag_data_to_graph_format(self.source_data)

        # Step 3: 建立圖（以 CanToMoveTag 為來源 → TagNo 為目的）
        self.graph = self.build_graph_from_converted_data(self.converted_data)

        # 初始化 A* 用資料（如果有給定 start/end）
        if start_node is not None and end_node is not None:
            self.start_node = start_node
            self.end_node = end_node
            self.open_set = set([self.start_node])
            self.closed_set = set()
            self.came_from = {}

            self.g_score = {node: float('inf') for node in self.graph}
            self.f_score = {node: float('inf') for node in self.graph}
            self.g_score[self.start_node] = 0
            self.f_score[self.start_node] = self.heuristic(self.start_node, self.end_node)
        
    # ✅ 類別層級方法：讀取站點設定
    @classmethod
    def load_site_map(cls, path="/app/config/stationID.yaml"):
        if cls._site_map is None:
            if not os.path.exists(path):
                raise FileNotFoundError(f"❌ 找不到站點設定檔: {path}")
            with open(path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
                cls._site_map = data.get("StationID", {})  # ✅ 修正這裡
        return cls._site_map

    # ✅ 類別層級方法：查詢站點對應的 TAG
    @classmethod
    def get_tag_by_station(cls, station_id):
        site_map = cls.load_site_map()
        return site_map.get(station_id)

    # ✅ 類別層級方法：查詢站點對應的 TAG
    @classmethod
    def load_path_test_json(self ,config_path="/app/config/path.yaml"):
        #file_path = "/app/config/20250509_pathtest.json"
        if not os.path.exists(config_path):
            raise FileNotFoundError(f"❌ YAML 設定檔不存在: {config_path}")
        
        with open(config_path, 'r', encoding='utf-8') as yaml_file:
            config = yaml.safe_load(yaml_file)

        # 取得 JSON 檔案路徑
        file_path = config.get("path_data_file", {}).get("file_path", None)
        if not file_path:
            raise ValueError("❌ 無法在 config.yaml 中找到 'path_data_file.file_path'")

        if not os.path.exists(file_path):
            raise FileNotFoundError(f"❌ 指定的 JSON 檔案不存在: {file_path}")

        # 讀取 JSON 資料
        with open(file_path, 'r', encoding='utf-8') as file:
            data = json.load(file)
            return data



        with open(file_path, 'r', encoding='utf-8') as file:
            data = json.load(file)
            return data

    # ✅ 你也可以新增反查 TAG 對應的站點 ID（可選）
    @classmethod
    def get_station_by_tag(cls, tag):
        site_map = cls.load_site_map()
        for site_id, t in site_map.items():
            if t == tag:
                return site_id
        return None


    def getXY(self,tag_id):
        if tag_id in self.converted_data:
            x = self.converted_data[tag_id]['x']
            y = self.converted_data[tag_id]['y']
            #print(f"Tag {tag_id} 的座標是 X: {x}, Y: {y}")
            return x,y





    #將檔案轉換成指定格式
    def convert_tag_data_to_graph_format(self,tag_data: list) -> dict:
        """
        將原始 Tag 資料轉換為指定格式:
        {
            tag_id: {
                "x": ...,
                "y": ...,
                "neighbors": [...]
         }
         """   
        converted={}#初始化轉換資料
        for entry in tag_data:
            tag_no = entry["TagNo"]
            x = entry["Tag_X"]
            y = entry["Tag_Y"]
            neighbors = [
                item["CanToMoveTag"]
                for item in entry.get("CanToMoveSet", [])
                if item["CanToMoveTag"] != 0
            ]
            converted[tag_no] = {
                "x": x,
                "y": y,
                "neighbors": neighbors
            }
        return converted



    #將轉換後的資料轉換成圖
    def build_graph_from_converted_data(self, converted: dict) -> nx.DiGraph:
            G = nx.DiGraph()#將G轉為有向圖
            for tag_id, data in converted.items():
                G.add_node(tag_id, pos=(data["x"], data["y"]))
            for to_tag, data in converted.items():
                for from_tag in data["neighbors"]:  #誰可以來到我這裡
                    if from_tag in converted:
                        x1, y1 = converted[from_tag]["x"], converted[from_tag]["y"]
                        x2, y2 = data["x"], data["y"]
                        dist = math.hypot(x2 - x1, y2 - y1)
                        G.add_edge(from_tag, to_tag, weight=dist)
            return G


    #"""歐式距離啟發函數"""
    def heuristic(self, u, v):
            ux, uy = self.graph.nodes[u]['pos']
            vx, vy = self.graph.nodes[v]['pos']
            return math.hypot(ux - vx, uy - vy)




    

    # 反轉路徑順序
    #def reconstruct_path(self, current):
    #        path = [current]
    #        while current in self.came_from:
    #            current = self.came_from[current]
    #            path.append(current)
    #        return path[::-1]



    # 選擇 f_score 最小的節點
    #def run(self):
    #        while self.open_set:
    #            current = min(self.open_set, key=lambda n: self.f_score[n])
#
    #            if current == self.end_node:
    #                return self.reconstruct_path(current)
#
    #            self.open_set.remove(current)
    #            self.closed_set.add(current)
#
    #            for neighbor in self.graph.neighbors(current):
    #                if neighbor in self.closed_set:
    #                    continue
#
    #                tentative_g = self.g_score[current] + self.graph[current][neighbor]["weight"]
#
    #                if neighbor not in self.open_set:
    #                    self.open_set.add(neighbor)
    #                elif tentative_g >= self.g_score[neighbor]:
    #                    continue
#
    #                self.came_from[neighbor] = current
    #                self.g_score[neighbor] = tentative_g
    #                self.f_score[neighbor] = tentative_g + self.heuristic(neighbor, self.end_node)
#
    #        raise ValueError("❌ 找不到路徑")

    #使用內建的A*演算法
    def run(self):
        try:
            path = nx.astar_path(
                self.graph,#有向圖
                self.start_node,#起始點
                self.end_node,#結束點
                heuristic=self.heuristic,#啟發函數
                weight="weight"#權重
            )
            return path
        except nx.NetworkXNoPath:
            raise ValueError(f"❌ 找不到從 {self.start_node} 到 {self.end_node} 的路徑")

    def split_32_to_16(self,value):
    # 確保是無符號 32 位元整數（如有需要）
        value &= 0xFFFFFFFF
        low = value & 0xFFFF          # 取低 16 位元
        high = (value >> 16) & 0xFFFF # 取高 16 位元
        return low, high
    


#運行測試
if __name__ == "__main__":
    astar = AStarAlgorithm()
    #path = astar.run()
    print(astar.converted_data)
    #print("最短路徑:", path)
    x,y = astar.getXY(2)
    
    print(type(x))
    
    
    #print(AStarAlgorithm.load_site_map())  # 預先載入站點設定

    tag = AStarAlgorithm.get_tag_by_station("Soaking01")
    #print(f"Soaking01 對應的 ID: {tag}")  # ➜ 應該印出 4

    station = AStarAlgorithm.get_station_by_tag(4)
    #print(f"ID 4 對應的站點: {station}")  # ➜ 應該印出 Soaking01
    #print(astar.source_data.get("TagNo"))



















    #寫入給PLC的測試資料
    #A_cantomove_tag = 0
    #pgv = 0
    #act = []
    #speed = []
    #shift = []
    #inposition = []
    #safe_sensor_setting = []
    #dataValue = [0] * 2000  # 初始化 dataValue 為長度 2000 的列表
    #x = 0
    #y = 0
    ##print("轉換前的資料:", astar.source_data)
    #for i in range(len(path)): 
#
#
    #        x = 0
    #        y = False
    #        if len(path)-1 == i:
    #            x = path[i]
    #            y = True
    #            print(f"最後一個點{tag1.get('TagNo')}")
    #        else:
    #            x = path[i+1]
    #            y = False   
#
    #        for tag1 in astar.source_data:
#
    #                
    #            
    #            
#
    #                if tag1.get('TagNo') == x:
    #                    #print(f"找到 TagNo: {tag1.get('TagNo')}")
    #                    cantomove_tag = tag1.get('CanToMoveSet')
    #                    for j in range(len(cantomove_tag)):
    #                        if cantomove_tag[j].get('CanToMoveTag') == path[i]:
    #                            A_cantomove_tag = cantomove_tag[j].get('CanToMoveTag')
    #                            pgv = cantomove_tag[j].get('PGV')
    #                            act = cantomove_tag[j].get('Act')
    #                            speed = cantomove_tag[j].get('Speed')
    #                            shift = cantomove_tag[j].get('SHIFT')
    #                            inposition = cantomove_tag[j].get('Inposition')
    #                            safe_sensor_setting = cantomove_tag[j].get('SafeSensorSetting')
    #                            print(f"找到 CanToMoveTag: {A_cantomove_tag}, PGV: {pgv}, Act: {act}, Speed: {speed}, Shift: {shift}, Inposition: {inposition}, SafeSensorSetting: {safe_sensor_setting}")
    #                            #print(f"safe{safe_sensor_setting}")
    #                            
    #                            # 將 TagNo, Tag_X, Tag_Y 寫入 dataValue
    #                            # 假設每個 tag 有 'TagNo', 'Tag_X', 'Tag_Y' 等屬性
    #                            # 並且每個 tag 的索引是 i*20 (20 是每個 tag 的資料長
    #                    if y:
    #                        dataValue[i*20+0] = tag1.get('TagNo')  #Tag No_Index=0
    #                        print(f"找到 TagNo: {dataValue[i*20+0]}, 位置: {i*20}")
    #                        #print(f"最後一個點{tag1.get('TagNo')}")
    #                    else:
    #                        dataValue[i*20+0] = A_cantomove_tag  #Tag No_Index
    #                        print(f"找到 TagNo: {dataValue[i*20+0]}, 位置: {i*20}")
#
    #                    if y:
    #                        #print(f"最後一個點{tag1.get('TagNo')}")
    #                        dataValue[i*20+2] =  tag1.get('Station')+20 #Station_Index=2
    #                        break
    #                    else:
    #                        if len(act)>=1:
    #                            dataValue[i*20+2] =  act[0]#ACT_Index=2
#
    #                    dataValue[i*20+4] ,dataValue[i*20+5] = astar.split_32_to_16(tag1.get('Tag_X')) # Tag_X_Index=4
    #                    dataValue[i*20+9] ,dataValue[i*20+10] = astar.split_32_to_16(tag1.get('Tag_Y'))# Tag_Y_Index=9
    #                    dataValue[i*20+1] =  pgv#PGV_Index=1
    #                    
    #                    
    #                    dataValue[i*20+7] =  12 #ACT_Index=7
    #                    
    #                    dataValue[i*20+12] =  12
    #                    if len(safe_sensor_setting)>=1:
    #                        dataValue[i*20+6] =  safe_sensor_setting[0] #SafeSensorSetting_Index=6
    #                    if len(safe_sensor_setting)>=2:
    #                        dataValue[i*20+11] =  safe_sensor_setting[1]
    #                    if len(safe_sensor_setting)>=3:    
    #                        dataValue[i*20+16] =  safe_sensor_setting[2]
#
    #                    if len(speed)>=1:
    #                        dataValue[i*20+3] = speed[0]  #Speed_Index=3
    #                    if len(speed)>=2:
    #                        dataValue[i*20+8] =  speed[1]
    #                    if len(speed)>=3:
    #                        dataValue[i*20+13] =  speed[2]
    #                    if len(shift) >= 3:
    #                        dataValue[i*20+14] ,dataValue[i*20+15] =  astar.split_32_to_16(shift[2])  #旋轉角度#print(astar.converted_data)
    #                    break
    ###     # 印出轉換後的資料
    #print("轉換後的資料:", dataValue)