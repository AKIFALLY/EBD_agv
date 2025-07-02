import networkx as nx

# 建立有向圖
G = nx.DiGraph()

# 加入帶權重的邊（from, to, weight）
G.add_edge("A", "B", weight=1)
G.add_edge("B", "C", weight=1)
G.add_edge("A", "C", weight=1)
G.add_edge("C", "D", weight=1)
G.add_edge("B", "D", weight=1)

# 計算從 A 到 D 的最短路徑（根據 weight 欄位）
shortest_path = nx.shortest_path(G, source="A", target="D", weight="weight")
path_length = nx.shortest_path_length(
    G, source="A", target="D", weight="weight")

print("最短路徑：", shortest_path)
print("總距離：", path_length)
