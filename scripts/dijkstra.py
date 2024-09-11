import yaml
import networkx as nx
import matplotlib.pyplot as plt
import numpy as np
import random
import threading

# YAMLデータを読み込む
# yaml_file_path = '/home/osuke/gamma_ws/src/scenario_generator/config/topo_cit3f.yaml'
yaml_file_path = '/home/osuke/gamma_ws/src/scenario_generator/config/topo_cit3f.yaml'
with open(yaml_file_path) as yaml_data:
    data = yaml.load(yaml_data, Loader=yaml.FullLoader)

type_color_dict = {"straight_road":"blue", "dead_end":"lime", "corner":"green","3way":"red"}
                #    , "corner", "cross_road", "3way_right", "3way_center", "3way_left"]
# color_list = ["blue", "lime", "blueviolet", "green", "gold", "aqua", "red", "orange"]


def check_overlapping(pos1,pos2 ,min_dist=0.1):
    return np.linalg.norm(np.array(pos1) - np.array(pos2)) < min_dist

# グラフを作成
G = nx.Graph()
pos = {}  # ノードの位置を格納する辞書
edge_length = 3  # すべてのエッジの長さを一定とする

# 初期ノードの位置を設定
pos[1] = (0, 0)  # 最初のノードの位置を原点とする

# Register all nodes
edge_labels = {}
edge_to_nodes = {}
edge_info_dict = {} 

for item in data['topomap']:
    if 'node' in item:
        node_data = item['node']
        node_id = node_data['id']
        node_type = node_data['type']
        G.add_node(node_id,type=node_type)
        # print(node_data)
        # print(pos)
        #Register edge data for current node
        for edge_info in node_data['edge']:
            edge_id = edge_info['edge_id']
            deg = edge_info['deg']
            edge_info_dict[(node_id,edge_id)] = deg #Registar deg using node_id and edge_id as keys
            if edge_id not in edge_to_nodes :
                edge_to_nodes[edge_id] =[]
            edge_to_nodes[edge_id].append(node_id)
            # print(edge_info_dict)
#Register edge 
for edge_id, nodes in edge_to_nodes.items():
    for i in range(len(nodes)):
        for j in range(i + 1, len(nodes)):
            edge_in_node_id1 = nodes[i]
            edge_in_node_id2 = nodes[j]
    # for edge_id1, edge_id2  in edge_to_nodes[edge_id]:
            # print(edge_in_node_id1,edge_in_node_id2)
            G.add_edge(edge_in_node_id1,edge_in_node_id2,id=edge_id)
            edge_labels[(edge_in_node_id1, edge_in_node_id2)] = str(edge_id)


pos = {1: (0,0)} # The position of the initial node is the origin.
for (node_id,edge_id),deg in  edge_info_dict.items():
    # print(pos)
    if node_id in pos: #nodeの位置を現在のnodeをもとに決定
        rad = np.radians(deg)  # 角度をラジアンに変換
        for other_node_id in edge_to_nodes[edge_id]:
            if node_id != other_node_id and other_node_id not in pos:
                pos_check = (round((pos[node_id][0] + edge_length * np.cos(rad)),1 ),
                            round((pos[node_id][1] + edge_length * np.sin(rad)),1))
                #Duplicate detection
                if pos_check in pos.values():
                    pos[other_node_id] = (
                                    round((pos[node_id][0] + edge_length/2 * np.cos(rad)),1 ),
                                    round((pos[node_id][1] + edge_length/2 * np.sin(rad)),1)
                                    )
                else:
                    pos[other_node_id] = pos_check

pos2 = dict(sorted(pos.items()))
node_colors = [type_color_dict[G.nodes[node]["type"]] for node in G.nodes()]

# グラフの描画を別スレッドで実行する関数
def draw_graph():
    plt.figure()
    nx.draw(G, pos2, with_labels=True, node_size=800, node_color=node_colors, edge_color='gray', font_size=12)
    nx.draw_networkx_edge_labels(G, pos2, edge_labels=edge_labels, font_color='blue')
    plt.show()

draw_graph()
#　最短経路の計算
def load_topomap(file_path):
    with open(file_path, 'r') as file:
        data = yaml.safe_load(file)
    return data['topomap']

def create_graph(topomap):
    G = nx.DiGraph()
    for entry in topomap:
        node = entry['node']
        node_id = node['id']
        for edge in node['edge']:
            edge_id = edge['edge_id']
            target_node_id = find_target_node(topomap, edge_id, node_id)
            if target_node_id is not None:
                G.add_edge(node_id, target_node_id, edge_id=edge_id, weight=1)  # Assuming all edges have a weight of 1
    return G

def find_target_node(topomap, edge_id, current_node_id):
    for entry in topomap:
        node = entry['node']
        node_id = node['id']
        if node_id == current_node_id:
            continue
        for edge in node['edge']:
            if edge['edge_id'] == edge_id:
                return node_id
    return None

def find_nodes_by_edge(topomap, edge_id):
    nodes = []
    for entry in topomap:
        node = entry['node']
        for edge in node['edge']:
            if edge['edge_id'] == edge_id:
                nodes.append(node['id'])
    return nodes

def find_shortest_path_with_mandatory_edge(G, topomap, start_edge, target):
    start_nodes = find_nodes_by_edge(topomap, start_edge)
    valid_paths = []

    for start_node in start_nodes:
        try:
            second_leg = nx.shortest_path(G, start_node, target, weight='weight')
            complete_path = [start_node] + second_leg
            valid_paths.append(complete_path)
        except nx.NetworkXNoPath:
            continue
    
    if valid_paths:
        # 最初のノードからの最短経路を探す
        shortest_path = min(valid_paths, key=len)
        
        # スタートエッジが最初になるように調整
        start_edge_node = find_nodes_by_edge(topomap, start_edge)[0]
        if start_edge_node in shortest_path:
            idx = shortest_path.index(start_edge_node)
            # スタートエッジが最初になるように調整
            shortest_path = shortest_path[idx:] + shortest_path[:idx]
        
        return shortest_path
    else:
        return None

def extract_edges_from_path(G, path):
    edges = []
    for i in range(len(path) - 1):
        edge_data = G.get_edge_data(path[i], path[i + 1])
        if edge_data:
            edge_id = edge_data['edge_id']
            edges.append(edge_id)
    return edges

# YAMLファイルを読み込む
file_path = '/home/osuke/gamma_ws/src/scenario_generator/config/topo_cit3f.yaml'  # トポロジカルマップのYAMLファイルのパスを指定
topomap = load_topomap(file_path)

# グラフを作成する
G = create_graph(topomap)

# スタートエッジと目標ノードを入力
start_edge = int(input("エッジ[1~14]より出発エッジを選択してください:"))
print(f"出発エッジ: {start_edge}")

target = int(input("ノード[1~12]より目標ノードを選択してください:"))
print(f"目標ノード: {target}")

# スタートエッジを通る最短経路を取得する
shortest_path = find_shortest_path_with_mandatory_edge(G, topomap, start_edge, target)


if shortest_path:
    # ノードの最初の１つを削除するが、エッジはそのままにする
    nodes = shortest_path
    edges = extract_edges_from_path(G, nodes)
    edges_with_start = [start_edge] + edges


    # 最初のノードを出力から削除する
    print("最短経路のノード:")
    print(f"Nodes: {' -> '.join(str(node) for node in nodes[1:])}")
    
    print("最短経路のエッジ:")
    print(f"Edges: {' -> '.join(str(edge_id) for edge_id in edges_with_start)}")

else:
    print("スタートエッジから目標ノードへの経路が見つかりませんでした。")