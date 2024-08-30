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

def all_shortest_paths(G, start, target):
    all_paths = list(nx.all_shortest_paths(G, start, target, weight='weight'))
    return all_paths

def paths_with_edges(G, paths):
    paths_with_edges = []
    for path in paths:
        edges = []
        for i in range(len(path) - 1):
            edge_data = G.get_edge_data(path[i], path[i + 1])
            edge_id = edge_data['edge_id']
            edges.append(edge_id)
        paths_with_edges.append((path, edges))
    return paths_with_edges

def random_shortest_path(G, start, target):
    all_paths = all_shortest_paths(G, start, target)
    random_path = random.choice(all_paths)
    edges = []
    for i in range(len(random_path) - 1):
        edge_data = G.get_edge_data(random_path[i], random_path[i + 1])
        edge_id = edge_data['edge_id']
        edges.append(edge_id)
    return random_path, edges

# YAMLファイルを読み込む
file_path = '/home/osuke/gamma_ws/src/scenario_generator/config/topo_cit3f.yaml'  # トポロジカルマップのYAMLファイルのパスを指定
topomap = load_topomap(file_path)

# グラフを作成する
G = create_graph(topomap)

start = int(input("ノード[1~12]より出発ノードを選択してください:"))
print(f"出発ノード: {start}")
target = int(input("ノード[1~12]より目標ノードを選択してください:"))
print(f"目標ノード: {target}")

# 全ての最短経路を取得する
all_paths = all_shortest_paths(G, start, target)

# 経路とエッジを表示する
paths_with_edges = paths_with_edges(G, all_paths)


print("すべての最短経路:")
for path, edges in paths_with_edges:
    print(f"node: {path}")
    print(f"Edges: {' -> '.join(str(edge_id) for edge_id in edges)}")

# ランダムな最短経路を取得して表示する
random_path, random_edges = random_shortest_path(G, start, target)
print("\n最短経路の中からランダムに選んだ１例:")
print(f"node: {random_path}") 
print(f"Edges: {' -> '.join(str(edge_id) for edge_id in random_edges)}")
