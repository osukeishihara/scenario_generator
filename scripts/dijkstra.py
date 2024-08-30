import yaml
import networkx as nx
import random

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
