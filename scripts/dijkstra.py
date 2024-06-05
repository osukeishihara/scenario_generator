import yaml
import networkx as nx

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

def dijkstra_path_with_edges(G, source, target):
    path = nx.dijkstra_path(G, source, target)
    edges = []
    for i in range(len(path) - 1):
        edge_data = G.get_edge_data(path[i], path[i + 1])
        edge_id = edge_data['edge_id']
        edges.append(edge_id)
    return path, edges

# YAMLファイルを読み込む
file_path = '/home/osuke/gamma_ws/src/scenario_generator/config/topo_cit3f.yaml'  # トポロジカルマップのYAMLファイルのパスを指定
topomap = load_topomap(file_path)

# デバッグプリント
# print("Loaded topomap:", topomap)

# グラフを作成する
G = create_graph(topomap)

# グラフのエッジをデバッグプリント
# print("Graph edges:", G.edges(data=True))

# 出発ノードと到着ノードを指定して経路を計算する
source = 11  # 出発ノードのID
target = 4 # 到着ノードのID
path, edges = dijkstra_path_with_edges(G, source, target)

print(f"Path from node {source} to node {target}: {path}")
print(f"edges:", " -> ".join(str(edge_id) for edge_id in edges))