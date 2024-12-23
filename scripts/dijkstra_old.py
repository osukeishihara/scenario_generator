import yaml
import networkx as nx
import matplotlib.pyplot as plt
import numpy as np
import random
import threading

# YAMLデータを読み込む
# yaml_file_path = '/home/osuke/gamma_ws/src/scenario_generator/config/topo_cit3f.yaml'
# yaml_file_path = '/home/osuke/gamma_ws/src/scenario_generator/config/topo_cit3f.yaml'
yaml_file_path = '/home/osuke/gamma_ws/src/scenario_generator/config/topo_cit3f_expansion.yaml'

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
                G.add_edge(node_id, target_node_id, edge_id=edge_id, deg=edge['deg'])
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

def format_node(node_id):
    return f"[{node_id}]"

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

def random_shortest_path(G, all_paths_with_edges):
    random_path_with_edges = random.choice(all_paths_with_edges)
    return random_path_with_edges

def interleave_lists(nodes, edges):
    interleaved = []
    for edge, node in zip(edges, nodes):
        interleaved.append(edge)
        interleaved.append(node)
    if len(nodes) > len(edges):
        interleaved.append(nodes[-1])
    return interleaved

def describe_route(topomap, path, edges):
    instructions = []
    node_count = 0
    in_straight = False
    prev_deg = None
    first_straight = True  # 最初の直進かどうかを確認するフラグ

    # ノードのタイプを辞書にする
    node_type = {node['node']['id']: node['node']['type'] for node in topomap}

    # エッジの度数を辞書にする
    edge_degrees = {(edge['edge_id'], find_target_node(topomap, edge['edge_id'], node_id)): edge['deg']
                    for node in topomap for edge in node['node']['edge']
                    for node_id in [node['node']['id']]}

    def add_straight_instruction():
        """通常の直進指示を追加"""
        if node_count > 0:
            # ランダムに選択肢を選ぶ
            choice = random.choice([
                f"{node_count}つ目の角が見えるまで直進",
                f"{node_count}つ目の三叉路まで直進"
            ])
            instructions.append(choice)
            return True
        return False

    def has_180_degree_edge(current_node, next_node):
        """次のノードに接続する180°方向のエッジがあるか確認"""
        current_degree = edge_degrees.get((G.get_edge_data(current_node, next_node)['edge_id'], next_node), None)
        if current_degree is None:
            return False

        # 180°方向のエッジが存在するか確認
        for edge in G.edges(current_node):
            target_node = edge[1]
            edge_data = G.get_edge_data(edge[0], target_node)
            edge_degree = edge_data['deg']
            if (edge_degree - current_degree) % 360 == 180:
                return True
        return False

    def process_straight():
        """直進処理を統一して行う"""
        if in_straight:
            # print(f"Debug: In straight, node_count = {node_count}, checking 180-degree edge")

            if first_straight and node_count == 1 and is_start_dead_end():
                # スタートが dead_end ならランダムに選択
                choice = random.choice([
                    "1つ目の角が見えるまで直進",
                    "１つ目の三叉路まで直進",
                    "通路が見えるまで直進"
                ])
                instructions.append(choice)
                return True  # 最初の直進処理はこれで終了

            elif node_count == 1 and has_180_degree_edge(path[i-1], current_node):
                # 通るノードが1つで、かつ180°エッジがある場合
                choice = random.choice([
                    "1つ目の角が見えるまで直進",
                    "１つ目の三叉路まで直進",
                    "通路が見えるまで直進"
                ])
                instructions.append(choice)
                return True

            elif not has_180_degree_edge(path[i-1], current_node):
                instructions.append("突き当りまで直進")
            else:
                add_straight_instruction()
        return False

    def is_start_dead_end():
        """スタートノードが dead_end かどうかを確認"""
        return node_type.get(path[0]) == 'dead_end'

    # path または edges が空の場合の処理
    if not path or not edges:
        return "経路データが不足しています"

    for i, (current_node, next_node) in enumerate(zip(path[:-1], path[1:])):
        edge_id = G.get_edge_data(current_node, next_node)['edge_id']
        edge_data = edge_degrees.get((edge_id, next_node), None)

        if edge_data is None:
            continue

        if prev_deg is not None:
            deg_change = (edge_data - prev_deg) % 360

            if deg_change in [90, -270]:
                # 左折
                process_straight()
                instructions.append("左折")
                node_count = 0
                in_straight = True
                first_straight = False

            elif deg_change in [-90, 270]:
                # 右折
                process_straight()
                instructions.append("右折")
                node_count = 0
                in_straight = True
                first_straight = False

        # 直進の処理
        if prev_deg is None or deg_change in [0, 180]:
            if not in_straight:
                node_count = 1
                in_straight = True
            else:
                node_count += 1
        elif in_straight:
            node_count += 1

        prev_deg = edge_data

    # 最後の直進処理
    final_node = path[-1]
    if in_straight:
        if (node_type.get(path[-2]) == 'dead_end' or node_type.get(final_node) == 'dead_end'):
            instructions.append("突き当りまで直進")
        elif not has_180_degree_edge(path[-2], final_node):
            instructions.append("突き当りまで直進")
        else:
            add_straight_instruction()

    instructions.append("停止")

    # instructions が空でない場合はその内容を結合して返す
    if instructions:
        return ', '.join(instructions)

    # デフォルトのメッセージ
    return "経路が見つかりませんでした"





def find_start_and_end_nodes(G, edge_id):
    for edge in G.edges:
        if G.get_edge_data(*edge)['edge_id'] == edge_id:
            return edge
    return None

def input_int(prompt):
    while True:
        try:
            value = int(input(prompt))
            return value
        except ValueError:
            print("整数を入力してください。")

# YAMLファイルを読み込む
# file_path = '/home/osuke/gamma_ws/src/scenario_generator/config/topo_cit3f.yaml'  # トポロジカルマップのYAMLファイルのパスを指定
file_path = '/home/osuke/gamma_ws/src/scenario_generator/config/topo_cit3f_expansion.yaml'
topomap = load_topomap(file_path)

# グラフを作成する
G = create_graph(topomap)

# 整数入力を求める（整数以外は再入力）
start_edge = input_int("edge[1~14]よりstart_edgeを選択してください: ")
print(f"start_edge: {start_edge}")

target_node = input_int("node[1~12]よりtarget_nodeを選択してください: ")
print(f"target_node: {target_node}")

# スタートエッジの両端ノードを見つける
start_nodes = find_start_and_end_nodes(G, start_edge)

if start_nodes is None:
    print("指定されたスタートエッジはグラフに存在しません。")
else:
    # 各ノードから目標ノードまでの最短経路を取得する
    all_paths = []
    for start_node in start_nodes:
        all_paths.extend(all_shortest_paths(G, start_node, target_node))

    # スタートエッジを含む最短経路のみを選別
    paths_with_edges_list = paths_with_edges(G, all_paths)
    valid_paths_with_edges = [path for path in paths_with_edges_list if start_edge in path[1]]

    if not valid_paths_with_edges:
        print("startエッジを通る最短経路が見つかりませんでした。")
    else:
        # 経路とエッジを表示する
        print("\nすべての最短経路:")
        for idx, (path, edges) in enumerate(valid_paths_with_edges, start=1):
            print(f"\n経路 {idx}:")
            if len(path) > 1:
                trimmed_path = path[1:]
                formatted_path = [format_node(node) for node in trimmed_path]
            else:
                trimmed_path = path
                formatted_path = [format_node(node) for node in trimmed_path]
            
            interleaved = interleave_lists(formatted_path, edges)
            print(f"edge: {' -> '.join(str(edge_id) for edge_id in edges)}")
            print(f"node: {' -> '.join(formatted_path)}")
            print("route: " + ' -> '.join(str(item) for item in interleaved))

        # ランダムな最短経路を取得して表示する
        random_path_with_edges = random_shortest_path(G, valid_paths_with_edges)
        random_path, random_edges = random_path_with_edges

        # ランダム経路の番号を取得
        random_route_number = valid_paths_with_edges.index(random_path_with_edges) + 1

        print(f"\nrandom route (経路 {random_route_number}):")
        if len(random_path) > 1:
            trimmed_random_path = random_path[1:]
            formatted_random_path = [format_node(node) for node in trimmed_random_path]
        else:
            trimmed_random_path = random_path
            formatted_random_path = [format_node(node) for node in trimmed_random_path]
   
        random_interleaved = interleave_lists(formatted_random_path, random_edges)
        print(f"edge: {' -> '.join(str(edge_id) for edge_id in random_edges)}")
        print(f"node: {' -> '.join(formatted_random_path)}")
        print("route: " + ' -> '.join(str(item) for item in random_interleaved))

        # ランダム経路の説明を追加
        description = describe_route(topomap, random_path, random_edges)
        print(f"\nscenario:\n{description}")