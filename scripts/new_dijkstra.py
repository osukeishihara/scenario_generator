import yaml
import networkx as nx
import matplotlib.pyplot as plt
import numpy as np
import random
import os

# --- 設定 ---
YAML_PATH = '/home/osuke/gamma_ws/src/scenario_generator/config/topo_cit3f_sotsuron.yaml'
SAVE_PATH = '/home/osuke/gamma_ws/src/scenario_navigation/config/Scenarios/scenario_generator.txt'

class ScenarioGenerator:
    def __init__(self, yaml_path):
        if not os.path.exists(yaml_path):
            print(f"Error: YAML file not found at {yaml_path}")
            return
        with open(yaml_path, 'r') as f:
            self.raw_data = yaml.safe_load(f)
        self.topomap = self.raw_data['topomap']

        self.type_color_dict = {"straight_road": "blue", "dead_end": "blueviolet", "corner": "aqua", "3way": "lime"}
        
        self.edge_info_dict = {}
        for item in self.topomap:
            node = item['node']
            for e in node['edge']:
                self.edge_info_dict[(node['id'], e['edge_id'])] = e['deg']
        
        self.G = self._build_graph()
        self.pos = self._calculate_positions()

    def _build_graph(self):
        G = nx.Graph()
        for item in self.topomap:
            node = item['node']
            G.add_node(node['id'], type=node['type'])
            for e in node['edge']:
                target = self._find_target(e['edge_id'], node['id'])
                if target:
                    G.add_edge(node['id'], target, edge_id=e['edge_id'])
        return G

    def _find_target(self, edge_id, current_id):
        for item in self.topomap:
            node = item['node']
            if node['id'] == current_id: continue
            if any(e['edge_id'] == edge_id for e in node['edge']):
                return node['id']
        return None

    def _calculate_positions(self):
        pos = {1: (0, 0)}
        edge_length = 3
        edge_to_nodes = {}
        for item in self.topomap:
            for e in item['node']['edge']:
                edge_to_nodes.setdefault(e['edge_id'], []).append(item['node']['id'])

        for _ in range(len(self.topomap) * 2):
            for (node_id, edge_id), deg in self.edge_info_dict.items():
                if node_id in pos:
                    rad = np.radians(deg)
                    for other in edge_to_nodes[edge_id]:
                        if node_id != other and other not in pos:
                            pos[other] = (round(pos[node_id][0] + edge_length * np.cos(rad), 2),
                                          round(pos[node_id][1] + edge_length * np.sin(rad), 2))
        return pos

    def _has_straight_path(self, node_id, incoming_deg):
        for (n_id, e_id), out_deg in self.edge_info_dict.items():
            if n_id == node_id:
                diff = abs((out_deg % 360) - (incoming_deg % 360))
                if diff < 1.0 or diff > 359.0:
                    return True
        return False

    def describe_route(self, path):
        instructions = []
        node_count = 0
        prev_deg = None
        edges_in_path = [self.G[path[i]][path[i+1]]['edge_id'] for i in range(len(path)-1)]

        for i in range(len(path) - 1):
            curr_node = path[i]
            curr_deg = self.edge_info_dict[(curr_node, edges_in_path[i])]

            if prev_deg is not None:
                deg_change = (curr_deg - prev_deg) % 360
                if deg_change in [90, 270]:
                    if not self._has_straight_path(curr_node, prev_deg):
                        instructions.append("突き当りまで直進")
                    else:
                        c = max(1, node_count)
                        phrase = random.choice([f"{c}つ目の角が見えるまで直進", f"{c}つ目の三叉路まで直進"])
                        instructions.append(phrase)
                    instructions.append("左折" if deg_change == 90 else "右折")
                    node_count = 0 

            node_count += 1
            prev_deg = curr_deg

        last_node = path[-1]
        if not self._has_straight_path(last_node, prev_deg):
            instructions.append("突き当りまで直進")
        else:
            c = max(1, node_count)
            instructions.append(random.choice([f"{c}つ目の角が見えるまで直進", f"{c}つ目の三叉路まで直進"]))

        instructions.append("停止．")
        return "．".join(instructions)

    def show_map(self, path=None):
        plt.figure(figsize=(10, 8))
        node_colors = [self.type_color_dict.get(self.G.nodes[n].get('type'), "gray") for n in self.G.nodes()]
        nx.draw_networkx_nodes(self.G, self.pos, node_color=node_colors, edgecolors='black', node_size=600)
        nx.draw_networkx_labels(self.G, self.pos, font_size=12)
        nx.draw_networkx_edges(self.G, self.pos, edge_color='black', width=1)
        edge_labels = {(u, v): d['edge_id'] for u, v, d in self.G.edges(data=True)}
        nx.draw_networkx_edge_labels(self.G, self.pos, edge_labels=edge_labels, font_color='blue')
        if path:
            path_edges = list(zip(path, path[1:]))
            nx.draw_networkx_edges(self.G, self.pos, edgelist=path_edges, edge_color='red', width=3)
        plt.axis('off')
        plt.show()

# --- 実行部（修正：経路探索ロジックの改善） ---
if __name__ == "__main__":
    gen = ScenarioGenerator(YAML_PATH)
    if not hasattr(gen, 'G'): exit() # ファイル読み込み失敗時
    
    gen.show_map()

    try:
        start_edge_id = int(input("start_edge: "))
        target_node_id = int(input("target_node: "))
    except ValueError:
        print("数値のみを入力してください。")
        exit()

    # 指定されたエッジを構成する2つのノード(u, v)を探す
    edge_nodes = []
    for u, v, d in gen.G.edges(data=True):
        if d['edge_id'] == start_edge_id:
            edge_nodes.append((u, v))
            edge_nodes.append((v, u))

    if not edge_nodes:
        print(f"エッジID {start_edge_id} がマップ内に見つかりません。")
        exit()

    all_valid_paths = []
    for u, v in edge_nodes:
        try:
            # vからターゲットへの最短経路を探し、その頭にuを足す（u->vがstart_edgeになる）
            # もしターゲットがvそのものなら、経路は [u, v]
            if v == target_node_id:
                all_valid_paths.append([u, v])
            else:
                paths_from_v = nx.all_shortest_paths(gen.G, v, target_node_id)
                for p in paths_from_v:
                    # uに戻るような経路を除外（必要なら）
                    if u not in p:
                        all_valid_paths.append([u] + p)
        except (nx.NetworkXNoPath, nx.NodeNotFound):
            continue

    if all_valid_paths:
        # 最も短い経路を選択
        shortest_len = min(len(p) for p in all_valid_paths)
        valid_shortest_paths = [p for p in all_valid_paths if len(p) == shortest_len]
        chosen_path = random.choice(valid_shortest_paths)
        
        scenario = gen.describe_route(chosen_path)
        print(f"\nPath: {chosen_path}")
        print(f"Scenario:\n{scenario}")
        
        os.makedirs(os.path.dirname(SAVE_PATH), exist_ok=True)
        with open(SAVE_PATH, 'w', encoding='utf-8') as f: f.write(scenario)
        gen.show_map(path=chosen_path)
    else:
        print(f"エッジ {start_edge_id} からノード {target_node_id} への経路が見つかりませんでした。")