#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import yaml
import networkx as nx
import matplotlib.pyplot as plt
import numpy as np
import random
import os
from std_msgs.msg import String, Bool, Int32

# --- 設定 ---
YAML_PATH = '/home/osuke/gamma_ws/src/scenario_generator/config/topo_cit3f_sotsuron.yaml'
SAVE_PATH = '/home/osuke/gamma_ws/src/scenario_navigation/config/Scenarios/scenario_generator.txt'

class ScenarioManagerNode:
    def __init__(self):
        rospy.init_node('scenario_manager', anonymous=True)
        self.current_edge_id = None
        self.target_node_id = None
        self.load_map(YAML_PATH)
        
        self.type_color_dict = {"straight_road": "blue", "dead_end": "blueviolet", "corner": "aqua", "3way": "lime"}
        self.pos = self._calculate_positions()

        # --- サブスクライバー ---
        rospy.Subscriber('/current_edge_id', Int32, self.edge_cb)
        rospy.Subscriber('/request_replanning', Bool, self.replan_cb)

    def load_map(self, yaml_path):
        with open(yaml_path, 'r') as f:
            self.raw_data = yaml.safe_load(f)
        self.topomap = self.raw_data['topomap']
        self.edge_info_dict = {(n['node']['id'], e['edge_id']): e['deg'] 
                               for n in self.topomap for e in n['node']['edge']}
        self.G = self._build_graph()

    def _build_graph(self):
        G = nx.Graph()
        for item in self.topomap:
            node = item['node']
            G.add_node(node['id'], type=node['type'])
            for e in node['edge']:
                target = self._find_target(e['edge_id'], node['id'])
                if target: G.add_edge(node['id'], target, edge_id=e['edge_id'])
        return G

    def _find_target(self, edge_id, current_id):
        for item in self.topomap:
            node = item['node']
            if node['id'] == current_id: continue
            if any(e['edge_id'] == edge_id for e in node['edge']): return node['id']
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

    def show_map(self):
        plt.figure(figsize=(10, 8))
        plt.title("Topological Map - Close to proceed")
        node_colors = [self.type_color_dict.get(self.G.nodes[n].get('type'), "gray") for n in self.G.nodes()]
        nx.draw_networkx_nodes(self.G, self.pos, node_color=node_colors, edgecolors='black', node_size=600)
        nx.draw_networkx_labels(self.G, self.pos, font_size=12)
        nx.draw_networkx_edges(self.G, self.pos, edge_color='black', width=1)
        edge_labels = {(u, v): d['edge_id'] for u, v, d in self.G.edges(data=True)}
        nx.draw_networkx_edge_labels(self.G, self.pos, edge_labels=edge_labels, font_color='blue')
        plt.axis('off')
        plt.show()

    def generate_and_save(self, is_replan=False):
        """経路計算・保存・ターミナル出力を行う"""
        edge_nodes = []
        for u, v, d in self.G.edges(data=True):
            if d['edge_id'] == self.current_edge_id:
                edge_nodes.extend([(u, v), (v, u)])

        all_valid_paths = []
        for u, v in edge_nodes:
            try:
                if v == self.target_node_id: all_valid_paths.append([u, v])
                else:
                    for p in nx.all_shortest_paths(self.G, v, self.target_node_id):
                        if u not in p: all_valid_paths.append([u] + p)
            except: continue

        if all_valid_paths:
            shortest_len = min(len(p) for p in all_valid_paths)
            chosen_path = random.choice([p for p in all_valid_paths if len(p) == shortest_len])
            scenario = self.describe_route(chosen_path)
            
            # --- ターミナルへの出力（ここを追加） ---
            print("\n" + "="*40)
            status = "REPLANNING" if is_replan else "INITIAL SCENARIO"
            print(f"[{status}] Path: {chosen_path}")
            print(f"Generated Instructions:\n{scenario}")
            print("="*40 + "\n")
            # -------------------------------------

            with open(SAVE_PATH, 'w', encoding='utf-8') as f:
                f.write(scenario)
            
            if is_replan:
                os.system("rosnode kill /scenario_parser")
            return True
        else:
            rospy.logerr("No valid path found!")
            return False

    def describe_route(self, path):
        """元コードのロジックで文章を生成"""
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
                        instructions.append(f"{max(1, node_count)}つ目の角が見えるまで直進")
                    instructions.append("左折" if deg_change == 90 else "右折")
                    node_count = 0 
            node_count += 1
            prev_deg = curr_deg

        if not self._has_straight_path(path[-1], prev_deg):
            instructions.append("突き当りまで直進")
        else:
            instructions.append(f"{max(1, node_count)}つ目の角が見えるまで直進")
        instructions.append("停止．")
        return "．".join(instructions)

    def _has_straight_path(self, node_id, incoming_deg):
        return any(abs((out_deg % 360) - (incoming_deg % 360)) < 1.0 
                   for (n_id, e_id), out_deg in self.edge_info_dict.items() if n_id == node_id)

    def edge_cb(self, msg):
        self.current_edge_id = msg.data

    def replan_cb(self, msg):
        if msg.data and self.current_edge_id is not None:
            rospy.logwarn(f"Replanning initiated on Edge {self.current_edge_id}")
            edges_to_remove = [(u, v) for u, v, d in self.G.edges(data=True) if d['edge_id'] == self.current_edge_id]
            for u, v in edges_to_remove:
                if self.G.has_edge(u, v): self.G.remove_edge(u, v)
            self.generate_and_save(is_replan=True)

    def get_initial_input(self):
        self.show_map()
        print("\n--- Initial Setup ---")
        try:
            self.current_edge_id = int(input("Start Edge ID: "))
            self.target_node_id = int(input("Target Node ID: "))
            self.generate_and_save(is_replan=False)
        except ValueError:
            print("Invalid input.")
            exit()

if __name__ == '__main__':
    mgr = ScenarioManagerNode()
    mgr.get_initial_input()
    rospy.spin()