#!/usr/bin/env python
# coding: utf-8

# In[3]:


import networkx as nx
import matplotlib.pyplot as plt
import heapq
from collections import deque


# In[7]:


G = nx.Graph()
edges = [
    ('Museum', 'Park', 4),
    ('Museum', 'Mall', 2),
    ('Park', 'Zoo', 5),
    ('Mall', 'Theatre', 10),
    ('Zoo', 'Beach', 6),
    ('Theatre', 'Beach', 3),
    ('Museum', 'Zoo', 9)
]

G.add_weighted_edges_from(edges)

# --------- Visualization ---------
def draw_graph(path=None):
    pos = nx.spring_layout(G)
    plt.figure(figsize=(8, 6))
    nx.draw(G, pos, with_labels=True, node_color='skyblue', edge_color='gray', node_size=2000, font_size=12)
    labels = nx.get_edge_attributes(G, 'weight')
    nx.draw_networkx_edge_labels(G, pos, edge_labels=labels)

    if path:
        path_edges = list(zip(path, path[1:]))
        nx.draw_networkx_edges(G, pos, edgelist=path_edges, edge_color='red', width=2)
        nx.draw_networkx_nodes(G, pos, nodelist=path, node_color='lightgreen')

    plt.title("Travel Itinerary Optimizer")
    plt.show()

# --------- Algorithms ---------
def bfs(start, goal):
    visited = set()
    queue = deque([[start]])
    while queue:
        path = queue.popleft()
        node = path[-1]
        if node == goal:
            return path
        if node not in visited:
            visited.add(node)
            for neighbor in G.neighbors(node):
                new_path = list(path)
                new_path.append(neighbor)
                queue.append(new_path)
    return None

def dfs(start, goal):
    visited = set()
    stack = [[start]]
    while stack:
        path = stack.pop()
        node = path[-1]
        if node == goal:
            return path
        if node not in visited:
            visited.add(node)
            for neighbor in G.neighbors(node):
                new_path = list(path)
                new_path.append(neighbor)
                stack.append(new_path)
    return None

def dijkstra(start, goal):
    queue = [(0, start, [start])]
    visited = set()
    while queue:
        cost, node, path = heapq.heappop(queue)
        if node == goal:
            return path
        if node not in visited:
            visited.add(node)
            for neighbor in G.neighbors(node):
                weight = G[node][neighbor]['weight']
                heapq.heappush(queue, (cost + weight, neighbor, path + [neighbor]))
    return None

def a_star(start, goal):
    def heuristic(u, v):
        return abs(len(u) - len(v))

    open_set = [(0, start, [start])]
    g_score = {start: 0}
    visited = set()

    while open_set:
        f, current, path = heapq.heappop(open_set)
        if current == goal:
            return path
        visited.add(current)
        for neighbor in G.neighbors(current):
            temp_g = g_score[current] + G[current][neighbor]['weight']
            if neighbor not in g_score or temp_g < g_score[neighbor]:
                g_score[neighbor] = temp_g
                f_score = temp_g + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score, neighbor, path + [neighbor]))
    return None
def menu():
    print("Attractions:", list(G.nodes))
    start = input("Enter Start Point: ")
    goal = input("Enter Goal Point: ")
    print("\nChoose Algorithm:")
    print("1. BFS\n2. DFS\n3. Dijkstra\n4. A* Search")
    choice = input("Your choice: ")

    if choice == '1':
        path = bfs(start, goal)
        print("BFS Path:", path)
    elif choice == '2':
        path = dfs(start, goal)
        print("DFS Path:", path)
    elif choice == '3':
        path = dijkstra(start, goal)
        print("Dijkstra Path:", path)
    elif choice == '4':
        path = a_star(start, goal)
        print("A* Path:", path)
    else:
        print("Invalid option.")
        return

    if path:
        draw_graph(path)
    else:
        print("No path found.")

if __name__ == "__main__":
    menu()


# In[ ]:




