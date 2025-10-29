import osmnx as ox
import folium
import random
import math
from heapq import heappush, heappop

# === 1. Download map of your ward ===
G = ox.graph_from_place("Phường Tương Mai, Hà Nội, Việt Nam", network_type="all")

# === 2. Convert to GeoDataFrames ===
nodes, edges = ox.graph_to_gdfs(G)

# === 3. Pick random start & end nodes ===
start_node = random.choice(list(G.nodes))
end_node = random.choice(list(G.nodes))
print(f"Start node: {start_node}")
print(f"End node: {end_node}")

# === 4. Define haversine distance ===
def haversine(node1, node2):
    lat1, lon1 = G.nodes[node1]["y"], G.nodes[node1]["x"]
    lat2, lon2 = G.nodes[node2]["y"], G.nodes[node2]["x"]
    R = 6371e3
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

# === 5. Custom A* implementation ===
def a_star(graph, start, goal):
    open_set = []
    heappush(open_set, (0, start))
    came_from = {}
    g_score = {n: float("inf") for n in graph.nodes}
    g_score[start] = 0
    f_score = {n: float("inf") for n in graph.nodes}
    f_score[start] = haversine(start, goal)

    while open_set:
        _, current = heappop(open_set)
        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            return path[::-1]

        for neighbor in graph.neighbors(current):
            edge_data = graph[current][neighbor][0]
            weight = edge_data.get("length", haversine(current, neighbor))
            tentative_g = g_score[current] + weight
            if tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + haversine(neighbor, goal)
                heappush(open_set, (f_score[neighbor], neighbor))
    return None

# === 6. Run A* ===
path = a_star(G, start_node, end_node)

# === 7. Display in terminal ===
if path:
    print("✅ Path found:")
    print(" → ".join(map(str, path)))
else:
    print("❌ No path found.")

# === 8. Draw everything on Folium map ===
center = [nodes["y"].mean(), nodes["x"].mean()]
m = folium.Map(location=center, zoom_start=15)

# --- draw edges ---
for _, row in edges.iterrows():
    coords = [(lat, lon) for lon, lat in row["geometry"].coords]
    folium.PolyLine(coords, color="gray", weight=2, opacity=0.5).add_to(m)

# --- draw all nodes ---
for _, row in nodes.iterrows():
    folium.CircleMarker(
        location=[row["y"], row["x"]],
        radius=2,
        color="red",
        fill=True,
        fill_opacity=0.7
    ).add_to(m)

# --- draw A* path if found ---
if path:
    path_coords = [(G.nodes[n]["y"], G.nodes[n]["x"]) for n in path]
    folium.PolyLine(path_coords, color="blue", weight=5, opacity=0.8).add_to(m)
    folium.Marker(path_coords[0], popup="Start", icon=folium.Icon(color="green")).add_to(m)
    folium.Marker(path_coords[-1], popup="End", icon=folium.Icon(color="blue")).add_to(m)

# === 9. Save ===
m.save("astar_with_nodes.html")
print("✅ Map saved as astar_with_nodes.html")
