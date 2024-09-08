import heapq
from pickle import NONE
import osmium as osm
import networkx as nx
from mpl_interactions import panhandler, zoom_factory
import matplotlib.pyplot as plt
import matplotlib
from matplotlib.backend_bases import MouseButton
import math

path_edges = []
Edges_set=set()
matplotlib.use('TkAgg')
plt.rcParams['lines.antialiased'] = False

# Class để xử lý dữ liệu OSM và tạo đồ thị NetworkX
class OSMHandler(osm.SimpleHandler):
    def __init__(self):
        osm.SimpleHandler.__init__(self)
        self.graph = nx.DiGraph()

    def node(self, n):
        self.graph.add_node(n.id, lon=n.location.lon, lat=n.location.lat)

    def way(self, w):
        if 'highway' in w.tags:
            nodes = list(w.nodes)
            for i in range(len(nodes) - 1):
                self.graph.add_edge(nodes[i].ref, nodes[i + 1].ref)


# Hàm chuyển đổi file .osm.pbf thành NetworkX Graph
def convert_osm_pbf_to_networkx(file_path):
    handler = OSMHandler()
    handler.apply_file(file_path, locations=True)
    return handler.graph
def haversine(lon1, lat1, lon2, lat2):
     R = 6371.0 # Bán kính Trái Đất tính bằng km
     dlon = math.radians(lon2 - lon1)
     dlat = math.radians(lat2 - lat1)
     a = math.sin(dlat / 2)**2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon / 2)**2
     c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
     return R * c
def filter_nodes(G):
    # loại bỏ các đỉnh không kết nối
    to_remove = [node for node, degree in G.degree() if degree == 0]
    G.remove_nodes_from(to_remove)
    return G

file_path = 'map.osm'
G = convert_osm_pbf_to_networkx(file_path)
G = filter_nodes(G)
def heuristic(u, v):
    lon_u, lat_u = G.nodes[u]['lon'], G.nodes[u]['lat']
    lon_v, lat_v = G.nodes[v]['lon'], G.nodes[v]['lat']
    return haversine(lon_u, lat_u, lon_v, lat_v)
def G_function(current, neighbor):
    lon_u, lat_u = G.nodes[current]['lon'], G.nodes[current]['lat']
    lon_v, lat_v = G.nodes[neighbor]['lon'], G.nodes[neighbor]['lat']
    return haversine(lon_u, lat_u, lon_v, lat_v)
def astar_path(G, start, end):
    open_set = []
    global Edges_set

    closed_set=set()
    heapq.heappush(open_set, (0, start))

    came_from = {}
    g_score = {node: float('inf') for node in G.nodes}
    g_score[start] = 0

    f_score = {node: float('inf') for node in G.nodes}
    f_score[start] = heuristic(start, end)#f=0+h

    while open_set:
        current = heapq.heappop(open_set)[1]# lấy min do heapq là cây, gtnn làm root

        # Nếu đã đến nút đích, truy vết và trả về đường đi
        if current == end:
            return Build_path(came_from, current)

        closed_set.add(current)

        # Duyệt qua các nút hàng xóm
        for neighbor in G.neighbors(current):
            if neighbor in closed_set:
                continue  # Bỏ qua nút đã duyệt nếu đã duyệt rồi
            #hàng xóm chưa được duyệt
            #G(neighbor)=G từ Start đến current + chi phí từ current tới neighbor ( hoặc 1 nếu Vô Hướng)
            tentative_g_score = g_score[current] + G_function(current,neighbor)
            #Tính H
            h=heuristic(neighbor,end)
            f = tentative_g_score + h

            if tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = f
                if neighbor not in [i[1] for i in open_set]:
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
                    Edges_set.add((current, neighbor))
    return None  # Không tìm thấy đường đi

def Build_path(came_from, current):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    return path[::-1]  # Đảo ngược để trả về từ start -> end
def update_plot(start_node=None, end_node=None, xlim=None, ylim=None):
    ax.clear()
    edges=list(G.edges());
    nx.draw_networkx_edges(G, pos, edgelist=edges, ax=ax, edge_color="gray", width=0.8, alpha=1,style='solid',arrows=False)
    global path_edges
    if Edges_set:
        Edges_set_list=list(Edges_set)
        nx.draw_networkx_edges(G, pos, edgelist=Edges_set_list, ax=ax, edge_color="blue", width=1, alpha=1,style='solid',arrows=False)
    if path_edges:
        nx.draw_networkx_edges(G, pos, edgelist=path_edges, ax=ax, edge_color="red", width=1, alpha=1,style='solid',arrows=False)
    elif start_node!=None and end_node!=None:
        DrawWay(start_node,end_node)
    if xlim and ylim:
        ax.set_xlim(xlim)
        ax.set_ylim(ylim)

    plt.draw()
def DrawWay(start_node=None,end_node=None):
    global path_edges, Edges_set
    if path_edges:
        nx.draw_networkx_edges(G, pos, edgelist=path_edges, ax=ax, edge_color="gray", width=1.1, alpha=1,style='solid',arrows=False)
        path_edges.clear();
    if Edges_set:
        Edges_set_list=list(Edges_set)
        nx.draw_networkx_edges(G, pos, edgelist=Edges_set_list, ax=ax, edge_color="gray", width=1.1, alpha=1,style='solid',arrows=False)
        Edges_set.clear()
    if start_node and end_node:
        try:
            path = astar_path(G,start_node,end_node)
            if path is not None:
                #Tạo thành các cặp cạnh (Path là các Node đã đi)
                path_edges = list(zip(path, path[1:]))
                if Edges_set:
                    Edges_set_list=list(Edges_set)
                    nx.draw_networkx_edges(G, pos, edgelist=Edges_set_list, ax=ax, edge_color="blue", width=1, alpha=1,style='solid',arrows=False)
                nx.draw_networkx_edges(G, pos, edgelist=path_edges, ax=ax, edge_color="red", width=1, alpha=1,style='solid',arrows=False)
                distance = sum(haversine(G.nodes[path[i]]['lon'], G.nodes[path[i]]['lat'],
                                         G.nodes[path[i+1]]['lon'], G.nodes[path[i+1]]['lat'])
                               for i in range(len(path)-1))
                ax.set_title(f"Khoảng cách: {distance:.2f} km")
                print("Đã tìm thấy đường")
            else:
                ax.set_title("Không tìm thấy đường")
                print("Không tìm thấy đường")
        except nx.NetworkXNoPath:
            ax.set_title("Không tìm thấy đường")
            print("Không tìm thấy đường")
    plt.draw()
def on_click(event):
    if event.inaxes:
        closest_node = min(G.nodes, key=lambda n: (G.nodes[n]['lon'] - event.xdata)**2 + (G.nodes[n]['lat'] - event.ydata)**2)
        xlim = ax.get_xlim()
        ylim = ax.get_ylim()
        if event.button == MouseButton.LEFT:
            if not hasattr(on_click, "start_node"):
                on_click.start_node = closest_node
                print(f'Điểm bắt đầu: {closest_node}')
            else:
                end_node = closest_node
                print(f'Điểm kết thúc: {end_node}')
                DrawWay(on_click.start_node,end_node)
                del on_click.start_node
        elif event.button == MouseButton.RIGHT:
            G.nodes[closest_node]['lon'], G.nodes[closest_node]['lat'] = event.xdata, event.ydata
            pos[closest_node] = (event.xdata, event.ydata)
            update_plot(xlim=xlim, ylim=ylim)

pos = {node: (data['lon'], data['lat']) for node, data in G.nodes(data=True)}

fig, ax = plt.subplots(figsize=(10, 10))

update_plot()

fig.canvas.mpl_connect('button_press_event', on_click)

disconnect_zoom = zoom_factory(ax)
pan_handler = panhandler(fig)

plt.show()
