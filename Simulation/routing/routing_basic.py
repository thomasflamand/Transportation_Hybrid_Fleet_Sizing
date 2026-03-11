import osmnx as ox
import networkx as nx
from shapely.geometry import LineString
from datetime import datetime, timedelta
import matplotlib.pyplot as plt
import warnings
import os

warnings.filterwarnings("ignore")

class Route:
    def __init__(self, start_coords, end_coords, start_time, speed_mph):
        self.start_coords = start_coords
        self.end_coords = end_coords
        self.start_time = start_time
        self.speed_mps = speed_mph * 0.44704 
        
        self.route_line = None      
        self.total_distance = 0     
        self.G_proj = None          
        self.route_nodes = None
        
    def generate_route(self, preloaded_graph=None, dist_buffer=2000):
        # 1. Get Map
        if preloaded_graph is not None:
            G = preloaded_graph
        else:
            mid_lat = (self.start_coords[0] + self.end_coords[0]) / 2
            mid_lon = (self.start_coords[1] + self.end_coords[1]) / 2
            G = ox.graph_from_point((mid_lat, mid_lon), dist=dist_buffer, network_type='drive')
        
        # 2. Find Nodes (Lat/Lon)
        orig_node = ox.nearest_nodes(G, X=self.start_coords[1], Y=self.start_coords[0])
        dest_node = ox.nearest_nodes(G, X=self.end_coords[1], Y=self.end_coords[0])
        
        # 3. Project to Meters
        self.G_proj = ox.project_graph(G)
        
        # 4. Calculate Path
        try:
            self.route_nodes = nx.shortest_path(self.G_proj, orig_node, dest_node, weight='length')
            
            # Build Geometry
            route_points = [(self.G_proj.nodes[n]['x'], self.G_proj.nodes[n]['y']) for n in self.route_nodes]
            if len(route_points) < 2: route_points.append(route_points[0]) 

            self.route_line = LineString(route_points)
            self.total_distance = self.route_line.length
            return True
            
        except nx.NetworkXNoPath:
            return False

    def get_location_at_time(self, query_time):
        if not self.route_line: return self.start_coords
        time_elapsed = (query_time - self.start_time).total_seconds()
        if time_elapsed <= 0: return self.start_coords
        dist_traveled = time_elapsed * self.speed_mps
        if dist_traveled >= self.total_distance: return self.end_coords

        point = self.route_line.interpolate(dist_traveled)
        
        import pyproj
        transformer = pyproj.Transformer.from_crs(
            self.G_proj.graph['crs'], "EPSG:4326", always_xy=True
        )
        lon, lat = transformer.transform(point.x, point.y)
        return (lat, lon)

    # --- NEW METHOD ---
    def get_total_duration(self):
        """ Returns total travel time in minutes """
        if self.total_distance == 0: return 0.0
        
        total_seconds = self.total_distance / self.speed_mps
        return round(total_seconds / 60, 2)

    def plot_route_with_minutes(self):
        if not self.G_proj or not self.route_nodes: return

        fig, ax = ox.plot_graph_route(
            self.G_proj, self.route_nodes, 
            route_color="r", route_linewidth=4, node_size=0, 
            bgcolor='k', edge_color='#555555', show=False, close=False
        )
        
        marker_x, marker_y = [], []
        total_minutes = int((self.total_distance / self.speed_mps) // 60)
        
        for i in range(1, total_minutes + 1):
            point = self.route_line.interpolate(i * 60 * self.speed_mps)
            marker_x.append(point.x)
            marker_y.append(point.y)

        ax.scatter(marker_x, marker_y, c='cyan', s=60, zorder=10, label="1-min markers")
        ax.set_title(f"Route: {len(marker_x)} mins", color="white")
        plt.show()

if os.path.exists("manhattan.graphml"):
    G = ox.load_graphml("manhattan.graphml")
else:
    G = ox.graph_from_place("Manhattan, New York, USA", network_type='drive')
    ox.save_graphml(G, "manhattan.graphml")

#==========================================
# start = (40.808490, -73.963427) 
# end = (40.716971, -73.998195)   
# trip_start_time = datetime.now()
# route = Route(start, end, trip_start_time, speed_mph=15)
# route.generate_route(preloaded_graph=G)
# check_time = trip_start_time + timedelta(minutes=3)
# lat, lon = route.get_location_at_time(check_time)
# print(f"At {check_time.strftime('%H:%M:%S')}, car is at: {lat}, {lon}")
# total_minutes = route.get_total_duration()
# print(f"Total Trip Duration: {total_minutes} minutes")
# # route.plot_route_with_minutes()