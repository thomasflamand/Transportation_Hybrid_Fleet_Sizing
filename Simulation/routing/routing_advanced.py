import osmnx as ox
import networkx as nx
from shapely.geometry import LineString, Point
from datetime import datetime, timedelta
import matplotlib.pyplot as plt
import warnings
import math
import gurobipy as gp
from gurobipy import GRB
import numpy as np
import pandas as pd

#==========================================

# Suppress warnings for cleaner output
warnings.filterwarnings("ignore")

if os.path.exists("manhattan.graphml"):
    G = ox.load_graphml("manhattan.graphml")
else:
    G = ox.graph_from_place("Manhattan, New York, USA", network_type='drive')
    ox.save_graphml(G, "manhattan.graphml")

#==========================================

class Stop:
    def __init__(self, request_id, lat, lon, is_pickup):
        """
        request_id: int or string. IMPORTANT: Pickup and Dropoff for the same 
                    passenger must share the same request_id.
        lat, lon: coordinates
        is_pickup: bool (True for pickup, False for dropoff)
        """
        self.request_id = request_id
        self.lat = lat
        self.lon = lon
        self.is_pickup = is_pickup

    def __repr__(self):
        type_str = "PICKUP" if self.is_pickup else "DROPOFF"
        return f"<Stop {self.request_id}: {type_str}>"

#==========================================

class Route:
    def __init__(self, G, stops, vehicle_start_coords, start_time, speed_mph):
        """
        G: The OSMnx graph (Lat/Lon)
        stops: List of Stop objects (Pickups and Dropoffs)
        vehicle_start_coords: Tuple (lat, lon) of the car's initial position
        start_time: datetime object
        speed_mph: float
        """
        self.G_latlon = G
        self.all_stops = stops
        self.start_coords = vehicle_start_coords
        self.start_time = start_time
        self.speed_mps = speed_mph * 0.44704 
        
        # State variables
        self.optimal_sequence = [] # List of Stop objects in order
        self.stop_arrival_times = [] # List of arrival datetimes for each stop
        self.route_line = None     # Shapely LineString of the full path
        self.total_distance = 0    # Meters
        self.G_proj = None         # Projected graph (meters)
        self.route_nodes = []      # List of Node IDs for the full path

        # Project graph immediately to enable Manhattan calc in meters
        self.G_proj = ox.project_graph(G)

    def _solve_tsp_gurobi(self):
        """
        Uses Gurobi to find optimal visit order using Manhattan Distance.
        Constraints: 
        1. Precedence (Pickup before Dropoff)
        2. Capacity <= 4
        3. Open TSP (Fixed Start, Open End - Do NOT return to start)
        """
        # 1. Prepare Nodes: [Start_Node] + [Stop_Objects...]
        # We create a temporary list where index 0 is vehicle start
        node_list = [{'lat': self.start_coords[0], 'lon': self.start_coords[1], 'type': 'start', 'id': 'start'}]
        
        # Map request_id to indices for precedence constraints
        req_to_pickup_idx = {}
        req_to_dropoff_idx = {}

        for i, stop in enumerate(self.all_stops):
            idx = i + 1 # Offset by 1 because 0 is start
            node_list.append({'lat': stop.lat, 'lon': stop.lon, 'obj': stop})
            
            if stop.is_pickup:
                req_to_pickup_idx[stop.request_id] = idx
            else:
                req_to_dropoff_idx[stop.request_id] = idx

        n = len(node_list)
        capacity = 4
        
        # 2. Project all points to obtain X/Y in meters for Manhattan Calc
        # We use the graph's CRS transformer
        import pyproj
        transformer = pyproj.Transformer.from_crs(self.G_latlon.graph['crs'], self.G_proj.graph['crs'], always_xy=True)
        
        coords_xy = []
        for node in node_list:
            x, y = transformer.transform(node['lon'], node['lat'])
            coords_xy.append((x, y))

        # 3. Calculate Cost Matrix (Manhattan Distance)
        dist = np.zeros((n, n))
        for i in range(n):
            for j in range(n):
                if i != j:
                    # Manhattan: |x1 - x2| + |y1 - y2|
                    dist[i, j] = abs(coords_xy[i][0] - coords_xy[j][0]) + abs(coords_xy[i][1] - coords_xy[j][1])

        # 4. Gurobi Model
        m = gp.Model("Rideshare_TSP")
        m.setParam('OutputFlag', 0)

        # Variables
        x = m.addVars(n, n, vtype=GRB.BINARY, name="x") # Edge selection
        u = m.addVars(n, vtype=GRB.CONTINUOUS, name="u") # Visit order
        l = m.addVars(n, vtype=GRB.CONTINUOUS, lb=0, ub=capacity, name="load") # Capacity

        # Objective: Minimize Total Distance
        m.setObjective(gp.quicksum(dist[i, j] * x[i, j] for i in range(n) for j in range(n)), GRB.MINIMIZE)

        # --- Open TSP Flow Constraints ---
        # 1. Start Node (0): Leaves exactly once. Enters 0 times.
        m.addConstr(gp.quicksum(x[0, j] for j in range(1, n)) == 1, name="Start_Leave")
        m.addConstr(gp.quicksum(x[i, 0] for i in range(n)) == 0, name="Start_Enter")

        # 2. Stop Nodes (1..N): Must be entered exactly once.
        for j in range(1, n):
            m.addConstr(gp.quicksum(x[i, j] for i in range(n) if i != j) == 1, name=f"Enter_{j}")

        # 3. Stop Nodes (1..N): Leave at most once (The last node leaves 0 times).
        for i in range(1, n):
            m.addConstr(gp.quicksum(x[i, j] for j in range(1, n) if i != j) <= 1, name=f"Leave_{i}")

        # 4. Total Edges: Must equal n-1 (Linear path visiting n nodes)
        m.addConstr(gp.quicksum(x[i, j] for i in range(n) for j in range(n)) == n - 1, name="Total_Edges")

        # --- Subtour Elimination & Timing ---
        # u[i] represents the order in sequence.
        for i in range(n):
            for j in range(n):
                if i != j:
                    m.addConstr(u[i] - u[j] + n * x[i, j] <= n - 1)

        # --- Precedence Constraints (Pickup time < Dropoff time) ---
        for req_id, p_idx in req_to_pickup_idx.items():
            if req_id in req_to_dropoff_idx:
                d_idx = req_to_dropoff_idx[req_id]
                m.addConstr(u[p_idx] <= u[d_idx] - 1)

        # --- Capacity Logic ---
        # Load at start is 0
        m.addConstr(l[0] == 0)
        
        for i in range(n):
            for j in range(1, n):
                if i != j:
                    # Determine change
                    change = 0
                    if j in req_to_pickup_idx.values(): change = 1
                    if j in req_to_dropoff_idx.values(): change = -1
                    
                    # If x[i,j]=1 -> l[j] = l[i] + change
                    M = capacity + 5
                    m.addConstr(l[j] >= l[i] + change - M * (1 - x[i, j]))
                    m.addConstr(l[j] <= l[i] + change + M * (1 - x[i, j]))

        m.optimize()

        if m.status == GRB.OPTIMAL:
            # Extract Sequence (Follow the path from 0)
            seq_indices = [0]
            curr = 0
            while True:
                found_next = False
                for j in range(n):
                    if curr != j and x[curr, j].X > 0.5:
                        seq_indices.append(j)
                        curr = j
                        found_next = True
                        break
                if not found_next:
                    # Reached the end of the path (node with no outgoing edges)
                    break
            
            # Convert indices back to Stop Objects
            # seq_indices[0] is vehicle start, so we skip it
            self.optimal_sequence = [node_list[i]['obj'] for i in seq_indices if i != 0]
            return True
        else:
            print("Infeasible or Failed to solve.")
            return False

    def generate_route(self):
        # 1. Solve Optimal Order
        success = self._solve_tsp_gurobi()
        if not success: return False
        
        # 2. Construct Full Path (Segment by Segment)
        full_route_points = []
        self.route_nodes = []
        self.stop_arrival_times = [] # Reset arrival times
        
        # Tracking variables for time calculation
        current_cumulative_dist = 0
        
        # Complete sequence: Start_Coords -> Stop1 -> Stop2 ... -> StopN
        # We create a temporary list of coordinate tuples for routing
        routing_sequence = [self.start_coords] + [(s.lat, s.lon) for s in self.optimal_sequence]

        print(f"Generating route for {len(routing_sequence)} points...")

        for i in range(len(routing_sequence) - 1):
            start_pt = routing_sequence[i]
            end_pt = routing_sequence[i+1]

            # Find nearest nodes on Projected Graph
            # Lat/Lon -> Project -> Nearest Node
            
            import pyproj
            transformer = pyproj.Transformer.from_crs(self.G_latlon.graph['crs'], self.G_proj.graph['crs'], always_xy=True)
            
            # Start
            sx, sy = transformer.transform(start_pt[1], start_pt[0])
            orig_node = ox.nearest_nodes(self.G_proj, X=sx, Y=sy)
            
            # End
            ex, ey = transformer.transform(end_pt[1], end_pt[0])
            dest_node = ox.nearest_nodes(self.G_proj, X=ex, Y=ey)

            try:
                # Calculate path for this segment
                segment_nodes = nx.shortest_path(self.G_proj, orig_node, dest_node, weight='length')
                
                # Append nodes
                if i > 0:
                    self.route_nodes.extend(segment_nodes[1:])
                else:
                    self.route_nodes.extend(segment_nodes)

                # Extract geometry points (x, y)
                segment_points = [(self.G_proj.nodes[n]['x'], self.G_proj.nodes[n]['y']) for n in segment_nodes]
                
                # -- Calculate distance and time for this specific segment --
                if len(segment_points) >= 2:
                    seg_len = LineString(segment_points).length
                else:
                    seg_len = 0
                
                current_cumulative_dist += seg_len
                
                # Calculate arrival time at the END of this segment (which is Stop i+1)
                arrival_time = self.start_time + timedelta(seconds=current_cumulative_dist / self.speed_mps)
                self.stop_arrival_times.append(arrival_time)
                # -----------------------------------------------------------

                if i > 0:
                    full_route_points.extend(segment_points[1:])
                else:
                    full_route_points.extend(segment_points)
                    
            except nx.NetworkXNoPath:
                print(f"Warning: No path found between sequence index {i} and {i+1}")
                # If path fails, we assume 0 distance/time addition to avoid index misalignment,
                # though the time will be inaccurate.
                self.stop_arrival_times.append(self.stop_arrival_times[-1] if self.stop_arrival_times else self.start_time)

        # 3. Create LineString
        if len(full_route_points) < 2: 
            full_route_points.append(full_route_points[0])

        self.route_line = LineString(full_route_points)
        self.total_distance = self.route_line.length
        return True

    def get_location_at_time(self, query_time):
        if not self.route_line: return self.start_coords
        
        time_elapsed = (query_time - self.start_time).total_seconds()
        if time_elapsed <= 0: return self.start_coords
        
        dist_traveled = time_elapsed * self.speed_mps
        
        # If finished trip
        if dist_traveled >= self.total_distance: 
            last = self.optimal_sequence[-1]
            return (last.lat, last.lon)

        # Interpolate
        point = self.route_line.interpolate(dist_traveled)
        
        # Project back to Lat/Lon
        import pyproj
        transformer = pyproj.Transformer.from_crs(
            self.G_proj.graph['crs'], "EPSG:4326", always_xy=True
        )
        lon, lat = transformer.transform(point.x, point.y)
        return (lat, lon)

    def get_total_duration(self):
        """ Returns total travel time in minutes """
        if self.total_distance == 0: return 0.0
        total_seconds = self.total_distance / self.speed_mps
        return round(total_seconds / 60, 2)

    def get_optimal_sequence(self):
        return self.optimal_sequence

    # important method
    def get_optimal_time(self):
        """
        Returns a pandas DataFrame with columns: 
        [Step, Request ID, Type, Latitude, Longitude, Arrival Time]
        """
        if not self.optimal_sequence or not self.stop_arrival_times:
            print("Route not generated yet.")
            return pd.DataFrame()

        data = []
        # zip ensures we pair the Stop object with its calculated Arrival Time
        for i, (stop, time) in enumerate(zip(self.optimal_sequence, self.stop_arrival_times)):
            data.append({
                'Step': i + 1,
                'Request ID': stop.request_id,
                'Type': 'PICKUP' if stop.is_pickup else 'DROPOFF',
                'Lat': stop.lat,
                'Lon': stop.lon,
                'Arrival Time': time.strftime('%H:%M:%S')
            })
        
        return pd.DataFrame(data)

    def plot_route_final(self):
        if not self.G_proj or not self.route_nodes: return

        fig, ax = ox.plot_graph_route(
            self.G_proj, self.route_nodes, 
            route_color="r", route_linewidth=4, node_size=0, 
            bgcolor='k', edge_color='#555555', show=False, close=False
        )
        
        # 1. Plot Minute Markers (Cyan)
        marker_x, marker_y = [], []
        total_minutes = int(self.get_total_duration())
        
        for i in range(1, total_minutes + 1):
            dist = i * 60 * self.speed_mps
            point = self.route_line.interpolate(dist)
            marker_x.append(point.x)
            marker_y.append(point.y)
            
        ax.scatter(marker_x, marker_y, c='cyan', s=30, zorder=5, label="1-min")

        # 2. Plot Stops (White numbers)
        import pyproj
        transformer = pyproj.Transformer.from_crs("EPSG:4326", self.G_proj.graph['crs'], always_xy=True)

        # Plot Start
        sx, sy = transformer.transform(self.start_coords[1], self.start_coords[0])
        ax.scatter(sx, sy, c='lime', s=100, zorder=10, label="Start")
        ax.text(sx, sy, "START", color='lime', fontweight='bold', zorder=12)

        # Plot Sequence
        for i, stop in enumerate(self.optimal_sequence):
            sx, sy = transformer.transform(stop.lon, stop.lat)
            color = 'white' if stop.is_pickup else 'yellow'
            ax.scatter(sx, sy, c=color, s=100, zorder=10, edgecolors='black')
            ax.text(sx, sy, f"{i+1}", color='black', fontweight='bold', ha='center', va='center', zorder=11)

        plt.legend()
        plt.show()

#==========================================
# # Request A: Times Square -> Central Park
# s1 = Stop(request_id='A', lat=40.7580, lon=-73.9855, is_pickup=True)
# s2 = Stop(request_id='A', lat=40.7829, lon=-73.9654, is_pickup=False)
# # Request B: Wall St -> Midtown
# s3 = Stop(request_id='B', lat=40.7060, lon=-74.0088, is_pickup=True)
# s4 = Stop(request_id='B', lat=40.7589, lon=-73.9750, is_pickup=False)
# stops_list = [s1, s2, s3, s4]

# # Vehicle Starts at Empire State
# vehicle_start = (40.748817, -73.985428)
# start_time = datetime.now()# 3. Initialize Route
# print("Initializing Route...")
# route = Route(G, stops_list, vehicle_start, start_time, speed_mph=20)# 4. Generate (Solves TSP + Routes)
# print("Solving Optimal Sequence & Generating Path...")
# route.generate_route()# 5. Outputs
# print("\n--- RESULTS ---")
# print("Optimal Sequence:", route.get_optimal_sequence())
# print("Total Duration:", route.get_total_duration(), "mins")

# print("\n--- STOP TIMETABLE ---")
# df = route.get_optimal_time()
# print(df)# Query Location
# query_t = start_time + timedelta(minutes=10)
# loc = route.get_location_at_time(query_t)
# print(f"\nLocation at +10 mins: {loc}")# 6. Plot
# route.plot_route_final()
