# Required imports
import numpy as np
import networkx as nx
from Boundaries import Boundaries
from Map import EPSILON

# Number of nodes expanded in the heuristic search (stored in a global variable to be updated from the heuristic functions)
NODES_EXPANDED = 0

def h1(current_node, objective_node) -> np.float32:
    """ First heuristic to implement 
        Manhattan distance * minimum cost of all nodes
    """
    global NODES_EXPANDED
    h = 0
    x1, y1 = current_node
    x2, y2 = objective_node
    h += abs(x1 - x2) + abs(y1 - y2)

    NODES_EXPANDED += 1
    return h * EPSILON

def h2(current_node, objective_node) -> np.float32:
    """ Second heuristic to implement 
        Euclidean distance * minimum cost of all nodes
    """
    global NODES_EXPANDED
    h = 0
    x1, y1 = current_node
    x2, y2 = objective_node
    h += np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

    NODES_EXPANDED += 1
    return h * EPSILON

def build_graph(detection_map: np.array, tolerance: np.float32) -> nx.DiGraph:
    """ Builds an adjacency graph (not an adjacency matrix) from the detection map """
    # The only possible connections from a point in space (now a node in the graph) are:
    #   -> Go up
    #   -> Go down
    #   -> Go left
    #   -> Go right
    # Not every point has always 4 possible neighbors
    
    height, width = detection_map.shape
    G = nx.DiGraph()

    for i in range(height):
        for j in range(width):
            current_node = (i, j)
            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                ni, nj = i + dx, j + dy
                if 0 <= ni < height and 0 <= nj < width:
                    neighbor_node = (ni, nj)
                    weight = detection_map[ni, nj]
                    G.add_edge(current_node, neighbor_node, weight=weight)
    return G

def discretize_coords(high_level_plan: np.array, boundaries: Boundaries, map_width: np.int32, map_height: np.int32) -> np.array:
    """ Converts coordiantes from (lat, lon) into (x, y) """
    lat_range = np.linspace(boundaries.min_lat, boundaries.max_lat, map_height)
    lon_range = np.linspace(boundaries.min_lon, boundaries.max_lon, map_width)
    
    discretized = []
    for lat, lon in high_level_plan:
        # Find nearest grid index
        x = np.argmin(np.abs(lat_range - lat))
        y = np.argmin(np.abs(lon_range - lon))
        discretized.append((x, y))
    
    return np.array(discretized)

def path_finding(G: nx.DiGraph,
                 heuristic_function,
                 locations: np.array, 
                 initial_location_index: np.int32, 
                 boundaries: Boundaries,
                 map_width: np.int32,
                 map_height: np.int32) -> tuple:
    """ Implementation of the main searching / path finding algorithm """
    global NODES_EXPANDED
    NODES_EXPANDED = 0  # Reset the counter
    
    # Convert coordinates to grid indices
    discrete_locs = discretize_coords(locations, boundaries, map_width, map_height)
    
    plan = []
    total_path = []
    
    current_index = initial_location_index
    
    for next_index in range(len(discrete_locs)):
        if next_index == current_index:
            continue
        start = tuple(discrete_locs[current_index])
        goal = tuple(discrete_locs[next_index])

        path = nx.astar_path(G, start, goal, heuristic=heuristic_function, weight='weight')
        plan.append([str(p) for p in path])
        total_path.extend(path if not total_path else path[1:])
        
        current_index = next_index
    
    return plan, NODES_EXPANDED

def compute_path_cost(G: nx.DiGraph, solution_plan: list) -> np.float32:
    """ Computes the total cost of the whole planning solution """
    total_cost = 0.0
    for segment in solution_plan:
        path = [eval(p) for p in segment]
        for u, v in zip(path[:-1], path[1:]):
            total_cost += G[u][v]['weight']
    return total_cost
