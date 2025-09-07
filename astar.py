import math
import heapq
from typing import Dict, List, Tuple, Set, Optional


class City:
    """Represents a city with its coordinates and label."""
    def __init__(self, label: str, x: float, y: float):
        self.label = label
        self.x = x
        self.y = y
    
    def distance_to(self, other: 'City') -> float:
        """Calculate Euclidean distance to another city."""
        return math.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)
    
    def __repr__(self):
        return f"{self.label}({self.x},{self.y})"


class Graph:
    """Represents the graph of cities and roads."""
    def __init__(self):
        self.cities: Dict[str, City] = {}
        self.edges: Dict[str, Set[str]] = {}
    
    def add_city(self, city: City):
        """Add a city to the graph."""
        self.cities[city.label] = city
        if city.label not in self.edges:
            self.edges[city.label] = set()
    
    def add_edge(self, city1: str, city2: str):
        """Add an edge between two cities (bidirectional)."""
        if city1 not in self.edges:
            self.edges[city1] = set()
        if city2 not in self.edges:
            self.edges[city2] = set()
        
        self.edges[city1].add(city2)
        self.edges[city2].add(city1)
    
    def get_neighbors(self, city_label: str) -> Set[str]:
        """Get all neighboring cities."""
        return self.edges.get(city_label, set())


class AStarNode:
    """Represents a node in the A* search."""
    def __init__(self, city_label: str, g_cost: float, h_cost: float, parent: Optional['AStarNode'] = None):
        self.city_label = city_label
        self.g_cost = g_cost  # Cost from start to this node
        self.h_cost = h_cost  # Heuristic cost from this node to goal
        self.f_cost = g_cost + h_cost  # Total cost
        self.parent = parent
    
    def __lt__(self, other):
        """For priority queue ordering."""
        return self.f_cost < other.f_cost


def read_graph_from_file(filename: str) -> Tuple[Graph, str, str]:
    """
    Read graph data from file and return graph, start city, and goal city.
    
    Returns:
        Tuple of (graph, start_city_label, goal_city_label)
    """
    graph = Graph()
    
    with open(filename, 'r') as file:
        lines = [line.strip() for line in file.readlines() if line.strip()]
    
    # Read number of cities
    num_cities = int(lines[0])
    
    # Read cities
    city_labels = []
    for i in range(1, num_cities + 1):
        parts = lines[i].split(',')
        label = parts[0]
        x = float(parts[1])
        y = float(parts[2])
        
        city = City(label, x, y)
        graph.add_city(city)
        city_labels.append(label)
    
    # Read edges
    for i in range(num_cities + 1, len(lines)):
        parts = lines[i].split(',')
        city1 = parts[0]
        city2 = parts[1]
        graph.add_edge(city1, city2)
    
    # First city is start, last city is goal
    start_city = city_labels[0]
    goal_city = city_labels[-1]
    
    return graph, start_city, goal_city


def astar_search(graph: Graph, start: str, goal: str) -> Tuple[List[str], List[str]]:
    """
    Perform A* search to find shortest path from start to goal.
    
    Returns:
        Tuple of (shortest_path, exploration_order)
    """
    if start not in graph.cities or goal not in graph.cities:
        raise ValueError("Start or goal city not found in graph")
    
    # Priority queue: (f_cost, node)
    open_set = []
    # Set of city labels that have been explored
    closed_set: Set[str] = set()
    # Dictionary to track best g_cost for each city
    g_costs: Dict[str, float] = {start: 0}
    # Dictionary to track nodes for path reconstruction
    came_from: Dict[str, AStarNode] = {}
    
    # Initialize with start node
    start_city = graph.cities[start]
    goal_city = graph.cities[goal]
    h_cost = start_city.distance_to(goal_city)
    start_node = AStarNode(start, 0, h_cost)
    heapq.heappush(open_set, (start_node.f_cost, start_node))
    
    exploration_order = []
    
    while open_set:
        # Get node with lowest f_cost
        current_f_cost, current_node = heapq.heappop(open_set)
        
        # Skip if already explored
        if current_node.city_label in closed_set:
            continue
        
        # Mark as explored
        closed_set.add(current_node.city_label)
        exploration_order.append(current_node.city_label)
        
        # Check if we reached the goal
        if current_node.city_label == goal:
            # Reconstruct path
            path = []
            node = current_node
            while node is not None:
                path.append(node.city_label)
                node = node.parent
            path.reverse()
            return path, exploration_order
        
        # Explore neighbors
        current_city = graph.cities[current_node.city_label]
        for neighbor_label in graph.get_neighbors(current_node.city_label):
            if neighbor_label in closed_set:
                continue
            
            neighbor_city = graph.cities[neighbor_label]
            # Calculate edge cost (Euclidean distance)
            edge_cost = current_city.distance_to(neighbor_city)
            tentative_g_cost = current_node.g_cost + edge_cost
            
            # Check if this is a better path to the neighbor
            if neighbor_label not in g_costs or tentative_g_cost < g_costs[neighbor_label]:
                g_costs[neighbor_label] = tentative_g_cost
                h_cost = neighbor_city.distance_to(goal_city)
                neighbor_node = AStarNode(neighbor_label, tentative_g_cost, h_cost, current_node)
                heapq.heappush(open_set, (neighbor_node.f_cost, neighbor_node))
    
    # No path found
    return [], exploration_order


def main():
    """Main function to run the A* algorithm."""
    import sys
    
    if len(sys.argv) != 2:
        print("Usage: python astar.py <data_file>")
        sys.exit(1)
    
    filename = sys.argv[1]
    
    try:
        # Read graph from file
        graph, start_city, goal_city = read_graph_from_file(filename)

        # Run A* search
        shortest_path, exploration_order = astar_search(graph, start_city, goal_city)
        
        # Print results
        if shortest_path:
            print("Shortest path found:")
            print(" -> ".join(shortest_path))
            

        
        print()
        print("Exploration order:")
        print(" -> ".join(exploration_order))
        
    except FileNotFoundError:
        print(f"Error: File '{filename}' not found.")
        sys.exit(1)
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
