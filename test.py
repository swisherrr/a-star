#!/usr/bin/env python3
"""
Test script that uses the A* algorithm implementation from astar.py
to find the shortest path in test_example.data
"""

from astar import read_graph_from_file, astar_search

def main():
    """Test the A* algorithm with test_example.data"""
    filename = "test_example.data"
    
    print("A* Algorithm Test Script")
    print("=" * 50)
    print(f"Testing with file: {filename}")
    print()
    
    try:
        # Read the graph from the data file
        graph, start_city, goal_city = read_graph_from_file(filename)
        
        print(f"Graph loaded successfully!")
        print(f"Start city: {start_city}")
        print(f"Goal city: {goal_city}")
        print(f"Number of cities: {len(graph.cities)}")
        print()
        
        # Display all cities and their coordinates
        print("Cities in the graph:")
        for label, city in graph.cities.items():
            print(f"  {city.label}: ({city.x}, {city.y})")
        print()
        
        # Display all edges
        print("Edges in the graph:")
        for city_label, neighbors in graph.edges.items():
            for neighbor in neighbors:
                if city_label < neighbor:  # Avoid printing duplicate edges
                    city1 = graph.cities[city_label]
                    city2 = graph.cities[neighbor]
                    distance = city1.distance_to(city2)
                    print(f"  {city_label} <-> {neighbor}: distance = {distance:.2f}")
        print()
        
        # Run A* search
        print("Running A* search...")
        shortest_path, exploration_order = astar_search(graph, start_city, goal_city)
        
        # Display results
        if shortest_path:
            print("✅ Shortest path found:")
            print("   " + " -> ".join(shortest_path))
            
            # Calculate and display total distance
            total_distance = 0
            for i in range(len(shortest_path) - 1):
                city1 = graph.cities[shortest_path[i]]
                city2 = graph.cities[shortest_path[i + 1]]
                segment_distance = city1.distance_to(city2)
                total_distance += segment_distance
                print(f"   {shortest_path[i]} to {shortest_path[i+1]}: {segment_distance:.2f}")
            
            print(f"\n   Total distance: {total_distance:.2f}")
        else:
            print("❌ No path found from start to goal!")
        
        print()
        print("Exploration order:")
        print("   " + " -> ".join(exploration_order))
        
        # Verify the expected result
        expected_path = ["A", "B", "E"]
        if shortest_path == expected_path:
            print("\n✅ SUCCESS: Found the expected shortest path A -> B -> E!")
        else:
            print(f"\n❌ UNEXPECTED: Expected A -> B -> E, but got {' -> '.join(shortest_path)}")
        
    except FileNotFoundError:
        print(f"❌ Error: File '{filename}' not found.")
    except Exception as e:
        print(f"❌ Error: {e}")

if __name__ == "__main__":
    main()
