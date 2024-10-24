# A* Search Algorithm

## Overview:
A* (A-Star) is a widely-used pathfinding and graph traversal algorithm that combines the benefits of Dijkstra's algorithm and heuristic-driven approaches. It is both efficient and optimal, making it popular for use in navigation, AI in games, and robotics. 


## Use Cases
* **Robotics:** Used in robotic motion planning to find optimal paths for mobile robots.
* **Maps and GPS systems:** Used to find the shortest or fastest route between two points.
* **Game Development:** Employed for NPC navigation to find the shortest paths in game environments.
* **Graph traversal:** Used in logistics, networking, and other domains to find the least-cost path in weighted graphs.


## Basic Idea of A* Search Algorithm:
* A (A-Star) Search* finds the shortest path from a start node to a goal node in a graph or grid.
* It checks for neighbors of the current node, and maintains a priority queue (open list).
    * The priority queue is sorted by evaluation function f = g + h, where:
        * g is the actual cost from the start to the current node.
        * h is the heuristic (estimated) cost from the current node to the goal node (often calculated using Manhattan or Euclidean distance).
* The algorithm iteratively selects the node with the lowest f value to explore, expands its neighbors, and continues this process until the goal is reached.


## pseudocode
```
Search(grid, initial_point, goal_point):
1. Initialize an empty list of open nodes.
2. Create the start node with:
   - x, y coordinates from initial_point.
   - g_value = 0 (cost from start).
   - h_value = heuristic estimate to goal.
3. Add the start node to the open list.
4. While the open list is not empty:
    - Sort the open list by f_value (f = g + h).
    - Pop the node with the lowest f_value (current node).
    - Mark the current node in the grid as part of the path.
    - If the current node is the goal:
        - Return the grid with the path.
    - Otherwise, expand the current node's neighbors:
        - For each neighbor, check if it's valid (within the grid, not an obstacle, not closed).
        - If valid, calculate g and h values and add the neighbor to the open list.
        - Mark the neighbor as closed.
5. If the open list is empty and no path is found, return "No path found."
```

## Time and Space Complexity:
* **Time Complexity:** O(b^d), where b is the branching factor (number of successors) and d is the depth of the solution.
* **Space Complexity:** O(b^d), as A* stores all generated nodes in memory.


## Advantages:
* **Optimal:** A* always finds the shortest path if the heuristic is admissible (never overestimates the true cost).
* **Complete:** If a solution exists, A* is guaranteed to find it.
* **Flexible:** Different heuristics can be used depending on the use case (e.g., Euclidean distance, Manhattan distance).
* **Combines Benefits:** A* combines the guaranteed optimality of Dijkstra's Algorithm (guaranteed optimal solution)   with Greedy Best-First Search (efficient exploration).

## Disadvantages:
* **Memory usage:** A* requires significant memory to keep the open list, especially for large or dense graphs.
* **Depends on heuristic quality:** If the heuristic is poor or overestimates the cost to the goal, A* can lose its efficiency.


