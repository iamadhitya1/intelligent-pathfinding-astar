import heapq
import matplotlib.pyplot as plt

# --- 1. Define the Grid Environment ---
class Grid:
    def __init__(self, width, height, start, goal, obstacles):
        self.width = width
        self.height = height
        self.start = start
        self.goal = goal
        self.obstacles = obstacles
        # Create a list of all possible nodes
        self.nodes = [[(x, y) for y in range(height)] for x in range(width)]

    def is_valid(self, node):
        x, y = node
        # Check if node is within bounds and not an obstacle
        return 0 <= x < self.width and 0 <= y < self.height and node not in self.obstacles

    def get_neighbors(self, node):
        x, y = node
        # Define movements: Right, Left, Up, Down
        neighbors = [(x+1, y), (x-1, y), (x, y+1), (x, y-1)]
        valid_neighbors = [n for n in neighbors if self.is_valid(n)]
        return valid_neighbors

# --- 2. Implement A* Search Algorithm ---
class AStar:
    def __init__(self, grid):
        self.grid = grid
        self.start = grid.start
        self.goal = grid.goal

    def manhattan_distance(self, node1, node2):
        # Heuristic: |x1 - x2| + |y1 - y2|
        return abs(node1[0] - node2[0]) + abs(node1[1] - node2[1])

    def reconstruct_path(self, came_from, current):
        path = []
        while current in came_from:
            path.append(current)
            current = came_from[current]
        path.append(self.start)
        path.reverse()
        return path

    def find_path(self):
        open_set = []
        # Priority Queue: stores (f_score, node)
        heapq.heappush(open_set, (0, self.start))
        
        came_from = {}
        
        # g_score: Cost from start to current node
        # Initialize all to infinity
        g_score = {node: float('inf') for row in self.grid.nodes for node in row}
        g_score[self.start] = 0
        
        # f_score: Estimated total cost (g_score + heuristic)
        f_score = {node: float('inf') for row in self.grid.nodes for node in row}
        f_score[self.start] = self.manhattan_distance(self.start, self.goal)

        while open_set:
            # Pop node with lowest f_score
            current_f, current = heapq.heappop(open_set)

            if current == self.goal:
                return self.reconstruct_path(came_from, current)

            for neighbor in self.grid.get_neighbors(current):
                # Tentative g_score is current g + 1 (assuming cost of 1 per step)
                tentative_g = g_score[current] + 1

                if tentative_g < g_score[neighbor]:
                    # This path to neighbor is better than any previous one
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.manhattan_distance(neighbor, self.goal)
                    
                    # Add to priority queue if not already there (or update it)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return None # No path found

# --- 3. Visualization ---
def visualize_path(grid, path):
    plt.figure(figsize=(8, 8))
    
    # Draw the grid
    for x in range(grid.width):
        for y in range(grid.height):
            if (x, y) in grid.obstacles:
                # Draw Obstacle (Gray)
                plt.fill_between([x, x+1], [y, y], [y+1, y+1], color='gray')
            else:
                # Draw Empty Space (White with borders)
                plt.plot([x, x+1, x+1, x, x], [y, y, y+1, y+1, y], color='black', linewidth=0.5)

    # Draw the Path
    if path:
        for node in path:
            # Mark path nodes with 'X'
            plt.text(node[0] + 0.5, node[1] + 0.5, 'X', ha='center', va='center', fontsize=12, color='green', weight='bold')

    # Draw Start (Blue Circle) and Goal (Red Circle)
    plt.plot(grid.start[0] + 0.5, grid.start[1] + 0.5, marker='o', markersize=15, color='blue', label='Start')
    plt.plot(grid.goal[0] + 0.5, grid.goal[1] + 0.5, marker='o', markersize=15, color='red', label='Goal')

    plt.xlim(0, grid.width)
    plt.ylim(0, grid.height)
    plt.grid(False)
    plt.title('A* Pathfinding Simulation')
    plt.legend(loc='upper right')
    plt.show()

# --- Main Execution ---
if __name__ == "__main__":
    # Define Start, Goal, and Obstacles
    start_pos = (1, 1)
    goal_pos = (8, 8)
    
    # Let's create a "wall" of obstacles to force the algorithm to go around
    obstacles_list = [
        (3, 3), (4, 3), (5, 3), (6, 3), # Horizontal wall
        (6, 4), (6, 5), (6, 6)          # Vertical wall
    ]

    # Initialize Environment
    print("Initializing Grid...")
    my_grid = Grid(10, 10, start_pos, goal_pos, obstacles_list)
    
    # Run A* Algorithm
    print("Running A* Search...")
    astar_solver = AStar(my_grid)
    final_path = astar_solver.find_path()

    if final_path:
        print(f"Path Found: {final_path}")
        print("Displaying visualization...")
        visualize_path(my_grid, final_path)
    else:
        print("No path found.")