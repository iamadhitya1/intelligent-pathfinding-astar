"""
Intelligent Pathfinding using the A* Search Algorithm
----------------------------------------------------
Finds the shortest path between two points on a 2D grid with obstacles.

A* explores nodes in order of f(n) = g(n) + h(n):
    g(n) - actual cost from the start to n
    h(n) - heuristic estimate from n to the goal (Manhattan distance)

Movement is 4-directional (up/down/left/right) at a uniform cost of 1.
Manhattan distance is admissible and consistent for this movement model,
so A* is guaranteed to return an optimal path. (If diagonal moves were
added, the heuristic would need to change to octile/Euclidean distance.)
"""

import heapq
from itertools import count

import matplotlib.pyplot as plt


# --- 1. Define the grid environment ---
class Grid:
    def __init__(self, width, height, start, goal, obstacles):
        self.width = width
        self.height = height
        self.start = start
        self.goal = goal
        self.obstacles = set(obstacles)

    def is_valid(self, node):
        x, y = node
        return (
            0 <= x < self.width
            and 0 <= y < self.height
            and node not in self.obstacles
        )

    def get_neighbors(self, node):
        x, y = node
        candidates = [(x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)]
        return [n for n in candidates if self.is_valid(n)]


# --- 2. A* search ---
class AStar:
    def __init__(self, grid):
        self.grid = grid
        self.start = grid.start
        self.goal = grid.goal

    @staticmethod
    def manhattan_distance(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def find_path(self):
        # Priority queue holds (f_score, tie_breaker, node). The counter keeps
        # entries strictly ordered so two nodes with equal f never get compared.
        counter = count()
        open_set = [(self.manhattan_distance(self.start, self.goal),
                     next(counter), self.start)]

        came_from = {}
        g_score = {self.start: 0}          # default is treated as infinity
        closed = set()                     # nodes already expanded

        while open_set:
            _, _, current = heapq.heappop(open_set)

            # Skip stale queue entries left behind by an earlier, worse path.
            if current in closed:
                continue
            closed.add(current)

            if current == self.goal:
                return self.reconstruct_path(came_from, current)

            for neighbor in self.grid.get_neighbors(current):
                if neighbor in closed:
                    continue
                tentative_g = g_score[current] + 1
                if tentative_g < g_score.get(neighbor, float("inf")):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + self.manhattan_distance(neighbor, self.goal)
                    heapq.heappush(open_set, (f_score, next(counter), neighbor))

        return None  # no path exists


# --- 3. Visualization ---
def visualize_path(grid, path, save_as="Figure_1.png"):
    fig, ax = plt.subplots(figsize=(8, 8))

    for (ox, oy) in grid.obstacles:
        ax.add_patch(plt.Rectangle((ox, oy), 1, 1, color="gray"))

    if path:
        xs = [n[0] + 0.5 for n in path]
        ys = [n[1] + 0.5 for n in path]
        ax.plot(xs, ys, color="green", linewidth=2, marker="o", markersize=4,
                label="path")

    ax.plot(grid.start[0] + 0.5, grid.start[1] + 0.5, "o", markersize=14,
            color="blue", label="start")
    ax.plot(grid.goal[0] + 0.5, grid.goal[1] + 0.5, "o", markersize=14,
            color="red", label="goal")

    ax.set_xticks(range(grid.width + 1))
    ax.set_yticks(range(grid.height + 1))
    ax.set_xlim(0, grid.width)
    ax.set_ylim(0, grid.height)
    ax.set_aspect("equal")
    ax.grid(True, linewidth=0.5)
    ax.set_title("A* Pathfinding Simulation")
    ax.legend(loc="upper left")

    fig.savefig(save_as, dpi=120, bbox_inches="tight")
    plt.show()


# --- Main execution ---
if __name__ == "__main__":
    start_pos = (1, 1)
    goal_pos = (8, 8)
    obstacles_list = [
        (3, 3), (4, 3), (5, 3), (6, 3),   # horizontal wall
        (6, 4), (6, 5), (6, 6),           # vertical wall
    ]

    grid = Grid(10, 10, start_pos, goal_pos, obstacles_list)

    print("Running A* search...")
    path = AStar(grid).find_path()

    if path:
        print(f"Path found ({len(path)} steps, cost {len(path) - 1}):")
        print(path)
        visualize_path(grid, path)
    else:
        print("No path found.")
