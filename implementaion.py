import numpy as np
from collections import deque
import heapq
import random
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap


grid_size = 128
start = [0, 0]
goal = [127, 127]



def move_robot(current_pos, direction):
    directions = {'up': [-1, 0], 'down': [1, 0], 'left': [0, -1], 'right': [0, 1]}
    new_pos = [current_pos[0] + directions[direction][0], current_pos[1] + directions[direction][1]]
    return new_pos

custom_cmap = ListedColormap(['white', 'black', 'green', 'red', 'blue', 'yellow', 'purple'])


def dfs(start, goal, grid):
    stack = [start]
    visited = set()
    parent = {}
    
    while stack:
        current = stack.pop()
        if tuple(current) == tuple(goal):
            return parent
        
        visited.add(tuple(current))
        
        for direction in ['up', 'down', 'left', 'right']:
            next_pos = move_robot(current, direction)
            if 0 <= next_pos[0] < grid_size and 0 <= next_pos[1] < grid_size:
                if tuple(next_pos) not in visited and grid[next_pos[0]][next_pos[1]] != 1:
                    stack.append(next_pos)
                    parent[tuple(next_pos)] = current

def bfs(start, goal, grid):
    queue = deque([start])
    visited = set()
    parent = {}
    
    while queue:
        current = queue.popleft()
        if tuple(current) == tuple(goal):
            return parent
        
        visited.add(tuple(current))
        
        for direction in ['up', 'down', 'left', 'right']:
            next_pos = move_robot(current, direction)
            if 0 <= next_pos[0] < grid_size and 0 <= next_pos[1] < grid_size:
                if tuple(next_pos) not in visited and grid[next_pos[0]][next_pos[1]] != 1:
                    queue.append(next_pos)
                    visited.add(tuple(next_pos))
                    parent[tuple(next_pos)] = current

def dijkstra(start, goal, grid):
    pq = [(0, start)]
    distances = {tuple(start): 0}
    parent = {}
    
    while pq:
        current_distance, current = heapq.heappop(pq)
        
        if tuple(current) == tuple(goal):
            return parent
        
        for direction in ['up', 'down', 'left', 'right']:
            next_pos = move_robot(current, direction)
            if 0 <= next_pos[0] < grid_size and 0 <= next_pos[1] < grid_size:
                new_distance = current_distance + 1
                if new_distance < distances.get(tuple(next_pos), float('inf')) and grid[next_pos[0]][next_pos[1]] != 1:
                    distances[tuple(next_pos)] = new_distance
                    heapq.heappush(pq, (new_distance, next_pos))
                    parent[tuple(next_pos)] = current

def random_planner(start, goal, grid, max_iterations=10000):
    current = start
    iterations = 0
    parent = {}
    
    while iterations < max_iterations:
        if tuple(current) == tuple(goal):
            return parent
        
        direction = random.choice(['up', 'down', 'left', 'right'])
        next_pos = move_robot(current, direction)
        
        if 0 <= next_pos[0] < grid_size and 0 <= next_pos[1] < grid_size:
            if grid[next_pos[0]][next_pos[1]] != 1:
                current = next_pos
                parent[tuple(next_pos)] = current
        
        iterations += 1

def find_and_visualize_path(start, goal, parent, algorithm_name, path_value):
    temp_grid = grid.copy()
    if parent:
        path = []
        current = goal
        while current != start:
            path.append(current)
            current = parent[tuple(current)]
        path.append(start)
        
        for cell in path:
            temp_grid[cell[0]][cell[1]] = path_value 
        
        plt.imshow(np.flipud(temp_grid), cmap=custom_cmap) 
        plt.title(f'Path found by {algorithm_name}')
        plt.colorbar(ticks=[0, 1, 2, 4, 5, 6, 7])
        plt.show()
    else:
        print(f"No path found by {algorithm_name}")


for density in np.linspace(0, 0.75, 16):

    grid = np.zeros((grid_size, grid_size))
    obstacle_positions = np.random.rand(grid_size, grid_size) < density
    grid[obstacle_positions] = 1

    parent_dfs = dfs(start, goal, grid.copy())
    parent_bfs = bfs(start, goal, grid.copy())
    parent_dijkstra = dijkstra(start, goal, grid.copy())
    parent_random = random_planner(start, goal, grid.copy())

    if density in [0.1, 0.4, 0.7]:
        find_and_visualize_path(start, goal, parent_dfs, f"DFS at {int(density * 100)}% density", 4)
        find_and_visualize_path(start, goal, parent_bfs, f"BFS at {int(density * 100)}% density", 5)
        find_and_visualize_path(start, goal, parent_dijkstra, f"Dijkstra's at {int(density * 100)}% density", 6)
        find_and_visualize_path(start, goal, parent_random, f"Random Planner at {int(density * 100)}% density", 7)

def count_iterations(parent):
    if parent is None:
        return None  
    
    count = 0
    current = goal
    while current != start:
        count += 1
        current = parent[tuple(current)]
    return count

dfs_iterations = []
bfs_iterations = []
dijkstra_iterations = []
random_iterations = []


for density in np.linspace(0, 0.75, 16):

    grid = np.zeros((grid_size, grid_size))
    obstacle_positions = np.random.rand(grid_size, grid_size) < density
    grid[obstacle_positions] = 1
    

    parent_dfs = dfs(start, goal, grid.copy())
    parent_bfs = bfs(start, goal, grid.copy())
    parent_dijkstra = dijkstra(start, goal, grid.copy())
    parent_random = random_planner(start, goal, grid.copy())

    dfs_iterations.append(count_iterations(parent_dfs) if parent_dfs is not None else None)
    bfs_iterations.append(count_iterations(parent_bfs) if parent_bfs is not None else None)
    dijkstra_iterations.append(count_iterations(parent_dijkstra) if parent_dijkstra is not None else None)
    random_iterations.append(count_iterations(parent_random) if parent_random is not None else None)

if density in [0.1, 0.4, 0.7]:
    if parent_dfs:
        find_and_visualize_path(start, goal, parent_dfs, f"DFS at {int(density * 100)}% density", 4)
    else:
        print(f"No path found by DFS at {int(density * 100)}% density")
    
    if parent_bfs:
        find_and_visualize_path(start, goal, parent_bfs, f"BFS at {int(density * 100)}% density", 5)
    else:
        print(f"No path found by BFS at {int(density * 100)}% density")
        
    if parent_dijkstra:
        find_and_visualize_path(start, goal, parent_dijkstra, f"Dijkstra's at {int(density * 100)}% density", 6)
    else:
        print(f"No path found by Dijkstra's at {int(density * 100)}% density")
        
    if parent_random:
        find_and_visualize_path(start, goal, parent_random, f"Random Planner at {int(density * 100)}% density", 7)
    else:
        print(f"No path found by Random Planner at {int(density * 100)}% density")



dfs_iterations = [-1 if x is None else x for x in dfs_iterations]
bfs_iterations = [-1 if x is None else x for x in bfs_iterations]
dijkstra_iterations = [-1 if x is None else x for x in dijkstra_iterations]
random_iterations = [-1 if x is None else x for x in random_iterations]


plt.figure()
plt.plot(np.linspace(0, 0.75, 16), dfs_iterations, label='DFS')
plt.plot(np.linspace(0, 0.75, 16), bfs_iterations, label='BFS')
plt.plot(np.linspace(0, 0.75, 16), dijkstra_iterations, label="Dijkstra's")
plt.plot(np.linspace(0, 0.75, 16), random_iterations, label='Random Planner')
plt.xlabel('Obstacle Density')
plt.ylabel('Number of Iterations to Reach Goal')
plt.legend()
plt.show()
