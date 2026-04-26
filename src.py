import heapq

ROOM_GRID = [
    [0, 0, 0, 0, 0, 0, 0],
    [0, 1, 1, 1, 1, 1, 0],  
    [0, 1, 0, 0, 0, 0, 0],  
    [0, 1, 0, 1, 1, 1, 1],  
    [0, 0, 0, 0, 0, 0, 0]
]
START_POS = (0, 0)

def get_neighbors(pos, grid):
    """Returns valid adjacent tiles (Up, Down, Left, Right)."""
    neighbors = []
    rows, cols = len(grid), len(grid[0])
    for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
        nx, ny = pos[0] + dx, pos[1] + dy
        if 0 <= nx < rows and 0 <= ny < cols and grid[nx][ny] != 1:
            neighbors.append((nx, ny))
    return neighbors

def dist(p1, p2):
    """ distance: |x1 - x2| + |y1 - y2|"""
    return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])

def a_star_path(grid, start, goal):
    pq = [(0, start)]
    came_from = {start: None}
    cost_so_far = {start: 0}
    while pq:
        _, current = heapq.heappop(pq)
        if current == goal:
            path = []
            while current:
                path.append(current)
                current = came_from[current]
            return path[::-1]
        for n in get_neighbors(current, grid):
            new_cost = cost_so_far[current] + 1
            if n not in cost_so_far or new_cost < cost_so_far[n]:
                cost_so_far[n] = new_cost
                heapq.heappush(pq, (new_cost + dist(n, goal), n))
                came_from[n] = current
    return None

def run_simulation(strategy):
    grid = [row[:] for row in ROOM_GRID]
    current_pos = START_POS
    visited = {START_POS}
    total_steps = 0
    
    cleanable_tiles = sum(row.count(0) for row in grid)

    while len(visited) < cleanable_tiles:
        unvisited = [(r, c) for r in range(len(grid)) for c in range(len(grid[0])) 
                     if grid[r][c] == 0 and (r, c) not in visited]
        
        if not unvisited:
            break

        target = min(unvisited, key=lambda p: dist(current_pos, p))

        if strategy == "Greedy":
            neighbors = get_neighbors(current_pos, grid)
            if not neighbors: break
            
            current_pos = min(neighbors, key=lambda p: dist(p, target))
            if current_pos not in visited:
                visited.add(current_pos)
            total_steps += 1
            
        elif strategy == "A*":
            path = a_star_path(grid, current_pos, target)
            if not path: break
            
            for step in path[1:]: 
                current_pos = step
                if current_pos not in visited:
                    visited.add(current_pos)
                total_steps += 1
                
        if total_steps > 500: break

    coverage = (len(visited) / cleanable_tiles) * 100
    return total_steps, coverage


def main():
    print("Starting Robot Vacuum Simulation...\n")
    
    g_steps, g_cov = run_simulation("Greedy")
    a_steps, a_cov = run_simulation("A*")

    print(f"{'Metric':<20} | {'Greedy':<10} | {'A*':<10}")
    print("-" * 45)
    print(f"{'Total Steps (Time)':<20} | {g_steps:<10} | {a_steps:<10}")
    print(f"{'Coverage (%)':<20} | {g_cov:<10.2f} | {a_cov:<10.2f}")
    
    print("\nObservation:")
    if a_steps < g_steps:
        print("A* is more efficient because it plans paths around obstacles.")
    elif g_steps < a_steps:
        print("Greedy was faster, likely because it cleaned 'as it went' without long-distance travel.")
    else:
        print("Both algorithms performed equally on this specific grid layout.")

if __name__ == "__main__":
    main()