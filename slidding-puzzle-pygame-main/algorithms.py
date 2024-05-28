import heapq
from collections import deque
def heuristic_cost_estimate(start, goal):
    """Heuristic function for A* (Manhattan distance)"""
    return sum(abs(s % 3 - g % 3) + abs(s // 3 - g // 3) for s, g in zip(start, goal))

def astar_solver(start, goal):
    """A* algorithm to solve the sliding puzzle"""
    start_flat = tuple(tile for row in start for tile in row)
    goal_flat = tuple(tile for row in goal for tile in row)

    open_set = []
    heapq.heappush(open_set, (0, start_flat))
    came_from = {start_flat: None}
    g_score = {start_flat: 0}
    f_score = {start_flat: heuristic_cost_estimate(start_flat, goal_flat)}

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal_flat:
            path = []
            while current:
                path.append(current)
                current = came_from[current]
            return path[::-1]

        empty_index = current.index(0)
        current_list = list(current)
        for move in possible_moves(current_list, empty_index):
            tentative_g_score = g_score[current] + 1
            neighbor = tuple(move)
            if tentative_g_score < g_score.get(neighbor, float('inf')):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic_cost_estimate(neighbor, goal_flat)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return None

def bfs_solver(start, goal):
    """Breadth-First Search algorithm to solve the sliding puzzle"""
    start_flat = tuple(tile for row in start for tile in row)
    goal_flat = tuple(tile for row in goal for tile in row)
    queue = deque([(start_flat, [])])  # Initial state and path
    visited = set()

    while queue:
        state, path = queue.popleft()
        if state == goal_flat:
            return path
        if state not in visited:
            visited.add(state)
            empty_index = state.index(0)
            for move in possible_moves(list(state), empty_index):
                queue.append((tuple(move), path + [tuple(move)]))

    return None


def possible_moves(state, empty_index):
    """Generate possible moves given the empty index"""
    moves = []
    x, y = empty_index % 3, empty_index // 3
    if x > 0:
        state[empty_index], state[empty_index - 1] = state[empty_index - 1], state[empty_index]
        moves.append(state[:])
        state[empty_index], state[empty_index - 1] = state[empty_index - 1], state[empty_index]
    if x < 2:
        state[empty_index], state[empty_index + 1] = state[empty_index + 1], state[empty_index]
        moves.append(state[:])
        state[empty_index], state[empty_index + 1] = state[empty_index + 1], state[empty_index]
    if y > 0:
        state[empty_index], state[empty_index - 3] = state[empty_index - 3], state[empty_index]
        moves.append(state[:])
        state[empty_index], state[empty_index - 3] = state[empty_index - 3], state[empty_index]
    if y < 2:
        state[empty_index], state[empty_index + 3] = state[empty_index + 3], state[empty_index]
        moves.append(state[:])
        state[empty_index], state[empty_index + 3] = state[empty_index + 3], state[empty_index]
    return moves
