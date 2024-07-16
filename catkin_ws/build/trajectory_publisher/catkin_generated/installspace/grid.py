#!/usr/bin/env python3
import numpy as np
import heapq

class Grid:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.grid = np.zeros((width, height))

    def set_obstacle(self, x, y):
        self.grid[x][y] = 1

    def is_free(self, x, y):
        return self.grid[x][y] == 0

    def neighbors(self, x, y):
        neighbors = []
        if x > 0:
            neighbors.append((x-1, y))
        if x < self.width - 1:
            neighbors.append((x+1, y))
        if y > 0:
            neighbors.append((x, y-1))
        if y < self.height - 1:
            neighbors.append((x, y+1))
        return neighbors

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def a_star_search(grid, start, goal):
    open_list = []
    heapq.heappush(open_list, (0, start))
    came_from = {}
    cost_so_far = {start: 0}

    while open_list:
        current = heapq.heappop(open_list)[1]

        if current == goal:
            break

        for next in grid.neighbors(current[0], current[1]):
            if not grid.is_free(next[0], next[1]):
                continue
            new_cost = cost_so_far[current] + 1  # Assume uniform cost for simplicity
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(goal, next)
                heapq.heappush(open_list, (priority, next))
                came_from[next] = current

    return reconstruct_path(came_from, start, goal)

def reconstruct_path(came_from, start, goal):
    current = goal
    path = []
    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start)
    path.reverse()
    return path
