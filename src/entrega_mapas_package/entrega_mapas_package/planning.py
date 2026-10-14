"""Theta-style any-angle planner extracted from original coursework in 2026."""
import heapq
import logging
import numpy as np

class ThetaStar:
    def get_logger(self):
        return logging.getLogger(__name__)

    def heuristic(self, a, b):
            """Euclidean distance heuristic for Theta* (Any-angle)."""
            return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    def euclidean_distance(self, a, b):
            """Euclidean distance between two points."""
            return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    def line_of_sight(self, start, end, grid):
            """Bresenham's Line Algorithm to check visibility."""
            x0, y0 = start
            x1, y1 = end

            dx = abs(x1 - x0)
            dy = abs(y1 - y0)

            sx = 1 if x0 < x1 else -1
            sy = 1 if y0 < y1 else -1

            err = dx - dy

            while True:
                if grid[x0, y0] == 1:
                    return False

                if x0 == x1 and y0 == y1:
                    break

                e2 = 2 * err
                if e2 > -dy:
                    err -= dy
                    x0 += sx
                if e2 < dx:
                    err += dx
                    y0 += sy
            return True

    def get_neighbors(self, node, grid):
            """Returns valid neighbors (8-connected) and move costs."""
            rows, cols = grid.shape
            r, c = node
            # (dr, dc, cost)
            moves = [
                (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0), # Straight
                (-1, -1, np.sqrt(2)), (-1, 1, np.sqrt(2)), 
                (1, -1, np.sqrt(2)), (1, 1, np.sqrt(2)) # Diagonal
            ]
            valid_neighbors = []
            for dr, dc, cost in moves:
                nr, nc = r + dr, c + dc
                if 0 <= nr < rows and 0 <= nc < cols:
                    if grid[nr, nc] == 0: # 0 is free space
                        valid_neighbors.append(((nr, nc), cost))
            return valid_neighbors

    def plan(self, grid, start, goal):
            """Theta-style parent relaxation on an 8-connected occupancy grid."""
            rows, cols = grid.shape
            if not (0 <= start[0] < rows and 0 <= start[1] < cols):
                self.get_logger().error("Start point out of bounds")
                return None, 0
            if not (0 <= goal[0] < rows and 0 <= goal[1] < cols):
                self.get_logger().error("Goal point out of bounds")
                return None, 0
            if grid[start] == 1:
                self.get_logger().error("Start point is inside an obstacle")
                return None, 0
            if grid[goal] == 1:
                self.get_logger().error("Goal point is inside an obstacle")
                return None, 0

            open_set = []
            heapq.heappush(open_set, (0, start))
            came_from = {}
            g_score = {start: 0}
            f_score = {start: self.heuristic(start, goal)}
            count = 0

            while open_set:
                count += 1
                _, current = heapq.heappop(open_set)

                if current == goal:
                    self.get_logger().info(f"¡Meta encontrada! Nodos explorados: {count}")
                    return self.reconstruct_path(came_from, current), count

                # Theta* Logic
                parent_node = came_from.get(current)

                for neighbor, cost in self.get_neighbors(current, grid):
                    # Check for line of sight from parent to neighbor
                    if parent_node and self.line_of_sight(parent_node, neighbor, grid):
                        # Path 2: parent -> neighbor
                        new_g_score = g_score[parent_node] + self.euclidean_distance(parent_node, neighbor)
                        if neighbor not in g_score or new_g_score < g_score[neighbor]:
                            came_from[neighbor] = parent_node
                            g_score[neighbor] = new_g_score
                            f = new_g_score + self.heuristic(neighbor, goal)
                            f_score[neighbor] = f
                            heapq.heappush(open_set, (f, neighbor))
                    else:
                        # Path 1: current -> neighbor (Standard A*)
                        tentative_g_score = g_score[current] + cost

                        if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                            came_from[neighbor] = current
                            g_score[neighbor] = tentative_g_score
                            f = tentative_g_score + self.heuristic(neighbor, goal)
                            f_score[neighbor] = f
                            heapq.heappush(open_set, (f, neighbor))

            self.get_logger().warning("No path found!")
            return None, count

    def reconstruct_path(self, came_from, current):
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path
