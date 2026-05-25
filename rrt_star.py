"""RRT* path planning algorithm implementation."""

from dataclasses import dataclass, field
import numpy as np

from map2d5 import Map2D5


@dataclass
class Node:
    """A node in the RRT* tree."""
    x: float
    y: float
    cost: float = 0.0
    parent: "Node | None" = field(default=None, repr=False)


class RRTStar:
    """RRT* path planner on a 2.5D map.

    Plans a path from start to goal on the xy-plane, avoiding obstacles.
    """

    def __init__(
        self,
        map_env: Map2D5,
        start: tuple[float, float],
        goal: tuple[float, float],
        step_size: float = 0.2,
        search_radius: float = 0.5,
        max_iterations: int = 5000,
        goal_tolerance: float = 0.1,
    ):
        self.map_env = map_env
        self.start = Node(x=start[0], y=start[1], cost=0.0)
        self.goal_pos = goal
        self.step_size = step_size
        self.search_radius = search_radius
        self.max_iterations = max_iterations
        self.goal_tolerance = goal_tolerance
        self.nodes: list[Node] = [self.start]
        self._rng = np.random.default_rng()

    def plan(self) -> list[tuple[float, float]] | None:
        """Run RRT* and return the path as a list of (x, y) or None if no path found."""
        for _ in range(self.max_iterations):
            random_point = self._sample_random()
            nearest_node = self._nearest(random_point)
            new_node = self._steer(nearest_node, random_point)

            if not self._collision_free(nearest_node, new_node):
                continue

            neighbors = self._near(new_node)
            new_node = self._choose_parent(neighbors, nearest_node, new_node)
            self.nodes.append(new_node)
            self._rewire(neighbors, new_node)

            if self._reached_goal(new_node):
                return self._extract_path(new_node)

        # If max iterations reached, try to find the closest node to goal
        best_node = self._best_goal_node()
        if best_node is not None:
            return self._extract_path(best_node)
        return None

    def _sample_random(self) -> tuple[float, float]:
        """Sample a random point in the map. Bias toward goal 5% of the time."""
        if self._rng.random() < 0.05:
            return self.goal_pos
        x = self._rng.uniform(0, self.map_env.size_x)
        y = self._rng.uniform(0, self.map_env.size_y)
        return (x, y)

    def _nearest(self, point: tuple[float, float]) -> Node:
        """Find the nearest node in the tree to the given point."""
        min_dist = float("inf")
        nearest = self.nodes[0]
        for node in self.nodes:
            dist = self._distance_to_point(node, point)
            if dist < min_dist:
                min_dist = dist
                nearest = node
        return nearest

    def _steer(self, from_node: Node, to_point: tuple[float, float]) -> Node:
        """Steer from from_node toward to_point by at most step_size."""
        dx = to_point[0] - from_node.x
        dy = to_point[1] - from_node.y
        dist = np.hypot(dx, dy)
        if dist <= self.step_size:
            return Node(x=to_point[0], y=to_point[1])
        ratio = self.step_size / dist
        new_x = from_node.x + dx * ratio
        new_y = from_node.y + dy * ratio
        return Node(x=new_x, y=new_y)

    def _near(self, node: Node) -> list[Node]:
        """Find all nodes within search_radius of the given node."""
        neighbors = []
        for n in self.nodes:
            if self._distance(n, node) <= self.search_radius:
                neighbors.append(n)
        return neighbors

    def _collision_free(self, from_node: Node, to_node: Node) -> bool:
        """Check if the straight-line path between two nodes is obstacle-free.

        Samples points along the line at half-cell resolution.
        """
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        dist = np.hypot(dx, dy)
        if dist < 1e-9:
            return not self.map_env.is_obstacle(from_node.x, from_node.y)

        n_samples = int(np.ceil(dist / (self.map_env.resolution * 0.5))) + 1
        for i in range(n_samples + 1):
            t = i / n_samples
            x = from_node.x + t * dx
            y = from_node.y + t * dy
            if self.map_env.is_obstacle(x, y):
                return False
        return True

    def _choose_parent(self, neighbors: list[Node], nearest: Node, new_node: Node) -> Node:
        """Choose the best parent for new_node from neighbors."""
        best_parent = nearest
        best_cost = nearest.cost + self._distance(nearest, new_node)

        for neighbor in neighbors:
            cost = neighbor.cost + self._distance(neighbor, new_node)
            if cost < best_cost and self._collision_free(neighbor, new_node):
                best_parent = neighbor
                best_cost = cost

        new_node.parent = best_parent
        new_node.cost = best_cost
        return new_node

    def _rewire(self, neighbors: list[Node], new_node: Node):
        """Rewire neighbors through new_node if it reduces their cost."""
        for neighbor in neighbors:
            new_cost = new_node.cost + self._distance(new_node, neighbor)
            if new_cost < neighbor.cost and self._collision_free(new_node, neighbor):
                neighbor.parent = new_node
                neighbor.cost = new_cost

    def _reached_goal(self, node: Node) -> bool:
        """Check if node is within goal tolerance."""
        dist = np.hypot(node.x - self.goal_pos[0], node.y - self.goal_pos[1])
        return dist <= self.goal_tolerance

    def _best_goal_node(self) -> Node | None:
        """Find the lowest-cost node within goal tolerance."""
        best = None
        best_cost = float("inf")
        for node in self.nodes:
            if self._reached_goal(node) and node.cost < best_cost:
                best = node
                best_cost = node.cost
        return best

    def _extract_path(self, goal_node: Node) -> list[tuple[float, float]]:
        """Trace back from goal_node to start, return path as list of (x, y)."""
        path = []
        node = goal_node
        while node is not None:
            path.append((node.x, node.y))
            node = node.parent
        path.reverse()
        return path

    @staticmethod
    def _distance(n1: Node, n2: Node) -> float:
        return np.hypot(n1.x - n2.x, n1.y - n2.y)

    @staticmethod
    def _distance_to_point(node: Node, point: tuple[float, float]) -> float:
        return np.hypot(node.x - point[0], node.y - point[1])
