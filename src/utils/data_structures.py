"""
data_structures.py
Core data classes for the Drone Delivery Optimization Problem.
"""

import json
import math
import numpy as np


class Customer:
    def __init__(self, idx, x, y, demand):
        self.idx = idx
        self.x = x
        self.y = y
        self.demand = demand

    def __repr__(self):
        return f"Customer(id={self.idx}, pos=({self.x:.1f},{self.y:.1f}), demand={self.demand:.1f})"


class DroneInstance:
    """
    Encapsulates a full problem instance.

    Nodes: 0 = depot, 1..n = customers
    """

    def __init__(self, n_customers, depot_x, depot_y, customers,
                 payload_capacity, battery_capacity, no_fly_edges=None):
        self.n_customers = n_customers
        self.depot_x = depot_x
        self.depot_y = depot_y
        self.customers = customers          # list of Customer objects (index 1..n)
        self.payload_capacity = payload_capacity
        self.battery_capacity = battery_capacity
        self.no_fly_edges = set(no_fly_edges) if no_fly_edges else set()

        # Build distance matrix (node 0 = depot)
        self._build_distance_matrix()

    def _build_distance_matrix(self):
        n = self.n_customers + 1  # includes depot
        self.dist = np.zeros((n, n))
        xs = [self.depot_x] + [c.x for c in self.customers]
        ys = [self.depot_y] + [c.y for c in self.customers]

        for i in range(n):
            for j in range(n):
                if i == j:
                    self.dist[i][j] = 0.0
                elif (i, j) in self.no_fly_edges or (j, i) in self.no_fly_edges:
                    self.dist[i][j] = float('inf')
                else:
                    dx = xs[i] - xs[j]
                    dy = ys[i] - ys[j]
                    self.dist[i][j] = math.sqrt(dx*dx + dy*dy)

    def demands(self):
        """Returns demand array indexed 0..n (depot demand = 0)."""
        return [0.0] + [c.demand for c in self.customers]

    def route_cost(self, route):
        """
        Cost of a route given as list of customer indices (1-based).
        Route is: depot -> c1 -> c2 -> ... -> depot
        Returns inf if any arc is forbidden.
        """
        if not route:
            return 0.0
        total = self.dist[0][route[0]]
        for i in range(len(route) - 1):
            total += self.dist[route[i]][route[i+1]]
        total += self.dist[route[-1]][0]
        return total

    def route_demand(self, route):
        """Total demand of a route."""
        demands = self.demands()
        return sum(demands[c] for c in route)

    def is_route_feasible(self, route):
        """Check payload and battery feasibility."""
        if self.route_demand(route) > self.payload_capacity:
            return False
        if self.route_cost(route) > self.battery_capacity:
            return False
        return True

    def solution_cost(self, solution):
        """
        solution: list of routes, each route = list of customer indices.
        Returns total energy cost.
        """
        return sum(self.route_cost(r) for r in solution)

    def is_solution_valid(self, solution):
        """Validate: all customers covered once, all routes feasible."""
        visited = []
        for route in solution:
            if not self.is_route_feasible(route):
                return False, f"Route {route} infeasible"
            visited.extend(route)
        all_customers = set(range(1, self.n_customers + 1))
        if set(visited) != all_customers:
            return False, f"Coverage mismatch: visited={set(visited)}"
        if len(visited) != len(all_customers):
            return False, "Duplicate visits"
        return True, "OK"

    def to_dict(self):
        return {
            "n_customers": self.n_customers,
            "depot": {"x": self.depot_x, "y": self.depot_y},
            "customers": [{"idx": c.idx, "x": c.x, "y": c.y, "demand": c.demand}
                          for c in self.customers],
            "payload_capacity": self.payload_capacity,
            "battery_capacity": self.battery_capacity,
            "no_fly_edges": list(self.no_fly_edges)
        }

    def save(self, path):
        with open(path, 'w') as f:
            json.dump(self.to_dict(), f, indent=2)

    @classmethod
    def load(cls, path):
        with open(path) as f:
            data = json.load(f)
        customers = [Customer(c['idx'], c['x'], c['y'], c['demand'])
                     for c in data['customers']]
        no_fly = [tuple(e) for e in data.get('no_fly_edges', [])]
        return cls(
            n_customers=data['n_customers'],
            depot_x=data['depot']['x'],
            depot_y=data['depot']['y'],
            customers=customers,
            payload_capacity=data['payload_capacity'],
            battery_capacity=data['battery_capacity'],
            no_fly_edges=no_fly
        )

    def __repr__(self):
        return (f"DroneInstance(n={self.n_customers}, "
                f"Q={self.payload_capacity}, B={self.battery_capacity:.1f}, "
                f"no_fly={len(self.no_fly_edges)} edges)")
