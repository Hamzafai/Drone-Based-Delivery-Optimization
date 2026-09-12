"""
branch_and_bound.py
Exact Branch and Bound solver for the Drone Delivery Optimization Problem.

Strategy:
- Node: partial assignment of customers to routes (routes built incrementally)
- Branching: at each step, select the next unassigned customer and branch
  on which drone/route position to assign it to.
- Lower Bound: sum of current committed costs + assignment relaxation
  (each remaining customer's minimum possible edge cost from/to nearest node).
- Pruning: prune if LB >= best known cost, or if capacity constraints violated.
"""

import time
import sys
import os
import math
from copy import deepcopy

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from src.utils.data_structures import DroneInstance


class BBNode:
    """State in the B&B tree."""

    def __init__(self, routes, unassigned, committed_cost):
        """
        Parameters
        ----------
        routes : list of lists  — partial routes for each drone (customer indices)
        unassigned : set of int — remaining customers to assign
        committed_cost : float  — cost of arcs already fixed in routes
        """
        self.routes = routes
        self.unassigned = unassigned
        self.committed_cost = committed_cost

    def is_complete(self):
        return len(self.unassigned) == 0


class BranchAndBound:
    """
    Branch and Bound solver.

    Parameters
    ----------
    instance : DroneInstance
    max_drones : int   — max fleet size (default: n_customers)
    time_limit : float — seconds before stopping (returns best found)
    verbose : bool
    """

    def __init__(self, instance, max_drones=None, time_limit=300, verbose=True):
        self.inst = instance
        self.n = instance.n_customers
        self.max_drones = max_drones or self.n
        self.time_limit = time_limit
        self.verbose = verbose

        self.best_cost = float('inf')
        self.best_solution = None
        self.nodes_explored = 0
        self.nodes_pruned = 0
        self.start_time = None

    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------

    def solve(self):
        self.start_time = time.time()

        # Compute initial upper bound with greedy nearest-neighbor
        greedy_sol, greedy_cost = self._greedy_initial()
        if greedy_sol is not None:
            self.best_cost = greedy_cost
            self.best_solution = greedy_sol
            if self.verbose:
                print(f"  [B&B] Greedy UB = {self.best_cost:.2f}")

        # Root node: empty routes for each possible drone
        root = BBNode(
            routes=[[] for _ in range(self.max_drones)],
            unassigned=set(range(1, self.n + 1)),
            committed_cost=0.0
        )

        self._branch(root)

        elapsed = time.time() - self.start_time
        if self.verbose:
            print(f"  [B&B] Done. Best cost={self.best_cost:.2f}, "
                  f"nodes={self.nodes_explored}, pruned={self.nodes_pruned}, "
                  f"time={elapsed:.2f}s")

        return self.best_solution, self.best_cost, elapsed

    # ------------------------------------------------------------------
    # Core B&B recursion
    # ------------------------------------------------------------------

    def _branch(self, node):
        if time.time() - self.start_time > self.time_limit:
            return

        self.nodes_explored += 1

        if node.is_complete():
            cost = self._full_solution_cost(node.routes)
            if cost < self.best_cost:
                self.best_cost = cost
                self.best_solution = [list(r) for r in node.routes]
                if self.verbose:
                    print(f"  [B&B] New best: {self.best_cost:.2f} "
                          f"(nodes={self.nodes_explored})")
            return

        # Lower bound check
        lb = self._lower_bound(node)
        if lb >= self.best_cost:
            self.nodes_pruned += 1
            return

        # Select next customer to assign (most constrained first)
        customer = self._select_customer(node)

        # Branch: try inserting customer into each drone's route
        children = []
        for k in range(self.max_drones):
            route = node.routes[k]

            # Try appending to existing route of drone k
            new_route = list(route) + [customer]
            if self._route_feasible_partial(new_route):
                new_routes = [list(r) for r in node.routes]
                new_routes[k] = new_route

                # Committed cost: add the new arc cost
                if route:
                    arc_cost = self.inst.dist[route[-1]][customer]
                else:
                    arc_cost = self.inst.dist[0][customer]

                if arc_cost < float('inf'):
                    child = BBNode(
                        routes=new_routes,
                        unassigned=node.unassigned - {customer},
                        committed_cost=node.committed_cost + arc_cost
                    )
                    child_lb = self._lower_bound(child)
                    children.append((child_lb, child))

        # Sort children by lower bound (best-first)
        children.sort(key=lambda x: x[0])

        for _, child in children:
            self._branch(child)
            if time.time() - self.start_time > self.time_limit:
                return

    # ------------------------------------------------------------------
    # Lower bound
    # ------------------------------------------------------------------

    def _lower_bound(self, node):
        """
        LB = committed cost
             + for each remaining customer: min cost to reach it from any current
               route tail or depot + min cost from it back toward depot.
        This is the assignment relaxation (ignores capacity, route continuity).
        """
        lb = node.committed_cost

        for c in node.unassigned:
            # Cheapest way to reach c from any current tail or depot
            min_in = float('inf')
            for k in range(self.max_drones):
                tail = node.routes[k][-1] if node.routes[k] else 0
                cost = self.inst.dist[tail][c]
                if cost < min_in:
                    min_in = cost

            # Cheapest way out from c (toward depot or any other unassigned)
            min_out = self.inst.dist[c][0]  # at least the return to depot

            lb += min_in + min_out * 0.5  # factor 0.5 to avoid double counting

        return lb

    # ------------------------------------------------------------------
    # Feasibility
    # ------------------------------------------------------------------

    def _route_feasible_partial(self, route):
        """Check if a partial route can still be extended (payload check only)."""
        demands = self.inst.demands()
        total_demand = sum(demands[c] for c in route)
        if total_demand > self.inst.payload_capacity:
            return False
        # Partial battery check: current path cost + return to depot
        cost = self.inst.dist[0][route[0]]
        for i in range(len(route) - 1):
            cost += self.inst.dist[route[i]][route[i+1]]
        cost += self.inst.dist[route[-1]][0]
        if cost > self.inst.battery_capacity:
            return False
        return True

    def _full_solution_cost(self, routes):
        return sum(self.inst.route_cost(r) for r in routes if r)

    # ------------------------------------------------------------------
    # Customer selection heuristic
    # ------------------------------------------------------------------

    def _select_customer(self, node):
        """Select the unassigned customer with highest demand (most constrained)."""
        demands = self.inst.demands()
        return max(node.unassigned, key=lambda c: demands[c])

    # ------------------------------------------------------------------
    # Greedy initial solution (nearest neighbor)
    # ------------------------------------------------------------------

    def _greedy_initial(self):
        """
        Greedy nearest-neighbor construction heuristic.
        Repeatedly assigns the nearest unvisited customer to the current drone
        while capacity allows. Opens a new drone when needed.
        """
        unvisited = set(range(1, self.n + 1))
        routes = []
        demands = self.inst.demands()

        while unvisited:
            route = []
            current = 0
            route_demand = 0.0
            route_dist = 0.0

            while True:
                # Find nearest feasible unvisited customer
                best_c, best_d = None, float('inf')
                for c in unvisited:
                    if (demands[c] + route_demand > self.inst.payload_capacity):
                        continue
                    d = self.inst.dist[current][c]
                    # Check if we can still return to depot
                    return_d = self.inst.dist[c][0]
                    if route_dist + d + return_d > self.inst.battery_capacity:
                        continue
                    if d < best_d:
                        best_d, best_c = d, c

                if best_c is None:
                    break

                route.append(best_c)
                route_demand += demands[best_c]
                route_dist += best_d
                current = best_c
                unvisited.remove(best_c)

            if not route and unvisited:
                # Cannot serve remaining customers — infeasible
                return None, float('inf')

            routes.append(route)

        cost = sum(self.inst.route_cost(r) for r in routes)
        return routes, cost


# ------------------------------------------------------------------
# Convenience function
# ------------------------------------------------------------------

def solve_bb(instance, max_drones=None, time_limit=120, verbose=True):
    solver = BranchAndBound(instance, max_drones=max_drones,
                            time_limit=time_limit, verbose=verbose)
    solution, cost, elapsed = solver.solve()
    return solution, cost, elapsed


if __name__ == "__main__":
    from src.utils.instance_generator import generate_instance
    inst = generate_instance(n_customers=8, seed=42)
    print(inst)
    sol, cost, t = solve_bb(inst, time_limit=30)
    print(f"Solution cost: {cost:.2f}  ({t:.2f}s)")
    for i, r in enumerate(sol):
        print(f"  Drone {i+1}: {r}  demand={inst.route_demand(r):.1f}  dist={inst.route_cost(r):.2f}")
