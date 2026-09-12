"""
simulated_annealing.py
Local Search Metaheuristic: Simulated Annealing for Drone Delivery.

Representation:
  A solution is a list of drone routes: [[c1,c2,...], [c3,c4,...], ...]
  Each route is a sequence of customer indices served by one drone.

Neighborhood operators (problem-specific):
  1. 2-opt intra-route  : reverse a sub-sequence within one route
  2. Or-opt (1/2/3)     : relocate 1, 2, or 3 consecutive customers
                          within or between routes
  3. Cross-exchange     : swap a customer between two different routes
  4. Route merge/split  : merge two short routes or split a long route

Feasibility:
  After each move, check payload and battery constraints.
  Infeasible moves are rejected (feasibility maintained at all times).

Cooling Schedule:
  Geometric: T(t) = T0 * alpha^t
  Reheat: if stuck for too many iterations, temperature is partially reheated.
"""

import random
import math
import time
import sys
import os
from copy import deepcopy

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from src.utils.data_structures import DroneInstance


# ============================================================
# Neighborhood Operators
# ============================================================

def two_opt_intra(routes, instance):
    """2-opt within a single route: reverse sub-segment [i..j]."""
    k = random.randrange(len(routes))
    route = routes[k]
    if len(route) < 3:
        return None

    i, j = sorted(random.sample(range(len(route)), 2))
    if i == j:
        return None
    new_route = route[:i] + list(reversed(route[i:j+1])) + route[j+1:]

    if not instance.is_route_feasible(new_route):
        return None

    new_routes = [list(r) for r in routes]
    new_routes[k] = new_route
    return new_routes


def or_opt_move(routes, instance, seg_len=1):
    """
    Relocate a segment of seg_len customers from one route to another position.
    Can be intra-route or inter-route.
    """
    # Pick a source route and segment
    non_empty = [k for k in range(len(routes)) if len(routes[k]) >= seg_len]
    if not non_empty:
        return None
    k_src = random.choice(non_empty)
    route_src = routes[k_src]

    if len(route_src) < seg_len:
        return None
    seg_start = random.randint(0, len(route_src) - seg_len)
    segment = route_src[seg_start:seg_start + seg_len]

    # Remove segment from source
    new_src = route_src[:seg_start] + route_src[seg_start + seg_len:]

    # Choose destination route and insertion position
    k_dst = random.randrange(len(routes))
    route_dst = routes[k_dst] if k_dst != k_src else new_src
    ins_pos = random.randint(0, len(route_dst))

    if k_dst == k_src:
        new_dst = route_dst[:ins_pos] + segment + route_dst[ins_pos:]
        if not instance.is_route_feasible(new_dst):
            return None
        new_routes = [list(r) for r in routes]
        new_routes[k_src] = new_dst
    else:
        new_dst = route_dst[:ins_pos] + segment + route_dst[ins_pos:]
        if new_src and not instance.is_route_feasible(new_src):
            return None
        if not instance.is_route_feasible(new_dst):
            return None
        new_routes = [list(r) for r in routes]
        new_routes[k_src] = new_src
        new_routes[k_dst] = new_dst
        # Remove empty routes
        new_routes = [r for r in new_routes if r]

    return new_routes


def cross_exchange(routes, instance):
    """Swap one customer between two different routes."""
    non_empty = [k for k in range(len(routes)) if routes[k]]
    if len(non_empty) < 2:
        return None
    k1, k2 = random.sample(non_empty, 2)

    i = random.randrange(len(routes[k1]))
    j = random.randrange(len(routes[k2]))
    c1 = routes[k1][i]
    c2 = routes[k2][j]

    new_r1 = list(routes[k1])
    new_r2 = list(routes[k2])
    new_r1[i] = c2
    new_r2[j] = c1

    if not instance.is_route_feasible(new_r1):
        return None
    if not instance.is_route_feasible(new_r2):
        return None

    new_routes = [list(r) for r in routes]
    new_routes[k1] = new_r1
    new_routes[k2] = new_r2
    return new_routes


def route_merge(routes, instance):
    """Merge two compatible routes into one (if feasible)."""
    non_empty = [k for k in range(len(routes)) if routes[k]]
    if len(non_empty) < 2:
        return None
    k1, k2 = random.sample(non_empty, 2)
    merged = routes[k1] + routes[k2]
    if instance.is_route_feasible(merged):
        new_routes = [list(r) for r in routes]
        new_routes[k1] = merged
        new_routes[k2] = []
        new_routes = [r for r in new_routes if r]
        return new_routes
    return None


def get_neighbor(routes, instance):
    """
    Generate a random neighbor by applying one of the neighborhood operators.
    Operator selection is randomized.
    """
    operators = [
        lambda: two_opt_intra(routes, instance),
        lambda: or_opt_move(routes, instance, seg_len=1),
        lambda: or_opt_move(routes, instance, seg_len=2),
        lambda: or_opt_move(routes, instance, seg_len=3),
        lambda: cross_exchange(routes, instance),
        lambda: route_merge(routes, instance),
    ]
    weights = [0.20, 0.25, 0.15, 0.10, 0.20, 0.10]

    for _ in range(20):  # try up to 20 operators before giving up
        op = random.choices(operators, weights=weights, k=1)[0]
        result = op()
        if result is not None:
            return result

    return [list(r) for r in routes]  # no improving move found


# ============================================================
# Initial Solution
# ============================================================

def greedy_initial(instance):
    """
    Greedy nearest-neighbor heuristic to build an initial feasible solution.
    """
    unvisited = set(range(1, instance.n_customers + 1))
    demands = instance.demands()
    routes = []

    while unvisited:
        route = []
        current = 0
        route_demand = 0.0
        route_dist = 0.0

        while True:
            best_c, best_d = None, float('inf')
            for c in unvisited:
                if demands[c] + route_demand > instance.payload_capacity:
                    continue
                d = instance.dist[current][c]
                if d == float('inf'):
                    continue
                return_d = instance.dist[c][0]
                if route_dist + d + return_d > instance.battery_capacity:
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

        if route:
            routes.append(route)
        elif unvisited:
            # Force add unrouted customer
            c = next(iter(unvisited))
            routes.append([c])
            unvisited.remove(c)

    return routes


# ============================================================
# Simulated Annealing
# ============================================================

class SimulatedAnnealing:
    """
    Simulated Annealing for Drone Delivery.

    Parameters
    ----------
    instance        : DroneInstance
    T0              : float — initial temperature
    alpha           : float — cooling rate (geometric)
    T_min           : float — stopping temperature
    iter_per_temp   : int   — iterations at each temperature level
    time_limit      : float — max seconds
    reheat_interval : int   — iterations without improvement before reheat
    reheat_factor   : float — temperature multiplied by this on reheat
    verbose         : bool
    """

    def __init__(self, instance, T0=None, alpha=0.995, T_min=0.1,
                 iter_per_temp=50, time_limit=60,
                 reheat_interval=500, reheat_factor=1.5, verbose=True):
        self.inst = instance
        self.alpha = alpha
        self.T_min = T_min
        self.iter_per_temp = iter_per_temp
        self.time_limit = time_limit
        self.reheat_interval = reheat_interval
        self.reheat_factor = reheat_factor
        self.verbose = verbose

        # Auto-set T0 based on instance if not provided
        if T0 is None:
            n = instance.n_customers
            avg_d = sum(instance.dist[i][j]
                        for i in range(n+1)
                        for j in range(n+1) if i != j
                        and instance.dist[i][j] < float('inf')) / max(1, n * n)
            self.T0 = avg_d * 5.0
        else:
            self.T0 = T0

        self.best_solution = None
        self.best_cost = float('inf')
        self.history = []  # best cost at each accepted move

    def solve(self):
        start = time.time()
        random.seed(None)

        # Build initial solution
        current_solution = greedy_initial(self.inst)
        current_cost = self.inst.solution_cost(current_solution)

        self.best_solution = deepcopy(current_solution)
        self.best_cost = current_cost

        T = self.T0
        iteration = 0
        no_improve = 0
        accepted = 0
        total_moves = 0

        if self.verbose:
            print(f"  [SA] T0={T:.2f}, alpha={self.alpha}, "
                  f"initial cost={current_cost:.2f}")

        while T > self.T_min:
            if time.time() - start > self.time_limit:
                break

            for _ in range(self.iter_per_temp):
                neighbor = get_neighbor(current_solution, self.inst)
                neighbor_cost = self.inst.solution_cost(neighbor)
                delta = neighbor_cost - current_cost
                total_moves += 1

                # Accept or reject
                if delta < 0 or random.random() < math.exp(-delta / T):
                    current_solution = neighbor
                    current_cost = neighbor_cost
                    accepted += 1

                    if current_cost < self.best_cost:
                        self.best_cost = current_cost
                        self.best_solution = deepcopy(current_solution)
                        no_improve = 0
                        if self.verbose:
                            print(f"  [SA] Iter {iteration}: new best = "
                                  f"{self.best_cost:.2f}  T={T:.3f}")
                    else:
                        no_improve += 1
                else:
                    no_improve += 1

                self.history.append(self.best_cost)
                iteration += 1

            # Cool down
            T *= self.alpha

            # Reheat if stuck
            if no_improve >= self.reheat_interval:
                T = min(T * self.reheat_factor, self.T0 * 0.5)
                no_improve = 0

        elapsed = time.time() - start
        accept_rate = accepted / max(1, total_moves)

        if self.verbose:
            print(f"  [SA] Done. Best={self.best_cost:.2f}, "
                  f"iter={iteration}, accept_rate={accept_rate:.2%}, "
                  f"time={elapsed:.2f}s")

        return self.best_solution, self.best_cost, elapsed, self.history


def solve_sa(instance, T0=None, alpha=0.995, time_limit=60, verbose=True):
    sa = SimulatedAnnealing(instance, T0=T0, alpha=alpha,
                            time_limit=time_limit, verbose=verbose)
    return sa.solve()


if __name__ == "__main__":
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
    from src.utils.instance_generator import generate_instance
    inst = generate_instance(n_customers=20, seed=42)
    print(inst)
    sol, cost, t, hist = solve_sa(inst, time_limit=30)
    print(f"SA Solution cost: {cost:.2f}  ({t:.2f}s)")
    valid, msg = inst.is_solution_valid(sol)
    print(f"Valid: {valid} — {msg}")
