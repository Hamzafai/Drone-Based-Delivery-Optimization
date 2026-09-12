"""
genetic_algorithm.py
Population-based metaheuristic: Genetic Algorithm for Drone Delivery.

Encoding:
  A chromosome is a permutation of all customer indices (1..n).
  A decoder splits the permutation into drone routes using a greedy
  bin-packing strategy that respects payload and battery constraints.
  This representation is called the "Giant Tour" encoding (Clarke & Wright style).

Operators:
  - Selection : Binary tournament
  - Crossover : Order Crossover (OX)
  - Mutation  : Swap mutation + Inversion mutation (adaptive rate)
  - Repair    : Infeasible genes are fixed by a route-split repair

Population diversity is maintained via fitness sharing distance.
"""

import random
import time
import sys
import os
import math
from copy import deepcopy

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from src.utils.data_structures import DroneInstance


# ============================================================
# Decoder: Giant Tour → feasible set of drone routes
# ============================================================

def decode(chromosome, instance):
    """
    Split a permutation of customers into feasible drone routes.

    Strategy: Greedy split — add next customer to current route if feasible,
    otherwise close current route and start new drone.

    Parameters
    ----------
    chromosome : list of int  — permutation of [1..n]
    instance   : DroneInstance

    Returns
    -------
    list of lists — one sub-list per drone
    """
    routes = []
    demands = instance.demands()
    current_route = []
    current_demand = 0.0
    current_dist = 0.0
    current_node = 0  # depot

    for customer in chromosome:
        d_to = instance.dist[current_node][customer]
        d_back = instance.dist[customer][0]

        # Feasibility check (append + return)
        can_add = (
            current_demand + demands[customer] <= instance.payload_capacity
            and current_dist + d_to + d_back <= instance.battery_capacity
            and d_to < float('inf')
        )

        if can_add:
            current_route.append(customer)
            current_demand += demands[customer]
            current_dist += d_to
            current_node = customer
        else:
            # Close current route, start new drone
            if current_route:
                routes.append(current_route)
            # Start fresh with this customer (might still be infeasible individually)
            d_to_depot = instance.dist[0][customer]
            d_back_depot = instance.dist[customer][0]
            if (demands[customer] <= instance.payload_capacity
                    and d_to_depot + d_back_depot <= instance.battery_capacity
                    and d_to_depot < float('inf')):
                current_route = [customer]
                current_demand = demands[customer]
                current_dist = d_to_depot
                current_node = customer
            else:
                # Single customer route infeasible: force assign to any open route
                # (repair: best insertion)
                best_k, best_pos, best_inc = _best_insertion(customer, routes, instance)
                if best_k is not None:
                    routes[best_k].insert(best_pos, customer)
                else:
                    # Last resort: new single-customer route
                    routes.append([customer])
                    current_route = []
                    current_demand = 0.0
                    current_dist = 0.0
                    current_node = 0

    if current_route:
        routes.append(current_route)

    return routes


def _best_insertion(customer, routes, instance):
    """Find best insertion position across all routes (minimum cost increase)."""
    demands = instance.demands()
    best_k, best_pos, best_inc = None, None, float('inf')
    for k, route in enumerate(routes):
        if instance.route_demand(route) + demands[customer] > instance.payload_capacity:
            continue
        for pos in range(len(route) + 1):
            new_route = route[:pos] + [customer] + route[pos:]
            if instance.route_cost(new_route) <= instance.battery_capacity:
                prev = route[pos-1] if pos > 0 else 0
                nxt = route[pos] if pos < len(route) else 0
                inc = (instance.dist[prev][customer] +
                       instance.dist[customer][nxt] -
                       instance.dist[prev][nxt])
                if inc < best_inc:
                    best_inc, best_k, best_pos = inc, k, pos
    return best_k, best_pos, best_inc


# ============================================================
# Fitness
# ============================================================

def fitness(chromosome, instance):
    """Return total energy cost of decoded solution. Lower = better."""
    routes = decode(chromosome, instance)
    return instance.solution_cost(routes)


# ============================================================
# Genetic Operators
# ============================================================

def tournament_selection(population, fitnesses, k=3):
    """Binary (k-way) tournament selection. Returns index of winner."""
    candidates = random.sample(range(len(population)), k)
    return min(candidates, key=lambda i: fitnesses[i])


def order_crossover(parent1, parent2):
    """
    Order Crossover (OX): preserves relative order of genes.

    1. Choose a random segment from parent1.
    2. Fill remaining positions with genes from parent2 in order.
    """
    n = len(parent1)
    a, b = sorted(random.sample(range(n), 2))
    child = [None] * n
    child[a:b+1] = parent1[a:b+1]
    segment_set = set(parent1[a:b+1])
    fill = [g for g in parent2 if g not in segment_set]
    j = 0
    for i in range(n):
        if child[i] is None:
            child[i] = fill[j]
            j += 1
    return child


def swap_mutation(chromosome, rate=0.15):
    """Randomly swap pairs of genes."""
    chrom = list(chromosome)
    for i in range(len(chrom)):
        if random.random() < rate:
            j = random.randint(0, len(chrom) - 1)
            chrom[i], chrom[j] = chrom[j], chrom[i]
    return chrom


def inversion_mutation(chromosome, rate=0.1):
    """Reverse a random sub-segment of the chromosome."""
    chrom = list(chromosome)
    if random.random() < rate:
        a, b = sorted(random.sample(range(len(chrom)), 2))
        chrom[a:b+1] = reversed(chrom[a:b+1])
    return chrom


def or_opt_local_search(chromosome, instance, iterations=20):
    """
    Or-opt on the chromosome level: relocate a segment of 1–3 genes
    to a better position. This acts as a local search on the permutation.
    """
    chrom = list(chromosome)
    best_f = fitness(chrom, instance)
    n = len(chrom)
    for _ in range(iterations):
        seg_len = random.choice([1, 2, 3])
        i = random.randint(0, n - seg_len)
        seg = chrom[i:i+seg_len]
        rest = chrom[:i] + chrom[i+seg_len:]
        j = random.randint(0, len(rest))
        new_chrom = rest[:j] + seg + rest[j:]
        new_f = fitness(new_chrom, instance)
        if new_f < best_f:
            chrom = new_chrom
            best_f = new_f
    return chrom


# ============================================================
# Genetic Algorithm
# ============================================================

class GeneticAlgorithm:
    """
    Genetic Algorithm for Drone Delivery.

    Parameters
    ----------
    instance        : DroneInstance
    pop_size        : int    — population size
    max_generations : int    — stopping criterion
    time_limit      : float  — seconds
    crossover_rate  : float  — probability of crossover
    mutation_rate   : float  — base mutation rate (adaptive)
    elite_size      : int    — number of elite solutions preserved
    local_search    : bool   — apply Or-opt to each new offspring
    verbose         : bool
    """

    def __init__(self, instance, pop_size=80, max_generations=300,
                 time_limit=60, crossover_rate=0.85, mutation_rate=0.15,
                 elite_size=5, local_search=True, verbose=True):
        self.inst = instance
        self.n = instance.n_customers
        self.pop_size = pop_size
        self.max_generations = max_generations
        self.time_limit = time_limit
        self.crossover_rate = crossover_rate
        self.mutation_rate = mutation_rate
        self.elite_size = elite_size
        self.local_search = local_search
        self.verbose = verbose

        self.best_solution = None
        self.best_cost = float('inf')
        self.history = []  # best cost per generation

    def _initial_population(self):
        """Generate diverse initial population."""
        pop = []
        base = list(range(1, self.n + 1))

        # Greedy nearest-neighbor chromosome
        demands = self.inst.demands()
        greedy = self._greedy_chromosome()
        pop.append(greedy)

        # Random permutations
        while len(pop) < self.pop_size:
            chrom = list(base)
            random.shuffle(chrom)
            pop.append(chrom)

        return pop

    def _greedy_chromosome(self):
        """Build a greedy chromosome using nearest-neighbor heuristic."""
        unvisited = list(range(1, self.n + 1))
        chromosome = []
        current = 0
        while unvisited:
            nearest = min(unvisited,
                          key=lambda c: self.inst.dist[current][c]
                          if self.inst.dist[current][c] < float('inf') else 1e9)
            chromosome.append(nearest)
            unvisited.remove(nearest)
            current = nearest
        return chromosome

    def solve(self):
        start = time.time()
        random.seed(None)

        population = self._initial_population()
        fitnesses = [fitness(c, self.inst) for c in population]

        # Initialize best
        for i, f in enumerate(fitnesses):
            if f < self.best_cost:
                self.best_cost = f
                self.best_solution = decode(population[i], self.inst)

        if self.verbose:
            print(f"  [GA] Initial best = {self.best_cost:.2f}")

        no_improve = 0

        for gen in range(self.max_generations):
            if time.time() - start > self.time_limit:
                break

            new_population = []

            # Elitism: preserve best individuals
            sorted_idx = sorted(range(len(population)), key=lambda i: fitnesses[i])
            for i in range(self.elite_size):
                new_population.append(list(population[sorted_idx[i]]))

            # Adaptive mutation rate: increase if stagnating
            adapt_mut = self.mutation_rate + 0.1 * (no_improve / max(1, self.max_generations // 3))
            adapt_mut = min(adapt_mut, 0.5)

            while len(new_population) < self.pop_size:
                # Selection
                p1_idx = tournament_selection(population, fitnesses, k=3)
                p2_idx = tournament_selection(population, fitnesses, k=3)
                p1, p2 = population[p1_idx], population[p2_idx]

                # Crossover
                if random.random() < self.crossover_rate:
                    child = order_crossover(p1, p2)
                else:
                    child = list(p1)

                # Mutation
                child = swap_mutation(child, rate=adapt_mut)
                child = inversion_mutation(child, rate=adapt_mut * 0.7)

                # Local search
                if self.local_search and random.random() < 0.4:
                    child = or_opt_local_search(child, self.inst, iterations=15)

                new_population.append(child)

            population = new_population
            fitnesses = [fitness(c, self.inst) for c in population]

            gen_best = min(fitnesses)
            self.history.append(gen_best)

            if gen_best < self.best_cost:
                best_idx = fitnesses.index(gen_best)
                self.best_cost = gen_best
                self.best_solution = decode(population[best_idx], self.inst)
                no_improve = 0
                if self.verbose:
                    print(f"  [GA] Gen {gen+1}: new best = {self.best_cost:.2f}")
            else:
                no_improve += 1

        elapsed = time.time() - start
        if self.verbose:
            print(f"  [GA] Done. Best={self.best_cost:.2f}, "
                  f"gen={len(self.history)}, time={elapsed:.2f}s")

        return self.best_solution, self.best_cost, elapsed, self.history


def solve_ga(instance, pop_size=80, max_generations=300, time_limit=60, verbose=True):
    ga = GeneticAlgorithm(instance, pop_size=pop_size,
                          max_generations=max_generations,
                          time_limit=time_limit, verbose=verbose)
    return ga.solve()


if __name__ == "__main__":
    import sys
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
    from src.utils.instance_generator import generate_instance
    inst = generate_instance(n_customers=20, seed=42)
    print(inst)
    sol, cost, t, hist = solve_ga(inst, time_limit=30)
    print(f"GA Solution cost: {cost:.2f}  ({t:.2f}s)")
    valid, msg = inst.is_solution_valid(sol)
    print(f"Valid: {valid} — {msg}")
