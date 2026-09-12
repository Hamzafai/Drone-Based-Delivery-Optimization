"""
instance_generator.py
Generates random Drone Delivery Problem instances.
"""

import random
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from src.utils.data_structures import DroneInstance, Customer


def generate_instance(n_customers, seed=None, no_fly_zone_prob=0.05,
                      area_size=100, depot_pos=(50, 50)):
    """
    Generate a random DroneInstance.

    Parameters
    ----------
    n_customers : int
        Number of customers (excluding depot).
    seed : int or None
        Random seed for reproducibility.
    no_fly_zone_prob : float
        Probability that any edge is a no-fly zone.
    area_size : float
        Grid size (positions in [0, area_size]²).
    depot_pos : tuple
        (x, y) coordinates of the depot.

    Returns
    -------
    DroneInstance
    """
    rng = random.Random(seed)
    np_rng = __import__('numpy').random.default_rng(seed)

    # Generate customer positions and demands
    customers = []
    for i in range(1, n_customers + 1):
        x = rng.uniform(0, area_size)
        y = rng.uniform(0, area_size)
        demand = round(rng.uniform(1, 10), 2)
        customers.append(Customer(i, x, y, demand))

    # Payload capacity: enough to serve ~3 customers on average
    avg_demand = sum(c.demand for c in customers) / n_customers
    payload_capacity = round(3.0 * avg_demand + rng.uniform(0, avg_demand), 2)
    payload_capacity = max(payload_capacity, max(c.demand for c in customers))

    # Battery capacity: based on max round-trip from depot
    depot_x, depot_y = depot_pos
    max_dist_from_depot = max(
        math.sqrt((c.x - depot_x)**2 + (c.y - depot_y)**2)
        for c in customers
    )
    # Battery covers ~2 hops + return with margin
    battery_capacity = round(2.5 * max_dist_from_depot + 0.3 * area_size, 2)

    # No-fly edges: only between customer-customer pairs (never depot edges)
    # This ensures every customer is always reachable from the depot.
    no_fly_edges = []
    customer_edges = [(i, j) for i in range(1, n_customers + 1)
                      for j in range(i + 1, n_customers + 1)]
    for edge in customer_edges:
        if rng.random() < no_fly_zone_prob:
            no_fly_edges.append(edge)

    return DroneInstance(
        n_customers=n_customers,
        depot_x=depot_x,
        depot_y=depot_y,
        customers=customers,
        payload_capacity=payload_capacity,
        battery_capacity=battery_capacity,
        no_fly_edges=no_fly_edges
    )


def generate_benchmark_suite(output_dir, sizes=None, seeds_per_size=2):
    """
    Generate a suite of benchmark instances and save them to disk.

    Parameters
    ----------
    output_dir : str
        Directory to save .json instance files.
    sizes : list of int
        Customer counts for which to generate instances.
    seeds_per_size : int
        Number of random seeds per size.
    """
    if sizes is None:
        sizes = [5, 8, 10, 12, 15, 20, 30, 50, 75, 100]

    os.makedirs(output_dir, exist_ok=True)
    instances = []

    instance_id = 1
    for n in sizes:
        for seed in range(seeds_per_size):
            inst = generate_instance(n_customers=n, seed=seed * 100 + n)
            filename = f"instance_{instance_id:02d}_n{n}_s{seed}.json"
            path = os.path.join(output_dir, filename)
            inst.save(path)
            instances.append((filename, n, seed, path))
            print(f"  Generated: {filename}  ({inst})")
            instance_id += 1

    print(f"\nTotal instances generated: {len(instances)}")
    return instances


if __name__ == "__main__":
    print("Generating benchmark instances...")
    generate_benchmark_suite("instances")
