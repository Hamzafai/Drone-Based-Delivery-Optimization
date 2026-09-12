# Drone-Based Delivery Optimization (DDOP)

Combinatorial optimization course project — ENSIA, 3rd year (Numerical Methods and Optimisation).

## Problem

DDOP is a variant of the Capacitated Vehicle Routing Problem (CVRP): a fleet of drones departs from a central depot to serve customers with known demand, subject to:
- **Payload capacity** (kg per route)
- **Battery/range limit** (max distance per route)
- **No-fly zones** (forbidden arcs)

Objective: minimize total energy (distance) across all drone routes.

DDOP is proven **NP-Hard** via a polynomial reduction from Bin Packing.

## Formulations

- **Arc-flow MILP** — classical CVRP-style model with MTZ subtour elimination constraints (`O(n²K)` binary variables).
- **Set-partitioning / flow model** — route-level formulation, no subtour constraints, natural for column generation (`O(|R|)` variables).

## Methods Implemented

| Method | Type | Notes |
|---|---|---|
| Branch & Bound | Exact | Assignment-relaxation lower bound, greedy NN upper bound init. Tractable for n ≲ 12. |
| Genetic Algorithm | Metaheuristic | Giant Tour encoding, Order Crossover (OX), adaptive mutation, embedded Or-opt local search, elitism. |
| Simulated Annealing | Metaheuristic | Explicit route representation, 4 neighborhood operators (2-opt, Or-opt relocation, cross-exchange, route merge), geometric cooling with reheating. |

## Results

Benchmarked on 10 instances (n = 5 to 75 customers).

- GA and SA match or beat B&B on small instances within B&B's time budget.
- SA is competitive with GA for n ≤ 20.
- **GA outperforms SA by 10–15% on large instances (n ≥ 30)**, thanks to population diversity and embedded Or-opt local search.

| n | B&B | GA | SA |
|---|---|---|---|
| 5 | 248.96 | 248.96 | 248.96 |
| 10 | 506.78 | 483.52 | 483.52 |
| 30 | N/A | 941.52 | 1052.90 |
| 75 | N/A | 2271.19 | 2414.25 |

## Authors

Hamza Faiz Ahmed Fouatih, Youcef Belaib, Dhiaa Eddine Zeroual, Ishak Dib — ENSIA, Department of Intelligent Systems Engineering.

## References

Full report and slides included in this repo.
