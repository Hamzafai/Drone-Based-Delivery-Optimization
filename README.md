# Drone-Based Delivery Optimization Problem
## ENSIA – Third Year – Combinatorial Optimization and Metaheuristics – 2026

---

## Project Structure

```
drone_delivery_project/
├── README.md                        ← This file
├── requirements.txt                 ← Python dependencies
├── run_experiments.py               ← Main experiment runner
├── instances/                       ← Generated test instances (JSON)
├── results/                         ← Output CSV + plots
├── report/
│   └── formulations.md              ← Mathematical formulations (LaTeX-ready)
└── src/
    ├── utils/
    │   ├── instance_generator.py    ← Random instance generator
    │   ├── data_structures.py       ← Problem data classes
    │   └── visualizer.py            ← Route visualization
    ├── exact/
    │   └── branch_and_bound.py      ← B&B exact solver
    └── metaheuristics/
        ├── genetic_algorithm.py     ← Population-based metaheuristic
        └── simulated_annealing.py   ← Local search metaheuristic
```

---

## Problem Summary

A fleet of drones must deliver packages from a central **depot** (node 0) to a set of **customers**.
Each drone:
- Starts and ends at the depot
- Has a **battery capacity** (max total distance per route)
- Has a **payload capacity** (max total weight per route)

**Objective**: Minimize total energy consumption (proportional to total distance flown).

**No-fly zones** are modeled as forbidden edges (infinite cost / removed from the graph).

---

## How to Run

### 1. Install dependencies
```bash
pip install -r requirements.txt
```

### 2. Generate instances
```bash
python run_experiments.py --mode generate
```

### 3. Run all solvers on all instances
```bash
python run_experiments.py --mode solve
```

### 4. Run a quick demo (one instance, all solvers)
```bash
python run_experiments.py --mode demo --instance instances/instance_01.json
```

### 5. Full experiment + plots
```bash
python run_experiments.py --mode full
```

---

## Assumptions

| Parameter | Value / Model |
|-----------|---------------|
| Energy consumption | Proportional to Euclidean distance: `E = d × weight_factor` |
| Weight factor | 1.0 (uniform energy per unit distance) |
| Depot | Node index 0, located at (50, 50) in a 100×100 grid |
| Customer positions | Uniformly random in [0,100]² |
| Demand per customer | Uniform in [1, 10] kg |
| Battery capacity | Scales with instance size (see generator) |
| Payload capacity | 30 kg per drone |
| No-fly zones | Random rectangular forbidden zones |

---

## Methods Implemented

| Method | Type | Notes |
|--------|------|-------|
| Branch & Bound | Exact | With lower bound via assignment relaxation |
| Genetic Algorithm | Population-based | Order crossover (OX), adaptive mutation |
| Simulated Annealing | Local search | 2-opt + Or-opt neighborhood |

---

## Complexity

The Drone Delivery Problem is a variant of the **Capacitated Vehicle Routing Problem (CVRP)**,
which is **NP-Hard** (reduction from bin-packing / TSP).
Exact methods are feasible only for small instances (n ≤ 15).
