# Mathematical Formulations – Drone-Based Delivery Optimization

## Problem Definition

Let:
- `n` = number of customers, indexed 1..n
- Node 0 = depot
- `K` = number of drones available
- `V = {0, 1, ..., n}` = set of all nodes (depot + customers)
- `A = {(i,j) : i,j ∈ V, i≠j, (i,j) not a no-fly edge}` = set of allowed arcs
- `d_{ij}` = energy cost (distance) to travel from node i to node j
- `q_i` = demand (weight) of customer i (q_0 = 0)
- `Q` = payload capacity of each drone (max weight per route)
- `B` = battery capacity of each drone (max energy per route)

---

## Formulation 1 — MILP Assignment/Routing Model (Vehicle Routing)

This is a classical **Capacitated Vehicle Routing Problem (CVRP)** formulation.

### Decision Variables

| Variable | Domain | Meaning |
|----------|--------|---------|
| `x_{ijk}` | {0,1} | 1 if drone k travels directly from node i to node j |
| `u_{ik}` | ℝ≥0 | Cumulative load on drone k after visiting node i (MTZ subtour elimination) |

### Objective

```
Minimize  Σ_{k∈K} Σ_{(i,j)∈A} d_{ij} · x_{ijk}
```

Minimize total energy (distance) flown by all drones.

### Constraints

**(C1) – Each customer visited exactly once:**
```
Σ_{k∈K} Σ_{j∈V, j≠i} x_{ijk} = 1     ∀ i ∈ {1,...,n}
```

**(C2) – Flow conservation (each drone enters = exits each node):**
```
Σ_{j∈V, j≠i} x_{ijk} = Σ_{j∈V, j≠i} x_{jik}     ∀ i ∈ V, ∀ k ∈ K
```

**(C3) – Each drone departs from depot at most once:**
```
Σ_{j=1}^{n} x_{0jk} ≤ 1     ∀ k ∈ K
```

**(C4) – Payload capacity:**
```
Σ_{i=1}^{n} Σ_{j∈V, j≠i} q_i · x_{ijk} ≤ Q     ∀ k ∈ K
```

**(C5) – Battery capacity:**
```
Σ_{(i,j)∈A} d_{ij} · x_{ijk} ≤ B     ∀ k ∈ K
```

**(C6) – Subtour elimination (Miller-Tucker-Zemlin):**
```
u_{ik} - u_{jk} + Q · x_{ijk} ≤ Q - q_j     ∀ i,j ∈ {1,...,n}, i≠j, ∀ k ∈ K
q_i ≤ u_{ik} ≤ Q                              ∀ i ∈ {1,...,n}, ∀ k ∈ K
```

**(C7) – No-fly zone enforcement:**
```
x_{ijk} = 0     ∀ (i,j) ∉ A, ∀ k ∈ K
```

**(C8) – Binary and non-negativity:**
```
x_{ijk} ∈ {0,1}     ∀ i,j ∈ V, ∀ k ∈ K
u_{ik} ≥ 0          ∀ i ∈ V, ∀ k ∈ K
```

### Complexity Note
This is an NP-Hard problem (CVRP generalizes TSP). The number of binary variables is O(n²·K).

---

## Formulation 2 — Graph/Flow-Based Model (Set Partitioning)

This formulation takes a **column generation / set partitioning** perspective.
Instead of routing variables per arc, we enumerate feasible routes and select a subset.

### Preprocessing

Let `R` = set of all feasible routes. A route `r ∈ R` is a sequence of customers
`(c_1, c_2, ..., c_p)` starting and ending at depot 0, satisfying:
- `Σ q_{c_l} ≤ Q` (payload)
- `d(0,c_1) + Σ d(c_l,c_{l+1}) + d(c_p,0) ≤ B` (battery)
- All arcs `(c_l, c_{l+1})` are in `A` (no no-fly zones)

For each route `r`, define:
- `c_r` = total energy cost of route r
- `a_{ir}` = 1 if customer i is visited in route r, 0 otherwise

### Decision Variables

| Variable | Domain | Meaning |
|----------|--------|---------|
| `y_r` | {0,1} | 1 if route r is selected |

### Objective

```
Minimize  Σ_{r∈R} c_r · y_r
```

### Constraints

**(D1) – Each customer covered exactly once:**
```
Σ_{r∈R} a_{ir} · y_r = 1     ∀ i ∈ {1,...,n}
```

**(D2) – Fleet size:**
```
Σ_{r∈R} y_r ≤ K
```

**(D3) – Binary:**
```
y_r ∈ {0,1}     ∀ r ∈ R
```

### Key Differences from Formulation 1

| Aspect | Formulation 1 (MILP/Arc) | Formulation 2 (Flow/Set Partition) |
|--------|--------------------------|-------------------------------------|
| Variables | O(n²·K) binary arc vars | O(|R|) binary route vars |
| Subtour elimination | Explicit MTZ constraints | Implicit (routes are pre-validated) |
| No-fly zones | Arc removal | Route feasibility filter |
| Relaxation | LP relaxation on arcs | Column generation (LP over routes) |
| Abstraction level | Arc/edge level | Route/path level |
| Scalability | Better for small-medium n | Better for LP bounds, column gen |

These are **not equivalent reformulations** — they differ in variable space, constraint structure,
and the algorithmic approaches they naturally suggest (B&B with cutting planes vs. column generation).

---

## Complexity Analysis

### Decision Version
**DRONE-DELIVERY**: Given an instance and a bound W, does there exist a feasible assignment
of routes to drones with total energy ≤ W?

### NP-Hardness Proof Sketch
**Theorem**: DRONE-DELIVERY is NP-Hard.

**Proof**: By reduction from BIN-PACKING.
- Given a bin-packing instance with items (s_1,...,s_n) and bin capacity C:
- Create n customers with demands s_i, place all at the same location (zero distances).
- Set Q = C (payload), B = ∞ (no battery limit), K = number of bins allowed.
- A feasible drone delivery exists iff there exists a valid bin packing.
- Since BIN-PACKING is NP-Complete, DRONE-DELIVERY is NP-Hard. ∎

### Membership in NP
Given a solution (set of routes), we can verify feasibility in polynomial time:
- Check each customer appears exactly once: O(n·K)
- Check payload per route: O(n)
- Check battery per route: O(n)
- Check no-fly zones: O(n·|forbidden|)

Thus DRONE-DELIVERY ∈ NP, and therefore **DRONE-DELIVERY is NP-Complete**.

### Practical Implication
Exact methods (B&B) are tractable only for n ≤ ~15. For larger instances,
metaheuristics are the practical choice.
