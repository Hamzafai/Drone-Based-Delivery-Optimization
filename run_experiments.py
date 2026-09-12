"""
run_experiments.py
Main entry point for all experiments.

Usage:
  python run_experiments.py --mode generate         # generate instances
  python run_experiments.py --mode solve            # run all solvers
  python run_experiments.py --mode demo             # quick demo
  python run_experiments.py --mode full             # generate + solve + report
"""

import argparse
import os
import sys
import time
import json
import csv
from copy import deepcopy

import numpy as np

# ─── Path setup ─────────────────────────────────────────────────────────────
PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, PROJECT_ROOT)

from src.utils.instance_generator import generate_benchmark_suite, generate_instance
from src.utils.data_structures import DroneInstance
from src.exact.branch_and_bound import solve_bb
from src.metaheuristics.genetic_algorithm import solve_ga
from src.metaheuristics.simulated_annealing import solve_sa
from src.utils.visualizer import plot_solution, plot_convergence, plot_comparison_bar

# ─── Configuration ─────────────────────────────────────────────────────────

INSTANCES_DIR = os.path.join(PROJECT_ROOT, "instances")
RESULTS_DIR   = os.path.join(PROJECT_ROOT, "results")

# Instance sizes (small for B&B, larger for metaheuristics)
SMALL_SIZES  = [5, 7, 8, 10, 12]      # B&B tractable
LARGE_SIZES  = [15, 20, 30, 50, 75]   # metaheuristics only
ALL_SIZES    = SMALL_SIZES + LARGE_SIZES

BB_TIME_LIMIT = 60    # seconds per instance for B&B
GA_TIME_LIMIT = 45    # seconds per instance for GA
SA_TIME_LIMIT = 45    # seconds per instance for SA
SEEDS_PER_SIZE = 1    # instances per size (set to 2 for richer study)


# ─── Experiment Functions ───────────────────────────────────────────────────

def run_generate():
    print("=" * 60)
    print("GENERATING BENCHMARK INSTANCES")
    print("=" * 60)
    os.makedirs(INSTANCES_DIR, exist_ok=True)
    instances = generate_benchmark_suite(
        output_dir=INSTANCES_DIR,
        sizes=ALL_SIZES,
        seeds_per_size=SEEDS_PER_SIZE
    )
    print(f"\nInstances saved to: {INSTANCES_DIR}")
    return instances


def run_solver_on_instance(inst_path, run_bb=True):
    """Run all solvers on a single instance. Returns dict of results."""
    inst = DroneInstance.load(inst_path)
    inst_name = os.path.splitext(os.path.basename(inst_path))[0]
    n = inst.n_customers

    print(f"\n{'─'*55}")
    print(f"Instance: {inst_name}  (n={n})")
    print(f"  {inst}")

    results = {}

    # ── Branch & Bound ────────────────────────────────────────
    if run_bb and n <= 12:
        print(f"\n  Running Branch & Bound (limit={BB_TIME_LIMIT}s)...")
        sol_bb, cost_bb, t_bb = solve_bb(inst, time_limit=BB_TIME_LIMIT, verbose=True)
        valid_bb, _ = inst.is_solution_valid(sol_bb) if sol_bb else (False, "No solution")
        results['BranchAndBound'] = {
            'cost': cost_bb, 'time': t_bb, 'valid': valid_bb,
            'solution': sol_bb, 'n_routes': len(sol_bb) if sol_bb else 0
        }
        # Save route plot
        if sol_bb:
            plot_solution(inst, sol_bb,
                          title=f"B&B — {inst_name}",
                          save_path=os.path.join(RESULTS_DIR, f"{inst_name}_bb.png"))
    else:
        results['BranchAndBound'] = {'cost': None, 'time': None, 'valid': None,
                                      'solution': None, 'n_routes': None}

    # ── Genetic Algorithm ─────────────────────────────────────
    print(f"\n  Running Genetic Algorithm (limit={GA_TIME_LIMIT}s)...")
    sol_ga, cost_ga, t_ga, hist_ga = solve_ga(
        inst, pop_size=80, max_generations=500,
        time_limit=GA_TIME_LIMIT, verbose=True
    )
    valid_ga, _ = inst.is_solution_valid(sol_ga) if sol_ga else (False, "No solution")
    results['GeneticAlgorithm'] = {
        'cost': cost_ga, 'time': t_ga, 'valid': valid_ga,
        'solution': sol_ga, 'n_routes': len(sol_ga) if sol_ga else 0,
        'history': hist_ga
    }
    if sol_ga:
        plot_solution(inst, sol_ga,
                      title=f"GA — {inst_name}",
                      save_path=os.path.join(RESULTS_DIR, f"{inst_name}_ga.png"))

    # ── Simulated Annealing ───────────────────────────────────
    print(f"\n  Running Simulated Annealing (limit={SA_TIME_LIMIT}s)...")
    sol_sa, cost_sa, t_sa, hist_sa = solve_sa(
        inst, time_limit=SA_TIME_LIMIT, verbose=True
    )
    valid_sa, _ = inst.is_solution_valid(sol_sa) if sol_sa else (False, "No solution")
    results['SimulatedAnnealing'] = {
        'cost': cost_sa, 'time': t_sa, 'valid': valid_sa,
        'solution': sol_sa, 'n_routes': len(sol_sa) if sol_sa else 0,
        'history': hist_sa
    }
    if sol_sa:
        plot_solution(inst, sol_sa,
                      title=f"SA — {inst_name}",
                      save_path=os.path.join(RESULTS_DIR, f"{inst_name}_sa.png"))

    # ── Convergence Plot ──────────────────────────────────────
    history_dict = {}
    if hist_ga:
        history_dict['Genetic Algorithm'] = hist_ga
    if hist_sa:
        history_dict['Simulated Annealing'] = hist_sa
    if history_dict:
        plot_convergence(
            history_dict,
            title=f"Convergence — {inst_name}",
            save_path=os.path.join(RESULTS_DIR, f"{inst_name}_convergence.png")
        )

    return inst_name, n, results


def run_all_experiments():
    """Run all solvers on all instances and save results."""
    print("=" * 60)
    print("RUNNING ALL EXPERIMENTS")
    print("=" * 60)
    os.makedirs(RESULTS_DIR, exist_ok=True)

    inst_files = sorted([
        os.path.join(INSTANCES_DIR, f)
        for f in os.listdir(INSTANCES_DIR)
        if f.endswith('.json')
    ])

    if not inst_files:
        print("No instances found! Run with --mode generate first.")
        return

    all_rows = []

    for inst_path in inst_files:
        inst_name, n, results = run_solver_on_instance(inst_path, run_bb=True)

        for method, data in results.items():
            all_rows.append({
                'instance': inst_name,
                'n_customers': n,
                'method': method,
                'cost': data['cost'] if data['cost'] is not None else '',
                'time': data['time'] if data['time'] is not None else '',
                'valid': data['valid'] if data['valid'] is not None else '',
                'n_routes': data['n_routes'] if data['n_routes'] is not None else '',
            })

    # Save CSV
    csv_path = os.path.join(RESULTS_DIR, "experiment_results.csv")
    with open(csv_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=['instance', 'n_customers', 'method',
                                                'cost', 'time', 'valid', 'n_routes'])
        writer.writeheader()
        writer.writerows(all_rows)

    print(f"\n\nResults saved to: {csv_path}")

    # Summary table
    print_summary(all_rows)

    # Comparison plots
    try:
        import pandas as pd
        df = pd.read_csv(csv_path)
        df = df[df['cost'] != '']
        df['cost'] = df['cost'].astype(float)
        df['time'] = df['time'].astype(float)
        plot_comparison_bar(df, save_path=os.path.join(RESULTS_DIR, "comparison_bar.png"))
        print(f"Comparison plot saved to results/comparison_bar.png")
    except Exception as e:
        print(f"Could not generate comparison plot: {e}")


def print_summary(rows):
    """Print a formatted summary table."""
    print("\n" + "=" * 80)
    print("SUMMARY TABLE")
    print("=" * 80)
    print(f"{'Instance':<30} {'n':>4}  {'Method':<20} {'Cost':>10}  {'Time(s)':>8}  {'Valid':>6}")
    print("-" * 80)
    for row in rows:
        cost_str = f"{float(row['cost']):.2f}" if row['cost'] != '' else "  N/A"
        time_str = f"{float(row['time']):.2f}" if row['time'] != '' else "  N/A"
        valid_str = str(row['valid']) if row['valid'] != '' else "  N/A"
        print(f"{row['instance']:<30} {row['n_customers']:>4}  "
              f"{row['method']:<20} {cost_str:>10}  {time_str:>8}  {valid_str:>6}")
    print("=" * 80)


def run_demo(inst_path=None):
    """Quick demo: one small instance, all solvers, visual output."""
    print("=" * 60)
    print("DEMO MODE")
    print("=" * 60)
    os.makedirs(RESULTS_DIR, exist_ok=True)

    if inst_path and os.path.exists(inst_path):
        inst = DroneInstance.load(inst_path)
    else:
        print("Generating a demo instance (n=10)...")
        inst = generate_instance(n_customers=10, seed=7)
        demo_path = os.path.join(INSTANCES_DIR, "demo_instance.json")
        os.makedirs(INSTANCES_DIR, exist_ok=True)
        inst.save(demo_path)
        inst_path = demo_path

    print(f"Instance: {inst}")
    run_solver_on_instance(inst_path, run_bb=True)
    print(f"\nAll output saved in: {RESULTS_DIR}/")


# ─── Main ──────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Drone Delivery Optimization – Experiment Runner"
    )
    parser.add_argument('--mode', choices=['generate', 'solve', 'demo', 'full'],
                        default='full',
                        help="Execution mode")
    parser.add_argument('--instance', type=str, default=None,
                        help="Path to instance file (for demo mode)")
    args = parser.parse_args()

    if args.mode == 'generate':
        run_generate()
    elif args.mode == 'solve':
        run_all_experiments()
    elif args.mode == 'demo':
        run_demo(args.instance)
    elif args.mode == 'full':
        run_generate()
        run_all_experiments()


if __name__ == "__main__":
    main()
