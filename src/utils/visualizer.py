"""
visualizer.py
Visualization utilities for Drone Delivery routes.
"""

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
import os


COLORS = [
    '#e74c3c', '#3498db', '#2ecc71', '#f39c12', '#9b59b6',
    '#1abc9c', '#e67e22', '#34495e', '#e91e63', '#00bcd4'
]


def plot_solution(instance, solution, title="Drone Delivery Solution",
                  save_path=None, show=False):
    """
    Plot the drone delivery routes on a 2D map.

    Parameters
    ----------
    instance : DroneInstance
    solution : list of list of int  (each inner list = one drone's route)
    title : str
    save_path : str or None
    show : bool
    """
    fig, ax = plt.subplots(1, 1, figsize=(9, 8))
    ax.set_facecolor('#f8f9fa')
    fig.patch.set_facecolor('#ffffff')

    # Plot no-fly zone edges
    n = instance.n_customers + 1
    xs = [instance.depot_x] + [c.x for c in instance.customers]
    ys = [instance.depot_y] + [c.y for c in instance.customers]

    for (i, j) in instance.no_fly_edges:
        ax.plot([xs[i], xs[j]], [ys[i], ys[j]],
                color='#cc0000', linewidth=1.2, linestyle='--',
                alpha=0.4, zorder=1)

    # Plot routes
    for k, route in enumerate(solution):
        if not route:
            continue
        color = COLORS[k % len(COLORS)]
        nodes = [0] + route + [0]
        rx = [xs[n_] for n_ in nodes]
        ry = [ys[n_] for n_ in nodes]

        ax.plot(rx, ry, color=color, linewidth=2.0,
                alpha=0.8, zorder=2, label=f'Drone {k+1}')

        # Arrows for direction
        for seg in range(len(nodes) - 1):
            dx = rx[seg+1] - rx[seg]
            dy = ry[seg+1] - ry[seg]
            ax.annotate('', xy=(rx[seg+1], ry[seg+1]),
                        xytext=(rx[seg], ry[seg]),
                        arrowprops=dict(arrowstyle='->', color=color,
                                        lw=1.5, mutation_scale=15),
                        zorder=3)

    # Plot customers
    for c in instance.customers:
        ax.scatter(c.x, c.y, s=120, c='#2c3e50', zorder=5,
                   edgecolors='white', linewidths=1.5)
        ax.annotate(f'{c.idx}', (c.x, c.y),
                    textcoords='offset points', xytext=(6, 6),
                    fontsize=8, color='#2c3e50', fontweight='bold')

    # Plot depot
    ax.scatter(instance.depot_x, instance.depot_y,
               s=300, c='#f39c12', zorder=6,
               marker='*', edgecolors='#e67e22', linewidths=2)
    ax.annotate('DEPOT', (instance.depot_x, instance.depot_y),
                textcoords='offset points', xytext=(8, -14),
                fontsize=10, color='#e67e22', fontweight='bold')

    # Legend and labels
    legend_handles = []
    for k, route in enumerate(solution):
        if route:
            cost = instance.route_cost(route)
            demand = instance.route_demand(route)
            patch = mpatches.Patch(
                color=COLORS[k % len(COLORS)],
                label=f'Drone {k+1}: {len(route)} stops, '
                      f'dist={cost:.1f}, load={demand:.1f}'
            )
            legend_handles.append(patch)

    total_cost = instance.solution_cost(solution)
    ax.legend(handles=legend_handles, loc='upper left',
              fontsize=9, framealpha=0.9)

    ax.set_title(f'{title}\nTotal Energy: {total_cost:.2f}',
                 fontsize=12, fontweight='bold', pad=12)
    ax.set_xlabel('X coordinate')
    ax.set_ylabel('Y coordinate')
    ax.set_xlim(-5, 105)
    ax.set_ylim(-5, 105)
    ax.grid(True, alpha=0.3, linewidth=0.5)

    plt.tight_layout()

    if save_path:
        os.makedirs(os.path.dirname(save_path) if os.path.dirname(save_path) else '.', exist_ok=True)
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
    if show:
        plt.show()
    plt.close()
    return fig


def plot_convergence(history_dict, title="Convergence", save_path=None, show=False):
    """
    Plot convergence curves for one or more methods.

    Parameters
    ----------
    history_dict : dict  {method_name: list_of_best_costs_over_time}
    title : str
    save_path : str or None
    show : bool
    """
    fig, ax = plt.subplots(figsize=(10, 5))
    colors = ['#e74c3c', '#3498db', '#2ecc71', '#f39c12']

    for idx, (name, history) in enumerate(history_dict.items()):
        ax.plot(history, color=colors[idx % len(colors)],
                linewidth=2, label=name, alpha=0.9)

    ax.set_title(title, fontsize=13, fontweight='bold')
    ax.set_xlabel('Iteration / Generation')
    ax.set_ylabel('Best Cost')
    ax.legend(fontsize=10)
    ax.grid(True, alpha=0.3)
    plt.tight_layout()

    if save_path:
        os.makedirs(os.path.dirname(save_path) if os.path.dirname(save_path) else '.', exist_ok=True)
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
    if show:
        plt.show()
    plt.close()
    return fig


def plot_comparison_bar(results_df, save_path=None, show=False):
    """
    Bar chart comparing methods across instances.

    Parameters
    ----------
    results_df : pandas DataFrame with columns [instance, method, cost, time]
    """
    import pandas as pd
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    methods = results_df['method'].unique()
    instances = results_df['instance'].unique()

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))

    # Cost comparison
    ax = axes[0]
    x = np.arange(len(instances))
    width = 0.8 / len(methods)
    for i, method in enumerate(methods):
        vals = []
        for inst in instances:
            row = results_df[(results_df['instance'] == inst) &
                             (results_df['method'] == method)]
            vals.append(row['cost'].values[0] if len(row) > 0 else 0)
        ax.bar(x + i * width, vals, width, label=method,
               color=COLORS[i % len(COLORS)], alpha=0.85)

    ax.set_xticks(x + width * (len(methods) - 1) / 2)
    ax.set_xticklabels([inst.replace('instance_', '') for inst in instances],
                       rotation=45, ha='right', fontsize=8)
    ax.set_title('Solution Cost by Method and Instance', fontweight='bold')
    ax.set_ylabel('Total Energy Cost')
    ax.legend()
    ax.grid(True, alpha=0.3, axis='y')

    # Time comparison
    ax = axes[1]
    for i, method in enumerate(methods):
        vals = []
        for inst in instances:
            row = results_df[(results_df['instance'] == inst) &
                             (results_df['method'] == method)]
            vals.append(row['time'].values[0] if len(row) > 0 else 0)
        ax.bar(x + i * width, vals, width, label=method,
               color=COLORS[i % len(COLORS)], alpha=0.85)

    ax.set_xticks(x + width * (len(methods) - 1) / 2)
    ax.set_xticklabels([inst.replace('instance_', '') for inst in instances],
                       rotation=45, ha='right', fontsize=8)
    ax.set_title('Computation Time by Method and Instance', fontweight='bold')
    ax.set_ylabel('Time (seconds)')
    ax.legend()
    ax.grid(True, alpha=0.3, axis='y')

    plt.tight_layout()
    if save_path:
        os.makedirs(os.path.dirname(save_path) if os.path.dirname(save_path) else '.', exist_ok=True)
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
    if show:
        plt.show()
    plt.close()
    return fig
