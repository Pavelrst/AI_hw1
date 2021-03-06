from framework import *
from deliveries import *

from matplotlib import pyplot as plt
import numpy as np
from typing import List, Union

# Load the map
roads = load_map_from_csv(Consts.get_data_file_path("tlv.csv"))

# Make `np.random` behave deterministic.
Consts.set_seed()


# --------------------------------------------------------------------
# -------------------------- Map Problem -----------------------------
# --------------------------------------------------------------------

def plot_distance_and_expanded_wrt_weight_figure(
        weights: Union[np.ndarray, List[float]],
        total_distance: Union[np.ndarray, List[float]],
        total_expanded: Union[np.ndarray, List[int]]):
    """
    Use `matplotlib` to generate a figure of the distance & #expanded-nodes
     w.r.t. the weight.
    """
    assert len(weights) == len(total_distance) == len(total_expanded)

    fig, ax1 = plt.subplots()

    ax1.plot(weights, total_distance)
    ax1.set_ylabel('distance traveled', color='b')
    ax1.tick_params('y', colors='b')
    ax1.set_xlabel('weight')

    # Create another axis for the #expanded curve.
    ax2 = ax1.twinx()

    # TODO: Plot the total expanded with ax2. Use `ax2.plot(...)`.
    # TODO: ax2: Make the y-axis label, ticks and tick labels match the line color.
    # TODO: Make this curve colored red with solid line style.
    ax2.plot(weights, total_expanded,'r')
    ax2.set_ylabel('states expanded', color='r')
    ax2.tick_params('y', colors='r')

    fig.tight_layout()
    plt.show()


def run_astar_for_weights_in_range(heuristic_type: HeuristicFunctionType, problem: GraphProblem):
    results = []
    costs = []
    num_expanded = []
    weights = np.linspace(0.5, 1, 20)
    for weight in weights:
        my_astar = AStar(heuristic_type, weight)
        res = my_astar.solve_problem(problem)
        results.append(res)
        costs.append(res.final_search_node.cost)
        num_expanded.append(res.nr_expanded_states)

    return weights, costs, num_expanded


def map_problem():
    print()
    print('Solve the map problem.')

    # Ex.8
    map_prob = MapProblem(roads, 54, 549)
    uc = UniformCost()
    res = uc.solve_problem(map_prob)
    print(res)

    # Ex.10
    null_astar = AStar(NullHeuristic)
    res = null_astar.solve_problem(map_prob)
    print(res)


    # Ex.11
    air_dist_astar = AStar(AirDistHeuristic)
    res = air_dist_astar.solve_problem(map_prob)
    print(res)

    # Ex.12
    weights, dists, nums_expanded = run_astar_for_weights_in_range(AirDistHeuristic, map_prob)
    plot_distance_and_expanded_wrt_weight_figure(weights, dists, nums_expanded)



# --------------------------------------------------------------------
# ----------------------- Deliveries Problem -------------------------
# --------------------------------------------------------------------

def relaxed_deliveries_problem():

    print()
    print('Solve the relaxed deliveries problem.')

    big_delivery = DeliveriesProblemInput.load_from_file('big_delivery.in', roads)
    big_deliveries_prob = RelaxedDeliveriesProblem(big_delivery)

    # Ex.16
    max_air_astar = AStar(MaxAirDistHeuristic)
    res = max_air_astar.solve_problem(big_deliveries_prob)
    print(res)

    # Ex.17
    mst_astar = AStar(MSTAirDistHeuristic)
    res = mst_astar.solve_problem(big_deliveries_prob)
    print(res)

    # Ex.18
    weights, dists, nums_expanded = run_astar_for_weights_in_range(MSTAirDistHeuristic, big_deliveries_prob)
    plot_distance_and_expanded_wrt_weight_figure(weights, dists, nums_expanded)
    # for next Ex.:
    w05_res = dists[0]
    w1_res = dists[19]

    # Ex.24
    num_of_runs = 100
    costs_list = []
    greedy_costs_list = []
    iters_list = []
    min_cost = np.inf
    for iter in range(num_of_runs):
        my_stochastic = GreedyStochastic(MSTAirDistHeuristic)
        res = my_stochastic.solve_problem(big_deliveries_prob)
        curr_cost = res.final_search_node.cost
        iters_list.append(iter)
        greedy_costs_list.append(curr_cost)
        if curr_cost < min_cost:
            costs_list.append(curr_cost)
            min_cost = curr_cost
        else:
            costs_list.append(min_cost)
    #print("costs list:", costs_list)
    #print("iters list:", iters_list)

    fig, ax1 = plt.subplots()
    ax1.plot(iters_list, costs_list, 'b', label='Anytime')
    ax1.set_ylabel('cost', color='b')
    ax1.tick_params('y', colors='b')
    ax1.set_xlabel('iteration')

    # Calculate and store the cost of the solution received by the A* algorithm (with w=0.5).
    w05_costs_vec = np.full(num_of_runs, w05_res)

    # Calculate and store the cost of the solution received by the deterministic greedy algorithm (A* with w=1).
    w1_costs_vec = np.full(num_of_runs, w1_res)

    # Plot a figure with the costs (y-axis) wrt the #iteration
    #    (x-axis). Of course that the costs of A*, and deterministic
    #    greedy are not dependent with the iteration number, so
    #    these two should be represented by horizontal lines.
    ax1.plot(iters_list, w05_costs_vec, 'r', label='A* w=0.5')
    ax1.plot(iters_list, w1_costs_vec, 'g', label='A* w=1')
    ax1.plot(iters_list, greedy_costs_list, 'm', label='Greedy Stochastic')
    plt.legend()
    fig.tight_layout()
    plt.show()


def strict_deliveries_problem():
    print()
    print('Solve the strict deliveries problem.')

    small_delivery = DeliveriesProblemInput.load_from_file('small_delivery.in', roads)
    small_deliveries_strict_problem = StrictDeliveriesProblem(
        small_delivery, roads, inner_problem_solver=AStar(AirDistHeuristic))

    # Ex.26
    weights, dists, nums_expanded = run_astar_for_weights_in_range(MSTAirDistHeuristic, small_deliveries_strict_problem)
    plot_distance_and_expanded_wrt_weight_figure(weights, dists, nums_expanded)

    # Ex.28
    weights, dists, nums_expanded = run_astar_for_weights_in_range(RelaxedDeliveriesHeuristic, small_deliveries_strict_problem)
    plot_distance_and_expanded_wrt_weight_figure(weights, dists, nums_expanded)

    mst_astar = AStar(MSTAirDistHeuristic)
    res = mst_astar.solve_problem(small_deliveries_strict_problem)
    print(res)

    rdp_astar = AStar(RelaxedDeliveriesHeuristic)
    res = rdp_astar.solve_problem(small_deliveries_strict_problem)
    print(res)


def main():
    map_problem()
    relaxed_deliveries_problem()
    strict_deliveries_problem()


if __name__ == '__main__':
    main()
