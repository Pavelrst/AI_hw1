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

    # TODO: Plot the total distances with ax1. Use `ax1.plot(...)`.
    # TODO: Make this curve colored blue with solid line style.
    # See documentation here:
    # https://matplotlib.org/2.0.0/api/_as_gen/matplotlib.axes.Axes.plot.html
    # You can also search google for additional examples.
    ax1.plot(weights, total_distance)


    # ax1: Make the y-axis label, ticks and tick labels match the line color.
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
    # TODO:
    # 1. Create an array of 20 numbers equally spreaded in [0.5, 1]
    #    (including the edges). You can use `np.linspace()` for that.
    # 2. For each weight in that array run the A* algorithm, with the
    #    given `heuristic_type` over the map problem. For each such run,
    #    store the cost of the solution (res.final_search_node.cost)
    #    and the number of expanded states (res.nr_expanded_states).
    #    Store these in 2 lists (array for the costs and array for
    #    the #expanded.
    # Call the function `plot_distance_and_expanded_by_weight_figure()`
    #  with that data.
    results = []
    costs = []
    expanded = []
    weights = np.linspace(0.5, 1, 20)
    for weight in weights:
        my_astar = AStar(heuristic_type, weight)
        res = my_astar.solve_problem(problem)
        results.append(res)
        costs.append(res.final_search_node.cost)
        expanded.append(res.nr_expanded_states)

    return weights, costs, expanded



def map_problem():
    print()
    print('Solve the map problem.')

    # Ex.8
    map_prob = MapProblem(roads, 54, 549)
    uc = UniformCost()
    res = uc.solve_problem(map_prob)
    print(res)

    # Ex.10
    # TODO: create an instance of `AStar` with the `NullHeuristic`,
    #       solve the same `map_prob` with it and print the results (as before).
    # Notice: AStar constructor receives the heuristic *type* (ex: `MyHeuristicClass`),
    #         and not an instance of the heuristic (eg: not `MyHeuristicClass()`).
    my_astar = AStar(NullHeuristic)
    res = my_astar.solve_problem(map_prob)
    print(res)


    # Ex.11
    my_astar_air = AStar(AirDistHeuristic)
    res = my_astar_air.solve_problem(map_prob)
    print(res)

    # Ex.12
    weights, dists, exps = run_astar_for_weights_in_range(AirDistHeuristic, map_prob)
    plot_distance_and_expanded_wrt_weight_figure(weights, dists, exps)



# --------------------------------------------------------------------
# ----------------------- Deliveries Problem -------------------------
# --------------------------------------------------------------------

def relaxed_deliveries_problem():

    print()
    print('Solve the relaxed deliveries problem.')

    big_delivery = DeliveriesProblemInput.load_from_file('big_delivery.in', roads)
    big_deliveries_prob = RelaxedDeliveriesProblem(big_delivery)

    # Ex.16
    my_astar_max_air = AStar(MaxAirDistHeuristic)
    res = my_astar_max_air.solve_problem(big_deliveries_prob)
    print(res)

    # Ex.17
    my_astar_mst = AStar(MSTAirDistHeuristic)
    res = my_astar_mst.solve_problem(big_deliveries_prob)
    print(res)

    # Ex.18
    weights, dists, exps = run_astar_for_weights_in_range(MSTAirDistHeuristic, big_deliveries_prob)
    plot_distance_and_expanded_wrt_weight_figure(weights, dists, exps)
    # for next Ex.:
    w05_res = dists[0]
    w1_res = dists[19]

    # Ex.24
    # TODO:
    # 1. Run the stochastic greedy algorithm for 100 times.
    #    For each run, store the cost of the found solution.
    #    Store these costs in a list.
    # 2. The "Anytime Greedy Stochastic Algorithm" runs the greedy
    #     #    greedy stochastic for N times, and after each iteration
    #     #    stores the best solution found so far. It means that after
    #     #    iteration #i, the cost of the solution found by the anytime
    #     #    algorithm is the MINIMUM among the costs of the solutions
    #     #    found in iterations {1,...,i}. Calculate the costs of the
    #     #    anytime algorithm wrt the #iteration and store them in a list.
    num_of_runs = 100
    costs_list = []
    iters_list = []
    min_cost = np.inf
    for iter in range(num_of_runs):
        my_stochastic = GreedyStochastic(MSTAirDistHeuristic)
        res = my_stochastic.solve_problem(big_deliveries_prob)
        #print(res)
        # copied from run_astar_for_weights_in_range
        curr_cost = res.final_search_node.cost
        iters_list.append(iter)
        if curr_cost < min_cost:
            costs_list.append(curr_cost)
            min_cost = curr_cost
        else:
            costs_list.append(min_cost)
    print("costs list:", costs_list)
    print("iters list:", iters_list)

    fig, ax1 = plt.subplots()
    ax1.plot(iters_list, costs_list, 'b',label='Greedy Stochastic')
    ax1.set_ylabel('cost', color='b')
    ax1.tick_params('y', colors='b')
    ax1.set_xlabel('iteration')



    # 3. Calculate and store the cost of the solution received by
    #    the A* algorithm (with w=0.5).
    w05_costs_vec = np.full(num_of_runs, w05_res)

    # 4. Calculate and store the cost of the solution received by
    #    the deterministic greedy algorithm (A* with w=1).
    w1_costs_vec = np.full(num_of_runs, w1_res)

    # 5. Plot a figure with the costs (y-axis) wrt the #iteration
    #    (x-axis). Of course that the costs of A*, and deterministic
    #    greedy are not dependent with the iteration number, so
    #    these two should be represented by horizontal lines.
    ax1.plot(iters_list, w05_costs_vec, 'r',label='A* w=0.5')
    ax1.plot(iters_list, w1_costs_vec, 'g',label='A* w=1')
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
    # TODO: Call here the function `run_astar_for_weights_in_range()`
    #       with `MSTAirDistHeuristic` and `small_deliveries_strict_problem`.

    weights, dists, exps = run_astar_for_weights_in_range(MSTAirDistHeuristic, small_deliveries_strict_problem)
    plot_distance_and_expanded_wrt_weight_figure(weights, dists, exps)
    exit()  # TODO: remove!

    # Ex.28
    # TODO: create an instance of `AStar` with the `RelaxedDeliveriesHeuristic`,
    #       solve the `small_deliveries_strict_problem` with it and print the results (as before).
    exit()  # TODO: remove!


def main():
    #map_problem()
    #TODO - remove upper #
    #relaxed_deliveries_problem()
    strict_deliveries_problem()


if __name__ == '__main__':
    main()
