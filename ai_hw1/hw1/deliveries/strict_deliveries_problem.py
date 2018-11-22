from framework.graph_search import *
from framework.ways import *
from .map_problem import MapProblem
from .deliveries_problem_input import DeliveriesProblemInput
from .relaxed_deliveries_problem import RelaxedDeliveriesState, RelaxedDeliveriesProblem

from typing import Set, FrozenSet, Optional, Iterator, Tuple, Union


class StrictDeliveriesState(RelaxedDeliveriesState):
    """
    An instance of this class represents a state of the strict
     deliveries problem.
    This state is basically similar to the state of the relaxed
     problem. Hence, this class inherits from `RelaxedDeliveriesState`.

    TODO:
        If you believe you need to modify the state for the strict
         problem in some sense, please go ahead and do so.
    """
    pass


class StrictDeliveriesProblem(RelaxedDeliveriesProblem):
    """
    An instance of this class represents a strict deliveries problem.
    """

    name = 'StrictDeliveries'

    def __init__(self, problem_input: DeliveriesProblemInput, roads: Roads,
                 inner_problem_solver: GraphProblemSolver, use_cache: bool = True):
        super(StrictDeliveriesProblem, self).__init__(problem_input)
        self.initial_state = StrictDeliveriesState(
            problem_input.start_point, frozenset(), problem_input.gas_tank_init_fuel)
        self.inner_problem_solver = inner_problem_solver
        self.roads = roads
        self.use_cache = use_cache
        self._init_cache()

    def _init_cache(self):
        self._cache = {}
        self.nr_cache_hits = 0
        self.nr_cache_misses = 0

    def _insert_to_cache(self, key, val):
        if self.use_cache:
            self._cache[key] = val

    def _get_from_cache(self, key):
        if not self.use_cache:
            return None
        if key in self._cache:
            self.nr_cache_hits += 1
        else:
            self.nr_cache_misses += 1
        return self._cache.get(key)

    def expand_state_with_costs(self, state_to_expand: GraphProblemState) -> Iterator[Tuple[GraphProblemState, float]]:
        """
        TODO: implement this method!
        This method represents the `Succ: S -> P(S)` function of the strict deliveries problem.
        The `Succ` function is defined by the problem operators as shown in class.
        The relaxed problem operators are defined in the assignment instructions.
        It receives a state and iterates over the successor states.
        Notice that this is an *Iterator*. Hence it should be implemented using the `yield` keyword.
        For each successor, a pair of the successor state and the operator cost is yielded.
        """
        assert isinstance(state_to_expand, StrictDeliveriesState)

        # Uri - copied from relaxed_deliveries_prolem.py
        assert isinstance(state_to_expand, RelaxedDeliveriesState)

        small_delivery = DeliveriesProblemInput.load_from_file('small_delivery.in', self.roads)


        for stop in self.possible_stop_points:
            # Uri - modification; try to read from cache
            dist = self._get_from_cache((state_to_expand, stop))
            if dist is None:
                #inner_astar = AStar(AirDistHeuristic)
                #res = inner_astar.solve_problem(self.roads)
                #dist = res.final_search_node.cost

                res = self.inner_problem_solver.solve_problem()
                dist = res.final_search_node.cost
                self._insert_to_cache((state_to_expand, stop), dist)
            # ^ end of Uri modification
            if state_to_expand.fuel > dist:
                if stop in self.gas_stations:
                    next_state = RelaxedDeliveriesState(stop, state_to_expand.dropped_so_far, self.gas_tank_capacity)
                    yield [next_state, dist]
                elif stop not in state_to_expand.dropped_so_far:
                    new_dropped_so_far = set(e for e in state_to_expand.dropped_so_far)
                    new_dropped_so_far.add(stop)
                    assert len(new_dropped_so_far) == len(state_to_expand.dropped_so_far) + 1
                    next_state = RelaxedDeliveriesState(stop, new_dropped_so_far, state_to_expand.fuel - dist)
                    yield [next_state, dist]

    def is_goal(self, state: GraphProblemState) -> bool:
        assert isinstance(state, StrictDeliveriesState)

        if len(state.dropped_so_far) == len(self.drop_points):
            return True
        return False
