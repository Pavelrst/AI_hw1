from .graph_problem_interface import *
from .best_first_search import BestFirstSearch
from typing import Optional
import numpy as np


class GreedyStochastic(BestFirstSearch):
    def __init__(self, heuristic_function_type: HeuristicFunctionType,
                 T_init: float = 1.0, N: int = 5, T_scale_factor: float = 0.95):
        # GreedyStochastic is a graph search algorithm. Hence, we use close set.
        super(GreedyStochastic, self).__init__(use_close=True)
        self.heuristic_function_type = heuristic_function_type
        self.T = T_init
        self.N = N
        self.T_scale_factor = T_scale_factor
        self.solver_name = 'GreedyStochastic (h={heuristic_name})'.format(
            heuristic_name=heuristic_function_type.heuristic_name)

    def _init_solver(self, problem: GraphProblem):
        super(GreedyStochastic, self)._init_solver(problem)
        self.heuristic_function = self.heuristic_function_type(problem)

    def _open_successor_node(self, problem: GraphProblem, successor_node: SearchNode):
        # Greedy, so identical to uniform_cost.py's _open_successor_node()
        if self.close.has_state(successor_node.state):
            return

        if self.open.has_state(successor_node.state):
            already_found_node_with_same_state = self.open.get_node_by_state(successor_node.state)
            if already_found_node_with_same_state.expanding_priority > successor_node.expanding_priority:
                self.open.extract_node(already_found_node_with_same_state)

        if not self.open.has_state(successor_node.state):
            self.open.push_node(successor_node)

    def _calc_node_expanding_priority(self, search_node: SearchNode) -> float:
        return self.heuristic_function.estimate(search_node.state)

    def _extract_next_search_node_to_expand(self) -> Optional[SearchNode]:

        """
        Extracts the next node to expand from the open queue,
         using the stochastic method to choose out of the N
         best items from open.
        """

        if self.open.is_empty():
            return None
        num_contenders = min(self.N, len(self.open))
        contenders = []
        for i in range(0, num_contenders):
            contenders.append(self.open.pop_next_node())

        P = np.zeros(num_contenders)

        alpha = contenders[0].expanding_priority
        for con in contenders:
            if con.expanding_priority < alpha:
                alpha = con.expanding_priority

        if alpha == 0:
            # choose a node with cost 0 at random
            zero_idx = []
            for i, cont in enumerate(contenders):
                if cont.expanding_priority == 0:
                    zero_idx.append(i)
            rnd_idx = np.random.choice(len(zero_idx), 1)[0]
            chosen_idx = zero_idx[rnd_idx]

            for i, cont in enumerate(contenders):
                if i != chosen_idx:
                    self.open.push_node(cont)

            self.T = self.T * self.T_scale_factor
            return contenders[chosen_idx]

        # Calculating denominator

        assert alpha > 0 and self.T > 0
        denom = 0.0
        for cont in contenders:
            denom += (cont.expanding_priority / alpha) ** (-1 / self.T)

        assert denom > 0

        for i, node in enumerate(contenders):
            cost = node.expanding_priority
            P[i] = ((cost/alpha) ** (-1 / self.T)) / denom

        chosen_idx = np.random.choice(num_contenders, 1, False, P)[0]

        for i, cont in enumerate(contenders):
            if i != chosen_idx:
                self.open.push_node(cont)

        self.T = self.T * self.T_scale_factor
        return contenders[chosen_idx]
