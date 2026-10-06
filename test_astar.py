import math
import itertools

import networkx as nx

from astar import (
    astar,
    heuristic,
    find_best_command_order
)


def create_test_graph():
    """
    Create the same type of graph used for testing.
    """

    graph = nx.grid_2d_graph(
        5,
        5
    )

    blocked_edges = [
        ((1, 1), (1, 2)),
        ((1, 2), (2, 2)),
        ((2, 2), (3, 2)),
        ((3, 2), (3, 3))
    ]

    graph.remove_edges_from(
        blocked_edges
    )

    positions = {
        node: node
        for node in graph.nodes
    }

    for u, v in graph.edges:

        graph.edges[
            u,
            v
        ]["weight"] = heuristic(
            u,
            v,
            positions
        )

    return (
        graph,
        positions
    )


# ======================================================
# TEST 1
# Basic A* functionality
# ======================================================

def test_astar_single_goal():

    graph, positions = create_test_graph()

    start = (
        0,
        0
    )

    goal = (
        4,
        4
    )

    path, cost, expanded = astar(
        graph,
        start,
        goal,
        positions
    )

    assert path[0] == start

    assert path[-1] == goal

    assert cost >= 0

    assert expanded > 0


# ======================================================
# TEST 2
# A* should match NetworkX Dijkstra shortest path cost
# ======================================================

def test_astar_matches_dijkstra():

    graph, positions = create_test_graph()

    start = (
        0,
        0
    )

    goal = (
        4,
        4
    )

    (
        path,
        astar_cost,
        expanded
    ) = astar(
        graph,
        start,
        goal,
        positions
    )

    dijkstra_cost = nx.shortest_path_length(
        graph,
        start,
        goal,
        weight="weight"
    )

    assert math.isclose(
        astar_cost,
        dijkstra_cost
    )


# ======================================================
# TEST 3
# All commands must appear in final route
# ======================================================

def test_multi_command_route():

    graph, positions = create_test_graph()

    start = (
        0,
        0
    )

    commands = [
        (0, 4),
        (4, 4),
        (4, 0)
    ]

    (
        best_path,
        best_cost,
        best_order,
        ordering_results,
        expanded
    ) = find_best_command_order(
        graph,
        start,
        commands,
        positions
    )

    # Every command must be visited
    for command in commands:

        assert command in best_path

    # Best order must contain exactly all commands
    assert set(
        best_order
    ) == set(
        commands
    )

    assert best_cost >= 0


# ======================================================
# TEST 4
# Number of evaluated orderings must equal n!
# ======================================================

def test_number_of_orderings_is_factorial():

    graph, positions = create_test_graph()

    start = (
        0,
        0
    )

    commands = [
        (0, 4),
        (4, 4),
        (4, 0)
    ]

    (
        best_path,
        best_cost,
        best_order,
        ordering_results,
        expanded
    ) = find_best_command_order(
        graph,
        start,
        commands,
        positions
    )

    expected_number = math.factorial(
        len(commands)
    )

    assert len(
        ordering_results
    ) == expected_number


# ======================================================
# TEST 5
# Selected result must equal the minimum among all n!
# ordering costs
# ======================================================

def test_best_cost_is_minimum():

    graph, positions = create_test_graph()

    start = (
        0,
        0
    )

    commands = [
        (0, 4),
        (4, 4),
        (4, 0)
    ]

    (
        best_path,
        best_cost,
        best_order,
        ordering_results,
        expanded
    ) = find_best_command_order(
        graph,
        start,
        commands,
        positions
    )

    all_costs = [
        cost
        for order, cost
        in ordering_results
    ]

    minimum_cost = min(
        all_costs
    )

    assert math.isclose(
        best_cost,
        minimum_cost
    )


# ======================================================
# TEST 6
# Independently calculate every permutation using
# NetworkX shortest paths and confirm our algorithm finds
# the same global minimum.
# ======================================================

def test_optimal_command_order_against_networkx():

    graph, positions = create_test_graph()

    start = (
        0,
        0
    )

    commands = [
        (0, 4),
        (4, 4),
        (4, 0)
    ]

    (
        best_path,
        algorithm_best_cost,
        best_order,
        ordering_results,
        expanded
    ) = find_best_command_order(
        graph,
        start,
        commands,
        positions
    )

    # Independent baseline calculation
    networkx_best_cost = float(
        "inf"
    )

    for order in itertools.permutations(
        commands
    ):

        current = start

        total_cost = 0.0

        for command in order:

            total_cost += nx.shortest_path_length(
                graph,
                current,
                command,
                weight="weight"
            )

            current = command

        networkx_best_cost = min(
            networkx_best_cost,
            total_cost
        )

    assert math.isclose(
        algorithm_best_cost,
        networkx_best_cost
    )