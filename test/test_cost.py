import math

import networkx as nx

from astar import (
    astar
)

from advanced.cost_model import (
    apply_distance_clearance_cost
)


def create_test_environment():

    graph = nx.grid_2d_graph(
        7,
        7
    )

    obstacle = (
        3,
        3
    )

    graph.remove_node(
        obstacle
    )

    positions = {
        node: node
        for node
        in graph.nodes
    }

    return (
        graph,
        positions,
        [
            obstacle
        ]
    )


def test_edges_near_obstacle_cost_more():

    (
        graph,
        positions,
        blocked_nodes
    ) = create_test_environment()

    apply_distance_clearance_cost(
        graph,
        positions,
        blocked_nodes,
        distance_weight=1.0,
        clearance_weight=2.0
    )

    # Edge close to obstacle
    near_cost = graph.edges[
        (2, 3),
        (2, 4)
    ]["weight"]

    # Edge farther away
    far_cost = graph.edges[
        (0, 0),
        (1, 0)
    ]["weight"]

    assert near_cost > far_cost


def test_safety_aware_astar_is_still_optimal():

    (
        graph,
        positions,
        blocked_nodes
    ) = create_test_environment()

    apply_distance_clearance_cost(
        graph,
        positions,
        blocked_nodes
    )

    start = (
        0,
        0
    )

    goal = (
        6,
        6
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

    networkx_cost = (
        nx.shortest_path_length(
            graph,
            start,
            goal,
            weight="weight"
        )
    )

    assert math.isclose(
        astar_cost,
        networkx_cost
    )