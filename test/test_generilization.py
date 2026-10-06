import math

import networkx as nx
import pytest

from astar import (
    astar,
    find_best_command_order
)

from advanced.random_grid import (
    create_random_grid
)

from advanced.cost_model import (
    apply_distance_only_cost
)


@pytest.mark.parametrize(
    "width,height,seed",
    [
        (8, 8, 10),
        (12, 12, 20),
        (20, 20, 30)
    ]
)
def test_astar_on_different_map_sizes(
    width,
    height,
    seed
):

    start = (
        0,
        0
    )

    goal = (
        width - 1,
        height - 1
    )

    (
        graph,
        positions,
        blocked_nodes
    ) = create_random_grid(
        width=width,
        height=height,
        obstacle_probability=0.15,
        protected_nodes=[
            start,
            goal
        ],
        seed=seed
    )

    apply_distance_only_cost(
        graph,
        positions
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

    dijkstra_cost = (
        nx.shortest_path_length(
            graph,
            start,
            goal,
            weight="weight"
        )
    )

    assert path[0] == start
    assert path[-1] == goal

    assert math.isclose(
        astar_cost,
        dijkstra_cost
    )

    assert expanded > 0


def test_multi_command_generalization():

    start = (
        0,
        0
    )

    commands = [
        (1, 8),
        (8, 8),
        (8, 1)
    ]

    (
        graph,
        positions,
        blocked_nodes
    ) = create_random_grid(
        width=10,
        height=10,
        obstacle_probability=0.15,
        protected_nodes=[
            start,
            *commands
        ],
        seed=42
    )

    apply_distance_only_cost(
        graph,
        positions
    )

    (
        path,
        cost,
        order,
        ordering_results,
        expanded
    ) = find_best_command_order(
        graph,
        start,
        commands,
        positions
    )

    assert len(
        ordering_results
    ) == math.factorial(
        len(commands)
    )

    for command in commands:

        assert command in path