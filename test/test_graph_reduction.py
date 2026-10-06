import math
import networkx as nx

from astar import (
    astar
)

from advanced.graph_reduction import (
    contract_corridors,
    expand_compressed_path
)


def test_corridor_contraction():

    graph = nx.Graph()

    nodes = [
        (0, 0),
        (1, 0),
        (2, 0),
        (3, 0),
        (4, 0),
        (5, 0)
    ]

    graph.add_nodes_from(
        nodes
    )

    for node_a, node_b in zip(
        nodes[:-1],
        nodes[1:]
    ):

        graph.add_edge(
            node_a,
            node_b,
            weight=1.0,
            distance_cost=1.0,
            clearance_penalty=0.0
        )

    positions = {
        node: node
        for node in nodes
    }

    start = (
        0,
        0
    )

    goal = (
        5,
        0
    )

    (
        compressed_graph,
        compressed_positions,
        removed_nodes
    ) = contract_corridors(
        graph,
        positions,
        protected_nodes=[
            start,
            goal
        ]
    )

    # Only start and goal should remain
    assert (
        compressed_graph.number_of_nodes()
        == 2
    )

    (
        compressed_path,
        compressed_cost,
        expanded
    ) = astar(
        compressed_graph,
        start,
        goal,
        compressed_positions
    )

    assert math.isclose(
        compressed_cost,
        5.0
    )

    detailed_path = (
        expand_compressed_path(
            compressed_graph,
            compressed_path
        )
    )

    assert detailed_path == nodes