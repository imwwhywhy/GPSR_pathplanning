'''geometric travel planning'''

def _orient_path(
    path,
    start,
    end
):
    """
    Make sure a stored original path runs from
    start to end.
    """

    if (
        path[0] == start
        and path[-1] == end
    ):
        return list(
            path
        )

    if (
        path[0] == end
        and path[-1] == start
    ):
        return list(
            reversed(path)
        )

    # Fallback
    return [
        start,
        end
    ]


def contract_corridors(
    graph,
    positions,
    protected_nodes=None
):
    """
    Contract degree-2 corridor nodes.

    Important nodes such as:
    - start
    - commands

    are protected and will never be removed.

    Example:

        A -- B -- C

    becomes:

        A ------ C

    where:

        weight(A,C)
        =
        weight(A,B)
        +
        weight(B,C)

    Returns
    -------
    compressed_graph

    compressed_positions

    removed_nodes
    """

    compressed_graph = (
        graph.copy()
    )

    if protected_nodes is None:
        protected_nodes = []

    protected_nodes = set(
        protected_nodes
    )

    # ----------------------------------------------
    # Store original path for every edge
    # ----------------------------------------------

    for u, v in compressed_graph.edges:

        compressed_graph.edges[
            u,
            v
        ]["original_path"] = [
            u,
            v
        ]

    removed_nodes = []

    changed = True

    while changed:

        changed = False

        for node in list(
            compressed_graph.nodes
        ):

            if node in protected_nodes:
                continue

            if (
                compressed_graph.degree(
                    node
                )
                != 2
            ):
                continue

            neighbours = list(
                compressed_graph.neighbors(
                    node
                )
            )

            node_a = neighbours[0]
            node_b = neighbours[1]

            if node_a == node_b:
                continue

            # --------------------------------------
            # Edge information
            # --------------------------------------

            edge_a = dict(
                compressed_graph.edges[
                    node_a,
                    node
                ]
            )

            edge_b = dict(
                compressed_graph.edges[
                    node,
                    node_b
                ]
            )

            weight_a = edge_a.get(
                "weight",
                1.0
            )

            weight_b = edge_b.get(
                "weight",
                1.0
            )

            new_weight = (
                weight_a
                + weight_b
            )

            distance_cost = (
                edge_a.get(
                    "distance_cost",
                    weight_a
                )
                +
                edge_b.get(
                    "distance_cost",
                    weight_b
                )
            )

            clearance_penalty = (
                edge_a.get(
                    "clearance_penalty",
                    0.0
                )
                +
                edge_b.get(
                    "clearance_penalty",
                    0.0
                )
            )

            # --------------------------------------
            # Preserve original detailed path
            # --------------------------------------

            path_a = _orient_path(
                edge_a.get(
                    "original_path",
                    [
                        node_a,
                        node
                    ]
                ),
                node_a,
                node
            )

            path_b = _orient_path(
                edge_b.get(
                    "original_path",
                    [
                        node,
                        node_b
                    ]
                ),
                node,
                node_b
            )

            combined_original_path = (
                path_a
                + path_b[1:]
            )

            # --------------------------------------
            # If an edge already exists,
            # keep whichever one has lower cost.
            # --------------------------------------

            should_add = True

            if compressed_graph.has_edge(
                node_a,
                node_b
            ):

                existing_weight = (
                    compressed_graph.edges[
                        node_a,
                        node_b
                    ].get(
                        "weight",
                        float("inf")
                    )
                )

                if (
                    existing_weight
                    <= new_weight
                ):

                    should_add = False

            if should_add:

                compressed_graph.add_edge(
                    node_a,
                    node_b,

                    weight=new_weight,

                    distance_cost=(
                        distance_cost
                    ),

                    clearance_penalty=(
                        clearance_penalty
                    ),

                    original_path=(
                        combined_original_path
                    )
                )

            # --------------------------------------
            # Remove corridor node
            # --------------------------------------

            compressed_graph.remove_node(
                node
            )

            removed_nodes.append(
                node
            )

            changed = True

            # Graph changed.
            # Restart loop safely.
            break

    compressed_positions = {

        node: positions[node]

        for node
        in compressed_graph.nodes

        if node in positions
    }

    return (
        compressed_graph,
        compressed_positions,
        removed_nodes
    )


def expand_compressed_path(
    compressed_graph,
    compressed_path
):
    """
    Convert a compressed path back into the
    original detailed path.

    Example:

        compressed:
        A -> E

    may expand into:

        A -> B -> C -> D -> E
    """

    if len(
        compressed_path
    ) <= 1:

        return list(
            compressed_path
        )

    full_path = [
        compressed_path[0]
    ]

    for node_a, node_b in zip(
        compressed_path[:-1],
        compressed_path[1:]
    ):

        edge_data = (
            compressed_graph.edges[
                node_a,
                node_b
            ]
        )

        original_path = (
            edge_data.get(
                "original_path",
                [
                    node_a,
                    node_b
                ]
            )
        )

        original_path = _orient_path(
            original_path,
            node_a,
            node_b
        )

        full_path.extend(
            original_path[1:]
        )

    return full_path


def graph_reduction_statistics(
    original_graph,
    compressed_graph
):
    """
    Return simple graph reduction statistics.
    """

    original_nodes = (
        original_graph.number_of_nodes()
    )

    compressed_nodes = (
        compressed_graph.number_of_nodes()
    )

    removed_nodes = (
        original_nodes
        - compressed_nodes
    )

    if original_nodes == 0:
        reduction_percent = 0.0

    else:

        reduction_percent = (
            removed_nodes
            / original_nodes
            * 100
        )

    return {
        "original_nodes":
            original_nodes,

        "compressed_nodes":
            compressed_nodes,

        "removed_nodes":
            removed_nodes,

        "reduction_percent":
            reduction_percent
    }