import math


def euclidean_distance(
    position_a,
    position_b
):
    """
    Euclidean distance between two coordinates.
    """

    x1, y1 = position_a
    x2, y2 = position_b

    return math.hypot(
        x1 - x2,
        y1 - y2
    )


def compute_node_clearance(
    graph,
    blocked_nodes
):
    """
    Compute each free node's distance to the
    nearest obstacle.

    Larger clearance means the node is farther
    from obstacles and is generally safer.
    """

    node_clearance = {}

    if not blocked_nodes:

        for node in graph.nodes:

            node_clearance[node] = (
                float("inf")
            )

        return node_clearance

    for node in graph.nodes:

        minimum_distance = min(

            euclidean_distance(
                node,
                obstacle
            )

            for obstacle
            in blocked_nodes
        )

        node_clearance[node] = (
            minimum_distance
        )

    return node_clearance


def apply_distance_only_cost(
    graph,
    positions
):
    """
    Baseline cost model.

    Cost = distance only.
    """

    for u, v in graph.edges:

        distance = euclidean_distance(
            positions[u],
            positions[v]
        )

        graph.edges[
            u,
            v
        ]["distance_cost"] = distance

        graph.edges[
            u,
            v
        ]["clearance_penalty"] = 0.0

        graph.edges[
            u,
            v
        ]["weight"] = distance


def apply_distance_clearance_cost(
    graph,
    positions,
    blocked_nodes,
    distance_weight=1.0,
    clearance_weight=2.0,
    minimum_clearance=0.5
):
    """
    Apply a safety-aware edge cost.

    Total edge cost:

        cost =
        distance_weight * distance
        +
        clearance_weight / clearance

    Therefore:

    - short edges are preferred
    - edges very close to obstacles receive
      a larger penalty

    Parameters
    ----------
    distance_weight : float
        Importance of distance.

    clearance_weight : float
        Importance of obstacle clearance.

    minimum_clearance : float
        Prevents division by zero.

    Returns
    -------
    node_clearance : dict
    """

    node_clearance = (
        compute_node_clearance(
            graph,
            blocked_nodes
        )
    )

    for u, v in graph.edges:

        # ------------------------------------------
        # Distance
        # ------------------------------------------

        distance = euclidean_distance(
            positions[u],
            positions[v]
        )

        # ------------------------------------------
        # Clearance of this edge
        #
        # Use the less-safe endpoint.
        # ------------------------------------------

        edge_clearance = min(
            node_clearance[u],
            node_clearance[v]
        )

        if math.isinf(
            edge_clearance
        ):

            clearance_penalty = 0.0

        else:

            safe_clearance = max(
                edge_clearance,
                minimum_clearance
            )

            clearance_penalty = (
                clearance_weight
                / safe_clearance
            )

        distance_cost = (
            distance_weight
            * distance
        )

        total_cost = (
            distance_cost
            + clearance_penalty
        )

        graph.edges[
            u,
            v
        ]["distance_cost"] = (
            distance_cost
        )

        graph.edges[
            u,
            v
        ]["clearance_penalty"] = (
            clearance_penalty
        )

        graph.edges[
            u,
            v
        ]["weight"] = (
            total_cost
        )

    return node_clearance


def calculate_path_distance(
    path,
    positions
):
    """
    Calculate geometric distance of a path.

    This ignores clearance penalty.
    """

    if len(path) < 2:
        return 0.0

    total_distance = 0.0

    for node_a, node_b in zip(
        path[:-1],
        path[1:]
    ):

        total_distance += (
            euclidean_distance(
                positions[node_a],
                positions[node_b]
            )
        )

    return total_distance


def calculate_path_objective_cost(
    graph,
    path
):
    """
    Sum the actual edge weights used by A*.
    """

    if len(path) < 2:
        return 0.0

    total_cost = 0.0

    for node_a, node_b in zip(
        path[:-1],
        path[1:]
    ):

        total_cost += graph.edges[
            node_a,
            node_b
        ].get(
            "weight",
            1.0
        )

    return total_cost


def minimum_path_clearance(
    path,
    node_clearance
):
    """
    Find minimum obstacle clearance along a path.

    This gives a simple safety indicator.

    Larger is safer.
    """

    if not path:
        return 0.0

    return min(
        node_clearance.get(
            node,
            float("inf")
        )

        for node in path
    )