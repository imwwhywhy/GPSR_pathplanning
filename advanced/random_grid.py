import random
import networkx as nx


def create_random_grid(
    width,
    height,
    obstacle_probability=0.20,
    protected_nodes=None,
    seed=42,
    max_attempts=200
):
    """
    Create a random 4-connected grid map.

    Parameters
    ----------
    width : int
        Width of the grid.

    height : int
        Height of the grid.

    obstacle_probability : float
        Probability that a node becomes an obstacle.

        Example:
        0.20 means approximately 20% of nodes
        may become obstacles.

    protected_nodes : list
        Nodes that cannot become obstacles.

        Normally:
        [start] + commands

    seed : int
        Random seed.

        Using the same seed gives the same map,
        which makes experiments reproducible.

    max_attempts : int
        Maximum attempts to generate a connected map.

    Returns
    -------
    graph : networkx.Graph

    positions : dict
        Node -> coordinate.

    blocked_nodes : list
        Nodes removed as obstacles.
    """

    if width <= 0 or height <= 0:
        raise ValueError(
            "width and height must be positive."
        )

    if not 0 <= obstacle_probability < 1:
        raise ValueError(
            "obstacle_probability must be between "
            "0 and 1."
        )

    if protected_nodes is None:
        protected_nodes = []

    protected_nodes = list(
        protected_nodes
    )

    rng = random.Random(
        seed
    )

    for attempt in range(
        max_attempts
    ):

        graph = nx.grid_2d_graph(
            width,
            height
        )

        blocked_nodes = []

        # ------------------------------------------
        # Randomly generate obstacles
        # ------------------------------------------

        for node in list(
            graph.nodes
        ):

            if node in protected_nodes:
                continue

            if (
                rng.random()
                < obstacle_probability
            ):

                blocked_nodes.append(
                    node
                )

        graph.remove_nodes_from(
            blocked_nodes
        )

        # ------------------------------------------
        # Check protected nodes still exist
        # ------------------------------------------

        if not all(
            node in graph
            for node in protected_nodes
        ):
            continue

        # ------------------------------------------
        # Check connectivity
        # ------------------------------------------

        if protected_nodes:

            reference_node = (
                protected_nodes[0]
            )

            connected_component = (
                nx.node_connected_component(
                    graph,
                    reference_node
                )
            )

            all_connected = all(
                node in connected_component
                for node
                in protected_nodes
            )

            if not all_connected:
                continue

        # ------------------------------------------
        # Positions
        # ------------------------------------------

        positions = {
            node: node
            for node
            in graph.nodes
        }

        # ------------------------------------------
        # Default distance weight
        # ------------------------------------------

        for u, v in graph.edges:

            graph.edges[
                u,
                v
            ]["weight"] = 1.0

        return (
            graph,
            positions,
            blocked_nodes
        )

    raise RuntimeError(
        "Unable to generate a connected random map "
        f"after {max_attempts} attempts."
    )