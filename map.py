import random
import networkx as nx


def _set_edge_weights(graph):
    """
    Assign a cost of 1.0 to every edge.

    The map module only describes the environment.
    Path planning itself is handled by astar.py.
    """

    for u, v in graph.edges:
        graph.edges[u, v]["weight"] = 1.0


def create_grid_map(
    width,
    height,
    blocked_nodes=None
):
    """
    Create a normal 4-connected rectangular grid map.

    Parameters
    ----------
    width : int
        Number of nodes in x direction.

    height : int
        Number of nodes in y direction.

    blocked_nodes : list, optional
        Nodes that should behave as obstacles.

        Example:
        [
            (2, 2),
            (2, 3),
            (3, 3)
        ]

    Returns
    -------
    graph
        NetworkX graph.

    positions
        Dictionary containing node coordinates.

    blocked_nodes
        List of removed obstacle nodes.
    """

    graph = nx.grid_2d_graph(
        width,
        height
    )

    if blocked_nodes is None:
        blocked_nodes = []

    blocked_nodes = list(
        blocked_nodes
    )

    graph.remove_nodes_from(
        blocked_nodes
    )

    positions = {
        node: node
        for node in graph.nodes
    }

    _set_edge_weights(
        graph
    )

    return (
        graph,
        positions,
        blocked_nodes
    )


def create_maze_map(
    width=15,
    height=15
):
    """
    Create a deterministic maze-like map.

    This is not a random map.

    It contains several walls with openings so that
    A* must navigate around obstacles.

    The same A* implementation can be used without
    modification.
    """

    if width < 10 or height < 10:

        raise ValueError(
            "Maze map should be at least 10 x 10."
        )

    blocked_nodes = []

    # --------------------------------------------------
    # Vertical wall 1
    # --------------------------------------------------

    wall_1_x = width // 3

    gap_1_y = height // 4
    gap_2_y = 3 * height // 4

    for y in range(height):

        if y not in (
            gap_1_y,
            gap_2_y
        ):
            blocked_nodes.append(
                (
                    wall_1_x,
                    y
                )
            )

    # --------------------------------------------------
    # Vertical wall 2
    # --------------------------------------------------

    wall_2_x = 2 * width // 3

    gap_3_y = height // 3
    gap_4_y = 5 * height // 6

    for y in range(height):

        if y not in (
            gap_3_y,
            gap_4_y
        ):
            blocked_nodes.append(
                (
                    wall_2_x,
                    y
                )
            )

    # --------------------------------------------------
    # Horizontal wall in the middle
    # --------------------------------------------------

    middle_y = height // 2

    middle_gap_x = width // 2

    for x in range(
        wall_1_x + 1,
        wall_2_x
    ):

        if x != middle_gap_x:

            blocked_nodes.append(
                (
                    x,
                    middle_y
                )
            )

    return create_grid_map(
        width,
        height,
        blocked_nodes
    )


def create_random_grid_map(
    width,
    height,
    obstacle_probability=0.20,
    protected_nodes=None,
    seed=42,
    max_attempts=100
):
    """
    Create a random obstacle grid.

    Parameters
    ----------
    width : int
        Grid width.

    height : int
        Grid height.

    obstacle_probability : float
        Probability that a node becomes an obstacle.

        Example:
        0.20 = approximately 20% obstacles.

    protected_nodes : list
        Nodes that cannot become obstacles.

        Normally this should include:
        - start
        - all command destinations

    seed : int
        Random seed for reproducibility.

    max_attempts : int
        Maximum number of maps generated while trying
        to keep all protected nodes connected.

    Important
    ---------
    The function checks that all protected nodes are
    connected before returning the map.

    Therefore the generated map is guaranteed to have
    valid routes between start and commands.
    """

    if not (
        0 <= obstacle_probability < 1
    ):

        raise ValueError(
            "obstacle_probability must be "
            "between 0 and 1."
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

        # Randomly remove nodes
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

        # --------------------------------------------------
        # Check protected nodes still exist
        # --------------------------------------------------

        protected_valid = all(
            node in graph
            for node in protected_nodes
        )

        if not protected_valid:
            continue

        # --------------------------------------------------
        # Check all protected nodes are mutually reachable
        # --------------------------------------------------

        if protected_nodes:

            reference = protected_nodes[0]

            connected = all(
                nx.has_path(
                    graph,
                    reference,
                    node
                )
                for node
                in protected_nodes[1:]
            )

            if not connected:
                continue

        positions = {
            node: node
            for node in graph.nodes
        }

        _set_edge_weights(
            graph
        )

        return (
            graph,
            positions,
            blocked_nodes
        )

    raise RuntimeError(
        "Unable to generate a connected random map "
        f"after {max_attempts} attempts."
    )
