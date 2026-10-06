import math
import heapq
import itertools
import networkx as nx


def heuristic(node_a, node_b, positions):
    """
    Euclidean-distance heuristic used by A*.

    positions:
        {
            node: (x, y)
        }
    """

    x1, y1 = positions[node_a]
    x2, y2 = positions[node_b]

    return math.hypot(x1 - x2, y1 - y2)


def reconstruct_path(came_from, goal):
    """
    Reconstruct path from start to goal.
    """

    path = []
    current = goal

    while current is not None:
        path.append(current)
        current = came_from[current]

    path.reverse()

    return path


def astar(graph, start, goal, positions):
    """
    Find the shortest path from start to goal using A*.

    Returns:
        path:
            List of nodes from start to goal.

        total_cost:
            Total path cost.

        expanded_nodes:
            Number of nodes expanded by A*.
    """

    if start not in graph:
        raise ValueError("Start node is not in graph.")

    if goal not in graph:
        raise ValueError("Goal node is not in graph.")

    # Priority queue:
    # (f_score, g_score, node)
    frontier = []

    heapq.heappush(
        frontier,
        (
            heuristic(start, goal, positions),
            0.0,
            start
        )
    )

    came_from = {
        start: None
    }

    g_score = {
        start: 0.0
    }

    expanded_nodes = 0

    while frontier:

        f_score, current_g, current = heapq.heappop(frontier)

        # Ignore outdated queue entries
        if current_g != g_score.get(current):
            continue

        expanded_nodes += 1

        # Goal reached
        if current == goal:

            path = reconstruct_path(
                came_from,
                goal
            )

            return (
                path,
                g_score[goal],
                expanded_nodes
            )

        # Explore neighbours
        for neighbour, edge_data in graph[current].items():

            edge_cost = edge_data.get(
                "weight",
                1.0
            )

            tentative_g = (
                g_score[current]
                + edge_cost
            )

            if tentative_g < g_score.get(
                neighbour,
                float("inf")
            ):

                came_from[neighbour] = current

                g_score[neighbour] = tentative_g

                h_score = heuristic(
                    neighbour,
                    goal,
                    positions
                )

                f_score = (
                    tentative_g
                    + h_score
                )

                heapq.heappush(
                    frontier,
                    (
                        f_score,
                        tentative_g,
                        neighbour
                    )
                )

    raise nx.NetworkXNoPath(
        f"No path exists from {start} to {goal}."
    )


def find_best_command_order(
    graph,
    start,
    commands,
    positions
):
    """
    When n commands arrive simultaneously:

    1. Compute shortest paths between all relevant node pairs.
    2. Generate all n! command orderings.
    3. Compute total path cost for every ordering.
    4. Choose the ordering with minimum total cost.

    Returns:
        best_path:
            Full path following the best command order.

        best_cost:
            Minimum total path cost.

        best_order:
            Optimal command ordering.

        ordering_results:
            List containing:
            [
                (ordering, total_cost),
                ...
            ]

        expanded_nodes:
            Number of A* node expansions used by the
            path segments of the final best route.
    """

    commands = list(commands)

    # No commands
    if not commands:
        return (
            [start],
            0.0,
            [],
            [],
            0
        )

    # --------------------------------------------------
    # STEP 1:
    # Precompute shortest path and cost between all
    # relevant node pairs.
    # --------------------------------------------------

    relevant_nodes = [
        start
    ] + commands

    pair_cost = {}
    pair_path = {}
    pair_expanded = {}

    for source in relevant_nodes:

        for target in relevant_nodes:

            if source == target:
                continue

            path, cost, expanded = astar(
                graph,
                source,
                target,
                positions
            )

            pair_cost[
                (source, target)
            ] = cost

            pair_path[
                (source, target)
            ] = path

            pair_expanded[
                (source, target)
            ] = expanded

    # --------------------------------------------------
    # STEP 2:
    # Generate all n! possible command orderings.
    # --------------------------------------------------

    all_orderings = itertools.permutations(
        commands
    )

    ordering_results = []

    best_order = None
    best_cost = float("inf")

    # --------------------------------------------------
    # STEP 3:
    # Compute total path cost for every ordering.
    # --------------------------------------------------

    for order in all_orderings:

        total_cost = 0.0

        current = start

        for command in order:

            total_cost += pair_cost[
                (current, command)
            ]

            current = command

        ordering_results.append(
            (
                order,
                total_cost
            )
        )

        # --------------------------------------------------
        # STEP 4:
        # Keep the minimum-cost ordering.
        # --------------------------------------------------

        if total_cost < best_cost:

            best_cost = total_cost
            best_order = order

    # --------------------------------------------------
    # Reconstruct full path for the best ordering.
    # --------------------------------------------------

    best_path = [
        start
    ]

    total_expanded = 0

    current = start

    for command in best_order:

        segment_path = pair_path[
            (current, command)
        ]

        # Avoid repeating the first node
        best_path.extend(
            segment_path[1:]
        )

        total_expanded += pair_expanded[
            (current, command)
        ]

        current = command

    return (
        best_path,
        best_cost,
        list(best_order),
        ordering_results,
        total_expanded
    )