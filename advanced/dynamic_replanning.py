import networkx as nx

from astar import (
    find_best_command_order
)

from advanced.cost_model import (
    apply_distance_clearance_cost
)


def update_environment(
    graph,
    positions,
    current_position,
    remaining_commands,
    blocked_nodes,
    new_blocked_nodes
):
    """
    Add new obstacles to the environment.

    The original graph is not modified.
    """

    updated_graph = (
        graph.copy()
    )

    updated_positions = dict(
        positions
    )

    protected_nodes = {
        current_position,
        *remaining_commands
    }

    # ----------------------------------------------
    # Make sure obstacle does not destroy
    # current position or command locations.
    # ----------------------------------------------

    for node in new_blocked_nodes:

        if node in protected_nodes:

            raise ValueError(
                f"Cannot block protected node "
                f"{node}."
            )

    # ----------------------------------------------
    # Remove new obstacle nodes
    # ----------------------------------------------

    updated_graph.remove_nodes_from(
        new_blocked_nodes
    )

    for node in new_blocked_nodes:

        updated_positions.pop(
            node,
            None
        )

    updated_blocked_nodes = list(
        set(
            blocked_nodes
        )
        |
        set(
            new_blocked_nodes
        )
    )

    return (
        updated_graph,
        updated_positions,
        updated_blocked_nodes
    )


def replan_remaining_commands(
    graph,
    positions,
    current_position,
    remaining_commands,
    blocked_nodes,
    new_blocked_nodes,
    distance_weight=1.0,
    clearance_weight=2.0
):
    """
    Update environment and replan all remaining
    commands from the robot's current position.
    """

    (
        updated_graph,
        updated_positions,
        updated_blocked_nodes
    ) = update_environment(
        graph,
        positions,
        current_position,
        remaining_commands,
        blocked_nodes,
        new_blocked_nodes
    )

    # ----------------------------------------------
    # Make sure all remaining goals are reachable
    # ----------------------------------------------

    for command in remaining_commands:

        if not nx.has_path(
            updated_graph,
            current_position,
            command
        ):

            raise nx.NetworkXNoPath(
                f"Command {command} became "
                f"unreachable after environment "
                f"update."
            )

    # ----------------------------------------------
    # Environment changed:
    # recompute safety-aware cost.
    # ----------------------------------------------

    node_clearance = (
        apply_distance_clearance_cost(
            updated_graph,
            updated_positions,
            updated_blocked_nodes,
            distance_weight=(
                distance_weight
            ),
            clearance_weight=(
                clearance_weight
            )
        )
    )

    (
        best_path,
        best_cost,
        best_order,
        ordering_results,
        expanded_nodes
    ) = find_best_command_order(
        updated_graph,
        current_position,
        remaining_commands,
        updated_positions
    )

    return {

        "graph":
            updated_graph,

        "positions":
            updated_positions,

        "blocked_nodes":
            updated_blocked_nodes,

        "node_clearance":
            node_clearance,

        "best_path":
            best_path,

        "best_cost":
            best_cost,

        "best_order":
            best_order,

        "ordering_results":
            ordering_results,

        "expanded_nodes":
            expanded_nodes
    }


def choose_replannable_obstacle(
    graph,
    positions,
    current_position,
    remaining_commands
):
    """
    Automatically choose one node from the currently
    planned route that can become a new obstacle,
    while keeping all remaining commands reachable.

    This is only for simulation/demo purposes.
    """

    if not remaining_commands:
        return None

    (
        planned_path,
        _,
        _,
        _,
        _
    ) = find_best_command_order(
        graph,
        current_position,
        remaining_commands,
        positions
    )

    protected_nodes = {
        current_position,
        *remaining_commands
    }

    # ----------------------------------------------
    # Try path nodes one by one
    # ----------------------------------------------

    for candidate in planned_path[1:-1]:

        if candidate in protected_nodes:
            continue

        temporary_graph = (
            graph.copy()
        )

        temporary_graph.remove_node(
            candidate
        )

        still_connected = all(

            command
            in temporary_graph

            and nx.has_path(
                temporary_graph,
                current_position,
                command
            )

            for command
            in remaining_commands
        )

        if still_connected:

            return candidate

    return None


def simulate_replanning_after_first_command(
    graph,
    positions,
    start,
    commands,
    blocked_nodes,
    distance_weight=1.0,
    clearance_weight=2.0
):
    """
    Demonstration workflow:

    1. Plan all commands.
    2. Assume robot reaches the first command.
    3. Create a new obstacle.
    4. Replan all remaining commands.
    """

    # ----------------------------------------------
    # Initial planning
    # ----------------------------------------------

    initial_graph = (
        graph.copy()
    )

    apply_distance_clearance_cost(
        initial_graph,
        positions,
        blocked_nodes,
        distance_weight=(
            distance_weight
        ),
        clearance_weight=(
            clearance_weight
        )
    )

    (
        initial_path,
        initial_cost,
        initial_order,
        initial_results,
        initial_expanded
    ) = find_best_command_order(
        initial_graph,
        start,
        commands,
        positions
    )

    if len(
        initial_order
    ) < 2:

        return {
            "initial_path":
                initial_path,

            "initial_order":
                initial_order,

            "initial_cost":
                initial_cost,

            "replanned":
                False
        }

    # ----------------------------------------------
    # Robot reaches first command
    # ----------------------------------------------

    current_position = (
        initial_order[0]
    )

    remaining_commands = (
        initial_order[1:]
    )

    # ----------------------------------------------
    # Simulate new obstacle
    # ----------------------------------------------

    new_obstacle = (
        choose_replannable_obstacle(
            initial_graph,
            positions,
            current_position,
            remaining_commands
        )
    )

    if new_obstacle is None:

        return {
            "initial_path":
                initial_path,

            "initial_order":
                initial_order,

            "initial_cost":
                initial_cost,

            "current_position":
                current_position,

            "remaining_commands":
                remaining_commands,

            "replanned":
                False
        }

    replanning_result = (
        replan_remaining_commands(
            initial_graph,
            positions,
            current_position,
            remaining_commands,
            blocked_nodes,
            [
                new_obstacle
            ],
            distance_weight=(
                distance_weight
            ),
            clearance_weight=(
                clearance_weight
            )
        )
    )

    return {

        "initial_path":
            initial_path,

        "initial_order":
            initial_order,

        "initial_cost":
            initial_cost,

        "initial_expanded":
            initial_expanded,

        "current_position":
            current_position,

        "remaining_commands":
            remaining_commands,

        "new_obstacle":
            new_obstacle,

        "replanned":
            True,

        "replanning_result":
            replanning_result
    }