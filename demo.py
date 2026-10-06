import argparse
import math
import time

from astar import find_best_command_order

from map import (
    create_grid_map,
    create_maze_map,
    create_random_grid_map
)

from demograph import draw_route


def create_scenario(
    map_type
):
    """
    Build one of several test environments.

    The important idea is:

        different maps
             ↓
        same graph interface
             ↓
        same A* implementation
             ↓
        same command ordering algorithm
    """

    # ==================================================
    # Scenario 1:
    # Simple small grid
    # ==================================================

    if map_type == "simple":

        width = 8
        height = 8

        start = (
            0,
            0
        )

        commands = [
            (0, 7),
            (7, 7),
            (7, 0)
        ]

        blocked_nodes = [
            (2, 1),
            (2, 2),
            (2, 3),

            (4, 4),
            (4, 5),
            (4, 6),

            (5, 2),
            (6, 2)
        ]

        (
            graph,
            positions,
            blocked_nodes
        ) = create_grid_map(
            width,
            height,
            blocked_nodes
        )

        return (
            graph,
            positions,
            blocked_nodes,
            start,
            commands
        )

    # ==================================================
    # Scenario 2:
    # Larger maze map
    # ==================================================

    elif map_type == "maze":

        width = 15
        height = 15

        start = (
            0,
            0
        )

        commands = [
            (0, 14),
            (14, 14),
            (14, 0)
        ]

        (
            graph,
            positions,
            blocked_nodes
        ) = create_maze_map(
            width,
            height
        )

        return (
            graph,
            positions,
            blocked_nodes,
            start,
            commands
        )

    # ==================================================
    # Scenario 3:
    # Random obstacle map
    # ==================================================

    elif map_type == "random":

        width = 20
        height = 20

        start = (
            0,
            0
        )

        commands = [
            (1, 18),
            (18, 18),
            (18, 1)
        ]

        protected_nodes = [
            start
        ] + commands

        (
            graph,
            positions,
            blocked_nodes
        ) = create_random_grid_map(
            width=width,
            height=height,
            obstacle_probability=0.20,
            protected_nodes=protected_nodes,
            seed=42
        )

        return (
            graph,
            positions,
            blocked_nodes,
            start,
            commands
        )

    else:

        raise ValueError(
            "Unknown map type. "
            "Choose: simple, maze, or random."
        )


def print_results(
    map_type,
    graph,
    commands,
    start,
    best_path,
    best_cost,
    best_order,
    ordering_results,
    expanded_nodes,
    planning_time
):
    """
    Print planning information to terminal.
    """

    print(
        "\n"
        "======================================"
    )

    print(
        "     Multi-Command Path Planning"
    )

    print(
        "======================================"
    )

    print(
        f"\nMap type: {map_type}"
    )

    print(
        f"Graph nodes: "
        f"{graph.number_of_nodes()}"
    )

    print(
        f"Graph edges: "
        f"{graph.number_of_edges()}"
    )

    print(
        "\nStart:"
    )

    print(
        start
    )

    print(
        "\nCommands:"
    )

    print(
        commands
    )

    number_of_commands = len(
        commands
    )

    number_of_orderings = math.factorial(
        number_of_commands
    )

    print(
        f"\nNumber of commands: "
        f"{number_of_commands}"
    )

    print(
        f"Number of possible orderings: "
        f"{number_of_commands}! "
        f"= {number_of_orderings}"
    )

    # ==================================================
    # Show all command orderings
    # ==================================================

    print(
        "\n"
        "===== All Ordering Costs ====="
    )

    for index, (
        order,
        cost
    ) in enumerate(
        ordering_results,
        start=1
    ):

        print(
            f"\nOrdering {index}:"
        )

        print(
            order
        )

        print(
            f"Cost = {cost:.2f}"
        )

    # ==================================================
    # Best result
    # ==================================================

    print(
        "\n"
        "===== Best Result ====="
    )

    print(
        "\nOptimal command order:"
    )

    print(
        best_order
    )

    print(
        "\nFull route:"
    )

    print(
        best_path
    )

    print(
        f"\nMinimum total cost: "
        f"{best_cost:.2f}"
    )

    print(
        f"Expanded nodes "
        f"(best route segments): "
        f"{expanded_nodes}"
    )

    print(
        f"Planning time: "
        f"{planning_time * 1000:.3f} ms"
    )


def main():

    # ==================================================
    # Command-line arguments
    # ==================================================

    parser = argparse.ArgumentParser(
        description=(
            "Run the same A* + multi-command "
            "planner on different environments."
        )
    )

    parser.add_argument(
        "map_type",
        nargs="?",
        default="simple",
        choices=[
            "simple",
            "maze",
            "random"
        ],
        help=(
            "Environment to test: "
            "simple, maze, or random."
        )
    )

    args = parser.parse_args()

    map_type = args.map_type

    # ==================================================
    # Create selected environment
    # ==================================================

    (
        graph,
        positions,
        blocked_nodes,
        start,
        commands
    ) = create_scenario(
        map_type
    )

    # ==================================================
    # Run path planning
    # ==================================================

    start_time = time.perf_counter()

    (
        best_path,
        best_cost,
        best_order,
        ordering_results,
        expanded_nodes
    ) = find_best_command_order(
        graph,
        start,
        commands,
        positions
    )

    end_time = time.perf_counter()

    planning_time = (
        end_time
        - start_time
    )

    # ==================================================
    # Terminal output
    # ==================================================

    print_results(
        map_type,
        graph,
        commands,
        start,
        best_path,
        best_cost,
        best_order,
        ordering_results,
        expanded_nodes,
        planning_time
    )

    # ==================================================
    # Visualization
    # ==================================================

    output_file = (
        f"route_{map_type}.png"
    )

    draw_route(
        graph=graph,
        positions=positions,
        best_path=best_path,
        start=start,
        commands=commands,
        best_order=best_order,
        blocked_nodes=blocked_nodes,
        title=(
            f"A* Multi-Command Planning "
            f"- {map_type.capitalize()} Map"
        ),
        save_path=output_file
    )

    print(
        f"\nRoute image saved as: "
        f"{output_file}"
    )


if __name__ == "__main__":
    main()