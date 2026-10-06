import argparse
import time

from astar import (
    find_best_command_order
)

from demograph import (
    draw_route
)

from advanced.random_grid import (
    create_random_grid
)

from advanced.scenarios import (
    get_scenario,
    get_available_scenarios
)

from advanced.cost_model import (
    apply_distance_only_cost,
    apply_distance_clearance_cost,
    calculate_path_distance,
    minimum_path_clearance
)

from advanced.graph_reduction import (
    contract_corridors,
    expand_compressed_path,
    graph_reduction_statistics
)

from advanced.dynamic_replanning import (
    simulate_replanning_after_first_command
)


def run_planner(
    graph,
    positions,
    start,
    commands
):
    """
    Run multi-command planning and measure time.
    """

    start_time = (
        time.perf_counter()
    )

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

    elapsed_time = (
        time.perf_counter()
        - start_time
    )

    return {
        "path":
            best_path,

        "cost":
            best_cost,

        "order":
            best_order,

        "ordering_results":
            ordering_results,

        "expanded_nodes":
            expanded_nodes,

        "planning_time":
            elapsed_time
    }


def print_result(
    name,
    result,
    path_distance_value=None,
    minimum_clearance_value=None
):
    """
    Print planner result.
    """

    print(
        "\n"
        "----------------------------------------"
    )

    print(
        name
    )

    print(
        "----------------------------------------"
    )

    print(
        "Best command order:"
    )

    print(
        result["order"]
    )

    print(
        f"Objective cost: "
        f"{result['cost']:.3f}"
    )

    if path_distance_value is not None:

        print(
            f"Geometric distance: "
            f"{path_distance_value:.3f}"
        )

    if minimum_clearance_value is not None:

        print(
            f"Minimum clearance: "
            f"{minimum_clearance_value:.3f}"
        )

    print(
        f"Expanded nodes: "
        f"{result['expanded_nodes']}"
    )

    print(
        f"Planning time: "
        f"{result['planning_time'] * 1000:.3f} ms"
    )


def main():

    parser = argparse.ArgumentParser(
        description=(
            "Advanced A* path-planning demo."
        )
    )

    parser.add_argument(
        "--scenario",
        default="small_random",
        choices=(
            get_available_scenarios()
        )
    )

    parser.add_argument(
        "--clearance-weight",
        type=float,
        default=2.0
    )

    args = parser.parse_args()

    # ==================================================
    # Load scenario
    # ==================================================

    scenario = get_scenario(
        args.scenario
    )

    start = (
        scenario["start"]
    )

    commands = (
        scenario["commands"]
    )

    protected_nodes = [
        start
    ] + commands

    (
        graph,
        positions,
        blocked_nodes
    ) = create_random_grid(
        width=(
            scenario["width"]
        ),
        height=(
            scenario["height"]
        ),
        obstacle_probability=(
            scenario[
                "obstacle_probability"
            ]
        ),
        protected_nodes=(
            protected_nodes
        ),
        seed=(
            scenario["seed"]
        )
    )

    print(
        "\n"
        "========================================"
    )

    print(
        " Advanced GPSR Path Planning Demo"
    )

    print(
        "========================================"
    )

    print(
        f"\nScenario: "
        f"{args.scenario}"
    )

    print(
        f"Map size: "
        f"{scenario['width']} x "
        f"{scenario['height']}"
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
        f"Obstacles: "
        f"{len(blocked_nodes)}"
    )

    print(
        f"Start: {start}"
    )

    print(
        f"Commands: {commands}"
    )

    # ==================================================
    # 1. BASELINE
    # distance only
    # ==================================================

    baseline_graph = (
        graph.copy()
    )

    apply_distance_only_cost(
        baseline_graph,
        positions
    )

    baseline_result = (
        run_planner(
            baseline_graph,
            positions,
            start,
            commands
        )
    )

    baseline_distance = (
        calculate_path_distance(
            baseline_result["path"],
            positions
        )
    )

    baseline_clearance_data = (
        apply_distance_clearance_cost(
            graph.copy(),
            positions,
            blocked_nodes,
            distance_weight=1.0,
            clearance_weight=0.0
        )
    )

    baseline_min_clearance = (
        minimum_path_clearance(
            baseline_result["path"],
            baseline_clearance_data
        )
    )

    print_result(
        "1. Distance-only baseline",
        baseline_result,
        baseline_distance,
        baseline_min_clearance
    )

    # ==================================================
    # 2. SAFETY-AWARE COST
    # distance + clearance
    # ==================================================

    safety_graph = (
        graph.copy()
    )

    safety_clearance = (
        apply_distance_clearance_cost(
            safety_graph,
            positions,
            blocked_nodes,
            distance_weight=1.0,
            clearance_weight=(
                args.clearance_weight
            )
        )
    )

    safety_result = (
        run_planner(
            safety_graph,
            positions,
            start,
            commands
        )
    )

    safety_distance = (
        calculate_path_distance(
            safety_result["path"],
            positions
        )
    )

    safety_min_clearance = (
        minimum_path_clearance(
            safety_result["path"],
            safety_clearance
        )
    )

    print_result(
        "2. Distance + obstacle-clearance cost",
        safety_result,
        safety_distance,
        safety_min_clearance
    )

    # ==================================================
    # Visualize safety-aware route
    # ==================================================

    draw_route(
        graph=safety_graph,
        positions=positions,
        best_path=(
            safety_result["path"]
        ),
        start=start,
        commands=commands,
        best_order=(
            safety_result["order"]
        ),
        blocked_nodes=(
            blocked_nodes
        ),
        title=(
            f"Safety-Aware A* - "
            f"{args.scenario}"
        ),
        save_path=(
            f"advanced_route_"
            f"{args.scenario}.png"
        )
    )

    # ==================================================
    # 3. GRAPH REDUCTION
    # ==================================================

    (
        compressed_graph,
        compressed_positions,
        removed_nodes
    ) = contract_corridors(
        safety_graph,
        positions,
        protected_nodes=(
            protected_nodes
        )
    )

    reduction_stats = (
        graph_reduction_statistics(
            safety_graph,
            compressed_graph
        )
    )

    compressed_result = (
        run_planner(
            compressed_graph,
            compressed_positions,
            start,
            commands
        )
    )

    expanded_compressed_path = (
        expand_compressed_path(
            compressed_graph,
            compressed_result[
                "path"
            ]
        )
    )

    compressed_distance = (
        calculate_path_distance(
            expanded_compressed_path,
            positions
        )
    )

    compressed_min_clearance = (
        minimum_path_clearance(
            expanded_compressed_path,
            safety_clearance
        )
    )

    print_result(
        "3. Safety-aware + graph reduction",
        compressed_result,
        compressed_distance,
        compressed_min_clearance
    )

    print(
        "\nGraph reduction:"
    )

    print(
        f"Original nodes: "
        f"{reduction_stats['original_nodes']}"
    )

    print(
        f"Compressed nodes: "
        f"{reduction_stats['compressed_nodes']}"
    )

    print(
        f"Removed nodes: "
        f"{reduction_stats['removed_nodes']}"
    )

    print(
        f"Reduction: "
        f"{reduction_stats['reduction_percent']:.2f}%"
    )

    # ==================================================
    # 4. DYNAMIC REPLANNING
    # ==================================================

    print(
        "\n"
        "========================================"
    )

    print(
        " Dynamic Replanning Demo"
    )

    print(
        "========================================"
    )

    dynamic_result = (
        simulate_replanning_after_first_command(
            graph=graph,
            positions=positions,
            start=start,
            commands=commands,
            blocked_nodes=blocked_nodes,
            distance_weight=1.0,
            clearance_weight=(
                args.clearance_weight
            )
        )
    )

    print(
        "\nInitial command order:"
    )

    print(
        dynamic_result[
            "initial_order"
        ]
    )

    if not dynamic_result[
        "replanned"
    ]:

        print(
            "\nNo suitable simulated obstacle "
            "was found for replanning."
        )

        return

    print(
        f"\nRobot reaches first command: "
        f"{dynamic_result['current_position']}"
    )

    print(
        f"New obstacle detected at: "
        f"{dynamic_result['new_obstacle']}"
    )

    replanning_result = (
        dynamic_result[
            "replanning_result"
        ]
    )

    print(
        "\nRemaining commands:"
    )

    print(
        dynamic_result[
            "remaining_commands"
        ]
    )

    print(
        "\nNew command order:"
    )

    print(
        replanning_result[
            "best_order"
        ]
    )

    print(
        f"\nNew remaining cost: "
        f"{replanning_result['best_cost']:.3f}"
    )

    # ==================================================
    # Visualize replanned route
    # ==================================================

    draw_route(
        graph=(
            replanning_result[
                "graph"
            ]
        ),
        positions=(
            replanning_result[
                "positions"
            ]
        ),
        best_path=(
            replanning_result[
                "best_path"
            ]
        ),
        start=(
            dynamic_result[
                "current_position"
            ]
        ),
        commands=(
            dynamic_result[
                "remaining_commands"
            ]
        ),
        best_order=(
            replanning_result[
                "best_order"
            ]
        ),
        blocked_nodes=(
            replanning_result[
                "blocked_nodes"
            ]
        ),
        title=(
            "Dynamic Replanning "
            f"- {args.scenario}"
        ),
        save_path=(
            f"replanned_route_"
            f"{args.scenario}.png"
        )
    )


if __name__ == "__main__":
    main()