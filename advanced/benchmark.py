import time

from astar import (
    find_best_command_order
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
    expand_compressed_path
)


def run_planner(
    graph,
    positions,
    start,
    commands
):
    """
    Run planner and record planning time.
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

    elapsed = (
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

        "expanded":
            expanded_nodes,

        "time":
            elapsed
    }


def benchmark_scenario(
    scenario_name
):
    """
    Benchmark one scenario.
    """

    scenario = get_scenario(
        scenario_name
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

    results = []

    # ==================================================
    # Distance-only baseline
    # ==================================================

    baseline_graph = (
        graph.copy()
    )

    apply_distance_only_cost(
        baseline_graph,
        positions
    )

    baseline = run_planner(
        baseline_graph,
        positions,
        start,
        commands
    )

    clearance_reference = (
        apply_distance_clearance_cost(
            graph.copy(),
            positions,
            blocked_nodes,
            clearance_weight=0.0
        )
    )

    results.append({

        "scenario":
            scenario_name,

        "version":
            "distance",

        "nodes":
            baseline_graph.number_of_nodes(),

        "objective_cost":
            baseline["cost"],

        "distance":
            calculate_path_distance(
                baseline["path"],
                positions
            ),

        "clearance":
            minimum_path_clearance(
                baseline["path"],
                clearance_reference
            ),

        "expanded":
            baseline["expanded"],

        "time_ms":
            baseline["time"]
            * 1000
    })

    # ==================================================
    # Distance + clearance
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
            clearance_weight=2.0
        )
    )

    safety = run_planner(
        safety_graph,
        positions,
        start,
        commands
    )

    results.append({

        "scenario":
            scenario_name,

        "version":
            "safety",

        "nodes":
            safety_graph.number_of_nodes(),

        "objective_cost":
            safety["cost"],

        "distance":
            calculate_path_distance(
                safety["path"],
                positions
            ),

        "clearance":
            minimum_path_clearance(
                safety["path"],
                safety_clearance
            ),

        "expanded":
            safety["expanded"],

        "time_ms":
            safety["time"]
            * 1000
    })

    # ==================================================
    # Safety + graph compression
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

    compressed = run_planner(
        compressed_graph,
        compressed_positions,
        start,
        commands
    )

    expanded_path = (
        expand_compressed_path(
            compressed_graph,
            compressed["path"]
        )
    )

    results.append({

        "scenario":
            scenario_name,

        "version":
            "compressed",

        "nodes":
            compressed_graph.number_of_nodes(),

        "objective_cost":
            compressed["cost"],

        "distance":
            calculate_path_distance(
                expanded_path,
                positions
            ),

        "clearance":
            minimum_path_clearance(
                expanded_path,
                safety_clearance
            ),

        "expanded":
            compressed["expanded"],

        "time_ms":
            compressed["time"]
            * 1000
    })

    return results


def print_table(
    all_results
):
    """
    Print benchmark result table.
    """

    header = (
        f"{'Scenario':<16}"
        f"{'Version':<14}"
        f"{'Nodes':<8}"
        f"{'Obj Cost':<12}"
        f"{'Distance':<12}"
        f"{'Clearance':<12}"
        f"{'Expanded':<10}"
        f"{'Time(ms)':<10}"
    )

    print(
        "\n"
        + header
    )

    print(
        "-" * len(
            header
        )
    )

    for result in all_results:

        print(
            f"{result['scenario']:<16}"
            f"{result['version']:<14}"
            f"{result['nodes']:<8}"
            f"{result['objective_cost']:<12.2f}"
            f"{result['distance']:<12.2f}"
            f"{result['clearance']:<12.2f}"
            f"{result['expanded']:<10}"
            f"{result['time_ms']:<10.3f}"
        )


def main():

    all_results = []

    for scenario_name in (
        get_available_scenarios()
    ):

        scenario_results = (
            benchmark_scenario(
                scenario_name
            )
        )

        all_results.extend(
            scenario_results
        )

    print(
        "\n"
        "========================================"
    )

    print(
        " Path Planning Benchmark"
    )

    print(
        "========================================"
    )

    print_table(
        all_results
    )


if __name__ == "__main__":
    main()