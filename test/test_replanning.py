from advanced.cost_model import (
    apply_distance_clearance_cost
)

from advanced.dynamic_replanning import (
    replan_remaining_commands
)

from advanced.random_grid import (
    create_random_grid
)


def test_dynamic_replanning():

    start = (
        5,
        0
    )

    remaining_commands = [
        (
            5,
            5
        )
    ]

    protected_nodes = [
        start,
        *remaining_commands
    ]

    (
        graph,
        positions,
        blocked_nodes
    ) = create_random_grid(
        width=6,
        height=6,
        obstacle_probability=0.0,
        protected_nodes=(
            protected_nodes
        ),
        seed=1
    )

    apply_distance_clearance_cost(
        graph,
        positions,
        blocked_nodes
    )

    # Originally robot would probably use:
    #
    # (5,0)
    # (5,1)
    # (5,2)
    # ...
    #
    # Now create a new obstacle.
    new_obstacle = (
        5,
        2
    )

    result = (
        replan_remaining_commands(
            graph=graph,
            positions=positions,
            current_position=start,
            remaining_commands=(
                remaining_commands
            ),
            blocked_nodes=(
                blocked_nodes
            ),
            new_blocked_nodes=[
                new_obstacle
            ]
        )
    )

    new_path = (
        result["best_path"]
    )

    assert (
        new_obstacle
        not in new_path
    )

    assert (
        new_path[0]
        == start
    )

    assert (
        new_path[-1]
        == remaining_commands[0]
    )