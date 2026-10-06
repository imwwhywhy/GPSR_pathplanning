from copy import deepcopy


SCENARIOS = {

    # ==================================================
    # Small random environment
    # ==================================================

    "small_random": {

        "width": 12,
        "height": 12,

        "obstacle_probability": 0.15,

        "start": (
            0,
            0
        ),

        "commands": [
            (1, 10),
            (10, 10),
            (10, 1)
        ],

        "seed": 42
    },

    # ==================================================
    # Medium environment
    # ==================================================

    "medium_random": {

        "width": 20,
        "height": 20,

        "obstacle_probability": 0.20,

        "start": (
            0,
            0
        ),

        "commands": [
            (2, 17),
            (17, 17),
            (17, 2)
        ],

        "seed": 100
    },

    # ==================================================
    # Denser obstacle environment
    # ==================================================

    "dense_random": {

        "width": 30,
        "height": 30,

        "obstacle_probability": 0.28,

        "start": (
            1,
            1
        ),

        "commands": [
            (3, 26),
            (26, 26),
            (26, 3)
        ],

        "seed": 2026
    }
}


def get_scenario(
    name
):
    """
    Return a copy of a scenario configuration.
    """

    if name not in SCENARIOS:

        available = ", ".join(
            SCENARIOS.keys()
        )

        raise ValueError(
            f"Unknown scenario '{name}'. "
            f"Available scenarios: {available}"
        )

    return deepcopy(
        SCENARIOS[name]
    )


def get_available_scenarios():
    """
    Return all scenario names.
    """

    return list(
        SCENARIOS.keys()
    )