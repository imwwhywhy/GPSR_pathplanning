import matplotlib.pyplot as plt
import networkx as nx


def draw_route(
    graph,
    positions,
    best_path,
    start,
    commands,
    best_order,
    blocked_nodes=None,
    title="Path Planning Result",
    save_path=None,
    show=True
):
    """
    Visualize a graph-based path planning result.

    Parameters
    ----------
    graph
        NetworkX graph.

    positions
        Node positions.

    best_path
        Full optimal path returned by the planner.

    start
        Robot starting node.

    commands
        All command destinations.

    best_order
        Optimal command execution order.

    blocked_nodes
        Obstacle nodes.

    title
        Figure title.

    save_path
        Optional image file path.

    show
        Whether to display the matplotlib window.
    """

    if blocked_nodes is None:
        blocked_nodes = []

    plt.figure(
        figsize=(10, 10)
    )

    # --------------------------------------------------
    # Draw normal graph edges
    # --------------------------------------------------

    nx.draw_networkx_edges(
        graph,
        positions,
        edge_color="lightgray",
        width=1
    )

    # --------------------------------------------------
    # Draw normal nodes
    # --------------------------------------------------

    normal_nodes = [
        node
        for node in graph.nodes
        if node != start
        and node not in commands
    ]

    nx.draw_networkx_nodes(
        graph,
        positions,
        nodelist=normal_nodes,
        node_color="lightblue",
        node_size=250
    )

    # --------------------------------------------------
    # Draw start node
    # --------------------------------------------------

    nx.draw_networkx_nodes(
        graph,
        positions,
        nodelist=[
            start
        ],
        node_color="limegreen",
        node_size=700
    )

    # --------------------------------------------------
    # Draw command nodes
    # --------------------------------------------------

    nx.draw_networkx_nodes(
        graph,
        positions,
        nodelist=commands,
        node_color="orange",
        node_size=700
    )

    # --------------------------------------------------
    # Draw obstacles
    # --------------------------------------------------

    if blocked_nodes:

        obstacle_x = [
            node[0]
            for node
            in blocked_nodes
        ]

        obstacle_y = [
            node[1]
            for node
            in blocked_nodes
        ]

        plt.scatter(
            obstacle_x,
            obstacle_y,
            marker="s",
            s=180,
            color="black",
            label="Obstacle"
        )

    # --------------------------------------------------
    # Draw optimal route
    # --------------------------------------------------

    route_edges = list(
        zip(
            best_path[:-1],
            best_path[1:]
        )
    )

    nx.draw_networkx_edges(
        graph,
        positions,
        edgelist=route_edges,
        edge_color="red",
        width=4
    )

    # --------------------------------------------------
    # Node coordinate labels
    # --------------------------------------------------

    nx.draw_networkx_labels(
        graph,
        positions,
        font_size=7
    )

    # --------------------------------------------------
    # Start label
    # --------------------------------------------------

    start_x, start_y = start

    plt.text(
        start_x,
        start_y - 0.35,
        "START",
        fontsize=11,
        fontweight="bold",
        ha="center",
        color="green"
    )

    # --------------------------------------------------
    # Command execution order labels
    # --------------------------------------------------

    for index, command in enumerate(
        best_order,
        start=1
    ):

        x, y = command

        plt.text(
            x,
            y + 0.35,
            f"Command {index}",
            fontsize=10,
            fontweight="bold",
            ha="center",
            color="darkorange"
        )

    plt.title(
        title,
        fontsize=15
    )

    plt.axis(
        "equal"
    )

    plt.tight_layout()

    # --------------------------------------------------
    # Save figure
    # --------------------------------------------------

    if save_path is not None:

        plt.savefig(
            save_path,
            dpi=200,
            bbox_inches="tight"
        )

    if show:
        plt.show()

    else:
        plt.close()


def draw_map_only(
    graph,
    positions,
    blocked_nodes=None,
    title="Map"
):
    """
    Draw only the map without a planned route.

    Useful for checking whether the generated map
    looks correct.
    """

    if blocked_nodes is None:
        blocked_nodes = []

    plt.figure(
        figsize=(10, 10)
    )

    nx.draw_networkx_edges(
        graph,
        positions,
        edge_color="lightgray",
        width=1
    )

    nx.draw_networkx_nodes(
        graph,
        positions,
        node_color="lightblue",
        node_size=250
    )

    if blocked_nodes:

        obstacle_x = [
            node[0]
            for node
            in blocked_nodes
        ]

        obstacle_y = [
            node[1]
            for node
            in blocked_nodes
        ]

        plt.scatter(
            obstacle_x,
            obstacle_y,
            marker="s",
            s=180,
            color="black"
        )

    plt.title(
        title
    )

    plt.axis(
        "equal"
    )

    plt.tight_layout()

    plt.show()