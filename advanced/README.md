# Advanced Path Planning

This folder extends the baseline A* + multi-command
ordering implementation without replacing the original code.

## Motivation

The baseline implementation demonstrates two ideas:

1. A* finds the shortest path between two locations.
2. When multiple commands arrive simultaneously,
   all n! command orders are evaluated and the
   minimum-cost order is selected.

The advanced implementation explores how this approach
can become more suitable for robot navigation.

---

## 1. Map Generalization

`random_grid.py`

The planner is no longer tested on only one fixed grid.

Random maps can be generated with different:

- map sizes
- obstacle densities
- random seeds

The same A* implementation is used without modification.

---

## 2. Safety-Aware Cost

`cost_model.py`

The baseline cost is:

    cost = distance

The advanced version uses:

    cost =
    distance_weight * distance
    +
    clearance_penalty

Paths close to obstacles therefore receive a larger cost.

This allows the planner to prefer a slightly longer but
safer path.

---

## 3. Graph Reduction

`graph_reduction.py`

Degree-2 corridor nodes may be contracted.

For example:

    A -- B -- C -- D -- E

can become:

    A ------------ E

while preserving the total path cost.

The goal is to reduce the number of states searched by A*
without changing the optimal route cost.

Important nodes such as the start position and command
destinations are never contracted.

---

## 4. Dynamic Replanning

`dynamic_replanning.py`

The original planner assumes that the environment does not
change after planning.

The advanced implementation simulates:

    initial planning
          ↓
    robot reaches first command
          ↓
    new obstacle appears
          ↓
    update graph
          ↓
    replan remaining commands

This provides a simple foundation for future integration
with dynamic robot navigation.

---

## 5. Benchmark

`benchmark.py`

The following versions are compared:

- distance-only baseline
- distance + obstacle clearance
- safety-aware planning + graph reduction

Metrics include:

- objective cost
- geometric path distance
- minimum obstacle clearance
- expanded nodes
- planning time
- graph size

---

## Running

Run commands from the repository root.

### Advanced demo

```bash
python -m advanced.advanced_demo --scenario small_random