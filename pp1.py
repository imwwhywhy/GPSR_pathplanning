from itertools import permutations

# shortest distance
distance = {
    "S": {"S": 0, "A": 2, "B": 3, "C": 7},
    "A": {"S": 2, "A": 0, "B": 5, "C": 6},
    "B": {"S": 3, "A": 5, "B": 0, "C": 10},
    "C": {"S": 7, "A": 6, "B": 10, "C": 0},
}

start = "S"
tasks = ["A", "B", "C"]

best_order = None
best_cost = float("inf")
checked_count = 0

# Permutation and combination task sequencing
for order in permutations(tasks):
    current = start
    total_cost = 0

    # calculate total distance
    for destination in order:
        total_cost = total_cost + distance[current][destination]
        current = destination

    checked_count = checked_count + 1
    print(" -> ".join([start] + list(order)), "| cost =", total_cost)

    # record the shortest distance
    if total_cost < best_cost:
        best_cost = total_cost
        best_order = order

print("Checked orders:", checked_count)
print("Best route:", " -> ".join([start] + list(best_order)))
print("Best cost:", best_cost)