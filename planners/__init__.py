from planners.example_planner import NaiveDirectPlanner

# Each teammate registers their planner here with a short name used on the
# command line: `python main.py --planner astar`
REGISTRY = {
    "naive": NaiveDirectPlanner,
}
