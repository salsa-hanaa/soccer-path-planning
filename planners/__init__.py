from planners.example_planner import NaiveDirectPlanner
from planners.astar import AStarPlanner

# Each teammate registers their planner here with a short name used on the
# command line: `python main.py --planner astar`
REGISTRY = {
    "naive": NaiveDirectPlanner,
    "astar": AStarPlanner,
}
