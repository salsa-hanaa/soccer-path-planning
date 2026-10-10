from planners.example_planner import NaiveDirectPlanner
from planners.astar import AStarPlanner
from planners.greedy import GreedyPlanner

# Each teammate registers their planner here with a short name used on the
# command line: `python main.py --planner astar`
REGISTRY = {
    "naive": NaiveDirectPlanner,
    "astar": AStarPlanner,
    "greedy": GreedyPlanner,
}
