from aegis_game.stub import *
import heapq 
import math 

#Tiebreak order for pathfinding
DIRECTIONS_TIEBREAK_ORDER = [
    Direction.NORTH,
    Direction.NORTHEAST,
    Direction.EAST,
    Direction.SOUTHEAST,
    Direction.SOUTH,
    Direction.SOUTHWEST,
    Direction.WEST,
    Direction.NORTHWEST,
    Direction.CENTER,
]
#Lookup for order index
DIR_ORDER_IDX = {d: i for i, d in enumerate(DIRECTIONS_TIEBREAK_ORDER)}

#Helpers for the A* algorithm
def isSame(a: Location, b: Location) -> bool:
    return a.x == b.x and a.y == b.y

def moveCostOf(loc: Location) -> int:
    #When HIDDEN_MOVE_COSTS is turned on the unseen cells act like cost=1 until they are revealed
    cell = get_cell_info_at(loc)
    return max(1, cell.move_cost)

def isSafe(loc: Location) -> bool:
    #Skip killer cells at expansion time so that our dude doesnt commit suicide
    return not get_cell_info_at(loc).is_killer_cell()

def neighborsInOrder(loc: Location):
    #Look at neighbors exactly in the tie-break order
    for d in DIRECTIONS_TIEBREAK_ORDER[:-1]:  #Exclude the CENTER for movement expansions
        nxt = loc.add(d)
        if on_map(nxt) and isSafe(nxt):
            yield nxt, d

def heuristic(a: Location, b: Location) -> int:
    #Gets how far points are vertically and horozontally then accounts for diagonal movement: cost root 2
    dx = abs(a.x - b.x)
    dy = abs(a.y - b.y)
    return (dx + dy) + (math.sqrt(2) - 2) * min(dx, dy)

def rebuildPath(came_from: dict[tuple[int, int], Location],
                     start: Location, goal: Location) -> list[Location]:
    cur = goal
    path: list[Location] = []
    while not isSame(cur, start):
        path.append(cur)
        cur = came_from[(cur.x, cur.y)]
    path.reverse()
    return path

def aStar(start: Location, goal: Location) -> list[Location] | None:
    #A* with tie-breaking:
    #priority = f, then g, then last-move direction order (N,NE,E,SE,S,SW,W,NW,C)
    #This enforces the required order when f and g are equal
    frontier: list[tuple[float, int, int, int, int]] = []  #(f, g, dirOrder, x, y)
    f0 = heuristic(start, goal)
    heapq.heappush(frontier, (f0, 0, DIR_ORDER_IDX[Direction.CENTER], start.x, start.y))

    came_from: dict[tuple[int, int], Location] = {(start.x, start.y): start}
    gCost: dict[tuple[int, int], int] = {(start.x, start.y): 0}

    #Early exit when I POP the goal
    while frontier:
        f, g, dirOrder, cx, cy = heapq.heappop(frontier)
        current = Location(cx, cy)
        if isSame(current, goal):
            return rebuildPath(came_from, start, goal)

        for nxt, d in neighborsInOrder(current):  #The neighbor expansion is also tie-broken
            gNew = gCost[(cx, cy)] + moveCostOf(nxt)
            key = (nxt.x, nxt.y)
            if key not in gCost or gNew < gCost[key]:
                gCost[key] = gNew
                fNew = gNew + heuristic(nxt, goal)
                heapq.heappush(frontier, (fNew, gNew, DIR_ORDER_IDX[d], nxt.x, nxt.y))
                came_from[key] = current
    return None



#Caleb: Navigation and Sensing Pathing

#Route object to carry pathing and sensing results between modules
class Route:
    #directions:list[Direction], cost:int, blocked:bool, ambiguous:bool, probe_loc:Location|None
    def __init__(self, directions, cost, blocked, ambiguous, probe_loc):
        self.directions = directions
        self.cost = cost
        self.blocked = blocked
        self.ambiguous = ambiguous
        self.probe_loc = probe_loc

#rubble_here_requires_pair:True/False if known; None if unknown it triggers scan or adjacency
def rubble_here_requires_pair(blk, loc: Location) -> bool | None:
    #cache lookup if known
    key = (loc.x, loc.y)
    if hasattr(blk, "known_rubble") and key in blk.known_rubble and "agents_required" in blk.known_rubble[key]:
        return blk.known_rubble[key]["agents_required"] >= 2
    #adjacent or same tile reveals full layers
    me = get_location()
    if max(abs(me.x - loc.x), abs(me.y - loc.y)) <= 1:
        top = get_cell_info_at(loc).top_layer
        if isinstance(top, Rubble):
            if hasattr(blk, "known_rubble"):
                blk.known_rubble[key] = {"agents_required": top.agents_required, "energy_required": getattr(top, "energy_required", None)}
            return top.agents_required >= 2
        return False
    #unknown if not adjacent and not scanned into cache
    return None

#scan_policy:return probe_loc if route is unknown otherwise None
def scan_policy(blk, route: Route) -> Location | None:
    if route.ambiguous and route.probe_loc is not None:
        return route.probe_loc
    return None

#estimate_path_cost:sum of move costs via A* and fallback to Manhattan method if route is blocked
def estimate_path_cost(src: Location, dst: Location) -> int:
    if not on_map(src) or not on_map(dst):
        return 1_000_000
    path, cost = _astar(src, dst)
    if path is None:
        dx = abs(src.x - dst.x)
        dy = abs(src.y - dst.y)
        return dx + dy
    return cost

#plan_route:returns the Route with uncertainty flagged when rubble requirements are unknown on path
def plan_route(src: Location, dst: Location, blk) -> Route:
    if src.x == dst.x and src.y == dst.y:
        return Route(directions=[], cost=0, blocked=False, ambiguous=False, probe_loc=None)
    path_locs, total_cost = _astar(src, dst)
    if path_locs is None or len(path_locs) == 0:
        return Route(directions=[], cost=0, blocked=True, ambiguous=False, probe_loc=None)
    probe_loc = None
    ambiguous = False
    for step in path_locs:
        top = get_cell_info_at(step).top_layer
        if isinstance(top, Rubble):
            needs_pair = rubble_here_requires_pair(blk, step)
            if needs_pair is None:
                probe_loc = step
                ambiguous = True
                break
    directions = _locations_to_directions(src, path_locs)
    return Route(directions=directions, cost=total_cost, blocked=False, ambiguous=ambiguous, probe_loc=probe_loc)

#step_toward:returns next direction or None if at the destination
def step_toward(route: Route) -> Direction | None:
    if not route.directions:
        return None
    return route.directions[0]


#internal helpers

#locations to directions from source
def _locations_to_directions(src: Location, waypoints: list[Location]) -> list[Direction]:
    dirs: list[Direction] = []
    cur = src
    for nxt in waypoints:
        dirs.append(cur.direction_to(nxt))
        cur = nxt
    return dirs

#safe check for killer cells uses the existing map and cell info
def _is_safe(loc: Location) -> bool:
    return on_map(loc) and (not get_cell_info_at(loc).is_killer_cell())

#hidden-cost guard: unknown treated as cost=1
def _move_cost_of(loc: Location) -> int:
    cell = get_cell_info_at(loc)
    return max(1, cell.move_cost)

#diagonal-aware heuristic
def _diag_heuristic(a: Location, b: Location) -> float:
    dx = abs(a.x - b.x)
    dy = abs(a.y - b.y)
    return (dx + dy) + (math.sqrt(2) - 2) * min(dx, dy)

#neighbors in the fixed order reuses the global tiebreak list, skip CENTER
def _neighbors_in_order(loc: Location):
    for d in DIRECTIONS_TIEBREAK_ORDER[:-1]:
        nxt = loc.add(d)
        if _is_safe(nxt):
            yield nxt, d

#A* with tie-breaking(f,g,dirOrder) using the global DIR_ORDER_IDX
def _astar(start: Location, goal: Location) -> tuple[list[Location] | None, int]:
    frontier: list[tuple[float, int, int, int, int]] = []
    heapq.heappush(frontier, (_diag_heuristic(start, goal), 0, DIR_ORDER_IDX[Direction.CENTER], start.x, start.y))
    came_from: dict[tuple[int, int], Location] = {(start.x, start.y): start}
    g_cost: dict[tuple[int, int], int] = {(start.x, start.y): 0}
    while frontier:
        f, g, order, cx, cy = heapq.heappop(frontier)
        cur = Location(cx, cy)
        if cur.x == goal.x and cur.y == goal.y:
            path: list[Location] = []
            while not (cur.x == start.x and cur.y == start.y):
                path.append(cur)
                cur = came_from[(cur.x, cur.y)]
            path.reverse()
            return path, g_cost[(goal.x, goal.y)]
        for nxt, d in _neighbors_in_order(cur):
            key = (nxt.x, nxt.y)
            tentative_g = g_cost[(cx, cy)] + _move_cost_of(nxt)
            if key not in g_cost or tentative_g < g_cost[key]:
                g_cost[key] = tentative_g
                came_from[key] = cur
                heapq.heappush(frontier, (tentative_g + _diag_heuristic(nxt, goal), tentative_g, DIR_ORDER_IDX[d], nxt.x, nxt.y))
    return None, 0

def think() -> None:
    """Do not remove this function, it must always be defined."""
    log("Thinking")

    # On the first round, send a request for surrounding information
    # by moving to the center (not moving). This will help initiate pathfinding.
    if get_round_number() == 1:
        move(Direction.CENTER)
        send_message("hello world", [])  # Broadcast to all teammates
        return

    # On subsequent rounds, read and log all received messages.
    messages = read_messages()
    log(messages)

    # Fetch the cell at the agent's current location.
    # If you want to check a different location, use `on_map(loc)` first
    # to ensure it's within the world bounds. The agent's own location is always valid.
    cell = get_cell_info_at(get_location())

    # Get the top layer at the agent's current location.
    # If a survivor is present, save it and end the turn.
    top_layer = cell.top_layer
    if isinstance(top_layer, Survivor):
        save()
        return

    #Pathfinding added
    survs = get_survs()
    if survs:
        me = get_location()
        goal = survs[0]  #Exactly one survivor per specified map
        path = aStar(me, goal)  #Recomputes each round to handle hidden costs: V3
        if path:
            nxt = path[0]
            step_dir = me.direction_to(nxt)
            move(step_dir)
            return

    # Default action: Move the agent north if no other specific conditions are met.
    move(Direction.NORTH)
