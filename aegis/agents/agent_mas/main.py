from aegis_game.stub import *
import heapq
import math



DIRECTIONS_TIEBREAK_ORDER = [
    Direction.NORTH, Direction.NORTHEAST, Direction.EAST, Direction.SOUTHEAST,
    Direction.SOUTH, Direction.SOUTHWEST, Direction.WEST, Direction.NORTHWEST,
    Direction.CENTER,
]
DIR_ORDER_IDX = {d: i for i, d in enumerate(DIRECTIONS_TIEBREAK_ORDER)}

def isSame(a: Location, b: Location) -> bool:
    return a.x == b.x and a.y == b.y

def moveCostOf(loc: Location) -> int:
    cell = get_cell_info_at(loc)
    return max(1, cell.move_cost)

def isSafe(loc: Location) -> bool:
    return not get_cell_info_at(loc).is_killer_cell()

def neighborsInOrder(loc: Location):
    for d in DIRECTIONS_TIEBREAK_ORDER[:-1]:
        nxt = loc.add(d)
        if on_map(nxt) and isSafe(nxt):
            yield nxt, d

def heuristic(a: Location, b: Location) -> float:
    dx = abs(a.x - b.x)
    dy = abs(a.y - b.y)
    return (dx + dy) + (math.sqrt(2) - 2) * min(dx, dy)

def rebuildPath(came_from, start, goal):
    cur = goal
    path = []
    while not isSame(cur, start):
        path.append(cur)
        cur = came_from[(cur.x, cur.y)]
    path.reverse()
    return path

def aStar(start: Location, goal: Location):
    frontier = []
    f0 = heuristic(start, goal)
    heapq.heappush(frontier, (f0, 0, DIR_ORDER_IDX[Direction.CENTER], start.x, start.y))
    came_from = {(start.x, start.y): start}
    gCost = {(start.x, start.y): 0}

    while frontier:
        f, g, order, cx, cy = heapq.heappop(frontier)
        current = Location(cx, cy)
        if isSame(current, goal):
            return rebuildPath(came_from, start, goal)
        for nxt, d in neighborsInOrder(current):
            gNew = gCost[(cx, cy)] + moveCostOf(nxt)
            key = (nxt.x, nxt.y)
            if key not in gCost or gNew < gCost[key]:
                gCost[key] = gNew
                fNew = gNew + heuristic(nxt, goal)
                heapq.heappush(frontier, (fNew, gNew, DIR_ORDER_IDX[d], nxt.x, nxt.y))
                came_from[key] = current
    return None

class Route:
    def __init__(self, directions, cost, blocked, ambiguous, probe_loc):
        self.directions = directions
        self.cost = cost
        self.blocked = blocked
        self.ambiguous = ambiguous
        self.probe_loc = probe_loc

def rubble_here_requires_pair(block, loc):
    key = (loc.x, loc.y)
    if hasattr(block, "known_rubble") and key in block.known_rubble and "agents_required" in block.known_rubble[key]:
        return block.known_rubble[key]["agents_required"] >= 2
    me = get_location()
    if max(abs(me.x - loc.x), abs(me.y - loc.y)) <= 1:
        top = get_cell_info_at(loc).top_layer
        if isinstance(top, Rubble):
            if hasattr(block, "known_rubble"):
                block.known_rubble[key] = {"agents_required": top.agents_required, "energy_required": getattr(top, "energy_required", None)}
            return top.agents_required >= 2
        return False
    return None

def scan_policy(block, route):
    if route.ambiguous and route.probe_loc is not None:
        return route.probe_loc
    return None

# renamed helpers
def safe_is_safe(loc): return on_map(loc) and (not get_cell_info_at(loc).is_killer_cell())
def safe_move_cost(loc): return max(1, get_cell_info_at(loc).move_cost)
def safe_diag_heuristic(a, b):
    dx = abs(a.x - b.x)
    dy = abs(a.y - b.y)
    return (dx + dy) + (math.sqrt(2) - 2) * min(dx, dy)
def safe_neighbors_in_order(loc):
    for d in DIRECTIONS_TIEBREAK_ORDER[:-1]:
        nxt = loc.add(d)
        if safe_is_safe(nxt):
            yield nxt, d

def safe_astar(start, goal):
    frontier = []
    heapq.heappush(frontier, (safe_diag_heuristic(start, goal), 0, DIR_ORDER_IDX[Direction.CENTER], start.x, start.y))
    came_from = {(start.x, start.y): start}
    g_cost = {(start.x, start.y): 0}
    while frontier:
        f, g, order, cx, cy = heapq.heappop(frontier)
        cur = Location(cx, cy)
        if cur.x == goal.x and cur.y == goal.y:
            path = []
            while not (cur.x == start.x and cur.y == start.y):
                path.append(cur)
                cur = came_from[(cur.x, cur.y)]
            path.reverse()
            return path, g_cost[(goal.x, goal.y)]
        for nxt, d in safe_neighbors_in_order(cur):
            key = (nxt.x, nxt.y)
            tentative_g = g_cost[(cx, cy)] + safe_move_cost(nxt)
            if key not in g_cost or tentative_g < g_cost[key]:
                g_cost[key] = tentative_g
                came_from[key] = cur
                heapq.heappush(frontier, (tentative_g + safe_diag_heuristic(nxt, goal), tentative_g, DIR_ORDER_IDX[d], nxt.x, nxt.y))
    return None, 0

def plan_route(src, dst, block):
    if src.x == dst.x and src.y == dst.y:
        return Route([], 0, False, False, None)
    path_locs, total_cost = safe_astar(src, dst)
    if path_locs is None or len(path_locs) == 0:
        return Route([], 0, True, False, None)
    probe_loc = None
    ambiguous = False
    for step in path_locs:
        top = get_cell_info_at(step).top_layer
        if isinstance(top, Rubble):
            needs_pair = rubble_here_requires_pair(block, step)
            if needs_pair is None:
                probe_loc = step
                ambiguous = True
                break
    directions = locations_to_directions(src, path_locs)
    return Route(directions, total_cost, False, ambiguous, probe_loc)

def locations_to_directions(src, waypoints):
    dirs = []
    cur = src
    for nxt in waypoints:
        dirs.append(cur.direction_to(nxt))
        cur = nxt
    return dirs

def step_toward(route):
    if not route.directions:
        return None
    return route.directions[0]


# PERSON Alaik : PLANNER / COORDINATION


ASSIGNMENTS = {}
LEADER_ID = None

def choose_leader():
    ids = [get_id()]
    for msg in read_messages():
        try:
            text = getattr(msg, "text", "")
            if isinstance(text, str) and text.startswith("STATUS|"):
                parts = text.split("|")
                if len(parts) > 1 and parts[1].isdigit():
                    ids.append(int(parts[1]))
        except Exception as e:
            log(f"Leader parse fail: {e}")
    return min(ids)
def nearest_survivor(my_loc, survivors):
    if not survivors:
        return None
    return min(survivors, key=lambda s: abs(my_loc.x - s.x) + abs(my_loc.y - s.y))

def choose_charger(my_loc):
    chargers = get_chrg()
    if not chargers:
        return None
    return min(chargers, key=lambda c: abs(c.x - my_loc.x) + abs(c.y - my_loc.y))

def think():
    me = get_id()
    my_loc = get_location()
    my_energy = get_energy_level()
    round_num = get_round_number()

    if round_num == 1:
        send_message(f"STATUS|{me}", [])
        move(Direction.CENTER)
        return

    global LEADER_ID
    if LEADER_ID is None:
        LEADER_ID = choose_leader()

    top = get_cell_info_at(my_loc).top_layer
    if isinstance(top, Survivor):
        save()
        send_message(f"SAVED|{me}|{my_loc.x},{my_loc.y}", [])
        return

    if my_energy < 10:
        charger = choose_charger(my_loc)
        if charger:
            path = aStar(my_loc, charger)
            if path:
                nxt = path[0]
                move(my_loc.direction_to(nxt))
            else:
                move(Direction.CENTER)
        else:
            move(Direction.CENTER)
        return

    survivors = get_survs()
    if me not in ASSIGNMENTS or ASSIGNMENTS[me] is None:
        target = nearest_survivor(my_loc, survivors)
        if target:
            ASSIGNMENTS[me] = (target.x, target.y)
        else:
            move(Direction.CENTER)
            return

    target = ASSIGNMENTS[me]
    goal = Location(target[0], target[1])
    route = aStar(my_loc, goal)

    if route and len(route) > 0:
        nxt = route[0]
        top_next = get_cell_info_at(nxt).top_layer
        if isinstance(top_next, Rubble):
            send_message(f"HELP_REQ|{me}|{nxt.x},{nxt.y}", [])
            move(Direction.CENTER)
            return

    if route and len(route) > 0:
        nxt = route[0]
        move(my_loc.direction_to(nxt))
        return

    move(Direction.CENTER)
