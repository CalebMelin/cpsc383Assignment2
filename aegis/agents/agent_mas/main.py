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

#Helpers for the A* algorithma
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


def distance_approx(a: Location, b: Location) -> float:
    """Compute approximate Euclidean distance (with diagonal steps)."""
    dx, dy = abs(a.x - b.x), abs(a.y - b.y)
    return (dx + dy) + (math.sqrt(2) - 2) * min(dx, dy)

def assign_initial_targets(bb: dict) -> dict[int, int | None]:
    """
    Assign each agent to the closest unclaimed survivor initially.
    Returns: dict of agent_id -> survivor_id
    """
    agents, survivors = bb.get("agents", {}), bb.get("survivors", {})
    if not survivors:
        return {aid: None for aid in agents}
    agent_ids = sorted(agents.keys())
    surv_ids = list(survivors.keys())
    remaining = set(surv_ids)
    assign = {}
    for aid in agent_ids:
        if not remaining:
            assign[aid] = None
            continue
        a_loc = agents[aid]["loc"]
        best, bestd = None, float("inf")
        for sid in remaining:
            d = euclidean_approx(a_loc, survivors[sid]["loc"])
            if d < bestd or (d == bestd and (best is None or sid < best)):
                best, bestd = sid, d
        assign[aid] = best
        remaining.discard(best)
    return assign

def replan_after_event(bb: dict, event: str) -> dict[int, int | None]:
    """
    Update survivor assignments when an event occurs (e.g., help request, low energy).
    Leader triggers reassignment when needed.
    """
    agents, survivors = bb.get("agents", {}), bb.get("survivors", {})
    cur = bb.get("assignments", {})
    if not survivors:
        return {aid: None for aid in agents}
    assigned = set(v for v in cur.values() if v is not None)
    free = set(survivors.keys()) - assigned
    leader = bb.get("leader_id")
    my_id = bb.get("my_id")

    # Only leader triggers major replanning
    if event in {"HELP_REQ_ADDED", "LOW_ENERGY"} and my_id == leader:
        return assign_initial_targets(bb)

    new = dict(cur)
    for aid in sorted(agents):
        if new.get(aid) in survivors:
            continue
        a_loc = agents[aid]["loc"]
        best, bestd = None, float("inf")
        for sid in free:
            d = euclidean_approx(a_loc, survivors[sid]["loc"])
            if d < bestd or (d == bestd and (best is None or sid < best)):
                best, bestd = sid, d
        new[aid] = best
        free.discard(best)
    return new

def choose_action_priority(bb: dict, agent_id: int) -> list[str]:
    """
    Determine which actions are most important for this agent right now.
    Returns a list of prioritized action types.
    """
    out = []
    myinfo = bb["agents"].get(agent_id, {})
    energy = myinfo.get("energy", 0)
    proj = bb.get("projected_costs", {}).get(agent_id, None)

    # Priority 1: If requesting help
    if bb.get("help_queue") and any(h["requesting_agent"] == agent_id for h in bb["help_queue"]):
        out.append("AskForHelp")

    # Priority 2: Low energy (needs recharge)
    if proj is not None and energy <= proj:
        out.append("Energy")

    # Priority 3: Unclear map areas
    if bb.get("ambiguous_routes", {}).get(agent_id, False):
        out.append("DroneScan")

    # Priority 4: Assigned survivor target
    if bb["assignments"].get(agent_id) is not None:
        out.append("PathToSurvivor")

    # Default action
    out.append("Idle/Assist")
    return out

def planner_summary(bb: dict) -> dict:
    """Return a compact summary of planner state for debugging/logging."""
    return {
        "round": bb.get("round_num"),
        "leader": bb.get("leader_id"),
        "assignments": bb.get("assignments"),
        "help_queue_len": len(bb.get("help_queue", [])),
    }
def think() -> None:
    """Person 2 main control loop"""
    round_num = get_round_number()
    my_id = get_id()  # Changed from get_agent_id()
    my_loc = get_location()
    my_energy = get_energy()

    # Initialize global blackboard (shared state across rounds)
    global BB
    if "BB" not in globals():
        BB = {
            "agents": {}, "survivors": {}, "help_queue": [],
            "leader_id": None, "assignments": {}, "projected_costs": {},
            "ambiguous_routes": {}, "round_num": round_num
        }

    # Update local agent info
    BB["round_num"] = round_num
    BB["my_id"] = my_id
    BB["agents"][my_id] = {
        "loc": my_loc,
        "energy": my_energy,
        "status": BB["agents"].get(my_id, {}).get("status", "")
    }

    # Collect survivor data
    surv_locs = get_survs()
    BB["survivors"] = {i: {"loc": loc} for i, loc in enumerate(surv_locs)}

    # Determine team leader (agent with smallest ID)
    BB["leader_id"] = min(BB["agents"].keys())

    # Initial assignment (first round only)
    if round_num == 1 or not BB["assignments"]:
        BB["assignments"] = assign_initial_targets(BB)
        send_message("INIT_ASSIGN_DONE", [])

    # Replan if help requests exist
    if BB["help_queue"]:
        BB["assignments"] = replan_after_event(BB, "HELP_REQ_ADDED")

    # Determine and log action priorities
    priorities = choose_action_priority(BB, my_id)
    log(f"Priorities: {priorities}")
    log(planner_summary(BB))

    # ==========================================================
    # 🚶 PATH EXECUTION LOGIC
    # ----------------------------------------------------------
    # Follows assigned path to nearest survivor if possible.
    # ==========================================================
    survs = get_survs()
    if survs:
        me = get_location()
        my_target_idx = BB["assignments"].get(my_id, 0)
        if my_target_idx is not None and my_target_idx < len(survs):
            goal = survs[my_target_idx]
        else:
            goal = survs[0]
        path = aStar(me, goal)
        if path:
            nxt = path[0]
            step = me.direction_to(nxt)
            move(step)
            return

    # Default: stay still if no path or survivors
    move(Direction.CENTER)
