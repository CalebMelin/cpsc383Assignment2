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
