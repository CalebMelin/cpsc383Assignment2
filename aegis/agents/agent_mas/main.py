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


from math import inf
from typing import Dict, Tuple, List


#  Helper functions

def distance_finder(a: Tuple[int, int], b: Tuple[int, int]) -> int:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def nearest_charger(my_loc: Tuple[int, int], chargers: List[Tuple[int, int]]):
    if not chargers:
        return None
    return min(chargers, key=lambda c: manhattan(my_loc, c))


def nearest_survivor(my_loc: Tuple[int, int], survivors: List[Tuple[int, int]]):
    if not survivors:
        return None
    return min(survivors, key=lambda s: manhattan(my_loc, s))


def get_leader_id(agent_ids: List[int]) -> int:
    return min(agent_ids) if agent_ids else -1

#  Main planner logic
def planner_step(all_agents: Dict[int, Tuple[int, int]]):
    from aegis import (
        get_id,
        get_location,
        get_energy_level,
        get_survs,
        get_chrg,
        send_message,
        read_messages,
        log,
    )

    # 1. Agent state
    
    my_id = get_id()
    my_loc = get_location()
    energy = get_energy_level()
    survivors = get_survs()
    chargers = get_chrg()

    log(f"[Planner] Agent {my_id} | Location={my_loc} | Energy={energy}")
    log(f"[Planner] Survivors: {survivors}")
    log(f"[Planner] Chargers: {chargers}")

    # --------------------------------------------------------
    # 2. Leader election (lowest ID)
    # --------------------------------------------------------
    leader_id = get_leader_id(list(all_agents.keys()))
    is_leader = my_id == leader_id
    if is_leader:
        log(f"[Planner] I am the leader (ID {leader_id})")

    # 3. Energy check — go charge if needed

    if energy < 10:
        target_charger = nearest_charger(my_loc, chargers)
        if target_charger:
            log(f"[Energy] Low energy ({energy}). Moving to charger at {target_charger}")
            send_message(f"CHARGE {target_charger[0]} {target_charger[1]}", [])
            # TODO: call your movement/navigation here, e.g. move_toward(target_charger)
        else:
            log("[Energy] WARNING: No chargers detected!")


    # 4. Leader assigns survivors
    
    if is_leader and survivors:
        for agent_id, loc in all_agents.items():
            target = nearest_survivor(loc, survivors)
            if target:
                send_message(f"ASSIGN {target[0]} {target[1]}", [agent_id])
                log(f"[Leader] Assigned Agent {agent_id} → Survivor at {target}")

   
    # 5. Read & react to messages
    inbox = read_messages()
    for msg in inbox:
        parts = msg.split()
        if not parts:
            continue

        tag = parts[0]
        if tag == "ASSIGN":
            x, y = int(parts[1]), int(parts[2])
            log(f"[Message] Assigned to survivor at ({x}, {y})")
            # TODO: move_toward((x, y))
        elif tag == "CHARGE":
            x, y = int(parts[1]), int(parts[2])
            log(f"[Message] Another agent is charging at ({x}, {y})")
        elif tag == "HELP":
            x, y = int(parts[1]), int(parts[2])
            log(f"[Message] Help requested at ({x}, {y}) — consider assisting.")
        else:
            log(f"[Message] Unknown message type: {msg}")

    send_message(f"STATUS {my_id} {energy} {my_loc[0]} {my_loc[1]}", [])
    log(f"[Planner] Step complete for Agent {my_id}\n")
