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

# Connor: Comms & Coordination

# Encode status of an agent into a dictionary per agent.
def encode_status(blk):
    me  = get_id()
    loc = get_location()
    round_number = get_round_number()

    # Get each part of the status message from the shared blackboard. If there is None, assign the part an empty dict.
    status = (getattr(blk, "status", None) or {}).get(me)
    target = (getattr(blk, "target", None) or {}).get(me)
    target_agent = (getattr(blk, "target_agent", None) or {}).get(me)
    command = (getattr(blk, "command", None) or {}).get(me)
    priority_of_command = (getattr(blk, "priority_of_command", None) or {}).get(me)


    msg = {
        "round_num": round_number,
        "msg_type": "STATUS",
        "status": status,
        "target": target,
        "from_id": me,
        "loc": loc,
        "target_agent": target_agent,
        "command": command,
        "priority_of_command": priority_of_command,
    }
    return msg

# Send a simple message of an agents status to everyone.
def broadcast(msg: dict):
    # If there is an encoded status and the msg_type is STATUS, then get the senders id, and send that
    # message to everyone but the broadcasting agent
    if msg and msg.get("msg_type") == "STATUS":
        me = msg.get("from_id")
        send_message(f"STATUS|{me}", [])

# read and parse through messages from the previous round (messages send at the end of a round)
def read_inbox():
    round_number = get_round_number()
    # There is no messages that can be sent on round 0, so return an empty list.
    if round_number <= 1:
        return []
    # Read the messages sent at the end of last round.
    old_msg = read_messages(round_number - 1)
    output = []
    for m in old_msg:
        # Get the str attribute from the Message object from read_messages()
        message = m.message
        # if | isn't in the str then don't do anything.
        if "|" not in message:
            continue
        # To get each part of a message, split the message on |.
        parts = message.split("|")
        output.append({
            "round_num": m.round_num,
            "sender_id": m.sender_id,
            "msg_type": parts[0],
            "parts": parts,
            "message": message,
        })
    return output

# Two agent dig at a rubble location
def handshake_for_dig(blk, rubble_loc: Location):
    round_number = get_round_number()
    me = get_id()
    loc = get_location()
    x, y = (rubble_loc.x, rubble_loc.y)
    loc_string = f"{x},{y}"

    # if there is no help_queue, set the help_queue to be an empty list.
    if getattr(blk, "help_queue", None) is None:
        blk.help_queue = []


    # Look if there is already a record of help request for this rubble
    record = None
    for i in blk.help_queue:
        if i.get("loc") == loc_string:
            record = i
            break
    # Read inbox.
    inbox = read_inbox()

    # If there is no record, then either become the requestor for help or become the helper.
    if record is None:
        req_msg = None
        # Check inbox for an agents HELP_REQ for this rubble location.
        for m in inbox:
            if (m["msg_type"] == "HELP_REQ" and len(m["parts"]) >= 3
            and m["parts"][2] == loc_string and m["sender_id"] != me):
                req_msg = m
                break
        # Become the helper and send a message of acknowledgement that you will help.
        if req_msg is not None:
            req_id = req_msg["sender_id"]
            send_message(f"ACK|{me}|{loc_string}", [req_id])
            record = {
                "loc": loc_string,
                "req_id": req_id,
                "helper_id": me,
                "phase": "EN_ROUTE",
                "dig_round": None
            }
            blk.help_queue.append(record)
            return None


        if not (loc.x == x and loc.y == y):
            return None
        send_message(f"HELP_REQ|{me}|{loc_string}", [])
        record = {
            "loc": loc_string,
            "req_id": me,
            "helper_id": None,
            "phase": "waiting_for_ack",
            "dig_round": None
        }
        blk.help_queue.append(record)
        return None

    # Little helper to check if a message exists at the location. if the msg type does not match the parameter then continue.
    def find_msg(msg_type, from_id = None, min_parts = 3):
        for m in inbox:
            if m["msg_type"] != msg_type:
                continue
            # Make sure the length of the message matches the required parts, and that the index 2 is the location.
            # Then if the from_id is None let the message come from whoever, otherwise if there is a specific "sender_id"
            # then make that the from_id.
            if len(m["parts"]) >= min_parts and m["parts"][2] == loc_string:
                if from_id is None or m["sender_id"] == from_id:
                    return m
        return None

    # Requestor
    if record["req_id"] == me:
        if record["phase"] == "waiting_for_ack":
            ack = find_msg("ACK")
            # If acknowledgement, then the helper id is who sent the acknowledgement message.
            if ack:
                record["helper_id"] = ack["sender_id"]
                record["phase"] = "waiting_at_loc"
            return None

        # A requestor agent is waiting at the rubble location.
        if record["phase"] == "waiting_at_loc":
            at_loc = find_msg("AT_LOC", from_id = record["helper_id"])
            if at_loc:
                dig_round = round_number + 1
                send_message(f"READY_DIG|{me}|{loc_string}|{dig_round}", [record["helper_id"]])
                record["dig_round"] = dig_round
                record["phase"] = "ready"
                return dig_round
            return None

        # A requestor has phase ready and while it is ready it returns "ready"
        if record["phase"] == "ready":
            return record.get("dig_round")
        return None

    # Helper:
    # If the record of a location has a phase of EN_ROUTE and the agent is at the location of the rubble,
    # send a waiting_until_ready message to the requestor.
    if record["phase"] == "EN_ROUTE":
        if loc.x == x and loc.y == y:
            send_message(f"AT_LOC|{me}|{loc_string}", [record["req_id"]])
            record["phase"] = "waiting_until_ready"
        return None

    # If the record of a location has a phase of waiting_until_ready, then the helper and requestor will be able to dig.
    if record["phase"] == "waiting_until_ready":
        ready_to_dig = find_msg("READY_DIG", from_id = record["req_id"], min_parts = 4)
        if ready_to_dig:
            record["dig_round"] = int(ready_to_dig["parts"][3])
            record["phase"] = "ready"
            return record["dig_round"]
        return None

    # If both requestor and helper emit a "ready" message then both will return the dig round.
    if record["phase"] == "ready":
        return record.get("dig_round")

    return None

# read and parse through messages from the previous round (messages send at the end of a round)
def read_inbox():
    round_number = get_round_number()
    # There is no messages that can be sent on round 0, so return an empty list.
    if round_number <= 1:
        return []
    # Read the messages sent at the end of last round.
    old_msg = read_messages(round_number - 1)
    output = []
    for m in old_msg:
        # Get the str attribute from the Message object from read_messages()
        message = m.message
        # if | isn't in the str then don't do anything.
        if "|" not in message:
            continue
        # To get each part of a message, split the message on |.
        parts = message.split("|")
        output.append({
            "round_num": m.round_num,
            "sender_id": m.sender_id,
            "msg_type": parts[0],
            "parts": parts,
            "message": message,
        })
    return output

# Two agent dig at a rubble location
def handshake_for_dig(blk, rubble_loc: Location):
    round_number = get_round_number()
    me = get_id()
    loc = get_location()
    x, y = (rubble_loc.x, rubble_loc.y)
    loc_string = f"{x},{y}"

    # if there is no help_queue, set the help_queue to be an empty list.
    if getattr(blk, "help_queue", None) is None:
        blk.help_queue = []


    # Look if there is already a record of help request for this rubble
    record = None
    for i in blk.help_queue:
        if i.get("loc") == loc_string:
            record = i
            break
    # Read inbox.
    inbox = read_inbox()

    # If there is no record, then either become the requestor for help or become the helper.
    if record is None:
        req_msg = None
        # Check inbox for an agents HELP_REQ for this rubble location.
        for m in inbox:
            if (m["msg_type"] == "HELP_REQ" and len(m["parts"]) >= 3
            and m["parts"][2] == loc_string and m["sender_id"] != me):
                req_msg = m
                break
        # Become the helper and send a message of acknowledgement that you will help.
        if req_msg is not None:
            req_id = req_msg["sender_id"]
            send_message(f"ACK|{me}|{loc_string}", [req_id])
            record = {
                "loc": loc_string,
                "req_id": req_id,
                "helper_id": me,
                "phase": "EN_ROUTE",
                "dig_round": None
            }
            blk.help_queue.append(record)
            return None


        if not (loc.x == x and loc.y == y):
            return None
        send_message(f"HELP_REQ|{me}|{loc_string}", [])
        record = {
            "loc": loc_string,
            "req_id": me,
            "helper_id": None,
            "phase": "waiting_for_ack",
            "dig_round": None
        }
        blk.help_queue.append(record)
        return None

    # Little helper to check if a message exists at the location. if the msg type does not match the parameter then continue.
    def find_msg(msg_type, from_id = None, min_parts = 3):
        for m in inbox:
            if m["msg_type"] != msg_type:
                continue
            # Make sure the length of the message matches the required parts, and that the index 2 is the location.
            # Then if the from_id is None let the message come from whoever, otherwise if there is a specific "sender_id"
            # then make that the from_id.
            if len(m["parts"]) >= min_parts and m["parts"][2] == loc_string:
                if from_id is None or m["sender_id"] == from_id:
                    return m
        return None

    # Requestor
    if record["req_id"] == me:
        if record["phase"] == "waiting_for_ack":
            ack = find_msg("ACK")
            # If acknowledgement, then the helper id is who sent the acknowledgement message.
            if ack:
                record["helper_id"] = ack["sender_id"]
                record["phase"] = "waiting_at_loc"
            return None

        # A requestor agent is waiting at the rubble location.
        if record["phase"] == "waiting_at_loc":
            at_loc = find_msg("AT_LOC", from_id = record["helper_id"])
            if at_loc:
                dig_round = round_number + 1
                send_message(f"READY_DIG|{me}|{loc_string}|{dig_round}", [record["helper_id"]])
                record["dig_round"] = dig_round
                record["phase"] = "ready"
                return dig_round
            return None

        # A requestor has phase ready and while it is ready it returns "ready"
        if record["phase"] == "ready":
            return record.get("dig_round")
        return None

    # Helper:
    # If the record of a location has a phase of EN_ROUTE and the agent is at the location of the rubble,
    # send a waiting_until_ready message to the requestor.
    if record["phase"] == "EN_ROUTE":
        if loc.x == x and loc.y == y:
            send_message(f"AT_LOC|{me}|{loc_string}", [record["req_id"]])
            record["phase"] = "waiting_until_ready"
        return None

    # If the record of a location has a phase of waiting_until_ready, then the helper and requestor will be able to dig.
    if record["phase"] == "waiting_until_ready":
        ready_to_dig = find_msg("READY_DIG", from_id = record["req_id"], min_parts = 4)
        if ready_to_dig:
            record["dig_round"] = int(ready_to_dig["parts"][3])
            record["phase"] = "ready"
            return record["dig_round"]
        return None

    # If both requestor and helper emit a "ready" message then both will return the dig round.
    if record["phase"] == "ready":
        return record.get("dig_round")

    return None


# Caleb: Navigation and Sensing Pathing

# Route object to carry pathing and sensing results between modules
class Route:
    # directions:list[Direction], cost:int, blocked:bool, ambiguous:bool, probe_loc:Location|None
    def __init__(self, directions, cost, blocked, ambiguous, probe_loc):
        self.directions = directions
        self.cost = cost
        self.blocked = blocked
        self.ambiguous = ambiguous
        self.probe_loc = probe_loc


# rubble_here_requires_pair:True/False if known; None if unknown it triggers scan or adjacency
def rubble_here_requires_pair(blk, loc: Location) -> bool | None:
    # cache lookup if known
    key = (loc.x, loc.y)
    if hasattr(blk, "known_rubble") and key in blk.known_rubble and "agents_required" in blk.known_rubble[key]:
        return blk.known_rubble[key]["agents_required"] >= 2
    # adjacent or same tile reveals full layers
    me = get_location()
    if max(abs(me.x - loc.x), abs(me.y - loc.y)) <= 1:
        top = get_cell_info_at(loc).top_layer
        if isinstance(top, Rubble):
            if hasattr(blk, "known_rubble"):
                blk.known_rubble[key] = {"agents_required": top.agents_required,
                                         "energy_required": getattr(top, "energy_required", None)}
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
                heapq.heappush(frontier,
                               (tentative_g + safe_diag_heuristic(nxt, goal), tentative_g, DIR_ORDER_IDX[d], nxt.x,
                                nxt.y))
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


#  Alaik : PLANNER / COORDINATION


ASSIGNMENTS = {}
LEADER_ID = None
TEAM_STATUS = {}



# def choose_leader():
#     ids = [get_id()]
#     for msg in read_messages():
#         try:
#             text = getattr(msg, "text", "")
#             if isinstance(text, str) and text.startswith("STATUS|"):
#                 parts = text.split("|")
#                 if len(parts) > 1 and parts[1].isdigit():
#                     ids.append(int(parts[1]))
#         except Exception as e:
#             log(f"Leader parse fail: {e}")
#     return min(ids)

def choose_leader():
    ids = [get_id()]
    for msg in read_messages():
        try:
            text = msg.text
        except Exception:
            text = ""
        if isinstance(text, str) and text.startswith("STATUS|"):
            parts = text.split("|")
            if len(parts) > 1 and parts[1].isdigit():
                ids.append(int(parts[1]))
    return min(ids)


def broadcast_status():
    """Send this agent’s location and ID to the team each turn."""
    me = get_id()
    loc = get_location()
    send_message(f"STATUS|{me}|{loc.x},{loc.y}", [])


def handle_messages():
    """Process incoming messages and update assignments."""
    global ASSIGNMENTS, TEAM_STATUS
    for msg in read_messages():
        try:
            text = msg.text
        except Exception:
            continue

        if not isinstance(text, str):
            continue

        if text.startswith("STATUS|"):
            try:
                parts = text.split("|")
                if len(parts) >= 3:
                    aid = int(parts[1])
                    xy = parts[2].split(",")
                    TEAM_STATUS[aid] = (int(xy[0]), int(xy[1]))
            except Exception:
                pass

        elif text.startswith("ASSIGN|"):
            try:
                parts = text.split("|")
                if len(parts) >= 3:
                    aid = int(parts[1])
                    xy = parts[2].split(",")
                    ASSIGNMENTS[aid] = (int(xy[0]), int(xy[1]))
            except Exception:
                pass


def broadcast_assignments():
    """Leader broadcasts survivor assignments to all teammates."""
    for aid, tgt in ASSIGNMENTS.items():
        if tgt is not None:
            send_message(f"ASSIGN|{aid}|{tgt[0]},{tgt[1]}", [])


def nearest_survivor(my_loc, survivors):
    if not survivors:
        return None
    return min(survivors, key=lambda s: abs(my_loc.x - s.x) + abs(my_loc.y - s.y))


def choose_charger(my_loc):
    chargers = get_chrg()
    if not chargers:
        return None
    return min(chargers, key=lambda c: abs(c.x - my_loc.x) + abs(c.y - my_loc.y))



######## yeogiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiii##############
# UCID HEEYOUND HAN
# heeoyoun han
BUFFER = 5
DEFAULT_DIG = 10
SAVE_COST = 1


# estimate whether there is a dig or not needed at the end
# def estimate_goal_tail_energy(goal: Location) -> int:

#     top = get_cell_info_at(goal).top_layer
#     tail = SAVE_COST
#     if isinstance(top, Rubble):
#         tail += getattr(top, "energy_required", DEFAULT_DIG)
#     return tail
def estimate_goal_tail_energy(goal: Location) -> int:
    top = get_cell_info_at(goal).top_layer
    tail = SAVE_COST
    if isinstance(top, Rubble):
        energy_req = DEFAULT_DIG
        try:
            energy_req = top.energy_required
        except Exception:
            pass
        tail = tail + energy_req
    return tail


# if no path return a large number or else calculate total energy cost of path
def estimate_path_energy_to_a_goal(path: list[Location]) -> int:
    if not path:
        return 10 ** 9
    total = 0
    for p in path:
        total += moveCostOf(p)

    total += estimate_goal_tail_energy(path[-1])
    return total


# for going to recharging centers, we dont need a dig or save
def estimate_move_cost(path: list[Location]) -> int:
    if not path:
        return 10 ** 9
    total = 0
    for p in path:
        total += moveCostOf(p)
    return total


# best survivor by actual energy cost of remaining by the agent
# goes through all survivors and finds the one with least energy cost path
def best_survivor_by_energy(my_loc: Location, survivors: list[Location]):
    best_s, best_path, best_cost = None, None, 10 ** 9
    for s in survivors:
        p = aStar(my_loc, s)
        if p:
            c = estimate_path_energy_to_a_goal(p)
            if c < best_cost:
                best_s, best_path, best_cost = s, p, c
    return best_s, best_path, best_cost


# find the charger with the least actual energy cost to reach almost the same as survivor code
# but separated for furture use including maintenance
def best_charger_by_energy(my_loc: Location):
    chargers = get_charging_cells()
    if not chargers:
        return None, None, 10 ** 9
    best_c, best_p, best_cst = None, None, 10 ** 9
    for c in chargers:
        p = aStar(my_loc, c)
        if p:
            cst = estimate_move_cost(p)
            if cst < best_cst:
                best_c, best_p, best_cst = c, p, cst
    return best_c, best_p, best_cst


# checks if the agent is really on a charger where it can recharge
def on_charger(loc: Location) -> bool:
    for c in get_charging_cells():
        if c.x == loc.x and c.y == loc.y:
            return True
    return False


##############yeogiiiiiiiiiiiiiiiii################
def think():
    me = get_id()
    my_loc = get_location()
    my_energy = get_energy_level()
    round_num = get_round_number()

    # --- Communication each round ---
    broadcast_status()
    handle_messages()

    global LEADER_ID
    if LEADER_ID is None:
        LEADER_ID = choose_leader()

    # --- Leader behavior ---
    if me == LEADER_ID:
        survivors = get_survs()
        agents = list(TEAM_STATUS.keys()) or [me]
        if survivors:
            # Assign each agent to a different survivor (split work)
            for i, aid in enumerate(agents):
                if i < len(survivors):
                    s = survivors[i]
                    ASSIGNMENTS[aid] = (s.x, s.y)
        broadcast_assignments()

    # --- Rescue behavior ---
    top = get_cell_info_at(my_loc).top_layer
    if isinstance(top, Survivor):
        save()
        send_message(f"SAVED|{me}|{my_loc.x},{my_loc.y}", [])
        return

    # --- Recharge logic ---
    if my_energy < 10:
        charger = choose_charger(my_loc)
        if charger:
            path = aStar(my_loc, charger)
            if path and len(path) > 0:
                nxt = path[0]
                move(my_loc.direction_to(nxt))
            else:
                move(Direction.CENTER)
        else:
            move(Direction.CENTER)
        return

    # --- Follower behavior: follow assignment ---
    if me not in ASSIGNMENTS or ASSIGNMENTS[me] is None:
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
        move(my_loc.direction_to(nxt))
        return

    move(Direction.CENTER)
