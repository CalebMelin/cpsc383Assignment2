# ================================================
# CPSC 383 — Assignment 2 (MAS)
# Team Roles:
#   Person 1 — Navigation & Sensing (A* + scan): Caleb
#   Person 2 — Planner / Assignment Logic: Alaik
#   Person 3 — Energy & Charging math/policy: HeeYoung Han
#   Person 4 — Comms & Coordination: Connor
# Notes:
#   - Pathfinding: safe A* that avoids killer cells.
#   - Charging rule (simple & deterministic): ONLY charge if
#       current_energy < energy needed to reach assigned survivor
#       + (dig+save) at destination.
#   - Dig rule:
#       • If rubble needs 1 agent → dig when energy >= energy_required.
#       • If required agents already present on the tile → dig now (no handshake).
#       • Otherwise use a 2-agent handshake (HELP_REQ/ACK/AT_LOC/READY_DIG).
#   - Help Dispatcher:
#       Agents adopt HELP_REQ messages and head there automatically.
# ================================================

from aegis_game.stub import *
import heapq, math

# ------------------------------------------------
# [Caleb] Directions & basic A* primitives
# ------------------------------------------------
DIRECTIONS_TIEBREAK_ORDER = [
    Direction.NORTH, Direction.NORTHEAST, Direction.EAST, Direction.SOUTHEAST,
    Direction.SOUTH, Direction.SOUTHWEST, Direction.WEST, Direction.NORTHWEST,
    Direction.CENTER,
]
DIR_ORDER_IDX = {d: i for i, d in enumerate(DIRECTIONS_TIEBREAK_ORDER)}

def isSame(a: Location, b: Location) -> bool:
    return a.x == b.x and a.y == b.y

def moveCostOf(loc: Location) -> int:
    # movement always costs at least 1
    return max(1, get_cell_info_at(loc).move_cost)

def isSafe(loc: Location) -> bool:
    # avoid killer cells and off-map
    return on_map(loc) and not get_cell_info_at(loc).is_killer_cell()

def neighborsInOrder(loc: Location):
    # 8-neighborhood, deterministic tie-break order
    for d in DIRECTIONS_TIEBREAK_ORDER[:-1]:
        nxt = loc.add(d)
        if isSafe(nxt):
            yield nxt, d

def heuristic(a: Location, b: Location) -> float:
    # diagonal distance heuristic (admissible for 8-way grid with unit+ costs)
    dx = abs(a.x - b.x); dy = abs(a.y - b.y)
    return (dx + dy) + (math.sqrt(2) - 2) * min(dx, dy)

def safe_astar(start: Location, goal: Location):
    # classic A*; returns (path, g_cost_to_goal)
    if isSame(start, goal): return [], 0
    frontier = []
    heapq.heappush(frontier, (heuristic(start, goal), 0, DIR_ORDER_IDX[Direction.CENTER], start.x, start.y))
    came_from = {(start.x, start.y): start}
    g_cost = {(start.x, start.y): 0}
    while frontier:
        f, g, order, cx, cy = heapq.heappop(frontier)
        cur = Location(cx, cy)
        if isSame(cur, goal):
            path = []
            while not isSame(cur, start):
                path.append(cur); cur = came_from[(cur.x, cur.y)]
            path.reverse(); return path, g_cost[(goal.x, goal.y)]
        for d in DIRECTIONS_TIEBREAK_ORDER[:-1]:
            nxt = cur.add(d)
            if not isSafe(nxt): continue
            key = (nxt.x, nxt.y)
            ng = g_cost[(cx, cy)] + moveCostOf(nxt)
            if key not in g_cost or ng < g_cost[key]:
                g_cost[key] = ng; came_from[key] = cur
                heapq.heappush(frontier, (ng + heuristic(nxt, goal), ng, DIR_ORDER_IDX[d], nxt.x, nxt.y))
    return None, 0

def locations_to_directions(src, waypoints):
    dirs = []; cur = src
    for nxt in waypoints:
        dirs.append(cur.direction_to(nxt)); cur = nxt
    return dirs

class Route:
    # [Caleb] compact carrier for navigation & sensing results
    def __init__(self, directions, cost, blocked, ambiguous, probe_loc):
        self.directions = directions
        self.cost = cost
        self.blocked = blocked
        self.ambiguous = ambiguous
        self.probe_loc = probe_loc

def plan_route(src, dst, blk):
    # [Caleb] compute path & flag if we need a scan to disambiguate rubble
    if isSame(src, dst): return Route([],0,False,False,None)
    path, cost = safe_astar(src, dst)
    if not path: return Route([],0,True,False,None)
    probe, amb = None, False
    for step in path:
        top = get_cell_info_at(step).top_layer
        if isinstance(top, Rubble):
            need = rubble_here_requires_pair(blk, step)
            if need is None: probe, amb = step, True; break
    return Route(locations_to_directions(src, path), cost, False, amb, probe)

# ------------------------------------------------
# [Caleb] Rubble knowledge cache
# ------------------------------------------------
class Blackboard: pass

BLACKBOARD = Blackboard()
BLACKBOARD.known_rubble = {}  # (x,y) -> {"agents_required": int, "energy_required": int|None}
BLACKBOARD.scanned = set()    # set[(x,y)]
BLACKBOARD.help_queue = []    # handshake FSM records

DEFAULT_DIG = 10
SAVE_COST = 1

def capture_rubble_knowledge(blk, loc: Location):
    # cache what we can see about rubble at 'loc'
    try:
        top = get_cell_info_at(loc).top_layer
        if isinstance(top, Rubble):
            try: er = top.energy_required
            except Exception: er = None
            blk.known_rubble[(loc.x, loc.y)] = {
                "agents_required": top.agents_required,
                "energy_required": er
            }
    except Exception:
        pass

def rubble_here_requires_pair(blk, loc: Location):
    # return True/False if known; None if we need a scan or adjacency reveal
    key = (loc.x, loc.y)
    info = blk.known_rubble.get(key)
    if isinstance(info, dict) and "agents_required" in info:
        return info["agents_required"] >= 2
    me = get_location()
    # adjacency or same-tile: we can inspect fully
    if max(abs(me.x - loc.x), abs(me.y - loc.y)) <= 1:
        top = get_cell_info_at(loc).top_layer
        if isinstance(top, Rubble):
            try: er = top.energy_required
            except Exception: er = None
            blk.known_rubble[key] = {"agents_required": top.agents_required, "energy_required": er}
            return top.agents_required >= 2
        return False
    return None

def scan_policy(block, route):
    # [Caleb] when path has ambiguous rubble ahead, scan if close & not already scanned
    if route.ambiguous and route.probe_loc is not None:
        me = get_location(); p = route.probe_loc
        dist = max(abs(me.x - p.x), abs(me.y - p.y))
        if dist <= 3 and (p.x, p.y) not in block.scanned:
            return route.probe_loc
    return None

# ------------------------------------------------
# [HeeYoung] Energy math (reach + dig + save)
# ------------------------------------------------
def estimate_goal_tail_energy(goal: Location) -> int:
    # energy to finish at the destination: dig (if rubble) + save
    top = get_cell_info_at(goal).top_layer
    tail = SAVE_COST
    if isinstance(top, Rubble):
        try: energy_req = top.energy_required
        except Exception: energy_req = DEFAULT_DIG
        tail += energy_req
    return tail

def energy_needed_to(start: Location, dest: Location) -> int:
    # full cost to move + (dig+save) at destination; 1e9 if no path
    path, _ = safe_astar(start, dest)
    if not path: return 10**9
    move_energy = sum(moveCostOf(p) for p in path)
    return move_energy + estimate_goal_tail_energy(path[-1])

def on_charger(loc: Location) -> bool:
    for c in get_charging_cells():
        if c.x == loc.x and c.y == loc.y: return True
    return False

# ------------------------------------------------
# [Connor] Pair-dig handshake (HELP_REQ/ACK/AT_LOC/READY_DIG)
# ------------------------------------------------
def read_inbox_prev_round():
    rn = get_round_number()
    if rn <= 1: return []
    prev = read_messages(rn - 1); out = []
    for m in prev:
        msg = m.message
        if "|" not in msg: continue
        parts = msg.split("|")
        out.append({"round_num": m.round_num, "sender_id": m.sender_id, "msg_type": parts[0], "parts": parts})
    return out

def handshake_for_dig(blk, rubble_loc: Location):
    rn = get_round_number(); me = get_id(); here = get_location()
    locs = f"{rubble_loc.x},{rubble_loc.y}"
    try: blk.help_queue
    except Exception: blk.help_queue = []
    record = None
    for r in blk.help_queue:
        if r.get("loc") == locs: record = r; break
    if record is not None and record.get("phase") == "pre_announce":
        try: blk.help_queue.remove(record)
        except Exception: pass
        record = None
    inbox = read_inbox_prev_round()
    if record is None:
        # become helper if someone else already asked
        req = None
        for m in inbox:
            if m["msg_type"] == "HELP_REQ" and len(m["parts"]) >= 3 and m["parts"][2] == locs and m["sender_id"] != me:
                req = m; break
        if req is not None:
            req_id = req["sender_id"]; send_message(f"ACK|{me}|{locs}", [req_id])
            blk.help_queue.append({"loc":locs,"req_id":req_id,"helper_id":me,"phase":"EN_ROUTE","dig_round":None})
            return None
        # otherwise, announce if we are standing on rubble
        if not (here.x == rubble_loc.x and here.y == rubble_loc.y): return None
        send_message(f"HELP_REQ|{me}|{locs}", [])
        blk.help_queue.append({"loc":locs,"req_id":me,"helper_id":None,"phase":"waiting_for_ack","dig_round":None})
        return None
    # lookup util
    def find_msg(t, from_id=None, min_parts=3):
        for m in inbox:
            if m["msg_type"]!=t: continue
            if len(m["parts"])>=min_parts and m["parts"][2]==locs and (from_id is None or m["sender_id"]==from_id):
                return m
        return None
    # requester side
    if record["req_id"] == me:
        if record["phase"] == "waiting_for_ack":
            ack = find_msg("ACK")
            if ack: record["helper_id"]=ack["sender_id"]; record["phase"]="waiting_at_loc"
            return None
        if record["phase"] == "waiting_at_loc":
            at_loc = find_msg("AT_LOC", from_id=record["helper_id"])
            if at_loc:
                dr = rn + 1; send_message(f"READY_DIG|{me}|{locs}|{dr}", [record["helper_id"]])
                record["dig_round"]=dr; record["phase"]="ready"; return dr
            return None
        if record["phase"] == "ready": return record.get("dig_round")
        return None
    # helper side
    if record["phase"] == "EN_ROUTE":
        if here.x==rubble_loc.x and here.y==rubble_loc.y:
            send_message(f"AT_LOC|{me}|{locs}", [record["req_id"]]); record["phase"]="waiting_until_ready"
        return None
    if record["phase"] == "waiting_until_ready":
        ready = find_msg("READY_DIG", from_id=record["req_id"], min_parts=4)
        if ready: record["dig_round"]=int(ready["parts"][3]); record["phase"]="ready"; return record["dig_round"]
        return None
    if record["phase"] == "ready": return record.get("dig_round")
    return None

# ------------------------------------------------
# [Alaik] Team state & assignments
# ------------------------------------------------
ASSIGNMENTS = {}   # agent_id -> (x,y) survivor target
LEADER_ID = None
TEAM_STATUS = {}   # agent_id -> (x,y)

def choose_leader():
    # lowest ID observed through STATUS beacons
    ids = [get_id()]
    for msg in read_messages():
        try: text = msg.message
        except Exception: text = ""
        if isinstance(text, str) and text.startswith("STATUS|"):
            parts = text.split("|")
            if len(parts) > 1 and parts[1].isdigit(): ids.append(int(parts[1]))
    return min(ids)

def broadcast_status():
    me = get_id(); loc = get_location()
    send_message(f"STATUS|{me}|{loc.x},{loc.y}", [])

def handle_messages():
    # ingest STATUS and ASSIGN broadcasts
    global ASSIGNMENTS, TEAM_STATUS
    for msg in read_messages():
        try: text = msg.message
        except Exception: continue
        if not isinstance(text, str): continue
        if text.startswith("STATUS|"):
            try:
                parts = text.split("|"); xy = parts[2].split(",")
                TEAM_STATUS[int(parts[1])] = (int(xy[0]), int(xy[1]))
            except Exception: pass
        elif text.startswith("ASSIGN|"):
            try:
                parts = text.split("|"); xy = parts[2].split(",")
                ASSIGNMENTS[int(parts[1])] = (int(xy[0]), int(xy[1]))
            except Exception: pass

def broadcast_assignments():
    for aid, tgt in ASSIGNMENTS.items():
        if tgt is not None:
            send_message(f"ASSIGN|{aid}|{tgt[0]},{tgt[1]}", [])

def nearest_survivor(my_loc, survivors):
    if not survivors: return None
    return min(survivors, key=lambda s: abs(my_loc.x - s.x) + abs(my_loc.y - s.y))

def choose_charger(my_loc):
    cs = get_charging_cells()
    if not cs: return None
    return min(cs, key=lambda c: abs(c.x - my_loc.x) + abs(c.y - my_loc.y))

# ------------------------------------------------
# [Connor/Alaik] Help Dispatcher — adopt HELP_REQs
# ------------------------------------------------
HELP_TASK = None  # (x, y, requester_id)

def check_help_requests():
    """Adopt a HELP_REQ from last round if free."""
    global HELP_TASK
    inbox = read_inbox_prev_round()
    me = get_id()
    if HELP_TASK is not None:
        x, y, _ = HELP_TASK
        # drop task if rubble disappeared
        top = get_cell_info_at(Location(x, y)).top_layer
        if not isinstance(top, Rubble):
            HELP_TASK = None
        return
    for m in inbox:
        if m["msg_type"] == "HELP_REQ" and len(m["parts"]) >= 3 and m["sender_id"] != me:
            try:
                xs, ys = m["parts"][2].split(",")
                HELP_TASK = (int(xs), int(ys), m["sender_id"])
                return
            except Exception:
                continue

def clear_help_task_if_done():
    """Clear HELP_TASK once rubble is gone."""
    global HELP_TASK
    if HELP_TASK is None: return
    x, y, _ = HELP_TASK
    top = get_cell_info_at(Location(x, y)).top_layer
    if not isinstance(top, Rubble):
        HELP_TASK = None

# ------------------------------------------------
# MAIN AGENT LOOP
# ------------------------------------------------
def think():
    # broadcast & ingest team beacons every tick
    broadcast_status()
    handle_messages()

    me = get_id()
    my_loc = get_location()
    my_en  = get_energy_level()

    # leader selection (lowest ID)
    global LEADER_ID
    if LEADER_ID is None:
        LEADER_ID = choose_leader()

    # [Alaik] Leader assigns survivors in simple round-robin
    if me == LEADER_ID:
        survs = list(get_survs())
        if survs:
            agents = sorted(list(TEAM_STATUS.keys()) or [me])
            survs_sorted = sorted(survs, key=lambda s: (s.x, s.y))
            for i, aid in enumerate(agents):
                s = survs_sorted[i % len(survs_sorted)]
                ASSIGNMENTS[aid] = (s.x, s.y)
        broadcast_assignments()

    # watch for help requests / release finished tasks
    check_help_requests()
    clear_help_task_if_done()

    # If on survivor: save immediately
    top_here = get_cell_info_at(my_loc).top_layer
    if isinstance(top_here, Survivor):
        save()
        send_message(f"SAVED|{me}|{my_loc.x},{my_loc.y}", [])
        return

    # -------------------- DIG at current tile --------------------
    if isinstance(top_here, Rubble):
        # learn exact requirements
        capture_rubble_knowledge(BLACKBOARD, my_loc)
        info = BLACKBOARD.known_rubble.get((my_loc.x, my_loc.y), {})
        agents_required = info.get("agents_required", 1)
        try:
            energy_required = info.get("energy_required", top_here.energy_required)
        except Exception:
            energy_required = DEFAULT_DIG

        # If enough agents already on the tile → dig now (no handshake)
        agents_here = len(get_cell_info_at(my_loc).agents)
        if agents_required <= agents_here and my_en >= energy_required:
            dig(); return

        # Single-agent rubble → dig when we have exactly-enough energy (no +1 margin)
        if agents_required <= 1:
            if my_en >= energy_required:
                dig(); return
            # need energy → head to charger
            ch = choose_charger(my_loc)
            if ch:
                r = plan_route(my_loc, ch, BLACKBOARD)
                if r.directions: move(r.directions[0]); return
            move(Direction.CENTER); return

        # Multi-agent rubble:
        # ensure we have our own energy to contribute
        if my_en < energy_required:
            ch = choose_charger(my_loc)
            if ch:
                r = plan_route(my_loc, ch, BLACKBOARD)
                if r.directions: move(r.directions[0]); return
            move(Direction.CENTER); return

        # run handshake until synchronized, then dig on scheduled round
        dr = handshake_for_dig(BLACKBOARD, my_loc)
        if dr is not None and dr == get_round_number():
            dig(); return
        move(Direction.CENTER); return
    # ------------------ end DIG at current tile ------------------

    # ------------------ Decide target (help overrides) -----------
    helper_goal = None
    if HELP_TASK is not None:
        hx, hy, _ = HELP_TASK
        helper_goal = Location(hx, hy)

    if helper_goal is not None:
        # keep handshake alive en-route so ACK/AT_LOC flow progresses
        _ = handshake_for_dig(BLACKBOARD, helper_goal)
        # charge iff energy insufficient to reach help tile + dig there
        need_energy = energy_needed_to(my_loc, helper_goal)
        if on_charger(my_loc):
            if need_energy < 10**9 and my_en < need_energy:
                recharge(); return
        elif need_energy < 10**9 and my_en < need_energy:
            ch = choose_charger(my_loc)
            if ch:
                r = plan_route(my_loc, ch, BLACKBOARD)
                if r.directions: move(r.directions[0]); return
            move(Direction.CENTER); return
        # move toward help tile (scan if ambiguous)
        r = plan_route(my_loc, helper_goal, BLACKBOARD)
        probe = scan_policy(BLACKBOARD, r)
        if probe is not None:
            drone_scan(probe); BLACKBOARD.scanned.add((probe.x, probe.y)); capture_rubble_knowledge(BLACKBOARD, probe); return
        if r.directions:
            move(r.directions[0]); return
        move(Direction.CENTER); return

    # else go to assigned survivor
    if me not in ASSIGNMENTS or ASSIGNMENTS[me] is None:
        move(Direction.CENTER); return

    goal_xy = ASSIGNMENTS[me]
    goal = Location(goal_xy[0], goal_xy[1])

    # [HeeYoung] Simple charging rule: ONLY charge if energy < need(goal)
    need_energy = energy_needed_to(my_loc, goal)

    if on_charger(my_loc):
        if need_energy >= 10**9:
            # no path from here → don't sit forever
            move(Direction.CENTER); return
        if my_en < need_energy:
            recharge(); return
        # enough → leave immediately toward goal
        route = plan_route(my_loc, goal, BLACKBOARD)
        if route.directions: move(route.directions[0])
        else: move(Direction.CENTER)
        return

    if need_energy < 10**9 and my_en < need_energy:
        ch = choose_charger(my_loc)
        if ch:
            r = plan_route(my_loc, ch, BLACKBOARD)
            probe = scan_policy(BLACKBOARD, r)
            if probe is not None:
                drone_scan(probe); BLACKBOARD.scanned.add((probe.x, probe.y)); capture_rubble_knowledge(BLACKBOARD, probe); return
            if r.directions: move(r.directions[0]); return
        # no reachable charger: wait a tick rather than thrash
        move(Direction.CENTER); return

    # move toward survivor (step onto rubble to reveal; dig next tick)
    route = plan_route(my_loc, goal, BLACKBOARD)
    probe = scan_policy(BLACKBOARD, route)
    if probe is not None:
        drone_scan(probe); BLACKBOARD.scanned.add((probe.x, probe.y)); capture_rubble_knowledge(BLACKBOARD, probe); return
    if route.directions:
        nxt = my_loc.add(route.directions[0])
        if isinstance(get_cell_info_at(nxt).top_layer, Rubble):
            move(route.directions[0]); return
        move(route.directions[0]); return

    # nothing better to do this tick
    move(Direction.CENTER)
