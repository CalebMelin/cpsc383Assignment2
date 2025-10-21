# cpsc383Assignment2

When opening the repo from this git, you will not have the .exe immediately
To fix this you must open a python 13 venv then run the command "aegis init --type mas" in your terminal.


Requirements:

* Agents need to navigate around the map avoiding death squares and illegal moves
* Agents need to know how and when to go get energy
* Agents need to be able to communicate with each other to ask for help
* Agents need to know when to drone scan

Assumptions:

* If there are multiple survivors on one square, then we can assume that we will need more than one agent to help clear the rubble on the path
  (Refined: the number of survivors doesn’t guarantee multi-agent rubble; agents confirm rubble requirements via adjacency or drone scan and then coordinate.)

What we Know:

* Locations of agents
* Locations of charging squares
* Locations of survivors

Decision Making flow:

* How can we get them to split up??

  * The first initialized agent should be the leader - lowest ID will be the leader

    * This allows us to prioritize who needs help first
    * Each agent will have statuses and will communicate their status, target, target agent, command, priority of command
* Agent priority will be as follows:

  1. Ask for Help/Deligate

     * Only case is when they need assistance to dig
     * Will assign a priority to their help call

       1. If it is the path to the energy
       2. Path to a survivor
     * Simultaneous dig is required when two or more agents are needed, with one-round lag expected; messages include round tags and ACKs.
  2. Energy

     1. When they will use up all their energy to get to the energy square
     2. When distance to agent + distance back to energy is greater than their current energy

     * Agents will determine whether to recharge more than once based on this same statistic
     * Energy math includes: path to goal + (possible) drone scan + (possible) multi-round dig + save + path to next target or charger.
  3. Pathing to current prioritized survivor
  4. Drone scan?

     * Trigger: planned path is blocked/ambiguous by top-layer rubble or unknown lower layers; scan the critical location to confirm rubble requirements and alternatives.

Status:

* For communicating with the other agents the current motive or objective
* Statuses:

  * Waiting (for help to dig)
  * Moving to help
  * Moving to charge
  * Moving to survivor
  * Waiting to help

Target:

* For communicating what survivor they are targeting or what agent they are trying to help

  * Ex.

    * Helping 1010001
    * Targeting surv (surv id)

Target Agent, Command, Priority of Command:

* All for asking for help from other agents
* Target Agent

  * Ex.

    * 10010011
* Command

  * Come to (location)
* Priority of command

  1. Get to charge station
  2. Save Survivor
  3. Clear rubble on path to survivor

At the start determine what surv to target:

* Each agent is assigned a survuivor to target

  * This is easy if they all spawn on the same square, because they will just be targeted to the closest survivors
  * If they are not on the same square, we need a program to assign each agent to the closest survivors
  * Leader is the lowest ID always and can break ties or reassign on the fly.

After goal is accomplished determine what surv to target:

* If a survivor exists that is not being targeted to be saved by another agent, this is now the target of this agent
* Otherwise, agent goes to charge square and waits to help

Sacrificing for the greater good:

* An agent should theoretically be able to run out of energy to save the survivor if it knows it is not needed anymore
* Need to figure out logic for this

  * Refined: only allow “sacrifice” when (a) SAVE this round is guaranteed and (b) no future tasks require that agent and (c) team has enough agents/energy to finish remaining survivors; leader confirms via message ACK to avoid double sacrifice.

Messaging details (fits your Status/Target/Command/Priority):

* Include: round_num, message_type (HELP_REQ, ACK, EN_ROUTE, AT_LOC, READY_DIG, ABORT, STATUS), from_id, loc, target, target_agent, command, priority_of_command.
* One-round lag is expected; agents treat messages as stale if round_num < current_round - 1.

Two-agent simultaneous dig handshake (accounts for one-round lag):

1. Leader at rubble sends HELP_REQ with loc and round_num.
2. Helper replies ACK and moves; upon arrival sends AT_LOC with round_arrived.
3. Leader responds next round with READY_DIG announcing dig_round = current_round + 1.
4. Both call dig() on dig_round. If anything slips (energy shortfall, displacement), leader sends ABORT and restarts the handshake.

# FOUR-PERSON DIVISION WITH METHOD CONTRACTS AND HAND-OFFS

Shared “Blackboard” (team state) per tick:
Blackboard {
my_id, my_loc, my_energy, round_num,
leader_id, chargers[], survivors[], known_rubble: {loc -> {agents_required, energy_required?}},
assignments: {agent_id -> target_surv_id | help_loc | idle},
statuses: {agent_id -> status_string},
help_queue: [help_request...],
messages_out: [msg...], messages_in: [msg...]
}

## Person 1 — Comms & Coordination

Focus: messaging schema, leader delegation, two-agent dig handshake, status beacons.

Methods:

* encode_status(blackboard) -> dict
  Output: message dict including status, target, target_agent, command, priority_of_command, round_num.
  Sends to: everyone (broadcast).

* broadcast(msg: dict) -> None
  Uses send_message(str(json), []).

* read_inbox() -> list[dict]
  Wrapper over read_messages() that parses and filters stale messages (based on round_num).

* handshake_for_dig(blk: Blackboard, rubble_loc: Location) -> Optional[int]
  Input: blackboard, rubble location.
  Output: dig_round if synchronized; None if still negotiating.
  Emits messages: HELP_REQ, ACK, EN_ROUTE, AT_LOC, READY_DIG, ABORT as needed.
  Depends on: Person 4 for path_to(rubble_loc) readiness; Person 3 for energy feasibility flags.

Consumes from Person 2:

* assignments (who is helping whom / which survivor)
* priority_of_command for help requests

Produces to Person 2 and Person 4:

* Confirmed dig_round or ABORT
* Updated help_queue order after ACKs
* Recent statuses for global planning

## Person 2 — Planner & “How can we get them to split up??”

Focus: initial assignment, reassign on events (save/dig/charge), permanent lowest-ID leader policy, decision making flow order.

Methods:

* assign_initial_targets(blk) -> dict[agent_id, target]
  Output: assignments mapping; nearest survivor or help.
  Uses: get_survs(), agents’ spawn positions, leader tie-break (lowest ID).

* replan_after_event(blk, event: str) -> dict[agent_id, target]
  Events: SAVED, RUBBLE_SCANNED, HELP_REQ_ADDED, HELP_FULFILLED, LOW_ENERGY.
  Output: updated assignments.

* choose_action_priority(blk, agent_id) -> list[str]
  Output: ordered priorities [Ask for Help/Deligate, Energy, Pathing to current prioritized survivor, Drone scan?] customized by current context.

* select_help_targets(blk) -> list[HelpRequest]
  Output: help tasks with (loc, priority_of_command, requesting_agent_id).

Consumes from Person 3:

* Energy feasibility: can_reach_without_charge, projected costs
* Suggested charger choice and recharge ticks if needed

Consumes from Person 4:

* Path existence/length estimates, rubble ambiguity flags that trigger “Drone scan?”

Produces to Person 1:

* help_queue (who to ping), priority_of_command, and target locations for messages

Produces to Person 4:

* Per-agent intent (Moving to survivor / Moving to help / Moving to charge / Waiting (for help to dig) / Waiting to help) and target (surv id or help loc)

## Person 3 — Energy & Charging

Focus: thresholds and math for “Energy” step; when to charge; how long to sit and recharge; sacrifice logic guardrails.

Methods:

* project_action_costs(path_len: int, actions: list[str]) -> int
  Output: total projected energy cost.

* needs_charge(now_energy: int, projected_cost: int) -> bool
  Output: True if energy insufficient according to your two rules and including dig/save/next-step costs.

* choose_charger(blk, agent_loc) -> Location
  Output: chosen charging square.

* recharge_policy(now_energy: int, target_cost: int) -> int
  Output: number of recharge() ticks to wait (could be 0+).

* allow_sacrifice(blk) -> bool
  Output: True only if SAVE is guaranteed this round and team can still complete remaining tasks (leader confirmation expected).

Consumes from Person 4:

* estimate_path_cost(src, dst) -> int (or at least path length)
* Flags for “blocked/needs scan” so charging isn’t chosen due to phantom blocks

Produces to Person 2:

* can_reach_without_charge, recommended charger, and recharge_ticks
* Warning flags: LOW_ENERGY, SACRIFICE_ALLOWED

Produces to Person 4:

* “You have N move budget before mandatory detour to charge.”

## Person 4 — Navigation & Sensing (Pathing + Drone scan?)

Focus: safe neighbor expansion (avoid death squares), routing to targets, and when to drone scan to resolve uncertainty about rubble.

Methods:

* plan_route(src: Location, dst: Location, blk) -> Route
  Output: Route {directions: [Direction], cost: int, blocked: bool, ambiguous: bool, probe_loc?: Location}
  Note: ambiguous=True when top-layer indicates rubble/unknown lower layers and we need info before committing; sets probe_loc.

* step_toward(route) -> Optional[Direction]
  Output: next move or None if at destination.

* estimate_path_cost(src, dst) -> int
  Output: move cost for Person 3’s math.

* scan_policy(blk, route) -> Optional[Location]
  Output: a Location to drone_scan() when ambiguous=True or when cheaper than detouring.

* rubble_here_requires_pair(blk, loc) -> Optional[bool]
  Output: True/False if known; None if unknown (triggers scan or adjacency reveal).

Consumes from Person 2:

* The current intent and target (survivor/help/charger)

Produces to Person 1:

* “Ready for dig” signal when at rubble location; provides loc and arrival round for handshake
* “ABORT dig” trigger if route changed or energy became insufficient

Produces to Person 2:

* blocked/ambiguous flags to elevate “Drone scan?” priority
* Updated estimates that might flip assignments (e.g., closer helper)

Produces to Person 3:

* estimate_path_cost and probe_loc (if extra scans are expected)

# GLUE ORDER (ROUND LOOP)

Per agent each round:

1. Person 1: read_inbox() → update blackboard.messages_in, statuses, help_queue
2. Person 2: assign_initial_targets (once) or replan_after_event → set assignments, intent, target, priority_of_command
3. Person 4: plan_route → if ambiguous, Person 2 upgrades “Drone scan?”; Person 4 may suggest probe_loc
4. Person 3: project_action_costs & needs_charge → if True, Person 2 sets intent=Moving to charge and Person 3 gives recharge_policy
5. If at rubble and pair needed: Person 1 runs handshake_for_dig → either schedules dig_round or keeps waiting
6. Execute the single action for this round: move | save | dig | recharge | drone_scan
7. Person 1: encode_status and broadcast (free message alongside the action)

# EXPLICIT HAND-OFFS (WHO EXPECTS WHAT FROM WHOM)

* Person 1 expects from Person 2:
  priority_of_command, assignments, and the target_agent for help flows.
  Person 1 outputs to Person 2:
  confirmed dig_round or ABORT, plus fresh statuses.

* Person 2 expects from Person 3:
  can_reach_without_charge, recharge_ticks, allow_sacrifice.
  Person 2 outputs to Person 3:
  selected intent and target so energy math is contextual.

* Person 3 expects from Person 4:
  estimate_path_cost(src,dst) and any probe_loc that might add costs (scan).
  Person 3 outputs to Person 4:
  a move-budget hint (“you have N moves before mandatory charge”).

* Person 4 expects from Person 2:
  intent and target.
  Person 4 outputs to Person 1:
  AT_LOC readiness details for the simultaneous dig handshake.
  Person 4 outputs to Person 2/3:
  blocked/ambiguous flags and refined costs that can trigger re-planning or charging.
