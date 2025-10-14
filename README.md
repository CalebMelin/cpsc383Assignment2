# cpsc383Assignment2


Requirements:
- Agents need to navigate around the map avoiding death squares and illegal moves
- Agents need to know how and when to go get energy
- Agents need to be able to communicate with each other to ask for help
- Agents need to know when to drone scan

Assumptions:
- If there are multiple survivors on one square, then we can assume that we will need more than one agent to help clear the rubble on the path

What we Know:
- Locations of agents
- Locations of charging squares
- Locations of survivors

Decision Making flow:
- How can we get them to split up??
    - The first initialized agent should be the leader - lowest ID will be the leader
        - This allows us to prioritize who needs help first
        - Each agent will have statuses and will communicate their status, target, target agent, command, priority of command
- Agent priority will be as follows:
    1. Ask for Help/Deligate
        - Only case is when they need assistance to dig
        - Will assign a priority to their help call
            1. If it is the path to the energy
            2. Path to a survivor
    2. Energy 
        1. When they will use up all their energy to get to the energy square
        2. When distance to agent + distance back to energy is greater than their current energy
        - Agents will determine whether to recharge more than once based on this same statistic
    3. Pathing to current prioritized survivor

    4. Drone scan?

Status:
- For communicating with the other agents the current motive or objective
- Statuses:
    - Waiting (for help to dig)
    - Moving to help
    - Moving to charge
    - Moving to survivor
    - Waiting to help

Target:
- For communicating what survivor they are targeting or what agent they are trying to help
    - Ex.
        - Helping 1010001
        - Targeting surv (surv id)


Target Agent, Command, Priority of Command
- All for asking for help from other agents
- Target Agent
    - Ex.
        - 10010011
- Command
    - Come to (location)
- Priority of command
    1. Get to charge station
    2. Save Survivor
    3. Clear rubble on path to survivor


At the start determine what surv to target
- Each agent is assigned a survuivor to target
    - This is easy if they all spawn on the same square, because they will just be targeted to the closest survivors
    - If they are not on the same square, we need a program to assign each agent to the closest survivors

After goal is accomplished determine what surv to target
- If a survivor exists that is not being targeted to be saved by another agent, this is now the target of this agent
- Otherwise, agent goes to charge square and waits to help

Sacrificing for the greater good
- An agent should theoretically be able to run out of energy to save the survivor if it knows it is not needed anymore
- Need to figure out logic for this