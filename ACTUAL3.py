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
