from pickle import TRUE
import websocket as ws
import numpy as np
import matplotlib.pyplot as plt
import struct
import time
import string
import math
from cmath import nan
import asyncio
import websockets

# --- Globals for dynamic slot/angle replacement ---
last_slot_used = None
last_action_used = None



##### CI INITIALIZATION VALUE ######
#[ 1.181  0.781 -1.618  0.726 -1.934] f400 
#[ 0.73   0.318 -2.637  0.192 -2.368] f200
########################
###Robot Definitions ###
########################
#go up and rotate 
# forbid opoposite placement 
# try to have intermediate pos lower or no intermediate at all 
# begin to look for all other placing pos
# reflechir a tout les autres movements


cont = True

# Global tracker for last commanded joint angles
last_commanded_angles = None

ids = [20,21,22,23,24] #i2c addresses for actuators

fids = [25,26] #i2c addresses for gripper feet

idsBytes = b""
for i in ids:
    idsBytes = idsBytes + struct.pack("!B", int(i))

position0 = np.array([0.5, 3.25, 3.62, 5.056, 2.05])#np.array([0.5, 3.25, 3.67, 5.056, 2.05])

offset = position0

addr = "ws://192.168.4.30" # ipconfig getifaddr en0 to get current ip Autodesk 192.168.4.30
# addr = "ws://192.168.1.109" 
transmit = True # should we send commands over wifi or no. 

global LAST_M20_ANGLE_RAW

# Launch websocket server to listen to simulator
if transmit == True:
    esp32 = ws.WebSocket()
    delay = 0.1
    esp32.connect(addr)

def generate_movement_sequence(relative_pos, front_delta, back_delta, switch, carried_voxel):
    """
    Compute sub-poses to move one leg followed by the other using a generic rule.
    """

    def pose_from_vector(leg, vec, carried_voxel, is_p=False, switch=False):
        x, y, z = map(int, vec)

        def format_dim(val):
            return f"m{abs(val)}" if val < 0 else str(val)

        def carrying(val):
            return "carrying" if (val > 0 and leg == "b") else ""
        
        # For Y only, apply mirroring logic if switch == True
        y_switched = -y if switch else y

        return f"C:M:{carrying(carried_voxel)}{leg}{format_dim(x)}{format_dim(y_switched)}{format_dim(z)}{'p' if is_p else ''}"

    def interpolate_z(start, end, leg):
        path = []
        z0 = int(start[2])
        z1 = int(end[2])
        if z0 == z1:
            # print('start/end', start, end)
            mid_vect3 = None
            if leg == "b" and start[0] == 4 and end[0] == 2 and end[1] == 0 and end[2] != 0:
                mid_vect3 = (3, 0, z0 + 1)
            if start[0] == 2 and end[0] == 4 and end[1] == 0:
                mid_vect3 = (3, 0, z0 + 1)

            mid_vec1 = (start[0], start[1], z0 + 1)
            mid_vec2 = (end[0], end[1], z1 + 1)

            path.append(pose_from_vector(leg, mid_vec1, carried_voxel, True, switch))
            path.append("C:M:DMcarried" if leg == "b" and carried_voxel > 0 else "C:M:DM")

            if mid_vect3 is not None:
                path.append(pose_from_vector(leg, mid_vect3, carried_voxel, True, switch))
                path.append("C:M:DMcarried" if leg == "b" and carried_voxel > 0 else "C:M:DM")

            path.append(pose_from_vector(leg, mid_vec2, carried_voxel, True, switch))
            path.append("C:M:DMcarried" if leg == "b" and carried_voxel > 0 else "C:M:DM")
        else:
            if z1 > z0:
                for z in range(z0 + 1, z1 + 2):
                    # print(f"🦿a Interpolating Z from {z0} to {z1+1} for leg {leg}")
                    mid_vec = (start[0], start[1], z)
                    path.append(pose_from_vector(leg, mid_vec, carried_voxel, True, switch))
                    path.append("C:M:DMcarried" if leg == "b" and carried_voxel > 0 else "C:M:DM")
                mid_vec2 = (end[0], end[1], z1 + 1)
                path.append(pose_from_vector(leg, mid_vec2, carried_voxel, True, switch))
                path.append("C:M:DMcarried" if leg == "b" and carried_voxel > 0 else "C:M:DM")
            else:
                # print(f" aadzad za Interpolating Z down from {z0 + 1} to {z1} for leg {leg}")
                mid_vec2 = (start[0], start[1], z0 + 1)
                path.append(pose_from_vector(leg, mid_vec2, carried_voxel, True, switch))
                path.append("C:M:DMcarried" if leg == "b" and carried_voxel > 0 else "C:M:DM")
                for z in range(z0 + 1, z1, -1):
                    mid_vec = (end[0], end[1], z)
                    path.append(pose_from_vector(leg, mid_vec, carried_voxel, True, switch))
                    path.append("C:M:DMcarried" if leg == "b" and carried_voxel > 0 else "C:M:DM")
        print("path", path)
        return path
    
    sequence = []

    # Determine first and second leg
    first_leg = "b" if switch else "f"
    second_leg = "f" if switch else "b"
    first_delta = back_delta if switch else front_delta
    second_delta = front_delta if switch else back_delta

    # Prepare gripper commands leg1 
    sequence.append(f"C:M:C{2 if first_leg == 'b' else 1}")
    sequence.append("C:M:DF")
    sequence.append(f"C:M:O{1 if first_leg == 'b' else 2}")
    sequence.append("C:M:DF")

    # Start and end position of first leg
    base_x = abs(relative_pos[0]) + abs(relative_pos[1])
    # print(f"🦿 Relative X: {relative_pos}")
    startPos = (int(2 * base_x), 0, int(2 * relative_pos[2]))
    endPos = tuple(startPos[i] + int(2 * first_delta[i]) for i in range(3))
    # print(f"🦿 Start Position: {startPos}, End Position: {endPos}")
    # First leg movement
    sequence.extend(interpolate_z(startPos, endPos, first_leg))
    sequence.append(pose_from_vector(first_leg, endPos, carried_voxel, False, switch))
    sequence.append("C:M:DM") 

    # Prepare gripper commands leg2 
    sequence.append(f"C:M:C{2 if second_leg == 'b' else 1}")
    sequence.append("C:M:DF")
    sequence.append(f"C:M:O{1 if second_leg == 'b' else 2}")
    sequence.append("C:M:DF")
    # Start and end position of second leg
    startPos2 = (endPos[0], endPos[1], -endPos[2])
    endPos2 =((startPos2[0] - int(2 * second_delta[0]), 
               startPos2[1] - int(2 * second_delta[1]), 
               startPos2[2] + int(2 * second_delta[2])))
    # print(f"afaf Relative stat: {startPos2}")
    # print(f"fafaf Relative end: {endPos2}")
    if endPos2[0] == 0:
        endPos2 = (abs(endPos2[1]), 0, endPos2[2])
    # print(f"🦿 Start Position 2: {startPos2}, End Position 2: {endPos2}")
    # Second leg movement
    sequence.extend(interpolate_z(startPos2, endPos2, second_leg))
    sequence.append(pose_from_vector(second_leg, endPos2, carried_voxel, False, switch))
    sequence.append("C:M:DM") 

    return sequence

def getslotAngle(action, slot, M20value):
    global LAST_M20_ANGLE_RAW
    reverse = action.startswith("placing_r")
    # Remove "placing_" or "placing_r_" prefix for parsing
    if action.startswith("placing_r_"):
        base_action = action[len("placing_r_"):]
    elif action.startswith("placing_"):
        base_action = action[len("placing_"):]
    else:
        base_action = action  
    parts = base_action.split("_")

    if len(parts) < 2:
        print("⚠️ Unrecognized format:", action)
        return 0

    first_arg = parts[0]
    angle = 0
    if first_arg.startswith("3"):
        # Center-based placing logic
        if slot == "backward":
            angle = 0
        elif slot == "left":
            angle = ((math.pi / 2)-0.1)
        elif slot == "right":
            angle = -math.pi / 2
    else:
        if first_arg.endswith("l"):
            if slot == "right":
                angle = 0
            elif slot == "backward":
                angle = math.pi / 2
            elif slot == "left":
                angle = math.pi
        elif first_arg.endswith("r"):
            if slot == "left":
                angle = 0
            elif slot == "backward":
                angle = -math.pi / 2
            elif slot == "right":
                angle = -math.pi
    
    if(not reverse):
        updatedValue = round(M20value + angle, 3)
        LAST_M20_ANGLE_RAW = updatedValue
    else: 
        updatedValue = round(M20value - angle, 3)
    print('angle',angle)
    print('M20value',M20value)
    print("update Value", updatedValue)

    return updatedValue 

def generate_placing_sequence(action, slot=None):
    sequence_map = {
        "placing_1l_1_0": "C:M:O1,C:M:DF,C:M:C2,C:M:DF,C:M:b201place,C:M:DM,b201placeUp,C:M:DM,C:M:b021place,C:M:DM,M:20:SLOT,C:M:DM,C:M:b020place,M:20:{LAST_M20_ANGLE_RAW},C:M:DMplacing,C:M:OpenSLOT,C:M:DFplacing",
        "placing_3r_0": "C:M:O1,C:M:DF,C:M:C2,C:M:DF,C:M:b201place,C:M:DM,C:M:b021place,C:M:DM,M:20:SLOT,C:M:DM,C:M:b020place,M:20:{LAST_M20_ANGLE_RAW},C:M:DMplacing,C:M:OpenSLOT,C:M:DFplacing",
        "placing_1r_1_0": "C:M:O1,C:M:DF,C:M:C2,C:M:DF,C:M:b201place,C:M:DM,b201placeUp,C:M:DM,C:M:b0m21place,C:M:DM,M:20:SLOT,C:M:DM,C:M:b0m20place,M:20:{LAST_M20_ANGLE_RAW},C:M:DM,C:M:b0m20,M:20:{LAST_M20_ANGLE_RAW},C:M:DMplacing,C:M:OpenSLOT,C:M:DFplacing",
        "placing_3l_0": "C:M:O1,C:M:DF,C:M:C2,C:M:DF,C:M:b201place,C:M:DM,C:M:b0m21place,C:M:DM,M:20:SLOT,C:M:DM,C:M:b0m20place,M:20:{LAST_M20_ANGLE_RAW},C:M:DM,C:M:b0m20,M:20:{LAST_M20_ANGLE_RAW},C:M:DMplacing,C:M:OpenSLOT,C:M:DFplacing",
        "placing_1r_2_0": "C:M:O1,C:M:DF,C:M:C2,C:M:DF,C:M:b201place,C:M:DM,C:M:b201placeUp,C:M:DM,C:M:bm202rplace,C:M:DM,C:M:bm201rplace,C:M:DM,M:20:SLOT,C:M:DM,C:M:bm200r,M:20:{LAST_M20_ANGLE_RAW},C:M:DMplacing,C:M:OpenSLOT,C:M:DFplacing",
        "placing_3c_0": "C:M:O1,C:M:DF,C:M:C2,C:M:DF,C:M:b201place,C:M:DM,C:M:b201placeUp,C:M:DM,C:M:bm202rplace,C:M:DM,C:M:bm201cplace,C:M:DM,M:20:SLOT,C:M:DM,C:M:bm200r,M:20:{LAST_M20_ANGLE_RAW},C:M:DMplacing,C:M:OpenSLOT,C:M:DFplacing",
        "placing_1l_2_0": "C:M:O1,C:M:DF,C:M:C2,C:M:DF,C:M:b201place,C:M:DM,C:M:bm201lplace,C:M:DM,M:20:SLOT,C:M:DM,C:M:bm200l,M:20:{LAST_M20_ANGLE_RAW},C:M:DMplacing,C:M:OpenSLOT,C:M:DFplacing",
        "placing_2l_0": "C:M:O1,C:M:DF,C:M:C2,C:M:DF,C:M:b201place,C:M:DM,C:M:b201placeUp,C:M:DM,C:M:b021place,C:M:DM,C:M:bm221place,C:M:DM,M:20:SLOT,C:M:DM,C:M:bm220,M:20:{LAST_M20_ANGLE_RAW},C:M:DMplacing,C:M:OpenSLOT,C:M:DFplacing",
        "placing_2r_0": "C:M:O1,C:M:DF,C:M:C2,C:M:DF,C:M:b201place,C:M:DM,C:M:b201placeUp,C:M:DM,C:M:b0m21place,C:M:DM,C:M:bm2m21place,C:M:DM,M:20:SLOT,C:M:DM,C:M:bm2m20,M:20:{LAST_M20_ANGLE_RAW},C:M:DMplacing,C:M:OpenSLOT,C:M:DFplacing",
        "placing_4l_0": "C:M:O1,C:M:DF,C:M:C2,C:M:DF,C:M:b201place,C:M:DM,C:M:b2m21placeI,C:M:DM,C:M:b2m21place,C:M:DM,M:20:SLOT,C:M:DM,C:M:b2m20place,M:20:{LAST_M20_ANGLE_RAW},C:M:DMplacing,C:M:OpenSLOT,C:M:DFplacing",
        "placing_4r_0": "C:M:O1,C:M:DF,C:M:C2,C:M:DF,C:M:b201place,C:M:DM,C:M:b221placeI,C:M:DM,C:M:b221place,C:M:DM,M:20:SLOT,C:M:DM,C:M:b220place,M:20:{LAST_M20_ANGLE_RAW},C:M:DMplacing,C:M:OpenSLOT,C:M:DFplacing",
        "placing_3c_1": "C:M:O1,C:M:DF,C:M:C2,C:M:DF,C:M:b201place,C:M:DM,C:M:b202place,C:M:DM,M:20:SLOT,C:M:DF,C:M:b203place,C:M:DM,C:M:bm203rplace,C:M:DM,C:M:bm202r,M:20:{LAST_M20_ANGLE_RAW},C:M:DMplacing,C:M:OpenSLOT,C:M:DFplacing",
        "placing_3c_2": "C:M:O1,C:M:DF,C:M:C2,C:M:DF,C:M:b20m1place,C:M:DF,C:M:b200place,C:M:DF,M:20:SLOT,C:M:DF,C:M:b201place,C:M:DM,C:M:b202place,C:M:DM,C:M:b203place,C:M:DM,C:M:bm203rplace,C:M:DM,C:M:bm202r,M:20:{LAST_M20_ANGLE_RAW},C:M:DMplacing,C:M:OpenSLOT,C:M:DFplacing",
        
        "placing_r_1l_1_0": "C:M:b021place,M:20:{LAST_M20_ANGLE_RAW},C:M:DMplacing,M:20:SLOT,C:M:DM,C:M:b201placeUp,C:M:DM,C:M:b201place,C:M:DM,C:M:b200,C:M:DM",
        "placing_r_3r_0": "C:M:b021place,M:20:{LAST_M20_ANGLE_RAW},C:M:DMplacing,M:20:SLOT,C:M:DM,C:M:b201place,C:M:DM,C:M:b200,C:M:DM",
        "placing_r_1r_1_0": "C:M:b0m21place,M:20:{LAST_M20_ANGLE_RAW},C:M:DMplacing,M:20:SLOT,C:M:DM,C:M:b201placeUp,C:M:DM,C:M:b201place,C:M:DM,C:M:b200,C:M:DM",
        "placing_r_3l_0": "C:M:b0m21place,M:20:{LAST_M20_ANGLE_RAW},C:M:DMplacing,C:M:b0m22place,M:20:{LAST_M20_ANGLE_RAW},C:M:DMplacing,M:20:SLOT,C:M:DM,C:M:b201place,C:M:DM,C:M:b200,C:M:DM",
        "placing_r_1r_2_0": "C:M:bm201rplace,M:20:{LAST_M20_ANGLE_RAW},C:M:DMplacing,M:20:SLOT,C:M:DM,C:M:bm202rplace,C:M:DM,C:M:b201placeUp,C:M:DM,C:M:b201place,C:M:DM,C:M:b200,C:M:DM",
        "placing_r_3c_0": "C:M:bm201rplace,M:20:{LAST_M20_ANGLE_RAW},C:M:DMplacing,C:M:bm202rplace,M:20:{LAST_M20_ANGLE_RAW},C:M:DMplacing,M:20:SLOT,C:M:DM,C:M:b201placeUp,C:M:DM,C:M:b201place,C:M:DM,2:1,C:M:b200,C:M:DM",
        "placing_r_1l_2_0": "C:M:bm201lplace,M:20:{LAST_M20_ANGLE_RAW},C:M:DMplacing,M:20:SLOT,C:M:DM,C:M:b201place,C:M:DM,C:M:b200,C:M:DM",
        "placing_r_2l_0": "C:M:bm221place,M:20:{LAST_M20_ANGLE_RAW},C:M:DMplacing,M:20:SLOT,C:M:DM,C:M:b201placeUp,C:M:DM,C:M:b201place,C:M:DM,C:M:b200,C:M:DM",
        "placing_r_2r_0": "C:M:bm2m21place,M:20:{LAST_M20_ANGLE_RAW},C:M:DMplacing,M:20:SLOT,C:M:DM,C:M:b201placeUp,C:M:DM,C:M:b201place,C:M:DM,C:M:b200,C:M:DM",
        "placing_r_4l_0": "C:M:b2m21place,M:20:{LAST_M20_ANGLE_RAW},C:M:DMplacing,M:20:SLOT,C:M:DM,C:M:b201place,C:M:DM,C:M:b200,C:M:DM",
        "placing_r_4r_0": "C:M:b221place,M:20:{LAST_M20_ANGLE_RAW},C:M:DMplacing,M:20:SLOT,C:M:DM,C:M:b201place,C:M:DM,C:M:b200,C:M:DM",
        "placing_r_3c_1": "C:M:bm203rplace,M:20:{LAST_M20_ANGLE_RAW},C:M:DMplacing,M:20:SLOT,C:M:DM,C:M:b203place,C:M:DM,C:M:b202place,C:M:DM,C:M:b201place,C:M:DM,C:M:b200,C:M:DM",
        "placing_r_3c_2": "C:M:bm203rplace,M:20:{LAST_M20_ANGLE_RAW},C:M:DMplacing,M:20:SLOT,C:M:DM,C:M:b203place,C:M:DM,C:M:b202place,C:M:DM,C:M:b201place,C:M:DM,C:M:b200place,C:M:DM,C:M:b20m1place,C:M:DM,C:M:b20m2,C:M:DM"
    }

    if action in sequence_map:
        commands = sequence_map[action].split(",")
        return commands
    else:
        print(f"⚠️ Unknown placing action: {action}")
        return []


# --- Shared function for parsing WebMvt messages ---
def parse_webmvt_message(message):
    try:
        parts = message.split(":")
        if len(parts) != 13:
            print("❌ Invalid WebMvt format")
            return
        _, carried, switch, relX, relY, relZ, dx1, dy1, dz1, dx2, dy2, dz2, actionName = parts

        carried_voxel = int(carried)
        relative_pos = (float(relX), float(relY), float(relZ))
        front_delta = (float(dx1), float(dy1), float(dz1))
        back_delta = (float(dx2), float(dy2), float(dz2))
        use_switch = bool(int(switch))

        if use_switch:
            front_delta, back_delta = back_delta, front_delta

        print("🔧 WebMvt Received:")
        print(f"  ▶ Switch       : {use_switch}")
        print(f"  ▶ Carried      : {carried_voxel} ")
        print(f"  ▶ FrontPos Rel : {relative_pos}")
        print(f"  ▶ Front Δ      : {front_delta}")
        print(f"  ▶ Back  Δ      : {back_delta}")
        print(f"  ▶ Action       : {actionName}")

        subactions = generate_movement_sequence(relative_pos, front_delta, back_delta, use_switch, carried_voxel)
        print("🧩 Executing generated sequence:")
        for cmd in subactions:
            parse_input(cmd)

    except Exception as e:
        print("❌ Error parsing WebMvt message:", e)


# --- Shared function for parsing PlaceMvt and PlaceMvtReverse messages ---
def parse_placemvt_message(message, reverse=False):
    try:
        global last_slot_used, last_action_used
        parts = message.split(":")
        _, slot, action = parts
        print(f"🦿 {'PlaceMvtReverse' if reverse else 'PlaceMvt'} message decoded:")
        print(f"   ▶ Action Name   : {action}")
        print(f"   ▶ Slot          : {slot}")
        last_slot_used = slot
        last_action_used = action
        subactions = generate_placing_sequence(action, slot)
        print("🧩 Executing generated sequence:", subactions)
        for cmd in subactions:
            parse_input(cmd)
    except Exception as e:
        print(f"⚠️ Failed to parse {'PlaceMvtReverse' if reverse else 'PlaceMvt'} message:", e)

def parse_pickmvt_message(message):
    try:
        parts = message.split(":")
        _, action = parts
        print(f"🦿 'PickMvt' message decoded:")
        print(f"   ▶ Action Name   : {action}")       
        parse_input("C:M:RC,C:M:DM,C:M:LC,C:M:DM,C:M:BC")  

# for cmd in subactions:
#             parse_input(cmd)
    except Exception as e:
        print(f"⚠️ Failed to parse 'PickMvt' message:", e)

async def listen_simulator(websocket):
    print("🌐 Connected to Digital Twin")
    async for message in websocket:
        print("📨 Received:", message)
        # You can parse or log here
        if message.startswith("WebMvt"):
            parse_webmvt_message(message)
        elif message.startswith("PickMvt"):
            parse_pickmvt_message(message)
        elif message.startswith("PlaceMvt"):
            parse_placemvt_message(message, reverse=False)
        elif message.startswith("PlaceMvtReverse"):
            parse_placemvt_message(message, reverse=True)
        

#################
### Main Loop ###
#################
def motor_to_input(pos):
    zeros = position0#np.array([0.5, 3.181, 4.02, 5.056, 2.05])
    frame = np.zeros(5)
    frame[0] = -1 * (pos[0] - zeros[0]) + 1.57
    frame[1] = -1 * (pos[1] - zeros[1])
    frame[2] = -1 * (pos[2] - zeros[2])
    frame[3] = -1 * (pos[3] - zeros[3])
    frame[4] = -1 * (pos[4] - zeros[4]) - 1.57
    return np.round(frame, 4)

def interpolate_and_execute(joint_angles, Eall, ids, inp, parse_input, is_slow=False):
    # Use fast mode unless explicitly told to use 'slow'
    #is_slow = len(inp) > 3 and inp[3] == "slow"

    if not is_slow:
        # Fast mode: send command directly without interpolation
        for j in range(len(ids)):
            parse_input(f"M:{ids[j]}:{joint_angles[j]}")
        return

    if len(inp) == 4:
        speed = float(inp[3])
    else:
        speed = 0.05

    step_amt = 10
    steps = np.linspace(0, 1, step_amt + 2).reshape(-1, 1)
    joint_sequence = (1 - steps) * Eall + steps * joint_angles
    print(joint_sequence)

    for i in range(step_amt + 2):
        pos = joint_sequence[i]
        for j in range(len(ids)):
            print("M:" + str(ids[j]) + ":" + str(pos[j]))
            parse_input("M:" + str(ids[j]) + ":" + str(pos[j]))
        time.sleep(speed)


def parse_input(inputs):

    cont = True
    global last_commanded_angles

    inputs = inputs.translate({ord(c): None for c in string.whitespace})
    inputs = inputs.split(",")

    # print(inputs)
    if len(inputs) > 1:
        queue = list(inputs)
        while queue:
            cmd = queue.pop(0)
            cont = parse_input(cmd)
            if not cont:
                break
        return cont
    # else:
    #     return cont

    if inputs == ['']:
        return cont

    inp = inputs[0]

    # --- Preprocess slot and angle placeholders ---
    global last_slot_used, last_action_used
    # Preprocess slot and angle placeholders
    if "M:20:SLOT" in inp or "M:20:{LAST_M20_ANGLE_RAW}" in inp or "C:M:OpenSLOT" in inp:
        # Attempt to extract slot from prior context
        slot = last_slot_used
        print(slot)
        # Check if there's a recent PlaceMvt or PlaceMvtReverse message
        # (This assumes you will store `last_slot_used` and `last_action_used` globally when parsing those messages)
        try:
            if "M:20:SLOT" in inp:
                boardAddress = struct.pack("!B", 20)
                packet = b"E" + boardAddress
                if transmit:
                    esp32.send(packet)
                    M20value = struct.unpack("<f", esp32.recv())[0]
                else:
                    M20value = 0
                slotAngle = getslotAngle(last_action_used, last_slot_used, M20value)
                inp = inp.replace("M:20:SLOT", f"M:20:{slotAngle:.3f}")
                last_commanded_angles[0]=slotAngle
            if "M:20:{LAST_M20_ANGLE_RAW}" in inp:
                inp = inp.replace("M:20:{LAST_M20_ANGLE_RAW}", f"M:20:{LAST_M20_ANGLE_RAW:.3f}")
            if "C:M:OpenSLOT" in inp and last_slot_used:
                # print("a")
                orientation = None
                if last_slot_used == "left":
                    orientation = "LO"
                elif last_slot_used == "backward":
                    orientation = "BO"
                elif last_slot_used == "right":
                    orientation = "RO"
                    # print("b")
                if orientation:
                    inp = inp.replace("OpenSLOT", f"{orientation}")
                    # print(inp)
        except Exception as e:
            print("⚠️ Failed to resolve dynamic SLOT/ANGLE replacements:", e)

    # --- WebSocket multi-robot support: route by robot id prefix ---
    # e.g., "robot1:M:25:1.65"

    if inp[0] == "A": # Attach next module
        # this is the attach next module command.  
        inp = inp.split(":")
        boardAddress = struct.pack("!B", int(inp[1]))
        heatTime = struct.pack("!B",int(inp[2])) #inp[2].endcode('utf-8')#struct.pack("@f",float(inp[2]))
        packet = b"A" + boardAddress + heatTime
        if transmit:
            esp32.send(packet)
        # print("done")
    elif inp[0] == "Z": # Zero fasteners
        # this is to zero the fasteners. other module types will not process this
        inp = inp.split(":")
        boardAddress = struct.pack("!B", int(inp[1]))
        #print(boardAddress)
        packet = b"Z" + boardAddress
        if transmit:
            esp32.send(packet)
        # print("done")
    elif inp[0] == "M": # Move single motor
        # This sends a new position (in degrees) to a module
        inp = inp.split(":")
        boardAddress = struct.pack("!B", int(inp[1]))
        thetaDes = struct.pack("@f", float(inp[2]))
        #print(boardAddress+thetaDes)
        packet = b"M" + boardAddress + thetaDes
        if transmit:
            esp32.send(packet)
        # print(packet)
        # print("sending:")
        # print(inp[1])
        # print(inp[2])
        # print("done")
    elif inp[0] == "E": # Get motor position
        # This requests the module's position to be reported back
        inp = inp.split(":")
        boardAddress = struct.pack("!B", int(inp[1]))
        #request data...
        packet = b"E"+boardAddress
        if transmit:
            esp32.send(packet)
            message = struct.unpack("<f", esp32.recv())
            print(round(message[0],3))
        # print("done")

    elif inp[0] == "L": # LED
        # this flashes the LED on the ESP32 board
        inp = inp.split(":")
        LEDrepeats = struct.pack("!B", int(inp[1]))
        packet = b"L"+LEDrepeats
        if transmit:
            esp32.send(packet)
        print(packet)
        # print("done")
    elif inp[0] == "J": # i2c recovery method
        #i2c recovery method
        inp = inp.split(":")
        packet = b"J"
        if transmit:
            # esp32.send(packet)
            print("bus recovery kills the bus sorry")
        else:
            print("wifi off")
        print(packet)
        # print("done")
    elif inp[0] == "1":
        message = input("input: ")
        # print(message)
        if message.startswith("WebMvt"):
            parse_webmvt_message(message)
        elif message.startswith("PickMvt"):
            parse_pickmvt_message(message)
        elif message.startswith("PlaceMvt"):
            parse_placemvt_message(message, reverse=False)
        elif message.startswith("PlaceMvtReverse"):
            parse_placemvt_message(message, reverse=True)

    elif inp[0] == "V": # voxel gripper position
        # This sends a new position to the additional voxel gripper
        inp = inp.split(":")
        boardAddress = struct.pack("!B", int(inp[1]))
        thetaDes = struct.pack("@f", float(inp[2]))
        #print(boardAddress+thetaDes)
        packet = b"V" + boardAddress + thetaDes
        if transmit:
            esp32.send(packet)
        #print(packet)
        # print("done")
    elif inp[0] == "W": # voxel gripper position # this is P in the firmware parsing FYI
        # This sends a new position to the upper rail of the voxel gripper (lower voxel into place)
        inp = inp.split(":")
        boardAddress = struct.pack("!B", int(inp[1]))
        thetaDes = struct.pack("@f", float(inp[2]))
        #print(boardAddress+thetaDes)
        packet = b"P" + boardAddress + thetaDes
        if transmit:
            esp32.send(packet)
        #print(packet)
        # print("done")
    elif inp[0] == "F": # fastener position
        # This sends a new position to the lower rail of the voxel gripper (the screw position)
        inp = inp.split(":")
        boardAddress = struct.pack("!B", int(inp[1]))
        thetaDes = struct.pack("@f", float(inp[2]))
        #print(boardAddress+thetaDes)
        packet = b"F" + boardAddress + thetaDes
        if transmit:
            esp32.send(packet)
        #print(packet)
        # print("done")
    elif inp[0] == "R": # fastener rotation speed
        # This sends a new desired rotation SPEED to the fastener (fastener will convert it to an int and 10x it, e.g. send 5 to get PWM 50)
        inp = inp.split(":")
        boardAddress = struct.pack("!B", int(inp[1]))
        thetaDes = struct.pack("@f", float(inp[2]))
        #print(boardAddress+thetaDes)
        packet = b"R" + boardAddress + thetaDes
        if transmit:
            esp32.send(packet)
        #print(packet)
        # print("done")
    elif inp[0] == "2":
        #sleep
        inp = inp.split(":")
        # print(inp[1])
        print("sleep")
        time.sleep(float(inp[1]))
    elif inp[0] == "P": # Pause
        # This is the Pause command, use it to manually space out moves, or to have a chance to manually recover the robot between moves
        keepon = input("press a key to continue, or a cmd to add... ")
        if keepon == "Q":
            cont = parse_input("Q")
            return cont
        elif len(keepon)>=1:
            parse_input(keepon)
            parse_input("P")
    elif inp[0] == "WebMvt":
        parse_webmvt_message(":".join(inp))
    elif inp[0] == "C": # command
        # This is the multi-command, for sending commands to multiple modules at once. 
        inp = inp.split(":")
        if inp[1] == "E": # Get motor positions
            if len(inp) == 2:
                Eall = []
                for i in range(len(ids)):
                    boardAddress = struct.pack("!B", ids[i])
                    packet = b"E"+boardAddress
                    if transmit:
                        esp32.send(packet)
                        message = struct.unpack("<f", esp32.recv())
                    #EConverter(boardAddress, message[0])
                        Eall.append(round(message[0],3))
                    else:
                        print("WiFi disabled")
                    #Eall.append(EConverter(ids[i], message[0]))
                print(Eall)
            else:
                # This is vestigial but I'm not getting rid of yet b/c I'm not sure what I intended this for (as of April 2025)
                if inp[2] == "M":  # Single motor command
                    Eall = []
                    for i in range(len(ids)):
                        boardAddress = struct.pack("!B", ids[i])
                        packet = b"E"+boardAddress
                        esp32.send(packet)
                        message = struct.unpack("<f", esp32.recv())
                        #EConverter(boardAddress, message[0])
                        Eall.append(round(message[0],3))
                        #Eall.append(EConverter(ids[i], message[0]))
                    print(Eall)
                elif inp[2] == "F": # Single fastener command
                    Eall = []
                    for i in range(4):
                        boardAddress = struct.pack("!B", fids[i])
                        packet = b"E"+boardAddress
                        esp32.send(packet)
                        message = struct.unpack("<f", esp32.recv())
                        #EConverter(boardAddress, message[0])
                        Eall.append(round(message[0],3))
                        #Eall.append(EConverter(ids[i], message[0]))
                    print(Eall)
                else:
                    print("unrecognized")
        elif inp[1] == "I": #WIFI
            Eall = []
            for i in range(len(ids)):
                boardAddress = struct.pack("!B", ids[i])
                packet = b"E"+boardAddress
                if transmit:
                    esp32.send(packet)
                    message = struct.unpack("<f", esp32.recv())
                    Eall.append(round(message[0],3))
                else:
                    print("WiFi disabled")
            print(Eall)
            input_frame = motor_to_input(np.array(Eall))
            print("Recovered input frame:", input_frame)
             
        elif inp[1] == "M": # MULTI-MOVE
            ##################
            ### MULTI-MOVE ###
            ##################

            #Get current joint positions:
            Eall = []
            validPosition = True
            if transmit:
                for i in range(len(ids)):

                    Eall_val = False
                    count = 0
                    while not Eall_val:
                        boardAddress = struct.pack("!B", ids[i])
                        packet = b"E"+boardAddress
                        esp32.send(packet)
                        message = struct.unpack("<f", esp32.recv())
                        if np.isnan(message[0]) == False :
                            Eall.append(round(message[0],3))
                            Eall_val = True
                        elif count > 5:
                            Eall_val = True
                            print(f"repeat E failure at {i}")
                            validPosition = False
                        else:
                            Eall_val = False
                            #print(type(message[0]))
                            count += 1
                            #print(f"E failure at '{i}' ")
            else:
                Eall = position0 #for testing

            # Only continue if the modules are able to respond (this indicates the bus is working)
            if validPosition == True:
                # frame definitions
                f000 = np.array([0, 0, 0, 0, 0, 0]) #vertical

                f200l = np.array([ 0.74,  0.31, -2.605,  0.193, -2.363]) # ([ 0.74,  0.31, -2.605,  0.193, -2.363])
                f201l = np.array([0.73, -0.264, -2.4,  0.894, -2.368])      #
                f200d = np.array([ 0.74, -0.02, -2.63,  0.55, -2.368])

                f400l = np.array([ 1.178,  0.779, -1.644,  0.724, -1.93]) 
                f401l = np.array([ 1.178,  0.534,  -1.5,  1.019, -1.93]) 
                f402l = np.array([ 1.178,  0.372, -1.414,  1.362, -1.93]) 
       
                # =================================

                # Exact positions for the front feet

                # ===================== 200 =====================
                f200 = np.array([ 0.73,  0.247, -2.655,  0.193, -2.368])    # 
                f201 = np.array([0.73, -0.264, -2.529,  0.894, -2.368])     #
                f202 = np.array([ 0.73, -0.56, -2.28,  1.404, -2.368])    #
                f203 = np.array([ 0.73, -0.47,  -1.896,  1.759, -2.368])    #   
                f204 = np.array([ 0.73, -0.24, -1.348,  2.0955, -2.368])    #  IMPORTANT
                f205 = np.array([ 0.73, -0.44,   -1.08,   2.095, -2.368])    #  IMPORTANT
                b200 = f200
                b201 = np.array([ 0.81,   0.86,  -2.498, -0.351, -2.378]) 

                b20m1 = f201
                b20m2 = np.array([ 0.73, -0.56, -2.244, 1.404, -2.368])
                b20m3 = f203
                b20m4 = f204
                b20m5 = f205
                # ===================== 400 =====================
                f400 = np.array([ 1.178,  0.709, -1.644,  0.724, -1.93]) 
                f401 = np.array([ 1.178,  0.52,  -1.58,  1.019, -1.93])  
                f402 = np.array([ 1.178,  0.313, -1.4,  1.3, -1.93]) 
                f403 = np.array([ 1.178,  0.328, -1.093,  1.697, -1.93]) 
                f404 = np.array([ 1.178 ,  0.415, -0.509,  2.153, -1.93]) 
                f405 = np.array([ 1.178 ,  0.169, -0.545,  2.016, -1.93]) 

                b401 = np.array([ 1.201,  1.051, -1.564,  0.497, -1.92])
                b400 = f400
                
                b40m1 = f401
                b40m2 = f402
                b40m3 = f403
                b40m4 = np.array([ 1.187,  0.493, -0.435,  2.157, -1.938])
                b40m5 = f405

                # ===================== LEFT =====================
                f220 = np.array([ 1.752,  0.442, -2.23,   0.413, -2.894,]) #
                f221 = np.array([ 1.751,  0.129,  -2.147,   0.834, -2.903])
                f222 = np.array([ 1.751,  -0.055,  -1.942,   1.267, -2.903])
                f223 = np.array([ 1.751, -0.098, -1.605,  1.636, -2.903])
                f224 = np.array([ 1.751, -0.032, -1.164,  2.022, -2.903])
                f225 = np.array([ 1.751, -0.568,  -1.23,    1.807, -2.903]) # TO DO 

                b221 = np.array([ 1.751, 0.898,  -2.097,  0.089, -2.903])
                b220 = f220
                b22m1 = f221
                b22m2 = f222
                b22m3 = f223
                b22m4 = np.array([ 1.751, -0.122, -1.164,  2.022, -2.902])
                b22m5 = f225

                # ===================== RIGHT =====================
                

                f2m20 = np.array([ 0.229, 0.433, -2.247, 0.403, -1.317])#
                f2m21 = np.array([ 0.2398,  0.129,  -2.147,   0.834,  -1.307 ])#
                f2m22 = np.array([ 0.274,  -0.055,  -1.942,   1.267,  -1.275])#
                f2m23 = np.array([ 0.249, -0.098, -1.605,  1.636, -1.302])#
                f2m24 = np.array([ 0.245, -0.129, -1.154,  2.023, -1.367])#
                f2m25 = np.array([ 0.244, -0.45, -1.079, 2.0, -1.3  ])#

                b2m21 = np.array([ 0.242,  0.898,  -2.097,  0.089,  -1.294 ])
                b2m20 = f2m20   
                b2m2m1 = f2m21
                b2m2m2 = f2m22
                b2m2m3 = f2m23
                b2m2m4 = np.array([ 0.241, -0.117, -1.174,  2.001, -1.302])
                b2m2m5 = f2m25

                # ===================== 200 =====================
                f200p = f200
                f201p = np.array([ 0.8, -0.139, -2.529,  0.66,  -2.313])
                f202p = np.array([ 0.786, -0.549, -2.396, 1.159, -2.352])
                f203p = np.array([ 0.73, -0.6, -1.895, 1.759, -2.369])
                f204p = np.array([ 0.921, -0.562, -1.43, 2.005, -2.22 ])
                f205p = np.array([ 0.87, -0.441, -1.082, 2.1, -3.72 ])

                b200p = f200l  
                b201p = np.array([ 0.91,   0.71,  -2.5,   -0.229, -2.209]) 
                b20m1p = np.array([ 0.918, -0.604, -2.475,  1.098, -2.221])
                b20m2p = np.array([ 0.73, -0.56, -2.281, 1.306, -2.368])  
                b20m3p = f203p  
                b20m4p = np.array([ 0.729, -0.41, -1.21,  2.25, -2.367])
                b20m5p = f205p

                # ===================== 400 =====================
                f400p = f400 
                f401p = np.array([ 1.236,  0.375, -1.64, 0.95, -1.923])
                f402p = f402  
                f403p = np.array([ 1.183,  0.22, -1.203,  1.563, -1.928])
                f404p = f404  
                f405p = np.array([ 1.16, 0.095, -0.627, 2.048, -1.977])
                b400p = b400 
                b401p = np.array([ 1.201,  0.972, -1.53,  0.43, -1.84])
                b40m1p = np.array([ 1.286,  0.273, -1.635, 0.964, -1.887])
                b40m2p = np.array([ 1.178, 0.25, -1.4, 1.3, -1.93]) 
                b40m3p = np.array([ 1.229,  0.134, -0.973, 1.753, -1.916]) 
                b40m4p = f404p 
                b40m5p = f405p

                # f401p = np.array([ 1.18,   0.49,  -1.699,  0.893, -1.931]) 
                # f402p = np.array([ 1.18,   0.49,  -1.699,  0.893, -1.931])

                f220p = f220  
                f221p = np.array([ 1.92, 0.0, -2.28, 0.806, -2.84])
                f222p = f222 
                f223p = f223 
                f224p = f224 
                f225p = f225 

                b221p = np.array([ 1.898,  0.678, -2.123,  0.176, -2.825]) 
                b220p = b220 
                b22m1p = np.array([ 1.751, -0.1, -2.159, 0.957, -2.901]) 
                b22m2p = np.array([ 1.847, -0.338, -1.733,  1.481, -2.882]) 
                b22m3p = np.array([ 1.816, -0.265, -1.455,  1.756, -2.864]) 
                b22m4p = b22m4 
                b22m5p = b22m5 

                f2m20p = f2m20
                f2m21p = np.array([ 0.372, 0.116, -2.164, 0.733, -1.322])
                f2m22p = f2m22 
                f2m23p = f2m23                 
                f2m24p = f2m24                 
                f2m25p = f2m25 

                b2m21p = np.array([0.519, 0.801, -2.18, -0.044, -1.17])#np.array([ 0.368,  0.727, -2.093,  0.2,   -1.27 ])
                b2m20p = b2m20   
                b2m2m1p = np.array([ 0.287, 0.049, -2.055,  0.856, -1.276])
                b2m2m2p = np.array([ 0.306, -0.1 , -1.76, 1.333,-1.265])
                b2m2m3p = np.array([ 0.249, -0.098, -1.43, 1.656, -1.301])
                b2m2m4p = b2m2m4
                b2m2m5p = b2m2m5


                # ============================= not used =============================

                f300 = np.array([ 1.039, 0.521, -2.138, 0.484, -2.087]) 
                f302 = np.array([ 1.073,  0.01, -1.874,  1.249, -2.04])
                f303 = np.array([ 1.12,  0.01, -1.37,  1.76, -2.03])

                b301 = np.array([ 1.091,  0.918, -2.016,  0.194, -2.032])
                b302 = np.array([ 1.077,  1.313 , -1.823,  0.0, -2.057])

                f500 = np.array([ 1.3 ,  1.05, -1.05,  1.04, -1.85])
                f501 = np.array([ 1.306 ,  0.91, -0.922,  1.29, -1.83])
                f502 = np.array([ 1.31 ,  0.853, -0.598,  1.677, -1.83])

                b402 = np.array([ 1.175,  1.366, -1.373,  0.381, -1.933])

                # =============================  Helper for colliding moves =============================

            
                f301p = np.array([ 1.064, 0.178, -2.03, 0.9, -2.045])
                b301p = np.array([1.164, 0.85, -1.879, 0.106, -1.955])#np.array([ 1.164,  0.85,  -2.022,  0.206, -1.956])
                b30m1p = np.array([ 1.064, 0.178, -2.073, 0.877, -2.055])

                # ============================= CARRYING POSITION  =============================
                carryingb200 = b200
                carryingb400 = b400

                carryingb220 = b220
                carryingb2m20 = b2m20

                carryingb20m2 = b20m2

                carryingb201p = np.array([1.051, 0.709, -2.431, -0.294, -2.05])#np.array([ 0.91,   0.71,  -2.5,   -0.229, -2.209]) #np.array([ 1.051, 0.757, -2.488, -0.361, -2.194])
                carryingb401p = np.array([ 1.175,  1.366, -1.373,  0.381, -1.933])#np.array([ 1.203, 0.977,-1.469, 0.418, -1.916])#([ 1.175,  1.366, -1.373,  0.381, -1.933])

                carryingb221p = np.array([ 1.903, 0.636, -2.125, 0.114, -2.823])
                carryingb2m21p = np.array([0.57, 0.785, -2.029,  -0.144, -1.07])

                carryingb40m1p = np.array([ 1.287, 0.274, -1.617, 1.027, -1.884])
                carryingb30m1p = b30m1p
                carryingb20m1p = b20m1p

                carryingb40m3p = np.array([ 1.318, 0.134, -0.954, 1.633, -1.86])
                carryingb40m2p = b40m2p

                # ===================== PLACING =============================

                b020  = np.array([ 0.729,  0.316, -2.637,  0.193, -0.82 ]) # f2m1 1_1 #

                b0m20 = np.array([ 1.02, 0.25, -2.561, 0.156, -3.72 ])##np.array([ 0.73,   0.317, -2.637,  0.193, -3.92 ])# right 1_1
                
                bm200r = np.array([ 1.02, 0.301, -2.58, 0.194, -5.47 ])#
                bm200c = np.array([1.12, 0.301, -2.58, 0.194, -5.47])

                bm200l = np.array([ 0.72,  0.247, -2.655,  0.193, 0.78])


                bm2m20 =  np.array([ 1.84, 0.15, -2.28, 0.506, -4.52])##  # right #### 

                bm220 =  np.array([ 0.32, 0.151, -2.28, 0.506, 0.331])#np.array([ 0.246, 0.472, -2.24, 0.42,  0.233])   # left #

                bm202r  = np.array([None, None, None, None, None])

                b2m20place =  np.array([ 0.44, 0.351, -2.181, 0.456, -1.17 ])###

                b201place = np.array([0.99, 0.8, -2.46, -0.303, -2.09])#b201p #
                b201placeUp = np.array( [ 1.219, 1.051, -2.28, -0.544, -1.97 ])

                b021place = np.array([ 0.97,  0.974, -2.25, -0.352, -0.62])#
                b020place = np.array([ 0.97,  0.316, -2.48,  0.193, -0.62 ])#

                b0m21place = np.array([ 1.02,  0.974, -2.38, -0.352, -3.72])#

                b0m22place = np.array([ 0.92, 1.221, -2.229, -0.444, -3.82 ])#

                b0m20place = np.array([ 1.02,  0.7, -2.38, -0.043, -3.72])#

                bm201rplace = np.array([ 1.02, 0.71, -2.497, -0.229, -5.37 ])##

                bm201cplace = np.array([ 1.02, 0.71, -2.497, -0.229, -5.22 ]) #

                bm202rplace = np.array([ 1.02, 1.051, -2.28, -0.544, -5.37 ])#

                bm201lplace = np.array([ 0.72, 0.71, -2.497, -0.229, 0.929]) # 
                bm221place =  np.array([ 0.32, 0.701, -2.029, 0.156, 0.331]) ## left 
                bm2m21place = np.array([ 1.84,  0.895, -2.03, -0.01, -4.52])## right 

                b2m21place = np.array([ 0.44,  0.727, -2.093,  0.2,   -1.17 ])
                b2m21placeI = np.array([ 1.97,  1.1, -1.93, -0.044, -1.17 ])

                b220place = np.array([ 1.861, 0.379, -2.18, 0.413, -2.894])
                b221place  = np.array([ 1.861,  0.678, -2.123,  0.176, -2.894]) 
                b221placeI = np.array([ 0.271,  0.851, -2.123,  -0.044, -2.894]) 

                b202place  = np.array(([0.99, 0.8, -2.46, -0.303, -2.09]))
                b203place  = np.array([None, None, None, None, None])
                b20m1place  = b20m1p
                b200place  = b200p
                bm203rplace = np.array([None, None, None, None, None])

                b021placeUp = np.array([ 0.77, 1.079, -2.41, -0.426, -0.769])
                b0m21placeUp = np.array([ 0.77, 1.08, -2.41, -0.425, -4.071])

                bm201rplaceUp = np.array([ 0.77, 1.079, -2.41,  -0.425, -5.57 ]) # OK
                bm201lplaceUp = np.array([ 0.77, 1.079, -2.41,  -0.425, 0.731 ])  #OK
                bm221placeUp = np.array([ 0.246, 1.079, -2.031, -0.124, 0.234])
                bm2m21placeUp = np.array([ 1.766, 1.079, -2.158, -0.125, -4.552])

                b2m21placeUp = np.array([ 0.238, 0.779, -2.294, -0.125, -1.38 ])
                b221placeUp = np.array([ 1.771, 0.879, -2.28, -0.125, -2.982])

                def offset_for_real(pos):
                    pos[0] = -1*(pos[0] - 1.57) + position0[0] #
                    pos[1] = -1*pos[1] + position0[1]
                    pos[2] = -1*pos[2] + position0[2]
                    pos[3] = -1*pos[3] + position0[3]
                    pos[4] = -1*(pos[4]+1.57) + position0[4]
                    pos = np.round(pos,4)
                    return pos


                if len(inp) > 2:
                    # get current positions
                    # print("raw position:")
                    # print(Eall)
                    position_current = Eall - position0
                    # print("current position:")
                    # print(position_current)

                    # we will run the motors by recursively calling parse_input("M:address:thetades") with delays

                    poses = {
                        "f000": f000, # vertical

                        # FORWARD
                        "f200": f200, "f201": f201, "f202": f202, "f203": f203,  "f204": f204, "f205": f205,  
                        "b201": b201,"b200": b200, "b20m1": b20m1, "b20m2": b20m2, "b20m3": b20m3, "b20m4": b20m4, "b20m5": b20m5,

                        "f200p": f200p, "f201p": f201p, "f202p": f202p, "f203p": f203p, "f204p": f204p, "f205p": f205p,
                        "b200p": b200p, "b201p": b201p, "b20m1p": b20m1p, "b20m2p": b20m2p, "b20m3p": b20m3p, "b20m4p": b20m4p, "b20m5p": b20m5p,


                        "f400": f400, "f401": f401, "f402": f402, "f403": f403, "f404": f404, "f405": f405,
                        "b401": b401, "b400": b400, "b40m1": b40m1, "b40m2": b40m2, "b40m3": b40m3, "b40m4": b40m4,"b40m5": b40m5,

                        "f400p": f400p, "f401p": f401p, "f402p": f402p, "f403p": f403p, "f404p": f404p, "f405p": f405p,
                        "b400p": b400p, "b401p": b401p, "b40m1p": b40m1p, "b40m2p": b40m2p, "b40m3p": b40m3p, "b40m4p": b40m4p,"b40m5p": b40m5p,

                        # LEFT
                        "f220": f220, "f221": f221, "f222": f222, "f223": f223, "f224": f224, "f225": f225,
                        "b220": b220, "b221": b221, "b22m1": b22m1, "b22m2": b22m2, "b22m3": b22m3, "b22m4": b22m4, "b22m5": b22m5,
                        "f220p": f220p, "f221p": f221p, "f222p": f222p, "f223p": f223p, "f224p": f224p, "f225p": f225p,
                        "b221p": b221p, "b220p": b220p, "b22m1p": b22m1p, "b22m2p": b22m2p, "b22m3p": b22m3p, "b22m4p": b22m4p, "b22m5p": b22m5p,

                        # RIGHT
                        "f2m20": f2m20, "f2m21": f2m21, "f2m22": f2m22, "f2m23": f2m23, "f2m24": f2m24, "f2m25": f2m25,
                        "b2m20": b2m20, "b2m21": b2m21, "b2m2m1": b2m2m1, "b2m2m2": b2m2m2, "b2m2m3": b2m2m3, "b2m2m4": b2m2m4, "b2m2m5": b2m2m5,

                        "f2m20p": f2m20p, "f2m21p": f2m21p, "f2m22p": f2m22p, "f2m23p": f2m23p, "f2m24p": f2m24p, "f2m25p": f2m25p,
                        "b2m21p": b2m21p, "b2m20p": b2m20p, "b2m2m1p": b2m2m1p, "b2m2m2p": b2m2m2p, "b2m2m3p": b2m2m3p, "b2m2m4p": b2m2m4p, "b2m2m5p": b2m2m5p,

                        "b0m20":b0m20,
                        "b020":b020,
                        "b020place" : b020place,
                        "b2m20place": b2m20place,

                        "b201place":b201place,
                        "b021place":b021place,
                        "b0m21place":b0m21place,
                        "b0m20place":b0m20place,
                        "b2m21placeI":b2m21placeI,
                        "b220place":b220place,
                        "b221placeI":b221placeI,
                        "b0m22place":b0m22place,
                        "bm202rplace":bm202rplace,

                        "bm200r":bm200r,
                        "bm201rplace": bm201rplace,
 
                        "bm200l":bm200l,
                        "bm201lplace":bm201lplace,

                        "bm220":bm220,
                        "bm221place":bm221place,
                                                
                        "bm2m20":bm2m20,
                        "bm2m21place":bm2m21place,
                        "bm202r": bm202r,
                        "b2m21place": b2m21place,
                        "b221place": b221place,
                        "b202place": b202place,
                        "b203place": b203place,
                        "b20m1place": b20m1place,
                        "b200place": b200place,
                        "bm203rplace ": bm203rplace,
 
                        "carryingb40m2p":carryingb40m2p,
                        "carryingb40m3p":carryingb40m3p,
                        "carryingb30m1p":carryingb30m1p,
                        "carryingb20m1p":carryingb20m1p,
                        "carryingb40m1p":carryingb40m1p,
                        "carryingb2m21p":carryingb2m21p,
                        "carryingb221p":carryingb221p,
                        "carryingb401p":carryingb401p,
                        "carryingb201p":carryingb201p,
                        "carryingb20m2":carryingb20m2,
                        "carryingb2m20":carryingb2m20,
                        "carryingb220":carryingb220,
                        "carryingb400":carryingb400,
                        "carryingb200":carryingb200,
                        "bm200c":bm200c,
                        "bm201cplace":bm201cplace,


                        "f301p": f301p,
                        "b30m1p":b30m1p,
                        "b301p":b301p,
                        # UNUSED POSITIONS

                        # "f400inter":f400inter,
                        # "f500": f500, "f501": f501, "f502": f502,
                        "b301": b301, "b302": b302,
                        "b402":b402,
                        "b201placeUp":b201placeUp
                        # "f300": f300, "f301": f301, "f302": f302, "f303": f303,
                        # "f201p":f201l,"f401p":f401l
                    }

                    if inp[2] in poses:
                        joint_angles = offset_for_real(poses[inp[2]])
                        last_commanded_angles = joint_angles
                        interpolate_and_execute(joint_angles, Eall, ids, inp, parse_input, is_slow=False)
                        
                    elif inp[2] == "DF" or inp[2] == "DM" or inp[2] == "DMplacing" or inp[2] == "DFplacing" or inp[2] == "DMcarried":  # DF === fixing delay, DM === moving delay
                        if last_commanded_angles is None:
                            print(" No previous joint command found.")
                        else:
                            max_wait_time = 5.0  # seconds
                            wait_start = time.time()
                            success = False

                            while time.time() - wait_start < max_wait_time:
                                Eall_check = []
                                for i in range(len(ids)):
                                    boardAddress = struct.pack("!B", ids[i])
                                    packet = b"E" + boardAddress
                                    esp32.send(packet)
                                    message = struct.unpack("<f", esp32.recv())
                                    if not ((inp[2] == "DMplacing" or inp[2] == "DFplacing") and i == 0):
                                        Eall_check.append(round(message[0], 3))

                                Eall_check = np.array(Eall_check)
                                if inp[2] == "DMplacing" or inp[2] == "DFplacing":
                                    error = np.abs(Eall_check - last_commanded_angles[1:])  # exclude motor 0
                                else:
                                    error = np.abs(Eall_check - last_commanded_angles)
                                if (inp[2] == "DMcarried" and np.all(error < 0.09)):
                                    time.sleep(0.75)
                                    success = True
                                    break
                                elif np.all(error < 0.01):
                                    success = True
                                    break
                                time.sleep(0.1)

                            if success:
                                if inp[2] == "DM" or inp[2] == "DMplacing" or inp[2]=="DMcarried":
                                    time.sleep(0.01)  # small delay for movement //was 0.01
                                elif inp[2] == "DF" or inp[2] == "DFplacing":
                                    time.sleep(0.5)  # longer delay for fixing
                            else:
                                print(" Timeout waiting for joint convergence")
                                print(Eall_check)
                                print(last_commanded_angles)

# ------------------- GRIPPER OPEN/CLOSE

                    elif inp[2]=="O1":
                        # open gripper1 - hardcode - bigger closes
                        parse_input("M:25:0.65")
    
                    elif inp[2]=="C1":
                        # close gripper1
                        parse_input("M:25:1.5")

                    elif inp[2]=="O2":
                        # open gripper2 - hardcode.. smaller closes, bigger opens
                        parse_input("M:26:1.95")
                        # print("done")
                    elif inp[2]=="C2":
                        # close gripper2
                        parse_input("M:26:1")
                        # print("done")
                    # elif inp[2]=="O3":
                    #     #open gripper 3, voxel holder
                    #     parse_input("M:27:1.35")
                    #     # print("done")
                    # elif inp[2]=="C3":
                    #     #close gripper 3, voxel holder
                    #     parse_input("M:27:2.2")
                    #     # print("done")
                    elif inp[2]=="RO":
                        parse_input("V:27:1.1")
                    elif inp[2]=="RC":
                        parse_input("V:27:2.0")
                    elif inp[2]=="LO":
                        parse_input("M:27:2.0")
                    elif inp[2]=="LC":
                        parse_input("M:27:1.1")
                    elif inp[2]=="BO":
                        parse_input("W:27:1.7")
                    elif inp[2]=="BC":
                        parse_input("W:27:0.9")
                    

# ------------------- PLACING MOV (back leg mooving ==> right/left inverted)

                    # Use generate_placing_sequence for all placing actions
                    placing_actions = [
                        "placing_1l_1_0", "placing_3r_0", "placing_1r_1_0", "placing_3l_0",
                        "placing_1r_2_0", "placing_3c_0", "placing_1l_2_0", "placing_2l_0",
                        "placing_2r_0", "placing_4l_0", "placing_4r_0", "placing_3c_1",
                        "placing_3c_2",
                         
                        "placing_r_1l_1_0", "placing_r_3r_0", "placing_r_1r_1_0",
                        "placing_r_3l_0", "placing_r_1r_2_0", "placing_r_3c_0", "placing_r_1l_2_0",
                        "placing_r_2l_0", "placing_r_2r_0", "placing_r_4l_0", "placing_r_4r_0",
                        "placing_r_3c_1", "placing_r_3c_2"
                    ]
                    if inp[2] in placing_actions:
                        subactions = generate_placing_sequence(inp[2])
                        for cmd in subactions:
                            parse_input(cmd)

                ############################
            else:
                print("invalid joint positions, please re-try")
        elif inp[1] == "U":
            #THIS HAS BEEN DISABLED -- CHECK IF IT's STILL VALID
            print("disabeled)")
        else:
            print("invalid cmd")

    elif inp[0] == "file":
        #load file
        
        pass
    elif inp[0] == "Q":
        cont = False
        print("yeeting")
    else:
        print("whomst?")

    return cont

def initialize(inputs):
    inputs = inputs.translate({ord(c): None for c in string.whitespace})
    pass

# --- WebSocket server startup (runs in background thread) ---
def start_websocket_server():
    asyncio.run(_run_websocket_server())

async def _run_websocket_server():
    async with websockets.serve(listen_simulator, "0.0.0.0", 8765):
        print("🔌 WebSocket server started on ws://0.0.0.0:8765")
        await asyncio.Future()  # run forever
import threading
threading.Thread(target=start_websocket_server, daemon=True).start()

while cont:
    inputs = input(
        "Single commands: CMD:Addr:DesVal or CMD:Addr. Multi-commands: C:CMD. Input: "
    )
    cont = parse_input(inputs)
    # wb_manual = input("websocket message? ")
    # if wb_manual == "Y":
    #     message = input("input: ")
    #     print(message)
    #     if message.startswith("WebMvt"):
    #         parse_webmvt_message(message)
    #     elif message.startswith("PickMvt"):
    #         parse_pickmvt_message(message)
    #     elif message.startswith("PlaceMvt"):
    #         parse_placemvt_message(message, reverse=False)
    #     elif message.startswith("PlaceMvtReverse"):
    #         parse_placemvt_message(message, reverse=True)


###Single commands: CMD:Addr:DesVal or CMD:Addr. Multi-commands: C:CMD. Input: C:I
# [1.34, 3.003, 0.061, 4.872, 2.844]
# Recovered input frame: [ 0.73   0.247  3.609  0.184 -2.364]

# [1.34, 3.001, 6.22, 4.872, 2.844]
# Recovered input frame: [ 0.73   0.249 -2.55   0.184 -2.364]