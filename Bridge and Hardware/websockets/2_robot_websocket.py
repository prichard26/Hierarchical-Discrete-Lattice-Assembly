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

##### CE INITIALIZATION VALUE ######
# [1.682, 6.182, 2.728, 5.181, 5.439]

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

position0 = np.array([-0.65, 4.88, 3.105, 2.438, 4.7])

offset = position0

# addr = "ws://192.168.1.116" # ipconfig getifaddr en0 to get current ip

addr = "ws://192.168.2.7" 

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
        mid_vect3 = None
        mid_vect4 = None
        if z0 == z1:
            # print('start/end', start, end)
            if leg == "b" and start[0] == 4 and end[0] == 2:
                mid_vect3 = (3, 0, z0 + 1)
            if start[0] == 2 and end[0] == 4 and end[1] == 0:
                mid_vect3 = (3, 0, z0 + 1)

            if leg =="b" and start[0] == 2 and end[0] == 2 and carried_voxel > 0:
                mid_vect3 = (start[0], start[1], z0 + 2)
                mid_vect4 = (end[0], end[1], z1 + 2)

            mid_vec1 = (start[0], start[1], z0 + 1)
            mid_vec2 = (end[0], end[1], z1 + 1)

            path.append(pose_from_vector(leg, mid_vec1, carried_voxel, True, switch))
            path.append("C:M:DMcarried" if leg == "b" and carried_voxel > 0 else "C:M:DM")

            if mid_vect3 is not None:
                path.append(pose_from_vector(leg, mid_vect3, carried_voxel, True, switch))
                path.append("C:M:DMcarried" if leg == "b" and carried_voxel > 0 else "C:M:DM")

            if mid_vect4 is not None:
                path.append(pose_from_vector(leg, mid_vect4, carried_voxel, True, switch))
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
                if start[0] == 2 and end[0] and leg == "b":
                    mid_vect3 = (3, 0, z0 + 1)
                # print(f" aadzad za Interpolating Z down from {z0 + 1} to {z1} for leg {leg}")
                mid_vec2 = (start[0], start[1], z0 + 1)
                path.append(pose_from_vector(leg, mid_vec2, carried_voxel, True, switch))
                path.append("C:M:DMcarried" if leg == "b" and carried_voxel > 0 else "C:M:DM")
                
                if mid_vect3 is not None:
                    path.append(pose_from_vector(leg, mid_vect3, carried_voxel, True, switch))
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
            angle = math.pi / 2
        elif slot == "right":
            angle = math.pi / 2        
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
        "placing_1l_1_0": "C:M:O1,C:M:DF,C:M:C2,C:M:DF,C:M:b201place,C:M:DM,C:M:b201placeUp,C:M:DM,b021placeUp,C:M:DM,C:M:b021place,C:M:DM,M:20:SLOT,C:M:DM,C:M:b020,M:20:{LAST_M20_ANGLE_RAW},C:M:DMplacing,C:M:OpenSLOT,C:M:DFplacing",
        "placing_1r_1_0": "C:M:O1,C:M:DF,C:M:C2,C:M:DF,C:M:b201place,C:M:DM,C:M:b201placeUp,C:M:DM,C:M:b0m21placeUp,C:M:DM,C:M:b0m21place,C:M:DM,M:20:SLOT,C:M:DM,C:M:b0m20,M:20:{LAST_M20_ANGLE_RAW},C:M:DMplacing,C:M:OpenSLOT,C:M:DFplacing",
        "placing_3r_0": "C:M:O1,C:M:DF,C:M:C2,C:M:DF,C:M:b201place,C:M:DM,C:M:b201placeUp,C:M:DM,b021placeUp,C:M:DM,C:M:b021place,C:M:DM,M:20:SLOT,C:M:DM,C:M:b020,M:20:{LAST_M20_ANGLE_RAW},C:M:DMplacing,C:M:OpenSLOT,C:M:DFplacing",
        "placing_3l_0": "C:M:O1,C:M:DF,C:M:C2,C:M:DF,C:M:b201place,C:M:DMC:M:b201placeUp,C:M:DM,C:M:b0m21placeUp,C:M:DM,C:M:b0m21place,C:M:DM,M:20:SLOT,C:M:DM,C:M:b0m20,M:20:{LAST_M20_ANGLE_RAW},C:M:DMplacing,C:M:OpenSLOT,C:M:DFplacing",
        "placing_1r_2_0": "C:M:O1,C:M:DF,C:M:C2,C:M:DF,C:M:b201place,C:M:DM,C:M:b201placeUp,C:M:DM,C:M:bm201rplaceUp,C:M:DM,C:M:bm201rplace,C:M:DM,M:20:SLOT,C:M:DM,C:M:bm200r,M:20:{LAST_M20_ANGLE_RAW},C:M:DMplacing,C:M:OpenSLOT,C:M:DFplacing",
        "placing_3c_0": "C:M:O1,C:M:DF,C:M:C2,C:M:DF,C:M:b201place,C:M:DM,C:M:b201placeUp,C:M:DM,C:M:bm201rplaceUp,C:M:DM,C:M:bm201rplace,C:M:DM,M:20:SLOT,C:M:DM,C:M:bm200r,M:20:{LAST_M20_ANGLE_RAW},C:M:DMplacing,P,C:M:OpenSLOT,C:M:DFplacing",
        "placing_1l_2_0": "C:M:O1,C:M:DF,C:M:C2,C:M:DF,C:M:b201place,C:M:DM,C:M:b201placeUp,C:M:DM,C:M:bm201lplaceUp,C:M:DM,M:20:SLOT,C:M:DM,C:M:bm200l,M:20:{LAST_M20_ANGLE_RAW},C:M:DMplacing,C:M:OpenSLOT,C:M:DFplacing",
        "placing_2l_0": "C:M:O1,C:M:DF,C:M:C2,C:M:DF,C:M:b201place,C:M:DM,C:M:b201placeUp,C:M:DM,C:M:bm221placeUp,C:M:DM,C:M:bm221place,C:M:DM,M:20:SLOT,C:M:DM,C:M:bm220,M:20:{LAST_M20_ANGLE_RAW},C:M:DMplacing,C:M:OpenSLOT,C:M:DFplacing",
        "placing_2r_0": "C:M:O1,C:M:DF,C:M:C2,C:M:DF,C:M:b201place,C:M:DM,C:M:b201placeUp,C:M:DM,C:M:bm2m21placeUp,C:M:DM,C:M:bm2m21place,C:M:DM,M:20:SLOT,C:M:DM,C:M:bm2m20,M:20:{LAST_M20_ANGLE_RAW},C:M:DMplacing,C:M:OpenSLOT,C:M:DFplacing",

        "placing_4l_0": "C:M:O1,C:M:DF,C:M:C2,C:M:DF,C:M:b201place,C:M:DM,C:M:b201placeUp,C:M:DM,C:M:b2m21placeUp,C:M:DM,C:M:b2m21place,C:M:DM,M:20:SLOT,C:M:DM,C:M:b2m20place,M:20:{LAST_M20_ANGLE_RAW},C:M:DMplacing,C:M:OpenSLOT,C:M:DFplacing",
        "placing_4r_0": "C:M:O1,C:M:DF,C:M:C2,C:M:DF,C:M:b201place,C:M:DM,C:M:b201placeUp,C:M:DM,C:M:b221placeUp,C:M:DM,C:M:b221place,C:M:DM,M:20:SLOT,C:M:DM,C:M:b220place,M:20:{LAST_M20_ANGLE_RAW},C:M:DMplacing,C:M:OpenSLOT,C:M:DFplacing",
             

        "placing_r_1l_1_0": "C:M:b021place,M:20:{LAST_M20_ANGLE_RAW},C:M:DMplacing,M:20:SLOT,C:M:DM,C:M:b201placeUp,C:M:DM,C:M:b201place,C:M:DM,C:M:b200,C:M:DM",
        "placing_r_3r_0": "C:M:b021place,M:20:{LAST_M20_ANGLE_RAW},C:M:DMplacing,M:20:SLOT,C:M:DM,C:M:b201placeUp,C:M:DM,C:M:b201place,C:M:DM,C:M:b200,C:M:DM",
        "placing_r_1r_1_0": "C:M:b0m21place,M:20:{LAST_M20_ANGLE_RAW},C:M:DMplacing,M:20:SLOT,C:M:DM,C:M:b201placeUp,C:M:DM,C:M:b201place,C:M:DM,C:M:b200,C:M:DM",
        "placing_r_3l_0": "C:M:b0m21place,M:20:{LAST_M20_ANGLE_RAW},C:M:DMplacing,M:20:SLOT,C:M:DM,C:M:b201placeUp,C:M:DM,C:M:b201place,C:M:DM,C:M:b200,C:M:DM",
        "placing_r_1r_2_0": "C:M:bm201rplace,M:20:{LAST_M20_ANGLE_RAW},C:M:DMplacing,M:20:SLOT,C:M:DM,C:M:b201placeUp,C:M:DM,C:M:b201place,C:M:DM,C:M:b200,C:M:DM",
        "placing_r_3c_0": "C:M:bm201rplace,M:20:{LAST_M20_ANGLE_RAW},C:M:DMplacing,M:20:SLOT,C:M:DM,C:M:b201placeUp,C:M:DM,C:M:b201place,C:M:DM,C:M:b200,C:M:DM",
        "placing_r_1l_2_0": "C:M:bm201lplace,M:20:{LAST_M20_ANGLE_RAW},C:M:DMplacing,M:20:SLOT,C:M:DM,C:M:b201placeUp,C:M:DM,C:M:b201place,C:M:DM,C:M:b200,C:M:DM",
        "placing_r_2l_0": "C:M:bm221place,M:20:{LAST_M20_ANGLE_RAW},C:M:DMplacing,M:20:SLOT,C:M:DM,C:M:b201placeUp,C:M:DM,C:M:b201place,C:M:DM,C:M:b200,C:M:DM",
        "placing_r_2r_0": "C:M:bm2m21place,M:20:{LAST_M20_ANGLE_RAW},C:M:DMplacing,M:20:SLOT,C:M:DM,C:M:bm2m21placeUp,C:M:DM,C:M:b201placeUp,C:M:DM,C:M:b201place,C:M:DM,C:M:b200,C:M:DM",
        "placing_r_4l_0": "C:M:b2m21placeOut,M:20:{LAST_M20_ANGLE_RAW},C:M:DMplacing,M:20:SLOT,C:M:DM,C:M:b201placeUp,C:M:DM,C:M:b201place,C:M:DM,C:M:b200,C:M:DM",
        "placing_r_4r_0": "C:M:b221placeOut,M:20:{LAST_M20_ANGLE_RAW},C:M:DMplacing,M:20:SLOT,C:M:DM,C:M:b201placeUp,C:M:DM,C:M:b201place,C:M:DM,C:M:b200,C:M:DM",

        "placing_3c_1": "C:M:O1,C:M:DF,C:M:C2,C:M:DF,C:M:b201place,C:M:DM,C:M:b202place,C:M:DM,M:20:SLOT,C:M:DF,C:M:b203place,C:M:DM,C:M:bm203rplace,C:M:DM,C:M:bm202r,M:20:{LAST_M20_ANGLE_RAW},C:M:DMplacing,C:M:OpenSLOT,C:M:DFplacing",
        "placing_3c_2": "C:M:O1,C:M:DF,C:M:C2,C:M:DF,C:M:b20m1place,C:M:DF,C:M:b200place,C:M:DF,M:20:SLOT,C:M:DF,C:M:b201place,C:M:DM,C:M:b202place,C:M:DM,C:M:b203place,C:M:DM,C:M:bm203rplace,C:M:DM,C:M:bm202r,M:20:{LAST_M20_ANGLE_RAW},C:M:DMplacing,C:M:OpenSLOT,C:M:DFplacing",
          
        "placing_r_3c_1": "C:M:bm203rplace,M:20:{LAST_M20_ANGLE_RAW},C:M:DMplacing,M:20:SLOT,C:M:DM,C:M:b203place,C:M:DM,C:M:b202place,C:M:DM,C:M:b201place,C:M:DM,C:M:b200,C:M:DM",
        "placing_r_3c_2": "C:M:bm203rplace,M:20:{LAST_M20_ANGLE_RAW},C:M:DMplacing,M:20:SLOT,C:M:DM,C:M:b203place,C:M:DM,C:M:b202place,C:M:DM,C:M:b201place,C:M:DM,C:M:b200place,C:M:DM,C:M:b20m1place,C:M:DM,C:M:b20m2,C:M:DM"
    }
# [0.113, 3.753, 2.449, 6.078, 8.7]
# Recovered input frame: [ 0.807  1.126 -2.359 -0.703 -5.57 ]
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

    #print(inputs)
    if len(inputs) > 1:
        queue = list(inputs)
        while queue:
            cmd = queue.pop(0)
            cont = parse_input(cmd)
            if not cont:
                break
        return cont
    
    if inputs == ['']:
        return cont
    
    inp = inputs[0]

    # --- Preprocess slot and angle placeholders ---
    global last_slot_used, last_action_used
    # Preprocess slot and angle placeholders
    if "M:20:SLOT" in inp or "M:20:{LAST_M20_ANGLE_RAW}" in inp or "C:M:OpenSLOT" in inp:
        # Attempt to extract slot from prior context
        slot = last_slot_used
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
                last_commanded_angles[0] = slotAngle

            if "M:20:{LAST_M20_ANGLE_RAW}" in inp:
                inp = inp.replace("M:20:{LAST_M20_ANGLE_RAW}", f"M:20:{LAST_M20_ANGLE_RAW:.3f}")
            if "C:M:OpenSLOT" in inp and last_slot_used:
                orientation = None
                if last_slot_used == "left":
                    orientation = "LO"
                elif last_slot_used == "backward":
                    orientation = "BO"
                elif last_slot_used == "right":
                    orientation = "RO"
                if orientation:
                    inp = inp.replace("OpenSLOT", f"{orientation}")
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
        #print(packet)
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
            esp32.send(packet)
        else:
            print("wifi off")
        print(packet)
        # print("done")

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
    elif inp[0] == "W": # voxel gripper position
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
    elif inp[0] == "P": # Pause
        # This is the Pause command, use it to manually space out moves, or to have a chance to manually recover the robot between moves
        keepon = input("press a key to continue, or a cmd to add... ")
        if keepon == "Q":
            cont = parse_input("Q")
            return cont
        elif len(keepon)>=1:
            parse_input(keepon)
            parse_input("P")
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

                # Exact positions for the front feet

                # ===================== 200 =====================
                
                f200 = np.array([ 0.92, 0.313, -2.646, 0.098, -2.37 ])      # 
                f201 = np.array([ 0.73, -0.264, -2.529, 0.894, -2.368])     #
                f202 = np.array([ 0.839, -0.42, -2.293, 1.241, -2.378])     #
                                
                f203 = np.array([ 0.73, -0.47,  -1.896,  1.759, -2.368])    #   
                f204 = np.array([ 0.73, -0.24, -1.348,  2.0955, -2.368])    #   
                f205 = np.array([ 0.73, -0.44,   -1.08,   2.095, -2.368])   #   
                b200 = f200
                b201 = np.array([ 0.81,   0.86,  -2.498, -0.351, -2.378]) 

                b20m1 = f201
                b20m2 = np.array([ 0.84, -0.35, -2.287, 1.241, -2.379])
                b20m3 = f203
                b20m4 = f204
                b20m5 = f205
                # ===================== 400 =====================
                f400 = np.array([ 1.217, 0.733, -1.748, 0.588, -2.019 ]) 
                
                f401 = np.array([ 1.378, 0.52, -1.58, 1.019, -1.93]) 
                f402 = np.array([ 1.208, 0.382, -1.491, 1.199, -1.993]) 
                f403 = np.array([ 1.178,  0.328, -1.093,  1.697, -1.93]) 
                f404 = np.array([ 1.2, 0.77, -0.0257, 2.363, -1.955 ]) 
                f405 = np.array([ 1.178 ,  0.169, -0.545,  2.016, -1.93]) 
                b401 = np.array([ 1.201,  1.051, -1.564,  0.497, -1.92])
                b400 = f400
                
                b40m1 = f401
                b40m2 = np.array([ 1.194, 0.442, -1.398, 1.272, -1.927])
                b40m3 = f403
                b40m4 = np.array([1.185, 0.689, -0.2097, 2.216, -1.933 ])
                
                b40m5 = f405

                # ===================== LEFT =====================
                
                f220 = np.array([ 1.841, 0.511, -2.287, 0.295, -2.951])
                f221 = np.array([ 1.751,  0.129,  -2.147,   0.834, -2.903])
                f222 = np.array([ 1.751, -0.011, -1.995, 1.137, -2.92 ])
                f223 = np.array([ 1.919, -0.098, -1.695, 1.438, -2.77 ])
                f224 = np.array([ 1.713, 0.044, -1.127, 1.992, -2.977])
                f225 = np.array([ 1.779, -0.509, -1.232,  1.881, -2.905]) 

                b221 = np.array([ 1.751, 0.898,  -2.097,  0.089, -2.903])
                b220 = f220
                b22m1 = f221
                b22m2 = f222
                b22m3 = f223
                b22m4 = np.array([ 1.751, -0.122, -1.164,  2.022, -2.902])
                b22m5 = f225

                # ===================== RIGHT =====================
                

                f2m20 = np.array([ 0.321, 0.518, -2.296, 0.264, -1.367])
                f2m21 = np.array([ 0.2398,  0.129,  -2.147,   0.834,  -1.307 ])#
                f2m22 = np.array([ 0.273, -0.054, -1.941, 1.258, -1.353])#
                f2m23 = np.array([ 0.249, -0.098, -1.605,  1.636, -1.302])#
                f2m24 = np.array([ 0.25, 0.041, -1.111, 1.946, -1.289])#
                f2m25 = np.array([ 0.245, -0.568, -1.23, 1.875, -1.302])#

                b2m21 = np.array([ 0.242,  0.898,  -2.097,  0.089,  -1.294 ])
                b2m20 = f2m20   
                b2m2m1 = f2m21
                b2m2m2 = f2m22
                b2m2m3 = f2m23
                b2m2m4 = np.array([0.241, 0.017, -1.178, 2.003, -1.304])
                b2m2m5 = f2m25

             
                # ===================== 200 =====================
                f200p = f200
                f201p = np.array([ 0.668, -0.02, -2.646, 0.488, -2.571])
                f202p = np.array([ 0.732, -0.411, -2.453, 1.087, -2.405])
                f203p = np.array([ 0.821, -0.418, -1.995, 1.512, -2.37 ])
                f204p = np.array([ 0.728, -0.265, -1.2337, 2., -2.358 ])
                f205p = np.array([ 0.833, -0.437, -1.083, 2.099, -3.72 ])
                b200p = f200
                b201p  = np.array([ 0.919, 0.73, -2.646, -0.362, -2.367]) 
                b20m1p = np.array([ 0.82, -0.42, -2.496,  1.,  -2.37])
                b20m2p = np.array([ 0.73,  -0.699, -2.403, 1.265, -2.351])#b20m2  
                b20m3p = f203p  
                b20m4p = np.array([ 0.729, -0.41, -1.21,  2.25, -2.367])
                b20m5p = f205p

                # ===================== 400 =====================
                f400p = f400 
                f401p = np.array([ 1.285, 0.43, -1.944, 0.613, -1.969])
                f402p = f402  
                f403p = np.array([ 1.21, 0.181, -1.395, 1.438, -1.975])
                
                f404p = f404  
                f405p = np.array([ 1.162,  0.278, -0.31, 2.325, -1.922])  
                b400p = f400

                b401p = np.array([ 1.22, 0.98, -1.795, 0.25, -1.97 ])
                b40m1p = np.array([ 1.22, 0.279, -1.783, 0.896, -1.969])
                b40m2p = np.array([ 1.177, 0.397, -1.417, 1.273, -1.934]) 
                b40m3p = np.array([ 1.3, 0.348, -0.899, 1.699, -1.864])
                b40m4p = f404p 
                b40m5p = f405p


                f220p = f220  
                f221p = np.array([ 1.834,  0.231, -2.174,  0.65, -2.94 ]) 
                
                f222p = f222 
                f223p = f223 
                f224p = f224 
                f225p = np.array([ 1.712, -0.172, -1.009, 2.126, -2.9  ]) 

                b221p = np.array([ 1.92, 0.779, -2.259, 0.038, -2.871]) 
                b220p = b220 
                b22m1p = np.array([ 1.919, -0.1, -2.159, 0.957, -2.901]) 
                b22m2p = np.array([ 1.847, -0.121, -1.73,1.482, -2.882]) 
                b22m3p = np.array([ 1.817, -0.121, -1.455, 1.757, -2.863]) 
                b22m4p = b22m4 
                b22m5p = b22m5 

                f2m20p = f2m20
                f2m21p = np.array([ 0.32, 0.239, -2.245, 0.589, -1.37 ])
                f2m22p = f2m22 
                f2m23p = np.array([ 0.204, -0.022, -1.686, 1.534, -1.373])                 
                f2m24p = f2m24                 
                f2m25p = f2m25  

                b2m21p = np.array([ 0.321, 0.674, -2.394, -0.063, -1.37 ])
                b2m20p = b2m20   
                b2m2m1p = np.array([ 0.287, 0.049, -2.055,  0.856, -1.276])
                b2m2m2p = np.array([ 0.306, -0.1 , -1.76, 1.333,-1.265])
                b2m2m3p = np.array([ 0.405, 0.031, -1.423, 1.679, -1.2])
                b2m2m4p = b2m2m4
                b2m2m5p = b2m2m5

                # ============================= not used =============================

                f300 = np.array([ 1.039, 0.521, -2.138, 0.484, -2.087]) 
                f302 = np.array([ 1.073,  0.01, -1.874,  1.249, -2.04])
                f303 = np.array([ 1.164, -0.22,  -1.694,  1.408, -1.991])

                b301 = np.array([ 1.091,  0.918, -2.016,  0.194, -2.032])
                b302 = np.array([ 1.077,  1.313 , -1.823,  0.0, -2.057])

                f500 = np.array([ 1.3 ,  1.05, -1.05,  1.04, -1.85])
                f501 = np.array([ 1.306 ,  0.91, -0.922,  1.29, -1.83])
                f502 = np.array([ 1.31 ,  0.853, -0.598,  1.677, -1.83])

                b402 = np.array([ 1.175,  1.366, -1.373,  0.381, -1.933])

                # =============================  Helper for colliding moves =============================

            
                f301p = np.array([ 1.064, 0.178, -2.03, 0.9, -2.045])
                b301p = np.array([ 1.165, 0.984, -2.245, -0.149, -2.058])
                carryingb301p = np.array([ 1.158, 0.984, -2.094, -0.119, -2.05 ])
                b30m1p = np.array([ 1.075, 0.379, -1.999, 0.782, -2.048])

                # ============================= CARRYING POSITION  =============================
                carryingb200 = b200
                carryingb400 = b400

                carryingb220 = b220
                carryingb2m20 = b2m20

                carryingb20m2 = b20m2
                                # b201p  = np.array([ 0.919, 0.65,  -2.62, -0.225, -2.22 ]) 

                carryingb201p = np.array([ 1.117, 0.685, -2.628, -0.336, -2.163])
                carryingb401p = np.array([ 1.257, 0.88, -1.897, 0.151, -1.97 ])

                carryingb221p = np.array([ 1.922, 0.809, -2.17, 0.028, -2.876])
                carryingb2m21p = np.array([ 0.376, 0.855, -2.125, 0.016, -1.275])


                
                carryingb40m1p = np.array([ 1.199, 0.426, -1.624, 0.884, -1.942])
                carryingb30m1p = np.array([ 1.072, 0.19,  -2.062,  0.775, -2.054])
                carryingb20m1p = b20m1p

                carryingb40m3p = np.array([ 1.318, 0.134, -0.954, 1.633, -1.86])
                carryingb40m2p = b40m2p
    

                # ===================== PLACING =============================
                b020  = np.array([ 0.813,  0.315, -2.607, 0.245, -0.819]) # f2m1 1_1 ok
                b0m20 = np.array([ 0.813, 0.315, -2.607, 0.245, -3.94])# right 1_1 ok
                bm200r = np.array([ 0.881, 0.314, -2.646, 0.098, -5.471]) # OK
                bm200l = np.array([ 0.789, 0.296, -2.675,  0.183, 0.731 ]) # ok

                bm2m20 =  np.array([ 1.671, -0.045, -2.395, 0.65, -4.62 ])#  # right ok
                bm220 =  np.array([ 0.269, 0.33,  -2.394,  0.351,  0.238])   # left
                bm202r  = np.array([None, None, None, None, None])

                b2m20place = np.array( [0.251, 0.313 ,-2.254, 0.427, -1.342])
                b220place = np.array([ 1.859, 0.401, -2.186, 0.456, -2.906])


                b201placeUp = np.array([ 0.92, 0.98, -2.41, -0.461, -2.219])


                b201place = np.array([ 1.17, 0.73, -2.646, -0.363, -2.169]) # np.array([ 0.719 , 0.753, -2.619, -0.297, -2.493])
                b021place = np.array([ 0.882, 0.738, -2.56, -0.226, -0.87 ])# left  ok
                b0m21place = np.array([ 0.921, 0.679, -2.51, -0.126, -3.97 ])# right  ok 


                bm201rplace = np.array([ 1.119, 0.68, -2.62, -0.362, -5.273]) # OK
                bm201lplace = np.array([ 0.816,  0.73,  -2.62,  -0.225, 0.811 ])  #OK

                bm221place =  np.array([ 0.17, 0.68, -2.21 , 0.175,  0.131]) # left 
                bm2m21place = np.array([ 1.767,  0.58, -2.209 , 0.175, -4.57 ])#  # right 

                b2m21place = np.array([ 0.237,  0.619, -2.294 , 0.004, -1.38 ])
                b2m21placeOut = np.array([ 0.251,  0.38, -2.254, -0.025, -1.343])

                b221place  = np.array([ 1.771, 0.642, -2.193, 0.176, -2.983])
                b221placeOut  = np.array([ 1.939,  0.617, -2.005, 0.244, -2.684])

                b202place  = np.array([None, None, None, None, None])
                b203place  = np.array([None, None, None, None, None])
                b20m1place  = b20m1p
                b200place  = b200p
                bm203rplace = np.array([None, None, None, None, None])

                b021placeUp = np.array([ 0.77, 1.079, -2.41, -0.426, -0.769])
                b0m21placeUp = np.array([ 0.77, 1.08, -2.41, -0.425, -4.071])

                bm201rplaceUp = np.array([ 1.321, 1.082, -2.303, -0.427, -5.071]) # OK
                bm201lplaceUp = np.array([ 0.77, 1.079, -2.41,  -0.425, 0.731 ])  #OK
                bm221placeUp = np.array([ 0.246,  0.98,  -2.045, -0.125, 0.234])
                bm2m21placeUp = np.array([ 1.757,  0.98,  -2.045, -0.125, -4.561])

                b2m21placeUp = np.array([ 0.238, 0.779, -2.294, -0.125, -1.38 ])
                b221placeUp = np.array([ 1.771, 0.879, -2.28, -0.125, -2.982])

                carryingb202p = b201placeUp
                carryingb222p = b221placeUp
                carryingb2m22p = b2m21placeUp

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

                        "b201placeUp":b201placeUp,

                        "b201place":b201place,
                        "b021place":b021place,
                        "b0m21place":b0m21place,

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

                        "b021placeUp": b021placeUp, 
                        
                        "b0m21placeUp":b0m21placeUp,
                        "bm201rplaceUp":bm201rplaceUp,
                        "bm201lplaceUp":bm201lplaceUp,
                        "bm221placeUp":bm221placeUp,
                        "bm2m21placeUp":bm2m21placeUp,
                        "b2m21placeUp":b2m21placeUp,
                        "b221placeUp":b221placeUp,                       
                        "b2m21placeOut":b2m21placeOut,
                        "b221placeOut":b221placeOut,

                        "b2m20place":b2m20place,
                        "b220place":b220place,

                        "carryingb202p":carryingb202p,

                        "carryingb40m2p":carryingb40m2p,
                        "carryingb40m3p":carryingb40m3p,
                        "carryingb30m1p":carryingb30m1p,
                        "carryingb20m1p":carryingb20m1p,
                        "carryingb40m1p":carryingb40m1p,
                        "carryingb2m21p":carryingb2m21p,
                        "carryingb221p":carryingb221p,
                        "carryingb401p":carryingb401p,
                        "carryingb301p":carryingb301p,
                        "carryingb201p":carryingb201p,
                        "carryingb20m2":carryingb20m2,
                        "carryingb2m20":carryingb2m20,
                        "carryingb220":carryingb220,
                        "carryingb400":carryingb400,
                        "carryingb200":carryingb200,
                        "carryingb222p":carryingb222p,
                        "carryingb2m22p":carryingb2m22p,
                        
                        "f301p": f301p,
                        "b30m1p":b30m1p,
                        "b301p":b301p,
                        # UNUSED POSITIONS
                        "f300":f300,
                        "f303":f303,
                        # "f400inter":f400inter,
                        # "f500": f500, "f501": f501, "f502": f502,
                        # "b301": b301, "b302": b302,
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
                            max_wait_time = 3.5  # seconds
                            if inp[2] == "DMcarried":
                                max_wait_time = 3 #seconds
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
                                if (inp[2] == "DMcarried" and np.all(error < 0.03)):
                                    success = True
                                    break
                                elif np.all(error < 0.03):
                                    success = True
                                    break
                                time.sleep(0.1)

                            if success:
                                if inp[2] == "DM" or inp[2] == "DMplacing" or inp[2] == "DMcarried":
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
                        parse_input("M:25:0.35")
                        print("done")
                        # time.sleep(0.5)
                        # parse_input("M:25:0.68")
                    elif inp[2]=="C1":
                        # close gripper1
                        parse_input("M:25:1.3")
                        print("done")
                        # time.sleep(0.5)
                        # parse_input("M:25:1.48")
                    elif inp[2]=="O2":
                        # open gripper2 - hardcode.. bigger closes
                        parse_input("M:26:0.3")#was 0.7
                        print("done")
                    elif inp[2]=="C2":
                        # close gripper2
                        parse_input("M:26:1.05")
                        print("done")
                    elif inp[2]=="BO":
                        parse_input("M:27:1.3")
                    elif inp[2]=="BC":
                        parse_input("M:27:2.1")
                    elif inp[2]=="LO":
                        parse_input("V:27:1.4")
                    elif inp[2]=="LC":
                        parse_input("V:27:2.3")
                    elif inp[2] == "RC":
                        parse_input("W:27:1.2")
                    elif inp[2] == "RO":
                        parse_input("W:27:2.1")
                    # elif inp[2]=="O3":
                    #     #open gripper 3, voxel holder
                    #     parse_input("M:27:1.35")
                    #     # print("done")
                    # elif inp[2]=="C3":
                    #     #close gripper 3, voxel holder
                    #     parse_input("M:27:2.2")
                    #     # print("done")

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

###Single commands: CMD:Addr:DesVal or CMD:Addr. Multi-commands: C:CMD. Input: C:I
# [1.34, 3.003, 0.061, 4.872, 2.844]
# Recovered input frame: [ 0.73   0.247  3.609  0.184 -2.364]

# [1.34, 3.001, 6.22, 4.872, 2.844]
# Recovered input frame: [ 0.73   0.249 -2.55   0.184 -2.364]