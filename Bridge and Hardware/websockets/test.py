import asyncio
import websockets

async def listen_simulator(websocket):
    print("🌐 Connected to Digital Twin")
    async for message in websocket:
        print("📨 Received:", message)
        # You can parse or log here
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
            # for cmd in subactions:
            #     parse_input(cmd)

        except Exception as e:
            print("❌ Error parsing WebMvt message:", e)


async def run_server():
    async with websockets.serve(listen_simulator, "0.0.0.0", 8765):
        print("🛰️ Listening on ws://0.0.0.0:8765")
        await asyncio.Future()  # run forever


def generate_movement_sequence(relative_pos, front_delta, back_delta, switch):
    """
    Compute sub-poses to move one leg followed by the other using a generic rule.
    """

    def pose_from_vector(leg, vec, is_p=False, switch=False):
        x, y, z = map(int, vec)

        def format_dim(val):
            return f"m{abs(val)}" if val < 0 else str(val)

        # For Y only, apply mirroring logic if switch == True
        y_switched = -y if switch else y

        return f"C:M:{leg}{format_dim(x)}{format_dim(y_switched)}{format_dim(z)}{'p' if is_p else ''}"


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

            path.append(pose_from_vector(leg, mid_vec1, True, switch))
            path.append("C:M:DM")

            if mid_vect3 is not None:
                path.append(pose_from_vector(leg, mid_vect3, True, switch))
                path.append("C:M:DM")

            path.append(pose_from_vector(leg, mid_vec2, True, switch))
            path.append("C:M:DM")
        else:
            if z1 > z0:
                for z in range(z0 + 1, z1 + 2):
                    # print(f"🦿a Interpolating Z from {z0} to {z1+1} for leg {leg}")
                    mid_vec = (start[0], start[1], z)
                    path.append(pose_from_vector(leg, mid_vec, True, switch))
                    path.append("C:M:DM")
                mid_vec2 = (end[0], end[1], z1 + 1)
                path.append(pose_from_vector(leg, mid_vec2, True, switch))
                path.append("C:M:DM")
            else:
                # print(f" aadzad za Interpolating Z down from {z0 + 1} to {z1} for leg {leg}")
                mid_vec2 = (start[0], start[1], z0 + 1)
                path.append(pose_from_vector(leg, mid_vec2, True, switch))
                path.append("C:M:DM")
                for z in range(z0 + 1, z1, -1):
                    mid_vec = (end[0], end[1], z)
                    path.append(pose_from_vector(leg, mid_vec, True, switch))
                    path.append("C:M:DM")
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
    sequence.append(pose_from_vector(first_leg, endPos, False, switch))
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
    sequence.append(pose_from_vector(second_leg, endPos2, False, switch))
    sequence.append("C:M:DM") 

    return sequence

if __name__ == "__main__":
    asyncio.run(run_server())
