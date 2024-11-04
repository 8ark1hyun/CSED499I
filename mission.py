import asyncio
from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan
import math
import time

async def log_data(drone, label):
    # Collect and Print the Log in real-time
    async for position in drone.telemetry.position():
        print(f"[{label}] Current Location: latitude {position.latitude_deg:.6f},\tlongitude {position.longitude_deg:.6f},\taltitude: {position.relative_altitude_m:.6f}")

async def leader_follower_run():
    # Initialize and Connect the Drone Systems
    leader = System()
    follower = System()

    await leader.connect(system_address="udp://:14540")
    await follower.connect(system_address="udp://:14541")

    print("Connecting Leader...")
    async for state in leader.core.connection_state():
        if state.is_connected:
            print("Leader Connected!")
            break

    print("Connecting Follower...")
    async for state in follower.core.connection_state():
        if state.is_connected:
            print("Follower Connected!")
            break
            
    # Load the Original Location of Leader
    print("\nLoading the Original Location of Leader...")
    async for position in leader.telemetry.position():
        home_lat = position.latitude_deg
        home_lon = position.longitude_deg
        print(f"Original Location of Leader: latitude {home_lat:.6f},\tlongitude {home_lon:.6f}\n")
        break

    # Calculate the offset
    offset_meters = 30
    lat_offset = offset_meters / 111320
    lon_offset = offset_meters / (111320 * math.cos(math.radians(home_lat)))

    # Mission
    mission_items = [
        MissionItem(home_lat, home_lon, 10, 5, True, float('nan'), float('nan'),
                    MissionItem.CameraAction.NONE, float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), vehicle_action=MissionItem.VehicleAction.NONE),
        MissionItem(home_lat, home_lon + lon_offset, 10, 5, True, float('nan'), float('nan'),
                    MissionItem.CameraAction.NONE, float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), vehicle_action=MissionItem.VehicleAction.NONE),
        MissionItem(home_lat + lat_offset, home_lon + lon_offset, 10, 5, True, float('nan'), float('nan'),
                    MissionItem.CameraAction.NONE, float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), vehicle_action=MissionItem.VehicleAction.NONE),
        MissionItem(home_lat + lat_offset, home_lon, 10, 5, True, float('nan'), float('nan'),
                    MissionItem.CameraAction.NONE, float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), vehicle_action=MissionItem.VehicleAction.NONE),
        MissionItem(home_lat, home_lon, 10, 5, True, float('nan'), float('nan'),
                    MissionItem.CameraAction.NONE, float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), vehicle_action=MissionItem.VehicleAction.NONE),
    ]

    mission_plan = MissionPlan(mission_items)

    # Upload the Mission
    await leader.mission.upload_mission(mission_plan)
    print("Leader Mission Uploaded!")

    # Arm the Leader Drone
    await leader.action.arm()
    print("Leader Armed!")

    # Start the Mission
    await leader.mission.start_mission()
    print("Leader Mission Started!\n")

    asyncio.create_task(log_data(leader, "Leader"))

    # Arm the Follower Drone
    await follower.action.arm()
    print("Follower Armed!")

    #
    last_follower_update = time.time()
    follow_interval = 1

    # Wait the Mission Complete & Check the Success or Failure
    try:
        async for mission_progress in leader.mission.mission_progress():
            print(f"\nLeader Mission progress: {mission_progress.current}/{mission_progress.total}\n")
            if mission_progress.current == mission_progress.total:
                print("\nLeader Mission Complete!")
                break
            await asyncio.sleep(1)

            if time.time() - last_follower_update > follow_interval:
                async for position in leader.telemetry.position():
                    await follower.action.goto_location(
                        position.latitude_deg,
                        position.longitude_deg,
                        position.relative_altitude_m,
                        0
                    )
                    last_follower_update = time.time()
                    break

        if await leader.mission.is_mission_finished():
            print("Leader Mission finished successfully.")
        else:
            print("Leader Mission failed to complete successfully...")

    except Exception as e:
        print(f"An error occured during the mission: {e}")
        return

    # Land
    print("\nLanding Leader...\n")
    await leader.action.land()
    print("\nLanding Follower...\n")
    await follower.action.land()

asyncio.run(leader_follower_run())
