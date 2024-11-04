import asyncio
from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan
import math

async def log_data(drone):
    # Collect and Print the Log in real-time
    async for position in drone.telemetry.position():
        print(f"Current Location: latitude {position.latitude_deg},\tlongitude {position.longitude_deg},\taltitude: {position.relative_altitude_m}")

async def run():
    # Initialize and Connect the Drone System
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Connecting...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected!")
            break

    # Load the Current Location
    print("\nLoading the current location...")
    async for position in drone.telemetry.position():
        home_lat = position.latitude_deg
        home_lon = position.longitude_deg
        print(f"Original Location: latitude {home_lat}, longitude {home_lon}\n")
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
    await drone.mission.upload_mission(mission_plan)
    print("Mission Uploaded!")

    # Arm the Drone
    await drone.action.arm()
    print("Drone Armed!")

    # Start the Mission
    await drone.mission.start_mission()
    print("Mission Started!\n")

    asyncio.create_task(log_data(drone))

    # Wait the Mission Complete & Check the Success or Failure
    try:
        async for mission_progress in drone.mission.mission_progress():
            print(f"\nMission progress: {mission_progress.current}/{mission_progress.total}\n")
            if mission_progress.current == mission_progress.total:
                print("\nMission Complete!")
                break
            await asyncio.sleep(1)

        if await drone.mission.is_mission_finished():
            print("Mission finished successfully.")
        else:
            print("Mission failed to complete successfully...")

    except Exception as e:
        print(f"An error occured during the mission: {e}")
        return

    # Land
    print("\nLanding...\n")
    await drone.action.land()

asyncio.run(run())
