import asyncio
from mavsdk import System

async def get_leader_position(leader_drone):
    """ Continuously gets the leader drone's position """
    async for position in leader_drone.telemetry.position():
        print(f"Leader Position - Latitude: {position.latitude_deg}, Longitude: {position.longitude_deg}, Altitude: {position.relative_altitude_m}")
        return position  # Return the current position

async def move_follower_to_position(follower_drone, latitude, longitude, altitude):
    """ Commands the follower drone to move to a specific position """
    await follower_drone.action.goto_location(latitude, longitude, altitude, 0)
    print(f"Follower moving to Latitude: {latitude}, Longitude: {longitude}, Altitude: {altitude}")

async def follow_leader(leader_drone, follower_drone, follow_distance=10):
    """ Main function to implement leader-follower algorithm """
    while True:
        # Get leader's position
        leader_position = await get_leader_position(leader_drone)

        # Calculate the target position for the follower to maintain the follow distance
        follower_latitude = leader_position.latitude_deg - (follow_distance / 111320)  # Adjust latitude for follow distance
        follower_longitude = leader_position.longitude_deg

        # Command the follower to move to the calculated position
        await move_follower_to_position(follower_drone, follower_latitude, follower_longitude, leader_position.relative_altitude_m)

        # Wait a short interval before updating the follower's position
        await asyncio.sleep(1)

async def main():
    # Initialize and connect to the leader and follower drones
    leader_drone = System()
    follower_drone = System()

    await leader_drone.connect(system_address="udp://:14540")
    await follower_drone.connect(system_address="udp://:14541")

    print("Connecting to leader and follower drones...")
    async for state in leader_drone.core.connection_state():
        if state.is_connected:
            print("Leader drone connected!")
            break

    async for state in follower_drone.core.connection_state():
        if state.is_connected:
            print("Follower drone connected!")
            break

    # Arm both drones
    print("Arming leader and follower drones...")
    await leader_drone.action.arm()
    await follower_drone.action.arm()

    # Verify arm status
    print("Verifying arm status...")
    async for is_armed_leader in leader_drone.telemetry.armed():
        print(f"Leader drone armed status: {is_armed_leader}")
        break

    async for is_armed_follower in follower_drone.telemetry.armed():
        print(f"Follower drone armed status: {is_armed_follower}")
        break

    # Wait until both drones are armed
    armed_leader = False
    armed_follower = False

    while not (armed_leader and armed_follower):
        async for is_armed_leader in leader_drone.telemetry.armed():
            armed_leader = is_armed_leader
            break
        async for is_armed_follower in follower_drone.telemetry.armed():
            armed_follower = is_armed_follower
            break
        await asyncio.sleep(0.5)

    print("Both drones are armed. Taking off the leader drone to 10 meters altitude...")

    # Leader drone takes off to 10 meters
    await leader_drone.action.takeoff()
    await asyncio.sleep(5)  # Wait for the drone to reach the altitude

    # Hover at 10 meters altitude for a few seconds
    await asyncio.sleep(5)
    print("Leader drone reached 10 meters altitude. Preparing to land...")

    # Land the leader drone
    await leader_drone.action.land()
    print("Leader drone is landing...")

    # Wait until the leader drone has landed
    async for in_air in leader_drone.telemetry.in_air():
        if not in_air:
            print("Leader drone has landed.")
            break

    # Start the leader-follower algorithm
    print("Starting leader-follower algorithm...")
    await follow_leader(leader_drone, follower_drone)

# Run the main function
asyncio.run(main())