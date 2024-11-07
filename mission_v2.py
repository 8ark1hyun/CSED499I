import asyncio
from mavsdk import System

async def connect_drone(drone, address):
    """ Connects to a drone with a specified address and verifies the connection """
    await drone.connect(system_address=address)
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"Drone connected at {address}!")
            break

async def arm_drones(leader_drone, follower_drone):
    """ Arms both leader and follower drones concurrently """
    await asyncio.gather(
        leader_drone.action.arm(),
        follower_drone.action.arm()
    )
    print("Both drones are armed.")

async def takeoff_drones(leader_drone, follower_drone, altitude=10):
    """ Takes off both leader and follower drones to the specified altitude concurrently """
    await asyncio.gather(
        leader_drone.action.takeoff(),
        follower_drone.action.takeoff()
    )
    print(f"Both drones are taking off to {altitude} meters.")

async def land_drones(leader_drone, follower_drone):
    """ Lands both leader and follower drones concurrently """
    await asyncio.gather(
        leader_drone.action.land(),
        follower_drone.action.land()
    )
    print("Both drones are landing.")

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

    print("Connecting to leader drone...")
    await connect_drone(leader_drone, "udp://:14540")

    print("Connecting to follower drone...")
    await connect_drone(follower_drone, "udp://:14541")

    # Arm both drones concurrently
    await arm_drones(leader_drone, follower_drone)

    """
    # Verify arm status
    print("Verifying arm status...")
    armed_leader = False
    armed_follower = False

    while not (armed_leader and armed_follower):
        async for is_armed_leader in leader_drone.telemetry.armed():
            armed_leader = is_armed_leader
            print(f"Leader drone armed status: {is_armed_leader}")
            break

        async for is_armed_follower in follower_drone.telemetry.armed():
            armed_follower = is_armed_follower
            print(f"Follower drone armed status: {is_armed_follower}")
            break
        await asyncio.sleep(0.5)


    print("Both drones are armed. Taking off the leader drone to 10 meters altitude...")

    await leader_drone.action.takeoff()
    await asyncio.sleep(5)
    await follower_drone.action.takeoff()

    await asyncio.sleep(5)
    print("Drones reached 10 meters altitude. Preparing to land...")

    await leader_drone.action.land()
    await asyncio.sleep(5)
    await follower_drone.action.land()
    print("Drones are landing...")
    """

    # Take off both drones concurrently to an altitude of 10 meters
    await takeoff_drones(leader_drone, follower_drone, altitude=10)
    await asyncio.sleep(5)

    # Land both drones concurrently
    await land_drones(leader_drone, follower_drone)
    
    # Start the leader-follower algorithm
    # print("Starting leader-follower algorithm...")
    # await follow_leader(leader_drone, follower_drone)

# Run the main function
asyncio.run(main())