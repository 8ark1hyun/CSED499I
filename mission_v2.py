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

    # Start the leader-follower algorithm
    print("Starting leader-follower algorithm...")
    await follow_leader(leader_drone, follower_drone)

# Run the main function
asyncio.run(main())