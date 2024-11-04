import asyncio
from mavsdk import System

async def main():
    # Connect to the leader drone
    leader_drone = System()
    await leader_drone.connect(system_address="udp://:14540")

    # Wait for the drone to connect
    print("Connecting to the leader drone...")
    async for state in leader_drone.core.connection_state():
        if state.is_connected:
            print("Leader drone connected!")
            break

    # Arm the drone
    print("Arming the leader drone...")
    await leader_drone.action.arm()

    # Takeoff
    print("Leader drone taking off...")
    await leader_drone.action.takeoff()

    # Wait for a while to let the drone takeoff
    await asyncio.sleep(10)

    # Land after takeoff
    print("Landing the leader drone...")
    await leader_drone.action.land()

# Run the main function
asyncio.run(main())