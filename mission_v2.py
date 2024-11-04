import time
import threading
from pymavlink import mavutil

# Function to connect to the drone
def connect_drone(connection_string):
    drone = mavutil.mavlink_connection(connection_string)
    drone.wait_heartbeat()
    print(f"Connected to drone on {connection_string}")
    return drone

# Function to set the drone to ARM state
def arm_drone(drone):
    drone.mav.command_long_send(
        drone.target_system, 
        drone.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, # ARM/Disarm command
        0,
        1,  # 1 to ARM, 0 to Disarm
        0, 0, 0, 0, 0, 0
    )
    print("Drone armed!")

# Function to command the drone to take off
def takeoff(drone, altitude):
    drone.mav.command_long_send(
        drone.target_system, 
        drone.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, # Takeoff command
        0,
        0, 0, 0, 0, 0, 0, altitude  # Target altitude
    )
    print(f"Drone taking off to {altitude} meters")

# Function to move the drone to a specific global position
def move_to_position(drone, latitude, longitude, altitude):
    drone.mav.set_position_target_global_int_send(
        0,
        drone.target_system,
        drone.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b110111111000,  # Set position control flag for x, y, z
        int(latitude * 1e7),  # Latitude
        int(longitude * 1e7),  # Longitude
        altitude,              # Relative altitude
        0, 0, 0,               # Velocity (not used)
        0, 0, 0,               # Acceleration (not used)
        0, 0                   # Yaw (not used)
    )
    print(f"Moving to latitude: {latitude}, longitude: {longitude}, altitude: {altitude}")
    time.sleep(5)  # Wait for the drone to reach the position

# Function to execute a mission of moving in a square pattern and landing
def execute_mission(drone):
    # Get current position as home position
    msg = drone.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    home_lat = msg.lat / 1e7
    home_lon = msg.lon / 1e7
    altitude = 10  # Target altitude in meters

    # Calculate offsets for 10 meters in each direction
    offset_m = 10  # 10 meter offset
    lat_offset = offset_m / 111320  # Latitude offset (meters to degrees)
    lon_offset = offset_m / (111320 * abs(math.cos(math.radians(home_lat))))  # Longitude offset

    # Take off
    takeoff(drone, altitude)

    # Move east by 10 meters
    move_to_position(drone, home_lat, home_lon + lon_offset, altitude)

    # Move north by 10 meters
    move_to_position(drone, home_lat + lat_offset, home_lon + lon_offset, altitude)

    # Move west by 10 meters
    move_to_position(drone, home_lat + lat_offset, home_lon, altitude)

    # Move south by 10 meters, returning to the starting point
    move_to_position(drone, home_lat, home_lon, altitude)

    # Land the drone
    drone.mav.command_long_send(
        drone.target_system,
        drone.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,
        0, 0, 0, 0, 0, 0, home_lat  # Land at home position
    )
    print("Landing initiated")

# Leader-follower function to follow leader's position
def follow_leader(drone_leader, drone_follower):
    while True:
        # Receive the leader drone's position
        msg = drone_leader.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg:
            latitude = msg.lat / 1e7
            longitude = msg.lon / 1e7
            altitude = msg.relative_alt / 1e3
            print(f"Leader Position: {latitude}, {longitude}, {altitude}")

            # Command the follower drone to move to the leader's position
            move_to_position(drone_follower, latitude, longitude, altitude)
            print("Follower moving to leader position.")
        time.sleep(1)  # Periodically update position

# Connect to the leader and follower drones
leader_connection = 'udp:127.0.0.1:14550'  # Connection address for leader drone
follower_connection = 'udp:127.0.0.1:14551'  # Connection address for follower drone

# Connect to leader and follower drones
drone_leader = mavutil.mavlink_connection(leader_connection)
drone_follower = mavutil.mavlink_connection(follower_connection)

drone_leader.wait_heartbeat()
print("Connected to leader drone")
drone_follower.wait_heartbeat()
print("Connected to follower drone")

# Set both the leader and follower drones to ARM state
arm_drone(drone_leader)
arm_drone(drone_follower)

# Execute mission for leader drone in a separate thread
mission_thread = threading.Thread(target=execute_mission, args=(drone_leader,))
mission_thread.start()

# Start leader-follower behavior in another thread
follow_thread = threading.Thread(target=follow_leader, args=(drone_leader, drone_follower))
follow_thread.start()