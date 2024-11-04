import time
import threading
from pymavlink import mavutil

# Function to connect to the drone
def connect_drone(connection_string):
    drone = mavutil.mavlink_connection(connection_string)
    drone.wait_heartbeat()
    print(f"Connected to drone on {connection_string}")
    return drone

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
    print(f"Follower moving to latitude: {latitude}, longitude: {longitude}, altitude: {altitude}")

# Leader-follower function to send leader's position to the follower
def follow_leader(drone_leader, drone_follower, follow_distance=10):
    while True:
        # Receive the leader drone's position
        msg = drone_leader.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg:
            latitude = msg.lat / 1e7
            longitude = msg.lon / 1e7
            altitude = msg.relative_alt / 1e3
            print(f"Leader Position: {latitude}, {longitude}, {altitude}")

            # Command the follower drone to move to a position behind the leader
            # Adjust latitude/longitude slightly to maintain a following distance
            follow_latitude = latitude - (follow_distance / 111320)  # Adjust latitude
            follow_longitude = longitude

            move_to_position(drone_follower, follow_latitude, follow_longitude, altitude)
            print("Follower moving to leader position with offset.")
        time.sleep(1)  # Periodically update position

# Connect to the leader and follower drones
leader_connection = 'udp:127.0.0.1:14550'  # Connection address for leader drone
follower_connection = 'udp:127.0.0.1:14551'  # Connection address for follower drone

drone_leader = connect_drone(leader_connection)
drone_follower = connect_drone(follower_connection)

# Start leader-follower behavior in a separate thread
follow_thread = threading.Thread(target=follow_leader, args=(drone_leader, drone_follower))
follow_thread.start()