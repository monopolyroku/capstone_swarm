# Description: Supervisor for a drone simulation.
#              Sends the coordinates and orientations of each drone to the drones via an emitter.

from controller import Supervisor, Emitter
import math

# Constants
DRONES = 3  # Number of drones
TIME_STEP = 64  # Simulation time step
MISSION_DURATION = 5 * 60  # Mission duration: 5 minutes

# Initialize the supervisor
supervisor = Supervisor()

# Get the emitter device
emitter = supervisor.getDevice("emitter")

# Drone names and initial positions/orientations
drone_names = ["Drone1", "Drone2", "Drone3"]
drone_initial_translation = [
    [0.5, 0.5, 1.0],  # Initial position for Drone1
    [-0.5, 0.5, 1.0], # Initial position for Drone2
    [0.0, -0.5, 1.0]  # Initial position for Drone3
]
drone_initial_rotation = [
    [0, 0, 1, 0],  # Initial orientation for Drone1
    [0, 0, 1, 0],  # Initial orientation for Drone2
    [0, 0, 1, 0]   # Initial orientation for Drone3
]

# Retrieve drone nodes and fields
drone_translation_fields = []
drone_rotation_fields = []
for name in drone_names:
    node = supervisor.getFromDef(name)
    drone_translation_fields.append(node.getField("translation"))
    drone_rotation_fields.append(node.getField("rotation"))

# Initialize mission timer and reset timer
mission_timer = MISSION_DURATION
reset_timer = 0

# Main simulation loop
while supervisor.step(TIME_STEP) != -1:
    # Prepare packet to send drone positions
    packet = []

    # Retrieve current positions and orientations of drones
    for i in range(DRONES):
        translation = drone_translation_fields[i].getSFVec3f()
        rotation = drone_rotation_fields[i].getSFRotation()

        # Append drone position (X, Y, Z) to the packet
        packet.extend([translation[0], translation[1], translation[2]])
    
    # Convert packet to space separated string and encode to bytes 
    # Send drone positions to all drones via emitter
    packet_str = ' '.join(map(str, packet))
    emitter.send(packet_str.encode('utf-8'))

    # Decrease mission timer
    mission_timer -= TIME_STEP / 1000
    if mission_timer <= 0:
        mission_timer = MISSION_DURATION  # Restart mission timer
        reset_timer = 3  # Activate reset timer (3 seconds)

    # Reset drones to initial positions after mission ends
    if reset_timer > 0:
        reset_timer -= TIME_STEP / 1000
        if reset_timer <= 0:
            for i in range(DRONES):
                drone_translation_fields[i].setSFVec3f(drone_initial_translation[i])
                drone_rotation_fields[i].setSFRotation(drone_initial_rotation[i])