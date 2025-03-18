#
# Author: @monopolyroku
#
# Description: Code for autonomous control of three drones with wall following and mapping
#

from controller import Robot, Keyboard
import random
from pid_controller import pid_velocity_fixed_height_controller
from wall_following import WallFollowing
from math import cos, sin, pi, atan2

# function to evaluate the position of the drone
def evaluate_position(position):
    # assuming the target is the kitchen area of the apartment
    target = [-1.3, -3, 1.5]
    distance = sum((p - t) ** 2 for p, t in zip(position, target))
    return -distance  # Negative to reduce distance

# BCO parameters
class BCOParams:
    def __init__(self):
        self.num_employed_bees = 3  # No. of employed bees 
        self.max_trials = 5  # Max no. of trials before abandoning a food source
        self.search_radius = 2.0  # Max distance for local search

# BCO helper functions
def calculate_fitness(position):
    """Calculate fitness value for a given position"""
    evaluation = evaluate_position(position)
    return 1 / (1 + abs(evaluation)) if evaluation < 0 else 1 + evaluation

def generate_new_position(current_pos, best_pos):
    """Generate a new position based on current and best positions"""
    # This will be implemented later when we want to use BCO for movement
    return current_pos

def abandon_position(position):
    """Generate a new random position when current one is abandoned"""
    # This will be implemented later when we want to use BCO for movement
    return position

# Add this variable near the other initialization variables
program_start_time = 0.0  # Will be set to the actual start time in the first loop iteration

# Modify the is_position_valid function to include a time check
def is_position_valid(position, current_time):
    """Check if a position is within reasonable bounds, only after 10 seconds from start"""
    # Don't check positions during the first 10 seconds
    if current_time - program_start_time < 10.0:
        return True
        
    if position is None:
        return False
        
    x, y, z = position
    return (POSITION_BOUNDS['x_min'] <= x <= POSITION_BOUNDS['x_max'] and
            POSITION_BOUNDS['y_min'] <= y <= POSITION_BOUNDS['y_max'] and
            POSITION_BOUNDS['z_min'] <= z <= POSITION_BOUNDS['z_max'])

# Define reasonable bounds for drone positions
POSITION_BOUNDS = {
    'x_min': -20.0, 'x_max': 20.0,
    'y_min': -20.0, 'y_max': 20.0,
    'z_min': 0.5, 'z_max': 1.5
}

# Define reasonable bounds for drone orientation (in radians)
ORIENTATION_BOUNDS = {
    'roll_max': 0.35,  # About 20 degrees
    'pitch_max': 0.35,  # About 20 degrees
}

# Function to check if a drone's orientation is valid
def is_orientation_valid(roll, pitch):
    """Check if a drone's orientation is within reasonable bounds"""
    return (abs(roll) <= ORIENTATION_BOUNDS['roll_max'] and 
            abs(pitch) <= ORIENTATION_BOUNDS['pitch_max'])

# Modify the check_drone_failures function to pass the current time
def check_drone_failures(current_time):
    """Check if any drones have failed and record obstacle positions"""
    global obstacle_positions, drone_orientations
    
    for i in range(3):
        position = shared_positions[i]
        
        # Skip if the drone is already marked as failed
        if not drone_active[i]:
            continue
            
        # Check position validity (after 10 seconds)
        position_valid = is_position_valid(position, current_time)
        
        # Check orientation validity if we have orientation data
        orientation_valid = True
        if i in drone_orientations:
            roll, pitch = drone_orientations[i][:2]  # Get roll and pitch
            orientation_valid = is_orientation_valid(roll, pitch)
            
            # Print warning if orientation is becoming unstable but not yet invalid
            if orientation_valid and (abs(roll) > ORIENTATION_BOUNDS['roll_max'] * 0.7 or 
                                     abs(pitch) > ORIENTATION_BOUNDS['pitch_max'] * 0.7):
                print(f"Warning: Drone {i+1} orientation approaching limits. Roll: {roll:.4f}, Pitch: {pitch:.4f}")
        
        # If either position or orientation is invalid, mark the drone as failed
        if not position_valid or not orientation_valid:
            # Get the last valid position before failure
            last_valid_position = None
            
            # If the current position is completely invalid, use the previous valid position
            if not position_valid and (position is None or any(abs(coord) > 100 for coord in position)):
                # Find the last valid position from shared history
                for j in range(len(position_history[i]) - 1, -1, -1):
                    if is_position_valid(position_history[i][j], current_time):
                        last_valid_position = position_history[i][j]
                        break
            else:
                # The current position might be just outside bounds but still useful
                last_valid_position = position
            
            if last_valid_position is not None:
                # Add this position to obstacles if it's not too close to existing ones
                is_new_obstacle = True
                for obs in obstacle_positions:
                    distance = sum((last_valid_position[j] - obs[j])**2 for j in range(3))**0.5
                    if distance < obstacle_radius:
                        is_new_obstacle = False
                        break
                
                if is_new_obstacle:
                    obstacle_positions.append(last_valid_position)
                    print(f"New obstacle detected at {last_valid_position}")
            
            drone_active[i] = False
            
            # Print the specific reason for failure
            if not position_valid and not orientation_valid:
                print(f"Drone {i+1} has failed! Invalid position and orientation.")
            elif not position_valid:
                print(f"Drone {i+1} has failed! Invalid position: {position}")
            else:
                roll, pitch = drone_orientations[i][:2]
                print(f"Drone {i+1} has failed! Disoriented with roll: {roll:.4f}, pitch: {pitch:.4f}")
    
    # Return the number of active drones
    return sum(drone_active)

def is_near_obstacle(position, current_time, min_distance=None):
    """Check if a position is too close to a known obstacle"""
    if not obstacle_positions or not is_position_valid(position, current_time):
        return False
        
    if min_distance is None:
        min_distance = obstacle_radius
        
    for obs in obstacle_positions:
        distance = sum((position[j] - obs[j])**2 for j in range(3))**0.5
        if distance < min_distance:
            return True
    
    return False

def adjust_wall_following_for_obstacles(cmd_vel_x, cmd_vel_y, current_pos, current_time):
    """Adjust wall following commands to avoid known obstacles"""
    # If no obstacles, return original commands
    if not obstacle_positions:
        return cmd_vel_x, cmd_vel_y
    
    # Look ahead to predict future positions at different time steps
    look_ahead_times = [0.5, 1.0, 1.5]  # Look ahead 0.5, 1.0, and 1.5 seconds
    danger_level = 0
    nearest_obstacle = None
    min_distance = float('inf')
    
    # Check multiple points along the predicted path
    for t in look_ahead_times:
        next_pos = [
            current_pos[0] + cmd_vel_x * t,
            current_pos[1] + cmd_vel_y * t,
            current_pos[2]
        ]
        
        # Find the nearest obstacle to this predicted position
        for obs in obstacle_positions:
            distance = sum((next_pos[j] - obs[j])**2 for j in range(3))**0.5
            
            # If this is closer than our current nearest obstacle
            if distance < min_distance:
                min_distance = distance
                nearest_obstacle = obs
                
                # Determine danger level based on which look-ahead time detected the obstacle
                if t == look_ahead_times[0]:
                    danger_level = 3  # Immediate danger
                elif t == look_ahead_times[1]:
                    danger_level = 2  # Medium danger
                else:
                    danger_level = 1  # Low danger
    
    # If no obstacle is near enough to be a concern
    if nearest_obstacle is None or min_distance > obstacle_radius * 1.5:
        return cmd_vel_x, cmd_vel_y
    
    print(f"Obstacle detected at distance {min_distance:.2f}m, danger level: {danger_level}")
    
    # Calculate avoidance vector (direction from obstacle to drone)
    avoidance_vector = [current_pos[j] - nearest_obstacle[j] for j in range(2)]  # Just x and y
    
    # Normalize the avoidance vector
    magnitude = (avoidance_vector[0]**2 + avoidance_vector[1]**2)**0.5
    if magnitude < 0.001:  # Avoid division by zero
        # If we're practically on top of the obstacle, move in a random direction
        avoidance_vector = [1.0, 0.0]  # Default to moving along x-axis
    else:
        avoidance_vector = [v / magnitude for v in avoidance_vector]
    
    # Calculate dot product to see if we're moving toward the obstacle
    movement_vector = [cmd_vel_x, cmd_vel_y]
    movement_magnitude = (movement_vector[0]**2 + movement_vector[1]**2)**0.5
    
    if movement_magnitude < 0.001:  # If barely moving
        dot_product = 0
    else:
        normalized_movement = [v / movement_magnitude for v in movement_vector]
        dot_product = -sum(a*b for a, b in zip(normalized_movement, avoidance_vector))
    
    # Adjust velocity based on danger level and movement direction
    if danger_level == 3:
        # Immediate danger - strong avoidance
        avoidance_strength = 0.5
        # Reduce forward speed significantly
        speed_reduction = 0.3
    elif danger_level == 2:
        # Medium danger - moderate avoidance
        avoidance_strength = 0.3
        # Reduce forward speed moderately
        speed_reduction = 0.6
    else:
        # Low danger - gentle avoidance
        avoidance_strength = 0.2
        # Reduce forward speed slightly
        speed_reduction = 0.8
    
    # If we're moving toward the obstacle (positive dot product), apply stronger avoidance
    if dot_product > 0:
        avoidance_strength *= (1 + dot_product)
    
    # Calculate adjusted velocities
    adjusted_cmd_vel_x = cmd_vel_x * speed_reduction
    adjusted_cmd_vel_y = cmd_vel_y + avoidance_vector[1] * avoidance_strength
    
    # Add some of the avoidance vector to the x component as well
    adjusted_cmd_vel_x += avoidance_vector[0] * avoidance_strength
    
    print(f"Adjusting velocity from [{cmd_vel_x:.2f}, {cmd_vel_y:.2f}] to [{adjusted_cmd_vel_x:.2f}, {adjusted_cmd_vel_y:.2f}]")
    
    return adjusted_cmd_vel_x, adjusted_cmd_vel_y

# Add a function to find a safe path around obstacles
def find_safe_path(current_pos, target_pos, current_time):
    """Find a safe path from current position to target position, avoiding obstacles"""
    if not obstacle_positions:
        return target_pos  # No obstacles, go directly to target
    
    # Check if direct path to target is safe
    if not is_near_obstacle([
        (current_pos[0] + target_pos[0]) / 2,  # Midpoint between current and target
        (current_pos[1] + target_pos[1]) / 2,
        current_pos[2]
    ], current_time, obstacle_radius * 1.5):
        return target_pos  # Direct path is safe
    
    # Find the nearest obstacle to the path
    nearest_obstacle = None
    min_distance = float('inf')
    
    # Check distance from each obstacle to the line segment between current and target
    for obs in obstacle_positions:
        # Calculate distance from obstacle to line segment
        # Using the formula for distance from point to line segment
        line_dir = [target_pos[j] - current_pos[j] for j in range(2)]
        line_length = (line_dir[0]**2 + line_dir[1]**2)**0.5
        
        if line_length < 0.001:  # If current and target are very close
            distance = sum((current_pos[j] - obs[j])**2 for j in range(2))**0.5
        else:
            # Normalize line direction
            line_dir = [d / line_length for d in line_dir]
            
            # Vector from current position to obstacle
            to_obs = [obs[j] - current_pos[j] for j in range(2)]
            
            # Project to_obs onto line_dir
            projection = sum(to_obs[j] * line_dir[j] for j in range(2))
            
            # Clamp projection to line segment
            projection = max(0, min(line_length, projection))
            
            # Find closest point on line segment
            closest_point = [
                current_pos[0] + line_dir[0] * projection,
                current_pos[1] + line_dir[1] * projection
            ]
            
            # Calculate distance from obstacle to closest point
            distance = sum((closest_point[j] - obs[j])**2 for j in range(2))**0.5
        
        if distance < min_distance:
            min_distance = distance
            nearest_obstacle = obs
    
    if nearest_obstacle is None or min_distance > obstacle_radius * 1.5:
        return target_pos  # No obstacles close enough to the path
    
    # Calculate waypoint to go around the obstacle
    # Create a vector perpendicular to the line from current to target
    line_dir = [target_pos[j] - current_pos[j] for j in range(2)]
    line_length = (line_dir[0]**2 + line_dir[1]**2)**0.5
    
    if line_length < 0.001:  # If current and target are very close
        # Move in a consistent direction for all drones
        perp_vector = [1.0, 0.0]  # Right
    else:
        # Normalize line direction
        line_dir = [d / line_length for d in line_dir]
        
        # Create perpendicular vector (rotate 90 degrees clockwise)
        # All drones will take the same side of obstacles
        perp_vector = [-line_dir[1], line_dir[0]]
    
    # Calculate waypoint by moving perpendicular to the path
    # The distance is based on the obstacle radius plus a safety margin
    safe_distance = obstacle_radius * 2.0
    
    # Vector from obstacle to the line
    obs_to_line = [
        nearest_obstacle[0] - current_pos[0],
        nearest_obstacle[1] - current_pos[1]
    ]
    
    # Determine which side of the line the obstacle is on
    # by taking the dot product with the perpendicular vector
    side = sum(obs_to_line[j] * perp_vector[j] for j in range(2))
    
    # If the obstacle is on the same side we're planning to go around,
    # flip the perpendicular vector
    if side > 0:
        perp_vector = [-perp_vector[0], -perp_vector[1]]
    
    # Calculate waypoint
    waypoint = [
        nearest_obstacle[0] + perp_vector[0] * safe_distance,
        nearest_obstacle[1] + perp_vector[1] * safe_distance,
        current_pos[2]  # Keep the same height
    ]
    
    # Ensure the waypoint itself is not too close to any obstacle
    if is_near_obstacle(waypoint, current_time, obstacle_radius * 1.5):
        # If waypoint is too close to an obstacle, move it further out
        waypoint = [
            nearest_obstacle[0] + perp_vector[0] * safe_distance * 2.0,
            nearest_obstacle[1] + perp_vector[1] * safe_distance * 2.0,
            current_pos[2]
        ]
    
    print(f"Drone {drone_id}: Created safe waypoint at {waypoint} to avoid obstacle at {nearest_obstacle}")
    return waypoint

# Visualize obstacles for debugging
def print_obstacle_map():
    """Print a simple 2D map of obstacles for debugging"""
    if not obstacle_positions:
        print("No obstacles detected yet.")
        return
    
    # Define map size and resolution
    map_size = 20  # Map covers -10 to +10 meters in both x and y
    cell_size = 1.0  # Each cell represents 1 meter
    grid_size = int(map_size / cell_size)  # Number of cells in each dimension
    
    # Create empty map
    obstacle_map = [[' ' for _ in range(grid_size)] for _ in range(grid_size)]
    
    # Mark obstacles on map
    for obs in obstacle_positions:
        # Convert world coordinates to map indices
        # For x: -10 maps to index 0, +10 maps to index (grid_size-1)
        # For y: -10 maps to index (grid_size-1), +10 maps to index 0 (inverted)
        x_idx = int((obs[0] + map_size/2) / cell_size)
        y_idx = int((map_size/2 - obs[1]) / cell_size)  # Invert y-axis
        
        # Ensure indices are within bounds
        x_idx = max(0, min(grid_size-1, x_idx))
        y_idx = max(0, min(grid_size-1, y_idx))
        
        # Mark obstacle
        obstacle_map[y_idx][x_idx] = 'X'
    
    # Mark drone positions
    for i, pos in enumerate(shared_positions):
        if pos is not None and drone_active[i]:
            # Convert world coordinates to map indices
            x_idx = int((pos[0] + map_size/2) / cell_size)
            y_idx = int((map_size/2 - pos[1]) / cell_size)  # Invert y-axis
            
            # Ensure indices are within bounds
            x_idx = max(0, min(grid_size-1, x_idx))
            y_idx = max(0, min(grid_size-1, y_idx))
            
            # Mark drone
            obstacle_map[y_idx][x_idx] = str(i+1)
    
    # Print map with coordinate labels
    print("Obstacle Map (X = obstacle, 1-3 = drones):")
    print("    " + "".join([f"{int(i-grid_size/2):2d}" if i % 5 == 0 else "  " for i in range(grid_size)]))
    print("   +" + "-" * grid_size + "+")
    
    for y in range(grid_size):
        y_coord = int(grid_size/2 - y)
        if y_coord % 5 == 0:
            print(f"{y_coord:3d}|" + "".join(obstacle_map[y]) + "|")
        else:
            print("   |" + "".join(obstacle_map[y]) + "|")
    
    print("   +" + "-" * grid_size + "+")


WALL_MAP_SIZE = 20  
WALL_CELL_SIZE = 0.5  
WALL_GRID_SIZE = int(WALL_MAP_SIZE / WALL_CELL_SIZE)
wall_map = [[' ' for _ in range(WALL_GRID_SIZE)] for _ in range(WALL_GRID_SIZE)]  # Initialize empty wall map

def update_wall_map(current_pos, range_front, range_right, range_left, yaw):
    """Update the wall map based on range sensor readings"""
    global wall_map
    
    # Convert drone position to map coordinates
    drone_x_idx = int((current_pos[0] + WALL_MAP_SIZE/2) / WALL_CELL_SIZE)
    drone_y_idx = int((WALL_MAP_SIZE/2 - current_pos[1]) / WALL_CELL_SIZE)
    
    # Calculate wall positions based on range sensors and drone orientation
    cos_yaw = cos(yaw)
    sin_yaw = sin(yaw)
    
    # Front wall detection
    if range_front < 2.0:  # Only record walls within 2 meters
        wall_x = current_pos[0] + range_front * cos_yaw
        wall_y = current_pos[1] + range_front * sin_yaw
        wall_x_idx = int((wall_x + WALL_MAP_SIZE/2) / WALL_CELL_SIZE)
        wall_y_idx = int((WALL_MAP_SIZE/2 - wall_y) / WALL_CELL_SIZE)
        
        if 0 <= wall_x_idx < WALL_GRID_SIZE and 0 <= wall_y_idx < WALL_GRID_SIZE:
            wall_map[wall_y_idx][wall_x_idx] = '#'
    
    # Right wall detection
    if range_right < 2.0:
        wall_x = current_pos[0] + range_right * cos(yaw + pi/2)
        wall_y = current_pos[1] + range_right * sin(yaw + pi/2)
        wall_x_idx = int((wall_x + WALL_MAP_SIZE/2) / WALL_CELL_SIZE)
        wall_y_idx = int((WALL_MAP_SIZE/2 - wall_y) / WALL_CELL_SIZE)
        
        if 0 <= wall_x_idx < WALL_GRID_SIZE and 0 <= wall_y_idx < WALL_GRID_SIZE:
            wall_map[wall_y_idx][wall_x_idx] = '#'
    
    # Left wall detection
    if range_left < 2.0:
        wall_x = current_pos[0] + range_left * cos(yaw - pi/2)
        wall_y = current_pos[1] + range_left * sin(yaw - pi/2)
        wall_x_idx = int((wall_x + WALL_MAP_SIZE/2) / WALL_CELL_SIZE)
        wall_y_idx = int((WALL_MAP_SIZE/2 - wall_y) / WALL_CELL_SIZE)
        
        if 0 <= wall_x_idx < WALL_GRID_SIZE and 0 <= wall_y_idx < WALL_GRID_SIZE:
            wall_map[wall_y_idx][wall_x_idx] = '#'
    
    # Mark drone position
    if 0 <= drone_x_idx < WALL_GRID_SIZE and 0 <= drone_y_idx < WALL_GRID_SIZE:
        wall_map[drone_y_idx][drone_x_idx] = str(drone_id)

def print_wall_map():
    """Print the current wall map with adjusted spacing for better proportions"""
    print("\nWall Map (# = wall, 1-3 = drones):")
    
    # Double the spacing for x-axis to make it less compressed
    print("    " + "".join([f"{int(i*WALL_CELL_SIZE-WALL_MAP_SIZE/2):6d}" if i % 4 == 0 else "      " 
                           for i in range(WALL_GRID_SIZE)]))
    print("   +" + "-" * (WALL_GRID_SIZE * 2) + "+")  # Double the width of the border
    
    for y in range(WALL_GRID_SIZE):
        y_coord = int(WALL_MAP_SIZE/2 - y * WALL_CELL_SIZE)
        if y % 4 == 0:
            # Double space between characters for better x-axis proportion
            print(f"{y_coord:3d}|" + "  ".join(wall_map[y]) + "|")
        else:
            print("   |" + "  ".join(wall_map[y]) + "|")
    
    print("   +" + "-" * (WALL_GRID_SIZE * 2) + "+")  # Double the width of the border

# wall measuring params
# wall_segments = []  # List to store wall segments with start and end points
# current_wall_start = None  # Starting point of current wall segment
# last_wall_point = None  # Last detected wall point
# MIN_WALL_LENGTH = 0.5  # Minimum length to consider as a wall segment
# wall_measurements = {
#     'lengths': [],  # Store different wall lengths
#     'total_distance': 0  # Total distance traveled along walls
# }

# def update_wall_measurements(current_pos, range_front, range_right, range_left, yaw, v_x, v_y, dt):
#     """Update wall measurements based on drone movement and sensor readings"""
#     global current_wall_start, last_wall_point, wall_segments, wall_measurements
    
#     # Calculate current velocity magnitude
#     velocity_magnitude = (v_x**2 + v_y**2)**0.5
    
#     # Check if we're detecting a wall (using any of the range sensors)
#     detecting_wall = (range_front < 2.0 or range_right < 2.0 or range_left < 2.0)
    
#     # Calculate the current wall point if we're detecting a wall
#     if detecting_wall:
#         # Use the closest sensor reading to determine wall position
#         min_range = min(
#             range_front if range_front < 2.0 else float('inf'),
#             range_right if range_right < 2.0 else float('inf'),
#             range_left if range_left < 2.0 else float('inf')
#         )
        
#         # Calculate wall point based on drone position and sensor reading
#         if min_range == range_front:
#             wall_point = [
#                 current_pos[0] + range_front * cos(yaw),
#                 current_pos[1] + range_front * sin(yaw)
#             ]
#         elif min_range == range_right:
#             wall_point = [
#                 current_pos[0] + range_right * cos(yaw + pi/2),
#                 current_pos[1] + range_right * sin(yaw + pi/2)
#             ]
#         else:  # Left sensor
#             wall_point = [
#                 current_pos[0] + range_left * cos(yaw - pi/2),
#                 current_pos[1] + range_left * sin(yaw - pi/2)
#             ]
        
#         # Initialize wall segment if we don't have one
#         if current_wall_start is None:
#             current_wall_start = wall_point
#             last_wall_point = wall_point
#         else:
#             # Calculate distance from last point
#             dist_from_last = ((wall_point[0] - last_wall_point[0])**2 + 
#                             (wall_point[1] - last_wall_point[1])**2)**0.5
            
#             # Update total distance
#             wall_measurements['total_distance'] += dist_from_last
            
#             # Check if we've moved significantly from the start of the wall
#             dist_from_start = ((wall_point[0] - current_wall_start[0])**2 + 
#                              (wall_point[1] - current_wall_start[1])**2)**0.5
            
#             # If we've moved significantly and the direction has changed, end the wall segment
#             if dist_from_start > MIN_WALL_LENGTH:
#                 # Calculate angle change
#                 start_angle = atan2(last_wall_point[1] - current_wall_start[1],
#                                   last_wall_point[0] - current_wall_start[0])
#                 current_angle = atan2(wall_point[1] - last_wall_point[1],
#                                     wall_point[0] - last_wall_point[0])
#                 angle_diff = abs(normalize_angle(current_angle - start_angle))
                
#                 # If angle change is significant (e.g., > 45 degrees), end segment
#                 if angle_diff > pi/4:
#                     wall_segments.append({
#                         'start': current_wall_start,
#                         'end': last_wall_point,
#                         'length': dist_from_start
#                     })
#                     wall_measurements['lengths'].append(dist_from_start)
#                     current_wall_start = wall_point
            
#             last_wall_point = wall_point
#     else:
#         # If we've lost wall detection and had a wall segment, save it
#         if current_wall_start is not None and last_wall_point is not None:
#             dist = ((last_wall_point[0] - current_wall_start[0])**2 + 
#                    (last_wall_point[1] - current_wall_start[1])**2)**0.5
#             if dist > MIN_WALL_LENGTH:
#                 wall_segments.append({
#                     'start': current_wall_start,
#                     'end': last_wall_point,
#                     'length': dist
#                 })
#                 wall_measurements['lengths'].append(dist)
#             current_wall_start = None
#             last_wall_point = None

# def normalize_angle(angle):
#     """Normalize angle to [-pi, pi]"""
#     while angle > pi:
#         angle -= 2 * pi
#     while angle < -pi:
#         angle += 2 * pi
#     return angle

# def print_wall_measurements():
#     """Print the current wall measurements"""
#     if not wall_measurements['lengths']:
#         print("\nNo wall measurements yet.")
#         return
    
#     print("\nWall Measurements:")
#     print(f"Total distance along walls: {wall_measurements['total_distance']:.2f} meters")
#     print(f"Number of wall segments: {len(wall_segments)}")
    
#     # Print individual segment lengths
#     print("\nWall segment lengths:")
#     for i, segment in enumerate(wall_segments):
#         print(f"Segment {i+1}: {segment['length']:.2f} meters")
    
#     # Calculate and print statistics
#     lengths = wall_measurements['lengths']
#     avg_length = sum(lengths) / len(lengths)
#     max_length = max(lengths)
#     min_length = min(lengths)
    
#     print(f"\nStatistics:")
#     print(f"Average wall length: {avg_length:.2f} meters")
#     print(f"Longest wall: {max_length:.2f} meters")
#     print(f"Shortest wall: {min_length:.2f} meters")

if __name__ == '__main__':
    # Initialize robot, motors
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())  # Timestep for the main loop

    # Initialize keyboard
    keyboard = Keyboard()
    keyboard.enable(timestep)  # Enable keyboard with timestep

    # Retrieve drone name and ID
    name = robot.getName() 
    drone_id = int(name.replace("Drone", ""))  # Extract numeric ID from name
    drone_names = ["Drone1", "Drone2", "Drone3"]
    drone_team = name[0]  

    # Initialize motors
    mot1 = robot.getDevice("m1_motor")
    mot2 = robot.getDevice("m2_motor")
    mot3 = robot.getDevice("m3_motor")
    mot4 = robot.getDevice("m4_motor")

    # Set motor position and velocity
    mot1.setPosition(float('inf'))  
    mot2.setPosition(float('inf'))
    mot3.setPosition(float('inf'))
    mot4.setPosition(float('inf'))

    mot1.setVelocity(-1)  
    mot2.setVelocity(1)
    mot3.setVelocity(-1)
    mot4.setVelocity(1)

    # Initialize sensors
    imu = robot.getDevice("inertial_unit")
    imu.enable(timestep)
    gps = robot.getDevice("gps")
    gps.enable(timestep)
    gyro = robot.getDevice("gyro")
    gyro.enable(timestep)
    camera = robot.getDevice("camera")
    camera.enable(timestep)
    range_front = robot.getDevice("range_front")
    range_front.enable(timestep)
    range_left = robot.getDevice("range_left")
    range_left.enable(timestep)
    range_back = robot.getDevice("range_back")
    range_back.enable(timestep)
    range_right = robot.getDevice("range_right")
    range_right.enable(timestep)

    # Set default hover height and position
    HOVER_HEIGHT = 1.0  # 1 meter above ground
    initial_position = None
    first_time = True
    past_x_global = 0
    past_y_global = 0
    past_time = 0

    # Initialize wall following
    wall_following = WallFollowing(
        angle_value_buffer=0.01,
        reference_distance_from_wall=0.5,
        max_forward_speed=0.3,
        init_state=WallFollowing.StateWallFollowing.FORWARD
    )

    # Shared list to hold positions of all drones
    shared_positions = [None, None, None]  # Assuming 3 drones
    drone_active = [True, True, True]  # Track which drones are active
    obstacle_positions = []  # List to store positions of detected obstacles
    obstacle_radius = 1.0  # Radius around obstacle positions to avoid (in meters)
    
    # Dictionary to store orientation data for each drone
    drone_orientations = {}  # Will be populated with {drone_id: (roll, pitch, yaw, yaw_rate)}

    # Add position history tracking for each drone
    position_history = [[] for _ in range(3)]
    max_history_length = 10  # Keep the last 10 positions

    # Modify the update_shared_positions function to track position history
    def update_shared_positions(drone_id, current_pos):
        """Update the shared positions list and position history for a drone"""
        shared_positions[drone_id - 1] = current_pos  # Update the position for the current drone
        
        # Update position history
        position_history[drone_id - 1].append(current_pos)
        if len(position_history[drone_id - 1]) > max_history_length:
            position_history[drone_id - 1].pop(0)  # Remove oldest position

    # Function to send the current position to other drones
    def send_position(current_pos):
        update_shared_positions(drone_id, current_pos)  # Update the shared position list
        
    # Function to send the current orientation to other drones
    def send_orientation(roll, pitch, yaw, yaw_rate):
        """Update the shared orientation data for this drone"""
        drone_orientations[drone_id - 1] = (roll, pitch, yaw, yaw_rate)

    # Function to switch between different drones based on key press
    def switchDrone(key):
        if key == '1':
            print(f"Switching to Drone1")
            # Add specific logic for Drone1 here
        elif key == '2':
            print(f"Switching to Drone2")
            # Add specific logic for Drone2 here
        elif key == '3':
            print(f"Switching to Drone3")
            # Add specific logic for Drone3 here
        else:
            print("Invalid key. Press '1', '2', or '3' to switch drones.")

    # Initialize PID controller for each drone
    pid_controller = pid_velocity_fixed_height_controller()
    
    # Initialize autonomous mode
    autonomous_mode = False
    autonomous_start_time = 0  # Track when autonomous mode was enabled

    # Main loop
    while robot.step(timestep) != -1:
        dt = robot.getTime() - past_time
        current_time = robot.getTime()

        # Set program start time on first iteration
        if first_time:
            initial_position = gps.getValues()
            past_x_global = gps.getValues()[0]
            past_y_global = gps.getValues()[1]
            past_time = robot.getTime()
            program_start_time = robot.getTime()  # Set the program start time
            first_time = False

        current_pos = gps.getValues()
        roll = imu.getRollPitchYaw()[0]
        pitch = imu.getRollPitchYaw()[1]
        yaw = imu.getRollPitchYaw()[2]
        yaw_rate = gyro.getValues()[2]

        # Calculate velocities
        x_global = gps.getValues()[0]
        v_x_global = (x_global - past_x_global)/dt if dt > 0 else 0
        y_global = gps.getValues()[1]
        v_y_global = (y_global - past_y_global)/dt if dt > 0 else 0

        # Get body fixed velocities
        cos_yaw = cos(yaw)
        sin_yaw = sin(yaw)
        v_x = v_x_global * cos_yaw + v_y_global * sin_yaw
        v_y = - v_x_global * sin_yaw + v_y_global * cos_yaw

        # Send the current position to other drones
        send_position(current_pos)
        
        # Send orientation data
        send_orientation(roll, pitch, yaw, yaw_rate)
        
        # Check for drone failures, passing the current time
        active_drones = check_drone_failures(current_time)
        
        # Print the shared positions list with differentiation
        # for i, pos in enumerate(shared_positions):
            # if pos is not None:
            #     status = "ACTIVE" if drone_active[i] else "FAILED"
            #     time_info = "" if current_time - program_start_time >= 10.0 else " (monitoring disabled during first 10s)"
            #     print(f"Shared Position of Drone {i + 1} ({status}){time_info}: {pos}")
                
        # Adjust BCO behavior based on drone failures
        if active_drones < 3:  # If any drones have failed
            # Only consider positions from active drones
            valid_positions = [pos for i, pos in enumerate(shared_positions) 
                              if drone_active[i] and is_position_valid(pos, current_time)]
            
            # Process positions from active drones only
            best_position = current_pos  # Start with current position as best
            for pos in valid_positions:
                if evaluate_position(pos) > evaluate_position(best_position):
                    best_position = pos

        # Get range sensor values in meters
        range_front_value = range_front.getValue() / 1000
        range_right_value = range_right.getValue() / 1000
        range_left_value = range_left.getValue() / 1000

        # Choose wall following direction based on drone ID
        if drone_id == 3:
            direction = WallFollowing.WallFollowingDirection.LEFT
            range_side_value = range_right_value  # Use left sensor for right wall following
        else:
            direction = WallFollowing.WallFollowingDirection.LEFT
            range_side_value = range_right_value  # Use right sensor for left wall following

        # Get wall following commands
        cmd_vel_x, cmd_vel_y, cmd_ang_w, state_wf = wall_following.wall_follower(
            range_front_value, range_side_value, yaw, direction, robot.getTime())
            
        # Check for obstacles and adjust wall following if needed
        if obstacle_positions:
            # First try to find a safe path
            safe_target = find_safe_path(current_pos, 
                                        [current_pos[0] + cmd_vel_x, current_pos[1] + cmd_vel_y, current_pos[2]], 
                                        current_time)
            
            # Calculate new velocity commands toward the safe target
            safe_dir_x = safe_target[0] - current_pos[0]
            safe_dir_y = safe_target[1] - current_pos[1]
            
            # Normalize and scale to maintain similar speed
            safe_dir_magnitude = (safe_dir_x**2 + safe_dir_y**2)**0.5
            if safe_dir_magnitude > 0.001:
                original_magnitude = (cmd_vel_x**2 + cmd_vel_y**2)**0.5
                safe_dir_x = safe_dir_x / safe_dir_magnitude * original_magnitude
                safe_dir_y = safe_dir_y / safe_dir_magnitude * original_magnitude
            
            # Apply the standard obstacle avoidance on top of the safe path
            cmd_vel_x, cmd_vel_y = adjust_wall_following_for_obstacles(safe_dir_x, safe_dir_y, current_pos, current_time)
            
            # Periodically print the obstacle map (every 5 seconds)
            if int(current_time) % 5 == 0 and int(current_time) != int(past_time):
                print_obstacle_map()

        # Initialize movement commands
        forward_desired = 0
        sideways_desired = 0
        yaw_desired = 0

        # Handle keyboard input
        key = keyboard.getKey()
        if key != -1:
            key_char = chr(key)
            if key_char == 'A':
                autonomous_mode = True
                autonomous_start_time = current_time  # Record the time when autonomous mode was enabled
                print("Autonomous mode: ON")
            elif key_char == 'D':
                autonomous_mode = False
                print("Autonomous mode: OFF")
            else:
                print(f"Key pressed: {key_char}")
                switchDrone(key_char)

        # Set movement commands based on mode and drone ID
        if autonomous_mode:
            # Calculate time since autonomous mode was enabled
            time_since_autonomous = current_time - autonomous_start_time
            
            # Update wall map and measurements
            update_wall_map(current_pos, range_front_value, range_left_value, range_right_value, yaw)
            # update_wall_measurements(current_pos, range_front_value, range_right_value, range_left_value, 
            #                       yaw, v_x, v_y, dt)
            
            # Print maps and measurements periodically (every 5 seconds)
            if int(current_time) % 5 == 0 and int(current_time) != int(past_time):
                print_obstacle_map()
                print_wall_map()
                # print_wall_measurements()
            
            # Apply delays based on drone ID
            if drone_id == 1:
                # Drone 1 starts immediately
                forward_desired = cmd_vel_x
                sideways_desired = cmd_vel_y
                yaw_desired = cmd_ang_w
            elif drone_id == 2 and time_since_autonomous >= 4.0:
                # Drone 2 starts after 3 seconds
                forward_desired = cmd_vel_x
                sideways_desired = cmd_vel_y
                yaw_desired = cmd_ang_w
            elif drone_id == 3 and time_since_autonomous >= 8.0:
                # Drone 3 starts after 6 seconds
                forward_desired = cmd_vel_x
                sideways_desired = cmd_vel_y
                yaw_desired = cmd_ang_w
            else:
                # Drones waiting for their delay to complete will hover
                forward_desired = 0
                sideways_desired = 0
                yaw_desired = 0

        # Calculate motor commands using PID controller
        motor_power = pid_controller.pid(
            timestep/1000.0,  # dt in seconds
            forward_desired,  # desired_vx
            sideways_desired,  # desired_vy
            yaw_desired,  # desired_yaw_rate
            HOVER_HEIGHT,  # desired_altitude
            roll, pitch, yaw_rate,  # current roll, pitch, yaw_rate
            current_pos[2],  # current_altitude
            v_x, v_y  # current_vx, current_vy
        )

        # Apply motor commands
        mot1.setVelocity(-motor_power[0])
        mot2.setVelocity(motor_power[1])
        mot3.setVelocity(-motor_power[2])
        mot4.setVelocity(motor_power[3])

        # Update past values
        past_time = robot.getTime()
        past_x_global = x_global
        past_y_global = y_global

    # Exit the loop when the simulation ends