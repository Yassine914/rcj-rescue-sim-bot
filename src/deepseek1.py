from controller import Robot, Motor
import math

# Initialize robot and devices
robot = Robot()
timestep = 32

# Motors
left_wheel = robot.getDevice("left_wheel motor")
right_wheel = robot.getDevice("right_wheel motor")
left_wheel.setPosition(float('inf'))
right_wheel.setPosition(float('inf'))
left_wheel.setVelocity(0.0)
right_wheel.setVelocity(0.0)

# Sensors
gps = robot.getDevice("gps")
gps.enable(timestep)
compass = robot.getDevice("inertial_unit")
compass.enable(timestep)
lidar = robot.getDevice("lidar")
lidar.enable(timestep)

# Configuration
TILE_SIZE = 0.12  # 12cm in meters
OBSTACLE_DISTANCE = 0.15  # 15cm
HEADING_TOLERANCE = 2  # degrees

def get_compass_heading():
    return math.degrees(compass.getRollPitchYaw()[2]) % 360

def get_position():
    return gps.getValues()[0], gps.getValues()[2]  # x, z coordinates

def print_status():
    x, z = get_position()
    print(f"Position: ({x:.2f}, {z:.2f}) | Heading: {get_compass_heading():.1f}°")

def rotate_to_target(target_heading):
    print(f"Rotating to {target_heading}°")
    while robot.step(timestep) != -1:
        current = get_compass_heading()
        diff = (target_heading - current + 180) % 360 - 180
        
        if abs(diff) < HEADING_TOLERANCE:
            left_wheel.setVelocity(0)
            right_wheel.setVelocity(0)
            return
        
        speed = 0.6 * min(max(diff/5, -1), 1)  # Proportional control
        left_wheel.setVelocity(-speed)
        right_wheel.setVelocity(speed)
        print(f"Current: {current:.1f}° | Target: {target_heading}° | Diff: {diff:.1f}")

def move_forward(distance=TILE_SIZE):
    start_x, start_z = get_position()
    print(f"Moving forward {distance*100:.0f}cm")
    
    while robot.step(timestep) != -1:
        current_x, current_z = get_position()
        traveled = math.hypot(current_x - start_x, current_z - start_z)
        
        if traveled >= distance - 0.02:  # 2cm tolerance
            left_wheel.setVelocity(0)
            right_wheel.setVelocity(0)
            print(f"Reached target (traveled {traveled*100:.1f}cm)")
            return
        
        left_wheel.setVelocity(3.0)
        right_wheel.setVelocity(3.0)

def check_obstacle():
    ranges = lidar.getRangeImage()
    front_distance = min(ranges[256:768])  # Middle layer, front 180°
    print(f"Front distance: {front_distance:.2f}m")
    return front_distance < OBSTACLE_DISTANCE

def explore_dfs():
    stack = [(round(get_position()[0]/TILE_SIZE), 
             round(get_position()[2]/TILE_SIZE))]
    visited = set()
    
    while robot.step(timestep) != -1 and stack:
        current = stack.pop()
        if current in visited:
            continue
        visited.add(current)
        
        print(f"\n--- Exploring cell {current} ---")
        
        # Check neighbors in N, E, S, W order
        for dx, dz, target_heading in [(0,1,0), (1,0,90), (0,-1,180), (-1,0,270)]:
            neighbor = (current[0] + dx, current[1] + dz)
            
            if neighbor in visited:
                continue
                
            # Rotate to face direction
            rotate_to_target(target_heading)
            
            # Check for obstacles
            if check_obstacle():
                print(f"Obstacle detected at {neighbor}")
                visited.add(neighbor)
                continue
                
            # Move to neighbor cell
            move_forward()
            stack.append(neighbor)
            break  # DFS: explore first valid neighbor immediately

# Start exploration
print("Starting exploration...")
explore_dfs()
print("Exploration complete!")