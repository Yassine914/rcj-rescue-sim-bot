import math
from controller import Robot, GPS, Motor, Camera, Emitter
import numpy as np
import cv2 as cv
import struct

DEBUG = True
robot = Robot()
timestep = int(robot.getBasicTimeStep())
max_velocity = 6.28

# Initialize devices with correct names
rwheel = robot.getDevice("right_wheel motor")
lwheel = robot.getDevice("left_wheel motor")
rwheel.setPosition(float("inf"))
lwheel.setPosition(float("inf"))

rcam = robot.getDevice("right_cam")
lcam = robot.getDevice("left_cam")
rcam.enable(timestep)
lcam.enable(timestep)

color_sensor = robot.getDevice("color_sensor")
color_sensor.enable(timestep)

emitter = robot.getDevice("emitter")
gps = robot.getDevice("gps")
gps.enable(timestep)
lidar = robot.getDevice("lidar")
lidar.enable(timestep)
compass = robot.getDevice("inertial_unit")
compass.enable(timestep)

# Navigation variables
TILE_SIZE = 12
current_grid = (0, 0)
stack = []
visited = set()
blocked = set()
state = "INIT"
target_dir = 0
CARDINALS = [0, math.pi/2, math.pi, 3*math.pi/2]

def get_current_pose():
    position = gps.getValues()
    grid_x = position[0] * 100 // TILE_SIZE
    grid_z = position[2] * 100 // TILE_SIZE
    yaw = compass.getRollPitchYaw()[2]
    return (grid_x, grid_z), yaw

def rotate_to_target():
    current_yaw = compass.getRollPitchYaw()[2]
    target_yaw = CARDINALS[target_dir]
    error = (target_yaw - current_yaw + math.pi) % (2*math.pi) - math.pi
    if abs(error) < math.radians(5):
        return True
    speed = 3.0 * min(1.0, abs(error)/0.5)
    if error > 0:
        lwheel.setVelocity(-speed)
        rwheel.setVelocity(speed)
    else:
        lwheel.setVelocity(speed)
        rwheel.setVelocity(-speed)
    return False

def move_forward():
    start_pos = gps.getValues()
    current_pos = gps.getValues()
    distance = math.hypot(current_pos[0]-start_pos[0], current_pos[2]-start_pos[2])
    if distance < TILE_SIZE:
        lwheel.setVelocity(max_velocity)
        rwheel.setVelocity(max_velocity)
        return False
    lwheel.setVelocity(0)
    rwheel.setVelocity(0)
    return True

def get_neighbors(grid):
    return [
        (grid[0], grid[1]+1),  # North
        (grid[0]+1, grid[1]),  # East
        (grid[0], grid[1]-1),  # South
        (grid[0]-1, grid[1])   # West
    ]

# Main loop
current_grid, _ = get_current_pose()
visited.add(current_grid)
stack.append(current_grid)

color_sensor_values = [0, 0, 0]

def get_colour_sensor_values():
    image = color_sensor.getImage()
    r =     color_sensor.imageGetRed(image, 1, 0, 0)
    g =     color_sensor.imageGetGreen(image, 1, 0, 0)
    b =     color_sensor.imageGetBlue(image, 1, 0, 0)
    color_sensor_values[0] = r
    color_sensor_values[1] = g
    color_sensor_values[2] = b

while robot.step(timestep) != -1:
    current_grid, current_yaw = get_current_pose()
    
    get_colour_sensor_values()
    r, g, b = color_sensor_values
    # Print sensor values
    if DEBUG:
        print(f"Grid: {current_grid} | Yaw: {math.degrees(current_yaw):.1f}°")
        print(f"LIDAR: {lidar.getRangeImage()[:3]}")
        print(f"GPS: {gps.getValues()}")
        print(f"Compass: {compass.getRollPitchYaw()}")
        print(f"Color sensor: {r}, {g}, {b}")
        print(f"State: {state}")
        print(f"Stack: {stack}")
        print(f"Visited: {visited}")
        print(f"Blocked: {blocked}")
   
    # Black hole detection
    if (r + g + b) / 3 < 50:  # Arbitrary threshold for black hole detection
        print("Black hole detected! Blocking grid!")
        blocked.add(current_grid)
        state = "ESCAPE"
    
    if state == "INIT":
        state = "DECIDE"
    
    elif state == "DECIDE":
        neighbors = get_neighbors(current_grid)
        valid_neighbors = [
            n for n in neighbors
            if n not in visited and 
            n not in blocked and
            lidar.getRangeImage()[0] > TILE_SIZE*0.8
        ]
        
        if valid_neighbors:
            target_grid = valid_neighbors[0]
            visited.add(target_grid)
            stack.append(current_grid)
            dx = target_grid[0] - current_grid[0]
            dz = target_grid[1] - current_grid[1]
            target_dir = 0 if dz == 1 else 1 if dx == 1 else 2 if dz == -1 else 3
            print(f"Moving to {target_grid} {['N','E','S','W'][target_dir]}")
            state = "ROTATE"
        else:
            if stack:
                target_grid = stack.pop()
                dx = target_grid[0] - current_grid[0]
                dz = target_grid[1] - current_grid[1]
                target_dir = 0 if dz == 1 else 1 if dx == 1 else 2 if dz == -1 else 3
                print(f"Backtracking to {target_grid}")
                state = "ROTATE"
            else:
                print("Exploration complete!")
                break
    
    elif state == "ROTATE":
        if rotate_to_target():
            state = "MOVE"
    
    elif state == "MOVE":
        if move_forward():
            state = "DECIDE"
    
    elif state == "ESCAPE":
        if stack:
            target_grid = stack.pop()
            dx = target_grid[0] - current_grid[0]
            dz = target_grid[1] - current_grid[1]
            target_dir = 0 if dz == 1 else 1 if dx == 1 else 2 if dz == -1 else 3
            state = "ROTATE"
        else:
            print("No escape route!")
            break

# Stop motors
lwheel.setVelocity(0)
rwheel.setVelocity(0)