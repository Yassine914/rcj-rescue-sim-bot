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

# Sensor initialization
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

# Navigation parameters
TILE_SIZE = 12
ROTATION_TOLERANCE = math.radians(3)
MOVEMENT_SPEED = 5.0
ROTATION_SPEED = 4.0
CARDINALS = [0, math.pi/2, math.pi, 3*math.pi/2]  # N, E, S, W

# State management
current_grid = (0, 0)
stack = []
visited = set()
blocked = set()
state = "INIT"
target_dir = 0
rotation_start_time = 0

# Color sensor handling
color_sensor_values = [0, 0, 0]

def get_colour_sensor_values():
    image = color_sensor.getImage()
    color_sensor_values[0] = color_sensor.imageGetRed(image, 1, 0, 0)
    color_sensor_values[1] = color_sensor.imageGetGreen(image, 1, 0, 0)
    color_sensor_values[2] = color_sensor.imageGetBlue(image, 1, 0, 0)
    return color_sensor_values

def get_current_pose():
    position = gps.getValues()
    grid_x = position[0] * 100 / TILE_SIZE
    grid_z = position[2] * 100 / TILE_SIZE
    yaw = compass.getRollPitchYaw()[2]
    return (grid_x, grid_z), yaw

def rotate_to_target():
    global rotation_start_time
    current_yaw = compass.getRollPitchYaw()[2]
    target_yaw = CARDINALS[target_dir]
    
    error = (target_yaw - current_yaw + math.pi) % (2*math.pi) - math.pi
    
    if abs(error) < ROTATION_TOLERANCE:
        return True
    
    speed = max(1.0, min(ROTATION_SPEED, abs(error) * 2.0))
    
    if robot.getTime() - rotation_start_time > 2.0:
        print("Rotation timeout! Forcing completion.")
        return True
    
    if error > 0:
        lwheel.setVelocity(-speed)
        rwheel.setVelocity(speed)
    else:
        lwheel.setVelocity(speed)
        rwheel.setVelocity(-speed)
    
    if DEBUG:
        print(f"Rotating: Current {math.degrees(current_yaw):.1f}° -> Target {math.degrees(target_yaw):.1f}° | Error {math.degrees(error):.1f}°")
    
    return False

def move_forward():
    start_pos = gps.getValues()
    current_pos = gps.getValues()
    distance = math.hypot(current_pos[0]-start_pos[0], current_pos[2]-start_pos[2])
    
    if distance < TILE_SIZE * 0.95:
        lwheel.setVelocity(MOVEMENT_SPEED)
        rwheel.setVelocity(MOVEMENT_SPEED)
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

# Initialization
current_grid, _ = get_current_pose()
visited.add(current_grid)
stack.append(current_grid)

while robot.step(timestep) != -1:
    current_grid, current_yaw = get_current_pose()
    
    # Get color values using specified method
    rgb = get_colour_sensor_values()
    brightness = sum(rgb)/3
    
    # Black hole detection (adjust threshold as needed)
    if brightness < 25:  # Assuming 0-255 range from imageGet functions
        print(f"Black hole detected! RGB: {rgb} Blocking grid!")
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
            rotation_start_time = robot.getTime()
        else:
            if stack:
                target_grid = stack.pop()
                dx = target_grid[0] - current_grid[0]
                dz = target_grid[1] - current_grid[1]
                target_dir = 0 if dz == 1 else 1 if dx == 1 else 2 if dz == -1 else 3
                print(f"Backtracking to {target_grid}")
                state = "ROTATE"
                rotation_start_time = robot.getTime()
            else:
                print("Exploration complete!")
                break
    
    elif state == "ROTATE":
        if rotate_to_target():
            print("Rotation complete!")
            state = "MOVE"
            lwheel.setVelocity(0)
            rwheel.setVelocity(0)
    
    elif state == "MOVE":
        if move_forward():
            print("Movement complete!")
            state = "DECIDE"
    
    elif state == "ESCAPE":
        if stack:
            target_grid = stack.pop()
            dx = target_grid[0] - current_grid[0]
            dz = target_grid[1] - current_grid[1]
            target_dir = 0 if dz == 1 else 1 if dx == 1 else 2 if dz == -1 else 3
            state = "ROTATE"
            rotation_start_time = robot.getTime()
        else:
            print("No escape route!")
            break

# Stop motors
lwheel.setVelocity(0)
rwheel.setVelocity(0)