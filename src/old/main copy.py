import math
from controller import Robot, GPS, Motor, Camera, Emitter
import numpy as np
import cv2 as cv
import struct

DEBUG = True

robot = Robot()

# region globals
timestep = int(robot.getBasicTimeStep())
max_velocity = 6.28

rwheel = robot.getDevice("right_wheel motor")
rwheel.setPosition(float("inf"))

lwheel = robot.getDevice("left_wheel motor")
lwheel.setPosition(float("inf"))

speeds = [max_velocity, max_velocity]

rcam = robot.getDevice("right_cam")
rcam.enable(timestep)

lcam = robot.getDevice("left_cam")
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

gps_data = []
compass_data = []
color_data = []

front_dist = 0
back_dist = 0
right_dist = 0
left_dist = 0

pos_x = 0
pos_z = 0

# endregion

# region sensors
def get_lidar_data():
    global front_dist, back_dist, right_dist, left_dist
    front_dist = 0
    back_dist = 0
    right_dist = 0
    left_dist = 0
    
    lidar_data = np.zeros((2, 360), dtype=np.float32)
    for i in range(360):
        lidar_data[0, i] = lidar.getRangeImage()[i]
        lidar_data[1, i] = lidar.getHorizontalFov()[i]
        
        if i >= 0 and i < 90:
            front_dist += lidar_data[0, i]
        elif i >= 90 and i < 180:
            right_dist += lidar_data[0, i]
        elif i >= 180 and i < 270:
            back_dist += lidar_data[0, i]
        elif i >= 270 and i < 360:
            left_dist += lidar_data[0, i]
            
    return lidar_data

def get_sensor_data():
    global gps_data, compass_data,color_data, front_dist, back_dist, right_dist, left_dist, pos_x, pos_z

    # gps data
    gps_data = [gps.getValues()[0], gps.getValues()[1], gps.getValues()[2]]
    pos_x = round(gps_data[0] * 100 + 50)
    pos_z = round(gps_data[2] * 100 + 50)
    
    # compass data
    compass_data = compass.getRollPitchYaw()[2]
    compass_data = compass_data * 180 / math.pi
    compass_data = round(compass_data, 1)
    
    # distance data
    # front_dist = front_dist_sens.getValue();
    # back_dist = back_dist_sens.getValue();
    # right_dist = right_dist_sens.getValue();
    # left_dist = left_dist_sens.getValue();
    
    # lidar data
   
    # color data
    image = color_sensor.getImage();
    
    r = color_sensor.imageGetRed(image, 1, 0, 0)
    g = color_sensor.imageGetGreen(image, 1, 0, 0)
    b = color_sensor.imageGetBlue(image, 1, 0, 0)
    
    color_data = []
    color_data.append(r)
    color_data.append(g)
    color_data.append(b)
    
# endregion
    
# region movement
def turn_right():
    # left wheel
    speeds[0] = 0.6 * max_velocity
    # right wheel
    speeds[1] = -0.2 * max_velocity

def turn_left():
    # left wheel
    speeds[0] = -0.2 * max_velocity
    # right wheel
    speeds[1] = 0.6 * max_velocity
    
def spin():
    # left wheel
    speeds[0] = 0.6 * max_velocity
    # right wheel
    speeds[1] = -0.6 * max_velocity
    
def delay(ms):
    init_time = robot.getTime()
    while (robot.step(timestep) != -1):
        if (robot.getTime() - init_time) * 1000.0 > ms:
            break
        
def stop(ms):
    rwheel.setVelocity(0)
    lwheel.setVelocity(0)
    delay(ms)
    
# endregion

# region signs

def detect_signs():
    # get image from rcam
    img = rcam.getImage()
    if img: 
        if check_sign(img, rcam):
            return
    
    # get image from lcam
    img = lcam.getImage()
    if img:
        if check_sign(img, lcam):
            return

def check_sign(img, cam):
    img = np.frombuffer(img, np.uint8).reshape((cam.getHeight(), cam.getWidth(), 4))
    img = cv.cvtColor(img, cv.COLOR_BGRA2BGR)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    
    # Apply thresholding to get binary image
    _, thresh = cv.threshold(gray, 80, 255, cv.THRESH_BINARY_INV)
    
    # Find contours in the thresholded image
    contours, _ = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    
    for cnt in contours:
        # Approximate the contour to a polygon
        epsilon = 0.02 * cv.arcLength(cnt, True)
        approx = cv.approxPolyDP(cnt, epsilon, True)
        
        # Check if the polygon has 4 vertices (potential square or rhombus)
        if len(approx) == 4:
            # Calculate the angles between the vertices
            angles = []
            for i in range(4):
                p1 = approx[i][0]
                p2 = approx[(i + 1) % 4][0]
                p3 = approx[(i + 2) % 4][0]
                
                v1 = p1 - p2
                v2 = p3 - p2
                
                angle = np.arccos(np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2)))
                angles.append(np.degrees(angle))
            
            # Check if the angles are close to 90 degrees (square) or 45/135 degrees (rhombus)
            if all(80 <= angle <= 100 for angle in angles):
                print("Square detected") if DEBUG else None
                check_victims(img, cam)
                return True
            elif all(40 <= angle <= 50 or 130 <= angle <= 140 for angle in angles):
                print("Rhombus detected") if DEBUG else None
                check_hazards(img, cam)
                return True
    
    return False

def check_victims(img, cam):
    # H S U
    img = np.frombuffer(img, np.uint8).reshape((cam.getHeight(), cam.getWidth(), 4))
    img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    _, thresh = cv.threshold(img, 80, 255, cv.THRESH_BINARY_INV)
    
    height, width = thresh.shape
    section_width = width // 3
    
    first_section = thresh[:, :section_width]
    # ignore the middle section
    third_section = thresh[:, 2 * section_width:]
    
    contours_first, _ = cv.findContours(first_section, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    contours_third, _ = cv.findContours(third_section, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    
    # output image for debugging
    cv.imshow("First Section", first_section)
    cv.imshow("Third Section", third_section)
    cv.waitKey(1)
    
    num_contours_first = len(contours_first)
    num_contours_third = len(contours_third)
    
    print("First Section Contours: ", num_contours_first)
    print("Third Section Contours: ", num_contours_third)
    
    if num_contours_first == 1 and num_contours_third == 1:
        print("found H")
        report('H')
        return 'H'
    elif num_contours_first == 2 and num_contours_third == 2:
        print("found S")
        report('S')
        return 'S'
    elif num_contours_first == 3 and num_contours_third == 3:
        print("found U")
        report('U')
        return 'U'
    else:
        return None

def check_hazards(img):
    # F P . .
    pass

reported_victims = []

def report(sign_type):
    global reported_victims

    stop(1300)
    pos_x = int(gps.getValues()[0] * 100)  # Convert to cm
    pos_z = int(gps.getValues()[2] * 100)
    
    if (pos_x, pos_z) in reported_victims:
        print(f"Victim at ({pos_x}, {pos_z}) already reported.") if DEBUG else None
        return
    
    reported_victims.append((pos_x, pos_z))
    
    sign_type = bytes(sign_type, "utf-8")
    msg = struct.pack("i i c", pos_x, pos_z, sign_type)
    
    emitter.send(msg)
    robot.step(timestep)
    
    if DEBUG:
        print(f"Reported victim of type '{sign_type.decode()}' at ({pos_x}, {pos_z}).")

# endregion

# region map
# -------------------------------------------------------------------------------
map_size = (200, 200)
grid_map = np.zeros(map_size, dtype=np.int32)
map = np.array([-1, -1, -1, -1], dtype="int32").reshape(2, 2);

# Sensor properties
MAX_DISTANCE_CM = 80  # max dist_sensor range
TILE_SIZE = 12
THRESHOLD = 70

def update_map(x, z, distance, direction_enum):
    global grid_map
    
    if(distance > MAX_DISTANCE_CM):
        distance = MAX_DISTANCE_CM
    
    num_tiles = min(distance // TILE_SIZE, map_size[0] - 1)
    grid_x, grid_z = int(x), int(z);
    
    dir = compass_data
    
    # get proper direction from compass and direction enum
    if dir < 0:
        dir += 360
        
    if dir >= 315 or dir < 45:
        direction = 0
    elif dir >= 45 and dir < 135:
        direction = 1
    elif dir >= 135 and dir < 225:
        direction = 2
    elif dir >= 225 and dir < 315:
        direction = 3
        
    if direction_enum == "front":
        direction = direction
    elif direction_enum == "back":
        direction = (direction + 2) % 4
    elif direction_enum == "left":
        direction = (direction + 1) % 4
    elif direction_enum == "right":
        direction = (direction + 3) % 4
        
    # Update map based on sensor data
    for i in range(num_tiles):
        tile_x, tile_z = grid_x, grid_z
        
        if direction == 0:
            tile_x += i
        elif direction == 1:
            tile_z += i
        elif direction == 2:
            tile_x -= i
        elif direction == 3:
            tile_z -= i
        
        if tile_x < 0 or tile_x >= map_size[0] or tile_z < 0 or tile_z >= map_size[1]:
            break
        
        if grid_map[tile_x, tile_z] == 0:
            grid_map[tile_x, tile_z] = 1
        elif grid_map[tile_x, tile_z] == -1:
            grid_map[tile_x, tile_z] = 2
            break
       
def display_map():
    # Convert to a color map for better visualization
    color_map = np.zeros((*grid_map.shape, 3), dtype=np.uint8)
    
    color_map[grid_map == 0] = (50, 50, 50)    # Unexplored (dark gray)
    color_map[grid_map == 1] = (255, 255, 255) # Free space (white)
    color_map[grid_map == 2] = (0, 0, 0)       # Walls (black)

    cv.imshow("Map", color_map)
    cv.waitKey(1)  # Refresh display

def map_env():
    global grid_map, gps_data, front_dist, right_dist, left_dist, back_dist
    
    # get gps data
    # x, z = gps_data[0] * 100, gps_data[2] * 100
    print('GPS:\tx: ' + str(pos_x) + ', z: ' + str(pos_z))
    
    print('DIST:\n');
    print('\tfront:', round(front_dist * 100))
    print('\tback:' , round(back_dist  * 100))
    print('\tright:', round(right_dist * 100))
    print('\tleft:' , round(left_dist  * 100))
    
    # get direction from compass
    if compass_data < 0:
        compass_data += 360
        
    if compass_data >= 315 or compass_data < 45:
        dir = "front"
    elif compass_data >= 45 and compass_data < 135:
        dir = "right"
    elif compass_data >= 135 and compass_data < 225:
        dir = "back"
    elif compass_data >= 225 and compass_data < 315:
        dir = "left"
    
    update_map(pos_x, pos_z, front_dist, dir)
    update_map(pos_x, pos_z, back_dist, dir)
    update_map(pos_x, pos_z, right_dist, dir)
    update_map(pos_x, pos_z, left_dist, dir)
    
    # calculate points from dist_sensors + gps to mark as unvisited or walls
    
    # expand map if necesssary
    # update map
    
# endregion

# region movement

visited_tiles = set()
# approach 1: grid exploration
def grid_exploration():
    global pos_x, pos_z

    current_tile = (pos_x // TILE_SIZE, pos_z // TILE_SIZE)
    if current_tile not in visited_tiles:
        visited_tiles.add(current_tile)
        print(f"Visited tile: {current_tile}")

    # Move to the next unvisited tile
    if front_dist < 0.2:  # Obstacle in front
        turn_right()
    else:
        speeds[0] = max_velocity
        speeds[1] = max_velocity
        
# approach 2: frontier exploration
def frontier_exploration():
    global grid_map

    # Identify frontiers (cells adjacent to unexplored areas)
    frontiers = []
    for x in range(grid_map.shape[0]):
        for z in range(grid_map.shape[1]):
            if grid_map[x, z] == 1:  # Explored cell
                for dx, dz in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    nx, nz = x + dx, z + dz
                    if 0 <= nx < grid_map.shape[0] and 0 <= nz < grid_map.shape[1]:
                        if grid_map[nx, nz] == 0:  # Unexplored cell
                            frontiers.append((nx, nz))

    if frontiers:
        # Move toward the nearest frontier
        target = frontiers[0]
        print(f"Moving to frontier: {target}")
        # Implement path planning to move to the target

# approach 3: flood fill
def flood_fill(x, z):
    global grid_map, visited_tiles

    # Convert the robot's position to grid coordinates
    grid_x, grid_z = int(x // TILE_SIZE), int(z // TILE_SIZE)

    # Check if the current cell is out of bounds or already visited
    if grid_x < 0 or grid_x >= map_size[0] or grid_z < 0 or grid_z >= map_size[1]:
        return
    if (grid_x, grid_z) in visited_tiles or grid_map[grid_x, grid_z] == 2:  # 2 = obstacle
        return

    # Mark the current cell as visited
    visited_tiles.add((grid_x, grid_z))
    print(f"Visited tile: ({grid_x}, {grid_z})")

    # Move to adjacent cells (front, back, left, right)
    move_to_tile(grid_x + 1, grid_z)  # Move right
    flood_fill((grid_x + 1) * TILE_SIZE, grid_z * TILE_SIZE)

    move_to_tile(grid_x - 1, grid_z)  # Move left
    flood_fill((grid_x - 1) * TILE_SIZE, grid_z * TILE_SIZE)

    move_to_tile(grid_x, grid_z + 1)  # Move forward
    flood_fill(grid_x * TILE_SIZE, (grid_z + 1) * TILE_SIZE)

    move_to_tile(grid_x, grid_z - 1)  # Move backward
    flood_fill(grid_x * TILE_SIZE, (grid_z - 1) * TILE_SIZE)

import heapq

def explore_entire_map():
    global grid_map

    while True:
        # Identify frontiers (unexplored cells adjacent to explored cells)
        frontiers = []
        for x in range(grid_map.shape[0]):
            for z in range(grid_map.shape[1]):
                if grid_map[x, z] == 0:  # Unexplored cell
                    for dx, dz in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                        nx, nz = x + dx, z + dz
                        if 0 <= nx < grid_map.shape[0] and 0 <= nz < grid_map.shape[1]:
                            if grid_map[nx, nz] == 1:  # Adjacent to explored cell
                                frontiers.append((x, z))
                                break

        # If no frontiers remain, the map is fully explored
        if not frontiers:
            print("Exploration complete!")
            break

        # Select the nearest frontier as the goal
        start = (pos_x // TILE_SIZE, pos_z // TILE_SIZE)
        goal = min(frontiers, key=lambda f: abs(f[0] - start[0]) + abs(f[1] - start[1]))  # Manhattan distance

        print(f"Exploring frontier at: {goal}")

        # Use Best-First Search to move to the goal
        best_first_search(start, goal)


def best_first_search(start, goal):
    global grid_map, visited_tiles

    # Priority queue for BFS (stores (heuristic, (x, z)))
    priority_queue = []
    heapq.heappush(priority_queue, (0, start))

    # Set to track visited cells
    visited = set()

    while priority_queue:
        # Get the cell with the lowest heuristic value
        _, current = heapq.heappop(priority_queue)
        x, z = current

        # If the goal is reached, stop
        if current == goal:
            print(f"Reached goal: {goal}")
            return True

        # Mark the current cell as visited
        visited.add(current)

        # Explore neighbors (front, back, left, right)
        for dx, dz in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            neighbor = (x + dx, z + dz)

            # Skip if the neighbor is out of bounds, visited, or an obstacle
            if (
                neighbor in visited or
                neighbor[0] < 0 or neighbor[0] >= map_size[0] or
                neighbor[1] < 0 or neighbor[1] >= map_size[1] or
                grid_map[neighbor[0], neighbor[1]] == 2
            ):
                continue

            # Calculate the heuristic (e.g., Manhattan distance to the goal)
            heuristic = abs(neighbor[0] - goal[0]) + abs(neighbor[1] - goal[1])

            # Add the neighbor to the priority queue
            heapq.heappush(priority_queue, (heuristic, neighbor))

            # Move the robot to the neighbor
            move_to_tile(neighbor[0] * TILE_SIZE, neighbor[1] * TILE_SIZE)

    print("Goal not reachable")
    return False

def move_to_tile(target_x, target_z):
    global pos_x, pos_z, compass_data

    # Calculate the direction to the target tile
    dx = target_x - pos_x
    dz = target_z - pos_z
    angle_to_target = math.atan2(dz, dx)

    # Rotate the robot to face the target direction
    while abs(compass_data - angle_to_target) > 0.1:
        if compass_data < angle_to_target:
            turn_left()
        else:
            turn_right()

    # Move forward to the target tile
    speeds[0] = max_velocity
    speeds[1] = max_velocity
    while abs(pos_x - target_x) > 0.1 or abs(pos_z - target_z) > 0.1:
        get_sensor_data()
        robot.step(timestep)

    # Stop the robot
    stop(100)

# endregion

# region main

i = 1;

print("HEEREEEEEEEEEEEE");
while robot.step(timestep) != -1:
    print("HEEREEEEEEEEEEEE");
    speeds[0] = max_velocity;
    speeds[1] = max_velocity;
    
    get_sensor_data();
    
    print("HEEREEEEEEEEEEEE");
    
    move_to_tile(pos_x + i, pos_z);
    i += 1;
    # detect_signs();
    
    # explore_entire_map();
    
    # flood_fill(pos_x, pos_z);
    
    # map_env();
    # display_map();
    
    # if lidar_data[1, 270 // 2] < 0.05:
        # turn_right()
    
    # if lidar_data[1, 90 // 2] < 0.05:
        # turn_left()
        
    # if lidar_data[1, 0] < 0.05:
        # spin()
        
    # if get_color() < 80:
    #     spin()
    #     rwheel.setVelocity(speeds[1])
    #     lwheel.setVelocity(speeds[0])
    #     delay(600)
        
    # if check_victim(cam.getImage()):
    #     report('H') # report as H for now
        
    # rwheel.setVelocity(speeds[1])
    # lwheel.setVelocity(speeds[0])
    
    rwheel.setVelocity(0);
    lwheel.setVelocity(0);

# endregion