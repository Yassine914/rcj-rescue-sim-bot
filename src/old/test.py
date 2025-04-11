from controller import Robot, Camera, Motor, Lidar
import math

robot = Robot()
timeStep = 32
rightWheel = robot.getDevice("right_wheel motor")
leftWheel = robot.getDevice("left_wheel motor")
leftWheel.setPosition(float("inf"))
rightWheel.setPosition(float("inf"))
receiver = robot.getDevice("receiver")
# receiver.enable(timeStep)
emitter = robot.getDevice("emitter")







camera_right = robot.getDevice("right_cam")
camera_left = robot.getDevice("left_cam")
camera_right.enable(timeStep)
camera_left.enable(timeStep)

lidar = robot.getDevice("lidar")
lidar.enable(timeStep)

colorSensor = robot.getDevice("color_sensor")
colorSensor.enable(timeStep)
color_sensor_values = [0, 0, 0]

gps = robot.getDevice("gps")
gps.enable(timeStep)
gps_readings = [0, 0, 0]

compass = robot.getDevice("inertial_unit")
compass.enable(timeStep)

def get_compass_value():
    global compass_value
    compass_value = compass.getRollPitchYaw()[2]
    compass_value = compass_value * 180 / math.pi

def get_gps_readings():
    gps_readings[0] = gps.getValues()[0]
    gps_readings[1] = gps.getValues()[1]
    gps_readings[2] = gps.getValues()[2]

lidar_values = []
def get_lidar_values():
    global lidar_values
    lidar_values = []
    range_image = lidar.getRangeImage()
    for layer in range(4):
        lidar_values.append([])
        for point in range(512):
            lidar_values[layer].append(
                round(
                    range_image[layer * 512 + point] * 100, 2
                )
            )

def get_colour_sensor_values():
    image = colorSensor.getImage()
    r = colorSensor.imageGetRed(image, 1, 0, 0)
    g = colorSensor.imageGetGreen(image, 1, 0, 0)
    b = colorSensor.imageGetBlue(image, 1, 0, 0)
    color_sensor_values[0] = r
    color_sensor_values[1] = g
    color_sensor_values[2] = b

def get_all_sensor_values():
    get_lidar_values()
    get_colour_sensor_values()
    get_gps_readings()
    get_compass_value()

# def turn_90():
#     get_compass_value()
#     compass_value_rounded_to_nearest_90 = round(compass_value / 90) * 90
#     next_angle = compass_value_rounded_to_nearest_90 + 90
#     if next_angle >= 180:
#         next_angle -= 360
#     while robot.step(timeStep) != -1:
#         get_compass_value()
#         rightWheel.setVelocity(3)
#         leftWheel.setVelocity(-3)
#         if abs(compass_value - next_angle) < 7:
#             rightWheel.setVelocity(0)
#             leftWheel.setVelocity(0)
#             break


def turn_90():
    get_compass_value()
    start_angle = compass_value
    target_angle = start_angle + 90
    if target_angle >= 180:
        target_angle -= 360

    turn_speed = 5.0
    while robot.step(timeStep) != -1:
        
        get_compass_value()

        current_angle = compass_value

        angle_diff = target_angle - current_angle
        if angle_diff > 180:
            angle_diff -= 360
        elif angle_diff < -180:
            angle_diff += 360

        if abs(angle_diff) < 2:
            stop()
            # rightWheel.setVelocity(0)
            # leftWheel.setVelocity(0)
            return
            # break

        rightWheel.setVelocity(turn_speed)
        leftWheel.setVelocity(-turn_speed)

    rightWheel.setVelocity(0)
    leftWheel.setVelocity(0)


def stop():
    """
    Stop the robot for a short duration.
    """
    stop_duration = 500  # Stop duration in ms
    elapsed_time = 0

    while elapsed_time < stop_duration and robot.step(timeStep) != -1:
        rightWheel.setVelocity(0)
        leftWheel.setVelocity(0)
        elapsed_time += timeStep







def is_black_detected():
    BLACK_THRESHOLD = 50
    r, g, b = color_sensor_values[0], color_sensor_values[1], color_sensor_values[2]
    return r < BLACK_THRESHOLD and g < BLACK_THRESHOLD and b < BLACK_THRESHOLD







leftWheel.setVelocity(0.0)
rightWheel.setVelocity(0.0)

# while robot.step(timeStep) != -1:

#     if is_black_detected():
#         print("Black detected! Turning 90 degrees")
#         turn_90()



#     get_all_sensor_values()
#     print("front layer 1: ", lidar_values[0][0])
#     print("front layer 2: ", lidar_values[1][0])
#     print("front layer 3: ", lidar_values[2][0])
#     print("front layer 4: ", lidar_values[3][0])
#     print("---------------------------------")
#     print("Front: ", lidar_values[2][0])
#     print("Right: ", lidar_values[2][127])
#     print("Left: ", lidar_values[2][383])
#     print("Back: ", lidar_values[2][255])
#     print("---------------------------------")
#     print("Color Sensor RGB: ", color_sensor_values)
#     front = lidar_values[2][0]
# # what is the unit of this value?
#     if front < 8:
#         turn_90()
#     else:
#         rightWheel.setVelocity(3)
#         leftWheel.setVelocity(3)




# def find_open_space():
#     """
#     Analyze lidar data to find the direction with the most open space.
#     Returns the target angle (in degrees) to rotate toward the open space.
#     """
#     global lidar_values

#     # Lidar has 512 points; divide into 4 quadrants (front, right, back, left)
#     quadrant_size = len(lidar_values[2]) // 4
#     front = sum(lidar_values[2][:quadrant_size]) / quadrant_size
#     right = sum(lidar_values[2][quadrant_size:2 * quadrant_size]) / quadrant_size
#     back = sum(lidar_values[2][2 * quadrant_size:3 * quadrant_size]) / quadrant_size
#     left = sum(lidar_values[2][3 * quadrant_size:]) / quadrant_size

#     # Find the direction with the largest average distance
#     distances = {"front": front, "right": right, "back": back, "left": left}
#     best_direction = max(distances, key=distances.get)

#     print(f"Distances: {distances}")
#     print(f"Best direction: {best_direction}")

#     # Map the direction to a target angle
#     if best_direction == "front":
#         return 0
#     elif best_direction == "right":
#         return 90
#     elif best_direction == "back":
#         return 180
#     elif best_direction == "left":
#         return -90


# def rotate_to_angle(target_angle):
#     """
#     Rotate the robot to face the target angle using the compass.
#     """
#     get_compass_value()
#     start_angle = compass_value
#     target_angle = (start_angle + target_angle) % 360

#     if target_angle > 180:
#         target_angle -= 360

#     turn_speed = 5.0
#     while robot.step(timeStep) != -1:
#         get_compass_value()
#         current_angle = compass_value

#         angle_diff = target_angle - current_angle
#         if angle_diff > 180:
#             angle_diff -= 360
#         elif angle_diff < -180:
#             angle_diff += 360

#         if abs(angle_diff) < 2:  # Stop when close to the target angle
#             stop()
#             return

#         # Rotate the robot
#         rightWheel.setVelocity(turn_speed)
#         leftWheel.setVelocity(-turn_speed)

#     rightWheel.setVelocity(0)
#     leftWheel.setVelocity(0)


# def explore_map():
#     """
#     Explore the map by moving toward open spaces and avoiding obstacles.
#     """
#     while robot.step(timeStep) != -1:
#         get_all_sensor_values()

#         # Check for obstacles in front
#         front_distance = lidar_values[2][0]
#         if front_distance < 8:  # Threshold for obstacle detection
#             print("Obstacle detected! Finding open space...")
#             target_angle = find_open_space()
#             rotate_to_angle(target_angle)
#         else:
#             # Move forward
#             rightWheel.setVelocity(3)
#             leftWheel.setVelocity(3)

#         # Debugging information
#         print("Front Distance: ", front_distance)
#         print("Color Sensor RGB: ", color_sensor_values)
#         print("---------------------------------")


# # Main loop
# leftWheel.setVelocity(0.0)
# rightWheel.setVelocity(0.0)

# explore_map()


# Define grid map
TILE_SIZE = 12  # Size of each grid cell in cm
GRID_SIZE = 20  # Number of cells in each dimension
grid_map = [[0 for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]  # 0 = unvisited, 1 = visited, 2 = obstacle
visited_tiles = set()

def get_current_grid_position():
    """
    Convert the robot's GPS position to grid coordinates.
    """
    x = int(gps_readings[0] * 100 // TILE_SIZE)
    z = int(gps_readings[2] * 100 // TILE_SIZE)
    return x, z

def mark_cell_as_visited(x, z):
    """
    Mark the current grid cell as visited.
    """
    if 0 <= x < GRID_SIZE and 0 <= z < GRID_SIZE:
        grid_map[x][z] = 1
        visited_tiles.add((x, z))
        print(f"Visited cell: ({x}, {z})")

def is_cell_unvisited(x, z):
    """
    Check if a grid cell is unvisited and within bounds, and not an obstacle.
    """
    return 0 <= x < GRID_SIZE and 0 <= z < GRID_SIZE and grid_map[x][z] == 0

def move_to_cell(target_x, target_z):
    """
    Move the robot to the target grid cell.
    """
    global gps_readings

    # Calculate the target position in cm
    target_x_cm = target_x * TILE_SIZE
    target_z_cm = target_z * TILE_SIZE

    while robot.step(timeStep) != -1:  # Ensure the simulation progresses
        get_gps_readings()
        current_x_cm = gps_readings[0] * 100
        current_z_cm = gps_readings[2] * 100

        # Check if the robot has reached the target cell
        if abs(current_x_cm - target_x_cm) < 2 and abs(current_z_cm - target_z_cm) < 2:
            stop()
            return

        # Move toward the target cell
        rightWheel.setVelocity(3)
        leftWheel.setVelocity(3)

        # Call robot.step to ensure the simulation progresses
        robot.step(timeStep)

def explore_map_with_dfs():
    """
    Explore the map using a DFS algorithm with obstacle detection.
    """
    stack = []  # Stack for DFS
    current_x, current_z = get_current_grid_position()
    stack.append((current_x, current_z))

    while stack and robot.step(timeStep) != -1:  # Ensure the simulation progresses
        # Get the current cell
        current_x, current_z = stack.pop()

        # Mark the current cell as visited
        mark_cell_as_visited(current_x, current_z)

        print(f"Current cell: ({current_x}, {current_z})")

        # Explore neighbors (front, back, left, right)
        for dx, dz in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            neighbor_x = current_x + dx
            neighbor_z = current_z + dz

            # Skip if the neighbor is already visited or is an obstacle
            if not is_cell_unvisited(neighbor_x, neighbor_z):
                continue

            # Check for obstacles using lidar
            if is_obstacle_in_direction(dx, dz):
                print(f"Obstacle detected at ({neighbor_x}, {neighbor_z}). Marking as blocked.")
                grid_map[neighbor_x][neighbor_z] = 2  # Mark as obstacle
                continue

            print(f"Moving to cell: ({neighbor_x}, {neighbor_z})")

            # Move to the neighbor cell
            move_to_cell(neighbor_x, neighbor_z)

            # Add the neighbor to the stack
            stack.append((neighbor_x, neighbor_z))


def is_obstacle_in_direction(dx, dz):
    """
    Check if there is an obstacle in the direction (dx, dz) using lidar data.
    """
    global lidar_values

    # Map direction to lidar index
    if dx == -1:  # Moving left
        lidar_index = 383  # Left direction in lidar
    elif dx == 1:  # Moving right
        lidar_index = 127  # Right direction in lidar
    elif dz == -1:  # Moving backward
        lidar_index = 255  # Back direction in lidar
    elif dz == 1:  # Moving forward
        lidar_index = 0  # Front direction in lidar
    else:
        return False

    # Check if the lidar value in the direction is below the threshold
    OBSTACLE_THRESHOLD = 8  # Threshold in cm
    return lidar_values[2][lidar_index] < OBSTACLE_THRESHOLD

# Main loop
leftWheel.setVelocity(0.0)
rightWheel.setVelocity(0.0)

explore_map_with_dfs()