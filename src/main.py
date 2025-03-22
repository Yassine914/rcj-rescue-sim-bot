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
    # detect signs in the environment
    # determine type (victim, hazard)
    # call ckeck_victims() or check_hazards()
    # report to emitter
    
    # get image from rcam
    img = rcam.getImage()
    # img = np.frombuffer(img, np.uint8).reshape((rcam.getHeight(), rcam.getWidth(), 4))
    check_victims(img, rcam);
    
    # get image from lcam
    img = lcam.getImage()
    # img = np.frombuffer(img, np.uint8).reshape((lcam.getHeight(), lcam.getWidth(), 4))
    check_victims(img, rcam);
    pass

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

def check_victim(img):
    img = np.frombuffer(img, np.uint8).reshape((cam.getHeight(), cam.getWidth(), 4))
    img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    img, thresh = cv.threshold(img, 80, 255, cv.THRESH_BINARY_INV)
    
    contours, heirarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    
    for cnt in contours:
        x, y, w, h = cv.boundingRect(cnt)
        contArea = cv.contourArea(cnt)
        ratio = w / h
        
        if contArea > 300 and contArea < 1000 and ratio > 0.65 and ratio < 0.95:
            return True
    return False


def report(victim_type):
    # stop robot for 1300ms
    stop(1300)
    
    victim_type = bytes(victim_type, "utf-8")
    pos_x = int(gps.getValues()[0] * 100) # convert to cm
    pos_z = int(gps.getValues()[2] * 100)
    
    msg = struct.pack("i i c", pos_x, pos_z, victim_type)
    emitter.send(msg)
    robot.step(timestep)
    
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

# region main
while robot.step(timestep) != -1:
    speeds[0] = max_velocity;
    speeds[1] = max_velocity;
    
    get_sensor_data();
    
    detect_signs();
    
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