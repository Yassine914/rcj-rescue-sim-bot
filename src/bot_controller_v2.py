from collections import deque
from controller import Robot
import math
import numpy as np
import cv2 as cv
import struct
import random

# region globals

RED_THRESHOLD = 30  # The threshold for the red color in the image
TILE_WIDTH = 12 # the width of the tile in cm
BLACK_THRESHOLD = 60 # The threshold for the black color in the image (holes)

timestep = 32
max_velocity = 6.28
robot = Robot()

# Define the wheels 
wheel1 = robot.getDevice("wheel1 motor")   # Create an object to control the left wheel
wheel2 = robot.getDevice("wheel2 motor") # Create an object to control the right wheel

# Set the wheels to have infinite rotation 
wheel1.setPosition(float("inf"))       
wheel2.setPosition(float("inf"))

# initialise the sensors
gps = robot.getDevice("gps")
compass = robot.getDevice("compass")
camera_right = robot.getDevice("camera_r")
camera_left = robot.getDevice("camera_l")
color_sensor = robot.getDevice("colour_sensor")
lidar = robot.getDevice("lidar")
emitter = robot.getDevice("emitter")

# enable the sensors
gps.enable(timestep)
compass.enable(timestep)
camera_right.enable(timestep)
camera_left.enable(timestep)
color_sensor.enable(timestep)
lidar.enable(timestep)

gps_readings = [0, 0, 0]
compass_value = 0
color_sensor_values = [0, 0, 0]
lidar_values = []
lidar_front = False
lidar_back = False
lidar_right = False
lidar_left = False

scanned_signs = []

img_right = None
img_left = None

curr_area = 1

total_time_passed = 0

# endregion

# region sensor_data

# a function to add the gps readings to the gps_readings list
def get_gps_readings():
    gps_readings[0] = gps.getValues()[0]*100
    gps_readings[1] = gps.getValues()[1]*100
    gps_readings[2] = gps.getValues()[2]*100

# a function to add the compass value to the compass_value list
def get_compass_value():
    global compass_value

    compass_value = compass.getRollPitchYaw()[2]
    compass_value = compass_value * 180 / math.pi  # convert to degrees
    compass_value = round(compass_value, 1)

# a function to add the camera images to the img_right and img_left lists
def get_camera_image():
    global img_right,img_left

    img_right = camera_right.getImage()
    img_left =  camera_left.getImage()

# a function to add the colour sensor values to the color_sensor_values list
def get_colour_sensor_value():
    image = color_sensor.getImage()
    r = color_sensor.imageGetRed(image, 1, 0, 0)
    g = color_sensor.imageGetGreen(image, 1, 0, 0)
    b = color_sensor.imageGetBlue(image, 1, 0, 0)

    color_sensor_values[0] = r
    color_sensor_values[1] = g
    color_sensor_values[2] = b


def get_lidar_values():
    global lidar_values
    # Loop on lidar data and add it to a 2D array
    # empty the array
    lidar_values = []
    range_image = lidar.getRangeImage() # 1d 2048 values
    # 2048 --> 4 layers * 512 points

    for layer in range(4):
        lidar_values.append([])
        for point in range(512):
            lidar_values[layer].append(
                round(
                    range_image[layer * 512 + point] * 100,
                    2
                )
            )

def get_all_sesnor_values():
    # a function to abstract getting all sensor values
    get_compass_value()
    get_lidar_values()
    get_colour_sensor_value()
    get_gps_readings()
    get_camera_image()
    get_lidar_directions()

def get_lidar_directions():
    global lidar_front,lidar_back,lidar_left,lidar_right
    # every robot step, we reset the lidar values to False and recheck them
    lidar_back = False
    lidar_front = False
    lidar_right = False
    lidar_left = False

    # we check a range of rays in the front, back, left, and right of the robot (15 rays in each direction)
    # if any of the rays is less than 7, then we set the corresponding lidar value to True
    # which means that there is an object in that direction
    for i in range(-15,15):
        if lidar_values[2][i] < 7:
            lidar_front = True
            # print(lidar_values[2][0])

    for i in range(112, 142):
        if lidar_values[2][i] < 7:
            lidar_right = True
            # print(lidar_values[2][127])


    for i in range(246 - 5, 266 + 5):
        if lidar_values[2][i] < 7:
            lidar_back = True
            # print(lidar_values[2][256])


    for i in range(373 - 5, 393 + 5):
        if lidar_values[2][i] < 7:
            lidar_left = True
            # print(lidar_values[2][3])
            # print(lidar_values[2][370])

current_area_number = 1
mp = {} #stores color el tile dy
color_done={}
area_done={}
#dont call this one
def Color_tile(tile_id, color):
    if color!=0:
        pass
    tile_value = [[color,0,color],[0,0,0],[color,0,color]]
    mp[tile_id] = tile_value
    color_done[color]=True

#byshof kona fen w el robot 2ary anhy color fa byreturn el robot lw dakhal 7yro7 fen
def get_area(color):
    if color==2:                            #red
        if current_area_number==3:
            return 4
        return 3
    elif color==3:                          #green  
        if current_area_number==1:
            return 4
        return 1
    elif color==4:                          #blue
        if current_area_number==2:
            return 1
        return 2
    elif color==5:                          #purple
        if current_area_number==3:
            return 2
        return 3
    elif color==7:                          #Orange
        if current_area_number==4:
            return 2
        return 4
    elif color==8:                          #Yellow
        if current_area_number==1:
            return 3
        return 1
target_color = 0

curr_area = 1

def get_color_sensor_color() -> str:
    global curr_area
    image = color_sensor.getImage()

    r = color_sensor.imageGetRed(image, 1, 0, 0)
    g = color_sensor.imageGetGreen(image, 1, 0, 0)
    b = color_sensor.imageGetBlue(image, 1, 0, 0)

    if abs(r - g) <= 10 and abs(g - b) <= 10 and abs(r - b) <= 10 and r>=200:
        return 'white'
    elif abs(r - g) <= 10 and abs(g - b) <= 10 and abs(r - b) <= 10 and r<=50:
        return 'black'
    elif r<=120 and b<=120 and g<=120:
        return 'grey'
    elif r > g and r > b and abs(g-b) <=10 and g<=80 and b<=80 and r>=200:
        return 'red'
    elif g > r and g > b and abs(r-b) <=10 and r<=80 and b<=80 and g>=200:
        return 'green'
    elif b > r and b > g and abs(r-g) <=10 and r<=80 and g<=80 and b>=200:
        return 'blue'
    elif r>=100 and g<=100 and b>=100:
        return 'purple'
    elif r>=200 and 200<=g<=220 and b<=100:
        return 'orange'
    elif r>=200 and g>=220 and b<=100:
        return 'yellow'
    elif r>=100 and g>=100 and b<=100:
        return 'swamp'
    else:
        return None
    


def change_area(col):
    global curr_area
    if curr_area == 1:
        match col:
            case 'blue': curr_area = 2
            case 'yellow': curr_area = 3
            case 'green': curr_area = 4
            case _: curr_area = 1
    elif curr_area == 2:
        match col:
            case 'blue': curr_area = 1
            case 'purple': curr_area = 3
            case 'orange': curr_area = 4
            case _: curr_area = 2
    elif curr_area == 3:
        match col:
            case 'yellow': curr_area = 1
            case 'purple': curr_area = 2
            case 'red': curr_area = 4
            case _: curr_area = 3
    elif curr_area == 4:
        match col:
            case 'green': curr_area = 1
            case 'orange': curr_area = 2
            case 'red': curr_area = 3
            case _: curr_area = 4
            
    print("area::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: ", curr_area)

# def will_go_out_of_area1():
#     image = color.getImage()
#     r = color.imageGetRed(image, 1, 0, 0)
#     g = color.imageGetGreen(image, 1, 0, 0)
#     b = color.imageGetBlue(image, 1, 0, 0)
    # print("color prinitng here",r,g,b)
#     if r==28 and g<=240 and b==28:
        # print("green")
#         return True
#     if 53<=r<=54 and 53<=g<=54 and 244<=b<=245:
        
        # print("blue")
#         return True
#     return False


# #function hany 3amlha to return lw e7na 3nd black el hya trap/hole
# ##needs to be utilised in the code
# def detect_trap():
#     image = color.getImage()
#     if image is None:
#         return False
#     return b'###\xff' in image

# endregion

# region movement

def turn_90(right = True):

    # in this function, first we set the wheel velocities and the angle the robot should move to
    # then we start moving the robot using the while loop

    # round current robot angle to the nearest multiple of 90 (0, 90, 180, -90)
    compass_value_rounded_to_nearest_90 = round(compass_value / 90) * 90

    # then add or subtract 90 to get the next angle the robot should move to
    
    if right: # subtract 90 if the robot should turn right
        next_angle = compass_value_rounded_to_nearest_90 - 90
    else: # add 90 if the robot should turn left
        next_angle = compass_value_rounded_to_nearest_90 + 90


    # to make sure that the angle is between -180 to 180
    if next_angle > 180:
        next_angle -= 360
    elif next_angle < -180:
        next_angle += 360

    # see if robot should turn left or right then set the wheel velocities accordingly
    if right:
        s1 =-3
        s2 = 3
    else:
        s1 = 3
        s2 = -3

    # start moving the robot
    while robot.step(timestep) != -1:
        # whenever the robot is moving, we should get the sensor values to update the global variables
        get_all_sesnor_values()
        detect_victims(img_right, camera_right)
        detect_victims(img_left, camera_left)

        # move the robot with the calculated wheel velocities
        wheel1.setVelocity(s1)
        wheel2.setVelocity(s2)

        # check if robot is close to the next angle he should move to (if difference is smaller than 7)
        if abs(compass_value - next_angle) < 7:
            # robot is close to next angle then we should break the loop
            break

def stop(duration):
    # we call robot.step but with wheel velocities set to 0
    # the simulation will keep running but the robot will stop

    stop = duration
    while robot.step(timestep) != -1:
        # keep looping until 5000ms pass then break the loop
        wheel1.setVelocity(0)
        wheel2.setVelocity(0)
        stop -= timestep
        if stop <= 0:
            break

start = robot.getTime()

def move_one_tile(tile_size=TILE_WIDTH):
    global coords
    
    #if tile_size == TILE_WIDTH and curr_area != 1:
    tile_size = TILE_WIDTH / 2
    
    compass_value_rounded_to_nearest_90 = round(compass_value / 90) * 90

    # get the current x and y of the robot
    x = gps_readings[0]
    y = gps_readings[2]

    # round x and y to nearest multiple of 12
    x = round(x / tile_size) * tile_size 
    y = round(y / tile_size) * tile_size 

    # save_coords(x, y);
    
    # if the robot is facing horizontally (90 or -90) we should move in the x direction, so x should change and y should stay the same
    # if the robot is facing vertically (0 or 180) we should move in the y direction, so y should change and x should stay the same

    if compass_value_rounded_to_nearest_90 in (90, -90):
        if compass_value_rounded_to_nearest_90 == 90:
            # if the robot is facing 90, then we should move to the left, so we subtract 12 from x
            x_new = x -  tile_size
            y_new = y
        else:
            # if the robot is facing -90, then we should move to the right, so we add 12 to x
            x_new = x + tile_size 
            y_new = y

    else:
        # if the robot is facing 0, then we should move up, so we subtract 12 from y
        if compass_value_rounded_to_nearest_90 == 0:
            x_new = x
            y_new = y - tile_size
        else:
            # if the robot is facing 180, then we should move down, so we add 12 to y
            x_new = x
            y_new = y + tile_size
            
    while robot.step(timestep) != -1:

        # whenever the robot is moving, we should get the sensor values to update the global variables
        get_all_sesnor_values()
        print_info()
        detect_victims(img_right, camera_right)
        detect_victims(img_left, camera_left)

        # calculate the angle difference between the robot's current angle and the angle he should move to
        # To see if the robot is inclined to the right or left
        angle_difference = compass_value_rounded_to_nearest_90 - compass_value

        # we should make sure that the angle difference is between -180 to 180
        if angle_difference > 180:
            angle_difference -= 360
        elif angle_difference < -180:
            angle_difference += 360

        # in order to make sure the robot is moving straight,we should prevent the robot from inclining to the right or left
        # if the robot is inclined to the right, we should make the robot move slightly to the left
        if angle_difference > 0:
            s1 = 6.28
            s2 = 4
        else:
            # if the robot is inclined to the left, we should make the robot move slightly to the right
            s1 = 4
            s2 = 6.28

        # start moving the robot with the calculated wheel velocities
        wheel1.setVelocity(s1)
        wheel2.setVelocity(s2)


        # the robot stops moving in 3 cases:
        # 1. the robot sees an object in front of him
        # 2. the robot reaches the new x coordinate (if he is moving horizontally) or the new y coordinate (if he is moving vertically)
        # 3. there is a hole infront of the robot (we check the colour sensor to see if it is black)

        # if the robot sees an object in front of him, we break the loop, then the function will end
        if lidar_front:
            break
        
        # we look at the current robot position and the new position he should move to (if the difference is smaller than 1, then the robot reached the new position)
        # if the robot is moving horizontally, we should check if the robot reached the new x coordinate

        if compass_value_rounded_to_nearest_90 in (90, -90):
            if x_new - 1 < gps_readings[0] < x_new + 1:
                break
        else :
            # if the robot is moving vertically, we should check if the robot reached the new y coordinate
            if y_new - 1 < gps_readings[2] < y_new + 1:
                break
            
        # return "hole" if color sensor reads black
        if color_sensor_values[0] < BLACK_THRESHOLD and color_sensor_values[1] < BLACK_THRESHOLD and color_sensor_values[2] < BLACK_THRESHOLD:
            add_to_map(*get_grid_coords(), '2')
            return "hole"

# endregion

# region detect_signs
def should_scan():
    # check if the robot has scanned the sign before
    # loop the scanned signs check for a near colunms
    for point in scanned_signs:
        dist = math.sqrt(
            math.pow(gps_readings[0] - point[0], 2) +
            math.pow(gps_readings[2] - point[1], 2)
        )
        if dist < 10: # no 2 signs are closer than 10cm so we should be at an already scanned sign
            return False
    return True

def report(type):
    global scanned_signs, pos, gps, scanned_signs
    print("-----------------reporting sign-------------------------------")
    
    pos = gps.getValues()
    pos_x = int(pos[0] * 100)
    pos_z = int(pos[2] * 100)
    scanned_signs.append((pos_x, pos_z))
    
    sign_type = bytes(type, "utf-8")
    msg = struct.pack("i i c", pos_x, pos_z, sign_type)  # Pack the message.
    
    emitter.send(msg)
    robot.step(timestep)  # Wait for the message to be sent.
    
    print("------------------------------Victim detected at: ", pos_x, pos_z, "of type: ", type)
    

def detect(img):
    print("-----------------detecting sign-------------------------------")
    
    # check it has any red
    
    # shape = detect_shape(img)
    # if shape == 'S':
    #     sign_type = detect_letters(img)
    # else:
    #     sign_type = detect_F_O(img)
    #     if sign_type == 'N':
    #         sign_type = detect_P_C(img)
    stop(2500)
    
    sign_type = detect_F_O(img)
    
    # if no red then it might be a letter
    print("_____________ SHAPE: ___________________ :", detect_shape(img));
    # FIXME:
    if sign_type == 'N':
        sign_type = detect_letters_old(img)
        # sign_type, bottom = detect_letters(img)
        
    # if it's not a letter then it must be P or C
    if sign_type == 'N':
        sign_type = detect_P_C(img)
        
    if sign_type != 'N':
        report(sign_type);
        print("-------------------------------------sign type: ", sign_type)

    # need to stop the robot for some time to detect the sign
    return

def detect_victims(image_data, camera):
    coords_list = []
    img = np.array(np.frombuffer(image_data, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4)))
    img = cv.cvtColor(img, cv.COLOR_BGRA2BGR)

    # convert from BGR to HSV color space
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    # apply threshold
    thresh = cv.threshold(gray, 140, 255, cv.THRESH_BINARY)[1]

    # draw all contours in green and accepted ones in red
    contours, h = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    # Iterate over the detected contours
    for contour in contours:
        # Calculate the bounding rectangle of the contour
        x, y, w, h = cv.boundingRect(contour)

        # Check if the contour is large enough to be a sign
        if cv.contourArea(contour) > 500 and w / h < 2 and 5 < x < 25 and y < 25:
            # Crop the image
            cropped_image = img[y:y + h, x:x + w]
            if should_scan(): # Check if the robot has scanned the sign before
                detect(cropped_image)


# NOTE: returns S (Square), R (Rhombus), or N (Not S or R)
def detect_shape(sign_colored) -> str:

    # Convert to grayscale and apply thresholding
    if sign_colored is None:
        return 'N'
        
    gray = cv.cvtColor(sign_colored, cv.COLOR_BGR2GRAY)
    _, thresh = cv.threshold(gray, 80, 255, cv.THRESH_BINARY_INV)
    
    # Find contours
    contours, _ = cv.findContours(thresh, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    
    for contour in contours:
        # Approximate the contour to a polygon
        perimeter = cv.arcLength(contour, True)
        epsilon = 0.04 * perimeter  # Precision parameter
        approx = cv.approxPolyDP(contour, epsilon, True)
        
        # Check if the polygon has 4 vertices (both square and rhombus have 4 vertices)
        if len(approx) == 4:
            # Calculate the angles between the vertices
            angles = []
            for i in range(4):
                p1 = approx[i][0]
                p2 = approx[(i + 1) % 4][0]
                p3 = approx[(i + 2) % 4][0]
                
                # Calculate vectors
                v1 = np.array([p2[0] - p1[0], p2[1] - p1[1]])
                v2 = np.array([p3[0] - p2[0], p3[1] - p2[1]])
                
                # Calculate angle using dot product
                dot = np.dot(v1, v2)
                mag1 = np.linalg.norm(v1)
                mag2 = np.linalg.norm(v2)
                
                # Avoid division by zero
                if mag1 * mag2 == 0:
                    continue
                    
                cos_angle = dot / (mag1 * mag2)
                # Clamp to avoid numerical errors
                cos_angle = max(-1, min(1, cos_angle))
                angle = np.degrees(np.arccos(cos_angle))
                angles.append(angle)
            
            # Check if it's a square (all angles close to 90 degrees)
            if all(abs(angle - 90) < 10 for angle in angles):
                print("Square detected - angles:", angles)
                return 'S'
            
            # Check if it's a rhombus (opposite angles are equal)
            elif len(angles) == 4 and abs(angles[0] - angles[2]) < 10 and abs(angles[1] - angles[3]) < 10:
                print("Rhombus detected - angles:", angles)
                return 'R'
    
    return 'N'  # Not a square or rhombus
# NOTE: returns N (Not F or O), F (Flammable), or O (Organic)
def detect_F_O(sign_colored) -> str:
    
    sign_colored = cv.cvtColor(sign_colored, cv.COLOR_BGR2RGB)
    sign_type = 'N' # N means it wasn't F or O
    h, w, _ = sign_colored.shape
    half = h // 2
    bottom = sign_colored[half:, :]

    # mask orange color
    lower_orange = np.array([192, 142, 0])
    upper_orange = np.array([204, 190, 20])

    orange_mask = cv.inRange(bottom, lower_orange, upper_orange)
    # cv.imshow("orange mask" , orange_mask)

    # check if the orange color exists in the img
    pixels = cv.countNonZero(orange_mask)
    if pixels > 10:
        sign_type = 'O'

    # mask red color
    lower_red = np.array([185, 0, 0])
    upper_red = np.array([205, 100, 118])

    red_mask = cv.inRange(bottom, lower_red, upper_red)

    # check if the red color exists in the img
    pixels = cv.countNonZero(red_mask)

    if pixels > 25:
        sign_type = 'F'

    return sign_type

# NOTE: returns P (Poison), or C (Corrosive)
def detect_P_C(sign) -> str:
    # make the background in gray
    sign = cv.cvtColor(sign, cv.COLOR_BGR2GRAY)
    h, w = sign.shape
    # get bottom half of the image
    bottom = sign[int(h * 0.5):, int(w * 0.3):int(w * 0.85)]
    white_pixel = cv.countNonZero(bottom)
    black_pixel = bottom.size - white_pixel

    if black_pixel > white_pixel:
        return 'C'
    else:
        return 'P'

def detect_letters2(sign) -> str:
    img = cv.cvtColor(sign, cv.COLOR_BGR2GRAY)
    _, thresh = cv.threshold(img, 80, 255, cv.THRESH_BINARY_INV)
    
    height, width = thresh.shape
    section_width = width // 3
    
    first_section = thresh[:, :section_width]
    middle_section = thresh[:, section_width:2 * section_width]
    third_section = thresh[:, 2 * section_width:]
    
    contours_first, _ = cv.findContours(first_section, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    contours_middle, _ = cv.findContours(middle_section, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    contours_third, _ = cv.findContours(third_section, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    
    # Output images for debugging
    # cv.imshow("First Section", first_section)
    # cv.imshow("Middle Section", middle_section)
    # cv.imshow("Third Section", third_section)
    # cv.waitKey(1)
    
    num_contours_first = len(contours_first)
    num_contours_middle = len(contours_middle)
    num_contours_third = len(contours_third)
    
    print("First Section Contours: ", num_contours_first)
    print("Middle Section Contours: ", num_contours_middle)
    print("Third Section Contours: ", num_contours_third)
    
    # H has vertical lines on both sides (1 contour each) and a horizontal line in the middle
    if num_contours_first == 2 and num_contours_middle == 1 and num_contours_third == 2:
        return 'H'
    
    # S has a distinctive pattern with typically 2 contours in each section
    elif num_contours_first == 1 and num_contours_third == 1:
        return 'S'
    
    # U has vertical lines on both sides (1 contour each) with a curve at the bottom
    elif num_contours_first == 1 and num_contours_third == 1:
        # Additional check for the bottom curve
        bottom_third = height * 2 // 3
        bottom_middle = middle_section[bottom_third:, :]
        bottom_pixels = cv.countNonZero(bottom_middle)
        if bottom_pixels > 10:  # There should be pixels in the bottom middle for U
            return 'U'
    
    # If no pattern matches
    return 'N'

def detect_letters_old(sign) -> str:
    img = cv.cvtColor(sign, cv.COLOR_BGR2GRAY)
    _, thresh = cv.threshold(img, 80, 255, cv.THRESH_BINARY_INV)
    
    cv.imwrite("sign.png", img)
    
    height, width = thresh.shape
    section_width = width // 3

    first_section = thresh[:, :section_width]
    middle_section = thresh[:, section_width:2 * section_width]
    third_section = thresh[:, 2 * section_width:]

    contours_first, _ = cv.findContours(first_section, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    contours_middle, _ = cv.findContours(middle_section, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    contours_third, _ = cv.findContours(third_section, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

    # output image for debugging
    # cv.imshow("First Section", first_section)
    # cv.imshow("Third Section", third_section)
    # cv.waitKey(1)

    num_contours_first = len(contours_first)
    num_contours_middle = len(contours_middle)
    num_contours_third = len(contours_third)

    print("First Section Contours: ", num_contours_first)
    print("Middle Section Contours: ", num_contours_middle)
    print("Third Section Contours: ", num_contours_third)
    
    if num_contours_first == 3 and num_contours_third == 3:
        return 'U'
    elif num_contours_first == 1 and num_contours_third == 1:
        return 'H'
    elif num_contours_first == 4 and num_contours_third == 4:
        return 'S'
    else:
        return 'N' # return N for not found

# NOTE: returns N (Not H, S, U), H, S, or U
def detect_letters(sign) -> str:
    # FIXME: I think letter is not correct somehow (it should have a height and a width)
    # invert the image
    sign = cv.cvtColor(sign, cv.COLOR_BGR2GRAY)
    letter = cv.bitwise_not(sign)

    # filling the background of the img with black
    h, w = letter.shape
    for x in range(0, h):
        for y in range(0, w):
            pixel = letter[x, y]

            if pixel < 20:
                break
            else:
                letter[x, y] = (0)

        for y_inversed in range(w - 1, 0, -1):
            pixel = letter[x, y_inversed]
            if pixel < 20:
                break
            else:
                letter[x, y_inversed] = (0)

    # find contours in the letter img
    cnts = cv.findContours(letter, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]

    for c in cnts:
        (x, y, w, h) = cv.boundingRect(c)
        # crop the img to the letter itself
        letter = letter[y:y + h, x:x + w]

    # letter detection
    h, w = letter.shape
    letter_type = 'N'

    third = h // 3
    top = letter[:third, :]
    middle = letter[third:third * 2, :]
    bottom = letter[third * 2:, :]

    # finding contours in the three part of the image to be able to recognize the letter
    cnts = cv.findContours(top, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]
    c1 = (len(cnts))

    cnts = cv.findContours(middle, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]
    c2 = (len(cnts))

    cnts = cv.findContours(bottom, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]
    c3 = (len(cnts))
    
    # print contours
    print("________________________________________________________________")
    print("Top: ", c1, "Middle: ", c2, "Bottom: ", c3)

    # check whether a letter is detected and change the bool value if yes
    # print ("LETTER CODE: ", c1,c2,c3)
    if c1 == 1 and c3 == 1:
        letter_type = 'S'
    elif c1 == 2 and c2 == 1 and c3 == 2:
        letter_type = 'H'
    elif c1 == 2 and c2 == 2 and c3 == 1:
        letter_type = 'U'

    return letter_type, bottom


def detect_letters3(sign) -> str:
    sign = cv.cvtColor(sign, cv.COLOR_BGR2GRAY)
    # inverse the img to color the letter(if the img is human) in white to be able to find contours in it
    letter = cv.bitwise_not(sign)
    # cv.imshow("letter" , letter)
    # save letter
    
    # cv.imwrite("sign.png", sign)
    # cv.imwrite("letter.png", letter)

    # filling the background of the img with black
    h, w = letter.shape
    for x in range(0, h):
        for y in range(0, w):
            pixel = letter[x, y]

            if pixel < 20:
                break
            else:
                letter[x, y] = (0)

        for y_inversed in range(w - 1, 0, -1):
            pixel = letter[x, y_inversed]
            if pixel < 20:
                break
            else:
                letter[x, y_inversed] = (0)

    # find contours in the letter img
    cnts = cv.findContours(letter, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]

    for i, c in enumerate(cnts):
        cv.imwrite("contour_{i}.png", c)
        (x, y, w, h) = cv.boundingRect(c)
        # crop the img to the letter itself
        letter = letter[y:y + h, x:x + w]

    # letter detection
    h, w = letter.shape
    letter_type = "N"

    third = h // 3
    top = letter[:third, :]
    middle = letter[third:third * 2, :]
    bottom = letter[third * 2:, :]

    # finding contours in the three part of the image to be able to recognize the letter
    cnts = cv.findContours(top, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]
    c1 = (len(cnts))

    cnts = cv.findContours(middle, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]
    c2 = (len(cnts))

    cnts = cv.findContours(bottom, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]
    c3 = (len(cnts))
    
    print("First Section Contours: ",  c1)
    print("Middle Section Contours: ", c2)
    print("Third Section Contours: ",  c3)

    # check whether a letter is detected and change the bool value if yes
    # print ("LETTER CODE: ", c1,c2,c3)
    if c1 == 1 and c3 == 1:
        # print("S victim")
        letter_type = "S"
    elif c1 == 2 and c2 == 1 and c3 == 2:
        # print("H victim")
        letter_type = "H"

    elif c1 == 2 and c2 == 2 and c3 == 1:
        # print("U victim")
        letter_type = "U"

    return letter_type

# endregion

# region movement_2

def navigate():
    global counter

def print_info():
    return
    print("---------------------------------")
    print(f"Compass value:  {compass_value}")
    print(f"gps readings:  {gps_readings}")
    print(f"Color sensor values:  {color_sensor_values}")
    print("---------------------------------")


coords = set()

def current_coords():
    get_gps_readings()
    # return the current coordinates of the robot
    x = round(gps_readings[0] / TILE_WIDTH) * TILE_WIDTH
    y = round(gps_readings[2] / TILE_WIDTH) * TILE_WIDTH
    return x, y

def save_coords(x, y):
    global coords
    coords.add((x, y))
    
def passed():
    cx, cy = current_coords()
    for (i, j) in coords:
        if math.sqrt(
            math.pow(cx - i, 2) +
            math.pow(cy - j, 2)
        ) < TILE_WIDTH / 2:
            print("___________________________ COORDS: ", i, j, " PASSED");
            return True
        
    return False

def get_next_coords(x, y, direction):
    # Get the compass value rounded to the nearest 90 degrees
    compass_value_rounded = round(compass_value / 90) * 90
    
    # Adjust direction based on current orientation
    if direction == "front":
        # Keep same direction
        new_dir = compass_value_rounded
    elif direction == "right":
        # Turn 90 degrees right
        new_dir = compass_value_rounded - 90
    elif direction == "left":
        # Turn 90 degrees left
        new_dir = compass_value_rounded + 90
    else:
        # Turn around (180 degrees)
        new_dir = compass_value_rounded + 180
    
    # Normalize angle to -180 to 180 range
    if new_dir > 180:
        new_dir -= 360
    elif new_dir < -180:
        new_dir += 360
    
    # Calculate next coordinates based on new direction
    if new_dir == 0:  # North
        return x, y - TILE_WIDTH
    elif new_dir == 90:  # East
        return x - TILE_WIDTH, y
    elif new_dir == -90:  # West
        return x + TILE_WIDTH, y
    else:  # South (180 or -180)
        return x, y + TILE_WIDTH
    
MAP_CONSTANT = 50 # Add 50 to idx to make them +ve

max_x = 1 + MAP_CONSTANT
max_y = 1 + MAP_CONSTANT
min_x = 0 + MAP_CONSTANT
min_y = 0 + MAP_CONSTANT


class Tile:
    def __init__(self, type, n, s, e, w):
        self.type = type
        self.n = n
        self.s = s
        self.e = e
        self.w = w
        
    def N(self):
        return self.n
    
    def S(self):
        return self.s
    
    def E(self):
        return self.e
    
    def W(self):
        return self.w
    
    def Type(self):
        return self.type
 
grid = [[Tile('-1', False, False, False, False) for _ in range(300)] for _ in range(300)]

def has_explored_most_of_the_map() -> bool:
    # return False;
    global grid, min_x, min_y, max_x, max_y
    # return False
    cnt_neg_1 =  0
    total_cnt =  0
    for i in range(min_x, max_x + 1):
        for j in range(min_y, max_y + 1):
            cnt_neg_1 += grid[i][j].type == '-1'
            total_cnt += 1
    
    ratio = (cnt_neg_1 / total_cnt) * 100
    
    print("______________________ UNEXPLORED PERCENTAGE: " + str(ratio) + "%")
    return ratio <= 10 and ratio > 0


old_x, old_y = None, None
# Zeyad was here
def move2():
    global old_x, old_y, coords
    
    # FIXME: handle getting stuck
    
    deltatime = total_time_passed
    
    cx, cy = current_coords()
    x = (cx // TILE_WIDTH) + MAP_CONSTANT
    y = (cy // TILE_WIDTH) + MAP_CONSTANT

    current_visited = passed()
    
    
    # Save the current coordinates as visited if not already visited
    if not current_visited:
        save_coords(cx, cy)
        
        # TODO: get color from color sensor
        
        print(f"-----------------------------------saving new tile: ({cx}, {cy})")
    
    # Track attempt count to prevent infinite loops
    attempt_count = 0
    max_attempts = 4  # Maximum number of attempts before forcing a move
    
    # Repeat until a successful move or max attempts reached
    while attempt_count < max_attempts:
        deltatime = total_time_passed - deltatime
        
        if deltatime > 1000:
            turn_90()
            turn_90()
            move_one_tile(tile_size=3)
            turn_90(right=False)
            
        attempt_count += 1
        print(f"Movement attempt {attempt_count} of {max_attempts}")
        
        # Create a priority list of possible movement directions
        directions = []
        
        # Check which directions are free and add them to the priority list
        if not lidar_right:
            directions.append(("right", False))
        if not lidar_front:
            directions.append(("front", False))
        if not lidar_left:
            directions.append(("left", False))
        # if not lidar_back:
            # directions.append(("back", False))
        
        # If no directions available (surrounded by walls), force a turn
        if not directions:
            print("All directions blocked by walls, turning around")
            turn_90(right=False)
            # turn_90()
            move_one_tile()
            return
        
        # Mark which directions lead to visited tiles
        for i, (direction, _) in enumerate(directions):
            next_x, next_y = get_next_coords(cx, cy, direction)
            for visited_x, visited_y in coords:
                if math.sqrt(math.pow(next_x - visited_x, 2) + math.pow(next_y - visited_y, 2)) < TILE_WIDTH / 2:
                    directions[i] = (direction, True)
                    break
        
        # Find the best direction to move
        unvisited_directions = [d for d, visited in directions if not visited]
        
        if unvisited_directions:
            best_direction = unvisited_directions[0]
            print(f"Moving to unvisited direction: {best_direction}")
        elif directions:
            # best_direction = directions[0][0];
            # NOTE:
            # print(f"All directions visited, using priority direction: {best_direction}")
            print("ALL DIRECTIONS VISITED. MOVING TO NEAREST UNVISITED TILE")
            move_to_nearest_unvisited_tile_dfs()
            return # FIXME: remove this
        else:
            # move_to_nearest_unvisited_tile_dfs2()
            # This shouldn't happen due to the earlier check, but just in case
            best_direction = "turn_around"
            print("No valid directions, turning around")
        
        # Execute the movement
        result = None
        if best_direction == "right":
            turn_90(right=True)
            result = move_one_tile()
        elif best_direction == "front":
            result = move_one_tile()
        elif best_direction == "left":
            turn_90(right=False)
            result = move_one_tile()
        elif best_direction == "back" or best_direction == "turn_around":
            turn_90()
            turn_90()
            result = move_one_tile()
            turn_90()
            turn_90()
        
        
        if has_explored_most_of_the_map():
            print("- - - - - - - - - - - we have explored most of the map - - - - - -  - - - - ")
            create_new_map()
            submit_map()
            
        
        # If successful move or max attempts reached, exit the loop
        if result != "hole":
            return
        
        # If hole detected, mark this direction as invalid and try again
        print(f"Hole detected in {best_direction} direction! Trying a different direction.")
        
        # Mark the coordinates where the hole was detected to avoid going there again
        hole_x, hole_y = get_next_coords(cx, cy, best_direction)
        save_coords(hole_x, hole_y)
        
        # Turn away from the hole and continue the loop to find a new direction
        turn_90()
        turn_90()
        move_one_tile(tile_size=3)
        move_to_nearest_unvisited_tile_dfs2()
        old_x, old_y = cx, cy
        
    
    # If we've tried all directions and still can't move, perform a random turn
    print("Maximum movement attempts reached. Performing random turn.")
    if random.choice([True, False]):
        turn_90()
    else:
        turn_90(right=False)

def detect_current_area():
    global curr_area
    get_colour_sensor_value()

def move():
    # print("___________ CURRENT: ", *current_coords());
    if passed():
        if not lidar_left:
            turn_90(right=False)
        elif not lidar_right:
            turn_90(right=True)
        else:
            turn_90()
        return
    
    save_coords(*current_coords());
    # for (i, j) in coords:
        # print("___________________________ COORDS: ", i, j, " ARE IN COORDS_SET");

    if not lidar_right:
        turn_90()
        r = move_one_tile()
        if r == "hole":
            turn_90()
            # r = random.randint(0, 1)
            # turn_90(right=False) if r else turn_90(right=True)
    elif lidar_front:
        turn_90(right=False)
        # r = random.randint(0, 1)
        # turn_90(right=False) if r else turn_90(right=True)
    else:
        r = move_one_tile()
        if r == "hole":
            turn_90()

def get_new_direction(dx, dy):
    # Get the compass value rounded to the nearest 90 degrees
    compass_value_rounded = round(compass_value / 90) * 90
    
    if dx == 0 and dy == 0 and compass_value_rounded == 0:
        return "f"
    if dx == 0 and dy == 0 and compass_value_rounded == 90:
        return "r"
    if dx == 0 and dy == 0 and compass_value_rounded == -90:
        return "l"
    if dx == 0 and dy == 0 and abs(compass_value_rounded) == 180:
        return "b"
    if dx == 0 and dy == -1 and compass_value_rounded == 0:
        return "f"
    if dx == 0 and dy == -1 and compass_value_rounded == 90:
        return "r"
    if dx == 0 and dy == -1 and compass_value_rounded == -90:
        return "l"
    if dx == 0 and dy == -1 and abs(compass_value_rounded) == 180:
        return "b"
    if dx == 0 and dy == 1 and compass_value_rounded == 0:
        return "b"
    if dx == 0 and dy == 1 and compass_value_rounded == 90:
        return "l"
    if dx == 0 and dy == 1 and compass_value_rounded == -90:
        return "r"
    if dx == 0 and dy == 1 and abs(compass_value_rounded) == 180:
        return "f"
    if dx == -1 and dy == 0 and compass_value_rounded == 0:
        return "l"
    if dx == -1 and dy == 0 and compass_value_rounded == 90:
        return "b"
    if dx == -1 and dy == 0 and compass_value_rounded == -90:
        return "f"
    if dx == -1 and dy == 0 and abs(compass_value_rounded) == 180:
        return "r"
    if dx == 1 and dy == 0 and compass_value_rounded == 0:
        return "r"
    if dx == 1 and dy == 0 and compass_value_rounded == 90:
        return "f"
    if dx == 1 and dy == 0 and compass_value_rounded == -90:
        return "b"
    if dx == 1 and dy == 0 and abs(compass_value_rounded) == 180:
        return "l"
    
    return None

def get_directions_to_unvisited_tile(cx, cy) -> str:
    path = ""
    # use bfs to find the closest unvisited tile
    q = deque()
    q.append((cx, cy, get_new_direction(0, 0)))
    
    while q:
        x, y, d = q.popleft()
        path += str(d)
        
        # check if the tile is unvisited
        if grid[x][y].type == '-1':
            return d
        
        # add possible directions to the list
        dirs = []
        if not grid[x][y].N(): # no wall North
            dirs.append((x, y - 1))
        if not grid[x][y].S(): # no wall South
            dirs.append((x, y + 1))
        if not grid[x][y].E(): # no wall East
            dirs.append((x + 1, y))
        if not grid[x][y].W(): # no wall West
            dirs.append((x - 1, y))
            
        # add neighbors to the queue
        for (dx, dy) in dirs:
            nx, ny = x + dx, y + dy
            if 0 <= nx <= max_x + 1 and 0 <= ny <= max_y + 1 and grid[nx][ny].type != '-1':
                q.append((nx, ny, d + get_new_direction(x, y, dx, dy)))
    return path
                
def get_directions_to_unvisited_tile2(cx, cy) -> str:
    
    cx = (cx // TILE_WIDTH) + MAP_CONSTANT
    cy = (cy // TILE_WIDTH) + MAP_CONSTANT
    
    # Use bfs to find the closest unvisited tile
    visited = set()
    queue = deque()
    paths = {}
    
    start_pos = (cx, cy)
    queue.append(start_pos)
    visited.add(start_pos)
    paths[start_pos] = ""
    
    compass_value_rounded = round(compass_value / 90) * 90
    if compass_value_rounded > 180:
        compass_value_rounded -= 360
    elif compass_value_rounded < -180:
        compass_value_rounded += 360
    
    while queue:
        x, y = queue.popleft()
        print("___________________________ X: ", x, " Y: ", y)
        print("___________________________ PATH: ", paths[(x, y)])
        
        if x != cx or y != cy:
            if 0 <= x <= max_x + 1 and 0 <= y <= max_y + 1 and grid[x][y].Type() == '-1':
                print("___________________________ UNVISITED TILE: ", x, y)
                return paths[(x, y)]
        
        # Add all valid neighbors to the queue
        dirs = [
            (x, y - 1, "f" if compass_value_rounded == 0 else 
                    "r" if compass_value_rounded == 90 else
                    "l" if compass_value_rounded == -90 else "b"),  # N
            (x, y + 1, "b" if compass_value_rounded == 0 else 
                    "l" if compass_value_rounded == 90 else
                    "r" if compass_value_rounded == -90 else "f"),  # S
            (x + 1, y, "r" if compass_value_rounded == 0 else 
                   "b" if compass_value_rounded == 90 else
                   "f" if compass_value_rounded == -90 else "l"),  # E
            (x - 1, y, "l" if compass_value_rounded == 0 else 
                   "f" if compass_value_rounded == 90 else
                   "b" if compass_value_rounded == -90 else "r")   # W
        ]
        
        for nx, ny, direction in dirs:
            next_pos = (nx, ny)
            if next_pos not in visited and 0 <= nx < len(grid) and 0 <= ny < len(grid[0]):
                # Check if there's no wall in this direction
                if ((direction == "f" and not grid[x][y].N()) or
                    (direction == "b" and not grid[x][y].S()) or
                    (direction == "r" and not grid[x][y].E()) or
                    (direction == "l" and not grid[x][y].W())):
                    
                    queue.append(next_pos)
                    visited.add(next_pos)
                    paths[next_pos] = paths[(x, y)] + direction
                    
    return ""

def get_grid_coords(x=None, y=None) -> tuple:
    cx, cy = current_coords() if x is None and y is None else (x, y)
    x = round(cx / TILE_WIDTH) + MAP_CONSTANT
    y = round(cy / TILE_WIDTH) + MAP_CONSTANT
    return x, y

visited = set()

def get_directions_to_unvisited_tile3() -> str:
    global max_x, max_y, min_x, min_y, grid, visited
    
    cx, cy = get_grid_coords()
    print(":::::::::::::::::::::::::: NOW GETTING DIRECTIONS TO NEAREST UNVISITED TILE ::::::::::::::::::::::::::::::")
    print("\tCURR COORDS: ", cx, cy)
    
    # Track visited nodes to avoid cycles
    
    q = deque()
    q.append((cx, cy, ""))
    visited.add((cx, cy))
    
    while q:
        x, y, path = q.popleft()
        
        x -= 1
        y -= 1
        
        # print(f"({x}, {y}) PATH: {path}, GRID: {grid[x][y].Type()}, WALLS: {grid[x][y].N()}, {grid[x][y].S()}, {grid[x][y].E()}, {grid[x][y].W()}")
        # x += 1
        # print(f"({x}, {y}) PATH: {path}, GRID: {grid[x][y].Type()}, WALLS: {grid[x][y].N()}, {grid[x][y].S()}, {grid[x][y].E()}, {grid[x][y].W()}")
        # x -= 1
        # y += 1
        # print(f"({x}, {y}) PATH: {path}, GRID: {grid[x][y].Type()}, WALLS: {grid[x][y].N()}, {grid[x][y].S()}, {grid[x][y].E()}, {grid[x][y].W()}")
        # y -= 1
        # print(f"({x}, {y}) PATH: {path}, GRID: {grid[x][y].Type()}, WALLS: {grid[x][y].N()}, {grid[x][y].S()}, {grid[x][y].E()}, {grid[x][y].W()}")

        if (x != cx or y != cy) and grid[x][y].Type() == '-1':
            print("::::::::::::: FOUND A VIABLE PATH TO AN UNVISITED TILE :::::::::::::::::")
            print("\t\tUNVISITED TILE: ", x, y, path)
            return path
            
        dirs = []
        
        add_to_map(*get_grid_coords(), '0')
        # print(f"CURRENT GRID CELLL {x}, {y} : {grid[x][y].Type()}, {grid[x][y].N()}, {grid[x][y].S()}, {grid[x][y].E()}, {grid[x][y].W()}")
        
        if not grid[x][y].N() and not lidar_front:  # no wall North
            dirs.append((x - 1, y, "f"))
        if not grid[x][y].S() and not lidar_back:  # no wall South
            dirs.append((x + 1, y, "b"))
        if not grid[x][y].E() and not lidar_right:  # no wall East
            dirs.append((x, y + 1, "r"))
        if not grid[x][y].W() and not lidar_left:  # no wall West
            dirs.append((x, y - 1, "l"))
            
        for nx, ny, d in dirs:
            if nx < 0 or ny < 0 or nx >= max_x + 1 or ny >= max_y + 1:
                continue
                
            # Skip if already visited
            if (nx, ny) in visited:
                continue
                
            # Add to visited set and queue
            visited.add((nx, ny))
            q.append((nx, ny, path + d))
            print(f"Added to queue: ({nx}, {ny}) with path: {path + d}")
        
    print("No path to unvisited tile found!")
    return ""

def move_to_nearest_unvisited_tile_dfs():
    turn_90()
    turn_90()
    
    while True:
        
        print("ENTERED DDDDDDDDDDDDDDDDDFFFFFFFFFFFFFFFFFFFSSSSSSSSSSSSSSSSSSSSS")
        cx, cy = current_coords()
        x = (cx // TILE_WIDTH) + MAP_CONSTANT
        y = (cy // TILE_WIDTH) + MAP_CONSTANT
        
        # Check if the current tile is unvisited
        if grid[x - 1][y - 1].type == '-1':
            print("Found unvisited tile at: ", x, y)
            return
        
        # Move to the next tile in the direction of the wall
        r = move_one_tile()
        if r == "hole":
            print("Found hole, turning around")
            turn_90()
            move_one_tile(tile_size=3)
        elif not lidar_front:
            move_one_tile()
            turn_90(right=False)
        elif not lidar_back:
            move_one_tile()
            turn_90(right=True)
        elif not lidar_right:
            move_one_tile()
            turn_90(right=True)
        elif not lidar_left:
            move_one_tile()
            turn_90(right=False)

def move_to_nearest_unvisited_tile_dfs2():
    turn_90()
    turn_90()
    
    # Track visited positions during this search to avoid loops
    dfs_visited = set()
    
    # Set a maximum number of iterations to prevent infinite loops
    max_iterations = 4
    iterations = 0
    
    while iterations < max_iterations:
        iterations += 1
        
        print(f"DFS iteration {iterations}/{max_iterations}")
        cx, cy = current_coords()
        x = (cx // TILE_WIDTH) + MAP_CONSTANT
        y = (cy // TILE_WIDTH) + MAP_CONSTANT
        
        # Add current position to visited set
        current_pos = (x, y)
        if current_pos in dfs_visited:
            print("Already visited this position during DFS, turning...")
            turn_90()
            continue
            
        dfs_visited.add(current_pos)
        
        # Check if the current tile is unvisited on the global grid
        if grid[x - 1][y - 1].type == '-1':
            print("Found unvisited tile at: ", x, y)
            return
        
        # Move to the next tile in the direction of the wall
        r = move_one_tile()
        if r == "hole":
            print("Found hole, turning around")
            turn_90()
            move_one_tile(tile_size=3)
        elif not lidar_front:
            move_one_tile()
            turn_90(right=False)
        elif not lidar_back:
            move_one_tile()
            turn_90(right=True)
        elif not lidar_right:
            move_one_tile()
            turn_90(right=True)
        elif not lidar_left:
            move_one_tile()
            turn_90(right=False)
        else:
            # If all directions have obstacles, turn around
            print("All directions blocked, turning around")
            turn_90()
            turn_90()
    
    print("DFS search reached maximum iterations without finding unvisited tile")
    return

def move_to_nearest_unvisited_tile():
    # get the direction to the unvisited tile
    # directions = get_directions_to_unvisited_tile2(cx, cy)
    print("________________________ ENTERED MOVE TO NEAREST UNVISITED TILE FUNCTION __________________________")
    directions = get_directions_to_unvisited_tile3()
    print("___________________________ DIRECTIONS: ", directions)
    
    if directions is None:
        print("No unvisited tiles found")
        return
    
    # # reset current direction
    # compass_value_rounded = round(compass_value / 90) * 90
    # if compass_value_rounded > 180:
    #     compass_value_rounded -= 360
    # elif compass_value_rounded < -180:
    #     compass_value_rounded += 360
    
    # while compass_value_rounded != 0:
    #     turn_90(right=False)
    #     compass_value_rounded = round(compass_value / 90) * 90
    #     if compass_value_rounded > 180:
    #         compass_value_rounded -= 360
    #     elif compass_value_rounded < -180:
    #         compass_value_rounded += 360
    
    
    # move to the unvisited tile
    for d in directions:
        if d == "f":
            move_one_tile(tile_size=8)
        elif d == "r":
            turn_90(right=True)
            move_one_tile(tile_size=8)
        elif d == "l":
            turn_90(right=False)
            move_one_tile(tile_size=8)
        elif d == "b":
            turn_90()
            turn_90()
            move_one_tile(tile_size=8)
    
def move_to_nearest_unvisited_tile_dfs_enhanced():
    """
    Enhanced DFS exploration that frequently checks for better paths
    and avoids revisiting paths taken within the same DFS.
    """
    # Track visited positions during this DFS search only
    dfs_visited = set()
    
    # Set a maximum number of iterations to prevent infinite loops
    max_iterations = 5
    iterations = 0
    
    # Track previous directions to detect cycles
    last_directions = []
    
    while iterations < max_iterations:
        iterations += 1
        
        print(f"Enhanced DFS iteration {iterations}/{max_iterations}")
        cx, cy = current_coords()
        x = round(cx / TILE_WIDTH) + MAP_CONSTANT
        y = round(cy / TILE_WIDTH) + MAP_CONSTANT
        
        # Check current position
        current_pos = (x, y)
        if current_pos in dfs_visited:
            print(f"Already visited position {current_pos} during this DFS search")
            # Choose a direction we haven't gone yet
            if not lidar_left and ("left", current_pos) not in last_directions:
                print("Trying left direction to avoid revisiting")
                turn_90(right=False)
                last_directions.append(("left", current_pos))
            elif not lidar_right and ("right", current_pos) not in last_directions:
                print("Trying right direction to avoid revisiting")
                turn_90(right=True)
                last_directions.append(("right", current_pos))
            else:
                print("No new directions, turning around")
                turn_90()
                turn_90()
            continue
            
        dfs_visited.add(current_pos)
        
        # Check if we found an unvisited tile
        if grid[x - 1][y - 1].type == '-1':
            print(f"Found unvisited tile at: {current_pos}")
            return
        
        # Look for optimal direction every half tile
        # Try directions in order: front, right, left, back
        directions = []
        
        if not lidar_front:
            directions.append(("front", False))
        if not lidar_right:
            directions.append(("right", False))
        if not lidar_left:
            directions.append(("left", False))
        if not lidar_back:
            directions.append(("back", False))
            
        # If all directions blocked, force a turn and try a small backup
        if not directions:
            print("All directions blocked, turning around")
            turn_90()
            turn_90()
            move_one_tile(tile_size=TILE_WIDTH/4)  # Tiny backup
            continue
        
        # Mark which directions were already taken in this DFS
        for i, (direction, _) in enumerate(directions):
            next_pos = None
            if direction == "front":
                next_x, next_y = get_next_coords(cx, cy, "front")
                next_pos = (round(next_x / TILE_WIDTH) + MAP_CONSTANT, 
                           round(next_y / TILE_WIDTH) + MAP_CONSTANT)
            elif direction == "right":
                next_x, next_y = get_next_coords(cx, cy, "right")
                next_pos = (round(next_x / TILE_WIDTH) + MAP_CONSTANT, 
                           round(next_y / TILE_WIDTH) + MAP_CONSTANT)
            elif direction == "left":
                next_x, next_y = get_next_coords(cx, cy, "left")
                next_pos = (round(next_x / TILE_WIDTH) + MAP_CONSTANT, 
                           round(next_y / TILE_WIDTH) + MAP_CONSTANT)
            elif direction == "back":
                next_x, next_y = get_next_coords(cx, cy, "back")
                next_pos = (round(next_x / TILE_WIDTH) + MAP_CONSTANT, 
                           round(next_y / TILE_WIDTH) + MAP_CONSTANT)
                
            if next_pos in dfs_visited:
                directions[i] = (direction, True)
        
        # Get unvisited directions
        unvisited_directions = [d for d, visited in directions if not visited]
        
        # Choose best direction
        if unvisited_directions:
            best_direction = unvisited_directions[0]
            print(f"Moving to unvisited direction: {best_direction}")
        else:
            # All directions visited in this DFS, choose first available
            best_direction = directions[0][0]
            print(f"All directions visited in this DFS, using: {best_direction}")
        
        # Move in chosen direction, but only half a tile
        if best_direction == "front":
            result = move_one_tile(tile_size=TILE_WIDTH/2)
            last_directions.append(("front", current_pos))
        elif best_direction == "right":
            turn_90(right=True)
            result = move_one_tile(tile_size=TILE_WIDTH/2)
            last_directions.append(("right", current_pos))
        elif best_direction == "left":
            turn_90(right=False)
            result = move_one_tile(tile_size=TILE_WIDTH/2)
            last_directions.append(("left", current_pos))
        elif best_direction == "back":
            turn_90()
            turn_90()
            result = move_one_tile(tile_size=TILE_WIDTH/2)
            last_directions.append(("back", current_pos))
        
        # Handle holes
        if result == "hole":
            print(f"Hole detected in {best_direction} direction!")
            # Mark hole in map
            hole_x, hole_y = get_next_coords(cx, cy, best_direction)
            hole_grid_x = round(hole_x / TILE_WIDTH) + MAP_CONSTANT
            hole_grid_y = round(hole_y / TILE_WIDTH) + MAP_CONSTANT
            dfs_visited.add((hole_grid_x, hole_grid_y))
            
            # Turn away from hole and try again
            turn_90()
            turn_90()
            move_one_tile(tile_size=TILE_WIDTH/4)  # Small backup
        
        # Limit the size of direction history
        if len(last_directions) > 10:
            last_directions = last_directions[-10:]
    
    print("Enhanced DFS reached maximum iterations without finding unvisited tile")
    return

tile_visit_counts = {}  # Dictionary to track visit count for each tile

def save_coords(x, y):
    global coords, tile_visit_counts
    coords.add((x, y))
    
    # Track visit count
    if (x, y) in tile_visit_counts:
        tile_visit_counts[(x, y)] += 1
    else:
        tile_visit_counts[(x, y)] = 1
    
    print(f"Tile ({x}, {y}) has been visited {tile_visit_counts[(x, y)]} times")

def move3():
    global old_x, old_y, coords, tile_visit_counts
    
    cx, cy = current_coords()
    x = (cx // TILE_WIDTH) + MAP_CONSTANT
    y = (cy // TILE_WIDTH) + MAP_CONSTANT

    current_visited = passed()
    
    # Save and track the current coordinates
    save_coords(cx, cy)
    if not current_visited:
        print(f"-----------------------------------saving new tile: ({cx}, {cy})")
    
    # Check if we might be in a loop (high visit count)
    in_loop = False
    if (cx, cy) in tile_visit_counts and tile_visit_counts[(cx, cy)] > 3:
        print(f" POTENTIAL LOOP DETECTED: Tile ({cx}, {cy}) has been visited {tile_visit_counts[(cx, cy)]} times")
        in_loop = True
    
    # Track attempt count to prevent infinite loops
    attempt_count = 0
    max_attempts = 4  # Maximum number of attempts before forcing a move
    
    # Repeat until a successful move or max attempts reached
    while attempt_count < max_attempts:
        attempt_count += 1
        print(f"Movement attempt {attempt_count} of {max_attempts}")
        
        # Create a list of possible movement directions
        directions = []
        
        # Check which directions are free and add them to the priority list
        if not lidar_right:
            directions.append(("right", False, 0))  # (direction, visited, visit_count)
        if not lidar_front:
            directions.append(("front", False, 0))
        if not lidar_left:
            directions.append(("left", False, 0))
        
        # If no directions available (surrounded by walls), force a turn
        if not directions:
            print("All directions blocked by walls, turning around")
            turn_90(right=False)
            move_one_tile()
            return
        
        # For each direction, check if it leads to a visited tile and its visit count
        for i, (direction, _, _) in enumerate(directions):
            next_x, next_y = get_next_coords(cx, cy, direction)
            visited = False
            visit_count = 0
            
            # Check if this direction leads to a previously visited tile
            for visited_x, visited_y in coords:
                if math.sqrt(math.pow(next_x - visited_x, 2) + math.pow(next_y - visited_y, 2)) < TILE_WIDTH / 2:
                    visited = True
                    visit_count = tile_visit_counts.get((visited_x, visited_y), 0)
                    break
            
            # Update the direction data with visit information
            directions[i] = (direction, visited, visit_count)
        
        # If we're in a loop, prioritize least visited tiles regardless of unvisited status
        if in_loop:
            # Sort directions solely by visit count when in a loop
            directions.sort(key=lambda x: x[2])
            best_direction = directions[0][0]
            visit_count = directions[0][2]
            print(f"LOOP ESCAPE: Moving to least visited direction (count: {visit_count}): {best_direction}")
        else:
            # Normal prioritization: first unvisited, then least visited
            directions.sort(key=lambda x: (x[1], x[2]))
            
            # First priority: unvisited directions
            unvisited_directions = [d for d, visited, _ in directions if not visited]
            
            if unvisited_directions:
                best_direction = unvisited_directions[0]
                print(f"Moving to unvisited direction: {best_direction}")
            elif directions:
                # Next priority: least visited direction
                best_direction = directions[0][0]
                visit_count = directions[0][2]
                print(f"Moving to least visited direction (count: {visit_count}): {best_direction}")
            else:
                # This shouldn't happen due to the earlier check, but just in case
                best_direction = "turn_around"
                print("No valid directions, turning around")
        
        # Execute the movement
        result = None
        if best_direction == "right":
            turn_90(right=True)
            result = move_one_tile()
        elif best_direction == "front":
            result = move_one_tile()
        elif best_direction == "left":
            turn_90(right=False)
            result = move_one_tile()
        elif best_direction == "back" or best_direction == "turn_around":
            turn_90()
            turn_90()
            result = move_one_tile()
            turn_90()
            turn_90()
        
        # If successful move or max attempts reached, exit the loop
        if result != "hole":
            return
        
        # If hole detected, mark this direction as invalid and try again
        print(f"Hole detected in {best_direction} direction! Trying a different direction.")
        
        # Mark the coordinates where the hole was detected to avoid going there again
        hole_x, hole_y = get_next_coords(cx, cy, best_direction)
        save_coords(hole_x, hole_y)
        # Add very high visit count to holes to discourage visiting
        tile_visit_counts[(hole_x, hole_y)] = 999
        
        # Turn away from the hole and continue the loop to find a new direction
        turn_90()
        turn_90()
        move_one_tile(tile_size=3)
        old_x, old_y = cx, cy
    
    # If we've tried all directions and still can't move, perform a random turn
    print("Maximum movement attempts reached. Performing random turn.")
    if random.choice([True, False]):
        turn_90()
    else:
        turn_90(right=False)
        
def move4():
    global old_x, old_y, coords, tile_visit_counts
    
    cx, cy = current_coords()
    x = (cx // TILE_WIDTH) + MAP_CONSTANT
    y = (cy // TILE_WIDTH) + MAP_CONSTANT

    # Save and track the current coordinates
    save_coords(cx, cy)
    current_visited = passed()
    if not current_visited:
        print(f"-----------------------------------saving new tile: ({cx}, {cy})")
    
    # Track attempt count to prevent infinite loops
    attempt_count = 0
    max_attempts = 4  # Maximum number of attempts before forcing a move
    
    # Repeat until a successful move or max attempts reached
    while attempt_count < max_attempts:
        attempt_count += 1
        print(f"Movement attempt {attempt_count} of {max_attempts}")
        
        # Create a list of possible movement directions
        directions = []
        
        # Check which directions are free and add them to the priority list
        if not lidar_right:
            directions.append(("right", False, 0))  # (direction, visited, visit_count)
        if not lidar_front:
            directions.append(("front", False, 0))
        if not lidar_left:
            directions.append(("left", False, 0))
        
        # If no directions available (surrounded by walls), force a turn
        if not directions:
            print("All directions blocked by walls, turning around")
            turn_90(right=False)
            move_one_tile()
            return
        
        # For each direction, check if it leads to a visited tile and its visit count
        for i, (direction, _, _) in enumerate(directions):
            next_x, next_y = get_next_coords(cx, cy, direction)
            visited = False
            visit_count = 0
            
            # Check if this direction leads to a previously visited tile
            for visited_x, visited_y in coords:
                if math.sqrt(math.pow(next_x - visited_x, 2) + math.pow(next_y - visited_y, 2)) < TILE_WIDTH / 2:
                    visited = True
                    visit_count = tile_visit_counts.get((visited_x, visited_y), 0)
                    break
            
            # Update the direction data with visit information
            directions[i] = (direction, visited, visit_count)
        
        # First, get unvisited directions as they're always highest priority
        unvisited_directions = [d for d, visited, _ in directions if not visited]
        
        if unvisited_directions:
            # If we have unvisited directions, choose the first one
            best_direction = unvisited_directions[0]
            print(f"Moving to unvisited direction: {best_direction}")
        else:
            # If all directions are visited, choose the one with the lowest visit count
            directions.sort(key=lambda x: x[2])  # Sort by visit count only
            best_direction = directions[0][0]
            visit_count = directions[0][2]
            print(f"All directions visited: Moving to least visited direction (count: {visit_count}): {best_direction}")
        
        # Execute the movement
        result = None
        if best_direction == "right":
            turn_90(right=True)
            result = move_one_tile()
        elif best_direction == "front":
            result = move_one_tile()
        elif best_direction == "left":
            turn_90(right=False)
            result = move_one_tile()
        elif best_direction == "back" or best_direction == "turn_around":
            turn_90()
            turn_90()
            result = move_one_tile()
            turn_90()
            turn_90()
        
        # If successful move or max attempts reached, exit the loop
        if result != "hole":
            return
        
        # If hole detected, mark this direction as invalid and try again
        print(f"Hole detected in {best_direction} direction! Trying a different direction.")
        
        # Mark the coordinates where the hole was detected to avoid going there again
        hole_x, hole_y = get_next_coords(cx, cy, best_direction)
        save_coords(hole_x, hole_y)
        # Add very high visit count to holes to discourage visiting
        tile_visit_counts[(hole_x, hole_y)] = 999
        
        # Turn away from the hole and continue the loop to find a new direction
        turn_90()
        turn_90()
        move_one_tile(tile_size=3)
    
    # If we've tried all directions and still can't move, perform a random turn
    print("Maximum movement attempts reached. Performing random turn.")
    if random.choice([True, False]):
        turn_90()
    else:
        turn_90(right=False)
# def move3():
#     global old_x, old_y, coords
    
#     # FIXME: handle getting stuck
#     cx, cy = current_coords()
#     x = (cx // TILE_WIDTH) + MAP_CONSTANT
#     y = (cy // TILE_WIDTH) + MAP_CONSTANT
    
#     current_visited = passed()
    
#     if not current_visited:
#         save_coords(cx, cy)
#         # TODO: get color from color sensor
#         print(f"-----------------------------------saving new tile: ({cx}, {cy})")
        
#     nx, ny = find_unvisited_tile(x, y)
    
#     move_to_tile(nx, ny)
    
#     pass
            
start = True

def add_to_map(x, y, type):
    global grid, max_x, max_y, min_x, min_y, start
    
    # x = (x // TILE_WIDTH) + MAP_CONSTANT
    # y = (y // TILE_WIDTH) + MAP_CONSTANT
    # print(" _________ X ________________ Y ______________ : ", x, y)
    
    # update x and y
    if not start:
        min_x = min(min_x, x);
        min_y = min(min_y, y);
        max_x = max(max_x, x);
        max_y = max(max_y, y);
    else:
        min_x = x - 1
        min_y = y - 1
        max_x = x
        max_y = y
        start = False
    
    print("_______________ MIN X", min_x)
    print("_______________ MIN Y", min_y)
    print("_______________ MAX X", max_x)
    print("_______________ MAX Y", max_y)
    
    print("__________ STARTING: ", min_x, min_y, " ____________________")
    print("__________ ENDING  : ", max_x, max_y, " ____________________")
    
    print("___________ GRID SIZE: ", max_x - min_x, max_y - min_y, " ____________________")
    
    print(":::::::::::::::::::::::::::: ADDING ", x, y, " TO MAP ::::::::::::::::::::")
    # get walls
    n = True if lidar_front else False
    s = True if lidar_back else False
    e = True if lidar_right else False
    w = True if lidar_left else False
    print("___________________________ WALLS: ", n, s, e, w)
    
    grid[x - 1][y - 1] = Tile(type, n, s, e, w)
        # match type:
        #     case "wall":       grid[x][y] = Tile('1', False, False, False, False)
        #     case "start":      grid[x][y] = Tile('5', False, False, False, False)
        #     case "ground":     grid[x][y] = Tile('0', False, False, False, False)
        #     case "hole":       grid[x][y] = Tile('2', False, False, False, False)
        #     case "swamp":      grid[x][y] = Tile('3', False, False, False, False)
        #     case "checkpoint": grid[x][y] = Tile('4', False, False, False, False)
            
        #     # TODO: area transitions
            
        #     case _: grid[x][y] = Tile('0', False, False, False, False)
    
    # print map
    print("MAPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPP\n\n")
    for i in range(min_y, max_y):
        for j in range(min_x, max_x):
            print(grid[j][i].Type(), end=' ')
        print()
        
    # grid[x][y] = Tile(type, False, False, False, False)
    print("MAPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPP\n\n")
    

map = None
def create_new_map():
    global grid, max_x, max_y, min_x, min_y, map
    # area becomes [4 * (max_x + abs(min_x)), 4 * (max_y + abs(min_y))]
    MAX_X = 4 *(max_x + abs(min_x) + 1) + 1
    MAX_Y = 4 * (max_y + abs(min_y) + 1) + 1
    map = [['0'] * (MAX_X) for _ in range(MAX_Y)]
    
    print("___________ TYPE OF GRID CELL: ", type(grid[0][0]))
    
    # loop over the og grid from (min_x, min_y) to (max_x, max_y)
    # each tile becomes 4x4 with the same value on the corners and 0 in the middle
    
    n, m = 0, 0
    # every tile add 4 to n, m
    
    for i in range(min_x, max_x + 1):
        for j in range(min_y, max_y + 1):
            
            # print("_____________________________ : ", grid[i][j].Type())
            # convert to 4x4
            
            for k in range(m, m + 4):
                map[n][k] = 1 if grid[i][j].N() else 0
            
            for k in range(n + 1, n + 4):
                map[k][m] = 1 if grid[i][j].W() else 0
            
            map[n + 1][m + 1] = grid[i][j].Type()
            map[n + 3][m + 1] = grid[i][j].Type()
            map[n + 1][m + 3] = grid[i][j].Type()
            map[n + 3][m + 3] = grid[i][j].Type()
            
            # rightmost side (handle east wall)
            if i == max_x:
                for k in range(m, m + 5):
                    map[n + 4][k] = 1 if grid[i][j].E() else 0
                
            if j == max_y:
                for k in range(n, n + 5):
                    map[k][m + 4] = 1 if grid[i][j].S() else 0
            
            n += 4
        m += 4
            
    # print the map
    for i in range(min_x, max_x + 1):
        for j in range(min_y, max_y + 1):
            print(grid[i][j].Type(), end=' ')
        print()
    
            
def submit_map():
    submat = np.array(map, dtype=str)
    
    s = submat.shape
    ## Get shape as bytes
    s_bytes = struct.pack('2i',*s)

    ## Flattening the matrix and join with ','
    flatMap = ','.join(submat.flatten())
    ## Encode
    sub_bytes = flatMap.encode('utf-8')

    ## Add togeather, shape + map
    a_bytes = s_bytes + sub_bytes

    ## Send map data
    emitter.send(a_bytes)

    #STEP3 Send map evaluate request
    map_evaluate_request = struct.pack('c', b'M')
    emitter.send(map_evaluate_request)

    #STEP4 Send an Exit message to get Map Bonus
    ## Exit message
    exit_mes = struct.pack('c', b'E')
    emitter.send(exit_mes)

# endregion

# region main

startt = True
while robot.step(timestep) != -1:
    total_time_passed += timestep;

    # whenever the robot is moving, we should get the sensor values to update the global variables
    get_all_sesnor_values()
    if startt:
        add_to_map(*get_grid_coords(), '5')
        startt = False
    else:
        type = '0'
        # add_to_map(*get_grid_coords(), '0')
    
        col = get_color_sensor_color()
        if col not in ['white', 'black', 'swamp', 'grey']:
            type = '1'
            # TODO: map types of cells
            change_area(col)
        
        # TODO: check if checkpoint, swamp, hole
        add_to_map(*get_grid_coords(), type)
        
    detect_victims(img_right, camera_right)
    detect_victims(img_left, camera_left)
    
    print_info()
    
    coords_right = detect_victims(camera_right.getImage(), camera_right)
    coords_left  = detect_victims(camera_left.getImage(),   camera_left)
    
    move2();
    
navigate()

# endregion