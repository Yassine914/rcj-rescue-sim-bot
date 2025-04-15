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

def move_one_tile():
    global coords
    

    # in this function, first we get the current x and y of the robot
    # then we round x and y to the nearest multiple of 12 (because the tiles are 12x12)
    # then we calculate the new x or y the robot should move to
    # then we start moving the robot using the while loop

    # round robot compass value to the nearest multiple of 90 (0, 90, 180, -90)
    compass_value_rounded_to_nearest_90 = round(compass_value / 90) * 90

    # get the current x and y of the robot
    x = gps_readings[0]
    y = gps_readings[2]

    # round x and y to nearest multiple of 12
    x = round(x / TILE_WIDTH) * TILE_WIDTH
    y = round(y / TILE_WIDTH) * TILE_WIDTH

    # save_coords(x, y);
    
    # if the robot is facing horizontally (90 or -90) we should move in the x direction, so x should change and y should stay the same
    # if the robot is facing vertically (0 or 180) we should move in the y direction, so y should change and x should stay the same

    if compass_value_rounded_to_nearest_90 in (90, -90):
        if compass_value_rounded_to_nearest_90 == 90:
            # if the robot is facing 90, then we should move to the left, so we subtract 12 from x
            x_new = x - TILE_WIDTH
            y_new = y
        else:
            # if the robot is facing -90, then we should move to the right, so we add 12 to x
            x_new = x + TILE_WIDTH
            y_new = y

    else:
        # if the robot is facing 0, then we should move up, so we subtract 12 from y
        if compass_value_rounded_to_nearest_90 == 0:
            x_new = x
            y_new = y - TILE_WIDTH
        else:
            # if the robot is facing 180, then we should move down, so we add 12 to y
            x_new = x
            y_new = y + TILE_WIDTH
            
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
    # FIXME:
    if sign_type == 'N':
        sign_type = detect_letters2(img)
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
    # determine wether the sign is a square or rhombus
    
    sign_colored = cv.cvtColor(sign_colored, cv.COLOR_BGR2GRAY)
    _, thresh = cv.threshold(sign_colored, 80, 255, cv.THRESH_BINARY_INV)
    contours, _ = cv.findContours(thresh, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    
    # iterate over the contours and check if any of them is a square or rhombus
    for contour in contours:
        # get the bounding rectangle of the contour
        x, y, w, h = cv.boundingRect(contour)
        # check if the contour is a square or rhombus
        if abs(w - h) < 10 and 0.5 < w / h < 2:
            return 'S'
        elif abs(w - h) < 10 and 2 < w / h < 3:
            return 'R'
        
    return 'N'  # return N if no square or rhombus is found

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
    # cv2.imshow("orange mask" , orange_mask)

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
    sign_type = 'N'
    h, w = sign.shape
    # get bottom half of the image
    bottom = sign[int(h * 0.5):, int(w * 0.3):int(w * 0.85)]
    white_pixel = cv.countNonZero(bottom)
    black_pixel = bottom.size - white_pixel

    if black_pixel <= white_pixel:
        sign_type = 'P'
    else:
        sign_type = 'C'
        
    return sign_type

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
    cv.imshow("First Section", first_section)
    cv.imshow("Middle Section", middle_section)
    cv.imshow("Third Section", third_section)
    cv.waitKey(1)
    
    num_contours_first = len(contours_first)
    num_contours_middle = len(contours_middle)
    num_contours_third = len(contours_third)
    
    print("First Section Contours: ", num_contours_first)
    print("Middle Section Contours: ", num_contours_middle)
    print("Third Section Contours: ", num_contours_third)
    
    # H has vertical lines on both sides (1 contour each) and a horizontal line in the middle
    if num_contours_first == 1 and num_contours_middle == 1 and num_contours_third == 1:
        return 'H'
    
    # S has a distinctive pattern with typically 2 contours in each section
    elif num_contours_first >= 2 and num_contours_middle >= 1 and num_contours_third >= 2:
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
    
    height, width = thresh.shape
    section_width = width // 3
    
    first_section = thresh[:, :section_width]
    middle_section = thresh[:, section_width:2 * section_width]
    third_section = thresh[:, 2 * section_width:]
    
    contours_first, _ = cv.findContours(first_section, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    contours_middle, _ = cv.findContours(middle_section, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    contours_third, _ = cv.findContours(third_section, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    
    # output image for debugging
    cv.imshow("First Section", first_section)
    cv.imshow("Third Section", third_section)
    cv.waitKey(1)
    
    num_contours_first = len(contours_first)
    num_contours_middle = len(contours_middle)
    num_contours_third = len(contours_third)
    
    print("First Section Contours: ", num_contours_first)
    print("Middle Section Contours: ", num_contours_middle)
    print("Third Section Contours: ", num_contours_third)
    
    if num_contours_first == 1 and num_contours_middle == 1 and num_contours_third == 2:
        # print("found H")
        return 'U'
    elif num_contours_first == 1 and num_contours_middle == 1 and num_contours_third == 1:
        # print("found S")
        return 'H'
    elif num_contours_first == 3 and num_contours_middle == 4 and num_contours_third == 2:
        return 'S'
        # print("found U")
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

# endregion

# region main

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
    # return the current coordinates of the robot
    x = gps_readings[0]
    y = gps_readings[2]
    return round(x), round(y)

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
    """
    Calculate the next coordinates based on current position and direction.
    """
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


old_x, old_y = None, None
# Zeyad was here
def move2():
    global old_x, old_y, coords
    
    cx, cy = current_coords()
    # print(f"___________ CURRENT: ({cx}, {cy})")
    
    # Check if current tile has been visited
    current_visited = passed()
    
    if old_x is not None and old_y is not None:
        # Check if the robot has moved to a new tile
        if (cx, cy) != (old_x, old_y):
            # Save the previous coordinates as visited
            save_coords(old_x, old_y)
            print(f"Saving new tile: ({old_x}, {old_y})")
        else:
            # If the robot hasn't moved, return early
            turn_90()
            move_one_tile()
            return
    
    # Save the current coordinates as visited if not already visited
    if not current_visited:
        save_coords(cx, cy)
        print(f"Saving new tile: ({cx}, {cy})")
    
    # Track attempt count to prevent infinite loops
    attempt_count = 0
    max_attempts = 4  # Maximum number of attempts before forcing a move
    
    # Repeat until a successful move or max attempts reached
    while attempt_count < max_attempts:
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
            turn_90()
            turn_90()
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
            best_direction = directions[0][0];
            print(f"All directions visited, using priority direction: {best_direction}")
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
            
        old_x, old_y = cx, cy
        
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

while robot.step(timestep) != -1:

    # whenever the robot is moving, we should get the sensor values to update the global variables
    get_all_sesnor_values()
    detect_victims(img_right, camera_right)
    detect_victims(img_left, camera_left)
    
    print_info()
    
    coords_right = detect_victims(camera_right.getImage(), camera_right)
    coords_left  = detect_victims(camera_left.getImage(),   camera_left)
    
    move2();
    
navigate()

# endregion