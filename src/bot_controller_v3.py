from collections import deque
from controller import Robot
import math
import numpy as np
import cv2 as cv
import struct
import random

# region globals

RED_THRESHOLD = 30  # The threshold for the red color in the image
TILE_WIDTH = 6 # the width of the tile in cm
BLACK_THRESHOLD = 60 # The threshold for the black color in the image (holes)

timestep = 32
max_velocity = 6.28
velocity = max_velocity
robot = Robot()

# Define the wheels 
right_wheel = robot.getDevice("wheel1 motor")   # Create an object to control the left wheel
left_wheel = robot.getDevice("wheel2 motor") # Create an object to control the right wheel

# Set the wheels to have infinite rotation 
right_wheel.setPosition(float("inf"))       
left_wheel.setPosition(float("inf"))

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

# endregion
# region detect_signs
def scan_for_signs():
    x = detect_victims(camera_right.getImage(), camera_right)
    y = detect_victims(camera_left.getImage(),   camera_left)
    return x, y

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
# region movement
def print_info():
    return
    print("---------------------------------")
    print(f"Compass value:  {compass_value}")
    print(f"gps readings:  {gps_readings}")
    print(f"Color sensor values:  {color_sensor_values}")
    print("---------------------------------")
    
    
def turn_90(right = True):
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
        right_wheel.setVelocity(s1)
        left_wheel.setVelocity(s2)

        # check if robot is close to the next angle he should move to (if difference is smaller than 7)
        if abs(compass_value - next_angle) < 7:
            # robot is close to next angle then we should break the loop
            break

def stop(duration = 0):
    # we call robot.step but with wheel velocities set to 0
    # the simulation will keep running but the robot will stop
    
    if duration == 0:
        right_wheel.setVelocity(0)
        left_wheel.setVelocity(0)
        return

    stop = duration
    while robot.step(timestep) != -1:
        # keep looping until 5000ms pass then break the loop
        right_wheel.setVelocity(0)
        left_wheel.setVelocity(0)
        stop -= timestep
        if stop <= 0:
            break

start = robot.getTime()

def move_one_tile(tile_size=TILE_WIDTH):
    global coords
    
    scan_for_signs()
    
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
        
        if stuck():
            return "stuck"
        
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
        right_wheel.setVelocity(s1)
        left_wheel.setVelocity(s2)


        # the robot stops moving in 3 cases:
        # 1. the robot sees an object in front of him
        # 2. the robot reaches the new x coordinate (if he is moving horizontally) or the new y coordinate (if he is moving vertically)
        # 3. there is a hole infront of the robot (we check the colour sensor to see if it is black)
        
        # FIXME: make sure the robot goes back to normal nav mode


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
            handle_hole()
            return "hole"

def move_one_tile_backwards(tile_size=TILE_WIDTH):
    global coords
    
    scan_for_signs()
    
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
            x_new = x + tile_size
            y_new = y
        else:
            # if the robot is facing -90, then we should move to the right, so we add 12 to x
            x_new = x - tile_size 
            y_new = y

    else:
        # if the robot is facing 0, then we should move up, so we subtract 12 from y
        if compass_value_rounded_to_nearest_90 == 0:
            x_new = x
            y_new = y + tile_size
        else:
            # if the robot is facing 180, then we should move down, so we add 12 to y
            x_new = x
            y_new = y - tile_size
            
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
            s1 = -6.28
            s2 = -4
        else:
            # if the robot is inclined to the left, we should make the robot move slightly to the right
            s1 = -4
            s2 = -6.28

        # start moving the robot with the calculated wheel velocities
        right_wheel.setVelocity(s1)
        left_wheel.setVelocity(s2)
       
        if stuck():
            break

        # if the robot sees an object in front of him, we break the loop, then the function will end
        if lidar_front:
            break
        
        if compass_value_rounded_to_nearest_90 in (90, -90):
            if x_new - 1 < gps_readings[0] < x_new + 1:
                break
        else :
            if y_new - 1 < gps_readings[2] < y_new + 1:
                break
            
        if color_sensor_values[0] < BLACK_THRESHOLD and color_sensor_values[1] < BLACK_THRESHOLD and color_sensor_values[2] < BLACK_THRESHOLD:
            handle_hole()
            return "hole"

coords = set()
def current_coords():
    get_gps_readings()
    # return the current coordinates of the robot
    # NOTE: each tile is now 4 * (6cm * 6cm)
    x = round(gps_readings[0] / TILE_WIDTH) * TILE_WIDTH
    y = round(gps_readings[2] / TILE_WIDTH) * TILE_WIDTH
    return x, y

def save_coords(x, y):
    global coords
    coords.add((x, y))
    
def has_already_passed_current_tile():
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
    
MAP_CONSTANT = 100 # Add 50 to idx to make them +ve

max_x = 1 + MAP_CONSTANT
max_y = 1 + MAP_CONSTANT
min_x = 0 + MAP_CONSTANT
min_y = 0 + MAP_CONSTANT

# each tile is now 6cm * 6cm
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


MAX_GRID_SIZE = 500 
grid = [[Tile('-1', False, False, False, False) for _ in range(MAX_GRID_SIZE)] for _ in range(MAX_GRID_SIZE)]

# TODO: fix this function
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

def move_forward():
    right_wheel.setVelocity(velocity)
    left_wheel.setVelocity(velocity)

def move_backward():
    right_wheel.setVelocity(-velocity)
    left_wheel.setVelocity(-velocity)

def move_to_previous_tile():
    print("started moving to previous tile")
    move_one_tile_backwards(tile_size=3)

    return

def stuck():
    get_lidar_values()
    
    f = True
    for i in range(0, 512):
        f &= (lidar_values[2][i]>4.1)

    if f == False:
        
        # stop()
        # move_to_previous_tile()
        move_one_tile_backwards(tile_size=3)
        add_to_map(*get_next_grid_coords(), '1')
        
        return True
    
    return False

def handle_hole():
    x, y = get_next_grid_coords()
    add_to_map(x, y, '2')
    add_to_map(x - 1, y, '2')
    nx, ny = get_next_grid_coords(x, y)
    add_to_map(nx, ny, '2')
    nx, ny = get_next_grid_coords(x - 1, y)
    add_to_map(nx, ny, '2')
    move_one_tile_backwards(tile_size=3)
    pass


def get_current_tile_type():
    # TODO: get the tile type from the color sensor
    pass

def nav_to_nearest_unvisited_tile():
    print("Robot may be stuck in a loop - searching for nearest unvisited tile...")
    
    # Get current grid position
    cx, cy = get_grid_coords()
    print(f"current grid position: ({cx}, {cy})")
    
    # queue for bfs and visited set to avoid cycles
    queue = deque([(cx, cy, [])])
    visited = set([(cx, cy)])
    
    cells_checked = 0
    max_cells = 1000 # limit to avoid infinite loops TODO: adjust this value
    
    # BFS to find closest unvisited tile
    while queue and cells_checked < max_cells:
        x, y, path = queue.popleft()
        cells_checked += 1
        
        # Check if current cell is unvisited
        if grid[x][y].type == '-1':
            print(f"Found unvisited tile at ({x}, {y}) after checking {cells_checked} cells")
            print(f"Path directions: {path}")
            
            # Execute the path
            return execute_path_to_target(path)
        
        # Check neighboring cells with consistency checks
        # North (both cells agree there's no wall)
        if not grid[x][y].N() and y > min_y and not grid[x][y-1].S() and grid[x][y-1].type not in ['2', '1']:
            if (x, y-1) not in visited:
                visited.add((x, y-1))
                queue.append((x, y-1, path + ['north']))
            
        # South (both cells agree there's no wall)
        if not grid[x][y].S() and y < max_y and not grid[x][y+1].N() and grid[x][y+1].type not in ['2', '1']:
            if (x, y+1) not in visited:
                visited.add((x, y+1))
                queue.append((x, y+1, path + ['south']))
            
        # East (both cells agree there's no wall)
        if not grid[x][y].E() and x < max_x and not grid[x+1][y].W() and grid[x+1][y].type not in ['2', '1']:
            if (x+1, y) not in visited:
                visited.add((x+1, y))
                queue.append((x+1, y, path + ['east']))
            
        # West (both cells agree there's no wall)
        if not grid[x][y].W() and x > min_x and not grid[x-1][y].E() and grid[x-1][y].type not in ['2', '1']:
            if (x-1, y) not in visited:
                visited.add((x-1, y))
                queue.append((x-1, y, path + ['west']))
    
    print(f"No unvisited tiles found after checking {cells_checked} cells")
    
    # If we can't find any unvisited tile, try looking for the least visited tile
    return nav_to_least_visited_tile()

def execute_path_to_target(path):
    if not path:
        print("no path to target")
        return False
    
    print(f"executing path: {path}")
    
    current_dir = get_compass_rounded()
    compass_dir_map = {
        0: 'north',
        90: 'west',
        -90: 'east',
        180: 'south',
        -180: 'south'
    }
    
    current_orientation = compass_dir_map[current_dir]
    print(f"starting orientation: {current_orientation}")
    
    for target_dir in path:
        # Calculate turns needed to face target direction
        turns_needed = calculate_turns(current_orientation, target_dir)
        
        # Execute the turns
        for turn_right in turns_needed:
            turn_90(right=turn_right)
            current_orientation = get_direction_after_turn(current_orientation, turn_right)
        
        print(f"now facing {current_orientation}, moving forward")
        
        result = move_one_tile()
        
        # Check color and update map TODO: handle color sensor values
        col = get_color_sensor_color()
        if col not in ['white', 'black', 'swamp', 'grey']:
            type = '1'
            change_area(col)
        else:
            type = '0'
            
        cx, cy = get_grid_coords()
        add_to_map(cx, cy, type)
        
        # Handle holes
        if result == "hole":
            print("encountered a hole! Recalculating path...")
            move_to_previous_tile()
            add_to_map(cx, cy, '2')  # '2' represents a hole
            return nav_to_nearest_unvisited_tile()  # Recalculate path
    
    print("successfully reached target tile")
    return True

def calculate_turns(current, target):
    direction_map = {'north': 0, 'east': 1, 'south': 2, 'west': 3}
    
    current_val = direction_map[current]
    target_val = direction_map[target]
    
    # Calculate difference (0 to 3 steps)
    diff = (target_val - current_val) % 4
    
    # Choose the shorter path (clockwise or counterclockwise)
    if diff <= 2:
        # Clockwise turns (right turns)
        return [True] * diff
    else:
        # Counterclockwise turns (left turns)
        return [False] * (4 - diff)

def get_direction_after_turn(current, turn_right):
    direction_cycle = ['north', 'east', 'south', 'west']
    current_index = direction_cycle.index(current)
    
    if turn_right:
        return direction_cycle[(current_index + 1) % 4]
    else:
        return direction_cycle[(current_index - 1) % 4]

def nav_to_least_visited_tile():
    print("no unvisited tiles found - navigating to least visited tile...")
    
    # Get current grid position
    cx, cy = get_grid_coords()
    print(f"current grid position: ({cx}, {cy})")
    
    # Priority queue for BFS - (visit_count, x, y, path)
    from heapq import heappush, heappop
    
    # Initialize with current position
    priority_queue = [(0, cx, cy, [])]
    visited = set([(cx, cy)])
    
    # Track exploration stats
    cells_checked = 0
    max_cells = 2000  # Safety limit
    
    while priority_queue and cells_checked < max_cells:
        # Get tile with lowest visit count
        _, x, y, path = heappop(priority_queue)
        cells_checked += 1
        
        # Get coordinates in real space
        real_x, real_y = (x - MAP_CONSTANT) * TILE_WIDTH, (y - MAP_CONSTANT) * TILE_WIDTH
        
        # Get visit count from our tracking dictionary
        visit_count = tile_visit_counts.get((real_x, real_y), 0)
        
        # Check if this is a valid destination (less visited than current position)
        current_real_x, current_real_y = (cx - MAP_CONSTANT) * TILE_WIDTH, (cy - MAP_CONSTANT) * TILE_WIDTH
        current_visit_count = tile_visit_counts.get((current_real_x, current_real_y), 0)
        
        if visit_count < current_visit_count and grid[x][y].type != '2':  # Not a hole
            print(f"Found less-visited tile at ({x}, {y}) - visit count: {visit_count} vs current: {current_visit_count}")
            print(f"Path directions: {path}")
            return execute_path_to_target(path)
        
        # Check neighboring cells
        # North (both cells agree there's no wall)
        if not grid[x][y].N() and y > min_y and not grid[x][y-1].S():
            if (x, y-1) not in visited:
                visited.add((x, y-1))
                real_nx, real_ny = (x - MAP_CONSTANT) * TILE_WIDTH, ((y-1) - MAP_CONSTANT) * TILE_WIDTH
                next_visit_count = tile_visit_counts.get((real_nx, real_ny), 0)
                heappush(priority_queue, (next_visit_count, x, y-1, path + ['north']))
        
        # South (both cells agree there's no wall)
        if not grid[x][y].S() and y < max_y and not grid[x][y+1].N():
            if (x, y+1) not in visited:
                visited.add((x, y+1))
                real_nx, real_ny = (x - MAP_CONSTANT) * TILE_WIDTH, ((y+1) - MAP_CONSTANT) * TILE_WIDTH
                next_visit_count = tile_visit_counts.get((real_nx, real_ny), 0)
                heappush(priority_queue, (next_visit_count, x, y+1, path + ['south']))
        
        # East (both cells agree there's no wall)
        if not grid[x][y].E() and x < max_x and not grid[x+1][y].W():
            if (x+1, y) not in visited:
                visited.add((x+1, y))
                real_nx, real_ny = ((x+1) - MAP_CONSTANT) * TILE_WIDTH, (y - MAP_CONSTANT) * TILE_WIDTH
                next_visit_count = tile_visit_counts.get((real_nx, real_ny), 0)
                heappush(priority_queue, (next_visit_count, x+1, y, path + ['east']))
        
        # West (both cells agree there's no wall)
        if not grid[x][y].W() and x > min_x and not grid[x-1][y].E():
            if (x-1, y) not in visited:
                visited.add((x-1, y))
                real_nx, real_ny = ((x-1) - MAP_CONSTANT) * TILE_WIDTH, (y - MAP_CONSTANT) * TILE_WIDTH
                next_visit_count = tile_visit_counts.get((real_nx, real_ny), 0)
                heappush(priority_queue, (next_visit_count, x-1, y, path + ['west']))
    
    print("navigation failed - couldn't find a better tile to visit")
    
    turn_90(right=random.choice([True, False]))
    move_one_tile()
    return False

def move_old():
    global coords
    
    # FIXME: handle getting stuck
    
    cx, cy = current_coords()
    current_visited = has_already_passed_current_tile()
    
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
            nav_to_nearest_unvisited_tile()
            # print("All directions blocked by walls, turning around")
            # turn_90(right=False)
            # turn_90()
            # move_one_tile()
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
            print("ALL DIRECTIONS VISITED. MOVING TO NEAREST UNVISITED TILE")
            nav_to_nearest_unvisited_tile()           
            return
        else:
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
            # turn_90()
            # turn_90()
        
        
        if has_explored_most_of_the_map():
            print("- - - - - - - - - - - we have explored most of the map - - - - - -  - - - - ")
            create_new_map()
            submit_map()
        
        # If successful move or max attempts reached, exit the loop
        if result != "hole" and result != "stuck":
            return
        
        stop(300)
        
        if stuck():
            print("stuck")
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
        
    
    # If we've tried all directions and still can't move, perform a random turn
    print("Maximum movement attempts reached. Performing random turn.")
    if random.choice([True, False]):
        turn_90()
    else:
        turn_90(right=False)


# NEW MOVE FUNCTION
def move_final():
    global min_x, min_y, max_x, max_y, grid
    
    cx, cy = get_grid_coords()
    print("entered movement function")
    print(f"current position: ({cx}, {cy})")
    
    # use the map to determine visited and unvisited cells
    
    if grid[cx][cy].type == '-1':
        # FIXME: add type to the cell
        add_to_map(cx, cy, '0')
        
    # before moving using the stack, move normally untill we are surrounded by visited tiles
        
    # push neighboring -1s to a stack in order to reverse explore the map later
    stack = []
    # if opposite walls match then go there
    # NOTE: if we go out of grid bounds add another row/col
    n, s, e, w = grid[cx][cy].N(), grid[cx][cy].S(), grid[cx][cy].E(), grid[cx][cy].W()
    
    # NOTE: ADDED POSSIBLE DIRECTIONS TO THE STACK (should be in a loop)
    if not n: # no north wall
        if grid[cx][cy - 1].type == '-1' and not grid[cx][cy - 1].S():
            stack.append((cx, cy - 1))
            
            # FIXME: NOTE: increase map size if needed
            if cy - 1 < min_y:
                min_y = cy - 1
                for i in range(min_x, max_x + 1):
                    grid[i][min_y] = Tile('-1', False, False, False, False)
            
    if not s: # no south wall
        if grid[cx][cy + 1].type == '-1' and not grid[cx][cy + 1].N():
            stack.append((cx, cy + 1))
            
            if cy + 1 > max_y:
                max_y = cy + 1
                for i in range(min_x, max_x + 1):
                    grid[i][max_y] = Tile('-1', False, False, False, False)
    
    if not e: # no east wall
        if grid[cx + 1][cy].type == '-1' and not grid[cx + 1][cy].W():
            stack.append((cx + 1, cy))
            
            if cx + 1 > max_x:
                max_x = cx + 1
                for i in range(min_y, max_y + 1):
                    grid[max_x][i] = Tile('-1', False, False, False, False)
                    
    if not w: # no west wall
        if grid[cx - 1][cy].type == '-1' and not grid[cx - 1][cy].E():
            stack.append((cx - 1, cy))
            
            if cx - 1 < min_x:
                min_x = cx - 1
                for i in range(min_y, max_y + 1):
                    grid[min_x][i] = Tile('-1', False, False, False, False)
    
    return

def get_grid_coords(x=None, y=None) -> tuple:
    if x is not None and y is not None:
        return x, y
    
    cx, cy = current_coords()
    x = round(cx / TILE_WIDTH) + MAP_CONSTANT
    y = round(cy / TILE_WIDTH) + MAP_CONSTANT
    return x, y

def get_next_grid_coords(x=None, y=None) -> tuple:
    cx, cy = get_grid_coords(x, y)
    compass_value_rounded = get_compass_rounded()
    
    if compass_value_rounded == 0:
        return cx, cy - 1
    elif compass_value_rounded == 90:
        return cx - 1, cy
    elif compass_value_rounded == -90:
        return cx + 1, cy
    else:
        return cx, cy + 1

tile_visit_counts = {} # save visit count per cell
def save_coords(x, y):
    global coords, tile_visit_counts
    coords.add((x, y))
    
    # Track visit count
    if (x, y) in tile_visit_counts:
        tile_visit_counts[(x, y)] += 1
    else:
        tile_visit_counts[(x, y)] = 1
    
    print(f"Tile ({x}, {y}) has been visited {tile_visit_counts[(x, y)]} times")
           
start = True


def get_compass_rounded() -> int:
    compass_rounded = round(compass_value / 90) * 90
    
    if compass_rounded > 180:
        compass_rounded -= 360
    elif compass_rounded < -180:
        compass_rounded += 360
    
    return compass_rounded

def get_global_walls() -> tuple:
    global lidar_front, lidar_back, lidar_left, lidar_right
    
    compass_rounded = get_compass_rounded()
    
    print("current direction: ", "north" if compass_rounded == 0 else "south" if abs(compass_rounded) == 180 else "west" if compass_rounded == 90 else "east")
    
    f, b, l, r = (lidar_front, lidar_back, lidar_left, lidar_right)
    if compass_rounded == 0: # facing north
        f = lidar_front
        b = lidar_back
        l = lidar_left
        r = lidar_right
    elif abs(compass_rounded) == 180: # facing south
        f = lidar_back
        b = lidar_front
        l = lidar_right
        r = lidar_left
    elif compass_rounded == 90: # facing west
        f = lidar_right
        b = lidar_left
        l = lidar_front
        r = lidar_back
    else:                      # facing east
        f = lidar_left
        b = lidar_right
        l = lidar_back
        r = lidar_front
   
    # ret   n  s  e  w
    return (f, b, r, l)

def add_to_map(x, y, type):
    global grid, max_x, max_y, min_x, min_y, start
    
    # update x and y
    if not start:
        min_x = min(min_x, x);
        min_y = min(min_y, y);
        max_x = max(max_x, x);
        max_y = max(max_y, y);
    else:
        min_x = x
        min_y = y
        max_x = x + 2
        max_y = y + 2
        start = False
    
    print(f"grid starts from ({min_x}, {min_y}) and ends at ({max_x}, {max_y})")
    print(f"grid size: {max_x - min_x} x {max_y - min_y}")
    
    print(f"adding ({x}, {y}) to grid")
    
    # get walls
    n, s, e, w = get_global_walls()
    print(f"\twalls: north: {n}, south: {s}, east: {e}, west: {w}")
    
    # TODO: for normal tiles: make sure we check for the entire range of the ladar
    if grid[x][y].type == '-1':
        grid[x][y] = Tile(type, n, s, e, w)
        
    print(f"grid[{x}][{y}] = {grid[x][y].type}, {grid[x][y].N()}, {grid[x][y].S()}, {grid[x][y].E()}, {grid[x][y].W()}")
    
    # print map
    print("---map start---\n\n")
    for i in range(min_y - 1, max_y + 2):
        for j in range(min_x - 1, max_x + 2):
            print(grid[j][i].Type(), end=' ')
        print()
        
    # grid[x][y] = Tile(type, False, False, False, False)
    print("---map end---\n\n")

map = None
def create_new_map():
    global grid, max_x, max_y, min_x, min_y, map
    # area becomes [4 * (max_x + abs(min_x)), 4 * (max_y + abs(min_y))]
    MAX_X = 4 *(max_x + abs(min_x) + 1) + 1
    MAX_Y = 4 * (max_y + abs(min_y) + 1) + 1
    map = [['0'] * (MAX_X) for _ in range(MAX_Y)]
    
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
    for i in range(min_x, max_x + 2):
        for j in range(min_y, max_y + 2):
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
        
    # detect_victims(img_right, camera_right)
    # detect_victims(img_left, camera_left)
    
    print_info()
    
    scan_for_signs()
    
    # move_final();
    move_old();
    pass

# endregion