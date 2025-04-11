from controller import Robot
import math
import numpy as np
import cv2 as cv
import struct

timeStep = 32
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
gps.enable(timeStep)
compass.enable(timeStep)
camera_right.enable(timeStep)
camera_left.enable(timeStep)
color_sensor.enable(timeStep)
lidar.enable(timeStep)


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


def turn_90(right = True):

    # in this function, first we set the wheel velocities and the angle the robot should move to
    # then we start moving the robot using the while loop

    # round current robot angle to the nearest multiple of 90 (0, 90, 180, -90)
    compass_value_rounded_to_nearest_90 = round(compass_value / 90) * 90

    # then add or subtract 90 to get the next angle the robot should move to
    if right:
        # subtract 90 if the robot should turn right
        next_angle = compass_value_rounded_to_nearest_90 - 90
    else :
        # add 90 if the robot should turn left
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
    while robot.step(timeStep) != -1:
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
            print(lidar_values[2][0])

    for i in range(112, 142):
        if lidar_values[2][i] < 7:
            lidar_right = True
            print(lidar_values[2][127])


    for i in range(246 - 5, 266 + 5):
        if lidar_values[2][i] < 7:
            lidar_back = True
            print(lidar_values[2][256])


    for i in range(373 - 5, 393 + 5):
        if lidar_values[2][i] < 7:
            lidar_left = True
            print(lidar_values[2][3])
            print(lidar_values[2][370])




def stop():
    # we call robot.step but with wheel velocities set to 0
    # the simulation will keep running but the robot will stop

    #TODO: pass duration as a parameter

    stop = 5000
    while robot.step(timeStep) != -1:
        # keep looping until 5000ms pass then break the loop
        wheel1.setVelocity(0)
        wheel2.setVelocity(0)
        stop -= timeStep
        if stop <= 0:
            break

start = robot.getTime()

def move_one_tile():

    # in this function, first we get the current x and y of the robot
    # then we round x and y to the nearest multiple of 12 (because the tiles are 12x12)
    # then we calculate the new x or y the robot should move to
    # then we start moving the robot using the while loop

    # round robot compass value to the nearest multiple of 90 (0, 90, 180, -90)
    compass_value_rounded_to_nearest_90 = round(compass_value / 90) * 90

    # get the current x and y of the robot
    x = gps_readings[0]
    y = gps_readings[2]

    # round x and y to nearest multiple of 12 (because the tiles are 12x12)
    x = round(x / 12) * 12
    y = round(y / 12) * 12

    # if the robot is facing horizontally (90 or -90) we should move in the x direction, so x should change and y should stay the same
    # if the robot is facing vertically (0 or 180) we should move in the y direction, so y should change and x should stay the same

    if compass_value_rounded_to_nearest_90 in (90, -90):
        if compass_value_rounded_to_nearest_90 == 90:
            # if the robot is facing 90, then we should move to the left, so we subtract 12 from x
            x_new = x - 12
            y_new = y
        else:
            # if the robot is facing -90, then we should move to the right, so we add 12 to x
            x_new = x + 12
            y_new = y

    else:
        # if the robot is facing 0, then we should move up, so we subtract 12 from y
        if compass_value_rounded_to_nearest_90 == 0:
            x_new = x
            y_new = y - 12
        else:
            # if the robot is facing 180, then we should move down, so we add 12 to y
            x_new = x
            y_new = y + 12

    while robot.step(timeStep) != -1:

        # whenever the robot is moving, we should get the sensor values to update the global variables
        get_all_sesnor_values()
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


        # the robot stops moving in two cases:
        # 1. the robot sees an object in front of him
        # 2. the robot reaches the new x coordinate (if he is moving horizontally) or the new y coordinate (if he is moving vertically)

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


def should_scan():
    # check if the robot has scanned the sign before
    # loop the scanned signs check for a near colunms
    for point in scanned_signs:
        dist = math.sqrt(
            math.pow(gps_readings[0] - point[0], 2) +
            math.pow(gps_readings[2] - point[1], 2)
        )
        if dist < 10:
            return False
    return True


def detect(img):
    lower_red = [170, 150, 150]
    upper_red = [180, 255, 255]
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

    red_mask = cv.inRange(hsv, np.array(lower_red), np.array(upper_red))

    red_count = cv.countNonZero(red_mask)

    if red_count > 25:
        stop()

        victimType = bytes('F', "utf-8")  # The victim type being sent is the letter 'H' for harmed victim

        position = gps.getValues()  # Get the current gps position of the robot
        x = int(position[0] * 100)  # Get the xy coordinates, multiplying by 100 to convert from meters to cm
        y = int(position[2] * 100)  # We will use these coordinates as an estimate for the victim's position

        scanned_signs.append((x, y))

        message = struct.pack("i i c", x, y, victimType)  # Pack the message.

        emitter.send(message)  # Send out the message

        stop()

        print("------------------------------Victim detected at: ", x, y)
        return

    lower_yellow = [20, 100, 100]
    upper_yellow = [30, 255, 255]


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

        # Check if the contour is large enough to be a victim
        if cv.contourArea(contour) > 500 and w / h < 2 and 5 < x < 25 and y < 25:
            # Crop the image
            cropped_image = img[y:y + h, x:x + w]
            if should_scan(): # Check if the robot has scanned the sign before
                detect(cropped_image)

def navigate():
    global counter

while robot.step(timeStep) != -1:

    # whenever the robot is moving, we should get the sensor values to update the global variables
    get_all_sesnor_values()
    detect_victims(img_right, camera_right)
    detect_victims(img_left, camera_left)
    print("---------------------------------")
    print(f"Compass value:  {compass_value}")
    print(f"gps readings:  {gps_readings}")
    print(f"Color sensor values:  {color_sensor_values}")
    print("---------------------------------")

    coords_right = detect_victims(camera_right.getImage(), camera_right)
    coords_left = detect_victims(camera_left.getImage(), camera_left)

    if not lidar_right:
        turn_90()
        r = move_one_tile()
        if r == "hole":
            turn_90()
    elif lidar_front:
        turn_90(right=False)
    else:
        r = move_one_tile()
        if r == "hole":
            turn_90()

navigate()