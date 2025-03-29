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
    stop = 5000
    while robot.step(timeStep) != -1:
        rightWheel.setVelocity(0)
        leftWheel.setVelocity(0)
        stop -= timeStep
        if stop <= 0:
            break








def is_black_detected():
    BLACK_THRESHOLD = 50
    r, g, b = color_sensor_values[0], color_sensor_values[1], color_sensor_values[2]
    return r < BLACK_THRESHOLD and g < BLACK_THRESHOLD and b < BLACK_THRESHOLD







leftWheel.setVelocity(0.0)
rightWheel.setVelocity(0.0)

while robot.step(timeStep) != -1:

    if is_black_detected():
        print("Black detected! Turning 90 degrees")
        turn_90()



    get_all_sensor_values()
    print("front layer 1: ", lidar_values[0][0])
    print("front layer 2: ", lidar_values[1][0])
    print("front layer 3: ", lidar_values[2][0])
    print("front layer 4: ", lidar_values[3][0])
    print("---------------------------------")
    print("Front: ", lidar_values[2][0])
    print("Right: ", lidar_values[2][127])
    print("Left: ", lidar_values[2][383])
    print("Back: ", lidar_values[2][255])
    print("---------------------------------")
    print("Color Sensor RGB: ", color_sensor_values)
    front = lidar_values[2][0]
# what is the unit of this value?
    if front < 8:
        turn_90()
    else:
        rightWheel.setVelocity(3)
        leftWheel.setVelocity(3)

