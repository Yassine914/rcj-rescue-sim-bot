from controller import Robot, Camera
import time
import math
import cv2 # Include OpenCV library
import numpy as np # For python, include numpy as well
import struct

timeStep = 32            # Set the time step for the simulation
max_velocity = 6.28      # Set a maximum velocity time constant

# Make robot controller instance
robot = Robot()


receiver = robot.getDevice("receiver") # Retrieve the receiver and emitter by device name
emitter = robot.getDevice("emitter")
# Define the wheels 
wheel1 = robot.getDevice("wheel1 motor")   # Create an object to control the left wheel
wheel2 = robot.getDevice("wheel2 motor") # Create an object to control the right wheel
wheel1.setPosition(float("inf"))       
wheel2.setPosition(float("inf"))

# Define sensors
left_distance = robot.getDevice("distance sensor1")
front_distance = robot.getDevice("distance sensor2")
front2_distance = robot.getDevice("distance sensor3") 
right_distance = robot.getDevice("distance sensor4")


left_cam=robot.getDevice("camera1")
right_cam=robot.getDevice("camera2")
color=robot.getDevice("colour_sensor")

gps = robot.getDevice("gps")
imu = robot.getDevice("imu")
# acc = robot.getDevice("a")


receiver.enable(timeStep) # Enable the receiver. Note that the emitter does not need to call enable()
# Enable distance sensors N.B.: This needs to be done for every sensor
left_distance.enable(timeStep)
front_distance.enable(timeStep)
front2_distance.enable(timeStep)
right_distance.enable(timeStep)

color.enable(timeStep)
left_cam.enable(timeStep)
right_cam.enable(timeStep)

gps.enable(timeStep)
imu.enable(timeStep)


# _______Functions_______
def delay(time):
    counter = 0
    while robot.step(timeStep) != -1:
        counter = counter + 1
        if(counter >= 31*time):
            break

def get_yaw():
    current = imu.getRollPitchYaw()[2] * 180 / math.pi
    if(current < 0):
        current = current + 360
    return current


def adjust_angle(angle):
    if(angle > 360):
        angle -= 360
    elif(angle < 0):
        angle +=360
    return angle

def rotate_to_angle(angle):
    error  = 2
    wanted_angle = angle
    upper = adjust_angle(wanted_angle + error)
    lower = adjust_angle(wanted_angle - error)
    current = get_yaw()

    difference  = adjust_angle(wanted_angle - current)
    if(difference > 180):
        direction = "right"
    else:
        direction = "left"

    if((wanted_angle >= (360-error) or wanted_angle<=error ) and (current < upper or current > lower)):        
        wheel1.setVelocity(0)
        wheel2.setVelocity(0)
        return True
    elif(current < upper and current > lower ):
        wheel1.setVelocity(0)
        wheel2.setVelocity(0)
        return True
    else:
        if(direction == "right"):
            # Turn right          
            wheel1.setVelocity(-max_velocity/2)
            wheel2.setVelocity(max_velocity/2)
        else:
            # Turn Left
            wheel1.setVelocity(max_velocity/2)
            wheel2.setVelocity(-max_velocity/2)
        return False

def stop_motors():
    wheel1.setVelocity(0)
    wheel2.setVelocity(0)

def which_direction():
    current = get_yaw()
    
    if(current > 315 or current < 45):
        return  "up"
    elif(45 < current < 135):
        return "left"
    elif(135 < current < 225):
        return "down"
    else:
        return "right"


def rotate_right_90_loop():
    current = get_yaw()
    if(current > 315 or current < 45):
        current = 0
    elif(45 < current < 135):
        current = 90
    elif(135 < current < 225):
        current = 180
    else:
        current = 270

    wanted_angle = adjust_angle(current - 90)
    
    while robot.step(timeStep) != -1:
        arrived = rotate_to_angle(wanted_angle)
        if(arrived):
            wheel1.setVelocity(0)
            wheel2.setVelocity(0)
            delay(1)
            break
    
def rotate_right_90_noLoop(update_wanted, target):
    
    if(update_wanted):
        update_wanted = False
        stop_motors()
        delay(1)
    
    update_wanted =  rotate_to_angle(target)
    return update_wanted

def get_x():
    return gps.getValues()[0]

def get_y():
    return gps.getValues()[2]

def has_arrived(wanted):
    direction = which_direction()
    if(direction == "right" or direction == "left"):
        current_x = get_x()
        if(wanted[0] + 0.03 > current_x > wanted[0] - 0.03):
            return True
        else:
            return False
    else:
        current_y = get_y()
        if(wanted[1] + 0.03 > current_y > wanted[1] - 0.03):
            return True
        else:
            return False

def get_wanted(wanted):
    direction = which_direction()
    if(direction == "right"):
        wanted[0] = wanted[0] + 0.12
    elif(direction == "left"):
        wanted[0] = wanted[0] - 0.12
    elif(direction == "down"):
        wanted[1] = wanted[1] + 0.12
    else:
        wanted[1] = wanted[1] - 0.12
    return wanted

def move_forward_tile(wanted):
    if(has_arrived(wanted)):
        print("I have arrived")
        stop_motors()
        delay(1)
        wanted = get_wanted(wanted)
    return wanted

def check_hole():
    image = color.getImage()
    r = color.imageGetRed(image, 1, 0, 0)
    g = color.imageGetGreen(image, 1, 0, 0)
    b = color.imageGetBlue(image, 1, 0, 0)

    if(r < 48 and g < 48 and b < 48):
        return True
    return False

def check_wall():
    walls = {
        "front": False,
        "right": False,
        "left": False
    }
    if(front_distance.getValue() < 0.32):
        walls["front"] = True
    if(left_distance.getValue() < 0.32):
        walls["left"] = True
    if(right_distance.getValue() < 0.32):
        walls["right"] = True
    return walls

def is_close_to_wall():
    walls = {
        "right": False,
        "left": False
    }
    if(left_distance.getValue() < 0.10):
        walls["left"] = True
    if(right_distance.getValue() < 0.10):
        walls["right"] = True
    return walls

def isSign(x):
    return x

def show_img(image, label="frame"):
    scale = 5
    big_image = cv2.resize(image, None, fx=scale, fy=scale, interpolation=cv2.INTER_LINEAR)
    cv2.imshow(label , big_image)


def get_nonBlue_image(side):
    lower_bound = np.array([90, 110, 30])
    upper_bound = np.array([100, 150, 150])

    if side == "right":
        camera = right_cam
    else:
        camera = left_cam

    image = camera.getImage()
    image = np.frombuffer(image, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4))
    frame = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Create a mask for blueish colors
    blue_mask = cv2.inRange(hsv, lower_bound, upper_bound)

    # Invert the mask to detect non-blue regions
    non_blue_mask = cv2.bitwise_not(blue_mask)

    # Apply the mask to the original frame
    non_blue_regions = cv2.bitwise_and(frame, frame, mask=non_blue_mask)
    # show_img(non_blue_regions, "Non Blue")
    return non_blue_regions

def get_contours(side):
    non_blue_regions = get_nonBlue_image(side)
    # show_img(non_blue_regions, "Non Blue")

    gray = cv2.cvtColor(non_blue_regions, cv2.COLOR_BGR2GRAY)

    canny = cv2.Canny(gray, 125, 175)
    # Find Contours
    contours, hierarchies = cv2.findContours(canny, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    print(f"{len(contours)} contour(s) found!")
    return contours

def is_image_centred(side):
    non_blue_regions = get_nonBlue_image(side)
    grayscale_mask = cv2.cvtColor(non_blue_regions, cv2.COLOR_BGR2GRAY)

    image_height, image_width = grayscale_mask.shape[:2]
    center_x, center_y = image_width // 2, image_height // 2

    # Define a small vertical region around the center
    center_region = grayscale_mask[:, center_x-10:center_x+10]  # Narrow strip around center

    # Check if there's a significant number of non-zero pixels in the center region
    if cv2.countNonZero(center_region) > 90:  # Threshold: adjust as needed
        center_aligned = True
    else:
        center_aligned = False

    # Step 2: Check for Black on Both Sides
    left_region = grayscale_mask[:, :center_x-10]  # Left side
    right_region = grayscale_mask[:, center_x+10:]  # Right side

    left_black_ratio = (left_region.size - cv2.countNonZero(left_region)) / left_region.size
    right_black_ratio = (right_region.size - cv2.countNonZero(right_region)) / right_region.size
    threshold = 0.95  # 95% of pixels should be black

    if left_black_ratio > threshold and right_black_ratio > threshold:
        sides_black = True
        # print("Both sides are black.")
    else:
        sides_black = False
        # print("Black regions not detected on both sides.")

    # Step 3: Decide Movement
    if center_aligned and sides_black:
        # print("Object is centered and isolated. Stop moving.")
        return True
    else:
        # print("Keep moving forward or adjust position.")
        return False

def cam_test():
    non_blue_regions = get_nonBlue_image("right")
    # show_img(non_blue_regions, "Non Blue")

    gray = cv2.cvtColor(non_blue_regions, cv2.COLOR_BGR2GRAY)

    canny = cv2.Canny(gray, 125, 175)
    # show_img(canny, "Canny")

    # Find Contours
    contours, hierarchies = cv2.findContours(canny, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    print(f"{len(contours)} contour(s) found!")

    # Draw Contours
    # blank = np.zeros_like(non_blue_regions)  # Create a blank image with the same shape as the frame
    cv2.drawContours(non_blue_regions, contours, -1, (0, 0, 255), 1)  # Draw contours in red
    show_img(non_blue_regions, "Contours")

    cv2.waitKey(1) # Render imshows on screen

def see_image():
    return is_image_centred("right") and is_close_to_wall()["right"]

# _______Variables_______
update_wanted = True

# _____MAIN______
while robot.step(timeStep) != -1:
    # _______INITIAL_______
    if(update_wanted):
        rotate_right_90_loop()
        wanted = [get_x(), get_y()]
        wanted = get_wanted(wanted)
        update_wanted = False

    # _______LOOP_______
    wanted = move_forward_tile(wanted)
    wheel1.setVelocity(max_velocity/3)
    wheel2.setVelocity(max_velocity/3)

    if(see_image()):
        stop_motors()
        delay(2)
        victimType = bytes('F', "utf-8") # The victim type being sent is the letter 'H' for harmed victim

        position = gps.getValues() # Get the current gps position of the robot
        x = int(position[0] * 100) # Get the xy coordinates, multiplying by 100 to convert from meters to cm 
        y = int(position[2] * 100) # We will use these coordinates as an estimate for the victim's position

        message = struct.pack("i i c", x, y, victimType) # Pack the message.
            
        emitter.send(message) # Send out the message

        break