import math
from controller import Robot, GPS, Motor, Camera, Emitter
import numpy as np
import cv2 as cv
import struct

DEBUG = True

COLOR_CHANNELS = 4

# State constants
STATES = {
    "INIT": 0,
    "EXPLORING": 1,
    "ROTATING": 2,
    "MOVING": 3,
    "SIGN_DETECTED": 4,
    "OBSTACLE_AVOIDANCE": 5,
    "ESCAPE": 6
}

MAX_VELOCITY = 6.28

class RescueBot:
    def __init__(self):
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
       
        # Initialize devices (stubs)
        self.init_devices()
        
        # State management
        self.current_state = STATES["INIT"]
        self.target_direction = None
        self.last_sign_time = 0

    def init_devices(self):
        self.rwheel = self.robot.getDevice("right_wheel motor")
        self.rwheel.setPosition(float("inf"))

        self.lwheel = self.robot.getDevice("left_wheel motor")
        self.lwheel.setPosition(float("inf"))

        self.speeds = [MAX_VELOCITY, MAX_VELOCITY]

        self.rcam = self.robot.getDevice("right_cam")
        self.rcam.enable(self.timestep)

        self.lcam = self.robot.getDevice("left_cam")
        self.lcam.enable(self.timestep)

        self.color_sensor = self.robot.getDevice("color_sensor")
        self.color_sensor.enable(self.timestep)

        self.emitter = self.robot.getDevice("emitter")

        self.gps = self.robot.getDevice("gps")
        self.gps.enable(self.timestep)

        self.lidar = self.robot.getDevice("lidar")
        self.lidar.enable(self.timestep)

        self.compass = self.robot.getDevice("inertial_unit")
        self.compass.enable(self.timestep)

        # Initialize other sensors here

    def get_sensor_data(self):
        # gps data
        self.gps_data = [self.gps.getValues()[0], self.gps.getValues()[1], self.gps.getValues()[2]]
        self.pos_x = round(self.gps_data[0] * 100 + 50)
        self.pos_z = round(self.gps_data[2] * 100 + 50)

        # compass data
        self.compass_data = self.compass.getRollPitchYaw()[2]
        self.compass_data = self.compass_data * 180 / math.pi
        self.compass_data = round(self.compass_data, 1)

        # color data
        image = self.color_sensor.getImage();

        r = self.color_sensor.imageGetRed(image, 1, 0, 0)
        g = self.color_sensor.imageGetGreen(image, 1, 0, 0)
        b = self.color_sensor.imageGetBlue(image, 1, 0, 0)

        self.color_data = []
        self.color_data.append(r)
        self.color_data.append(g)
        self.color_data.append(b)
        
        # lidar data
        self.lidar_values = []
        range_image = self.lidar.getRangeImage()
        for layer in range(4):
            self.lidar_values.append([])
            for point in range(512):
                self.lidar_values[layer].append(
                    round(
                        range_image[layer * 512 + point] * 100, 2
                    )
                ) 

        self.reported_victims = []
 
    def state_machine(self):
        """Main state machine loop"""
        while self.robot.step(self.timestep) != -1:
            # Common sensor readings
            self.get_sensor_data()
            
            if self.current_state == STATES["INIT"]:
                self.init_state()
            
            elif self.current_state == STATES["EXPLORING"]:
                self.exploring_state()
            
            elif self.current_state == STATES["ROTATING"]:
                self.rotating_state()
            
            elif self.current_state == STATES["MOVING"]:
                self.moving_state()
            
            elif self.current_state == STATES["SIGN_DETECTED"]:
                self.sign_detection_state()
            
            elif self.current_state == STATES["OBSTACLE_AVOIDANCE"]:
                self.obstacle_state()
            
            elif self.current_state == STATES["ESCAPE"]:
                self.escape_state()

    # state handlers
    def init_state(self):
        print("INIT STATE")
        self.current_state = STATES["EXPLORING"]

    def exploring_state(self):
        print("EXPLORING STATE")
        
        if self.detect_sign(): # If sign detected
            self.current_state = STATES["SIGN_DETECTED"]
        elif self.check_obstacle(): # If path blocked
            self.current_state = STATES["OBSTACLE_AVOIDANCE"]
        else: # If needs rotation
            self.current_state = STATES["ROTATING"]

    def rotating_state(self):
        """Rotation handling"""
        print("ROTATING STATE")
        # If rotation complete
        self.current_state = STATES["MOVING"]

    def moving_state(self):
        """Forward movement"""
        print("MOVING STATE")
        # If movement complete
        self.current_state = STATES["EXPLORING"]

    def sign_detection_state(self):
        
        self.current_state = STATES["EXPLORING"]

    def obstacle_state(self):
        """Obstacle avoidance"""
        print("OBSTACLE AVOIDANCE STATE")
        # After avoiding obstacle
        self.current_state = STATES["ROTATING"]

    def escape_state(self):
        """Emergency escape"""
        print("ESCAPE STATE")
        # After escaping
        self.current_state = STATES["EXPLORING"]

    # helper functions
    def detect_sign(self):
        # return False; # TODO: remove this line
        img = self.rcam.getImage()
        if img:
            if self.check_sign(img, "RIGHT"):
                return True
            
        img = self.lcam.getImage()
        if img:
            if self.check_sign(img, "LEFT"):
                return True
        
        return False
    
    def check_sign(self, img, cam):
        if cam == "RIGHT":
            img = self.rcam.getImage()
            width = self.rcam.getWidth()
            height = self.rcam.getHeight()
        else:
            img = self.lcam.getImage()
            width = self.lcam.getWidth()
            height = self.lcam.getHeight()
            
        channels = len(img) // (width * height)
        
        if channels not in [3, 4]:
            raise ValueError("Invalid image format")
        
        img = np.frombuffer(img, np.uint8).reshape((height, width, channels))
        img =  cv.cvtColor(img, cv.COLOR_BGRA2BGR)
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
                    self.check_victims(img, cam)
                    return True
                elif all(40 <= angle <= 50 or 130 <= angle <= 140 for angle in angles):
                    print("Rhombus detected") if DEBUG else None
                    self.check_hazards(img, cam)
                    return True
    
        return False
    
    def check_victims(self, img, cam):
        # H S U
        if cam == "RIGHT":
            img = self.rcam.getImage()
            width = self.rcam.getWidth()
            height = self.rcam.getHeight()
        else:
            img = self.lcam.getImage()
            width = self.lcam.getWidth()
            height = self.lcam.getHeight()
            
        channels = len(img) // (width * height)
        
        img = np.frombuffer(img, np.uint8).reshape((height, width, channels))
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
            self.report('H')
            return 'H'
        elif num_contours_first == 2 and num_contours_third == 2:
            print("found S")
            self.report('S')
            return 'S'
        elif num_contours_first == 3 and num_contours_third == 3:
            print("found U")
            self.report('U')
            return 'U'
        else:
            return None
        pass
    
    def check_hazards(self, img, cam):
        # TODO: Implement hazard detection logic
        pass
    

    def report(self, sign_type):
        global reported_victims

        self.stop(1300)

        if (self.pos_x, self.pos_z) in self.reported_victims:
            print(f"Victim at ({self.pos_x}, {self.pos_z}) already reported.")
            return

        self.reported_victims.append((self.pos_x, self.pos_z))

        sign_type = bytes(sign_type, "utf-8")
        msg = struct.pack("i i c", self.pos_x, self.pos_z, sign_type)

        self.emitter.send(msg)
        self.robot.step(self.timestep)

        if DEBUG:
            print(f"Reported victim of type '{sign_type.decode()}' at ({self.pos_x}, {self.pos_z}).")

    def delay(self, ms):
        init_time = self.robot.getTime()
        while (self.robot.step(self.timestep) != -1):
            if (self.robot.getTime() - init_time) * 1000.0 > ms:
                break
        
    def stop(self, duration):
        self.lwheel.setVelocity(0)
        self.rwheel.setVelocity(0)
        self.delay(duration)

    def check_obstacle(self):
        if self.lidar_values[2][0] < 0.5:
            print("Obstacle detected!") if DEBUG else None
            return True
        return False

# Main execution
if __name__ == "__main__":
    bot = RescueBot()
    bot.state_machine()