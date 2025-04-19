from controller import Robot, Receiver, Emitter # Import Receiver and Emitter
import struct # Use the struct module in order to pack the message sent to the Main Supervisor

robot = Robot()

receiver = robot.getDevice("receiver") # Retrieve the receiver and emitter by device name
emitter = robot.getDevice("emitter")
gps = robot.getDevice("gps") # Retrieve the gps by device name

timestep = int(robot.getBasicTimeStep())

receiver.enable(timestep) # Enable the receiver. Note that the emitter does not need to call enable()

while robot.step(timestep) != -1:
    #_______SENDING_______
    victimType = bytes('H', "utf-8") # The victim type being sent is the letter 'H' for harmed victim

    position = gps.getValues() # Get the current gps position of the robot
    x = int(position[0] * 100) # Get the xy coordinates, multiplying by 100 to convert from meters to cm 
    y = int(position[2] * 100) # We will use these coordinates as an estimate for the victim's position

    message = struct.pack("i i c", x, y, victimType) # Pack the message.
        
    emitter.send(message) # Send out the message

    #_______RECEIVING_______
    if receiver.getQueueLength() > 0: # If receiver queue is not empty
        receivedData = receiver.getBytes()
        tup = struct.unpack('c', receivedData) # Parse data into character
        if tup[0].decode("utf-8") == 'L': # 'L' means lack of progress occurred
            print("Detected Lack of Progress!") 
        receiver.nextPacket() # Discard the current data packet