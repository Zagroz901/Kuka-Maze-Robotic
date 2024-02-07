"""youBot controller for picking a cube from the conveyor and 
    placing it on robot carrier."""

from controller import Robot

# Create the Robot instance.
robot = Robot()

# Get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

#! Inizialize base motors.
wheels = []
wheels.append(robot.getDevice("wheel1"))
wheels.append(robot.getDevice("wheel2"))
wheels.append(robot.getDevice("wheel3"))
wheels.append(robot.getDevice("wheel4"))
for wheel in wheels:
    # Activate controlling the motors setting the velocity.
    # Otherwise by default the motor expects to be controlled in force or position,
    # and setVelocity will set the maximum motor velocity instead of the target velocity.
    wheel.setPosition(float('+inf'))

#! Initialize arm motors.
armMotors = []
armMotors.append(robot.getDevice("arm1"))
armMotors.append(robot.getDevice("arm2"))
armMotors.append(robot.getDevice("arm3"))
armMotors.append(robot.getDevice("arm4"))
armMotors.append(robot.getDevice("arm5"))
# Set the maximum motor velocity.
armMotors[0].setVelocity(1.5) # maxVelocity = 1.5
armMotors[1].setVelocity(1.5)
armMotors[2].setVelocity(1.5)
armMotors[3].setVelocity(0.5)
armMotors[4].setVelocity(1.5)

#! Initialize arm position sensors.
# These sensors can be used to get the current 
# joint position and monitor the joint movements.
armPositionSensors = []
armPositionSensors.append(robot.getDevice("arm1sensor"))
armPositionSensors.append(robot.getDevice("arm2sensor"))
armPositionSensors.append(robot.getDevice("arm3sensor"))
armPositionSensors.append(robot.getDevice("arm4sensor"))
armPositionSensors.append(robot.getDevice("arm5sensor"))
for sensor in armPositionSensors:
    sensor.enable(timestep)

#! Initialize gripper motors.
finger1 = robot.getDevice("finger::left")
finger2 = robot.getDevice("finger::right")
# Set the maximum motor velocity.
finger1.setVelocity(1.5)
finger2.setVelocity(1.5) # 0.03
# Read the miminum and maximum position of the gripper motors.
fingerMinPosition = finger1.getMinPosition()
fingerMaxPosition = finger1.getMaxPosition()

#! Generic forward motion function
def forward(time):
    for wheel in wheels:
        wheel.setVelocity(7.0) # maxVelocity = 14.81
    robot.step(time * timestep)

def backward(time):
    for wheel in wheels:
        wheel.setVelocity(-7.0) # maxVelocity = 14.81
    robot.step(time * timestep)

#! Generic stop function
def halt():
    for wheel in wheels:
        wheel.setVelocity(0.0)

def fold_arms():
    armMotors[0].setPosition(-2.9)
    armMotors[1].setPosition(1.5)
    armMotors[2].setPosition(-2.6)
    armMotors[3].setPosition(1.7)
    armMotors[4].setPosition(0)

def stretch_arms():
    armMotors[0].setPosition(2.9)
    armMotors[1].setPosition(-1.0)
    armMotors[2].setPosition(2.5)
    armMotors[3].setPosition(-1.7)
    armMotors[4].setPosition(0)

def turn_around(time):
#TODO: robot.step(70 * timestep) - produces a 90 degrees turn
    wheels[0].setVelocity(14)
    wheels[1].setVelocity(-14)
    wheels[2].setVelocity(14)
    wheels[3].setVelocity(-14)
    robot.step(time * timestep)
    # forward()
#? arm[0] maxPosition = 2.9 || -2.9
#? arm[1] maxPosition = 1.5 || -1.0
#? arm[2] maxPosition = 2.5 || -2.6
#? arm[3] maxPosition = 1.7 || -1.7
#? arm[4] maxPosition = 2.9 || -2.9
def pick_up():
    print("pick")
    armMotors[1].setPosition(-1.13)
    armMotors[2].setPosition(-1.10)
    armMotors[3].setPosition(-1.3)
    # armMotors[1].setPosition(-1.13)
    # armMotors[2].setPosition(-1.14)
    # armMotors[3].setPosition(-1.4)
    # armMotors[1].setPosition(0)
    # armMotors[2].setPosition(0)
    # armMotors[3].setPosition(0)
    finger1.setPosition(fingerMaxPosition)
    finger2.setPosition(fingerMaxPosition)

    # Monitor the arm joint position to 
    # detect when the motion is completed.
    while robot.step(timestep) != -1:
        if abs(armPositionSensors[3].getValue() - (-1.2)) < 0.01:
        # Motion completed.
            break
    finger1.setPosition(0.013)     # Close gripper.
    finger2.setPosition(0.013)
    robot.step(50 * timestep)    # Wait until the gripper is closed.
    armMotors[1].setPosition(0)    # Lift arm.
    # Wait until the arm is lifted.
    # robot.step(200 * timestep)
def open_grippers():
    finger1.setPosition(fingerMaxPosition)
    finger2.setPosition(fingerMaxPosition)
    robot.step(70 * timestep)

def drop():
    armMotors[0].setPosition(-2.9)
    armMotors[1].setPosition(0)
    armMotors[2].setPosition(-1)
    armMotors[3].setPosition(-1)
    armMotors[2].setPosition(-1.7)
    armMotors[4].setPosition(2.9)

def hand_up():
    armMotors[0].setPosition(0)
    armMotors[1].setPosition(0)
    armMotors[2].setPosition(0)
    armMotors[3].setPosition(0)
    armMotors[4].setPosition(0)
    finger1.setPosition(fingerMaxPosition)
    finger2.setPosition(fingerMaxPosition)


#TODO: youBot default position:
#TODO: x: -2.4; y: 0; z: 0.101
#TODO: robot.step(70 * timestep) - produces a 90 degrees turn
# Move arm and open gripper.
#? arm[0] maxPosition = 2.9 || -2.9
#? arm[1] maxPosition = 1.5 || -1.0
#? arm[2] maxPosition = 2.5 || -2.6
#? arm[3] maxPosition = 1.7 || -1.7
#? arm[4] maxPosition = 2.9 || -2.9


#! Functions call start here
# forward(520)# Move forward for specified timeStep value
halt()
pick_up()
# turn_around(50)
drop()
# forward(400)
# turn_around(70)
# halt()
# open_grippers()
# hand_up()
# backward(60)
# halt()
# robot.step(200 * timestep)
# turn_around(70)
# fold_arms()
# forward(300)
# halt()
