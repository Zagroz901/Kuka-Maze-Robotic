from controller import Robot

class RobotController(Robot):
    def __init__(self):
        super(RobotController, self).__init__()
        
        # Time step for the simulation
        self.time_step = int(self.getBasicTimeStep())
        
        # Sensor threshold to detect walls
        self.WALL_THRESHOLD = 1000
        
        # Movement velocities
        self.MOVEMENT_VELOCITY = 12
        self.base_speed = 6.28
        self.turn_speed = 8
        
        # Initialize wheel motors
        self.front_left_motor = self.getDevice('wheel1')
        self.front_right_motor = self.getDevice('wheel2')
        self.back_left_motor = self.getDevice('wheel3')
        self.back_right_motor = self.getDevice('wheel4')
        
        # Set wheel motors position and velocity
        motors = [self.front_left_motor, self.front_right_motor, self.back_left_motor, self.back_right_motor]
        for motor in motors:
            motor.setPosition(float('inf'))
            motor.setVelocity(0.0)
        
        # Initialize sensors
        self.wall_sensors = []
        self.sensors_index = ['if'+str(i) for i in range(1, 13)]
        for sensor_name in self.sensors_index:
            sensor = self.getDevice(sensor_name)
            sensor.enable(self.time_step)
            self.wall_sensors.append(sensor)
        
        # Initialize the camera
        self.camera = self.getDevice("camera")
        self.camera.enable(self.time_step)
        self.camera.recognitionEnable(self.time_step)

        # Set initial values for control variables
        self.turn_duration = 1500  # Placeholder: the duration of a turn (to be calibrated)
        self.turn_around_duration = 3000  # Placeholder: the duration of a 180-degree turn (to be calibrated)

    # Main robot control loop
    def run(self):
        while self.step(self.time_step) != -1:
            if self.is_path_right():
                # If there is a path to the right, take it.
                print("turn_right")
                self.turn_left(self.turn_speed)
                self.wait_for_turn_to_complete()
                self.move_forward(self.MOVEMENT_VELOCITY)
                self.step(50 *self.time_step)
            elif self.is_path_forward():
                # If there's no path to the right but a path forward, continue forward.
                print("move_forward")
                self.move_forward(self.MOVEMENT_VELOCITY)
                
            elif self.is_path_left():
                # If there's no path to the right or forward, but there is a path to the left, take it.
                print("turn_left")
                self.turn_right(self.turn_speed)
                self.wait_for_turn_to_complete()
                self.move_forward(self.MOVEMENT_VELOCITY)
                self.step(50 *self.time_step)

            else:
                # If all paths are blocked, turn around.
                
                print("turn_around")
                self.turn_around(self.turn_speed)
                self.wait_for_turn_around_to_complete()
    
    def is_path_right(self):
        # Check if there is a path to the right using the right sensors
        return all(sensor.getValue() >= self.WALL_THRESHOLD for sensor in self.wall_sensors[9:12])
    
    def is_path_forward(self):
        # Check if there is a path forward using the front sensors
        return all(sensor.getValue() >= self.WALL_THRESHOLD for sensor in self.wall_sensors[:3])
    
    def is_path_left(self):
        # Check if there is a path to the left using the left sensors
        return all(sensor.getValue() >= self.WALL_THRESHOLD for sensor in self.wall_sensors[3:6])
    
    def move_forward(self, speed):
        self.set_wheel_speeds(speed, speed, speed, speed)
    
    def turn_right(self, speed):
        # To turn right, the robot should rotate clockwise
        self.rotate_clockwise(speed)
    
    def turn_left(self, speed):
        # To turn left, the robot should rotate counter-clockwise
        self.rotate_counterclockwise(speed)
    
    def turn_around(self, speed):
        # To turn around, the robot can rotate clockwise roughly 180 degrees
        self.rotate_clockwise(speed)
    
    def rotate_clockwise(self, speed):
        # Rotates the robot clockwise by setting appropriate wheel speeds
        self.set_wheel_speeds(speed, -speed, speed, -speed)
    
    def rotate_counterclockwise(self, speed):
        # Rotates the robot counter-clockwise by setting appropriate wheel speeds
        self.set_wheel_speeds(-speed, speed, -speed, speed)
    
    def set_wheel_speeds(self, fl, fr, bl, br):
        self.front_left_motor.setVelocity(fl)
        self.front_right_motor.setVelocity(fr)
        self.back_left_motor.setVelocity(bl)
        self.back_right_motor.setVelocity(br)
    
    def wait_for_turn_to_complete(self):
        # Wait for a turn to complete by stepping through simulation time
        self.step(int(self.turn_duration))
    
    def wait_for_turn_around_to_complete(self):
        # Wait longer for a full turn around
        self.step(int(self.turn_around_duration))


robot = RobotController()
robot.run() 