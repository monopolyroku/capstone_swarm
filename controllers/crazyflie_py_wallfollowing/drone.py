"""
coding: utf-8

file: drone.py

Author: @monopolyroku

"""

from controller import Robot
from pid_controller import pid_velocity_fixed_height_controller
from wall_following import WallFollowing
from math import cos, sin


class Drone:
    def __init__(self, robot, name, timestep):
        self.robot = Robot()
        self.name = name
        self.timestep = timestep
        FLYING_ATTITUDE = 1
        
        # initialise motors
        self.m1_motor = robot.getDevice("m1_motor")
        self.m2_motor = robot.getDevice("m2_motor")
        self.m3_motor = robot.getDevice("m3_motor")
        self.m4_motor = robot.getDevice("m4_motor")
        
        for motor in [self.m1_motor, self.m2_motor, self.m3_motor, self.m4_motor]:
            motor.setPosition(float('inf'))
            motor.setVelocity(0)
            
        
        # initliase sensors
        self.imu = robot.getDevice("inertial_unit")
        self.imu.enable(timestep)
        self.gps = robot.getDevice("gps")
        self.gps.enable(timestep)
        self.gyro = robot.getDevice("gyro")
        self.gyro.enable(timestep)
        self.camera = robot.getDevice("camera")
        self.camera.enable(timestep)
        self.range_front = robot.getDevice("range_front")
        self.range_front.enable(timestep)
        self.range_left = robot.getDevice("range_left")
        self.range_left.enable(timestep)
        self.range_back = robot.getDevice("range_back")
        self.range_back.enable(timestep)
        self.range_right = robot.getDevice("range_right")
        self.range_right.enable(timestep)
        
        # initialise PID and wall-following modules
        self.PID_controller = pid_velocity_fixed_height_controller()
        self.wall_following = WallFollowing(angle_value_buffer=0.01, reference_distance_from_wall=0.5,
        max_forward_speed=0.3, init_state=WallFollowing.StateWallFollowing.FORWARD)
        self.height_desired = FLYING_ATTITUDE
        self.autonomous_mode = False
        
        # initialise state variables
        self.forward_desired = 0
        self.sideways_desired = 0
        self.yaw_desired = 0
        self.height_diff_desired = 0
        self.first_time = True
        self.past_time = 0 
        self.past_x_global = 0
        self.past_y_global = 0
     
     
    def handle_keyboard_input(self, key):
            """Handles keyboard inputs to control the drone."""
            if key == Keyboard.UP:
                self.forward_desired += 0.5
            elif key == Keyboard.DOWN:
                self.forward_desired -= 0.5
            elif key == Keyboard.RIGHT:
                self.sideways_desired -= 0.5
            elif key == Keyboard.LEFT:
                self.sideways_desired += 0.5
            elif key == ord('Q'):
                self.yaw_desired += 1
            elif key == ord('E'):
                self.yaw_desired -= 1
            elif key == ord('W'):
                self.height_diff_desired = 0.1
            elif key == ord('S'):
                self.height_diff_desired = -0.1
            elif key == ord('A'):
                self.autonomous_mode = True
                print(f"{self.name}: Autonomous mode ON")
            elif key == ord('D'):
                self.autonomous_mode = False
                print(f"{self.name}: Autonomous mode OFF")   
        
    def update(self, dt): 
      
           # Handle first-time initialization
           if self.first_time:
                self.past_x_global = self.gps.getValues()[0]
                self.past_y_global = self.gps.getValues()[1]
                self.past_time = self.robot.getTime()
                self.first_time = False
              
           # get sensor data
           roll = self.imu.getRollPitchYaw()[0]
           pitch = self.imu.getRollPitchYaw()[1]
           yaw = self.imu.getRollPitchYaw()[2]
           yaw_rate = self.gyro.getValues()[2]
           x_global = self.gps.getValues()[0]
           y_global = self.gps.getValues()[1]
           altitude = self.gps.getValues()[2]
           
           # Calculate global velocities
           v_x_global = (x_global - self.past_x_global) / dt
           v_y_global = (y_global - self.past_y_global) / dt
            
           # Get body-fixed velocities
           cos_yaw = cos(yaw)
           sin_yaw = sin(yaw)
           v_x = v_x_global * cos_yaw + v_y_global * sin_yaw
           v_y = -v_x_global * sin_yaw + v_y_global * cos_yaw
           
           # Update height
           self.height_desired += self.height_diff_desired * dt
    
           # Get range sensor data
           range_front_value = self.range_front.getValue() / 1000
           range_right_value = self.range_right.getValue() / 1000
           range_left_value = self.range_left.getValue() / 1000
    
           # Wall following
           direction = WallFollowing.WallFollowingDirection.LEFT
           range_side_value = range_right_value
           cmd_vel_x, cmd_vel_y, cmd_ang_w, _ = self.wall_following.wall_follower(
                range_front_value, range_side_value, yaw, direction, self.robot.getTime())
           
           
            # Update commands if in autonomous mode
           if self.autonomous_mode:
                self.forward_desired = cmd_vel_x
                self.sideways_desired = cmd_vel_y
                self.yaw_desired = cmd_ang_w
          
           # Example: Use PID to compute motor commands
           motor_power = self.PID_controller.pid(dt, 0, 0, 0, self.height_desired,
                                                  roll, pitch, yaw_rate, altitude, 0, 0)
           self.m1_motor.setVelocity(-motor_power[0])
           self.m2_motor.setVelocity(motor_power[1])
           self.m3_motor.setVelocity(-motor_power[2])
           self.m4_motor.setVelocity(motor_power[3])
           
           # update past state var
           past_time = robot.getTime()
           past_x_global = x_global
           past_y_global = y_global          
              
              