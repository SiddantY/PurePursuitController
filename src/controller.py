import rospy
from gazebo_msgs.srv import GetModelState, GetModelStateResponse
from gazebo_msgs.msg import ModelState
from ackermann_msgs.msg import AckermannDrive
import numpy as np
from std_msgs.msg import Float32MultiArray
import math
from util import euler_to_quaternion, quaternion_to_euler
import time

class vehicleController():

    def __init__(self):
        # Publisher to publish the control input to the vehicle model
        self.controlPub = rospy.Publisher("/ackermann_cmd", AckermannDrive, queue_size = 1)
        self.prev_vel = 0
        self.L = 1.75 # Wheelbase, can be get from gem_control.py
        self.log_acceleration = True
        #logging
        self.prev_time = rospy.get_time()
        self.file_object_results  = open("results_part1.txt", "w+")
    def getModelState(self):
        # Get the current state of the vehicle
        # Input: None
        # Output: ModelState, the state of the vehicle, contain the
        #   position, orientation, linear velocity, angular velocity
        #   of the vehicle
        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            serviceResponse = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            resp = serviceResponse(model_name='gem')
        except rospy.ServiceException as exc:
            rospy.loginfo("Service did not process request: "+str(exc))
            resp = GetModelStateResponse()
            resp.success = False
        return resp


    # Tasks 1: Read the documentation https://docs.ros.org/en/fuerte/api/gazebo/html/msg/ModelState.html
    #       and extract yaw, velocity, vehicle_position_x, vehicle_position_y
    # Hint: you may use the the helper function(quaternion_to_euler()) we provide to convert from quaternion to euler
    def extract_vehicle_info(self, currentPose):

        ####################### TODO: Your TASK 1 code starts Here #######################
        pos_x, pos_y, vel, yaw = 0, 0, 0, 0
        # print(currentPose)
        pos_x = currentPose.pose.position.x
        pos_y = currentPose.pose.position.y
        rot_q = currentPose.pose.orientation

        (roll,pitch,yaw) = quaternion_to_euler(rot_q.x,rot_q.y,rot_q.z,rot_q.w)

        # Calculate linear velocity using the Euclidean distance formula
        vel_x = currentPose.twist.linear.x
        vel_y = currentPose.twist.linear.y
        vel_z = currentPose.twist.linear.z
        # vel_v = [vel_x, vel_y, vel_z]
        
        vel = math.sqrt(vel_x**2 + vel_y**2 + vel_z**2)
        # print("X",pos_x,"Y", pos_y,"Yaw", yaw, "Vel", vel)
        ####################### TODO: Your Task 1 code ends Here #######################

        return pos_x, pos_y, vel, yaw # note that yaw is in radian

    # Task 2: Longtitudal Controller
    # Based on all unreached waypoints, and your current vehicle state, decide your velocity
    def longititudal_controller(self, curr_x, curr_y, curr_vel, curr_yaw, future_unreached_waypoints):
        straight_speed = 14.0
        turn_speed = 8.0
        if len(future_unreached_waypoints) >= 2:
            first_waypoint = future_unreached_waypoints[0]
            second_waypoint = future_unreached_waypoints[1]
            angle = math.atan2(second_waypoint[1] - first_waypoint[1], second_waypoint[0] - first_waypoint[0])
            # Ensure the angle is within the range [0, 2*pi]
            angle = (angle + 2 * math.pi) % (2 * math.pi)
        else:
            angle = 0.0
        angle_error = abs(math.degrees(curr_yaw) - math.degrees(angle))
        angle_error = angle_error % 360
        if 340 > angle_error > 10: 
            target_velocity = turn_speed
        else:
            target_velocity = straight_speed
        # print(target_velocity,math.degrees(angle),math.degrees(curr_yaw),angle_error)
        return target_velocity


    # Task 3: Lateral Controller (Pure Pursuit)
    def pure_pursuit_lateral_controller(self, curr_x, curr_y, curr_yaw, target_point, future_unreached_waypoints):

        ####################### TODO: Your TASK 3 code starts Here #######################
        target_steering = 0.0
        lookahead_distance = 5.0
        lookahead_point = None
        for waypoint in future_unreached_waypoints:
            dist = math.dist((curr_x, curr_y), waypoint)
            if dist > lookahead_distance:
                lookahead_point = waypoint
                break
        
        if lookahead_point:
            dist = math.dist((curr_x, curr_y), lookahead_point)
            angle_to_waypoint = math.atan2(lookahead_point[1] - curr_y, lookahead_point[0] - curr_x)
            target_steering = np.arctan(2 * self.L * np.sin(angle_to_waypoint - curr_yaw) / dist)


        
        ####################### TODO: Your TASK 3 code starts Here #######################
        # print(target_steering)
        return target_steering


    def execute(self, currentPose, target_point, future_unreached_waypoints):
        # Compute the control input to the vehicle according to the
        # current and reference pose of the vehicle
        # Input:
        #   currentPose: ModelState, the current state of the vehicle
        #   target_point: [target_x, target_y]
        #   future_unreached_waypoints: a list of future waypoints[[target_x, target_y]]
        # Output: None

        curr_x, curr_y, curr_vel, curr_yaw = self.extract_vehicle_info(currentPose)

        # Acceleration Profile
        if self.log_acceleration:
            acceleration = (curr_vel- self.prev_vel) * 100 # Since we are running in 100Hz
            self.prev_vel = curr_vel
            # print(acceleration)


        target_velocity = self.longititudal_controller(curr_x, curr_y, curr_vel, curr_yaw, future_unreached_waypoints)
        target_steering = self.pure_pursuit_lateral_controller(curr_x, curr_y, curr_yaw, target_point, future_unreached_waypoints)

        # current_time = rospy.get_time()
        # time_diff = current_time - self.prev_time
        # if time_diff > 0:
        #     acceleration = (curr_vel - self.prev_vel) / time_diff
        #     # print(current_time)
        # else:
        #     acceleration = 0
        # self.prev_vel = curr_vel
        # self.prev_time = current_time
        self.file_object_results.write(str(curr_x) + "," + str(curr_y)+","+str(acceleration)+","+str(future_unreached_waypoints[0][0])+","+str(future_unreached_waypoints[0][1])+"\n")


        #Pack computed velocity and steering angle into Ackermann command
        newAckermannCmd = AckermannDrive()
        newAckermannCmd.speed = target_velocity
        newAckermannCmd.steering_angle = target_steering

        # Publish the computed control input to vehicle model
        self.controlPub.publish(newAckermannCmd)

    def stop(self):
        newAckermannCmd = AckermannDrive()
        newAckermannCmd.speed = 0
        self.controlPub.publish(newAckermannCmd)





