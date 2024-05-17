#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
import pickle
import os
import rospkg
from mapping_operations import get_map_paths, set_room_bounds, get_all_map, get_map_coords_from_index
import numpy as np
from room_selector import select_room
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

MAP_PATH, ROOMS_PATH, MAP_INFO_PATH = get_map_paths()

class HumanSubscriber:
    def __init__(self):
        self.human_orientation = None
        self.human_found = False
        self.subscriber()

    def update_human_orientation(self, data):
        self.human_found = True
        self.human_orientation = data.data

    def subscriber(self):
        rospy.Subscriber('/human_orientation', Float32, self.update_human_orientation)
        rospy.sleep(0.2)

class MovementClient:
    def __init__(self):
        rospy.init_node("room_traveler", anonymous=True)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.map, self.dist_mat, self.map_res, self.map_origin = get_all_map()
        

        if not os.path.exists(ROOMS_PATH) or input("Set rooms? (Y/n)") == "Y":
            set_room_bounds() 

        with open(ROOMS_PATH, 'rb') as file:
            self.rooms = pickle.load(file)
        
        print("========================================\nROOMS:")
        for i, room in enumerate(self.rooms):
            print("Room", i, ":", room)
        print("========================================")

        self.goal_room = select_room()
        #Subscribers
        self.human_subscriber = HumanSubscriber()

        #Publishers
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
    

    def coord_user_input(self):
        def request_coord(val):
            return float(input(val + ": "))
        
        x = request_coord("X")
        y = request_coord("Y")
        return x, y
    
    def move_to_coords(self, x, y):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 1.0 #delete this?

        self.client.send_goal(goal)
        self.client.wait_for_result()

        return self.client.get_state() == GoalStatus.SUCCEEDED


    '''
    Randomly sample coordinates in the room until coordinates are some distance away from all obstacles

    Returns:
    - x, y: sampled goal coordinates
    '''
    def sample_room(self, room_num, min_obstacle_dist=0.25, max_samples=100):
        rects = self.rooms[room_num]['rects']
        weights = np.zeros(len(rects))
        for i, rect in enumerate(rects):
            weights[i] = rect['D_x'] * rect['D_y']
        
        weights /= np.sum(weights)
        rand_rect = rects[np.random.choice(len(rects), p=weights)]

        min_x = rand_rect['x']
        min_y = rand_rect['y']
        max_x = min_x + rand_rect['D_x']
        max_y = min_y + rand_rect['D_y']

        valid_sample = False
        sample_num = 0
        while not valid_sample:
            sample_num += 1
            x = int(np.random.uniform(min_x, max_x))
            y = int(np.random.uniform(min_y, max_y))
            if self.dist_mat[y][x] * self.map_res > min_obstacle_dist: #Check obstacle colision
                valid_sample = True
            
            if sample_num >= max_samples and not valid_sample:
                print("Too many samples")
                break
        
        return x, y

    def move_to_room(self, room_num):
        goal_x_index, goal_y_index = self.sample_room(room_num)
        print("GOAL:", goal_x_index, ",", goal_y_index)
        goal_x, goal_y = get_map_coords_from_index(goal_x_index, goal_y_index, self.map, self.map_res, self.map_origin)
        print("GOAL COORDS", goal_x, goal_y)
        goal_reached = self.move_to_coords(goal_x, goal_y)
        print(goal_reached)
    

    def publish_velocity(self, linear, angular):
        cmd_vel = Twist()
        cmd_vel.linear.x = linear
        cmd_vel.angular.z = angular
        self.vel_pub.publish(cmd_vel)


    def move_to_human(self, linear_speed=0.2, angular_speed= 0.2):
        MAX_LINEAR = 0.2
        MAX_ANGULAR = 1 * np.pi

        linear = min(MAX_LINEAR, linear_speed)

        orientation = self.human_subscriber.human_orientation
        angular = (orientation - 0.5) * angular_speed
        angular = min(MAX_ANGULAR, angular)
        self.publish_velocity(linear, angular)


    def explore_room(self, room_num):
        while not rospy.is_shutdown():
            if self.human_subscriber.human_found:
                self.move_to_human()
            else:
                self.move_to_room(room_num)

        
    def find_human(self):
        self.move_to_room(self.goal_room)
        self.explore_room(self.goal_room)

# if __name__ =='main':
movement_client = MovementClient()
movement_client.find_human()
# while not rospy.is_shutdown():
#     x, y = movement_client.coord_user_input()
#     movement_client.move_to_coords(x, y)