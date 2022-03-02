#!/usr/bin/env python3.8
import copy
import math
import random

import numpy as np
from numpy.linalg import inv
import rospy
# VERY IMPORTANT TO SUBSCRIBE TO MULTIPLE TOPICS
import message_filters
import tf2_ros
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import *
from cv_bridge import CvBridge
from visualization_msgs.msg import *
import cv2
import tf2_geometry_msgs # Do not use geometry_msgs. Use this for PoseStamped (depois perguntar porque)


class Driver:

    def __init__(self):
        # name of the car with \ and without
        self.node = rospy.get_name()
        self.name = self.node[1:len(self.node)]
        self.id = 1
        # Define the goal to which the robot should move
        self.goal = PoseStamped
        self.goal_active = False
        self.navigation_active = False
        # get param to see the photos
        self.image_flag = rospy.get_param('~image_flag', 'True')
        # pos initialization -------------------
        self.preyPos = PoseStamped()
        self.preyPos.pose.position.x = math.inf
        self.preyPos.pose.position.y = math.inf
        self.attackerPos = PoseStamped()
        self.attackerPos.pose.position.x = math.inf
        self.attackerPos.pose.position.y = math.inf
        self.teammatePos = PoseStamped()
        self.preyPos_back = PoseStamped()
        self.preyPos_back.pose.position.x = math.inf
        self.preyPos_back.pose.position.y = math.inf
        self.attackerPos_back = PoseStamped()
        self.attackerPos_back.pose.position.x = math.inf
        self.attackerPos_back.pose.position.y = math.inf
        self.teammatePos_back = PoseStamped()
        self.Hunting = False
        self.Running = False
        self.Navigating = False
        # ----------------------------
        # colors initialization ----------------
        self.attacker_color_min = (0, 0, 239)
        self.attacker_color_max = (31, 31, 255)
        self.prey_color_min = (0, 239, 0)
        self.prey_color_max = (31, 255, 31)
        self.teammate_color_min = (236, 0, 0)
        self.teammate_color_max = (255, 31, 31)
        # ---------------------------------
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        # publishes the marker of the cars
        self.publish_marker = rospy.Publisher(self.node + '/Markers', Marker, queue_size=1)
        # publishes the velocity of the car
        self.publisher_goal = rospy.Publisher(str(self.node) + '/cmd_vel', Twist, queue_size=1)
        # sees the goal 0.1s at a time
        self.timer = rospy.Timer(rospy.Duration(0.1), self.sendCommandCallback)

        # subscribes to see if there is a goal ( this part is going to be changed to the value )
        # stops existing, we need an if or a switch to choose the mode (attack, defense, navigating)
        self.goal_subscriber = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goalReceivedCallBack)
        # sees the team of the car
        self.whichTeam()
        self.br = CvBridge()
        # initialization of the list of laser scan points
        self.points = []
        self.points_back = []
        self.wp_to_pixels = []
        # subscribe to the laser scan values
        self.laser_subscriber = rospy.Subscriber(self.node + '/scan', LaserScan, self.Laser_Points)
        # Get the camera info, in this case is better in static values since the cameras have all the same values
        self.cameraIntrinsic = np.array([[1060.9338813153618, 0.0, 640.5],
                                        [0.0, 1060.9338813153618, 360.5],
                                        [0.0, 0.0, 1.0]])

        # camera extrinsic from the lidar to the camera (back and front are different extrinsic values )
        # this is the value from camera_rgb_optical_frame to scan
        self.lidar2cam = np.array([[0.0006, -1.0, -0.0008, -0.0],
                                  [0.0006, 0.0008, -1.0, -0.029],
                                  [1.0, 0.0006, 0.0006, -0.140]])

        # self.lidar2cam = np.array([[0, -1.0, 0, -0.0],
        #                           [0, 0, -1.0, -0.029],
        #                           [1.0, 0, 0, -0.140]])

        self.lidar2cam_back= np.array([[-0.0027, 1.0, -0.0002, 0.022],
                                        [-0.0009, -0.0002, -1.0, -0.029],
                                        [-1.0, -0.0027, 0.0009, -0.139]])
        # self.lidar2cam_back = np.array([[-0.002, 1.0, -0.000002, 0.022],
        #                                 [-0.002, -0.000002, -1.0, -0.029],
        #                                 [-1.0, -0.002, 0.002, -0.139]])

        # subscribes to the back and front images of the car
        self.image_subscriber_front = message_filters.Subscriber(self.node + '/camera/rgb/image_raw', Image)
        self.image_subscriber_back = message_filters.Subscriber(self.node + '/camera_back/rgb/image_raw', Image)
        ts = message_filters.TimeSynchronizer([self.image_subscriber_front, self.image_subscriber_back], 1)
        ts.registerCallback(self.GetImage)


    def whichTeam(self):
        red_names = rospy.get_param('/red_players')
        green_names = rospy.get_param('/green_players')
        blue_names = rospy.get_param('/blue_players')
        for idx, x in enumerate(red_names):
            if self.name == x:
                print('I am ' + str(self.name) + ' I am team red. I am hunting' + str(green_names) + 'and fleeing from' + str(blue_names))
                self.attacker_color_min = (100, 0, 0)
                self.attacker_color_max = (255, 31, 31)
                self.prey_color_min = (0, 170, 0)
                self.prey_color_max = (30, 255, 30)
                self.teammate_color_min = (0, 0, 100)
                self.teammate_color_max = (31, 31, 255)

            elif self.name == green_names[idx]:
                print('I am ' + str(self.name) + ' I am team green. I am hunting' + str(blue_names) + 'and fleeing from' + str(red_names))
                self.prey_color_min = (100, 0, 0)
                self.prey_color_max = (255, 31, 31)
                self.teammate_color_min = (0, 170, 0)
                self.teammate_color_max = (31, 255, 31)
                self.attacker_color_min = (0, 0, 100)
                self.attacker_color_max = (31, 31, 255)

            elif self.name == blue_names[idx]:
                print('I am ' + str(self.name) + ' I am team blue. I am hunting' + str(red_names) + 'and fleeing from' + str(green_names))
                self.teammate_color_min = (100, 0, 0)
                self.teammate_color_max = (255, 31, 31)
                self.attacker_color_min = (0, 170, 0)
                self.attacker_color_max = (31, 255, 31)
                self.prey_color_min = (0, 0, 100)
                self.prey_color_max = (31, 31, 255)
            else:
                pass

    def goalReceivedCallBack(self, goal_msg):

        self.goal = goal_msg # storing the goal inside the class
        self.goal_active = True


    def sendCommandCallback(self, msg):

        # Decision outputs a speed (linear velocity) and an angle (angular velocity)
        # input : goal
        # output : angle and speed
        print(self.Hunting, self.Running, self.Navigating)
        if self.Hunting is True:
            # print(self.goal)
            self.sendMarker_main("Hunting")
            self.goal = self.preyPos
            self.goal.header.frame_id = self.name + '/base_link'
            self.goal_active = True
            self.Navigating = False
            self.Running = False
            print("attack")
        else:
            if self.Navigating is True:
                self.goal_active = False
                self.Running = False
                self.sendMarker_main("Navigating")
                print('navigating while waiting goal')
            else:
                if self.Running is True:
                    self.Navigating = False
                    self.sendMarker_main("Running")
                    if math.isinf(self.attackerPos_back.pose.position.x) is False:
                        self.goal = self.attackerPos_back
                        self.goal.pose.position.x = self.attackerPos_back.pose.position.x
                        self.goal.pose.position.y = - self.attackerPos_back.pose.position.y
                    else:
                        self.goal = self.attackerPos
                        self.goal.pose.position.x = self.attackerPos.pose.position.x
                        self.goal.pose.position.y = - self.attackerPos.pose.position.y
                    self.goal.header.frame_id = self.name + '/base_link'
                    self.goal_active = True
                    print("running")
                else:
                    self.Navigating = False
                    self.Running = False
                    self.Hunting = False

        # verify if the goal is achieved
        if self.goal_active:
            distance_to_goal = self.computeDistanceToGoal(self.goal)
            # print(distance_to_goal)
            if distance_to_goal < 0.05:
                rospy.logwarn('I have achieved my goal!!!')
                self.goal_active = False


        # define driving behaviour according to the goal
        if self.goal_active:
            angle, speed = self.driveStraight(self.goal)
            if angle is None or speed is None: # can't transform target frame
                angle = 0
                speed = 0
        else:
            angle = 0
            speed = 0

        # Build the command message (Twist) and publish it
        command_msg = Twist()
        command_msg.linear.x = speed
        command_msg.angular.z = angle
        if self.Navigating is False:
            self.publisher_goal.publish(command_msg)

    def computeDistanceToGoal(self, goal):

        goal_present_time = copy.deepcopy(goal)
        goal_present_time.header.stamp = rospy.Time.now()

        target_frame = self.name + '/base_link'
        try:
            goal_in_base_link = self.tf_buffer.transform(goal_present_time, target_frame, rospy.Duration(1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr(
                'Could not transform goal from ' + goal.header.frame_id + ' to ' + target_frame + '. Will ignore this goal')
            return None, None

        x = goal_in_base_link.pose.position.x
        y = goal_in_base_link.pose.position.y

        distance = math.sqrt(x**2 + y**2)
        return distance

    def driveStraight(self, goal, minimum_speed=0.4, maximum_speed=1.4):
        """
        :param goal: where the robot wants to go
        :param minimum_speed: min speed the robot can go
        :param maximum_speed: max speed the robot can go
        :return: the angle and speed to use as command
        """
        goal_present_time = copy.deepcopy(goal)
        goal_present_time.header.stamp = rospy.Time.now()

        target_frame = self.name + '/base_link'
        try:
            goal_in_base_link = self.tf_buffer.transform(goal_present_time, target_frame, rospy.Duration(1))

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr('Could not transform goal from ' + goal.header.frame_id + ' to ' + target_frame + '. Will ignore')
            return None, None

        x = goal_in_base_link.pose.position.x
        y = goal_in_base_link.pose.position.y

        angle = math.atan2(y, x) # compute the angle

        distance = math.sqrt(x**2 + y**2)
        speed = 0.5 * (1/distance)
        # saturates the speed to minimum and maximum values
        speed = min(speed, maximum_speed)
        speed = max(speed, minimum_speed)
        return angle, speed

    def GetImage(self, data_front, data_back):
        # rospy.loginfo('Image received...')
        image_front = self.br.imgmsg_to_cv2(data_front, "bgr8")
        image_back = self.br.imgmsg_to_cv2(data_back, "bgr8")

        # Convert the image to a Numpy array since most cv2 functions

        # require Numpy arrays.
        frame_front = np.array(image_front, dtype=np.uint8)
        frame_back = np.array(image_back, dtype=np.uint8)

        # Process the frame using the process_image() function
        display_image_front = self.discover_car(frame_front, self.lidar2cam)
        display_image_back = self.discover_car_back(frame_back, self.lidar2cam_back)
        # with this we can see the red, blue and green cars
        if self.image_flag is True:
            cv2.imshow('front', display_image_front)
            cv2.imshow('back', display_image_back)
        cv2.waitKey(1)

    # camera que cria o modo de caça
    def discover_car(self, frame, camera_matrix):
        # Convert to HSV
        frame = copy.deepcopy(frame)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # create 3 channels !
        gray = cv2.merge((gray, gray, gray))
        mask_attacker = cv2.inRange(frame, self.attacker_color_min, self.attacker_color_max)
        mask_prey = cv2.inRange(frame, self.prey_color_min, self.prey_color_max)
        mask_teammate = cv2.inRange(frame, self.teammate_color_min, self.teammate_color_max)
        Center_t, mask_teammate = self.GetCentroid(mask_teammate, frame)
        Center_p, mask_prey = self.GetCentroid(mask_prey, frame)
        Center_a, mask_attacker = self.GetCentroid(mask_attacker, frame)

        # creates the image
        mask_final = mask_attacker + mask_prey + mask_teammate
        image = cv2.bitwise_or(frame, frame, mask=mask_final)
        image = cv2.add(gray, image)

        # get the transform the lidar values to pixel, draws it in the image
        pixel_cloud = self.lidar_to_image(camera_matrix, image, self.points)
        # probably here it receives the self.attackerPos , self.preyPos and self.teammatePos in case they exist
        self.preyPos = self.ClosestPoint(Center_p,  pixel_cloud, self.points)
        self.attackerPos = self.ClosestPoint(Center_a, pixel_cloud, self.points)
        self.teammatePos = self.ClosestPoint(Center_t, pixel_cloud, self.points)

        if math.isinf(self.preyPos.pose.position.x) is False:
            self.sendMarker(self.preyPos, self.prey_color_max, "prey")
        if math.isinf(self.attackerPos.pose.position.x) is False:
            self.sendMarker(self.attackerPos, self.attacker_color_max, "attacker")
        if math.isinf(self.teammatePos.pose.position.x) is False:
            self.sendMarker(self.teammatePos, self.teammate_color_max, "teammate")

        # probably this is for the other function ( we gonna need two flags, one back and other front, so they can't be here
        # comment to stop the car
        if math.isinf(self.preyPos.pose.position.x) is False:
            self.Hunting = True
            self.goal = self.preyPos  # storing the goal inside the class
            self.goal.header.frame_id = self.name + '/base_link'
        else:
            if math.isinf(self.attackerPos.pose.position.x) is False:
                self.Running = True
                self.Hunting = False
                self.Navigating = False
            else:
                self.Navigating = True
                self.Running = False
                self.Hunting = False

        return image

    # Camera que cria o modo de fuga !
    def discover_car_back(self, frame, camera_matrix):
        # Convert to HSV
        frame = copy.deepcopy(frame)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # create 3 channels !
        gray = cv2.merge((gray, gray, gray))
        mask_attacker = cv2.inRange(frame, self.attacker_color_min, self.attacker_color_max)
        mask_prey = cv2.inRange(frame, self.prey_color_min, self.prey_color_max)
        mask_teammate = cv2.inRange(frame, self.teammate_color_min, self.teammate_color_max)
        Center_t, mask_teammate = self.GetCentroid(mask_teammate, frame)
        Center_p, mask_prey = self.GetCentroid(mask_prey, frame)
        Center_a, mask_attacker = self.GetCentroid(mask_attacker, frame)

        # creates the image
        mask_final = mask_attacker + mask_prey + mask_teammate
        image = cv2.bitwise_or(frame, frame, mask=mask_final)
        image = cv2.add(gray, image)

        # get the transform the lidar values to pixel, draws it in the image
        pixel_cloud = self.lidar_to_image(camera_matrix, image, self.points_back)
        # probably here it receives the self.attackerPos , self.preyPos and self.teammatePos in case they exist
        # it is based by the closest point of the camera centroid
        self.preyPos_back = self.ClosestPoint(Center_p,  pixel_cloud, self.points_back)
        self.attackerPos_back = self.ClosestPoint(Center_a, pixel_cloud, self.points_back)
        self.teammatePos_back = self.ClosestPoint(Center_t, pixel_cloud, self.points_back)

        if math.isinf(self.preyPos_back.pose.position.x) is False:
            self.sendMarker(self.preyPos_back, self.prey_color_max, "prey")
        if math.isinf(self.attackerPos_back.pose.position.x) is False:
            self.sendMarker(self.attackerPos_back, self.attacker_color_max, "attacker")
        if math.isinf(self.teammatePos_back.pose.position.x) is False:
            self.sendMarker(self.teammatePos_back, self.teammate_color_max, "teammate")

        # probably this is for the other function ( we gonna need two flags, one back and other front, so they can't be here
        # comment to stop the car
        if math.isinf(self.preyPos.pose.position.x) is True:
            if math.isinf(self.attackerPos_back.pose.position.x) is False:
                self.Running = True
                self.Hunting = False
                self.Navigating = False

        return image

    def ClosestPoint(self, Center,  pixel_cloud, points):
        Flag_lidar_points = PoseStamped()
        Close_lidar_point = PoseStamped()
        Close_lidar_point.pose.position.x = math.inf
        Close_lidar_point.pose.position.y = math.inf
        Flag_lidar_points.pose.position.x = math.inf
        Flag_lidar_points.pose.position.y = math.inf
        dist = []
        flag = 1000
        if Center[0] is None:
            Close_lidar_point.pose.position.x = math.inf
            Close_lidar_point.pose.position.y = math.inf
            return Close_lidar_point
        else:
            for idx, pixel in enumerate(pixel_cloud):
                # como os valores de trás estao a ir para a frente e vice versa, o melhor é so ver pelo x
                dist.append(math.sqrt((Center[0]-pixel[0])**2 + (Center[1]-pixel[1])**2))
                # dist.append(math.sqrt((Center[0] - pixel[0]) ** 2))
                # if dist[idx] < 200:
                if dist[idx] < flag:
                    Flag_lidar_points.pose.position.x = points[idx][0]
                    Flag_lidar_points.pose.position.y = points[idx][1]
                    flag = dist[idx]

            # point to the base of the robot
            # manter-se em inf caso não exista valores proximos
            Close_lidar_point.pose.position.x = Flag_lidar_points.pose.position.x
            Close_lidar_point.pose.position.y = Flag_lidar_points.pose.position.y
            return Close_lidar_point

    def sendMarker(self, coord, color, text):
        marker = Marker()
        marker.id = self.id
        marker.header.frame_id = "red1/base_link"
        marker.type = marker.CYLINDER
        marker.action = marker.ADD
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.r = color[2]/255
        marker.color.g = color[1]/255
        marker.color.b = color[0]/255
        marker.color.a = 0.3
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = coord.pose.position.x
        marker.pose.position.y = coord.pose.position.y
        marker.pose.position.z = 0.2

        marker2 = Marker()
        marker2.header.frame_id = "red1/base_link"
        marker2.type = marker2.TEXT_VIEW_FACING
        marker2.action = marker2.ADD
        marker2.scale.x = 0.1
        marker2.scale.y = 0.1
        marker2.scale.z = 0.1

        marker2.color.a = 1.0
        marker2.color.r = 1.0
        marker2.color.g = 1.0
        marker2.color.b = 1.0
        marker2.pose.orientation.w = 0.0
        marker2.pose.position.x = coord.pose.position.x
        marker2.pose.position.y = coord.pose.position.y
        marker2.pose.position.z = 0.6
        marker2.id = self.id + 1
        marker2.text = text + ' ' + str(self.id)

        self.publish_marker.publish(marker)
        self.publish_marker.publish(marker2)
        self.id = self.id + 2
        if self.id > 12:
            marker.DELETEALL
            marker2.DELETEALL
            self.id = 0

    def sendMarker_main(self, text):
        marker3 = Marker()
        marker3.header.frame_id = "red1/base_link"
        marker3.type = marker3.TEXT_VIEW_FACING
        marker3.action = marker3.ADD
        marker3.scale.x = 0.1
        marker3.scale.y = 0.1
        marker3.scale.z = 0.1

        marker3.color.a = 1.0
        marker3.color.r = 1.0
        marker3.color.g = 1.0
        marker3.color.b = 1.0
        marker3.pose.orientation.w = 0.0
        marker3.pose.position.x = 0
        marker3.pose.position.y = 0
        marker3.pose.position.z = 0.6
        marker3.id = 13
        marker3.text = text
        self.publish_marker.publish(marker3)

    def lidar_to_image(self, camera_matrix, image, points):
        """
        :param camera_matrix: attacker points from the camera
        :return:
        """
        # so testar os valores front por agora
        pixel_cloud =[]
        pixels_final = []
        for value in points:
            value_array = np.array(value)
            pixel = np.dot(camera_matrix, value_array.transpose())
            pixel = np.dot(self.cameraIntrinsic, pixel)
            pixel_cloud.append(pixel)

        for value in pixel_cloud:
            if math.isnan(value[0]) is False:
                world_pixels = [value[0] / value[2], value[1] / value[2], value[2]]
                if ((world_pixels[0] < 1280) & (world_pixels[0] > 0)) & ((world_pixels[1] < 720) & (world_pixels[1] > 0)):
                    cv2.circle(image, (int(world_pixels[0]), int(world_pixels[1])), 5, (100, 160, 200), -1)
                pixels_final.append(world_pixels)
                # with this we have the pixel points of the lidar, now we need to use this list, check the closest point
                # from the centroid (depending which centroid is, or if there is one)
            else:
                # to maintain the same index of the lidar points
                pixels_final.append([-1000, -1000, -1000])

        return pixels_final

    def Laser_Points(self, msg):
        """
        :param msg: scan data received from the car
        :return:
        """
        # creates a list of world coordinates
        self.points = []
        self.points_back = []
        move = Twist()
        z = 0
        for idx, range in enumerate(msg.ranges):
            theta = msg.angle_min + idx * msg.angle_increment
            x = range * math.cos(theta)
            y = range * math.sin(theta)
            if (x > 0) & (y > 0) | (x > 0) & (y < 0):
                self.points.append([x, y, z, 1])
            elif (x < 0) & (y < 0) | (x > 0) & (y < 0) :
                self.points_back.append([x, y, z, 1])

        # we need to detect the obstacles ( meaning that the points we see arent from the prey nor the attacker, so
        # they are considered obstacles
        thr1 = 1.0 # Laser scan range threshold
        thr2 = 1.0
        if self.Navigating is True:
            if msg.ranges[0] > thr1 and msg.ranges[15] > thr2 and msg.ranges[345] > thr2:
                move.linear.x = random.uniform(0.2, 0.5) # go forward (linear velocity)
                move.angular.z = 0.0  # do not rotate (angular velocity)
            else:
                move.linear.x = 0.0  # stop
                move.angular.z = 1.5 # rotate counter-clockwise
                if msg.ranges[0] > thr1 and msg.ranges[15] > thr2 and msg.ranges[345] > thr2:
                    move.linear.x = random.uniform(0.2, 0.5)
                    move.angular.z = 0.0
            self.publisher_goal.publish(move)  # publish the move object


    def GetCentroid(self, mask, image):
        # Morph close and invert image
        # kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        # close = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        close = mask
        # voltar a encontrar a centroid ( nao desta maneira) e so dar return a maior (mas precisamos de ambas para o rviz)
        contours, hierarchy = cv2.findContours(close, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            # if it finds objects, it will sort them from biggest to smallest
            sorted_contours = sorted(contours, key=cv2.contourArea, reverse=True)
            # the biggest will be the first on the list
            biggest_object = sorted_contours[0]

            # list of all x coordinates for biggest_object edges
            center_x_raw=[coord[0][0] for coord in biggest_object]
            # list of all y coordinates for biggest_object edges
            center_y_raw=[coord[0][1] for coord in biggest_object]

            # avg x value, used as centroid x coord
            cX = int(sum(center_x_raw)/len(center_x_raw))
            # avg y value, used as centroid y coord
            cY = int(sum(center_y_raw)/len(center_y_raw))
            cv2.circle(image, (cX, cY), 7, (255, 255, 255), -1)
            Center = (cX, cY)
            # print(Center)
            return Center, close
        else:
            # sends nothing
            Center = (None, None)
            return Center, close


def main():
    rospy.init_node('p_spombinho_driver', anonymous=False)
    driver = Driver()
    rospy.spin()


if __name__ == '__main__':
  main()