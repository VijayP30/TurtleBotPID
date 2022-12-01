#!/usr/bin/env python
import rospy
from marvelmind_nav.msg import hedge_imu_fusion,hedge_pos_ang
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import time
import json

rospy.init_node("turtlebotControl", anonymous = True)

# Initializing Coordinate Variables
currentX = 0
currentY = 0
currentX_47 = 0
currentY_47 = 0
currentX_73 = 0
currentY_73 = 0
referX = 0
referY = 0
currentHeading = 0

distanceTol = 0.2
headingTol = 0.2
pidScale = 1.5 # 4.5
speedNormal = 0.2
speedTurn = 0.2
flag = 1
base_cmdForward = Twist()

# Function to update location of beacons and turtlebot heading
def update_location(data):
    global referX, referY, flag, currentX_47, currentY_47, currentX_73, currentY_73, currentX, currentY, currentHeading
    if (data.address == 47):
        currentX_47 = data.x_m
        currentY_47 = data.y_m
    elif (data.address == 73):
        currentX_73 = data.x_m
        currentY_73 = data.y_m
    currentX = (currentX_47 + currentX_73) / 2
    currentY = (currentY_47 + currentY_73) / 2
    if (flag == 1):
        referX = currentX
        referY = currentY
	flag = 0
    else:
        if ((currentY - referY)*(currentY - referY)+(currentX - referX)*(currentX - referX)) >= 0.05:
	    currentHeading = math.atan2(currentY - referY, currentX - referX)
	    referX = currentX
	    referY = currentY
    # current47_ang = math.atan2(currentY_47, currentX_47)
    # current73_ang = math.atan2(currentY_73, currentX_73)
    # if (current73_ang < 0) or (current47_ang < 0):
        # current73_ang += 2*math.pi
        # current47_ang += 2*math.pi
    # currentHeading = (current47_ang + current73_ang) / 2
    # if (current47_ang > current73_ang):
        # currentHeading += math.pi
    if (currentHeading < 0):
        currentHeading += 2*math.pi
    while (currentHeading > 2*math.pi):
        currentHeading -= 2*math.pi
    
        
# Set up Subcribers and Publishers
turtlebot_cmd_vel_pub = rospy.Publisher("/cmd_vel_mux/input/navi", Twist, queue_size = 1)
position_callback = rospy.Subscriber("/hedge_pos_ang", hedge_pos_ang, update_location)
time.sleep(5)
move_once = False

# Set Goal Point in JSON goal.json file. Change path if necessary
f = open('/home/turtlebot/catkin_ws/src/turtlebot_control_movement_fall_22/src/goal.json')
data = json.load(f)
goal_pos = (data["goal_x"], data["goal_y"])
f.close()

# Controller
while True:# not rospy.shutdown():
    if not move_once:
        end_t = time.time() + 2
        while (time.time() < end_t):
            base_cmdForward.linear.x = speedNormal
            base_cmdForward.linear.y = 0
            base_cmdForward.linear.z = 0
            base_cmdForward.angular.x = 0
            base_cmdForward.angular.y = 0
            base_cmdForward.angular.z = 0
            turtlebot_cmd_vel_pub.publish(base_cmdForward)
        move_once = True
	# print(currentX,currentY)
        while (abs(currentX - goal_pos[0]) + abs(currentY - goal_pos[1]) > distanceTol):
            destiHeading = math.atan2(goal_pos[1] - currentY, goal_pos[0] - currentX)
	    if (destiHeading < 0):
	        destiHeading += 2*math.pi
            print("current: ", currentHeading)
            print("desti: ", destiHeading)
            if (abs(destiHeading - currentHeading) < headingTol):
                base_cmdForward.linear.x = speedNormal
                base_cmdForward.angular.z = 0
                turtlebot_cmd_vel_pub.publish(base_cmdForward)
            else:
                if (destiHeading < 0):
                    destiHeading += 2*math.pi
                if (currentHeading < 0):
                    currentHeading += 2*math.pi
                diff = destiHeading - currentHeading
                if (diff < 0):
                    diff += 2*math.pi
                if (diff > math.pi):
                    diff = 2*math.pi - diff
                    base_cmdForward.linear.x = speedTurn
                    base_cmdForward.angular.z = -(diff/math.pi)*pidScale
                    turtlebot_cmd_vel_pub.publish(base_cmdForward)
		else:
		    base_cmdForward.linear.x = speedTurn
		    base_cmdForward.angular.z = diff*pidScale
		    turtlebot_cmd_vel_pub.publish(base_cmdForward)
            rospy.sleep(0.1)
        base_cmdForward.linear.x = 0
        base_cmdForward.linear.y = 0
        base_cmdForward.linear.z = 0
        base_cmdForward.angular.x = 0
        base_cmdForward.angular.y = 0
        base_cmdForward.angular.z = 0
        turtlebot_cmd_vel_pub.publish(base_cmdForward)
        print("Point Reached!!!")
        print("Final X: ", currentX)
        print("Final Y: ",currentY)
        rospy.spin()
