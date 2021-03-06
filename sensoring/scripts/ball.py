#!/usr/bin/env python3

"""ROS node used to move the ball"""

# Import of libraries
import rospy
import random
import actionlib
from simulation.msg import PlanningAction, PlanningActionGoal

## Min delay for moving the ball
min_delay_ball = rospy.get_param("min_delay_ball")

## Max delay for moving the ball
max_delay_ball = rospy.get_param("max_delay_ball")

## Min delay for disappeariung the ball
min_dis_ball_delay = rospy.get_param("min_dis_ball_delay")

## Max delay for disappeariung the ball
max_dis_ball_delay = rospy.get_param("max_dis_ball_delay")

## x coordinate of the map
map_x = rospy.get_param("map_x")
## y coordinate of the map
map_y = rospy.get_param("map_y")

def ball():

    """
        Main function that moves the ball randomly
        into the arena
    """

    rospy.init_node('ball_node', anonymous=True)

    disappear_ball = random.randint(min_dis_ball_delay,max_dis_ball_delay)
    count_ball = 0

    client = actionlib.SimpleActionClient('/reaching_goal', PlanningAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    goal = PlanningActionGoal()

    neg_map_x = map_x*-1
    neg_map_y = map_y*-1

    while not rospy.is_shutdown():                                                                                                                                                                                                                                                                                                                                                                                              
         
        # Creates a goal to send to the action server.
        goal.goal.target_pose.pose.position.x = random.uniform(neg_map_x,map_x)
        goal.goal.target_pose.pose.position.y = random.uniform(neg_map_y,map_y)
        goal.goal.target_pose.pose.position.z = 0.5

        if (count_ball == disappear_ball):
            goal.goal.target_pose.pose.position.z = -1
            count_ball = 0

        # Sends the goal to the action server.
        client.send_goal(goal.goal)

        # Waits for the server to finish performing the action.
        client.wait_for_result()

        count_ball = count_ball + 1
        rospy.sleep(random.uniform(min_delay_ball,max_delay_ball))

if __name__ == '__main__':
    try:
        ball()
    except rospy.ROSInterruptException:
        pass