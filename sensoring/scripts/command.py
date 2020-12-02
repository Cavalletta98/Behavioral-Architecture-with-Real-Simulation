#!/usr/bin/env python3

"""ROS node used to generate user command"""

# Import of libraries
import rospy
import random
import actionlib
from simulation.msg import PlanningAction, PlanningActionGoal

## Min delay for command generation
#min_delay_command = rospy.get_param("min_delay_command")
min_delay_command = 1

## Max delay for command generation
#max_delay_command = rospy.get_param("max_delay_command")
max_delay_command = 10

def command_generator():

    """
        Main function that generates user command
        and publishes it on topic "command"
    """

    rospy.init_node('command_node', anonymous=True)

    disappear_ball = random.randint(1,10)
    count_ball = 0

    client = actionlib.SimpleActionClient('/reaching_goal', PlanningAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    goal = PlanningActionGoal()

    while not rospy.is_shutdown():                                                                                                                                                                                                                                                                                                                                                                                              
         
        # Creates a goal to send to the action server.
        goal.goal.target_pose.pose.position.x = random.uniform(-7,7)
        goal.goal.target_pose.pose.position.y = random.uniform(-7,7)
        goal.goal.target_pose.pose.position.z = 0.5

        if (count_ball == disappear_ball):
            goal.goal.target_pose.pose.position.z = -1
            count_ball = 0

        # Sends the goal to the action server.
        client.send_goal(goal.goal)

        # Waits for the server to finish performing the action.
        client.wait_for_result()

        count_ball = count_ball + 1
        rospy.sleep(random.uniform(min_delay_command,max_delay_command))

if __name__ == '__main__':
    try:
        command_generator()
    except rospy.ROSInterruptException:
        pass