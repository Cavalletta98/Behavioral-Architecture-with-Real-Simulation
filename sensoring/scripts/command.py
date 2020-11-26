#!/usr/bin/env python3

"""ROS node used to generate user command"""

# Import of libraries
import rospy
from simulation.msg import PlanningActionGoal
import random

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

    pub = rospy.Publisher('reaching_goal/goal', PlanningActionGoal, queue_size=1)
    rospy.init_node('command_node', anonymous=True)

    disappear_ball = random.randint(1,10)
    count_ball = 0
    print(disappear_ball)

    while not rospy.is_shutdown():
        print(count_ball)
        command = PlanningActionGoal()
        command.goal.target_pose.pose.position.x = random.uniform(-10,10)
        command.goal.target_pose.pose.position.y = random.uniform(-10,10)
        command.goal.target_pose.pose.position.z = 0.5

        if (count_ball == disappear_ball):
            command.goal.target_pose.pose.position.z = -1
            count_ball = 0

        rospy.loginfo(command.goal.target_pose.pose.position)
        pub.publish(command)
        count_ball = count_ball + 1
        rospy.sleep(random.uniform(min_delay_command,max_delay_command))

if __name__ == '__main__':
    try:
        command_generator()
    except rospy.ROSInterruptException:
        pass