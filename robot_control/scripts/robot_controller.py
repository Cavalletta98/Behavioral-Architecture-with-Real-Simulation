#!/usr/bin/env python3

"""
    ROS action server that moves the robot
    toward a target position
"""

# Import of libraries
import rospy
from geometry_msgs.msg import Twist, Point, Pose
from nav_msgs.msg import Odometry
from tf import transformations
import math
import actionlib
import actionlib.msg
import robot_control.msg

## Robot position
position_ = Point()
## Robot pose
pose_ = Pose()
## Robot yaw angle
yaw_ = 0

## Machine state
state_ = 0

## Goal position
desired_position_ = Point()
## Z coordinate of goal position
desired_position_.z = 0

## Precision for yaw angle
yaw_precision_ = math.pi / 9  # +/- 20 degree allowed
## Precision for yaw angle2
yaw_precision_2_ = math.pi / 90  # +/- 2 degree allowed
## Precision for distance
dist_precision_ = 0.1
## PID parameters
kp_a = -3.0  ## In ROS Noetic, it may be necessary to change the sign of this proportional controller
kp_d = 0.2
ub_a = 0.6
lb_a = -0.5
ub_d = 0.6

## ROS publisher object
pub = None

## ROS action server object
act_s = None

def clbk_odom(msg):

    '''
        Callback function to obtain the odometry information
        from the robot

        @param msg: robot state
        @type msg: Odometry
    '''

    global position_
    global pose_
    global yaw_

    # position
    position_ = msg.pose.pose.position
    pose_ = msg.pose.pose

    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]


def change_state(state):

    '''
        Function to change the state of the machine

        @param state: next state of the machine
        @type state: Int
    '''

    global state_
    state_ = state
    print ('State changed to [%s]' % state_)


def normalize_angle(angle):

    '''
        Function for normalizing the angle

        @param angle: error on the yaw
        @type angle: Float

        @returns: normalized error on the yaw
        @rtype: Float
    '''

    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle


def fix_yaw(des_pos):

    '''
        Function to rotate the robot toward the
        target position

        @param des_pos: robot target position
        @type des_pos: Point
    '''

    global yaw_, pub, yaw_precision_2_, state_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = normalize_angle(desired_yaw - yaw_)
    rospy.loginfo(err_yaw)

    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = kp_a*err_yaw
        if twist_msg.angular.z > ub_a:
            twist_msg.angular.z = ub_a
        elif twist_msg.angular.z < lb_a:
            twist_msg.angular.z = lb_a

    pub.publish(twist_msg)

    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_2_:
        print ('Yaw error: [%s]' % err_yaw)
        change_state(1)


def go_straight_ahead(des_pos):

    '''
        Function to move the robot straight ahead

        @param des_pos: robot target position
        @type des_pos: Point
    '''

    global yaw_, pub, yaw_precision_, state_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) +
                        pow(des_pos.x - position_.x, 2))
    err_yaw = normalize_angle(desired_yaw - yaw_)
    rospy.loginfo(err_yaw)

    if err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = 0.3
        if twist_msg.linear.x > ub_d:
            twist_msg.linear.x = ub_d

        twist_msg.angular.z = kp_a*err_yaw
        pub.publish(twist_msg)
    else:
        print ('Position error: [%s]' % err_pos)
        change_state(2)

    # state change conditions
    if math.fabs(err_yaw) > yaw_precision_:
        print ('Yaw error: [%s]' % err_yaw)
        change_state(0)


def done():

    '''
        Function used to stop the robot when it
        reaches the target position
    '''

    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub.publish(twist_msg)


def planning(goal):

    '''
        Implement action server through a state machine that 
        initially rotate the robot toward the goal position 
        and then move it

        @param goal: robot target position
        @type goal: PoseStamped
    '''

    global state_, desired_position_
    global act_s

    desired_position_.x = goal.target_pose.pose.position.x
    desired_position_.y = goal.target_pose.pose.position.y

    state_ = 0
    rate = rospy.Rate(20)
    success = True

    feedback = robot_control.msg.RobotPlanningFeedback()
    result = robot_control.msg.RobotPlanningResult()

    while not rospy.is_shutdown():
        if act_s.is_preempt_requested():
            rospy.loginfo('Goal was preempted')
            act_s.set_preempted()
            success = False
            break
        elif state_ == 0:
            feedback.stat = "Fixing the yaw"
            feedback.position = pose_
            act_s.publish_feedback(feedback)
            fix_yaw(desired_position_)
        elif state_ == 1:
            feedback.stat = "Angle aligned"
            feedback.position = pose_
            act_s.publish_feedback(feedback)
            go_straight_ahead(desired_position_)
        elif state_ == 2:
            feedback.stat = "Target reached!"
            feedback.position = pose_
            act_s.publish_feedback(feedback)
            done()
            break
        else:
            rospy.logerr('Unknown state!')

        rate.sleep()
    if success:
        rospy.loginfo('Goal: Succeeded!')
        result.position = pose_
        act_s.set_succeeded(result)


def main():

    '''
        Main function. Start the node and subscribe to topic "/robot/odom".
        Start the action server
    '''

    global pub, active_, act_s

    rospy.init_node('go_to_point')

    pub = rospy.Publisher('/robot/cmd_vel', Twist, queue_size=1)
    sub_odom = rospy.Subscriber('/robot/odom', Odometry, clbk_odom)

    act_s = actionlib.SimpleActionServer('/robot_reaching_goal', robot_control.msg.RobotPlanningAction, planning, auto_start=False)
    act_s.start()

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    main()