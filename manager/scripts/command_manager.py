#!/usr/bin/env python3

"""
    ROS node that implement the FSM of 
    robot behaviours
"""

# Import of libraries
import roslib
import rospy
import smach
import smach_ros
import time
import random
import actionlib

from robot_control.msg import RobotPlanningAction, RobotPlanningActionGoal
from sensoring.srv import DetectImage
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from geometry_msgs.msg import Point
from control_msgs.msg import JointControllerState

## Min delay for transition between NORMAL and SLEEP states
min_transition_normal_sleep = rospy.get_param("min_transition_normal_sleep")

## Max delay for transition between NORMAL and SLEEP states
max_transition_normal_sleep = rospy.get_param("max_transition_normal_sleep")

## Min delay for transition between NORMAL and SLEEP states
min_transition_play_normal = rospy.get_param("min_transition_play_normal")

## Max delay for transition between NORMAL and SLEEP states
max_transition_play_normal = rospy.get_param("max_transition_play_normal")

## 2D home position
home_pos = Point(rospy.get_param("home_pos_x"),rospy.get_param("home_pos_y"),0)

## Min delay for SLEEP state
min_sleep_delay = rospy.get_param("min_sleep_delay")

## Max delay for SLEEP state
max_sleep_delay = rospy.get_param("max_sleep_delay")

## Min delay for mantaining 45° clockwise the head
min_clock_head_delay = rospy.get_param("min_clock_head_delay")

## Max delay for mantaining 45° clockwise the head
max_clock_head_delay = rospy.get_param("max_clock_head_delay")

## Min delay for mantaining 45° counterclockwise the head
min_countclock_head_delay = rospy.get_param("min_countclock_head_delay")

## Max delay for mantaining 45° counterclockwise the head
max_countclock_head_delay = rospy.get_param("max_countclock_head_delay")

## Min delay for mantaining 0° the head
min_head_delay = rospy.get_param("min_head_delay")

## Max delay for mantaining 0° the head
max_head_delay = rospy.get_param("max_head_delay")

## x coordinate of the map
map_x = rospy.get_param("map_x")
## y coordinate of the map
map_y = rospy.get_param("map_y")

# define state Play
class play(smach.State):

    """
        A class used to represent the PLAY behaviour
        of the robot

        Attributes
        -----
        @param vel_pub: variable that represents a publisher for the robot velocities
        @type vel_pub: Publisher

        @param head_pub: variable that represents a publisher for controlling the head of the robot
        @type head_pub: Publisher

        @param head_sleep: variable that notifies when it is reached the desired angle for the head joint
                            and it is time to sleep
        @type head_sleep: Int

        Methods
        -----
        object_detector_client():
            Makes a request to detector server and wait for the response
        jointState(data)
            Get the head joint state in order to check if it is reached the desired angle
        execute(userdata)
            It checks if there is the ball.If there is the ball, it moves the robot toward the ball and if the
            robot reaches the ball and it is stopped, it starts to move the head 45° to the left, 45° to the right and back to
            the 0 position. After some times the robot doesen't see the ball, it switches to the NORMAL state
    """


    def __init__(self):
        # initialisation function, it should not wait

        """
            Constrcutor. It inizializes the attribute
        """
        smach.State.__init__(self,outcomes=['someTimes'])
        ## ROS Publisher object for controlling robot velocities
        self.vel_pub = rospy.Publisher('/robot/cmd_vel', Twist, queue_size=1)

        ## ROS Publisher object for controlling the head of the robot
        self.head_pub = rospy.Publisher('/myrobot/head_joint_position_controller/command', Float64, queue_size=1)
        ## Variable that notifies when it is reached the desired angle for the head joint and it is time to sleep
        self.head_sleep = 0

    def jointState(self,data):

        '''
            Get the head joint state in order to check if it is reached the desired angle

            @param data: joint state
            @type data: JointControllerState
        '''
        if(abs(data.process_value - data.set_point) <= 0.017):
            self.head_sleep = 1
            self.joint_state.unregister()

    def object_detector_client(self):

        """
            Makes a request to detector server and wait for the response

            @returns: radius and center of the ball
            @rtyper: String
        """

        rospy.wait_for_service('detect_image')
        try:
            detect_obj_serv = rospy.ServiceProxy('detect_image', DetectImage)
            resp = detect_obj_serv()
            return resp
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s",e)
    
   
    def execute(self, userdata):

        '''
            It checks if there is the ball.If there is the ball, it moves the robot toward the ball and if the
            robot reaches the ball and it is stopped, it starts to move the head 45° to the left, 45° to the right and back to
            the 0 position. After some times the robot doesen't see the ball, it switches to the NORMAL state

            @param userdata: used to pass data bejoint_state = tween states
            @type userdata: list

            @returns: transition value
            @rtype: String
        '''

        rospy.loginfo('Executing state PLAY')
        
        count = 0
        transition_value = random.randint(min_transition_play_normal,max_transition_play_normal)
        
        while (1):
            resp = self.object_detector_client()
            ball = resp.object.split()
            center = float(ball[0])
            radius = float(ball[1])
            
            if ((center == -1) and (radius == -1)):
                vel = Twist()
                vel.angular.z = 0.5
                self.vel_pub.publish(vel)
                count += 1
                if(count == transition_value):
                    return 'someTimes'
            elif(radius > 10):
                count = 0
                vel = Twist()
                vel.angular.z = 0.003*(center-400)
                vel.linear.x = -0.007*(radius-110)
                if((radius >= 100) and (abs(vel.linear.x) <= 0.06) and (abs(vel.angular.z) <= 0.1)):
                    vel.angular.z = 0
                    vel.linear.x = 0
                    self.vel_pub.publish(vel)

                    head_angle = Float64()
                    head_angle.data = 0.78
                    self.head_pub.publish(head_angle)
                    ## ROS subscriber for the head joint state
                    self.joint_state = rospy.Subscriber('/myrobot/head_joint_position_controller/state',JointControllerState,self.jointState,queue_size=1)
                    while(self.head_sleep != 1):
                        pass
                    time.sleep(random.uniform(min_sleep_delay,max_sleep_delay))

                    self.head_sleep = 0
                    head_angle.data = -0.78
                    self.head_pub.publish(head_angle)
                    self.joint_state = rospy.Subscriber('/myrobot/head_joint_position_controller/state',JointControllerState,self.jointState,queue_size=1)
                    while(self.head_sleep != 1):
                        pass
                    time.sleep(random.uniform(min_countclock_head_delay,max_countclock_head_delay))
                    
                    self.head_sleep = 0
                    head_angle.data = 0
                    self.head_pub.publish(head_angle)
                    self.joint_state = rospy.Subscriber('/myrobot/head_joint_position_controller/state',JointControllerState,self.jointState,queue_size=1)
                    while(self.head_sleep != 1):
                        pass
                    time.sleep(random.uniform(min_head_delay,max_head_delay))
                    self.head_sleep = 0

                else:
                    self.vel_pub.publish(vel)
            elif(radius < 10):
                count = 0
                vel = Twist()
                vel.linear.x = 0.09
                self.vel_pub.publish(vel)

                           
# define state Sleep
class sleep(smach.State):

    """
        A class used to represent the SLEEP behaviour
        of the robot

        Methods
        -----
        target_pos_client(x, y):
            Send a goal to the action server of the robot and waits until it reaches the goal.
        execute(userdata)
            It send the robot to the home position and, after the robot reaches the position,
            sleeps for a random time of seconds. After that change the state to NORMAL
    """

    def __init__(self):

        """
            Constrcutor
        """

        # initialisation function, it should not wait
        smach.State.__init__(self,outcomes=['wakeUp'])

    def target_pos_client(self,x, y):

        """
            Send a goal to the action server of the robot and waits until it reaches the goal.

            @param x: x coordinate of the target position
            @type x: int
            @param y: y coordinate of the target position
            @type y: int

            @returns: the position reached by the robot
            @rtype: Pose

        """
        client = actionlib.SimpleActionClient('/robot_reaching_goal', RobotPlanningAction)

        # Waits until the action server has started up and started
        # listening for goals.
        client.wait_for_server()


        # Creates a goal to send to the action server.
        goal = RobotPlanningActionGoal()
        goal.goal.target_pose.pose.position.x = x
        goal.goal.target_pose.pose.position.y = y

        # Sends the goal to the action server.
        client.send_goal(goal.goal)

        # Waits for the server to finish performing the action.
        client.wait_for_result()

        # Prints out the result of executing the action
        return client.get_result()
        
    def execute(self, userdata):

        """
            It send the robot to the home position and, after the robot reaches the position,
            sleeps for a random time of seconds. After that change the state to NORMAL

            @param userdata: used to pass data between states
            @type userdata: list

            @returns: transition value
            @rtype: String
        """

        # function called when exiting from the node, it can be blacking
        rospy.loginfo('Executing state SLEEP')


        result = self.target_pos_client(home_pos.x,home_pos.y)
        rospy.loginfo("Robot arrived in home(%ld,%ld)",result.position.position.x,result.position.position.y)
        time.sleep(random.uniform(min_sleep_delay,max_sleep_delay))
        
        return 'wakeUp'
    

# define state Normal
class normal(smach.State):

    """
        A class used to represent the NORMAL behaviour
        of the robot

        Methods
        -----
        object_detector_client():
            Makes a request to detector server and wait for the response
        target_pos_client(x,y)
            Send a goal to the action server of the robot and waits until it reaches the goal.
            While it is waiting, if there is the ball, it stops the robot and return None
        execute(userdata)
            It checks if there is the ball, otherwise it will generate a random goal (x and y)
            for the robot. If there is the ball, it switches to the PLAY state.
            After some times, it switches to the SLEEP state
    """

    def __init__(self):

        """
            Constrcutor
        """

        smach.State.__init__(self,outcomes=['someTimes','ball'])
    
    def object_detector_client(self):

        """
            Makes a request to detector server and wait for the response

            @returns: radius and center of the ball
            @rtyper: string

        """

        rospy.wait_for_service('detect_image')
        try:
            detect_obj_serv = rospy.ServiceProxy('detect_image', DetectImage)
            resp = detect_obj_serv()
            return resp
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s",e)

    def target_pos_client(self,x, y):

        """
            Send a goal to the action server of the robot and waits until it reaches the goal.
            While it is waiting, if there is the ball, it stops the robot and return None

            @param x: x coordinate of the target position
            @type x: int
            @param y: y coordinate of the target position
            @type y: int

            @returns: the position reached by the robot or None
            @rtype: Pose

        """
        client = actionlib.SimpleActionClient('/robot_reaching_goal', RobotPlanningAction)

        # Waits until the action server has started up and started
        # listening for goals.
        client.wait_for_server()

        # Creates a goal to send to the action server.
        goal = RobotPlanningActionGoal()
        goal.goal.target_pose.pose.position.x = x
        goal.goal.target_pose.pose.position.y = y

        # Sends the goal to the action server.
        client.send_goal(goal.goal)

        while(client.get_state() != 3):
            resp = self.object_detector_client()
            ball = resp.object.split()
            center = float(ball[0])
            radius = float(ball[1])
            if ((center != -1) and (radius != -1)):
                client.cancel_all_goals()
                return None

        # Prints out the result of executing the action
        return client.get_result()

    def execute(self, userdata):

        """
            It checks if there is the ball, otherwise it will generate a random goal (x and y)
            for the robot. If there is the ball, it switches to the PLAY state.
            After some times, it switches to the SLEEP state

            @param userdata: used to pass data bejoint_state = tween states
            @type userdata: list

            @returns: transition value
            @rtype: String
        """
        rospy.loginfo('Executing state NORMAL')

        count_value = random.randint(min_transition_normal_sleep,max_transition_normal_sleep)
        neg_map_x = map_x*-1
        neg_map_y = map_y*-1

        for count in range(0,count_value):
            resp = self.object_detector_client()
            ball = resp.object.split()
            center = float(ball[0])
            radius = float(ball[1])
            if ((center != -1) and (radius != -1)):
                return 'ball'    
            x = random.uniform(neg_map_x,map_x)
            y = random.uniform(neg_map_y,map_y)
            result = self.target_pos_client(x,y)
            if (result != None):
                rospy.loginfo("Robot arrived in (%lf,%lf)",result.position.position.x,result.position.position.y)
            else:
                return 'ball'

        return 'someTimes'

        
def main():

    """
        Main function that initializes the node and the FSM.
        After that it starts the node and the FSM
    """

    rospy.init_node('command_manager_state_machine')
     
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['container_interface'])
    sm.userdata.sm_counter = 0

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('NORMAL', normal(), 
                               transitions={'ball':'PLAY', 
                                            'someTimes':'SLEEP'})
        smach.StateMachine.add('SLEEP', sleep(), 
                               transitions={'wakeUp':'NORMAL'})
        smach.StateMachine.add('PLAY', play(), 
                               transitions={'someTimes':'NORMAL'})
        
    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()