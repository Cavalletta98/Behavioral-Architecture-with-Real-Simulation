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
from geometry_msgs.msg import Point,PoseStamped
import actionlib
from robot_control.msg import RobotPlanningAction, RobotPlanningActionGoal
from sensoring.srv import DetectImage

## Min delay for transition between NORMAL and SLEEP states
min_transition_normal_sleep = rospy.get_param("min_transition_normal_sleep")

## Max delay for transition between NORMAL and SLEEP states
max_transition_normal_sleep = rospy.get_param("max_transition_normal_sleep")

## 2D home position
home_pos = Point(rospy.get_param("home_pos_x"),rospy.get_param("home_pos_y"),0)

## Min delay for SLEEP state
min_sleep_delay = rospy.get_param("min_sleep_delay")

## Max delay for SLEEP state
max_sleep_delay = rospy.get_param("max_sleep_delay")

## x coordinate of the map
map_x = rospy.get_param("map_x")
## y coordinate of the map
map_y = rospy.get_param("map_y")

# define state Sleep
class sleep(smach.State):

    """
        A class used to represent the SLEEP behaviour
        of the robot

        Methods
        -----
        target_pos_client(x, y):
            Makes a request to motion server using the target position (x,y) 
            and wait for the response
        getFeedback(data)
            Callback method that received the "arrived" message,waits for a random number of
            seconds and set the attribute arrived to 1
        execute()
            It publishes the home position and, after the robot reaches the position, it
            changes the state to NORMAL
    """

    def __init__(self):

        """
            Constrcutor
        """

        # initialisation function, it should not wait
        smach.State.__init__(self,outcomes=['wakeUp'])

    def target_pos_client(self,x, y):

        """
            Makes a request to motion server using the target position (x,y) 
            and wait for the response

            @param x: x coordinate of the target position
            @type x: int
            @param y: y coordinate of the target position
            @type y: int

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
            It publishes the home position and, after the robot reaches the position, it
            changes the state to NORMAL

            @param userdata: used to pass data between states
            @type userdata: list
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

        Attributes
        -----
        @param play: variable that will be set to 1 when the command "play" arrives
        @type play: int

        Methods
        -----
        target_pos_client(x, y):
            Makes a request to motion server using the target position (x,y) 
            and wait for the response
        getFeedback(data)
            Callback method that received the "arrived" message and set the attribute
            arrived to 1
        getCommand(data)
            Callback method that sets the attribute play to 1
        execute()
            It publishes a random position and, after the robot reaches the position, it
            checks if it necessary to change state (PLAY) or not
    """

    def __init__(self):

        """
            Constrcutor. It subscribes to "command" topic
        """

        smach.State.__init__(self,outcomes=['someTimes'])
    
    def object_detector_client(self):

        """
            Makes a request to motion server using the target position (x,y) 
            and wait for the response

            @param x: x coordinate of the target position
            @type x: int
            @param y: y coordinate of the target position
            @type y: int

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
            Makes a request to motion server using the target position (x,y) 
            and wait for the response

            @param x: x coordinate of the target position
            @type x: int
            @param y: y coordinate of the target position
            @type y: int

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
            It publishes a random position and, after the robot reaches the position, it
            checks if it necessary to change state (PLAY) or not

            @param userdata: used to pass data between states
            @type userdata: list
        """
        rospy.loginfo('Executing state NORMAL')

        count_value = random.randint(min_transition_normal_sleep,max_transition_normal_sleep)
        neg_map_x = map_x*-1
        neg_map_y = map_y*-1

        for count in range(0,count_value):
            resp = self.object_detector_client()
            rospy.loginfo(resp)       
            x = random.uniform(neg_map_x,map_x)
            y = random.uniform(neg_map_y,map_y)
            result = self.target_pos_client(x,y)
            rospy.loginfo("Robot arrived in (%lf,%lf)",result.position.position.x,result.position.position.y)

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
                               transitions={'someTimes':'SLEEP'})
        smach.StateMachine.add('SLEEP', sleep(), 
                               transitions={'wakeUp':'NORMAL'})
        #smach.StateMachine.add('PLAY', play(), 
                               #transitions={'someTimes':'NORMAL'})
        
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