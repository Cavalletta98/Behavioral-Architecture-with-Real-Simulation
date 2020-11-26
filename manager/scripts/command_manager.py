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
from geometry_msgs.msg import Point
from std_msgs.msg import String
from robot_control.srv import TargetPos

## 2D home position
home_pos = Point(rospy.get_param("home_pos_x"),rospy.get_param("home_pos_y"),0)

## 2D person position
person_pos = Point(rospy.get_param("person_pos_x"),rospy.get_param("person_pos_y"),0)

## x coordinate of the map
map_x = rospy.get_param("map_x")
## y coordinate of the map
map_y = rospy.get_param("map_y")

## Min delay for transition between PLAY and NORMAL states
min_transition_play_normal = rospy.get_param("min_transition_play_normal")

## Max delay for transition between PLAY and NORMAL states
max_transition_play_normal = rospy.get_param("max_transition_play_normal")

## Min delay for transition between NORMAL and SLEEP states
min_transition_normal_sleep = rospy.get_param("min_transition_normal_sleep")

## Max delay for transition between NORMAL and SLEEP states
max_transition_normal_sleep = rospy.get_param("max_transition_normal_sleep")

## Min delay for SLEEP state
min_sleep_delay = rospy.get_param("min_sleep_delay")

## Max delay for SLEEP state
max_sleep_delay = rospy.get_param("max_sleep_delay")

# define state Play
class play(smach.State):

    """
        A class used to represent the PLAY behaviour
        of the robot

        Attributes
        --------
        @param transition_value: define the value at wich it is performed the state transition
        @type transition_value: int

        @param transition: transition value to decide whenever we have to change state
        @type transition: int

        @param count: value to count how many times we perform the PLAY state before make a transition
        @type count: int

        Methods
        -----
        target_pos_client(x, y):
            Makes a request to motion server using the target position (x,y) 
            and wait for the response
        getGesture(data)
            Callback method that receives the pointed gesture and perform the PLAY 
            behaviour
        execute()
            It publishes the person position and, after the robot reaches the position, it
            subsribes to "gesture" topic. At the end, it waits for the state transition
    """

    def __init__(self):
        # initialisation function, it should not wait

        """
            Constrcutor. It inizializes the attribute
        """
        smach.State.__init__(self,outcomes=['someTimes'])

        ## Define the value at wich it is performed the state transition
        self.transition_value = random.randint(min_transition_play_normal,max_transition_play_normal)
        
    def target_pos_client(self,x, y):

        """
            Makes a request to motion server using the target position (x,y) 
            and wait for the response

            @param x: x coordinate of the target position
            @type x: int
            @param y: y coordinate of the target position
            @type y: int

        """

        rospy.wait_for_service('target_pos')
        try:
            target_pos = rospy.ServiceProxy('target_pos', TargetPos)
            resp = target_pos(x, y)
            return resp
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s",e)
    
    def getGesture(self,data):

        """
            Callback method that receives the pointed gesture and perform the PLAY 
            behaviour

            @param data: 2D pointed gesture
            @type data: Point
        """

        if self.transition == 0:
            self.target_pos_client(data.x,data.y)
            rospy.loginfo("Robot arrived in (%d,%d)",data.x,data.y)
            self.target_pos_client(person_pos.x,person_pos.y)
            rospy.loginfo("Robot arrived in person position (%d,%d)",person_pos.x,person_pos.y)
            self.count += 1
            if self.count == self.transition_value:
                self.transition = 1

        
    def execute(self, userdata):

        """
            It publishes the person position and, after the robot reaches the position, it
            subsribes to "gesture" topic. At the end, it waits for the state transition

            @param userdata: used to pass data between states
            @type userdata: list
        """

        # function called when exiting from the node, it can be blacking

        ## Transition value to decide whenever we have to change state
        self.transition = 0

        ## Value to count how many times we perform the PLAY state before make a transition
        self.count = 0
        self.transition_value = random.randint(min_transition_play_normal,max_transition_play_normal)
        rospy.loginfo('Executing state PLAY')

        self.target_pos_client(person_pos.x,person_pos.y)

        rospy.loginfo("Robot arrived in person position (%d,%d)",person_pos.x,person_pos.y)

        sub_gesture = rospy.Subscriber("gesture", Point, self.getGesture)

        while self.transition == 0:
            pass
        sub_gesture.unregister()
        return 'someTimes'

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

        rospy.wait_for_service('target_pos')
        try:
            target_pos = rospy.ServiceProxy('target_pos', TargetPos)
            resp = target_pos(x, y)
            return resp
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s",e)
        
    def execute(self, userdata):

        """
            It publishes the home position and, after the robot reaches the position, it
            changes the state to NORMAL

            @param userdata: used to pass data between states
            @type userdata: list
        """

        # function called when exiting from the node, it can be blacking
        rospy.loginfo('Executing state SLEEP')

        self.target_pos_client(home_pos.x,home_pos.y)
        
        rospy.loginfo("Robot arrived in home (%d,%d)",home_pos.x,home_pos.y)
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

        smach.State.__init__(self,outcomes=['play','someTimes'])
        rospy.Subscriber("command", String, self.getCommand)
   
    def target_pos_client(self,x, y):

        """
            Makes a request to motion server using the target position (x,y) 
            and wait for the response

            @param x: x coordinate of the target position
            @type x: int
            @param y: y coordinate of the target position
            @type y: int

        """

        rospy.wait_for_service('target_pos')
        try:
            target_pos = rospy.ServiceProxy('target_pos', TargetPos)
            resp = target_pos(x, y)
            return resp
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s",e)

    def getCommand(self,data):

        """
            Callback method that received the "play" command and set the attribute
            play to 1

            @param data: command message
            @type data: str
        """

        self.play = 1

    def execute(self, userdata):

        """
            It publishes a random position and, after the robot reaches the position, it
            checks if it necessary to change state (PLAY) or not

            @param userdata: used to pass data between states
            @type userdata: list
        """

        ## Variable that will be set to 1 when the command "play" arrives
        self.play = 0

        rospy.loginfo('Executing state NORMAL')

        count_value = random.randint(min_transition_normal_sleep,max_transition_normal_sleep)

        for count in range(0,count_value):
            position = Point()
            position.x = random.randint(0,map_x)
            position.y = random.randint(0,map_y)
            self.target_pos_client(position.x,position.y)

            rospy.loginfo("Robot arrived in (%d,%d)",position.x,position.y)
            if self.play == 1:
                rospy.loginfo("User says PLAY")
                return 'play'

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
                               transitions={'play':'PLAY', 
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