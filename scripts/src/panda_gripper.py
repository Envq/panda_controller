#!/usr/bin/env python3

# ROS and Moveit
from moveit_commander.exception import MoveItCommanderException
import moveit_commander
import actionlib
import rospy

# Action msg
from franka_gripper.msg import HomingAction, MoveAction, GraspAction, StopAction
from franka_gripper.msg import HomingGoal, MoveGoal, GraspGoal, StopGoal, GraspEpsilon

# Other
import sys, os
sys.path.append(os.path.abspath(os.path.dirname(__file__)))



class PandaGripper():
    OPEN  = 0.08  # [m]

    def __init__(self, delay = 0, real_robot = False):
        # Attributes
        self._MIN_WIDTH = 0.0                  # [m] closed
        self._MAX_WIDTH = PandaGripper.OPEN    # [m] opened
        self._MIN_FORCE = 0.01                 # [N]
        self._MAX_FORCE = 50.0                 # [N]
        self._timeout = 0.0                    # [sec] -> inf
        self._current_width = -1               # [m] use homing for override it
        self._real_robot = real_robot

        if self._real_robot:
            # Create action clients
            self._client_homing = actionlib.SimpleActionClient('franka_gripper/homing', HomingAction)
            self._client_move = actionlib.SimpleActionClient('franka_gripper/move', MoveAction)
            self._client_grasp = actionlib.SimpleActionClient('franka_gripper/grasp', GraspAction)
            self._client_stop = actionlib.SimpleActionClient('franka_gripper/stop', StopAction)

            # Wait action servers
            self._client_homing.wait_for_server(rospy.Duration.from_sec(self._timeout))
            self._client_move.wait_for_server(rospy.Duration.from_sec(self._timeout))
            self._client_grasp.wait_for_server(rospy.Duration.from_sec(self._timeout))
            self._client_stop.wait_for_server(rospy.Duration.from_sec(self._timeout))

        else:
            # enable hand
            self.hand = moveit_commander.MoveGroupCommander("hand")

        # wait for correct loading
        rospy.sleep(delay)


    def _normalize(self, val, min, max):
        if val < min:
            print("Adust min")
            return min
        if val > max:
            print("Adjust max")
            return max
        return val 


    # HAND---------------------------------------------------------------------
    def _moveHand(self, width_normalized):
        """[width] is the distance between fingers (min=0.0, max=0.08)"""
        try:
            self.hand.set_joint_value_target([width_normalized/2.0, width_normalized/2.0])
            self.hand.plan()
            self.hand.go(wait=True)
        except MoveItCommanderException:
            return False
        return True
      
    
    def getGripperWidth(self):
        return self._current_width


    def homingGripper(self):
        if self._real_robot:
            # Create goal
            goal = HomingGoal()
            # Send goal
            self._client_homing.send_goal_and_wait(goal, rospy.Duration.from_sec(self._timeout))
        else:
            self._moveHand(PandaGripper.OPEN)
        self._current_width = PandaGripper.OPEN
      

    def moveGripper(self, width, speed=0.5):
        width_normalized = self._normalize(width, self._MIN_WIDTH, self._MAX_WIDTH)
        if self._real_robot:
            # Create goal
            goal = MoveGoal()
            goal.width = self.safe_width(width_normalized)
            goal.speed = speed
            # Send goal
            self._client_move.send_goal_and_wait(goal, rospy.Duration.from_sec(self._timeout))
        else:
            self._moveHand(width_normalized)
        self._current_width = width_normalized
      

    def graspGripper(self, width, speed=0.5, force=10, epsilon_inner=0.02, epsilon_outer=0.02):
        width_normalized = self._normalize(width, self._MIN_WIDTH, self._MAX_WIDTH)
        force_normalized = self._normalize(force, self._MIN_FORCE, self._MAX_FORCE)
        if self._real_robot:
            # Create goal
            goal = GraspGoal()
            goal.width = width_normalized
            goal.epsilon = GraspEpsilon(epsilon_inner, epsilon_outer)
            goal.speed = speed
            goal.force = force_normalized
            # Send goal
            self._client_grasp.send_goal_and_wait(goal, rospy.Duration.from_sec(self._timeout))
        else:
            self._moveHand(width_normalized)
            print("GRASPING...")
        self._current_width = width_normalized
      
      
    def stopGripper(self):
        if self._real_robot:
            self.gripper.stop()
        else:
            print("hand.stopGripper() NOT IMPLEMENTED YET")



def test_gripper(gripper):

    print("getGripperWidth(): {}\n".format(gripper.getGripperWidth()))

    print("homingGripper()")
    gripper.homingGripper()
    rospy.sleep(1)
    print("Current gripper width: {}\n".format(gripper.getGripperWidth()))

    print("moveGripper(width=10, speed=0.01)")
    gripper.moveGripper(width=10, speed=0.01)
    rospy.sleep(1)
    print("Current gripper width: {}\n".format(gripper.getGripperWidth()))

    print("graspGripper(width=0.02, epsilon_inner=0.02, epsilon_outer=0.02, speed=0.01, force=10)")
    gripper.graspGripper(width=0.02, epsilon_inner=0.02, epsilon_outer=0.02, speed=0.01, force=10)
    rospy.sleep(1)
    print("Current gripper width: {}\n".format(gripper.getGripperWidth()))



if __name__ == '__main__':
    """TEST"""
    import sys
    # Initialize moveit_commander and rospy
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('panda_gripper_moveit_test_node', anonymous=True)

    try:
        # Inizialize movegroupinterface
        gripper = PandaGripper(delay=1, real_robot=False)

        test_gripper(gripper)

    except rospy.ROSInterruptException:
        print("ROS interrupted")
      