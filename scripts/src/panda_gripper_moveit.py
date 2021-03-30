#!/usr/bin/env python3

# ROS and Moveit
from moveit_commander.exception import MoveItCommanderException
import moveit_commander

# Other
import time
import sys, os
sys.path.append(os.path.abspath(os.path.dirname(__file__)))

# Custom
from panda_gripper_action import PandaGripperAction, normalize



class PandaGripperMoveit():    
    def __init__(self, delay = 0, startup_homing=False, real_robot = False):
        # gripper settings
        self.real_robot = real_robot
        self.current_width = -1  # use homing for override it
        self.open_value = PandaGripperAction.OPEN_VALUE
        self.close_value = PandaGripperAction.CLOSE_VALUE

        if self.real_robot:
            # enable real hand
            self.gripper = PandaGripperAction(startup_homing=startup_homing)
        else:
            # enable fake hand
            self.hand = moveit_commander.MoveGroupCommander("hand")

        # wait for correct loading
        time.sleep(delay) 


    # GRIPPER AND HAND---------------------------------------------------------
    def _moveHand(self, width):
        """[width] is the distance between fingers (min=0.0, max=0.08)"""
        width = normalize(width, self.close_value, self.open_value)
        try:
            self.hand.set_joint_value_target([width/2.0, width/2.0])
            self.hand.plan()
            self.hand.go(wait=True)
        except MoveItCommanderException:
            return False
        return True
      
    
    def getGripperWidth(self):
        return self.current_width


    def homingGripper(self):
        if self.real_robot:
            self.gripper.homing()
        else:
            self._moveHand(self.open_value)
        self.current_width = self.open_value
      

    def moveGripper(self, width, speed=0.5):
        if self.real_robot:
            self.gripper.move(width, speed)
        else:
            self._moveHand(width)
        self.current_width = width
      

    def graspGripper(self, width, speed=0.5, force=10, epsilon_inner=0.02, epsilon_outer=0.02):
        if self.real_robot:
            self.gripper.grasp(width, epsilon_inner, epsilon_outer, speed, force)
        else:
            self._moveHand(width)
            print("GRASPING...")
        self.current_width = width
      
      
    def stopGripper(self):
        if self.real_robot:
            self.gripper.stop()
        else:
            print("ERROR NOT IMPLEMENTED YET")



def test_gripper(gripper):

    print("getGripperWidth(): {}\n".format(gripper.getGripperWidth()))

    print("homingGripper()")
    gripper.homingGripper()
    rospy.sleep(1)
    print("Current gripper width: {}\n".format(gripper.getGripperWidth()))

    print("moveGripper(width=0.05, speed=0.01)")
    gripper.moveGripper(width=0.05, speed=0.01)
    rospy.sleep(1)
    print("Current gripper width: {}\n".format(gripper.getGripperWidth()))

    print("graspGripper(width=0.02, epsilon_inner=0.02, epsilon_outer=0.02, speed=0.01, force=10)")
    gripper.graspGripper(width=0.02, epsilon_inner=0.02, epsilon_outer=0.02, speed=0.01, force=10)
    rospy.sleep(1)
    print("Current gripper width: {}\n".format(gripper.getGripperWidth()))



if __name__ == '__main__':
    """TEST"""
    import sys
    import rospy
    # Initialize moveit_commander and rospy
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('panda_gripper_moveit_test_node', anonymous=True)

    try:
        # Inizialize movegroupinterface
        gripper = PandaGripperMoveit(delay=1, startup_homing=False, real_robot=False)

        test_gripper(gripper)

    except rospy.ROSInterruptException:
        print("ROS interrupted")
      