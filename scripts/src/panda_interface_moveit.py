#!/usr/bin/env python3

# Moveit
import moveit_commander

# Custom
from panda_arm_moveit import PandaArmMoveit
from panda_gripper_moveit import PandaGripperMoveit



class PandaInterfaceMoveit(PandaArmMoveit, PandaGripperMoveit):
    def __init__(self, \
            delay = 0, arm_velocity_factor = 0.1, wait_execution=True, \
            startup_homing=False, real_robot = False):
        PandaArmMoveit.__init__(self, delay=delay/2.0, velocity_factor=arm_velocity_factor, wait_execution=wait_execution)
        PandaGripperMoveit.__init__(self, delay=delay/2.0, startup_homing=startup_homing, real_robot=real_robot)


    def getPose(self):
        """[px, py, pz, ox, oy, oz, ow, fd] fd is the distance between the two fingers"""
        return self.getArmPoseTCP() + [self.getGripperWidth()]


    def movePose(self, goal_pose, gripper_option=None):
        """ [px, py, pz, ox, oy, oz, ow, fd, gr]
            fd is the distance between the two fingers
            gr is 1 if the grasp is active, otherwise 0
            gripper_option are the option for gripper (speed,force,epsilon_inner,epsilon_outer"""
        if len(goal_pose) != 9:
            return False
        if self.moveArmPoseTCP(goal_pose[:7]):
            if goal_pose[8] == 1:
                if gripper_option == None:
                    self.graspGripper(width=goal_pose[7])
                else:
                    self.graspGripper( \
                        width=goal_pose[7], \
                        speed=gripper_option['speed'], \
                        force=gripper_option['force'], \
                        epsilon_inner=gripper_option['epsilon_inner'], \
                        epsilon_outer=gripper_option['epsilon_outer'])     
            else:
                if gripper_option == None:
                    self.moveGripper(width=goal_pose[7])
                else:
                    self.moveGripper( \
                        width=goal_pose[7], \
                        speed=gripper_option['speed'])
            return True
        return False



def test_interface(panda):
    if isinstance(panda, PandaInterfaceMoveit):
        print("Move to Ready")
        print("> Success: ", panda.moveArmReady())
        goal = [0.4, 0.0, 0.4,  0, 0, 0, 1,  0.03, 0]
        print("Move to: ", goal)
        print("> Success: ", panda.movePose(goal))
        print(panda.getPose())



if __name__ == '__main__':
    """TEST"""
    import sys
    import rospy
    # Initialize moveit_commander and rospy
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('panda_moveit_interface_test_node', anonymous=True)

    try:
        # Inizialize movegroupinterface
        panda = PandaInterfaceMoveit(delay=1, real_robot=False)

        test_interface(panda)

    except rospy.ROSInterruptException:
        print("ROS interrupted")
      