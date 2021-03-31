#!/usr/bin/env python3

# ROS and Moveit
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose

# Other
import time
import sys, os
sys.path.append(os.path.abspath(os.path.dirname(__file__)))

# Custom
from colors import print_col
from panda_arm_moveit import PandaArmMoveit
from panda_gripper_moveit import PandaGripperMoveit
import data_manager as dm



class PandaInterfaceMoveit(PandaArmMoveit, PandaGripperMoveit):
    def __init__(self, \
            delay = 1.0, arm_velocity_factor = 0.1, \
            startup_homing=False, real_robot = False):
        """Note: if delay is too small, Moveit may not load properly"""
        # init arm and gripper
        PandaArmMoveit.__init__(self, delay=0, velocity_factor=arm_velocity_factor)
        PandaGripperMoveit.__init__(self, delay=0, startup_homing=startup_homing, real_robot=real_robot)

        # create scene handler
        self.scene = moveit_commander.PlanningSceneInterface()
        time.sleep(delay)
    

    # SCENE-------------------------------------------------------------
    def _createObject(self, obj):
        obj_pose = PoseStamped()
        obj_pose.header.frame_id = "world"
        obj_pose.pose.position.x = obj['position']['x']
        obj_pose.pose.position.y = obj['position']['y']
        obj_pose.pose.position.z = obj['position']['z']
        obj_pose.pose.orientation.x = obj['orientation']['x']
        obj_pose.pose.orientation.y = obj['orientation']['y']
        obj_pose.pose.orientation.z = obj['orientation']['z']
        obj_pose.pose.orientation.w = obj['orientation']['w']
  
        if obj['type']  == 'mesh':
            obj_scale = obj['scale']
            self.scene.add_mesh(obj['name'], obj_pose, dm.getPathMesh(obj['file']), size=(obj_scale['x'], obj_scale['y'], obj_scale['x']))
        else:
            obj_dim = obj['dimensions']
            if obj['type']  == 'box':
                self.scene.add_box(obj['name'], obj_pose, size=(obj_dim['x'], obj_dim['y'], obj_dim['x']))
            elif obj['type']  == 'sphere':
                self.scene.add_sphere(obj['name'], obj_pose, radius=obj_dim['r'])
            elif obj['type']  == 'cylinder':
                self.scene.add_cylinder(obj['name'], obj_pose, height=obj_dim['h'], radius=obj_dim['r'])
            elif obj['type']  == 'cone':
                self.scene.add_box(obj['name'], obj_pose, size=(obj_dim['x'], obj_dim['y'], obj_dim['x']))

    def loadScene(self, scene_name):
        scene = dm.load_scene(scene_name)
        if scene:
            try:
                for obj in scene:
                    self._createObject(obj)
                return True
            except KeyError:
                print_col("[loadScene] Error: badly formed scenes file", 'FG_RED')   
        return False
    
    def resetScene(self):
        self.scene.remove_world_object()


    # POSE DATA---------------------------------------------------------
    def savePose(self, pose, name):
        dm.save_pose(pose, name)


    def gotoPose(self, name):
        target = dm.load_pose(name)
        if target == None:
            return False
        return self.moveArmPoseTCP(target)


    # CUSTOM------------------------------------------------------------
    def getPose(self):
        """[px, py, pz, ox, oy, oz, ow, fd] fd is the distance between the two fingers"""
        return self.getArmPoseTCP() + [self.getGripperWidth()]


    def movePose(self, goal_pose, gripper_option=None, wait_execution=True):
        """ [px, py, pz, ox, oy, oz, ow, fd, gr]
            fd is the distance between the two fingers
            gr is 1 if the grasp is active, otherwise 0
            gripper_option are the option for gripper (speed,force,epsilon_inner,epsilon_outer"""
        if len(goal_pose) != 9:
            return False
        if self.moveArmPoseTCP(goal_pose[:7], wait_execution=wait_execution):
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
        input("Press enter to continue...")
        panda.loadScene("pick_place")
        input("Press enter to continue...")
        panda.resetScene()



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
      