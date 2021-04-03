#!/usr/bin/env python3

# ROS and Moveit
from moveit_commander.exception import MoveItCommanderException
import moveit_commander
from geometry_msgs.msg import Pose

# Other
from math import pi
import time
import sys, os
sys.path.append(os.path.abspath(os.path.dirname(__file__)))

# Custom
from utils import quaternion_from_euler, transform, transform_inverse



class PandaArm():
    def __init__(self, delay = 0, velocity_factor = 0.1):
        # arm settings
        self.arm = moveit_commander.MoveGroupCommander("panda_arm")

        # transformations
        self.flange_to_tcp = [0.0, 0.0, 0.1035, 0.923879533, -0.382683432, 0.0, 0.0]
        self.tcp_to_flange = transform_inverse(self.flange_to_tcp).tolist()

        # set velocity scaling factor
        self.setMaxVelocityScalingFactor(velocity_factor)
        
        # wait for correct loading
        time.sleep(delay)

    def setMaxVelocityScalingFactor(self, velocity):
        self.arm.set_max_velocity_scaling_factor(velocity)


    def setWaitExecution(self, wait):
        self.wait_execution = wait


    # JOINTS------------------------------------------------------------
    def getArmJoints(self):
        """[j0, j1, j2, j3, j4, j5, j6]"""
        return self.arm.get_current_joint_values()


    def moveArmJoints(self, goal_joints, wait_execution=True):
        """[j0, j1, j2, j3, j4, j5, j6]"""
        if len(goal_joints) != 7:
            return False
        try:
            self.arm.set_joint_value_target(goal_joints)
            self.arm.plan()
            self.arm.go(wait=wait_execution)
        except MoveItCommanderException:
            return False
        return True
    

    def moveArmReady(self, wait_execution=True):
        return self.moveArmJoints((0.00, -0.25 * pi, 0.00, -0.75 * pi, 0.00, 0.50 * pi, 0.25 * pi), wait_execution)


    # FLANGE POSE--------------------------------------------------------
    def getArmPoseFlange(self):
        """[px, py, pz, ox, oy, oz, ow]"""
        pose = self.arm.get_current_pose().pose
        flange = list()
        flange.append(pose.position.x)
        flange.append(pose.position.y)
        flange.append(pose.position.z)
        flange.append(pose.orientation.x)
        flange.append(pose.orientation.y)
        flange.append(pose.orientation.z)
        flange.append(pose.orientation.w)
        return flange


    def moveArmPoseFlange(self, goal_pose, wait_execution=True):
        """[px, py, pz, ox, oy, oz, ow]"""
        if len(goal_pose) != 7:
            return False
        target = Pose()
        target.position.x = goal_pose[0]
        target.position.y = goal_pose[1]
        target.position.z = goal_pose[2]
        target.orientation.x = goal_pose[3]
        target.orientation.y = goal_pose[4]
        target.orientation.z = goal_pose[5]
        target.orientation.w = goal_pose[6]
        try:
            self.arm.set_pose_target(target)
            self.arm.plan()
            self.arm.go(wait=wait_execution)
        except MoveItCommanderException:
            return False
        return True

 
    # TCP POSE----------------------------------------------------------
    def getFlangeFromTCP(self, world_to_tcp):
        """Get the world-to-flange (panda_link8) pose"""
        if len(world_to_tcp) != 7:
            return -1
        return transform(world_to_tcp, self.tcp_to_flange).tolist()


    def getArmPoseTCP(self):
        """[px, py, pz, ox, oy, oz, ow] get the world-to-tcp (tool center point) pose"""
        world_to_flange = self.getArmPoseFlange()
        return transform(world_to_flange, self.flange_to_tcp).tolist()

  
    def moveArmPoseTCP(self, goal_pose, wait_execution=True):
        """[px, py, pz, ox, oy, oz, ow]"""
        if len(goal_pose) != 7:
            return False
        flange_goal_pose = self.getFlangeFromTCP(goal_pose)
        return self.moveArmPoseFlange(flange_goal_pose, wait_execution)


    def moveRelative(self, x, y, z, roll, pitch, yaw):
        """euler in radian"""
        offset = list()
        offset.append(x)
        offset.append(y)
        offset.append(z)
        offset += quaternion_from_euler(roll, pitch, yaw).tolist()
        target = transform(self.getArmPoseTCP(), offset)
        return self.moveArmPoseTCP(target.tolist())

    def moveRelativePosition(self, x, y, z):
        return self.moveRelative(x, y, z, 0, 0, 0)


    def moveRelativeEulerRad(self, roll, pitch, yaw):
        return self.moveRelative(0, 0, 0, roll, pitch, yaw)


    def moveRelativeEulerDeg(self, roll, pitch, yaw):
        return self.moveRelativeEulerRad(roll*pi/180.0, pitch*pi/180.0, yaw*pi/180.0)
   

    def execute_tcp_cartesian_path(self, waypoints, eef_step = 0.01, jump_threashould = 0.0, wait_execution=True):
        """Compute and execute a sequence of waypoints that make the end-effector move in straight line segments that follow the poses specified as waypoints. Configurations are computed for every eef_step meters; The jump_threshold specifies the maximum distance in configuration space between consecutive points in the resultingpath;"""
        # Convert waypoint list in waypoint Pose()
        waypoints_pose = list()
        for point in waypoints:
            point = self.getFlangeFromTCP(point)    # transform into world_to_flange
            waypoint = Pose()
            waypoint.position.x = point[0]
            waypoint.position.y = point[1]
            waypoint.position.z = point[2]
            waypoint.orientation.x = point[3]
            waypoint.orientation.y = point[4]
            waypoint.orientation.z = point[5]
            waypoint.orientation.w = point[6]
            waypoints_pose.append(waypoint)
        # Generate planning
        (plan, fraction) = self.arm.compute_cartesian_path(
                                            waypoints_pose,   # waypoints to follow
                                            eef_step,         # interpolation
                                            jump_threashould) # jump_threshold -> with 0 not check invalid jumps in joint space
        if fraction != 1.0:
            return False, fraction
        # Execute planning
        self.arm.execute(plan, wait=wait_execution)
        return True, fraction



def test_tf(arm):
    if isinstance(arm, PandaArm):
        import tf2_ros
        from geometry_msgs.msg import TransformStamped
        from utils import quaternion_equals 

        # BROADCAST BASE-TO-TCP TRANSFORMATION 
        # broadcaster = tf2_ros.StaticTransformBroadcaster()
        # static_transformStamped = TransformStamped()
        # static_transformStamped.header.stamp = rospy.Time.now()
        # static_transformStamped.header.frame_id = "panda_link8"
        # static_transformStamped.child_frame_id = "tcp"
        # static_transformStamped.transform.translation.x = 0.0
        # static_transformStamped.transform.translation.y = 0.0
        # static_transformStamped.transform.translation.z = 0.1035
        # static_transformStamped.transform.rotation.x = 0.923879533
        # static_transformStamped.transform.rotation.y = -0.382683432
        # static_transformStamped.transform.rotation.z = 0.0
        # static_transformStamped.transform.rotation.w = 0.0
        # broadcaster.sendTransform(static_transformStamped)

        # CREATE TF2 LISTENER
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        time.sleep(2)


        # TESTO0
        # generate current pose with TF2
        world_to_tcp_listened = tf_buffer.lookup_transform("world", "tcp", rospy.Time())
        world_to_tcp_tf2 = list()
        world_to_tcp_tf2.append(world_to_tcp_listened.transform.translation.x)
        world_to_tcp_tf2.append(world_to_tcp_listened.transform.translation.y)
        world_to_tcp_tf2.append(world_to_tcp_listened.transform.translation.z)
        world_to_tcp_tf2.append(world_to_tcp_listened.transform.rotation.x)
        world_to_tcp_tf2.append(world_to_tcp_listened.transform.rotation.y)
        world_to_tcp_tf2.append(world_to_tcp_listened.transform.rotation.z)
        world_to_tcp_tf2.append(world_to_tcp_listened.transform.rotation.w)
        print("current pose with TF2:")
        print(world_to_tcp_tf2)


        # TEST1
        # generate transformations with TF2
        print("WITH TF2: ")
        flange_to_tcp_listened = tf_buffer.lookup_transform("panda_link8", "tcp", rospy.Time())
        flange_to_tcp_tf2 = list()
        flange_to_tcp_tf2.append(flange_to_tcp_listened.transform.translation.x)
        flange_to_tcp_tf2.append(flange_to_tcp_listened.transform.translation.y)
        flange_to_tcp_tf2.append(flange_to_tcp_listened.transform.translation.z)
        flange_to_tcp_tf2.append(flange_to_tcp_listened.transform.rotation.x)
        flange_to_tcp_tf2.append(flange_to_tcp_listened.transform.rotation.y)
        flange_to_tcp_tf2.append(flange_to_tcp_listened.transform.rotation.z)
        flange_to_tcp_tf2.append(flange_to_tcp_listened.transform.rotation.w)
        print("flange_to_tcp:     {}".format(flange_to_tcp_tf2))

        tcp_to_flange_listened = tf_buffer.lookup_transform("tcp", "panda_link8", rospy.Time())
        tcp_to_flange_tf2 = list()
        tcp_to_flange_tf2.append(tcp_to_flange_listened.transform.translation.x)
        tcp_to_flange_tf2.append(tcp_to_flange_listened.transform.translation.y)
        tcp_to_flange_tf2.append(tcp_to_flange_listened.transform.translation.z)
        tcp_to_flange_tf2.append(tcp_to_flange_listened.transform.rotation.x)
        tcp_to_flange_tf2.append(tcp_to_flange_listened.transform.rotation.y)
        tcp_to_flange_tf2.append(tcp_to_flange_listened.transform.rotation.z)
        tcp_to_flange_tf2.append(tcp_to_flange_listened.transform.rotation.w)
        print("tcp_to_flange:     {}\n".format(tcp_to_flange_tf2))

        # generete transformations with my UTILS file
        print("WITH UTILS")
        flange_to_tcp_utils_fake= [0.0, 0.0, 0.1035, 0.923879533, -0.382683432, 0.0, 0.0]
        print("flange_to_tcp:     {}".format(flange_to_tcp_utils_fake))
        
        tcp_to_flange_utils  = transform_inverse(flange_to_tcp_utils_fake)
        print("tcp_to_flange:     {}".format(tcp_to_flange_utils.tolist()))
        
        flange_to_tcp_utils  = transform_inverse(tcp_to_flange_utils)
        print("flange_to_tcp_utils: {}\n".format(flange_to_tcp_utils.tolist()))

        # checks tf2 vs utils
        print("Check TF2 with UTILS: ({}, {})".format( \
            quaternion_equals(flange_to_tcp_tf2[3:], flange_to_tcp_utils[3:]), \
            quaternion_equals(tcp_to_flange_tf2[3:], tcp_to_flange_utils[3:])))
        
        print("Check UTILS: fake vs inverse(inverse(fake)): {}\n".format( \
            quaternion_equals(flange_to_tcp_utils_fake[3:], flange_to_tcp_utils[3:])))

        # checks consistence with PandaArm
        print("Check ARM. with UTILS: ({}, {})".format( \
            quaternion_equals(arm.flange_to_tcp[3:], flange_to_tcp_utils[3:]), \
            quaternion_equals(arm.tcp_to_flange[3:], tcp_to_flange_utils[3:])))


        # TEST2
        print("----------------------------")
        world_to_tcp_fake = [0.3, 0, 0.5,  0, 0, 0, 1]
        print("world_to_tcp:      {}".format(world_to_tcp_fake))

        world_to_flange_utils = transform(world_to_tcp_fake, tcp_to_flange_utils)
        print("world_to_flange_utils: {}".format(world_to_flange_utils.tolist()))

        world_to_flange_arm_method = arm.getFlangeFromTCP(world_to_tcp_fake)
        print("world_to_flange from getFlangeFromTCP(): {}".format(world_to_flange_arm_method))

        print("Check Utils with getFlangeFromTCP() {}\n".format( \
            quaternion_equals(world_to_flange_utils[3:], world_to_flange_arm_method[3:])))

        world_to_tcp_utils = transform(world_to_flange_utils, flange_to_tcp_utils)
        print("world_to_tcp_my:   {}".format(world_to_tcp_utils.tolist()))
        
        print("Check world_to_tcp_fake with utils: {}\n".format( \
            quaternion_equals(world_to_tcp_fake[3:], world_to_tcp_utils[3:])))


def test_joints(arm):
    if isinstance(arm, PandaArm):
        print("Move to Ready")
        print("> Success: ", arm.moveArmReady())
        goal = [0.00, -0.25 * pi, 0.00, -0.75 * pi, 0.00, 0.50 * pi, -0.25 * pi]
        print("Move joints to: ", goal)
        print("> Success: ", arm.moveArmJoints(goal))
        print(arm.getArmJoints())


def test_flange(arm):
    if isinstance(arm, PandaArm):
        print("Move to Ready")
        print("> Success: ", arm.moveArmReady())
        goal = [0.4, 0.0, 0.4,  0, 0, 0, 1]
        print("Move flange to: ", goal)
        print("> Success: ", arm.moveArmPoseFlange(goal))
        print(arm.getArmPoseFlange())


def test_tcp(arm):
    if isinstance(arm, PandaArm):
        print("Move to Ready")
        print("> Success: ", arm.moveArmReady())
        goal = [0.4, 0.0, 0.4,  0, 0, 0, 1]
        print("Move TCP to: ", goal)
        print("> Success: ", arm.moveArmPoseTCP(goal))
        print(arm.getArmPoseTCP())


def test_waypoints(arm):
    if isinstance(arm, PandaArm):
        print("Move to Ready")
        print("> Success: ", arm.moveArmReady())
        waypoints = list()
        waypoints.append([0.4, 0.0, 0.4,  0, 0, 0, 1])
        waypoints.append([0.4, 0.0, 0.2,  0, 0, 0, 1])
        waypoints.append([0.5, 0.0, 0.2,  0, 0, 0, 1])
        print("Move TCP to: ")
        for point in waypoints:
            print(point)
        success, fraction = arm.execute_tcp_cartesian_path(waypoints, 0.01, 0.0)
        print("> Success: ", success)
        print("> Execution: {}%".format(fraction * 100))


def test_wait(arm):
    if isinstance(arm, PandaArm):
        print("Move to Ready")
        print("> Success: ", arm.moveArmReady(wait_execution=False))
        time.sleep(5)

        goal = [0.4, 0.0, 0.4,  0, 0, 0, 1]
        print("Move TCP to: ", goal)
        print("> Success: ", arm.moveArmPoseTCP(goal, wait_execution=False))
        time.sleep(1)
        print(arm.getArmPoseTCP())

        goal = [0.4, 0.0, 0.3,  0, 0, 0, 1]
        print("Move TCP to: ", goal)
        print("> Success: ", arm.moveArmPoseTCP(goal, wait_execution=False))
        time.sleep(1)
        print(arm.getArmPoseTCP())

        goal = [0.3, 0.0, 0.3,  0, 0, 0, 1]
        print("Move TCP to: ", goal)
        print("> Success: ", arm.moveArmPoseTCP(goal, wait_execution=False))
        time.sleep(1)
        print(arm.getArmPoseTCP())



if __name__ == '__main__':
    """TEST"""
    import sys
    import rospy
    # Initialize moveit_commander and rospy
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('panda_arm_moveit_test_node', anonymous=True)

    try:
        # Inizialize movegroupinterface
        arm = PandaArm(delay=1, velocity_factor=1.0)

        test_tf(arm)
        # test_joints(arm)
        # test_flange(arm)
        # test_tcp(arm)
        # test_waypoints(arm)
        # test_wait(arm)

    except rospy.ROSInterruptException:
        print("ROS interrupted")
      