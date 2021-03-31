#!/usr/bin/env python3

# ROS and Moveit
import rospy
import moveit_commander

# Other
import sys

# Custom
from src.panda_interface_moveit import PandaInterfaceMoveit
from src.colors import colorize, print_col
from src.utils import quaternion_from_euler, euler_from_quaternion


# COLORS
COLOR_CMD = 'FG_GREEN'
COLOR_HELP = 'FG_CYAN'
COLOR_ERROR = 'FG_RED'
COLOR_WARN = 'FG_YELLOW'
COLOR_SUCCESS = 'FG_BLACK_BRIGHT'



def help():
    print_col("-----------------------------------------------", COLOR_HELP)
    print_col("Commands available:", COLOR_HELP)
    print_col("',' are automatically ignored\n", COLOR_HELP)

    print_col("[Move commands:]", COLOR_HELP)
    print_col("(j=joint; f=flange; t=tcp)", COLOR_HELP)
    print_col("   j 'j0' 'j1' 'j2' 'j3' 'j4' 'j5' 'j6'", COLOR_HELP)
    print_col("   f 'px' 'py' 'pz'  'ox' 'oy' 'oz, 'ow'", COLOR_HELP)
    print_col("   t 'px' 'py' 'pz'  'ox' 'oy' 'oz' 'ow'", COLOR_HELP)
    print_col("(r=relative(radian); rp=relative_position; ro=relative_euler(degree))", COLOR_HELP)
    print_col("   r  'px' 'py' 'pz'  'roll' 'pitch' 'yaw'", COLOR_HELP)
    print_col("   rp 'px' 'py' 'pz'", COLOR_HELP)
    print_col("   ro 'roll' 'pitch' 'yaw'", COLOR_HELP)
    print_col("(ready -> ready_pose; custom -> launch file)", COLOR_HELP)
    print_col("   ready", COLOR_HELP)
    print_col("   custom", COLOR_HELP)

    print_col("[Get pose of:]", COLOR_HELP)
    print_col("   joints", COLOR_HELP)
    print_col("   flange", COLOR_HELP)
    print_col("   tcp", COLOR_HELP)
    print_col("   gripper", COLOR_HELP)

    print_col("[Gripper commands:]", COLOR_HELP)
    print_col("   gripper homing", COLOR_HELP)
    print_col("   gripper 'width' 'speed'", COLOR_HELP)
    print_col("   grasp   'width' 'speed' 'force' 'epsilon_inner' 'epsilon_outer'", COLOR_HELP)

    print_col("[convert: quaternion <-> euler in radian]", COLOR_HELP)
    print_col("   convert 'x' 'y' 'z' 'w'", COLOR_HELP)
    print_col("   convert 'roll' 'pitch' 'yaw'", COLOR_HELP)

    print_col("[data:]", COLOR_HELP)
    print_col("   scene 'scene_name'", COLOR_HELP)
    print_col("   scene reset", COLOR_HELP)
    print_col("   save 'tcp_pose_name'", COLOR_HELP)
    print_col("   goto 'tcp_pose_name'", COLOR_HELP)

    print_col("[other:]", COLOR_HELP)
    print_col("   velocity 'vel'", COLOR_HELP)
    print_col("   quit", COLOR_HELP)
    print_col("   help", COLOR_HELP)
    print_col("-----------------------------------------------", COLOR_HELP)


def main():
    try:
        # Initialize moveit_commander and rospy
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('console_node', anonymous=True)

        # Get real_robot and arm_velocity_factor
        real_robot = rospy.get_param('~real_robot', False)
        arm_velocity_factor = rospy.get_param('~arm_velocity_factor', 0.1)

        # Get custom pose from param
        custom_pose = list()
        custom_pose.append(rospy.get_param('~px', 0.4))
        custom_pose.append(rospy.get_param('~py', 0.0))
        custom_pose.append(rospy.get_param('~pz', 0.4))
        custom_pose.append(rospy.get_param('~ox', 0.0))
        custom_pose.append(rospy.get_param('~oy', 0.0))
        custom_pose.append(rospy.get_param('~oz', 0.0))
        custom_pose.append(rospy.get_param('~ow', 1.0))


        # Create panda moveit interface
        panda = PandaInterfaceMoveit(   delay=1,\
                                        arm_velocity_factor=arm_velocity_factor,\
                                        startup_homing=False,\
                                        real_robot=real_robot)
        
        # Print help
        help()

        # Perform commands
        while True:
            # Read command
            command = input(colorize("> ", COLOR_CMD))

            if (command == "quit"):
                break

            elif (command == "help"):
                help()
            
            elif (command == "joints"):
                print("   {}".format(panda.getArmJoints()))

            elif (command == "flange"):
                print("   {}".format(panda.getArmPoseFlange()))
            
            elif (command == "tcp"):
                print("   {}".format(panda.getArmPoseTCP()))

            elif (command == "gripper"):
                print("   {}".format(panda.getGripperWidth()))
                        
            elif (command == "ready"):
                 print_col("   Success? {}".format(panda.moveArmReady()), COLOR_SUCCESS)

            elif (command == "custom"):
                print("Custom Pose: ", custom_pose)
                print_col("   Success? {}".format(panda.moveArmPoseTCP(custom_pose)), COLOR_SUCCESS)
            
            else:
                cmd = command.split(" ")
                cmd = list(filter(lambda e: e != '', cmd))
                if cmd[0] == 'velocity':
                    if len(cmd) == 2:
                        panda.setMaxVelocityScalingFactor(float(cmd[1]))
                    else:
                        print_col("   Command not valid", COLOR_WARN)

                elif cmd[0] == 'save':
                    if len(cmd) == 2:
                        panda.savePose(panda.getArmPoseTCP(), cmd[1])
                    else:
                        print_col("   Command not valid", COLOR_WARN)

                elif cmd[0] == 'goto':
                    if len(cmd) == 2:
                        print_col("   Success? {}".format(panda.gotoPose(cmd[1])), COLOR_SUCCESS)
                    else:
                        print_col("   Command not valid", COLOR_WARN)

                elif cmd[0] == 'scene':
                    if len(cmd) == 2:
                        if cmd[1] == 'reset':
                            panda.resetScene()
                        else:
                            print_col("   Success? {}".format(panda.loadScene(cmd[1])), COLOR_SUCCESS)
                    else:
                        print_col("   Command not valid", COLOR_WARN)

                elif cmd[0] == 'j':
                    goal = list()
                    for i in range(len(cmd) - 1):
                        goal.append(float(cmd[i + 1].replace(',','')))
                    if len(goal) == 7:
                        print_col("   Success? {}".format(panda.moveArmJoints(goal)), COLOR_SUCCESS)
                    else:
                        print_col("   Command not valid", COLOR_WARN)

                elif cmd[0] == 'f':
                    goal = list()
                    for i in range(len(cmd) - 1):
                        goal.append(float(cmd[i + 1].replace(',','')))
                    if len(goal) == 7:
                        print_col("   Success? {}".format(panda.moveArmPoseFlange(goal)), COLOR_SUCCESS)
                    else:
                        print_col("   Command not valid", COLOR_WARN)

                elif cmd[0] == 't':
                    goal = list()
                    for i in range(len(cmd) - 1):
                        goal.append(float(cmd[i + 1].replace(',','')))
                    if len(goal) == 7:
                        print_col("   Success? {}".format(panda.moveArmPoseTCP(goal)), COLOR_SUCCESS)
                    else:
                        print_col("   Command not valid", COLOR_WARN)

                elif cmd[0] == 'r':
                    offset = list()
                    for i in range(len(cmd) - 1):
                        offset.append(float(cmd[i + 1].replace(',','')))
                    if len(offset) == 6:
                        print_col("   Success? {}".format(panda.moveRelative(*offset)), COLOR_SUCCESS)
                    else:
                        print_col("   Command not valid", COLOR_WARN)

                elif cmd[0] == 'rp':
                    offset = list()
                    for i in range(len(cmd) - 1):
                        offset.append(float(cmd[i + 1].replace(',','')))
                    if len(offset) == 3:
                        print_col("   Success? {}".format(panda.moveRelativePosition(*offset)), COLOR_SUCCESS)
                    else:
                        print_col("   Command not valid", COLOR_WARN)

                elif cmd[0] == 'ro':
                    offset = list()
                    for i in range(len(cmd) - 1):
                        offset.append(float(cmd[i + 1].replace(',','')))
                    if len(offset) == 3:
                        print_col("   Success? {}".format(panda.moveRelativeEulerDeg(*offset)), COLOR_SUCCESS)
                    else:
                        print_col("   Command not valid", COLOR_WARN)
                
                elif cmd[0] == 'convert':
                    val = list()
                    for i in range(len(cmd) - 1):
                        val.append(float(cmd[i + 1].replace(',','')))
                    if len(val) == 4:
                        print("   Euler -> {}".format(euler_from_quaternion(val[0], val[1], val[2], val[3])))
                    elif len(val) == 3:
                        print("   Quaternion -> {}".format(quaternion_from_euler(val[0], val[1], val[2])))
                    else:
                        print_col("   Command not valid", COLOR_WARN)
                
                elif cmd[0] == 'gripper':
                    val = list()
                    for i in range(len(cmd) - 1):
                        val.append(cmd[i + 1].replace(',',''))
                    if len(val) == 1:
                        if val[0] == 'homing':
                            panda.homingGripper()
                        else:
                            panda.moveGripper(float(val[0]))
                    elif len(val) == 2:
                        panda.moveGripper(float(val[0]), float(val[1]))
                    else:
                        print_col("   Command not valid", COLOR_WARN)
                
                elif cmd[0] == 'grasp':
                    val = list()
                    for i in range(len(cmd) - 1):
                        val.append(float(cmd[i + 1].replace(',','')))
                    if len(val) == 1:
                        panda.graspGripper(val[0])
                    elif len(val) == 2:
                        panda.graspGripper(val[0], val[1])
                    elif len(val) == 3:
                        panda.graspGripper(val[0], val[1], val[2])
                    elif len(val) == 5:
                        panda.graspGripper(val[0], val[1], val[2], val[3], val[4])
                    else:
                        print_col("   Command not valid", COLOR_WARN)

                else:
                    print_col("   Command not found", COLOR_WARN)

    except rospy.ROSInterruptException:
        print_col("ROS interrupted", COLOR_ERROR)



if __name__ == "__main__":
    main()