#!/usr/bin/env python3

# ROS and Moveit
import rospy
import moveit_commander

# Other
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "src")))

# Custom
from src.panda_interface_moveit import PandaInterfaceMoveit
from src.colors import colorize, print_col
from src.utils import quaternion_from_euler, euler_from_quaternion


# COLORS
COLOR_HELP = 'FG_CYAN'
COLOR_ERROR = 'FG_RED'
COLOR_WARN = 'FG_YELLOW'



def help():
    print_col("-----------------------------------------------", COLOR_HELP)
    print_col("  Commands available:", COLOR_HELP)
    print_col("Note: ',' are automatically ignored\n", COLOR_HELP)

    print_col("j 'j0' 'j1' 'j2' 'j3' 'j4' 'j5' 'j6'", COLOR_HELP)
    print_col("f 'px' 'py' 'pz'  'ox' 'oy' 'oz, 'ow'", COLOR_HELP)
    print_col("t 'px' 'py' 'pz'  'ox' 'oy' 'oz' 'ow'", COLOR_HELP)
    print_col("p 'px' 'py' 'pz'  'ox' 'oy' 'oz' 'ow'  'fd' 'grasp'", COLOR_HELP)

    print_col("get joints", COLOR_HELP)
    print_col("get flange", COLOR_HELP)
    print_col("get tcp", COLOR_HELP)
    print_col("get pose", COLOR_HELP)
    print_col("get gripper", COLOR_HELP)

    print_col("set velocity 'vel'", COLOR_HELP)

    print_col("gripper homing", COLOR_HELP)
    print_col("gripper 'width' 'speed'", COLOR_HELP)
    print_col("grasp 'width' 'speed' 'force' 'epsilon_inner' 'epsilon_outer'", COLOR_HELP)

    print_col("convert 'x' 'y' 'z' 'w'", COLOR_HELP)
    print_col("convert 'roll' 'pitch' 'yaw' [Note: use radiant]", COLOR_HELP)

    print_col("ready", COLOR_HELP)
    print_col("custom", COLOR_HELP)
    print_col("quit", COLOR_HELP)
    print_col("help", COLOR_HELP)
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
        panda = PandaInterfaceMoveit(\
                                delay=1,\
                                arm_velocity_factor=arm_velocity_factor,\
                                startup_homing=False,\
                                real_robot=real_robot)
        
        # Print help
        help()

        # Perform commands
        while True:
            # Read command
            command = input(colorize("> ", 'FG_GREEN'))

            if (command == "quit"):
                break

            elif (command == "help"):
                help()
            
            elif (command == "get joints"):
                print("  {}".format(panda.getArmJoints()))

            elif (command == "get flange"):
                print("  {}".format(panda.getArmPoseFlange()))
            
            elif (command == "get tcp"):
                print("  {}".format(panda.getArmPoseTCP()))

            elif (command == "get pose"):
                print("  {}".format(panda.getPose()))

            elif (command == "get gripper"):
                print("  {}".format(panda.getGripperWidth()))
                        
            elif (command == "ready"):
                 print("  Success? {}".format(panda.moveArmReady()))

            elif (command == "custom"):
                print("Custom Pose: ", custom_pose)
                print("  Success? {}".format(panda.moveArmPoseTCP(custom_pose)))
            
            else:
                cmd = command.split(" ")
                cmd = list(filter(lambda e: e != '', cmd))
                if cmd[0] == 'set' and cmd[1] == 'velocity':
                    if len(cmd) == 3:
                        panda.setMaxVelocityScalingFactor(float(cmd[2]))
                    else:
                        print_col("  Command not valid", COLOR_WARN)

                elif cmd[0] == 'j':
                    goal = list()
                    for i in range(len(cmd) - 1):
                        goal.append(float(cmd[i + 1].replace(',','')))
                    if len(goal) == 7:
                        print("  Success? {}".format(panda.moveArmJoints(goal)))
                    else:
                        print_col("  Command not valid", COLOR_WARN)

                elif cmd[0] == 'f':
                    goal = list()
                    for i in range(len(cmd) - 1):
                        goal.append(float(cmd[i + 1].replace(',','')))
                    if len(goal) == 7:
                        print("  Success? {}".format(panda.moveArmPoseFlange(goal)))
                    else:
                        print_col("  Command not valid", COLOR_WARN)

                elif cmd[0] == 't':
                    goal = list()
                    for i in range(len(cmd) - 1):
                        goal.append(float(cmd[i + 1].replace(',','')))
                    if len(goal) == 7:
                        print("  Success? {}".format(panda.moveArmPoseTCP(goal)))
                    else:
                        print_col("  Command not valid", COLOR_WARN)

                elif cmd[0] == 'p':
                    goal = list()
                    for i in range(len(cmd) - 1):
                        goal.append(float(cmd[i + 1].replace(',','')))
                    if len(goal) == 9:
                        print("  Success? {}".format(panda.movePose(goal)))
                    else:
                        print_col("  Command not valid", COLOR_WARN)
                
                elif cmd[0] == 'convert':
                    val = list()
                    for i in range(len(cmd) - 1):
                        val.append(float(cmd[i + 1].replace(',','')))
                    if len(val) == 4:
                        print("  Euler -> {}".format(euler_from_quaternion(val[0], val[1], val[2], val[3])))
                    elif len(val) == 3:
                        print("  Quaternion -> {}".format(quaternion_from_euler(val[0], val[1], val[2])))
                    else:
                        print_col("  Command not valid", COLOR_WARN)
                
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
                        print_col("  Command not valid", COLOR_WARN)
                
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
                        print_col("  Command not valid", COLOR_WARN)

                else:
                    print_col("  Command not found", COLOR_WARN)

    except rospy.ROSInterruptException:
        print_col("ROS interrupted", COLOR_ERROR)



if __name__ == "__main__":
    main()