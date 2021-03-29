#! /usr/bin/env python3

# ROS
import rospy
import actionlib

# Action msg
from franka_gripper.msg import HomingAction, MoveAction, GraspAction, StopAction
from franka_gripper.msg import HomingGoal, MoveGoal, GraspGoal, StopGoal, GraspEpsilon



def normalize(val, min, max):
    if val < min:
        print("Adust min")
        return min
    elif val > max:
        print("Adjust max")
        return max
    else:
        return val 


class PandaGripperAction():
    OPEN_VALUE  = 0.08  # [m]
    CLOSE_VALUE = 0.00  # [m]

    def __init__(self, startup_homing = False, timeout = 10.0): 
        # Attributes
        self.MIN_WIDTH = PandaGripperAction.CLOSE_VALUE     # [m] closed
        self.MAX_WIDTH = PandaGripperAction.OPEN_VALUE      # [m] opened
        self.MIN_FORCE = 0.01                               # [N]
        self.MAX_FORCE = 50.0                               # [N]
        self._timeout = timeout                             # [sec]

        # Create action clients
        self._client_homing = actionlib.SimpleActionClient('franka_gripper/homing', HomingAction)
        self._client_move = actionlib.SimpleActionClient('franka_gripper/move', MoveAction)
        self._client_grasp = actionlib.SimpleActionClient('franka_gripper/grasp', GraspAction)
        self._client_stop = actionlib.SimpleActionClient('franka_gripper/stop', StopAction)

        # Wait action servers
        self._client_homing.wait_for_server()
        self._client_move.wait_for_server()
        self._client_grasp.wait_for_server()
        self._client_stop.wait_for_server()

        if startup_homing:
            self.homing()


    def safe_force(self, val):
        return normalize(val, self.MIN_FORCE, self.MAX_FORCE)


    def safe_width(self, val):
        return normalize(val, self.MIN_WIDTH, self.MAX_WIDTH)


    def setTimeout(self):
        self._timeout = 10.0
    

    def homing(self):
        # Create goal
        goal = HomingGoal()
        # Send goal
        self._client_homing.send_goal_and_wait(goal, rospy.Duration.from_sec(self._timeout))
    

    def move(self, width, speed):
        # Create goal
        goal = MoveGoal()
        goal.width = self.safe_width(width)
        goal.speed = speed
        # Send goal
        self._client_move.send_goal_and_wait(goal, rospy.Duration.from_sec(self._timeout))
    

    def grasp(self, width, epsilon_inner, epsilon_outer, speed, force):
        # Create goal
        goal = GraspGoal()
        goal.width = self.safe_width(width)
        goal.epsilon = GraspEpsilon(epsilon_inner, epsilon_outer)
        goal.speed = speed
        goal.force = self.safe_force(force)
        # Send goal
        self._client_grasp.send_goal_and_wait(goal, rospy.Duration.from_sec(self._timeout))
    

    def stop(self):
        # Create goal
        goal = StopGoal()
        # Send goal
        self._client_stop.send_goal_and_wait(goal, rospy.Duration.from_sec(self._timeout))



if __name__ == '__main__':
    rospy.init_node('panda_gripper_action_test_node')

    print("Create PandaGripper")
    gripper = PandaGripperAction(startup_homing=False)

    print("PandaGripper.homing()")
    gripper.homing()
    rospy.sleep(1)

    print("PandaGripper.move()")
    gripper.move(width=0.05, speed=0.01)
    rospy.sleep(1)

    print("PandaGripper.grasp()")
    gripper.grasp(width=0.02, epsilon_inner=0.02, epsilon_outer=0.02, speed=0.01, force=10)