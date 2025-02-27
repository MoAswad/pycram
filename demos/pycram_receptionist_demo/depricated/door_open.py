import rospy
from geometry_msgs.msg import Vector3

from pycram.designators.action_designator import *
from pycram.designators.motion_designator import *
from pycram.designators.object_designator import *
from pycram.external_interfaces.giskard import grasp_doorhandle, open_doorhandle
from pycram.external_interfaces.knowrob import get_object_info
from pycram.process_module import real_robot
from pycram.ros_utils.robot_state_updater import RobotStateUpdater
from pycram.ros_utils.viz_marker_publisher import VizMarkerPublisher
from pycram.utilities.robocup_utils import ImageSwitchPublisher, GraspListener
from pycram.utils import axis_angle_to_quaternion
from pycram.world_concepts.world_object import Object
from pycram.worlds.bullet_world import BulletWorld

# Initialize the Bullet world for simulation
world = BulletWorld()

# Visualization Marker Publisher for ROS
v = VizMarkerPublisher()

# Create and configure the robot object
robot = Object("hsrb", ObjectType.ROBOT, "../../resources/hsrb.urdf", pose=Pose([0, 0, 0]))
# Update robot state
RobotStateUpdater("/tf", "/giskard_joint_states")

# robot.set_color([0.5, 0.5, 0.9, 1])

# Create environmental objects
apartment = Object("kitchen", ObjectType.ENVIRONMENT, "suturo_lab_2.urdf")

image_switch_publisher = ImageSwitchPublisher()


def demo(step: int):
    with (real_robot):
        # if step <= 12:
        #     grasp_listener = GraspListener()
        #     if grasp_listener.check_grasp():
        #         print("TRUE")
        #     else: print("false")
        #
        #     MoveGripperMotion(GripperState.CLOSE, Arms.LEFT).perform()
        #     rospy.sleep(1)
        #     if grasp_listener.check_grasp():
        #         print("TRUE")
        #     else:
        #         print("false")
        #
        #     MoveGripperMotion(GripperState.OPEN, Arms.LEFT).perform()
        #     rospy.sleep(1)
        #     if grasp_listener.check_grasp():
        #         print("TRUE")
        #     else:
        #         print("false")
        # # TalkingMotion("start demo").perform()

        ParkArmsAction([Arms.LEFT]).resolve().perform()
        MoveJointsMotion(["torso_lift_joint"], [0.1]).perform()
        # MoveJointsMotion(["head_tilt_joint"], [0.0]).perform()
        # MoveGripperMotion(GripperState.OPEN, Arms.LEFT).perform()
        # door_pre = Pose([2.2, -0.9, 0])
        # NavigateAction([door_pre]).resolve().perform()
        #offset = Vector3()
        #offset.z = -0.015
        #offset.y = -0.03
        # grasp_doorhandle("iai_kitchen/iai_kitchen:arena:door_handle_inside", offset)
        # GraspHandleMotion("iai_kitchen/iai_kitchen:arena:door_handle_inside", offset).perform()
        # MoveGripperMotion(GripperState.CLOSE, Arms.LEFT).perform()
        # DoorOpenMotion("iai_kitchen/iai_kitchen:arena:door_handle_inside").perform()
        # open_doorhandle("iai_kitchen/iai_kitchen:arena:door_handle_inside")
        giskard.door_open_ft(handle_name="iai_kitchen/iai_kitchen:arena:door_handle_inside",
                             tip='hand_gripper_tool_frame',
                             handle_turn_limit=0.5  ,
                             hinge_turn_limit=-0.8,
                             handle_length=0.1,
                             ref_speed=0.3,
                             pre_grasp_distance=-0.1,
                             grasp_into_distance=0.15,
                             handle_retract_distance=0.075,
                             offset_leftright=-0.015)
        MoveGripperMotion(GripperState.OPEN, Arms.LEFT).perform()
        ParkArmsAction([Arms.LEFT]).resolve().perform()


demo(12)
