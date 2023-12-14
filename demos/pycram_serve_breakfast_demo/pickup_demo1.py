from pycram.process_module import simulated_robot, with_simulated_robot, real_robot, with_real_robot, semi_real_robot
from pycram.designators.action_designator import *
from pycram.enums import Arms
from pycram.designators.object_designator import *
from pycram.designators.object_designator import BelieveObject
import pycram.helper as helper
import pycram.external_interfaces.giskard as giskardpy
import pycram.external_interfaces.robokudo
from pycram.fluent_language_misc import failure_handling
import threading
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.plan_failures import TorsoFailure
from pycram.language import macros, par
from pycram.enums import ObjectType
import sys
# from pycram.ros.tf_broadcaster import TFBroadcaster

from pycram.ros.robot_state_updater import RobotStateUpdater

# from pycram.ros.joint_state_publisher import JointStatePublisher

import numpy as np

print(sys.meta_path)
world = BulletWorld("DIRECT")
v = VizMarkerPublisher()

world.set_gravity([0, 0, -9.8])
robot = Object("hsrb", "robot", "../../resources/" + robot_description.name + ".urdf")
robot_desig = ObjectDesignatorDescription(names=["hsrb"]).resolve()
robot.set_color([0.5, 0.5, 0.9, 1])
kitchen = Object("kitchen", "environment", "kitchen.urdf")

milk = Object("milk", ObjectType.MILK, "milk.stl", pose=Pose([-2.3, 2.33, 0.45]), color=[1, 0, 0, 1])
cereal = Object("cereal", ObjectType.BREAKFAST_CEREAL, "breakfast_cereal.stl",
                pose=Pose([-2.5, 2.33, 0.45], [0, 0, 1, 1]), color=[0, 1, 0, 1])
metalMug = Object("metalmug", ObjectType.METALMUG, "spoon.stl", pose=Pose([-2.7, 2.33, 0.36]), color=[0, 0, 1, 1])
bowl = Object("bowl", ObjectType.BOWL, "bowl.stl", pose=Pose([-2.9, 2.33, 0.39]), color=[1, 1, 0, 1])
milk2 = Object("milk2", ObjectType.MILK, "milk.stl", pose=Pose([-3.1, 2.33, 0.45]), color=[1, 0, 0, 1])

robot.set_joint_state(robot_description.torso_joint, 0.24)
kitchen_desig = ObjectDesignatorDescription(names=["kitchen"])

milk_BO = BelieveObject(names=["milk"])
milk_BO2 = BelieveObject(names=["milk2"])
cereal_BO = BelieveObject(types=["cereal"])
metalMug_BO = BelieveObject(names=["metalmug"])
bowl_BO = BelieveObject(names=["bowl"])

# objects_list = [milk_BO, milk_BO2, cereal_BO, metalMug_BO, bowl_BO]

robo_orientation = axis_angle_to_quaternion([0, 0, 1], 90)
x = robo_orientation[0]
y = robo_orientation[1]
z = robo_orientation[2]
w = robo_orientation[3]

isPerceived = False

pickup_pose = Pose([0, -1, 0], [0, 0, -1, 1])

giskardpy.init_giskard_interface()
giskardpy.sync_worlds()

RobotStateUpdater("/tf", "/joint_states")

'''
@with_real_robot
def move_and_detect(obj_type, y1, y2):
    NavigateAction(target_locations=[Pose([1.7, y1, 0])]).resolve().perform()

    LookAtAction(targets=[pickup_pose]).resolve().perform()

    object_desig = DetectAction(BelieveObject(types=[obj_type])).resolve().perform()

    return object_desig
'''
with semi_real_robot:
    # while len(objects_list) != 0:
    ParkArmsAction([Arms.LEFT]).resolve().perform()

    print("navigate to table")
    NavigateAction(target_locations=[Pose([-2.6, 0, 0], [0, 0, 1, 1])]).resolve().perform()
    NavigateAction(target_locations=[Pose([-2.6, 1.5, 0], [0, 0, 1, 1])]).resolve().perform()

    # NavigateAction(target_locations=[Pose([-2, 0, 0], [0, 0, 1, 1])]).resolve().perform()
    # NavigateAction(target_locations=[Pose([-2.1, 1.5, 0], [0, 0, 1, 1])]).resolve().perform()

    print("look for objects")
    LookAtAction(targets=[Pose([-2.6, 2.0, 0.5])]).resolve().perform()
    object_desig = DetectAction(milk_BO, technique='all').resolve().perform()
    objects_list = [object_desig["milk"], object_desig["milk2"], object_desig["cereal"],
                    object_desig["metalmug"], object_desig["bowl"]]
    #objects_list = [object_desig["milk"], object_desig["milk2"], object_desig["cereal"],
                    #object_desig["metalmug"], object_desig["bowl"]]

    # LookAtAction(targets=[Pose([-2.4, 2, 0.78])]).resolve().perform()
    # object_design = DetectAction(cereal_BO).resolve().perform()

    for index in range(len(objects_list)):
        object_pose = objects_list[index].pose
        print(object_pose)
        # navigate hsr to the next object to pick up
        NavigateAction(target_locations=[Pose([object_pose.position.x, 1.5, 0], [0, 0, 1, 1])]).resolve().perform()
        # differentiate the grasping movement between metalmug and other objects
        if objects_list[index].name == "Metalmug" or objects_list[index].name == "bowl":
            PickUpAction(object_designator_description=objects_list[index],
                         arms=["left"],
                         grasps=["top"]).resolve().perform()
            print("grasp from top")
        else:
            PickUpAction(object_designator_description=objects_list[index],
                         arms=["left"],
                         grasps=["front"]).resolve().perform()
            print("grasp from front")

        # inform user before dropping the object
        TalkingMotion("I drop the object now!").resolve().perform()
        print("I drop the object now!")
        # drop/place the object
        place_pose = Pose([object_pose.position.x - 0.3, object_pose.position.y, object_pose.position.z])
        PlaceAction(objects_list[index], [place_pose], ["left"]).resolve().perform()
        print("placing finished")
        # go back for perceiving the next object
        NavigateAction(target_locations=[Pose([-2.6, 1.5, 0], [0, 0, 1, 1])]).resolve().perform()

    ParkArmsAction([Arms.LEFT]).resolve().perform()
    NavigateAction(target_locations=[Pose([-2.6, 0, 0], [0, 0, 1, 1])]).resolve().perform()
    print("finished")

    # print("pickup object")

    # PickUpAction(object_designator_description=object_design,
    # arms=["left"],
    # grasps=["front"]).resolve().perform()

    # MoveGripperMotion("close", "left", allow_gripper_collision=True)
