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
# broadcaster = TFBroadcaster()
# joint_publisher = JointStatePublisher("joint_states", 0.1)

world.set_gravity([0, 0, -9.8])
robot = Object("hsrb", "robot", "../../resources/" + robot_description.name + ".urdf")
robot_desig = ObjectDesignatorDescription(names=["hsrb"]).resolve()
robot.set_color([0.5, 0.5, 0.9, 1])
kitchen = Object("kitchen", "environment", "kitchen.urdf")


#milk = Object("milk", ObjectType.MILK, "milk.stl", pose=Pose([-2, 2.33, 0.45]), color=[1, 0, 0, 1])
#cereal = Object("cereal", ObjectType.BREAKFAST_CEREAL, "breakfast_cereal.stl", pose=Pose([-2.27, 2.4, 0.45], [0, 0, 1, 1]), color=[0, 1, 0, 1])
#spoon = Object("spoon", ObjectType.SPOON, "spoon.stl", pose=Pose([-3, 2.44, 0.36]), color=[0, 0, 1, 1])
#bowl = Object("bowl", ObjectType.BOWL, "bowl.stl", pose=Pose([-2.58, 2.4, 0.39]), color=[1, 1, 0, 1])
#milk = Object("milk", ObjectType.MILK, "milk.stl", pose=Pose([-2, 2.33, 0.45]), color=[1, 0, 0, 1])

milk = Object("milk", ObjectType.MILK, "milk.stl", pose=Pose([0, -1.66, 0.45]), color=[1, 0, 0, 1])



robot.set_joint_state(robot_description.torso_joint, 0.24)
kitchen_desig = ObjectDesignatorDescription(names=["kitchen"])

'''
spawning_poses = {
    # 'bigknife': Pose([-0.95, 1.2, 1.3], [1, -1, 1, -1]),
    'bigknife': Pose([0.9, 0.6, 0.5], [0, 0, 0, -1]),
    # 'bread': Pose([-0.85, 0.9, 0.90], [0, 0, -1, 1])
    'bread': Pose([-0.85, 0.9, 0.90], [0, 0, -1, -1]),
    'board': Pose([-0.85, 0.9, 0.85], [0, 0, -1, -1]),
    'cocumber': Pose([-0.85, 0.9, 0.87], [0, 0, -1, -1])
}

bigknife = Object("bigknife", "bigknife", "big-knife.stl", spawning_poses["bigknife"])
cocumber = Object("cocumber", "cocumber", "cocumber.stl", spawning_poses["cocumber"])
board = Object("board", "board", "board.stl", spawning_poses["board"])
cocumber.set_color([0, 1, 0.04, 1])
board.set_color([0.4, 0.2, 0.06, 1])
bigknife_BO = BelieveObject(names=["bigknife"])
bread_BO = BelieveObject(names=["bread"])
cocumber_BO = BelieveObject(names=["cocumber"])
'''

milk_BO = BelieveObject(names=["milk"])
cereal_BO = BelieveObject(names=["cereal"])
spoon_BO = BelieveObject(names=["spoon"])
bowl_BO = BelieveObject(names=["bowl"])

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

# giskardpy.achieve_joint_goal({"torso_lift_joint": 0.28})
import random

'''
with real_robot:
    # loop until there is no more objects
    #while
    ParkArmsAction([Arms.LEFT]).resolve().perform()

    print("navigate to table")

    #NavigateAction(target_locations=[Pose([-0.5, 0.58, 0], [0, 0, 1, 1])]).resolve().perform()

    NavigateAction(target_locations=[Pose([-2, 0, 0], [0, 0, 1, 1])]).resolve().perform()

    NavigateAction(target_locations=[Pose([-2.4, 1.6, 0], [0, 0, 1, 1])]).resolve().perform()

    print("look for objects")

    MoveTorsoAction([0.2]).resolve().perform()

    LookAtAction(targets=[Pose([-2, 2.66, 0])]).resolve().perform()

    #print("perceive objects")

    #pickup_pose_milk = CostmapLocation(target=milk_BO.resolve(), reachable_for=robot_desig).resolve()

    print("move to pick_pose")

    #pickup_pose_milk = CostmapLocation(target=milk_BO.resolve(), reachable_for=robot_desig).resolve()

    #ParkArmsAction([Arms.LEFT]).resolve().perform()

    NavigateAction(target_locations=[Pose([-1.9, 1.8, 0], [0, 0, 1, 1])]).resolve().perform()

    print("pickup an object")

    #milk_design = DetectAction(milk_BO).resolve().perform()
    #TransportAction(milk_design, ["left"], [Pose([-2.5, 2.66, 0.8])]).resolve().perform()

    #ParkArmsAction([Arms.LEFT]).resolve().perform()

    #NavigateAction(target_locations=[Pose([-0.4, 1.1, 0]), robot_orientation]).resolve().perform()

    #MoveTorsoAction([0.5]).resolve().perform()

    MoveGripperMotion("open", "left", allow_gripper_collision=True)

    PickUpAction(object_designator_description=milk_BO,
                 arms=["left"],
                 grasps=["front"]).resolve().perform()

    NavigateAction(target_locations=[Pose([-1.5, 1.8, 0])]).resolve().perform()

    NavigateAction(target_locations=[Pose([-1.9, 2.4, 0])]).resolve().perform()
    #print("transport an object")

    #TransportAction(object_designator_description=milk_BO,
                    #arms=["left"],
                    #target_locations=[Pose([-3.224, 2.29, 0.36])])

    print("place object")
    #PlaceAction(object_designator_description=milk_BO,
                #arms=["left"],
                #target_locations=[Pose([-2.4, 2.44, 0.45], [0, 0, 1, 1])]).resolve().perform()
'''
with semi_real_robot:
    #while
    ParkArmsAction([Arms.LEFT]).resolve().perform()

    print("navigate to table")

    NavigateAction(target_locations=[pickup_pose]).resolve().perform()

    print("look for objects")

    MoveTorsoAction([0.3]).resolve().perform()

    LookAtAction(targets=[Pose([0, -1.66, 0.45])]).resolve().perform()

    #if not isPerceived:
        #object_desig_desc = ObjectDesignatorDescription(types=[ObjectType.MILK])
        #robokudo.query(object_desig_desc)

    print("pickup an object")

    MoveGripperMotion("open", "left", allow_gripper_collision=True)

    PickUpAction(object_designator_description=milk_BO,
                 arms=["left"],
                 grasps=["front"]).resolve().perform()

    ParkArmsAction(arms=[Arms.LEFT]).resolve().perform()


    print("navigate to placePose")

    NavigateAction(target_locations=[Pose([0, 0, 0], [0, 0, 0, 1])]).resolve().perform()

    NavigateAction(target_locations=[Pose([-2, 0, 0], [0, 0, 1, 1])]).resolve().perform()

    NavigateAction(target_locations=[Pose([-2.4, 1.8, 0], [0, 0, 1, 1])]).resolve().perform()

    print("place object")

    MoveTorsoAction([0.5]).resolve().perform()

    PlaceAction(object_designator_description=milk_BO,
                arms=["left"],
                target_locations=[Pose([-2.4, 2.5, 0.45], [0, 0, 1, 1])]).resolve().perform()

    MoveGripperMotion("close", "left", allow_gripper_collision=True)

    print("navigate back to shelf for other objects")

    ParkArmsAction(arms=[Arms.LEFT]).resolve().perform()

    NavigateAction(target_locations=[Pose([-2, 0, 0], [0, 0, -1, 1])]).resolve().perform()

    NavigateAction(target_locations=[Pose([0, 0, 0], [0, 0, 0, 1])]).resolve().perform()

    NavigateAction(target_locations=[pickup_pose]).resolve().perform()

'''
    arm = "left"
    print("pickuppose")
    pickup_pose_knife = CostmapLocation(target=bigknife_BO.resolve(), reachable_for=robot_desig).resolve()
    print("nav")
    pickup_arm = pickup_pose_knife.reachable_arms[0]
    pipose = Pose([0.4, 0.6, 0], [0, 0, 0, -1])
    NavigateAction(target_locations=[pipose]).resolve().perform()
    print("pickup")
    PickUpAction(object_designator_description=bigknife_BO,
                 arms=["left"],
                 grasps=["top"]).resolve().perform()

    MoveGripperMotion("open", "left", allow_gripper_collision=True)
'''