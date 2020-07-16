#!/usr/bin/env python3

import pybullet as p
import numpy as np
import rospy
import time
import threading
import pybullet_data
from sensor_msgs.msg import JointState


# from pybullet_debuger import pybulletDebug
# from kinematic_model import robotKinematics
# from gaitPlanner import trotGait

def init_simulation():
    global boxId, motor_id_list, compensateReal
    physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
    motor_id_list = [0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14]
    # motor_id_list = [4, 5, 6, 12, 13, 14, 0, 1, 2, 8, 9, 10]
    compensateReal = [-1, -1, -1, 1, -1, -1, -1, 1, 1, 1, 1, 1]
    p.setGravity(0, 0, -9.8)

    cubeStartPos = [0, 0, 0.5]
    p.resetDebugVisualizerCamera(0.2, 45, -30, [1, -1, 1])
    FixedBase = False  # if fixed no plane is imported
    if (FixedBase == False):
        p.loadURDF("plane.urdf")
    boxId = p.loadURDF("/home/wgx/Workspace_Ros/src/cloudrobot/src/quadruped_robot.urdf", cubeStartPos,
                       useFixedBase=FixedBase)
    # boxId = p.loadURDF("mini_cheetah/mini_cheetah.urdf", cubeStartPos,
    #                    useFixedBase=FixedBase)


    jointIds = []
    paramIds = []
    # time.sleep(0.5)
    for j in range(p.getNumJoints(boxId)):
        #    p.changeDynamics(boxId, j, linearDamping=0, angularDamping=0)
        info = p.getJointInfo(boxId, j)
        # print(info)
        jointName = info[1]
        jointType = info[2]
        jointIds.append(j)

    footFR_index = 3
    footFL_index = 7
    footBR_index = 11
    footBL_index = 15
    init_new_pos = [0.02, -0.78, 1.74, -0.02, -0.78, 1.74, 0.02, -0.78, 1.74, -0.02, -0.78, 1.74]
    for j in range(12):
        # p.changeVisualShape(quadruped, j, rgbaColor=[1, 1, 1, 1])
        force = 500
        p.setJointMotorControl2(boxId, motor_id_list[j], p.POSITION_CONTROL, init_new_pos[j])
    p.setRealTimeSimulation(1)


def callback_state(msg):
    get_position = []
    compensate = [1, -1, -1, 1, -1, -1, 1, -1, -1, 1, -1, -1]
    for i in range(12):
        get_position.append(msg.position[i] * compensate[i])
        # get_position.append(msg.position[i])
    p.setJointMotorControlArray(boxId,
                                jointIndices=motor_id_list,
                                controlMode=p.POSITION_CONTROL,
                                targetPositions=get_position,
                                )
    # joint_state = p.getJointStates(boxId, motor_id_list)
    # position = [joint_state[0][0], joint_state[1][0], joint_state[2][0],
    #             joint_state[3][0], joint_state[4][0], joint_state[5][0],
    #             joint_state[6][0], joint_state[7][0], joint_state[8][0],
    #             joint_state[9][0], joint_state[10][0], joint_state[11][0]]
    # print(position)


def thread_job():
    rospy.spin()


def talker():
    pub1 = rospy.Publisher('/get_js', JointState, queue_size=10)
    freq = 200
    rate = rospy.Rate(freq)  # hz
    while not rospy.is_shutdown():
        # p.stepSimulation()
        joint_state = p.getJointState(quadruped, motor_id_list)
        position = [joint_state[0][0], joint_state[1][0], joint_state[2][0],
                    joint_state[3][0], joint_state[4][0], joint_state[5][0],
                    joint_state[6][0], joint_state[7][0], joint_state[8][0],
                    joint_state[9][0], joint_state[10][0], joint_state[11][0]]
        print(position)
        p.setRealTimeSimulation(1)
        rate.sleep()


if __name__ == '__main__':
    init_simulation()
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("set_js", JointState, callback_state)
    # add_thread = threading.Thread(target=thread_job)
    # add_thread.start()
    # talker()
    rospy.spin()