import time
import sys
import os

from pndbotics_sdk_py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from pndbotics_sdk_py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from pndbotics_sdk_py.idl.default import adam_u_msg_dds__LowCmd_
from pndbotics_sdk_py.idl.default import adam_u_msg_dds__LowState_
from pndbotics_sdk_py.idl.default import adam_u_msg_dds__HandCmd_
from pndbotics_sdk_py.idl.adam_u.msg.dds_ import LowCmd_
from pndbotics_sdk_py.idl.adam_u.msg.dds_ import LowState_
from pndbotics_sdk_py.idl.adam_u.msg.dds_ import HandCmd_
from pndbotics_sdk_py.utils.thread import RecurrentThread

import numpy as np

ADAM_U_NUM_MOTOR = 19
Kp = [
    60.0,  # waistRoll (0)
    60.0,  # waistPitch (1)
    60.0,  # waistYaw (2)
    9.0,   # neckYaw (3)
    9.0,   # neckPitch (4)
    18.0,  # shoulderPitch_Left (5)
    9.0,   # shoulderRoll_Left (6)
    9.0,   # shoulderYaw_Left (7)
    9.0,   # elbow_Left (8)
    9.0,   # wristYaw_Left (9)
    9.0,   # wristPitch_Left (10)
    9.0,   # wristRoll_Left (11)
    18.0,  # shoulderPitch_Right (12)
    9.0,   # shoulderRoll_Right (13)
    9.0,   # shoulderYaw_Right (14)
    9.0,   # elbow_Right (15)
    9.0,   # wristYaw_Right (16)
    9.0,   # wristPitch_Right (17)
    9.0    # wristRoll_Right (18)
]

# Kd 配置数组（对应19个关节）
Kd = [
    1.0,   # waistRoll (0)
    1.0,   # waistPitch (1)
    1.0,   # waistYaw (2)
    0.9,   # neckYaw (3)
    0.9,   # neckPitch (4)
    0.9,   # shoulderPitch_Left (5)
    0.9,   # shoulderRoll_Left (6)
    0.9,   # shoulderYaw_Left (7)
    0.9,   # elbow_Left (8)
    0.9,   # wristYaw_Left (9)
    0.9,   # wristPitch_Left (10)
    0.9,   # wristRoll_Left (11)
    0.9,   # shoulderPitch_Right (12)
    0.9,   # shoulderRoll_Right (13)
    0.9,   # shoulderYaw_Right (14)
    0.9,   # elbow_Right (15)
    0.9,   # wristYaw_Right (16)
    0.9,   # wristPitch_Right (17)
    0.9    # wristRoll_Right (18)
]
class ADAMUJointIndex:
    waistRoll = 0
    waistPitch = 1
    waistYaw = 2
    neckYaw = 3
    neckPitch = 4
    shoulderPitchLeft = 5
    shoulderRollLeft = 6
    shoulderYawLeft = 7
    elbowLeft = 8
    wristYawLeft = 9
    wristPitchLeft = 10
    wristRollLeft = 11
    shoulderPitchRight = 12
    shoulderRollRight = 13
    shoulderYawRight = 14
    elbowRight = 15
    wristYawRight = 16
    wristPitchRight = 17
    wristRollRight = 18

class Custom:
    def __init__(self):
        self.time_ = 0.0
        self.control_dt_ = 0.002  # [2ms]
        self.duration_ = 3.0    # [3 s]
        self.counter_ = 0
        self.mode_machine_ = 0
        self.low_cmd = adam_u_msg_dds__LowCmd_(ADAM_U_NUM_MOTOR)  
        self.low_state = adam_u_msg_dds__LowState_(ADAM_U_NUM_MOTOR) 
        self.hand_cmd = adam_u_msg_dds__HandCmd_()
        self.close_hand = np.array([500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500], dtype=int)
        self.getstate_flag = False
    def Init(self):
        self.lowcmd_publisher_ = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.lowcmd_publisher_.Init()

        self.hand_pub = ChannelPublisher("rt/handcmd", HandCmd_)
        self.hand_pub.Init()

        # create subscriber # 
        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.LowStateHandler, 10)

    def Start(self):
        self.lowCmdWriteThreadPtr = RecurrentThread(
            interval=self.control_dt_, target=self.LowCmdWrite, name="control"
        )
        self.lowCmdWriteThreadPtr.Start()

    def LowStateHandler(self, msg: LowState_):
        self.getstate_flag = True
        self.low_state = msg
        # print("Received LowState")

    def LowCmdWrite(self):
        if(self.getstate_flag):
            self.time_ += self.control_dt_

            if self.time_ < self.duration_ :
                # [Stage 1]: set robot to zero posture
                for i in range(ADAM_U_NUM_MOTOR):
                    ratio = np.clip(self.time_ / self.duration_, 0.0, 1.0)
                    self.low_cmd.motor_cmd[i].mode =  1 # 1:Enable, 0:Disable
                    self.low_cmd.motor_cmd[i].tau = 0. 
                    self.low_cmd.motor_cmd[i].q = (1.0 - ratio) * self.low_state.motor_state[i].q 
                    self.low_cmd.motor_cmd[i].dq = 0. 
                    self.low_cmd.motor_cmd[i].kp = Kp[i] 
                    self.low_cmd.motor_cmd[i].kd = Kd[i]
                for i in range(12):
                    self.hand_cmd.position[i] = self.close_hand[i]
                self.hand_pub.Write(self.hand_cmd)
                self.lowcmd_publisher_.Write(self.low_cmd)

        else:
            print("Waiting for LowState...")
            # elif self.time_ < self.duration_ * 2 :
            #     # [Stage 2]: swing ankle using PR mode
            #     max_P = np.pi * 30.0 / 180.0
            #     max_R = np.pi * 10.0 / 180.0
            #     t = self.time_ - self.duration_
            #     L_P_des = max_P * np.sin(2.0 * np.pi * t)
            #     L_R_des = max_R * np.sin(2.0 * np.pi * t)
            #     R_P_des = max_P * np.sin(2.0 * np.pi * t)
            #     R_R_des = -max_R * np.sin(2.0 * np.pi * t)

            #     self.low_cmd.motor_cmd[ADAMJointIndex.LeftAnklePitch].q = L_P_des
            #     self.low_cmd.motor_cmd[ADAMJointIndex.LeftAnkleRoll].q = L_R_des
            #     self.low_cmd.motor_cmd[ADAMJointIndex.RightAnklePitch].q = R_P_des
            #     self.low_cmd.motor_cmd[ADAMJointIndex.RightAnkleRoll].q = R_R_des

            # else :
            #     # [Stage 3]: swing ankle using AB mode
            #     max_A = np.pi * 30.0 / 180.0
            #     max_B = np.pi * 10.0 / 180.0
            #     t = self.time_ - self.duration_ * 2
            #     L_A_des = max_A * np.sin(2.0 * np.pi * t)
            #     L_B_des = max_B * np.sin(2.0 * np.pi * t + np.pi)
            #     R_A_des = -max_A * np.sin(2.0 * np.pi * t)
            #     R_B_des = -max_B * np.sin(2.0 * np.pi * t + np.pi)

            #     self.low_cmd.motor_cmd[ADAMJointIndex.LeftAnkleA].q = L_A_des
            #     self.low_cmd.motor_cmd[ADAMJointIndex.LeftAnkleB].q = L_B_des
            #     self.low_cmd.motor_cmd[ADAMJointIndex.RightAnkleA].q = R_A_des
            #     self.low_cmd.motor_cmd[ADAMJointIndex.RightAnkleB].q = R_B_des
                
            #     max_WristYaw = np.pi * 30.0 / 180.0
            #     L_WristYaw_des = max_WristYaw * np.sin(2.0 * np.pi * t)
            #     R_WristYaw_des = max_WristYaw * np.sin(2.0 * np.pi * t)
            #     self.low_cmd.motor_cmd[ADAMJointIndex.LeftWristRoll].q = L_WristYaw_des
            #     self.low_cmd.motor_cmd[ADAMJointIndex.RightWristRoll].q = R_WristYaw_des

    

if __name__ == '__main__':

    print("WARNING: Please ensure there are no obstacles around the robot while running this example.")
    input("Press Enter to continue...")

    if len(sys.argv)>1:
        ChannelFactoryInitialize(1, sys.argv[1])  # Use domain ID 1 instead of 0
    else:
        ChannelFactoryInitialize(1)  # Use domain ID 1 instead of 0

    custom = Custom()
    custom.Init()
    print("Initialized")
    custom.Start()

    while True:        
        time.sleep(1)