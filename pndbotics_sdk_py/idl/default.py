# IDL for adam_u
from .adam_u.msg.dds_ import LowCmd_ as ADAMLowCmd_
from .adam_u.msg.dds_ import LowState_ as ADAMLowState_
from .adam_u.msg.dds_ import MotorCmd_ as ADAMMotorCmd_
from .adam_u.msg.dds_ import MotorState_ as ADAMMotorState_
from .adam_u.msg.dds_ import HandCmd_ as ADAMHandCmd_
from .adam_u.msg.dds_ import HandState_ as ADAMHandState_

from .pnd_adam.msg.dds_ import *

"""
" std_msgs.msg.dds_ dafault
"""

def adam_u_msg_dds__MotorCmd_():
    return ADAMMotorCmd_(0, 0.0, 0.0, 0.0, 0.0, 0.0, 0)

def adam_u_msg_dds__MotorState_():
    return ADAMMotorState_(0, 0.0, 0.0, 0.0, 0)


def adam_u_msg_dds__LowCmd_(num):
    return ADAMLowCmd_([adam_u_msg_dds__MotorCmd_() for i in range(num)], 0)

def adam_u_msg_dds__LowState_(num):
    return ADAMLowState_([adam_u_msg_dds__MotorState_() for i in range(num)],
                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 
                     0)

def adam_u_msg_dds__HandCmd_():
    return ADAMHandCmd_([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 0)

def adam_u_msg_dds__HandState_():
    return ADAMHandState_([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 0)


# IDL for pnd_adam

"""
" std_msgs.msg.dds_ dafault
"""
def pnd_adam_msg_dds__IMUState_():
    return IMUState_([0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], 0)

def pnd_adam_msg_dds__MotorCmd_():
    return MotorCmd_(0, 0.0, 0.0, 0.0, 0.0, 0.0, 0)

def pnd_adam_msg_dds__MotorState_():
    return MotorState_(0, 0.0, 0.0, 0.0, 0)

def pnd_adam_msg_dds__LowCmd_(num):
    # 使用空的 sequence 初始化 motor_cmd
    return LowCmd_([pnd_adam_msg_dds__MotorCmd_() for i in range(num)], 0)


def pnd_adam_msg_dds__LowState_(num):
    # 使用空的 sequence 初始化 motor_cmd
    return LowState_(pnd_adam_msg_dds__IMUState_(), [pnd_adam_msg_dds__MotorState_() for i in range(num)], 
                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 
                     0)

def pnd_adam_msg_dds__HandCmd_():
    return HandCmd_([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 0)

def pnd_adam_msg_dds__HandState_():
    return HandState_([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 0)
