# IDL for adam_u
from .adam_u.msg.dds_ import LowCmd_ as ADAMLowCmd_
from .adam_u.msg.dds_ import LowState_ as ADAMLowState_
from .adam_u.msg.dds_ import MotorCmd_ as ADAMMotorCmd_
from .adam_u.msg.dds_ import MotorState_ as ADAMMotorState_
from .adam_u.msg.dds_ import HandCmd_ as ADAMHandCmd_
from .adam_u.msg.dds_ import HandState_ as ADAMHandState_


"""
" std_msgs.msg.dds_ dafault
"""

def adam_u_msg_dds__MotorCmd_():
    return ADAMMotorCmd_(0, 0.0, 0.0, 0.0, 0.0, 0.0, 0)

def adam_u_msg_dds__MotorState_():
    return ADAMMotorState_(0, 0, 0, 0, 0, 0)


def adam_u_msg_dds__LowCmd_():
    return ADAMLowCmd_([adam_u_msg_dds__MotorCmd_() for i in range(19)], 0)

def adam_u_msg_dds__LowState_():
    return ADAMLowState_([adam_u_msg_dds__MotorState_() for i in range(19)], 0)

def adam_u_msg_dds__HandCmd_():
    return ADAMHandCmd_([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 0)

def adam_u_msg_dds__HandState_():
    return ADAMHandState_([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 0)
