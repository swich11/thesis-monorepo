import h5py
import numpy as np


from DepthMapNet import DepthMapNet
from DepthMapVelocityNet import DepthMapVelocityNet
from OpticalFlowNet import OpticalFlowNet
from OpticalFlowVelocityNet import OpticalFlowVelocityNet
from VelocityNet import VelocityNet


from enum import Enum


class DataName(Enum):
    RENDER_TIME = 0,
    PHYSICS_TIME = 1,
    IMU_LIN_ACC = 2,
    IMU_ANG_VEL = 3,
    FRAME_VELOCITIES = 4,
    ON_EVENTS = 5,
    OFF_EVENTS = 6,
    DEPTH_MAP = 7,
    MOTION_FLOW = 8,

name_map = {
    DataName.RENDER_TIME: "RenderTime",
    DataName.PHYSICS_TIME: "IMUTime",
    DataName.IMU_LIN_ACC: "IMULinearAcceleration",
    DataName.IMU_ANG_VEL: "IMUAngularVelocity",
    DataName.FRAME_VELOCITIES: "Velocities",
    DataName.ON_EVENTS: "OnEvents",
    DataName.OFF_EVENTS: "OffEvents",
    DataName.DEPTH_MAP: "Depths",
    DataName.MOTION_FLOW: "MotionFlow",
}




# # Takes a hdf5 file, doesn't really matter what the heck the end tag is
# f = h5py.File('sim_dataset.hdf5', 'r')
# dset =f[DataName.RENDER_TIME]






# print(dset)
# print(dset.size)


# f.close()