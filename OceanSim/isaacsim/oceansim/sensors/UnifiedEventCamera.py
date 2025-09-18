import numpy as np


from isaacsim.core.prims import SingleRigidPrim


from isaacsim.sensors.physics import IMUSensor
from EventCamera import EventCamera


class UnifiedEventCamera():
    def __init__(self, prim_path, name = "EventCamera", translation = None, camera_focal_length = 21):
        self.rigid_prim = SingleRigidPrim(prim_path, name=__name__, translation=translation)

        self._camera = EventCamera(prim_path=prim_path + "/Camera")
        self._camera.set_focal_length(0.1*camera_focal_length)
        self._camera.set_clipping_range(0.1, 100)

        self._IMU = IMUSensor(
            prim_path=prim_path + "/Imu",
            name="event_cam_IMU",
            frequency=60,
            translation=np.array([0, 0, 0]),
            orientation=np.array([1, 0, 0, 0]),
            linear_acceleration_filter_size = 10,
            angular_velocity_filter_size = 10,
            orientation_filter_size = 10,
        )

    
    def initialize(self, 
                   UW_param: np.ndarray = np.array([0.0, 0.31, 0.24, 0.05, 0.05, 0.2, 0.05, 0.05, 0.05 ]),
                   viewport: bool = True,
                   writing_dir: str = None,
                   UW_yaml_path: str = None,
                   physics_sim_view = None):
        self._camera.initialize(UW_param, viewport, writing_dir, UW_yaml_path, physics_sim_view)
        self._IMU.initialize(physics_sim_view=physics_sim_view)


    def update(self):
        self._camera.render()
        
        
        