# Omniverse import
import numpy as np
from pxr import Gf, PhysxSchema

# Isaac sim import
from isaacsim.core.prims import SingleRigidPrim
from isaacsim.core.utils.prims import get_prim_path


# for typing
from ....oceansim.sensors.BarometerSensor import BarometerSensor
from ....oceansim.sensors.ImagingSonarSensor import ImagingSonarSensor
from ....oceansim.sensors.DVLsensor import DVLsensor
from ....oceansim.sensors.UW_Camera import UW_Camera
from ....oceansim.sensors.EventCamera import EventCamera


class Sensor_Scenario():
    def __init__(self):
        self._rob = None
        self._sonar = None
        self._ev_cam = None
        self._cam = None
        self._DVL = None
        self._baro = None

        self._ctrl_mode = None

        self._running_scenario = False
        self._time = 0.0


    def setup_scenario(self, rob, sonar: ImagingSonarSensor | None, ev_cam: EventCamera | None, cam: UW_Camera | None, 
                       DVL: DVLsensor | None, baro: BarometerSensor | None, ctrl_mode: str):
        self._rob = rob
        self._sonar = sonar
        self._ev_cam = ev_cam
        self._cam = cam
        self._DVL = DVL
        self._baro = baro
        self._ctrl_mode = ctrl_mode

        if self._sonar:
            self._sonar.sonar_initialize(include_unlabelled = True)
        if self._ev_cam:
            self._cam.initialize()
        if self._cam:
            self._cam.initialize()
        if self._baro:
            self._baro_reading = 101325.0 # 1atm in pascals
        if self._DVL:
            self._DVL_reading = [0.0, 0.0, 0.0]
        
        match self._ctrl_mode:
            case "Manual control":
                self.setup_manual_control()
            case "Waypoints":
                self.setup_waypoint_control()
            case "Controller":
                self.setup_controller_control()
            case _:
                # wtf .. do default control
                self.setup_manual_control()
                
        self._running_scenario = True


    def setup_manual_control(self):
        """
            Sets up manual control for the rov
        """
        from ...utils.keyboard_cmd import keyboard_cmd

        self._rob_forceAPI = PhysxSchema.PhysxForceAPI.Apply(self._rob)
        self._force_cmd = keyboard_cmd(base_command=np.array([0.0, 0.0, 0.0]),
                                    input_keyboard_mapping={
                                    # forward command
                                    "W": [10.0, 0.0, 0.0],
                                    # backward command
                                    "S": [-10.0, 0.0, 0.0],
                                    # leftward command
                                    "A": [0.0, 10.0, 0.0],
                                    # rightward command
                                    "D": [0.0, -10.0, 0.0],
                                        # rise command
                                    "UP": [0.0, 0.0, 10.0],
                                    # sink command
                                    "DOWN": [0.0, 0.0, -10.0],
                                    })
        self._torque_cmd = keyboard_cmd(base_command=np.array([0.0, 0.0, 0.0]),
                                    input_keyboard_mapping={
                                    # yaw command (left)
                                    "J": [0.0, 0.0, 10.0],
                                    # yaw command (right)
                                    "L": [0.0, 0.0, -10.0],
                                    # pitch command (up)
                                    "I": [0.0, -10.0, 0.0],
                                    # pitch command (down)
                                    "K": [0.0, 10.0, 0.0],
                                    # row command (left)
                                    "LEFT": [-10.0, 0.0, 0.0],
                                    # row command (negative)
                                    "RIGHT": [10.0, 0.0, 0.0],
                                    })


    def setup_waypoint_control():
        pass


    def setup_controller_control():
        # TODO: add controller control
        pass
    
