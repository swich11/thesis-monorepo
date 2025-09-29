# Omniverse import
import numpy as np
from pxr import Gf, PhysxSchema
from pxr import Usd

# Isaac sim import
from isaacsim.core.prims import SingleRigidPrim
from isaacsim.core.utils.prims import get_prim_path


# for typing
from ....oceansim.sensors.BarometerSensor import BarometerSensor
from ....oceansim.sensors.ImagingSonarSensor import ImagingSonarSensor
from ....oceansim.sensors.DVLsensor import DVLsensor
from ....oceansim.sensors.UW_Camera import UW_Camera
from ....oceansim.sensors.UnifiedEventCamera import UnifiedEventCamera

# OceanSim imports
from ...utils.keyboard_cmd import keyboard_cmd
from ...utils.GamePad import GamePad


from carb.input import GamepadInput
from typing import List # type: ignore


class Sensor_Scenario():
    def __init__(self):
        self._rob: Usd.Prim = None
        self._sonar = None
        self._ev_cam = None
        self._cam = None
        self._DVL = None
        self._baro = None

        self._ctrl_mode = None

        self._running_scenario = False
        self._controllers: List[GamePad | keyboard_cmd] = []
        self._time = 0.0
        self._controls = []


    def setup_scenario(self, rob: Usd.Prim, rob_forceAPI, sonar: ImagingSonarSensor | None, ev_cam: UnifiedEventCamera | None, cam: UW_Camera | None, 
                       DVL: DVLsensor | None, baro: BarometerSensor | None, ctrl_mode: str):
        self._rob = rob
        self._rob_forceAPI = rob_forceAPI
        self._sonar = sonar
        self._ev_cam = ev_cam
        self._cam = cam
        self._DVL = DVL
        self._baro = baro
        self._ctrl_mode = ctrl_mode


        if self._sonar:
            self._sonar.sonar_initialize(include_unlabelled = True)
        if self._ev_cam:
            self._ev_cam.initialize()
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
                pass # this is called elsewhere?
            case "Controller":
                self.setup_controller_control()
            case "Straight line":
                pass # no need to set up control
            case _:
                # wtf .. do default control
                self.setup_manual_control()

        self._controllers: List[GamePad | keyboard_cmd] = []
        self._running_scenario = True


    def setup_manual_control(self):
        """
            Sets up manual control for the rov
        """
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
        self._controllers.append([self._force_cmd, self._torque_cmd])



    def setup_waypoint_control(self, waypoint_path, default_waypoint_path):
        try:
            self.waypoints = read_waypoints_from_file(waypoint_path)
            print("Waypoints loaded succesfully")
            print(f"Waypoint[0]: {self.waypoints[0]}")
        except:
            self.waypoints = read_waypoints_from_file(default_waypoint_path)
            print('Fail to load this waypoints. Back to default waypoints.')


        def read_waypoints_from_file(file_path):
            with open(file_path, 'r') as f:
                return [(float(x) for x in line.strip().split()) for line in f] # extract waypoints


    def setup_controller_control(self):
        self._gamepad = GamePad()
        self._controllers.append(self._gamepad)


    def teardown_scenario(self):
        # Cleanup GPU annotators
        if self._sonar:
            self._sonar.close()
        if self._cam:
            self._cam.close()
        if self._ev_cam:
            self._ev_cam.close()

        # release controllers
        for controller in self._controllers:
            controller.cleanup()


        self._rob = None
        self._sonar = None
        self._cam = None
        self._ev_cam = None
        self._DVL = None
        self._baro = None
        self._running_scenario = False
        self._time = 0.0


    def update_scenario(self, step: float):
        if not self._running_scenario:
            return
        
        self._time += step
        
        if self._sonar:
            self._sonar.make_sonar_data()
        if self._cam:
            self._cam.render()
        if self._ev_cam:
            self._ev_cam.render()
        if self._DVL:
            self._DVL_reading = self._DVL.get_linear_vel()
        if self._baro:
            self._baro_reading = self._baro.get_pressure()

        match self._ctrl_mode:
            case "Manual control":
                force_cmd = Gf.Vec3f(*self._force_cmd._base_command)
                torque_cmd = Gf.Vec3f(*self._torque_cmd._base_command)
                self._rob_forceAPI.CreateForceAttr().Set(force_cmd)
                self._rob_forceAPI.CreateTorqueAttr().Set(torque_cmd)
            case "Waypoints":
                if len(self.waypoints) > 0:
                    waypoints = self.waypoints[0]
                    self._rob.GetAttribute('xformOp:translate').Set(Gf.Vec3f(waypoints[0], waypoints[1], waypoints[2]))
                    self._rob.GetAttribute('xformOp:orient').Set(Gf.Quatd(waypoints[3], waypoints[4], waypoints[5], waypoints[6]))
                    self.waypoints.pop(0)
                else:
                    print('Waypoints finished') 
            case "Straight line":
                SingleRigidPrim(prim_path=get_prim_path(self._rob)).set_linear_velocity(np.array([0.5,0,0]))
            case "Controller":
                commands = self._gamepad.get_gamepad_output()
                force_cmd = 20 * Gf.Vec3f(commands[GamepadInput.LEFT_STICK_UP] - commands[GamepadInput.LEFT_STICK_DOWN],
                                     commands[GamepadInput.LEFT_STICK_LEFT] - commands[GamepadInput.LEFT_STICK_RIGHT],
                                     commands[GamepadInput.RIGHT_STICK_UP] - commands[GamepadInput.RIGHT_STICK_DOWN])
                torque_cmd = 20 * Gf.Vec3f(commands[GamepadInput.RIGHT_SHOULDER] - commands[GamepadInput.LEFT_SHOULDER],
                                           commands[GamepadInput.RIGHT_TRIGGER] - commands[GamepadInput.LEFT_TRIGGER],
                                           commands[GamepadInput.RIGHT_STICK_LEFT] - commands[GamepadInput.RIGHT_STICK_RIGHT])
                self._rob_forceAPI.CreateForceAttr().Set(force_cmd)
                self._rob_forceAPI.CreateTorqueAttr().Set(torque_cmd)

