# Copyright (c) 2020-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import carb.input
import numpy as np
import omni
import omni.appwindow


from carb.input import GamepadConnectionEventType


from collections import defaultdict
from typing import DefaultDict # type: ignore


class GamePad:
    """
        Handles all gamepad commands for the window connected gamepad. 
        This gamepad is automatically detected by omniverse kit.
    """
    def __init__(self, base_command: np.array = np.array([0.0, 0.0, 0.0])) -> None:
        self._base_command = base_command
        self._appwindow: omni.appwindow.IAppWindow = omni.appwindow.get_default_app_window()

        self._gamepad_output: DefaultDict[carb.input.GamepadInput, float] = defaultdict(lambda: 0.0)
        
        self._input = carb.input.acquire_input_interface()
        self._gamepad_connection_sub = self._input.subscribe_to_gamepad_connection_events(self._gamepad_connection_event_callback)
        self._gamepad = self._appwindow.get_gamepad(0)
        if self._gamepad:
            self._gamepad_event_sub = self._input.subscribe_to_gamepad_events(self._gamepad, self._gamepad_event_callback)



    def _gamepad_connection_event_callback(self, event: carb.input.GamepadConnectionEvent) -> None:
        match event.type.value:
            case GamepadConnectionEventType.CREATED:
                pass
            case GamepadConnectionEventType.CONNECTED:
                self._gamepad = self._appwindow.get_gamepad()
                self._gamepad_event_sub = self._input.subscribe_to_gamepad_events(self._gamepad, self._gamepad_event_callback)
            case GamepadConnectionEventType.DISCONNECTED:
                self._gamepad = None
                self._input.unsubscribe_to_gamepad_events(self._gamepad_event_sub)
            case GamepadConnectionEventType.DESTROYED:
                pass



    def _gamepad_event_callback(self, event: carb.input.GamepadEvent) -> bool:
        self._gamepad_output[event.input] = self._input.get_gamepad_value(self._gamepad, event.input) # add latest input to dictionary
        return True
    

    def get_gamepad_output(self) -> DefaultDict[carb.input.GamepadInput, float]:
        """
            Returns the current gamepad inputs as a dictionary
        """
        return self._gamepad_output
    

    def cleanup(self):
        """
            Cleans up the gamepad subscriptions. Must be called when the scenario is destroyed
        """
        self._input.unsubscribe_to_gamepad_connection_events(self._gamepad_connection_sub)
        self._input.unsubscribe_to_gamepad_events(self._gamepad_event_sub)
        self._appwindow = None
        self._input = None
        self._gamepad = None

        
