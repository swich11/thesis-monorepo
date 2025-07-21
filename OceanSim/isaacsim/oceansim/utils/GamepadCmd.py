# Copyright (c) 2020-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import carb
import numpy as np
import omni
import omni.appwindow


class GamePadCmd:
    def __init__(self, base_command: np.array = np.array([0.0, 0.0, 0.0])) -> None:
        self._base_command = base_command
        self._appwindow = omni.appwindow.get_default_app_window()
        self._gamepad = self._appwindow.get_gamepad()
        self._input = carb.input.acquire_input_interface()
        self._input.subscribe_to_gamepad_connection_events(self._sub_gamepad_event)


    def _gamepad_connection_event(self)


    def _sub_gamepad_event(self, event, *args, **kwargs) -> bool:


