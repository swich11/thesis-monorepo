import omni.replicator.core as rep
import omni.ui as ui
from omni.gpu_foundation_factory._gpu_foundation_factory import TextureFormat


from isaacsim.sensors.camera import Camera
from isaacsim.sensors.physics import IMUSensor


from isaacsim.oceansim.utils.TuplePair import TuplePair
from isaacsim.oceansim.utils.quaternion import angular_velocities
from isaacsim.oceansim.render.EventRenderer import EventRenderer


import numpy as np
import warp as wp
import cv2
import yaml
import carb
import h5py
from pathlib import Path
import quaternion


from typing import Dict # type: ignore


# TODO: create all viewports and make them viewable (do not change outputs quite yet)
# TODO: make output dataset path choosable


class EventCamera(Camera):
    def __init__(self,
                 prim_path,
                 name = "Event_Camera",
                 frequency = None,
                 dt = None,
                 resolution = (346, 260),
                 position = None,
                 orientation = None,
                 translation = None,
                 render_product_path = None,):
        """Initialize an event camera sensor.
    
        Args:
            prim_path (str): prim path of the Camera Prim to encapsulate or create.
            name (str, optional): shortname to be used as a key by Scene class.
                                    Note: needs to be unique if the object is added to the Scene.
                                    Defaults to "UW_Camera".
            frequency (Optional[int], optional): Frequency of the sensor (i.e: how often is the data frame updated).
                                                Defaults to None.
            dt (Optional[str], optional): dt of the sensor (i.e: period at which a the data frame updated). Defaults to None.
            resolution (Optional[Tuple[int, int]], optional): resolution of the camera (width, height). Defaults to None.
            position (Optional[Sequence[float]], optional): position in the world frame of the prim. shape is (3, ).
                                                        Defaults to None, which means left unchanged.
            translation (Optional[Sequence[float]], optional): translation in the local frame of the prim
                                                            (with respect to its parent prim). shape is (3, ).
                                                            Defaults to None, which means left unchanged.
            orientation (Optional[Sequence[float]], optional): quaternion orientation in the world/ local frame of the prim
                                                            (depends if translation or position is specified).
                                                            quaternion is scalar-first (w, x, y, z). shape is (4, ).
                                                            Defaults to None, which means left unchanged.
            render_product_path (str): path to an existing render product, will be used instead of creating a new render product
                                    the resolution and camera attached to this render product will be set based on the input arguments.
                                    Note: Using same render product path on two Camera objects with different camera prims, resolutions is not supported
                                    Defaults to None
        """
        self._name = name
        self._res = resolution
        self._writing = False
        super().__init__(prim_path, name, frequency, dt, resolution, position, orientation, translation, render_product_path)


    def initialize(self,
                   UW_param: np.ndarray = np.array([0.0, 0.31, 0.24, 0.05, 0.05, 0.2, 0.05, 0.05, 0.05 ]),
                   viewport: bool = True,
                   writing_dir: str = None,
                   UW_yaml_path: str = None,
                   physics_sim_view = None):

        """Configure underwater rendering properties and initialize pipelines.

        Args:
            UW_param (np.ndarray, optional): Underwater parameters array:
                [0:3] - Backscatter value (RGB)
                [3:6] - Attenuation coefficients (RGB)
                [6:9] - Backscatter coefficients (RGB)
                Defaults to typical coastal water values.
            viewport (bool, optional): Enable viewport visualization. Defaults to True.
            writing_dir (str, optional): Directory to save camera events. Defaults to None.
            UW_yaml_path (str, optional): Path to YAML file with water properties. Defaults to None.
            physics_sim_view (_type_, optional): _description_. Defaults to None.            

        """
        self._id: int = 0
        self._viewport = viewport
        self._device = wp.get_preferred_device()
        super().initialize(physics_sim_view)

        # add child sensors
        self._IMU = IMUSensor(
            prim_path=self.prim_path + "/IMU",
            name="event_cam_IMU",
            frequency=60,
            translation=np.array([0, 0, 0]),
            orientation=np.array([1, 0, 0, 0]),
            linear_acceleration_filter_size = 10,
            angular_velocity_filter_size = 10,
            orientation_filter_size = 10,
        )
        self._IMU.initialize(physics_sim_view)


        if UW_yaml_path is not None:
            with open(UW_yaml_path, 'r') as file:
                try:
                    # Load the YAML content
                    yaml_content = yaml.safe_load(file)
                    backscatter_value = wp.vec3f(*yaml_content['backscatter_value'])
                    attenuation_coeff = wp.vec3f(*yaml_content['atten_coeff'])
                    backscatter_coeff = wp.vec3f(*yaml_content['backscatter_coeff'])
                    print(f"[{self._name}] On {str(self._device)}. Using loaded render parameters:")
                    print(f"[{self._name}] Render parameters: {yaml_content}")
                except yaml.YAMLError as exc:
                    carb.log_error(f"[{self._name}] Error reading YAML file: {exc}")
        else:
            backscatter_value = wp.vec3f(*UW_param[0:3])
            attenuation_coeff = wp.vec3f(*UW_param[6:9])
            backscatter_coeff = wp.vec3f(*UW_param[3:6])
            print(f'[{self._name}] On {str(self._device)}. Using default render parameters.')

        self._renderer = EventRenderer(backscatter_value=backscatter_value,
                                       atten_coeff=attenuation_coeff,
                                       backscatter_coeff=backscatter_coeff)

        # Add event annotators here
        self._annot_dict = {
            "HdrColor": TuplePair(tuple([rep.AnnotatorRegistry.get_annotator('HdrColor', device=str(self._device))])),
            "Depths": TuplePair(tuple([rep.AnnotatorRegistry.get_annotator('distance_to_image_plane', device=str(self._device))])),
            "Dists": TuplePair(tuple([rep.AnnotatorRegistry.get_annotator('distance_to_camera', device=str(self._device))])),
            "MotionFlow":  TuplePair(tuple([rep.AnnotatorRegistry.get_annotator('motion_vectors', device=str(self._device))])),
        }

        # attach annotators
        for key in self._annot_dict.keys():
            self._annot_dict[key][0].attach(self._render_product_path)

        # velocity inference because there is no physics sim
        self.last_trans_pos: np.ndarray = np.array([0.0, 0.0, 0.0])
        self.last_quat_pos: np.ndarray = np.array([1.0, 0.0, 0.0, 0.0])
        self.last_time = None

        if self._viewport:
            self.make_viewports()

        if writing_dir is not None:
            self._writing = True
            self._writing_backend = rep.BackendDispatch()
            self._write_dict = {
                "OnEvents": TuplePair((np.bool_, (1, self._resolution[0], self._resolution[1]))),
                "OffEvents": TuplePair((np.bool_, (1, self._resolution[0], self._resolution[1]))),
                "Depths": TuplePair((np.float32, (1, self._resolution[0], self._resolution[1]))),
                "MotionFlow": TuplePair((np.float32, (4, self._resolution[0], self._resolution[1]))),
                "Velocities": TuplePair((np.float32, (6, 1))),
                # "FrameTimes"
                # "AccelerometerReadings"
            }
            self.open_h5py(Path(writing_dir, "sim_dataset").resolve())

        print(f'[{self._name}] Initialized successfully. Data writing: {self._writing}')


    def render(self):
        """Process continuous events for the time period of a single frame. Display the accumulated events in an image frame.
        Also processes and saves:
             - low dynamic range images
             - depth maps to image plane
             - motion flow
             - camera prim velocity.
        
        Note:
            - Updates viewport display if enabled
            - Saves all to disk if writing_dir was specified
        """
        for key in self._annot_dict.keys():
            self._annot_dict[key].data = self._annot_dict[key][0].get_data()
        ldr = self._rgb_annotator.get_data() # from the Camera class
        hdr_curr = self._annot_dict["HdrColor"].data
        depths = self._annot_dict["Dists"].data # distance to camera for water attenuation
        frame_time = self._current_frame["rendering_time"] # simulator time for the frame

        velocities: np.ndarray = self.get_velocities()
        imu_readings = self._IMU.get_current_frame()
        print(imu_readings)
        
        if hdr_curr.size != 0:
            on_events, off_events = self._renderer.calculate_events(hdr_curr, depths)
            event_frame = self._renderer.render(on_events, off_events)
            if self._viewport:
                # convert depth map values to grayscale image in rgba format
                # probably run these async
                depth_image = np.clip(np.nan_to_num(self._annot_dict["Depths"].data.numpy(), nan=0.0), 0.0, 20.0)
                depth_image = np.stack([np.uint8(depth_image / 20.0 * 255)]*3 + [np.ones_like(depth_image)*255], axis=2)

                motion_flow = self._annot_dict["MotionFlow"].data.numpy()
                motion_flow_image = self.draw_motion_flow(motion_flow[:, :, 0:2]) # only need the x and y flow

                res = self.get_resolution()
                self._image_providers["Events"].set_bytes_data_from_gpu(event_frame.ptr, res)
                self._image_providers["Depths"].set_data_array(depth_image, res)
                self._image_providers["MotionFlow"].set_data_array(motion_flow_image, res)


            if self._writing:
                # write all the data required from the renderer -> need to include base velocities here
                # need to add sim times here aswell
                self._writing_backend.schedule(self.write_h5py_numpy, data=velocities, key="Velocities")
                self._writing_backend.schedule(self.write_h5py_warp, data=on_events, key="OnEvents")
                self._writing_backend.schedule(self.write_h5py_warp, data=off_events, key="OffEvents")
                self._writing_backend.schedule(self.write_h5py_warp, data=self._annot_dict["Depths"].data, key="Depths")
                self._writing_backend.schedule(self.write_h5py_warp, data=self._annot_dict["MotionFlow"].data, key="MotionFlow")
                print(f'[{self._name}] [{self._id}] events saved to {self._writing_backend.output_dir}')
                
            self._id += 1


    def draw_motion_flow(self, flow: np.ndarray) -> np.ndarray:
        output = np.zeros((self._res[1], self._res[0], 4), dtype=np.uint8)
        output[:, :, 3] = 255
        for i in range(0, self._res[0] - 12, 12):
            for j in range(0, self._res[1] - 12, 12):
                average = np.int8(np.average(flow[j:(j+12), i:(i+12)], axis=(1, 0)))
                cv2.arrowedLine(output, (i + 6, j + 6), (i + 6 + average[0], j + 6 + average[1]), 
                                color=(255, 255, 255, 255), thickness=1, tipLength=0.3)
        return output


    def get_velocities(self) -> np.ndarray:
        """
            Gets the current velocities for the current render frame.

            Returns: np.ndarray(1, 6);
        """
        pose = self.get_world_pose()
        q_new = quaternion.from_float_array(pose[1])
        trans_new = quaternion.as_vector_part(q_new.conj() * quaternion.from_vector_part(pose[0]) * q_new)
        velocities = np.zeros(6)

        if self.last_time != self._previous_time and self.last_time:
            dt = self._previous_time - self.last_time
            q_last = self.last_quat_pos
            ang_vel = angular_velocities(quaternion.as_float_array(q_last), 
                                        quaternion.as_float_array(q_new), 
                                        dt)
            lin_vel = (trans_new - self.last_trans_pos) / dt
            velocities = np.hstack([lin_vel, ang_vel])
            
        self.last_time = self._previous_time
        self.last_quat_pos = q_new
        self.last_trans_pos = trans_new

        return velocities

        
    def make_image_viewport(self, key: str):
        window = ui.Window(key, width=self._resolution[0], height=self._resolution[1] + 40, visible=True)
        self._image_providers[key] = ui.ByteImageProvider()
        with window.frame:
            with ui.ZStack(height=self._resolution[1]):
                ui.Rectangle(style={"background_color": 0xFF000000})
                ui.Label('Run the scenario for data to be received',
                         style={'font_size': 55, 'alignment': ui.Alignment.CENTER},
                         word_wrap=True)
                image_provider = ui.ImageWithProvider(self._image_providers[key], 
                                                      width=self._resolution[0],
                                                      height=self._resolution[1],
                                                      style={'fill_policy': ui.FillPolicy.PRESERVE_ASPECT_FIT,
                                                      'alignment' :ui.Alignment.CENTER})
        self._wrapped_ui_elements.append(self._image_providers[key])
        self._wrapped_ui_elements.append(image_provider)
        self._wrapped_ui_elements.append(window)


    def make_graph_viewport(self, key: str):
        # TODO: make this a thing for 6-DOF velocities
        pass


    def make_viewports(self) -> None:
        self._wrapped_ui_elements = []
        self._image_providers: Dict[str, ui.ByteImageProvider] = {}

        self.make_image_viewport("Events")
        self.make_image_viewport("Depths")
        self.make_image_viewport("MotionFlow")


        # TODO: Move Graph Viewports to a velocity logging class
        # self.make_graph_viewport("IMU_Linear")
        # self.make_graph_viewport("IMU_Angular")
        # self.make_graph_viewport("GroundTruthLinear")
        # self.make_graph_viewport("GroundTruthAngular")


    def close(self):
        """Clean up resources by detaching annotators and clearing caches.
    
        Note:
            - Required for proper shutdown when done using the sensor
            - Also closes viewport window if one was created
        """
        # this will lose data that hasn't been written yet but that is ok
        for key in self._annot_dict.keys():
            self._annot_dict[key][0].detach(self._render_product_path)
            rep.AnnotatorCache.clear(self._annot_dict[key][0])

        if self._writing:
            self.close_h5py() # stop writing data
        if self._viewport:
            self.ui_destroy()
        
        
    def ui_destroy(self):
        """Explicitly destroy viewport UI elements.
    
        Note:
            - Called automatically by close()
            - Only needed if manually managing UI lifecycle
        """
        for elem in self._wrapped_ui_elements:
            elem.destroy()


    def open_h5py(self, path: str):
        """Create the h5py dataset on path. Destroys dataset if it already exists.
        
        Returns -> None
        """
        # TODO: filter dir for no overwrites
        self._dataset_file = h5py.File(path, 'w')
        for key in self._write_dict.keys():
            data_shape = list(self._write_dict[key][1])
            self._write_dict[key].data = self._dataset_file.create_dataset(
                                                                name=key,
                                                                shape=tuple([0] + data_shape),
                                                                dtype=self._write_dict[key][0],
                                                                maxshape=tuple([None] + data_shape),
                                                                compression="lzf",        
            ) # this will autochunk and autocompress
                                        

    def write_h5py_warp(self, data: wp.array, key: str) -> None:
        """
            Write warp array to h5py file format. The key defines the dataset group to store into.

            Returns -> None
        """
        data: np.ndarray = data.numpy()  # convert to numpy array
        self.write_h5py_numpy(data.numpy(), key)


    def write_h5py_numpy(self, data: np.array, key: str) -> None:
        """
            Write numpy array to h5py file format. The key defines the dataset group to store into. 
        """
        dset: h5py.Dataset = self._write_dict[key].data
        dset.resize(tuple([dset.shape[0] + 1]) + dset.shape[1:len(dset.shape)]) # add 1 to dataset shape
        dset[-1] = data


    def close_h5py(self, file: h5py.File) -> None:
        """Close the h5py dataset for the run

        Returns -> None
        """
        file.close()

