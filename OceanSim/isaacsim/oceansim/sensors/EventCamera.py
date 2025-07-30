# Omniverse Import
import omni.replicator.core as rep
import omni.ui as ui
from omni.gpu_foundation_factory._gpu_foundation_factory import TextureFormat

# Isaac sim import
from isaacsim.sensors.camera import Camera


import numpy as np
import warp as wp
import cv2
import yaml
import carb
import h5py
from pathlib import Path
import quaternion
from scipy.spatial.transform import Rotation


from isaacsim.oceansim.utils.TuplePair import TuplePair
from isaacsim.oceansim.render.EventRenderer import EventRenderer



# TODO: grab ground truth velocities (there is no physics prim on the robot -> grab local pose and use this between frames for now)
# TODO: make output dataset path choosable
# TODO: create all viewports and make them viewable (do not change outputs quite yet)


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
        self._prim_path = prim_path
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
        self.last_rot_pos: np.ndarray = np.array([0.0, 0.0, 0.0]) # zyx euler angles
        self.last_time = self._previous_time # gets the current time of the sim when the camera is launched

        if self._viewport:
            self.make_viewport()

        if writing_dir is not None:
            self._writing = True
            self._writing_backend = rep.BackendDispatch()
            self._write_dict = {
                "Events": TuplePair((np.uint8, (1, self._resolution[0], self._resolution[1]))),
                "Depths": TuplePair((np.float32, (1, self._resolution[0], self._resolution[1]))),
                "MotionFlow": TuplePair((np.float32, (4, self._resolution[0], self._resolution[1]))),
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

        # calculate camera prim velocity (this is the stupidest thing ever, but the robot doesn't have physics)
        pose = self.get_world_pose()
        
        print(pose[1])
        

        # lin_diff = pose[0] - self.last_trans_pos    # xyz world translation
        # quat_diff = pose[1] - self.last_rot_pos    # quarternion world orientation

        
        if hdr_curr.size != 0:
            event_frame = self._renderer.render(hdr_curr, depths)
            if self._viewport:
                # convert depth map values to grayscale image in rgba format
                # probably run these async
                depth_image = np.clip(np.nan_to_num(self._annot_dict["Depths"].data.numpy(), nan=0.0), 0.0, 20.0)
                depth_image = np.stack([np.uint8(depth_image / 20.0 * 255)]*3 + [np.ones_like(depth_image)*255], axis=2)

                motion_flow = self._annot_dict["MotionFlow"].data.numpy()
                motion_flow_image = self.draw_motion_flow(motion_flow[:, :, 0:2]) # only need the x and y flow
                #

                res = self.get_resolution()
                self._provider.set_bytes_data_from_gpu(event_frame.ptr, res)
                self._depth_provider.set_data_array(depth_image, res)
                self._flow_provider.set_data_array(motion_flow_image, res)


            if self._writing:
                # write all the data required from the renderer -> need to include base velocities here
                # need to add sim times here aswell
                self._writing_backend.schedule(self.write_h5py, data=event_frame, key="Events")
                self._writing_backend.schedule(self.write_h5py, data=self._annot_dict["Depths"].data, key="Depths")
                self._writing_backend.schedule(self.write_h5py, data=self._annot_dict["MotionFlow"].data, key="MotionFlow")
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


    def make_viewport(self):
        """
            Create a viewport for real-time visualization of events as frames.
        """
        self.wrapped_ui_elements = []
        # event window
        evcam_window = ui.Window(self._name, width=self._resolution[0], height=self._resolution[1] + 40, visible=True)
        self._provider = ui.ByteImageProvider()
        with evcam_window.frame:
            with ui.ZStack(height=self._resolution[1]):
                ui.Rectangle(style={"background_color": 0xFF000000})
                ui.Label('Run the scenario for events to be received',
                         style={'font_size': 55,'alignment': ui.Alignment.CENTER},
                         word_wrap=True)
                image_provider = ui.ImageWithProvider(self._provider, width=self._resolution[0], 
                                                      height=self._resolution[1], 
                                                      style={'fill_policy': ui.FillPolicy.PRESERVE_ASPECT_FIT,
                                                      'alignment' :ui.Alignment.CENTER})
        self.wrapped_ui_elements.append(image_provider)
        self.wrapped_ui_elements.append(self._provider)
        self.wrapped_ui_elements.append(evcam_window)
        # depth window
        depths_window = ui.Window("Depths", width=self._resolution[0], height=self._resolution[1] + 40, visible=True)
        self._depth_provider = ui.ByteImageProvider()
        with depths_window.frame:
            with ui.ZStack(height=self._resolution[1]):
                ui.Rectangle(style={"background_color": 0xFF000000})
                ui.Label('Run the scenario for depths to be received',
                         style={'font_size': 55,'alignment': ui.Alignment.CENTER},
                         word_wrap=True)
                depth_provider = ui.ImageWithProvider(self._depth_provider, width=self._resolution[0], 
                                                      height=self._resolution[1], 
                                                      style={'fill_policy': ui.FillPolicy.PRESERVE_ASPECT_FIT,
                                                      'alignment' :ui.Alignment.CENTER})
        self.wrapped_ui_elements.append(depths_window)
        self.wrapped_ui_elements.append(self._depth_provider)
        self.wrapped_ui_elements.append(depth_provider)
        # motion flow window
        flow_window = ui.Window("Motion Flow", width=self._resolution[0], height=self._resolution[1] + 40, visible=True)
        self._flow_provider = ui.ByteImageProvider()
        with flow_window.frame:
            with ui.ZStack(height=self._resolution[1]):
                ui.Rectangle(style={"background_color": 0xFF000000})
                ui.Label('Run the scenario for flow to be received',
                         style={'font_size': 55,'alignment': ui.Alignment.CENTER},
                         word_wrap=True)
                flow_provider = ui.ImageWithProvider(self._flow_provider, width=self._resolution[0], 
                                                      height=self._resolution[1], 
                                                      style={'fill_policy': ui.FillPolicy.PRESERVE_ASPECT_FIT,
                                                      'alignment' :ui.Alignment.CENTER})
        self.wrapped_ui_elements.append(flow_window)
        self.wrapped_ui_elements.append(self._flow_provider)
        self.wrapped_ui_elements.append(flow_provider)


    def make_image_viewport(self, key: str):
        window = ui.Window(key, width=self._resolution[0], height=self._resolution[1] + 40, visible=True)
        self._providers[key] = ui.ByteImageProvider()
        with window.frame:
            with ui.ZStack(height=self._resolution[1]):
                ui.Rectangle(style={"background_color": 0xFF000000})
                ui.Label('Run the scenario for data to be received',
                         style={'font_size': 55, 'alignment': ui.Alignment.CENTER},
                         word_wrap=True)
                image_provider = ui.ImageWithProvider(self._providers[key], 
                                                      width=self._resolution[0],
                                                      height=self._resolution[1],
                                                      style={'fill_policy': ui.FillPolicy.PRESERVE_ASPECT_FIT,
                                                      'alignment' :ui.Alignment.CENTER})
        self.wrapped_ui_elements.append(self._providers[key])
        self.wrapped_ui_elements.append(image_provider)
        self.wrapped_ui_elements.append(window)


    def make_graph_viewport(self, key: str):
        # TODO: make this a thing for 6-DOF IMU and 6-DOF velocities
        pass


    def make_viewports(self) -> None:
        # these are automatically used by viewport makers
        self._wrapped_ui_elements = []
        self._providers = {}

        self.make_image_viewport("Events")
        self.make_image_viewport("Depths")
        self.make_image_viewport("MotionFlow")

        self.make_graph_viewport("IMU")
        self.make_graph_viewport("Velocities")


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
        for elem in self.wrapped_ui_elements:
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
                                        

    def write_h5py(self, data: wp.array, key: str):
        """Write numpy array to h5py file format. The key defines the dataset group to store into.

        Returns -> None
        """
        data: np.ndarray = data.numpy()  # convert to numpy array
        dset: h5py.Dataset = self._write_dict[key].data
        dset.resize(tuple([dset.shape[0] + 1]) + dset.shape[1:len(dset.shape)]) # add 1 to dataset shape
        dset[-1] = data



    def close_h5py(self, file: h5py.File):
        """Close the h5py dataset for the run

        Returns -> None
        """
        file.close()

