# Omniverse Import
import omni.replicator.core as rep
import omni.ui as ui
from omni.replicator.core.scripts.functional import write_np


# Isaac sim import
from isaacsim.sensors.camera import Camera


import numpy as np
import warp as wp
import yaml
import carb
import h5py
from pathlib import Path


from utils.TuplePair import TuplePair


# TODO: grab ground truth velocities
# TODO: make output dataset path choosable
# TODO: change event renderer to use warp
# TODO: apply UW renderer to event camera renderer


class EventCamera(Camera):
    def __init__(self,
                 prim_path,
                 name = "Event_Camera",
                 frequency = None,
                 dt = None,
                 resolution = (346, 260),
                 position = None,
                 translation = None,
                 orientation = None,
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
        super.__init__(prim_path, name, frequency, dt, resolution, position, translation, orientation, render_product_path)


    def __del__(self):
        self.close()


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
        self._id = 0
        self._viewport = viewport
        self._device = wp.get_preferred_device()
        super().initialize(physics_sim_view)

        if UW_yaml_path is not None:
            with open(UW_yaml_path, 'r') as file:
                try:
                    # Load the YAML content
                    yaml_content = yaml.safe_load(file)
                    self._backscatter_value = wp.vec3f(*yaml_content['backscatter_value'])
                    self._atten_coeff = wp.vec3f(*yaml_content['atten_coeff'])
                    self._backscatter_coeff = wp.vec3f(*yaml_content['backscatter_coeff'])
                    print(f"[{self._name}] On {str(self._device)}. Using loaded render parameters:")
                    print(f"[{self._name}] Render parameters: {yaml_content}")
                except yaml.YAMLError as exc:
                    carb.log_error(f"[{self._name}] Error reading YAML file: {exc}")
        else:
            self._backscatter_value = wp.vec3f(*UW_param[0:3])
            self._atten_coeff = wp.vec3f(*UW_param[6:9])
            self._backscatter_coeff = wp.vec3f(*UW_param[3:6])
            print(f'[{self._name}] On {str(self._device)}. Using default render parameters.')


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
        # TODO: import oceansim warp on hdr make warp kernel to turn data into events by interpolation, actually show event image
        hdr_last = self._annot_dict["hdrColor"].data
        for key in self._annot_dict.keys():
            self._annot_dict[key].data = self._annot_dict[key][0].get_data()
        ldr = self._rgb_annotator.get_data() # from the Camera class
        hdr_curr = self._annot_dict["hdrColor"].data
        
        # testing
        if hdr_last:
            # test prints
            print(hdr_curr.dtype)
            print(hdr_curr)
            event_frame = self.generate_event_frame(hdr_last, hdr_curr)
            print(event_frame)


        # TODO: add event data to viewport

        if ldr.size != 0: # probably don't need this check
            if self._viewport:
                self._provider.set_image_data(ldr)

            if self._writing:
                self._writing_backend.schedule(write_np, path=f"RGB_image_{self._id}.png", data=ldr)
                self._writing_backend.schedule()
                print(f'[{self._name}] [{self._id}] rendered rgb image saved to {self._writing_backend.output_dir}')
                
            
            self._id += 1


    def make_viewport(self):
        """
            Create a viewport for real-time visualization of accumulated events as frames.
        """
        self.wrapped_ui_elements = []
        self.window = ui.Window(self._name, width=self._resolution[0], height=self._resolution[1] + 40, visible=True)

        
        self._provider = ui.ImageProvider() # TODO: make ui.ImageProvider for event camera images
        with self.window.frame:
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
        self.wrapped_ui_elements.append(self.window)


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
                                        



    def write_h5py(self, data: np.ndarray, key: str):
        """Write numpy array to h5py file format. The key defines the dataset group to store into.

        Returns -> None
        """
        dset: h5py.Dataset = self._write_dict[key].data
        dset.resize(tuple([dset.shape[0] + 1]) + dset.shape[1:len(dset.shape)]) # add 1 to dataset shape
        dset[-1] = data



    def close_h5py(self, file: h5py.File):
        """Close the h5py dataset for the run

        Returns -> None
        """
        file.close()



# TODO: convert to use warp later
class EventRenderer():
    def __init__(self, 
                 resolution = (346, 260),
                 threshold_on: float = 0.143,
                 threshold_off: float = 0.225,
                 std: float = 0.05,):
        self._threshold_on = threshold_on
        self._threshold_off = threshold_off
        self.pixel_store: np.ndarray = np.zeros(resolution, np.float32)
        self._noise_std = std


    def render(self, hdrCurr: np.ndarray) -> np.ndarray:
        hdrCurr = np.sum(hdrCurr[:, :, :3], axis=2) # add luminance for each colour channel together
        hdrCurr = np.log10(hdrCurr) + np.random.normal(0, self._noise_std, hdrCurr.shape) # log scale and add threshold noise
        on_pixels = hdrCurr - self.pixel_store > self._threshold_on
        off_pixels = self.pixel_store - hdrCurr > self._threshold_off
        self.pixel_store = np.logical_and(on_pixels, off_pixels)*hdrCurr
        return on_pixels*np.int8(1) - off_pixels*np.int8(1) # off pixels represented as -1



        
        











