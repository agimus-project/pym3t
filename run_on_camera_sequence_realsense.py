import numpy as np
import pyicg
from pathlib import Path

"""
Frames of reference:
- w = World: either = C or D, the camera is fixed wrt. the World
- c = Camera: frame of the RGB camera (origin = optical center, XYZ = Right-Down-Front)
- d = Depth: frame of the Depth camera (//)
- b = Body: frame associated to one of the tracked objects 
- g = Geometry: frame associated with the origin of the body geometric model (may be != b)

For the following comments, we adopt the notation: a 3D point p expressed in frames a and b by
the transformation T_a_b: p_a = T_a_b * p_b
In ICG (aka. "b2a" in ICG files) verifies: 

Commentaries:
Notes about the main method and data to interface with the different objects used in the project, by no means exhaustive. 
"""


# TODO: script arguments
body_name = 'cheezit'
detector_name = 'static_detector.yaml'
directory = Path('/home/mfourmy/sandbox/3DObjectTracking/ICG/examples/generator_example')

"""
Tracker: 
Conductor tying together all parts of ICG. Has an inner clock with predifined cycle time (default to 33 ms).
track cycle, each step execution depending on the state machine variables (@ -> only at setup time):
    1) Update cameras
    2) Detect new objects@
    3) Start modalities@
    4) ExecuteTrackingCycle
    5) UpdateViewers
* data: (@ -> can be added/removed)
    - With methods to add/remove: Optimizers, Detectors, Refiners, Viewers, Publishers, Modalities
    - Without: Models, Cameras, RendererGeometrys, Bodys, Renderers
        -> Bodys in particular: all tracked bodys should be present in the scene from the start?
    - global parameters and state machine variables
* methods:
    - highest level: good for 'automatic' use with GUI
        > RunTrackerProcess: main loop, runs the track cycle
        > QuitTrackerProcess
    - lower level: the user is in charge of the complete cycle
"""
# synchronize_cameras: to be able to print elapsed time
tracker = pyicg.Tracker('tracker', synchronize_cameras=False)


"""
RendererGeometry: 
Handle the GLFW context for rendering, used by the different renderers.
Loads and store mesh vertices from bodies, compute the triangle normals in geometry frame and add them to the GLFW context
For bodies added before setup, does this computation in SetUp, after that, it is done directly in AddBody.  
* data:
    - vector of body pointers
    - vertex_data: vertices and normals data, [x1,y1,z1,nx1,ny1,nz1,x2,y2...], size == 6*nb_vertices
    - glfw context
* methods:
    - AddBody
    - DeletBody
"""
renderer_geometry = pyicg.RendererGeometry('renderer geometry')

"""
RealSense(Color|Depth)Camera:
Connects to a RealSense device and implements intrinsics/extrinsics retrieval+frame updates.
* data:
    - current (color|depth) frame
    - intrinsics/extrinsics
    - is the (color|depth) camera considered as world frame
* methods:
    - UpdateImage -> grabs color|depth image from device
"""

color_camera = pyicg.RealSenseColorCamera('realsense_color')
depth_camera = pyicg.RealSenseDepthCamera('realsense_depth')

# Most time is spent on rendering (tested without GPU: ~15 ms for both, 8 for color only)
"""
Normal(Color|Depth)Viewer:
Renders (color|normalized depth) image using info stored in renderer_geometry.
"""
depth_viewer = pyicg.NormalDepthViewer('depth_viewer_name', depth_camera, renderer_geometry, 0.3, 1.0)
tracker.AddViewer(depth_viewer)

color_viewer = pyicg.NormalColorViewer('color_viewer', color_camera, renderer_geometry)
tracker.AddViewer(color_viewer)

"""
FocusedRenderer: interface with OpenGL, render only part of the image where tracked objects are present 
                 -> projection matrix is recomputed each time a new render is done (contrary to FullRender)
- 
"""
# We need 2 renderers because depth and color are slightly not aligned
color_depth_renderer = pyicg.FocusedBasicDepthRenderer('color_depth_renderer', renderer_geometry, color_camera)
depth_depth_renderer = pyicg.FocusedBasicDepthRenderer('depth_depth_renderer', renderer_geometry, depth_camera)

"""
 Body: 
 * data
    - metadata of the 3D model
    - data of the 3D model (vertices + triangles = mesh indices) 
    - T_w_b (body2world, equivalent to body2camera|depth) transformation
 * methods
    - compute and store maximum body diameter: used in the model and renderers 
"""
metafile_path = directory / (body_name+'.yaml')
body = pyicg.Body(body_name, metafile_path.as_posix())
renderer_geometry.AddBody(body)
color_depth_renderer.AddReferencedBody(body)
depth_depth_renderer.AddReferencedBody(body)

"""
StaticDetector:
For one body, stores an initial body2world_pose to start tracking with.
* data
    - body ptr
    - init pose
"""
detector_path = directory / detector_name
detector = pyicg.StaticDetector('static_detector', detector_path.as_posix(), body)
tracker.AddDetector(detector)

"""
(Region|Depth)Model
For one body, create a Sparse Viewpoint Model with virtual camera (of canonic intrinsics) on a sphere centered on the object.
A ".bin" file is a binary format file containing in order: [model parameters, body data, n_views, view_1, view_2, ..., view_n_views]
A stored View contains: [DataPoint1, DataPoint2, ..., DataPoint_n_points, orientation]
DataPoint: 3D contour points, 3D suface normal
orientation: direction from object center to camera center
Uses FullNormalRenderer: renders 4 chanel normal images -> X,Y,Z,silhouette
* data
    - view rendering variables
    - vector of views
* methods
    - SetUpRenderer: copy body object, sets up virtual camera, sets up a FullNormalRenderer
    - LoadModel: loads data from existing .bin file. Calls LoadModelParameters and LoadBodyData
    - GenerateModel: generate geodesic poses, setup multiple RendererGeometry (omp), for each view/pose generate datapoints (only part differing between color|depth)
    - DepthModel::GeneratePointData: uses the rendered silhouette image to sample n_points pixels in the silhouette, get 3D point and 3D normal from the renderer, 
        transform/rotate to body coordinate, depth offsets as well 
    - RegionModel::GeneratePointData: uses the rendered silhouette image to generate a vector of contours (cv::findContours -> can be several contours depending on the topology),
        sample point on these contours, compute corresponding 3D point coordinates (camera frame) using rendered depth img, transform them in body coord and store in DataPoint,
        approximate normal vec from local contour segment, rotate in body frame and stored in DataPoint

"""
region_model_path = directory / (body_name + '_region_model.bin')
region_model = pyicg.RegionModel(body_name + '_region_model', body, region_model_path.as_posix())
depth_model_path = directory / (body_name + '_depth_model.bin')
depth_model = pyicg.DepthModel(body_name + '_depth_model', body, depth_model_path.as_posix())

"""
(Region|Depth)Modality
Based on Body, Camera, and Model, compute gradient vector and hessian used by Optimizer to find body pose.
* data
    - qwdqwd
* methods
    - qwdqwd
"""
region_modality = pyicg.RegionModality(body_name + '_region_modality', body, color_camera, region_model)
depth_modality = pyicg.DepthModality(body_name + '_depth_modality', body, depth_camera, depth_model)

optimizer = pyicg.Optimizer(body_name+'_optimizer')
optimizer.AddModality(region_modality)
optimizer.AddModality(depth_modality)
tracker.AddOptimizer(optimizer)

ok = tracker.SetUp()
print('tracker.SetUp ok: ', ok)
tracker.RunTrackerProcess(execute_detection=True, start_tracking=True)
