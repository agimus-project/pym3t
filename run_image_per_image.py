import cv2
import glob
import numpy as np
from pathlib import Path

import pyicg

# TODO: script arguments
body_name = 'banana'
detector_name = 'static_detector.yaml'
models_dir = Path('/home/mfourmy/sandbox/3DObjectTracking/ICG/examples/generator_example')

config_dir = Path('config')
tmp_dir = Path('tmp')

tracker = pyicg.Tracker('tracker', synchronize_cameras=False)

renderer_geometry = pyicg.RendererGeometry('renderer geometry')


color_camera = pyicg.DummyColorCamera('cam_color')
T_d_c = np.eye(4)  # TODO
color_camera.color2depth_pose = T_d_c
intrinsics_color = {
    'fu': 615.2,
    'fv': 615.2,
    'ppu': 324.0,
    'ppv': 237.0,
    'width': 640,
    'height': 480,
}
color_camera.intrinsics = pyicg.Intrinsics(**intrinsics_color)

depth_camera = pyicg.DummyDepthCamera('cam_depth')
T_d_c = np.eye(4)  # TODO
color_camera.depth2color_pose = T_d_c
intrinsics_depth = {
    'fu':  423.893,
    'fv':  423.893,
    'ppu': 424.877,
    'ppv': 226.946,
    'width':  848,
    'height': 480,
}
depth_camera.intrinsics = pyicg.Intrinsics(**intrinsics_depth)

# Viewers
color_viewer = pyicg.NormalColorViewer('color_viewer', color_camera, renderer_geometry)
# color_viewer.StartSavingImages('tmp', "bmp")
color_viewer.set_opacity(0.5)  # [0.0-1.0]
tracker.AddViewer(color_viewer)

depth_viewer = pyicg.NormalDepthViewer('depth_viewer', depth_camera, renderer_geometry)
tracker.AddViewer(depth_viewer)

color_depth_renderer = pyicg.FocusedBasicDepthRenderer('color_depth_renderer', renderer_geometry, color_camera)
depth_depth_renderer = pyicg.FocusedBasicDepthRenderer('depth_depth_renderer', renderer_geometry, depth_camera)

metafile_path = models_dir / (body_name+'.yaml')
body = pyicg.Body(body_name, metafile_path.as_posix())
renderer_geometry.AddBody(body)
color_depth_renderer.AddReferencedBody(body)

detector_path = config_dir / detector_name
detector = pyicg.StaticDetector('static_detector', detector_path.as_posix(), body)
tracker.AddDetector(detector)

# Models
region_model_path = tmp_dir / (body_name + '_region_model.bin')
region_model = pyicg.RegionModel(body_name + '_region_model', body, region_model_path.as_posix())

depth_model_path = tmp_dir / (body_name + '_depth_model.bin')
depth_model = pyicg.DepthModel(body_name + '_depth_model', body, depth_model_path.as_posix())
depth_modality = pyicg.DepthModality(body_name + '_depth_modality', body, depth_camera, depth_model)

# Modalities
region_modality = pyicg.RegionModality(body_name + '_region_modality', body, color_camera, region_model)
depth_modality = pyicg.DepthModality(body_name + '_depth_modality', body, depth_camera, depth_model)

optimizer = pyicg.Optimizer(body_name+'_optimizer')
optimizer.AddModality(region_modality)
# optimizer.AddModality(depth_modality)
tracker.AddOptimizer(optimizer)

# Do all the necessary heavy preprocessing
ok = tracker.SetUp()
print('tracker.SetUp ok: ', ok)

# read images from disk
rgb_names = sorted(glob.glob('/home/mfourmy/sandbox/data/videos/bananas2/bgr*'))
depth_names = sorted(glob.glob('/home/mfourmy/sandbox/data/videos/bananas2/depth*'))

# LIMIT nb images
# rgb_names = rgb_names[:3]
# depth_names = depth_names[:3]
print(f'{len(rgb_names)} images to load')

# load images from disk
img_bgr_lst = [cv2.imread(name, cv2.IMREAD_COLOR) for name in rgb_names]  # loads a dtype=uint8 array
img_depth_lst = [cv2.imread(name, cv2.IMREAD_GRAYSCALE) for name in depth_names]  # loads a dtype=uint8 array


# Simulate one iteration of Tracker::RunTrackerProcess for loop
for iter, (img_bgr, img_depth) in enumerate(zip(img_bgr_lst, img_depth_lst)):
    print('Iter: ', iter)
    # 1) Update camera image -> replaces a call to the camera UpdateImage method (which does nothing for Dummy(Color|Depth)Camera) 
    color_camera.image = img_bgr
    depth_camera.image = img_depth

    if iter == 0:
        # 2) Use detector or external init to update initial object pose
        body.body2world_pose = detector.body2world_pose  # simulate external initial pose
        # tracker.ExecuteDetectionCycle(iter)
        # tracker.DetectBodies()

        # 3) Initialise the different modalities (e.g. the color histograms)
        tracker.StartModalities(iter)
        
    # 4) One tracking cycle (could call it several time)
    tracker.ExecuteTrackingCycle(iter)

    # 5) Render results
    tracker.UpdateViewers(iter)

    cv2.waitKey(0)

