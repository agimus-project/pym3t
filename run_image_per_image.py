import cv2
import glob
import numpy as np
from pathlib import Path

import pyicg

# TODO: script arguments
body_name = 'banana'
detector_name = 'static_detector.yaml'
directory = Path('/home/mfourmy/sandbox/3DObjectTracking/ICG/examples/generator_example')



tracker = pyicg.Tracker('tracker', synchronize_cameras=False)

renderer_geometry = pyicg.RendererGeometry('renderer geometry')

# color_camera = pyicg.RealSenseColorCamera('realsense_color')


T_d_c = np.eye(4)  # TODO
color_camera = pyicg.DummyColorCamera('realsense_color')
color_camera.color2depth_pose = T_d_c
intrinsics_dic = {
    'fu': 615,
    'fv': 615.2452392578125,
    'ppu': 324,
    'ppv': 237,
    'width': 8*640,
    'height': 480,
}
color_camera.intrinsics = pyicg.Intrinsics(**intrinsics_dic)
print(color_camera.intrinsics)
print(color_camera.intrinsics.fu)
print(color_camera.intrinsics.fv)
print(color_camera.intrinsics.ppu)
print(color_camera.intrinsics.ppv)
print(color_camera.intrinsics.width)
print(color_camera.intrinsics.height)

color_viewer = pyicg.NormalColorViewer('color_viewer', color_camera, renderer_geometry)
color_viewer.StartSavingImages('/home/mfourmy/sandbox/pyicg', "bmp")
color_viewer.set_opacity(1.0)
tracker.AddViewer(color_viewer)

color_depth_renderer = pyicg.FocusedBasicDepthRenderer('color_depth_renderer', renderer_geometry, color_camera)

metafile_path = directory / (body_name+'.yaml')
body = pyicg.Body(body_name, metafile_path.as_posix())
renderer_geometry.AddBody(body)
color_depth_renderer.AddReferencedBody(body)

# TODO: instead of using a static detector, use instead body->set_world2body_pose
detector_path = directory / detector_name
detector = pyicg.StaticDetector('static_detector', detector_path.as_posix(), body)
tracker.AddDetector(detector)

region_model_path = directory / (body_name + '_region_model.bin')
region_model = pyicg.RegionModel(body_name + '_region_model', body, region_model_path.as_posix())

region_modality = pyicg.RegionModality(body_name + '_region_modality', body, color_camera, region_model)

optimizer = pyicg.Optimizer(body_name+'_optimizer')
optimizer.AddModality(region_modality)
tracker.AddOptimizer(optimizer)


ok = tracker.SetUp()
print('tracker.SetUp ok: ', ok)

# read images from disk
img_names = glob.glob('/home/mfourmy/sandbox/data/videos/bananas/*.png')

# read image and do the thing
img_bgr = cv2.imread(img_names[0], cv2.IMREAD_COLOR)  # loads a dtype=uint8 array
# cv2.imshow('YO', img_bgr)
# cv2.waitKey(0)


# Simulate one iteration of Tracker::RunTrackerProcess for loop
# 1) Update cameras
color_camera.image = img_bgr 
print('HEY')
color_viewer.UpdateViewer(save_index=42)
print('HEY')
# import time
# time.sleep(3)

# # cv2.imshow('YO', color_camera.image)
# # cv2.waitKey(0)

# # 2) Detect new objects
# print(body.body2world_pose)
# ok = tracker.ExecuteDetectionCycle(1)
# print(body.body2world_pose)
# print('ExecuteDetectionCycle ok: ', ok)

# # 3) Start modalities@
# ok = tracker.UpdateViewers(1)
# ok = tracker.StartModalities(1)
# print('StartModalities ok: ', ok)


# # 4) ExecuteTrackingCycle
# ok = tracker.ExecuteTrackingCycle(1)
# print('ExecuteTrackingCycle ok: ', ok)
# print(body.body2world_pose)

# # # 5) UpdateViewers
# # ok = tracker.UpdateViewers(1)
# # print('UpdateViewers ok: ', ok)

