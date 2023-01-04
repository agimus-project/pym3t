import numpy as np
import pyicg
from pathlib import Path

body_name = 'banana'
detector_name = 'static_detector.yaml'
directory = Path('/home/mfourmy/sandbox/3DObjectTracking/ICG/examples/generator_example')

tracker = pyicg.Tracker('tracker')
renderer_geometry = pyicg.RendererGeometry('renderer geometry')

color_camera = pyicg.RealSenseColorCamera('realsense_color')
rs_ok = color_camera.SetUp()
print('rs.SetUp ok: ', rs_ok)
depth_camera = pyicg.RealSenseDepthCamera('realsense_color')
# rs_ok = depth_camera.SetUp()
# print('rs.SetUp ok: ', rs_ok)

# depth_viewer = pyicg.NormalDepthViewer('depth_viewer_name', depth_camera, renderer_geometry, 0.3, 1.0)
# tracker.AddViewer(depth_viewer)

color_viewer = pyicg.NormalColorViewer('color_viewer', color_camera, renderer_geometry)
tracker.AddViewer(color_viewer)

color_depth_renderer = pyicg.FocusedBasicDepthRenderer('color_depth_renderer', renderer_geometry, color_camera)
# depth_depth_renderer = pyicg.FocusedBasicDepthRenderer('depth_depth_renderer', renderer_geometry, depth_camera)

metafile_path = directory / (body_name+'.yaml')
print(metafile_path.is_file())
body = pyicg.Body(body_name, metafile_path.as_posix())
renderer_geometry.AddBody(body)
color_depth_renderer.AddReferencedBody(body)
# depth_depth_renderer.AddReferencedBody(body)

detector_path = directory / detector_name
print(detector_path.is_file())
detector = pyicg.StaticDetector('static_detector', detector_path.as_posix(), body)
tracker.AddDetector(detector)

region_model_path = directory / (body_name + '_region_model.bin')
print(region_model_path.is_file())
region_model = pyicg.RegionModel(body_name + '_region_model', body, region_model_path.as_posix())
# depth_model_path = directory / (body_name + '_depth_model.bin')
# print(depth_model_path.is_file())
# depth_model = pyicg.DepthModel(body_name + '_depth_model', body, depth_model_path.as_posix())

region_modality = pyicg.RegionModality(body_name + '_region_modality', body, color_camera, region_model)
# depth_modality = pyicg.DepthModality(body_name + '_depth_modality', body, depth_camera, depth_model)

optimizer = pyicg.Optimizer(body_name+'_optimizer')
optimizer.AddModality(region_modality)
# optimizer.AddModality(depth_modality)
tracker.AddOptimizer(optimizer)

ok = tracker.SetUp()
print('tracker.SetUp ok: ', ok)
tracker.RunTrackerProcess(execute_detection=True, start_tracking=False)
