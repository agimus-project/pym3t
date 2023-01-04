import numpy as np
import pyicg

body_name = 'banana'
detector_name = 'toto.yaml'
directory = 'toto/blabla'

tracker = pyicg.Tracker('tracker')
renderer_geometry = pyicg.RendererGeometry('renderer geometry')

color_camera = pyicg.RealSenseColorCamera("realsense_color")
depth_camera = pyicg.RealSenseDepthCamera("realsense_color")

depth_viewer = pyicg.NormalDepthViewer('depth_viewer_name', depth_camera, renderer_geometry, 0.3, 1.0)
# depth_viewer_ptr->StartSavingImages
tracker.AddViewer(depth_viewer)

color_viewer = pyicg.NormalColorViewer('color_viewer', color_camera, renderer_geometry)
tracker.AddViewer(color_viewer)

color_depth_renderer = pyicg.FocusedBasicDepthRenderer("color_depth_renderer", renderer_geometry, color_camera)
depth_depth_renderer = pyicg.FocusedBasicDepthRenderer("depth_depth_renderer", renderer_geometry, depth_camera)

metafile_path = directory + body_name
body = pyicg.Body(body_name, metafile_path)
renderer_geometry.AddBody(body)
color_depth_renderer.AddReferencedBody(body)
depth_depth_renderer.AddReferencedBody(body)

detector_path = directory + detector_name
detector = pyicg.StaticDetector("static_detector", detector_path, body)
tracker.AddDetector(detector)

region_model = pyicg.RegionModel(body_name + "_region_model", body, directory + body_name + "_region_model.bin")
depth_model = pyicg.DepthModel(body_name + "_depth_model", body, directory + body_name + "_depth_model.bin")

region_modality = pyicg.RegionModality(body_name + "_region_modality", body, color_camera, region_model)
depth_modality = pyicg.DepthModality(body_name + "_depth_modality", body, depth_camera, depth_model)

optimizer = pyicg.Optimizer(body_name+'_optimizer')
optimizer.AddModality(region_modality)
optimizer.AddModality(depth_modality)
tracker.AddOptimizer(optimizer)

# tracker.SetUp()
# tracker.RunTrackerProcess(execute_detection=True, start_tracking=False)
