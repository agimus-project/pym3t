import numpy as np
import pyicg

body_name = 'banana'
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
# renderer_geometry.AddBody(body)
# color_depth_renderer_ptr.AddReferencedBody(body)
# depth_depth_renderer_ptr.AddReferencedBody(body)
