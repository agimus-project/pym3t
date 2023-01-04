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
# tracker.AddViewer(depth_viewer)

color_viewer = pyicg.NormalColorViewer('color_viewer', color_camera, renderer_geometry)
# tracker_ptr.AddViewer(color_viewer)
