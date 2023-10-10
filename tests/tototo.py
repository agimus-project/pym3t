"""Tests asserting proper behaviour of python bindings."""
import numpy as np
import pym3t

import unittest



color_camera = pym3t.DummyColorCamera('cam_color')
depth_camera = pym3t.DummyDepthCamera('cam_color')

# numpy <--> Transform3fA aka Eigen::Transform<Scalar, 3, Eigen::Affine>
T1 = np.random.random((4,4)).astype(np.float64)
T2 = np.random.random((4,4)).astype(np.float32)

color_camera.color2depth_pose = T1
assert(np.isclose(color_camera.color2depth_pose, T1).any())
color_camera.color2depth_pose = T2
assert(np.isclose(color_camera.color2depth_pose, T2).any())

# numpy <--> cv::Mat
img_rgb = np.random.random((60,40,3)).astype(np.uint8)
img_depth = np.random.random((60,40,1)).astype(np.uint16)
color_camera.image = img_rgb
depth_camera.image = img_depth
depth_camera.image = img_rgb
color_camera.image = img_depth
