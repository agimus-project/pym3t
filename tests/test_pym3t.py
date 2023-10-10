"""Tests asserting proper behaviour of python bindings."""
import numpy as np
import pym3t

import pytest



"""
Checking that type casting works properly.

Use dummy cameras class for now. 
Would be better to decouple the 2 tests in the future by defining 
special checking fonctions in C++ and binding them.
"""
def test_types_and_dummy_camera():
    color_camera = pym3t.DummyColorCamera('cam_color')
    depth_camera = pym3t.DummyDepthCamera('cam_color')

    # numpy <--> Transform3fA aka Eigen::Transform<float, 3, Eigen::Affine>
    T1 = np.random.random((4,4)).astype(np.float64)
    T2 = np.random.random((4,4)).astype(np.float32)
    color_camera.color2depth_pose = T1
    assert(color_camera.color2depth_pose.dtype == np.float32)
    assert(color_camera.color2depth_pose.shape == (4,4))
    assert(np.isclose(color_camera.color2depth_pose, T1).any())
    color_camera.color2depth_pose = T2
    assert(color_camera.color2depth_pose.dtype == np.float32)
    assert(color_camera.color2depth_pose.shape == (4,4))
    assert(np.isclose(color_camera.color2depth_pose, T2).any())

    # numpy <--> cv::Mat
    color_size = (60,40,3)
    depth_size = (80,50)
    img_rgb = np.random.random(color_size).astype(np.uint8)
    img_depth = np.random.random(depth_size).astype(np.uint16)
    color_camera.image = img_rgb
    depth_camera.image = img_depth
    assert(color_camera.image.shape == color_size)
    assert(depth_camera.image.shape == depth_size)
    assert(color_camera.image.dtype == np.uint8)
    assert(depth_camera.image.dtype == np.uint16)

    # Error raising when passing wrong types
    with pytest.raises(ValueError):
        color_camera.image = img_depth
    with pytest.raises(ValueError):
        depth_camera.image = img_rgb



