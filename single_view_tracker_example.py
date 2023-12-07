import argparse
import numpy as np
import quaternion
from pathlib import Path

import pym3t


def inv_SE3(T: np.ndarray):
    """
    Inverse of an SE(3) 4x4 array.
    """
    Tinv = np.eye(4)
    Tinv[:3,:3] = T[:3,:3].T
    Tinv[:3,3] = -T[:3,:3].T@T[:3,3]
    return Tinv


def tq_to_SE3(t, q):
    """
    :param t: translation as list or array
    :param q: quaternion as list or array, expected order: xyzw
    :return: 4x4 array representing the SE(3) transformation
    """
    T = np.eye(4)
    T[:3,3] = t
    # np.quaternion constructor uses wxyz convention
    quat = np.quaternion(q[3], q[0], q[1], q[2]).normalized()
    T[:3,:3] = quaternion.as_rotation_matrix(quat)
    return T


def setup_single_object_tracker(args: argparse.Namespace, cam_intrinsics: dict = None, use_realsense=False):
    """
    Setup and example pym3t object tracker and related objects

    :param args: cli arguments from argparse
        should have at least these attributes:
        body_name
        models_dir
        fov
        scale_geometry
        tmp_dir
        use_region
        use_texture
        model_occlusions
    :param cam_intrinsics: dict containing camera intrinsics, 
                           same structure as config/cam_d435_640.yaml
    """

    # Missing arguments
    if 'use_depth' not in args:
        args.use_depth = False
    if 'use_depth_viewer' not in args:
        args.use_depth_viewer = False
    if 'measure_occlusions' not in args:
        args.measure_occlusions = False
    
    print('use_realsense', use_realsense)
    print('cam_intrinsics', cam_intrinsics)
    assert not (cam_intrinsics is not None and use_realsense), 'Do not pass cam intrinsics parameter if realsense is used'

    tmp_dir = Path(args.tmp_dir)
    tmp_dir.mkdir(exist_ok=True)

    # synchronize_cameras: to be able to print elapsed time
    tracker = pym3t.Tracker('tracker', synchronize_cameras=False)
    renderer_geometry = pym3t.RendererGeometry('renderer geometry')

    # Setup camera(s)
    if use_realsense:
        color_camera = pym3t.RealSenseColorCamera('realsense_color')
        if args.use_depth:
            depth_camera = pym3t.RealSenseDepthCamera('realsense_depth')
    else:
        color_camera = pym3t.DummyColorCamera('cam_color')
        color_camera.color2depth_pose = tq_to_SE3(cam_intrinsics['trans_d_c'], cam_intrinsics['quat_d_c_xyzw'])
        color_camera.intrinsics = pym3t.Intrinsics(**cam_intrinsics['intrinsics_color'])
        if args.use_depth: 
            depth_camera = pym3t.DummyDepthCamera('cam_depth')
            depth_camera.depth2color_pose = inv_SE3(color_camera.color2depth_pose)
            depth_camera.intrinsics = pym3t.Intrinsics(**cam_intrinsics['intrinsics_depth'])

    # Most time is spent on rendering (tested without GPU: ~15 ms for both, 8 for color only)
    color_viewer = pym3t.NormalColorViewer('color_viewer', color_camera, renderer_geometry)
    tracker.AddViewer(color_viewer)
    if args.use_depth and args.use_depth_viewer:
        depth_viewer = pym3t.NormalDepthViewer('depth_viewer_name', depth_camera, renderer_geometry)
        tracker.AddViewer(depth_viewer)
    else:
        depth_viewer = None

    # Setup body model and properties
    obj_model_path = Path(args.models_dir) / f'{args.body_name}.obj'
    if not obj_model_path.exists(): raise ValueError(f'{obj_model_path} is a wrong path')
    print(f'Loading object {obj_model_path}')
    body = pym3t.Body(
        name=args.body_name,
        geometry_path=obj_model_path.as_posix(),
        geometry_unit_in_meter=args.scale_geometry,
        geometry_counterclockwise=1,
        geometry_enable_culling=1,
        geometry2body_pose=np.eye(4)
    )
    renderer_geometry.AddBody(body)

    # Set up link: m3t handles polyarticulated systems but 
    # here we have only one link corresponding to the object with identity transform wrt to the body
    link = pym3t.Link(args.body_name + '_link', body)

    # Shared renderer between region and texture for inter objects occlusion handling (no effect for single object tracking)
    if args.model_occlusions and (args.use_region or args.use_texture):
        focused_color_depth_renderer = pym3t.FocusedBasicDepthRenderer('focused_color_depth_renderer', renderer_geometry, color_camera)
        focused_color_depth_renderer.AddReferencedBody(body)

    # Region Modality
    if args.use_region:
        region_model_path = tmp_dir / (args.body_name + '_region_model.bin')
        region_model = pym3t.RegionModel(args.body_name + '_region_model', body, region_model_path.as_posix())
        region_modality = pym3t.RegionModality(args.body_name + '_region_modality', body, color_camera, region_model)
        if args.model_occlusions:
            region_modality.ModelOcclusions(focused_color_depth_renderer)
        if args.measure_occlusions and args.use_depth:
            region_modality.MeasureOcclusions(depth_camera)
        link.AddModality(region_modality)

    # Depth Modality
    if args.use_depth:
        depth_model_path = tmp_dir / (args.body_name + '_depth_model.bin')
        depth_model = pym3t.DepthModel(args.body_name + '_depth_model', body, depth_model_path.as_posix())
        depth_modality = pym3t.DepthModality(args.body_name + '_depth_modality', body, depth_camera, depth_model)
        if args.model_occlusions:
            focused_depth_depth_renderer = pym3t.FocusedBasicDepthRenderer('focused_depth_depth_renderer', renderer_geometry, depth_camera)
            focused_depth_depth_renderer.AddReferencedBody(body)
            depth_modality.ModelOcclusions(focused_depth_depth_renderer)
        if args.measure_occlusions and args.use_depth:
            depth_modality.MeasureOcclusions()
        link.AddModality(depth_modality)

    # Texture Modality
    if args.use_texture:
        # Texture modality does not require a model contrary to region and depth (for sparse view precomputations)
        color_silhouette_renderer = pym3t.FocusedSilhouetteRenderer('color_silhouette_renderer', renderer_geometry, color_camera)
        color_silhouette_renderer.AddReferencedBody(body)
        texture_modality = pym3t.TextureModality(args.body_name + '_texture_modality', body, color_camera, color_silhouette_renderer)
        if args.model_occlusions:
            texture_modality.ModelOcclusions(focused_color_depth_renderer)
        if args.measure_occlusions and args.use_depth:
            texture_modality.MeasureOcclusions(depth_camera)
        link.AddModality(texture_modality)

    optimizer = pym3t.Optimizer(args.body_name+'_optimizer', link)
    tracker.AddOptimizer(optimizer)

    ok = tracker.SetUp()
    if not ok:
        raise ValueError('tracker SetUp failed')

    if args.use_depth:
        return tracker, optimizer, body, color_camera, depth_camera, color_viewer, depth_viewer
    else:
        return tracker, optimizer, body, color_camera, color_viewer
