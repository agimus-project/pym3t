// EIGEN
#include <Eigen/Dense>
#include <Eigen/Geometry>

// PYBIND11
// core
#include <pybind11/pybind11.h>
// implicit type conversions
#include <pybind11/eigen.h>
#include <pybind11/chrono.h>
#include <pybind11/stl/filesystem.h>

// ICG
#include <icg/common.h>
#include <icg/camera.h>
#include <icg/realsense_camera.h>
#include <icg/renderer_geometry.h>
#include <icg/basic_depth_renderer.h>
#include <icg/body.h>
#include <icg/common.h>
#include <icg/depth_modality.h>
#include <icg/depth_model.h>
#include <icg/normal_viewer.h>
#include <icg/region_modality.h>
#include <icg/region_model.h>
#include <icg/static_detector.h>
#include <icg/tracker.h>

// PYICG
#include "pyicg/type_caster_utils.h"
#include "pyicg/dummy_camera.h"

namespace py = pybind11;
// to be able to use "arg"_a shorthand
using namespace pybind11::literals;
using namespace icg;
using namespace Eigen;

/**
 * TODO: 
 * - Read the flag USE_REALSENSE to decide whether or not to create bindings
 * */ 



PYBIND11_MODULE(_pyicg_mod, m) {

    ///////////////////////
    // Classes
    ///////////////////////

    py::class_<Tracker>(m, "Tracker")
        .def(py::init<const std::string, int, int, bool, 
                      const std::chrono::milliseconds&, int, int>(), 
                      "name"_a, "n_corr_iterations"_a=5, "n_update_iterations"_a=2, "synchronize_cameras"_a=true, 
                      "cycle_duration"_a=std::chrono::milliseconds{33}, "visualization_time"_a=0, "viewer_time"_a=1)
        .def("SetUp", &Tracker::SetUp, "set_up_all_objects"_a=true)
        .def("RunTrackerProcess", &Tracker::RunTrackerProcess, "execute_detection"_a=true, "start_tracking"_a=true)
        .def("ExecuteDetectionCycle", &Tracker::ExecuteDetectionCycle, "iteration"_a=0, "Run all detectors, iteration arg is not used")
        .def("StartModalities", &Tracker::StartModalities, "iteration"_a)
        .def("ExecuteTrackingCycle", &Tracker::ExecuteTrackingCycle, "iteration"_a)
        .def("UpdateViewers", &Tracker::UpdateViewers, "iteration"_a)
        .def("UpdateCameras", &Tracker::UpdateCameras)
        .def("AddViewer", &Tracker::AddViewer)
        .def("AddDetector", &Tracker::AddDetector)
        .def("AddOptimizer", &Tracker::AddOptimizer)
        .def("DetectBodies", &Tracker::DetectBodies)

        .def_property("n_corr_iterations", &Tracker::n_corr_iterations, &Tracker::set_n_corr_iterations)
        .def_property("n_update_iterations", &Tracker::n_update_iterations, &Tracker::set_n_update_iterations)
        ;

    // RendererGeometry
    py::class_<icg::RendererGeometry, std::shared_ptr<icg::RendererGeometry>>(m, "RendererGeometry")
        .def(py::init<const std::string &>(), "name"_a)
        .def("AddBody", &RendererGeometry::AddBody)
        .def("DeleteBody", &RendererGeometry::DeleteBody)
        .def("ClearBodies", &RendererGeometry::ClearBodies)
        ;

    // Stores camera intrinsics parameters
    py::class_<icg::Intrinsics>(m, "Intrinsics")
        .def(py::init<float, float, float, float, int, int>(), "fu"_a, "fv"_a, "ppu"_a, "ppv"_a, "width"_a, "height"_a)
        .def_readwrite("fu", &Intrinsics::fu)
        .def_readwrite("fv", &Intrinsics::fv)
        .def_readwrite("ppu", &Intrinsics::ppu)
        .def_readwrite("ppv", &Intrinsics::ppv)
        .def_readwrite("width", &Intrinsics::width)
        .def_readwrite("height", &Intrinsics::height)
        ;

    ///
    class PyCamera: public icg::Camera {
        public:
            // Inherit the base class constructor
            using icg::Camera::Camera;

            bool SetUp() override {
                PYBIND11_OVERRIDE_PURE( bool, Camera, SetUp);}

            bool UpdateImage(bool synchronized) override {
                PYBIND11_OVERRIDE_PURE( bool, Camera, UpdateImage, synchronized);}
    };

    // Camera -> not constructible, just to be able to bind RealSenseColorCamera and enable automatic downcasting
    py::class_<icg::Camera, PyCamera, std::shared_ptr<icg::Camera>>(m, "Camera")
        .def("SetUp", &icg::Camera::SetUp)
        ;

    // ColorCamera -> not constructible, just to be able to bind RealSenseColorCamera
    py::class_<icg::ColorCamera, icg::Camera, std::shared_ptr<icg::ColorCamera>>(m, "ColorCamera");

    // DepthCamera -> not constructible, just to be able to bind RealSenseDepthCamera
    py::class_<icg::DepthCamera, icg::Camera, std::shared_ptr<icg::DepthCamera>>(m, "DepthCamera");

    // RealSenseColorCamera
    py::class_<icg::RealSenseColorCamera, icg::ColorCamera, std::shared_ptr<icg::RealSenseColorCamera>>(m, "RealSenseColorCamera")
        .def(py::init<const std::string &, bool>(), "name"_a, "use_depth_as_world_frame"_a=false)
        ;

    // RealSenseDepthCamera
    py::class_<icg::RealSenseDepthCamera, icg::DepthCamera, std::shared_ptr<icg::RealSenseDepthCamera>>(m, "RealSenseDepthCamera")
        .def(py::init<const std::string &, bool>(), "name"_a, "use_color_as_world_frame"_a=true)
        ;

    // DummyColorCamera
    py::class_<icg::DummyColorCamera, icg::ColorCamera, std::shared_ptr<icg::DummyColorCamera>>(m, "DummyColorCamera")
        .def(py::init<const std::string &, bool>(), "name"_a, "use_depth_as_world_frame"_a=false)
        .def_property("image", &icg::Camera::image, &icg::DummyColorCamera::set_image)
        .def_property("intrinsics", &icg::DummyColorCamera::get_intrinsics, &icg::DummyColorCamera::set_intrinsics)
        .def_property("color2depth_pose", &icg::DummyColorCamera::get_color2depth_pose, &icg::DummyColorCamera::set_color2depth_pose)
        .def_property("depth2color_pose", &icg::DummyColorCamera::get_depth2color_pose, &icg::DummyColorCamera::set_depth2color_pose)
        ;

    // DummyDepthCamera
    py::class_<icg::DummyDepthCamera, icg::DepthCamera, std::shared_ptr<icg::DummyDepthCamera>>(m, "DummyDepthCamera")
        .def(py::init<const std::string &, bool>(), "name"_a, "use_color_as_world_frame"_a=true)
        .def_property("image", &icg::Camera::image, &icg::DummyDepthCamera::set_image)
        .def_property("intrinsics", &icg::DummyDepthCamera::get_intrinsics, &icg::DummyDepthCamera::set_intrinsics)
        .def_property("color2depth_pose", &icg::DummyDepthCamera::get_color2depth_pose, &icg::DummyDepthCamera::set_color2depth_pose)
        .def_property("depth2color_pose", &icg::DummyDepthCamera::get_depth2color_pose, &icg::DummyDepthCamera::set_depth2color_pose)
        .def_property("depth_scale", &icg::DummyDepthCamera::depth_scale, &icg::DummyDepthCamera::set_depth_scale)
        ;

    ///
    class PyViewer: public icg::Viewer {
        public:
            // Inherit the base class constructor
            using icg::Viewer::Viewer;

            bool SetUp() override {
                PYBIND11_OVERRIDE_PURE( bool, Viewer, SetUp);}

            bool UpdateViewer(int save_index) override {
                PYBIND11_OVERRIDE_PURE( bool, Viewer, UpdateViewer, save_index);}
    };

    // Viewer
    py::class_<Viewer, PyViewer, std::shared_ptr<icg::Viewer>>(m, "Viewer")
        .def("StartSavingImages", &PyViewer::StartSavingImages, "save_directory"_a, "save_image_type"_a)
        .def("StopSavingImages", &PyViewer::StopSavingImages)
        .def_property("display_images", &PyViewer::display_images, &PyViewer::set_display_images)
        ;
    
    // NormalColorViewer
    py::class_<NormalColorViewer, Viewer, std::shared_ptr<icg::NormalColorViewer>>(m, "NormalColorViewer")
        .def(py::init<const std::string &, const std::shared_ptr<ColorCamera> &, const std::shared_ptr<RendererGeometry> &, float>(),
                      "name"_a, "color_camera_ptr"_a, "renderer_geometry_ptr"_a, "opacity"_a=0.5f)
        .def("SetUp", &NormalColorViewer::SetUp)
        .def("UpdateViewer", &NormalColorViewer::UpdateViewer, "save_index"_a)
        .def("set_opacity", &NormalColorViewer::set_opacity, "opacity"_a)
        ;

    // NormalDepthViewer
    py::class_<NormalDepthViewer, Viewer, std::shared_ptr<icg::NormalDepthViewer>>(m, "NormalDepthViewer")
        .def(py::init<const std::string &, const std::shared_ptr<DepthCamera> &, const std::shared_ptr<RendererGeometry> &, float, float, float>(),
                      "name"_a, "depth_camera_ptr"_a, "renderer_geometry_ptr"_a, "min_depth"_a=0.0f, "max_depth"_a=1.0f, "opacity"_a=0.5f)
        ;

    py::class_<FocusedBasicDepthRenderer>(m, "FocusedBasicDepthRenderer")
        // .def(py::init<const std::string &, const std::shared_ptr<RendererGeometry> &, const Transform3fA &, const Intrinsics &, int, float, float>(),
        //               "name"_a, "renderer_geometry_ptr"_a, "world2camera_pose"_a, "intrinsics"_a, "image_size"_a=200, "z_min"_a=0.01f, "z_max"_a=5.0f)       
        .def(py::init<const std::string &, const std::shared_ptr<RendererGeometry> &, const std::shared_ptr<Camera> &, int, float, float>(),
                      "name"_a, "renderer_geometry_ptr"_a, "camera_ptr"_a, "image_size"_a=200, "z_min"_a=0.01f, "z_max"_a=5.0f)
        .def("AddReferencedBody", &FocusedBasicDepthRenderer::AddReferencedBody)
        ;
    
    // Body
    py::class_<Body, std::shared_ptr<icg::Body>>(m, "Body")
        .def(py::init<const std::string &, const std::filesystem::path &, float, bool, bool, const Transform3fA &, uchar>(),
                      "name"_a, "geometry_path"_a, "geometry_unit_in_meter"_a, "geometry_counterclockwise"_a, "geometry_enable_culling"_a, "geometry2body_pose"_a, "silhouette_id"_a=0)
        .def(py::init<const std::string &, const std::filesystem::path &>(), "name"_a, "metafile_path"_a)
        .def_property("name", &Body::name, &Body::set_name)
        .def_property("geometry_path", &Body::geometry_path, &Body::set_geometry_path)
        .def_property("metafile_path", &Body::metafile_path, &Body::set_metafile_path)
        .def_property("geometry_unit_in_meter", &Body::geometry_unit_in_meter, &Body::set_geometry_unit_in_meter)
        .def_property("geometry_counterclockwise", &Body::geometry_counterclockwise, &Body::set_geometry_counterclockwise)
        .def_property("geometry_enable_culling", &Body::geometry_enable_culling, &Body::set_geometry_enable_culling)
        .def_property("geometry2body_pose", &Body::geometry2body_pose, &Body::set_geometry2body_pose)
        .def_property("body2world_pose", &Body::body2world_pose, &Body::set_body2world_pose)
        .def_property("world2body_pose", &Body::world2body_pose, &Body::set_world2body_pose)
        ;

    ///
    class PyDetector: public icg::Detector {
        public:
            // Inherit the base class constructor
            using icg::Detector::Detector;

            bool SetUp() override {
                PYBIND11_OVERRIDE_PURE( bool, Detector, SetUp);}

            bool DetectBody() override {
                PYBIND11_OVERRIDE_PURE( bool, Detector, DetectBody);}
    };

    // Detector
    py::class_<Detector, PyDetector, std::shared_ptr<icg::Detector>>(m, "Detector");


    // StaticDetector
    py::class_<StaticDetector, Detector, std::shared_ptr<icg::StaticDetector>>(m, "StaticDetector")
        .def(py::init<const std::string &, const std::shared_ptr<Body> &, const Transform3fA &>(),
                      "name"_a, "body_ptr"_a, "body2world_pose"_a)
        .def(py::init<const std::string &, const std::filesystem::path &, const std::shared_ptr<icg::Body> &>(),
                      "name"_a, "metafile_path"_a, "body_ptr"_a)
        .def_property("body2world_pose", &StaticDetector::body2world_pose, &StaticDetector::set_body2world_pose)
        ;


    // RegionModel
    py::class_<RegionModel, std::shared_ptr<icg::RegionModel>>(m, "RegionModel")
        .def(py::init<const std::string &, const std::shared_ptr<Body> &, const std::filesystem::path &, 
                      float, int, int, float, float, bool, int>(),
                      "name"_a, "body_ptr"_a, "model_path"_a, 
                      "sphere_radius"_a=0.8f, "n_divides"_a=4, "n_points"_a=200, "max_radius_depth_offset"_a=0.05f, "stride_depth_offset"_a=0.002f, "use_random_seed"_a=false, "image_size"_a=2000)
        ;

    // DepthModel
    py::class_<DepthModel, std::shared_ptr<icg::DepthModel>>(m, "DepthModel")
        .def(py::init<const std::string &, const std::shared_ptr<Body> &, const std::filesystem::path &, 
                      float, int, int, float, float, bool, int>(),
                      "name"_a, "body_ptr"_a, "model_path"_a, 
                      "sphere_radius"_a=0.8f, "n_divides"_a=4, "n_points"_a=200, "max_radius_depth_offset"_a=0.05f, "stride_depth_offset"_a=0.002f, "use_random_seed"_a=false, "image_size"_a=2000)
        ;



    ///
    class PyModality: public icg::Modality {
        public:
            // Inherit the base class constructor
            using icg::Modality::Modality;

            bool SetUp() override {
                PYBIND11_OVERRIDE_PURE(bool, icg::Modality, SetUp);}
            bool StartModality(int iteration, int corr_iteration) override {
                PYBIND11_OVERRIDE_PURE(bool, icg::Modality, StartModality, iteration, corr_iteration);}
            bool CalculateCorrespondences(int iteration, int corr_iteration) override {
                PYBIND11_OVERRIDE_PURE(bool, icg::Modality, CalculateCorrespondences, iteration, corr_iteration);}
            bool VisualizeCorrespondences(int save_idx) override {
                PYBIND11_OVERRIDE_PURE(bool, icg::Modality, VisualizeCorrespondences, save_idx);}
            bool CalculateGradientAndHessian(int iteration, int corr_iteration, int opt_iteration) override {
                PYBIND11_OVERRIDE_PURE(bool, icg::Modality, CalculateGradientAndHessian, iteration, corr_iteration, opt_iteration);}
            bool VisualizeOptimization(int save_idx) override {
                PYBIND11_OVERRIDE_PURE(bool, icg::Modality, VisualizeOptimization, save_idx);}
            bool CalculateResults(int iteration) override {
                PYBIND11_OVERRIDE_PURE(bool, icg::Modality, CalculateResults, iteration);}
            bool VisualizeResults(int save_idx) override {
                PYBIND11_OVERRIDE_PURE(bool, icg::Modality, VisualizeResults, save_idx);}

    };

    py::class_<Modality, PyModality, std::shared_ptr<icg::Modality>>(m, "Modality");

    // RegionModality
    py::class_<RegionModality, Modality, std::shared_ptr<icg::RegionModality>>(m, "RegionModality")
        .def(py::init<const std::string &, const std::shared_ptr<Body> &, const std::shared_ptr<ColorCamera> &, const std::shared_ptr<RegionModel> &>(),
                      "name"_a, "body_ptr"_a, "color_camera_ptr"_a, "region_model_ptr"_a)
        .def_property("visualize_pose_result", &RegionModality::visualize_pose_result, &RegionModality::set_visualize_pose_result)
        .def_property("visualize_lines_correspondence", &RegionModality::visualize_lines_correspondence, &RegionModality::set_visualize_lines_correspondence)
        .def_property("visualize_points_correspondence", &RegionModality::visualize_points_correspondence, &RegionModality::set_visualize_points_correspondence)
        .def_property("visualize_points_depth_image_correspondence", &RegionModality::visualize_points_depth_image_correspondence, &RegionModality::set_visualize_points_depth_image_correspondence)
        .def_property("visualize_points_depth_rendering_correspondence", &RegionModality::visualize_points_depth_rendering_correspondence, &RegionModality::set_visualize_points_depth_rendering_correspondence)
        .def_property("visualize_points_result", &RegionModality::visualize_points_result, &RegionModality::set_visualize_points_result)
        .def_property("visualize_points_histogram_image_result", &RegionModality::visualize_points_histogram_image_result, &RegionModality::set_visualize_points_histogram_image_result)
        .def_property("visualize_points_histogram_image_optimization", &RegionModality::visualize_points_histogram_image_optimization, &RegionModality::set_visualize_points_histogram_image_optimization)
        .def_property("visualize_points_optimization", &RegionModality::visualize_points_optimization, &RegionModality::set_visualize_points_optimization)
        .def_property("visualize_gradient_optimization", &RegionModality::visualize_gradient_optimization, &RegionModality::set_visualize_gradient_optimization)
        .def_property("visualize_hessian_optimization", &RegionModality::visualize_hessian_optimization, &RegionModality::set_visualize_hessian_optimization)
        .def("MeasureOcclusions", &RegionModality::MeasureOcclusions)
        ;


    // DepthModality
    py::class_<DepthModality, Modality, std::shared_ptr<icg::DepthModality>>(m, "DepthModality")
        .def(py::init<const std::string &, const std::shared_ptr<Body> &, const std::shared_ptr<DepthCamera> &, const std::shared_ptr<DepthModel> &>(),
                      "name"_a, "body_ptr"_a, "depth_camera_ptr"_a, "depth_model_ptr"_a)
        .def("MeasureOcclusions", &DepthModality::MeasureOcclusions)
        ;

    // Optimizer
    py::class_<Optimizer, std::shared_ptr<icg::Optimizer>>(m, "Optimizer")
        .def(py::init<const std::string &, float, float>(),
                      "name"_a, "tikhonov_parameter_rotation"_a=1000.0f, "tikhonov_parameter_translation"_a=30000.0f)
        .def(py::init<const std::string &, const std::filesystem::path &>(),
                      "name"_a, "metafile_path"_a)
        .def_property("name", &Optimizer::name, &Optimizer::set_name)
        .def_property("metafile_path", &Optimizer::metafile_path, &Optimizer::set_metafile_path)
        .def_property("tikhonov_parameter_rotation", &Optimizer::tikhonov_parameter_rotation, &Optimizer::set_tikhonov_parameter_rotation)
        .def_property("tikhonov_parameter_translation", &Optimizer::tikhonov_parameter_translation, &Optimizer::set_tikhonov_parameter_translation)
        .def("AddModality", &Optimizer::AddModality)
        ;

}   


