// EIGEN
#include <Eigen/Dense>
#include <Eigen/Geometry>

// PYBIND11
// core
#include <pybind11/pybind11.h>
// implicit type conversions
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <pybind11/chrono.h>
#include <pybind11/stl/filesystem.h>
// ICG
#include <m3t/common.h>
#include <m3t/camera.h>
#include <m3t/realsense_camera.h>
#include <m3t/renderer_geometry.h>
#include <m3t/basic_depth_renderer.h>
#include <m3t/silhouette_renderer.h>
#include <m3t/body.h>
#include <m3t/link.h>
#include <m3t/region_model.h>
#include <m3t/depth_model.h>
#include <m3t/region_modality.h>
#include <m3t/depth_modality.h>
#include <m3t/texture_modality.h>
#include <m3t/normal_viewer.h>
#include <m3t/static_detector.h>
#include <m3t/tracker.h>

// // pym3t
#include "pym3t/type_caster_utils.h"
#include "pym3t/dummy_camera.h"

namespace py = pybind11;
// to be able to use "arg"_a shorthand
using namespace pybind11::literals;
using namespace m3t;
using namespace Eigen;

/**
 * TODO: 
 * - Read the flag USE_REALSENSE to decide whether or not to create bindings
 * */ 



PYBIND11_MODULE(_pym3t_mod, m) {

    ///////////////////////
    // Classes
    ///////////////////////

    py::class_<Tracker>(m, "Tracker")
        .def(py::init<const std::string, int, int, bool, bool,
                      const std::chrono::milliseconds&, int, int>(), 
                      "name"_a, "n_corr_iterations"_a=5, "n_update_iterations"_a=2, "synchronize_cameras"_a=true, "start_tracking_after_detection"_a=false,
                      "cycle_duration"_a=std::chrono::milliseconds{33}, "visualization_time"_a=0, "viewer_time"_a=1)

        .def("SetUp", &Tracker::SetUp, "set_up_all_objects"_a=true)
        .def("RunTrackerProcess", &Tracker::RunTrackerProcess, "execute_detection"_a=true, "start_tracking"_a=true, "names_detecting"_a=nullptr, "names_starting"_a=nullptr)
        .def("ExecuteDetectingStep", &Tracker::ExecuteDetectingStep, "iteration"_a=0, "Run all detectors, iteration arg is not used")
        .def("ExecuteDetectingStep", &Tracker::ExecuteDetectingStep, "iteration"_a=0, "Run all detectors, iteration arg is not used")
        .def("ExecuteTrackingStep", &Tracker::ExecuteTrackingStep, "iteration"_a)
        .def("ExecuteStartingStep", &Tracker::ExecuteStartingStep, "iteration"_a)
        .def("StartModalities", &Tracker::StartModalities, "iteration"_a)
        .def("UpdateViewers", &Tracker::UpdateViewers, "iteration"_a)
        .def("UpdateCameras", &Tracker::UpdateCameras)
        .def("AddViewer", &Tracker::AddViewer)
        .def("AddDetector", &Tracker::AddDetector)
        .def("AddOptimizer", &Tracker::AddOptimizer)
        .def_property("n_corr_iterations", &Tracker::n_corr_iterations, &Tracker::set_n_corr_iterations)
        .def_property("n_update_iterations", &Tracker::n_update_iterations, &Tracker::set_n_update_iterations)
        ;



    // Stores camera intrinsics parameters
    py::class_<Intrinsics>(m, "Intrinsics")
        .def(py::init<float, float, float, float, int, int>(), "fu"_a, "fv"_a, "ppu"_a, "ppv"_a, "width"_a, "height"_a)
        .def_readwrite("fu", &Intrinsics::fu)
        .def_readwrite("fv", &Intrinsics::fv)
        .def_readwrite("ppu", &Intrinsics::ppu)
        .def_readwrite("ppv", &Intrinsics::ppv)
        .def_readwrite("width", &Intrinsics::width)
        .def_readwrite("height", &Intrinsics::height)
        ;

    py::enum_<IDType>(m, "IDType")
        .value("BODY", IDType::BODY)
        .value("REGION", IDType::REGION)
        .export_values();

    ///
    class PyCamera: public Camera {
        public:
            // Inherit the base class constructor
            using Camera::Camera;

            bool SetUp() override {
                PYBIND11_OVERRIDE_PURE( bool, Camera, SetUp);}

            bool UpdateImage(bool synchronized) override {
                PYBIND11_OVERRIDE_PURE( bool, Camera, UpdateImage, synchronized);}
    };

    // Camera -> not constructible, just to enable automatic downcasting and binding of child classes
    py::class_<Camera, PyCamera, std::shared_ptr<Camera>>(m, "Camera")
        .def("SetUp", &Camera::SetUp)
        .def_property("camera2world_pose", &Camera::camera2world_pose, &Camera::set_camera2world_pose)
        .def_property("world2camera_pose", &Camera::world2camera_pose, &Camera::set_world2camera_pose)
        ;

    // ColorCamera -> not constructible, just to enable automatic downcasting and binding of child classes
    py::class_<ColorCamera, Camera, std::shared_ptr<ColorCamera>>(m, "ColorCamera");

    // DepthCamera -> not constructible, just to enable automatic downcasting and binding of child classes
    py::class_<DepthCamera, Camera, std::shared_ptr<DepthCamera>>(m, "DepthCamera");

    // RealSenseColorCamera
    py::class_<RealSenseColorCamera, ColorCamera, std::shared_ptr<RealSenseColorCamera>>(m, "RealSenseColorCamera")
        .def(py::init<const std::string &, bool>(), "name"_a, "use_depth_as_world_frame"_a=false)
        ;

    // RealSenseDepthCamera
    py::class_<RealSenseDepthCamera, DepthCamera, std::shared_ptr<RealSenseDepthCamera>>(m, "RealSenseDepthCamera")
        .def(py::init<const std::string &, bool>(), "name"_a, "use_color_as_world_frame"_a=true)
        ;

    // DummyColorCamera
    py::class_<DummyColorCamera, ColorCamera, std::shared_ptr<DummyColorCamera>>(m, "DummyColorCamera")
        .def(py::init<const std::string &, bool>(), "name"_a, "use_depth_as_world_frame"_a=false)
        .def_property("image", &Camera::image, &DummyColorCamera::set_image)
        .def_property("intrinsics", &DummyColorCamera::get_intrinsics, &DummyColorCamera::set_intrinsics)
        .def_property("color2depth_pose", &DummyColorCamera::get_color2depth_pose, &DummyColorCamera::set_color2depth_pose)
        .def_property("depth2color_pose", &DummyColorCamera::get_depth2color_pose, &DummyColorCamera::set_depth2color_pose)
        ;

    // DummyDepthCamera
    py::class_<DummyDepthCamera, DepthCamera, std::shared_ptr<DummyDepthCamera>>(m, "DummyDepthCamera")
        .def(py::init<const std::string &, bool, float>(), "name"_a, "use_color_as_world_frame"_a=true, "depth_scale"_a=0.001)
        .def_property("image", &Camera::image, &DummyDepthCamera::set_image)
        .def_property("intrinsics", &DummyDepthCamera::get_intrinsics, &DummyDepthCamera::set_intrinsics)
        .def_property("color2depth_pose", &DummyDepthCamera::get_color2depth_pose, &DummyDepthCamera::set_color2depth_pose)
        .def_property("depth2color_pose", &DummyDepthCamera::get_depth2color_pose, &DummyDepthCamera::set_depth2color_pose)
        .def_property("depth_scale", &DummyDepthCamera::depth_scale, &DummyDepthCamera::set_depth_scale)
        ;


    // RendererGeometry
    py::class_<RendererGeometry, std::shared_ptr<RendererGeometry>>(m, "RendererGeometry")
        .def(py::init<const std::string &>(), "name"_a)
        .def("AddBody", &RendererGeometry::AddBody)
        .def("DeleteBody", &RendererGeometry::DeleteBody)
        .def("ClearBodies", &RendererGeometry::ClearBodies)
        ;

    ///
    class PyViewer: public Viewer {
        public:
            // Inherit the base class constructor
            using Viewer::Viewer;

            bool SetUp() override {
                PYBIND11_OVERRIDE_PURE( bool, Viewer, SetUp);}

            bool UpdateViewer(int save_index) override {
                PYBIND11_OVERRIDE_PURE( bool, Viewer, UpdateViewer, save_index);}
    };

    // Viewer
    py::class_<Viewer, PyViewer, std::shared_ptr<Viewer>>(m, "Viewer")
        .def("StartSavingImages", &PyViewer::StartSavingImages, "save_directory"_a, "save_image_type"_a)
        .def("StopSavingImages", &PyViewer::StopSavingImages)
        .def("save_images", &PyViewer::save_images)
        .def_property("display_images", &PyViewer::display_images, &PyViewer::set_display_images)
        ;
    
    // NormalColorViewer
    py::class_<NormalColorViewer, Viewer, std::shared_ptr<NormalColorViewer>>(m, "NormalColorViewer")
        .def(py::init<const std::string &, const std::shared_ptr<ColorCamera> &, const std::shared_ptr<RendererGeometry> &, float>(),
                      "name"_a, "color_camera_ptr"_a, "renderer_geometry_ptr"_a, "opacity"_a=0.5f)
        .def("SetUp", &NormalColorViewer::SetUp)
        .def("UpdateViewer", &NormalColorViewer::UpdateViewer, "save_index"_a)
        .def("set_opacity", &NormalColorViewer::set_opacity, "opacity"_a)
        ;

    // NormalDepthViewer
    py::class_<NormalDepthViewer, Viewer, std::shared_ptr<NormalDepthViewer>>(m, "NormalDepthViewer")
        .def(py::init<const std::string &, const std::shared_ptr<DepthCamera> &, const std::shared_ptr<RendererGeometry> &, float, float, float>(),
                      "name"_a, "depth_camera_ptr"_a, "renderer_geometry_ptr"_a, "min_depth"_a=0.0f, "max_depth"_a=1.0f, "opacity"_a=0.5f)
        ;

    /**
     * Renderers for occlusion handling
     * */ 
    py::class_<FocusedBasicDepthRenderer, std::shared_ptr<FocusedBasicDepthRenderer>>(m, "FocusedBasicDepthRenderer")
        // .def(py::init<const std::string &, const std::shared_ptr<RendererGeometry> &, const Transform3fA &, const Intrinsics &, int, float, float>(),
        //               "name"_a, "renderer_geometry_ptr"_a, "world2camera_pose"_a, "intrinsics"_a, "image_size"_a=200, "z_min"_a=0.01f, "z_max"_a=5.0f)       
        .def(py::init<const std::string &, const std::shared_ptr<RendererGeometry> &, const std::shared_ptr<Camera> &, int, float, float>(),
                      "name"_a, "renderer_geometry_ptr"_a, "camera_ptr"_a, "image_size"_a=200, "z_min"_a=0.01f, "z_max"_a=5.0f)
        .def("AddReferencedBody", &FocusedBasicDepthRenderer::AddReferencedBody)
        ;

    /**
     * Renderer for TextureModality
     * */ 
    py::class_<FocusedSilhouetteRenderer, std::shared_ptr<FocusedSilhouetteRenderer>>(m, "FocusedSilhouetteRenderer")
        // .def(py::init<const std::string &, const std::shared_ptr<RendererGeometry> &, const Transform3fA &, const Intrinsics &, IDType, int, float, float>(),
        //               "name"_a, "renderer_geometry_ptr"_a, "world2camera_pose"_a, "intrinsics"_a, "id_type"_a=IDType::BODY, "image_size"_a=200 "z_min"_a=0.01f, "z_max"_a=5.0f)       
        .def(py::init<const std::string &, const std::shared_ptr<RendererGeometry> &, const std::shared_ptr<Camera>&, IDType, int, float, float>(),
                      "name"_a, "renderer_geometry_ptr"_a, "camera_ptr"_a, "id_type"_a=IDType::BODY, "image_size"_a=200, "z_min"_a=0.01f, "z_max"_a=5.0f)       
        .def(py::init<const std::string &, const std::filesystem::path&, const std::shared_ptr<RendererGeometry> &, const std::shared_ptr<Camera>&>(),
                      "name"_a, "metafile_path"_a, "renderer_geometry_ptr"_a, "camera_ptr"_a)       
        .def("AddReferencedBody", &FocusedSilhouetteRenderer::AddReferencedBody)
        ;


    // Body
    py::class_<Body, std::shared_ptr<Body>>(m, "Body")
      // Constructors and initialization methods
        .def(py::init<const std::string &, const std::filesystem::path &, float, bool, bool, const Transform3fA &>(),
                      "name"_a, "geometry_path"_a, "geometry_unit_in_meter"_a, "geometry_counterclockwise"_a, "geometry_enable_culling"_a, "geometry2body_pose"_a)
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

    py::class_<Link, std::shared_ptr<Link>>(m, "Link")
        .def(py::init<const std::string &, const std::shared_ptr<Body> &, 
                      const Transform3fA &, const Transform3fA &, const Transform3fA &, 
                      const std::array<bool, 6> &, bool>(), 
                      "name"_a, "body_ptr"_a,
                      "body2joint_pose"_a=Transform3fA::Identity(), "joint2parent_pose"_a=Transform3fA::Identity(), "link2world_pose"_a=Transform3fA::Identity(),
                      "free_directions"_a=std::array<bool, 6>({true, true, true, true, true, true}), "fixed_body2joint_pose"_a=true)
        .def(py::init<const std::string &, const std::filesystem::path &, const std::shared_ptr<Body> &>(), "name"_a, "metafile_path"_a, "body_ptr"_a)
        .def("AddModality", &Link::AddModality)
    ;
    

    ///
    class PyDetector: public Detector {
        public:
            // Inherit the base class constructor
            using Detector::Detector;

            bool SetUp() override {
                PYBIND11_OVERRIDE_PURE( bool, Detector, SetUp);}

            // bool DetectBody() override {
            //     PYBIND11_OVERRIDE_PURE( bool, Detector, DetectBody);}
    };

    // Detector
    py::class_<Detector, PyDetector, std::shared_ptr<Detector>>(m, "Detector");

    // StaticDetector
    py::class_<StaticDetector, Detector, std::shared_ptr<StaticDetector>>(m, "StaticDetector")
      // Constructor and setup method
        .def(py::init<const std::string &, const std::shared_ptr<Optimizer> &, const Transform3fA &, bool>(),
                      "name"_a, "optimizer_ptr"_a, "link2world_pose"_a, "reset_joint_poses"_a)
        .def(py::init<const std::string &, const std::filesystem::path &, const std::shared_ptr<Optimizer> &>(),
                      "name"_a, "metafile_path"_a, "optimizer_ptr"_a)
        .def("SetUp", &StaticDetector::SetUp)
        .def_property("link2world_pose_", &StaticDetector::link2world_pose, &StaticDetector::set_link2world_pose)
        ;


    // RegionModel
    py::class_<RegionModel, std::shared_ptr<RegionModel>>(m, "RegionModel")
        .def(py::init<const std::string &, const std::shared_ptr<Body> &, const std::filesystem::path &, 
                      float, int, int, float, float, bool, int>(),
                      "name"_a, "body_ptr"_a, "model_path"_a, 
                      "sphere_radius"_a=0.8f, "n_divides"_a=4, "n_points_max"_a=200, "max_radius_depth_offset"_a=0.05f, "stride_depth_offset"_a=0.002f, "use_random_seed"_a=false, "image_size"_a=2000)
        ;

    // DepthModel
    py::class_<DepthModel, std::shared_ptr<DepthModel>>(m, "DepthModel")
        .def(py::init<const std::string &, const std::shared_ptr<Body> &, const std::filesystem::path &, 
                      float, int, int, float, float, bool, int>(),
                      "name"_a, "body_ptr"_a, "model_path"_a, 
                      "sphere_radius"_a=0.8f, "n_divides"_a=4, "n_points_max"_a=200, "max_radius_depth_offset"_a=0.05f, "stride_depth_offset"_a=0.002f, "use_random_seed"_a=false, "image_size"_a=2000)
        ;



    ///
    class PyModality: public Modality {
        public:
            // Inherit the base class constructor
            using Modality::Modality;

            bool SetUp() override {
                PYBIND11_OVERRIDE_PURE(bool, Modality, SetUp);}
            bool StartModality(int iteration, int corr_iteration) override {
                PYBIND11_OVERRIDE_PURE(bool, Modality, StartModality, iteration, corr_iteration);}
            bool CalculateCorrespondences(int iteration, int corr_iteration) override {
                PYBIND11_OVERRIDE_PURE(bool, Modality, CalculateCorrespondences, iteration, corr_iteration);}
            bool VisualizeCorrespondences(int save_idx) override {
                PYBIND11_OVERRIDE_PURE(bool, Modality, VisualizeCorrespondences, save_idx);}
            bool CalculateGradientAndHessian(int iteration, int corr_iteration, int opt_iteration) override {
                PYBIND11_OVERRIDE_PURE(bool, Modality, CalculateGradientAndHessian, iteration, corr_iteration, opt_iteration);}
            bool VisualizeOptimization(int save_idx) override {
                PYBIND11_OVERRIDE_PURE(bool, Modality, VisualizeOptimization, save_idx);}
            bool CalculateResults(int iteration) override {
                PYBIND11_OVERRIDE_PURE(bool, Modality, CalculateResults, iteration);}
            bool VisualizeResults(int save_idx) override {
                PYBIND11_OVERRIDE_PURE(bool, Modality, VisualizeResults, save_idx);}

    };

    py::class_<Modality, PyModality, std::shared_ptr<Modality>>(m, "Modality");

    // RegionModality
    py::class_<RegionModality, Modality, std::shared_ptr<RegionModality>>(m, "RegionModality")
        .def(py::init<const std::string &, const std::shared_ptr<Body> &, const std::shared_ptr<ColorCamera> &, const std::shared_ptr<RegionModel> &>(),
                      "name"_a, "body_ptr"_a, "color_camera_ptr"_a, "region_model_ptr"_a)
        .def_property("n_lines_max", &RegionModality::n_lines_max, &RegionModality::set_n_lines_max)
        .def_property("use_adaptive_coverage", &RegionModality::use_adaptive_coverage, &RegionModality::set_use_adaptive_coverage)
        .def_property("reference_contour_length", &RegionModality::reference_contour_length, &RegionModality::set_reference_contour_length)
        .def_property("min_continuous_distance", &RegionModality::min_continuous_distance, &RegionModality::set_min_continuous_distance)
        .def_property("function_length", &RegionModality::function_length, &RegionModality::set_function_length)
        .def_property("distribution_length", &RegionModality::distribution_length, &RegionModality::set_distribution_length)
        .def_property("function_amplitude", &RegionModality::function_amplitude, &RegionModality::set_function_amplitude)
        .def_property("function_slope", &RegionModality::function_slope, &RegionModality::set_function_slope)
        .def_property("learning_rate", &RegionModality::learning_rate, &RegionModality::set_learning_rate)
        .def_property("n_global_iterations", &RegionModality::n_global_iterations, &RegionModality::set_n_global_iterations)
        .def_property("scales", &RegionModality::scales, &RegionModality::set_scales)
        .def_property("standard_deviations", &RegionModality::standard_deviations, &RegionModality::set_standard_deviations)

        .def_property("n_histogram_bins", &RegionModality::n_histogram_bins, &RegionModality::set_n_histogram_bins)
        .def_property("learning_rate_f", &RegionModality::learning_rate_f, &RegionModality::set_learning_rate_f)
        .def_property("learning_rate_b", &RegionModality::learning_rate_b, &RegionModality::set_learning_rate_b)
        .def_property("unconsidered_line_length", &RegionModality::unconsidered_line_length, &RegionModality::set_unconsidered_line_length)
        .def_property("max_considered_line_length", &RegionModality::max_considered_line_length, &RegionModality::set_max_considered_line_length)

        .def("ModelOcclusions", &RegionModality::ModelOcclusions)
        .def("MeasureOcclusions", &RegionModality::MeasureOcclusions)

        .def_property("measured_depth_offset_radius", &RegionModality::measured_depth_offset_radius, &RegionModality::set_measured_depth_offset_radius)
        .def_property("measured_occlusion_radius", &RegionModality::measured_occlusion_radius, &RegionModality::set_measured_occlusion_radius)
        .def_property("measured_occlusion_threshold", &RegionModality::measured_occlusion_threshold, &RegionModality::set_measured_occlusion_threshold)
        .def_property("modeled_depth_offset_radius", &RegionModality::modeled_depth_offset_radius, &RegionModality::set_modeled_depth_offset_radius)
        .def_property("modeled_occlusion_radius", &RegionModality::modeled_occlusion_radius, &RegionModality::set_modeled_occlusion_radius)
        .def_property("modeled_occlusion_threshold", &RegionModality::modeled_occlusion_threshold, &RegionModality::set_modeled_occlusion_threshold)
        .def_property("n_unoccluded_iterations", &RegionModality::n_unoccluded_iterations, &RegionModality::set_n_unoccluded_iterations)
        .def_property("min_n_unoccluded_lines", &RegionModality::min_n_unoccluded_lines, &RegionModality::set_min_n_unoccluded_lines)

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
        ;


    // DepthModality
    py::class_<DepthModality, Modality, std::shared_ptr<DepthModality>>(m, "DepthModality")
        .def(py::init<const std::string &, const std::shared_ptr<Body> &, const std::shared_ptr<DepthCamera> &, const std::shared_ptr<DepthModel> &>(),
                      "name"_a, "body_ptr"_a, "depth_camera_ptr"_a, "depth_model_ptr"_a)

        .def_property("n_points_max", &DepthModality::n_points_max, &DepthModality::set_n_points_max)
        .def_property("use_adaptive_coverage", &DepthModality::use_adaptive_coverage, &DepthModality::set_use_adaptive_coverage)
        .def_property("stride_length", &DepthModality::stride_length, &DepthModality::set_stride_length)
        .def_property("considered_distances", &DepthModality::considered_distances, &DepthModality::set_considered_distances)
        .def_property("standard_deviations", &DepthModality::standard_deviations, &DepthModality::set_standard_deviations)

        .def("ModelOcclusions", &DepthModality::ModelOcclusions)
        .def("MeasureOcclusions", &DepthModality::MeasureOcclusions)

        .def_property("measured_depth_offset_radius", &DepthModality::measured_depth_offset_radius, &DepthModality::set_measured_depth_offset_radius)
        .def_property("measured_occlusion_radius", &DepthModality::measured_occlusion_radius, &DepthModality::set_measured_occlusion_radius)
        .def_property("measured_occlusion_threshold", &DepthModality::measured_occlusion_threshold, &DepthModality::set_measured_occlusion_threshold)
        .def_property("modeled_depth_offset_radius", &DepthModality::modeled_depth_offset_radius, &DepthModality::set_modeled_depth_offset_radius)
        .def_property("modeled_occlusion_radius", &DepthModality::modeled_occlusion_radius, &DepthModality::set_modeled_occlusion_radius)
        .def_property("modeled_occlusion_threshold", &DepthModality::modeled_occlusion_threshold, &DepthModality::set_modeled_occlusion_threshold)
        .def_property("n_unoccluded_iterations", &DepthModality::n_unoccluded_iterations, &DepthModality::set_n_unoccluded_iterations)
        .def_property("min_n_unoccluded_points", &DepthModality::min_n_unoccluded_points, &DepthModality::set_min_n_unoccluded_points)

        .def_property("visualize_correspondences_correspondence", &DepthModality::visualize_correspondences_correspondence, &DepthModality::set_visualize_correspondences_correspondence)
        .def_property("visualize_points_correspondence", &DepthModality::visualize_points_correspondence, &DepthModality::set_visualize_points_correspondence)
        .def_property("visualize_points_depth_rendering_correspondence", &DepthModality::visualize_points_depth_rendering_correspondence, &DepthModality::set_visualize_points_depth_rendering_correspondence)
        .def_property("visualize_points_optimization", &DepthModality::visualize_points_optimization, &DepthModality::set_visualize_points_optimization)
        .def_property("visualize_points_result", &DepthModality::visualize_points_result, &DepthModality::set_visualize_points_result)
        ;


    // TextureModality
    py::class_<TextureModality, Modality, std::shared_ptr<TextureModality>>(m, "TextureModality")
        .def(py::init<const std::string &, const std::shared_ptr<Body> &, const std::shared_ptr<ColorCamera> &, const std::shared_ptr<FocusedSilhouetteRenderer>&>(),
                      "name"_a, "body_ptr"_a, "color_camera_ptr"_a, "silhouette_renderer_ptr"_a)
        .def(py::init<const std::string &, const std::filesystem::path &, const std::shared_ptr<Body> &, const std::shared_ptr<ColorCamera> &, const std::shared_ptr<FocusedSilhouetteRenderer>&>(),
                      "name"_a, "metafile_path"_a, "body_ptr"_a, "color_camera_ptr"_a, "silhouette_renderer_ptr"_a)

        .def("ModelOcclusions", &TextureModality::ModelOcclusions)
        .def("MeasureOcclusions", &TextureModality::MeasureOcclusions)
        ;

    // Optimizer
    py::class_<Optimizer, std::shared_ptr<Optimizer>>(m, "Optimizer")
        .def(py::init<const std::string &, const std::shared_ptr<Link> &, float, float>(),
                      "name"_a, "root_link_ptr"_a, "tikhonov_parameter_rotation"_a=1000.0f, "tikhonov_parameter_translation"_a=30000.0f)
        // .def(py::init<const std::string &, const std::filesystem::path &, const std::shared_ptr<Link> &>(),
        //               "name"_a, "metafile_path"_a "root_link_ptr"_a)
        .def_property("name", &Optimizer::name, &Optimizer::set_name)
        .def_property("metafile_path", &Optimizer::metafile_path, &Optimizer::set_metafile_path)
        .def_property("tikhonov_parameter_rotation", &Optimizer::tikhonov_parameter_rotation, &Optimizer::set_tikhonov_parameter_rotation)
        .def_property("tikhonov_parameter_translation", &Optimizer::tikhonov_parameter_translation, &Optimizer::set_tikhonov_parameter_translation)
        ;

}   


