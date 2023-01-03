#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/chrono.h>

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

#include "pyicg/type_caster_utils.h"

namespace py = pybind11;
// to be able to use "arg"_a shorthand
using namespace pybind11::literals;
using namespace icg;
using namespace Eigen;


PYBIND11_MODULE(_pyicg_mod, m) {

    ///////////////////////
    // Implicit conversions
    // See:
    // https://pybind11.readthedocs.io/en/stable/advanced/classes.html?highlight=implicitly_convertible#implicit-conversions
    // https://stackoverflow.com/questions/56009999/python-bindings-using-pybind11-with-stdfilesystem-as-function-argument-giving
    ///////////////////////
    py::class_<std::filesystem::path>(m, "Path")
        .def(py::init<std::string>())
        .def("__repr__",
            [](const std::filesystem::path &p) {
                return "std::filesystem::path: " + p.string();
            })
        ;
    // py::implicitly_convertible<convert_from, convert_to>
    py::implicitly_convertible<std::string, std::filesystem::path>();

    ///////////////////////
    // Classes
    ///////////////////////

    py::class_<Tracker>(m, "Tracker")
        .def(py::init<const std::string, int, int, bool, 
                      const std::chrono::milliseconds&, int, int>(), 
                      "name"_a, "n_corr_iterations"_a=5, "n_update_iterations"_a=2, "synchronize_cameras"_a=true, 
                      "cycle_duration"_a=std::chrono::milliseconds{33}, "visualization_time"_a=0, "viewer_time"_a=1)
        .def("SetUp", &Tracker::SetUp)
        ;

    // RendererGeometry
    py::class_<icg::RendererGeometry>(m, "RendererGeometry")
        .def(py::init<const std::string &>(), "name"_a)
        ;
    
    // TODO: Somehow read the flag USE_REALSENSE to decide whether or not to create bindings
    // RealSenseColorCamera
    py::class_<icg::RealSenseColorCamera>(m, "RealSenseColorCamera")
        .def(py::init<const std::string &, bool>(), "name"_a, "use_depth_as_world_frame"_a=false)
        ;

    // RealSenseDepthCamera
    py::class_<icg::RealSenseDepthCamera>(m, "RealSenseDepthCamera")
        .def(py::init<const std::string &, bool>(), "name"_a, "use_color_as_world_frame"_a=true)
        ;

    // NormalColorViewer
    py::class_<NormalColorViewer>(m, "NormalColorViewer")
        .def(py::init<const std::string &, const std::shared_ptr<ColorCamera> &, const std::shared_ptr<RendererGeometry> &, float>(),
                      "name"_a, "color_camera_ptr"_a, "renderer_geometry_ptr"_a, "opacity"_a=0.5f)
        ;

    // NormalDepthViewer
    py::class_<NormalDepthViewer>(m, "NormalDepthViewer")
        .def(py::init<const std::string &, const std::shared_ptr<DepthCamera> &, const std::shared_ptr<RendererGeometry> &, float, float, float>(),
                      "name"_a, "depth_camera_ptr"_a, "renderer_geometry_ptr"_a, "min_depth"_a=0.0f, "max_depth"_a=1.0f, "opacity"_a=0.5f)
        ;

    py::class_<FocusedBasicDepthRenderer>(m, "FocusedBasicDepthRenderer")
        .def(py::init<const std::string &, const std::shared_ptr<RendererGeometry> &, const Transform3fA &, const Intrinsics &, int, float, float>(),
                      "name"_a, "renderer_geometry_ptr"_a, "world2camera_pose"_a, "intrinsics"_a, "image_size"_a=200, "z_min"_a=0.01f, "z_max"_a=5.0f)       
        .def(py::init<const std::string &, const std::shared_ptr<RendererGeometry> &, const std::shared_ptr<Camera> &, int, float, float>(),
                      "name"_a, "renderer_geometry_ptr"_a, "camera_ptr"_a, "image_size"_a=200, "z_min"_a=0.01f, "z_max"_a=5.0f)
        ;
    
    // Body
    py::class_<Body>(m, "Body")
        .def(py::init<const std::string &, const std::filesystem::path &>(), "name"_a, "geometry_path"_a)
        .def(py::init<const std::string &, const std::filesystem::path &, float, bool, bool, const Transform3fA &, uchar>(),
                      "name"_a, "geometry_path"_a, "geometry_unit_in_meter"_a, "geometry_counterclockwise"_a, "geometry_enable_culling"_a, "geometry2body_pose"_a, "silhouette_id"_a=0)
        .def_property("body2world_pose", &Body::body2world_pose, &Body::set_body2world_pose)
        .def_property("world2body_pose", &Body::world2body_pose, &Body::set_world2body_pose)
        ;

    // StaticDetector
    py::class_<StaticDetector>(m, "StaticDetector")
        .def(py::init<const std::string &, const std::shared_ptr<Body> &, const Transform3fA &>(),
                      "name"_a, "body_ptr"_a, "body2world_pose"_a);
 

    // RegionModel
    py::class_<RegionModel>(m, "RegionModel")
        .def(py::init<const std::string &, const std::shared_ptr<Body> &, const std::filesystem::path &, 
                      float, int, int, float, float, bool, int>(),
                      "name"_a, "body_ptr"_a, "model_path"_a, 
                      "sphere_radius"_a=0.8f, "n_divides"_a=4, "n_points"_a=200, "max_radius_depth_offset"_a=0.05f, "stride_depth_offset"_a=0.002f, "use_random_seed"_a=false, "image_size"_a=2000)
        ;

    // DepthModel
    py::class_<DepthModel>(m, "DepthModel")
        .def(py::init<const std::string &, const std::shared_ptr<Body> &, const std::filesystem::path &, 
                      float, int, int, float, float, bool, int>(),
                      "name"_a, "body_ptr"_a, "model_path"_a, 
                      "sphere_radius"_a=0.8f, "n_divides"_a=4, "n_points"_a=200, "max_radius_depth_offset"_a=0.05f, "stride_depth_offset"_a=0.002f, "use_random_seed"_a=false, "image_size"_a=2000)
        ;

    // RegionModality (because of RegionModel)
    py::class_<RegionModality>(m, "RegionModality")
        .def(py::init<const std::string &, const std::shared_ptr<Body> &, const std::shared_ptr<ColorCamera> &, const std::shared_ptr<RegionModel> &>(),
                      "name"_a, "body_ptr"_a, "color_camera_ptr"_a, "region_model_ptr"_a)
        ;


    // DepthModality (because of DepthModel)
    py::class_<DepthModality>(m, "DepthModality")
        .def(py::init<const std::string &, const std::shared_ptr<Body> &, const std::shared_ptr<DepthCamera> &, const std::shared_ptr<DepthModel> &>(),
                      "name"_a, "body_ptr"_a, "depth_camera_ptr"_a, "depth_model_ptr"_a)
        ;

    // Optimizer
    py::class_<Optimizer>(m, "Optimizer")
        .def(py::init<const std::string &, float, float>(),
                      "name"_a, "tikhonov_parameter_rotation"_a=1000.0f, "tikhonov_parameter_translation"_a=30000.0f)
        .def(py::init<const std::string &, const std::filesystem::path &>(),
                      "name"_a, "metafile_path"_a)
        .def_property("name", &Optimizer::name, &Optimizer::set_name)
        .def_property("metafile_path", &Optimizer::metafile_path, &Optimizer::set_metafile_path)
        .def_property("tikhonov_parameter_rotation", &Optimizer::tikhonov_parameter_rotation, &Optimizer::set_tikhonov_parameter_rotation)
        .def_property("tikhonov_parameter_translation", &Optimizer::tikhonov_parameter_translation, &Optimizer::set_tikhonov_parameter_translation)
        ;













}   


